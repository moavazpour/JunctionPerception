#include "jp/infer/detector.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "jp/infer/engine.hpp"

namespace jp {

Letterbox::Letterbox(cv::Size frame, cv::Size net, const cv::Matx33d& K, const std::vector<double>& dist, cv::Rect crop)
    : frame_(frame), net_(net) {
  remap_ = !dist.empty() && std::any_of(dist.begin(), dist.end(), [](double d) { return std::abs(d) > 1e-9; });
  crop_ = remap_ || crop.area() <= 0 ? cv::Rect(0, 0, frame.width, frame.height) : (crop & cv::Rect(0, 0, frame.width, frame.height));
  s_ = std::min(static_cast<double>(net.width) / crop_.width, static_cast<double>(net.height) / crop_.height);
  ox_ = (net.width - crop_.width * s_) / 2.0;
  oy_ = (net.height - crop_.height * s_) / 2.0;
  if (remap_) {
    cv::Matx33d Kn = K;
    Kn(0, 0) *= s_;
    Kn(1, 1) *= s_;
    Kn(0, 2) = K(0, 2) * s_ + ox_;
    Kn(1, 2) = K(1, 2) * s_ + oy_;
    cv::initUndistortRectifyMap(K, dist, cv::Matx33d::eye(), Kn, net, CV_16SC2, map1_, map2_);
  }
}

void Letterbox::apply(const cv::Mat& frame, cv::Mat& out) const {
  if (remap_) {
    cv::remap(frame, out, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    return;
  }
  out.create(net_, CV_8UC3);
  out.setTo(cv::Scalar(114, 114, 114));
  const cv::Mat src = frame(crop_);
  const int w = static_cast<int>(std::round(src.cols * s_)), h = static_cast<int>(std::round(src.rows * s_));
  cv::Mat roi = out(cv::Rect(static_cast<int>(std::round(ox_)), static_cast<int>(std::round(oy_)),
                             std::min(w, net_.width), std::min(h, net_.height)));
  cv::resize(src, roi, roi.size(), 0, 0, cv::INTER_LINEAR);
}

void bgrToNchwNormalized(const cv::Mat& bgr, float* dst, const float mean[3], const float std[3]) {
  const int H = bgr.rows, W = bgr.cols;
  const size_t plane = static_cast<size_t>(H) * W;
  for (int y = 0; y < H; ++y) {
    const uint8_t* row = bgr.ptr<uint8_t>(y);
    float* r = dst + static_cast<size_t>(y) * W;
    float* g = r + plane;
    float* b = g + plane;
    for (int x = 0; x < W; ++x) {
      r[x] = (row[3 * x + 2] - mean[0]) / std[0];
      g[x] = (row[3 * x + 1] - mean[1]) / std[1];
      b[x] = (row[3 * x + 0] - mean[2]) / std[2];
    }
  }
}

namespace {
constexpr float kMean[3] = {123.675f, 116.28f, 103.53f};
constexpr float kStd[3] = {58.395f, 57.12f, 57.375f};
}  // namespace

Detector3D::Detector3D(const DetectorConfig& cfg) : cfg_(cfg) {
  EngineOptions opt;
  opt.fp16 = cfg.fp16;
  opt.backend = cfg.backend;
  opt.threads = cfg.threads;
  engine_ = createEngine(cfg.onnx, opt);
  if (engine_->dynamicInput() && cfg_.imgsz > 0) engine_->setInputSize(cfg_.imgsz, cfg_.imgsz);
  const TensorInfo* inp = nullptr;
  const TensorInfo* outp = nullptr;
  for (const auto& t : engine_->tensors()) (t.is_input ? inp : outp) = &t;
  const auto& in = *inp;
  net_ = cv::Size(static_cast<int>(in.shape[3]), static_cast<int>(in.shape[2]));
  const auto& out = *outp;
  topk_ = static_cast<int>(out.shape[1]);
  fields_ = static_cast<int>(out.shape[2]);
  // Two layouts: GroundCenter3D [K,13] and YOLO26-pose end-to-end [K, 6 + 9*3] (x1 y1 x2 y2 score class kpts).
  yolo_ = fields_ == 6 + 27;
  if (fields_ != 13 && !yolo_) throw std::runtime_error("unexpected detector output layout");
  input_.resize(in.bytes / sizeof(float));
  output_.resize(out.bytes / sizeof(float));
  spdlog::info("Detector3D: input {}x{}, top-{}, {} on {} ({})", net_.width, net_.height, topk_, engine_->backend(),
               engine_->device(), engine_->precision());
}

Detector3D::~Detector3D() = default;

double Detector3D::lastLatencyMs() const { return engine_->lastLatencyMs(); }

bool Detector3D::dynamicInput() const { return engine_->dynamicInput(); }

cv::Size Detector3D::inputSizeFor(cv::Size region) const {
  if (!engine_->dynamicInput() || region.area() <= 0) return net_;
  // Square input as in training (the letterbox pads the region): "640" means 640 x 640.
  const int L = std::max(32, (cfg_.imgsz > 0 ? cfg_.imgsz : std::max(net_.width, net_.height)) / 32 * 32);
  return {L, L};
}

std::vector<ImageDetection> Detector3D::detect(const cv::Mat& net_bgr, const Letterbox& lb) {
  static constexpr float kZero[3] = {0.f, 0.f, 0.f}, k255[3] = {255.f, 255.f, 255.f};
  if (net_bgr.size() != net_) {
    // Dynamic-size model fed a new input size (other input size or crop).
    if (!engine_->dynamicInput()) throw std::runtime_error("network input size mismatch");
    net_ = net_bgr.size();
    engine_->setInputSize(net_.width, net_.height);
    input_.assign(static_cast<size_t>(3) * net_.area(), 0.f);
  }
  bgrToNchwNormalized(net_bgr, input_.data(), yolo_ ? kZero : kMean, yolo_ ? k255 : kStd);
  engine_->infer({input_.data()}, {output_.data()});
  std::vector<ImageDetection> dets;
  const double s = lb.scale();
  if (yolo_) {
    for (int k = 0; k < topk_; ++k) {
      const float* d = output_.data() + static_cast<size_t>(k) * fields_;
      if (d[4] < cfg_.score_threshold) continue;  // end-to-end output is not guaranteed sorted
      const int c = static_cast<int>(d[5]);
      if (c < 0 || c >= static_cast<int>(ObjectClass::Count)) continue;
      ImageDetection det;
      det.score = d[4];
      det.cls = static_cast<ObjectClass>(c);
      const cv::Point2d p0 = lb.toFrame({d[0], d[1]}), p1 = lb.toFrame({d[2], d[3]});
      det.box2d = {p0.x, p0.y, p1.x, p1.y};
      det.has_kpts = true;
      for (int j = 0; j < 9; ++j) {
        const cv::Point2d q = lb.toFrame({d[6 + 3 * j], d[7 + 3 * j]});
        det.kpts[j] = {q.x, q.y, d[8 + 3 * j]};
      }
      det.kp = det.kpts[0].head<2>();
      det.dims = classPriorDims(det.cls);
      dets.push_back(det);
    }
    std::sort(dets.begin(), dets.end(), [](const auto& a, const auto& b) { return a.score > b.score; });
    if (cfg_.max_det > 0 && static_cast<int>(dets.size()) > cfg_.max_det) dets.resize(cfg_.max_det);
    return dets;
  }
  for (int k = 0; k < topk_; ++k) {
    const float* d = output_.data() + static_cast<size_t>(k) * fields_;
    if (d[0] < cfg_.score_threshold) break;  // top-K is sorted by score
    const int c = static_cast<int>(d[1]);
    if (c < 0 || c >= static_cast<int>(ObjectClass::Count)) continue;
    ImageDetection det;
    det.score = d[0];
    det.cls = static_cast<ObjectClass>(c);
    const cv::Point2d kp = lb.toFrame({d[2], d[3]});
    det.kp = {kp.x, kp.y};
    det.box2d = {kp.x - d[4] / s, kp.y - d[5] / s, kp.x + d[6] / s, kp.y + d[7] / s};
    det.dz = d[8];
    det.dims = {d[9], d[10], d[11]};
    det.alpha = d[12];
    dets.push_back(det);
  }
  return dets;
}

Segmenter::Segmenter(const std::string& onnx, const EngineOptions& opt) {
  engine_ = createEngine(onnx, opt);
  const TensorInfo* in = nullptr;
  const TensorInfo* out = nullptr;
  for (const auto& t : engine_->tensors()) (t.is_input ? in : out) = &t;
  net_ = cv::Size(static_cast<int>(in->shape[3]), static_cast<int>(in->shape[2]));
  input_.resize(in->bytes / sizeof(float));
  argmax_in_graph_ = out->shape.size() == 3;
  if (argmax_in_graph_) {
    if (out->dtype == DType::I64) {
      labels64_.resize(out->bytes / 8);
    } else {
      labels32_.resize(out->bytes / 4);
    }
  } else {
    classes_ = static_cast<int>(out->shape[1]);
    logits_.resize(out->bytes / sizeof(float));
  }
}

Segmenter::~Segmenter() = default;

cv::Mat Segmenter::segment(const cv::Mat& frame_bgr) {
  cv::Mat small;
  cv::resize(frame_bgr, small, net_, 0, 0, cv::INTER_AREA);
  bgrToNchwNormalized(small, input_.data(), kMean, kStd);
  cv::Mat labels(net_, CV_8U);
  if (argmax_in_graph_) {
    void* out = labels64_.empty() ? static_cast<void*>(labels32_.data()) : static_cast<void*>(labels64_.data());
    engine_->infer({input_.data()}, {out});
    for (int i = 0; i < net_.area(); ++i)
      labels.data[i] = static_cast<uint8_t>(labels64_.empty() ? labels32_[i] : labels64_[i]);
  } else {
    engine_->infer({input_.data()}, {logits_.data()});
    const int plane = net_.area();
    for (int i = 0; i < plane; ++i) {
      int best = 0;
      float bv = logits_[i];
      for (int c = 1; c < classes_; ++c) {
        const float v = logits_[static_cast<size_t>(c) * plane + i];
        if (v > bv) {
          bv = v;
          best = c;
        }
      }
      labels.data[i] = static_cast<uint8_t>(best);
    }
  }
  return labels;
}

}  // namespace jp
