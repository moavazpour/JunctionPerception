#include "jp/infer/detector.hpp"

#include <NvInfer.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "jp/infer/trt_engine.hpp"

namespace jp {

Letterbox::Letterbox(cv::Size frame, cv::Size net, const cv::Matx33d& K, const std::vector<double>& dist)
    : frame_(frame), net_(net) {
  s_ = std::min(static_cast<double>(net.width) / frame.width, static_cast<double>(net.height) / frame.height);
  ox_ = (net.width - frame.width * s_) / 2.0;
  oy_ = (net.height - frame.height * s_) / 2.0;
  remap_ = !dist.empty() && std::any_of(dist.begin(), dist.end(), [](double d) { return std::abs(d) > 1e-9; });
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
  const int w = static_cast<int>(std::round(frame.cols * s_)), h = static_cast<int>(std::round(frame.rows * s_));
  cv::Mat roi = out(cv::Rect(static_cast<int>(std::round(ox_)), static_cast<int>(std::round(oy_)),
                             std::min(w, net_.width), std::min(h, net_.height)));
  cv::resize(frame, roi, roi.size(), 0, 0, cv::INTER_LINEAR);
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
  TrtEngine::Options opt;
  opt.fp16 = cfg.fp16;
  engine_ = TrtEngine::fromOnnx(cfg.onnx, opt);
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
  spdlog::info("Detector3D: input {}x{}, top-{}", net_.width, net_.height, topk_);
}

Detector3D::~Detector3D() = default;

double Detector3D::lastLatencyMs() const { return engine_->lastLatencyMs(); }

std::vector<ImageDetection> Detector3D::detect(const cv::Mat& net_bgr, const Letterbox& lb) {
  static constexpr float kZero[3] = {0.f, 0.f, 0.f}, k255[3] = {255.f, 255.f, 255.f};
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

Segmenter::Segmenter(const std::string& onnx, bool fp16) {
  TrtEngine::Options opt;
  opt.fp16 = fp16;
  engine_ = TrtEngine::fromOnnx(onnx, opt);
  const TensorInfo* in = nullptr;
  const TensorInfo* out = nullptr;
  for (const auto& t : engine_->tensors()) (t.is_input ? in : out) = &t;
  net_ = cv::Size(static_cast<int>(in->shape[3]), static_cast<int>(in->shape[2]));
  input_.resize(in->bytes / sizeof(float));
  argmax_in_graph_ = out->shape.size() == 3;
  if (argmax_in_graph_) {
    if (out->dtype == static_cast<int>(nvinfer1::DataType::kINT64)) {
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
