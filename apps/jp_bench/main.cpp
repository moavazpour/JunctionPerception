// Runs the deployed TensorRT detector over a JP3D jsonl split and writes camera-frame 3D predictions,
// using the same lifting code as the runtime. Evaluate with training/tools/eval_predictions.py.
//
//   jp_bench --onnx gc3d.onnx --data test.jsonl --images /datasets/TUMTraf --out preds.jsonl
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "jp/geometry/camera.hpp"
#include "jp/infer/detector.hpp"

// Development aid: calibration-free detections on a video (metadata), optional debug overlays kept local.
static int runVideo(const std::string& onnx, const std::string& video, const std::string& out, const std::string& overlay,
                    int every, int max_frames, float thr, bool fp16) {
  jp::DetectorConfig dc;
  dc.onnx = onnx;
  dc.fp16 = fp16;
  dc.score_threshold = thr;
  jp::Detector3D det(dc);
  cv::VideoCapture cap(video, cv::CAP_FFMPEG);
  if (!cap.isOpened()) return 1;
  const double fps = cap.get(cv::CAP_PROP_FPS);
  std::ofstream os(out);
  cv::Mat frame, net;
  int k = 0;
  std::unique_ptr<jp::Letterbox> lb;
  while (cap.read(frame)) {
    if (max_frames && k >= max_frames) break;
    if (k++ % every) continue;
    if (!lb) lb = std::make_unique<jp::Letterbox>(frame.size(), det.inputSize());
    lb->apply(frame, net);
    const auto dets = det.detect(net, *lb);
    nlohmann::json a = nlohmann::json::array();
    for (const auto& d : dets)
      a.push_back({{"cls", jp::className(d.cls)}, {"score", d.score}, {"kp", {d.kp.x(), d.kp.y()}},
                   {"box2d", {d.box2d[0], d.box2d[1], d.box2d[2], d.box2d[3]}}, {"dims", {d.dims.x(), d.dims.y(), d.dims.z()}},
                   {"dz", d.dz}, {"alpha", d.alpha}});
    os << nlohmann::json{{"frame", k - 1}, {"t", (k - 1) / fps}, {"dets", a}}.dump() << "\n";
    if (!overlay.empty() && (k - 1) % (every * 150) == 0) {
      for (const auto& d : dets) {
        if (d.score < 0.3) continue;
        cv::rectangle(frame, cv::Point2d(d.box2d[0], d.box2d[1]), cv::Point2d(d.box2d[2], d.box2d[3]), {255, 160, 60}, 2);
        cv::circle(frame, cv::Point2d(d.kp.x(), d.kp.y()), 4, {0, 0, 255}, -1);
        cv::putText(frame, std::string(jp::className(d.cls)) + " " + std::to_string(int(d.score * 100)),
                    cv::Point2d(d.box2d[0], d.box2d[1] - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
      }
      cv::imwrite(overlay + "/frame_" + std::to_string(k - 1) + ".jpg", frame);
    }
  }
  return 0;
}

int main(int argc, char** argv) {
  std::string onnx, data, images, out, video, overlay;
  int every = 1, max_frames = 0;
  bool fp16 = true;
  float thr = 0.05f;
  for (int i = 1; i < argc; ++i) {
    auto next = [&] { return std::string(argv[++i]); };
    if (!std::strcmp(argv[i], "--onnx")) onnx = next();
    else if (!std::strcmp(argv[i], "--data")) data = next();
    else if (!std::strcmp(argv[i], "--images")) images = next();
    else if (!std::strcmp(argv[i], "--out")) out = next();
    else if (!std::strcmp(argv[i], "--fp32")) fp16 = false;
    else if (!std::strcmp(argv[i], "--threshold")) thr = std::stof(next());
    else if (!std::strcmp(argv[i], "--video")) video = next();
    else if (!std::strcmp(argv[i], "--overlay")) overlay = next();
    else if (!std::strcmp(argv[i], "--every")) every = std::stoi(next());
    else if (!std::strcmp(argv[i], "--max-frames")) max_frames = std::stoi(next());
  }
  if (!video.empty() && !onnx.empty() && !out.empty()) return runVideo(onnx, video, out, overlay, every, max_frames, thr, fp16);
  if (onnx.empty() || data.empty() || images.empty() || out.empty()) {
    std::cerr << "usage: jp_bench --onnx M --data split.jsonl --images ROOT --out preds.jsonl [--fp32]\n";
    return 2;
  }
  jp::DetectorConfig dc;
  dc.onnx = onnx;
  dc.fp16 = fp16;
  dc.score_threshold = thr;
  jp::Detector3D det(dc);
  std::ifstream in(data);
  std::ofstream os(out);
  std::string line;
  size_t n = 0;
  double total_ms = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto s = nlohmann::json::parse(line);
    cv::Mat img = cv::imread(images + "/" + s["image"].get<std::string>());
    if (img.empty()) {
      spdlog::warn("missing image {}", s["image"].get<std::string>());
      continue;
    }
    Eigen::Matrix3d K;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) K(r, c) = s["K"][r][c];
    const auto p = s["plane"];
    jp::CameraModel cam(img.cols, img.rows, K, Eigen::Vector4d(p[0], p[1], p[2], p[3]), 0.0);
    jp::Letterbox lb(img.size(), det.inputSize());
    cv::Mat net;
    const auto t0 = std::chrono::steady_clock::now();
    lb.apply(img, net);
    const auto dets = det.detect(net, lb);
    total_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    nlohmann::json objs = nlohmann::json::array();
    for (const auto& d : dets) {
      auto x = cam.lift(d);
      if (!x) continue;
      const Eigen::Vector3d b = cam.groundToCamera(x->bottom);
      const Eigen::Vector3d h = cam.R_ground_cam().transpose() * Eigen::Vector3d(std::cos(x->yaw), std::sin(x->yaw), 0);
      objs.push_back({{"cls", jp::className(d.cls)},
                      {"score", d.score},
                      {"bottom", {b.x(), b.y(), b.z()}},
                      {"dims", {d.dims.x(), d.dims.y(), d.dims.z()}},
                      {"heading", {h.x(), h.y(), h.z()}},
                      {"box2d", {d.box2d[0], d.box2d[1], d.box2d[2], d.box2d[3]}}});
    }
    os << nlohmann::json{{"image", s["image"]}, {"objects", objs}}.dump() << "\n";
    ++n;
  }
  spdlog::info("{} images, mean pre-processing + inference {:.2f} ms", n, n ? total_ms / n : 0.0);
  return 0;
}
