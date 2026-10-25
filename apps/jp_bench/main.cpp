// Runs the deployed TensorRT detector over a JP3D jsonl split and writes camera-frame 3D predictions,
// using the same lifting code as the runtime. Evaluate with training/tools/eval_predictions.py.
//
//   jp_bench --onnx gc3d.onnx --data test.jsonl --images /datasets/TUMTraf --out preds.jsonl
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "jp/geometry/camera.hpp"
#include "jp/infer/detector.hpp"

int main(int argc, char** argv) {
  std::string onnx, data, images, out;
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
  }
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
