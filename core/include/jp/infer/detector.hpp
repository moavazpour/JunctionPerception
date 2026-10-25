#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "jp/common/types.hpp"

namespace jp {

class TrtEngine;

// Maps a raw camera frame into the network input (undistortion + letterbox in one remap) and back.
class Letterbox {
 public:
  Letterbox() = default;
  // K/dist describe the raw frame; empty dist means the frame is already undistorted.
  Letterbox(cv::Size frame, cv::Size net, const cv::Matx33d& K = cv::Matx33d::eye(),
            const std::vector<double>& dist = {});
  void apply(const cv::Mat& frame_bgr, cv::Mat& out_bgr) const;
  // Network-input pixel -> undistorted frame pixel.
  cv::Point2d toFrame(cv::Point2d p) const { return {(p.x - ox_) / s_, (p.y - oy_) / s_}; }
  double scale() const { return s_; }

 private:
  cv::Size frame_, net_;
  double s_ = 1, ox_ = 0, oy_ = 0;
  bool remap_ = false;
  cv::Mat map1_, map2_;
};

struct DetectorConfig {
  std::string onnx;
  float score_threshold = 0.15f;
  bool fp16 = true;
};

// Ground-aware mono-3D detector (calibration-free outputs).
class Detector3D {
 public:
  explicit Detector3D(const DetectorConfig& cfg);
  ~Detector3D();
  cv::Size inputSize() const { return net_; }
  // frame_bgr must already be letterboxed to inputSize() by the caller's Letterbox.
  std::vector<ImageDetection> detect(const cv::Mat& net_bgr, const Letterbox& lb);
  double lastLatencyMs() const;

 private:
  DetectorConfig cfg_;
  std::unique_ptr<TrtEngine> engine_;
  cv::Size net_;
  int topk_ = 0, fields_ = 0;
  bool yolo_ = false;
  std::vector<float> input_, output_;
};

// Semantic segmentation (Cityscapes label set) used for ground, static occlusion and visibility.
class Segmenter {
 public:
  enum Label : uint8_t {
    Road = 0, Sidewalk, Building, Wall, Fence, Pole, TrafficLight, TrafficSign, Vegetation, Terrain, Sky,
    Person, Rider, Car, Truck, Bus, Train, Motorcycle, Bicycle, Unknown = 255
  };
  explicit Segmenter(const std::string& onnx, bool fp16 = true);
  ~Segmenter();
  // Returns CV_8U label map at the network resolution.
  cv::Mat segment(const cv::Mat& frame_bgr);
  cv::Size inputSize() const { return net_; }

 private:
  std::unique_ptr<TrtEngine> engine_;
  cv::Size net_;
  int classes_ = 19;
  bool argmax_in_graph_ = false;
  std::vector<float> input_;
  std::vector<float> logits_;
  std::vector<int64_t> labels64_;
  std::vector<int32_t> labels32_;
};

void bgrToNchwNormalized(const cv::Mat& bgr, float* dst, const float mean[3], const float std[3]);

}  // namespace jp
