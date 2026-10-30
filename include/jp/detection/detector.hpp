#pragma once

#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

#include "jp/core/class_catalog.hpp"
#include "jp/detection/detection.hpp"

namespace jp::detection {

/// Time spent in the stages of the last detect() call (ms).
struct DetectorTimings {
  double preprocess_ms = 0.0;
  double inference_ms = 0.0;
  double postprocess_ms = 0.0;
};

/// 2D object detector. Backends: ONNX Runtime (CUDA or CPU); others can be added behind this
/// interface (e.g. TensorRT).
class Detector {
 public:
  Detector() = default;
  Detector(const Detector&) = delete;
  Detector& operator=(const Detector&) = delete;
  Detector(Detector&&) = delete;
  Detector& operator=(Detector&&) = delete;
  virtual ~Detector() = default;

  /// Detections in image pixels of a BGR image, with detector class labels set.
  [[nodiscard]] virtual std::vector<Detection> detect(const cv::Mat& image) = 0;
  [[nodiscard]] virtual DetectorTimings last_timings() const = 0;
  /// Execution backend in use, e.g. "onnxruntime-cuda".
  [[nodiscard]] virtual std::string backend() const = 0;
};

struct OnnxDetectorOptions {
  std::filesystem::path model;  ///< End-to-end YOLO26 ONNX model.
  float confidence_threshold = 0.25F;
  bool use_cuda = true;  ///< Falls back to the CPU when CUDA is unavailable.
  int cuda_device = 0;
};

/// YOLO26 end-to-end (NMS-free) ONNX model on ONNX Runtime. Throws std::runtime_error if the
/// model cannot be loaded or has an unexpected input/output layout.
[[nodiscard]] std::unique_ptr<Detector> make_onnx_detector(const OnnxDetectorOptions& options);

/// Maps detector labels onto twin classes (`source`, e.g. "coco", in the class catalog) and
/// drops detections whose label has no twin class.
[[nodiscard]] std::vector<Detection> map_to_twin_classes(std::vector<Detection> detections,
                                                         const core::ClassCatalog& catalog,
                                                         std::string_view source);

}  // namespace jp::detection
