#include <onnxruntime_cxx_api.h>
#include <spdlog/spdlog.h>

#include <array>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <span>
#include <stdexcept>

#include "jp/detection/detector.hpp"
#include "jp/detection/letterbox.hpp"
#include "jp/detection/yolo_output.hpp"

namespace jp::detection {

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

class OnnxYoloDetector final : public Detector {
 public:
  explicit OnnxYoloDetector(const OnnxDetectorOptions& options)
      : env_(ORT_LOGGING_LEVEL_WARNING, "jp"), threshold_(options.confidence_threshold) {
    bool loaded = false;
    if (options.use_cuda) {
      try {
        session_ = create_session(options, true);
        loaded = true;
        backend_ = "onnxruntime-cuda";
      } catch (const Ort::Exception& error) {
        spdlog::warn("CUDA execution provider unavailable ({}); using the CPU", error.what());
      }
    }
    if (!loaded) {
      session_ = create_session(options, false);
      backend_ = "onnxruntime-cpu";
    }
    read_layout();
  }

  std::vector<Detection> detect(const cv::Mat& image) override {
    const auto t0 = Clock::now();
    const Letterbox letterbox = Letterbox::fit(image.size(), input_size_);
    fill_input(letterbox.apply(image));
    const auto t1 = Clock::now();

    const Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const std::array<std::int64_t, 4> shape = {1, 3, input_size_, input_size_};
    const Ort::Value input = Ort::Value::CreateTensor<float>(memory, input_.data(), input_.size(),
                                                             shape.data(), shape.size());
    const std::array<const char*, 1> input_names = {input_name_.c_str()};
    const std::array<const char*, 1> output_names = {output_name_.c_str()};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names.data(), &input, 1,
                                output_names.data(), 1);
    const auto t2 = Clock::now();

    const auto info = outputs.front().GetTensorTypeAndShapeInfo();
    const std::span<const float> values(outputs.front().GetTensorData<float>(),
                                        info.GetElementCount());
    std::vector<Detection> detections;
    for (const auto& raw : parse_end2end_output(values, threshold_)) {
      Detection d;
      d.box = letterbox.to_image(raw.box, image.size());
      d.score = raw.score;
      d.source_class = raw.class_id;
      if (raw.class_id >= 0 && static_cast<std::size_t>(raw.class_id) < class_names_.size()) {
        d.source_label = class_names_[static_cast<std::size_t>(raw.class_id)];
      }
      detections.push_back(std::move(d));
    }
    const auto t3 = Clock::now();
    timings_ = {elapsed_ms(t0, t1), elapsed_ms(t1, t2), elapsed_ms(t2, t3)};
    return detections;
  }

  [[nodiscard]] DetectorTimings last_timings() const override { return timings_; }
  [[nodiscard]] std::string backend() const override { return backend_; }

 private:
  Ort::Session create_session(const OnnxDetectorOptions& options, bool cuda) {
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (cuda) {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = options.cuda_device;
      session_options.AppendExecutionProvider_CUDA(cuda_options);
    }
    return {env_, options.model.c_str(), session_options};
  }

  void read_layout() {
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      throw std::runtime_error("expected a model with one input and one output");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
    const auto input_shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 4 || input_shape[1] != 3 || input_shape[2] != input_shape[3] ||
        input_shape[2] <= 0) {
      throw std::runtime_error("expected a static square input of shape (1, 3, S, S)");
    }
    input_size_ = static_cast<int>(input_shape[2]);
    const auto output_shape = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (output_shape.size() != 3 || output_shape[2] != static_cast<std::int64_t>(kEnd2EndRowSize)) {
      throw std::runtime_error("expected an end-to-end output of shape (1, N, 6)");
    }
    const auto metadata = session_.GetModelMetadata();
    const auto names = metadata.LookupCustomMetadataMapAllocated("names", allocator);
    if (!names) {
      throw std::runtime_error("model has no class names metadata");
    }
    class_names_ = parse_class_names(names.get());
    const auto side = static_cast<std::size_t>(input_size_);
    input_.resize(3U * side * side);
  }

  // BGR uint8 HWC -> RGB float CHW in [0, 1].
  void fill_input(const cv::Mat& letterboxed) {
    cv::Mat rgb;
    cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);
    const auto plane =
        static_cast<std::size_t>(input_size_) * static_cast<std::size_t>(input_size_);
    std::array<cv::Mat, 3> channels;
    for (std::size_t c = 0; c < channels.size(); ++c) {
      channels.at(c) = cv::Mat(input_size_, input_size_, CV_32F,
                               std::span<float>(input_).subspan(c * plane, plane).data());
    }
    cv::Mat as_float;
    rgb.convertTo(as_float, CV_32F, 1.0 / 255.0);
    cv::split(as_float, channels.data());
  }

  Ort::Env env_;
  Ort::Session session_{nullptr};
  float threshold_;
  std::string backend_;
  std::string input_name_;
  std::string output_name_;
  int input_size_ = 0;
  std::vector<std::string> class_names_;
  std::vector<float> input_;
  DetectorTimings timings_;
};

}  // namespace

std::unique_ptr<Detector> make_onnx_detector(const OnnxDetectorOptions& options) {
  if (!std::filesystem::is_regular_file(options.model)) {
    throw std::runtime_error("model not found: " + options.model.string());
  }
  try {
    return std::make_unique<OnnxYoloDetector>(options);
  } catch (const Ort::Exception& error) {
    throw std::runtime_error(std::string("cannot load model: ") + error.what());
  }
}

std::vector<Detection> map_to_twin_classes(std::vector<Detection> detections,
                                           const core::ClassCatalog& catalog,
                                           std::string_view source) {
  std::vector<Detection> mapped;
  mapped.reserve(detections.size());
  for (auto& d : detections) {
    if (auto twin = catalog.map(source, d.source_label)) {
      d.twin_class = std::move(*twin);
      mapped.push_back(std::move(d));
    }
  }
  return mapped;
}

}  // namespace jp::detection
