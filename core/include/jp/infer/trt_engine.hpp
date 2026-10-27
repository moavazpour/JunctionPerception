#pragma once

#include <memory>
#include <string>
#include <vector>

#include "jp/infer/engine.hpp"

namespace jp {

// Thin TensorRT 10 wrapper: builds (and caches) an engine from ONNX, owns device buffers and a stream.
class TrtEngine : public InferenceEngine {
 public:
  struct Options {
    bool fp16 = true;
    bool int8 = false;
    size_t workspace_mb = 2048;
  };

  // Loads <onnx>.<gpu>.engine if present and newer than the ONNX file, otherwise builds and caches it.
  static std::unique_ptr<TrtEngine> fromOnnx(const std::string& onnx_path, const Options& opt);
  static std::unique_ptr<TrtEngine> fromEngine(const std::string& engine_path);
  static int deviceCount();
  ~TrtEngine() override;

  const std::vector<TensorInfo>& tensors() const override { return tensors_; }
  const TensorInfo& tensor(const std::string& name) const;

  // Copy host input -> device, run, copy outputs device -> host. Blocking.
  void infer(const std::vector<const void*>& host_inputs, const std::vector<void*>& host_outputs) override;
  double lastLatencyMs() const override { return last_ms_; }
  std::string backend() const override { return "tensorrt"; }
  std::string device() const override { return device_name_; }
  std::string precision() const override { return precision_; }

 private:
  TrtEngine() = default;
  void setup();
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<TensorInfo> tensors_;
  std::vector<void*> device_;
  double last_ms_ = 0;
  std::string device_name_, precision_ = "fp32";
};

}  // namespace jp
