#pragma once

#include <memory>
#include <string>
#include <vector>

namespace jp {

struct TensorInfo {
  std::string name;
  std::vector<int64_t> shape;
  bool is_input = false;
  int dtype = 0;  // nvinfer1::DataType
  size_t bytes = 0;
};

// Thin TensorRT 10 wrapper: builds (and caches) an engine from ONNX, owns device buffers and a stream.
class TrtEngine {
 public:
  struct Options {
    bool fp16 = true;
    bool int8 = false;
    size_t workspace_mb = 2048;
  };

  // Loads <onnx>.<gpu>.engine if present and newer than the ONNX file, otherwise builds and caches it.
  static std::unique_ptr<TrtEngine> fromOnnx(const std::string& onnx_path, const Options& opt);
  static std::unique_ptr<TrtEngine> fromEngine(const std::string& engine_path);
  ~TrtEngine();

  const std::vector<TensorInfo>& tensors() const { return tensors_; }
  const TensorInfo& tensor(const std::string& name) const;

  // Copy host input -> device, run, copy outputs device -> host. Blocking.
  void infer(const std::vector<const void*>& host_inputs, const std::vector<void*>& host_outputs);
  double lastLatencyMs() const { return last_ms_; }

 private:
  TrtEngine() = default;
  void setup();
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<TensorInfo> tensors_;
  std::vector<void*> device_;
  double last_ms_ = 0;
};

}  // namespace jp
