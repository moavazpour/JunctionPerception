#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace jp {

enum class DType { F32, F16, I32, I64, U8, Bool };

struct TensorInfo {
  std::string name;
  std::vector<int64_t> shape;
  bool is_input = false;
  DType dtype = DType::F32;
  size_t bytes = 0;
};

// One ONNX model behind a backend: TensorRT on an NVIDIA GPU or ONNX Runtime on the CPU.
class InferenceEngine {
 public:
  virtual ~InferenceEngine() = default;
  virtual const std::vector<TensorInfo>& tensors() const = 0;
  // Host input buffers in, host output buffers out (tensor order, inputs then outputs). Blocking.
  virtual void infer(const std::vector<const void*>& host_inputs, const std::vector<void*>& host_outputs) = 0;
  virtual double lastLatencyMs() const = 0;
  virtual std::string backend() const = 0;    // "tensorrt" | "cpu"
  virtual std::string device() const = 0;     // e.g. "NVIDIA GeForce RTX 4090" or "CPU (16 threads)"
  virtual std::string precision() const = 0;  // "fp16" | "fp32"
  // Models exported with dynamic height/width take any input size (multiples of 32 for YOLO).
  virtual bool dynamicInput() const { return false; }
  virtual void setInputSize(int /*width*/, int /*height*/) {}
};

struct EngineOptions {
  // auto: ONNX Runtime on the GPU (CUDA, FP32: the trained model as is) when available, else on the CPU.
  // tensorrt: TensorRT engine (faster, FP16 by default, slightly different numerics).
  std::string backend = "auto";  // auto | cuda | tensorrt | cpu
  bool fp16 = true;              // TensorRT only
  int threads = 0;               // CPU only; 0 = all cores
  size_t workspace_mb = 2048;
};

std::unique_ptr<InferenceEngine> createEngine(const std::string& onnx_path, const EngineOptions& opt);
// Backends compiled into this build and whether a usable GPU is present.
std::vector<std::string> availableBackends();
bool gpuAvailable();

}  // namespace jp
