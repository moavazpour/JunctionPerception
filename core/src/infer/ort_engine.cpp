// ONNX Runtime CPU backend: runs the same ONNX models as TensorRT on machines without an NVIDIA GPU.
#include <onnxruntime_cxx_api.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "jp/infer/engine.hpp"

namespace jp {
namespace {

Ort::Env& env() {
  static Ort::Env e(ORT_LOGGING_LEVEL_WARNING, "junction-perception");
  return e;
}

DType toDType(ONNXTensorElementDataType t) {
  switch (t) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: return DType::F16;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: return DType::I32;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: return DType::I64;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8: return DType::U8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL: return DType::Bool;
    default: return DType::F32;
  }
}

size_t elementSize(ONNXTensorElementDataType t) {
  switch (t) {
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: return 2;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE: return 8;
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
    case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL: return 1;
    default: return 4;
  }
}

class OrtEngine : public InferenceEngine {
 public:
  OrtEngine(const std::string& onnx, int threads) {
    threads_ = threads > 0 ? threads : std::max(1u, std::thread::hardware_concurrency());
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(threads_);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env(), onnx.c_str(), so);
    Ort::AllocatorWithDefaultOptions alloc;
    auto add = [&](bool input, size_t i) {
      auto name = input ? session_->GetInputNameAllocated(i, alloc) : session_->GetOutputNameAllocated(i, alloc);
      auto info = input ? session_->GetInputTypeInfo(i) : session_->GetOutputTypeInfo(i);
      const auto ts = info.GetTensorTypeAndShapeInfo();
      TensorInfo t;
      t.name = name.get();
      t.is_input = input;
      t.shape = ts.GetShape();
      for (auto& d : t.shape) d = std::max<int64_t>(d, 1);  // dynamic dims run with batch 1
      t.dtype = toDType(ts.GetElementType());
      size_t n = 1;
      for (auto d : t.shape) n *= static_cast<size_t>(d);
      t.bytes = n * elementSize(ts.GetElementType());
      tensors_.push_back(t);
      types_.push_back(ts.GetElementType());
    };
    for (size_t i = 0; i < session_->GetInputCount(); ++i) add(true, i);
    for (size_t i = 0; i < session_->GetOutputCount(); ++i) add(false, i);
    for (const auto& t : tensors_) (t.is_input ? in_names_ : out_names_).push_back(t.name);
    spdlog::info("ONNX Runtime CPU engine for {} ({} threads)", onnx, threads_);
  }

  const std::vector<TensorInfo>& tensors() const override { return tensors_; }
  double lastLatencyMs() const override { return last_ms_; }
  std::string backend() const override { return "cpu"; }
  std::string device() const override { return "CPU (" + std::to_string(threads_) + " threads)"; }
  std::string precision() const override { return "fp32"; }

  void infer(const std::vector<const void*>& host_inputs, const std::vector<void*>& host_outputs) override {
    const auto t0 = std::chrono::steady_clock::now();
    const auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> inputs;
    std::vector<const char*> in_names, out_names;
    size_t k = 0;
    for (size_t i = 0; i < tensors_.size(); ++i) {
      const auto& t = tensors_[i];
      if (!t.is_input) continue;
      inputs.push_back(Ort::Value::CreateTensor(mem, const_cast<void*>(host_inputs.at(k++)), t.bytes, t.shape.data(),
                                                t.shape.size(), types_[i]));
      in_names.push_back(t.name.c_str());
    }
    for (const auto& t : tensors_)
      if (!t.is_input) out_names.push_back(t.name.c_str());
    auto outs = session_->Run(Ort::RunOptions{nullptr}, in_names.data(), inputs.data(), inputs.size(), out_names.data(),
                              out_names.size());
    size_t o = 0;
    for (size_t i = 0; i < tensors_.size(); ++i) {
      if (tensors_[i].is_input) continue;
      const auto info = outs[o].GetTensorTypeAndShapeInfo();
      const size_t bytes = std::min(tensors_[i].bytes, info.GetElementCount() * elementSize(types_[i]));
      std::memcpy(host_outputs.at(o), outs[o].GetTensorRawData(), bytes);
      ++o;
    }
    last_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  }

 private:
  std::unique_ptr<Ort::Session> session_;
  std::vector<TensorInfo> tensors_;
  std::vector<ONNXTensorElementDataType> types_;
  std::vector<std::string> in_names_, out_names_;
  int threads_ = 1;
  double last_ms_ = 0;
};
}  // namespace

std::unique_ptr<InferenceEngine> createOrtEngine(const std::string& onnx, int threads) {
  return std::make_unique<OrtEngine>(onnx, threads);
}

}  // namespace jp
