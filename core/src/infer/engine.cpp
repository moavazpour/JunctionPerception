#include "jp/infer/engine.hpp"

#include <spdlog/spdlog.h>

#include <stdexcept>

#ifdef JP_HAVE_TENSORRT
#include "jp/infer/trt_engine.hpp"
#endif
#if defined(JP_HAVE_TENSORRT) || defined(JP_HAVE_ORT_CUDA)
#include <cuda_runtime_api.h>
#endif

namespace jp {

#ifdef JP_HAVE_ORT
std::unique_ptr<InferenceEngine> createOrtEngine(const std::string& onnx, int threads, bool cuda);
#endif

bool gpuAvailable() {
#if defined(JP_HAVE_TENSORRT) || defined(JP_HAVE_ORT_CUDA)
  static const bool ok = [] {
    int n = 0;
    return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
  }();
  return ok;
#else
  return false;
#endif
}

std::vector<std::string> availableBackends() {
  std::vector<std::string> b;
#ifdef JP_HAVE_ORT_CUDA
  if (gpuAvailable()) b.push_back("cuda");
#endif
#ifdef JP_HAVE_TENSORRT
  if (gpuAvailable()) b.push_back("tensorrt");
#endif
#ifdef JP_HAVE_ORT
  b.push_back("cpu");
#endif
  return b;
}

std::unique_ptr<InferenceEngine> createEngine(const std::string& onnx, const EngineOptions& opt) {
  std::string want = opt.backend.empty() ? "auto" : opt.backend;
  if (want == "gpu") want = "cuda";
  if (want == "auto") {
#if defined(JP_HAVE_ORT_CUDA)
    want = gpuAvailable() ? "cuda" : "cpu";
#else
    want = gpuAvailable() ? "tensorrt" : "cpu";
#endif
    if (want == "cpu") spdlog::warn("No usable NVIDIA GPU: running {} on the CPU", onnx);
  }
  if (want == "tensorrt") {
#ifdef JP_HAVE_TENSORRT
    if (!gpuAvailable()) throw std::runtime_error("backend tensorrt requested but no NVIDIA GPU is available");
    TrtEngine::Options o;
    o.fp16 = opt.fp16;
    o.workspace_mb = opt.workspace_mb;
    return TrtEngine::fromOnnx(onnx, o);
#else
    throw std::runtime_error("this build has no TensorRT backend");
#endif
  }
  if (want == "cuda") {
#ifdef JP_HAVE_ORT_CUDA
    if (!gpuAvailable()) throw std::runtime_error("backend cuda requested but no NVIDIA GPU is available");
    return createOrtEngine(onnx, opt.threads, true);
#else
    throw std::runtime_error("this build has no ONNX Runtime CUDA backend");
#endif
  }
  if (want == "cpu") {
#ifdef JP_HAVE_ORT
    return createOrtEngine(onnx, opt.threads, false);
#else
    throw std::runtime_error("this build has no CPU (ONNX Runtime) backend");
#endif
  }
  throw std::runtime_error("unknown inference backend " + want);
}

}  // namespace jp
