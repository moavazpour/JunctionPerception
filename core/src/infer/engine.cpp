#include "jp/infer/engine.hpp"

#include <spdlog/spdlog.h>

#include <stdexcept>

#ifdef JP_HAVE_TENSORRT
#include "jp/infer/trt_engine.hpp"
#endif

namespace jp {

#ifdef JP_HAVE_ORT
std::unique_ptr<InferenceEngine> createOrtEngine(const std::string& onnx, int threads);
#endif

bool gpuAvailable() {
#ifdef JP_HAVE_TENSORRT
  static const bool ok = [] {
    try {
      return TrtEngine::deviceCount() > 0;
    } catch (...) {
      return false;
    }
  }();
  return ok;
#else
  return false;
#endif
}

std::vector<std::string> availableBackends() {
  std::vector<std::string> b;
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
  if (want == "gpu") want = "tensorrt";
  if (want == "auto") {
    want = gpuAvailable() ? "tensorrt" : "cpu";
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
  if (want == "cpu") {
#ifdef JP_HAVE_ORT
    return createOrtEngine(onnx, opt.threads);
#else
    throw std::runtime_error("this build has no CPU (ONNX Runtime) backend");
#endif
  }
  throw std::runtime_error("unknown inference backend " + want);
}

}  // namespace jp
