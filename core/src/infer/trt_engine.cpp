#include "jp/infer/trt_engine.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace jp {
namespace {

class Logger : public nvinfer1::ILogger {
 public:
  void log(Severity s, const char* msg) noexcept override {
    if (s <= Severity::kERROR) {
      spdlog::error("[TensorRT] {}", msg);
    } else if (s == Severity::kWARNING) {
      spdlog::warn("[TensorRT] {}", msg);
    }
  }
};
Logger g_logger;

void check(cudaError_t e, const char* what) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

size_t elementSize(nvinfer1::DataType t) {
  switch (t) {
    case nvinfer1::DataType::kFLOAT: return 4;
    case nvinfer1::DataType::kHALF: return 2;
    case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kINT64: return 8;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kUINT8:
    case nvinfer1::DataType::kBOOL: return 1;
    default: return 4;
  }
}

std::string gpuTag() {
  cudaDeviceProp p{};
  int dev = 0;
  cudaGetDevice(&dev);
  cudaGetDeviceProperties(&p, dev);
  std::string name = p.name;
  for (auto& c : name)
    if (!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  return name + "_sm" + std::to_string(p.major) + std::to_string(p.minor) + "_trt" + std::to_string(NV_TENSORRT_MAJOR) +
         std::to_string(NV_TENSORRT_MINOR);
}
}  // namespace

struct TrtEngine::Impl {
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx;
  cudaStream_t stream = nullptr;
};

std::unique_ptr<TrtEngine> TrtEngine::fromOnnx(const std::string& onnx_path, const Options& opt) {
  namespace fs = std::filesystem;
  const std::string engine_path = onnx_path + "." + gpuTag() + (opt.int8 ? ".int8" : opt.fp16 ? ".fp16" : ".fp32") +
                                  ".engine";
  if (fs::exists(engine_path) && fs::last_write_time(engine_path) >= fs::last_write_time(onnx_path)) {
    return fromEngine(engine_path);
  }
  spdlog::info("Building TensorRT engine from {} (this can take a few minutes)", onnx_path);
  std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(g_logger));
  std::unique_ptr<nvinfer1::INetworkDefinition> net(builder->createNetworkV2(0));
  std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*net, g_logger));
  if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    throw std::runtime_error("failed to parse ONNX " + onnx_path);
  }
  std::unique_ptr<nvinfer1::IBuilderConfig> cfg(builder->createBuilderConfig());
  cfg->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, opt.workspace_mb << 20);
  if (opt.fp16) cfg->setFlag(nvinfer1::BuilderFlag::kFP16);
  std::unique_ptr<nvinfer1::IHostMemory> plan(builder->buildSerializedNetwork(*net, *cfg));
  if (!plan) throw std::runtime_error("TensorRT engine build failed for " + onnx_path);
  std::ofstream(engine_path, std::ios::binary).write(static_cast<const char*>(plan->data()), plan->size());
  spdlog::info("Cached engine at {}", engine_path);
  return fromEngine(engine_path);
}

std::unique_ptr<TrtEngine> TrtEngine::fromEngine(const std::string& engine_path) {
  std::ifstream f(engine_path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open engine " + engine_path);
  std::vector<char> blob((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::unique_ptr<TrtEngine> e(new TrtEngine());
  e->impl_ = std::make_unique<Impl>();
  e->impl_->runtime.reset(nvinfer1::createInferRuntime(g_logger));
  e->impl_->engine.reset(e->impl_->runtime->deserializeCudaEngine(blob.data(), blob.size()));
  if (!e->impl_->engine) throw std::runtime_error("failed to deserialize " + engine_path);
  e->impl_->ctx.reset(e->impl_->engine->createExecutionContext());
  check(cudaStreamCreate(&e->impl_->stream), "cudaStreamCreate");
  e->setup();
  return e;
}

void TrtEngine::setup() {
  auto& eng = *impl_->engine;
  for (int i = 0; i < eng.getNbIOTensors(); ++i) {
    TensorInfo t;
    t.name = eng.getIOTensorName(i);
    t.is_input = eng.getTensorIOMode(t.name.c_str()) == nvinfer1::TensorIOMode::kINPUT;
    const auto dims = eng.getTensorShape(t.name.c_str());
    const auto dt = eng.getTensorDataType(t.name.c_str());
    t.dtype = static_cast<int>(dt);
    size_t n = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      t.shape.push_back(dims.d[d]);
      n *= static_cast<size_t>(std::max<int64_t>(dims.d[d], 1));
    }
    t.bytes = n * elementSize(dt);
    void* ptr = nullptr;
    check(cudaMalloc(&ptr, t.bytes), "cudaMalloc");
    device_.push_back(ptr);
    impl_->ctx->setTensorAddress(t.name.c_str(), ptr);
    tensors_.push_back(t);
  }
}

const TensorInfo& TrtEngine::tensor(const std::string& name) const {
  for (const auto& t : tensors_)
    if (t.name == name) return t;
  throw std::runtime_error("no tensor named " + name);
}

void TrtEngine::infer(const std::vector<const void*>& host_inputs, const std::vector<void*>& host_outputs) {
  const auto t0 = std::chrono::steady_clock::now();
  size_t in = 0, out = 0;
  for (size_t i = 0; i < tensors_.size(); ++i) {
    if (tensors_[i].is_input) {
      check(cudaMemcpyAsync(device_[i], host_inputs.at(in++), tensors_[i].bytes, cudaMemcpyHostToDevice,
                            impl_->stream),
            "H2D");
    }
  }
  if (!impl_->ctx->enqueueV3(impl_->stream)) throw std::runtime_error("enqueueV3 failed");
  for (size_t i = 0; i < tensors_.size(); ++i) {
    if (!tensors_[i].is_input) {
      check(cudaMemcpyAsync(host_outputs.at(out++), device_[i], tensors_[i].bytes, cudaMemcpyDeviceToHost,
                            impl_->stream),
            "D2H");
    }
  }
  check(cudaStreamSynchronize(impl_->stream), "sync");
  last_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

TrtEngine::~TrtEngine() {
  for (void* p : device_) cudaFree(p);
  if (impl_ && impl_->stream) cudaStreamDestroy(impl_->stream);
}

}  // namespace jp
