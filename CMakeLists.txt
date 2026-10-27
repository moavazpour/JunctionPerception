cmake_minimum_required(VERSION 3.22)
project(JunctionPerception VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(JP_WITH_TENSORRT "Build TensorRT inference backend (NVIDIA GPU)" ON)
option(JP_WITH_ONNXRUNTIME "Build ONNX Runtime inference backend (CPU)" OFF)
option(JP_ORT_CUDA "The ONNX Runtime package has the CUDA execution provider (GPU, FP32)" OFF)
option(JP_WITH_MQTT "Build MQTT output adapter" ON)
option(JP_BUILD_TESTS "Build unit tests" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Dependencies)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_subdirectory(core)
add_subdirectory(apps)

if(JP_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
