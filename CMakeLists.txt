cmake_minimum_required(VERSION 3.25)

project(JunctionPerception
    VERSION 0.1.0
    DESCRIPTION "Intersection digital twin from a single fixed camera"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(JP_BUILD_TESTS "Build the unit tests" ON)
option(JP_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

add_library(jp_warnings INTERFACE)
target_compile_options(jp_warnings INTERFACE -Wall -Wextra -Wpedantic)
if(JP_WARNINGS_AS_ERRORS)
    target_compile_options(jp_warnings INTERFACE -Werror)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(CLI11 CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(OnnxRuntime REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d)
find_package(PROJ CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/benchmark)
add_subdirectory(src/calibration)
add_subdirectory(src/detection)
add_subdirectory(src/tracking)
add_subdirectory(src/localization)
add_subdirectory(src/twin)
add_subdirectory(src/vru)
add_subdirectory(src/sources)
add_subdirectory(src/server)
add_subdirectory(src/apps)

if(JP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
