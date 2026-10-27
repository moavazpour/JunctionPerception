set(JP_CORE_SOURCES
  src/geometry/camera.cpp
  src/tracking/tracker.cpp
  src/scene/site_map.cpp
  src/scene/intersection.cpp
  src/scene/user_map.cpp
  src/calib/refiner.cpp
  src/safety/near_miss.cpp
  src/health/monitors.cpp
  src/calib/auto_calibrator.cpp
  src/calib/manual_calibration.cpp
  src/ingest/video_source.cpp
  src/output/messages.cpp
  src/output/mqtt_sink.cpp
)

# Geometry, tracking, scene, safety, health, calibration and output: no GPU dependency.
add_library(jp_core ${JP_CORE_SOURCES})
target_include_directories(jp_core PUBLIC include)
target_link_libraries(jp_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS} nlohmann_json::nlohmann_json spdlog::spdlog
                                     yaml-cpp Threads::Threads)
if(JP_WITH_MQTT)
  target_link_libraries(jp_core PUBLIC jp_paho)
  target_compile_definitions(jp_core PUBLIC JP_HAVE_MQTT)
endif()

# Inference (TensorRT and/or ONNX Runtime) and the pipeline.
if(JP_WITH_TENSORRT OR JP_WITH_ONNXRUNTIME)
  add_library(jp_infer src/infer/engine.cpp src/infer/detector.cpp src/output/preview_server.cpp src/pipeline.cpp)
  target_link_libraries(jp_infer PUBLIC jp_core)
  if(JP_WITH_TENSORRT)
    target_sources(jp_infer PRIVATE src/infer/trt_engine.cpp)
    target_link_libraries(jp_infer PUBLIC jp_tensorrt)
    target_compile_definitions(jp_infer PUBLIC JP_HAVE_TENSORRT)
  endif()
  if(JP_WITH_ONNXRUNTIME)
    target_sources(jp_infer PRIVATE src/infer/ort_engine.cpp)
    target_link_libraries(jp_infer PUBLIC jp_onnxruntime)
    target_compile_definitions(jp_infer PUBLIC JP_HAVE_ORT)
  endif()
endif()
