add_library(jp_detection letterbox.cpp yolo_output.cpp onnx_detector.cpp)
add_library(jp::detection ALIAS jp_detection)

target_link_libraries(jp_detection
    PUBLIC jp::core opencv_core
    PRIVATE OnnxRuntime::OnnxRuntime opencv_imgproc spdlog::spdlog jp_warnings)
