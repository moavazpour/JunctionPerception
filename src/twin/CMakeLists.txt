add_library(jp_twin debug_overlay.cpp)
add_library(jp::twin ALIAS jp_twin)

target_link_libraries(jp_twin
    PUBLIC jp::detection opencv_core
    PRIVATE opencv_imgproc opencv_videoio jp_warnings)
