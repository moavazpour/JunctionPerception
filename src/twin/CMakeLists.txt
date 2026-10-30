add_library(jp_twin class_colors.cpp debug_overlay.cpp bev_plot.cpp)
add_library(jp::twin ALIAS jp_twin)

target_link_libraries(jp_twin
    PUBLIC jp::detection jp::localization opencv_core
    PRIVATE opencv_imgproc opencv_videoio jp_warnings)
