add_library(jp_twin class_colors.cpp debug_overlay.cpp bev_plot.cpp world_state_json.cpp)
add_library(jp::twin ALIAS jp_twin)

target_link_libraries(jp_twin
    PUBLIC jp::detection jp::localization opencv_core
    PRIVATE nlohmann_json::nlohmann_json opencv_imgproc opencv_videoio jp_warnings)
