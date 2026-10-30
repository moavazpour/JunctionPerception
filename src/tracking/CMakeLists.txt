add_library(jp_tracking box_kalman.cpp byte_tracker.cpp)
add_library(jp::tracking ALIAS jp_tracking)

target_link_libraries(jp_tracking
    PUBLIC jp::detection Eigen3::Eigen opencv_core
    PRIVATE jp::core jp_warnings)
