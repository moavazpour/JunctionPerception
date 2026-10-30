add_library(jp_calibration ground_plane.cpp ground_homography.cpp benchmark_calibration.cpp
    ground_calibration.cpp control_points.cpp)
add_library(jp::calibration ALIAS jp_calibration)

target_link_libraries(jp_calibration
    PUBLIC jp::core jp::benchmark Eigen3::Eigen opencv_core
    PRIVATE opencv_imgproc opencv_calib3d yaml-cpp::yaml-cpp jp_warnings)
