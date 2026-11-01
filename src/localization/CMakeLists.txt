add_library(jp_localization ground_projection.cpp bev_kalman.cpp box_fit.cpp track_localizer.cpp
    keypoint_footprint.cpp
    validated_region.cpp)
add_library(jp::localization ALIAS jp_localization)

target_link_libraries(jp_localization
    PUBLIC jp::core jp::detection jp::tracking Eigen3::Eigen opencv_core
    PRIVATE jp_warnings)
