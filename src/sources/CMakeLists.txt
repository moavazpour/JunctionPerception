add_library(jp_sources perception_pipeline.cpp world_sources.cpp)
add_library(jp::sources ALIAS jp_sources)

target_link_libraries(jp_sources
    PUBLIC jp::core jp::io jp::calibration jp::detection jp::tracking jp::localization opencv_core
    PRIVATE jp::benchmark jp::twin opencv_imgcodecs jp_warnings)
