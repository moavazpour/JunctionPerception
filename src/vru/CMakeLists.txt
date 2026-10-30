add_library(jp_vru zones.cpp event_detector.cpp)
add_library(jp::vru ALIAS jp_vru)

target_link_libraries(jp_vru
    PUBLIC jp::core Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json jp_warnings)
