add_library(jp_zones zones.cpp)
add_library(jp::zones ALIAS jp_zones)

target_link_libraries(jp_zones
    PUBLIC jp::core Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json jp_warnings)
