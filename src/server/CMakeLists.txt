find_package(Crow CONFIG REQUIRED)

add_library(jp_server_lib twin_server.cpp scene_routes.cpp site_clock.cpp video_encoder.cpp)
add_library(jp::server ALIAS jp_server_lib)

target_link_libraries(jp_server_lib
    PUBLIC jp::core jp::calibration jp::sources jp::zones
    PRIVATE jp::twin Crow::Crow nlohmann_json::nlohmann_json yaml-cpp::yaml-cpp spdlog::spdlog
        opencv_imgcodecs opencv_imgproc jp_warnings)
