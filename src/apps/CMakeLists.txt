foreach(app IN ITEMS jp_server jp_run jp_calibrate jp_eval)
    add_executable(${app} ${app}.cpp)
    target_link_libraries(${app} PRIVATE jp::core CLI11::CLI11 jp_warnings)
endforeach()

target_link_libraries(jp_calibrate PRIVATE jp::benchmark jp::calibration opencv_imgcodecs
    spdlog::spdlog)
target_link_libraries(jp_run PRIVATE jp::io jp::detection jp::twin spdlog::spdlog)
