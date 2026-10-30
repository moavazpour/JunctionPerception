foreach(app IN ITEMS jp_server jp_run jp_calibrate jp_eval)
    add_executable(${app} ${app}.cpp)
    target_link_libraries(${app} PRIVATE jp::core CLI11::CLI11 jp_warnings)
endforeach()
