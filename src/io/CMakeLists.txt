add_library(jp_io frame_source.cpp)
add_library(jp::io ALIAS jp_io)

target_link_libraries(jp_io
    PUBLIC jp::core opencv_core
    PRIVATE opencv_imgcodecs opencv_videoio jp_warnings)
