add_library(jp_benchmark openlabel.cpp tumtraf_sequence.cpp label_registration.cpp label_merge.cpp)
add_library(jp::benchmark ALIAS jp_benchmark)

target_link_libraries(jp_benchmark
    PUBLIC jp::core Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json jp_warnings)
