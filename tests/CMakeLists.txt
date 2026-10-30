find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(jp_tests
    core/version_test.cpp
    core/assignment_test.cpp
    core/class_catalog_test.cpp
    core/config_and_statistics_test.cpp
    core/geodesy_test.cpp
    io/frame_source_test.cpp
    benchmark/openlabel_test.cpp
    benchmark/label_fusion_test.cpp
    calibration/ground_geometry_test.cpp
    calibration/control_points_test.cpp
    detection/detection_test.cpp
    detection/rider_association_test.cpp
    tracking/byte_tracker_test.cpp
    localization/localization_test.cpp
    localization/box_fit_test.cpp
    twin/world_state_json_test.cpp
    vru/zones_events_test.cpp
    server/server_config_test.cpp)
target_compile_definitions(jp_tests PRIVATE JP_EXPECTED_VERSION="${PROJECT_VERSION}"
    JP_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(jp_tests PRIVATE jp::core jp::io jp::benchmark jp::calibration
    jp::detection jp::tracking jp::localization jp::twin jp::vru jp::sources jp::server
    nlohmann_json::nlohmann_json opencv_imgcodecs GTest::gtest_main jp_warnings)

gtest_discover_tests(jp_tests)
