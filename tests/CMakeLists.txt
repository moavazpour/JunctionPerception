find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(jp_tests
    core/version_test.cpp
    core/assignment_test.cpp
    core/class_catalog_test.cpp
    benchmark/openlabel_test.cpp
    benchmark/label_fusion_test.cpp
    calibration/ground_geometry_test.cpp)
target_compile_definitions(jp_tests PRIVATE JP_EXPECTED_VERSION="${PROJECT_VERSION}"
    JP_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(jp_tests PRIVATE jp::core jp::benchmark jp::calibration GTest::gtest_main
    jp_warnings)

gtest_discover_tests(jp_tests)
