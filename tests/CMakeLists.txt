find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(jp_tests core/version_test.cpp)
target_compile_definitions(jp_tests PRIVATE JP_EXPECTED_VERSION="${PROJECT_VERSION}")
target_link_libraries(jp_tests PRIVATE jp::core GTest::gtest_main jp_warnings)

gtest_discover_tests(jp_tests)
