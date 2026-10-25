find_package(GTest REQUIRED)
add_executable(jp_tests
  test_geometry.cpp
  test_tracking.cpp
  test_safety.cpp
  test_calibration.cpp
)
target_link_libraries(jp_tests PRIVATE jp_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(jp_tests)
