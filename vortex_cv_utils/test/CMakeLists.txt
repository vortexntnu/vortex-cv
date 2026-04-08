cmake_minimum_required(VERSION 3.8)

find_package(GTest REQUIRED)
include(GoogleTest)

set(TEST_BINARY_NAME ${PROJECT_NAME}_test)
add_executable(
    ${TEST_BINARY_NAME}
    test_image_conversions.cpp
    test_pose_conversions.cpp
)

target_link_libraries(
    ${TEST_BINARY_NAME}
    PRIVATE
    ${PROJECT_NAME}
    GTest::GTest
    GTest::gtest_main
)

gtest_discover_tests(${TEST_BINARY_NAME})
