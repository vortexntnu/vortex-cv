find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_docking_position_estimator
  test_docking_position_estimator.cpp
)

target_include_directories(test_docking_position_estimator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

ament_target_dependencies(test_docking_position_estimator
  rclcpp
  nav_msgs
  vortex_msgs
  Eigen3
)

target_link_libraries(test_docking_position_estimator
  ${PROJECT_NAME}_lib
)
