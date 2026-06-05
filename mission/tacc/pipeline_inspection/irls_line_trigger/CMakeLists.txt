cmake_minimum_required(VERSION 3.8)
project(irls_line_trigger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vortex_msgs REQUIRED)

add_executable(irls_line_trigger_node
  src/irls_line_trigger_node.cpp
)

ament_target_dependencies(irls_line_trigger_node
  rclcpp
  std_srvs
  vortex_msgs
)

install(TARGETS
  irls_line_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
