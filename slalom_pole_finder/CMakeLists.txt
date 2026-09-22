cmake_minimum_required(VERSION 3.8)
project(slalom_pole_finder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(slalom_pole_finder_node
  src/slalom_pole_finder.cpp
  src/main.cpp
)

target_include_directories(slalom_pole_finder_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(slalom_pole_finder_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  vision_msgs
)

install(TARGETS
  slalom_pole_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
