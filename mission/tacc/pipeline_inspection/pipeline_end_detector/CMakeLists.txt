cmake_minimum_required(VERSION 3.8)
project(pipeline_end_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)

include_directories(include)

add_executable(pipeline_end_detector_node
  src/main.cpp
  src/pipeline_end_detector_node.cpp
)

target_include_directories(pipeline_end_detector_node
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(pipeline_end_detector_node
  rclcpp
  std_msgs
  std_srvs
  vortex_utils_ros
)

install(
  TARGETS pipeline_end_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
