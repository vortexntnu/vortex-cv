cmake_minimum_required(VERSION 3.8)
project(vortex_yasmin_utils)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(vortex_yasmin_utils
  src/landmark_converge_state.cpp
  src/landmark_polling_state.cpp
  src/trigger_wait_state.cpp
)

target_include_directories(vortex_yasmin_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vortex_yasmin_utils PUBLIC
  rclcpp
  rclcpp_action
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_utils
  vortex_utils_ros
  geometry_msgs
  std_srvs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vortex_yasmin_utils
  EXPORT vortex_yasmin_utils_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(vortex_yasmin_utils_targets)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_utils
  vortex_utils_ros
  geometry_msgs
  std_srvs
)
ament_export_include_directories(include)

ament_package()
