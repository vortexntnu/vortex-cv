cmake_minimum_required(VERSION 3.8)
project(bearing_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

# --- ROS-independent library ---
add_library(bearing_localization_core SHARED
  src/lib/measurement_buffer.cpp
  src/lib/triangulation_solver.cpp
  src/lib/geometry_checks.cpp
  src/lib/bearing_localizer.cpp
)

target_include_directories(bearing_localization_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(bearing_localization_core
  yaml-cpp
  spdlog::spdlog
)

# --- ROS node executable ---
add_executable(bearing_localization_node
  src/ros/main.cpp
  src/ros/bearing_localization_node.cpp
  src/ros/debug_markers.cpp
)

target_include_directories(bearing_localization_node
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(bearing_localization_node
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  vortex_msgs
  Eigen3
  vortex_utils_ros
)

target_link_libraries(bearing_localization_node
  bearing_localization_core
)

install(
  TARGETS
    bearing_localization_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    bearing_localization_node
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(bearing_localization_core)

ament_package()
