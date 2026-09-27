cmake_minimum_required(VERSION 3.8)
project(docking_camera_yolo_direction_waypoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(LIB_NAME ${PROJECT_NAME}_lib)
add_library(${LIB_NAME} SHARED
  src/docking_camera_yolo_direction_waypoint.cpp
  src/docking_camera_yolo_direction_waypoint_ros.cpp
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  vision_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  vortex_msgs
  geometry_msgs
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "vortex::docking_camera_yolo_direction_waypoint::DockingCameraYoloDirectionWaypointNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
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
