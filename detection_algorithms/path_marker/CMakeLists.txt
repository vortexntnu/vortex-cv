cmake_minimum_required(VERSION 3.8)
project(path_marker_2d)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(path_marker_2d_node src/path_marker_node.cpp)
ament_target_dependencies(path_marker_2d_node rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs)
target_link_libraries(path_marker_2d_node ${OpenCV_LIBS})

install(TARGETS path_marker_2d_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
