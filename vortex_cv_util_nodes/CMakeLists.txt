cmake_minimum_required(VERSION 3.8)
project(vortex_cv_util_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)

# image_undistort
add_library(image_undistort_component SHARED
  src/image_undistort.cpp
)
target_include_directories(image_undistort_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(image_undistort_component
  rclcpp rclcpp_components sensor_msgs cv_bridge OpenCV
)
target_link_libraries(image_undistort_component yaml-cpp)
rclcpp_components_register_node(
  image_undistort_component
  PLUGIN "vortex_cv_util_nodes::ImageUndistort"
  EXECUTABLE image_undistort_node
)

# camera_info_sync
add_library(camera_info_sync_component SHARED
  src/camera_info_sync.cpp
)
target_include_directories(camera_info_sync_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_info_sync_component
  rclcpp rclcpp_components sensor_msgs
)
target_link_libraries(camera_info_sync_component yaml-cpp)
rclcpp_components_register_node(
  camera_info_sync_component
  PLUGIN "vortex_cv_util_nodes::CameraInfoSync"
  EXECUTABLE camera_info_sync_node
)

# image_roi_crop
add_library(image_roi_crop_component SHARED
  src/image_roi_crop.cpp
)
target_include_directories(image_roi_crop_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(image_roi_crop_component
  rclcpp rclcpp_components sensor_msgs cv_bridge OpenCV message_filters
)
rclcpp_components_register_node(
  image_roi_crop_component
  PLUGIN "vortex_cv_util_nodes::ImageRoiCrop"
  EXECUTABLE image_roi_crop_node
)

install(TARGETS
  image_undistort_component
  camera_info_sync_component
  image_roi_crop_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
