cmake_minimum_required(VERSION 3.8)
project(line_detection_sonar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

set(CORE_LIB_NAME "${PROJECT_NAME}")

add_library(${CORE_LIB_NAME} SHARED
  src/lib/houghp_line_detection.cpp
)

target_include_directories(${CORE_LIB_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${CORE_LIB_NAME}
  vortex_utils
)

target_link_libraries(${CORE_LIB_NAME}
  ${OpenCV_LIBS})

set(COMPONENT_LIB_NAME "${PROJECT_NAME}_component")

add_library(${COMPONENT_LIB_NAME} SHARED
  src/ros/${PROJECT_NAME}_ros.cpp
  src/ros/${PROJECT_NAME}_ros_utils.cpp
)

ament_target_dependencies(${COMPONENT_LIB_NAME}
  rclcpp
  rclcpp_components
  vortex_utils
  vortex_utils_ros
  vortex_msgs
  cv_bridge
  sensor_msgs
  spdlog
  fmt
)

target_link_libraries(${COMPONENT_LIB_NAME} ${PROJECT_NAME})

rclcpp_components_register_node(
  ${COMPONENT_LIB_NAME}
  PLUGIN "LineDetectionSonarNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS
    ${CORE_LIB_NAME}
    ${COMPONENT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
ament_export_libraries(${CORE_LIB_NAME})

ament_package()
