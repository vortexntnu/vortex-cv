cmake_minimum_required(VERSION 3.8)
project(bearing_direction_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

set(BASE_DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  vortex_msgs
  vortex_utils_ros
)

# Shared base library
add_library(bearing_direction_base SHARED
  src/bearing_direction_base.cpp
)
target_include_directories(bearing_direction_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(bearing_direction_base ${BASE_DEPS})
target_link_libraries(bearing_direction_base spdlog::spdlog)

# --- Pipeline bearing server ---
add_executable(pipeline_bearing_server
  src/pipeline_main.cpp
  src/pipeline_bearing_node.cpp
)
target_include_directories(pipeline_bearing_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(pipeline_bearing_server ${BASE_DEPS})
target_link_libraries(pipeline_bearing_server bearing_direction_base spdlog::spdlog)

# --- Platform (subsea docking) bearing server ---
add_executable(platform_bearing_server
  src/platform_main.cpp
  src/platform_bearing_node.cpp
)
target_include_directories(platform_bearing_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(platform_bearing_server ${BASE_DEPS})
target_link_libraries(platform_bearing_server bearing_direction_base spdlog::spdlog)

# --- Acoustics bearing server ---
add_executable(acoustics_bearing_server
  src/acoustics_main.cpp
  src/acoustics_bearing_node.cpp
)
target_include_directories(acoustics_bearing_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(acoustics_bearing_server ${BASE_DEPS})
target_link_libraries(acoustics_bearing_server bearing_direction_base spdlog::spdlog)

# --- Test pipeline direction publisher ---
add_executable(test_pipeline_direction_node
  src/test_pipeline_direction_main.cpp
  src/test_pipeline_direction_node.cpp
)
target_include_directories(test_pipeline_direction_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(test_pipeline_direction_node ${BASE_DEPS})

target_link_libraries(test_pipeline_direction_node spdlog::spdlog)

# --- Test acoustics direction publisher ---
add_executable(test_acoustics_direction_node
  src/test_acoustics_direction_main.cpp
  src/test_acoustics_direction_node.cpp
)
target_include_directories(test_acoustics_direction_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(test_acoustics_direction_node ${BASE_DEPS})
target_link_libraries(test_acoustics_direction_node spdlog::spdlog)

install(TARGETS
  bearing_direction_base
  pipeline_bearing_server
  platform_bearing_server
  acoustics_bearing_server
  test_pipeline_direction_node
  test_acoustics_direction_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
