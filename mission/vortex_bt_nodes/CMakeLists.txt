cmake_minimum_required(VERSION 3.8)
project(vortex_bt_nodes)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# One .cpp per node; new files under src/ are picked up on the next build.
file(GLOB_RECURSE NODE_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(vortex_bt_nodes ${NODE_SOURCES})

target_include_directories(vortex_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(vortex_bt_nodes PUBLIC
  rclcpp
  rclcpp_action
  behaviortree_cpp
  vortex_msgs
  geometry_msgs
)
target_link_libraries(vortex_bt_nodes PUBLIC spdlog::spdlog fmt::fmt)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vortex_bt_nodes
  EXPORT vortex_bt_nodes_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(vortex_bt_nodes_targets)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  vortex_msgs
  geometry_msgs
  spdlog
  fmt
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # One test file per node; each becomes its own test binary.
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    ament_add_gtest(${test_name} ${test_src} TIMEOUT 60)
    target_link_libraries(${test_name} vortex_bt_nodes)
  endforeach()
endif()

ament_package()
