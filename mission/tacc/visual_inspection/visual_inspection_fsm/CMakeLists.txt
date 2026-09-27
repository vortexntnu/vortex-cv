cmake_minimum_required(VERSION 3.8)
project(visual_inspection_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_yasmin_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(visual_inspection_fsm
  src/align_height_camera_state.cpp
  src/align_height_state.cpp
  src/converge_state.cpp
  src/retreat_state.cpp
  src/open_and_align_gripper_state.cpp
  src/twist_handle_state.cpp
  src/blackboard.cpp
  src/build_state_machine.cpp
  src/main.cpp
)

ament_target_dependencies(visual_inspection_fsm
  rclcpp
  std_srvs
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_utils
  vortex_utils_ros
  geometry_msgs
  vortex_yasmin_utils
  tf2_ros
  tf2
)

target_link_libraries(visual_inspection_fsm
  yasmin_viewer::yasmin_viewer
  yaml-cpp
)

install(TARGETS
  visual_inspection_fsm
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
