cmake_minimum_required(VERSION 3.8)
project(pipeline_inspection_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

include_directories(include)

add_executable(pipeline_inspection_fsm
  src/search_waypoint_goal_state.cpp
  src/converge_state.cpp
  src/blackboard.cpp
  src/fsm_runner.cpp
)

ament_target_dependencies(pipeline_inspection_fsm
  ament_index_cpp
  rclcpp
  std_srvs
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_utils
  vortex_utils_ros
  geometry_msgs
  vortex_yasmin_utils
)

target_link_libraries(pipeline_inspection_fsm
  yasmin_viewer::yasmin_viewer
)

target_link_libraries(pipeline_inspection_fsm
  yasmin_viewer::yasmin_viewer
)

target_link_libraries(pipeline_inspection_fsm
  yasmin_viewer::yasmin_viewer
)

install(TARGETS
  pipeline_inspection_fsm
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
