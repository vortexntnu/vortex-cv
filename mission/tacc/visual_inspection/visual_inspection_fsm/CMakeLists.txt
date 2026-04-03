cmake_minimum_required(VERSION 3.8)
project(valve_inspection_fsm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_yasmin_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(valve_inspection_fsm
  src/blackboard.cpp
  src/valve_converge_state.cpp
  src/fsm_runner.cpp
)

ament_target_dependencies(valve_inspection_fsm
  rclcpp
  std_srvs
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_yasmin_utils
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(valve_inspection_fsm
  yasmin_viewer::yasmin_viewer
)

install(TARGETS
  valve_inspection_fsm
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


ament_package()
