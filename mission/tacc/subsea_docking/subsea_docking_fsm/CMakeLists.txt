cmake_minimum_required(VERSION 3.8)
project(subsea_docking_fsm)

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
find_package(vortex_yasmin_utils REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_executable(subsea_docking_fsm
  src/blackboard.cpp
  src/build_state_machine.cpp
  src/main.cpp
)

target_link_libraries(subsea_docking_fsm
spdlog::spdlog fmt::fmt
yasmin_viewer::yasmin_viewer
)

ament_target_dependencies(subsea_docking_fsm
  ament_index_cpp
  rclcpp
  rclcpp_action
  std_srvs
  yasmin
  yasmin_ros
  vortex_msgs
  vortex_utils
  vortex_yasmin_utils
)

install(TARGETS
  subsea_docking_fsm
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
