cmake_minimum_required(VERSION 3.8)
project(robosub_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# Every .cpp under src/nodes is picked up, so adding a node needs no change
# here (re-run colcon build; CONFIGURE_DEPENDS notices new files).
file(GLOB_RECURSE NODE_SOURCES CONFIGURE_DEPENDS src/nodes/*.cpp)

add_executable(robosub_mission
  ${NODE_SOURCES}
  src/register_nodes.cpp
  src/main.cpp
)

ament_target_dependencies(robosub_mission
  ament_index_cpp
  rclcpp
  behaviortree_cpp
)

target_link_libraries(robosub_mission
  spdlog::spdlog
  fmt::fmt
)

install(TARGETS
  robosub_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
