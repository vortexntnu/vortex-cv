cmake_minimum_required(VERSION 3.8)
project(subsea_docking_setup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/subsea_docking.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
