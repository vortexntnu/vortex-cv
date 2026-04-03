cmake_minimum_required(VERSION 3.8)
project(pipeline_inspection_setup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pipeline_inspection.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
