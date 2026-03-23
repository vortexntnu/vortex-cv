cmake_minimum_required(VERSION 3.8)
project(vortex_cv_utils)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/image_conversions.cpp
  src/pose_conversions.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  vortex_utils
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
