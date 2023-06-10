cmake_minimum_required(VERSION 3.0.2)
project(image_preprocessing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

# Find CUDA enabled OpenCV build on Xavier
if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
  find_package(OpenCV)
else()
  find_package(OpenCV)
endif()

catkin_package(
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES image_preprocessing_cpp
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
 DEPENDS OpenCV #system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/image_preprocessing_cpp_node.cpp
  src/imagePreprocessing.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

catkin_install_python(PROGRAMS
  scripts/image_preprocessing_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



