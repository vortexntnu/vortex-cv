cmake_minimum_required(VERSION 3.0.2)
project(aruco_detection)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
  vortex_msgs
  roscpp
  aruco_msgs
  aruco_ros
)

find_package(OpenCV REQUIRED COMPONENTS 
  core 
  imgproc 
  highgui
  aruco
)


# Find CUDA enabled OpenCV build on Xavier
if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
  find_package(OpenCV 3.2.0 REQUIRED)
else()
  find_package(OpenCV)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_detection
  CATKIN_DEPENDS cv_bridge 
                 image_transport 
                 roscpp 
                 sensor_msgs 
                 std_msgs 
                 vortex_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/aruco_detection_node.cpp)


# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
