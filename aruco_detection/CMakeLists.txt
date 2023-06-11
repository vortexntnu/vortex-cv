cmake_minimum_required(VERSION 3.0.2)
project(aruco_detection)


find_package(catkin REQUIRED 
  COMPONENTS
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    vortex_msgs
    roscpp
    tf2_ros
)


find_package(OpenCV 4.2.0 REQUIRED 
  COMPONENTS 
    core 
    imgproc 
    highgui
    aruco
    calib3d
    xphoto
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    aruco_detection
  CATKIN_DEPENDS 
    cv_bridge 
    image_transport 
    roscpp 
    sensor_msgs 
    std_msgs 
    vortex_msgs
  DEPENDS 
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

#########################################################

add_executable(${PROJECT_NAME}_docking_node 
  src/aruco_detection_node.cpp
  src/aruco_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_docking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#########################################################

add_executable(${PROJECT_NAME}_pipeline_node 
  src/aruco_id_node.cpp
  src/file_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_pipeline_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#########################################################















###############
### Testing ###
###############

add_executable(${PROJECT_NAME}_test_handler 
  test/test_aruco_handler.cpp
  src/aruco_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_test_handler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


