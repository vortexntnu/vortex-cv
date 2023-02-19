cmake_minimum_required(VERSION 3.0.2)
project(aruco_detection)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  vortex_msgs
  roscpp
  # aruco_msgs
  # aruco_ros
)
find_package(OpenCV REQUIRED COMPONENTS 
  core 
  imgproc 
  highgui
  aruco
  calib3d
)
find_package(Eigen3 REQUIRED)

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
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/aruco_detection_node.cpp
  src/aruco_handler.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_test 
  test/test_aruco_handler.cpp
  src/aruco_handler.cpp
)
target_link_libraries(${PROJECT_NAME}_test
${catkin_LIBRARIES}
${OpenCV_LIBS}
)


# if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
# add_rostest_gtest(tests_mynode test/mynode.test src/test/test_mynode.cpp [more cpp files])
#   target_link_libraries(${PROJECT_NAME}_test
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
#   )
# endif()