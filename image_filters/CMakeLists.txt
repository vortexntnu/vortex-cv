cmake_minimum_required(VERSION 3.0.2)
project(image_filters)


find_package(catkin REQUIRED 
  COMPONENTS
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    roscpp
    dynamic_reconfigure
)

#########################
## Dynamic Reconfigure ##
#########################
generate_dynamic_reconfigure_options(
  cfg/image_filter_parameters.cfg
)
catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
)
#########################

find_package(OpenCV 4.2.0 REQUIRED 
  COMPONENTS 
    core 
    imgproc 
    highgui
    calib3d
    xphoto
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    image_filters
  CATKIN_DEPENDS 
    cv_bridge 
    image_transport 
    roscpp 
    sensor_msgs 
    std_msgs 
  DEPENDS 
    OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#########################################################

add_executable(${PROJECT_NAME}_node 
  src/image_filtering_node.cpp
  src/image_processing.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#########################################################