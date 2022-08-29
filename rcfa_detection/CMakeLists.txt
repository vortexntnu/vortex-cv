cmake_minimum_required(VERSION 3.0.2)
project(rcfa_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekf_node
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/rcfa_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
