cmake_minimum_required(VERSION 3.0.2)
project(pipeline_following)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)


catkin_package(

  CATKIN_DEPENDS rospy sensor_msgs

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




catkin_install_python(PROGRAMS
  scripts/pipeline_following_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
