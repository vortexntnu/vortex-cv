cmake_minimum_required(VERSION 3.0.2)
project(path_following_perception)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  dynamic_reconfigure
)


catkin_package(

  CATKIN_DEPENDS rospy sensor_msgs dynamic_reconfigure

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




catkin_install_python(PROGRAMS
  scripts/path_following_node.py
  scripts/dynam_recfg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
