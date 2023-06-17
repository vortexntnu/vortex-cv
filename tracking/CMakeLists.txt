cmake_minimum_required(VERSION 3.0.2)
project(tracking)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  cv_msgs
  dynamic_reconfigure
)


###################################
## catkin specific configuration ##
###################################
generate_dynamic_reconfigure_options(
   cfg/Tracking.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

# add_dependencies(MultiTargetTracker ${PROJECT_NAME}_gencfg)

# add_dependencies(
#    ${PROJECT_NAME}_node
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS}
# )

# install(
#    DIRECTORY cfg
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)






