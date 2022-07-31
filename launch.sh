#!/bin/bash

# The purpose of this file is to automate launching of 
# all nodes found in this repository FOR ROBOSUB 2022

cd ~/perception_ws
source devel/setup.bash
roslaunch vision_launch vision_launch.launch &
cd ~/sift_ws/src/Vortex-CV/sift_feature_detection && docker-compose up

