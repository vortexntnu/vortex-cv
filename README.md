# Vortex-CV
A repository of a set of modules for computer vision (CV) used and developed by Vortex NTNU in their AUV and ASV software stacks. The modules are meant to be pipelined serially with the end goal of classifying objects and determining their poses (positions and according orientations) in 3D space. Having that in mind, the individual modules are built to be able to use individually for singular goals, e.g. shape detection, pose estimation, etc.

## Table of contents
* [Overview](#overview)
* [Prerequisites](#prerequisites)
* [Setup](#setup)

## Overview
Dataflow diagram over the modules:
![dataflow diagram](https://github.com/vortexntnu/Vortex-CV/blob/documentation/repo_readme/docs/Vortex-CV_dataflow.jpg?raw=true)

Summary of the packages and their functionalities:
| Folder/Package        | Contents |
|-----------------------|--------------------------------------------------------------------------------------------------------------------------------|
| vision_launch         | A wrapper package for the whole stack - a top-level module for configurating and launching all of the necessary 'online' CV systems. |
| feature_detection     | Feature-based object detection and classification system, i.e. colour-, shape-, line-, point-, etc. fitting algorithm composites. |
| pointcloud_processing | Point(s) in 3D space and depth data handling for pose calculation of objects (point arrays) in the world. |
| object_ekf            | Object pose filtering and estimation using an Extended Kalman Filter. (WIP) |
| camera_calibration    | Calibration of intrinsic camera lense parameters, as well as extrinsic stereo camera parameters. Based on OpenCV calibration package. (WIP) |
| cv_utils              | A package meant for standalone CV utility programs/ROS launches. |
| cv_msgs               | Custom messages used in CV/OD pipeline. |

## Prerequisites
- Python 2.7
- ROS Melodic
- NumPy
- scikit-learn
- opencv-python
- [Dynamic dynamic reconfigure python](https://github.com/pal-robotics/ddynamic_reconfigure_python)
