FROM ros:melodic

ARG distro=melodic
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"] 

#ROS dependencies
RUN apt update && \
    apt install -y --no-install-recommends \
    python-pip \
    python-catkin-tools \
    libx11-dev \
    tree

#ROS packages
RUN apt update && \
    apt install -y --no-install-recommends \
    ros-${distro}-tf \
    ros-${distro}-cv-bridge \
    ros-${distro}-image-transport 

# Assume the root contains vortex-cv, darknet_ros_zed and ddynamic_reconfigure_python
# This is the case when we build using ghcr
COPY ./vortex-cv                    /vortex_ws/src/vortex-cv
COPY ./vortex-msgs                  /vortex_ws/src/vortex-msgs
COPY ./darknet_ros_zed              /vortex_ws/src/darknet_ros_zed
COPY ./ddynamic_reconfigure_python  /vortex_ws/src/ddynamic_reconfigure_python

# catkin build 
RUN source /opt/ros/${distro}/setup.bash && \
    cd ./vortex_ws && \
    tree -d -L 5 && \
    catkin build

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]