FROM ros:melodic

ARG distro=melodic
ARG DEBIAN_FRONTEND=noninteractive
#SHELL ["/bin/bash", "-c"] 

#ROS package dependencies
RUN apt update && \
    apt install -y --no-install-recommends \
    python-pip


RUN pip install \
    numpy \
    scipy \
    statistics \
    turtle

# Assume the root contains vortex-cv, darknet_ros_zed and ddynamic_reconfigure_python
COPY ./vortex-cv /vortex_ws/src/vortex-cv
COPY ./darknet_ros_zed /vortex_ws/src/darknet_ros_zed
COPY ./ddynamic_reconfigure_python /vortex_ws/src/ddynamic_reconfigure_python

RUN source /opt/ros/$distro/setup.bash && \
    cd /vortex_ws && \
    catkin build
#ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]