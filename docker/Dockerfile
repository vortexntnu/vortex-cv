FROM ros:melodic

ARG distro=melodic
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"] 


########################################################################
######################    Install OpenCV    ############################
########################################################################

# OpenCV Version 
ARG OPENCV_VERSION=4.2.0

# Install dependencies
RUN apt-get update || true && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install --no-install-recommends -y \
	build-essential gcc g++ \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    yasm libatlas-base-dev gfortran libpq-dev \
    libxine2-dev libglew-dev libtiff5-dev zlib1g-dev libavutil-dev libpostproc-dev \ 
    libeigen3-dev python3-dev python3-pip python3-numpy libx11-dev tzdata \
&& rm -rf /var/lib/apt/lists/*

# Set Working directory
WORKDIR /opt


# Install OpenCV from Source
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake \
	-D WITH_CUDA=ON \
	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/ \
	-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
	-D WITH_V4L=ON \
	-D WITH_QT=OFF \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D BUILD_EXAMPLES=OFF .. && \
   make -j"$(nproc)" && \
   make install

WORKDIR /


########################################################################
#######################    Install stuff    ############################
########################################################################


#ROS dependencies
RUN apt update && \
    apt install -y --no-install-recommends \
    python-pip \
    python-catkin-tools \
    libx11-dev 

#ROS packages
RUN apt update && \
    apt install -y --no-install-recommends \
    ros-${distro}-tf \
    ros-${distro}-cv-bridge \
    ros-${distro}-image-transport

RUN apt update && \
    apt install -y --no-install-recommends \
    apt-utils \
    libeigen3-dev \
    libglfw3-dev \
    libglew-dev \
    libjsoncpp-dev \
    ros-${distro}-tf \
    ros-${distro}-tf2 \
    ros-${distro}-tf2-geometry-msgs \
    libtclap-dev \
    wget \
    nano \ 
    protobuf-compiler \
    python-osrf-pycommon \
    python-openpyxl \
    python-yaml \
    python-enum34 \
    python-pip \
    python-catkin-tools \
    libgeographic-dev \
    ros-${distro}-vision-opencv \
    ros-${distro}-image-transport  \
    ros-${distro}-image-transport-plugins \
    ros-${distro}-ros-numpy \
    python-tk \
    software-properties-common \
    tk


# pip install
RUN pip install opencv-python==3.4.2.17 && \
    pip install opencv-contrib-python==3.4.2.17 \
    pip install statistics && \
    pip install scipy && \
    pip install matplotlib && \
    pip install sklearn && \
    pip install scikit-learn && \
    pip install plumbum


########################################################################
###########################    Build    ################################
########################################################################


# Assume the root contains vortex-cv, darknet_ros_zed and ddynamic_reconfigure_python
# This is the case when we build using ghcr
COPY ./vortex-cv                    /vortex_ws/src/vortex-cv
COPY ./vortex-msgs                  /vortex_ws/src/vortex-msgs
COPY ./darknet_ros_zed              /vortex_ws/src/darknet_ros_zed
COPY ./ddynamic_reconfigure_python  /vortex_ws/src/ddynamic_reconfigure_python

# catkin build 
RUN source /opt/ros/${distro}/setup.bash && \
    cd ./vortex_ws && \
    catkin build

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]