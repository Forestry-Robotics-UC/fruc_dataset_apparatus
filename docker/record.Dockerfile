ARG ARCH=
ARG CORES=2
FROM ${ARCH}ros:noetic-ros-base

LABEL maintainer="Mario Cristovao <mjpc13@protonmail.com>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    # Basic utilities
    build-essential \
    apt-utils \
    curl \
    git \
    wget \
    vim \
    libncurses5-dev \
    qtbase5-dev \
    nano

# Install some python packages
RUN apt-get -y install \
    python3 \
    python3-pip \
    python3-serial \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep

RUN pip3 install pybind11 \
    catkin_tools;

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-rosfmt ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-rqt
#RUN apt-get install -y ros-${ROS_DISTRO}-realsense2-camera \ 
#    ros-${ROS_DISTRO}-realsense2-description \ 
#    ros-${ROS_DISTRO}-xacro \
#    ros-${ROS_DISTRO}-cv-bridge \
#    ros-${ROS_DISTRO}-robot-state-publisher \
#    ros-${ROS_DISTRO}-pcl-ros \
#    ros-${ROS_DISTRO}-image-transport-plugins

# Clean-up
WORKDIR /
RUN apt-get clean

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

# Clean-up
WORKDIR /
RUN apt-get clean

COPY record-launch.sh /record-launch.sh
RUN chmod +x /record-launch.sh

CMD ["bash"]
