ARG ARCH=
ARG CORES=2
FROM ${ARCH}ros:noetic-ros-base

LABEL maintainer="Mario Cristovao <mjpc13@protonmail.com>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]

# Add librealsense keys
RUN apt-get update && \
    apt-get install -y software-properties-common
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main"

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
    nano \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

#Instal Livox SDK
RUN apt install -y cmake
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /Livox-SDK/build
RUN cmake .. && make -j ${CORES} install

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
RUN apt-get install -y ros-${ROS_DISTRO}-realsense2-camera \ 
    ros-${ROS_DISTRO}-realsense2-description \ 
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-image-transport-plugins

# Clean-up
WORKDIR /
RUN apt-get clean
RUN rm -rf Livox-SDK/

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
#WORKDIR $CATKIN_WS

#Install proprietary Velodyne Drivers
WORKDIR /workspace/proprietary
ADD ./proprietary/ .
#COPY ./proprietary/velodyne-lidar-driver_2.7.0-23.01.30.1915_focal_amd64.deb velodyne-lidar-driver.deb


RUN apt-get update && apt-get install -y libgoogle-glog0v5 libconfig++9v5 libzmq3-dev

RUN dpkg -i ./velodyne-lidar-driver_2.7.0-23.01.30.1915_focal_amd64.deb \
            ./ros-noetic-velodyne-lidar-driver_2.7.0-23.01.30.1915_focal_amd64.deb

# Clean-up
WORKDIR /
RUN apt-get clean

#RUN echo "source /usr/local/bin/catkin_entrypoint.sh" >> /root/.bashrc
COPY noetic-launch.sh /noetic-launch.sh
RUN chmod +x /noetic-launch.sh

#ENTRYPOINT ["/noetic-launch.sh"]
CMD ["bash"]
