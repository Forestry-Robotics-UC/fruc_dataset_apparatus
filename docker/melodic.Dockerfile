ARG ARCH=
ARG CORES=2
FROM ${ARCH}ros:melodic-ros-base

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
    cmake \
    git \
    wget \
    vim \
    nano

# Install some python packages
RUN apt-get -y install \
    python \
    python-catkin-pkg \
    python-pip \
    python-serial \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-rosdep

RUN pip install pybind11 \
    catkin_tools;

# --- INSTALL MYNT EYE SDK ---
#Install OpenCV dependencies
RUN apt-get -y install pkg-config libgtk2.0-dev

#Install ssl for https, v4l for video
RUN apt-get -y install libssl-dev libv4l-dev v4l-utils

#Install OpenCV, Mynt Eye requires the 3.4.3 version of OpenCV
 WORKDIR /root
 RUN git clone https://github.com/opencv/opencv.git
 WORKDIR /root/opencv
 RUN git checkout tags/3.4.3
 RUN mkdir build
 WORKDIR /root/opencv/build
 RUN cmake \
         -DCMAKE_BUILD_TYPE=RELEASE \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DWITH_CUDA=OFF \
         -DBUILD_DOCS=OFF \
         -DBUILD_EXAMPLES=OFF \
         -DBUILD_TESTS=OFF \
         -DBUILD_PERF_TESTS=OFF \
         ..
 RUN make -j ${CORES} install
 ENV OpenCV_DIR=/root/opencv

#Install Mynt Eye SDK
WORKDIR /root
RUN git clone -b devel https://github.com/Forestry-Robotics-UC/MYNT-EYE-S-SDK.git MYNT-EYE-S-SDK
WORKDIR /root/MYNT-EYE-S-SDK/
RUN make init
RUN make install

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins

RUN source ${ROS_ROOT}/setup.bash && make ros #Source ROS and compile Mynt ros wrapper
#---

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Clean-up
WORKDIR /
RUN apt-get clean

#RUN echo "source /usr/local/bin/catkin_entrypoint.sh" >> /root/.bashrc
COPY melodic-launch.sh /melodic-launch.sh
RUN chmod +x /melodic-launch.sh

#ENTRYPOINT ["/noetic-launch.sh"]
CMD ["bash"]
