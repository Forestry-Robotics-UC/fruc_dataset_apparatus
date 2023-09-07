ARG ARCH=
ARG CORES=2
FROM ${ARCH}ros:melodic-ros-core

LABEL maintainer="Mario Cristovao <mjpc13@protonmail.com>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    apt-utils \
    curl \
    cmake \
    git \
    wget \
    libv4l-dev \
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

#Install Dependencies (PCL, Eigen, OpenCV)
RUN apt install -y libpcl-dev libeigen3-dev libopencv-dev

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

#Install Livox mapping
RUN git clone https://github.com/Livox-SDK/livox_mapping.git $CATKIN_WS/src/livox_mapping
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git $CATKIN_WS/src/livox_ros_driver

RUN apt install -y ros-${ROS_DISTRO}-pcl-ros 

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
RUN mkdir -p Livox-SDK/build
RUN cd Livox-SDK/build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build . && sudo make install


# Clean-up
WORKDIR /
RUN apt-get clean

#RUN echo "source /usr/local/bin/catkin_entrypoint.sh" >> /root/.bashrc
COPY melodic-slam-launch.sh /melodic-slam-launch.sh
RUN chmod +x /melodic-slam-launch.sh

#ENTRYPOINT ["/noetic-launch.sh"]
CMD ["bash"]
