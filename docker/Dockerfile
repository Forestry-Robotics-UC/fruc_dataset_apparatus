ARG ROS_DISTRO=jazzy

FROM ros:$ROS_DISTRO-ros-base as base

ENV PYTHON_VERSION=python3
ENV DEBIAN_FRONTEND=noninteractive

LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="Base ROS2 Image"


RUN apt-get update \
    && apt-get install -y git $PYTHON_VERSION-colcon-common-extensions $PYTHON_VERSION-pip $PYTHON_VERSION-rosdep

# FILE TO SOURCE THE ENVIRONMENT AND COMPILE ANY DEVEL WS THAT MIGHT EXIST
RUN echo '#!/bin/bash\n\
    set -e\n\
    # Source ROS2\n\
    source /opt/ros/$ROS_DISTRO/setup.bash\n\
    # Source the docker container workspace if it exists\n\
    if [ -d /docker_ws/src ]; then\n\
        cd /docker_ws\n\
        source /docker_ws/install/setup.bash\n\
    fi\n\
    # Source and compile the devel workspace if it exists\n\
    if [ -d /ros2_ws/src ]; then\n\
        cd /ros2_ws\n\
        colcon build --symlink-install \n\
        source /ros2_ws/install/setup.bash\n\
    fi\n\
    exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

WORKDIR /docker_ws/src/

#DOWNLOAD THE MONITOR TOOL
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor.git && rosdep update && rosdep install --from-paths . -y --ignore-src

WORKDIR /docker_ws
# BUILD THE ENVIRONMENT WITH THE MONITOR TOOL
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to greenwave_monitor

WORKDIR /

ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]

# REALSENSE
FROM base as realsense 

RUN apt install -y ros-$ROS_DISTRO-librealsense2* ros-$ROS_DISTRO-realsense2-*

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "launch", "realsense2_camera", "rs_launch.py" ]

# XSENS IMU
FROM base as xsens 

WORKDIR /docker_ws/src/

RUN git clone -b $ROS_DISTRO https://github.com/norlab-ulaval/norlab_xsens_driver.git && rosdep install --from-paths . --ignore-src  -r -y 

#Hotfix patch to a pressure error (force casting as a float) -- Only needed ROS2 Jazzy branch
RUN if [ "$ROS_DISTRO" = "jazzy" ]; then \
      sed -i "s/o\['Pressure'\]/float(o\['Pressure'\])/g" /docker_ws/src/norlab_xsens_driver/xsens_driver/mtnode.py; \
    fi
#RUN sed -i "s/o\['Pressure'\]/float(o\['Pressure'\])/g" /docker_ws/src/norlab_xsens_driver/xsens_driver/mtnode.py

WORKDIR /docker_ws

# BUILD DRIVER
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to xsens_driver

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "launch", "xsens_driver", "xsens_driver.launch.xml"]

# Ouster LiDAR
FROM base as ouster

ENV OUSTER_HOSTNAME="169.254.49.182"

WORKDIR /docker_ws/src/

RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && rosdep install --from-paths . -y --ignore-src

WORKDIR /docker_ws/

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to ouster_ros

RUN echo '#!/bin/bash\n\
    set -e\n\
    # Source ROS2\n\
    source /opt/ros/$ROS_DISTRO/setup.bash\n\
    # Source the docker container workspace if it exists\n\
    if [ -d /docker_ws/src ]; then\n\
        cd /docker_ws\n\
        source /docker_ws/install/setup.bash\n\
    fi\n\
    # Source and compile the devel workspace if it exists\n\
    if [ -d /ros2_ws/src ]; then\n\
        cd /ros2_ws\n\
        colcon build --symlink-install \n\
        source /ros2_ws/install/setup.bash\n\
    fi\n\
    cp -f /ros2_ws/shared/replay_runtime.launch.xml /docker_ws/install/ouster_ros/share/ouster_ros/launch/\n\
    exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "launch", "ouster_ros", "driver.launch.py", "viz:=false", "params_file:=/ros2_ws/shared/ouster_config.yaml" ]

#EMLID
FROM base as emlid

WORKDIR /docker_ws/src/

RUN git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git && rosdep install --from-paths . -y --ignore-src

WORKDIR /docker_ws

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to nmea_navsat_driver

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "run", "nmea_navsat_driver", "nmea_serial_driver" ]

#RECORDING
FROM base as recording

WORKDIR /docker_ws/src/

RUN apt update --fix-missing

RUN git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && git clone https://github.com/tu-darmstadt-ros-pkg/hector_recorder.git && rosdep install --from-paths . -y --ignore-src

WORKDIR /docker_ws/

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-up-to ouster_ros hector_recorder hector_recorder_msgs

RUN apt install -y ros-$ROS_DISTRO-librealsense2* ros-$ROS_DISTRO-realsense2-*

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "/bin/bash" ]

#STATE PUBLISHER
FROM base as publisher

WORKDIR /docker_ws/src/

RUN apt update --fix-missing

RUN apt install ros-jazzy-xacro

WORKDIR /docker_ws/

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args", "-p", "robot_description:='\$(xacro /ros2_ws/shared/apparatus.urdf.xacro)'" ]