#!/bin/bash
set -e

#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export FASTDDS_BUILTIN_TRANSPORTS=UDPv4


source /opt/ros/humble/setup.bash

#Build workspace
cd /root/ros2_ws/
colcon build --packages-select $BUILDLIST --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source /root/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py
