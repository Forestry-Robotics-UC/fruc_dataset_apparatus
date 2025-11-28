#!/bin/bash

set -e
source /opt/ros/jazzy/setup.bash
cp -f /ros2_ws/shared/lidar_packet_handler.h /docker_ws/src/ouster-ros/ouster-ros/src/lidar_packet_handler.h
cd /docker_ws
colcon build --packages-up-to ouster_ros
source /docker_ws/install/setup.bash

ros2 launch ouster_ros driver.launch.py viz:=false params_file:=/ros2_ws/shared/ouster_config.yaml
