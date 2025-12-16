#!/bin/bash
set -e

source /docker_ws/install/setup.bash

cat /shared/scripts/mtdevice.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtdevice.py
cat /shared/scripts/mtnode.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtnode.py

ros2 launch xsens_driver xsens_driver.launch.xml baudrate:=460800 device:=/dev/ttyUSB0 frame_id:=xsens_imu

