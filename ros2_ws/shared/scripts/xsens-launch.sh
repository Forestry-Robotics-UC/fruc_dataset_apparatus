#!/bin/bash
set -e

source /docker_ws/install/setup.bash

cat /ros2_ws/shared/scripts/mtdevice.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtdevice.py
cat /ros2_ws/shared/scripts/mtnode.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtnode.py

ros2 launch xsens_driver xsens_driver.launch.xml baudrate:=460800 device:=/dev/ttyUSB0
