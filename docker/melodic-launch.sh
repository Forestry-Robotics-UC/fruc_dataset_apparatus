#!/bin/bash
set -e

#Configure the Xsens IMU
#python /root/catkin_ws/src/depends/ethzasl_xsens_driver/nodes/mtdevice.py -l --output-mode=$XSENS_MODE --output-settings=$XSENS_SETTINGS

#Build the catkin workspace
cd /root/catkin_ws
catkin config --extend /root/MYNT-EYE-S-SDK/wrappers/ros/devel --whitelist $BUILDLIST #only builds these packages
catkin build -v
source $SETUP

roslaunch --wait $ROSPACKAGE $LAUNCHFILE #launch the file
