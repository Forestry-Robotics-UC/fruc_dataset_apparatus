# FRUC Test Apparatus

# Docker 

To run the apparatus with docker just run:

```bash
cd docker/
docker-compose up
```

## Tag - mjpc13/ros:melodic-mynteye

### Software 

- OS: **Ubuntu 18.04**
- ROS: **Melodic**
- OpenCV: **3.4.1** compile from source 
- PCL: **1.7.2** compiled from source
- [MYNT SDK](https://github.com/slightech/MYNT-EYE-D-SDK) compiled from source
- MYNT ROS DRIVER
- catkin_tools

## Tag -  mjpc13/ros:noetic-semfire-apparatus

### Software 

- OS: **Ubuntu 20.04**
- ROS: **Noetic**
- [librealsense2](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
- [realsense-ros ](https://github.com/IntelRealSense/realsense-ros) wrapper
- catkin_tools

### Volume

Mount the src/ folder of this repository into "/opt/app/catkin_ws/src"

### Run Script

The default script to be run is the ``launch.sh``
```bash
#!/bin/bash
set -e

# setup ros environment
if [ -z "${SETUP}" ]; then
        # basic ros environment
        source "/opt/ros/$ROS_DISTRO/setup.bash"
else
       #from environment variable; should be a absolute path to the appropriate workspaces's setup.bash
        source $SETUP
fi  

cd /opt/app/catkin_ws/

catkin config --blacklist mynteye_wrapper_d
catkin build
roslaunch $ROSPACKAGE $LAUNCHFILE
```

## Environment Variables

- SETUP: The path for the setup.bash file used in the source. default: "opt/ros/$ROS_DISTRO/devel/setup.bash"
- LAUNCHFILE: The launch file to be executed when running the container
- ROSPACKAGE: The package of the desired launch file

# Known Issues

## Failed to set power-state

The first time **$ docker-compose up** command runs it causes the following error
```bash
ros-noetic_1   | [ INFO] [1649690810.431061260]:  
ros-noetic_1   |  11/04 15:26:50,453 ERROR [139972816103168] (handle-libusb.h:51) failed to open usb interface: 0, error: RS2_USB_STATUS_NO_DEVICE
ros-noetic_1   |  11/04 15:26:50,453 ERROR [139972355016448] (sensor.cpp:572) acquire_power failed: failed to set power state
ros-noetic_1   |  11/04 15:26:50,453 WARNING [139972355016448] (rs.cpp:310) null pointer passed for argument "device"
ros-noetic_1   | [ WARN] [1649690816.495320536]: Device 1/1 failed with exception: failed to set power state
ros-noetic_1   | [ERROR] [1649690816.495398671]: The requested device with  is NOT found. Will Try again.
```

Exit the docker-compose and **without deleting the containers** (don't run docker-compose down) re-run:
```bash
docker-compose up
```

The error can be caused by:
```bash
ros-noetic_1   | [build] Note: Workspace packages have changed, please re-source setup files to use them.
```
**TODO**: Check if this is the source of the bug and fix it


