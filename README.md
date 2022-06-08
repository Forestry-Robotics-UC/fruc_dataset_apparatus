# FRUC Test Apparatus

# Docker 

To run the apparatus with docker just run:

```bash
cd docker/
docker compose up
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

### Volume

Mount the src/ folder of this repository into "/opt/app/catkin_ws/src"

### Environment Variables

- **SETUP**: The path for the setup.bash file used in the source. default: "opt/ros/$ROS_DISTRO/devel/setup.bash"
- **BUILDLIST**: Packages to be compiled by catkin_tools
- **LAUNCHFILE**: The launch file to be executed when running the container
- **ROSPACKAGE**: The package of the desired launch file
- **XSENS_MODE**: Paramenters related to configuration of Xsens MTi Legacy
- **XSENS_SETTINGS**: Paramenters related to configuration of Xsens MTi Legacy

#### Xsens Options

Xsens allows configuration of their devices these environments change the configuration of the Legacy devices.

- **XSENS_MODE** - Select the information to output of Legacy infos. Can be a string composed of the following characters (in any order):
	- t temperature
	- c calibrated data
	- o orientation data
        - a auxiliary data
        - p position data (requires MTi-G)
        - v velocity data (requires MTi-G)
        - s status data
        - g raw GPS mode (requires MTi-G)
        - r raw (incompatible with others except raw GPS)

- **XSENS_SETTINGS** - Settings of the Legacy device. This is required for 'legacy-configure' command.Can be a string composed of the following characters (in any order):
	- t sample count (excludes 'n')
	- n no sample count (excludes 't')
	- u UTC time
	- q orientation in quaternion (excludes 'e' and 'm')
	- e orientation in Euler angles (excludes 'm' and 'q')
	- m orientation in matrix (excludes 'q' and 'e')
	- A acceleration in calibrated data
	- G rate of turn in calibrated data
	- M magnetic field in calibrated data
	- i only analog input 1 (excludes 'j')
	- j only analog input 2 (excludes 'i')
	- N North-East-Down instead of default: X North Z up

### Run Script

```bash
#!/bin/bash
set -e

#Configure the Xsens IMU
python /opt/app/catkin_ws/src/depends/ethzasl_xsens_driver/nodes/mtdevice.py -l --output-mode=$XSENS_MODE --output-settings=$XSENS_SETTINGS

#Build the catkin workspace
cd /opt/app/catkin_ws
catkin config --extend /opt/app/MYNT-EYE-S-SDK/wrappers/ros/devel --buildlist $BUILDLIST #only builds these packages
catkin build -v
source $SETUP

roslaunch $ROSPACKAGE $LAUNCHFILE #launch the file
```

## Tag -  mjpc13/ros:noetic-semfire-apparatus

### Software 

- OS: **Ubuntu 20.04**
- ROS: **Noetic**
- [librealsense2](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
- [realsense-ros ](https://github.com/IntelRealSense/realsense-ros) wrapper
- catkin_tools

### Volume

Mount the src/ folder of this repository into "/opt/app/catkin_ws/src"

### Environment Variables

- **SETUP**: The path for the setup.bash file used in the source. default: "opt/ros/$ROS_DISTRO/devel/setup.bash"
- **BUILDLIST**: Packages to be compiled by catkin_tools
- **LAUNCHFILE**: The launch file to be executed when running the container
- **ROSPACKAGE**: The package of the desired launch file

### Run Script

The default script to be run is the ``launch.sh``
```bash
#!/bin/bash
set -e

cd /opt/app/catkin_ws/
catkin config --buildlist $BUILDLIST
catkin build

# setup ros environment
if [ -z "${SETUP}" ]; then
        # basic ros environment
	source "/opt/ros/$ROS_DISTRO/setup.bash"
else
       #from environment variable; should be a absolute path to the appropriate workspaces's setup.bash
        source $SETUP
fi
roslaunch $ROSPACKAGE $LAUNCHFILE
```


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
docker compose up
```

The error can be caused by:
```bash
ros-noetic_1   | [build] Note: Workspace packages have changed, please re-source setup files to use them.
```
**TODO**: Check if this is the source of the bug and fix it

## Simultaneous launch of melodic and noetic

There is a chance of creating two different rosmaster (that leads to an error and crashes one of the containers). This is happening because the 2 containers are launching at practly the same time. Possible solution is to add a delay in one of the apparatus-\*.yml launch file.

A workaround is to run ```roscore``` first in the host computer.

