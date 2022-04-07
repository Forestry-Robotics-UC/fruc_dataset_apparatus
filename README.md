#FRUC Test Apparatus

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

