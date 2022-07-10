# FRUC Test Apparatus

# Docker 

To run the apparatus with docker just run:

```bash
cd docker/
docker compose up
```

# Services

## ros-master

The core image of ros noetic with ngrok to create the master.

### Software 

- OS: **Ubuntu 20.04**
- ROS: **Noetic**
- ngrok 

### Environment Variables

- **NGROK_TOKEN (Optional)**: The ngrok token to set up and connect to ngrok

## ros-melodic

A base image of ros melodic with the software necessary to use the IMU and Mynt Eye camera.

### Software 

- OS: **Ubuntu 18.04**
- ROS: **Melodic**
- OpenCV: **3.4.1** compile from source 
- [MYNT SDK](https://github.com/slightech/MYNT-EYE-D-SDK) compiled from source
- catkin_tools

### Environment Variables

- **SETUP**: The path for the setup.bash file used in the source. default: "opt/ros/$ROS_DISTRO/devel/setup.bash"
- **BUILDLIST**: Packages to be compiled by catkin_tools
- **LAUNCHFILE**: The launch file to be executed when running the container
- **ROSPACKAGE**: The package of the desired launch file
- **XSENS_MODE**: Paramenters related to configuration of the IMU Xsens MTi Legacy (see options below)
- **XSENS_SETTINGS**: Paramenters related to configuration of the IMU Xsens MTi Legacy (see options below)

#### Xsense Options

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

## ros-noetic

A base noetic image with the software necessary to use the Livox LiDAR and the Intel RealSense camera

### Software 

- OS: **Ubuntu 20.04**
- ROS: **Noetic**
- [librealsense2](https://github.com/IntelRealSense/librealsense/releases/tag/v2.50.0)
- [realsense-ros ](https://github.com/IntelRealSense/realsense-ros) wrapper
- Livox-SDK
- catkin_tools

## Environment Variables

- **SETUP**: The path for the setup.bash file used in the source. default: "opt/ros/$ROS_DISTRO/devel/setup.bash"
- **BUILDLIST**: Packages to be compiled by catkin_tools
- **LAUNCHFILE**: The launch file to be executed when running the container
- **ROSPACKAGE**: The package of the desired launch file