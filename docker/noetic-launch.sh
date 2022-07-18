#!/bin/bash
set -e

cd /root/catkin_ws/
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