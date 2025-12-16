#!/bin/bash
set -e
# Source ROS2
source /opt/ros/$ROS_DISTRO/setup.bash
# Source the docker container workspace if it exists
if [ -d /docker_ws/src ]; then
    cd /docker_ws
    source /docker_ws/install/setup.bash
fi

exec "$@"