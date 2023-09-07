FROM ros:noetic-ros-core

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-rosbridge-server \
      ros-${ROS_DISTRO}-tf2-ros && \   
    rm -rf /var/lib/apt/lists/*

# launch ros package
CMD ["roslaunch", "rosbridge_server", "rosbridge_websocket.launch"]