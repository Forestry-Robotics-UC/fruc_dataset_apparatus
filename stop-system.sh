#!/bin/bash

podman exec recording pkill -SIGINT -f hector_recorder # Required for graceful shutdown, because it expects a CTRL+C from interactive terminal
podman exec recording /bin/bash -c "
    source /opt/ros/jazzy/setup.bash && \
    LATEST_BAG=\$(ls -dt /rosbags/*/ | head -n 1 | sed 's#/rosbags/\\(.*\\)/#\\1#') && \
    ros2 bag info /rosbags/\$LATEST_BAG > /rosbags/\$LATEST_BAG/info.txt
"
podman stop recording
podman stop monitoring 
podman stop xsens
podman stop emlid
podman stop foxglove-bridge
podman stop realsense
podman stop ouster
