#!/bin/bash

podman exec recording pkill -SIGINT -f hector_recorder # Required for graceful shutdown, because it expects a CTRL+C from interactive terminal
podman stop recording
podman stop monitoring 
podman stop xsens
podman stop  emlid
podman stop foxglove-bridge
podman stop realsense
podman stop ouster
