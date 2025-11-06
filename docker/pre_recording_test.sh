#!/bin/bash

# ======== Automated Pre-Recording Check ========

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Timestamped name
timestamp=$(date +%Y-%m-%d_%H-%M-%S)
recording_name="${timestamp}__pre_recording_check"

# Default topics (those marked "on" in your original script)
topics="/imu/data \
/ouster/lidar_packets \
/ouster/imu_packets \
/ouster/metadata \
/camera/camera/color/image_raw \
/camera/camera/color/camera_info \
/camera/camera/aligned_depth_to_color/image_raw \
/camera/camera/depth/metadata \
/fix"

# Split bags every 60 seconds
split_duration=60

# Stop recording after 7 minutes (420 seconds)
total_duration=420

# Storage preset profile
storage_profile="zstd_fast"

echo "==============================================="
echo " Starting Pre-Recording System Check"
echo " Bag Name:     ${recording_name}"
echo " Duration:     ${total_duration}s total"
echo " Split every:  ${split_duration}s"
echo " Storage:      ${storage_profile}"
echo " Topics:       ${topics}"
echo "==============================================="

# Launch containers if needed
cd "$SCRIPT_DIR/docker" || exit 1
podman-compose up -d

# Start recording in background
echo "Starting ROS2 bag recording..."
podman run --rm -t -d --name precheck_recording \
  --network docker_ros2-net \
  -v "$SCRIPT_DIR/rosbags:/rosbags" \
  base ros2 bag record \
  --storage-preset-profile "$storage_profile" \
  -d "$split_duration" \
  --topics $topics \
  -o "/rosbags/$recording_name"

# Wait for total duration (7 min)
echo "Recording for ${total_duration}s..."
sleep "$total_duration"

# Stop the recording container
echo "Stopping recording..."
podman stop precheck_recording >/dev/null 2>&1

echo "Recording finished."

# Analyze recorded bag
echo "Bag info and topic frequencies:"
echo "-----------------------------------------------"

# Find the latest bag directory
latest_bag=$(ls -td "$SCRIPT_DIR/rosbags"/"${recording_name}"* | head -n 1)

if [ -d "$latest_bag" ]; then
    podman run --rm -it \
      --network docker_ros2-net \
      -v "$SCRIPT_DIR/rosbags:/rosbags" \
      base bash -c "ros2 bag info /rosbags/$(basename "$latest_bag")"
else
    echo "No bag directory found!"
fi

echo "-----------------------------------------------"
echo "Pre-recording check complete."