#!/bin/bash

#-----SELECT RECORDINGS NAME-------
# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Read a random line from the files located in the same directory as the script
name1=$(shuf -n 1 "$SCRIPT_DIR/.names_01")
name2=$(shuf -n 1 "$SCRIPT_DIR/.names_02")

default_name="$(date +%Y-%m-%d_%H-%M-%S)__${name2}_${name1}"

# Prompt user for recording name with default (using zenity instead of kdialog)
recording_name=$(zenity --entry --title="Recording Name" --text="Enter a name for the recording:" --entry-text="$default_name")

if [ $? -ne 0 ]; then
    zenity --error --text="Recording cancelled."
    exit 1
fi

#SELECT TOPICS FOR RECORDING
topics=$(zenity --list --checklist --title="Select Topics" --text="Topics to record" --column="Select" --column="Topic" --column="Description" \
    TRUE "/imu/data" "Xsens IMU Data" \
    TRUE "/imu/mag" "Xsens IMU Magnetometer" \
    TRUE "/heading" "Heading" \
    TRUE "/tf_static" "Static TF" \
    TRUE "/tf" "TF" \
    TRUE "/robot_description" "Robot Description" \
    TRUE "/ouster/lidar_packets" "Ouster LiDAR Packets" \
    TRUE "/ouster/imu_packets" "Ouster IMU Packets" \
    TRUE "/ouster/metadata" "Ouster Metadata" \
    FALSE "/ouster/imu" "Ouster IMU" \
    FALSE "/ouster/points" "Ouster Points" \
    FALSE "/ouster/nearir_image" "Ouster NearIR Image" \
    FALSE "/ouster/range_image" "Ouster Range Image" \
    FALSE "/ouster/reflec_image" "Ouster Reflectivity Image" \
    TRUE "/camera/color/image_raw" "Realsense Color" \
    TRUE "/camera/color/image_raw/compressed" "Realsense Color (JPEG Compressed)" \
    FALSE "/camera/color/image_raw/png" "Realsense Color (PNG Lossless)" \
    TRUE "/camera/color/camera_info" "Realsense Color Camera Info" \
    TRUE "/camera/aligned_depth_to_color/camera_info" "Realsense Aligned Depth to Color" \
    TRUE "/camera/aligned_depth_to_color/image_raw" "Realsense Align Depth" \
    FALSE "/camera/depth/image_rect_raw" "Realsense Depth" \
    TRUE "/camera/color/metadata" "Realsense Color Metadata" \
    TRUE "/camera/depth/metadata" "Realsense Depth Metadata" \
    TRUE "/camera/extrinsics/depth_to_color" "Realsense Extrinsics Depth to Color" \
    TRUE "/camera/extrinsics/depth_to_depth" "Realsense Extrinsics Depth to Depth" \
    FALSE "/camera/left/image_raw" "Realsense IR Left" \
    FALSE "/camera/right/image_raw" "Realsense IR Right" \
    TRUE "/fix" "GPS" \
    --separator=" " --width=700 --height=600)

if [ $? -ne 0 ]; then
    zenity --error --text="Recording cancelled."
    exit 1
fi

if [ -z "$topics" ]; then
    zenity --error --text="No topics selected."
    exit 1
fi

#SELECT HOW TO SPLIT THE BAG (time or size)
recording_limit_option=$(zenity --list --radiolist --title="Recording Limit" --text="Select recording limit (only one):" --column="Select" --column="Option" --column="Description" \
    TRUE "size" "Size (in GB)" \
    FALSE "duration" "Duration (in seconds)" \
    --width=400 --height=200)

if [ $? -ne 0 ]; then
    zenity --error --text="Recording cancelled."
    exit 1
fi

if [ "$recording_limit_option" == "size" ]; then
    bag_limit_value=$(zenity --entry --title="Bag Size" --text="Enter max bag size (GB):" --entry-text="5")
    if [ $? -ne 0 ] || [ -z "$bag_limit_value" ]; then
        zenity --error --text="Bag size input cancelled or invalid."
        exit 1
    fi

    # Convert GB to bytes using awk (rounded down to integer)
    bag_limit_bytes=$(awk -v gb="$bag_limit_value" 'BEGIN { print int(gb * 1073741824) }')
    bag_limit_flag="-b $bag_limit_bytes"
else
    bag_limit_value=$(zenity --entry --title="Bag Duration" --text="Enter max bag duration (seconds):" --entry-text="60")
    if [ $? -ne 0 ] || [ -z "$bag_limit_value" ]; then
        zenity --error --text="Bag duration input cancelled or invalid."
        exit 1
    fi
    bag_limit_flag="-d $bag_limit_value"
fi

#START THE RECORDING
zenity --question --title="Confirm Recording" --text="Start recording named:\n '$recording_name'\n\nTopics:\n$topics\n\nLimit: $bag_limit_flag" \
    --ok-label="Start Recording" --cancel-label="Cancel" --width=500

if [ $? -eq 0 ]; then
    #Launch the compose files
    cd "$SCRIPT_DIR"

    (sleep 8 && docker compose up -d publisher ouster realsense xsens emlid > /dev/null 2>&1) &

    cmd1="ros2 run hector_recorder record $bag_limit_flag --topics $topics -o /rosbags/$recording_name"
    cmd2="ros2 bag info /rosbags/$recording_name > /rosbags/$recording_name/info.txt"

    #Start recording container
    docker compose run --rm recording none $cmd1
    #After recording is done, get the bag info
    docker compose run --rm recording none /bin/bash -c "$cmd2"
else
    zenity --info --text="Recording Cancelled."
fi