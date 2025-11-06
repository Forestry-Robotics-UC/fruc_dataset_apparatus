#!/bin/bash

#-----SELECT RECORDINGS NAME-------
# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Read a random line from the files located in the same directory as the script
name1=$(shuf -n 1 "$SCRIPT_DIR/.names_01")
name2=$(shuf -n 1 "$SCRIPT_DIR/.names_02")

default_name="$(date +%Y-%m-%d_%H-%M-%S)__${name2}_${name1}"

# Prompt user for recording name with default
recording_name=$(kdialog --inputbox "Enter a name for the recording:" "$default_name")

if [ $? -ne 0 ]; then
    kdialog --sorry "Recording cancelled."
    exit 1
fi

#SELECT TOPICS FOR RECORDING
topics=$(kdialog --checklist "Topics to record" \
    /imu/data "Xsens IMU" on \
    /heading "Heading" on \
    /ouster/imu "Ouster imu" on \
    /ouster/points "Ouster Points" on \
    /ouster/nearir_image "Ouster NearIR Image" off \
    /ouster/range_image "Ouster Range Image" off \
    /ouster/reflec_image "Ouster Reflectivity Image" off \
    /camera/camera/color/image_raw "Realsense Color" on \
    /camera/camera/color/camera_info "Realsense Color Camera Info" on \
    /camera/camera/aligned_depth_to_color/image_raw "Realsense Align Depth" on \
    /camera/camera/depth/image_rect_raw "Realsense Depth" off \
    /camera/camera/depth/metadata "Realsense Depth Metadata" on \
    /camera/camera/depth/metadata "Realsense Extrinsics Color-Depth" on \
    /camera/camera/left/image_raw "Realsense IR Left" off \
    /camera/camera/right/image_raw "Realsense IR Right" off \
    /fix "GPS" on)
topics=$(echo $topics | sed 's/"//g')


if [ $? -ne 0 ]; then
    kdialog --sorry "Recording cancelled."
    exit 1
fi

if [ -z "$topics" ]; then
    kdialog --error "No topics selected."
    exit 1
fi

#SELECT HOW TO SPLIT THE BAG (time or size)
recording_limit_option=$(kdialog --radiolist "Select recording limit (only one):" \
    size "Size (in GB)" on \
    duration "Duration (in seconds)" off)

if [ $? -ne 0 ]; then
    kdialog --sorry "Recording cancelled."
    exit 1
fi

if [ "$recording_limit_option" == "size" ]; then
    bag_limit_value=$(kdialog --inputbox "Enter max bag size (GB):" "10")
    if [ $? -ne 0 ] || [ -z "$bag_limit_value" ]; then
        kdialog --error "Bag size input cancelled or invalid."
        exit 1
    fi
    
    # Convert GB to bytes using awk (rounded down to integer)
    bag_limit_bytes=$(awk -v gb="$bag_limit_value" 'BEGIN { print int(gb * 1073741824) }')
    bag_limit_flag="-b $bag_limit_bytes"
else
    bag_limit_value=$(kdialog --inputbox "Enter max bag duration (seconds):" "60")
    if [ $? -ne 0 ] || [ -z "$bag_limit_value" ]; then
        kdialog --error "Bag duration input cancelled or invalid."
        exit 1
    fi
    bag_limit_flag="-d $bag_limit_value"
fi

#SELECT THE COMPRESSION PROFILE
storage_profile=$(kdialog --combobox "Select storage preset profile:" none fastwrite zstd_fast zstd_small --default zstd_fast)
if [ $? -ne 0 ] || [ -z "$storage_profile" ]; then
    kdialog --error "No storage preset selected."
    exit 1
fi

#START THE RECORDING
confirm=$(kdialog --yesno "Start recording named:\n '$recording_name'\n\nTopics:\n$topics\n\nLimit: $bag_limit_flag\nStorage: --storage-preset-profile $storage_profile" \
    --yes-label "Start Recording" --no-label "Cancel")

if [ $? -eq 0 ]; then 
    kdialog --msgbox "Recording Started!"
    # Insert the commands/bring up of everything! TODO
    #Launch the compose files

    cd $SCRIPT_DIR/docker

    podman-compose up -d

    echo $topics
    formatted_topics=$(echo $topics | awk '{for(i=1;i<=NF;i++) printf "\"%s\"%s", $i, (i==NF?"":", ")}')

    echo $formatted_topics

    #Start recording stuff
    podman run --rm -t -d --name recording --network docker_ros2-net -v $SCRIPT_DIR/rosbags:/rosbags base ros2 bag record --storage-preset-profile $storage_profile $bag_limit_flag --topics $topics -o /rosbags/$recording_name

    podman run --rm -it --name monitoring --network docker_ros2-net base ros2 topic hz $topics

else
    kdialog --sorry "Recording Cancelled."
fi
