#!/bin/bash
# Initialize docker_ws with all required ROS repositories
# Run this script once before building Docker images

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_WS="${SCRIPT_DIR}/docker_ws"
SRC_DIR="${DOCKER_WS}/src"

echo "Initializing docker_ws at: ${DOCKER_WS}"

# Create directories if they don't exist
mkdir -p "${SRC_DIR}"

# Function to clone or update a repository
clone_repo() {
    local url=$1
    local branch=$2
    local dest=$3
    
    local repo_name=$(basename "${dest}")
    
    if [ -d "${dest}" ]; then
        echo "✓ Repository already exists: ${repo_name}"
    else
        echo "📥 Cloning ${repo_name}..."
        if [ -z "${branch}" ]; then
            git clone "${url}" "${dest}"
        else
            git clone -b "${branch}" "${url}" "${dest}"
        fi
    fi
}

# Clone all necessary repositories
echo ""
echo "=== Cloning ROS packages ==="

# Realsense packages
clone_repo "https://github.com/IntelRealSense/realsense-ros.git" "" "${SRC_DIR}/realsense-ros"

# Xsens driver
clone_repo "https://github.com/norlab-ulaval/norlab_xsens_driver.git" "jazzy" "${SRC_DIR}/norlab_xsens_driver"

# Ouster lidar
clone_repo "https://github.com/errorcodecritical/ouster-ros" "ros2" "${SRC_DIR}/ouster-ros"

# Hector recorder
clone_repo "https://github.com/tu-darmstadt-ros-pkg/hector_recorder.git" "" "${SRC_DIR}/hector_recorder"

# NMEA NavSat driver
clone_repo "https://github.com/ros-drivers/nmea_navsat_driver.git" "ros2" "${SRC_DIR}/nmea_navsat_driver"

# Apparatus-specific packages
echo ""
echo "=== Checking apparatus-specific packages ==="

if [ -d "${SRC_DIR}/apparatus_description" ]; then
    echo "✓ apparatus_description already exists"
else
    echo "📋 apparatus_description not found - it should be in docker_ws/src/"
    echo "   Make sure it's been added to the workspace"
fi

if [ -d "${SRC_DIR}/rslaunch" ]; then
    echo "✓ rslaunch already exists"
else
    echo "📋 rslaunch not found - it should be in docker_ws/src/"
    echo "   Make sure it's been added to the workspace"
fi

# Apply xsens hotfix
if [ -f "${SRC_DIR}/norlab_xsens_driver/xsens_driver/mtnode.py" ]; then
    echo "📝 Applying xsens pressure hotfix..."
    sed -i "s/o\\['Pressure'\\]/float(o\\['Pressure'\\])/g" "${SRC_DIR}/norlab_xsens_driver/xsens_driver/mtnode.py"
fi

# Sparse checkout for ouster to reduce storage
if [ -d "${SRC_DIR}/ouster-ros" ]; then
    echo "📦 Configuring sparse checkout for ouster-ros..."
    cd "${SRC_DIR}/ouster-ros"
    git sparse-checkout set --cone ouster-sensor-msgs 2>/dev/null || true
    git checkout ros2 2>/dev/null || true
    cd "${SCRIPT_DIR}"
fi

echo ""
echo "✅ Docker workspace initialization complete!"
echo ""
echo "Next steps:"
echo "1. Start the containers: docker compose up -d"
echo "2. Monitor build progress: docker logs -f <container_name>"
echo ""
