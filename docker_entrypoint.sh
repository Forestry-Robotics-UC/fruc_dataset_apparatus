#!/bin/bash
set -e

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Extract package names from first argument (supports comma-separated list)
PACKAGE_NAMES="${1:?Package name(s) required as first argument (comma-separated for multiple)}"
shift  # Remove the package names, shift remaining args to $@

# Build only the specific packages and their dependencies
# (ROS dependencies are pre-installed during docker build)
# Colcon searches recursively in docker_ws/src for all packages, including nested ones
if [ -d "/docker_ws/src" ]; then
    echo "[docker_entrypoint] Building packages and their dependencies: ${PACKAGE_NAMES}"
    cd /docker_ws

    # Debug: list available packages
    echo "[docker_entrypoint] Available packages in /docker_ws/src:"
    colcon list 2>/dev/null | head -20 || find src -name "package.xml" -o -name "CMakeLists.txt" | head -10

    # Convert comma-separated list to space-separated array
    IFS=',' read -ra PKG_ARRAY <<< "${PACKAGE_NAMES}"

    # Trim whitespace from each package name and collect into a clean array
    CLEAN_PKGS=()
    for pkg in "${PKG_ARRAY[@]}"; do
        pkg=$(echo "${pkg}" | xargs)
        # Convert hyphens to underscores (e.g. ouster-ros -> ouster_ros)
        pkg=$(echo "${pkg}" | tr '-' '_')
        CLEAN_PKGS+=("${pkg}")
        echo "[docker_entrypoint] Will build: ${pkg}"
    done

    # --packages-up-to accepts multiple package names as space-separated values
    # in a single flag, so all packages and their deps are resolved together.
    BUILD_CMD="colcon build --packages-up-to ${CLEAN_PKGS[*]} --symlink-install --continue-on-error"
    echo "[docker_entrypoint] Executing: ${BUILD_CMD}"
    colcon build --packages-up-to "${CLEAN_PKGS[@]}" --symlink-install --continue-on-error 2>&1 | tail -30

    echo "[docker_entrypoint] Package build complete!"
else
    echo "[docker_entrypoint] ⚠️  /docker_ws/src not found"
    echo "[docker_entrypoint] Continuing anyway..."
fi

# Source the docker container workspace if it exists
if [ -d "/docker_ws/install" ]; then
    echo "[docker_entrypoint] Sourcing /docker_ws/install/setup.bash"
    source /docker_ws/install/setup.bash
fi

echo "[docker_entrypoint] Starting service with packages: ${PACKAGE_NAMES}"
echo "[docker_entrypoint] Command: $@"

# If the first remaining arg is bash, we need to ensure the environment is sourced in the subshell
if [ "$1" = "bash" ] && [ "$2" = "-c" ]; then
    # Inject sourcing into the bash command
    shift 2  # Remove 'bash' and '-c'
    exec bash -c "source /opt/ros/jazzy/setup.bash && if [ -d /docker_ws/install ]; then source /docker_ws/install/setup.bash; fi && $*"
else
    # Execute the provided command directly
    exec "$@"
fi