#!/bin/bash
set -e

source /docker_ws/install/setup.bash

XSENS_USE_SHARED_OVERLAY="${XSENS_USE_SHARED_OVERLAY:-1}"
XSENS_TIMEOUT="${XSENS_TIMEOUT:-0.002}"
XSENS_BAUDRATE="${XSENS_BAUDRATE:-460800}"
XSENS_DEVICE="${XSENS_DEVICE:-/dev/ttyUSB0}"
XSENS_FRAME_ID="${XSENS_FRAME_ID:-xsens_imu}"

if [[ "${XSENS_USE_SHARED_OVERLAY}" == "1" ]]; then
    echo "Applying shared Xsens overlay scripts."
    cat /shared/scripts/mtdevice.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtdevice.py
    cat /shared/scripts/mtnode.py > /docker_ws/src/norlab_xsens_driver/xsens_driver/mtnode.py

    PY_MTDEVICE_PATH="$(python3 - <<'PY'
import xsens_driver.mtdevice as m
print(m.__file__)
PY
)"
    PY_MTNODE_PATH="$(python3 - <<'PY'
import xsens_driver.mtnode as m
print(m.__file__)
PY
)"
    if [[ -n "${PY_MTDEVICE_PATH}" && -f "${PY_MTDEVICE_PATH}" ]]; then
        cat /shared/scripts/mtdevice.py > "${PY_MTDEVICE_PATH}"
    fi
    if [[ -n "${PY_MTNODE_PATH}" && -f "${PY_MTNODE_PATH}" ]]; then
        cat /shared/scripts/mtnode.py > "${PY_MTNODE_PATH}"
    fi
fi

echo "xsens_driver.mtdevice loaded from:"
python3 -c "import xsens_driver.mtdevice as d; print(d.__file__)"

ros2 launch xsens_driver xsens_driver.launch.xml \
    baudrate:="${XSENS_BAUDRATE}" \
    device:="${XSENS_DEVICE}" \
    frame_id:="${XSENS_FRAME_ID}" \
    timeout:="${XSENS_TIMEOUT}"
