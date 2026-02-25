#!/usr/bin/env bash
set -euo pipefail

# Avoid nounset failures in ROS setup scripts on some Jazzy images.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

TRACE_ROOT="${TRACE_ROOT:-/shared/traces}"
TRACE_SESSION_NAME="${TRACE_SESSION_NAME:-trace_$(date -u +%Y%m%dT%H%M%SZ)}"
TRACE_AUTOSTART="${TRACE_AUTOSTART:-0}"
TRACE_DURATION_SEC="${TRACE_DURATION_SEC:-}"

mkdir -p "${TRACE_ROOT}"

if [[ "${TRACE_AUTOSTART}" != "1" ]]; then
    echo "ros2_tracing autostart disabled (TRACE_AUTOSTART=${TRACE_AUTOSTART})."
    echo "Exec into this container to profile on demand, for example:"
    echo "  cd ${TRACE_ROOT}"
    echo "  ros2 trace ${TRACE_SESSION_NAME}"
    exec tail -f /dev/null
fi

cd "${TRACE_ROOT}"
if [[ -n "${TRACE_DURATION_SEC}" ]]; then
    timeout "${TRACE_DURATION_SEC}" ros2 trace "${TRACE_SESSION_NAME}" || true
else
    ros2 trace "${TRACE_SESSION_NAME}"
fi
