#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash

TOPIC_MONITOR_AUTOSTART="${TOPIC_MONITOR_AUTOSTART:-1}"

if [[ "${TOPIC_MONITOR_AUTOSTART}" != "1" ]]; then
    echo "topic-monitor autostart disabled (TOPIC_MONITOR_AUTOSTART=${TOPIC_MONITOR_AUTOSTART})."
    exec tail -f /dev/null
fi

echo "Starting topic_monitor..."
exec ros2 run topic_monitor topic_monitor
