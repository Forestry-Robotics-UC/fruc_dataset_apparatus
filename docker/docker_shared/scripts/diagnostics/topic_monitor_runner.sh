#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash

if [[ "${TOPIC_MONITOR_AUTOSTART:-0}" != "1" ]]; then
    echo "topic-monitor is idle by default (low-overhead mode)."
    echo "Set TOPIC_MONITOR_AUTOSTART=1 to run: ros2 run topic_monitor topic_monitor"
    exec tail -f /dev/null
fi

echo "Starting topic_monitor..."
exec ros2 run topic_monitor topic_monitor
