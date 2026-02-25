#!/usr/bin/env bash
set -euo pipefail

# Avoid nounset failures in ROS setup scripts on some Jazzy images.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

TOPIC_MONITOR_AUTOSTART="${TOPIC_MONITOR_AUTOSTART:-0}"
TOPIC_MONITOR_TOPIC="${TOPIC_MONITOR_TOPIC:-}"
TOPIC_MONITOR_TOPICS="${TOPIC_MONITOR_TOPICS:-/diagnostics_agg,/diagnostics}"

if [[ "${TOPIC_MONITOR_AUTOSTART}" != "1" ]]; then
    echo "topic-monitor autostart disabled (TOPIC_MONITOR_AUTOSTART=${TOPIC_MONITOR_AUTOSTART})."
    exec tail -f /dev/null
fi

if [[ -z "${TOPIC_MONITOR_TOPIC}" ]]; then
    normalized_topics="${TOPIC_MONITOR_TOPICS//,/ }"
    read -r -a topic_array <<< "${normalized_topics}"
    if [[ "${#topic_array[@]}" -gt 0 ]]; then
        TOPIC_MONITOR_TOPIC="${topic_array[0]}"
    else
        TOPIC_MONITOR_TOPIC="/diagnostics_agg"
    fi
fi

echo "Starting topic_monitor for topic: ${TOPIC_MONITOR_TOPIC}"
help_text="$(ros2 run topic_monitor topic_monitor -h 2>&1 || true)"
if grep -q -- '--topic' <<< "${help_text}"; then
    exec ros2 run topic_monitor topic_monitor --topic "${TOPIC_MONITOR_TOPIC}"
fi
if grep -Eq 'usage: .*topic_monitor .*topic' <<< "${help_text}"; then
    exec ros2 run topic_monitor topic_monitor "${TOPIC_MONITOR_TOPIC}"
fi
echo "topic_monitor in this image does not accept a topic argument; running with defaults."
exec ros2 run topic_monitor topic_monitor
