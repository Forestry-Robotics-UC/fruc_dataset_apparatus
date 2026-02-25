#!/usr/bin/env bash
set -euo pipefail

# Avoid nounset failures in ROS setup scripts on some Jazzy images.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ "${DIAG_COMMON_AUTOSTART:-0}" != "1" ]]; then
    echo "diagnostic-common is idle by default (low-overhead mode)."
    echo "Set DIAG_COMMON_AUTOSTART=1 to run diagnostic_common_diagnostics monitors."
    echo "Optional: set DIAG_COMMON_MONITORS=\"cpu_monitor mem_monitor\""
    exec tail -f /dev/null
fi

available="$(ros2 pkg executables diagnostic_common_diagnostics 2>/dev/null | awk '{print $2}' || true)"
if [[ -z "${available}" ]]; then
    echo "No diagnostic_common_diagnostics executables found."
    exit 1
fi

monitors="${DIAG_COMMON_MONITORS:-cpu_monitor mem_monitor}"
started=0
pids=()

for monitor in ${monitors}; do
    if grep -Fxq "${monitor}" <<< "${available}"; then
        echo "Starting diagnostic_common_diagnostics/${monitor}"
        ros2 run diagnostic_common_diagnostics "${monitor}" &
        pids+=("$!")
        started=1
    else
        echo "Skipping unavailable monitor: ${monitor}"
    fi
done

if [[ "${started}" -eq 0 ]]; then
    echo "No requested monitors could be started."
    exit 1
fi

cleanup() {
    for pid in "${pids[@]}"; do
        kill "${pid}" >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT INT TERM

wait
