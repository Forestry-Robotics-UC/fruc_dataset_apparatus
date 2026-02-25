#!/usr/bin/env bash
set -euo pipefail

# Canonical diagnostics recording entrypoint.
# Uses recorder QoS overrides by default and supports lightweight overrides via env vars.
QOS_OVERRIDES_PATH="${QOS_OVERRIDES_PATH:-/shared/scripts/diagnostics/recording_qos_overrides.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/rosbags}"
OUTPUT_NAME="${OUTPUT_NAME:-diag_$(date -u +%Y%m%dT%H%M%SZ)}"
RECORD_DURATION_SEC="${RECORD_DURATION_SEC:-0}"
TOPICS="${TOPICS:-/ouster/lidar_packets /ouster/imu_packets /ouster/points /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /imu/data /fix /diagnostics /diagnostics_agg}"

if [[ ! -f "${QOS_OVERRIDES_PATH}" ]]; then
  echo "Missing QoS overrides file: ${QOS_OVERRIDES_PATH}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

read -r -a TOPIC_ARRAY <<< "${TOPICS}"
if [[ "${#TOPIC_ARRAY[@]}" -eq 0 ]]; then
  echo "No topics configured for recording." >&2
  exit 1
fi

cmd=(
  ros2 bag record
  --qos-profile-overrides-path "${QOS_OVERRIDES_PATH}"
  -o "${OUTPUT_DIR%/}/${OUTPUT_NAME}"
)
cmd+=("${TOPIC_ARRAY[@]}")

echo "qos_overrides=${QOS_OVERRIDES_PATH}"
echo "output=${OUTPUT_DIR%/}/${OUTPUT_NAME}"
echo "topics=${TOPICS}"

if [[ "${RECORD_DURATION_SEC}" != "0" ]]; then
  exec timeout "${RECORD_DURATION_SEC}" "${cmd[@]}"
fi

exec "${cmd[@]}"
