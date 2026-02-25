#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
DOCKER_DIR="${REPO_ROOT}/docker"
OUTPUT_ROOT="${DOCKER_DIR}/diagnostics_runs"

RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
OUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "${OUT_DIR}"

# FRUC Steam Deck preset (this diagnostics flow is scoped to fruc_dataset_apparatus).
DIAG_SAMPLE_SEC="${DIAG_SAMPLE_SEC:-6}"
DIAG_ENABLE_BW="${DIAG_ENABLE_BW:-0}"
DIAG_ENABLE_MULTICAST="${DIAG_ENABLE_MULTICAST:-0}"
DIAG_COMPOSE_FILES="${DIAG_COMPOSE_FILES:-docker-compose.yml,docker-compose.debug.yml}"
DIAG_COMPOSE_IMPL="${DIAG_COMPOSE_IMPL:-podman}"
DIAG_USE_OUSTER_POINTS="${DIAG_USE_OUSTER_POINTS:-1}"
DIAG_OUSTER_POINTS_TOPIC="${DIAG_OUSTER_POINTS_TOPIC:-/ouster/points}"
DIAG_RECORDING_QOS_OVERRIDES="${DIAG_RECORDING_QOS_OVERRIDES:-${REPO_ROOT}/docker/docker_shared/scripts/diagnostics/recording_qos_overrides.yaml}"
DIAG_RECORDING_RUNNER_IN_CONTAINER="${DIAG_RECORDING_RUNNER_IN_CONTAINER:-/shared/scripts/diagnostics/record_with_qos_overrides.sh}"
DIAG_RECORDING_RUNNER_HOST="${DIAG_RECORDING_RUNNER_HOST:-${REPO_ROOT}/docker/docker_shared/scripts/diagnostics/record_with_qos_overrides.sh}"
DIAG_RECORD_TOPICS="${DIAG_RECORD_TOPICS:-}"
DIAG_CAMERA_IMAGE_TOPICS="${DIAG_CAMERA_IMAGE_TOPICS:-/camera/color/image_raw,/camera/aligned_depth_to_color/image_raw}"
DIAG_CAMERA_METADATA_TOPICS="${DIAG_CAMERA_METADATA_TOPICS:-/camera/color/metadata,/camera/depth/metadata}"
DIAG_ALIGNMENT_LIDAR_TOPIC="${DIAG_ALIGNMENT_LIDAR_TOPIC:-}"
DIAG_LIDAR_NIC="${DIAG_LIDAR_NIC:-}"
DIAG_HARDWARE_PROBE_SERVICE="${DIAG_HARDWARE_PROBE_SERVICE:-debug-host}"
DIAG_WINDOW_PROBE_IN_CONTAINER="${DIAG_WINDOW_PROBE_IN_CONTAINER:-/shared/scripts/diagnostics/window_topic_probe.py}"
DIAG_WINDOW_PROBE_HOST="${DIAG_WINDOW_PROBE_HOST:-${REPO_ROOT}/docker/docker_shared/scripts/diagnostics/window_topic_probe.py}"
DIAG_CPU_SAMPLE_INTERVAL_SEC="${DIAG_CPU_SAMPLE_INTERVAL_SEC:-1}"

PRIMARY_SERVICES=(
  realsense
  xsens
  ouster
  emlid
  publisher
  recording
  debug-bridge
  debug-host
  diagnostic-common
  topic-monitor
  diagnostic-aggregator
  ros2-tracing
)
SENSOR_TOPICS=(
  /ouster/lidar_packets
  /ouster/imu_packets
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /imu/data
  /fix
  /diagnostics
  /diagnostics_agg
)

STAMPED_TOPICS=(
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /imu/data
  /fix
  /diagnostics
  /diagnostics_agg
)

XSENS_REQUIRED_TOPICS=(
  /imu/data
  /imu/mag
)
XSENS_OPTIONAL_TOPICS=(
  /imu_data_str
  /diagnostics
)

COMPOSE_CMD=()
COMPOSE_FILE_ARGS=()
RUNTIME_SENSOR_TOPICS=()
RUNTIME_STAMPED_TOPICS=()
RUNTIME_OUSTER_POINTS_TOPIC=""
RUNTIME_CHECK_TOPICS=()
RUNTIME_RECORD_TOPICS=()
RUNTIME_CAMERA_IMAGE_TOPICS=()
RUNTIME_CAMERA_METADATA_TOPICS=()
RUNTIME_CAMERA_PAIRS=()
RUNTIME_ALIGNMENT_LIDAR_TOPIC=""
RUNTIME_HARDWARE_PROBE_SERVICE=""
RUNTIME_LIDAR_NIC=""

log() {
  printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"
}

warn() {
  printf '[%s] WARNING: %s\n' "$(date -u +%H:%M:%S)" "$*" >&2
}

csv_to_topics() {
  local csv="$1"
  tr ', ' '\n\n' <<< "${csv}" | sed '/^$/d'
}

dedupe_topics() {
  awk 'NF && !seen[$0]++'
}

pick_hardware_probe_service() {
  local fallback_service="$1"
  local candidate
  for candidate in "${DIAG_HARDWARE_PROBE_SERVICE}" debug-host debug-bridge "${fallback_service}"; do
    [[ -z "${candidate}" ]] && continue
    if service_running "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done
  echo "${fallback_service}"
}

detect_lidar_nic() {
  local hw_service="$1"
  local sensor_host=""
  local nic=""

  if [[ -n "${DIAG_LIDAR_NIC}" ]]; then
    echo "${DIAG_LIDAR_NIC}"
    return 0
  fi

  sensor_host="$(awk '/sensor_hostname:/ {gsub(/"/, "", $2); print $2; exit}' "${REPO_ROOT}/docker/docker_shared/ouster_config.yaml" 2>/dev/null || true)"
  if [[ -z "${sensor_host}" ]]; then
    return 0
  fi

  nic="$("${COMPOSE_CMD[@]}" exec -T "${hw_service}" bash -lc "
set -e
ip route get ${sensor_host} 2>/dev/null | awk '{for (i = 1; i <= NF; ++i) if (\$i == \"dev\") {print \$(i + 1); exit}}'
" 2>/dev/null | tr -d '\r' || true)"
  echo "${nic}"
}

capture_hardware_snapshot() {
  local phase="$1"
  local hw_service="$2"
  local lidar_nic="$3"
  local out_file="${OUT_DIR}/task4_hw_${phase}.log"

  "${COMPOSE_CMD[@]}" exec -T "${hw_service}" bash -lc "
set -e
echo \"phase=${phase}\"
echo \"service=${hw_service}\"
echo \"lidar_nic=${lidar_nic:-none}\"
echo
echo \"# lsusb -t\"
lsusb -t || true
echo
echo \"# ip -s link\"
ip -s link || true
if [[ -n \"${lidar_nic}\" ]]; then
  echo
  echo \"# nic stats (${lidar_nic})\"
  for k in rx_errors tx_errors rx_dropped tx_dropped rx_packets tx_packets; do
    v=\$(cat /sys/class/net/${lidar_nic}/statistics/\${k} 2>/dev/null || echo unavailable)
    echo \"nic_stat_\${k}=\${v}\"
  done
  echo
  echo \"# ethtool -S ${lidar_nic}\"
  ethtool -S ${lidar_nic} || ethtool ${lidar_nic} || true
fi
" > "${out_file}" 2>&1 || true
}

build_compose_args() {
  local file
  local trimmed
  local compose_files=()
  COMPOSE_FILE_ARGS=()
  IFS=',' read -r -a compose_files <<< "${DIAG_COMPOSE_FILES}"
  for file in "${compose_files[@]}"; do
    trimmed="$(echo "${file}" | xargs)"
    [[ -z "${trimmed}" ]] && continue
    COMPOSE_FILE_ARGS+=(-f "${trimmed}")
  done
}

detect_compose() {
  build_compose_args

  case "${DIAG_COMPOSE_IMPL}" in
    docker)
      if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
        COMPOSE_CMD=(docker compose "${COMPOSE_FILE_ARGS[@]}")
        return
      fi
      echo "DIAG_COMPOSE_IMPL=docker but docker compose is unavailable." >&2
      exit 1
      ;;
    podman)
      if command -v podman-compose >/dev/null 2>&1; then
        COMPOSE_CMD=(podman-compose "${COMPOSE_FILE_ARGS[@]}")
        return
      fi
      echo "DIAG_COMPOSE_IMPL=podman but podman-compose is unavailable." >&2
      exit 1
      ;;
    auto)
      if command -v podman-compose >/dev/null 2>&1; then
        COMPOSE_CMD=(podman-compose "${COMPOSE_FILE_ARGS[@]}")
        return
      fi
      if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
        COMPOSE_CMD=(docker compose "${COMPOSE_FILE_ARGS[@]}")
        return
      fi
      echo "Could not find docker compose or podman-compose." >&2
      exit 1
      ;;
    *)
      echo "Invalid DIAG_COMPOSE_IMPL='${DIAG_COMPOSE_IMPL}'. Use auto, docker, or podman." >&2
      exit 1
      ;;
  esac
}

service_running() {
  local service="$1"
  local cid
  cid="$("${COMPOSE_CMD[@]}" ps -q "${service}" 2>/dev/null || true)"
  [[ -n "${cid}" ]]
}

ensure_service_running() {
  local service="$1"
  if service_running "${service}"; then
    return 0
  fi
  if "${COMPOSE_CMD[@]}" up -d "${service}" >/dev/null 2>&1; then
    log "Started default diagnostics service: ${service}"
    return 0
  fi
  warn "Could not start default diagnostics service '${service}'."
  return 1
}

running_services() {
  local service
  for service in "${PRIMARY_SERVICES[@]}"; do
    if service_running "${service}"; then
      echo "${service}"
    fi
  done
}

pick_probe_service() {
  local ordered=(
    debug-bridge
    debug-host
    publisher
    realsense
    ouster
    xsens
    emlid
    recording
    topic-monitor
    diagnostic-common
    diagnostic-aggregator
    ros2-tracing
  )
  local candidate
  for candidate in "${ordered[@]}"; do
    if service_running "${candidate}"; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

sanitize_topic() {
  local topic="$1"
  topic="${topic#/}"
  echo "${topic//\//_}"
}

is_stamped_topic() {
  local topic="$1"
  local candidate
  for candidate in "${RUNTIME_STAMPED_TOPICS[@]}"; do
    if [[ "${candidate}" == "${topic}" ]]; then
      return 0
    fi
  done
  return 1
}

detect_ouster_points_topic() {
  local probe_service="$1"
  local topic_list
  local candidate
  local candidates=()

  topic_list="$("${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc 'ros2 topic list 2>/dev/null || true' 2>/dev/null || true)"
  [[ -z "${topic_list}" ]] && return 0

  if [[ -n "${DIAG_OUSTER_POINTS_TOPIC}" ]]; then
    candidates+=("${DIAG_OUSTER_POINTS_TOPIC}")
  fi
  candidates+=(
    /ouster/points
    /ouster/points_raw
    /ouster/points2
    /ouster/point_cloud
    /ouster/point_cloud2
  )

  for candidate in "${candidates[@]}"; do
    if grep -Fxq "${candidate}" <<< "${topic_list}"; then
      echo "${candidate}"
      return 0
    fi
  done
}

build_runtime_topics() {
  local probe_service="$1"
  local points_topic=""

  RUNTIME_SENSOR_TOPICS=("${SENSOR_TOPICS[@]}")
  RUNTIME_STAMPED_TOPICS=("${STAMPED_TOPICS[@]}")
  RUNTIME_OUSTER_POINTS_TOPIC=""

  if [[ "${DIAG_USE_OUSTER_POINTS}" == "1" ]]; then
    points_topic="$(detect_ouster_points_topic "${probe_service}")"
    if [[ -n "${points_topic}" ]]; then
      RUNTIME_OUSTER_POINTS_TOPIC="${points_topic}"
      RUNTIME_SENSOR_TOPICS+=("${points_topic}")
      RUNTIME_STAMPED_TOPICS+=("${points_topic}")
    fi
  fi
}

main() {
  detect_compose
  cd "${DOCKER_DIR}"

  ensure_service_running topic-monitor || true
  ensure_service_running ros2-tracing || true

  mapfile -t RUNNING_SERVICES < <(running_services)
  if [[ "${#RUNNING_SERVICES[@]}" -eq 0 ]]; then
    echo "No target services are running. Start containers first, then rerun." >&2
    exit 1
  fi

  local probe_service
  probe_service="$(pick_probe_service)"
  if [[ -z "${probe_service}" ]]; then
    echo "Could not pick a probe service." >&2
    exit 1
  fi
  build_runtime_topics "${probe_service}"

  mapfile -t RUNTIME_CAMERA_IMAGE_TOPICS < <(csv_to_topics "${DIAG_CAMERA_IMAGE_TOPICS}" | dedupe_topics)
  mapfile -t RUNTIME_CAMERA_METADATA_TOPICS < <(csv_to_topics "${DIAG_CAMERA_METADATA_TOPICS}" | dedupe_topics)
  RUNTIME_CAMERA_PAIRS=()
  local image_topic_count="${#RUNTIME_CAMERA_IMAGE_TOPICS[@]}"
  local metadata_topic_count="${#RUNTIME_CAMERA_METADATA_TOPICS[@]}"
  local camera_pair_count=0
  if [[ "${image_topic_count}" -lt "${metadata_topic_count}" ]]; then
    camera_pair_count="${image_topic_count}"
  else
    camera_pair_count="${metadata_topic_count}"
  fi
  local idx
  for ((idx=0; idx<camera_pair_count; idx++)); do
    RUNTIME_CAMERA_PAIRS+=("${RUNTIME_CAMERA_IMAGE_TOPICS[idx]}:${RUNTIME_CAMERA_METADATA_TOPICS[idx]}")
  done
  if [[ "${image_topic_count}" -ne "${metadata_topic_count}" ]]; then
    warn "Camera image topic count (${image_topic_count}) != metadata topic count (${metadata_topic_count}); pairing first ${camera_pair_count} entries."
  fi

  if [[ -n "${DIAG_RECORD_TOPICS}" ]]; then
    mapfile -t RUNTIME_RECORD_TOPICS < <(csv_to_topics "${DIAG_RECORD_TOPICS}" | dedupe_topics)
  else
    mapfile -t RUNTIME_RECORD_TOPICS < <(printf '%s\n' "${RUNTIME_SENSOR_TOPICS[@]}" | dedupe_topics)
  fi
  DIAG_RECORD_TOPICS="${RUNTIME_RECORD_TOPICS[*]}"

  mapfile -t RUNTIME_CHECK_TOPICS < <(
    printf '%s\n' \
      "${RUNTIME_SENSOR_TOPICS[@]}" \
      "${RUNTIME_CAMERA_IMAGE_TOPICS[@]}" \
      "${RUNTIME_CAMERA_METADATA_TOPICS[@]}" | dedupe_topics
  )

  if [[ -n "${DIAG_ALIGNMENT_LIDAR_TOPIC}" ]]; then
    RUNTIME_ALIGNMENT_LIDAR_TOPIC="${DIAG_ALIGNMENT_LIDAR_TOPIC}"
  elif [[ -n "${RUNTIME_OUSTER_POINTS_TOPIC}" ]]; then
    RUNTIME_ALIGNMENT_LIDAR_TOPIC="${RUNTIME_OUSTER_POINTS_TOPIC}"
  else
    RUNTIME_ALIGNMENT_LIDAR_TOPIC="/ouster/points"
  fi

  RUNTIME_HARDWARE_PROBE_SERVICE="$(pick_hardware_probe_service "${probe_service}")"
  RUNTIME_LIDAR_NIC="$(detect_lidar_nic "${RUNTIME_HARDWARE_PROBE_SERVICE}")"

  log "Diagnostics run id: ${RUN_ID}"
  log "Output folder: ${OUT_DIR}"
  log "Using compose command: ${COMPOSE_CMD[*]}"
  log "Sampling window (sec): ${DIAG_SAMPLE_SEC}"
  log "ros2 topic bw enabled: ${DIAG_ENABLE_BW}"
  log "multicast test enabled: ${DIAG_ENABLE_MULTICAST}"
  log "use ouster points for latency: ${DIAG_USE_OUSTER_POINTS}"
  log "recording QoS overrides file: ${DIAG_RECORDING_QOS_OVERRIDES}"
  log "check topics: ${RUNTIME_CHECK_TOPICS[*]}"
  log "recording topics: ${RUNTIME_RECORD_TOPICS[*]}"
  log "camera image topics: ${RUNTIME_CAMERA_IMAGE_TOPICS[*]}"
  log "camera metadata topics: ${RUNTIME_CAMERA_METADATA_TOPICS[*]}"
  log "camera pair topics: ${RUNTIME_CAMERA_PAIRS[*]}"
  log "alignment lidar topic: ${RUNTIME_ALIGNMENT_LIDAR_TOPIC}"
  log "hardware probe service: ${RUNTIME_HARDWARE_PROBE_SERVICE}"
  log "lidar nic: ${RUNTIME_LIDAR_NIC:-unresolved}"
  log "Running services: ${RUNNING_SERVICES[*]}"
  log "Probe service: ${probe_service}"
  if [[ -n "${RUNTIME_OUSTER_POINTS_TOPIC}" ]]; then
    log "Ouster point latency topic: ${RUNTIME_OUSTER_POINTS_TOPIC}"
  fi

  local service
  local topic
  local topic_file
  local -a probe_args

  log "Task 0/7 (preflight): tooling inventory (existing ROS2 diagnostics only)"
  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
echo "# ros2 doctor"
ros2 doctor --help >/dev/null && echo "available"
echo
echo "# diagnostic_common_diagnostics executables"
ros2 pkg executables diagnostic_common_diagnostics || true
echo
echo "# topic_monitor executables"
ros2 pkg executables topic_monitor || true
echo
echo "# diagnostic_aggregator executables"
ros2 pkg executables diagnostic_aggregator || true
echo
echo "# ros2 trace"
ros2 trace --help >/dev/null && echo "available" || echo "unavailable"
' > "${OUT_DIR}/task0_tool_inventory.log" 2>&1 || true

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
ros2 topic list || true
' > "${OUT_DIR}/task0_topic_list.log" 2>&1 || true

  {
    echo "# exact topic names used for checks"
    for topic in "${RUNTIME_CHECK_TOPICS[@]}"; do
      echo "check_topic=${topic}"
    done
    echo
    echo "# exact topic names used for recording"
    for topic in "${RUNTIME_RECORD_TOPICS[@]}"; do
      echo "record_topic=${topic}"
    done
    echo
    echo "# camera pair topics used for pairability checks"
    for topic in "${RUNTIME_CAMERA_PAIRS[@]}"; do
      echo "camera_pair=${topic}"
    done
    echo
    echo "# lidar-camera pairs used for alignment checks"
    for topic in "${RUNTIME_CAMERA_IMAGE_TOPICS[@]}"; do
      echo "alignment_pair=${RUNTIME_ALIGNMENT_LIDAR_TOPIC}:${topic}"
    done
  } > "${OUT_DIR}/task0_topics_used.log"

  {
    for topic in "${RUNTIME_CHECK_TOPICS[@]}"; do
      if grep -Fxq "${topic}" "${OUT_DIR}/task0_topic_list.log"; then
        echo "check_topic_present=${topic}"
      else
        echo "check_topic_missing=${topic}"
      fi
    done
    for topic in "${RUNTIME_RECORD_TOPICS[@]}"; do
      if grep -Fxq "${topic}" "${OUT_DIR}/task0_topic_list.log"; then
        echo "record_topic_present=${topic}"
      else
        echo "record_topic_missing=${topic}"
      fi
    done
  } > "${OUT_DIR}/task0_topic_presence.log"

  log "Task 1/7: container wall-clock and time namespace checks"
  for service in "${RUNNING_SERVICES[@]}"; do
    "${COMPOSE_CMD[@]}" exec -T "${service}" bash -lc "
set -e
echo \"service=${service}\"
echo \"utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
echo \"epoch_ns=\$(date +%s%N)\"
echo \"ns_time=\$(readlink /proc/self/ns/time 2>/dev/null || echo unavailable)\"
if [[ -r /proc/self/timens_offsets ]]; then
  echo \"timens_offsets:\"
  cat /proc/self/timens_offsets
else
  echo \"timens_offsets: unavailable\"
fi
" > "${OUT_DIR}/task1_clock_${service}.log" 2>&1 || true
  done

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
echo "probe_service='"${probe_service}"'"
if ros2 topic list | grep -Fxq /clock; then
  echo "clock_topic=present"
else
  echo "clock_topic=absent"
fi

echo "nodes:"
ros2 node list || true

echo "node_use_sim_time:"
for node in $(ros2 node list 2>/dev/null); do
  value=$(ros2 param get "$node" use_sim_time 2>/dev/null | awk -F": " "END {print \$2}")
  if [[ -z "$value" ]]; then
    value="unavailable"
  fi
  echo "$node $value"
done
' > "${OUT_DIR}/task1_ros_time_params.log" 2>&1 || true

  log "Task 2/7: driver stamp-source checks (static + live headers)"
  {
    echo "# Ouster timestamp mode from runtime config"
    rg -n "timestamp_mode" "${REPO_ROOT}/docker/docker_shared/ouster_config.yaml" || true
    echo
    echo "# Realsense global time settings from launch files"
    rg -n "global_time_enabled|diagnostics_period" \
      "${REPO_ROOT}/docker/docker_shared/scripts/rs_launch.py" \
      "${REPO_ROOT}/docker/docker_build/rslaunch/launch/rs_launch.py" || true
    echo
    echo "# Xsens header stamping logic in local mtnode.py"
    rg -n "self.h.stamp|SampleTimeFine|sync_timestamps|get_clock\\(\\)\\.now" \
      "${REPO_ROOT}/docker/docker_shared/scripts/mtnode.py" || true
    echo
    echo "# Emlid container launch command"
    rg -n "nmea_serial_driver|use_GNSS_time|port:=|baud:=" \
      "${REPO_ROOT}/docker/docker-compose.yml" \
      "${REPO_ROOT}/docker/docker_shared/scripts/emlid-launch.sh" || true
  } > "${OUT_DIR}/task2_stamp_source_static.log" 2>&1

  if service_running emlid; then
    "${COMPOSE_CMD[@]}" exec -T emlid bash -lc '
set -e
echo "node list:"
ros2 node list || true
echo
echo "nmea use_GNSS_time parameter:"
ros2 param get /nmea_serial_driver use_GNSS_time || true
' > "${OUT_DIR}/task2_emlid_runtime_params.log" 2>&1 || true
  fi

  for topic in "${RUNTIME_SENSOR_TOPICS[@]}"; do
    topic_file="$(sanitize_topic "${topic}")"
    if is_stamped_topic "${topic}"; then
      "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
echo \"topic=${topic}\"
echo \"probe_utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
if timeout ${DIAG_SAMPLE_SEC} ros2 topic echo --once --field header.stamp ${topic}; then
  exit 0
fi
echo \"fallback=ros2 topic echo --once ${topic} | first 40 lines\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic echo --once ${topic} | sed -n '1,40p'
" > "${OUT_DIR}/task2_header_${topic_file}.log" 2>&1 || true
    else
      "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
echo \"topic=${topic}\"
echo \"probe_utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
echo \"note=topic has no standard header.stamp field; capturing one sample payload\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic echo --once ${topic} | sed -n '1,40p'
" > "${OUT_DIR}/task2_header_${topic_file}.log" 2>&1 || true
    fi
  done

  log "Task 3/7: DDS/RMW/network consistency"
  for service in "${RUNNING_SERVICES[@]}"; do
    "${COMPOSE_CMD[@]}" exec -T "${service}" bash -lc '
set -e
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo
echo "IPv4 addresses:"
ip -o -4 addr show || true
echo
echo "Routes:"
ip route || true
' > "${OUT_DIR}/task3_env_${service}.log" 2>&1 || true
  done

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
ros2 doctor --report || true
' > "${OUT_DIR}/task3_ros2_doctor.log" 2>&1 || true

  if [[ "${DIAG_ENABLE_MULTICAST}" == "1" && "${#RUNNING_SERVICES[@]}" -ge 2 ]]; then
    local recv_service="${RUNNING_SERVICES[0]}"
    local send_service="${RUNNING_SERVICES[1]}"
    (
      "${COMPOSE_CMD[@]}" exec -T "${recv_service}" bash -lc 'timeout 8 ros2 multicast receive'
    ) > "${OUT_DIR}/task3_multicast_test.log" 2>&1 &
    local recv_pid=$!
    sleep 2
    "${COMPOSE_CMD[@]}" exec -T "${send_service}" bash -lc 'timeout 5 ros2 multicast send' >> "${OUT_DIR}/task3_multicast_test.log" 2>&1 || true
    wait "${recv_pid}" || true
  fi

  log "Task 4/7: QoS correctness and per-topic rate/delay sampling"
  capture_hardware_snapshot "before" "${RUNTIME_HARDWARE_PROBE_SERVICE}" "${RUNTIME_LIDAR_NIC}"

  local cpu_interval_sec=1
  local cpu_samples=1
  local cpu_sampler_pid=""
  if [[ "${DIAG_CPU_SAMPLE_INTERVAL_SEC}" =~ ^[0-9]+$ && "${DIAG_CPU_SAMPLE_INTERVAL_SEC}" -gt 0 ]]; then
    cpu_interval_sec="${DIAG_CPU_SAMPLE_INTERVAL_SEC}"
    cpu_samples="$((DIAG_SAMPLE_SEC / cpu_interval_sec + 1))"
    if [[ "${cpu_samples}" -lt 1 ]]; then
      cpu_samples=1
    fi
  fi

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
if command -v pidstat >/dev/null 2>&1; then
  echo \"cpu_sampler=pidstat\"
  timeout ${DIAG_SAMPLE_SEC} pidstat -ruhd ${cpu_interval_sec} || true
else
  echo \"cpu_sampler=top\"
  top -b -d ${cpu_interval_sec} -n ${cpu_samples} || true
fi
" > "${OUT_DIR}/task4_cpu_window.log" 2>&1 &
  cpu_sampler_pid=$!

  if [[ -f "${DIAG_WINDOW_PROBE_HOST}" ]]; then
    probe_args=(
      python3
      "${DIAG_WINDOW_PROBE_IN_CONTAINER}"
      --duration-sec "${DIAG_SAMPLE_SEC}"
    )
    for topic in "${RUNTIME_CAMERA_PAIRS[@]}"; do
      probe_args+=(--pair-topic "${topic}")
    done
    for topic in "${RUNTIME_CAMERA_IMAGE_TOPICS[@]}"; do
      probe_args+=(--align-topic "${RUNTIME_ALIGNMENT_LIDAR_TOPIC}:${topic}")
    done
    for topic in "${RUNTIME_CAMERA_IMAGE_TOPICS[@]}"; do
      probe_args+=(--count-topic "${topic}" --stamp-topic "${topic}")
    done
    for topic in "${RUNTIME_CAMERA_METADATA_TOPICS[@]}"; do
      probe_args+=(--count-topic "${topic}")
    done
    probe_args+=(--count-topic "${RUNTIME_ALIGNMENT_LIDAR_TOPIC}" --stamp-topic "${RUNTIME_ALIGNMENT_LIDAR_TOPIC}")

    "${COMPOSE_CMD[@]}" exec -T "${probe_service}" "${probe_args[@]}" > "${OUT_DIR}/task4_window_probe.json" 2> "${OUT_DIR}/task4_window_probe.stderr" || true
  else
    warn "Window probe script not found at ${DIAG_WINDOW_PROBE_HOST}; skipping pairability/alignment probe."
    echo "{\"status\":\"missing_probe_script\",\"path\":\"${DIAG_WINDOW_PROBE_HOST}\"}" > "${OUT_DIR}/task4_window_probe.json"
  fi

  if [[ -n "${cpu_sampler_pid}" ]]; then
    wait "${cpu_sampler_pid}" || true
  fi

  capture_hardware_snapshot "after" "${RUNTIME_HARDWARE_PROBE_SERVICE}" "${RUNTIME_LIDAR_NIC}"

  for topic in "${RUNTIME_SENSOR_TOPICS[@]}"; do
    topic_file="$(sanitize_topic "${topic}")"
    "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
ros2 topic info --verbose ${topic}
" > "${OUT_DIR}/task4_qos_${topic_file}.log" 2>&1 || true

    if is_stamped_topic "${topic}"; then
      "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
echo \"topic=${topic}\"
echo \"probe_utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
echo \"--- ros2 topic hz (${DIAG_SAMPLE_SEC}s window) ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic hz ${topic} || true
echo
if [[ \"${DIAG_ENABLE_BW}\" == \"1\" ]]; then
  echo \"--- ros2 topic bw (${DIAG_SAMPLE_SEC}s window) ---\"
  timeout ${DIAG_SAMPLE_SEC} ros2 topic bw ${topic} || true
  echo
else
  echo \"--- ros2 topic bw ---\"
  echo \"skipped (set DIAG_ENABLE_BW=1 to enable)\"
  echo
fi
echo \"--- ros2 topic delay (${DIAG_SAMPLE_SEC}s window) ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic delay ${topic} || true
" > "${OUT_DIR}/task4_rate_delay_${topic_file}.log" 2>&1 || true
    else
      "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
echo \"topic=${topic}\"
echo \"probe_utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
echo \"--- ros2 topic hz (${DIAG_SAMPLE_SEC}s window) ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic hz ${topic} || true
echo
if [[ \"${DIAG_ENABLE_BW}\" == \"1\" ]]; then
  echo \"--- ros2 topic bw (${DIAG_SAMPLE_SEC}s window) ---\"
  timeout ${DIAG_SAMPLE_SEC} ros2 topic bw ${topic} || true
  echo
else
  echo \"--- ros2 topic bw ---\"
  echo \"skipped (set DIAG_ENABLE_BW=1 to enable)\"
  echo
fi
echo \"--- ros2 topic delay ---\"
echo \"skipped: no standard header.stamp on this packet topic\"
" > "${OUT_DIR}/task4_rate_delay_${topic_file}.log" 2>&1 || true
    fi
  done

  for topic in "${RUNTIME_CHECK_TOPICS[@]}"; do
    topic_file="$(sanitize_topic "${topic}")"
    if [[ -f "${OUT_DIR}/task4_qos_${topic_file}.log" ]]; then
      continue
    fi
    "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
ros2 topic info --verbose ${topic}
" > "${OUT_DIR}/task4_qos_${topic_file}.log" 2>&1 || true
  done

  log "Task 5/7: QoS override audit"
  rg -n "qos_overrides|--qos-profile-overrides-path|_qos|SENSOR_DATA|best_effort|reliable|durability" \
    "${REPO_ROOT}/docker" > "${OUT_DIR}/task5_qos_static_scan.log" 2>&1 || true

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
for node in $(ros2 node list 2>/dev/null); do
  echo "## ${node}"
  ros2 param list "${node}" 2>/dev/null | grep "qos_overrides" || true
done
' > "${OUT_DIR}/task5_qos_runtime_params.log" 2>&1 || true

  log "Task 6/7: recorder compatibility snapshot"
  {
    echo "# Recorder QoS override template path:"
    echo "${DIAG_RECORDING_QOS_OVERRIDES}"
    echo
    if [[ -f "${DIAG_RECORDING_QOS_OVERRIDES}" ]]; then
      cat "${DIAG_RECORDING_QOS_OVERRIDES}"
    else
      echo "WARNING: missing file ${DIAG_RECORDING_QOS_OVERRIDES}"
    fi
    echo
    echo "# Canonical diagnostics recorder path (QoS overrides by default):"
    echo "container_script=${DIAG_RECORDING_RUNNER_IN_CONTAINER}"
    echo "host_script=${DIAG_RECORDING_RUNNER_HOST}"
    if [[ -f "${DIAG_RECORDING_RUNNER_HOST}" ]]; then
      echo "host_script_exists=yes"
    else
      echo "host_script_exists=no"
      echo "WARNING: missing file ${DIAG_RECORDING_RUNNER_HOST}"
    fi
    echo
    echo "# recording topics (exact)"
    for topic in "${RUNTIME_RECORD_TOPICS[@]}"; do
      echo "record_topic=${topic}"
    done
    echo
    echo "# Default diagnostics recording command:"
    echo "${COMPOSE_CMD[*]} run --rm -e TOPICS='${DIAG_RECORD_TOPICS}' recording ${DIAG_RECORDING_RUNNER_IN_CONTAINER}"
  } > "${OUT_DIR}/task6_recorder_notes.log"

  for topic in "${RUNTIME_RECORD_TOPICS[@]}"; do
    topic_file="$(sanitize_topic "${topic}")"
    "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
ros2 topic info --verbose ${topic}
" > "${OUT_DIR}/task6_recorder_compat_${topic_file}.log" 2>&1 || true
  done

  log "Task 7/7: Xsens timestamp/info regression quick validation"
  if service_running xsens; then
    {
      echo "xsens_validation=executed"
      echo "probe_service=${probe_service}"
    } > "${OUT_DIR}/task7_xsens_validation.log"

    "${COMPOSE_CMD[@]}" exec -T xsens bash -lc '
set -e
echo "node_list:"
ros2 node list || true
echo
echo "xsens_timeout_param:"
ros2 param get /xsens_driver timeout || true
echo
echo "xsens_read_chunk_size_param:"
ros2 param get /xsens_driver read_chunk_size || true
echo
echo "xsens_publish_imu_data_str_param:"
ros2 param get /xsens_driver publish_imu_data_str || true
' > "${OUT_DIR}/task7_xsens_params.log" 2>&1 || true

    "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
ros2 topic list || true
' > "${OUT_DIR}/task7_topic_list.log" 2>&1 || true

    missing_required=()
    present_optional=()
    missing_optional=()

    for topic in "${XSENS_REQUIRED_TOPICS[@]}"; do
      if grep -Fxq "${topic}" "${OUT_DIR}/task7_topic_list.log"; then
        printf 'xsens_required_topic_present=%s\n' "${topic}" >> "${OUT_DIR}/task7_xsens_validation.log"
      else
        missing_required+=("${topic}")
      fi
    done

    for topic in "${XSENS_OPTIONAL_TOPICS[@]}"; do
      if grep -Fxq "${topic}" "${OUT_DIR}/task7_topic_list.log"; then
        present_optional+=("${topic}")
      else
        missing_optional+=("${topic}")
      fi
    done

    if [[ "${#missing_required[@]}" -eq 0 ]]; then
      echo "xsens_required_topics_missing=none" >> "${OUT_DIR}/task7_xsens_validation.log"
    else
      echo "xsens_required_topics_missing=${missing_required[*]}" >> "${OUT_DIR}/task7_xsens_validation.log"
    fi

    if [[ "${#present_optional[@]}" -eq 0 ]]; then
      echo "xsens_optional_topics_present=none" >> "${OUT_DIR}/task7_xsens_validation.log"
    else
      echo "xsens_optional_topics_present=${present_optional[*]}" >> "${OUT_DIR}/task7_xsens_validation.log"
    fi

    if [[ "${#missing_optional[@]}" -eq 0 ]]; then
      echo "xsens_optional_topics_missing=none" >> "${OUT_DIR}/task7_xsens_validation.log"
    else
      echo "xsens_optional_topics_missing=${missing_optional[*]}" >> "${OUT_DIR}/task7_xsens_validation.log"
    fi

    for topic in "${XSENS_REQUIRED_TOPICS[@]}"; do
      topic_file="$(sanitize_topic "${topic}")"
      "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc "
set -e
echo \"topic=${topic}\"
echo \"probe_utc_iso=\$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)\"
echo \"--- ros2 topic hz (${DIAG_SAMPLE_SEC}s window) ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic hz ${topic} || true
echo
echo \"--- ros2 topic delay (${DIAG_SAMPLE_SEC}s window) ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic delay ${topic} || true
echo
echo \"--- header stamp sample ---\"
timeout ${DIAG_SAMPLE_SEC} ros2 topic echo --once --field header.stamp ${topic} || true
" > "${OUT_DIR}/task7_xsens_timing_${topic_file}.log" 2>&1 || true
    done
  else
    {
      echo "xsens_validation=skipped"
      echo "reason=xsens_service_not_running"
    } > "${OUT_DIR}/task7_xsens_validation.log"
  fi

  local clock_spread_ms="unknown"
  local ns_mismatch="unknown"
  {
    echo "# Diagnostic Summary"
    echo "run_id=${RUN_ID}"
    echo "compose_command=${COMPOSE_CMD[*]}"
    echo "compose_files=${DIAG_COMPOSE_FILES}"
    echo "sample_window_sec=${DIAG_SAMPLE_SEC}"
    echo "topic_bw_enabled=${DIAG_ENABLE_BW}"
    echo "multicast_enabled=${DIAG_ENABLE_MULTICAST}"
    echo "use_ouster_points_latency=${DIAG_USE_OUSTER_POINTS}"
    echo "ouster_points_topic=${RUNTIME_OUSTER_POINTS_TOPIC:-none}"
    echo "recording_qos_overrides_file=${DIAG_RECORDING_QOS_OVERRIDES}"
    echo "recording_runner_in_container=${DIAG_RECORDING_RUNNER_IN_CONTAINER}"
    echo "recording_runner_host=${DIAG_RECORDING_RUNNER_HOST}"
    echo "check_topics=${RUNTIME_CHECK_TOPICS[*]}"
    echo "recording_topics=${RUNTIME_RECORD_TOPICS[*]}"
    echo "camera_image_topics=${RUNTIME_CAMERA_IMAGE_TOPICS[*]}"
    echo "camera_metadata_topics=${RUNTIME_CAMERA_METADATA_TOPICS[*]}"
    echo "camera_pair_topics=${RUNTIME_CAMERA_PAIRS[*]}"
    echo "alignment_lidar_topic=${RUNTIME_ALIGNMENT_LIDAR_TOPIC}"
    echo "hardware_probe_service=${RUNTIME_HARDWARE_PROBE_SERVICE}"
    echo "lidar_nic=${RUNTIME_LIDAR_NIC:-unresolved}"
    echo "probe_service=${probe_service}"
    echo "running_services=${RUNNING_SERVICES[*]}"
    echo

    echo "## Task 0 summary"
    echo "check_topics_count=${#RUNTIME_CHECK_TOPICS[@]}"
    echo "recording_topics_count=${#RUNTIME_RECORD_TOPICS[@]}"
    if [[ -f "${OUT_DIR}/task0_topic_presence.log" ]]; then
      if grep -q '^check_topic_missing=' "${OUT_DIR}/task0_topic_presence.log"; then
        grep '^check_topic_missing=' "${OUT_DIR}/task0_topic_presence.log"
      else
        echo "check_topic_missing=none"
      fi
      if grep -q '^record_topic_missing=' "${OUT_DIR}/task0_topic_presence.log"; then
        grep '^record_topic_missing=' "${OUT_DIR}/task0_topic_presence.log"
      else
        echo "record_topic_missing=none"
      fi
    else
      echo "check_topic_missing=unavailable"
      echo "record_topic_missing=unavailable"
    fi
    echo

    echo "## Task 1 summary"
    {
      awk -F= '/^service=/{svc=$2} /^epoch_ns=/{print svc" "$2}' "${OUT_DIR}"/task1_clock_*.log 2>/dev/null
    } > "${OUT_DIR}/.clock_values.tmp" || true
    {
      awk -F= '/^service=/{svc=$2} /^ns_time=/{print svc" "$2}' "${OUT_DIR}"/task1_clock_*.log 2>/dev/null
    } > "${OUT_DIR}/.clock_ns.tmp" || true

    if [[ -s "${OUT_DIR}/.clock_values.tmp" ]]; then
      min_epoch="$(awk 'NR==1{m=$2} {if ($2<m) m=$2} END{print m}' "${OUT_DIR}/.clock_values.tmp")"
      max_epoch="$(awk 'NR==1{m=$2} {if ($2>m) m=$2} END{print m}' "${OUT_DIR}/.clock_values.tmp")"
      if [[ -n "${min_epoch}" && -n "${max_epoch}" ]]; then
        clock_spread_ms="$(( (max_epoch - min_epoch) / 1000000 ))"
      fi
      echo "clock_spread_ms=${clock_spread_ms}"
    else
      echo "clock_spread_ms=unavailable"
    fi

    if [[ -s "${OUT_DIR}/.clock_ns.tmp" ]]; then
      ns_unique_count="$(awk '{print $2}' "${OUT_DIR}/.clock_ns.tmp" | sort -u | wc -l | tr -d " ")"
      if [[ "${ns_unique_count}" -le 1 ]]; then
        ns_mismatch="no"
      else
        ns_mismatch="yes"
      fi
      echo "time_namespace_mismatch=${ns_mismatch}"
    else
      echo "time_namespace_mismatch=unavailable"
    fi

    if [[ -f "${OUT_DIR}/task1_ros_time_params.log" ]]; then
      grep -m1 "^clock_topic=" "${OUT_DIR}/task1_ros_time_params.log" || true
      if grep -E "node_use_sim_time:| unavailable" -q "${OUT_DIR}/task1_ros_time_params.log"; then
        true
      fi
      if grep -E " true$" "${OUT_DIR}/task1_ros_time_params.log" >/dev/null 2>&1; then
        echo "use_sim_time_true_detected=yes"
      else
        echo "use_sim_time_true_detected=no"
      fi
    fi
    echo

    echo "## Task 3 summary"
    if ls "${OUT_DIR}"/task3_env_*.log >/dev/null 2>&1; then
      domain_values="$(grep -h '^ROS_DOMAIN_ID=' "${OUT_DIR}"/task3_env_*.log | cut -d= -f2 | sort -u | tr '\n' ' ' | sed 's/[[:space:]]*$//')"
      rmw_values="$(grep -h '^RMW_IMPLEMENTATION=' "${OUT_DIR}"/task3_env_*.log | cut -d= -f2 | sort -u | tr '\n' ' ' | sed 's/[[:space:]]*$//')"
      echo "ros_domain_ids=${domain_values:-none}"
      echo "rmw_implementations=${rmw_values:-none}"
    fi
    echo
    echo "## Task 4 summary"
    echo "### QoS audit (required topics)"
    for topic in "${RUNTIME_CHECK_TOPICS[@]}"; do
      topic_file="$(sanitize_topic "${topic}")"
      echo "topic=${topic}"
      if [[ -f "${OUT_DIR}/task4_qos_${topic_file}.log" ]]; then
        grep -E "Topic type:|Publisher count:|Subscription count:|Reliability:|Durability:|History:|Depth:|Lifespan:|Deadline:|Liveliness:" \
          "${OUT_DIR}/task4_qos_${topic_file}.log" | sed 's/^/  /' || true
      else
        echo "  status=missing_qos_log"
      fi
    done

    echo
    echo "### Camera pairability + LiDAR-camera alignment"
    if [[ -f "${OUT_DIR}/task4_window_probe.json" ]]; then
      python3 - "${OUT_DIR}/task4_window_probe.json" <<'PY'
import json
import sys

path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
except Exception as exc:
    print(f"window_probe_status=error:{exc}")
    raise SystemExit(0)

print(f"window_probe_status={data.get('status', 'ok')}")
for topic in data.get("missing_topics", []):
    print(f"window_probe_missing_topic={topic}")

for pair, stats in data.get("pairability", {}).items():
    print(
        "pairability[{pair}] image_count={img} metadata_count={meta} mismatch_rate={rate}".format(
            pair=pair,
            img=stats.get("image_count", "NA"),
            meta=stats.get("metadata_count", "NA"),
            rate=stats.get("mismatch_rate", "NA"),
        )
    )

for pair, stats in data.get("alignment", {}).items():
    if stats.get("count", 0):
        print(
            "alignment[{pair}] count={count} p50_ms={p50} p95_ms={p95} p99_ms={p99} max_ms={maxv}".format(
                pair=pair,
                count=stats.get("count", "NA"),
                p50=stats.get("p50_ms", "NA"),
                p95=stats.get("p95_ms", "NA"),
                p99=stats.get("p99_ms", "NA"),
                maxv=stats.get("max_ms", "NA"),
            )
        )
    else:
        print(f"alignment[{pair}] status={stats.get('status', 'unavailable')}")
PY
    else
      echo "window_probe_status=missing_output"
    fi

    echo
    echo "### Hardware path deltas (before/after window)"
    if [[ -f "${OUT_DIR}/task4_hw_before.log" && -f "${OUT_DIR}/task4_hw_after.log" ]]; then
      for key in rx_errors tx_errors rx_dropped tx_dropped rx_packets tx_packets; do
        before_val="$(awk -F= -v k="nic_stat_${key}" '$1==k {print $2; exit}' "${OUT_DIR}/task4_hw_before.log" 2>/dev/null || true)"
        after_val="$(awk -F= -v k="nic_stat_${key}" '$1==k {print $2; exit}' "${OUT_DIR}/task4_hw_after.log" 2>/dev/null || true)"
        if [[ "${before_val}" =~ ^[0-9]+$ && "${after_val}" =~ ^[0-9]+$ ]]; then
          echo "nic_${key}_before=${before_val}"
          echo "nic_${key}_after=${after_val}"
          echo "nic_${key}_delta=$((after_val - before_val))"
        else
          echo "nic_${key}_before=${before_val:-unavailable}"
          echo "nic_${key}_after=${after_val:-unavailable}"
          echo "nic_${key}_delta=unavailable"
        fi
      done
    else
      echo "hardware_delta_status=missing_logs"
    fi

    echo
    echo "### CPU snapshot during probe window"
    if [[ -f "${OUT_DIR}/task4_cpu_window.log" ]]; then
      if grep -q '^cpu_sampler=pidstat' "${OUT_DIR}/task4_cpu_window.log"; then
        grep -E '^cpu_sampler=|^Average:|^[[:space:]]*[0-9]{2}:' "${OUT_DIR}/task4_cpu_window.log" | tail -n 40 || true
      else
        grep -E '^cpu_sampler=|%Cpu\\(s\\)|load average|Tasks:' "${OUT_DIR}/task4_cpu_window.log" || true
      fi
    else
      echo "cpu_window_status=missing_log"
    fi

    echo
    echo "## Task 7 summary (Xsens quick validation)"
    if [[ -f "${OUT_DIR}/task7_xsens_validation.log" ]]; then
      grep -E '^xsens_' "${OUT_DIR}/task7_xsens_validation.log" || true
    else
      echo "xsens_validation=unavailable"
    fi

    echo
    echo "## Recommended next checks"
    echo "1) Open task4_qos_*.log and task6_recorder_compat_*.log for request/offer mismatches."
    echo "2) If clock_topic=present or use_sim_time_true_detected=yes, fix sim-time settings first."
    echo "3) If time_namespace_mismatch=yes, inspect container runtime flags and /proc/*/ns/time."
    echo "4) If Ouster is TIME_FROM_ROS_TIME and you need absolute sync, move to TIME_FROM_PTP_1588."
    echo "5) Recorder default command: ${COMPOSE_CMD[*]} run --rm -e TOPICS='${DIAG_RECORD_TOPICS}' recording ${DIAG_RECORDING_RUNNER_IN_CONTAINER}"
    echo "6) Keep DIAG_ENABLE_BW=0 by default on Steam Deck; enable only when needed."
    echo "7) Review task7_xsens_params.log and task7_xsens_timing_*.log for timestamp/info regressions."
    echo "8) Correlate alignment jitter vs CPU load in task4_window_probe.json and task4_cpu_window.log."
  } > "${OUT_DIR}/summary.txt"

  rm -f "${OUT_DIR}/.clock_values.tmp" "${OUT_DIR}/.clock_ns.tmp"
  log "Complete. Review ${OUT_DIR}/summary.txt and task logs."
}

main "$@"
