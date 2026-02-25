#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
DOCKER_DIR="${REPO_ROOT}/docker"
OUTPUT_ROOT="${DOCKER_DIR}/diagnostics_runs"

RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
OUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "${OUT_DIR}"

DIAG_SAMPLE_SEC="${DIAG_SAMPLE_SEC:-6}"
DIAG_ENABLE_BW="${DIAG_ENABLE_BW:-0}"
DIAG_ENABLE_MULTICAST="${DIAG_ENABLE_MULTICAST:-0}"
DIAG_COMPOSE_FILES="${DIAG_COMPOSE_FILES:-docker-compose.yml}"
DIAG_USE_OUSTER_POINTS="${DIAG_USE_OUSTER_POINTS:-1}"
DIAG_OUSTER_POINTS_TOPIC="${DIAG_OUSTER_POINTS_TOPIC:-}"

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
  /fix
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

log() {
  printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"
}

warn() {
  printf '[%s] WARNING: %s\n' "$(date -u +%H:%M:%S)" "$*" >&2
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

  if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD=(docker compose "${COMPOSE_FILE_ARGS[@]}")
    return
  fi

  if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE_CMD=(podman-compose "${COMPOSE_FILE_ARGS[@]}")
    return
  fi

  echo "Could not find docker compose or podman-compose." >&2
  exit 1
}

service_running() {
  local service="$1"
  local cid
  cid="$("${COMPOSE_CMD[@]}" ps -q "${service}" 2>/dev/null || true)"
  [[ -n "${cid}" ]]
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

  log "Diagnostics run id: ${RUN_ID}"
  log "Output folder: ${OUT_DIR}"
  log "Using compose command: ${COMPOSE_CMD[*]}"
  log "Sampling window (sec): ${DIAG_SAMPLE_SEC}"
  log "ros2 topic bw enabled: ${DIAG_ENABLE_BW}"
  log "multicast test enabled: ${DIAG_ENABLE_MULTICAST}"
  log "use ouster points for latency: ${DIAG_USE_OUSTER_POINTS}"
  log "Running services: ${RUNNING_SERVICES[*]}"
  log "Probe service: ${probe_service}"
  if [[ -n "${RUNTIME_OUSTER_POINTS_TOPIC}" ]]; then
    log "Ouster point latency topic: ${RUNTIME_OUSTER_POINTS_TOPIC}"
  fi

  local service
  local topic
  local topic_file

  log "Task 0: tooling inventory (existing ROS2 diagnostics only)"
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

  log "Task 1/6: container wall-clock and time namespace checks"
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

  log "Task 2/6: driver stamp-source checks (static + live headers)"
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

  log "Task 3/6: DDS/RMW/network consistency"
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

  log "Task 4/6: QoS correctness and per-topic rate/delay sampling"
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

  log "Task 5/6: QoS override audit"
  rg -n "qos_overrides|--qos-profile-overrides-path|_qos|SENSOR_DATA|best_effort|reliable|durability" \
    "${REPO_ROOT}/docker" > "${OUT_DIR}/task5_qos_static_scan.log" 2>&1 || true

  "${COMPOSE_CMD[@]}" exec -T "${probe_service}" bash -lc '
set -e
for node in $(ros2 node list 2>/dev/null); do
  echo "## ${node}"
  ros2 param list "${node}" 2>/dev/null | grep "qos_overrides" || true
done
' > "${OUT_DIR}/task5_qos_runtime_params.log" 2>&1 || true

  log "Task 6/6: recorder compatibility snapshot"
  {
    echo "# Recorder QoS override template:"
    cat "${REPO_ROOT}/docker/docker_shared/scripts/diagnostics/recording_qos_overrides.yaml"
  } > "${OUT_DIR}/task6_recorder_notes.log"

  for topic in "${RUNTIME_SENSOR_TOPICS[@]}"; do
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
    echo "probe_service=${probe_service}"
    echo "running_services=${RUNNING_SERVICES[*]}"
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
    echo "5) For rosbag2 tests, use scripts/diagnostics/recording_qos_overrides.yaml."
    echo "6) Keep DIAG_ENABLE_BW=0 by default on Steam Deck; enable only when needed."
    echo "7) Review task7_xsens_params.log and task7_xsens_timing_*.log for timestamp/info regressions."
  } > "${OUT_DIR}/summary.txt"

  rm -f "${OUT_DIR}/.clock_values.tmp" "${OUT_DIR}/.clock_ns.tmp"
  log "Complete. Review ${OUT_DIR}/summary.txt and task logs."
}

main "$@"
