#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
DOCKER_DIR="${REPO_ROOT}/docker"
OUTPUT_ROOT="${DOCKER_DIR}/diagnostics_runs"
RUN_ID="${RUN_ID:-xsens_benchmark_$(date -u +%Y%m%dT%H%M%SZ)}"
OUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "${OUT_DIR}"

BEFORE_REPO_URL="${BEFORE_REPO_URL:-https://github.com/norlab-ulaval/norlab_xsens_driver.git}"
# Support both BEFORE_BRANCH and BEFORE_REPO_BRANCH naming.
BEFORE_BRANCH="${BEFORE_BRANCH:-${BEFORE_REPO_BRANCH:-jazzy}}"
# FRUC Steam Deck preset: compare upstream jazzy vs duda1202 perf/buffered-serial-read.
AFTER_REPO_URL="${AFTER_REPO_URL:-https://github.com/duda1202/norlab_xsens_driver.git}"
# Support both AFTER_BRANCH and AFTER_REPO_BRANCH naming.
AFTER_BRANCH="${AFTER_BRANCH:-${AFTER_REPO_BRANCH:-perf/buffered-serial-read}}"

XSENS_BENCH_SAMPLE_SEC="${XSENS_BENCH_SAMPLE_SEC:-8}"
XSENS_BENCH_WARMUP_SEC="${XSENS_BENCH_WARMUP_SEC:-6}"
# Keep branch checkouts fresh when benchmarking branch behavior.
XSENS_BENCH_NO_CACHE="${XSENS_BENCH_NO_CACHE:-1}"
XSENS_BENCH_CPU_SAMPLE_SEC="${XSENS_BENCH_CPU_SAMPLE_SEC:-12}"
XSENS_BENCH_ENABLE_PYSPY="${XSENS_BENCH_ENABLE_PYSPY:-0}"
XSENS_BENCH_PYSPY_DURATION="${XSENS_BENCH_PYSPY_DURATION:-20}"
XSENS_BENCH_COMPOSE_IMPL="${XSENS_BENCH_COMPOSE_IMPL:-podman}"
XSENS_BENCH_START_RETRIES="${XSENS_BENCH_START_RETRIES:-1}"
XSENS_BENCH_START_RETRY_SLEEP_SEC="${XSENS_BENCH_START_RETRY_SLEEP_SEC:-2}"
XSENS_BENCH_PID_WAIT_SEC="${XSENS_BENCH_PID_WAIT_SEC:-12}"
XSENS_BENCH_PID_POLL_SEC="${XSENS_BENCH_PID_POLL_SEC:-1}"
XSENS_BENCH_ENABLE_STAMP_DELTA="${XSENS_BENCH_ENABLE_STAMP_DELTA:-1}"
XSENS_BENCH_STAMP_DELTA_SEC="${XSENS_BENCH_STAMP_DELTA_SEC:-10}"
XSENS_BENCH_STAMP_DELTA_SCRIPT="${XSENS_BENCH_STAMP_DELTA_SCRIPT:-/shared/scripts/diagnostics/stamp_delta_probe.py}"
XSENS_BENCH_ENABLE_PIDSTAT="${XSENS_BENCH_ENABLE_PIDSTAT:-1}"
XSENS_BENCH_CPUSET="${XSENS_BENCH_CPUSET:-}"
XSENS_BENCH_NICE="${XSENS_BENCH_NICE:-0}"
XSENS_BENCH_ENABLE_STRACE_VERBOSE="${XSENS_BENCH_ENABLE_STRACE_VERBOSE:-1}"

COMPOSE_CMD=()

log() {
  printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"
}

detect_compose() {
  case "${XSENS_BENCH_COMPOSE_IMPL}" in
    docker)
      if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
        COMPOSE_CMD=(docker compose -f docker-compose.yml)
        return
      fi
      echo "XSENS_BENCH_COMPOSE_IMPL=docker but docker compose is unavailable." >&2
      exit 1
      ;;
    podman)
      if command -v podman-compose >/dev/null 2>&1; then
        COMPOSE_CMD=(podman-compose -f docker-compose.yml)
        return
      fi
      echo "XSENS_BENCH_COMPOSE_IMPL=podman but podman-compose is unavailable." >&2
      exit 1
      ;;
    auto)
      if command -v podman-compose >/dev/null 2>&1; then
        COMPOSE_CMD=(podman-compose -f docker-compose.yml)
        return
      fi
      if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
        COMPOSE_CMD=(docker compose -f docker-compose.yml)
        return
      fi
      echo "Could not find docker compose or podman-compose." >&2
      exit 1
      ;;
    *)
      echo "Invalid XSENS_BENCH_COMPOSE_IMPL='${XSENS_BENCH_COMPOSE_IMPL}'. Use auto, docker, or podman." >&2
      exit 1
      ;;
  esac
}

compose_env() {
  local repo_url="$1"
  local branch="$2"
  shift 2
  env \
    XSENS_REPO_URL="${repo_url}" \
    XSENS_REPO_BRANCH="${branch}" \
    XSENS_USE_SHARED_OVERLAY=0 \
    "${COMPOSE_CMD[@]}" "$@"
}

compose_service_ps() {
  local repo_url="$1"
  local branch="$2"
  local service="$3"

  if [[ "${COMPOSE_CMD[0]}" == "podman-compose" ]]; then
    compose_env "${repo_url}" "${branch}" ps
    return
  fi
  compose_env "${repo_url}" "${branch}" ps "${service}"
}

xsens_container_running() {
  local running_state=""

  if [[ "${COMPOSE_CMD[0]}" == "podman-compose" ]]; then
    if ! command -v podman >/dev/null 2>&1; then
      return 1
    fi
    running_state="$(podman inspect -f '{{.State.Running}}' ros2-apparatus-xsens 2>/dev/null || true)"
  else
    if ! command -v docker >/dev/null 2>&1; then
      return 1
    fi
    running_state="$(docker inspect -f '{{.State.Running}}' ros2-apparatus-xsens 2>/dev/null || true)"
  fi

  [[ "${running_state}" == "true" ]]
}

ros_setup_prelude() {
  cat <<'EOF'
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f /docker_ws/install/setup.bash ]]; then
  source /docker_ws/install/setup.bash
elif [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi
set -u
EOF
}

compose_env_exec_bash() {
  local repo_url="$1"
  local branch="$2"
  local service="$3"
  local command_body="$4"
  local script
  script="$(printf '%s\n%s\n' "$(ros_setup_prelude)" "${command_body}")"
  compose_env "${repo_url}" "${branch}" exec -T "${service}" bash -lc "${script}"
}

classify_xsens_failure_reason() {
  local log_file="$1"
  local fallback_reason="$2"

  if [[ ! -s "${log_file}" ]]; then
    echo "${fallback_reason}"
    return
  fi

  if grep -Eq 'MTException: could not find message\.' "${log_file}"; then
    echo "xsens_mt_exception_could_not_find_message"
    return
  fi
  if grep -Eq 'MTException: could not find MTData message\.' "${log_file}"; then
    echo "xsens_mt_exception_could_not_find_mtdata_message"
    return
  fi
  if grep -Eq "AttributeError: type object 'MTDevice' has no attribute '_read_from_serial'" "${log_file}"; then
    echo "xsens_attribute_error_missing__read_from_serial"
    return
  fi
  if grep -Eq 'SerialException|No such file or directory|Permission denied' "${log_file}"; then
    echo "xsens_serial_open_or_access_failure"
    return
  fi
  if grep -Eq 'process has died' "${log_file}"; then
    echo "xsens_process_died"
    return
  fi

  echo "${fallback_reason}"
}

extract_xsens_failure_detail_line() {
  local log_file="$1"

  if [[ ! -s "${log_file}" ]]; then
    return 0
  fi

  awk '
    /MTException:|AttributeError:|SerialException|process has died|No such file or directory|Permission denied/ {
      line = $0
    }
    END {
      print line
    }
  ' "${log_file}" 2>/dev/null || true
}

capture_case_failure_context() {
  local case_dir="$1"
  local repo_url="$2"
  local branch="$3"
  local reason="$4"
  local classified_reason=""
  local detail_line=""
  local log_file="${case_dir}/xsens_container.log"

  compose_service_ps "${repo_url}" "${branch}" xsens > "${case_dir}/xsens_service_ps.log" 2>&1 || true
  compose_env "${repo_url}" "${branch}" logs --no-color xsens > "${log_file}" 2>&1 || true
  compose_env_exec_bash "${repo_url}" "${branch}" xsens '
set -e
echo "# ps -eo pid,ppid,args"
ps -eo pid,ppid,args || true
echo
echo "# ros2 node list"
ros2 node list || true
echo
echo "# ros2 topic list"
ros2 topic list || true
' > "${case_dir}/xsens_runtime_snapshot.log" 2>&1 || true

  classified_reason="$(classify_xsens_failure_reason "${log_file}" "${reason}")"
  detail_line="$(extract_xsens_failure_detail_line "${log_file}")"

  {
    echo "reason=${classified_reason}"
    echo "reason_stage=${reason}"
    echo "repo_url=${repo_url}"
    echo "branch=${branch}"
    echo "utc_iso=$(date -u +%Y-%m-%dT%H:%M:%S.%N%z)"
    if [[ -n "${detail_line}" ]]; then
      echo "reason_detail=${detail_line}"
    fi
  } > "${case_dir}/failure_reason.log"
}

ensure_xsens_running_or_capture() {
  local case_name="$1"
  local repo_url="$2"
  local branch="$3"
  local case_dir="$4"
  local stage="$5"
  local attempt=0

  while true; do
    if xsens_container_running; then
      return 0
    fi

    if [[ "${attempt}" -lt "${XSENS_BENCH_START_RETRIES}" ]]; then
      attempt=$((attempt + 1))
      log "Case '${case_name}': xsens not running at ${stage}; retry up -d (${attempt}/${XSENS_BENCH_START_RETRIES})"
      compose_env "${repo_url}" "${branch}" up -d xsens >/dev/null 2>&1 || true
      sleep "${XSENS_BENCH_START_RETRY_SLEEP_SEC}"
      continue
    fi

    log "Case '${case_name}': xsens container is not running at ${stage}"
    capture_case_failure_context "${case_dir}" "${repo_url}" "${branch}" "xsens_not_running_${stage}"
    compose_env "${repo_url}" "${branch}" stop xsens || true
    return 1
  done
}

collect_xsens_pids() {
  local repo_url="$1"
  local branch="$2"

  compose_env_exec_bash "${repo_url}" "${branch}" xsens '
set -e
# Capture common entrypoints and fallback to process table matching.
pgrep -f "[m]tnode.py" || true
pgrep -f "[x]sens_driver_node" || true
ps -eo pid,args | awk "$0 ~ /(mtnode.py|xsens_driver_node)/ && $0 !~ /(pgrep -f|ps -eo pid,args|awk )/ {print \$1}" || true
' | tr -d '\r' | sed '/^$/d' | awk '!seen[$0]++'
}

wait_for_xsens_pids() {
  local repo_url="$1"
  local branch="$2"
  local waited=0
  local -a pids=()

  while true; do
    mapfile -t pids < <(collect_xsens_pids "${repo_url}" "${branch}" || true)
    if [[ "${#pids[@]}" -gt 0 ]]; then
      printf '%s\n' "${pids[@]}"
      return 0
    fi

    if [[ "${waited}" -ge "${XSENS_BENCH_PID_WAIT_SEC}" ]]; then
      return 1
    fi

    sleep "${XSENS_BENCH_PID_POLL_SEC}"
    waited=$((waited + XSENS_BENCH_PID_POLL_SEC))
  done
}

abort_if_xsens_not_running() {
  local case_name="$1"
  local repo_url="$2"
  local branch="$3"
  local case_dir="$4"
  local stage="$5"

  if xsens_container_running; then
    return 0
  fi

  log "Case '${case_name}': xsens stopped during ${stage}; aborting case"
  capture_case_failure_context "${case_dir}" "${repo_url}" "${branch}" "xsens_not_running_${stage}"
  compose_env "${repo_url}" "${branch}" stop xsens || true
  return 1
}

run_case_xsens_exec() {
  local case_name="$1"
  local repo_url="$2"
  local branch="$3"
  local case_dir="$4"
  local stage="$5"
  local out_file="$6"
  local command_body="$7"

  compose_env_exec_bash "${repo_url}" "${branch}" xsens "${command_body}" > "${out_file}" 2>&1 || true
  abort_if_xsens_not_running "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "${stage}"
}

run_case() {
  local case_name="$1"
  local repo_url="$2"
  local branch="$3"
  local case_dir="${OUT_DIR}/${case_name}"
  mkdir -p "${case_dir}"

  log "Case '${case_name}': repo=${repo_url} branch=${branch}"

  if [[ "${XSENS_BENCH_NO_CACHE}" == "1" ]]; then
    compose_env "${repo_url}" "${branch}" build --no-cache xsens
  else
    compose_env "${repo_url}" "${branch}" build xsens
  fi

  compose_env "${repo_url}" "${branch}" up -d xsens
  sleep "${XSENS_BENCH_WARMUP_SEC}"
  compose_service_ps "${repo_url}" "${branch}" xsens > "${case_dir}/xsens_service_ps.log" 2>&1 || true
  if ! ensure_xsens_running_or_capture "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "post_warmup"; then
    return
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "code_probe" "${case_dir}/code_path_and_snippet.log" '
set -e
echo "repo_url='"${repo_url}"'"
echo "branch='"${branch}"'"
python3 -c "import xsens_driver.mtdevice as d; print(\"mtdevice_file=\" + d.__file__)"
python3 -c "import xsens_driver.mtdevice as d, inspect; print(inspect.getsource(d.MTDevice.read_msg)[:1200])"
python3 -c "import xsens_driver.mtdevice as d, inspect; fn=getattr(d.MTDevice, \"_read_from_serial\", None); print(\"has__read_from_serial=\" + str(fn is not None)); print(inspect.getsource(fn)[:500] if fn else \"missing_method=_read_from_serial\"); print(\"available_read_methods=\" + \",\".join(sorted([n for n in dir(d.MTDevice) if \"read\" in n.lower()])))"
'; then
    return
  fi

  local -a pids
  mapfile -t pids < <(wait_for_xsens_pids "${repo_url}" "${branch}" || true)
  {
    echo "pid_patterns=mtnode.py xsens_driver_node"
    echo "pid_count=${#pids[@]}"
    if [[ "${#pids[@]}" -gt 0 ]]; then
      echo "pids=${pids[*]}"
      echo "primary_pid=${pids[0]}"
    fi
  } > "${case_dir}/pid.log"
  if [[ "${#pids[@]}" -eq 0 ]]; then
    log "Case '${case_name}': could not find xsens driver PID(s)"
    if xsens_container_running; then
      capture_case_failure_context "${case_dir}" "${repo_url}" "${branch}" "driver_pid_not_found"
    else
      capture_case_failure_context "${case_dir}" "${repo_url}" "${branch}" "xsens_not_running_during_pid_wait"
    fi
    compose_env "${repo_url}" "${branch}" stop xsens || true
    return
  fi

  local pid_csv
  local primary_pid="${pids[0]}"
  local strace_pid_args=""
  local pid
  pid_csv="$(IFS=,; echo "${pids[*]}")"
  for pid in "${pids[@]}"; do
    strace_pid_args+=" -p ${pid}"
  done
  if [[ -n "${XSENS_BENCH_CPUSET}" || "${XSENS_BENCH_NICE}" != "0" ]]; then
    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "pinning" "${case_dir}/pinning.log" "
set -e
echo \"Applying cpuset=${XSENS_BENCH_CPUSET:-unset} nice=${XSENS_BENCH_NICE}\"
for p in ${pid_csv//,/ }; do
  if [[ -n \"${XSENS_BENCH_CPUSET}\" ]]; then
    if command -v taskset >/dev/null 2>&1; then
      taskset -cp ${XSENS_BENCH_CPUSET} \$p || true
    else
      echo \"taskset_not_installed\"
    fi
  fi
  if [[ \"${XSENS_BENCH_NICE}\" != \"0\" ]]; then
    renice ${XSENS_BENCH_NICE} -p \$p || true
  fi
done
if command -v taskset >/dev/null 2>&1; then
  taskset -cp ${primary_pid} || true
fi
ps -p ${pid_csv} -o pid,ni,psr,pcpu,comm || true
"; then
      return
    fi
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "top_threads" "${case_dir}/top_threads.log" "
set -e
top -b -n 1 -H -p ${pid_csv}
"; then
    return
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "ps_cpu_samples" "${case_dir}/ps_cpu_samples.log" "
set -e
for _i in \$(seq 1 ${XSENS_BENCH_CPU_SAMPLE_SEC}); do
  date -u +%Y-%m-%dT%H:%M:%S.%3NZ
  ps -p ${pid_csv} -o pid=,pcpu=,pmem=,etimes=,comm=
  total_cpu=\$(ps -p ${pid_csv} -o pcpu= | awk '{s += \$1} END {printf \"%.2f\", s + 0}')
  echo \"sample_total_cpu=\${total_cpu}\"
  sleep 1
done
"; then
    return
  fi

  if [[ "${XSENS_BENCH_ENABLE_PIDSTAT}" == "1" ]]; then
    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "pidstat" "${case_dir}/pidstat.log" "
set -e
if command -v pidstat >/dev/null 2>&1; then
  timeout ${XSENS_BENCH_SAMPLE_SEC} pidstat -t -p ${pid_csv} 1
else
  echo \"pidstat_not_installed\"
fi
"; then
      return
    fi
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "strace_c" "${case_dir}/strace_c.log" "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} strace -c -f ${strace_pid_args}
"; then
    return
  fi

  if [[ "${XSENS_BENCH_ENABLE_PYSPY}" == "1" ]]; then
    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "pyspy_top" "${case_dir}/pyspy_top.log" "
set -e
if command -v py-spy >/dev/null 2>&1; then
  timeout ${XSENS_BENCH_PYSPY_DURATION} py-spy top --pid ${primary_pid} --rate 50
else
  echo \"py-spy not installed in container\"
fi
"; then
      return
    fi
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "topic_list" "${case_dir}/topic_list.log" '
set -e
ros2 topic list || true
'; then
    return
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "hz_imu_data" "${case_dir}/hz_imu_data.log" "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic hz /imu/data --window 2000
"; then
    return
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "hz_imu_mag" "${case_dir}/hz_imu_mag.log" "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic hz /imu/mag --window 2000
"; then
    return
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "delay_imu_data" "${case_dir}/delay_imu_data.log" "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic delay /imu/data
"; then
    return
  fi

  if [[ "${XSENS_BENCH_ENABLE_STAMP_DELTA}" == "1" ]]; then
    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "stamp_delta_imu_data" "${case_dir}/stamp_delta_imu_data.log" "
set -e
if [[ -f \"${XSENS_BENCH_STAMP_DELTA_SCRIPT}\" ]]; then
  python3 \"${XSENS_BENCH_STAMP_DELTA_SCRIPT}\" \
    --topic /imu/data \
    --msg-type sensor_msgs/msg/Imu \
    --duration-sec ${XSENS_BENCH_STAMP_DELTA_SEC} \
    --qos sensor_data
else
  echo \"stamp_delta_probe_missing=${XSENS_BENCH_STAMP_DELTA_SCRIPT}\"
fi
"; then
      return
    fi

    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "stamp_delta_imu_mag" "${case_dir}/stamp_delta_imu_mag.log" "
set -e
if [[ -f \"${XSENS_BENCH_STAMP_DELTA_SCRIPT}\" ]]; then
  python3 \"${XSENS_BENCH_STAMP_DELTA_SCRIPT}\" \
    --topic /imu/mag \
    --msg-type sensor_msgs/msg/MagneticField \
    --duration-sec ${XSENS_BENCH_STAMP_DELTA_SEC} \
    --qos sensor_data
else
  echo \"stamp_delta_probe_missing=${XSENS_BENCH_STAMP_DELTA_SCRIPT}\"
fi
"; then
      return
    fi
  fi

  if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "header_stamps" "${case_dir}/header_stamps.log" '
set -e
echo "imu_data_stamp:"
timeout 6 ros2 topic echo --once --field header.stamp /imu/data || true
echo
echo "imu_mag_stamp:"
timeout 6 ros2 topic echo --once --field header.stamp /imu/mag || true
'; then
    return
  fi

  if [[ "${XSENS_BENCH_ENABLE_STRACE_VERBOSE}" == "1" ]]; then
    if ! run_case_xsens_exec "${case_name}" "${repo_url}" "${branch}" "${case_dir}" "strace_read_pselect" "${case_dir}/strace_read_pselect.log" "
set -e
timeout 4 strace -tt -T -f -e trace=read,pselect6 ${strace_pid_args}
"; then
      return
    fi
  fi

  compose_env "${repo_url}" "${branch}" stop xsens || true
}

write_summary() {
  {
    echo "# Xsens Before/After Benchmark Summary"
    echo "run_id=${RUN_ID}"
    echo "before=${BEFORE_REPO_URL}#${BEFORE_BRANCH}"
    echo "after=${AFTER_REPO_URL}#${AFTER_BRANCH}"
    echo "sample_sec=${XSENS_BENCH_SAMPLE_SEC}"
    echo "no_cache=${XSENS_BENCH_NO_CACHE}"
    echo "cpu_sample_sec=${XSENS_BENCH_CPU_SAMPLE_SEC}"
    echo "pyspy_enabled=${XSENS_BENCH_ENABLE_PYSPY}"
    echo "stamp_delta_enabled=${XSENS_BENCH_ENABLE_STAMP_DELTA}"
    echo "stamp_delta_sec=${XSENS_BENCH_STAMP_DELTA_SEC}"
    echo "stamp_delta_script=${XSENS_BENCH_STAMP_DELTA_SCRIPT}"
    echo "pidstat_enabled=${XSENS_BENCH_ENABLE_PIDSTAT}"
    echo "cpuset=${XSENS_BENCH_CPUSET:-unset}"
    echo "nice=${XSENS_BENCH_NICE}"
    echo "strace_verbose_enabled=${XSENS_BENCH_ENABLE_STRACE_VERBOSE}"
    echo
    for case_name in before after; do
      local case_dir="${OUT_DIR}/${case_name}"
      local cpu_stats
      local pselect_calls
      local read_calls
      local recvfrom_calls
      local futex_calls
      echo "## ${case_name}"
      if [[ ! -d "${case_dir}" ]]; then
        echo "status=missing_case_dir"
        echo
        continue
      fi
      cat "${case_dir}/pid.log" 2>/dev/null || true
      cat "${case_dir}/failure_reason.log" 2>/dev/null || true
      cpu_stats="$(awk -F= '
        /^sample_total_cpu=/ {
          v = $2 + 0;
          sum += v;
          if (v > max) max = v;
          n += 1;
        }
        END {
          if (n > 0) {
            printf "samples=%d avg_cpu=%.2f max_cpu=%.2f mode=total_mtnode_cpu\n", n, sum / n, max;
          } else {
            print "samples=0 avg_cpu=NA max_cpu=NA mode=none";
          }
        }
      ' "${case_dir}/ps_cpu_samples.log" 2>/dev/null || true)"
      if [[ "${cpu_stats}" == *"mode=none" ]]; then
        cpu_stats="$(awk '
          NF >= 5 && $1 ~ /^[0-9]+$/ {
            sum += $2;
            if ($2 > max) max = $2;
            n += 1;
          }
          END {
            if (n > 0) {
              printf "samples=%d avg_cpu=%.2f max_cpu=%.2f mode=per_pid_rows\n", n, sum / n, max;
            } else {
              print "samples=0 avg_cpu=NA max_cpu=NA mode=none";
            }
          }
        ' "${case_dir}/ps_cpu_samples.log" 2>/dev/null || true)"
      fi
      echo "-- cpu samples --"
      echo "${cpu_stats}"
      echo "-- top mtnode line --"
      grep -E "mtnode.py|python3" "${case_dir}/top_threads.log" | head -n 1 || true
      echo "-- strace key syscalls --"
      grep -E " pselect6$| read$| recvfrom$| futex$| total$" "${case_dir}/strace_c.log" || true
      pselect_calls="$(awk '$NF=="pselect6" {print $4; exit}' "${case_dir}/strace_c.log" 2>/dev/null || true)"
      read_calls="$(awk '$NF=="read" {print $4; exit}' "${case_dir}/strace_c.log" 2>/dev/null || true)"
      recvfrom_calls="$(awk '$NF=="recvfrom" {print $4; exit}' "${case_dir}/strace_c.log" 2>/dev/null || true)"
      futex_calls="$(awk '$NF=="futex" {print $4; exit}' "${case_dir}/strace_c.log" 2>/dev/null || true)"
      echo "calls: pselect6=${pselect_calls:-NA} read=${read_calls:-NA} recvfrom=${recvfrom_calls:-NA} futex=${futex_calls:-NA}"
      echo "-- /imu/data hz summary --"
      grep -E "average rate|std dev|min:|max:" "${case_dir}/hz_imu_data.log" || true
      echo "-- /imu/data delay summary --"
      grep -E "average delay|min:|max:|std dev:" "${case_dir}/delay_imu_data.log" || true
      if [[ "${XSENS_BENCH_ENABLE_PIDSTAT}" == "1" ]]; then
        echo "-- pidstat summary --"
        grep -E "Linux|UID|Average:|^[0-9]{2}:[0-9]{2}:[0-9]{2}" "${case_dir}/pidstat.log" | head -n 30 || true
      fi
      if [[ "${XSENS_BENCH_ENABLE_STAMP_DELTA}" == "1" ]]; then
        echo "-- /imu/data stamp delta summary --"
        grep -E "^status=|^messages_seen=|^deltas_count=|^delta_ms_|^stamp_delta_probe_missing=" "${case_dir}/stamp_delta_imu_data.log" || true
        echo "-- /imu/mag stamp delta summary --"
        grep -E "^status=|^messages_seen=|^deltas_count=|^delta_ms_|^stamp_delta_probe_missing=" "${case_dir}/stamp_delta_imu_mag.log" || true
      fi
      if [[ -f "${case_dir}/pyspy_top.log" ]]; then
        echo "-- py-spy --"
        sed -n '1,25p' "${case_dir}/pyspy_top.log" || true
      fi
      echo
    done
    echo "Artifacts folder: ${OUT_DIR}"
  } > "${OUT_DIR}/summary.txt"
}

main() {
  detect_compose
  cd "${DOCKER_DIR}"
  log "Output folder: ${OUT_DIR}"
  run_case "before" "${BEFORE_REPO_URL}" "${BEFORE_BRANCH}"
  run_case "after" "${AFTER_REPO_URL}" "${AFTER_BRANCH}"
  write_summary
  log "Benchmark complete. See ${OUT_DIR}/summary.txt"
}

main "$@"
