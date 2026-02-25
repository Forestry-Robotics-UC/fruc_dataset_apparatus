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
AFTER_REPO_URL="${AFTER_REPO_URL:-${BEFORE_REPO_URL}}"
# Support both AFTER_BRANCH and AFTER_REPO_BRANCH naming.
AFTER_BRANCH="${AFTER_BRANCH:-${AFTER_REPO_BRANCH:-perf/buffered-serial-read}}"

XSENS_BENCH_SAMPLE_SEC="${XSENS_BENCH_SAMPLE_SEC:-8}"
XSENS_BENCH_WARMUP_SEC="${XSENS_BENCH_WARMUP_SEC:-6}"
XSENS_BENCH_NO_CACHE="${XSENS_BENCH_NO_CACHE:-0}"

COMPOSE_CMD=()

log() {
  printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"
}

detect_compose() {
  if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD=(docker compose -f docker-compose.yml)
    return
  fi

  if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE_CMD=(podman-compose -f docker-compose.yml)
    return
  fi

  echo "Could not find docker compose or podman-compose." >&2
  exit 1
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

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc '
set -e
echo "repo_url='"${repo_url}"'"
echo "branch='"${branch}"'"
python3 -c "import xsens_driver.mtdevice as d; print(\"mtdevice_file=\" + d.__file__)"
python3 -c "import xsens_driver.mtdevice as d, inspect; print(inspect.getsource(d.MTDevice.read_msg)[:1200])"
python3 -c "import xsens_driver.mtdevice as d, inspect; print(inspect.getsource(d.MTDevice._read_from_serial)[:500])"
' > "${case_dir}/code_path_and_snippet.log" 2>&1 || true

  local pid
  pid="$(compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "pgrep -f 'mtnode.py' | head -n1" | tr -d '\r' || true)"
  echo "pid=${pid}" > "${case_dir}/pid.log"
  if [[ -z "${pid}" ]]; then
    log "Case '${case_name}': could not find mtnode.py PID"
    compose_env "${repo_url}" "${branch}" stop xsens || true
    return
  fi

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
top -b -n 1 -H -p ${pid}
" > "${case_dir}/top_threads.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} strace -c -p ${pid} -f
" > "${case_dir}/strace_c.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout 4 strace -tt -T -p ${pid} -f -e trace=read,pselect6
" > "${case_dir}/strace_read_pselect.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc '
set -e
ros2 topic list || true
' > "${case_dir}/topic_list.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic hz /imu/data
" > "${case_dir}/hz_imu_data.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic hz /imu/mag
" > "${case_dir}/hz_imu_mag.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic hz /fix
" > "${case_dir}/hz_fix.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc "
set -e
timeout ${XSENS_BENCH_SAMPLE_SEC} ros2 topic delay /imu/data
" > "${case_dir}/delay_imu_data.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" exec -T xsens bash -lc '
set -e
echo "imu_data_stamp:"
timeout 6 ros2 topic echo --once --field header.stamp /imu/data || true
echo
echo "imu_mag_stamp:"
timeout 6 ros2 topic echo --once --field header.stamp /imu/mag || true
echo
echo "fix_stamp:"
timeout 6 ros2 topic echo --once --field header.stamp /fix || true
' > "${case_dir}/header_stamps.log" 2>&1 || true

  compose_env "${repo_url}" "${branch}" stop xsens || true
}

write_summary() {
  {
    echo "# Xsens Before/After Benchmark Summary"
    echo "run_id=${RUN_ID}"
    echo "before=${BEFORE_REPO_URL}#${BEFORE_BRANCH}"
    echo "after=${AFTER_REPO_URL}#${AFTER_BRANCH}"
    echo "sample_sec=${XSENS_BENCH_SAMPLE_SEC}"
    echo
    for case_name in before after; do
      local case_dir="${OUT_DIR}/${case_name}"
      echo "## ${case_name}"
      if [[ ! -d "${case_dir}" ]]; then
        echo "status=missing_case_dir"
        echo
        continue
      fi
      cat "${case_dir}/pid.log" 2>/dev/null || true
      echo "-- top mtnode line --"
      grep -E "mtnode.py" "${case_dir}/top_threads.log" | head -n 1 || true
      echo "-- strace key syscalls --"
      grep -E " pselect6$| read$| recvfrom$| futex$| total$" "${case_dir}/strace_c.log" || true
      echo "-- /imu/data hz summary --"
      grep -E "average rate|std dev|min:|max:" "${case_dir}/hz_imu_data.log" || true
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
