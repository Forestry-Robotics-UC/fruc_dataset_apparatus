# Steam Deck ROS 2 Diagnostics (Debug-Only, Low Overhead)

This diagnostics bundle only uses existing ROS 2/Linux tools and only touches debug assets.
No core runtime launch/config files are required for this workflow.

## Included tools

- `ros2 doctor --report`
- `ros2 topic info --verbose`
- `ros2 topic hz`
- `ros2 topic delay`
- `ros2 topic bw` (disabled by default)
- `ros2 topic echo --once --field header.stamp`
- `ros2 multicast send/receive` (disabled by default)
- `diagnostic_common_diagnostics`
- `topic_monitor`
- `diagnostic_aggregator`
- `ros2 trace` (`ros2_tracing`)
- Linux checks: `date`, `/proc/self/ns/time`, `/proc/self/timens_offsets`, `ip`

## Build debug image

From `docker/`:

```bash
docker compose -f docker-compose.debug.yml build
```

## Start debug containers (idle by default)

```bash
docker compose -f docker-compose.debug.yml up -d debug-bridge debug-host
docker compose -f docker-compose.debug.yml --profile observability up -d diagnostic-common topic-monitor diagnostic-aggregator
docker compose -f docker-compose.debug.yml --profile profiling up -d ros2-tracing
```

Notes:
- `diagnostic-common` and `topic-monitor` are intentionally idle by default.
- Enable when needed:
  - `DIAG_COMMON_AUTOSTART=1`
  - `TOPIC_MONITOR_AUTOSTART=1`

## Run the six-task diagnostics suite

From repo root:

```bash
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

Outputs:

```bash
docker/diagnostics_runs/<UTC_TIMESTAMP>/
```

Open summary first:

```bash
cat docker/diagnostics_runs/<UTC_TIMESTAMP>/summary.txt
```

## Low-overhead defaults (recommended on Steam Deck)

Defaults in `run_diagnostics_suite.sh`:
- `DIAG_SAMPLE_SEC=6`
- `DIAG_ENABLE_BW=0`
- `DIAG_ENABLE_MULTICAST=0`
- `DIAG_USE_OUSTER_POINTS=1` (auto-detects `/ouster/points*` and uses it for stamp-based delay)

Example with slightly longer sampling:

```bash
DIAG_SAMPLE_SEC=10 bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

Enable heavier probes only when needed:

```bash
DIAG_ENABLE_BW=1 DIAG_ENABLE_MULTICAST=1 \
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

Force a specific Ouster point topic if auto-detect is not correct:

```bash
DIAG_OUSTER_POINTS_TOPIC=/ouster/points \
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

## Multi-compose usage

If your services are spread across files:

```bash
DIAG_COMPOSE_FILES="docker-compose.yml,docker-compose.debug.yml" \
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

## Task mapping

1. Container wall clock + time namespace checks (`/clock`, `use_sim_time`, namespace ids).
2. Driver stamp-source checks (static config scan + live `header.stamp` samples).
3. DDS/RMW/network consistency (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, routes, `ros2 doctor`).
4. QoS and timing checks (`ros2 topic info/hz/delay`; bandwidth optional).
5. QoS override audit (static grep + runtime parameter scan).
6. Recorder compatibility snapshot (`recording_qos_overrides.yaml` + endpoint QoS snapshots).
7. Xsens quick regression checks (topics present, key params, `/imu/data|/imu/mag|/fix` rate/delay/stamp sample).

## Xsens regression quick checks (auto)

The suite now writes:
- `task7_xsens_validation.log`
- `task7_xsens_params.log`
- `task7_xsens_timing_imu_data.log`
- `task7_xsens_timing_imu_mag.log`
- `task7_xsens_timing_fix.log`

These are summarized automatically in `summary.txt` under "Task 7 summary".

## Recorder compatibility template

Template path:

```bash
docker/docker_shared/scripts/diagnostics/recording_qos_overrides.yaml
```

Example:

```bash
docker compose run --rm recording \
  ros2 bag record \
  --qos-profile-overrides-path /shared/scripts/diagnostics/recording_qos_overrides.yaml \
  /ouster/lidar_packets /ouster/imu_packets /ouster/points \
  /camera/color/image_raw /camera/aligned_depth_to_color/image_raw \
  /imu/data /fix /diagnostics /diagnostics_agg
```

## Debug override configs (no core file edits)

- RealSense: `docker/docker_shared/scripts/diagnostics/realsense_diagnostics.yaml`
- Ouster (packets + IMU + points): `docker/docker_shared/scripts/diagnostics/ouster_debug_override.yaml`

## Xsens CPU mitigation

The runtime Xsens launcher keeps the original timeout default:
- `XSENS_TIMEOUT=0.002` (original behavior)
- `XSENS_READ_CHUNK_SIZE=512` when using patched driver branches
- `XSENS_PUBLISH_IMU_DATA_STR=true` by default (keeps original topic behavior)
- `XSENS_USE_SHARED_OVERLAY=1` by default (for benchmark branch tests set `0`)

Override if needed:

```bash
XSENS_TIMEOUT=0.02 XSENS_READ_CHUNK_SIZE=1024 ./docker/docker_shared/scripts/xsens-launch.sh
```

## Xsens before/after branch benchmark

Use this to build and profile baseline (`jazzy`) vs your patched branch:

```bash
bash docker/docker_shared/scripts/diagnostics/benchmark_xsens_before_after.sh
```

Branch variable aliases are supported: `BEFORE_BRANCH`/`AFTER_BRANCH` and `BEFORE_REPO_BRANCH`/`AFTER_REPO_BRANCH`.

Useful overrides:

```bash
BEFORE_REPO_URL=https://github.com/norlab-ulaval/norlab_xsens_driver.git \
BEFORE_BRANCH=jazzy \
AFTER_REPO_URL=git@github.com:<you>/norlab_xsens_driver.git \
AFTER_BRANCH=perf/buffered-serial-read \
XSENS_BENCH_SAMPLE_SEC=10 \
XSENS_BENCH_NO_CACHE=1 \
bash docker/docker_shared/scripts/diagnostics/benchmark_xsens_before_after.sh
```

SteamOS pull-and-run example (duda1202 fork):

```bash
BEFORE_REPO_URL=https://github.com/norlab-ulaval/norlab_xsens_driver.git \
BEFORE_BRANCH=jazzy \
AFTER_REPO_URL=https://github.com/duda1202/norlab_xsens_driver.git \
AFTER_BRANCH=perf/buffered-serial-read \
XSENS_BENCH_SAMPLE_SEC=10 \
XSENS_BENCH_NO_CACHE=1 \
bash docker/docker_shared/scripts/diagnostics/benchmark_xsens_before_after.sh
```

Artifacts are written to:

```bash
docker/diagnostics_runs/xsens_benchmark_<UTC_TIMESTAMP>/
```

## ros2_tracing repository

The debug image includes:
- `ros-jazzy-ros2trace`
- source repo at `/opt/ros2_tracing`

Quick checks:

```bash
ros2 trace --help
ls -la /opt/ros2_tracing
```
