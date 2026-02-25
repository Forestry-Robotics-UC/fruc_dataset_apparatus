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
- Linux checks: `date`, `/proc/self/ns/time`, `/proc/self/timens_offsets`, `ip`, `lsusb`, `ethtool`, `pidstat`/`top`
- Lightweight custom ROS 2 probe: `window_topic_probe.py` (uses `rclpy`/ROS message libs already in image)

## Build debug image

From `docker/`:

```bash
podman-compose -f docker-compose.debug.yml build
```

## Start debug containers

```bash
podman-compose -f docker-compose.debug.yml up -d debug-bridge debug-host
podman-compose -f docker-compose.debug.yml --profile observability up -d diagnostic-common topic-monitor diagnostic-aggregator
podman-compose -f docker-compose.debug.yml --profile profiling up -d ros2-tracing
```

Notes:
- `topic-monitor` is disabled by default for now (`TOPIC_MONITOR_AUTOSTART=0`).
- If you enable it, it uses a default topic (`/diagnostics_agg`) as positional argument.
- `ros2-tracing` starts in idle mode by default (`TRACE_AUTOSTART=0`) to avoid RT-stream impact.
- ROS graph consistency is enforced by default in diagnostics:
  - `ROS2_NETWORK_NAME=ros2_net`
  - `ROS_DOMAIN_ID=0`
  - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Disable if needed:
  - `TOPIC_MONITOR_AUTOSTART=0`
- Override monitored topic if needed:
  - `TOPIC_MONITOR_TOPIC=/imu/data`
- Backward compatibility:
  - `TOPIC_MONITOR_TOPICS` is still accepted; the runner uses the first topic from that list.
- Enable tracing capture when needed:
  - `TRACE_AUTOSTART=1`
- `diagnostic-common` remains opt-in:
  - `DIAG_COMMON_AUTOSTART=1`

## Run diagnostics suite

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

## FRUC Steam Deck preset

`run_diagnostics_suite.sh` is now preset for this repository/Steam Deck setup:
- `DIAG_COMPOSE_FILES="docker-compose.yml,docker-compose.debug.yml"`
- `DIAG_COMPOSE_IMPL=podman`
- `DIAG_SAMPLE_SEC=10`
- `DIAG_ENABLE_BW=0`
- `DIAG_ENABLE_MULTICAST=0`
- `DIAG_EXPECTED_ROS_NETWORK=ros2_net`
- `DIAG_EXPECTED_ROS_DOMAIN_ID=0`
- `DIAG_EXPECTED_RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `DIAG_ENFORCE_GRAPH_CONSISTENCY=1`
- `DIAG_ENFORCE_USE_SIM_TIME_FALSE=1`
- `DIAG_USE_OUSTER_POINTS=1`
- `DIAG_OUSTER_POINTS_TOPIC=/ouster/points`
- `DIAG_RECORDING_QOS_OVERRIDES=docker/docker_shared/scripts/diagnostics/recording_qos_overrides.yaml`
- `DIAG_RECORDING_RUNNER_IN_CONTAINER=/shared/scripts/diagnostics/record_with_qos_overrides.sh`
- `DIAG_CAMERA_IMAGE_TOPICS=/camera/color/image_raw,/camera/aligned_depth_to_color/image_raw`
- `DIAG_CAMERA_METADATA_TOPICS=/camera/color/metadata,/camera/depth/metadata`
- `DIAG_WINDOW_PROBE_IN_CONTAINER=/shared/scripts/diagnostics/window_topic_probe.py`
- `run_diagnostics_suite.sh` ensures `ros2-tracing` is started and will start `debug-bridge` if no other target service is running

You can run diagnostics directly without passing env vars:

```bash
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

Optional overrides are still supported for troubleshooting:

```bash
DIAG_SAMPLE_SEC=10 DIAG_ENABLE_BW=1 \
DIAG_RECORD_TOPICS="/ouster/lidar_packets,/ouster/imu_packets,/ouster/points,/camera/color/image_raw,/camera/color/metadata,/imu/data,/diagnostics,/diagnostics_agg" \
bash docker/docker_shared/scripts/diagnostics/run_diagnostics_suite.sh
```

## Task mapping

0. Preflight inventory + topic presence validation (exact check/record topic names logged).
1. Container wall clock + time namespace checks (`/clock`, `use_sim_time`, namespace ids).
2. Driver stamp-source checks (static config scan + live `header.stamp` samples).
3. DDS/RMW/network consistency (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, routes, `ros2 doctor`).
4. QoS and timing checks (`ros2 topic info/hz/delay`; bandwidth optional) plus:
   - camera image↔metadata pairability mismatch rate
   - LiDAR↔camera nearest-neighbor stamp deltas (`p50/p95/p99/max`)
   - hardware path snapshots (`lsusb -t`, `ip -s link`, `ethtool`, NIC error counters before/after)
   - CPU snapshot over the same sample window (`pidstat` or `top`)
5. QoS override audit (static grep + runtime parameter scan).
6. Recorder compatibility snapshot (`recording_qos_overrides.yaml` + endpoint QoS snapshots).
7. Xsens quick regression checks (topics present, key params, `/imu/data|/imu/mag` rate/delay/stamp sample).

## New output files

Key additions in each run folder:
- `task0_service_detection.log`
- `task0_ros_graph_consistency.log`
- `task0_topics_used.log`
- `task0_topic_presence.log`
- `task4_window_probe.json`
- `task4_window_probe.stderr`
- `task4_hw_before.log`
- `task4_hw_after.log`
- `task4_cpu_window.log`

`summary.txt` now includes:
- required-topic QoS audit excerpts (`ros2 topic info --verbose`)
- camera pairability mismatch rates
- LiDAR-camera alignment delta stats (`p50/p95/p99/max`)
- NIC RX/TX error counter deltas and CPU sampler snippets
- `window_probe_status` is always structured JSON (`ok`, `no_data`, or `error`)

## Xsens regression quick checks (auto)

The suite now writes:
- `task7_xsens_validation.log`
- `task7_xsens_params.log`
- `task7_xsens_timing_imu_data.log`
- `task7_xsens_timing_imu_mag.log`

These are summarized automatically in `summary.txt` under "Task 7 summary".

## Recorder compatibility template

Default template path:

```bash
docker/docker_shared/scripts/diagnostics/recording_qos_overrides.yaml
```

Canonical diagnostics command (QoS overrides applied by default):

```bash
podman-compose run --rm recording /shared/scripts/diagnostics/record_with_qos_overrides.sh
```

Optional overrides:

```bash
podman-compose run --rm \
  -e RECORD_DURATION_SEC=60 \
  -e OUTPUT_NAME=diag_qos_probe \
  recording /shared/scripts/diagnostics/record_with_qos_overrides.sh
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

Use this to build and profile baseline vs patched branch.
The script is preset for Steam Deck as:

- before: `https://github.com/norlab-ulaval/norlab_xsens_driver.git#jazzy`
- after: `https://github.com/duda1202/norlab_xsens_driver.git#perf/buffered-serial-read`
- compose impl: `podman`
- image rebuild mode: `XSENS_BENCH_NO_CACHE=1`

Run:

```bash
bash docker/docker_shared/scripts/diagnostics/benchmark_xsens_before_after.sh
```

Branch variable aliases remain supported: `BEFORE_BRANCH`/`AFTER_BRANCH` and `BEFORE_REPO_BRANCH`/`AFTER_REPO_BRANCH`.

Useful overrides:

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
