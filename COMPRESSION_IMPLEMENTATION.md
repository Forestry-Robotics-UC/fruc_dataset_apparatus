# Realsense Image Transport Compression Implementation

## Overview

This implementation adds image transport compression to the realsense camera RGB topic, enabling more efficient data recording and transmission. Two compression types are available:

- **JPEG (Lossy)**: Smaller file sizes, faster processing, minor quality loss
- **PNG (Lossless)**: Larger files than JPEG, preserves full quality, no loss

## What Was Changed

### 1. Docker Configuration
- **File**: `docker/Dockerfile.realsense`
- **Change**: Added `ros-jazzy-image-transport-plugins` package to enable compression support

### 2. Launch Files
- **New File**: `docker/docker_build/rslaunch/launch/image_transport_relay.py`
  - Creates three relay nodes for the color image:
    1. JPEG compressed relay (`/camera/color/image_raw/compressed`)
    2. Jetpeg relay as alternative lossy (`/camera/color/image_raw/jetpeg`)
    3. PNG lossless relay (`/camera/color/image_raw/png`)

- **Modified File**: `docker/docker_build/rslaunch/launch/rs_launch.py`
  - Now includes the image transport relay launch file
  - Automatically starts compression nodes alongside the realsense driver

### 3. Recording Scripts
- **Modified Files**:
  - `launch-system.sh` (KDialog version)
  - `gnome-launch-system.sh` (Zenity version)
  - Added new recording options:
    - `/camera/color/image_raw/compressed` - JPEG compressed (ON by default)
    - `/camera/color/image_raw/png` - PNG lossless (OFF by default)

### 4. Test Tools
- **File**: `docker/test_image_compression.sh`
  - Bash script for running compression tests
  - Compares raw, JPEG compressed, and PNG lossless recordings
  - Configurable duration and quality settings

- **File**: `docker/compression_analysis.py`
  - Python tool for analyzing test results
  - Generates statistics on file sizes, data rates, and compression ratios
  - JSON output for further processing

## Available Topics

After the realsense launch, the following image topics are published:

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | Raw RGB image (no compression) |
| `/camera/color/image_raw/compressed` | sensor_msgs/msg/CompressedImage | JPEG compressed RGB |
| `/camera/color/image_raw/jetpeg` | Alternative lossy compression variant |
| `/camera/color/image_raw/png` | sensor_msgs/msg/CompressedImage | PNG lossless RGB |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera calibration info |

## Usage

### Recording with Compression

1. **Start the recording system**:
   ```bash
   cd /path/to/fruc_dataset_apparatus
   ./launch-system.sh  # or gnome-launch-system.sh for Gnome desktop
   ```

2. **In the topic selection dialog**, you'll see:
   - `Realsense Color` - Raw uncompressed image
   - `Realsense Color (JPEG Compressed)` - JPEG lossy (recommended for most uses)
   - `Realsense Color (PNG Lossless)` - PNG lossless (for maximum quality)

3. **Select desired topics** and proceed with recording
   - Compressed topics are included in the rosbag alongside raw topics
   - You can choose to record any combination

### Running Compression Tests

#### Bash Test Script
```bash
cd docker
./test_image_compression.sh [duration_seconds] [jpeg_quality]

# Examples:
./test_image_compression.sh              # Default: 30s, quality 95
./test_image_compression.sh 60 90        # 60 seconds, quality 90
```

The script will:
1. Launch containers
2. Record with each compression type
3. Generate size comparison reports
4. Save results to `/tmp/fruc_compression_test_[timestamp]/`

#### Python Analysis Tool
```bash
cd docker
python3 compression_analysis.py /path/to/test/directory [--output results.json]

# Example:
python3 compression_analysis.py /tmp/fruc_compression_test_2024-04-16_10-30-45 --output results.json
```

Output includes:
- File sizes and compression ratios
- Data rates (Mbps)
- Message counts and topics
- Efficiency summary

## Compression Type Comparison

### JPEG (Lossy)
- **Pros**:
  - ⭐ Smallest file size (50-80% reduction vs raw)
  - ⭐ Lowest bandwidth requirement
  - ⭐ Fastest to encode/decode
  - ⭐ Good for storage-constrained scenarios
- **Cons**:
  - Minor quality loss (usually imperceptible at quality > 90)
  - Recompression artifacts with multiple encode/decode cycles
- **Best For**: Large-scale dataset collection, bandwidth-limited networks, real-time streaming
- **Not For**: Fine-grained visual analysis, medical imaging

### PNG (Lossless)
- **Pros**:
  - ⭐ Perfect quality preservation
  - ⭐ Good compression (30-60% reduction vs raw)
  - ⭐ No quality loss through encode/decode cycles
  - ⭐ Better for post-processing
- **Cons**:
  - Larger files than JPEG
  - Slower encoding/decoding than JPEG
  - Higher memory requirements
- **Best For**: Detailed analysis, archival, situations where lossless is mandatory
- **Less suitable**: Real-time streaming, bandwidth-limited scenes

### Raw (Uncompressed)
- **Pros**:
  - Maximum quality preservation
  - No encoding overhead
  - Fastest to write (no compression)
- **Cons**:
  - ❌ Largest file size
  - ❌ Highest bandwidth/storage needs
  - ❌ Not practical for long recordings
- **Best For**: Development/testing, short recordings, comparison baseline

## Recommended Settings

### For Standard Dataset Collection
```
Include: /camera/color/image_raw/compressed
Include: /camera/color/camera_info
Skip: Other image variants (saves space)
```
**Reason**: JPEG compression provides ~70% size reduction with imperceptible quality loss

### For High-Quality Analysis
```
Include: /camera/color/image_raw/png
Include: /camera/color/camera_info
Optional: /camera/color/image_raw (for comparison)
```
**Reason**: PNG preserves full quality while reducing size by 30-60%

### For Research/Benchmarking
```
Include: /camera/color/image_raw (raw)
Include: /camera/color/image_raw/compressed (JPEG)
Include: /camera/color/image_raw/png (PNG)
Include: /camera/color/camera_info
```
**Reason**: Record all variants for offline analysis and comparison

## Performance Metrics (Expected)

Based on typical realsense RGB (640x480 @ 30 FPS):

| Format | Per-Frame Size | Data Rate | Storage (1 hour) |
|--------|----------------|-----------|------------------|
| Raw    | ~920 KB        | ~221 Mbps | ~99 GB          |
| JPEG   | ~90-180 KB     | ~21-43 Mbps | ~9-19 GB       |
| PNG    | ~200-400 KB    | ~48-96 Mbps | ~21-43 GB      |

*Actual values depend on image content and JPEG quality setting*

## Implementation Details

### Image Transport Relay
The relay nodes use ROS2's `image_transport` package to automatically:
1. Subscribe to raw image publishers
2. Apply compression from pluginlib-based image transport plugins
3. Publish compressed versions on separate topics
4. Allow rosbag to record both versions simultaneously

### Quality Parameters

To adjust JPEG quality, modify `image_transport_relay.py`:
```python
# Default quality is set in the relay configuration
# Typical range: 85-100 (higher = better quality, larger files)
```

For future scaling, you can add a launch parameter:
```python
# In image_transport_relay.py, add:
quality_arg = DeclareLaunchArgument(
    'jpeg_quality',
    default_value='95',
    description='JPEG compression quality (1-100)'
)
```

## Troubleshooting

### Compressed topics not appearing
1. Check if relay is running: `ros2 topic list | grep compressed`
2. Verify launch file is being included: Check logs for `image_transport_relay.py`
3. Ensure image-transport-plugins are installed: `apt list --installed | grep image-transport`

### Recording doesn't include compressed topics
1. Select them in the topic selection dialog
2. Check rosbag: `ros2 bag info /path/to/bag | grep compressed`
3. Verify topics are being published: `ros2 topic hz /camera/color/image_raw/compressed`

### Large file sizes despite compression
1. Verify you're recording compressed topics, not raw
2. Check JPEG quality setting (lower = smaller but lower quality)
3. Monitor bandwidth: `ros2 topic bw /camera/color/image_raw/compressed`

### Compression nodes not starting
```bash
# Check if realsense driver started successfully
docker logs ros2-apparatus-realsense

# Check if image transport plugins are available
ros2 pkg list | grep image_transport
```

## Future Enhancements

Potential improvements:
1. Add H.264/H.265 video codec compression for even better efficiency
2. Add quality parameter configuration via launch arguments
3. Add runtime switching between compression types
4. Implement adaptive compression based on bandwidth availability
5. Add image quality metrics (SSIM, PSNR) to test analysis
6. Create visualization tool for comparing images across compression methods

## References

- [ROS2 Image Transport](https://github.com/ros-perception/image_common)
- [Image Transport Plugins](https://github.com/ros-perception/image_transport_plugins)
- [Realsense ROS2 Driver](https://github.com/IntelRealSense/realsense-ros)
- [Sensor Msgs](http://docs.ros.org/en/melodic/api/sensor_msgs/html/)

## Testing Results

Run the compression test to see how it performs on your system:
```bash
cd docker
./test_image_compression.sh 60     # 1-minute test
python3 compression_analysis.py /tmp/fruc_compression_test_* --output results.json
```

Then review `results.json` for detailed metrics specific to your hardware and scene conditions.
