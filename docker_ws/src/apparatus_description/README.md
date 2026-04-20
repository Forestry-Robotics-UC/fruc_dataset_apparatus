# CURT Mini Robot Description Package

ROS 2 Jazzy package for launching the CURT Mini robot URDF description with Ouster OS1 LiDAR, RealSense D435 camera, and MAPIR Survey3 camera.

## Package Contents

- **URDF Files**: Robot description in Xacro format
- **Launch Files**: Python-based launch files for different use cases
- **Configuration Files**: ROS 2 control and Gazebo configurations
- **Meshes**: STL and OBJ mesh files for visualization
- **RViz Config**: Pre-configured RViz visualization settings

## Launch Files

### 1. `robot_state_publisher.launch.py`
Launches the robot state publisher with joint state publisher GUI.

```bash
ros2 launch apparatus_description robot_state_publisher.launch.py
```

**Arguments:**
- `use_sim_time` (default: false) - Use Gazebo simulation time
- `simulation` (default: false) - Enable simulation mode
- `gui` (default: true) - Show joint state publisher GUI

### 2. `view_robot.launch.py`
Launches robot state publisher with RViz2 visualization.

```bash
ros2 launch apparatus_description view_robot.launch.py
```

**Arguments:**
- `use_sim_time` (default: false)
- `simulation` (default: false)
- `gui` (default: true)

### 3. `robot_description.launch.py`
Basic launch file for just the robot state publisher.

```bash
ros2 launch apparatus_description robot_description.launch.py
```

## Installation

1. Navigate to your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
```

2. The package is already in your workspace at:
```
/home/tobias/Development/ROS2CALIB/bags/CURT_URDF
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select apparatus_description
source install/setup.bash
```

## Usage Examples

### View the robot in RViz
```bash
ros2 launch apparatus_description view_robot.launch.py gui:=true
```

### Control joint states with GUI
```bash
ros2 launch apparatus_description robot_state_publisher.launch.py gui:=true
```

### Use with Gazebo simulation
```bash
ros2 launch apparatus_description robot_state_publisher.launch.py simulation:=true use_sim_time:=true
```

## Robot Structure

The robot description includes:
- **Base Link**: Root coordinate frame (no inertia)
- **Chassis**: Main body link with inertial properties and mesh
- **Wheels**: Four continuous-type wheel joints (front/back, left/right)
- **Sensors**:
  - Ouster OS1 LiDAR (os_sensor → os_lidar, os_imu)
  - RealSense D435 Camera (camera_link → camera_color_optical_frame)
  - MAPIR Survey3 Camera (mapir_link)
- **Visual Elements**: Tower meshes for sensors

## Dependencies

The package depends on:
- `launch` and `launch_ros`
- `robot_state_publisher`
- `joint_state_publisher` and `joint_state_publisher_gui`
- `xacro`
- `ros2_control` and `gazebo_ros2_control`
- `rviz2`

## Customization

### Modify Robot Inertia
Edit `urdf/robot.urdf.xacro` to adjust inertial properties.

### Update Sensor Transforms
Modify the joint origins in `robot.urdf.xacro` to update calibrated camera transforms.

### Adjust RViz Display
Edit `config/rviz.rviz` to customize visualization settings.

## Troubleshooting

### Meshes not loading
Ensure mesh files are properly installed:
```bash
colcon build --packages-select apparatus_description --symlink-install
```

### Xacro variables not resolved
Check that all referenced properties in `properties.xacro` are defined correctly.

### TF tree not updating
Verify joint_state_publisher is running and publishing on `/joint_states` topic.

## Package Info

- **Robot Name**: curt_mini
- **Maintainer**: [Update in package.xml]
- **License**: [Update in package.xml]
- **Version**: 0.0.1

## Notes

- The package uses Python-based build system (`ament_python`)
- Xacro files are processed at runtime for flexibility
- Simulation mode includes Gazebo plugin support
- ROS 2 Jazzy compatible
