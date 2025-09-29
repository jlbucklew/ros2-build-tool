# Usage Guide

## Quick Start

### Interactive Wizard (Recommended)

The easiest way to get started:

```bash
./ros2_build_tool.py --wizard
```

Answer the prompts to configure your robot:
- Robot name
- ROS2 distribution (humble/jazzy)
- Use case (mapping, navigation, perception, full stack)
- Hardware components (lidar, camera, IMU, etc.)
- SLAM algorithm
- Optional features (Nav2, Foxglove, lifecycle management)

### From Existing Profile

If you have a saved configuration:

```bash
./ros2_build_tool.py --profile my_robot_profile.yaml
```

### With URDF File

Automatically extract robot specifications from URDF:

```bash
./ros2_build_tool.py --wizard --urdf path/to/robot.urdf
```

Supports both `.urdf` and `.xacro` files!

## Command-Line Options

```bash
./ros2_build_tool.py --help
```

### Options:

- `--wizard`, `-w`: Run interactive configuration wizard
- `--profile PATH`, `-p`: Load configuration from YAML profile
- `--urdf PATH`, `-u`: Path to robot URDF file (.urdf or .xacro)
- `--output PATH`, `-o`: Output workspace path (overrides profile)
- `--verbose`, `-v`: Enable verbose logging
- `--validate-only`: Only validate configuration, don't generate files

## Examples

### Example 1: Basic Navigation Robot

```bash
./ros2_build_tool.py --wizard
```

Configuration:
- Name: `my_nav_robot`
- Distribution: `humble`
- Use case: `Navigation`
- Hardware: `lidar_rplidar`, `wheel_encoders`
- SLAM: `SLAM Toolbox`
- Enable Nav2: `Yes`
- Lifecycle management: `Yes`

Result: Complete workspace with Nav2 navigation stack, SLAM Toolbox, and lifecycle-managed nodes.

### Example 2: Mapping with Xacro URDF

```bash
./ros2_build_tool.py --wizard --urdf robot.xacro --output ~/mapping_ws
```

The tool will:
1. Process xacro file to URDF
2. Extract robot dimensions, wheel parameters, sensors
3. Generate adaptive Nav2 parameters based on robot specs
4. Create workspace at `~/mapping_ws`

### Example 3: Validation Only

Check configuration without generating files:

```bash
./ros2_build_tool.py --profile robot_profile.yaml --validate-only
```

## Generated Workspace Structure

```
my_robot_ws/
├── src/
│   └── my_robot_bringup/
│       ├── launch/
│       │   ├── robot.launch.py           # Main launch file
│       │   ├── sensors.launch.py         # Sensor drivers
│       │   ├── slam.launch.py            # SLAM
│       │   ├── navigation.launch.py      # Nav2
│       │   ├── foxglove.launch.py        # Foxglove bridge
│       │   └── diagnostics.launch.py     # Diagnostic aggregator
│       ├── config/
│       │   ├── controller.yaml           # Nav2 controller
│       │   ├── planner.yaml              # Nav2 planner
│       │   ├── costmap.yaml              # Costmaps
│       │   ├── behavior.xml              # Behavior tree
│       │   ├── bt_navigator.yaml         # BT navigator
│       │   ├── behavior_server.yaml      # Recovery behaviors
│       │   ├── recovery_server.yaml      # Recovery server
│       │   ├── smoother_server.yaml      # Path smoother
│       │   └── diagnostics.yaml          # Diagnostic aggregator
│       ├── urdf/
│       │   └── robot.urdf                # Robot description
│       └── rviz/
│           └── robot.rviz                # RViz config
├── my_robot_profile.yaml                 # Saved configuration
└── .robot_build_tool/                    # Tool data
    └── hardware/
        └── registry.yaml                 # Hardware registry
```

## Building and Running

### Build Workspace

```bash
cd my_robot_ws
rosdep install --from-paths src -y --ignore-src --rosdistro humble
colcon build
```

### Source Workspace

```bash
source install/setup.bash
```

### Launch Robot

```bash
# Main launch file (everything)
ros2 launch my_robot_bringup robot.launch.py

# With options
ros2 launch my_robot_bringup robot.launch.py use_sim_time:=true rviz:=true

# Individual components
ros2 launch my_robot_bringup sensors.launch.py
ros2 launch my_robot_bringup slam.launch.py
ros2 launch my_robot_bringup navigation.launch.py
```

## Working with Profiles

### Save Profile

Profiles are automatically saved after generation:
```
my_robot_ws/my_robot_profile.yaml
```

### Load and Modify

```python
from ros2_build_tool_core.models import RobotProfile
from pathlib import Path

# Load
profile = RobotProfile.from_yaml(Path('my_robot_profile.yaml'))

# Modify
profile.foxglove = True
profile.self_healing = True

# Save
profile.to_yaml(Path('my_robot_profile_modified.yaml'))
```

### Regenerate from Profile

```bash
./ros2_build_tool.py --profile my_robot_profile_modified.yaml
```

## Advanced Features

### Lifecycle Node Management

When enabled (`lifecycle_management: true`):
- Nodes start in specific order (robot_state_publisher → sensors → controllers)
- Automatic state transitions (unconfigured → inactive → active)
- Graceful degradation on failures
- Event-driven startup

### Self-Healing Watchdog

When enabled (`self_healing: true`):
- Monitors critical topics (`/scan`, `/odom`, `/imu`)
- Detects stale data (no messages for >5 seconds)
- Automatic recovery attempts
- Health status publishing to `/watchdog/health_status`

### Composable Nodes

When enabled (`composable_nodes: true`):
- Intra-process communication (zero-copy)
- 10-30% performance improvement for large messages
- Reduced CPU and memory usage

### Diagnostic Aggregator

When enabled (`diagnostics: true`):
- Hierarchical health monitoring
- Groups diagnostics by component type (Sensors, Motors, Navigation, System)
- Aggregated status for easy monitoring
- Web dashboard compatible

## Troubleshooting

### Launch Fails

Check logs:
```bash
ros2 launch my_robot_bringup robot.launch.py 2>&1 | tee launch.log
```

### No Transform Between Frames

Validate TF tree:
```bash
ros2 run tf2_tools view_frames
# View frames.pdf

ros2 run tf2_ros tf2_echo map base_link
```

### Topics Not Publishing

Check topic list:
```bash
ros2 topic list
ros2 topic info /scan -v
ros2 topic hz /scan
```

### QoS Incompatibility

The tool sets recommended QoS, but you can check:
```bash
ros2 topic info /scan -v
# Check reliability and durability
```

### Build Errors

Clean build:
```bash
rm -rf build install log
colcon build --cmake-clean-cache
```

### Hardware Not Found

Check hardware registry:
```bash
cat my_robot_ws/.robot_build_tool/hardware/registry.yaml
```

Add custom hardware:
```python
from ros2_build_tool_hardware import HardwareRegistry

registry = HardwareRegistry(Path('.robot_build_tool'), logger)
registry.add_custom('my_lidar', {
    'name': 'My Custom Lidar',
    'type': 'lidar',
    'github_url': 'https://github.com/myorg/my_lidar_driver.git',
    'rosdeps': ['sensor_msgs']
})
registry.save()
```

## Tips and Best Practices

### 1. Start Simple
Begin with basic configuration (mapping only), then add navigation.

### 2. Use URDF
Provide a URDF for automatic parameter optimization based on actual robot dimensions.

### 3. Validate First
Use `--validate-only` to check configuration before generating files.

### 4. Leverage Lifecycle
Enable lifecycle management for production robots that need reliability.

### 5. Monitor with Watchdog
Enable self-healing for autonomous robots that run unattended.

### 6. Use Foxglove
Foxglove provides better visualization than RViz for remote monitoring.

### 7. Test Incrementally
Launch components one at a time:
1. Sensors → 2. SLAM → 3. Navigation

### 8. Check Diagnostics
Monitor `/diagnostics_agg` topic for system health.

## Next Steps

- Read [IMPROVEMENTS.md](IMPROVEMENTS.md) for detailed feature documentation
- Check [tests/](tests/) for usage examples
- See [INSTALLATION.md](INSTALLATION.md) for setup details