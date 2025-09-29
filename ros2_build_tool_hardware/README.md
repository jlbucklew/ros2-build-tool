# ROS2 Build Tool Hardware

Hardware discovery, URDF parsing with xacro support, and ros2_control integration.

## Features

- **Enhanced URDF Parsing**:
  - Automatic xacro processing
  - Improved bounding box calculation with transform tree
  - REP-105 frame validation
  - Transform tree completeness checking

- **ros2_control Integration**:
  - Hardware interface plugin discovery
  - Automatic controller configuration generation
  - System/Actuator/Sensor interface support

- **Package Discovery**:
  - Runtime executable discovery via ament_index
  - Intelligent primary executable selection
  - setup.py entry point parsing
  - Package metadata extraction

- **Hardware Registry**:
  - Pre-configured hardware components (lidars, cameras, IMUs, GPS)
  - Custom component registration
  - ROS distro compatibility validation

## Installation

```bash
pip install -e .
```

## Usage

```python
from ros2_build_tool_hardware import URDFParser, Ros2ControlGenerator, HardwareRegistry
import logging

logger = logging.getLogger(__name__)

# Parse URDF with xacro support
parser = URDFParser(logger)
robot_specs, sensor_frames, warnings = parser.parse(
    Path('robot.xacro'),
    xacro_args={'wheel_radius': '0.1', 'wheel_separation': '0.3'}
)

# Generate ros2_control block
generator = Ros2ControlGenerator(robot_specs, logger)
control_xml = generator.generate_differential_drive_control(
    hardware_plugin='my_robot_hardware/MyRobotHardware',
    left_wheel_joint='left_wheel_joint',
    right_wheel_joint='right_wheel_joint'
)

# Use hardware registry
registry = HardwareRegistry(Path('.'), logger)
lidar = registry.get('lidar_rplidar')
```