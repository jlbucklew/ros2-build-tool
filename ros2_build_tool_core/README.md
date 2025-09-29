# ROS2 Build Tool Core

Core data models, platform detection, and environment management for ROS2 Build Tool.

## Features

- **Validated Data Models**: Pydantic-based models with comprehensive validation
- **Platform Detection**: Automatic detection of OS, ROS2 distribution, and architecture
- **Environment Management**: Cached ROS2 environment setup
- **Command Execution**: Retry logic with intelligent error classification

## Installation

```bash
pip install -e .
```

## Usage

```python
from ros2_build_tool_core import RobotProfile, UseCase, SLAMType, Platform

# Create a validated robot profile
profile = RobotProfile(
    name="my_robot",
    ros_distro="humble",
    use_case=UseCase.NAVIGATION,
    hardware=["lidar_rplidar", "imu_xsens"],
    slam_type=SLAMType.SLAM_TOOLBOX,
    navigation=True
)

# Detect platform
platform_info = Platform.detect()
print(f"Running on {platform_info['os']} {platform_info['version']}")
```