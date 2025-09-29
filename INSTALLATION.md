# Installation Guide

## Prerequisites

- **Operating System**: Ubuntu 22.04 (Jammy) or 24.04 (Noble)
- **ROS2 Distribution**: Humble or Jazzy
- **Python**: 3.8 or higher
- **Git**: For cloning repositories

## Quick Install

### 1. Install ROS2

If you don't have ROS2 installed:

```bash
# Ubuntu 22.04 - Humble
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Ubuntu 24.04 - Jazzy
sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Clone Repository

```bash
git clone https://github.com/jlbucklew/ros2-build-tool.git
cd ROS-Build-Tool
```

### 3. Install ROS2 Build Tool Packages

Install all packages in dependency order:

```bash
# Core package
cd ros2_build_tool_core
pip install -e .
cd ..

# Hardware package
cd ros2_build_tool_hardware
pip install -e .
cd ..

# Generators package
cd ros2_build_tool_generators
pip install -e .
cd ..

# Validation package
cd ros2_build_tool_validation
pip install -e .
cd ..

# Watchdog package
cd ros2_build_tool_watchdog
pip install -e .
cd ..

# CLI package (with wizard support)
cd ros2_build_tool_cli
pip install -e ".[wizard]"
cd ..
```

### 4. Verify Installation

```bash
python3 -c "from ros2_build_tool_core import RobotProfile; print('✓ Installation successful')"
```

## Development Installation

For development with testing tools:

```bash
pip install -r requirements.txt

# Run tests
pytest tests/
```

## Package-by-Package Installation

If you only need specific functionality:

### Core Only (Data models, platform detection)
```bash
cd ros2_build_tool_core && pip install -e .
```

### Hardware Tools (URDF parsing, hardware discovery)
```bash
cd ros2_build_tool_core && pip install -e . && cd ..
cd ros2_build_tool_hardware && pip install -e .
```

### Generators (Launch files, Nav2 parameters)
```bash
cd ros2_build_tool_core && pip install -e . && cd ..
cd ros2_build_tool_generators && pip install -e .
```

## Optional Dependencies

### Questionary (Better wizard UX)
```bash
pip install questionary
```

### URDF Parser
```bash
sudo apt install ros-${ROS_DISTRO}-urdfdom-py
```

### Xacro Support
```bash
sudo apt install ros-${ROS_DISTRO}-xacro
```

### TF2 and ROS2 Python Libraries (for validation)
```bash
sudo apt install ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-rclpy
```

## Troubleshooting

### Import Errors

If you see import errors:

```bash
# Ensure packages are in Python path
export PYTHONPATH=$PYTHONPATH:$(pwd)

# Or reinstall with pip
pip install -e ros2_build_tool_core/
```

### ROS2 Not Found

```bash
# Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy

# Check installation
ros2 --version
```

### Permission Errors

```bash
# Install with user flag
pip install --user -e .
```

### Missing System Dependencies

```bash
# Install ROS2 development tools
sudo apt install python3-pip python3-dev build-essential
```

## Uninstallation

```bash
pip uninstall ros2_build_tool_core ros2_build_tool_hardware ros2_build_tool_generators \
    ros2_build_tool_validation ros2_build_tool_watchdog ros2_build_tool_cli
```

## Next Steps

See [README.md](README.md) for usage examples and [IMPROVEMENTS.md](IMPROVEMENTS.md) for feature details.