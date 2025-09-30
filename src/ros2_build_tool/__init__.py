"""ROS2 Build Tool - Production-grade automation for ROS2 workspaces.

This package provides automated workspace generation for ROS2 robots,
including Nav2 configuration, SLAM setup, and hardware integration.
"""

from ros2_build_tool.core.robot_spec import (
    Dimensions,
    RobotSpec,
    RobotType,
    Sensor,
    SensorType,
    Velocity,
)
from ros2_build_tool.core.urdf_analyzer import (
    Frame,
    Joint,
    Link,
    Transform,
    URDFAnalyzer,
    ValidationResult,
)

__version__ = "1.0.0"
__author__ = "ROS2 Build Tool Team"
__email__ = "team@ros2buildtool.org"

# Public API exports
__all__ = [
    # Metadata
    "__version__",
    "__author__",
    "__email__",
    # Robot specification
    "RobotSpec",
    "RobotType",
    "SensorType",
    "Dimensions",
    "Velocity",
    "Sensor",
    # URDF analyzer
    "URDFAnalyzer",
    "Transform",
    "Frame",
    "Joint",
    "Link",
    "ValidationResult",
]
