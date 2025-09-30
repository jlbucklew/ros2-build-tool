"""Robot specification data model and parser.

This module provides Pydantic models for defining and validating
robot specifications including dimensions, sensors, and capabilities.
"""

from enum import Enum
from pathlib import Path
from typing import List, Dict, Any, Optional
import yaml
from pydantic import BaseModel, Field, field_validator


class RobotType(Enum):
    """Enumeration of supported robot types."""
    DIFFERENTIAL_DRIVE = "differential_drive"
    ACKERMANN = "ackermann"
    OMNIDIRECTIONAL = "omnidirectional"
    LEGGED = "legged"


class SensorType(Enum):
    """Enumeration of supported sensor types."""
    LIDAR = "lidar"
    CAMERA = "camera"
    IMU = "imu"
    GPS = "gps"
    DEPTH_CAMERA = "depth_camera"


class Dimensions(BaseModel):
    """Robot physical dimensions."""
    length: float = Field(..., gt=0, description="Robot length in meters")
    width: float = Field(..., gt=0, description="Robot width in meters")
    height: float = Field(..., gt=0, description="Robot height in meters")
    wheel_radius: Optional[float] = Field(None, gt=0, description="Wheel radius in meters")
    wheel_separation: Optional[float] = Field(None, gt=0, description="Distance between wheels")


class Velocity(BaseModel):
    """Velocity limits."""
    linear: float = Field(..., description="Max linear velocity in m/s")
    angular: float = Field(..., description="Max angular velocity in rad/s")


class Sensor(BaseModel):
    """Sensor configuration."""
    type: str = Field(..., description="Sensor type")
    model: str = Field(..., description="Sensor model")
    frame: str = Field(..., description="TF frame name")
    topic: str = Field(..., description="ROS topic name")


class RobotSpec(BaseModel):
    """Complete robot specification."""
    name: str = Field(..., description="Robot name")
    type: RobotType = Field(..., description="Robot type")
    dimensions: Dimensions = Field(..., description="Physical dimensions")
    sensors: List[Sensor] = Field(default_factory=list, description="Sensor configurations")
    max_velocity: Optional[Velocity] = Field(None, description="Velocity limits")

    @field_validator('dimensions', mode='before')
    def validate_dimensions(cls, v):
        """Validate that dimensions are positive."""
        if isinstance(v, dict):
            for key, value in v.items():
                if key in ['length', 'width', 'height', 'wheel_radius', 'wheel_separation']:
                    if value is not None and value <= 0:
                        raise ValueError(f"Dimensions must be positive, got {key}={value}")
        return v

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RobotSpec':
        """Create RobotSpec from dictionary.

        Args:
            data: Dictionary containing robot specification

        Returns:
            RobotSpec instance

        Raises:
            ValueError: If required fields are missing or invalid
        """
        if 'type' not in data:
            raise ValueError("type is required")

        # Convert string type to enum if needed
        if isinstance(data.get('type'), str):
            try:
                data['type'] = RobotType(data['type'])
            except ValueError:
                # Try uppercase conversion
                data['type'] = RobotType(data['type'].upper())

        # Validate dimensions explicitly for negative values
        if 'dimensions' in data:
            dims = data['dimensions']
            for key in ['length', 'width', 'height']:
                if key in dims and dims[key] <= 0:
                    raise ValueError("Dimensions must be positive")

        return cls(**data)

    @classmethod
    def from_yaml(cls, file_path: Path) -> 'RobotSpec':
        """Load RobotSpec from YAML file.

        Args:
            file_path: Path to YAML file

        Returns:
            RobotSpec instance
        """
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    def to_yaml(self, file_path: Path) -> None:
        """Save RobotSpec to YAML file.

        Args:
            file_path: Path to save YAML file
        """
        # Convert to dict with enum values as strings
        data = self.model_dump()
        data['type'] = self.type.value

        # Convert sensor types to strings if they're enums
        for sensor in data.get('sensors', []):
            if isinstance(sensor.get('type'), Enum):
                sensor['type'] = sensor['type'].value

        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def compute_robot_radius(self) -> float:
        """Compute robot radius from dimensions.

        Returns:
            Robot radius in meters (diagonal of length and width divided by 2)
        """
        return ((self.dimensions.length ** 2 + self.dimensions.width ** 2) ** 0.5) / 2

    def get_sensors_by_type(self, sensor_type: SensorType) -> List[Sensor]:
        """Get all sensors of a specific type.

        Args:
            sensor_type: Type of sensor to filter by

        Returns:
            List of sensors matching the type
        """
        return [s for s in self.sensors if s.type == sensor_type.value]

    def get_recommended_controller(self) -> str:
        """Get recommended Nav2 controller based on robot type.

        Returns:
            Recommended controller plugin name
        """
        controller_map = {
            RobotType.DIFFERENTIAL_DRIVE: "nav2_regulated_pure_pursuit_controller",
            RobotType.ACKERMANN: "nav2_regulated_pure_pursuit_controller",
            RobotType.OMNIDIRECTIONAL: "nav2_mppi_controller",
            RobotType.LEGGED: "nav2_dwb_controller"
        }
        return controller_map.get(self.type, "nav2_regulated_pure_pursuit_controller")