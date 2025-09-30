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
    def validate_dimensions(cls, v: Any) -> Any:
        """Validate that dimensions are positive.

        Args:
            v: Value to validate

        Returns:
            Validated value

        Raises:
            ValueError: If dimensions are not positive
        """
        if isinstance(v, dict):
            for key, value in v.items():
                if key in ['length', 'width', 'height', 'wheel_radius', 'wheel_separation']:
                    if value is not None and value <= 0:
                        raise ValueError(
                            f"Dimension '{key}' must be positive, got {value}. "
                            f"All robot dimensions must be greater than zero."
                        )
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
            available_types = [t.value for t in RobotType]
            raise ValueError(
                f"Required field 'type' is missing. "
                f"Must be one of: {', '.join(available_types)}"
            )

        # Convert string type to enum if needed
        if isinstance(data.get('type'), str):
            type_str = data['type']
            try:
                data['type'] = RobotType(type_str)
            except ValueError:
                # Try uppercase conversion
                try:
                    data['type'] = RobotType(type_str.upper())
                except ValueError:
                    available_types = [t.value for t in RobotType]
                    raise ValueError(
                        f"Invalid robot type: '{type_str}'. "
                        f"Must be one of: {', '.join(available_types)}"
                    )

        # Pydantic will handle dimension validation via field_validator
        # No need for redundant validation here

        return cls(**data)

    @classmethod
    def from_yaml(cls, file_path: Path) -> 'RobotSpec':
        """Load RobotSpec from YAML file.

        Args:
            file_path: Path to YAML file

        Returns:
            RobotSpec instance

        Raises:
            FileNotFoundError: If file does not exist
            PermissionError: If file cannot be read
            yaml.YAMLError: If YAML parsing fails
        """
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Robot spec file not found: {file_path}")
        except PermissionError:
            raise PermissionError(f"Permission denied reading file: {file_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in file {file_path}: {e}")
        except Exception as e:
            raise IOError(f"Error reading file {file_path}: {e}")

        return cls.from_dict(data)

    def to_yaml(self, file_path: Path) -> None:
        """Save RobotSpec to YAML file.

        Args:
            file_path: Path to save YAML file

        Raises:
            PermissionError: If file cannot be written
            IOError: If writing fails
        """
        # Convert to dict with enum values as strings
        data = self.model_dump()
        data['type'] = self.type.value

        # Convert sensor types to strings if they're enums
        for sensor in data.get('sensors', []):
            if isinstance(sensor.get('type'), Enum):
                sensor['type'] = sensor['type'].value

        try:
            with open(file_path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
        except PermissionError:
            raise PermissionError(f"Permission denied writing to file: {file_path}")
        except Exception as e:
            raise IOError(f"Error writing to file {file_path}: {e}")

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