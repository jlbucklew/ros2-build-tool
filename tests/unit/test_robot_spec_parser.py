"""Unit tests for robot specification parser.

Written BEFORE implementation following TDD principles.
These tests define the expected behavior of the robot spec parser.
"""

import pytest
from pathlib import Path
from typing import Dict, Any

# Import will fail initially - that's expected in TDD
from ros2_build_tool.core.robot_spec import RobotSpec, RobotType, SensorType


class TestRobotSpec:
    """Test robot specification data model."""

    @pytest.mark.unit
    def test_create_robot_spec_from_dict(self, robot_spec: Dict[str, Any]):
        """Test creating RobotSpec from dictionary."""
        # This test will fail until we implement RobotSpec
        spec = RobotSpec.from_dict(robot_spec)

        assert spec.name == "test_robot"
        assert spec.type == RobotType.DIFFERENTIAL_DRIVE
        assert spec.dimensions.length == 0.5
        assert spec.dimensions.width == 0.3
        assert spec.dimensions.height == 0.2
        assert len(spec.sensors) == 2
        assert spec.max_velocity.linear == 1.0
        assert spec.max_velocity.angular == 2.0

    @pytest.mark.unit
    def test_robot_spec_validation_required_fields(self):
        """Test that missing required fields raise validation error."""
        invalid_spec = {
            "name": "test_robot"
            # Missing required fields
        }

        with pytest.raises(ValueError, match="type is required"):
            RobotSpec.from_dict(invalid_spec)

    @pytest.mark.unit
    def test_robot_spec_validation_positive_dimensions(self):
        """Test that dimensions must be positive."""
        invalid_spec = {
            "name": "test_robot",
            "type": "differential_drive",
            "dimensions": {
                "length": -0.5,  # Invalid negative
                "width": 0.3,
                "height": 0.2
            }
        }

        with pytest.raises(ValueError, match="Dimensions must be positive"):
            RobotSpec.from_dict(invalid_spec)

    @pytest.mark.unit
    def test_robot_spec_to_yaml(self, robot_spec: Dict[str, Any], temp_workspace: Path):
        """Test saving robot spec to YAML file."""
        spec = RobotSpec.from_dict(robot_spec)
        yaml_file = temp_workspace / "robot_spec.yaml"

        spec.to_yaml(yaml_file)

        assert yaml_file.exists()

        # Load and verify
        loaded_spec = RobotSpec.from_yaml(yaml_file)
        assert loaded_spec.name == spec.name
        assert loaded_spec.type == spec.type

    @pytest.mark.unit
    def test_robot_spec_from_yaml(self, robot_spec: Dict[str, Any], temp_workspace: Path):
        """Test loading robot spec from YAML file."""
        import yaml

        yaml_file = temp_workspace / "robot_spec.yaml"
        with open(yaml_file, 'w') as f:
            yaml.dump(robot_spec, f)

        spec = RobotSpec.from_yaml(yaml_file)

        assert spec.name == "test_robot"
        assert spec.type == RobotType.DIFFERENTIAL_DRIVE

    @pytest.mark.unit
    def test_robot_spec_compute_robot_radius(self, robot_spec: Dict[str, Any]):
        """Test computing robot radius from dimensions."""
        spec = RobotSpec.from_dict(robot_spec)

        # Robot radius should be computed from dimensions
        # Using the diagonal of length and width divided by 2
        expected_radius = ((0.5**2 + 0.3**2) ** 0.5) / 2

        assert abs(spec.compute_robot_radius() - expected_radius) < 0.01

    @pytest.mark.unit
    def test_sensor_type_enum(self):
        """Test SensorType enum values."""
        assert SensorType.LIDAR.value == "lidar"
        assert SensorType.CAMERA.value == "camera"
        assert SensorType.IMU.value == "imu"
        assert SensorType.GPS.value == "gps"
        assert SensorType.DEPTH_CAMERA.value == "depth_camera"

    @pytest.mark.unit
    def test_robot_type_enum(self):
        """Test RobotType enum values."""
        assert RobotType.DIFFERENTIAL_DRIVE.value == "differential_drive"
        assert RobotType.ACKERMANN.value == "ackermann"
        assert RobotType.OMNIDIRECTIONAL.value == "omnidirectional"
        assert RobotType.LEGGED.value == "legged"

    @pytest.mark.unit
    def test_robot_spec_get_sensor_by_type(self, robot_spec: Dict[str, Any]):
        """Test getting sensors by type."""
        spec = RobotSpec.from_dict(robot_spec)

        lidar_sensors = spec.get_sensors_by_type(SensorType.LIDAR)
        assert len(lidar_sensors) == 1
        assert lidar_sensors[0].model == "rplidar_a2"

        imu_sensors = spec.get_sensors_by_type(SensorType.IMU)
        assert len(imu_sensors) == 1
        assert imu_sensors[0].model == "mpu6050"

        camera_sensors = spec.get_sensors_by_type(SensorType.CAMERA)
        assert len(camera_sensors) == 0

    @pytest.mark.unit
    @pytest.mark.parametrize("robot_type,expected_controller", [
        ("differential_drive", "nav2_regulated_pure_pursuit_controller"),
        ("ackermann", "nav2_regulated_pure_pursuit_controller"),
        ("omnidirectional", "nav2_mppi_controller"),
        ("legged", "nav2_dwb_controller"),
    ])
    def test_robot_spec_recommended_controller(self, robot_type: str, expected_controller: str):
        """Test getting recommended controller based on robot type."""
        spec_dict = {
            "name": "test_robot",
            "type": robot_type,
            "dimensions": {"length": 0.5, "width": 0.3, "height": 0.2}
        }
        spec = RobotSpec.from_dict(spec_dict)

        assert spec.get_recommended_controller() == expected_controller