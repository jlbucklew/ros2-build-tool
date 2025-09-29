"""
Tests for Pydantic data models
"""

import pytest
from pathlib import Path
from ros2_build_tool_core.models import (
    RobotSpecs,
    RobotProfile,
    UseCase,
    SLAMType,
    BuildType,
    SensorFrame,
    HardwareComponent,
    DependencyManifest
)


class TestRobotSpecs:
    """Test RobotSpecs model validation"""

    def test_valid_robot_specs(self):
        """Test creating valid robot specs"""
        specs = RobotSpecs(
            width=0.5,
            length=0.6,
            height=0.3,
            wheel_radius=0.1,
            wheel_separation=0.3
        )
        assert specs.width == 0.5
        assert specs.robot_radius > 0  # Computed automatically

    def test_robot_radius_calculation(self):
        """Test automatic robot radius calculation"""
        specs = RobotSpecs(width=0.4, length=0.6, height=0.3)
        # Robot radius should be max(width, length) / 2.0
        assert specs.robot_radius == 0.3  # 0.6 / 2.0

    def test_invalid_dimensions(self):
        """Test validation of dimension constraints"""
        with pytest.raises(ValueError):
            RobotSpecs(width=-0.5, length=0.6, height=0.3)  # Negative width

        with pytest.raises(ValueError):
            RobotSpecs(width=10.0, length=0.6, height=0.3)  # Too large

    def test_velocity_limits(self):
        """Test velocity limit validation"""
        specs = RobotSpecs(
            width=0.5,
            length=0.6,
            height=0.3,
            max_linear_velocity=2.0,
            max_angular_velocity=3.0
        )
        assert specs.max_linear_velocity == 2.0

        with pytest.raises(ValueError):
            RobotSpecs(
                width=0.5, length=0.6, height=0.3,
                max_linear_velocity=15.0  # Too high
            )


class TestSensorFrame:
    """Test SensorFrame model validation"""

    def test_valid_sensor_frame(self):
        """Test creating valid sensor frame"""
        frame = SensorFrame(
            name="lidar_link",
            parent="base_link",
            child="lidar_link",
            xyz=[0.2, 0, 0.15],
            rpy=[0, 0, 0],
            sensor_type="lidar"
        )
        assert frame.name == "lidar_link"
        assert len(frame.xyz) == 3

    def test_invalid_transform_vectors(self):
        """Test validation of transform vectors"""
        with pytest.raises(ValueError):
            SensorFrame(
                name="test",
                parent="base",
                child="sensor",
                xyz=[0, 0],  # Only 2 elements
                rpy=[0, 0, 0],
                sensor_type="lidar"
            )

    def test_invalid_sensor_type(self):
        """Test validation of sensor type"""
        with pytest.raises(ValueError):
            SensorFrame(
                name="test",
                parent="base",
                child="sensor",
                xyz=[0, 0, 0],
                rpy=[0, 0, 0],
                sensor_type="invalid_type"  # Not in allowed types
            )


class TestHardwareComponent:
    """Test HardwareComponent model validation"""

    def test_valid_hardware_component(self):
        """Test creating valid hardware component"""
        hw = HardwareComponent(
            name="RPLidar",
            type="lidar",
            repo={'url': 'https://github.com/Slamtec/rplidar_ros.git'},
            rosdeps=['sensor_msgs'],
            github_url='https://github.com/Slamtec/rplidar_ros.git'
        )
        assert hw.name == "RPLidar"
        assert hw.type == "lidar"

    def test_invalid_github_url(self):
        """Test validation of GitHub URL"""
        with pytest.raises(ValueError):
            HardwareComponent(
                name="Test",
                type="lidar",
                github_url='http://invalid-url.com/repo.git'  # Not GitHub
            )


class TestRobotProfile:
    """Test RobotProfile model validation"""

    def test_valid_robot_profile(self, temp_dir):
        """Test creating valid robot profile"""
        profile = RobotProfile(
            name="test_robot",
            ros_distro="humble",
            use_case=UseCase.NAVIGATION,
            hardware=["lidar_rplidar"],
            slam_type=SLAMType.SLAM_TOOLBOX,
            workspace_path=temp_dir
        )
        assert profile.name == "test_robot"
        assert profile.ros_distro == "humble"

    def test_invalid_ros_distro(self, temp_dir):
        """Test validation of ROS distro"""
        with pytest.raises(ValueError):
            RobotProfile(
                name="test",
                ros_distro="invalid",  # Not humble or jazzy
                use_case=UseCase.NAVIGATION,
                hardware=["lidar"],
                slam_type=SLAMType.NONE,
                workspace_path=temp_dir
            )

    def test_empty_hardware_list(self, temp_dir):
        """Test validation of hardware list"""
        with pytest.raises(ValueError):
            RobotProfile(
                name="test",
                ros_distro="humble",
                use_case=UseCase.NAVIGATION,
                hardware=[],  # Empty list not allowed
                slam_type=SLAMType.NONE,
                workspace_path=temp_dir
            )

    def test_yaml_serialization(self, temp_dir):
        """Test saving and loading profile from YAML"""
        profile = RobotProfile(
            name="test_robot",
            ros_distro="humble",
            use_case=UseCase.NAVIGATION,
            hardware=["lidar_rplidar"],
            slam_type=SLAMType.SLAM_TOOLBOX,
            workspace_path=temp_dir
        )

        yaml_path = temp_dir / "profile.yaml"
        profile.to_yaml(yaml_path)

        assert yaml_path.exists()

        loaded_profile = RobotProfile.from_yaml(yaml_path)
        assert loaded_profile.name == profile.name
        assert loaded_profile.use_case == profile.use_case


class TestDependencyManifest:
    """Test DependencyManifest model"""

    def test_merge_dependencies(self):
        """Test merging dependency manifests"""
        manifest1 = DependencyManifest(
            system_apt=['pkg1', 'pkg2'],
            rosdep_keys=['ros_pkg1'],
            python_packages=['numpy']
        )

        manifest2 = DependencyManifest(
            system_apt=['pkg2', 'pkg3'],  # pkg2 is duplicate
            rosdep_keys=['ros_pkg2'],
            python_packages=['scipy']
        )

        manifest1.merge(manifest2)

        # Duplicates should be removed
        assert set(manifest1.system_apt) == {'pkg1', 'pkg2', 'pkg3'}
        assert set(manifest1.rosdep_keys) == {'ros_pkg1', 'ros_pkg2'}
        assert set(manifest1.python_packages) == {'numpy', 'scipy'}