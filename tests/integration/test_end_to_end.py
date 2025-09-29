"""
Integration tests for end-to-end workspace generation
"""

import pytest
import tempfile
import shutil
import logging
from pathlib import Path

from ros2_build_tool_core.models import (
    RobotProfile,
    RobotSpecs,
    UseCase,
    SLAMType,
    BuildType
)
from ros2_build_tool_core.orchestrator import WorkspaceOrchestrator
from ros2_build_tool_hardware.hardware_registry import HardwareRegistry


@pytest.fixture
def temp_workspace():
    """Create temporary workspace directory"""
    temp_dir = tempfile.mkdtemp(prefix='ros2_build_tool_test_')
    yield Path(temp_dir)
    # Cleanup
    shutil.rmtree(temp_dir, ignore_errors=True)


@pytest.fixture
def temp_data_dir():
    """Create temporary data directory"""
    temp_dir = tempfile.mkdtemp(prefix='ros2_build_tool_data_')
    yield Path(temp_dir)
    shutil.rmtree(temp_dir, ignore_errors=True)


@pytest.fixture
def test_profile():
    """Create test robot profile"""
    return RobotProfile(
        name='test_robot',
        ros_distro='humble',
        use_case=UseCase.MAPPING,
        hardware=['wheel_encoders', 'lidar_rplidar'],
        slam_type=SLAMType.SLAM_TOOLBOX,
        navigation=False,
        foxglove=False,
        rviz=True,
        teleop=False,
        recording=False,
        diagnostics=True,
        build_type=BuildType.RELEASE,
        lifecycle_management=True,
        self_healing=True,
        composable_nodes=False,
        robot_specs=RobotSpecs(
            width=0.5,
            length=0.6,
            height=0.3,
            wheel_radius=0.1,
            wheel_separation=0.4,
            max_linear_velocity=1.0,
            max_angular_velocity=2.0
        )
    )


@pytest.fixture
def orchestrator():
    """Create orchestrator instance"""
    logger = logging.getLogger('test_orchestrator')
    return WorkspaceOrchestrator(logger)


@pytest.fixture
def hardware_registry(temp_data_dir):
    """Create hardware registry instance"""
    logger = logging.getLogger('test_hardware_registry')
    return HardwareRegistry(temp_data_dir, logger)


class TestWorkspaceGeneration:
    """Test workspace generation pipeline"""

    def test_create_workspace_structure_only(
        self,
        orchestrator,
        test_profile,
        hardware_registry,
        temp_workspace
    ):
        """Test workspace structure creation without building"""
        result = orchestrator.create_workspace(
            profile=test_profile,
            workspace_path=temp_workspace,
            hardware_registry=hardware_registry,
            skip_build=True,
            skip_clone=True
        )

        # Check result
        assert result.success, f"Workspace creation failed: {result.errors}"
        assert result.workspace_path == temp_workspace
        assert len(result.created_packages) > 0

        # Check directory structure
        assert (temp_workspace / 'src').exists()
        assert (temp_workspace / 'build').exists()
        assert (temp_workspace / 'install').exists()
        assert (temp_workspace / 'log').exists()

        # Check bringup package
        bringup_pkg = temp_workspace / 'src' / f'{test_profile.name}_bringup'
        assert bringup_pkg.exists()
        assert (bringup_pkg / 'launch').exists()
        assert (bringup_pkg / 'config').exists()
        assert (bringup_pkg / 'urdf').exists()
        assert (bringup_pkg / 'package.xml').exists()
        assert (bringup_pkg / 'CMakeLists.txt').exists()

        # Check launch files exist
        launch_files = list((bringup_pkg / 'launch').glob('*.py'))
        assert len(launch_files) > 0, "No launch files generated"

    def test_create_workspace_with_navigation(
        self,
        orchestrator,
        hardware_registry,
        temp_workspace
    ):
        """Test workspace creation with navigation enabled"""
        profile = RobotProfile(
            name='nav_robot',
            ros_distro='humble',
            use_case=UseCase.NAVIGATION,
            hardware=['wheel_encoders', 'lidar_rplidar'],
            slam_type=SLAMType.SLAM_TOOLBOX,
            navigation=True,
            foxglove=False,
            rviz=True,
            robot_specs=RobotSpecs(
                width=0.5,
                length=0.6,
                height=0.3,
                wheel_radius=0.1,
                wheel_separation=0.4,
                max_linear_velocity=1.0,
                max_angular_velocity=2.0
            )
        )

        result = orchestrator.create_workspace(
            profile=profile,
            workspace_path=temp_workspace,
            hardware_registry=hardware_registry,
            skip_build=True,
            skip_clone=True
        )

        assert result.success
        assert len(result.created_packages) > 0

        # Check Nav2 config files
        bringup_pkg = temp_workspace / 'src' / f'{profile.name}_bringup'
        config_files = list((bringup_pkg / 'config').glob('*.yaml'))
        assert len(config_files) > 0, "No config files generated"

    def test_create_workspace_with_foxglove(
        self,
        orchestrator,
        hardware_registry,
        temp_workspace
    ):
        """Test workspace creation with Foxglove enabled"""
        profile = RobotProfile(
            name='foxglove_robot',
            ros_distro='humble',
            use_case=UseCase.MAPPING,
            hardware=['wheel_encoders'],
            slam_type=SLAMType.NONE,
            navigation=False,
            foxglove=True,
            rviz=False
        )

        result = orchestrator.create_workspace(
            profile=profile,
            workspace_path=temp_workspace,
            hardware_registry=hardware_registry,
            skip_build=True,
            skip_clone=True
        )

        assert result.success

        # Check Foxglove launch file
        bringup_pkg = temp_workspace / 'src' / f'{profile.name}_bringup'
        foxglove_launch = bringup_pkg / 'launch' / 'foxglove.launch.py'
        assert foxglove_launch.exists(), "Foxglove launch file not generated"

        # Check Foxglove config
        foxglove_config = bringup_pkg / 'config' / 'foxglove_topics.yaml'
        assert foxglove_config.exists(), "Foxglove config not generated"

    def test_validation_errors(self, orchestrator, hardware_registry, temp_workspace):
        """Test that validation catches errors"""
        # Create profile with invalid hardware
        profile = RobotProfile(
            name='invalid_robot',
            ros_distro='humble',
            use_case=UseCase.MAPPING,
            hardware=['nonexistent_hardware'],
            slam_type=SLAMType.NONE,
            navigation=False
        )

        result = orchestrator.create_workspace(
            profile=profile,
            workspace_path=temp_workspace,
            hardware_registry=hardware_registry,
            skip_build=True,
            skip_clone=True
        )

        # Should still succeed but with warnings
        assert len(result.warnings) > 0

    def test_rollback_on_error(self, orchestrator, hardware_registry):
        """Test rollback functionality"""
        # Create invalid workspace path (read-only or non-creatable)
        # This is platform-specific, so we'll just test rollback directly
        temp_dir = tempfile.mkdtemp(prefix='ros2_build_tool_rollback_')
        temp_path = Path(temp_dir)

        try:
            # Create some structure
            (temp_path / 'test_file.txt').write_text('test')

            # Test rollback
            result = orchestrator.rollback(temp_path)
            assert result, "Rollback should succeed"
            assert not temp_path.exists(), "Workspace should be removed after rollback"

        finally:
            # Cleanup if rollback failed
            if temp_path.exists():
                shutil.rmtree(temp_path, ignore_errors=True)


class TestProfileValidation:
    """Test profile validation"""

    def test_valid_profile(self):
        """Test that valid profile passes validation"""
        profile = RobotProfile(
            name='valid_robot',
            ros_distro='humble',
            use_case=UseCase.MAPPING,
            hardware=['wheel_encoders'],
            slam_type=SLAMType.SLAM_TOOLBOX,
            robot_specs=RobotSpecs(
                width=0.5,
                length=0.6,
                height=0.3
            )
        )
        assert profile.name == 'valid_robot'
        assert profile.ros_distro == 'humble'

    def test_invalid_ros_distro(self):
        """Test that invalid ROS distro is rejected"""
        with pytest.raises(Exception):  # Pydantic ValidationError
            RobotProfile(
                name='test',
                ros_distro='invalid_distro',
                use_case=UseCase.MAPPING,
                hardware=['wheel_encoders'],
                slam_type=SLAMType.NONE
            )


class TestHardwareRegistry:
    """Test hardware registry functionality"""

    def test_load_default_hardware(self, hardware_registry):
        """Test loading default hardware components"""
        available = hardware_registry.list_available()
        assert len(available) > 0, "No hardware components loaded"
        assert 'wheel_encoders' in available
        assert 'lidar_rplidar' in available

    def test_get_hardware_component(self, hardware_registry):
        """Test retrieving hardware component"""
        component = hardware_registry.get('wheel_encoders')
        assert component is not None
        assert component.name == 'Wheel Encoders'
        assert component.type == 'odometry'

    def test_search_hardware(self, hardware_registry):
        """Test hardware search"""
        results = hardware_registry.search('lidar')
        assert len(results) > 0
        assert any('lidar' in r for r in results)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])