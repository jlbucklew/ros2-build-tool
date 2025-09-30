"""Pytest configuration and shared fixtures.

This module provides fixtures and configuration for all tests.
Following test-first development, these fixtures are created
before any implementation code.
"""

import tempfile
from pathlib import Path
from typing import Any, Dict, Generator
from unittest.mock import MagicMock, Mock

import pytest


@pytest.fixture
def temp_workspace() -> Generator[Path, None, None]:
    """Create a temporary workspace for testing."""
    with tempfile.TemporaryDirectory() as tmpdir:
        workspace = Path(tmpdir)
        yield workspace


@pytest.fixture
def mock_urdf_content() -> str:
    """Provide sample URDF content for testing."""
    return """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="lidar_link"/>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>"""


@pytest.fixture
def robot_spec() -> Dict[str, Any]:
    """Provide a sample robot specification."""
    return {
        "name": "test_robot",
        "type": "differential_drive",
        "dimensions": {
            "length": 0.5,
            "width": 0.3,
            "height": 0.2,
            "wheel_radius": 0.1,
            "wheel_separation": 0.3,
        },
        "sensors": [
            {"type": "lidar", "model": "rplidar_a2", "frame": "lidar_link", "topic": "/scan"},
            {"type": "imu", "model": "mpu6050", "frame": "imu_link", "topic": "/imu/data"},
        ],
        "max_velocity": {"linear": 1.0, "angular": 2.0},
    }


@pytest.fixture
def mock_ros2_environment() -> Dict[str, str]:
    """Mock ROS2 environment variables."""
    return {
        "ROS_DISTRO": "humble",
        "ROS_VERSION": "2",
        "ROS_PYTHON_VERSION": "3",
        "AMENT_PREFIX_PATH": "/opt/ros/humble",
        "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages",
        "CMAKE_PREFIX_PATH": "/opt/ros/humble",
    }


@pytest.fixture
def mock_git_repo() -> Mock:
    """Mock GitPython repository."""
    repo = MagicMock()
    repo.clone_from = MagicMock(return_value=repo)
    repo.heads = MagicMock()
    repo.heads.main = MagicMock()
    repo.active_branch = MagicMock()
    repo.active_branch.name = "main"
    return repo


@pytest.fixture
def sample_nav2_params() -> Dict[str, Any]:
    """Sample Nav2 parameters for testing."""
    return {
        "controller_server": {
            "ros__parameters": {
                "controller_frequency": 20.0,
                "min_x_velocity_threshold": 0.001,
                "min_y_velocity_threshold": 0.5,
                "min_theta_velocity_threshold": 0.001,
                "progress_checker_plugin": "progress_checker",
                "goal_checker_plugin": "goal_checker",
                "controller_plugins": ["FollowPath"],
                "FollowPath": {
                    "plugin": (
                        "nav2_regulated_pure_pursuit_controller::" "RegulatedPurePursuitController"
                    ),
                    "desired_linear_vel": 0.5,
                    "lookahead_dist": 0.6,
                    "min_lookahead_dist": 0.3,
                    "max_lookahead_dist": 0.9,
                    "use_velocity_scaled_lookahead_dist": False,
                },
            }
        }
    }


@pytest.fixture
def mock_subprocess() -> Mock:
    """Mock subprocess for command execution."""
    mock_proc = MagicMock()
    mock_proc.returncode = 0
    mock_proc.stdout = "Success"
    mock_proc.stderr = ""
    mock_proc.communicate = MagicMock(return_value=("Success", ""))
    return mock_proc


@pytest.fixture(autouse=True)
def reset_singletons():
    """Reset any singleton instances between tests."""
    # This will be needed when we implement singleton patterns
    pass


@pytest.fixture
def capture_logs(caplog):
    """Fixture to capture log messages during tests."""
    with caplog.at_level("DEBUG"):
        yield caplog


# Markers for test categorization
def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "unit: Unit tests (fast, no external dependencies)")
    config.addinivalue_line("markers", "integration: Integration tests (may require ROS2)")
    config.addinivalue_line("markers", "e2e: End-to-end tests (full system test)")
    config.addinivalue_line("markers", "slow: Tests that take more than 1 second")
    config.addinivalue_line("markers", "requires_ros: Tests that require ROS2 environment")
