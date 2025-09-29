"""
Pytest configuration and fixtures
"""

import pytest
from pathlib import Path
import tempfile
import shutil


@pytest.fixture
def temp_dir():
    """Create a temporary directory for tests"""
    temp_path = Path(tempfile.mkdtemp())
    yield temp_path
    shutil.rmtree(temp_path, ignore_errors=True)


@pytest.fixture
def sample_urdf():
    """Sample URDF content for testing"""
    return '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit velocity="10.0" effort="1.0"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
    <limit velocity="10.0" effort="1.0"/>
  </joint>

  <link name="lidar_link"/>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
'''


@pytest.fixture
def sample_robot_specs():
    """Sample RobotSpecs for testing"""
    from ros2_build_tool_core.models import RobotSpecs
    return RobotSpecs(
        width=0.4,
        length=0.5,
        height=0.3,
        wheel_radius=0.1,
        wheel_separation=0.3,
        max_linear_velocity=1.0,
        max_angular_velocity=2.0
    )


@pytest.fixture
def sample_robot_profile(temp_dir):
    """Sample RobotProfile for testing"""
    from ros2_build_tool_core.models import RobotProfile, UseCase, SLAMType
    return RobotProfile(
        name="test_robot",
        ros_distro="humble",
        use_case=UseCase.NAVIGATION,
        hardware=["lidar_rplidar", "imu_xsens"],
        slam_type=SLAMType.SLAM_TOOLBOX,
        navigation=True,
        workspace_path=temp_dir
    )