"""Unit tests for URDF analyzer.

Written BEFORE implementation following TDD principles.
These tests define the expected behavior of the URDF analyzer.
"""

from pathlib import Path

import numpy as np
import pytest

# Import will fail initially - that's expected in TDD
from ros2_build_tool.core.urdf_analyzer import Transform, URDFAnalyzer


class TestURDFAnalyzer:
    """Test URDF analysis functionality."""

    @pytest.mark.unit
    def test_parse_urdf_from_string(self, mock_urdf_content: str):
        """Test parsing URDF from string content."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)

        assert analyzer.robot_name == "test_robot"
        assert len(analyzer.links) == 2
        assert len(analyzer.joints) == 1
        assert "base_link" in analyzer.links
        assert "lidar_link" in analyzer.links

    @pytest.mark.unit
    def test_parse_urdf_from_file(self, mock_urdf_content: str, temp_workspace: Path):
        """Test parsing URDF from file."""
        urdf_file = temp_workspace / "robot.urdf"
        urdf_file.write_text(mock_urdf_content)

        analyzer = URDFAnalyzer.from_file(urdf_file)

        assert analyzer.robot_name == "test_robot"
        assert len(analyzer.links) == 2

    @pytest.mark.unit
    def test_extract_frames(self, mock_urdf_content: str):
        """Test extracting frames from URDF."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)
        frames = analyzer.get_frames()

        assert len(frames) == 2
        frame_names = [f.name for f in frames]
        assert "base_link" in frame_names
        assert "lidar_link" in frame_names

    @pytest.mark.unit
    def test_extract_sensor_frames(self, mock_urdf_content: str):
        """Test extracting sensor frames specifically."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)
        sensor_frames = analyzer.get_sensor_frames()

        assert len(sensor_frames) == 1
        assert sensor_frames[0].name == "lidar_link"
        assert sensor_frames[0].parent == "base_link"

    @pytest.mark.unit
    def test_compute_transform_matrix(self):
        """Test computing 4x4 transform matrix from position and orientation."""
        transform = Transform(
            translation=(1.0, 2.0, 3.0), rotation=(0.0, 0.0, 1.57079632679)  # 90 degrees in Z
        )

        matrix = transform.to_matrix()

        assert matrix.shape == (4, 4)
        # Check translation
        assert np.allclose(matrix[:3, 3], [1.0, 2.0, 3.0])
        # Check rotation (90 degrees around Z)
        assert np.allclose(matrix[0, 0], 0.0, atol=1e-6)
        assert np.allclose(matrix[0, 1], -1.0, atol=1e-6)
        assert np.allclose(matrix[1, 0], 1.0, atol=1e-6)
        assert np.allclose(matrix[1, 1], 0.0, atol=1e-6)

    @pytest.mark.unit
    def test_get_transform_between_frames(self, mock_urdf_content: str):
        """Test getting transform between two frames."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)

        transform = analyzer.get_transform("base_link", "lidar_link")

        assert transform is not None
        assert np.allclose(transform.translation, [0.1, 0.0, 0.2])
        assert np.allclose(transform.rotation, [0.0, 0.0, 0.0])

    @pytest.mark.unit
    def test_validate_tf_tree(self, mock_urdf_content: str):
        """Test validating the TF tree structure."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)

        validation_result = analyzer.validate_tf_tree()

        assert validation_result.is_valid
        assert validation_result.has_single_root
        assert validation_result.root_frame == "base_link"
        assert not validation_result.has_cycles
        assert len(validation_result.disconnected_frames) == 0

    @pytest.mark.unit
    def test_detect_cycles_in_tf_tree(self):
        """Test detecting cycles in TF tree."""
        # Create URDF with cycle
        cyclic_urdf = """<?xml version="1.0"?>
<robot name="cyclic_robot">
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>

  <joint name="joint1" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>

  <joint name="joint2" type="fixed">
    <parent link="link2"/>
    <child link="link3"/>
  </joint>

  <joint name="joint3" type="fixed">
    <parent link="link3"/>
    <child link="link1"/>
  </joint>
</robot>"""

        analyzer = URDFAnalyzer.from_string(cyclic_urdf)
        validation_result = analyzer.validate_tf_tree()

        assert not validation_result.is_valid
        assert validation_result.has_cycles

    @pytest.mark.unit
    def test_get_robot_dimensions(self, mock_urdf_content: str):
        """Test extracting robot dimensions from URDF."""
        analyzer = URDFAnalyzer.from_string(mock_urdf_content)

        dimensions = analyzer.get_robot_dimensions()

        assert dimensions is not None
        assert dimensions.length == 0.5
        assert dimensions.width == 0.3
        assert dimensions.height == 0.2

    @pytest.mark.unit
    def test_joint_types(self):
        """Test different joint types."""
        urdf_with_joints = """<?xml version="1.0"?>
<robot name="joint_test">
  <link name="base"/>
  <link name="wheel"/>
  <link name="arm"/>

  <joint name="wheel_joint" type="continuous">
    <parent link="base"/>
    <child link="wheel"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="arm_joint" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>"""

        analyzer = URDFAnalyzer.from_string(urdf_with_joints)

        wheel_joint = analyzer.get_joint("wheel_joint")
        assert wheel_joint.type == "continuous"
        assert wheel_joint.axis == (0, 1, 0)

        arm_joint = analyzer.get_joint("arm_joint")
        assert arm_joint.type == "revolute"
        assert arm_joint.limits.lower == -1.57
        assert arm_joint.limits.upper == 1.57

    @pytest.mark.unit
    @pytest.mark.requires_ros
    def test_xacro_support(self, temp_workspace: Path):
        """Test processing xacro files - requires xacro tool or tests fallback error handling."""
        xacro_content = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <xacro:property name="width" value="0.3"/>
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${width} ${width} 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>"""

        xacro_file = temp_workspace / "robot.xacro"
        xacro_file.write_text(xacro_content)

        try:
            analyzer = URDFAnalyzer.from_xacro(xacro_file)
            assert analyzer.robot_name == "xacro_robot"
            assert "base_link" in analyzer.links
        except ValueError as e:
            assert "unresolved properties" in str(e).lower()
            pytest.skip("xacro tool not available - fallback correctly raises error")
