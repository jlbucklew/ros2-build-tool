"""
Tests for URDF parsing
"""

import pytest
from pathlib import Path
from ros2_build_tool_hardware.urdf_parser import URDFParser, Ros2ControlGenerator
import logging


class TestURDFParser:
    """Test URDF parsing functionality"""

    def test_parse_valid_urdf(self, temp_dir, sample_urdf):
        """Test parsing a valid URDF file"""
        urdf_file = temp_dir / "test_robot.urdf"
        urdf_file.write_text(sample_urdf)

        logger = logging.getLogger(__name__)
        parser = URDFParser(logger)

        robot_specs, sensor_frames, warnings = parser.parse(urdf_file)

        # Check robot specs extracted correctly
        assert robot_specs.width > 0
        assert robot_specs.length > 0
        assert robot_specs.height > 0
        assert robot_specs.wheel_radius > 0
        assert robot_specs.wheel_separation > 0

    def test_sensor_frame_extraction(self, temp_dir, sample_urdf):
        """Test extracting sensor frames from URDF"""
        urdf_file = temp_dir / "test_robot.urdf"
        urdf_file.write_text(sample_urdf)

        logger = logging.getLogger(__name__)
        parser = URDFParser(logger)

        robot_specs, sensor_frames, warnings = parser.parse(urdf_file)

        # Should find lidar_link sensor frame
        sensor_names = [sf.name for sf in sensor_frames]
        assert 'lidar_link' in sensor_names

        # Check sensor frame details
        lidar_frame = next(sf for sf in sensor_frames if sf.name == 'lidar_link')
        assert lidar_frame.sensor_type == 'lidar'
        assert lidar_frame.parent == 'base_link'

    def test_rep105_validation(self, temp_dir, sample_urdf):
        """Test REP-105 frame validation"""
        urdf_file = temp_dir / "test_robot.urdf"
        urdf_file.write_text(sample_urdf)

        logger = logging.getLogger(__name__)
        parser = URDFParser(logger)

        robot_specs, sensor_frames, warnings = parser.parse(urdf_file)

        # Should have base_link and base_footprint
        warning_msgs = ' '.join(warnings)
        assert 'base_link' not in warning_msgs or 'base_footprint' not in warning_msgs

    def test_wheel_parameter_extraction(self, temp_dir, sample_urdf):
        """Test extracting wheel parameters"""
        urdf_file = temp_dir / "test_robot.urdf"
        urdf_file.write_text(sample_urdf)

        logger = logging.getLogger(__name__)
        parser = URDFParser(logger)

        robot_specs, sensor_frames, warnings = parser.parse(urdf_file)

        # Check wheel parameters
        assert robot_specs.wheel_radius == pytest.approx(0.1, abs=0.01)
        assert robot_specs.wheel_separation == pytest.approx(0.3, abs=0.01)


class TestRos2ControlGenerator:
    """Test ros2_control URDF generation"""

    def test_differential_drive_generation(self, sample_robot_specs):
        """Test generating ros2_control block for differential drive"""
        logger = logging.getLogger(__name__)
        generator = Ros2ControlGenerator(sample_robot_specs, logger)

        control_xml = generator.generate_differential_drive_control(
            hardware_plugin='test_hardware/TestHardware',
            left_wheel_joint='left_wheel_joint',
            right_wheel_joint='right_wheel_joint'
        )

        # Check generated XML contains required elements
        assert '<ros2_control' in control_xml
        assert 'test_hardware/TestHardware' in control_xml
        assert 'left_wheel_joint' in control_xml
        assert 'right_wheel_joint' in control_xml
        assert 'velocity' in control_xml
        assert 'position' in control_xml

    def test_sensor_interface_generation(self, sample_robot_specs):
        """Test generating sensor interfaces"""
        from ros2_build_tool_core.models import SensorFrame

        logger = logging.getLogger(__name__)
        generator = Ros2ControlGenerator(sample_robot_specs, logger)

        sensor_frames = [
            SensorFrame(
                name='imu_link',
                parent='base_link',
                child='imu_link',
                xyz=[0, 0, 0.1],
                rpy=[0, 0, 0],
                sensor_type='imu'
            )
        ]

        sensor_xml = generator.generate_sensor_interfaces(sensor_frames)

        # Check IMU sensor interface
        assert '<ros2_control' in sensor_xml
        assert 'imu_link' in sensor_xml
        assert 'orientation' in sensor_xml
        assert 'angular_velocity' in sensor_xml
        assert 'linear_acceleration' in sensor_xml