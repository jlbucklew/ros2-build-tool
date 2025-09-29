"""
Tests for hardware discovery and registry
"""

import pytest
from pathlib import Path
from ros2_build_tool_hardware.hardware_registry import HardwareRegistry
from ros2_build_tool_hardware.hardware_interface_discovery import (
    HardwareInterfaceDiscovery,
    HardwareInterfaceType
)
from ros2_build_tool_core.models import HardwareComponent
import logging


class TestHardwareRegistry:
    """Test hardware component registry"""

    def test_load_default_registry(self, temp_dir):
        """Test loading default hardware components"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        # Should have default components
        assert len(registry.list_available()) > 0

        # Check specific components
        lidar = registry.get('lidar_rplidar')
        assert lidar is not None
        assert lidar.type == 'lidar'

    def test_add_custom_component(self, temp_dir):
        """Test adding custom hardware component"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        custom_hw = {
            'name': 'Custom Lidar',
            'type': 'lidar',
            'repo': {'url': 'https://github.com/custom/lidar.git'},
            'rosdeps': ['sensor_msgs']
        }

        component = registry.add_custom('custom_lidar', custom_hw)
        assert component is not None

        # Should be retrievable
        retrieved = registry.get('custom_lidar')
        assert retrieved.name == 'Custom Lidar'

    def test_search_hardware(self, temp_dir):
        """Test searching hardware components"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        # Search by type
        results = registry.search('lidar')
        assert len(results) > 0
        assert all('lidar' in result.lower() for result in results)

    def test_list_by_type(self, temp_dir):
        """Test filtering hardware by type"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        lidars = registry.list_by_type('lidar')
        assert len(lidars) > 0

        cameras = registry.list_by_type('camera')
        assert len(cameras) > 0

        # Types should be different
        assert set(lidars).isdisjoint(set(cameras))

    def test_save_and_load_registry(self, temp_dir):
        """Test saving and loading registry"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        # Add custom component
        custom_hw = {
            'name': 'Test Component',
            'type': 'test',
            'rosdeps': []
        }
        registry.add_custom('test_component', custom_hw)

        # Save registry
        registry.save()

        # Create new registry instance (should load from file)
        new_registry = HardwareRegistry(temp_dir, logger)

        # Custom component should be loaded
        component = new_registry.get('test_component')
        assert component is not None
        assert component.name == 'Test Component'

    def test_validate_compatibility(self, temp_dir):
        """Test validating hardware compatibility with ROS distro"""
        logger = logging.getLogger(__name__)
        registry = HardwareRegistry(temp_dir, logger)

        # Test with components that have branch info
        hardware_ids = ['lidar_rplidar', 'imu_xsens']
        compatibility = registry.validate_compatibility(hardware_ids, 'humble')

        assert 'lidar_rplidar' in compatibility
        assert 'imu_xsens' in compatibility


class TestHardwareInterfaceDiscovery:
    """Test hardware interface plugin discovery"""

    def test_classify_interface_type(self):
        """Test classifying hardware interface types"""
        logger = logging.getLogger(__name__)
        discovery = HardwareInterfaceDiscovery(logger)

        # Test system interface
        system_class = "hardware_interface::SystemInterface"
        result = discovery._classify_hardware_interface(system_class)
        assert result == HardwareInterfaceType.SYSTEM

        # Test actuator interface
        actuator_class = "hardware_interface::ActuatorInterface"
        result = discovery._classify_hardware_interface(actuator_class)
        assert result == HardwareInterfaceType.ACTUATOR

        # Test sensor interface
        sensor_class = "hardware_interface::SensorInterface"
        result = discovery._classify_hardware_interface(sensor_class)
        assert result == HardwareInterfaceType.SENSOR

    def test_validate_plugin_compatibility(self):
        """Test validating plugin compatibility with robot type"""
        logger = logging.getLogger(__name__)
        discovery = HardwareInterfaceDiscovery(logger)

        # SystemInterface compatible with differential_drive
        assert discovery.validate_plugin_compatibility(
            "hardware_interface::SystemInterface",
            "differential_drive"
        )

        # ActuatorInterface also compatible
        assert discovery.validate_plugin_compatibility(
            "hardware_interface::ActuatorInterface",
            "differential_drive"
        )

    def test_recommend_controller_config(self):
        """Test recommending controller configuration"""
        logger = logging.getLogger(__name__)
        discovery = HardwareInterfaceDiscovery(logger)

        plugin_info = {
            'type': 'my_robot::MyRobotHardware',
            'interface_type': 'hardware_interface::SystemInterface'
        }

        robot_specs = {
            'wheel_radius': 0.1,
            'wheel_separation': 0.3
        }

        config = discovery.recommend_controller_config(plugin_info, robot_specs)

        # Should recommend diff_drive_controller
        assert 'controllers' in config
        controllers = config['controllers']
        assert len(controllers) > 0

        # Check for expected controllers
        controller_names = [c['name'] for c in controllers]
        assert 'diff_drive_controller' in controller_names or 'joint_state_broadcaster' in controller_names

    def test_generate_ros2_control_params(self):
        """Test generating ros2_control parameter YAML"""
        logger = logging.getLogger(__name__)
        discovery = HardwareInterfaceDiscovery(logger)

        plugins = [
            {
                'type': 'my_robot::MyRobotHardware',
                'interface_type': 'hardware_interface::SystemInterface'
            }
        ]

        yaml_content = discovery.generate_ros2_control_params(plugins)

        # Check YAML structure
        assert 'controller_manager:' in yaml_content
        assert 'update_rate' in yaml_content
        assert 'diff_drive_controller' in yaml_content