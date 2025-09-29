"""
Hardware interface plugin discovery and validation for ros2_control
"""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Optional
from enum import Enum

try:
    from ament_index_python.packages import get_package_share_directory
    AMENT_INDEX_AVAILABLE = True
except ImportError:
    AMENT_INDEX_AVAILABLE = False


class HardwareInterfaceType(str, Enum):
    """Hardware interface types from ros2_control"""
    SYSTEM = "hardware_interface::SystemInterface"
    ACTUATOR = "hardware_interface::ActuatorInterface"
    SENSOR = "hardware_interface::SensorInterface"


class HardwareInterfaceDiscovery:
    """Discover and analyze ros2_control hardware interface plugins"""

    def __init__(self, logger: logging.Logger):
        self.logger = logger

    def discover_plugins(self, package_name: str) -> List[Dict]:
        """
        Discover ros2_control hardware interface plugins from package
        Returns list of plugin dictionaries with name, type, and base_class
        """
        if not AMENT_INDEX_AVAILABLE:
            self.logger.warning("ament_index_python not available")
            return []

        try:
            share_dir = Path(get_package_share_directory(package_name))
            plugins_xml = share_dir / 'plugins.xml'

            if not plugins_xml.exists():
                self.logger.debug(f"No plugins.xml found for {package_name}")
                return []

            return self._parse_plugins_xml(plugins_xml)

        except Exception as e:
            self.logger.warning(f"Failed to discover plugins for {package_name}: {e}")
            return []

    def _parse_plugins_xml(self, plugins_xml: Path) -> List[Dict]:
        """Parse plugins.xml file to extract hardware interface plugins"""
        try:
            tree = ET.parse(plugins_xml)
            root = tree.getroot()

            plugins = []

            # plugins.xml format:
            # <library path="plugin_name">
            #   <class type="package::ClassName" base_class_type="hardware_interface::SystemInterface">
            #     <description>...</description>
            #   </class>
            # </library>

            for library in root.findall('.//library'):
                library_path = library.get('path', '')

                for class_elem in library.findall('class'):
                    base_class = class_elem.get('base_class_type', '')

                    # Check if this is a hardware interface plugin
                    if 'hardware_interface' in base_class:
                        plugin_type = self._classify_hardware_interface(base_class)
                        description_elem = class_elem.find('description')

                        plugin_info = {
                            'name': class_elem.get('name', ''),
                            'type': class_elem.get('type', ''),
                            'base_class': base_class,
                            'interface_type': plugin_type.value if plugin_type else 'unknown',
                            'library_path': library_path,
                            'description': description_elem.text.strip() if description_elem is not None else ''
                        }

                        plugins.append(plugin_info)
                        self.logger.info(f"Found hardware interface plugin: {plugin_info['type']}")

            return plugins

        except Exception as e:
            self.logger.error(f"Failed to parse plugins.xml: {e}")
            return []

    def _classify_hardware_interface(self, base_class: str) -> Optional[HardwareInterfaceType]:
        """Classify hardware interface type from base class"""
        if HardwareInterfaceType.SYSTEM.value in base_class:
            return HardwareInterfaceType.SYSTEM
        elif HardwareInterfaceType.ACTUATOR.value in base_class:
            return HardwareInterfaceType.ACTUATOR
        elif HardwareInterfaceType.SENSOR.value in base_class:
            return HardwareInterfaceType.SENSOR
        return None

    def validate_plugin_compatibility(
        self,
        plugin_type: str,
        robot_type: str = "differential_drive"
    ) -> bool:
        """
        Validate if a hardware interface plugin is compatible with robot type
        """
        # SystemInterface is compatible with differential drive
        if HardwareInterfaceType.SYSTEM.value in plugin_type:
            return True

        # ActuatorInterface requires per-joint control
        if HardwareInterfaceType.ACTUATOR.value in plugin_type:
            if robot_type in ["differential_drive", "ackermann"]:
                return True

        return False

    def recommend_controller_config(
        self,
        plugin_info: Dict,
        robot_specs: Dict
    ) -> Dict:
        """
        Recommend controller configuration based on hardware interface plugin
        """
        interface_type = plugin_info.get('interface_type', 'unknown')

        # Default controller configuration
        controller_config = {
            'controller_manager': {
                'ros__parameters': {
                    'update_rate': 100  # Hz
                }
            },
            'controllers': []
        }

        # Recommend controllers based on interface type
        if interface_type == HardwareInterfaceType.SYSTEM.value:
            # System interface - typically differential drive
            controller_config['controllers'].append({
                'name': 'diff_drive_controller',
                'type': 'diff_drive_controller/DiffDriveController',
                'params': {
                    'left_wheel_names': ['left_wheel_joint'],
                    'right_wheel_names': ['right_wheel_joint'],
                    'wheel_separation': robot_specs.get('wheel_separation', 0.3),
                    'wheel_radius': robot_specs.get('wheel_radius', 0.1),
                    'publish_rate': 50.0,
                    'odom_frame_id': 'odom',
                    'base_frame_id': 'base_link',
                    'pose_covariance_diagonal': [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
                    'twist_covariance_diagonal': [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
                }
            })

            # Add joint state broadcaster
            controller_config['controllers'].append({
                'name': 'joint_state_broadcaster',
                'type': 'joint_state_broadcaster/JointStateBroadcaster',
                'params': {}
            })

        elif interface_type == HardwareInterfaceType.ACTUATOR.value:
            # Actuator interface - per-joint control
            controller_config['controllers'].append({
                'name': 'joint_trajectory_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
                'params': {
                    'joints': ['joint1', 'joint2'],  # Placeholder
                    'command_interfaces': ['position'],
                    'state_interfaces': ['position', 'velocity']
                }
            })

        elif interface_type == HardwareInterfaceType.SENSOR.value:
            # Sensor interface - only state publishing
            controller_config['controllers'].append({
                'name': 'sensor_broadcaster',
                'type': 'imu_sensor_broadcaster/IMUSensorBroadcaster',
                'params': {
                    'frame_id': 'imu_link'
                }
            })

        return controller_config

    def generate_ros2_control_params(self, plugins: List[Dict]) -> str:
        """Generate ros2_control parameter YAML from discovered plugins"""
        if not plugins:
            return ""

        yaml_content = "controller_manager:\n  ros__parameters:\n    update_rate: 100\n\n"

        for plugin in plugins:
            interface_type = plugin.get('interface_type', '')
            plugin_type = plugin.get('type', '')

            yaml_content += f"    # Hardware interface: {plugin_type}\n"
            yaml_content += f"    # Type: {interface_type}\n"

            if HardwareInterfaceType.SYSTEM.value in interface_type:
                yaml_content += """    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

"""
            elif HardwareInterfaceType.SENSOR.value in interface_type:
                yaml_content += """    imu_sensor_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

"""

        return yaml_content