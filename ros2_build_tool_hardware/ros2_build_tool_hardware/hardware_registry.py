"""
Hardware component registry with validation and discovery
"""

import yaml
import logging
from pathlib import Path
from typing import Dict, List, Optional

from ros2_build_tool_core.models import HardwareComponent


class HardwareRegistry:
    """Hardware component registry with remote sync"""

    def __init__(self, data_dir: Path, logger: logging.Logger):
        self.data_dir = data_dir
        self.logger = logger
        self.registry_file = data_dir / 'hardware' / 'registry.yaml'
        self.registry: Dict[str, HardwareComponent] = {}
        self.load()

    def load(self):
        """Load hardware registry from YAML"""
        if self.registry_file.exists():
            try:
                with open(self.registry_file, 'r') as f:
                    data = yaml.safe_load(f)

                    if data:
                        for hw_id, hw_data in data.items():
                            self.registry[hw_id] = HardwareComponent(**hw_data)

                self.logger.info(f"Loaded {len(self.registry)} hardware components from registry")
            except Exception as e:
                self.logger.error(f"Failed to load hardware registry: {e}")
                self._load_defaults()
        else:
            self.logger.info("No hardware registry found, loading defaults")
            self._load_defaults()

    def _load_defaults(self):
        """Load default hardware components"""
        defaults = {
            'lidar_rplidar': {
                'name': 'RPLidar A1/A2',
                'type': 'lidar',
                'repo': {
                    'url': 'https://github.com/Slamtec/rplidar_ros.git',
                    'branches': {'humble': 'ros2', 'jazzy': 'ros2'}
                },
                'rosdeps': ['sensor_msgs', 'std_srvs'],
                'system_deps': [],
                'frames': {'lidar': 'laser_frame'}
            },
            'lidar_ouster': {
                'name': 'Ouster OS1/OS2',
                'type': 'lidar',
                'repo': {
                    'url': 'https://github.com/ros-drivers/ros2_ouster_drivers.git',
                    'branches': {'humble': 'humble', 'jazzy': 'jazzy'}
                },
                'rosdeps': ['diagnostic_updater', 'pcl_conversions'],
                'system_deps': ['libpcap-dev', 'libeigen3-dev'],
                'frames': {'lidar': 'os_sensor'}
            },
            'lidar_velodyne': {
                'name': 'Velodyne VLP-16',
                'type': 'lidar',
                'repo': {
                    'url': 'https://github.com/ros-drivers/velodyne.git',
                    'branches': {'humble': 'ros2', 'jazzy': 'ros2'}
                },
                'rosdeps': ['pcl_conversions', 'diagnostic_updater'],
                'system_deps': ['libpcap-dev'],
                'frames': {'lidar': 'velodyne'}
            },
            'camera_v4l2': {
                'name': 'V4L2 Camera',
                'type': 'camera',
                'repo': {
                    'url': 'https://gitlab.com/boldhearts/ros2_v4l2_camera.git',
                    'branches': {'humble': 'humble', 'jazzy': 'rolling'}
                },
                'rosdeps': ['image_transport', 'camera_info_manager'],
                'system_deps': ['v4l-utils', 'libv4l-dev'],
                'frames': {'camera': 'camera_link'}
            },
            'camera_realsense': {
                'name': 'Intel RealSense',
                'type': 'camera',
                'repo': {
                    'url': 'https://github.com/IntelRealSense/realsense-ros.git',
                    'branches': {'humble': 'ros2-development', 'jazzy': 'ros2-development'}
                },
                'rosdeps': ['diagnostic_updater', 'image_transport'],
                'system_deps': ['librealsense2-dev'],
                'frames': {'camera': 'camera_link', 'depth': 'camera_depth_frame'}
            },
            'imu_xsens': {
                'name': 'Xsens MTi',
                'type': 'imu',
                'repo': {
                    'url': 'https://github.com/xsens/xsens_ros.git',
                    'branches': {'humble': 'humble', 'jazzy': 'jazzy'}
                },
                'rosdeps': ['sensor_msgs', 'geometry_msgs'],
                'system_deps': [],
                'frames': {'imu': 'imu_link'}
            },
            'imu_bno055': {
                'name': 'Bosch BNO055',
                'type': 'imu',
                'repo': {
                    'url': 'https://github.com/flynneva/bno055.git',
                    'branches': {'humble': 'humble', 'jazzy': 'main'}
                },
                'rosdeps': ['sensor_msgs', 'diagnostic_updater'],
                'system_deps': [],
                'frames': {'imu': 'imu_link'}
            },
            'gps_ublox': {
                'name': 'u-blox GPS',
                'type': 'gps',
                'repo': {
                    'url': 'https://github.com/KumarRobotics/ublox.git',
                    'branches': {'humble': 'ros2', 'jazzy': 'ros2'}
                },
                'rosdeps': ['sensor_msgs', 'diagnostic_updater'],
                'system_deps': [],
                'frames': {'gps': 'gps_link'}
            },
            'wheel_encoders': {
                'name': 'Wheel Encoders',
                'type': 'odometry',
                'repo': None,
                'rosdeps': ['nav_msgs', 'geometry_msgs'],
                'system_deps': [],
                'frames': {'odom': 'odom', 'base': 'base_link'}
            },
            'generic_diff_drive': {
                'name': 'Generic Differential Drive',
                'type': 'mobile_base',
                'repo': None,
                'rosdeps': ['nav_msgs', 'geometry_msgs', 'ros2_control'],
                'system_deps': [],
                'frames': {'base': 'base_link', 'odom': 'odom'},
                'plugin_type': 'hardware_interface::SystemInterface'
            }
        }

        for hw_id, hw_data in defaults.items():
            self.registry[hw_id] = HardwareComponent(**hw_data)

        self.logger.info(f"Loaded {len(self.registry)} default hardware components")

    def save(self):
        """Save hardware registry to YAML"""
        try:
            self.registry_file.parent.mkdir(parents=True, exist_ok=True)

            data = {}
            for hw_id, component in self.registry.items():
                data[hw_id] = component.model_dump(mode='json')

            with open(self.registry_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.logger.info(f"Saved hardware registry to {self.registry_file}")

        except Exception as e:
            self.logger.error(f"Failed to save hardware registry: {e}")

    def get(self, hw_id: str) -> Optional[HardwareComponent]:
        """Get hardware component by ID"""
        return self.registry.get(hw_id)

    def add_custom(self, hw_id: str, hw_data: Dict):
        """Add custom hardware component with validation"""
        try:
            # Validate with Pydantic model
            component = HardwareComponent(**hw_data)
            self.registry[hw_id] = component
            self.logger.info(f"Added custom hardware component: {hw_id}")
            return component
        except Exception as e:
            self.logger.error(f"Failed to add custom component: {e}")
            raise

    def list_available(self) -> List[str]:
        """List available hardware IDs"""
        return list(self.registry.keys())

    def list_by_type(self, hw_type: str) -> List[str]:
        """List hardware IDs filtered by type"""
        return [
            hw_id for hw_id, component in self.registry.items()
            if component.type == hw_type
        ]

    def search(self, query: str) -> List[str]:
        """Search hardware components by name or type"""
        query_lower = query.lower()
        results = []

        for hw_id, component in self.registry.items():
            if query_lower in hw_id.lower() or \
               query_lower in component.name.lower() or \
               query_lower in component.type.lower():
                results.append(hw_id)

        return results

    def validate_compatibility(self, hw_ids: List[str], ros_distro: str) -> Dict[str, bool]:
        """Validate hardware components are compatible with ROS distro"""
        compatibility = {}

        for hw_id in hw_ids:
            component = self.get(hw_id)
            if not component:
                compatibility[hw_id] = False
                continue

            # Check if repo has branch for this distro
            if component.repo and 'branches' in component.repo:
                branches = component.repo['branches']
                compatibility[hw_id] = ros_distro in branches
            else:
                # No repo or no branch info - assume compatible
                compatibility[hw_id] = True

        return compatibility