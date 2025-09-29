"""
Enhanced URDF parsing with xacro support, proper 3D transforms, and ros2_control generation
"""

import logging
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple, Optional, Dict
import math
import numpy as np
import tempfile

try:
    from urdf_parser_py.urdf import URDF
    URDF_AVAILABLE = True
except ImportError:
    URDF_AVAILABLE = False

from ros2_build_tool_core.models import RobotSpecs, SensorFrame


class URDFParser:
    """Enhanced URDF parsing with xacro support, proper 3D transforms, and validation"""

    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.robot = None
        self.urdf_tree = None  # ET tree for ros2_control extraction
        self.temp_files: List[Path] = []  # Track temporary files for cleanup

    def __del__(self):
        """Cleanup temporary files when object is destroyed"""
        self.cleanup()

    def cleanup(self) -> None:
        """Remove all temporary files created during parsing"""
        for temp_file in self.temp_files:
            try:
                if temp_file.exists():
                    temp_file.unlink()
                    self.logger.debug(f'Cleaned up temporary file: {temp_file}')
            except Exception as e:
                self.logger.warning(f'Failed to cleanup temporary file {temp_file}: {e}')
        self.temp_files.clear()

    @staticmethod
    def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Convert roll-pitch-yaw angles to 3x3 rotation matrix
        Uses ZYX euler angle convention (yaw-pitch-roll) as per URDF/ROS standards

        Args:
            roll: Rotation around X-axis (radians)
            pitch: Rotation around Y-axis (radians)
            yaw: Rotation around Z-axis (radians)

        Returns:
            3x3 rotation matrix as numpy array
        """
        # Rotation around X-axis (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        # Rotation around Y-axis (pitch)
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        # Rotation around Z-axis (yaw)
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Combined rotation: R = Rz * Ry * Rx (ZYX convention)
        return Rz @ Ry @ Rx

    @staticmethod
    def transform_point(point: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        """
        Apply 3D transformation (rotation + translation) to a point

        Args:
            point: 3D point as numpy array [x, y, z]
            rotation: 3x3 rotation matrix
            translation: 3D translation vector [x, y, z]

        Returns:
            Transformed 3D point
        """
        return rotation @ point + translation

    def parse(self, urdf_path: Path, xacro_args: Optional[Dict[str, str]] = None) -> Tuple[RobotSpecs, List[SensorFrame], List[str]]:
        """
        Parse URDF file (with optional xacro processing) and extract specifications
        Returns: (robot_specs, sensor_frames, validation_warnings)
        """
        if not URDF_AVAILABLE:
            raise RuntimeError("urdf_parser_py not available. Install with: sudo apt install ros-<distro>-urdfdom-py")

        warnings = []

        # Handle xacro files
        if urdf_path.suffix == '.xacro':
            self.logger.info(f"Processing xacro file: {urdf_path}")
            urdf_path = self._process_xacro(urdf_path, xacro_args)
            warnings.append("Processed xacro file - temporary URDF generated")

        try:
            self.logger.info(f"Parsing URDF: {urdf_path}")
            self.robot = URDF.from_xml_file(str(urdf_path))

            # Also parse with ElementTree for ros2_control tags
            self.urdf_tree = ET.parse(str(urdf_path))
        except Exception as e:
            raise RuntimeError(f"Failed to parse URDF: {e}")

        # Extract robot specifications with improved bounding box calculation
        robot_specs = self._extract_robot_specs()

        # Extract sensor frames
        sensor_frames = self._extract_sensor_frames()

        # Validate REP-105 compliance
        rep105_warnings = self._validate_rep105_frames()
        warnings.extend(rep105_warnings)

        # Validate transform tree completeness
        tf_warnings = self._validate_transform_tree()
        warnings.extend(tf_warnings)

        self.logger.info(f"Extracted specs: width={robot_specs.width:.2f}m, length={robot_specs.length:.2f}m, height={robot_specs.height:.2f}m")
        self.logger.info(f"Found {len(sensor_frames)} sensor frames")

        return robot_specs, sensor_frames, warnings

    def _process_xacro(self, xacro_path: Path, xacro_args: Optional[Dict[str, str]] = None) -> Path:
        """Process xacro file to generate URDF (creates temporary file)"""
        # Create a temporary file that will be cleaned up
        temp_fd, temp_path = tempfile.mkstemp(suffix='.urdf', prefix='ros2_build_tool_')
        output_path = Path(temp_path)
        self.temp_files.append(output_path)  # Track for cleanup

        cmd = ['xacro', str(xacro_path)]

        # Add xacro arguments
        if xacro_args:
            for key, value in xacro_args.items():
                cmd.append(f"{key}:={value}")

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True,
                timeout=30  # Add timeout to prevent hanging
            )

            with open(output_path, 'w') as f:
                f.write(result.stdout)

            self.logger.info(f"Generated URDF from xacro: {output_path}")
            return output_path

        except subprocess.TimeoutExpired:
            raise RuntimeError(
                f"xacro processing timed out after 30 seconds. "
                f"Check for infinite loops or complex macros in {xacro_path}"
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"Failed to process xacro: {e.stderr}\n"
                f"Command: {' '.join(cmd)}\n"
                f"Suggestion: Check xacro syntax and included files"
            )
        except FileNotFoundError:
            raise RuntimeError(
                "xacro command not found. Install with: sudo apt install ros-<distro>-xacro"
            )

    def _extract_robot_specs(self) -> RobotSpecs:
        """Extract physical robot specifications with improved geometry calculation"""
        # Build transform tree to account for joint offsets
        link_positions = self._compute_link_positions()

        # Compute actual bounding box accounting for transforms
        min_x, max_x = 0.0, 0.0
        min_y, max_y = 0.0, 0.0
        min_z, max_z = 0.0, 0.0

        for link in self.robot.links:
            link_pos = link_positions.get(link.name, [0, 0, 0])

            if link.collision and link.collision.geometry:
                geom = link.collision.geometry

                if hasattr(geom, 'size'):  # Box
                    max_x = max(max_x, link_pos[0] + geom.size[0] / 2)
                    min_x = min(min_x, link_pos[0] - geom.size[0] / 2)
                    max_y = max(max_y, link_pos[1] + geom.size[1] / 2)
                    min_y = min(min_y, link_pos[1] - geom.size[1] / 2)
                    max_z = max(max_z, link_pos[2] + geom.size[2] / 2)
                    min_z = min(min_z, link_pos[2] - geom.size[2] / 2)

                elif hasattr(geom, 'radius'):  # Cylinder/Sphere
                    max_x = max(max_x, link_pos[0] + geom.radius)
                    min_x = min(min_x, link_pos[0] - geom.radius)
                    max_y = max(max_y, link_pos[1] + geom.radius)
                    min_y = min(min_y, link_pos[1] - geom.radius)

                    if hasattr(geom, 'length'):  # Cylinder
                        max_z = max(max_z, link_pos[2] + geom.length / 2)
                        min_z = min(min_z, link_pos[2] - geom.length / 2)
                    else:  # Sphere
                        max_z = max(max_z, link_pos[2] + geom.radius)
                        min_z = min(min_z, link_pos[2] - geom.radius)

        # Compute dimensions
        width = max(max_y - min_y, 0.5)
        length = max(max_x - min_x, 0.5)
        height = max(max_z - min_z, 0.3)

        # Extract wheel parameters
        wheel_radius, wheel_separation = self._extract_wheel_params()

        # Extract velocity limits
        max_linear_vel, max_angular_vel = self._extract_velocity_limits(wheel_radius)

        return RobotSpecs(
            width=width,
            length=length,
            height=height,
            wheel_radius=wheel_radius,
            wheel_separation=wheel_separation,
            max_linear_velocity=max_linear_vel,
            max_angular_velocity=max_angular_vel
        )

    def _compute_link_positions(self) -> Dict[str, List[float]]:
        """
        Compute link positions and orientations in base_link frame using proper 3D transforms

        Returns:
            Dictionary mapping link names to [x, y, z] positions in base_link frame
        """
        # Store full transforms (position + rotation matrix) for each link
        transforms = {}

        # Find base_link
        base_link = 'base_link'
        for link in self.robot.links:
            if link.name in ['base_link', 'base_footprint']:
                base_link = link.name
                break

        # Base link has identity transform
        transforms[base_link] = {
            'position': np.array([0.0, 0.0, 0.0]),
            'rotation': np.eye(3)  # Identity rotation matrix
        }

        # Build map of child to parent relationships
        joint_map = {joint.child: joint for joint in self.robot.joints}

        # Recursively compute transforms for all links
        def compute_transform(link_name: str):
            """Recursively compute transform for a link"""
            if link_name in transforms:
                return transforms[link_name]

            # Find parent
            if link_name not in joint_map:
                # Orphaned link, use identity
                transforms[link_name] = {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'rotation': np.eye(3)
                }
                return transforms[link_name]

            joint = joint_map[link_name]

            # Recursively get parent transform
            parent_transform = compute_transform(joint.parent)

            # Get joint origin transform
            xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
            rpy = joint.origin.rpy if joint.origin else [0, 0, 0]

            # Convert to numpy arrays
            joint_translation = np.array(xyz)
            joint_rotation = self.rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2])

            # Compose transforms: T_child = T_parent * T_joint
            # Position: p_child = R_parent * p_joint + p_parent
            child_position = parent_transform['rotation'] @ joint_translation + parent_transform['position']

            # Rotation: R_child = R_parent * R_joint
            child_rotation = parent_transform['rotation'] @ joint_rotation

            transforms[link_name] = {
                'position': child_position,
                'rotation': child_rotation
            }

            return transforms[link_name]

        # Compute transforms for all links
        for link in self.robot.links:
            compute_transform(link.name)

        # Convert to simple position dict for backward compatibility
        positions = {name: transform['position'].tolist() for name, transform in transforms.items()}

        return positions

    def _extract_wheel_params(self) -> Tuple[float, float]:
        """Extract wheel radius and separation with improved detection"""
        wheel_radius = 0.1  # Default
        wheel_joints = []

        for joint in self.robot.joints:
            joint_name_lower = joint.name.lower()
            if any(kw in joint_name_lower for kw in ['wheel', 'drive']) and \
               joint.type in ['continuous', 'revolute']:
                wheel_joints.append(joint)

                # Get wheel radius from child link geometry
                for link in self.robot.links:
                    if link.name == joint.child:
                        if link.collision and link.collision.geometry:
                            geom = link.collision.geometry
                            if hasattr(geom, 'radius'):
                                wheel_radius = geom.radius
                                break

        # Compute wheel separation from joint positions
        wheel_separation = 0.3  # Default
        if len(wheel_joints) >= 2:
            left_y, right_y = None, None

            for joint in wheel_joints:
                joint_name_lower = joint.name.lower()
                y_pos = joint.origin.xyz[1] if joint.origin else 0.0

                if any(kw in joint_name_lower for kw in ['left', 'l_']):
                    left_y = y_pos
                elif any(kw in joint_name_lower for kw in ['right', 'r_']):
                    right_y = y_pos

            if left_y is not None and right_y is not None:
                wheel_separation = abs(left_y - right_y)
            elif len(wheel_joints) == 2:
                # If naming convention not followed, use first two wheels
                wheel_separation = abs(
                    (wheel_joints[0].origin.xyz[1] if wheel_joints[0].origin else 0.0) -
                    (wheel_joints[1].origin.xyz[1] if wheel_joints[1].origin else 0.0)
                )

        return wheel_radius, wheel_separation

    def _extract_velocity_limits(self, wheel_radius: float) -> Tuple[float, float]:
        """Extract velocity limits from joint definitions"""
        max_wheel_angular_vel = 10.0  # Default rad/s

        for joint in self.robot.joints:
            if joint.limit and hasattr(joint.limit, 'velocity'):
                joint_name_lower = joint.name.lower()
                if any(kw in joint_name_lower for kw in ['wheel', 'drive']):
                    max_wheel_angular_vel = max(max_wheel_angular_vel, joint.limit.velocity)

        # Convert wheel angular velocity to robot linear velocity
        max_linear_vel = wheel_radius * max_wheel_angular_vel

        # Angular velocity for differential drive: v_angular = v_linear / wheel_separation
        # Conservative estimate: max 2 rad/s for safety
        max_angular_vel = min(max_linear_vel * 2.0, 2.0)

        return max_linear_vel, max_angular_vel

    def _extract_sensor_frames(self) -> List[SensorFrame]:
        """Extract sensor mounting frames from URDF"""
        sensor_frames = []
        sensor_keywords = {
            'lidar': ['lidar', 'laser', 'scan', 'hokuyo', 'sick', 'rplidar', 'urg'],
            'camera': ['camera', 'rgb', 'depth', 'realsense', 'kinect', 'zed'],
            'imu': ['imu', 'accel', 'gyro', 'xsens', 'bno', 'mpu'],
            'gps': ['gps', 'gnss', 'navsat', 'ublox']
        }

        for joint in self.robot.joints:
            if joint.type != 'fixed':
                continue

            child_lower = joint.child.lower()
            sensor_type = None

            for stype, keywords in sensor_keywords.items():
                if any(kw in child_lower for kw in keywords):
                    sensor_type = stype
                    break

            if sensor_type:
                xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
                rpy = joint.origin.rpy if joint.origin else [0, 0, 0]

                sensor_frames.append(SensorFrame(
                    name=joint.child,
                    parent=joint.parent,
                    child=joint.child,
                    xyz=list(xyz),
                    rpy=list(rpy),
                    sensor_type=sensor_type
                ))

        return sensor_frames

    def _validate_rep105_frames(self) -> List[str]:
        """Validate REP-105 frame conventions"""
        warnings = []
        required_frames = {
            'base_link': 'Primary robot-fixed frame',
            'base_footprint': 'Ground projection of base_link'
        }
        link_names = {link.name for link in self.robot.links}

        for frame, description in required_frames.items():
            if frame not in link_names:
                warnings.append(f"Missing REP-105 frame: {frame} ({description})")

        return warnings

    def _validate_transform_tree(self) -> List[str]:
        """Validate that all links form a connected transform tree"""
        warnings = []

        # Build adjacency list
        parent_map = {}
        for joint in self.robot.joints:
            parent_map[joint.child] = joint.parent

        # Find root (link with no parent)
        all_links = {link.name for link in self.robot.links}
        children = set(parent_map.keys())
        roots = all_links - children

        if len(roots) == 0:
            warnings.append("No root link found - circular dependency in transform tree")
            return warnings

        if len(roots) > 1:
            warnings.append(f"Multiple root links found: {roots}. Expected single root.")

        # Check all links are reachable from root
        root = roots.pop()
        reachable = {root}
        to_visit = [child for child, parent in parent_map.items() if parent == root]

        while to_visit:
            current = to_visit.pop()
            if current in reachable:
                continue
            reachable.add(current)

            # Add children of current
            to_visit.extend([child for child, parent in parent_map.items() if parent == current])

        unreachable = all_links - reachable
        if unreachable:
            warnings.append(f"Disconnected links in transform tree: {unreachable}")

        return warnings

    def extract_ros2_control_config(self) -> Optional[Dict]:
        """Extract existing ros2_control configuration from URDF"""
        if self.urdf_tree is None:
            return None

        root = self.urdf_tree.getroot()
        ros2_control_tags = root.findall('ros2_control')

        if not ros2_control_tags:
            return None

        configs = []
        for tag in ros2_control_tags:
            config = {
                'name': tag.get('name', 'robot_control'),
                'type': tag.get('type', 'system'),
                'hardware': [],
                'joints': []
            }

            # Extract hardware plugin
            hardware_tag = tag.find('hardware')
            if hardware_tag is not None:
                plugin_tag = hardware_tag.find('plugin')
                if plugin_tag is not None:
                    config['hardware'].append({
                        'plugin': plugin_tag.text,
                        'params': {param.get('name'): param.text for param in hardware_tag.findall('param')}
                    })

            # Extract joint configurations
            for joint_tag in tag.findall('joint'):
                joint_config = {
                    'name': joint_tag.get('name'),
                    'command_interfaces': [],
                    'state_interfaces': []
                }

                for cmd_if in joint_tag.findall('command_interface'):
                    joint_config['command_interfaces'].append(cmd_if.get('name'))

                for state_if in joint_tag.findall('state_interface'):
                    joint_config['state_interfaces'].append(state_if.get('name'))

                config['joints'].append(joint_config)

            configs.append(config)

        return configs if configs else None


class Ros2ControlGenerator:
    """Generate ros2_control URDF blocks for hardware interfaces"""

    def __init__(self, robot_specs: RobotSpecs, logger: logging.Logger):
        self.robot_specs = robot_specs
        self.logger = logger

    def generate_differential_drive_control(
        self,
        hardware_plugin: str,
        left_wheel_joint: str = "left_wheel_joint",
        right_wheel_joint: str = "right_wheel_joint",
        plugin_params: Optional[Dict[str, str]] = None
    ) -> str:
        """Generate ros2_control block for differential drive robot"""

        params_xml = ""
        if plugin_params:
            for key, value in plugin_params.items():
                params_xml += f'      <param name="{key}">{value}</param>\n'

        control_xml = f'''  <ros2_control name="differential_drive_controller" type="system">
    <hardware>
      <plugin>{hardware_plugin}</plugin>
{params_xml}    </hardware>

    <joint name="{left_wheel_joint}">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="{right_wheel_joint}">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>'''

        return control_xml

    def generate_sensor_interfaces(self, sensor_frames: List[SensorFrame]) -> str:
        """Generate ros2_control sensor interfaces"""

        sensor_xml_parts = []

        for sensor in sensor_frames:
            if sensor.sensor_type == 'imu':
                sensor_xml_parts.append(f'''  <ros2_control name="{sensor.name}_sensor_interface" type="sensor">
    <hardware>
      <plugin>ros2_control_sensor_interfaces/IMUSensor</plugin>
    </hardware>

    <sensor name="{sensor.name}">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
    </sensor>
  </ros2_control>''')

        return '\n\n'.join(sensor_xml_parts)