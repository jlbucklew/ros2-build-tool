"""URDF analyzer for robot model parsing and validation.

This module provides functionality to parse URDF and Xacro files,
extract frames, compute transforms, and validate TF trees.
"""

import subprocess  # nosec B404 - needed for xacro processing
import xml.etree.ElementTree as ET  # nosec B405 - URDF files are trusted
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np


@dataclass
class Transform:
    """3D transform with translation and rotation."""

    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float]  # Roll, Pitch, Yaw

    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 transformation matrix.

        Returns:
            4x4 homogeneous transformation matrix
        """
        # Translation
        x, y, z = self.translation

        # Rotation (RPY to rotation matrix)
        roll, pitch, yaw = self.rotation

        # Compute rotation matrix
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        # Build 4x4 transformation matrix
        matrix = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
                [-sp, cp * sr, cp * cr, z],
                [0, 0, 0, 1],
            ]
        )

        return matrix


@dataclass
class Frame:
    """TF frame information."""

    name: str
    parent: Optional[str] = None
    transform: Optional[Transform] = None


@dataclass
class JointLimits:
    """Joint limits."""

    lower: float
    upper: float


@dataclass
class Joint:
    """Joint information."""

    name: str
    type: str
    parent: str
    child: str
    origin: Optional[Transform] = None
    axis: Optional[Tuple[float, float, float]] = None
    limits: Optional[JointLimits] = None


@dataclass
class Link:
    """Link information."""

    name: str
    visual: Optional[Dict[str, Any]] = None
    collision: Optional[Dict[str, Any]] = None
    inertial: Optional[Dict[str, Any]] = None


@dataclass
class Dimensions:
    """Robot dimensions."""

    length: float
    width: float
    height: float


@dataclass
class ValidationResult:
    """TF tree validation result."""

    is_valid: bool
    has_single_root: bool
    root_frame: Optional[str]
    has_cycles: bool
    disconnected_frames: List[str]


class URDFAnalyzer:
    """Analyzer for URDF robot models."""

    def __init__(self, robot_name: str):
        """Initialize URDF analyzer.

        Args:
            robot_name: Name of the robot
        """
        self._tree: Optional[ET.Element] = None
        self.robot_name = robot_name
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self._tree = None

    @classmethod
    def from_string(cls, urdf_content: str) -> "URDFAnalyzer":
        """Parse URDF from string content.

        Args:
            urdf_content: URDF XML content as string

        Returns:
            URDFAnalyzer instance
        """
        tree = ET.fromstring(urdf_content)  # nosec B314 - URDF content from trusted sources
        robot_name = tree.get("name", "robot")

        analyzer = cls(robot_name)
        analyzer._tree = tree
        analyzer._parse_urdf(tree)

        return analyzer

    @classmethod
    def from_file(cls, file_path: Path) -> "URDFAnalyzer":
        """Parse URDF from file.

        Args:
            file_path: Path to URDF file

        Returns:
            URDFAnalyzer instance

        Raises:
            FileNotFoundError: If file does not exist
            PermissionError: If file cannot be read
            IOError: If reading fails
        """
        try:
            with open(file_path, "r") as f:
                content = f.read()
        except FileNotFoundError:
            raise FileNotFoundError(f"URDF file not found: {file_path}")
        except PermissionError:
            raise PermissionError(f"Permission denied reading file: {file_path}")
        except Exception as e:
            raise IOError(f"Error reading URDF file {file_path}: {e}")

        return cls.from_string(content)

    @classmethod
    def from_xacro(cls, xacro_file: Union[Path, str]) -> "URDFAnalyzer":
        """Parse xacro file by converting to URDF first.

        Args:
            xacro_file: Path to xacro file

        Returns:
            URDFAnalyzer instance

        Raises:
            FileNotFoundError: If xacro file does not exist
            ValueError: If xacro file path is invalid
            subprocess.TimeoutExpired: If xacro processing times out
        """
        # Validate file path to prevent command injection
        if not isinstance(xacro_file, Path):
            xacro_path = Path(xacro_file)
        else:
            xacro_path = xacro_file

        # Ensure file exists and is a regular file
        if not xacro_path.exists():
            raise FileNotFoundError(f"Xacro file not found: {xacro_path}")

        if not xacro_path.is_file():
            raise ValueError(f"Path is not a file: {xacro_path}")

        # Resolve to absolute path to prevent path traversal attacks
        xacro_path = xacro_path.resolve()

        # Process xacro file using xacro command
        try:
            # Try to use ROS2 xacro command with timeout
            # Validated file path, trusted xacro tool
            result = subprocess.run(  # nosec B603, B607
                ["xacro", str(xacro_path)],
                capture_output=True,
                text=True,
                check=True,
                timeout=30,  # 30 second timeout to prevent hanging
            )
            urdf_content = result.stdout
        except subprocess.TimeoutExpired:
            raise subprocess.TimeoutExpired(
                cmd=["xacro", str(xacro_file)],
                timeout=30,
                output="Xacro processing timed out after 30 seconds",
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            # Fallback to simple XML parsing if xacro not available
            # NOTE: This fallback does NOT support xacro properties/macros properly
            # and should only be used for testing purposes
            try:
                with open(xacro_file, "r") as f:
                    content = f.read()
            except Exception as e:
                raise IOError(f"Error reading xacro file {xacro_file}: {e}")

            # Remove xacro namespace and properties for simple parsing
            content = content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', "")
            # Remove xacro:property tags
            import re

            content = re.sub(r"<xacro:property[^>]*>", "", content)
            content = re.sub(r"</xacro:property>", "", content)

            # Check for unresolved xacro properties and raise error
            if "${" in content:
                raise ValueError(
                    f"Xacro file contains unresolved properties (e.g., ${'{...}'}). "
                    f"Please install xacro tool or provide a fully resolved URDF file."
                )

            urdf_content = content

        return cls.from_string(urdf_content)

    def _parse_urdf(self, tree: ET.Element) -> None:
        """Parse URDF tree into links and joints.

        Args:
            tree: XML ElementTree root
        """
        # Parse links
        for link_elem in tree.findall("link"):
            link_name = link_elem.get("name")
            if not link_name:
                continue
            link = Link(name=link_name)

            # Parse visual if present
            visual = link_elem.find("visual")
            if visual is not None:
                link.visual = self._parse_visual(visual)

            self.links[link_name] = link

        # Parse joints
        for joint_elem in tree.findall("joint"):
            joint_name = joint_elem.get("name")
            if not joint_name:
                continue
            joint_type = joint_elem.get("type", "fixed")

            parent_elem = joint_elem.find("parent")
            child_elem = joint_elem.find("child")

            if parent_elem is None or child_elem is None:
                continue

            parent_link = parent_elem.get("link")
            child_link = child_elem.get("link")
            if not parent_link or not child_link:
                continue

            joint = Joint(
                name=joint_name,
                type=joint_type,
                parent=parent_link,
                child=child_link,
            )

            # Parse origin
            origin_elem = joint_elem.find("origin")
            if origin_elem is not None:
                xyz = origin_elem.get("xyz", "0 0 0").split()
                rpy = origin_elem.get("rpy", "0 0 0").split()
                joint.origin = Transform(
                    translation=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                    rotation=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
                )

            # Parse axis
            axis_elem = joint_elem.find("axis")
            if axis_elem is not None:
                xyz = axis_elem.get("xyz", "0 0 1").split()
                joint.axis = (float(xyz[0]), float(xyz[1]), float(xyz[2]))

            # Parse limits
            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                lower = float(limit_elem.get("lower", "0"))
                upper = float(limit_elem.get("upper", "0"))
                joint.limits = JointLimits(lower=lower, upper=upper)

            self.joints[joint_name] = joint

    def _parse_visual(self, visual_elem: ET.Element) -> Dict[str, Any]:
        """Parse visual element.

        Args:
            visual_elem: Visual XML element

        Returns:
            Dictionary with visual properties
        """
        visual_data = {}

        geometry = visual_elem.find("geometry")
        if geometry is not None:
            box = geometry.find("box")
            if box is not None:
                size_str = box.get("size", "0 0 0")
                sizes = [float(s) for s in size_str.split()]
                visual_data["box_size"] = sizes

        return visual_data

    def get_frames(self) -> List[Frame]:
        """Extract all frames from URDF.

        Returns:
            List of frames
        """
        frames = []

        # Add all links as frames
        for link_name in self.links:
            parent = None
            transform = None

            # Find parent through joints
            for joint in self.joints.values():
                if joint.child == link_name:
                    parent = joint.parent
                    transform = joint.origin
                    break

            frame = Frame(name=link_name, parent=parent, transform=transform)
            frames.append(frame)

        return frames

    def get_sensor_frames(self) -> List[Frame]:
        """Extract sensor frames (frames that likely contain sensors).

        Returns:
            List of sensor frames
        """
        sensor_keywords = ["lidar", "laser", "camera", "imu", "gps", "sensor"]
        frames = self.get_frames()

        sensor_frames = []
        for frame in frames:
            # Check if frame name contains sensor keywords
            frame_name_lower = frame.name.lower()
            if any(keyword in frame_name_lower for keyword in sensor_keywords):
                sensor_frames.append(frame)

        return sensor_frames

    def get_transform(self, from_frame: str, to_frame: str) -> Optional[Transform]:
        """Get transform between two frames.

        Computes the full transform chain using graph traversal.

        Args:
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            Transform if path exists, None otherwise
        """
        if from_frame == to_frame:
            return Transform(translation=(0, 0, 0), rotation=(0, 0, 0))

        # Check if frames exist
        if from_frame not in self.links or to_frame not in self.links:
            return None

        # Simple case: direct parent-child relationship
        for joint in self.joints.values():
            if joint.parent == from_frame and joint.child == to_frame:
                return joint.origin or Transform(translation=(0, 0, 0), rotation=(0, 0, 0))

        # Complex case: compute transform chain using BFS
        path = self._find_frame_path(from_frame, to_frame)

        if path is None:
            return None

        # Compute composed transform along the path
        return self._compose_transforms_along_path(path)

    def _find_frame_path(self, from_frame: str, to_frame: str) -> Optional[List[str]]:
        """Find path between two frames using BFS.

        Args:
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            List of frame names representing the path, or None if no path exists
        """
        from collections import deque

        # Build bidirectional graph
        graph: Dict[str, List[Tuple[str, bool]]] = {}
        for joint in self.joints.values():
            if joint.parent not in graph:
                graph[joint.parent] = []
            if joint.child not in graph:
                graph[joint.child] = []
            graph[joint.parent].append((joint.child, False))  # False = forward
            graph[joint.child].append((joint.parent, True))  # True = reverse

        # BFS to find path
        queue: deque[Tuple[str, List[str]]] = deque([(from_frame, [from_frame])])
        visited: set[str] = {from_frame}

        while queue:
            current, path = queue.popleft()

            if current == to_frame:
                return path

            for neighbor, _ in graph.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return None

    def _compose_transforms_along_path(self, path: List[str]) -> Transform:
        """Compose transforms along a path of frames.

        Args:
            path: List of frame names from source to target

        Returns:
            Composed transform
        """
        # Start with identity matrix
        result_matrix = np.eye(4)

        # Compose transforms along the path
        for i in range(len(path) - 1):
            parent = path[i]
            child = path[i + 1]

            # Find the joint connecting these frames
            transform = None
            forward = True

            for joint in self.joints.values():
                if joint.parent == parent and joint.child == child:
                    transform = joint.origin or Transform(translation=(0, 0, 0), rotation=(0, 0, 0))
                    forward = True
                    break
                elif joint.parent == child and joint.child == parent:
                    transform = joint.origin or Transform(translation=(0, 0, 0), rotation=(0, 0, 0))
                    forward = False
                    break

            if transform:
                transform_matrix = transform.to_matrix()

                if not forward:
                    # Invert the transform for reverse direction
                    transform_matrix = np.linalg.inv(transform_matrix)

                # Compose with cumulative result
                result_matrix = result_matrix @ transform_matrix

        # Extract translation and rotation from result matrix
        translation = tuple(result_matrix[:3, 3])

        # Extract RPY from rotation matrix
        r31 = result_matrix[2, 0]
        pitch = np.arcsin(np.clip(-r31, -1.0, 1.0))

        # Check for gimbal lock
        if np.abs(np.cos(pitch)) > 1e-6:
            roll = np.arctan2(result_matrix[2, 1], result_matrix[2, 2])
            yaw = np.arctan2(result_matrix[1, 0], result_matrix[0, 0])
        else:
            # Gimbal lock case
            roll = 0.0
            yaw = np.arctan2(-result_matrix[0, 1], result_matrix[1, 1])

        rotation = (roll, pitch, yaw)

        return Transform(translation=translation, rotation=rotation)

    def validate_tf_tree(self) -> ValidationResult:
        """Validate TF tree structure.

        Returns:
            Validation result with details
        """
        # Find root frames (links with no parent)
        root_frames = []
        all_frames = set(self.links.keys())
        child_frames = set()

        for joint in self.joints.values():
            child_frames.add(joint.child)

        root_frames = list(all_frames - child_frames)

        # Check for single root
        has_single_root = len(root_frames) == 1
        root_frame = root_frames[0] if has_single_root else None

        # Check for cycles
        has_cycles = self._check_cycles()

        # Check for disconnected frames
        disconnected: List[str] = []
        if has_single_root and root_frame:
            visited: set[str] = set()
            self._dfs_traverse(root_frame, visited)
            disconnected = list(all_frames - visited)

        is_valid = has_single_root and not has_cycles and len(disconnected) == 0

        return ValidationResult(
            is_valid=is_valid,
            has_single_root=has_single_root,
            root_frame=root_frame,
            has_cycles=has_cycles,
            disconnected_frames=disconnected,
        )

    def _check_cycles(self) -> bool:
        """Check if there are cycles in the TF tree.

        Returns:
            True if cycles exist, False otherwise
        """
        # Build adjacency list
        graph: Dict[str, List[str]] = {}
        for joint in self.joints.values():
            if joint.parent not in graph:
                graph[joint.parent] = []
            graph[joint.parent].append(joint.child)

        # DFS to detect cycles
        visited: set[str] = set()
        rec_stack: set[str] = set()

        def has_cycle(node: str) -> bool:
            visited.add(node)
            rec_stack.add(node)

            for neighbor in graph.get(node, []):
                if neighbor not in visited:
                    if has_cycle(neighbor):
                        return True
                elif neighbor in rec_stack:
                    return True

            rec_stack.remove(node)
            return False

        for node in graph:
            if node not in visited:
                if has_cycle(node):
                    return True

        return False

    def _dfs_traverse(self, node: str, visited: set[str]) -> None:
        """Depth-first search traversal.

        Args:
            node: Current node
            visited: Set of visited nodes
        """
        visited.add(node)

        for joint in self.joints.values():
            if joint.parent == node and joint.child not in visited:
                self._dfs_traverse(joint.child, visited)

    def get_robot_dimensions(self) -> Optional[Dimensions]:
        """Extract robot dimensions from URDF.

        Returns:
            Robot dimensions if available
        """
        # Look for base_link or similar
        base_link = self.links.get("base_link")
        if not base_link or not base_link.visual:
            # Try to find any link with visual
            for link in self.links.values():
                if link.visual and "box_size" in link.visual:
                    base_link = link
                    break

        if base_link and base_link.visual and "box_size" in base_link.visual:
            sizes = base_link.visual["box_size"]
            if len(sizes) >= 3:
                return Dimensions(length=sizes[0], width=sizes[1], height=sizes[2])

        return None

    def get_joint(self, joint_name: str) -> Optional[Joint]:
        """Get joint by name.

        Args:
            joint_name: Name of the joint

        Returns:
            Joint object if found, None otherwise
        """
        return self.joints.get(joint_name)
