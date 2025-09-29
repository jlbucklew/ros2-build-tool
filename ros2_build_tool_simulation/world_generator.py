"""
World Generator for ROS2 Build Tool

Generates Gazebo/Ignition world files with progressive complexity for testing
robot configurations in various environments.
"""

import os
import random
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import xml.etree.ElementTree as ET
from xml.dom import minidom

from ros2_build_tool_core.models import RobotProfile, RobotSpecs, SensorFrame


class ObstacleType(Enum):
    """Types of obstacles for world generation."""
    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"
    WALL = "wall"
    HUMAN = "human"
    FURNITURE = "furniture"


@dataclass
class Obstacle:
    """Represents an obstacle in the world."""
    name: str
    obstacle_type: ObstacleType
    position: Tuple[float, float, float]
    size: Tuple[float, float, float]  # width, depth, height for box; radius, height for cylinder
    orientation: Tuple[float, float, float] = (0, 0, 0)  # roll, pitch, yaw
    color: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)  # RGBA
    static: bool = True


@dataclass
class WorldConfig:
    """Configuration for world generation."""
    name: str = "test_world"
    size: Tuple[float, float] = (10.0, 10.0)  # width, depth
    ground_texture: str = "default"
    ambient_light: Tuple[float, float, float, float] = (0.4, 0.4, 0.4, 1.0)
    background: Tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)
    physics_step_size: float = 0.001
    real_time_update_rate: int = 1000
    max_contacts: int = 20
    gravity: Tuple[float, float, float] = (0, 0, -9.8)


class WorldGenerator:
    """Generates Gazebo/Ignition world files for simulation."""

    def __init__(self,
                 robot_profile: RobotProfile,
                 workspace_path: Path,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize world generator.

        Args:
            robot_profile: Robot configuration profile
            workspace_path: Path to ROS2 workspace
            logger: Optional logger instance
        """
        self.robot_profile = robot_profile
        self.workspace_path = Path(workspace_path)
        self.logger = logger or logging.getLogger(__name__)
        self.worlds_dir = self.workspace_path / "simulation" / "worlds"
        self.worlds_dir.mkdir(parents=True, exist_ok=True)

    def generate_empty_world(self, config: Optional[WorldConfig] = None) -> Path:
        """
        Generate an empty world with just ground plane.

        Args:
            config: Optional world configuration

        Returns:
            Path to generated world file
        """
        config = config or WorldConfig(name="empty_world")

        world = self._create_world_base(config)
        world_file = self.worlds_dir / f"{config.name}.world"

        self._save_world(world, world_file)
        self.logger.info(f"Generated empty world: {world_file}")

        return world_file

    def generate_simple_obstacles_world(self,
                                       num_obstacles: int = 5,
                                       config: Optional[WorldConfig] = None) -> Path:
        """
        Generate world with simple geometric obstacles.

        Args:
            num_obstacles: Number of obstacles to place
            config: Optional world configuration

        Returns:
            Path to generated world file
        """
        config = config or WorldConfig(name="simple_obstacles")

        world = self._create_world_base(config)

        # Add random obstacles
        obstacles = self._generate_random_obstacles(num_obstacles, config.size)
        for obstacle in obstacles:
            self._add_obstacle_to_world(world, obstacle)

        world_file = self.worlds_dir / f"{config.name}.world"
        self._save_world(world, world_file)

        self.logger.info(f"Generated world with {num_obstacles} obstacles: {world_file}")

        return world_file

    def generate_navigation_test_world(self, config: Optional[WorldConfig] = None) -> Path:
        """
        Generate world optimized for navigation testing with corridors and rooms.

        Args:
            config: Optional world configuration

        Returns:
            Path to generated world file
        """
        config = config or WorldConfig(name="navigation_test", size=(20.0, 20.0))

        world = self._create_world_base(config)

        # Create walls to form rooms and corridors
        walls = self._create_navigation_layout(config.size)
        for wall in walls:
            self._add_obstacle_to_world(world, wall)

        # Add some furniture-like obstacles
        furniture = self._create_furniture_obstacles()
        for item in furniture:
            self._add_obstacle_to_world(world, item)

        world_file = self.worlds_dir / f"{config.name}.world"
        self._save_world(world, world_file)

        self.logger.info(f"Generated navigation test world: {world_file}")

        return world_file

    def generate_sensor_test_world(self, config: Optional[WorldConfig] = None) -> Path:
        """
        Generate world optimized for sensor testing based on robot's sensors.

        Args:
            config: Optional world configuration

        Returns:
            Path to generated world file
        """
        config = config or WorldConfig(name="sensor_test")

        world = self._create_world_base(config)

        # Add features based on robot's sensors
        if self._has_lidar():
            # Add walls and cylinders for lidar testing
            self._add_lidar_test_features(world, config.size)

        if self._has_camera():
            # Add textured objects and markers for camera testing
            self._add_camera_test_features(world, config.size)

        if self._has_depth_camera():
            # Add objects at various depths
            self._add_depth_test_features(world, config.size)

        world_file = self.worlds_dir / f"{config.name}.world"
        self._save_world(world, world_file)

        self.logger.info(f"Generated sensor test world: {world_file}")

        return world_file

    def generate_custom_world(self,
                            obstacles: List[Obstacle],
                            config: Optional[WorldConfig] = None) -> Path:
        """
        Generate world with custom obstacles.

        Args:
            obstacles: List of obstacles to place
            config: Optional world configuration

        Returns:
            Path to generated world file
        """
        config = config or WorldConfig(name="custom_world")

        world = self._create_world_base(config)

        for obstacle in obstacles:
            self._add_obstacle_to_world(world, obstacle)

        world_file = self.worlds_dir / f"{config.name}.world"
        self._save_world(world, world_file)

        self.logger.info(f"Generated custom world with {len(obstacles)} obstacles: {world_file}")

        return world_file

    def _create_world_base(self, config: WorldConfig) -> ET.Element:
        """Create base world structure."""
        # Create SDF root
        sdf = ET.Element("sdf", version="1.6")
        world = ET.SubElement(sdf, "world", name=config.name)

        # Add physics
        physics = ET.SubElement(world, "physics", type="ode")
        ET.SubElement(physics, "max_step_size").text = str(config.physics_step_size)
        ET.SubElement(physics, "real_time_factor").text = "1"
        ET.SubElement(physics, "real_time_update_rate").text = str(config.real_time_update_rate)

        gravity = ET.SubElement(physics, "gravity")
        gravity.text = f"{config.gravity[0]} {config.gravity[1]} {config.gravity[2]}"

        ode = ET.SubElement(physics, "ode")
        solver = ET.SubElement(ode, "solver")
        ET.SubElement(solver, "type").text = "quick"
        ET.SubElement(solver, "iters").text = "50"
        ET.SubElement(solver, "sor").text = "1.3"

        constraints = ET.SubElement(ode, "constraints")
        ET.SubElement(constraints, "cfm").text = "0"
        ET.SubElement(constraints, "erp").text = "0.2"
        ET.SubElement(constraints, "contact_max_correcting_vel").text = "100"
        ET.SubElement(constraints, "contact_surface_layer").text = "0.001"

        # Add scene
        scene = ET.SubElement(world, "scene")
        ambient = ET.SubElement(scene, "ambient")
        ambient.text = f"{config.ambient_light[0]} {config.ambient_light[1]} {config.ambient_light[2]} {config.ambient_light[3]}"

        background = ET.SubElement(scene, "background")
        background.text = f"{config.background[0]} {config.background[1]} {config.background[2]} {config.background[3]}"

        ET.SubElement(scene, "shadows").text = "1"

        # Add sun light
        self._add_sun(world)

        # Add ground plane
        self._add_ground_plane(world, config.ground_texture)

        return sdf

    def _add_sun(self, world: ET.Element):
        """Add sun light source to world."""
        light = ET.SubElement(world, "light", name="sun", type="directional")
        ET.SubElement(light, "cast_shadows").text = "1"

        pose = ET.SubElement(light, "pose")
        pose.text = "0 0 10 0 0 0"

        diffuse = ET.SubElement(light, "diffuse")
        diffuse.text = "0.8 0.8 0.8 1"

        specular = ET.SubElement(light, "specular")
        specular.text = "0.2 0.2 0.2 1"

        attenuation = ET.SubElement(light, "attenuation")
        ET.SubElement(attenuation, "range").text = "1000"
        ET.SubElement(attenuation, "constant").text = "0.9"
        ET.SubElement(attenuation, "linear").text = "0.01"
        ET.SubElement(attenuation, "quadratic").text = "0.001"

        direction = ET.SubElement(light, "direction")
        direction.text = "0.5 0.1 -0.9"

    def _add_ground_plane(self, world: ET.Element, texture: str = "default"):
        """Add ground plane to world."""
        model = ET.SubElement(world, "model", name="ground_plane")
        ET.SubElement(model, "static").text = "1"

        link = ET.SubElement(model, "link", name="link")

        collision = ET.SubElement(link, "collision", name="collision")
        geometry = ET.SubElement(collision, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "100 100"

        surface = ET.SubElement(collision, "surface")
        friction = ET.SubElement(surface, "friction")
        ode = ET.SubElement(friction, "ode")
        ET.SubElement(ode, "mu").text = "100"
        ET.SubElement(ode, "mu2").text = "50"

        visual = ET.SubElement(link, "visual", name="visual")
        ET.SubElement(visual, "cast_shadows").text = "0"
        geometry = ET.SubElement(visual, "geometry")
        plane = ET.SubElement(geometry, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "100 100"

        material = ET.SubElement(visual, "material")
        script = ET.SubElement(material, "script")
        ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
        ET.SubElement(script, "name").text = "Gazebo/Grey"

    def _add_obstacle_to_world(self, world: ET.Element, obstacle: Obstacle):
        """Add an obstacle to the world."""
        model = ET.SubElement(world.find("."), "model", name=obstacle.name)
        ET.SubElement(model, "static").text = str(obstacle.static).lower()

        pose = ET.SubElement(model, "pose")
        pose.text = f"{obstacle.position[0]} {obstacle.position[1]} {obstacle.position[2]} " \
                   f"{obstacle.orientation[0]} {obstacle.orientation[1]} {obstacle.orientation[2]}"

        link = ET.SubElement(model, "link", name="link")

        # Add collision
        collision = ET.SubElement(link, "collision", name="collision")
        geometry = ET.SubElement(collision, "geometry")

        if obstacle.obstacle_type == ObstacleType.BOX:
            box = ET.SubElement(geometry, "box")
            size = ET.SubElement(box, "size")
            size.text = f"{obstacle.size[0]} {obstacle.size[1]} {obstacle.size[2]}"
        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            cylinder = ET.SubElement(geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = str(obstacle.size[0])
            ET.SubElement(cylinder, "length").text = str(obstacle.size[1])
        elif obstacle.obstacle_type == ObstacleType.SPHERE:
            sphere = ET.SubElement(geometry, "sphere")
            ET.SubElement(sphere, "radius").text = str(obstacle.size[0])

        # Add visual
        visual = ET.SubElement(link, "visual", name="visual")
        geometry = ET.SubElement(visual, "geometry")

        if obstacle.obstacle_type == ObstacleType.BOX:
            box = ET.SubElement(geometry, "box")
            size = ET.SubElement(box, "size")
            size.text = f"{obstacle.size[0]} {obstacle.size[1]} {obstacle.size[2]}"
        elif obstacle.obstacle_type == ObstacleType.CYLINDER:
            cylinder = ET.SubElement(geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = str(obstacle.size[0])
            ET.SubElement(cylinder, "length").text = str(obstacle.size[1])
        elif obstacle.obstacle_type == ObstacleType.SPHERE:
            sphere = ET.SubElement(geometry, "sphere")
            ET.SubElement(sphere, "radius").text = str(obstacle.size[0])

        material = ET.SubElement(visual, "material")
        ambient = ET.SubElement(material, "ambient")
        ambient.text = f"{obstacle.color[0]} {obstacle.color[1]} {obstacle.color[2]} {obstacle.color[3]}"
        diffuse = ET.SubElement(material, "diffuse")
        diffuse.text = f"{obstacle.color[0]} {obstacle.color[1]} {obstacle.color[2]} {obstacle.color[3]}"

    def _generate_random_obstacles(self,
                                  num_obstacles: int,
                                  world_size: Tuple[float, float]) -> List[Obstacle]:
        """Generate random obstacles within world bounds."""
        obstacles = []

        # Keep track of occupied positions to avoid overlaps
        occupied_positions = []

        for i in range(num_obstacles):
            # Random obstacle type
            obstacle_type = random.choice([ObstacleType.BOX, ObstacleType.CYLINDER, ObstacleType.SPHERE])

            # Find non-overlapping position
            attempts = 0
            while attempts < 100:
                x = random.uniform(-world_size[0]/2 + 1, world_size[0]/2 - 1)
                y = random.uniform(-world_size[1]/2 + 1, world_size[1]/2 - 1)

                # Check for overlap
                min_distance = 1.5  # Minimum distance between obstacles
                valid_position = True
                for ox, oy in occupied_positions:
                    if ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5 < min_distance:
                        valid_position = False
                        break

                if valid_position:
                    occupied_positions.append((x, y))
                    break

                attempts += 1

            if attempts >= 100:
                continue  # Skip if couldn't find valid position

            # Random size
            if obstacle_type == ObstacleType.BOX:
                size = (
                    random.uniform(0.3, 1.0),
                    random.uniform(0.3, 1.0),
                    random.uniform(0.5, 2.0)
                )
            elif obstacle_type == ObstacleType.CYLINDER:
                size = (
                    random.uniform(0.2, 0.5),  # radius
                    random.uniform(0.5, 2.0),  # height
                    0
                )
            else:  # SPHERE
                radius = random.uniform(0.2, 0.5)
                size = (radius, radius, radius)

            # Random color
            color = (
                random.random(),
                random.random(),
                random.random(),
                1.0
            )

            obstacle = Obstacle(
                name=f"obstacle_{i}",
                obstacle_type=obstacle_type,
                position=(x, y, size[2]/2 if obstacle_type == ObstacleType.BOX else size[1]/2),
                size=size,
                orientation=(0, 0, random.uniform(0, 3.14159)),
                color=color,
                static=True
            )

            obstacles.append(obstacle)

        return obstacles

    def _create_navigation_layout(self, world_size: Tuple[float, float]) -> List[Obstacle]:
        """Create walls for navigation testing."""
        walls = []
        wall_thickness = 0.2
        wall_height = 2.0

        # Outer walls
        # North wall
        walls.append(Obstacle(
            name="north_wall",
            obstacle_type=ObstacleType.BOX,
            position=(0, world_size[1]/2, wall_height/2),
            size=(world_size[0], wall_thickness, wall_height),
            color=(0.8, 0.8, 0.8, 1.0)
        ))

        # South wall
        walls.append(Obstacle(
            name="south_wall",
            obstacle_type=ObstacleType.BOX,
            position=(0, -world_size[1]/2, wall_height/2),
            size=(world_size[0], wall_thickness, wall_height),
            color=(0.8, 0.8, 0.8, 1.0)
        ))

        # East wall
        walls.append(Obstacle(
            name="east_wall",
            obstacle_type=ObstacleType.BOX,
            position=(world_size[0]/2, 0, wall_height/2),
            size=(wall_thickness, world_size[1], wall_height),
            color=(0.8, 0.8, 0.8, 1.0)
        ))

        # West wall
        walls.append(Obstacle(
            name="west_wall",
            obstacle_type=ObstacleType.BOX,
            position=(-world_size[0]/2, 0, wall_height/2),
            size=(wall_thickness, world_size[1], wall_height),
            color=(0.8, 0.8, 0.8, 1.0)
        ))

        # Interior walls to create rooms
        # Vertical divider with doorway
        walls.append(Obstacle(
            name="divider_north",
            obstacle_type=ObstacleType.BOX,
            position=(0, world_size[1]/4, wall_height/2),
            size=(wall_thickness, world_size[1]/2 - 2, wall_height),
            color=(0.7, 0.7, 0.7, 1.0)
        ))

        walls.append(Obstacle(
            name="divider_south",
            obstacle_type=ObstacleType.BOX,
            position=(0, -world_size[1]/4, wall_height/2),
            size=(wall_thickness, world_size[1]/2 - 2, wall_height),
            color=(0.7, 0.7, 0.7, 1.0)
        ))

        # Horizontal corridor walls
        walls.append(Obstacle(
            name="corridor_north",
            obstacle_type=ObstacleType.BOX,
            position=(world_size[0]/4, 2, wall_height/2),
            size=(world_size[0]/2 - 2, wall_thickness, wall_height),
            color=(0.7, 0.7, 0.7, 1.0)
        ))

        walls.append(Obstacle(
            name="corridor_south",
            obstacle_type=ObstacleType.BOX,
            position=(world_size[0]/4, -2, wall_height/2),
            size=(world_size[0]/2 - 2, wall_thickness, wall_height),
            color=(0.7, 0.7, 0.7, 1.0)
        ))

        return walls

    def _create_furniture_obstacles(self) -> List[Obstacle]:
        """Create furniture-like obstacles."""
        furniture = []

        # Tables
        furniture.append(Obstacle(
            name="table_1",
            obstacle_type=ObstacleType.BOX,
            position=(3, 3, 0.4),
            size=(1.5, 0.8, 0.8),
            color=(0.5, 0.3, 0.1, 1.0)
        ))

        # Chairs (cylinders)
        furniture.append(Obstacle(
            name="chair_1",
            obstacle_type=ObstacleType.CYLINDER,
            position=(2, 3, 0.25),
            size=(0.2, 0.5, 0),
            color=(0.4, 0.2, 0.1, 1.0)
        ))

        furniture.append(Obstacle(
            name="chair_2",
            obstacle_type=ObstacleType.CYLINDER,
            position=(4, 3, 0.25),
            size=(0.2, 0.5, 0),
            color=(0.4, 0.2, 0.1, 1.0)
        ))

        # Cabinets
        furniture.append(Obstacle(
            name="cabinet_1",
            obstacle_type=ObstacleType.BOX,
            position=(-3, -3, 0.75),
            size=(0.6, 0.4, 1.5),
            color=(0.6, 0.6, 0.6, 1.0)
        ))

        return furniture

    def _add_lidar_test_features(self, world: ET.Element, world_size: Tuple[float, float]):
        """Add features for lidar testing."""
        # Add various geometric shapes at different distances
        features = []

        # Corner reflectors (good for lidar)
        for i, pos in enumerate([(3, 3), (-3, 3), (3, -3), (-3, -3)]):
            features.append(Obstacle(
                name=f"corner_reflector_{i}",
                obstacle_type=ObstacleType.BOX,
                position=(pos[0], pos[1], 0.5),
                size=(0.3, 0.3, 1.0),
                color=(0.9, 0.9, 0.9, 1.0)  # High reflectivity
            ))

        # Cylinders at various distances
        for i in range(5):
            angle = i * 72 * 3.14159 / 180
            radius = 2 + i * 0.5
            x = radius * __import__('math').cos(angle)
            y = radius * __import__('math').sin(angle)

            features.append(Obstacle(
                name=f"lidar_cylinder_{i}",
                obstacle_type=ObstacleType.CYLINDER,
                position=(x, y, 0.5),
                size=(0.15, 1.0, 0),
                color=(0.5, 0.5, 0.5, 1.0)
            ))

        for feature in features:
            self._add_obstacle_to_world(world, feature)

    def _add_camera_test_features(self, world: ET.Element, world_size: Tuple[float, float]):
        """Add features for camera testing."""
        # Add colored markers
        colors = [
            (1.0, 0.0, 0.0, 1.0),  # Red
            (0.0, 1.0, 0.0, 1.0),  # Green
            (0.0, 0.0, 1.0, 1.0),  # Blue
            (1.0, 1.0, 0.0, 1.0),  # Yellow
        ]

        for i, color in enumerate(colors):
            marker = Obstacle(
                name=f"color_marker_{i}",
                obstacle_type=ObstacleType.BOX,
                position=(i - 1.5, 5, 0.5),
                size=(0.5, 0.1, 1.0),
                color=color
            )
            self._add_obstacle_to_world(world, marker)

        # Add AprilTag-like patterns (simplified as boxes)
        for i in range(3):
            tag = Obstacle(
                name=f"april_tag_{i}",
                obstacle_type=ObstacleType.BOX,
                position=(-2 + i * 2, -5, 1.0),
                size=(0.3, 0.05, 0.3),
                color=(0.0, 0.0, 0.0, 1.0) if i % 2 == 0 else (1.0, 1.0, 1.0, 1.0)
            )
            self._add_obstacle_to_world(world, tag)

    def _add_depth_test_features(self, world: ET.Element, world_size: Tuple[float, float]):
        """Add features for depth camera testing."""
        # Objects at various depths
        depths = [1.0, 2.0, 3.0, 4.0, 5.0]

        for i, depth in enumerate(depths):
            obj = Obstacle(
                name=f"depth_object_{i}",
                obstacle_type=ObstacleType.SPHERE,
                position=(0, depth, 0.5),
                size=(0.3, 0.3, 0.3),
                color=(0.5 + i * 0.1, 0.5, 0.5, 1.0)
            )
            self._add_obstacle_to_world(world, obj)

        # Staircase for depth variation
        for i in range(5):
            step = Obstacle(
                name=f"step_{i}",
                obstacle_type=ObstacleType.BOX,
                position=(5, -2 + i * 0.5, i * 0.2),
                size=(1.0, 0.5, 0.4),
                color=(0.6, 0.6, 0.6, 1.0)
            )
            self._add_obstacle_to_world(world, step)

    def _has_lidar(self) -> bool:
        """Check if robot has lidar sensor."""
        if not self.robot_profile.hardware_manifest:
            return False

        for component in self.robot_profile.hardware_manifest.hardware_components:
            if component.component_type == "lidar":
                return True
        return False

    def _has_camera(self) -> bool:
        """Check if robot has camera sensor."""
        if not self.robot_profile.hardware_manifest:
            return False

        for component in self.robot_profile.hardware_manifest.hardware_components:
            if component.component_type == "camera":
                return True
        return False

    def _has_depth_camera(self) -> bool:
        """Check if robot has depth camera sensor."""
        if not self.robot_profile.hardware_manifest:
            return False

        for component in self.robot_profile.hardware_manifest.hardware_components:
            if component.component_type in ["depth_camera", "rgbd_camera"]:
                return True
        return False

    def _save_world(self, world_tree: ET.Element, file_path: Path):
        """Save world to file with proper formatting."""
        # Convert to string with pretty printing
        rough_string = ET.tostring(world_tree, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")

        # Remove extra blank lines
        lines = pretty_xml.split('\n')
        non_empty_lines = [line for line in lines if line.strip()]
        pretty_xml = '\n'.join(non_empty_lines)

        # Write to file
        with open(file_path, 'w') as f:
            f.write(pretty_xml)

    def generate_progressive_worlds(self) -> Dict[str, Path]:
        """
        Generate a series of progressively complex worlds.

        Returns:
            Dictionary mapping complexity level to world file paths
        """
        worlds = {}

        # Level 1: Empty world
        worlds["empty"] = self.generate_empty_world(
            WorldConfig(name="level_1_empty")
        )

        # Level 2: Simple obstacles
        worlds["simple"] = self.generate_simple_obstacles_world(
            num_obstacles=3,
            config=WorldConfig(name="level_2_simple")
        )

        # Level 3: More obstacles
        worlds["moderate"] = self.generate_simple_obstacles_world(
            num_obstacles=8,
            config=WorldConfig(name="level_3_moderate", size=(15.0, 15.0))
        )

        # Level 4: Navigation test
        worlds["navigation"] = self.generate_navigation_test_world(
            WorldConfig(name="level_4_navigation", size=(20.0, 20.0))
        )

        # Level 5: Sensor test
        worlds["sensor"] = self.generate_sensor_test_world(
            WorldConfig(name="level_5_sensor", size=(25.0, 25.0))
        )

        self.logger.info(f"Generated {len(worlds)} progressive worlds")

        return worlds