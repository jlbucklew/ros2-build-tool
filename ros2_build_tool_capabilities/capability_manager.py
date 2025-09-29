"""
Capability Manager for ROS2 Build Tool

Manages high-level robot capabilities and translates them into technical configurations.
"""

import logging
from pathlib import Path
from typing import Dict, List, Optional, Set, Any
from dataclasses import dataclass, field
from enum import Enum

from ros2_build_tool_core.models import (
    RobotProfile,
    RobotSpecs,
    HardwareComponent,
    HardwareManifest,
    NavigationConfig,
    SafetyConfig
)


class CapabilityLevel(Enum):
    """Complexity level of capabilities."""
    BASIC = "basic"  # Fundamental capabilities
    INTERMEDIATE = "intermediate"  # Common robot tasks
    ADVANCED = "advanced"  # Complex behaviors
    EXPERT = "expert"  # Specialized capabilities


class CapabilityStatus(Enum):
    """Status of a capability."""
    AVAILABLE = "available"  # Can be enabled
    ENABLED = "enabled"  # Currently enabled
    MISSING_REQUIREMENTS = "missing_requirements"  # Cannot be enabled
    CONFLICTING = "conflicting"  # Conflicts with other capabilities
    ERROR = "error"  # Error in configuration


@dataclass
class Capability:
    """Represents a robot capability."""
    id: str
    name: str
    description: str
    level: CapabilityLevel
    category: str  # navigation, perception, manipulation, etc.

    # Requirements
    required_hardware: List[str] = field(default_factory=list)
    required_sensors: List[str] = field(default_factory=list)
    required_capabilities: List[str] = field(default_factory=list)
    conflicting_capabilities: List[str] = field(default_factory=list)

    # Configuration
    ros_packages: List[str] = field(default_factory=list)
    launch_files: List[str] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)

    # User-friendly info
    benefits: List[str] = field(default_factory=list)
    limitations: List[str] = field(default_factory=list)
    setup_time_estimate: str = "< 5 minutes"

    # Status
    status: CapabilityStatus = CapabilityStatus.AVAILABLE
    missing_requirements: List[str] = field(default_factory=list)


class CapabilityManager:
    """Manages robot capabilities and their configurations."""

    def __init__(self,
                 robot_profile: RobotProfile,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize capability manager.

        Args:
            robot_profile: Robot configuration profile
            logger: Optional logger instance
        """
        self.robot_profile = robot_profile
        self.logger = logger or logging.getLogger(__name__)

        # Initialize capability registry
        self.capabilities: Dict[str, Capability] = {}
        self.enabled_capabilities: Set[str] = set()

        # Load default capabilities
        self._load_default_capabilities()

        # Analyze robot profile
        self._analyze_robot_capabilities()

    def _load_default_capabilities(self):
        """Load default capability definitions."""

        # Basic Movement
        self.register_capability(Capability(
            id="basic_movement",
            name="Basic Movement",
            description="Move robot forward, backward, and rotate",
            level=CapabilityLevel.BASIC,
            category="motion",
            required_hardware=["differential_drive OR omnidirectional_drive OR ackermann_steering"],
            ros_packages=["geometry_msgs", "cmd_vel_mux"],
            benefits=[
                "Control robot movement with joystick or keyboard",
                "Emergency stop functionality",
                "Velocity limiting for safety"
            ],
            setup_time_estimate="< 2 minutes"
        ))

        # Obstacle Avoidance
        self.register_capability(Capability(
            id="obstacle_avoidance",
            name="Obstacle Avoidance",
            description="Automatically avoid obstacles while moving",
            level=CapabilityLevel.BASIC,
            category="safety",
            required_sensors=["lidar OR depth_camera"],
            required_capabilities=["basic_movement"],
            ros_packages=["nav2_collision_monitor", "pointcloud_to_laserscan"],
            parameters={
                "collision_monitor": {
                    "stop_distance": 0.3,
                    "slowdown_distance": 1.0
                }
            },
            benefits=[
                "Prevents collisions with walls and objects",
                "Automatic speed reduction near obstacles",
                "Emergency stop when too close to obstacles"
            ],
            limitations=[
                "Cannot detect glass or mirrors reliably",
                "Limited to sensor field of view"
            ],
            setup_time_estimate="< 5 minutes"
        ))

        # Autonomous Navigation
        self.register_capability(Capability(
            id="autonomous_navigation",
            name="Autonomous Navigation",
            description="Navigate to goals while avoiding obstacles",
            level=CapabilityLevel.INTERMEDIATE,
            category="navigation",
            required_sensors=["lidar OR depth_camera"],
            required_capabilities=["basic_movement", "obstacle_avoidance"],
            ros_packages=["nav2_bringup", "nav2_bt_navigator", "nav2_planner", "nav2_controller"],
            parameters={
                "nav2": {
                    "use_sim_time": False,
                    "controller_frequency": 10.0,
                    "planner_frequency": 1.0
                }
            },
            benefits=[
                "Point-and-click navigation to goals",
                "Dynamic obstacle avoidance",
                "Path re-planning when blocked",
                "Return to charging station"
            ],
            limitations=[
                "Requires a map or SLAM",
                "May get stuck in tight spaces",
                "Difficulty with dynamic environments"
            ],
            setup_time_estimate="10-15 minutes"
        ))

        # Mapping (SLAM)
        self.register_capability(Capability(
            id="mapping",
            name="Environment Mapping",
            description="Create maps of the environment while exploring",
            level=CapabilityLevel.INTERMEDIATE,
            category="perception",
            required_sensors=["lidar OR depth_camera"],
            required_capabilities=["basic_movement"],
            conflicting_capabilities=["localization"],
            ros_packages=["slam_toolbox", "nav2_map_server"],
            parameters={
                "slam_toolbox": {
                    "mode": "mapping",
                    "scan_topic": "/scan",
                    "map_update_interval": 1.0
                }
            },
            benefits=[
                "Create maps for later navigation",
                "Explore unknown environments",
                "Save maps for reuse",
                "Real-time map updates"
            ],
            limitations=[
                "Requires good odometry",
                "Large environments need more memory",
                "Moving objects can corrupt map"
            ],
            setup_time_estimate="5-10 minutes"
        ))

        # Localization
        self.register_capability(Capability(
            id="localization",
            name="Localization in Known Map",
            description="Determine robot position in a pre-existing map",
            level=CapabilityLevel.INTERMEDIATE,
            category="perception",
            required_sensors=["lidar OR depth_camera"],
            required_capabilities=["basic_movement"],
            conflicting_capabilities=["mapping"],
            ros_packages=["nav2_amcl", "nav2_map_server"],
            parameters={
                "amcl": {
                    "scan_topic": "/scan",
                    "initial_pose_topic": "/initialpose",
                    "min_particles": 500,
                    "max_particles": 2000
                }
            },
            benefits=[
                "Accurate position tracking",
                "Recovery from kidnapping",
                "Works with saved maps",
                "Supports multiple robots"
            ],
            limitations=[
                "Requires accurate map",
                "Initial pose needed",
                "Can lose tracking in symmetrical environments"
            ],
            setup_time_estimate="5-10 minutes"
        ))

        # Object Detection
        self.register_capability(Capability(
            id="object_detection",
            name="Object Detection",
            description="Detect and recognize objects in the environment",
            level=CapabilityLevel.INTERMEDIATE,
            category="perception",
            required_sensors=["camera OR depth_camera"],
            ros_packages=["vision_msgs", "image_pipeline", "opencv"],
            parameters={
                "detection": {
                    "confidence_threshold": 0.7,
                    "max_detections": 10
                }
            },
            benefits=[
                "Identify specific objects",
                "Track moving objects",
                "Classify obstacles",
                "Support for ML models"
            ],
            limitations=[
                "Lighting dependent",
                "Limited to trained objects",
                "Processing intensive"
            ],
            setup_time_estimate="10-20 minutes"
        ))

        # Person Following
        self.register_capability(Capability(
            id="person_following",
            name="Person Following",
            description="Follow a specific person while maintaining safe distance",
            level=CapabilityLevel.ADVANCED,
            category="behavior",
            required_sensors=["camera OR depth_camera", "lidar OR depth_camera"],
            required_capabilities=["basic_movement", "obstacle_avoidance", "object_detection"],
            ros_packages=["person_follower", "leg_detector"],
            parameters={
                "follower": {
                    "follow_distance": 1.5,
                    "max_speed": 1.0,
                    "loss_timeout": 5.0
                }
            },
            benefits=[
                "Hands-free guidance",
                "Maintains safe distance",
                "Re-acquires target if lost briefly",
                "Avoids other people and obstacles"
            ],
            limitations=[
                "Can lose track in crowds",
                "Requires clear view of person",
                "May follow wrong person"
            ],
            setup_time_estimate="15-30 minutes"
        ))

        # Docking/Charging
        self.register_capability(Capability(
            id="auto_docking",
            name="Automatic Docking",
            description="Automatically return to and dock with charging station",
            level=CapabilityLevel.ADVANCED,
            category="behavior",
            required_sensors=["camera OR lidar"],
            required_capabilities=["autonomous_navigation", "localization"],
            ros_packages=["autodock", "nav2_behaviors"],
            parameters={
                "autodock": {
                    "approach_distance": 1.0,
                    "dock_speed": 0.2,
                    "retry_count": 3
                }
            },
            benefits=[
                "Automatic recharging",
                "Scheduled docking",
                "Low battery return",
                "Precise alignment"
            ],
            limitations=[
                "Requires compatible dock",
                "Dock must be accessible",
                "May fail if dock is moved"
            ],
            setup_time_estimate="20-30 minutes"
        ))

        # Teleoperation
        self.register_capability(Capability(
            id="teleoperation",
            name="Remote Control",
            description="Control robot remotely via joystick, keyboard, or web interface",
            level=CapabilityLevel.BASIC,
            category="control",
            required_capabilities=["basic_movement"],
            ros_packages=["teleop_twist_keyboard", "teleop_twist_joy", "rosbridge_suite"],
            parameters={
                "teleop": {
                    "linear_scale": 1.0,
                    "angular_scale": 1.0,
                    "enable_turbo": True
                }
            },
            benefits=[
                "Manual override capability",
                "Remote operation",
                "Multiple control methods",
                "Web-based interface option"
            ],
            setup_time_estimate="< 5 minutes"
        ))

        # Fleet Management
        self.register_capability(Capability(
            id="fleet_management",
            name="Multi-Robot Coordination",
            description="Coordinate multiple robots in shared environment",
            level=CapabilityLevel.EXPERT,
            category="coordination",
            required_capabilities=["autonomous_navigation", "localization"],
            ros_packages=["rmf_core", "rmf_fleet_adapter"],
            parameters={
                "fleet": {
                    "robot_namespace": True,
                    "conflict_resolution": "priority",
                    "communication_range": 50.0
                }
            },
            benefits=[
                "Traffic management",
                "Task allocation",
                "Collision avoidance between robots",
                "Centralized monitoring"
            ],
            limitations=[
                "Requires network connectivity",
                "Complex setup",
                "Scalability limits"
            ],
            setup_time_estimate="1-2 hours"
        ))

    def register_capability(self, capability: Capability) -> bool:
        """
        Register a new capability.

        Args:
            capability: Capability to register

        Returns:
            True if registered successfully
        """
        if capability.id in self.capabilities:
            self.logger.warning(f"Capability '{capability.id}' already registered")
            return False

        self.capabilities[capability.id] = capability
        self.logger.debug(f"Registered capability: {capability.name}")
        return True

    def _analyze_robot_capabilities(self):
        """Analyze robot profile to determine available capabilities."""
        # Check each capability's requirements
        for cap_id, capability in self.capabilities.items():
            self._update_capability_status(capability)

    def _update_capability_status(self, capability: Capability):
        """Update capability status based on current robot profile."""
        capability.missing_requirements = []
        capability.status = CapabilityStatus.AVAILABLE

        # Check hardware requirements
        if capability.required_hardware:
            if not self._check_hardware_requirements(capability.required_hardware):
                capability.missing_requirements.append("Missing required hardware")
                capability.status = CapabilityStatus.MISSING_REQUIREMENTS

        # Check sensor requirements
        if capability.required_sensors:
            if not self._check_sensor_requirements(capability.required_sensors):
                capability.missing_requirements.append("Missing required sensors")
                capability.status = CapabilityStatus.MISSING_REQUIREMENTS

        # Check capability dependencies
        for req_cap in capability.required_capabilities:
            if req_cap not in self.enabled_capabilities:
                if req_cap not in self.capabilities or \
                   self.capabilities[req_cap].status != CapabilityStatus.ENABLED:
                    capability.missing_requirements.append(f"Requires capability: {req_cap}")
                    capability.status = CapabilityStatus.MISSING_REQUIREMENTS

        # Check for conflicts
        for conflict_cap in capability.conflicting_capabilities:
            if conflict_cap in self.enabled_capabilities:
                capability.status = CapabilityStatus.CONFLICTING
                capability.missing_requirements.append(f"Conflicts with: {conflict_cap}")

    def _check_hardware_requirements(self, requirements: List[str]) -> bool:
        """Check if hardware requirements are met."""
        if not self.robot_profile.hardware_manifest:
            return False

        for requirement in requirements:
            # Handle OR conditions
            if " OR " in requirement:
                options = requirement.split(" OR ")
                if not any(self._has_hardware_type(opt.strip()) for opt in options):
                    return False
            else:
                if not self._has_hardware_type(requirement):
                    return False

        return True

    def _check_sensor_requirements(self, requirements: List[str]) -> bool:
        """Check if sensor requirements are met."""
        if not self.robot_profile.hardware_manifest:
            return False

        for requirement in requirements:
            # Handle OR conditions
            if " OR " in requirement:
                options = requirement.split(" OR ")
                if not any(self._has_sensor_type(opt.strip()) for opt in options):
                    return False
            else:
                if not self._has_sensor_type(requirement):
                    return False

        return True

    def _has_hardware_type(self, hw_type: str) -> bool:
        """Check if robot has specific hardware type."""
        if not self.robot_profile.hardware_manifest:
            return False

        # Check robot specs for drive type
        if "drive" in hw_type.lower():
            if self.robot_profile.robot_specs:
                return hw_type.lower().replace("_", " ") in \
                       self.robot_profile.robot_specs.drive_type.lower()

        # Check hardware components
        for component in self.robot_profile.hardware_manifest.hardware_components:
            if hw_type.lower() in component.component_type.lower():
                return True

        return False

    def _has_sensor_type(self, sensor_type: str) -> bool:
        """Check if robot has specific sensor type."""
        if not self.robot_profile.hardware_manifest:
            return False

        for component in self.robot_profile.hardware_manifest.hardware_components:
            if sensor_type.lower() in component.component_type.lower():
                return True

        # Check URDF sensor frames
        if self.robot_profile.urdf_path and self.robot_profile.robot_specs:
            for frame in self.robot_profile.robot_specs.sensor_frames:
                if sensor_type.lower() in frame.sensor_type.lower():
                    return True

        return False

    def enable_capability(self, capability_id: str) -> Dict[str, Any]:
        """
        Enable a capability.

        Args:
            capability_id: ID of capability to enable

        Returns:
            Dictionary with success status and details
        """
        if capability_id not in self.capabilities:
            return {
                "success": False,
                "error": f"Unknown capability: {capability_id}"
            }

        capability = self.capabilities[capability_id]

        # Update status
        self._update_capability_status(capability)

        if capability.status == CapabilityStatus.MISSING_REQUIREMENTS:
            return {
                "success": False,
                "error": "Missing requirements",
                "missing": capability.missing_requirements
            }

        if capability.status == CapabilityStatus.CONFLICTING:
            return {
                "success": False,
                "error": "Conflicting capabilities",
                "conflicts": capability.missing_requirements
            }

        # Enable required capabilities first
        for req_cap in capability.required_capabilities:
            if req_cap not in self.enabled_capabilities:
                result = self.enable_capability(req_cap)
                if not result["success"]:
                    return {
                        "success": False,
                        "error": f"Failed to enable required capability: {req_cap}",
                        "details": result
                    }

        # Apply configuration to robot profile
        self._apply_capability_config(capability)

        # Mark as enabled
        self.enabled_capabilities.add(capability_id)
        capability.status = CapabilityStatus.ENABLED

        self.logger.info(f"Enabled capability: {capability.name}")

        return {
            "success": True,
            "capability": capability.name,
            "configuration_applied": {
                "packages": capability.ros_packages,
                "parameters": capability.parameters
            }
        }

    def disable_capability(self, capability_id: str) -> Dict[str, Any]:
        """
        Disable a capability.

        Args:
            capability_id: ID of capability to disable

        Returns:
            Dictionary with success status
        """
        if capability_id not in self.capabilities:
            return {
                "success": False,
                "error": f"Unknown capability: {capability_id}"
            }

        if capability_id not in self.enabled_capabilities:
            return {
                "success": False,
                "error": f"Capability not enabled: {capability_id}"
            }

        # Check if other capabilities depend on this
        dependent_caps = []
        for cap_id in self.enabled_capabilities:
            if cap_id == capability_id:
                continue
            cap = self.capabilities[cap_id]
            if capability_id in cap.required_capabilities:
                dependent_caps.append(cap.name)

        if dependent_caps:
            return {
                "success": False,
                "error": "Other capabilities depend on this",
                "dependent": dependent_caps
            }

        # Remove from enabled
        self.enabled_capabilities.remove(capability_id)
        self.capabilities[capability_id].status = CapabilityStatus.AVAILABLE

        # Remove configuration from robot profile
        self._remove_capability_config(self.capabilities[capability_id])

        self.logger.info(f"Disabled capability: {self.capabilities[capability_id].name}")

        return {
            "success": True,
            "capability": self.capabilities[capability_id].name
        }

    def _apply_capability_config(self, capability: Capability):
        """Apply capability configuration to robot profile."""
        # Add ROS packages
        if capability.ros_packages:
            if not self.robot_profile.additional_packages:
                self.robot_profile.additional_packages = []
            self.robot_profile.additional_packages.extend(capability.ros_packages)

        # Apply parameters
        if capability.parameters:
            # Update navigation config if applicable
            if "nav2" in capability.parameters and self.robot_profile.navigation_config:
                self.robot_profile.navigation_config.__dict__.update(capability.parameters["nav2"])

            # Store other parameters for later use
            if not hasattr(self.robot_profile, "capability_parameters"):
                self.robot_profile.capability_parameters = {}
            self.robot_profile.capability_parameters[capability.id] = capability.parameters

        # Enable relevant features
        if capability.category == "navigation":
            self.robot_profile.enable_navigation = True
        if capability.id == "mapping":
            self.robot_profile.enable_slam = True
            self.robot_profile.slam_package = "slam_toolbox"

    def _remove_capability_config(self, capability: Capability):
        """Remove capability configuration from robot profile."""
        # Remove packages (if not used by other capabilities)
        if capability.ros_packages and self.robot_profile.additional_packages:
            for package in capability.ros_packages:
                # Check if other enabled capabilities use this package
                used_by_others = False
                for cap_id in self.enabled_capabilities:
                    if package in self.capabilities[cap_id].ros_packages:
                        used_by_others = True
                        break

                if not used_by_others and package in self.robot_profile.additional_packages:
                    self.robot_profile.additional_packages.remove(package)

        # Remove parameters
        if hasattr(self.robot_profile, "capability_parameters"):
            if capability.id in self.robot_profile.capability_parameters:
                del self.robot_profile.capability_parameters[capability.id]

    def get_available_capabilities(self,
                                  level: Optional[CapabilityLevel] = None,
                                  category: Optional[str] = None) -> List[Capability]:
        """
        Get list of available capabilities.

        Args:
            level: Filter by complexity level
            category: Filter by category

        Returns:
            List of available capabilities
        """
        # Update all capability statuses
        for capability in self.capabilities.values():
            self._update_capability_status(capability)

        # Filter capabilities
        result = []
        for capability in self.capabilities.values():
            if capability.status in [CapabilityStatus.AVAILABLE, CapabilityStatus.ENABLED]:
                if level and capability.level != level:
                    continue
                if category and capability.category != category:
                    continue
                result.append(capability)

        # Sort by level and name
        result.sort(key=lambda c: (c.level.value, c.name))

        return result

    def get_enabled_capabilities(self) -> List[Capability]:
        """Get list of enabled capabilities."""
        return [self.capabilities[cap_id] for cap_id in self.enabled_capabilities]

    def get_capability_graph(self) -> Dict[str, Any]:
        """
        Get capability dependency graph.

        Returns:
            Dictionary representing capability dependencies
        """
        graph = {
            "nodes": [],
            "edges": []
        }

        for cap_id, capability in self.capabilities.items():
            node = {
                "id": cap_id,
                "name": capability.name,
                "level": capability.level.value,
                "category": capability.category,
                "status": capability.status.value,
                "enabled": cap_id in self.enabled_capabilities
            }
            graph["nodes"].append(node)

            # Add dependency edges
            for req_cap in capability.required_capabilities:
                graph["edges"].append({
                    "from": req_cap,
                    "to": cap_id,
                    "type": "requires"
                })

            # Add conflict edges
            for conflict_cap in capability.conflicting_capabilities:
                graph["edges"].append({
                    "from": cap_id,
                    "to": conflict_cap,
                    "type": "conflicts"
                })

        return graph

    def suggest_capabilities(self) -> List[Capability]:
        """
        Suggest capabilities based on current configuration.

        Returns:
            List of suggested capabilities
        """
        suggestions = []

        # If no capabilities enabled, suggest basics
        if not self.enabled_capabilities:
            basic_caps = self.get_available_capabilities(level=CapabilityLevel.BASIC)
            suggestions.extend(basic_caps[:3])  # Top 3 basic capabilities

        else:
            # Suggest logical next steps
            for cap_id in self.enabled_capabilities:
                capability = self.capabilities[cap_id]

                # Find capabilities that depend on this one
                for other_cap in self.capabilities.values():
                    if cap_id in other_cap.required_capabilities and \
                       other_cap.status == CapabilityStatus.AVAILABLE:
                        if other_cap not in suggestions:
                            suggestions.append(other_cap)

        return suggestions[:5]  # Return top 5 suggestions

    def validate_capability_set(self, capability_ids: List[str]) -> Dict[str, Any]:
        """
        Validate that a set of capabilities can work together.

        Args:
            capability_ids: List of capability IDs to validate

        Returns:
            Validation result dictionary
        """
        result = {
            "valid": True,
            "conflicts": [],
            "missing_dependencies": [],
            "warnings": []
        }

        # Check for conflicts
        for i, cap_id1 in enumerate(capability_ids):
            if cap_id1 not in self.capabilities:
                result["valid"] = False
                result["warnings"].append(f"Unknown capability: {cap_id1}")
                continue

            cap1 = self.capabilities[cap_id1]

            for cap_id2 in capability_ids[i+1:]:
                if cap_id2 in cap1.conflicting_capabilities:
                    result["valid"] = False
                    result["conflicts"].append({
                        "capability1": cap1.name,
                        "capability2": self.capabilities[cap_id2].name
                    })

        # Check dependencies
        all_enabled = set(capability_ids)
        for cap_id in capability_ids:
            if cap_id not in self.capabilities:
                continue

            cap = self.capabilities[cap_id]
            for req_cap in cap.required_capabilities:
                if req_cap not in all_enabled:
                    result["missing_dependencies"].append({
                        "capability": cap.name,
                        "requires": self.capabilities[req_cap].name if req_cap in self.capabilities else req_cap
                    })
                    result["valid"] = False

        return result

    def export_capability_config(self) -> Dict[str, Any]:
        """Export current capability configuration."""
        return {
            "enabled_capabilities": list(self.enabled_capabilities),
            "robot_profile_updates": {
                "enable_navigation": self.robot_profile.enable_navigation,
                "enable_slam": self.robot_profile.enable_slam,
                "additional_packages": self.robot_profile.additional_packages,
                "capability_parameters": getattr(self.robot_profile, "capability_parameters", {})
            },
            "recommendations": [cap.id for cap in self.suggest_capabilities()]
        }