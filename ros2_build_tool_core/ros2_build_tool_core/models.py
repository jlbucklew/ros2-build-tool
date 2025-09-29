"""
Core data models with Pydantic validation
"""

from enum import Enum
from pathlib import Path
from typing import List, Dict, Optional, Any
from pydantic import BaseModel, Field, field_validator, ConfigDict
import yaml


class UseCase(str, Enum):
    """Robot use case enumeration"""
    MAPPING = "mapping"
    NAVIGATION = "navigation"
    PERCEPTION = "perception"
    FULL_STACK = "full_stack"


class SLAMType(str, Enum):
    """SLAM algorithm type"""
    SLAM_TOOLBOX = "slam_toolbox"
    CARTOGRAPHER = "cartographer"
    RTABMAP = "rtabmap"
    NONE = "none"


class BuildType(str, Enum):
    """CMake build type"""
    DEBUG = "Debug"
    RELEASE = "Release"
    RELWITHDEBINFO = "RelWithDebInfo"


class SensorFrame(BaseModel):
    """Extracted sensor frame from URDF with validation"""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    name: str = Field(..., min_length=1, description="Frame name")
    parent: str = Field(..., min_length=1, description="Parent frame")
    child: str = Field(..., min_length=1, description="Child frame")
    xyz: List[float] = Field(..., min_length=3, max_length=3, description="Translation [x, y, z]")
    rpy: List[float] = Field(..., min_length=3, max_length=3, description="Rotation [roll, pitch, yaw]")
    sensor_type: str = Field(..., pattern="^(lidar|camera|imu|gps)$", description="Sensor type")

    @field_validator('xyz', 'rpy')
    @classmethod
    def validate_transform_vectors(cls, v):
        """Ensure transform vectors have exactly 3 elements"""
        if len(v) != 3:
            raise ValueError('Transform vector must have exactly 3 elements')
        return v


class RobotSpecs(BaseModel):
    """Robot physical specifications with validation"""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    width: float = Field(..., gt=0, le=5.0, description="Robot width in meters")
    length: float = Field(..., gt=0, le=5.0, description="Robot length in meters")
    height: float = Field(..., gt=0, le=3.0, description="Robot height in meters")
    wheel_radius: float = Field(default=0.1, gt=0, le=1.0, description="Wheel radius in meters")
    wheel_separation: float = Field(default=0.3, gt=0, le=3.0, description="Wheel separation in meters")
    max_linear_velocity: float = Field(default=1.0, gt=0, le=10.0, description="Max linear velocity m/s")
    max_angular_velocity: float = Field(default=2.0, gt=0, le=10.0, description="Max angular velocity rad/s")
    robot_radius: float = Field(default=0.0, ge=0, description="Computed robot radius")

    def model_post_init(self, __context: Any) -> None:
        """Compute robot radius after initialization"""
        if self.robot_radius == 0.0:
            self.robot_radius = max(self.width, self.length) / 2.0


class HardwareComponent(BaseModel):
    """Hardware component definition with validation"""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    name: str = Field(..., min_length=1, description="Component name")
    type: str = Field(..., min_length=1, description="Component type (lidar, camera, imu, etc.)")
    repo: Optional[Dict[str, Any]] = Field(default=None, description="Repository configuration")
    rosdeps: List[str] = Field(default_factory=list, description="ROS dependencies")
    system_deps: List[str] = Field(default_factory=list, description="System dependencies")
    python_deps: List[str] = Field(default_factory=list, description="Python dependencies")
    launch_fragment: Optional[str] = Field(default=None, description="Launch file fragment")
    config_template: Optional[str] = Field(default=None, description="Config template path")
    frames: Dict[str, str] = Field(default_factory=dict, description="Frame mappings")
    github_url: Optional[str] = Field(default=None, description="Direct GitHub URL")
    discovered_executable: Optional[str] = Field(default=None, description="Runtime discovered executable")
    package_name: Optional[str] = Field(default=None, description="Discovered package name")
    plugin_type: Optional[str] = Field(default=None, description="Hardware interface plugin type")
    plugin_class: Optional[str] = Field(default=None, description="Hardware interface plugin class")

    @field_validator('github_url')
    @classmethod
    def validate_github_url(cls, v):
        """Validate GitHub URL format"""
        if v and not v.startswith(('https://github.com/', 'git@github.com:')):
            raise ValueError('Must be a valid GitHub URL')
        return v


class RobotProfile(BaseModel):
    """Complete robot configuration profile with validation"""
    model_config = ConfigDict(arbitrary_types_allowed=True, use_enum_values=False)

    name: str = Field(..., min_length=1, max_length=100, description="Profile name")
    ros_distro: str = Field(..., pattern="^(humble|jazzy)$", description="ROS2 distribution")
    use_case: UseCase = Field(..., description="Robot use case")
    hardware: List[str] = Field(..., min_length=1, description="Hardware component IDs")
    slam_type: SLAMType = Field(..., description="SLAM algorithm type")
    navigation: bool = Field(default=False, description="Enable navigation stack")
    foxglove: bool = Field(default=False, description="Enable Foxglove bridge")
    rviz: bool = Field(default=True, description="Enable RViz")
    teleop: bool = Field(default=False, description="Enable teleoperation")
    recording: bool = Field(default=False, description="Enable rosbag recording")
    diagnostics: bool = Field(default=True, description="Enable diagnostics")
    frames: Dict[str, str] = Field(default_factory=dict, description="Frame mappings")
    workspace_path: Optional[Path] = Field(default=None, description="Workspace path")
    build_type: BuildType = Field(default=BuildType.RELEASE, description="Build type")
    map_resolution: float = Field(default=0.05, gt=0, le=1.0, description="Map resolution in meters")
    custom_repos: Dict[str, Dict] = Field(default_factory=dict, description="Custom repositories")
    urdf_path: Optional[Path] = Field(default=None, description="Path to URDF file")
    robot_specs: Optional[RobotSpecs] = Field(default=None, description="Extracted robot specifications")
    sensor_frames: List[SensorFrame] = Field(default_factory=list, description="Extracted sensor frames")
    lifecycle_management: bool = Field(default=True, description="Enable lifecycle node management")
    self_healing: bool = Field(default=True, description="Enable self-healing watchdog")
    composable_nodes: bool = Field(default=False, description="Use composable nodes for performance")

    @field_validator('workspace_path', 'urdf_path')
    @classmethod
    def validate_paths(cls, v):
        """Validate paths exist when specified"""
        if v and not Path(v).exists():
            raise ValueError(f'Path does not exist: {v}')
        return v

    def to_yaml(self, path: Path) -> None:
        """Save profile to YAML file"""
        data = self.model_dump(mode='json')
        data['use_case'] = self.use_case.value
        data['slam_type'] = self.slam_type.value
        data['build_type'] = self.build_type.value
        data['workspace_path'] = str(self.workspace_path) if self.workspace_path else None
        data['urdf_path'] = str(self.urdf_path) if self.urdf_path else None

        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    @classmethod
    def from_yaml(cls, path: Path) -> 'RobotProfile':
        """Load profile from YAML file"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # Convert string enums back to enum types
        data['use_case'] = UseCase(data['use_case'])
        data['slam_type'] = SLAMType(data['slam_type'])
        data['build_type'] = BuildType(data.get('build_type', 'Release'))

        if data.get('workspace_path'):
            data['workspace_path'] = Path(data['workspace_path'])
        if data.get('urdf_path'):
            data['urdf_path'] = Path(data['urdf_path'])

        return cls(**data)


class DependencyManifest(BaseModel):
    """Complete dependency specification with validation"""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    system_apt: List[str] = Field(default_factory=list, description="APT system packages")
    rosdep_keys: List[str] = Field(default_factory=list, description="ROS dependency keys")
    python_packages: List[str] = Field(default_factory=list, description="Python packages")
    source_repos: Dict[str, Dict] = Field(default_factory=dict, description="Source repositories")

    def merge(self, other: 'DependencyManifest') -> None:
        """Merge another manifest into this one"""
        self.system_apt = list(set(self.system_apt + other.system_apt))
        self.rosdep_keys = list(set(self.rosdep_keys + other.rosdep_keys))
        self.python_packages = list(set(self.python_packages + other.python_packages))
        self.source_repos.update(other.source_repos)