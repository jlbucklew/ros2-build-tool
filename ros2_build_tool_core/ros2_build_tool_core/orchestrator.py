"""
Complete workspace orchestration pipeline
"""

import logging
import shutil
from pathlib import Path
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass

from ros2_build_tool_core.models import RobotProfile, HardwareComponent


@dataclass
class OrchestrationResult:
    """Result of workspace orchestration"""
    success: bool
    workspace_path: Optional[Path]
    errors: List[str]
    warnings: List[str]
    created_packages: List[str]
    cloned_repos: List[str]
    build_log: Optional[str]


class WorkspaceOrchestrator:
    """
    Main orchestrator coordinating all phases:
    1. Discovery Phase - Hardware detection & validation
    2. Configuration Phase - Profile generation & validation
    3. Build Phase - Repo cloning & colcon build
    4. Deployment Phase - Launch file & config generation
    """

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self.progress_callback: Optional[Callable[[str, int], None]] = None

    def set_progress_callback(self, callback: Callable[[str, int], None]):
        """Set callback for progress updates (message, percentage)"""
        self.progress_callback = callback

    def _report_progress(self, message: str, percentage: int):
        """Report progress to callback if set"""
        if self.progress_callback:
            self.progress_callback(message, percentage)
        self.logger.info(f"[{percentage}%] {message}")

    def create_workspace(
        self,
        profile: RobotProfile,
        workspace_path: Path,
        hardware_registry,
        skip_build: bool = False,
        skip_clone: bool = False
    ) -> OrchestrationResult:
        """
        Create complete ROS2 workspace from profile

        Args:
            profile: Robot profile with configuration
            workspace_path: Target workspace directory
            hardware_registry: HardwareRegistry instance
            skip_build: Skip colcon build phase
            skip_clone: Skip repository cloning (use existing repos)

        Returns:
            OrchestrationResult with success status and details
        """
        errors = []
        warnings = []
        created_packages = []
        cloned_repos = []
        build_log = None

        try:
            # Phase 1: Workspace Structure Creation (0-20%)
            self._report_progress("Creating workspace structure", 5)
            if not self._create_workspace_structure(workspace_path):
                errors.append("Failed to create workspace structure")
                return OrchestrationResult(False, workspace_path, errors, warnings, [], [], None)

            self._report_progress("Workspace structure created", 20)

            # Phase 2: Repository Cloning (20-40%)
            if not skip_clone:
                self._report_progress("Cloning hardware driver repositories", 25)
                from ros2_build_tool_core.repo_manager import RepositoryManager

                repo_manager = RepositoryManager(workspace_path, self.logger)

                # Collect GitHub repos from hardware components
                repos_to_clone = self._collect_repos_from_profile(profile, hardware_registry)

                if repos_to_clone:
                    clone_result = repo_manager.clone_driver_repos(repos_to_clone, profile.ros_distro)
                    cloned_repos = clone_result.get('success', [])
                    errors.extend(clone_result.get('errors', []))
                    warnings.extend(clone_result.get('warnings', []))

                    self._report_progress(f"Cloned {len(cloned_repos)} repositories", 40)
                else:
                    self._report_progress("No repositories to clone", 40)
                    warnings.append("No hardware repositories found to clone")
            else:
                self._report_progress("Skipping repository cloning", 40)

            # Phase 3: Package Generation (40-60%)
            self._report_progress("Generating robot packages", 45)
            package_result = self._generate_packages(profile, workspace_path, hardware_registry)
            created_packages = package_result.get('packages', [])
            errors.extend(package_result.get('errors', []))
            warnings.extend(package_result.get('warnings', []))

            self._report_progress(f"Generated {len(created_packages)} packages", 60)

            # Phase 4: Dependency Installation (60-75%)
            if not skip_build:
                self._report_progress("Installing dependencies with rosdep", 65)
                from ros2_build_tool_core.dependency_manager import DependencyManager

                dep_manager = DependencyManager(workspace_path, self.logger)
                dep_result = dep_manager.install_dependencies(profile.ros_distro)

                if not dep_result.get('success'):
                    warnings.append("Some dependencies failed to install")
                    warnings.extend(dep_result.get('warnings', []))

                self._report_progress("Dependencies installed", 75)
            else:
                self._report_progress("Skipping dependency installation", 75)

            # Phase 5: Build Workspace (75-90%)
            if not skip_build:
                self._report_progress("Building workspace with colcon", 80)
                from ros2_build_tool_core.build_manager import BuildManager

                build_manager = BuildManager(workspace_path, self.logger)
                build_result = build_manager.build_workspace()

                build_log = build_result.get('log')
                if not build_result.get('success'):
                    errors.append("Workspace build failed")
                    errors.extend(build_result.get('errors', []))
                    return OrchestrationResult(False, workspace_path, errors, warnings,
                                               created_packages, cloned_repos, build_log)

                self._report_progress("Workspace built successfully", 90)

                # Discover executables after build
                discovered = build_manager.discover_executables()
                self.logger.info(f"Discovered {len(discovered)} executables")
            else:
                self._report_progress("Skipping workspace build", 90)

            # Phase 6: Validation (90-100%)
            self._report_progress("Validating workspace", 95)
            validation_result = self._validate_workspace(workspace_path, profile)
            warnings.extend(validation_result.get('warnings', []))

            self._report_progress("Workspace creation complete", 100)

            return OrchestrationResult(
                success=True,
                workspace_path=workspace_path,
                errors=errors,
                warnings=warnings,
                created_packages=created_packages,
                cloned_repos=cloned_repos,
                build_log=build_log
            )

        except Exception as e:
            self.logger.error(f"Orchestration failed: {e}", exc_info=True)
            errors.append(f"Orchestration error: {str(e)}")
            return OrchestrationResult(False, workspace_path, errors, warnings,
                                       created_packages, cloned_repos, build_log)

    def _create_workspace_structure(self, workspace_path: Path) -> bool:
        """Create standard ROS2 workspace directory structure"""
        try:
            workspace_path.mkdir(parents=True, exist_ok=True)
            (workspace_path / 'src').mkdir(exist_ok=True)
            (workspace_path / 'build').mkdir(exist_ok=True)
            (workspace_path / 'install').mkdir(exist_ok=True)
            (workspace_path / 'log').mkdir(exist_ok=True)

            self.logger.info(f"Created workspace structure at: {workspace_path}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to create workspace structure: {e}")
            return False

    def _collect_repos_from_profile(
        self,
        profile: RobotProfile,
        hardware_registry
    ) -> Dict[str, Dict]:
        """Collect all repositories needed from hardware components"""
        repos = {}

        for hw_id in profile.hardware:
            component = hardware_registry.get(hw_id)
            if not component:
                self.logger.warning(f"Hardware component not found: {hw_id}")
                continue

            # Check if component has a repo
            if component.repo and 'url' in component.repo:
                repos[hw_id] = component.repo
            elif component.github_url:
                # Handle direct GitHub URL
                repos[hw_id] = {
                    'url': component.github_url,
                    'branches': {}  # Will use default branch
                }

        # Add custom repos from profile
        for repo_name, repo_config in profile.custom_repos.items():
            repos[repo_name] = repo_config

        return repos

    def _generate_packages(
        self,
        profile: RobotProfile,
        workspace_path: Path,
        hardware_registry
    ) -> Dict:
        """Generate all ROS2 packages for the robot"""
        packages = []
        errors = []
        warnings = []

        try:
            # Create main bringup package
            bringup_package = f"{profile.name}_bringup"
            package_path = workspace_path / 'src' / bringup_package

            if self._create_bringup_package(profile, package_path, hardware_registry):
                packages.append(bringup_package)
            else:
                errors.append(f"Failed to create bringup package: {bringup_package}")

            # Create config package if needed
            if profile.navigation:
                config_package = f"{profile.name}_config"
                config_path = workspace_path / 'src' / config_package
                if self._create_config_package(profile, config_path):
                    packages.append(config_package)
                else:
                    warnings.append(f"Failed to create config package: {config_package}")

        except Exception as e:
            self.logger.error(f"Package generation failed: {e}", exc_info=True)
            errors.append(f"Package generation error: {str(e)}")

        return {
            'packages': packages,
            'errors': errors,
            'warnings': warnings
        }

    def _create_bringup_package(
        self,
        profile: RobotProfile,
        package_path: Path,
        hardware_registry
    ) -> bool:
        """Create main bringup package with launch files"""
        try:
            # Create package structure
            package_path.mkdir(parents=True, exist_ok=True)
            (package_path / 'launch').mkdir(exist_ok=True)
            (package_path / 'config').mkdir(exist_ok=True)
            (package_path / 'urdf').mkdir(exist_ok=True)

            # Generate package.xml
            self._generate_package_xml(profile, package_path)

            # Generate CMakeLists.txt
            self._generate_cmakelists(profile, package_path)

            # Generate launch files
            from ros2_build_tool_generators.launch_generator import LifecycleAwareLaunchGenerator

            launch_gen = LifecycleAwareLaunchGenerator(package_path, self.logger)

            # Collect hardware components
            hardware_components = []
            for hw_id in profile.hardware:
                component = hardware_registry.get(hw_id)
                if component:
                    hardware_components.append(component)

            launch_gen.generate_main_launch(profile, hardware_components)

            # Generate parameter files if navigation is enabled
            if profile.navigation and profile.robot_specs:
                from ros2_build_tool_generators.nav2_parameter_generator import Nav2ParameterGenerator

                nav2_gen = Nav2ParameterGenerator(package_path / 'config', self.logger)
                nav2_gen.generate_nav2_params(profile.robot_specs)

            # Generate Foxglove launch if enabled
            if profile.foxglove:
                from ros2_build_tool_generators.foxglove_generator import FoxgloveGenerator

                foxglove_gen = FoxgloveGenerator(package_path, self.logger)
                foxglove_gen.generate_foxglove_launch(profile)

            # Copy URDF if provided
            if profile.urdf_path and profile.urdf_path.exists():
                urdf_dest = package_path / 'urdf' / profile.urdf_path.name
                shutil.copy2(profile.urdf_path, urdf_dest)
                self.logger.info(f"Copied URDF to: {urdf_dest}")

            self.logger.info(f"Created bringup package: {package_path}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to create bringup package: {e}", exc_info=True)
            return False

    def _create_config_package(self, profile: RobotProfile, package_path: Path) -> bool:
        """Create configuration package for parameters"""
        try:
            package_path.mkdir(parents=True, exist_ok=True)
            (package_path / 'config').mkdir(exist_ok=True)

            # Generate package.xml for config package
            self._generate_package_xml(profile, package_path, is_config=True)
            self._generate_cmakelists(profile, package_path, is_config=True)

            self.logger.info(f"Created config package: {package_path}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to create config package: {e}", exc_info=True)
            return False

    def _generate_package_xml(
        self,
        profile: RobotProfile,
        package_path: Path,
        is_config: bool = False
    ):
        """Generate package.xml file"""
        package_name = package_path.name

        dependencies = [
            'rclpy',
            'launch',
            'launch_ros',
        ]

        if not is_config:
            dependencies.extend([
                'robot_state_publisher',
                'joint_state_publisher',
            ])

            if profile.navigation:
                dependencies.append('nav2_bringup')

            if profile.slam_type.value != 'none':
                dependencies.append(profile.slam_type.value)

        xml_content = f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.1.0</version>
  <description>{package_name} package</description>
  <maintainer email="robot@example.com">Robot Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

'''

        for dep in dependencies:
            xml_content += f'  <depend>{dep}</depend>\n'

        xml_content += '''
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
'''

        package_xml = package_path / 'package.xml'
        package_xml.write_text(xml_content)
        self.logger.debug(f"Generated package.xml: {package_xml}")

    def _generate_cmakelists(
        self,
        profile: RobotProfile,
        package_path: Path,
        is_config: bool = False
    ):
        """Generate CMakeLists.txt file"""
        package_name = package_path.name

        cmake_content = f'''cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

'''

        if not is_config:
            cmake_content += '''# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

'''
        else:
            cmake_content += '''# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

'''

        cmake_content += '''ament_package()
'''

        cmakelists_txt = package_path / 'CMakeLists.txt'
        cmakelists_txt.write_text(cmake_content)
        self.logger.debug(f"Generated CMakeLists.txt: {cmakelists_txt}")

    def _validate_workspace(self, workspace_path: Path, profile: RobotProfile) -> Dict:
        """Validate generated workspace"""
        warnings = []

        try:
            # Check required directories exist
            required_dirs = ['src', 'build', 'install', 'log']
            for dir_name in required_dirs:
                dir_path = workspace_path / dir_name
                if not dir_path.exists():
                    warnings.append(f"Missing directory: {dir_name}")

            # Check bringup package exists
            bringup_package = workspace_path / 'src' / f'{profile.name}_bringup'
            if not bringup_package.exists():
                warnings.append(f"Bringup package not found: {bringup_package}")

            # Check launch files exist
            launch_dir = bringup_package / 'launch'
            if launch_dir.exists():
                launch_files = list(launch_dir.glob('*.py'))
                if not launch_files:
                    warnings.append("No launch files generated")

        except Exception as e:
            self.logger.error(f"Workspace validation error: {e}")
            warnings.append(f"Validation error: {str(e)}")

        return {'warnings': warnings}

    def rollback(self, workspace_path: Path) -> bool:
        """Rollback workspace creation on failure"""
        try:
            if workspace_path.exists():
                self.logger.warning(f"Rolling back workspace: {workspace_path}")
                shutil.rmtree(workspace_path)
                self.logger.info("Rollback completed")
                return True
        except Exception as e:
            self.logger.error(f"Rollback failed: {e}")
            return False