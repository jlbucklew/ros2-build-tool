"""
Dependency management for rosdep and system dependencies
"""

import logging
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional


class DependencyManager:
    """
    Manages dependency resolution and installation with rosdep
    """

    def __init__(self, workspace_path: Path, logger: Optional[logging.Logger] = None):
        self.workspace_path = workspace_path
        self.logger = logger or logging.getLogger(__name__)
        self.max_retries = 3
        self.retry_delay = 2  # seconds

    def install_dependencies(
        self,
        ros_distro: str,
        skip_keys: Optional[List[str]] = None
    ) -> Dict:
        """
        Install dependencies using rosdep with retry logic

        Args:
            ros_distro: ROS distribution name
            skip_keys: Optional list of rosdep keys to skip

        Returns:
            Dict with 'success' bool, 'installed' list, 'warnings' list
        """
        installed = []
        warnings = []

        try:
            # Initialize rosdep if needed
            if not self._check_rosdep_initialized():
                self.logger.info("Initializing rosdep...")
                if not self._initialize_rosdep():
                    warnings.append("Failed to initialize rosdep")
                    return {'success': False, 'installed': [], 'warnings': warnings}

            # Update rosdep
            self.logger.info("Updating rosdep...")
            if not self._update_rosdep():
                warnings.append("Failed to update rosdep")

            # Install dependencies with retry
            src_path = self.workspace_path / 'src'
            if not src_path.exists():
                warnings.append("Source directory not found")
                return {'success': True, 'installed': [], 'warnings': warnings}

            for attempt in range(self.max_retries):
                result = self._run_rosdep_install(ros_distro, src_path, skip_keys)

                if result['success']:
                    installed = result.get('installed', [])
                    self.logger.info(f"✓ Dependencies installed successfully")
                    return {
                        'success': True,
                        'installed': installed,
                        'warnings': warnings
                    }
                else:
                    self.logger.warning(
                        f"rosdep install attempt {attempt + 1}/{self.max_retries} failed"
                    )
                    if attempt < self.max_retries - 1:
                        time.sleep(self.retry_delay * (attempt + 1))

            warnings.append("Failed to install all dependencies after retries")
            return {
                'success': False,
                'installed': installed,
                'warnings': warnings
            }

        except Exception as e:
            self.logger.error(f"Dependency installation error: {e}", exc_info=True)
            warnings.append(f"Error: {str(e)}")
            return {
                'success': False,
                'installed': installed,
                'warnings': warnings
            }

    def _check_rosdep_initialized(self) -> bool:
        """Check if rosdep is initialized"""
        try:
            # Check for /etc/ros/rosdep/sources.list.d/20-default.list
            rosdep_sources = Path('/etc/ros/rosdep/sources.list.d/20-default.list')
            return rosdep_sources.exists()
        except Exception:
            return False

    def _initialize_rosdep(self) -> bool:
        """Initialize rosdep (requires sudo)"""
        try:
            result = subprocess.run(
                ['sudo', 'rosdep', 'init'],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.logger.info("✓ rosdep initialized")
                return True
            else:
                # Check if already initialized
                if 'already initialized' in result.stderr.lower():
                    self.logger.info("rosdep already initialized")
                    return True

                self.logger.error(f"rosdep init failed: {result.stderr}")
                return False

        except FileNotFoundError:
            self.logger.error("rosdep command not found. Please install: sudo apt install python3-rosdep")
            return False
        except Exception as e:
            self.logger.error(f"Failed to initialize rosdep: {e}")
            return False

    def _update_rosdep(self) -> bool:
        """Update rosdep database"""
        try:
            result = subprocess.run(
                ['rosdep', 'update'],
                capture_output=True,
                text=True,
                timeout=120
            )

            if result.returncode == 0:
                self.logger.debug("✓ rosdep updated")
                return True
            else:
                self.logger.warning(f"rosdep update warning: {result.stderr}")
                return True  # Continue even if update has warnings

        except FileNotFoundError:
            self.logger.error("rosdep command not found")
            return False
        except Exception as e:
            self.logger.error(f"Failed to update rosdep: {e}")
            return False

    def _run_rosdep_install(
        self,
        ros_distro: str,
        src_path: Path,
        skip_keys: Optional[List[str]] = None
    ) -> Dict:
        """Run rosdep install command"""
        try:
            cmd = [
                'rosdep', 'install',
                '--from-paths', str(src_path),
                '--ignore-src',
                '--rosdistro', ros_distro,
                '-y'
            ]

            if skip_keys:
                for key in skip_keys:
                    cmd.extend(['--skip-keys', key])

            self.logger.debug(f"Running: {' '.join(cmd)}")

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=600  # 10 minute timeout
            )

            # Parse installed packages from output
            installed = self._parse_rosdep_output(result.stdout)

            if result.returncode == 0:
                return {
                    'success': True,
                    'installed': installed
                }
            else:
                self.logger.warning(f"rosdep install failed: {result.stderr}")
                return {
                    'success': False,
                    'installed': installed
                }

        except subprocess.TimeoutExpired:
            self.logger.error("rosdep install timed out")
            return {'success': False, 'installed': []}
        except Exception as e:
            self.logger.error(f"rosdep install error: {e}")
            return {'success': False, 'installed': []}

    def _parse_rosdep_output(self, output: str) -> List[str]:
        """Parse rosdep output for installed packages"""
        installed = []

        for line in output.split('\n'):
            # Look for lines like: "Installing package-name"
            if 'installing' in line.lower() or 'installed' in line.lower():
                # Extract package names (simple heuristic)
                words = line.split()
                for word in words:
                    if word.startswith('ros-') or '-' in word:
                        installed.append(word)

        return list(set(installed))  # Remove duplicates

    def check_missing_dependencies(self, ros_distro: str) -> Dict:
        """
        Check for missing dependencies without installing

        Returns:
            Dict with 'missing' list of dependency keys
        """
        missing = []

        try:
            src_path = self.workspace_path / 'src'
            if not src_path.exists():
                return {'missing': []}

            cmd = [
                'rosdep', 'check',
                '--from-paths', str(src_path),
                '--ignore-src',
                '--rosdistro', ros_distro
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60
            )

            # Parse missing dependencies
            if result.returncode != 0:
                for line in result.stdout.split('\n'):
                    line = line.strip()
                    if line and not line.startswith('#'):
                        missing.append(line)

        except Exception as e:
            self.logger.error(f"Dependency check error: {e}")

        return {'missing': missing}

    def generate_dependency_manifest(self, profile) -> Dict:
        """
        Generate complete dependency manifest from profile

        Returns:
            DependencyManifest-compatible dict
        """
        from ros2_build_tool_core.models import DependencyManifest

        manifest = DependencyManifest()

        # Add common ROS dependencies
        manifest.rosdep_keys.extend([
            'rclpy',
            'rclcpp',
            'std_msgs',
            'geometry_msgs',
            'sensor_msgs',
        ])

        # Add navigation dependencies
        if profile.navigation:
            manifest.rosdep_keys.extend([
                'nav2_bringup',
                'nav2_msgs',
            ])

        # Add SLAM dependencies
        if profile.slam_type.value != 'none':
            manifest.rosdep_keys.append(profile.slam_type.value)

        # Remove duplicates
        manifest.rosdep_keys = list(set(manifest.rosdep_keys))

        return manifest.model_dump()

    def install_pip_dependencies(self, requirements: List[str]) -> bool:
        """
        Install Python dependencies with pip

        Args:
            requirements: List of pip package names

        Returns:
            True if successful
        """
        if not requirements:
            return True

        try:
            cmd = ['pip3', 'install'] + requirements

            self.logger.info(f"Installing Python packages: {', '.join(requirements)}")

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300
            )

            if result.returncode == 0:
                self.logger.info("✓ Python packages installed")
                return True
            else:
                self.logger.error(f"pip install failed: {result.stderr}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to install Python packages: {e}")
            return False

    def install_apt_dependencies(self, packages: List[str]) -> bool:
        """
        Install system dependencies with apt (requires sudo)

        Args:
            packages: List of apt package names

        Returns:
            True if successful
        """
        if not packages:
            return True

        try:
            cmd = ['sudo', 'apt-get', 'install', '-y'] + packages

            self.logger.info(f"Installing system packages: {', '.join(packages)}")

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=600
            )

            if result.returncode == 0:
                self.logger.info("✓ System packages installed")
                return True
            else:
                self.logger.error(f"apt install failed: {result.stderr}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to install system packages: {e}")
            return False