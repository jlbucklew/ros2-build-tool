"""
Platform detection and ROS2 installation management
"""

import subprocess
import logging
from pathlib import Path
from typing import Dict, Tuple


class Platform:
    """Platform detection and ROS2 installation management"""

    SUPPORTED_DISTROS = {
        'humble': {
            'ubuntu_versions': ['22.04'],
            'codenames': ['jammy'],
            'eol': '2027-05',
            'packages': 'ros-humble-desktop'
        },
        'jazzy': {
            'ubuntu_versions': ['24.04'],
            'codenames': ['noble'],
            'eol': '2029-05',
            'packages': 'ros-jazzy-desktop'
        }
    }

    @staticmethod
    def detect() -> Dict[str, str]:
        """Detect platform information"""
        info = {
            'os': 'unknown',
            'version': 'unknown',
            'codename': 'unknown',
            'arch': 'unknown',
            'ros_distro': None
        }

        # Detect OS info
        try:
            with open('/etc/os-release', 'r') as f:
                for line in f:
                    if line.startswith('ID='):
                        info['os'] = line.split('=')[1].strip().strip('"')
                    elif line.startswith('VERSION_ID='):
                        info['version'] = line.split('=')[1].strip().strip('"')
                    elif line.startswith('VERSION_CODENAME='):
                        info['codename'] = line.split('=')[1].strip().strip('"')
        except:
            pass

        # Detect architecture
        try:
            result = subprocess.run(['dpkg', '--print-architecture'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                info['arch'] = result.stdout.strip()
        except:
            pass

        # Detect ROS distro
        for distro in Platform.SUPPORTED_DISTROS:
            if Path(f'/opt/ros/{distro}').exists():
                info['ros_distro'] = distro
                break

        return info

    @staticmethod
    def validate_distro(distro: str, platform_info: Dict[str, str]) -> Tuple[bool, str]:
        """Validate ROS2 distribution compatibility"""
        if distro not in Platform.SUPPORTED_DISTROS:
            return False, f"Unsupported ROS2 distribution: {distro}"

        distro_info = Platform.SUPPORTED_DISTROS[distro]
        if platform_info['codename'] not in distro_info['codenames']:
            return False, f"ROS2 {distro} not supported on {platform_info['codename']}"

        return True, "OK"

    @staticmethod
    def check_sudo_access() -> bool:
        """Check if user has sudo privileges without actually using sudo"""
        try:
            # Check if user is in sudo group or is root
            import os
            import grp

            if os.geteuid() == 0:
                return True

            try:
                sudo_group = grp.getgrnam('sudo')
                return os.getlogin() in sudo_group.gr_mem
            except (KeyError, OSError):
                # sudo group doesn't exist or can't get user info
                # Try running sudo -n true as a test
                result = subprocess.run(
                    ['sudo', '-n', 'true'],
                    capture_output=True,
                    timeout=5
                )
                return result.returncode == 0
        except Exception:
            return False

    @staticmethod
    def confirm_installation(distro: str, minimal: bool) -> bool:
        """Prompt user to confirm installation"""
        package_type = 'ros-base' if minimal else 'desktop'
        print(f"\n{'='*60}")
        print(f"WARNING: This will install ROS2 {distro} ({package_type})")
        print(f"{'='*60}")
        print("This operation will:")
        print("  - Add ROS2 APT repository to your system")
        print("  - Install ROS2 packages (may be several GB)")
        print("  - Modify system package sources")
        print("  - Require sudo privileges")
        print(f"\nEstimated download size: ~{1 if minimal else 3} GB")
        print(f"Estimated disk space needed: ~{2 if minimal else 5} GB")
        print()

        response = input("Do you want to proceed? (yes/no): ").strip().lower()
        return response in ['yes', 'y']

    @staticmethod
    def install_ros2(distro: str, minimal: bool = False, skip_confirmation: bool = False) -> bool:
        """
        Install ROS2 if not present with safety checks and rollback capability

        Args:
            distro: ROS2 distribution (humble, jazzy)
            minimal: Install minimal ros-base instead of desktop
            skip_confirmation: Skip user confirmation (for automated installs)

        Returns:
            True if installation succeeded, False otherwise
        """
        # Check if already installed
        if Path(f'/opt/ros/{distro}').exists():
            logging.info(f'ROS2 {distro} is already installed')
            return True

        # Check sudo access
        if not Platform.check_sudo_access():
            logging.error(
                "Sudo access required but not available.\n"
                "Please run with sudo or add your user to the sudo group:\n"
                "  sudo usermod -aG sudo $USER\n"
                "Then log out and log back in."
            )
            return False

        # Confirm with user unless skipped
        if not skip_confirmation and not Platform.confirm_installation(distro, minimal):
            logging.info("Installation cancelled by user")
            return False

        # Track what we've modified for potential rollback
        created_files = []
        installed_packages = []

        try:
            logging.info(f"Beginning ROS2 {distro} installation...")

            # Step 1: Install prerequisites
            logging.info("Step 1/5: Installing prerequisites...")
            prereq_packages = ['software-properties-common', 'curl', 'gnupg', 'lsb-release']
            subprocess.run(
                ['sudo', 'apt', 'update'],
                check=True,
                timeout=300
            )
            subprocess.run(
                ['sudo', 'apt', 'install', '-y'] + prereq_packages,
                check=True,
                timeout=300
            )
            installed_packages.extend(prereq_packages)

            # Step 2: Add universe repository
            logging.info("Step 2/5: Enabling universe repository...")
            subprocess.run(
                ['sudo', 'add-apt-repository', '-y', 'universe'],
                check=True,
                timeout=60
            )

            # Step 3: Add ROS2 GPG key
            logging.info("Step 3/5: Adding ROS2 GPG key...")
            key_url = 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key'
            key_cmd = f'curl -sSL {key_url} | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg'
            subprocess.run(
                ['bash', '-c', key_cmd],
                check=True,
                timeout=60
            )
            created_files.append('/usr/share/keyrings/ros-archive-keyring.gpg')

            # Step 4: Add ROS2 repository
            logging.info("Step 4/5: Adding ROS2 repository...")
            repo_file = '/etc/apt/sources.list.d/ros2.list'
            repo_cmd = f'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee {repo_file}'
            subprocess.run(
                ['bash', '-c', repo_cmd],
                check=True,
                timeout=60
            )
            created_files.append(repo_file)

            # Step 5: Install ROS2
            logging.info("Step 5/5: Installing ROS2 packages (this may take a while)...")
            subprocess.run(
                ['sudo', 'apt', 'update'],
                check=True,
                timeout=300
            )

            package = f"ros-{distro}-{'ros-base' if minimal else 'desktop'}"
            ros_packages = [package, 'python3-colcon-common-extensions', 'python3-rosdep']
            subprocess.run(
                ['sudo', 'apt', 'install', '-y'] + ros_packages,
                check=True,
                timeout=1800  # 30 minutes for ROS2 installation
            )
            installed_packages.extend(ros_packages)

            # Initialize rosdep (failures here are not critical)
            logging.info("Initializing rosdep...")
            try:
                subprocess.run(['sudo', 'rosdep', 'init'], check=False, timeout=30)
                subprocess.run(['rosdep', 'update'], check=False, timeout=300)
            except Exception as e:
                logging.warning(f"rosdep initialization had issues (not critical): {e}")

            logging.info(f"✓ ROS2 {distro} installation completed successfully!")
            return True

        except subprocess.TimeoutExpired as e:
            logging.error(f"Installation timed out: {e}")
            logging.error(
                "The installation process took too long. Possible causes:\n"
                "  - Slow network connection\n"
                "  - APT repository issues\n"
                "  - System resource constraints\n"
                "Try running the installation manually or check your internet connection."
            )
            return False

        except subprocess.CalledProcessError as e:
            logging.error(f"Installation failed: {e}")
            logging.error(
                f"Command failed: {e.cmd}\n"
                f"Return code: {e.returncode}\n"
                "Attempting rollback..."
            )

            # Attempt rollback
            Platform._rollback_installation(created_files, installed_packages)
            return False

        except Exception as e:
            logging.error(f"Unexpected error during installation: {e}")
            logging.error("Attempting rollback...")
            Platform._rollback_installation(created_files, installed_packages)
            return False

    @staticmethod
    def _rollback_installation(created_files: list, installed_packages: list) -> None:
        """Attempt to rollback a failed installation"""
        logging.info("Rolling back installation changes...")

        # Remove created files
        for file_path in created_files:
            try:
                if Path(file_path).exists():
                    subprocess.run(['sudo', 'rm', '-f', file_path], check=False, timeout=30)
                    logging.info(f"  Removed: {file_path}")
            except Exception as e:
                logging.warning(f"  Could not remove {file_path}: {e}")

        # Note: We don't automatically remove installed packages as they may be dependencies
        # for other software. User should manually remove if desired.
        if installed_packages:
            logging.info(
                f"The following packages were partially installed: {', '.join(installed_packages)}\n"
                "To remove them manually, run:\n"
                f"  sudo apt remove {' '.join(installed_packages)}\n"
                f"  sudo apt autoremove"
            )

        logging.info("Rollback completed. Your system should be in its previous state.")