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
    def install_ros2(distro: str, minimal: bool = False) -> bool:
        """Install ROS2 if not present"""
        if Path(f'/opt/ros/{distro}').exists():
            return True

        try:
            # Add ROS2 apt repository
            commands = [
                ['sudo', 'apt', 'update'],
                ['sudo', 'apt', 'install', '-y', 'software-properties-common'],
                ['sudo', 'add-apt-repository', '-y', 'universe'],
                ['sudo', 'apt', 'install', '-y', 'curl', 'gnupg', 'lsb-release'],
            ]

            # Add ROS2 GPG key
            key_url = 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key'
            key_cmd = f'curl -sSL {key_url} | sudo apt-key add -'
            commands.append(['bash', '-c', key_cmd])

            # Add repository
            repo_cmd = f'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list'
            commands.append(['bash', '-c', repo_cmd])

            # Update and install
            package = f"ros-{distro}-{'ros-base' if minimal else 'desktop'}"
            commands.extend([
                ['sudo', 'apt', 'update'],
                ['sudo', 'apt', 'install', '-y', package, 'python3-colcon-common-extensions', 'python3-rosdep']
            ])

            for cmd in commands:
                subprocess.run(cmd, check=True)

            # Initialize rosdep
            subprocess.run(['sudo', 'rosdep', 'init'], check=False)
            subprocess.run(['rosdep', 'update'], check=False)

            return True

        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to install ROS2: {e}")
            return False