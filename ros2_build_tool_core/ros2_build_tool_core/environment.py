"""
ROS2 environment management with caching
"""

import os
import json
import time
import subprocess
from pathlib import Path
from typing import Dict


class Environment:
    """ROS2 environment management with caching"""

    def __init__(self, distro: str, workspace_path: Path):
        self.distro = distro
        self.workspace_path = workspace_path
        self.cache_file = workspace_path / '.env.cache'
        self._env_cache = None

    def setup(self) -> Dict[str, str]:
        """Setup ROS2 environment with caching"""
        # Check cache
        if self._env_cache:
            return self._env_cache

        if self.cache_file.exists():
            cache_age = time.time() - self.cache_file.stat().st_mtime
            if cache_age < 3600:  # 1 hour cache
                with open(self.cache_file, 'r') as f:
                    self._env_cache = json.load(f)
                    return self._env_cache

        # Build environment
        env = os.environ.copy()

        # Source ROS2
        ros_setup = f'/opt/ros/{self.distro}/setup.bash'
        if Path(ros_setup).exists():
            result = subprocess.run(
                ['bash', '-c', f'source {ros_setup} && env'],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        if not key.startswith('_'):
                            env[key] = value

        # Source workspace if built
        ws_setup = self.workspace_path / 'install' / 'setup.bash'
        if ws_setup.exists():
            result = subprocess.run(
                ['bash', '-c', f'source {ws_setup} && env'],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        if not key.startswith('_'):
                            env[key] = value

        # Set ROS2 specific variables
        env.update({
            'ROS_DOMAIN_ID': env.get('ROS_DOMAIN_ID', '42'),
            'ROS_LOCALHOST_ONLY': env.get('ROS_LOCALHOST_ONLY', '0'),
            'RCUTILS_COLORIZED_OUTPUT': '1'
        })

        # Cache environment
        self._env_cache = env
        self.workspace_path.mkdir(parents=True, exist_ok=True)
        with open(self.cache_file, 'w') as f:
            json.dump(env, f)

        return env

    def clear_cache(self):
        """Clear environment cache"""
        if self.cache_file.exists():
            self.cache_file.unlink()
        self._env_cache = None