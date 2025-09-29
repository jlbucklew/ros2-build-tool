"""
Runtime package discovery using ament_index_python with enhanced detection
"""

import os
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Dict

try:
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    AMENT_INDEX_AVAILABLE = True
except ImportError:
    AMENT_INDEX_AVAILABLE = False


class PackageDiscovery:
    """Runtime package discovery using ament_index_python"""

    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.cache = {}

    def discover_executables(self, package_name: str) -> List[str]:
        """Discover all executables in a ROS2 package"""
        if not AMENT_INDEX_AVAILABLE:
            self.logger.warning("ament_index_python not available, skipping package discovery")
            return []

        # Check cache
        if package_name in self.cache:
            return self.cache[package_name]

        try:
            prefix = get_package_prefix(package_name)
            lib_dir = Path(prefix) / 'lib' / package_name

            if not lib_dir.exists():
                self.logger.warning(f"Package lib directory not found: {lib_dir}")
                return []

            executables = [
                f.name for f in lib_dir.iterdir()
                if f.is_file() and os.access(f, os.X_OK)
            ]

            # Cache results
            self.cache[package_name] = executables

            self.logger.info(f"Found {len(executables)} executables in {package_name}: {executables}")
            return executables

        except Exception as e:
            self.logger.error(f"Failed to discover executables for {package_name}: {e}")
            return []

    def select_primary_executable(self, package_name: str, executables: List[str]) -> Optional[str]:
        """Select primary executable using heuristics"""
        if not executables:
            return None

        if len(executables) == 1:
            return executables[0]

        # Priority patterns - more specific patterns first
        priority_patterns = [
            f"{package_name}_node",
            f"{package_name}_driver",
            f"{package_name}_server",
            "hardware_interface",
            "_node",
            "_driver",
            "_server",
            "node",
            "driver",
            "server"
        ]

        scores = {}
        for exe in executables:
            score = 0
            exe_lower = exe.lower()

            # Check for exact matches first
            if exe == f"{package_name}_node" or exe == f"{package_name}_driver":
                score = 1000
            else:
                # Score based on pattern priority
                for i, pattern in enumerate(priority_patterns):
                    if pattern in exe_lower:
                        score = len(priority_patterns) - i
                        break

            scores[exe] = score

        selected = max(scores, key=scores.get)
        self.logger.info(f"Selected primary executable: {selected} (scores: {scores})")
        return selected

    def discover_launch_files(self, package_name: str) -> List[Path]:
        """Discover launch files in a package"""
        if not AMENT_INDEX_AVAILABLE:
            return []

        try:
            share_dir = Path(get_package_share_directory(package_name))
            launch_dir = share_dir / 'launch'

            if not launch_dir.exists():
                return []

            launch_files = list(launch_dir.glob('*.launch.py')) + \
                          list(launch_dir.glob('*.launch.xml'))

            self.logger.info(f"Found {len(launch_files)} launch files in {package_name}")
            return launch_files

        except Exception as e:
            self.logger.warning(f"Failed to discover launch files for {package_name}: {e}")
            return []

    def parse_setup_py_entry_points(self, package_path: Path) -> Dict[str, str]:
        """Parse setup.py to find entry points (primary executables) using AST"""
        setup_py = package_path / 'setup.py'

        if not setup_py.exists():
            return {}

        try:
            import ast

            with open(setup_py, 'r', encoding='utf-8') as f:
                content = f.read()

            # Parse Python code with AST
            tree = ast.parse(content, filename=str(setup_py))

            scripts = {}

            # Find setup() call
            for node in ast.walk(tree):
                if isinstance(node, ast.Call):
                    # Check if this is a call to setup()
                    if isinstance(node.func, ast.Name) and node.func.id == 'setup':
                        # Look for entry_points keyword argument
                        for keyword in node.keywords:
                            if keyword.arg == 'entry_points':
                                scripts.update(self._parse_entry_points_ast(keyword.value))

            self.logger.info(f"Found {len(scripts)} entry points in setup.py")
            return scripts

        except SyntaxError as e:
            self.logger.warning(f"Syntax error parsing setup.py: {e}")
            return {}
        except Exception as e:
            self.logger.warning(f"Failed to parse setup.py: {e}")
            return {}

    def _parse_entry_points_ast(self, node) -> Dict[str, str]:
        """Parse entry_points AST node to extract console_scripts"""
        import ast

        scripts = {}

        try:
            # entry_points is typically a dict
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    # Look for 'console_scripts' key
                    if isinstance(key, ast.Constant) and key.value == 'console_scripts':
                        # Value should be a list of script entries
                        if isinstance(value, ast.List):
                            for elt in value.elts:
                                if isinstance(elt, ast.Constant):
                                    # Parse 'name = package.module:function'
                                    entry = elt.value
                                    if '=' in entry:
                                        name, target = entry.split('=', 1)
                                        scripts[name.strip()] = target.strip()

        except Exception as e:
            self.logger.debug(f"Error parsing entry_points AST: {e}")

        return scripts

    def get_package_metadata(self, package_name: str) -> Optional[Dict]:
        """Get package metadata from package.xml"""
        if not AMENT_INDEX_AVAILABLE:
            return None

        try:
            share_dir = Path(get_package_share_directory(package_name))
            package_xml = share_dir / 'package.xml'

            if not package_xml.exists():
                return None

            tree = ET.parse(package_xml)
            root = tree.getroot()

            metadata = {
                'name': root.find('name').text if root.find('name') is not None else package_name,
                'version': root.find('version').text if root.find('version') is not None else 'unknown',
                'description': root.find('description').text if root.find('description') is not None else '',
                'maintainer': root.find('maintainer').text if root.find('maintainer') is not None else '',
                'license': root.find('license').text if root.find('license') is not None else '',
                'dependencies': {
                    'build': [dep.text for dep in root.findall('build_depend')],
                    'exec': [dep.text for dep in root.findall('exec_depend')],
                    'test': [dep.text for dep in root.findall('test_depend')]
                }
            }

            return metadata

        except Exception as e:
            self.logger.warning(f"Failed to get package metadata for {package_name}: {e}")
            return None

    def clear_cache(self):
        """Clear the discovery cache"""
        self.cache.clear()