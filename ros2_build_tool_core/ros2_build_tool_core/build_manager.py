"""
Build management for colcon build and package discovery
"""

import logging
import subprocess
import re
from pathlib import Path
from typing import Dict, List, Optional


class BuildManager:
    """
    Manages workspace building with colcon and executable discovery
    """

    def __init__(self, workspace_path: Path, logger: Optional[logging.Logger] = None):
        self.workspace_path = workspace_path
        self.logger = logger or logging.getLogger(__name__)
        self.build_log = []

    def build_workspace(
        self,
        packages: Optional[List[str]] = None,
        build_type: str = 'Release',
        parallel_workers: Optional[int] = None
    ) -> Dict:
        """
        Build workspace with colcon

        Args:
            packages: Specific packages to build (None = all)
            build_type: CMake build type (Debug, Release, RelWithDebInfo)
            parallel_workers: Number of parallel build jobs

        Returns:
            Dict with 'success' bool, 'log' string, 'errors' list
        """
        errors = []
        self.build_log = []

        try:
            # Build colcon command
            cmd = [
                'colcon', 'build',
                '--cmake-args', f'-DCMAKE_BUILD_TYPE={build_type}',
                '--symlink-install'
            ]

            if packages:
                cmd.extend(['--packages-select'] + packages)

            if parallel_workers:
                cmd.extend(['--parallel-workers', str(parallel_workers)])

            self.logger.info(f"Building workspace: {' '.join(cmd)}")

            # Execute build
            process = subprocess.Popen(
                cmd,
                cwd=self.workspace_path,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            # Stream output
            for line in process.stdout:
                line = line.rstrip()
                self.build_log.append(line)
                self.logger.debug(line)

            process.wait()

            if process.returncode == 0:
                self.logger.info("✓ Workspace built successfully")
                return {
                    'success': True,
                    'log': '\n'.join(self.build_log),
                    'errors': errors
                }
            else:
                # Parse errors from log
                errors = self._parse_build_errors(self.build_log)
                self.logger.error("✗ Workspace build failed")
                return {
                    'success': False,
                    'log': '\n'.join(self.build_log),
                    'errors': errors
                }

        except FileNotFoundError:
            error_msg = 'colcon command not found. Please install colcon: pip3 install colcon-common-extensions'
            self.logger.error(error_msg)
            errors.append(error_msg)
            return {
                'success': False,
                'log': '\n'.join(self.build_log),
                'errors': errors
            }

        except Exception as e:
            error_msg = f'Build error: {str(e)}'
            self.logger.error(error_msg, exc_info=True)
            errors.append(error_msg)
            return {
                'success': False,
                'log': '\n'.join(self.build_log),
                'errors': errors
            }

    def _parse_build_errors(self, log_lines: List[str]) -> List[str]:
        """Parse build log for error messages"""
        errors = []
        error_pattern = re.compile(r'error:|Error:|ERROR:|CMake Error|fatal error:', re.IGNORECASE)

        for line in log_lines:
            if error_pattern.search(line):
                errors.append(line.strip())

        return errors[:20]  # Limit to first 20 errors

    def discover_executables(self) -> Dict[str, List[str]]:
        """
        Discover executables in built packages

        Returns:
            Dict mapping package_name to list of executable names
        """
        executables = {}
        install_path = self.workspace_path / 'install'

        if not install_path.exists():
            self.logger.warning("Install directory not found, cannot discover executables")
            return executables

        try:
            # Iterate through installed packages
            for package_dir in install_path.iterdir():
                if not package_dir.is_dir():
                    continue

                package_name = package_dir.name

                # Check lib directory for executables
                lib_dir = package_dir / 'lib' / package_name
                if lib_dir.exists():
                    execs = []
                    for item in lib_dir.iterdir():
                        # Check if file is executable (on Unix) or exists (on Windows)
                        if item.is_file():
                            # Skip .so/.dll libraries, only include executables
                            if not any(item.name.endswith(ext) for ext in ['.so', '.dll', '.dylib', '.a']):
                                execs.append(item.name)

                    if execs:
                        executables[package_name] = execs
                        self.logger.debug(f"Found executables in {package_name}: {execs}")

        except Exception as e:
            self.logger.error(f"Error discovering executables: {e}")

        return executables

    def clean_workspace(self, keep_src: bool = True) -> bool:
        """
        Clean workspace build artifacts

        Args:
            keep_src: Keep source directory (default: True)

        Returns:
            True if successful
        """
        try:
            import shutil

            dirs_to_clean = ['build', 'install', 'log']

            if not keep_src:
                dirs_to_clean.append('src')

            for dir_name in dirs_to_clean:
                dir_path = self.workspace_path / dir_name
                if dir_path.exists():
                    self.logger.info(f"Removing: {dir_path}")
                    shutil.rmtree(dir_path)

            self.logger.info("✓ Workspace cleaned")
            return True

        except Exception as e:
            self.logger.error(f"Failed to clean workspace: {e}")
            return False

    def test_workspace(self, packages: Optional[List[str]] = None) -> Dict:
        """
        Run tests with colcon test

        Args:
            packages: Specific packages to test (None = all)

        Returns:
            Dict with 'success' bool, 'log' string, 'errors' list
        """
        errors = []
        test_log = []

        try:
            # Build colcon test command
            cmd = ['colcon', 'test']

            if packages:
                cmd.extend(['--packages-select'] + packages)

            self.logger.info(f"Testing workspace: {' '.join(cmd)}")

            # Execute tests
            result = subprocess.run(
                cmd,
                cwd=self.workspace_path,
                capture_output=True,
                text=True,
                timeout=600  # 10 minute timeout
            )

            test_log = result.stdout.split('\n')

            # Get test results
            result_cmd = ['colcon', 'test-result', '--all']
            result_output = subprocess.run(
                result_cmd,
                cwd=self.workspace_path,
                capture_output=True,
                text=True,
                timeout=30
            )

            test_log.extend(result_output.stdout.split('\n'))

            success = result.returncode == 0

            if success:
                self.logger.info("✓ Tests passed")
            else:
                self.logger.error("✗ Tests failed")
                errors = self._parse_test_errors(test_log)

            return {
                'success': success,
                'log': '\n'.join(test_log),
                'errors': errors
            }

        except FileNotFoundError:
            error_msg = 'colcon command not found'
            self.logger.error(error_msg)
            errors.append(error_msg)
            return {
                'success': False,
                'log': '\n'.join(test_log),
                'errors': errors
            }

        except subprocess.TimeoutExpired:
            error_msg = 'Tests timed out after 600 seconds'
            self.logger.error(error_msg)
            errors.append(error_msg)
            return {
                'success': False,
                'log': '\n'.join(test_log),
                'errors': errors
            }

        except Exception as e:
            error_msg = f'Test error: {str(e)}'
            self.logger.error(error_msg, exc_info=True)
            errors.append(error_msg)
            return {
                'success': False,
                'log': '\n'.join(test_log),
                'errors': errors
            }

    def _parse_test_errors(self, log_lines: List[str]) -> List[str]:
        """Parse test log for error messages"""
        errors = []
        error_pattern = re.compile(r'failed|FAILED|ERROR', re.IGNORECASE)

        for line in log_lines:
            if error_pattern.search(line):
                errors.append(line.strip())

        return errors[:20]  # Limit to first 20 errors

    def get_package_list(self) -> List[str]:
        """Get list of packages in workspace"""
        packages = []
        src_path = self.workspace_path / 'src'

        if not src_path.exists():
            return packages

        try:
            result = subprocess.run(
                ['colcon', 'list', '-n'],
                cwd=self.workspace_path,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                packages = [line.strip() for line in result.stdout.split('\n') if line.strip()]

        except Exception as e:
            self.logger.error(f"Error getting package list: {e}")

        return packages

    def get_build_status(self) -> Dict:
        """Get build status information"""
        status = {
            'workspace_path': str(self.workspace_path),
            'build_exists': (self.workspace_path / 'build').exists(),
            'install_exists': (self.workspace_path / 'install').exists(),
            'packages': self.get_package_list(),
            'executables': self.discover_executables()
        }

        return status