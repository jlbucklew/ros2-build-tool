"""
Repository management for cloning and managing GitHub driver repos
"""

import logging
import subprocess
import time
import yaml
from pathlib import Path
from typing import Dict, List, Optional


class RepositoryManager:
    """
    Manages repository cloning and vcs tool integration
    """

    def __init__(self, workspace_path: Path, logger: Optional[logging.Logger] = None):
        self.workspace_path = workspace_path
        self.src_path = workspace_path / 'src'
        self.logger = logger or logging.getLogger(__name__)
        self.max_retries = 3
        self.retry_delay = 2  # seconds

    def clone_driver_repos(
        self,
        repos: Dict[str, Dict],
        ros_distro: str
    ) -> Dict:
        """
        Clone driver repositories from GitHub

        Args:
            repos: Dict mapping repo_id to repo config with 'url' and optional 'branches'
            ros_distro: ROS distribution to determine branch

        Returns:
            Dict with 'success' list, 'errors' list, 'warnings' list
        """
        success = []
        errors = []
        warnings = []

        for repo_id, repo_config in repos.items():
            self.logger.info(f"Processing repository: {repo_id}")

            url = repo_config.get('url')
            if not url:
                warnings.append(f"{repo_id}: No URL specified, skipping")
                continue

            # Determine branch to clone
            branch = None
            branches_config = repo_config.get('branches', {})
            if branches_config and ros_distro in branches_config:
                branch = branches_config[ros_distro]
            elif 'branch' in repo_config:
                branch = repo_config['branch']

            # Determine target directory
            target_dir = self._get_target_dir(repo_id, url)

            # Clone repository
            clone_result = self._clone_repo_with_retry(url, target_dir, branch)

            if clone_result['success']:
                success.append(repo_id)
                self.logger.info(f"✓ Cloned {repo_id}")
            else:
                errors.append(f"{repo_id}: {clone_result['error']}")
                self.logger.error(f"✗ Failed to clone {repo_id}: {clone_result['error']}")

        # Generate .repos file for vcs tool
        if success:
            self._generate_repos_file(repos, ros_distro)

        return {
            'success': success,
            'errors': errors,
            'warnings': warnings
        }

    def _get_target_dir(self, repo_id: str, url: str) -> Path:
        """Determine target directory for cloned repo"""
        # Extract repo name from URL
        # e.g., https://github.com/user/repo.git -> repo
        repo_name = url.rstrip('/').split('/')[-1]
        if repo_name.endswith('.git'):
            repo_name = repo_name[:-4]

        return self.src_path / repo_name

    def _clone_repo_with_retry(
        self,
        url: str,
        target_dir: Path,
        branch: Optional[str] = None
    ) -> Dict:
        """
        Clone repository with retry logic

        Returns:
            Dict with 'success' bool and optional 'error' message
        """
        # Skip if already exists
        if target_dir.exists():
            self.logger.info(f"Repository already exists: {target_dir}")
            return {'success': True}

        for attempt in range(self.max_retries):
            try:
                # Build git clone command
                cmd = ['git', 'clone']

                if branch:
                    cmd.extend(['--branch', branch])

                cmd.extend([url, str(target_dir)])

                # Execute clone
                self.logger.debug(f"Executing: {' '.join(cmd)}")
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=300  # 5 minute timeout
                )

                if result.returncode == 0:
                    return {'success': True}
                else:
                    error_msg = result.stderr.strip() or result.stdout.strip()
                    self.logger.warning(
                        f"Clone attempt {attempt + 1}/{self.max_retries} failed: {error_msg}"
                    )

                    # Don't retry on certain errors
                    if 'not found' in error_msg.lower() or 'does not exist' in error_msg.lower():
                        return {'success': False, 'error': error_msg}

                    if attempt < self.max_retries - 1:
                        time.sleep(self.retry_delay * (attempt + 1))  # Exponential backoff

            except subprocess.TimeoutExpired:
                error_msg = f"Clone timed out after 300 seconds"
                self.logger.warning(f"Attempt {attempt + 1}/{self.max_retries}: {error_msg}")
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay * (attempt + 1))

            except FileNotFoundError:
                return {'success': False, 'error': 'git command not found. Please install git.'}

            except Exception as e:
                error_msg = f"Unexpected error: {str(e)}"
                self.logger.warning(f"Attempt {attempt + 1}/{self.max_retries}: {error_msg}")
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay * (attempt + 1))

        return {'success': False, 'error': f'Failed after {self.max_retries} attempts'}

    def _generate_repos_file(self, repos: Dict[str, Dict], ros_distro: str):
        """
        Generate .repos file for vcs tool

        Format:
        repositories:
          repo_name:
            type: git
            url: https://github.com/...
            version: branch_name
        """
        repos_data = {'repositories': {}}

        for repo_id, repo_config in repos.items():
            url = repo_config.get('url')
            if not url:
                continue

            # Extract repo name from URL
            repo_name = url.rstrip('/').split('/')[-1]
            if repo_name.endswith('.git'):
                repo_name = repo_name[:-4]

            # Determine version (branch)
            version = None
            branches_config = repo_config.get('branches', {})
            if branches_config and ros_distro in branches_config:
                version = branches_config[ros_distro]
            elif 'branch' in repo_config:
                version = repo_config['branch']

            repo_entry = {
                'type': 'git',
                'url': url
            }

            if version:
                repo_entry['version'] = version

            repos_data['repositories'][repo_name] = repo_entry

        # Write .repos file
        repos_file = self.workspace_path / 'sources.repos'
        with open(repos_file, 'w') as f:
            yaml.dump(repos_data, f, default_flow_style=False)

        self.logger.info(f"Generated .repos file: {repos_file}")

    def pull_all_repos(self) -> Dict:
        """Pull latest changes for all repositories in src/"""
        success = []
        errors = []

        for repo_dir in self.src_path.iterdir():
            if not repo_dir.is_dir():
                continue

            git_dir = repo_dir / '.git'
            if not git_dir.exists():
                continue

            try:
                result = subprocess.run(
                    ['git', '-C', str(repo_dir), 'pull'],
                    capture_output=True,
                    text=True,
                    timeout=60
                )

                if result.returncode == 0:
                    success.append(repo_dir.name)
                    self.logger.info(f"✓ Pulled {repo_dir.name}")
                else:
                    error_msg = result.stderr.strip()
                    errors.append(f"{repo_dir.name}: {error_msg}")
                    self.logger.error(f"✗ Failed to pull {repo_dir.name}")

            except Exception as e:
                errors.append(f"{repo_dir.name}: {str(e)}")
                self.logger.error(f"✗ Error pulling {repo_dir.name}: {e}")

        return {
            'success': success,
            'errors': errors
        }

    def vcs_import(self, repos_file: Path) -> bool:
        """
        Import repositories using vcs tool

        Args:
            repos_file: Path to .repos file

        Returns:
            True if successful
        """
        if not repos_file.exists():
            self.logger.error(f"Repos file not found: {repos_file}")
            return False

        try:
            # Check if vcs is installed
            check_vcs = subprocess.run(
                ['vcs', '--version'],
                capture_output=True,
                timeout=10
            )

            if check_vcs.returncode != 0:
                self.logger.error(
                    "vcs tool not found. Install with: pip3 install vcstool"
                )
                return False

            # Import repos
            self.logger.info(f"Importing repositories from: {repos_file}")
            result = subprocess.run(
                ['vcs', 'import', str(self.src_path), '--input', str(repos_file)],
                capture_output=True,
                text=True,
                timeout=600  # 10 minute timeout
            )

            if result.returncode == 0:
                self.logger.info("✓ vcs import completed")
                return True
            else:
                self.logger.error(f"vcs import failed: {result.stderr}")
                return False

        except FileNotFoundError:
            self.logger.error(
                "vcs tool not found. Install with: pip3 install vcstool"
            )
            return False
        except Exception as e:
            self.logger.error(f"vcs import error: {e}")
            return False

    def get_repo_status(self) -> List[Dict]:
        """Get status of all git repositories in workspace"""
        repos_status = []

        for repo_dir in self.src_path.iterdir():
            if not repo_dir.is_dir():
                continue

            git_dir = repo_dir / '.git'
            if not git_dir.exists():
                continue

            try:
                # Get current branch
                branch_result = subprocess.run(
                    ['git', '-C', str(repo_dir), 'rev-parse', '--abbrev-ref', 'HEAD'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                branch = branch_result.stdout.strip() if branch_result.returncode == 0 else 'unknown'

                # Get current commit
                commit_result = subprocess.run(
                    ['git', '-C', str(repo_dir), 'rev-parse', '--short', 'HEAD'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                commit = commit_result.stdout.strip() if commit_result.returncode == 0 else 'unknown'

                # Get status
                status_result = subprocess.run(
                    ['git', '-C', str(repo_dir), 'status', '--porcelain'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                clean = len(status_result.stdout.strip()) == 0 if status_result.returncode == 0 else False

                repos_status.append({
                    'name': repo_dir.name,
                    'path': str(repo_dir),
                    'branch': branch,
                    'commit': commit,
                    'clean': clean
                })

            except Exception as e:
                self.logger.error(f"Error getting status for {repo_dir.name}: {e}")
                repos_status.append({
                    'name': repo_dir.name,
                    'path': str(repo_dir),
                    'error': str(e)
                })

        return repos_status