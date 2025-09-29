"""
Command execution with retry and error classification
"""

import time
import subprocess
import logging
from enum import Enum
from pathlib import Path
from typing import Dict, List, Union, Optional


class Executor:
    """Command execution with retry and error classification"""

    class ErrorType(Enum):
        NETWORK = "network"
        PERMISSION = "permission"
        MISSING_DEP = "missing_dependency"
        BUILD = "build"
        TIMEOUT = "timeout"
        UNKNOWN = "unknown"

    def __init__(self, env: Dict[str, str], logger: logging.Logger):
        self.env = env
        self.logger = logger
        self.history = []

    def run(self,
            cmd: Union[str, List[str]],
            cwd: Optional[Path] = None,
            timeout: int = 300,
            retries: int = 3,
            check: bool = True) -> Dict:
        """Execute command with retry logic"""

        if isinstance(cmd, str):
            cmd = cmd.split()

        for attempt in range(retries):
            try:
                self.logger.debug(f"Executing ({attempt+1}/{retries}): {' '.join(cmd)}")

                result = subprocess.run(
                    cmd,
                    cwd=str(cwd) if cwd else None,
                    env=self.env,
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                    check=check
                )

                output = {
                    'success': True,
                    'returncode': result.returncode,
                    'stdout': result.stdout,
                    'stderr': result.stderr,
                    'command': ' '.join(cmd),
                    'attempts': attempt + 1
                }

                self.history.append(output)
                return output

            except subprocess.TimeoutExpired:
                error_type = self.ErrorType.TIMEOUT
                error_msg = f"Command timed out after {timeout}s"

            except subprocess.CalledProcessError as e:
                error_type = self._classify_error(e.stderr)
                error_msg = self._get_error_message(error_type, e.stderr)

                if not check:
                    return {
                        'success': False,
                        'returncode': e.returncode,
                        'stdout': e.stdout,
                        'stderr': e.stderr,
                        'command': ' '.join(cmd),
                        'error_type': error_type.value,
                        'error_msg': error_msg
                    }

            except FileNotFoundError:
                error_type = self.ErrorType.MISSING_DEP
                error_msg = f"Command not found: {cmd[0]}"

            if attempt < retries - 1:
                wait = 2 ** attempt  # Exponential backoff
                self.logger.warning(f"{error_msg}. Retrying in {wait}s...")
                time.sleep(wait)
            else:
                raise RuntimeError(f"{error_msg} after {retries} attempts")

    def _classify_error(self, stderr: str) -> ErrorType:
        """Classify error type from stderr"""
        patterns = {
            self.ErrorType.NETWORK: [
                'could not resolve', 'connection timed out',
                'network is unreachable', 'connection refused'
            ],
            self.ErrorType.PERMISSION: [
                'permission denied', 'access denied', 'operation not permitted'
            ],
            self.ErrorType.MISSING_DEP: [
                'no module named', 'package not found', 'cannot find'
            ],
            self.ErrorType.BUILD: [
                'cmake error', 'compilation failed', 'undefined reference'
            ]
        }

        stderr_lower = stderr.lower()
        for error_type, keywords in patterns.items():
            if any(kw in stderr_lower for kw in keywords):
                return error_type

        return self.ErrorType.UNKNOWN

    def _get_error_message(self, error_type: ErrorType, stderr: str) -> str:
        """Get user-friendly error message"""
        messages = {
            self.ErrorType.NETWORK: "Network error - check connection",
            self.ErrorType.PERMISSION: "Permission denied - may need sudo",
            self.ErrorType.MISSING_DEP: "Missing dependency - install required packages",
            self.ErrorType.BUILD: "Build error - check logs for details",
            self.ErrorType.TIMEOUT: "Command timed out",
            self.ErrorType.UNKNOWN: "Unknown error occurred"
        }
        return messages.get(error_type, "Error occurred")