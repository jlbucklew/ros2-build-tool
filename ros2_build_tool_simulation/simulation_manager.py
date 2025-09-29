"""
Simulation Manager for ROS2 Build Tool

Orchestrates Gazebo/Ignition simulation environments for testing robot
configurations before hardware deployment.
"""

import os
import logging
import subprocess
import time
import signal
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum

from ros2_build_tool_core.models import RobotProfile, RobotSpecs
from ros2_build_tool_core.executor import Executor, ErrorType
from ros2_build_tool_core.environment import EnvironmentManager


class SimulationType(Enum):
    """Types of simulation environments."""
    GAZEBO_CLASSIC = "gazebo_classic"
    GAZEBO_IGNITION = "ignition"
    GAZEBO_HARMONIC = "gz_harmonic"


class SimulationComplexity(Enum):
    """Levels of world complexity for progressive testing."""
    EMPTY = "empty"
    SIMPLE_OBSTACLES = "simple_obstacles"
    COMPLEX_ENVIRONMENT = "complex_environment"
    REALISTIC_WORLD = "realistic_world"


@dataclass
class SimulationConfig:
    """Configuration for simulation environment."""
    simulation_type: SimulationType
    world_complexity: SimulationComplexity
    physics_engine: str = "ode"  # ode, bullet, dart
    real_time_factor: float = 1.0
    update_rate: int = 1000
    gui: bool = True
    verbose: bool = False
    record: bool = False
    playback_file: Optional[str] = None


@dataclass
class SimulationStatus:
    """Status of running simulation."""
    running: bool
    pid: Optional[int]
    start_time: Optional[float]
    sim_time: float = 0.0
    real_time: float = 0.0
    real_time_factor: float = 0.0
    paused: bool = False
    world_name: str = ""
    model_count: int = 0
    errors: List[str] = None


class SimulationManager:
    """Manages Gazebo/Ignition simulation environments."""

    def __init__(self,
                 workspace_path: Path,
                 robot_profile: RobotProfile,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize simulation manager.

        Args:
            workspace_path: Path to ROS2 workspace
            robot_profile: Robot configuration profile
            logger: Optional logger instance
        """
        self.workspace_path = Path(workspace_path)
        self.robot_profile = robot_profile
        self.logger = logger or logging.getLogger(__name__)
        self.executor = Executor()
        self.env_manager = EnvironmentManager()

        self.simulation_proc: Optional[subprocess.Popen] = None
        self.bridge_proc: Optional[subprocess.Popen] = None
        self.spawn_proc: Optional[subprocess.Popen] = None

        self._detect_simulation_type()

    def _detect_simulation_type(self) -> SimulationType:
        """Detect available Gazebo version."""
        # Check for Gazebo Harmonic (latest)
        result = self.executor.run_command("which gz", timeout=5)
        if result["returncode"] == 0:
            self.simulation_type = SimulationType.GAZEBO_HARMONIC
            self.logger.info("Detected Gazebo Harmonic")
            return self.simulation_type

        # Check for Ignition Gazebo
        result = self.executor.run_command("which ign", timeout=5)
        if result["returncode"] == 0:
            self.simulation_type = SimulationType.GAZEBO_IGNITION
            self.logger.info("Detected Ignition Gazebo")
            return self.simulation_type

        # Check for Gazebo Classic
        result = self.executor.run_command("which gazebo", timeout=5)
        if result["returncode"] == 0:
            self.simulation_type = SimulationType.GAZEBO_CLASSIC
            self.logger.info("Detected Gazebo Classic")
            return self.simulation_type

        self.logger.warning("No Gazebo installation detected. Simulation features will be limited.")
        self.simulation_type = None
        return None

    def create_simulation_workspace(self) -> Dict[str, Any]:
        """
        Create necessary directories and files for simulation.

        Returns:
            Dictionary with success status and created paths
        """
        try:
            # Create simulation directories
            sim_dir = self.workspace_path / "simulation"
            worlds_dir = sim_dir / "worlds"
            models_dir = sim_dir / "models"
            configs_dir = sim_dir / "configs"

            for directory in [sim_dir, worlds_dir, models_dir, configs_dir]:
                directory.mkdir(parents=True, exist_ok=True)

            self.logger.info(f"Created simulation workspace at {sim_dir}")

            return {
                "success": True,
                "simulation_dir": str(sim_dir),
                "worlds_dir": str(worlds_dir),
                "models_dir": str(models_dir),
                "configs_dir": str(configs_dir)
            }

        except Exception as e:
            self.logger.error(f"Failed to create simulation workspace: {e}")
            return {
                "success": False,
                "error": str(e)
            }

    def launch_simulation(self,
                         config: SimulationConfig,
                         world_file: Optional[Path] = None) -> Dict[str, Any]:
        """
        Launch Gazebo simulation with specified configuration.

        Args:
            config: Simulation configuration
            world_file: Optional world file to load

        Returns:
            Dictionary with launch status and process info
        """
        if not self.simulation_type:
            return {
                "success": False,
                "error": "No Gazebo installation detected"
            }

        try:
            # Build command based on simulation type
            cmd = self._build_launch_command(config, world_file)

            # Set up environment
            env = self.env_manager.get_ros_env()
            env["GAZEBO_MODEL_PATH"] = str(self.workspace_path / "simulation/models")
            env["GAZEBO_RESOURCE_PATH"] = str(self.workspace_path / "simulation")

            # Launch simulation
            self.logger.info(f"Launching simulation: {' '.join(cmd)}")
            self.simulation_proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # Wait for simulation to start
            time.sleep(3)

            if self.simulation_proc.poll() is not None:
                stderr = self.simulation_proc.stderr.read()
                return {
                    "success": False,
                    "error": f"Simulation failed to start: {stderr}"
                }

            self.logger.info(f"Simulation started with PID {self.simulation_proc.pid}")

            return {
                "success": True,
                "pid": self.simulation_proc.pid,
                "simulation_type": self.simulation_type.value,
                "config": config.__dict__
            }

        except Exception as e:
            self.logger.error(f"Failed to launch simulation: {e}")
            return {
                "success": False,
                "error": str(e)
            }

    def _build_launch_command(self,
                             config: SimulationConfig,
                             world_file: Optional[Path]) -> List[str]:
        """Build launch command based on simulation type."""
        cmd = []

        if self.simulation_type == SimulationType.GAZEBO_HARMONIC:
            cmd = ["gz", "sim"]
            if world_file:
                cmd.append(str(world_file))
            if not config.gui:
                cmd.append("-s")  # Server only
            if config.verbose:
                cmd.append("-v4")

        elif self.simulation_type == SimulationType.GAZEBO_IGNITION:
            cmd = ["ign", "gazebo"]
            if world_file:
                cmd.append(str(world_file))
            if not config.gui:
                cmd.append("-s")
            if config.verbose:
                cmd.append("-v4")

        else:  # GAZEBO_CLASSIC
            cmd = ["gazebo"]
            if world_file:
                cmd.append(str(world_file))
            if not config.gui:
                cmd.append("--headless")
            if config.verbose:
                cmd.append("--verbose")
            cmd.extend([
                f"--physics={config.physics_engine}",
                f"-u" if config.paused else ""
            ])

        return [c for c in cmd if c]  # Remove empty strings

    def spawn_robot(self,
                   urdf_path: Path,
                   robot_name: str = "robot",
                   position: Tuple[float, float, float] = (0, 0, 0),
                   orientation: Tuple[float, float, float, float] = (0, 0, 0, 1)) -> Dict[str, Any]:
        """
        Spawn robot model in simulation.

        Args:
            urdf_path: Path to URDF/SDF file
            robot_name: Name for spawned robot
            position: (x, y, z) position
            orientation: (x, y, z, w) quaternion

        Returns:
            Dictionary with spawn status
        """
        try:
            # Build spawn command
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                cmd = [
                    "ros2", "run", "ros_gz_sim", "create",
                    "-name", robot_name,
                    "-file", str(urdf_path),
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2])
                ]
            else:  # Classic
                cmd = [
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", robot_name,
                    "-file", str(urdf_path),
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2])
                ]

            # Execute spawn command
            result = self.executor.run_command(
                " ".join(cmd),
                timeout=30,
                env=self.env_manager.get_ros_env()
            )

            if result["returncode"] != 0:
                return {
                    "success": False,
                    "error": result.get("stderr", "Failed to spawn robot")
                }

            self.logger.info(f"Successfully spawned robot '{robot_name}' in simulation")

            return {
                "success": True,
                "robot_name": robot_name,
                "position": position,
                "orientation": orientation
            }

        except Exception as e:
            self.logger.error(f"Failed to spawn robot: {e}")
            return {
                "success": False,
                "error": str(e)
            }

    def get_simulation_status(self) -> SimulationStatus:
        """Get current simulation status."""
        if not self.simulation_proc:
            return SimulationStatus(running=False, pid=None, start_time=None)

        running = self.simulation_proc.poll() is None

        status = SimulationStatus(
            running=running,
            pid=self.simulation_proc.pid if running else None,
            start_time=time.time() if running else None
        )

        if running:
            # Get additional status via service calls
            self._update_simulation_status(status)

        return status

    def _update_simulation_status(self, status: SimulationStatus):
        """Update status with runtime information."""
        try:
            # Get simulation stats via ROS2 services
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                # Use gz/ign services
                result = self.executor.run_command(
                    "gz stats -p",
                    timeout=5,
                    env=self.env_manager.get_ros_env()
                )
                if result["returncode"] == 0:
                    # Parse stats output
                    lines = result["stdout"].strip().split('\n')
                    for line in lines:
                        if "Sim time:" in line:
                            status.sim_time = float(line.split(':')[1].strip())
                        elif "Real time:" in line:
                            status.real_time = float(line.split(':')[1].strip())
                        elif "Real time factor:" in line:
                            status.real_time_factor = float(line.split(':')[1].strip())

        except Exception as e:
            self.logger.debug(f"Could not get simulation stats: {e}")

    def pause_simulation(self) -> bool:
        """Pause running simulation."""
        try:
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                cmd = "gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'pause: true'"
            else:
                cmd = "ros2 service call /gazebo/pause_physics std_srvs/srv/Empty"

            result = self.executor.run_command(cmd, timeout=5, env=self.env_manager.get_ros_env())
            return result["returncode"] == 0

        except Exception as e:
            self.logger.error(f"Failed to pause simulation: {e}")
            return False

    def resume_simulation(self) -> bool:
        """Resume paused simulation."""
        try:
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                cmd = "gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'pause: false'"
            else:
                cmd = "ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty"

            result = self.executor.run_command(cmd, timeout=5, env=self.env_manager.get_ros_env())
            return result["returncode"] == 0

        except Exception as e:
            self.logger.error(f"Failed to resume simulation: {e}")
            return False

    def reset_simulation(self) -> bool:
        """Reset simulation to initial state."""
        try:
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                cmd = "gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'reset: {all: true}'"
            else:
                cmd = "ros2 service call /gazebo/reset_simulation std_srvs/srv/Empty"

            result = self.executor.run_command(cmd, timeout=5, env=self.env_manager.get_ros_env())
            return result["returncode"] == 0

        except Exception as e:
            self.logger.error(f"Failed to reset simulation: {e}")
            return False

    def stop_simulation(self, timeout: int = 10) -> bool:
        """
        Stop running simulation gracefully.

        Args:
            timeout: Seconds to wait for graceful shutdown

        Returns:
            True if successfully stopped
        """
        if not self.simulation_proc:
            return True

        try:
            # Send SIGTERM for graceful shutdown
            self.simulation_proc.terminate()

            # Wait for process to end
            try:
                self.simulation_proc.wait(timeout=timeout)
                self.logger.info("Simulation stopped gracefully")
                return True
            except subprocess.TimeoutExpired:
                # Force kill if not stopped
                self.simulation_proc.kill()
                self.simulation_proc.wait(timeout=5)
                self.logger.warning("Simulation force killed after timeout")
                return True

        except Exception as e:
            self.logger.error(f"Failed to stop simulation: {e}")
            return False
        finally:
            self.simulation_proc = None

    def validate_robot_in_simulation(self) -> Dict[str, Any]:
        """
        Validate that robot is properly loaded and functioning in simulation.

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "success": True,
            "checks": {},
            "warnings": [],
            "errors": []
        }

        try:
            # Check if simulation is running
            status = self.get_simulation_status()
            validation_results["checks"]["simulation_running"] = status.running

            if not status.running:
                validation_results["errors"].append("Simulation is not running")
                validation_results["success"] = False
                return validation_results

            # Check for robot model in simulation
            if self.simulation_type in [SimulationType.GAZEBO_HARMONIC, SimulationType.GAZEBO_IGNITION]:
                result = self.executor.run_command(
                    "gz model --list",
                    timeout=5,
                    env=self.env_manager.get_ros_env()
                )
            else:
                result = self.executor.run_command(
                    "ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList",
                    timeout=5,
                    env=self.env_manager.get_ros_env()
                )

            validation_results["checks"]["robot_spawned"] = result["returncode"] == 0

            # Check ROS2 topics
            result = self.executor.run_command(
                "ros2 topic list",
                timeout=5,
                env=self.env_manager.get_ros_env()
            )

            if result["returncode"] == 0:
                topics = result["stdout"].strip().split('\n')

                # Check for essential topics
                essential_topics = ["/tf", "/tf_static", "/joint_states", "/robot_description"]
                for topic in essential_topics:
                    if topic in topics:
                        validation_results["checks"][f"topic_{topic}"] = True
                    else:
                        validation_results["warnings"].append(f"Missing topic: {topic}")
                        validation_results["checks"][f"topic_{topic}"] = False

                # Check for sensor topics based on robot profile
                if self.robot_profile.hardware_manifest:
                    for component in self.robot_profile.hardware_manifest.hardware_components:
                        if component.component_type == "lidar":
                            if "/scan" not in topics:
                                validation_results["warnings"].append("Lidar configured but /scan topic not found")
                        elif component.component_type == "camera":
                            if not any("image" in t for t in topics):
                                validation_results["warnings"].append("Camera configured but no image topics found")

            # Check for controllers if ros2_control is configured
            if self.robot_profile.enable_ros2_control:
                result = self.executor.run_command(
                    "ros2 control list_controllers",
                    timeout=5,
                    env=self.env_manager.get_ros_env()
                )
                validation_results["checks"]["controllers_loaded"] = result["returncode"] == 0

            # Overall success if no errors
            validation_results["success"] = len(validation_results["errors"]) == 0

        except Exception as e:
            validation_results["errors"].append(f"Validation failed: {str(e)}")
            validation_results["success"] = False

        return validation_results

    def capture_simulation_metrics(self, duration: int = 10) -> Dict[str, Any]:
        """
        Capture performance metrics from running simulation.

        Args:
            duration: How long to capture metrics (seconds)

        Returns:
            Dictionary with performance metrics
        """
        metrics = {
            "real_time_factors": [],
            "cpu_usage": [],
            "memory_usage": [],
            "topic_frequencies": {},
            "duration": duration
        }

        try:
            start_time = time.time()

            while time.time() - start_time < duration:
                # Get real-time factor
                status = self.get_simulation_status()
                if status.real_time_factor > 0:
                    metrics["real_time_factors"].append(status.real_time_factor)

                # Get system resources
                if self.simulation_proc:
                    # Get CPU and memory for simulation process
                    result = self.executor.run_command(
                        f"ps -p {self.simulation_proc.pid} -o %cpu,rss --no-headers",
                        timeout=1
                    )
                    if result["returncode"] == 0:
                        parts = result["stdout"].strip().split()
                        if len(parts) >= 2:
                            metrics["cpu_usage"].append(float(parts[0]))
                            metrics["memory_usage"].append(int(parts[1]) / 1024)  # Convert to MB

                time.sleep(1)

            # Calculate averages
            if metrics["real_time_factors"]:
                metrics["avg_real_time_factor"] = sum(metrics["real_time_factors"]) / len(metrics["real_time_factors"])
            if metrics["cpu_usage"]:
                metrics["avg_cpu_usage"] = sum(metrics["cpu_usage"]) / len(metrics["cpu_usage"])
            if metrics["memory_usage"]:
                metrics["avg_memory_mb"] = sum(metrics["memory_usage"]) / len(metrics["memory_usage"])

            # Get topic frequencies
            result = self.executor.run_command(
                "ros2 topic list",
                timeout=5,
                env=self.env_manager.get_ros_env()
            )

            if result["returncode"] == 0:
                topics = result["stdout"].strip().split('\n')
                for topic in topics[:10]:  # Check first 10 topics
                    freq_result = self.executor.run_command(
                        f"timeout 2 ros2 topic hz {topic}",
                        timeout=3,
                        env=self.env_manager.get_ros_env()
                    )
                    if freq_result["returncode"] == 0 and "average rate:" in freq_result["stdout"]:
                        # Parse frequency from output
                        for line in freq_result["stdout"].split('\n'):
                            if "average rate:" in line:
                                try:
                                    freq = float(line.split(':')[1].strip().split()[0])
                                    metrics["topic_frequencies"][topic] = freq
                                except:
                                    pass

        except Exception as e:
            self.logger.error(f"Failed to capture metrics: {e}")
            metrics["error"] = str(e)

        return metrics

    def cleanup(self):
        """Clean up simulation resources."""
        self.stop_simulation()

        # Clean up any temporary files
        sim_dir = self.workspace_path / "simulation"
        if sim_dir.exists():
            temp_files = list(sim_dir.glob("*.tmp"))
            for f in temp_files:
                try:
                    f.unlink()
                except:
                    pass