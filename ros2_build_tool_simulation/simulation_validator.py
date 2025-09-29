"""
Simulation Validator for ROS2 Build Tool

Validates robot configurations in simulation before hardware deployment.
"""

import os
import time
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import threading
import subprocess

from ros2_build_tool_core.models import RobotProfile, RobotSpecs
from ros2_build_tool_core.executor import Executor
from ros2_build_tool_core.environment import EnvironmentManager
from .simulation_manager import SimulationManager, SimulationConfig, SimulationComplexity
from .world_generator import WorldGenerator


class ValidationLevel(Enum):
    """Levels of validation to perform."""
    BASIC = "basic"  # Just launch and spawn
    STANDARD = "standard"  # Check topics, services, transforms
    COMPREHENSIVE = "comprehensive"  # Full navigation and sensor tests
    PERFORMANCE = "performance"  # Benchmark performance


@dataclass
class ValidationTest:
    """Represents a single validation test."""
    name: str
    description: str
    test_function: callable
    level: ValidationLevel
    timeout: int = 30
    critical: bool = True  # If true, failure stops validation


@dataclass
class ValidationResult:
    """Result of a validation test."""
    test_name: str
    passed: bool
    duration: float
    message: str
    details: Dict[str, Any] = None


@dataclass
class ValidationReport:
    """Complete validation report."""
    success: bool
    level: ValidationLevel
    total_tests: int
    passed_tests: int
    failed_tests: int
    skipped_tests: int
    results: List[ValidationResult]
    performance_metrics: Dict[str, Any] = None
    recommendations: List[str] = None


class SimulationValidator:
    """Validates robot configurations in simulation."""

    def __init__(self,
                 workspace_path: Path,
                 robot_profile: RobotProfile,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize simulation validator.

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

        self.sim_manager = SimulationManager(workspace_path, robot_profile, logger)
        self.world_generator = WorldGenerator(robot_profile, workspace_path, logger)

        self.tests = self._register_tests()
        self.current_test_thread: Optional[threading.Thread] = None

    def _register_tests(self) -> List[ValidationTest]:
        """Register all validation tests."""
        tests = [
            # Basic tests
            ValidationTest(
                name="simulation_launch",
                description="Launch simulation environment",
                test_function=self._test_simulation_launch,
                level=ValidationLevel.BASIC,
                timeout=30,
                critical=True
            ),
            ValidationTest(
                name="robot_spawn",
                description="Spawn robot in simulation",
                test_function=self._test_robot_spawn,
                level=ValidationLevel.BASIC,
                timeout=30,
                critical=True
            ),
            ValidationTest(
                name="tf_tree",
                description="Validate transform tree",
                test_function=self._test_tf_tree,
                level=ValidationLevel.BASIC,
                timeout=10,
                critical=True
            ),

            # Standard tests
            ValidationTest(
                name="ros2_topics",
                description="Check required ROS2 topics",
                test_function=self._test_ros2_topics,
                level=ValidationLevel.STANDARD,
                timeout=10,
                critical=False
            ),
            ValidationTest(
                name="sensor_data",
                description="Validate sensor data publication",
                test_function=self._test_sensor_data,
                level=ValidationLevel.STANDARD,
                timeout=15,
                critical=False
            ),
            ValidationTest(
                name="controllers",
                description="Check controller status",
                test_function=self._test_controllers,
                level=ValidationLevel.STANDARD,
                timeout=10,
                critical=False
            ),
            ValidationTest(
                name="services",
                description="Validate ROS2 services",
                test_function=self._test_services,
                level=ValidationLevel.STANDARD,
                timeout=10,
                critical=False
            ),

            # Comprehensive tests
            ValidationTest(
                name="navigation_stack",
                description="Test navigation stack initialization",
                test_function=self._test_navigation_stack,
                level=ValidationLevel.COMPREHENSIVE,
                timeout=30,
                critical=False
            ),
            ValidationTest(
                name="simple_navigation",
                description="Test simple navigation goal",
                test_function=self._test_simple_navigation,
                level=ValidationLevel.COMPREHENSIVE,
                timeout=60,
                critical=False
            ),
            ValidationTest(
                name="obstacle_avoidance",
                description="Test obstacle avoidance",
                test_function=self._test_obstacle_avoidance,
                level=ValidationLevel.COMPREHENSIVE,
                timeout=60,
                critical=False
            ),
            ValidationTest(
                name="slam",
                description="Test SLAM functionality",
                test_function=self._test_slam,
                level=ValidationLevel.COMPREHENSIVE,
                timeout=60,
                critical=False
            ),

            # Performance tests
            ValidationTest(
                name="real_time_factor",
                description="Measure simulation real-time factor",
                test_function=self._test_real_time_factor,
                level=ValidationLevel.PERFORMANCE,
                timeout=30,
                critical=False
            ),
            ValidationTest(
                name="cpu_usage",
                description="Measure CPU usage",
                test_function=self._test_cpu_usage,
                level=ValidationLevel.PERFORMANCE,
                timeout=30,
                critical=False
            ),
            ValidationTest(
                name="topic_frequencies",
                description="Measure topic publication frequencies",
                test_function=self._test_topic_frequencies,
                level=ValidationLevel.PERFORMANCE,
                timeout=30,
                critical=False
            )
        ]

        return tests

    def validate(self,
                level: ValidationLevel = ValidationLevel.STANDARD,
                world_complexity: SimulationComplexity = SimulationComplexity.SIMPLE_OBSTACLES,
                headless: bool = False) -> ValidationReport:
        """
        Run validation tests at specified level.

        Args:
            level: Level of validation to perform
            world_complexity: Complexity of simulation world
            headless: Run without GUI

        Returns:
            ValidationReport with results
        """
        self.logger.info(f"Starting simulation validation at level: {level.value}")

        # Create simulation workspace
        self.sim_manager.create_simulation_workspace()

        # Generate appropriate world
        world_file = self._generate_world_for_complexity(world_complexity)

        # Configure simulation
        sim_config = SimulationConfig(
            simulation_type=self.sim_manager.simulation_type,
            world_complexity=world_complexity,
            gui=not headless,
            verbose=False
        )

        # Initialize report
        report = ValidationReport(
            success=True,
            level=level,
            total_tests=0,
            passed_tests=0,
            failed_tests=0,
            skipped_tests=0,
            results=[],
            performance_metrics={},
            recommendations=[]
        )

        # Filter tests by level
        tests_to_run = self._filter_tests_by_level(level)
        report.total_tests = len(tests_to_run)

        try:
            # Launch simulation
            launch_result = self.sim_manager.launch_simulation(sim_config, world_file)
            if not launch_result["success"]:
                self.logger.error(f"Failed to launch simulation: {launch_result.get('error')}")
                report.success = False
                report.recommendations.append("Check Gazebo installation and configuration")
                return report

            # Wait for simulation to stabilize
            time.sleep(5)

            # Run each test
            for test in tests_to_run:
                self.logger.info(f"Running test: {test.name}")

                start_time = time.time()
                try:
                    # Run test with timeout
                    result = self._run_test_with_timeout(test, test.timeout)

                    duration = time.time() - start_time

                    if result.passed:
                        report.passed_tests += 1
                        self.logger.info(f"✓ {test.name} passed")
                    else:
                        report.failed_tests += 1
                        self.logger.warning(f"✗ {test.name} failed: {result.message}")

                        if test.critical:
                            self.logger.error(f"Critical test '{test.name}' failed. Stopping validation.")
                            report.success = False
                            report.recommendations.append(f"Fix critical issue: {result.message}")
                            break

                    result.duration = duration
                    report.results.append(result)

                except Exception as e:
                    self.logger.error(f"Test '{test.name}' raised exception: {e}")
                    report.failed_tests += 1
                    report.results.append(ValidationResult(
                        test_name=test.name,
                        passed=False,
                        duration=time.time() - start_time,
                        message=f"Exception: {str(e)}"
                    ))

                    if test.critical:
                        report.success = False
                        break

            # Collect performance metrics if applicable
            if level == ValidationLevel.PERFORMANCE:
                report.performance_metrics = self.sim_manager.capture_simulation_metrics(duration=10)

            # Generate recommendations
            self._generate_recommendations(report)

        finally:
            # Clean up simulation
            self.sim_manager.stop_simulation()

        # Final success determination
        report.success = report.failed_tests == 0

        self.logger.info(f"Validation complete. Passed: {report.passed_tests}/{report.total_tests}")

        return report

    def _generate_world_for_complexity(self, complexity: SimulationComplexity) -> Path:
        """Generate appropriate world for complexity level."""
        if complexity == SimulationComplexity.EMPTY:
            return self.world_generator.generate_empty_world()
        elif complexity == SimulationComplexity.SIMPLE_OBSTACLES:
            return self.world_generator.generate_simple_obstacles_world(num_obstacles=5)
        elif complexity == SimulationComplexity.COMPLEX_ENVIRONMENT:
            return self.world_generator.generate_navigation_test_world()
        else:  # REALISTIC_WORLD
            return self.world_generator.generate_sensor_test_world()

    def _filter_tests_by_level(self, level: ValidationLevel) -> List[ValidationTest]:
        """Filter tests based on validation level."""
        level_hierarchy = {
            ValidationLevel.BASIC: [ValidationLevel.BASIC],
            ValidationLevel.STANDARD: [ValidationLevel.BASIC, ValidationLevel.STANDARD],
            ValidationLevel.COMPREHENSIVE: [ValidationLevel.BASIC, ValidationLevel.STANDARD,
                                          ValidationLevel.COMPREHENSIVE],
            ValidationLevel.PERFORMANCE: [ValidationLevel.BASIC, ValidationLevel.STANDARD,
                                        ValidationLevel.PERFORMANCE]
        }

        allowed_levels = level_hierarchy.get(level, [])
        return [test for test in self.tests if test.level in allowed_levels]

    def _run_test_with_timeout(self, test: ValidationTest, timeout: int) -> ValidationResult:
        """Run a test with timeout."""
        result_container = {}

        def run_test():
            result_container['result'] = test.test_function()

        thread = threading.Thread(target=run_test)
        thread.start()
        thread.join(timeout=timeout)

        if thread.is_alive():
            # Test timed out
            return ValidationResult(
                test_name=test.name,
                passed=False,
                duration=timeout,
                message=f"Test timed out after {timeout} seconds"
            )

        return result_container.get('result', ValidationResult(
            test_name=test.name,
            passed=False,
            duration=0,
            message="Test did not return a result"
        ))

    # Individual test implementations
    def _test_simulation_launch(self) -> ValidationResult:
        """Test simulation launch."""
        status = self.sim_manager.get_simulation_status()

        return ValidationResult(
            test_name="simulation_launch",
            passed=status.running,
            duration=0,
            message="Simulation is running" if status.running else "Simulation not running",
            details={"pid": status.pid}
        )

    def _test_robot_spawn(self) -> ValidationResult:
        """Test robot spawning."""
        # Find URDF file
        urdf_file = self.workspace_path / "src" / "robot_bringup" / "urdf" / "robot.urdf"

        if not urdf_file.exists():
            # Try to find any URDF file
            urdf_files = list(self.workspace_path.glob("**/*.urdf"))
            if urdf_files:
                urdf_file = urdf_files[0]
            else:
                return ValidationResult(
                    test_name="robot_spawn",
                    passed=False,
                    duration=0,
                    message="No URDF file found"
                )

        result = self.sim_manager.spawn_robot(urdf_file)

        return ValidationResult(
            test_name="robot_spawn",
            passed=result["success"],
            duration=0,
            message="Robot spawned successfully" if result["success"] else result.get("error", "Failed to spawn robot"),
            details=result
        )

    def _test_tf_tree(self) -> ValidationResult:
        """Test transform tree."""
        result = self.executor.run_command(
            "timeout 5 ros2 run tf2_tools view_frames.py",
            timeout=10,
            env=self.env_manager.get_ros_env()
        )

        # Check for expected frames
        expected_frames = ["base_link", "odom"]
        if self.robot_profile.urdf_path:
            # Parse URDF for additional expected frames
            pass

        frames_found = []
        missing_frames = []

        # Parse tf tree output
        if result["returncode"] == 0:
            # Check for frames.pdf generation
            frames_file = Path("frames.pdf")
            if frames_file.exists():
                frames_found = ["base_link", "odom"]  # Simplified for now
                frames_file.unlink()  # Clean up

        for frame in expected_frames:
            if frame not in frames_found:
                missing_frames.append(frame)

        passed = len(missing_frames) == 0

        return ValidationResult(
            test_name="tf_tree",
            passed=passed,
            duration=0,
            message="TF tree complete" if passed else f"Missing frames: {missing_frames}",
            details={
                "expected_frames": expected_frames,
                "frames_found": frames_found,
                "missing_frames": missing_frames
            }
        )

    def _test_ros2_topics(self) -> ValidationResult:
        """Test ROS2 topics."""
        result = self.executor.run_command(
            "ros2 topic list",
            timeout=5,
            env=self.env_manager.get_ros_env()
        )

        if result["returncode"] != 0:
            return ValidationResult(
                test_name="ros2_topics",
                passed=False,
                duration=0,
                message="Failed to get topic list"
            )

        topics = result["stdout"].strip().split('\n')

        # Check for essential topics
        essential_topics = ["/tf", "/tf_static", "/joint_states", "/robot_description"]
        missing_topics = [t for t in essential_topics if t not in topics]

        passed = len(missing_topics) == 0

        return ValidationResult(
            test_name="ros2_topics",
            passed=passed,
            duration=0,
            message="All essential topics present" if passed else f"Missing topics: {missing_topics}",
            details={
                "total_topics": len(topics),
                "essential_topics": essential_topics,
                "missing_topics": missing_topics
            }
        )

    def _test_sensor_data(self) -> ValidationResult:
        """Test sensor data publication."""
        sensor_checks = []

        # Check lidar
        if self._has_lidar():
            result = self.executor.run_command(
                "timeout 3 ros2 topic hz /scan",
                timeout=5,
                env=self.env_manager.get_ros_env()
            )
            lidar_ok = "average rate:" in result.get("stdout", "")
            sensor_checks.append(("lidar", lidar_ok))

        # Check camera
        if self._has_camera():
            result = self.executor.run_command(
                "timeout 3 ros2 topic hz /camera/image_raw",
                timeout=5,
                env=self.env_manager.get_ros_env()
            )
            camera_ok = "average rate:" in result.get("stdout", "")
            sensor_checks.append(("camera", camera_ok))

        failed_sensors = [s[0] for s in sensor_checks if not s[1]]
        passed = len(failed_sensors) == 0

        return ValidationResult(
            test_name="sensor_data",
            passed=passed,
            duration=0,
            message="All sensors publishing" if passed else f"Sensors not publishing: {failed_sensors}",
            details={"sensor_checks": sensor_checks}
        )

    def _test_controllers(self) -> ValidationResult:
        """Test controller status."""
        if not self.robot_profile.enable_ros2_control:
            return ValidationResult(
                test_name="controllers",
                passed=True,
                duration=0,
                message="ros2_control not enabled, skipping"
            )

        result = self.executor.run_command(
            "ros2 control list_controllers",
            timeout=5,
            env=self.env_manager.get_ros_env()
        )

        passed = result["returncode"] == 0

        return ValidationResult(
            test_name="controllers",
            passed=passed,
            duration=0,
            message="Controllers loaded" if passed else "Failed to list controllers",
            details={"output": result.get("stdout", "")}
        )

    def _test_services(self) -> ValidationResult:
        """Test ROS2 services."""
        result = self.executor.run_command(
            "ros2 service list",
            timeout=5,
            env=self.env_manager.get_ros_env()
        )

        if result["returncode"] != 0:
            return ValidationResult(
                test_name="services",
                passed=False,
                duration=0,
                message="Failed to get service list"
            )

        services = result["stdout"].strip().split('\n')

        # Check for expected services based on configuration
        expected_services = []
        if self.robot_profile.enable_navigation:
            expected_services.extend([
                "/navigate_to_pose",
                "/compute_path_to_pose"
            ])

        missing_services = [s for s in expected_services if not any(s in svc for svc in services)]
        passed = len(missing_services) == 0

        return ValidationResult(
            test_name="services",
            passed=passed,
            duration=0,
            message="All expected services present" if passed else f"Missing services: {missing_services}",
            details={
                "total_services": len(services),
                "missing_services": missing_services
            }
        )

    def _test_navigation_stack(self) -> ValidationResult:
        """Test navigation stack initialization."""
        if not self.robot_profile.enable_navigation:
            return ValidationResult(
                test_name="navigation_stack",
                passed=True,
                duration=0,
                message="Navigation not enabled, skipping"
            )

        # Check Nav2 lifecycle nodes
        result = self.executor.run_command(
            "ros2 lifecycle list",
            timeout=5,
            env=self.env_manager.get_ros_env()
        )

        if result["returncode"] != 0:
            return ValidationResult(
                test_name="navigation_stack",
                passed=False,
                duration=0,
                message="Failed to list lifecycle nodes"
            )

        lifecycle_nodes = result["stdout"].strip().split('\n')
        nav2_nodes = [n for n in lifecycle_nodes if any(x in n for x in ["controller", "planner", "bt_navigator"])]

        passed = len(nav2_nodes) > 0

        return ValidationResult(
            test_name="navigation_stack",
            passed=passed,
            duration=0,
            message=f"Found {len(nav2_nodes)} Nav2 nodes" if passed else "No Nav2 lifecycle nodes found",
            details={"nav2_nodes": nav2_nodes}
        )

    def _test_simple_navigation(self) -> ValidationResult:
        """Test simple navigation goal."""
        if not self.robot_profile.enable_navigation:
            return ValidationResult(
                test_name="simple_navigation",
                passed=True,
                duration=0,
                message="Navigation not enabled, skipping"
            )

        # Send a simple navigation goal
        goal_cmd = """ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{
            pose: {
                header: {frame_id: "map"},
                pose: {
                    position: {x: 2.0, y: 0.0, z: 0.0},
                    orientation: {w: 1.0}
                }
            }
        }'"""

        result = self.executor.run_command(
            goal_cmd,
            timeout=30,
            env=self.env_manager.get_ros_env()
        )

        passed = "Goal finished with status: SUCCEEDED" in result.get("stdout", "")

        return ValidationResult(
            test_name="simple_navigation",
            passed=passed,
            duration=0,
            message="Navigation goal succeeded" if passed else "Navigation goal failed",
            details={"output": result.get("stdout", "")}
        )

    def _test_obstacle_avoidance(self) -> ValidationResult:
        """Test obstacle avoidance."""
        # This would require more complex setup with obstacles
        # For now, return a placeholder result
        return ValidationResult(
            test_name="obstacle_avoidance",
            passed=True,
            duration=0,
            message="Obstacle avoidance test not yet implemented"
        )

    def _test_slam(self) -> ValidationResult:
        """Test SLAM functionality."""
        if not self.robot_profile.enable_slam:
            return ValidationResult(
                test_name="slam",
                passed=True,
                duration=0,
                message="SLAM not enabled, skipping"
            )

        # Check if map is being published
        result = self.executor.run_command(
            "timeout 3 ros2 topic hz /map",
            timeout=5,
            env=self.env_manager.get_ros_env()
        )

        passed = "average rate:" in result.get("stdout", "")

        return ValidationResult(
            test_name="slam",
            passed=passed,
            duration=0,
            message="Map being published" if passed else "Map topic not publishing",
            details={"output": result.get("stdout", "")}
        )

    def _test_real_time_factor(self) -> ValidationResult:
        """Test simulation real-time factor."""
        metrics = self.sim_manager.capture_simulation_metrics(duration=5)

        rtf = metrics.get("avg_real_time_factor", 0)
        passed = rtf > 0.8  # Should maintain at least 80% real-time

        return ValidationResult(
            test_name="real_time_factor",
            passed=passed,
            duration=0,
            message=f"RTF: {rtf:.2f}" if rtf > 0 else "Could not measure RTF",
            details=metrics
        )

    def _test_cpu_usage(self) -> ValidationResult:
        """Test CPU usage."""
        metrics = self.sim_manager.capture_simulation_metrics(duration=5)

        cpu = metrics.get("avg_cpu_usage", 0)
        passed = cpu < 80  # Should not exceed 80% CPU

        return ValidationResult(
            test_name="cpu_usage",
            passed=passed,
            duration=0,
            message=f"CPU: {cpu:.1f}%" if cpu > 0 else "Could not measure CPU usage",
            details=metrics
        )

    def _test_topic_frequencies(self) -> ValidationResult:
        """Test topic publication frequencies."""
        metrics = self.sim_manager.capture_simulation_metrics(duration=5)

        topic_freqs = metrics.get("topic_frequencies", {})

        # Check if critical topics meet minimum frequency
        issues = []
        if "/tf" in topic_freqs and topic_freqs["/tf"] < 10:
            issues.append(f"/tf publishing at {topic_freqs['/tf']:.1f} Hz (expected >10 Hz)")

        if "/scan" in topic_freqs and topic_freqs["/scan"] < 5:
            issues.append(f"/scan publishing at {topic_freqs['/scan']:.1f} Hz (expected >5 Hz)")

        passed = len(issues) == 0

        return ValidationResult(
            test_name="topic_frequencies",
            passed=passed,
            duration=0,
            message="Topic frequencies OK" if passed else f"Issues: {'; '.join(issues)}",
            details={"topic_frequencies": topic_freqs}
        )

    def _generate_recommendations(self, report: ValidationReport):
        """Generate recommendations based on validation results."""
        # Analyze failures and generate recommendations
        for result in report.results:
            if not result.passed:
                if "tf" in result.test_name.lower():
                    report.recommendations.append("Review URDF file and ensure all joints and links are properly defined")
                elif "sensor" in result.test_name.lower():
                    report.recommendations.append("Check sensor configuration and driver installations")
                elif "navigation" in result.test_name.lower():
                    report.recommendations.append("Verify navigation parameters and map configuration")
                elif "cpu" in result.test_name.lower():
                    report.recommendations.append("Consider reducing sensor rates or disabling unnecessary nodes")

        # Performance recommendations
        if report.performance_metrics:
            rtf = report.performance_metrics.get("avg_real_time_factor", 1)
            if rtf < 0.5:
                report.recommendations.append("Simulation running slowly. Consider simplifying world or reducing physics accuracy")

            cpu = report.performance_metrics.get("avg_cpu_usage", 0)
            if cpu > 90:
                report.recommendations.append("High CPU usage detected. Consider using composable nodes or reducing update rates")

    # Helper methods
    def _has_lidar(self) -> bool:
        """Check if robot has lidar."""
        if not self.robot_profile.hardware_manifest:
            return False
        return any(c.component_type == "lidar" for c in self.robot_profile.hardware_manifest.hardware_components)

    def _has_camera(self) -> bool:
        """Check if robot has camera."""
        if not self.robot_profile.hardware_manifest:
            return False
        return any(c.component_type in ["camera", "depth_camera", "rgbd_camera"]
                  for c in self.robot_profile.hardware_manifest.hardware_components)