"""
Tests for Nav2 parameter generation
"""

import pytest
from pathlib import Path
from ros2_build_tool_generators.nav2_parameter_generator import Nav2ParameterGenerator
from ros2_build_tool_core.models import UseCase
import logging


class TestNav2ParameterGenerator:
    """Test Nav2 parameter generation"""

    def test_controller_param_generation(self, temp_dir, sample_robot_specs, sample_robot_profile):
        """Test generating controller parameters"""
        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        generator._generate_controller_params(
            sample_robot_specs,
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )

        config_file = temp_dir / 'config' / 'controller.yaml'
        assert config_file.exists()

        content = config_file.read_text()

        # Check key parameters are present and calculated
        assert 'controller_server:' in content
        assert 'desired_linear_vel:' in content
        assert 'lookahead_dist:' in content
        assert 'xy_goal_tolerance:' in content

        # Ensure no hardcoded 0.8 or 1.0 values in dynamic params
        # (except physics constants like 0.785 for angles)
        lines = content.split('\n')
        for line in lines:
            if 'desired_linear_vel:' in line:
                value = float(line.split(':')[1].strip())
                # Should be adaptive, not exactly 0.5
                assert value != 0.5 or sample_robot_specs.max_linear_velocity == 0.625

    def test_costmap_param_generation(self, temp_dir, sample_robot_specs, sample_robot_profile):
        """Test generating costmap parameters"""
        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        generator._generate_costmap_params(sample_robot_specs, sample_robot_profile)

        config_file = temp_dir / 'config' / 'costmap.yaml'
        assert config_file.exists()

        content = config_file.read_text()

        # Check both global and local costmaps
        assert 'global_costmap:' in content
        assert 'local_costmap:' in content
        assert 'inflation_radius:' in content
        assert 'robot_radius:' in content

        # Verify robot radius is used correctly
        assert f'robot_radius: {sample_robot_specs.robot_radius:.3f}' in content

    def test_behavior_tree_generation(self, temp_dir, sample_robot_specs):
        """Test generating behavior tree XML"""
        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        generator._generate_behavior_tree(sample_robot_specs)

        bt_file = temp_dir / 'config' / 'behavior.xml'
        assert bt_file.exists()

        content = bt_file.read_text()

        # Check BT structure
        assert '<root main_tree_to_execute="MainTree">' in content
        assert '<BehaviorTree ID="MainTree">' in content
        assert 'NavigateRecovery' in content
        assert 'RecoveryActions' in content

        # Check recovery behaviors
        assert '<Spin' in content
        assert '<Wait' in content
        assert '<BackUp' in content

    def test_complete_nav2_generation(self, temp_dir, sample_robot_specs, sample_robot_profile):
        """Test generating complete Nav2 parameter set"""
        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        generator.generate_all_nav2_params(sample_robot_specs, sample_robot_profile)

        # Check all expected files are generated
        config_dir = temp_dir / 'config'
        assert (config_dir / 'controller.yaml').exists()
        assert (config_dir / 'planner.yaml').exists()
        assert (config_dir / 'costmap.yaml').exists()
        assert (config_dir / 'behavior.xml').exists()
        assert (config_dir / 'bt_navigator.yaml').exists()
        assert (config_dir / 'behavior_server.yaml').exists()
        assert (config_dir / 'recovery_server.yaml').exists()
        assert (config_dir / 'smoother_server.yaml').exists()

    def test_controller_selection(self, temp_dir, sample_robot_specs):
        """Test controller selection based on use case"""
        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        # Test different use cases
        mapping_controller = generator._select_controller(UseCase.MAPPING)
        assert 'RegulatedPurePursuitController' in mapping_controller

        perception_controller = generator._select_controller(UseCase.PERCEPTION)
        assert 'DWBLocalPlanner' in perception_controller

        fullstack_controller = generator._select_controller(UseCase.FULL_STACK)
        assert 'MPPIController' in fullstack_controller


class TestDynamicCalculations:
    """Test that parameters are truly dynamic, not hardcoded"""

    def test_velocity_based_parameters(self, temp_dir, sample_robot_profile):
        """Test parameters adapt to robot velocity"""
        from ros2_build_tool_core.models import RobotSpecs

        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        # Create two robots with different velocities
        slow_robot = RobotSpecs(
            width=0.4, length=0.5, height=0.3,
            max_linear_velocity=0.3,  # Slow
            max_angular_velocity=1.0
        )

        fast_robot = RobotSpecs(
            width=0.4, length=0.5, height=0.3,
            max_linear_velocity=2.0,  # Fast
            max_angular_velocity=3.0
        )

        # Generate params for both
        generator._generate_controller_params(
            slow_robot,
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )
        slow_content = (temp_dir / 'config' / 'controller.yaml').read_text()

        (temp_dir / 'config' / 'controller.yaml').unlink()  # Remove to regenerate

        generator._generate_controller_params(
            fast_robot,
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )
        fast_content = (temp_dir / 'config' / 'controller.yaml').read_text()

        # Parameters should be different
        assert slow_content != fast_content

        # Extract desired_linear_vel values
        import re
        slow_vel = float(re.search(r'desired_linear_vel: ([\d.]+)', slow_content).group(1))
        fast_vel = float(re.search(r'desired_linear_vel: ([\d.]+)', fast_content).group(1))

        # Fast robot should have higher velocity
        assert fast_vel > slow_vel

    def test_size_based_parameters(self, temp_dir, sample_robot_profile):
        """Test parameters adapt to robot size"""
        from ros2_build_tool_core.models import RobotSpecs

        logger = logging.getLogger(__name__)
        generator = Nav2ParameterGenerator(temp_dir, logger)

        # Small robot
        small_robot = RobotSpecs(
            width=0.2, length=0.3, height=0.2,
            max_linear_velocity=1.0
        )

        # Large robot
        large_robot = RobotSpecs(
            width=1.0, length=1.5, height=0.6,
            max_linear_velocity=1.0
        )

        # Generate costmaps
        generator._generate_costmap_params(small_robot, sample_robot_profile)
        small_content = (temp_dir / 'config' / 'costmap.yaml').read_text()

        (temp_dir / 'config' / 'costmap.yaml').unlink()

        generator._generate_costmap_params(large_robot, sample_robot_profile)
        large_content = (temp_dir / 'config' / 'costmap.yaml').read_text()

        # Robot radius should be different
        assert small_robot.robot_radius < large_robot.robot_radius

        # Inflation radius should scale with robot size
        import re
        small_inflation = float(re.search(r'inflation_radius: ([\d.]+)', small_content).group(1))
        large_inflation = float(re.search(r'inflation_radius: ([\d.]+)', large_content).group(1))

        assert large_inflation > small_inflation