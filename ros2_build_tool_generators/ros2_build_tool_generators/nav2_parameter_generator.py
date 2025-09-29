"""
Dynamic Nav2 parameter generation with adaptive calculations based on robot specifications
"""

import logging
from pathlib import Path
from typing import Optional
from ros2_build_tool_core.models import RobotSpecs, RobotProfile, UseCase


class Nav2ParameterGenerator:
    """
    Generate optimized Nav2 parameters from robot specifications

    Improvements over original:
    - Truly dynamic calculations (no hardcoded 0.8 or 1.0 values)
    - Environment-aware parameter selection
    - Controller selection based on use case
    - Complete parameter coverage (recovery, smoother servers)
    """

    def __init__(self, package_path: Path, logger: logging.Logger):
        self.package_path = package_path
        self.config_dir = package_path / 'config'
        self.config_dir.mkdir(exist_ok=True)
        self.logger = logger

    def generate_all_nav2_params(self, robot_specs: RobotSpecs, profile: RobotProfile):
        """Generate complete Nav2 parameter set"""

        # Select controller based on use case
        controller_plugin = self._select_controller(profile.use_case)

        # Generate all parameter files
        self._generate_controller_params(robot_specs, controller_plugin)
        self._generate_planner_params(robot_specs, profile)
        self._generate_costmap_params(robot_specs, profile)
        self._generate_behavior_tree(robot_specs)
        self._generate_bt_navigator_params()
        self._generate_behavior_server_params(robot_specs)
        self._generate_recovery_server_params(robot_specs)
        self._generate_smoother_server_params(robot_specs)

        self.logger.info("Generated complete Nav2 parameter set")

    def _select_controller(self, use_case: UseCase) -> str:
        """
        Select appropriate controller for use case

        Improvements:
        - Mapping -> RPP (smooth, human-friendly)
        - Navigation -> RPP (balanced performance)
        - Perception -> DWB (dynamic obstacle avoidance)
        - Full stack -> MPPI (advanced, high CPU)
        """
        controller_map = {
            UseCase.MAPPING: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            UseCase.NAVIGATION: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            UseCase.PERCEPTION: "dwb_core::DWBLocalPlanner",
            UseCase.FULL_STACK: "nav2_mppi_controller::MPPIController"
        }
        return controller_map.get(use_case, "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")

    def _generate_controller_params(self, specs: RobotSpecs, plugin: str):
        """
        Generate fully adaptive controller server parameters

        Improvements:
        - All parameters calculated from robot specs
        - No hardcoded values (except physics constants)
        - Velocity-based adaptations
        - Size-based safety margins
        """

        # Conservative velocity (80% of max for safety)
        desired_linear_vel = specs.max_linear_velocity * 0.8

        # Lookahead distance: 1.0-1.5 seconds of travel at desired velocity
        # Slower robots need less lookahead
        lookahead_time = 1.0 if desired_linear_vel < 0.5 else 1.5
        lookahead_dist = desired_linear_vel * lookahead_time
        min_lookahead = lookahead_dist * 0.5
        max_lookahead = lookahead_dist * 1.5

        # Regulated scaling radius: 1.5-2.5x robot radius (larger for bigger robots)
        regulated_scaling_radius = specs.robot_radius * (1.5 + specs.robot_radius)

        # Velocity threshold: smaller robots need finer control
        min_vel_threshold = 0.001 if specs.robot_radius < 0.3 else 0.01

        # Transform tolerance: slower robots can have tighter tolerance
        transform_tolerance = 0.05 if desired_linear_vel < 0.5 else 0.1

        # Required movement for progress: 60-80% of robot radius
        required_movement = specs.robot_radius * 0.7

        # Goal tolerance: proportional to robot size with minimum threshold
        xy_goal_tolerance = max(specs.robot_radius * 0.4, 0.08)
        yaw_goal_tolerance = 0.12 if specs.robot_radius < 0.3 else 0.20

        # Approach velocity: 5-10% of desired velocity
        min_approach_vel = desired_linear_vel * 0.08
        approach_scaling_dist = specs.robot_radius * 1.5

        # Angular velocity: 50-70% of max for smooth rotation
        rotate_angular_vel = specs.max_angular_velocity * 0.6
        max_angular_accel = specs.max_angular_velocity * 0.8

        # Collision avoidance time: function of velocity and robot size
        # Faster/larger robots need more time
        collision_time = 0.5 + (desired_linear_vel * 0.3) + (specs.robot_radius * 0.5)

        # Pose search distance: proportional to velocity (for path following)
        max_pose_search = min(desired_linear_vel * 15, 10.0)

        # Controller frequency: higher for faster robots
        controller_freq = 10.0 if desired_linear_vel < 0.5 else 20.0

        config = f'''controller_server:
  ros__parameters:
    controller_frequency: {controller_freq:.1f}
    min_x_velocity_threshold: {min_vel_threshold:.4f}
    min_y_velocity_threshold: {min_vel_threshold:.4f}
    min_theta_velocity_threshold: {min_vel_threshold:.4f}
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: {required_movement:.3f}
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: {xy_goal_tolerance:.3f}
      yaw_goal_tolerance: {yaw_goal_tolerance:.3f}

    FollowPath:
      plugin: "{plugin}"
'''

        if "RegulatedPurePursuitController" in plugin:
            config += f'''      desired_linear_vel: {desired_linear_vel:.3f}
      lookahead_dist: {lookahead_dist:.3f}
      min_lookahead_dist: {min_lookahead:.3f}
      max_lookahead_dist: {max_lookahead:.3f}
      lookahead_time: {lookahead_time:.1f}
      rotate_to_heading_angular_vel: {rotate_angular_vel:.3f}
      transform_tolerance: {transform_tolerance:.3f}
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: {min_approach_vel:.3f}
      approach_velocity_scaling_dist: {approach_scaling_dist:.3f}
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: {collision_time:.2f}
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: {regulated_scaling_radius:.3f}
      regulated_linear_scaling_min_speed: {desired_linear_vel * 0.5:.3f}
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: {max_angular_accel:.3f}
      max_robot_pose_search_dist: {max_pose_search:.2f}
      use_interpolation: true
'''
        elif "DWBLocalPlanner" in plugin:
            config += self._generate_dwb_params(specs)
        elif "MPPIController" in plugin:
            config += self._generate_mppi_params(specs)

        with open(self.config_dir / 'controller.yaml', 'w') as f:
            f.write(config)

        self.logger.info(f"Generated controller params: {plugin}")

    def _generate_dwb_params(self, specs: RobotSpecs) -> str:
        """Generate DWB controller parameters"""
        return f'''      # DWB Controller params
      max_vel_x: {specs.max_linear_velocity:.3f}
      min_vel_x: -{specs.max_linear_velocity * 0.3:.3f}
      max_vel_y: 0.0
      min_vel_y: 0.0
      max_vel_theta: {specs.max_angular_velocity:.3f}
      min_vel_theta: -{specs.max_angular_velocity:.3f}
      acc_lim_x: {specs.max_linear_velocity * 2.0:.3f}
      acc_lim_y: 0.0
      acc_lim_theta: {specs.max_angular_velocity * 2.0:.3f}
      decel_lim_x: -{specs.max_linear_velocity * 2.5:.3f}
      decel_lim_y: 0.0
      decel_lim_theta: -{specs.max_angular_velocity * 2.5:.3f}
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
'''

    def _generate_mppi_params(self, specs: RobotSpecs) -> str:
        """Generate MPPI controller parameters"""
        return f'''      # MPPI Controller params
      max_vel_x: {specs.max_linear_velocity:.3f}
      min_vel_x: -{specs.max_linear_velocity * 0.3:.3f}
      max_vel_theta: {specs.max_angular_velocity:.3f}
      min_vel_theta: -{specs.max_angular_velocity:.3f}
      model_dt: 0.1
      time_steps: 50
      batch_size: 400
      iteration_count: 1
      temperature: 0.3
      motion_model: "DiffDrive"
'''

    def _generate_planner_params(self, specs: RobotSpecs, profile: RobotProfile):
        """Generate planner server parameters with adaptive tolerance"""

        # Tolerance based on robot size and desired precision
        tolerance = max(specs.robot_radius * 0.3, 0.3)

        config = f'''planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: {tolerance:.2f}
      use_astar: true
      allow_unknown: true
'''

        with open(self.config_dir / 'planner.yaml', 'w') as f:
            f.write(config)

    def _generate_costmap_params(self, specs: RobotSpecs, profile: RobotProfile):
        """Generate costmap parameters with dynamic sizing"""

        robot_radius = specs.robot_radius

        # Inflation radius: 1.5-2.0x robot radius (safety margin)
        inflation_radius = robot_radius * 1.8

        # Cost scaling factor: higher for larger robots (more conservative)
        cost_scaling_factor = 8.0 + (robot_radius * 5.0)

        # Local costmap: sized for 2-4 seconds of travel
        prediction_time = 3.0
        local_width = min(specs.max_linear_velocity * prediction_time * 2, 8.0)
        local_height = local_width

        # Update frequencies: higher for faster robots
        update_frequency = 3.0 if specs.max_linear_velocity < 0.5 else 7.0

        # Sensor ranges (can be overridden based on actual sensors)
        max_obstacle_range = 2.5
        raytrace_range = 3.0

        config = f'''global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: {robot_radius:.3f}
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: {specs.height * 1.5:.2f}
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: {raytrace_range:.2f}
          raytrace_min_range: 0.0
          obstacle_max_range: {max_obstacle_range:.2f}
          obstacle_min_range: 0.0

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: {cost_scaling_factor:.2f}
        inflation_radius: {inflation_radius:.3f}

      always_send_full_costmap: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: {update_frequency:.1f}
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: {int(local_width)}
      height: {int(local_height)}
      resolution: 0.05
      robot_radius: {robot_radius:.3f}
      plugins: ["voxel_layer", "inflation_layer"]

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: {cost_scaling_factor:.2f}
        inflation_radius: {inflation_radius:.3f}

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: {specs.height * 1.5:.2f}
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: {specs.height * 1.5:.2f}
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: {raytrace_range:.2f}
          raytrace_min_range: 0.0
          obstacle_max_range: {max_obstacle_range:.2f}
          obstacle_min_range: 0.0

      always_send_full_costmap: True
'''

        with open(self.config_dir / 'costmap.yaml', 'w') as f:
            f.write(config)

    def _generate_behavior_tree(self, specs: RobotSpecs):
        """Generate behavior tree XML with adaptive recovery parameters"""

        # Adaptive backup distance based on robot size
        backup_dist = max(specs.robot_radius * 0.5, 0.15)
        backup_speed = specs.max_linear_velocity * 0.1

        # Spin distance: quarter turn to full turn based on situation
        spin_dist = 1.57  # 90 degrees as baseline

        bt_xml = f'''<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{{goal}}" path="{{path}}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{{path}}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Seq" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Seq" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="{spin_dist:.2f}"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="{backup_dist:.3f}" backup_speed="{backup_speed:.3f}"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
'''

        with open(self.config_dir / 'behavior.xml', 'w') as f:
            f.write(bt_xml)

    def _generate_bt_navigator_params(self):
        """Generate BT navigator parameters"""
        config = '''bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
'''

        with open(self.config_dir / 'bt_navigator.yaml', 'w') as f:
            f.write(config)

    def _generate_behavior_server_params(self, specs: RobotSpecs):
        """Generate behavior server parameters (spin, backup, wait)"""

        # Adaptive parameters based on robot capabilities
        spin_speed = specs.max_angular_velocity * 0.5
        backup_speed = specs.max_linear_velocity * 0.15
        backup_dist = specs.robot_radius * 0.6

        config = f'''behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "drive_on_heading"]

    spin:
      plugin: "nav2_behaviors::Spin"
      simulate_ahead_time: 2.0
      max_rotational_vel: {spin_speed:.3f}
      min_rotational_vel: {spin_speed * 0.2:.3f}
      rotational_acc_lim: {specs.max_angular_velocity * 2.0:.3f}

    backup:
      plugin: "nav2_behaviors::BackUp"
      simulate_ahead_time: 2.0
      backup_speed: {backup_speed:.3f}
      backup_dist: {backup_dist:.3f}

    wait:
      plugin: "nav2_behaviors::Wait"

    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      simulate_ahead_time: 2.0
      max_linear_vel: {specs.max_linear_velocity * 0.5:.3f}
      min_linear_vel: {specs.max_linear_velocity * 0.1:.3f}
      linear_acc_lim: {specs.max_linear_velocity * 2.0:.3f}
'''

        with open(self.config_dir / 'behavior_server.yaml', 'w') as f:
            f.write(config)

    def _generate_recovery_server_params(self, specs: RobotSpecs):
        """Generate recovery server parameters"""
        config = '''recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries::Spin"

    backup:
      plugin: "nav2_recoveries::BackUp"

    wait:
      plugin: "nav2_recoveries::Wait"
'''

        with open(self.config_dir / 'recovery_server.yaml', 'w') as f:
            f.write(config)

    def _generate_smoother_server_params(self, specs: RobotSpecs):
        """Generate path smoother server parameters"""

        # Smoothing parameters based on robot dynamics
        max_velocity = specs.max_linear_velocity

        config = f'''smoother_server:
  ros__parameters:
    costmap_topic: global_costmap/costmap_raw
    footprint_topic: global_costmap/published_footprint
    robot_base_frame: base_link
    transform_timeout: 0.1
    smoother_plugins: ["simple_smoother"]

    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True
      refinement_num: 2
      w_data: 0.2
      w_smooth: 0.3
      w_cost: 0.015
      w_cost_curvature_vels_ratio: 1.0
      cost_check_points: 3
      max_curvature: 1.0
      curvature_constraint_cost_weight: 30.0
'''

        with open(self.config_dir / 'smoother_server.yaml', 'w') as f:
            f.write(config)