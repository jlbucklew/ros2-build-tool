"""
Self-healing watchdog node for monitoring critical topics and recovering from failures
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
import time
from typing import Dict, List, Optional, Any, Type
from collections import defaultdict

try:
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py import message_to_ordereddict
    ROSIDL_AVAILABLE = True
except ImportError:
    ROSIDL_AVAILABLE = False


class TopicWatchdog(Node):
    """
    Monitor critical topics and trigger recovery actions when topics go stale

    Features:
    - Monitors topic publication rates
    - Detects stale topics (no messages for > timeout)
    - Attempts to restart lifecycle nodes
    - Publishes health status
    """

    def __init__(self):
        super().__init__('topic_watchdog')

        # Declare parameters
        self.declare_parameter('monitored_topics', ['/scan', '/odom', '/imu'])
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('check_rate_hz', 1.0)
        self.declare_parameter('recovery_attempts', 3)
        self.declare_parameter('recovery_cooldown_seconds', 30.0)
        self.declare_parameter('enable_recovery', True)

        # Get parameters
        self.monitored_topics: List[str] = self.get_parameter('monitored_topics').value
        self.timeout_seconds: float = self.get_parameter('timeout_seconds').value
        self.check_rate_hz: float = self.get_parameter('check_rate_hz').value
        self.recovery_attempts: int = self.get_parameter('recovery_attempts').value
        self.recovery_cooldown: float = self.get_parameter('recovery_cooldown_seconds').value
        self.enable_recovery: bool = self.get_parameter('enable_recovery').value

        # State tracking
        self.last_message_time: Dict[str, float] = {}
        self.topic_healthy: Dict[str, bool] = {}
        self.recovery_count: Dict[str, int] = defaultdict(int)
        self.last_recovery_time: Dict[str, float] = {}
        self.subscriptions = {}
        self.topic_types: Dict[str, str] = {}
        self.topic_to_node: Dict[str, List[str]] = {}  # Map topics to publishing nodes
        self.lifecycle_clients: Dict[str, Any] = {}  # Lifecycle service clients per node

        # Health status publisher
        self.health_publisher = self.create_publisher(
            String,
            '/watchdog/health_status',
            10
        )

        # Setup monitoring
        self._setup_topic_monitoring()

        # Create timer for health checks
        self.check_timer = self.create_timer(
            1.0 / self.check_rate_hz,
            self._check_topic_health
        )

        self.get_logger().info(
            f'Watchdog initialized. Monitoring {len(self.monitored_topics)} topics '
            f'with {self.timeout_seconds}s timeout'
        )

    def _setup_topic_monitoring(self) -> None:
        """Setup subscriptions to monitored topics with dynamic type introspection"""

        if not ROSIDL_AVAILABLE:
            self.get_logger().error(
                'rosidl_runtime_py not available. Cannot introspect topic types. '
                'Install with: sudo apt install ros-humble-rosidl-runtime-py'
            )
            return

        # Discover topic types and publishing nodes
        self._discover_topic_info()

        # QoS profile that matches sensor data (typically BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        for topic in self.monitored_topics:
            # Initialize state
            self.last_message_time[topic] = time.time()
            self.topic_healthy[topic] = True

            # Get actual message type for this topic
            msg_type = self._get_topic_message_type(topic)
            if msg_type is None:
                self.get_logger().warn(
                    f'Could not determine message type for {topic}. '
                    f'Skipping monitoring. Is the topic being published?'
                )
                continue

            try:
                # Create subscription with actual message type
                sub = self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, topic_name=topic: self._topic_callback(topic_name),
                    sensor_qos
                )
                self.subscriptions[topic] = sub
                self.get_logger().info(
                    f'Monitoring topic: {topic} (type: {self.topic_types.get(topic, "unknown")})'
                )
            except Exception as e:
                self.get_logger().error(
                    f'Failed to subscribe to {topic}: {e}. '
                    f'Topic type: {self.topic_types.get(topic, "unknown")}'
                )

    def _discover_topic_info(self) -> None:
        """Discover topic types and which nodes publish them"""
        # Get all topics and their types
        topic_names_and_types = self.get_topic_names_and_types()

        for topic_name, type_list in topic_names_and_types:
            if topic_name in self.monitored_topics:
                # Store the first type (topics should have only one type)
                if type_list:
                    self.topic_types[topic_name] = type_list[0]

        # Discover which nodes publish to monitored topics
        for topic in self.monitored_topics:
            publishers_info = self.get_publishers_info_by_topic(topic)
            node_names = []
            for pub_info in publishers_info:
                # Extract node name from node namespace and name
                full_node_name = f"{pub_info.node_namespace}/{pub_info.node_name}".replace('//', '/')
                node_names.append(full_node_name)

            if node_names:
                self.topic_to_node[topic] = node_names
                self.get_logger().info(
                    f'Topic {topic} published by: {", ".join(node_names)}'
                )
            else:
                self.get_logger().warn(
                    f'No publishers found for topic {topic}. '
                    f'Recovery will not be possible until a publisher is available.'
                )

    def _get_topic_message_type(self, topic: str) -> Optional[Type]:
        """Get the message type class for a given topic"""
        topic_type = self.topic_types.get(topic)
        if not topic_type:
            return None

        try:
            # Convert type string like 'sensor_msgs/msg/LaserScan' to message class
            return get_message(topic_type)
        except (ValueError, AttributeError, ModuleNotFoundError) as e:
            self.get_logger().error(
                f'Failed to load message type {topic_type}: {e}'
            )
            return None

    def _topic_callback(self, topic_name: str):
        """Record that a message was received on this topic"""
        self.last_message_time[topic_name] = time.time()

        # If topic was unhealthy, mark as recovered
        if not self.topic_healthy[topic_name]:
            self.get_logger().info(f'Topic {topic_name} recovered!')
            self.topic_healthy[topic_name] = True
            self._publish_health_status()

    def _check_topic_health(self):
        """Check if monitored topics are healthy"""
        current_time = time.time()
        health_changed = False

        for topic in self.monitored_topics:
            time_since_last_msg = current_time - self.last_message_time[topic]
            is_healthy = time_since_last_msg < self.timeout_seconds

            # State transition: healthy -> unhealthy
            if self.topic_healthy[topic] and not is_healthy:
                self.get_logger().warn(
                    f'Topic {topic} is stale! '
                    f'No messages for {time_since_last_msg:.1f}s (timeout: {self.timeout_seconds}s)'
                )
                self.topic_healthy[topic] = False
                health_changed = True

                # Attempt recovery
                self._attempt_recovery(topic)

            # State transition: unhealthy -> healthy (handled in callback)

        if health_changed:
            self._publish_health_status()

    def _attempt_recovery(self, topic: str) -> None:
        """Attempt to recover a stale topic by restarting publishing nodes"""
        current_time = time.time()

        if not self.enable_recovery:
            self.get_logger().info(f'Recovery disabled, skipping recovery for {topic}')
            return

        # Check if we're in cooldown period
        if topic in self.last_recovery_time:
            time_since_recovery = current_time - self.last_recovery_time[topic]
            if time_since_recovery < self.recovery_cooldown:
                self.get_logger().info(
                    f'Skipping recovery for {topic} - in cooldown period '
                    f'({time_since_recovery:.1f}s / {self.recovery_cooldown}s)'
                )
                return

        # Check if we've exceeded max recovery attempts
        if self.recovery_count[topic] >= self.recovery_attempts:
            self.get_logger().error(
                f'Max recovery attempts ({self.recovery_attempts}) reached for {topic}. '
                f'Manual intervention required. Suggestions:\n'
                f'  1. Check if the sensor/driver is connected: ros2 topic list\n'
                f'  2. Check node logs: ros2 node list, then ros2 run ... with --ros-args --log-level debug\n'
                f'  3. Restart the node manually\n'
                f'  4. Check hardware connections and power'
            )
            return

        self.get_logger().info(
            f'Attempting recovery for topic {topic} '
            f'(attempt {self.recovery_count[topic] + 1}/{self.recovery_attempts})'
        )

        # Update recovery tracking
        self.recovery_count[topic] += 1
        self.last_recovery_time[topic] = current_time

        # Try to recover nodes publishing this topic
        publishing_nodes = self.topic_to_node.get(topic, [])
        if not publishing_nodes:
            self.get_logger().warn(
                f'No publisher nodes found for {topic}. Cannot perform recovery. '
                f'Re-discovering topic info...'
            )
            # Re-discover in case node started after watchdog
            self._discover_topic_info()
            publishing_nodes = self.topic_to_node.get(topic, [])

        if not publishing_nodes:
            self.get_logger().error(
                f'Still no publishers for {topic}. Node may have crashed completely. '
                f'Try restarting the launch file.'
            )
            return

        # Attempt lifecycle recovery for each publishing node
        for node_name in publishing_nodes:
            self._recover_node(node_name, topic)

    def _recover_node(self, node_name: str, topic: str) -> None:
        """Attempt to recover a specific node via lifecycle transitions"""
        self.get_logger().info(f'Attempting to recover node: {node_name} (publisher of {topic})')

        # Try lifecycle recovery first (for lifecycle nodes)
        success = self._lifecycle_recovery(node_name)

        if success:
            self.get_logger().info(f'Successfully triggered lifecycle recovery for {node_name}')
        else:
            self.get_logger().warn(
                f'Lifecycle recovery failed for {node_name}. Node may not be a lifecycle node.\n'
                f'For automatic recovery of non-lifecycle nodes, the launch file should use respawn=True.\n'
                f'Manual recovery: ros2 lifecycle set {node_name} configure && ros2 lifecycle set {node_name} activate'
            )

    def _lifecycle_recovery(self, node_name: str) -> bool:
        """Attempt lifecycle state recovery for a node"""
        try:
            # Create service clients for lifecycle management if not already created
            if node_name not in self.lifecycle_clients:
                get_state_client = self.create_client(
                    GetState,
                    f'{node_name}/get_state'
                )
                change_state_client = self.create_client(
                    ChangeState,
                    f'{node_name}/change_state'
                )
                self.lifecycle_clients[node_name] = {
                    'get_state': get_state_client,
                    'change_state': change_state_client
                }
            else:
                get_state_client = self.lifecycle_clients[node_name]['get_state']
                change_state_client = self.lifecycle_clients[node_name]['change_state']

            # Wait for services to be available (with timeout)
            if not get_state_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().debug(f'Lifecycle services not available for {node_name}')
                return False

            # Get current state
            get_state_req = GetState.Request()
            future = get_state_client.call_async(get_state_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if not future.done() or future.result() is None:
                self.get_logger().debug(f'Failed to get state for {node_name}')
                return False

            current_state = future.result().current_state.id

            # Determine recovery path based on current state
            recovery_transitions = self._get_recovery_transitions(current_state)

            if not recovery_transitions:
                self.get_logger().warn(
                    f'Node {node_name} is in state {current_state}, no recovery path available'
                )
                return False

            # Execute recovery transitions
            for transition in recovery_transitions:
                change_state_req = ChangeState.Request()
                change_state_req.transition.id = transition

                future = change_state_client.call_async(change_state_req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if not future.done() or not future.result().success:
                    self.get_logger().error(
                        f'Failed to transition {node_name} with transition {transition}'
                    )
                    return False

                # Small delay between transitions
                time.sleep(0.5)

            self.get_logger().info(f'Successfully recovered {node_name} via lifecycle transitions')
            return True

        except Exception as e:
            self.get_logger().error(f'Exception during lifecycle recovery for {node_name}: {e}')
            return False

    def _get_recovery_transitions(self, current_state: int) -> List[int]:
        """
        Determine the sequence of lifecycle transitions needed to recover a node

        States (from lifecycle_msgs/msg/State):
        - UNKNOWN = 0
        - UNCONFIGURED = 1
        - INACTIVE = 2
        - ACTIVE = 3
        - FINALIZED = 4

        Transitions (from lifecycle_msgs/msg/Transition):
        - TRANSITION_CREATE = 0
        - TRANSITION_CONFIGURE = 1
        - TRANSITION_CLEANUP = 2
        - TRANSITION_ACTIVATE = 3
        - TRANSITION_DEACTIVATE = 4
        - TRANSITION_UNCONFIGURED_SHUTDOWN = 5
        - TRANSITION_INACTIVE_SHUTDOWN = 6
        - TRANSITION_ACTIVE_SHUTDOWN = 7
        - TRANSITION_DESTROY = 8
        """
        # Recovery strategy: transition node through deactivate->cleanup->configure->activate
        # This performs a full reset of the node

        if current_state == 3:  # ACTIVE - but not publishing (stale)
            # Deactivate -> Cleanup -> Configure -> Activate
            return [
                Transition.TRANSITION_DEACTIVATE,   # 4: active -> inactive
                Transition.TRANSITION_CLEANUP,       # 2: inactive -> unconfigured
                Transition.TRANSITION_CONFIGURE,     # 1: unconfigured -> inactive
                Transition.TRANSITION_ACTIVATE       # 3: inactive -> active
            ]
        elif current_state == 2:  # INACTIVE
            # Cleanup -> Configure -> Activate
            return [
                Transition.TRANSITION_CLEANUP,       # 2: inactive -> unconfigured
                Transition.TRANSITION_CONFIGURE,     # 1: unconfigured -> inactive
                Transition.TRANSITION_ACTIVATE       # 3: inactive -> active
            ]
        elif current_state == 1:  # UNCONFIGURED
            # Configure -> Activate
            return [
                Transition.TRANSITION_CONFIGURE,     # 1: unconfigured -> inactive
                Transition.TRANSITION_ACTIVATE       # 3: inactive -> active
            ]
        else:
            # Unknown or finalized state - cannot recover
            return []

    def _publish_health_status(self):
        """Publish overall system health status"""
        healthy_count = sum(1 for h in self.topic_healthy.values() if h)
        total_count = len(self.topic_healthy)

        status_msg = String()
        status_msg.data = f'Health: {healthy_count}/{total_count} topics healthy. '

        unhealthy = [topic for topic, healthy in self.topic_healthy.items() if not healthy]
        if unhealthy:
            status_msg.data += f'Unhealthy: {", ".join(unhealthy)}'

        self.health_publisher.publish(status_msg)


def main(args=None):
    """Main entry point for watchdog node"""
    rclpy.init(args=args)

    watchdog = TopicWatchdog()

    try:
        rclpy.spin(watchdog)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()