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
from typing import Dict, List
from collections import defaultdict


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

        # Get parameters
        self.monitored_topics: List[str] = self.get_parameter('monitored_topics').value
        self.timeout_seconds: float = self.get_parameter('timeout_seconds').value
        self.check_rate_hz: float = self.get_parameter('check_rate_hz').value
        self.recovery_attempts: int = self.get_parameter('recovery_attempts').value
        self.recovery_cooldown: float = self.get_parameter('recovery_cooldown_seconds').value

        # State tracking
        self.last_message_time: Dict[str, float] = {}
        self.topic_healthy: Dict[str, bool] = {}
        self.recovery_count: Dict[str, int] = defaultdict(int)
        self.last_recovery_time: Dict[str, float] = {}
        self.subscriptions = {}

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

    def _setup_topic_monitoring(self):
        """Setup subscriptions to monitored topics"""

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

            # Create generic subscription (we don't care about message content)
            # Using String as a placeholder - in real implementation would introspect topic type
            try:
                # Create subscription with callback that just records timestamp
                sub = self.create_subscription(
                    String,  # Placeholder - should be actual message type
                    topic,
                    lambda msg, topic_name=topic: self._topic_callback(topic_name),
                    sensor_qos
                )
                self.subscriptions[topic] = sub
                self.get_logger().info(f'Monitoring topic: {topic}')
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')

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

    def _attempt_recovery(self, topic: str):
        """Attempt to recover a stale topic"""
        current_time = time.time()

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
                f'Manual intervention required.'
            )
            return

        self.get_logger().info(
            f'Attempting recovery for topic {topic} '
            f'(attempt {self.recovery_count[topic] + 1}/{self.recovery_attempts})'
        )

        # Try to identify and restart the publishing node
        # In a real implementation, this would:
        # 1. Identify which node publishes this topic
        # 2. If it's a lifecycle node, transition it through inactive -> active
        # 3. If it's a regular node, trigger restart through launch system

        # For now, just log the attempt
        self.recovery_count[topic] += 1
        self.last_recovery_time[topic] = current_time

        # TODO: Implement actual recovery logic
        # This would require:
        # - Service client for lifecycle state changes
        # - Integration with launch system for node restart
        # - Topic-to-node mapping

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