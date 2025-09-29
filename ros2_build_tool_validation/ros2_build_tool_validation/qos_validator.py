"""
QoS policy validation for publisher/subscriber compatibility
"""

import logging
from typing import List, Dict, Optional, Tuple
from enum import Enum

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False


class QoSCompatibility(str, Enum):
    """QoS compatibility status"""
    COMPATIBLE = "compatible"
    INCOMPATIBLE = "incompatible"
    WARNING = "warning"


class QoSValidator(Node):
    """
    Validate QoS policy compatibility between publishers and subscribers

    QoS Compatibility Rules:
    - Reliability: RELIABLE pub can be read by BEST_EFFORT sub, but not vice versa
    - Durability: TRANSIENT_LOCAL pub can be read by VOLATILE sub, but not vice versa
    - History: Depth mismatch causes warnings but not failures
    - Liveliness: Must match exactly
    """

    def __init__(self, logger: Optional[logging.Logger] = None):
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy not available. Install ROS2 first.")

        super().__init__('qos_validator')

        self.logger = logger or logging.getLogger(__name__)
        self.get_logger().info('QoS Validator initialized')

    def get_topic_qos_profiles(self, topic_name: str) -> Dict[str, List[QoSProfile]]:
        """
        Get QoS profiles for all publishers and subscribers on a topic

        Returns: {'publishers': [QoSProfile, ...], 'subscribers': [QoSProfile, ...]}
        """
        try:
            # Get topic info
            topic_info = self.get_publishers_info_by_topic(topic_name)
            subscriber_info = self.get_subscriptions_info_by_topic(topic_name)

            # Extract QoS profiles
            pub_profiles = [info.qos_profile for info in topic_info]
            sub_profiles = [info.qos_profile for info in subscriber_info]

            self.logger.info(
                f"Topic {topic_name}: {len(pub_profiles)} publishers, {len(sub_profiles)} subscribers"
            )

            return {
                'publishers': pub_profiles,
                'subscribers': sub_profiles
            }

        except Exception as e:
            self.logger.error(f"Failed to get QoS profiles for {topic_name}: {e}")
            return {'publishers': [], 'subscribers': []}

    def validate_reliability(
        self,
        pub_reliability: ReliabilityPolicy,
        sub_reliability: ReliabilityPolicy
    ) -> Tuple[QoSCompatibility, str]:
        """
        Validate reliability policy compatibility

        Rules:
        - RELIABLE pub → RELIABLE sub: Compatible
        - RELIABLE pub → BEST_EFFORT sub: Compatible (sub gets all messages)
        - BEST_EFFORT pub → RELIABLE sub: Incompatible (sub expects reliability)
        - BEST_EFFORT pub → BEST_EFFORT sub: Compatible
        """
        if pub_reliability == ReliabilityPolicy.RELIABLE:
            # RELIABLE pub compatible with any sub
            return QoSCompatibility.COMPATIBLE, "Publisher is RELIABLE, compatible with any subscriber"

        elif pub_reliability == ReliabilityPolicy.BEST_EFFORT:
            if sub_reliability == ReliabilityPolicy.RELIABLE:
                return (
                    QoSCompatibility.INCOMPATIBLE,
                    "BEST_EFFORT publisher cannot satisfy RELIABLE subscriber requirements"
                )
            else:
                return QoSCompatibility.COMPATIBLE, "Both using BEST_EFFORT"

        return QoSCompatibility.WARNING, "Unknown reliability policy"

    def validate_durability(
        self,
        pub_durability: DurabilityPolicy,
        sub_durability: DurabilityPolicy
    ) -> Tuple[QoSCompatibility, str]:
        """
        Validate durability policy compatibility

        Rules:
        - TRANSIENT_LOCAL pub → TRANSIENT_LOCAL sub: Compatible (late joiners get last message)
        - TRANSIENT_LOCAL pub → VOLATILE sub: Compatible (sub ignores history)
        - VOLATILE pub → TRANSIENT_LOCAL sub: Incompatible (no history available)
        - VOLATILE pub → VOLATILE sub: Compatible
        """
        if pub_durability == DurabilityPolicy.TRANSIENT_LOCAL:
            # TRANSIENT_LOCAL pub compatible with any sub
            return QoSCompatibility.COMPATIBLE, "Publisher has TRANSIENT_LOCAL durability"

        elif pub_durability == DurabilityPolicy.VOLATILE:
            if sub_durability == DurabilityPolicy.TRANSIENT_LOCAL:
                return (
                    QoSCompatibility.INCOMPATIBLE,
                    "VOLATILE publisher cannot provide history for TRANSIENT_LOCAL subscriber"
                )
            else:
                return QoSCompatibility.COMPATIBLE, "Both using VOLATILE"

        return QoSCompatibility.WARNING, "Unknown durability policy"

    def validate_history(
        self,
        pub_history: HistoryPolicy,
        pub_depth: int,
        sub_history: HistoryPolicy,
        sub_depth: int
    ) -> Tuple[QoSCompatibility, str]:
        """
        Validate history policy compatibility

        Rules:
        - History policy mismatches cause warnings but not failures
        - Depth mismatches: If pub_depth < sub_depth, subscriber may not get desired history
        """
        if pub_depth < sub_depth:
            return (
                QoSCompatibility.WARNING,
                f"Publisher depth ({pub_depth}) < Subscriber depth ({sub_depth}). "
                f"Subscriber may not receive full requested history."
            )

        return QoSCompatibility.COMPATIBLE, f"History depth compatible (pub: {pub_depth}, sub: {sub_depth})"

    def validate_liveliness(
        self,
        pub_liveliness: LivelinessPolicy,
        sub_liveliness: LivelinessPolicy
    ) -> Tuple[QoSCompatibility, str]:
        """
        Validate liveliness policy compatibility

        Rules:
        - Liveliness policies must match exactly
        """
        if pub_liveliness != sub_liveliness:
            return (
                QoSCompatibility.INCOMPATIBLE,
                f"Liveliness mismatch: Publisher {pub_liveliness.name} vs Subscriber {sub_liveliness.name}"
            )

        return QoSCompatibility.COMPATIBLE, f"Both using {pub_liveliness.name} liveliness"

    def validate_qos_compatibility(
        self,
        pub_qos: QoSProfile,
        sub_qos: QoSProfile
    ) -> Dict[str, Tuple[QoSCompatibility, str]]:
        """
        Validate complete QoS profile compatibility

        Returns: Dictionary with compatibility status for each QoS parameter
        """
        results = {}

        # Validate reliability
        results['reliability'] = self.validate_reliability(
            pub_qos.reliability,
            sub_qos.reliability
        )

        # Validate durability
        results['durability'] = self.validate_durability(
            pub_qos.durability,
            sub_qos.durability
        )

        # Validate history
        results['history'] = self.validate_history(
            pub_qos.history,
            pub_qos.depth,
            sub_qos.history,
            sub_qos.depth
        )

        # Validate liveliness
        results['liveliness'] = self.validate_liveliness(
            pub_qos.liveliness,
            sub_qos.liveliness
        )

        return results

    def validate_topic(self, topic_name: str) -> Dict[str, any]:
        """
        Validate QoS compatibility for all publishers and subscribers on a topic

        Returns: Dictionary with overall compatibility status and details
        """
        qos_profiles = self.get_topic_qos_profiles(topic_name)

        publishers = qos_profiles['publishers']
        subscribers = qos_profiles['subscribers']

        if not publishers:
            self.logger.warning(f"No publishers found for topic {topic_name}")
            return {'status': 'no_publishers', 'compatible': False}

        if not subscribers:
            self.logger.warning(f"No subscribers found for topic {topic_name}")
            return {'status': 'no_subscribers', 'compatible': True}

        # Check each pub-sub pair
        incompatibilities = []
        warnings = []

        for i, pub_qos in enumerate(publishers):
            for j, sub_qos in enumerate(subscribers):
                results = self.validate_qos_compatibility(pub_qos, sub_qos)

                # Check for incompatibilities
                for param, (status, message) in results.items():
                    if status == QoSCompatibility.INCOMPATIBLE:
                        incompatibilities.append({
                            'publisher_index': i,
                            'subscriber_index': j,
                            'parameter': param,
                            'message': message
                        })
                        self.logger.error(
                            f"Topic {topic_name} - Pub[{i}] Sub[{j}] {param}: {message}"
                        )

                    elif status == QoSCompatibility.WARNING:
                        warnings.append({
                            'publisher_index': i,
                            'subscriber_index': j,
                            'parameter': param,
                            'message': message
                        })
                        self.logger.warning(
                            f"Topic {topic_name} - Pub[{i}] Sub[{j}] {param}: {message}"
                        )

        # Overall status
        compatible = len(incompatibilities) == 0

        return {
            'status': 'compatible' if compatible else 'incompatible',
            'compatible': compatible,
            'num_publishers': len(publishers),
            'num_subscribers': len(subscribers),
            'incompatibilities': incompatibilities,
            'warnings': warnings
        }

    def get_recommended_qos(self, topic_type: str) -> QoSProfile:
        """
        Get recommended QoS profile based on topic type

        Common patterns:
        - Sensor data: BEST_EFFORT, VOLATILE (prioritize latency)
        - Commands: RELIABLE, VOLATILE (ensure delivery)
        - State: RELIABLE, TRANSIENT_LOCAL (late joiners get state)
        - Diagnostics: BEST_EFFORT, VOLATILE
        """
        sensor_types = ['sensor_msgs', 'LaserScan', 'Image', 'PointCloud2', 'Imu']
        command_types = ['Twist', 'cmd_vel', 'geometry_msgs/Twist']
        state_types = ['State', 'JointState', 'robot_description']

        topic_lower = topic_type.lower()

        # Sensor data
        if any(sensor in topic_lower for sensor in sensor_types):
            return QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=1,
                history=HistoryPolicy.KEEP_LAST
            )

        # Commands
        elif any(cmd in topic_lower for cmd in command_types):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )

        # State
        elif any(state in topic_lower for state in state_types):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
                history=HistoryPolicy.KEEP_LAST
            )

        # Default: Reliable, Volatile
        else:
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )


def validate_topics_standalone(topic_names: List[str], timeout_sec: float = 5.0) -> Dict[str, Dict]:
    """
    Standalone function to validate QoS for multiple topics

    Args:
        topic_names: List of topic names to validate
        timeout_sec: Timeout for validation

    Returns:
        Dictionary mapping topic names to validation results
    """
    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy not available")

    rclpy.init()

    try:
        validator = QoSValidator()

        # Spin briefly to discover topics
        import time
        time.sleep(2.0)
        rclpy.spin_once(validator, timeout_sec=0.1)

        results = {}
        for topic_name in topic_names:
            results[topic_name] = validator.validate_topic(topic_name)

        return results

    finally:
        validator.destroy_node()
        rclpy.shutdown()