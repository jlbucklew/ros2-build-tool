"""
TF tree validation using tf2_ros Buffer API (not shell commands)
"""

import logging
from typing import List, Dict, Optional, Tuple
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class TFValidator(Node):
    """
    Validate TF tree using proper tf2_ros API instead of shell commands

    Features:
    - Check frame existence
    - Validate transform publication rates
    - Check for disconnected frames
    - Verify REP-105 compliance
    """

    def __init__(self, timeout_sec: float = 5.0, logger: Optional[logging.Logger] = None):
        if not TF2_AVAILABLE:
            raise RuntimeError("tf2_ros not available. Install ROS2 first.")

        super().__init__('tf_validator')

        self.logger = logger or logging.getLogger(__name__)
        self.timeout_sec = timeout_sec

        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'TF Validator initialized with {timeout_sec}s timeout')

    def validate_frame_exists(self, frame_id: str) -> bool:
        """Check if a frame exists in the TF tree"""
        try:
            # Check if frame is in the buffer
            frames = self.tf_buffer.all_frames_as_string()
            return frame_id in frames
        except Exception as e:
            self.logger.warning(f"Failed to check frame existence: {e}")
            return False

    def validate_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: Optional[float] = None
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate that a transform exists and is being published

        Returns: (success, error_message)
        """
        if timeout_sec is None:
            timeout_sec = self.timeout_sec

        try:
            # Try to lookup transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),  # Latest available
                Duration(seconds=timeout_sec)
            )

            self.logger.info(
                f"Transform {source_frame} -> {target_frame} is valid. "
                f"Translation: [{transform.transform.translation.x:.3f}, "
                f"{transform.transform.translation.y:.3f}, "
                f"{transform.transform.translation.z:.3f}]"
            )

            return True, None

        except tf2_ros.LookupException as e:
            error_msg = f"Transform lookup failed: {e}"
            self.logger.error(error_msg)
            return False, error_msg

        except tf2_ros.ConnectivityException as e:
            error_msg = f"Frames are not connected: {e}"
            self.logger.error(error_msg)
            return False, error_msg

        except tf2_ros.ExtrapolationException as e:
            error_msg = f"Transform extrapolation failed: {e}"
            self.logger.error(error_msg)
            return False, error_msg

        except Exception as e:
            error_msg = f"Unexpected error: {e}"
            self.logger.error(error_msg)
            return False, error_msg

    def validate_rep105_frames(self, base_frame: str = 'base_link') -> Dict[str, bool]:
        """
        Validate REP-105 frame compliance

        REP-105 defines standard frames:
        - map -> odom -> base_footprint -> base_link -> sensors
        """
        required_transforms = {
            'odom_to_base_footprint': ('odom', 'base_footprint'),
            'base_footprint_to_base_link': ('base_footprint', base_frame),
        }

        optional_transforms = {
            'map_to_odom': ('map', 'odom'),
        }

        results = {}

        # Check required transforms
        for name, (target, source) in required_transforms.items():
            success, error = self.validate_transform(target, source)
            results[name] = success

            if not success:
                self.logger.error(f"Required transform {name} failed: {error}")

        # Check optional transforms
        for name, (target, source) in optional_transforms.items():
            success, error = self.validate_transform(target, source, timeout_sec=2.0)
            results[name] = success

            if not success:
                self.logger.warning(f"Optional transform {name} not available: {error}")
            else:
                self.logger.info(f"Optional transform {name} is available")

        return results

    def get_all_frames(self) -> List[str]:
        """Get list of all frames in the TF tree"""
        try:
            frames_str = self.tf_buffer.all_frames_as_string()
            # Parse the string output (format: "Frame frame_id exists with parent parent_id")
            frames = []
            for line in frames_str.split('\n'):
                if 'Frame' in line and 'exists' in line:
                    parts = line.split()
                    if len(parts) >= 2:
                        frames.append(parts[1])
            return frames
        except Exception as e:
            self.logger.error(f"Failed to get frames: {e}")
            return []

    def validate_transform_rate(
        self,
        target_frame: str,
        source_frame: str,
        expected_rate_hz: float,
        sample_duration_sec: float = 5.0
    ) -> Tuple[bool, float]:
        """
        Validate that a transform is published at the expected rate

        Returns: (success, actual_rate_hz)
        """
        import time

        try:
            samples = []
            start_time = time.time()

            while time.time() - start_time < sample_duration_sec:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        Duration(seconds=0.1)
                    )

                    # Record timestamp
                    stamp = transform.header.stamp
                    timestamp = stamp.sec + stamp.nanosec * 1e-9
                    samples.append(timestamp)

                    time.sleep(0.01)  # Small delay

                except Exception:
                    pass

            if len(samples) < 2:
                self.logger.warning(f"Insufficient samples for rate validation: {len(samples)}")
                return False, 0.0

            # Calculate rate from samples
            time_span = samples[-1] - samples[0]
            actual_rate = (len(samples) - 1) / time_span if time_span > 0 else 0.0

            # Check if within 20% of expected rate
            tolerance = 0.2
            success = abs(actual_rate - expected_rate_hz) / expected_rate_hz < tolerance

            if success:
                self.logger.info(
                    f"Transform rate OK: {actual_rate:.1f}Hz "
                    f"(expected: {expected_rate_hz}Hz)"
                )
            else:
                self.logger.warning(
                    f"Transform rate OUT OF RANGE: {actual_rate:.1f}Hz "
                    f"(expected: {expected_rate_hz}Hz ± {tolerance*100}%)"
                )

            return success, actual_rate

        except Exception as e:
            self.logger.error(f"Failed to validate transform rate: {e}")
            return False, 0.0

    def check_for_transform_discontinuities(
        self,
        target_frame: str,
        source_frame: str,
        max_jump_meters: float = 1.0,
        sample_duration_sec: float = 10.0
    ) -> Tuple[bool, List[float]]:
        """
        Check for large discontinuities in transforms (jumps > threshold)

        Returns: (success, list_of_jumps)
        """
        import time
        import math

        jumps = []
        last_position = None

        try:
            start_time = time.time()

            while time.time() - start_time < sample_duration_sec:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        rclpy.time.Time(),
                        Duration(seconds=0.1)
                    )

                    # Extract position
                    pos = transform.transform.translation
                    current_position = (pos.x, pos.y, pos.z)

                    if last_position is not None:
                        # Calculate distance moved
                        dx = current_position[0] - last_position[0]
                        dy = current_position[1] - last_position[1]
                        dz = current_position[2] - last_position[2]
                        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                        if distance > max_jump_meters:
                            jumps.append(distance)
                            self.logger.warning(
                                f"Large transform jump detected: {distance:.3f}m "
                                f"(threshold: {max_jump_meters}m)"
                            )

                    last_position = current_position
                    time.sleep(0.1)

                except Exception:
                    pass

            success = len(jumps) == 0

            if success:
                self.logger.info("No transform discontinuities detected")
            else:
                self.logger.error(f"Detected {len(jumps)} transform jumps")

            return success, jumps

        except Exception as e:
            self.logger.error(f"Failed to check discontinuities: {e}")
            return False, []


def validate_tf_tree_standalone(
    required_frames: List[Tuple[str, str]],
    timeout_sec: float = 10.0
) -> Dict[str, bool]:
    """
    Standalone function to validate TF tree without creating a node

    Args:
        required_frames: List of (target_frame, source_frame) tuples to validate
        timeout_sec: Timeout for validation

    Returns:
        Dictionary mapping transform names to validation results
    """
    if not TF2_AVAILABLE:
        raise RuntimeError("tf2_ros not available")

    rclpy.init()

    try:
        validator = TFValidator(timeout_sec=timeout_sec)

        results = {}

        # Spin briefly to allow TF buffer to fill
        import time
        time.sleep(2.0)
        rclpy.spin_once(validator, timeout_sec=0.1)

        # Validate each required transform
        for target, source in required_frames:
            name = f"{source}_to_{target}"
            success, error = validator.validate_transform(target, source)
            results[name] = success

        return results

    finally:
        validator.destroy_node()
        rclpy.shutdown()