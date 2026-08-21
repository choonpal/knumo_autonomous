"""Absolute-yaw selector for the Global EKF.

The MOT0110 AHRS heading is relayed only until the first trustworthy RTK
course-over-ground (COG) is available.  After that lock, AHRS yaw is never fed
back into the Global EKF: turns are propagated by IMU gyro-z, and a COG yaw is
published only after a verified forward straight segment from raw GPS ENU
positions.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class HeadingSelector(Node):
    """Publish one absolute-yaw topic for robot_localization."""

    def __init__(self) -> None:
        super().__init__('heading_selector')

        self.declare_parameter('mag_heading_topic', '/imu/heading_enu')
        self.declare_parameter('gps_odom_topic', '/odometry/gps_enu')
        self.declare_parameter('gyro_topic', '/imu/data')
        self.declare_parameter('wheel_twist_topic', '/wheel/twist')
        self.declare_parameter('output_topic', '/imu/heading_selected')
        self.declare_parameter('status_topic', '/localization/heading_source')
        self.declare_parameter('output_frame', 'base_link')

        # The startup straight run is 0.4 m/s. A 1.5 m baseline initializes
        # heading during that one run. At 1.5 m/s it corrects again roughly one
        # second after each turn without stopping or lowering the drive speed.
        self.declare_parameter('min_forward_speed_mps', 0.20)
        self.declare_parameter('max_abs_gyro_rad_s', math.radians(3.0))
        self.declare_parameter('min_cog_baseline_m', 1.50)
        self.declare_parameter('min_cog_samples', 6)
        self.declare_parameter('min_linearity', 0.995)
        self.declare_parameter('max_position_std_m', 0.12)
        self.declare_parameter('max_position_step_m', 0.75)
        self.declare_parameter('input_timeout_sec', 0.50)
        self.declare_parameter(
            'cog_variance_floor_rad2',
            math.radians(3.0) ** 2,
        )

        self.output_frame = str(self.get_parameter('output_frame').value)
        self.min_forward_speed = float(
            self.get_parameter('min_forward_speed_mps').value
        )
        self.max_abs_gyro = float(
            self.get_parameter('max_abs_gyro_rad_s').value
        )
        self.min_cog_baseline = float(
            self.get_parameter('min_cog_baseline_m').value
        )
        self.min_cog_samples = int(
            self.get_parameter('min_cog_samples').value
        )
        self.min_linearity = float(
            self.get_parameter('min_linearity').value
        )
        self.max_position_std = float(
            self.get_parameter('max_position_std_m').value
        )
        self.max_position_step = float(
            self.get_parameter('max_position_step_m').value
        )
        self.input_timeout_sec = float(
            self.get_parameter('input_timeout_sec').value
        )
        self.cog_variance_floor = float(
            self.get_parameter('cog_variance_floor_rad2').value
        )

        positive = (
            self.min_forward_speed,
            self.max_abs_gyro,
            self.min_cog_baseline,
            self.max_position_std,
            self.max_position_step,
            self.input_timeout_sec,
            self.cog_variance_floor,
        )
        if not self.output_frame:
            raise ValueError('output_frame must not be empty')
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('heading selector thresholds must be finite and positive')
        if self.min_cog_samples < 3:
            raise ValueError('min_cog_samples must be at least 3')
        if not math.isfinite(self.min_linearity) or not 0.5 < self.min_linearity <= 1.0:
            raise ValueError('min_linearity must be in (0.5, 1.0]')

        self._gyro_z: Optional[float] = None
        self._gyro_rx_ns: Optional[int] = None
        self._wheel_vx: Optional[float] = None
        self._wheel_rx_ns: Optional[int] = None
        self._samples: List[Tuple[float, float, float]] = []
        self._cog_locked = False
        self._last_status = ''

        self.publisher = self.create_publisher(
            Imu,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('mag_heading_topic').value),
            self._on_mag_heading,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('gps_odom_topic').value),
            self._on_gps_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('gyro_topic').value),
            self._on_gyro,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            str(self.get_parameter('wheel_twist_topic').value),
            self._on_wheel,
            qos_profile_sensor_data,
        )

    def _status(self, value: str) -> None:
        if value != self._last_status:
            self.status_publisher.publish(String(data=value))
            self._last_status = value

    def _fresh(self, now_ns: int, received_ns: Optional[int]) -> bool:
        return (
            received_ns is not None
            and 0 <= now_ns - received_ns <= int(self.input_timeout_sec * 1e9)
        )

    def _reset_segment(self, reason: str) -> None:
        self._samples.clear()
        self._status(
            f'GYRO_ONLY_{reason}' if self._cog_locked else f'MAG_INIT_{reason}'
        )

    def _on_mag_heading(self, message: Imu) -> None:
        # AHRS yaw is only an initialization aid. Once a valid COG has locked,
        # direction-dependent magnetometer distortion must not re-enter the EKF.
        if self._cog_locked:
            return
        self.publisher.publish(message)
        self._status('MAG_INIT')

    def _on_gyro(self, message: Imu) -> None:
        value = float(message.angular_velocity.z)
        if not math.isfinite(value):
            return
        self._gyro_z = value
        self._gyro_rx_ns = self.get_clock().now().nanoseconds

    def _on_wheel(self, message: TwistWithCovarianceStamped) -> None:
        value = float(message.twist.twist.linear.x)
        if not math.isfinite(value):
            return
        self._wheel_vx = value
        self._wheel_rx_ns = self.get_clock().now().nanoseconds

    def _motion_reason(self, now_ns: int) -> Optional[str]:
        if not self._fresh(now_ns, self._gyro_rx_ns) or self._gyro_z is None:
            return 'GYRO_STALE'
        if not self._fresh(now_ns, self._wheel_rx_ns) or self._wheel_vx is None:
            return 'WHEEL_STALE'
        if self._wheel_vx < self.min_forward_speed:
            return 'NOT_FORWARD'
        if abs(self._gyro_z) > self.max_abs_gyro:
            return 'TURNING'
        return None

    def _on_gps_odom(self, message: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        reason = self._motion_reason(now_ns)
        if reason is not None:
            self._reset_segment(reason)
            return

        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        variance_x = float(message.pose.covariance[0])
        variance_y = float(message.pose.covariance[7])
        if not all(
            math.isfinite(value)
            for value in (x, y, variance_x, variance_y)
        ):
            self._reset_segment('GPS_NONFINITE')
            return
        if variance_x <= 0.0 or variance_y <= 0.0:
            self._reset_segment('GPS_COVARIANCE')
            return
        if max(math.sqrt(variance_x), math.sqrt(variance_y)) > self.max_position_std:
            # The GNSS adapter floors RTK-float variance at 0.0104 m^2 (0.102 m
            # std) regardless of the reported h_acc, so this gate reads a
            # quality label rather than the live accuracy.  The launch default
            # (0.12 m) therefore admits float and still rejects a float fix
            # whose own h_acc has degraded past 0.12 m.
            self._reset_segment('GPS_QUALITY')
            return

        if self._samples:
            px, py, _ = self._samples[-1]
            if math.hypot(x - px, y - py) > self.max_position_step:
                self._samples = [(x, y, max(variance_x, variance_y))]
                self._status(
                    'GYRO_ONLY_GPS_JUMP' if self._cog_locked else 'MAG_INIT_GPS_JUMP'
                )
                return

        self._samples.append((x, y, max(variance_x, variance_y)))
        if len(self._samples) < self.min_cog_samples:
            self._status('COG_CONFIRMING')
            return

        first_x, first_y, _ = self._samples[0]
        baseline = math.hypot(x - first_x, y - first_y)
        if baseline < self.min_cog_baseline:
            self._status('COG_CONFIRMING')
            return

        points = np.asarray([(sx, sy) for sx, sy, _ in self._samples], dtype=float)
        centered = points - points.mean(axis=0)
        covariance = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[order]
        principal = eigenvectors[:, order[0]]
        total = float(eigenvalues.sum())
        linearity = float(eigenvalues[0] / total) if total > 0.0 else 0.0
        if linearity < self.min_linearity:
            self._reset_segment('NOT_STRAIGHT')
            return

        course = math.atan2(float(principal[1]), float(principal[0]))
        displacement_x = x - first_x
        displacement_y = y - first_y
        if (
            displacement_x * math.cos(course)
            + displacement_y * math.sin(course)
            < 0.0
        ):
            course = _wrap_angle(course + math.pi)

        max_position_variance = max(sample[2] for sample in self._samples)
        yaw_variance = max(
            self.cog_variance_floor,
            2.0 * max_position_variance / (baseline * baseline),
        )

        output = Imu()
        output.header.stamp.sec = message.header.stamp.sec
        output.header.stamp.nanosec = message.header.stamp.nanosec
        output.header.frame_id = self.output_frame
        output.orientation.z = math.sin(0.5 * course)
        output.orientation.w = math.cos(0.5 * course)
        output.orientation_covariance = [0.0] * 9
        output.orientation_covariance[0] = 1_000_000.0
        output.orientation_covariance[4] = 1_000_000.0
        output.orientation_covariance[8] = yaw_variance
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(output)

        self._cog_locked = True
        self._status('COG_STRAIGHT')
        # Start a new independent straight baseline. At 1.5 m/s this refreshes
        # roughly every second; during a turn the gyro gate clears it immediately.
        self._samples = [(x, y, max(variance_x, variance_y))]


def main() -> None:
    rclpy.init()
    node = HeadingSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
