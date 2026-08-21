"""Convert Scout raw odometry into a measured, stamped EKF twist input."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .localization_core import scale_wheel_twist


def _stamp_ns(message: Odometry) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class WheelTwistAdapter(Node):
    def __init__(self) -> None:
        super().__init__('wheel_twist_adapter')
        self.declare_parameter(
            'input_topic',
            '/scout_mini_base_controller/odom',
        )
        self.declare_parameter('output_topic', '/wheel/twist')
        self.declare_parameter('status_topic', '/localization/wheel_status')
        self.declare_parameter('expected_header_frame', 'odom')
        self.declare_parameter('expected_child_frame', 'base_link')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('max_stamp_age_sec', 0.35)

        self.expected_header_frame = str(
            self.get_parameter('expected_header_frame').value
        )
        self.expected_child_frame = str(
            self.get_parameter('expected_child_frame').value
        )
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.linear_scale = float(
            self.get_parameter('linear_scale').value
        )
        self.max_stamp_age = float(
            self.get_parameter('max_stamp_age_sec').value
        )
        if not math.isfinite(self.linear_scale) or self.linear_scale <= 0.0:
            raise ValueError('linear_scale must be finite and positive')
        if not math.isfinite(self.max_stamp_age) or self.max_stamp_age <= 0.0:
            raise ValueError('max_stamp_age_sec must be finite and positive')
        if not self.output_frame:
            raise ValueError('output_frame must not be empty')

        self._last_stamp_ns: Optional[int] = None
        self._last_status = ''
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('input_topic').value),
            self._on_odom,
            10,
        )

    def _status(self, value: str) -> None:
        if value != self._last_status:
            self.status_publisher.publish(String(data=value))
            self._last_status = value

    def _on_odom(self, message: Odometry) -> None:
        stamp_ns = _stamp_ns(message)
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= 0:
            self._status('REJECT_ZERO_STAMP')
            return
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self._status('REJECT_NONADVANCING_STAMP')
            return
        age_ns = now_ns - stamp_ns
        if age_ns < 0 or age_ns > int(self.max_stamp_age * 1e9):
            self._status('REJECT_STALE_OR_FUTURE_STAMP')
            return
        if (
            self.expected_header_frame
            and message.header.frame_id != self.expected_header_frame
        ):
            self._status('REJECT_HEADER_FRAME')
            return
        if (
            self.expected_child_frame
            and message.child_frame_id != self.expected_child_frame
        ):
            self._status('REJECT_CHILD_FRAME')
            return

        covariance = tuple(float(value) for value in message.twist.covariance)
        if len(covariance) != 36 or not all(
            math.isfinite(value) for value in covariance
        ):
            self._status('REJECT_COVARIANCE')
            return
        try:
            scaled_x, scaled_var_x = scale_wheel_twist(
                message.twist.twist.linear.x,
                self.linear_scale,
                covariance[0],
            )
        except ValueError:
            self._status('REJECT_TWIST')
            return
        if covariance[7] <= 0.0:
            self._status('REJECT_LATERAL_COVARIANCE')
            return

        scaled_covariance = list(covariance)
        for index in range(6):
            scaled_covariance[index] *= self.linear_scale
            scaled_covariance[index * 6] *= self.linear_scale
        scaled_covariance[0] = scaled_var_x

        output = TwistWithCovarianceStamped()
        output.header.stamp.sec = message.header.stamp.sec
        output.header.stamp.nanosec = message.header.stamp.nanosec
        output.header.frame_id = self.output_frame
        output.twist.twist.linear.x = scaled_x
        output.twist.twist.linear.y = message.twist.twist.linear.y
        output.twist.twist.linear.z = message.twist.twist.linear.z
        output.twist.twist.angular.x = message.twist.twist.angular.x
        output.twist.twist.angular.y = message.twist.twist.angular.y
        output.twist.twist.angular.z = message.twist.twist.angular.z
        output.twist.covariance = scaled_covariance
        self.publisher.publish(output)
        self._last_stamp_ns = stamp_ns
        self._status('OK')


def main() -> None:
    rclpy.init()
    node = WheelTwistAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
