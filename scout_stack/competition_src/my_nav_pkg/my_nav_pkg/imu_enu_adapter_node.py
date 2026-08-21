"""Measured MOT0110 heading bridge for the planar EKF.

The onboard AHRS quaternion is not copied as a generic 3-D orientation.  Only
its scalar yaw is converted with the measured sign/offset contract.  Gyroscope
axes are already ROS-correct on this installation and are never sign-flipped.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String

from .localization_core import (
    corrected_enu_yaw,
    planar_quaternion,
    yaw_from_quaternion,
)


def _stamp_ns(message: Imu) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class ImuEnuAdapter(Node):
    def __init__(self) -> None:
        super().__init__('imu_enu_adapter')
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/heading_enu')
        self.declare_parameter('status_topic', '/localization/imu_status')
        self.declare_parameter('expected_input_frame', 'imu_link')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('yaw_sign', 0.0)
        self.declare_parameter('yaw_offset_rad', 1000.0)
        # 출발 직진 캘리브(waypoint_follower)가 RTK 궤적으로 구한 절대 헤딩
        # offset을 실어 보내는 토픽. 자력계 절대헤딩은 차체 자화 때문에 20도
        # 안팎의 오차가 남아서(2026-08-02 실측) 런치의 고정값만으로는 부족하다.
        # 여기서 갱신하면 EKF -> /odometry/global -> follower/회피/주차까지
        # 같은 보정 헤딩을 쓰게 된다.
        self.declare_parameter('yaw_offset_update_topic', '/localization/yaw_offset')
        self.declare_parameter('yaw_variance_rad2', -1.0)
        self.declare_parameter('max_yaw_variance_rad2', 0.25)
        self.declare_parameter('max_stamp_age_sec', 0.60)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.expected_input_frame = str(
            self.get_parameter('expected_input_frame').value
        )
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.yaw_offset = float(
            self.get_parameter('yaw_offset_rad').value
        )
        self.yaw_variance = float(
            self.get_parameter('yaw_variance_rad2').value
        )
        self.max_yaw_variance = float(
            self.get_parameter('max_yaw_variance_rad2').value
        )
        self.max_stamp_age = float(
            self.get_parameter('max_stamp_age_sec').value
        )
        if self.yaw_sign not in (-1.0, 1.0):
            raise ValueError('yaw_sign must be exactly +1 or -1')
        if not math.isfinite(self.yaw_offset) or abs(self.yaw_offset) > math.pi:
            raise ValueError('yaw_offset_rad must be finite in [-pi, pi]')
        if (
            not math.isfinite(self.yaw_variance)
            or self.yaw_variance <= 0.0
            or not math.isfinite(self.max_yaw_variance)
            or self.yaw_variance > self.max_yaw_variance
        ):
            raise ValueError('measured yaw variance is missing or out of bounds')
        if not math.isfinite(self.max_stamp_age) or self.max_stamp_age <= 0.0:
            raise ValueError('max_stamp_age_sec must be positive')
        if not self.output_frame:
            raise ValueError('output_frame must not be empty')

        self._last_stamp_ns: Optional[int] = None
        self._last_status = ''
        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        # 캘리브 결과는 늦게(주행 시작 후) 한 번 오고 그 뒤로는 바뀌지 않으므로
        # TRANSIENT_LOCAL로 받아 재시작한 구독자도 마지막 값을 놓치지 않게 한다.
        offset_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter('yaw_offset_update_topic').value),
            self._on_yaw_offset_update,
            offset_qos,
        )

    def _on_yaw_offset_update(self, message: Float64) -> None:
        """직진 캘리브가 구한 절대 헤딩 offset으로 갈아끼운다."""
        value = float(message.data)
        if not math.isfinite(value) or abs(value) > math.pi:
            self.get_logger().warn(
                f'Ignoring out-of-range yaw offset update: {value}'
            )
            return
        previous = self.yaw_offset
        self.yaw_offset = value
        self.get_logger().info(
            f'yaw_offset updated {math.degrees(previous):+.1f} -> '
            f'{math.degrees(value):+.1f} deg (straight-line GNSS calibration).'
        )

    def _status(self, value: str) -> None:
        if value != self._last_status:
            self.status_publisher.publish(String(data=value))
            self._last_status = value

    def _on_imu(self, message: Imu) -> None:
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
            self.expected_input_frame
            and message.header.frame_id != self.expected_input_frame
        ):
            self._status('REJECT_FRAME')
            return

        raw = message.orientation
        raw_yaw = yaw_from_quaternion(raw.x, raw.y, raw.z, raw.w)
        if raw_yaw is None:
            self._status('REJECT_QUATERNION')
            return
        try:
            corrected_yaw = corrected_enu_yaw(
                raw_yaw,
                self.yaw_sign,
                self.yaw_offset,
            )
        except ValueError:
            self._status('REJECT_HEADING_CONTRACT')
            return
        qx, qy, qz, qw = planar_quaternion(corrected_yaw)

        output = Imu()
        # Do not alias the incoming Header object: changing frame_id on an
        # aliased ROS Python message also changes the received message.
        output.header.stamp.sec = message.header.stamp.sec
        output.header.stamp.nanosec = message.header.stamp.nanosec
        output.header.frame_id = self.output_frame
        output.orientation.x = qx
        output.orientation.y = qy
        output.orientation.z = qz
        output.orientation.w = qw
        output.orientation_covariance = [
            1_000_000.0,
            0.0,
            0.0,
            0.0,
            1_000_000.0,
            0.0,
            0.0,
            0.0,
            self.yaw_variance,
        ]

        # This topic is an absolute-heading measurement only.  The measured
        # installation has positive raw gyro-z for a physical left turn, so
        # the EKF consumes gyro-z from the unmodified /imu/data topic.  Mark
        # every other field unavailable to prevent accidental double fusion.
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(output)
        self._last_stamp_ns = stamp_ns
        self._status('OK')


def main() -> None:
    rclpy.init()
    node = ImuEnuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
