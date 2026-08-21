"""RTK-gated WGS84-to-ENU odometry input for the global EKF."""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from ublox_msgs.msg import NavPVT, RxmRTCM

from .gps_quality import (
    RTK_FIXED,
    RTK_FLOAT,
    classify_rtk_status,
    horizontal_accuracy_ok,
    valid_lla,
)
from .localization_core import (
    antenna_to_base_xy,
    horizontal_variance,
    itow_progresses,
    lla_to_enu,
    yaw_from_quaternion,
)


def _stamp_ns(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class GnssEnuOdometry(Node):
    def __init__(self) -> None:
        super().__init__('gnss_enu_odometry')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('navpvt_topic', '/navpvt')
        self.declare_parameter('rtcm_status_topic', '/rxmrtcm')
        self.declare_parameter('heading_topic', '/imu/heading_enu')
        self.declare_parameter('output_topic', '/odometry/gps_enu')
        self.declare_parameter('gated_fix_topic', '/fix/ekf')
        self.declare_parameter('status_topic', '/localization/gnss_status')
        self.declare_parameter('expected_fix_frame', 'gps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('origin_lat', 91.0)
        self.declare_parameter('origin_lon', 181.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('antenna_x_m', 1000.0)
        self.declare_parameter('antenna_y_m', 1000.0)
        self.declare_parameter('antenna_z_m', 1000.0)
        self.declare_parameter('antenna_lever_arm_verified', False)
        self.declare_parameter('required_rtk', 'fixed')
        self.declare_parameter('max_h_acc_m', 0.05)
        self.declare_parameter('fixed_variance_floor_m2', 0.000225)
        self.declare_parameter('float_variance_floor_m2', 0.0104)
        self.declare_parameter('navpvt_timeout_sec', 1.0)
        self.declare_parameter('rtcm_timeout_sec', 3.0)
        self.declare_parameter('heading_timeout_sec', 0.60)
        self.declare_parameter('fix_stamp_timeout_sec', 1.0)
        self.declare_parameter('require_fix_stamp_age', True)
        self.declare_parameter('max_itow_step_ms', 5000)

        self.expected_fix_frame = str(
            self.get_parameter('expected_fix_frame').value
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        if not valid_lla(self.origin_lat, self.origin_lon, self.origin_alt):
            raise ValueError('manual EKF origin is invalid')
        self.antenna_x = float(self.get_parameter('antenna_x_m').value)
        self.antenna_y = float(self.get_parameter('antenna_y_m').value)
        self.antenna_z = float(self.get_parameter('antenna_z_m').value)
        self.lever_verified = bool(
            self.get_parameter('antenna_lever_arm_verified').value
        )
        if (
            not self.lever_verified
            or not all(
                math.isfinite(value) and abs(value) <= 2.0
                for value in (
                    self.antenna_x,
                    self.antenna_y,
                    self.antenna_z,
                )
            )
        ):
            raise ValueError(
                'measured base_link->gps lever arm is missing or invalid'
            )
        self.required_rtk = str(
            self.get_parameter('required_rtk').value
        ).strip().lower()
        if self.required_rtk not in (
            'fixed',
            'float_or_fixed',
            'float-or-fixed',
        ):
            raise ValueError('required_rtk must be fixed or float_or_fixed')
        self.max_h_acc = float(self.get_parameter('max_h_acc_m').value)
        self.fixed_floor = float(
            self.get_parameter('fixed_variance_floor_m2').value
        )
        self.float_floor = float(
            self.get_parameter('float_variance_floor_m2').value
        )
        self.navpvt_timeout = float(
            self.get_parameter('navpvt_timeout_sec').value
        )
        self.rtcm_timeout = float(
            self.get_parameter('rtcm_timeout_sec').value
        )
        self.heading_timeout = float(
            self.get_parameter('heading_timeout_sec').value
        )
        self.fix_stamp_timeout = float(
            self.get_parameter('fix_stamp_timeout_sec').value
        )
        self.require_fix_stamp_age = bool(
            self.get_parameter('require_fix_stamp_age').value
        )
        self.max_itow_step_ms = int(
            self.get_parameter('max_itow_step_ms').value
        )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.max_h_acc,
                self.fixed_floor,
                self.float_floor,
                self.navpvt_timeout,
                self.rtcm_timeout,
                self.heading_timeout,
                self.fix_stamp_timeout,
            )
        ):
            raise ValueError('GNSS EKF timeout/covariance parameters are invalid')

        self._previous_itow: Optional[int] = None
        self._latest_navpvt: Optional[NavPVT] = None
        self._latest_navpvt_rx_ns: Optional[int] = None
        self._navpvt_generation = 0
        self._published_generation = 0
        self._last_fix_stamp_ns: Optional[int] = None
        self._pending_fix: Optional[NavSatFix] = None
        self._pending_fix_rx_ns: Optional[int] = None
        self._last_rtcm_rx_ns: Optional[int] = None
        self._latest_heading: Optional[float] = None
        self._latest_heading_rx_ns: Optional[int] = None
        self._last_status = ''

        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.fix_publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter('gated_fix_topic').value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.fix_subscription = self.create_subscription(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            self._on_fix,
            qos_profile_sensor_data,
        )
        self.navpvt_subscription = self.create_subscription(
            NavPVT,
            str(self.get_parameter('navpvt_topic').value),
            self._on_navpvt,
            qos_profile_sensor_data,
        )
        self.rtcm_subscription = self.create_subscription(
            RxmRTCM,
            str(self.get_parameter('rtcm_status_topic').value),
            self._on_rtcm,
            qos_profile_sensor_data,
        )
        self.heading_subscription = self.create_subscription(
            Imu,
            str(self.get_parameter('heading_topic').value),
            self._on_heading,
            qos_profile_sensor_data,
        )

    def _status(self, value: str) -> None:
        if value != self._last_status:
            self.status_publisher.publish(String(data=value))
            self._last_status = value

    def _on_navpvt(self, message: NavPVT) -> None:
        current = int(message.i_tow)
        progressed = itow_progresses(
            self._previous_itow,
            current,
            self.max_itow_step_ms,
        )
        if self._previous_itow is None or current != self._previous_itow:
            # Rebase after one rejected discontinuity so a receiver restart or
            # long outage can recover, while a frozen duplicate remains locked.
            self._previous_itow = current
        if not progressed:
            self._status('REJECT_NAVPVT_ITOW')
            return
        self._latest_navpvt = message
        self._latest_navpvt_rx_ns = self.get_clock().now().nanoseconds
        self._navpvt_generation += 1
        # ublox_gps publishes NavPVT immediately before NavSatFix, but DDS
        # callbacks from different topics are not ordering guarantees. Retry a
        # fresh pending Fix here so either callback order produces one update.
        if self._pending_fix is not None:
            self._try_publish_fix(self._pending_fix)

    def _on_rtcm(self, message: RxmRTCM) -> None:
        if int(message.flags) & int(message.FLAGS_CRC_FAILED):
            self._status('REJECT_RTCM_CRC')
            return
        if int(message.msg_type) <= 0:
            self._status('REJECT_RTCM_TYPE')
            return
        self._last_rtcm_rx_ns = self.get_clock().now().nanoseconds

    def _on_heading(self, message: Imu) -> None:
        yaw = yaw_from_quaternion(
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        if yaw is None:
            return
        self._latest_heading = yaw
        self._latest_heading_rx_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def _fresh(now_ns: int, received_ns: Optional[int], timeout: float) -> bool:
        return (
            received_ns is not None
            and 0 <= now_ns - received_ns <= int(timeout * 1e9)
        )

    def _on_fix(self, message: NavSatFix) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = _stamp_ns(message)
        if stamp_ns <= 0:
            self._status('REJECT_FIX_ZERO_STAMP')
            return
        if (
            self._last_fix_stamp_ns is not None
            and stamp_ns <= self._last_fix_stamp_ns
        ):
            self._status('REJECT_FIX_NONADVANCING_STAMP')
            return
        if self.require_fix_stamp_age:
            age_ns = now_ns - stamp_ns
            if age_ns < 0 or age_ns > int(self.fix_stamp_timeout * 1e9):
                self._status('REJECT_FIX_STALE_OR_FUTURE_STAMP')
                return
        if (
            self.expected_fix_frame
            and message.header.frame_id != self.expected_fix_frame
        ):
            self._status('REJECT_FIX_FRAME')
            return
        if message.status.status < 0 or not valid_lla(
            message.latitude,
            message.longitude,
            message.altitude,
        ):
            self._status('REJECT_FIX_INVALID')
            return
        self._pending_fix = message
        self._pending_fix_rx_ns = now_ns
        self._try_publish_fix(message)

    def _try_publish_fix(self, message: NavSatFix) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = _stamp_ns(message)
        if not self._fresh(
            now_ns,
            self._pending_fix_rx_ns,
            self.fix_stamp_timeout,
        ):
            self._pending_fix = None
            self._pending_fix_rx_ns = None
            self._status('REJECT_FIX_RECEPTION_STALE')
            return
        if (
            self._last_fix_stamp_ns is not None
            and stamp_ns <= self._last_fix_stamp_ns
        ):
            self._pending_fix = None
            self._pending_fix_rx_ns = None
            self._status('REJECT_FIX_NONADVANCING_STAMP')
            return
        nav = self._latest_navpvt
        if (
            nav is None
            or self._navpvt_generation <= self._published_generation
            or not self._fresh(
                now_ns,
                self._latest_navpvt_rx_ns,
                self.navpvt_timeout,
            )
        ):
            self._status('REJECT_NAVPVT_STALE_OR_REPLAYED')
            return
        if not self._fresh(now_ns, self._last_rtcm_rx_ns, self.rtcm_timeout):
            self._status('REJECT_RTCM_STALE')
            return
        if (
            abs(message.latitude - float(nav.lat) * 1e-7) > 1e-8
            or abs(message.longitude - float(nav.lon) * 1e-7) > 1e-8
            or abs(message.altitude - float(nav.height) * 1e-3) > 0.02
        ):
            self._status('REJECT_FIX_NAVPVT_MISMATCH')
            return
        status = classify_rtk_status(
            flags=nav.flags,
            fix_type=nav.fix_type,
            now_ns=now_ns,
            received_ns=self._latest_navpvt_rx_ns,
            timeout_sec=self.navpvt_timeout,
        )
        if status not in (RTK_FIXED, RTK_FLOAT):
            self._status('REJECT_RTK_NONE')
            return
        if self.required_rtk == 'fixed' and status != RTK_FIXED:
            self._status('REJECT_RTK_NOT_FIXED')
            return
        if not horizontal_accuracy_ok(nav.h_acc, self.max_h_acc):
            self._status('REJECT_HACC')
            return
        h_acc_m = float(nav.h_acc) / 1000.0
        variance = horizontal_variance(
            h_acc_m,
            self.fixed_floor if status == RTK_FIXED else self.float_floor,
        )
        try:
            east, north, _ = lla_to_enu(
                message.latitude,
                message.longitude,
                message.altitude,
                self.origin_lat,
                self.origin_lon,
                self.origin_alt,
            )
        except ValueError:
            self._status('REJECT_ENU_CONVERSION')
            return

        if abs(self.antenna_x) > 1e-12 or abs(self.antenna_y) > 1e-12:
            if (
                self._latest_heading is None
                or not self._fresh(
                    now_ns,
                    self._latest_heading_rx_ns,
                    self.heading_timeout,
                )
            ):
                self._status('REJECT_HEADING_FOR_LEVER_ARM')
                return
            east, north = antenna_to_base_xy(
                east,
                north,
                self._latest_heading,
                self.antenna_x,
                self.antenna_y,
            )

        output = Odometry()
        # Keep the gated NavSatFix in its original GPS sensor frame.  Assigning
        # the Header object directly would alias it, and setting this output to
        # map would then silently relabel the republished /fix/ekf message.
        output.header.stamp.sec = message.header.stamp.sec
        output.header.stamp.nanosec = message.header.stamp.nanosec
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = east
        output.pose.pose.position.y = north
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.w = 1.0
        output.pose.covariance = [0.0] * 36
        output.pose.covariance[0] = variance
        output.pose.covariance[7] = variance
        output.pose.covariance[14] = 1_000_000.0
        output.pose.covariance[21] = 1_000_000.0
        output.pose.covariance[28] = 1_000_000.0
        output.pose.covariance[35] = 1_000_000.0
        output.twist.covariance = [0.0] * 36
        output.twist.covariance[0] = -1.0
        self.odom_publisher.publish(output)
        self.fix_publisher.publish(message)
        self._published_generation = self._navpvt_generation
        self._last_fix_stamp_ns = stamp_ns
        self._pending_fix = None
        self._pending_fix_rx_ns = None
        self._status(
            f'OK_{status}_HACC_{h_acc_m:.3f}_GPS_Z_{self.antenna_z:.3f}'
        )


def main() -> None:
    rclpy.init()
    node = GnssEnuOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
