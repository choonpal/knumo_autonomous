#!/usr/bin/env python3
"""Spline + Pure Pursuit waypoint follower using one Global EKF odometry input.

This node intentionally has one control path only:

    /odometry/global -> spline path tracking -> Pure Pursuit -> /cmd_vel/follow

It does not contain the removed GPS/IMU fallback controller, startup straight-line
calibration, COG yaw correction, route SHA/token locks, CTE stop, or legacy bearing
controller.  CTE is published for rosbag analysis but never blocks or slows driving.
"""

from __future__ import annotations

import csv
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Int32, String

from .gps_quality import valid_lla, valid_xy_waypoint
from .path_core import (
    PathBuildError,
    PathTracker,
    ang_norm,
    build_path,
    pure_pursuit,
    validate_waypoints,
)
from .pose_utils import checked_yaw_from_quaternion
from .speed_profile import is_stopline_waypoint, waypoint_speed_cap


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

Waypoint = Tuple[float, float, Optional[float], Optional[str]]


def _lla_to_ecef(lat_rad: float, lon_rad: float, alt: float) -> Tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + alt) * cos_lat * cos_lon,
        (radius + alt) * cos_lat * sin_lon,
        (radius * (1.0 - WGS84_E2) + alt) * sin_lat,
    )


def _ecef_to_enu(
    x: float,
    y: float,
    z: float,
    x0: float,
    y0: float,
    z0: float,
    lat0_rad: float,
    lon0_rad: float,
) -> Tuple[float, float, float]:
    dx = x - x0
    dy = y - y0
    dz = z - z0
    sin_lat0 = math.sin(lat0_rad)
    cos_lat0 = math.cos(lat0_rad)
    sin_lon0 = math.sin(lon0_rad)
    cos_lon0 = math.cos(lon0_rad)
    return (
        -sin_lon0 * dx + cos_lon0 * dy,
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz,
        cos_lat0 * cos_lon0 * dx
        + cos_lat0 * sin_lon0 * dy
        + sin_lat0 * dz,
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class WaypointFollower(Node):
    """Follow a fixed ENU spline using pose and yaw from Global EKF odometry."""

    def __init__(self) -> None:
        super().__init__('waypoint_follower_node')

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.waypoints = self._load_waypoints()
        points = [(x, y) for x, y, _yaw, _name in self.waypoints]
        for warning in validate_waypoints(points):
            self.get_logger().warn(f'waypoint check: {warning}')

        try:
            self.path = build_path(points, ds=self.path_resolution)
            self.tracker = PathTracker(
                self.path,
                back_window_m=self.path_back_window,
                fwd_window_m=self.path_fwd_window,
            )
        except (PathBuildError, ValueError) as exc:
            raise RuntimeError(f'failed to build waypoint spline: {exc}') from exc

        self.index = 0
        self.rotating = False
        self.finished = False
        # 정지선 정차 상태.
        #   _stopline_brake_ns   : 정차 명령을 시작한 시각(감속 시작)
        #   _stopline_release_ns : 실제로 멈춘 것을 확인한 뒤 정해지는 해제 시각
        #   _stopline_done       : 이미 정차를 마친 웨이포인트 index 집합
        # 규정이 "완전히 멈춘 상태로 3초"라서 감속 구간은 3초에 포함하지 않는다.
        self._stopline_brake_ns: Optional[int] = None
        self._stopline_release_ns: Optional[int] = None
        self._stopline_done: set = set()
        self._odom_speed: Optional[float] = None
        # IMU 안정화 게이트 상태
        self._imu_settle_sec = float(self.get_parameter('imu_settle_sec').value)
        self._calibrated_since_ns: Optional[int] = None
        self._first_odom_ns: Optional[int] = None
        # 출발 직진 헤딩 캘리브 상태
        self._calib_enabled = bool(self.get_parameter('gps_calib_enabled').value)
        self._calib_distance = float(self.get_parameter('gps_calib_distance').value)
        self._calib_speed = float(self.get_parameter('gps_calib_speed').value)
        self._calib_timeout = float(self.get_parameter('gps_calib_timeout').value)
        self._calib_min_samples = int(self.get_parameter('gps_calib_min_samples').value)
        self._calib_linearity = float(self.get_parameter('gps_calib_linearity').value)
        self._calib_max_step = float(self.get_parameter('gps_calib_max_step').value)
        self._configured_yaw_offset = float(
            self.get_parameter('configured_yaw_offset_rad').value
        )
        self._calib_done = not self._calib_enabled
        self._calib_start_ns: Optional[int] = None
        self._calib_origin: Optional[Tuple[float, float]] = None
        # (x, y, imu_yaw) 샘플. 코스와 IMU 헤딩의 차이를 평균해 offset을 만든다.
        self._calib_samples: List[Tuple[float, float, float]] = []
        # 캘리브로 얻은 보정각. None이면 보정 없이 EKF yaw를 그대로 쓴다.
        self._heading_offset: Optional[float] = None
        self.last_log_ns = 0


        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel/follow', 10)
        self.pub_control_pose = self.create_publisher(
            PoseStamped,
            '/follower/control_pose',
            10,
        )
        self.pub_metrics = self.create_publisher(
            Float32MultiArray,
            '/follower/debug/metrics',
            10,
        )
        self.pub_target = self.create_publisher(
            PoseStamped,
            '/follower/debug/target',
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            '/follower/debug/state',
            10,
        )
        self.pub_cte = self.create_publisher(Float32, '/follower/cte', 10)
        self.pub_active_wp_idx = self.create_publisher(
            Int32,
            '/follower/active_wp_idx',
            10,
        )
        self.pub_active_wp = self.create_publisher(
            String,
            '/follower/active_wp',
            10,
        )
        self.pub_active_speed_cap = self.create_publisher(
            Float32,
            self.active_speed_cap_topic,
            10,
        )
        # 캘리브 결과(절대 yaw offset)를 imu_enu_adapter로 보낸다. 늦게 한 번만
        # 발행되므로 TRANSIENT_LOCAL로 남겨 구독자가 나중에 붙어도 받게 한다.
        self.pub_yaw_offset = self.create_publisher(
            Float64,
            str(self.get_parameter('yaw_offset_topic').value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.create_subscription(
            Bool,
            '/imu/is_calibrated',
            self._on_is_calibrated,
            10,
        )

        self.get_logger().info(
            'Pure Pursuit ready: '
            f'{len(self.waypoints)} waypoints, '
            f'{len(self.path)} spline samples, '
            f'length={self.path.length:.2f}m, '
            f'lookahead={self.lookahead_min:.2f}~{self.lookahead_max:.2f}m, '
            f'odom={self.odom_topic}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('file_name', 'waypoints.csv')
        self.declare_parameter('csv_mode', 'llh')
        self.declare_parameter('origin_lat', 91.0)
        self.declare_parameter('origin_lon', 181.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('active_speed_cap_topic', '/follower/active_speed_cap')

        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('vision_mission_speed', 0.10)
        self.declare_parameter('obstacle_mission_speed', 0.10)
        self.declare_parameter('parking_mission_speed', 0.08)
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('target_tolerance', 0.30)

        # --- 정지선 정차 ---
        # 이름에 stopline/정지선이 든 웨이포인트에 이 거리까지 접근하면
        # 그 자리에서 stopline_hold_sec 동안 정차한 뒤 다시 출발한다.
        # 웨이포인트당 한 번만 발동한다(재진입/제자리 진동으로 반복 정차 방지).
        self.declare_parameter('stopline_hold_sec', 3.0)
        self.declare_parameter('stopline_trigger_dist', 0.50)
        # 계시를 시작할 "멈춤" 판정 속도. 측정 속도가 이 아래로 내려간 뒤에야
        # hold 시간을 세기 시작한다(감속 구간은 3초에 포함하지 않는다).
        self.declare_parameter('stopline_stopped_speed', 0.03)
        # 속도가 임계 아래로 안 떨어져도 이 시간이 지나면 계시를 시작한다.
        # 오도메트리 잡음/드리프트로 코스 한가운데 영구 정지하는 것을 막는다.
        self.declare_parameter('stopline_settle_timeout_sec', 5.0)

        self.declare_parameter('path_resolution', 0.05)
        self.declare_parameter('lookahead_min', 0.60)
        self.declare_parameter('lookahead_max', 1.60)
        self.declare_parameter('lookahead_gain', 4.0)
        self.declare_parameter('a_lat_max', 0.05)
        self.declare_parameter('min_drive_speed', 0.05)
        self.declare_parameter('rotate_enter_deg', 60.0)
        self.declare_parameter('rotate_exit_deg', 20.0)
        self.declare_parameter('path_back_window', 2.0)
        self.declare_parameter('path_fwd_window', 5.0)
        # 출발 전 IMU 안정화 대기: /imu/is_calibrated=true 후 이 시간(초)만큼
        # 정지 유지한 뒤 주행 시작. 자력계 AHRS 헤딩 수렴 대기용.
        self.declare_parameter('imu_settle_sec', 12.0)
        # --- 출발 직진 헤딩 캘리브레이션 ---
        # 자력계 절대헤딩은 차체 자화/주변 철재 때문에 20도 안팎의 오차가 남는다
        # (2026-08-02 재보정으로도 잔차 18~20도). 대신 출발 직후 정해진 거리를
        # 직진하면서 RTK 궤적으로 참 헤딩(코스)을 구해 IMU와의 offset을 즉석에서
        # 잡는다. RTK Fixed 1.4cm 기준 6m 직진의 각도 오차는 약 0.13도다.
        self.declare_parameter('gps_calib_enabled', True)
        self.declare_parameter('gps_calib_distance', 6.0)   # m
        self.declare_parameter('gps_calib_speed', 0.4)      # m/s (저속: 샘플 확보/안전)
        self.declare_parameter('gps_calib_timeout', 40.0)   # s (이 안에 못 끝내면 실패)
        self.declare_parameter('gps_calib_min_samples', 20)
        # 직선성 지표(설명분산비) 하한. 낮으면 직진이 아니었다는 뜻이라 폐기한다.
        self.declare_parameter('gps_calib_linearity', 0.98)
        # 한 스텝(약 20~50 ms)에 이 거리를 넘게 움직였으면 EKF 점프로 본다.
        self.declare_parameter('gps_calib_max_step', 0.5)
        # imu_enu_adapter가 현재 쓰고 있는 고정 offset. 캘리브 결과를 여기에 더해
        # 절대 offset으로 만들어 어댑터에 돌려준다(localization_launch와 같은 값).
        self.declare_parameter('configured_yaw_offset_rad', 0.0)
        self.declare_parameter('yaw_offset_topic', '/localization/yaw_offset')

    def _read_parameters(self) -> None:
        self.file_name = str(self.get_parameter('file_name').value)
        self.csv_mode = str(self.get_parameter('csv_mode').value).strip().lower()
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.active_speed_cap_topic = str(
            self.get_parameter('active_speed_cap_topic').value
        )

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.vision_mission_speed = float(
            self.get_parameter('vision_mission_speed').value
        )
        self.obstacle_mission_speed = float(
            self.get_parameter('obstacle_mission_speed').value
        )
        self.parking_mission_speed = float(
            self.get_parameter('parking_mission_speed').value
        )
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.target_tolerance = float(
            self.get_parameter('target_tolerance').value
        )

        self.stopline_hold_sec = float(
            self.get_parameter('stopline_hold_sec').value
        )
        self.stopline_trigger_dist = float(
            self.get_parameter('stopline_trigger_dist').value
        )
        self.stopline_stopped_speed = float(
            self.get_parameter('stopline_stopped_speed').value
        )
        self.stopline_settle_timeout_sec = float(
            self.get_parameter('stopline_settle_timeout_sec').value
        )
        self.path_resolution = float(
            self.get_parameter('path_resolution').value
        )
        self.lookahead_min = float(self.get_parameter('lookahead_min').value)
        self.lookahead_max = float(self.get_parameter('lookahead_max').value)
        self.lookahead_gain = float(self.get_parameter('lookahead_gain').value)
        self.a_lat_max = float(self.get_parameter('a_lat_max').value)
        self.min_drive_speed = float(
            self.get_parameter('min_drive_speed').value
        )
        self.rotate_enter = math.radians(
            float(self.get_parameter('rotate_enter_deg').value)
        )
        self.rotate_exit = math.radians(
            float(self.get_parameter('rotate_exit_deg').value)
        )
        self.path_back_window = float(
            self.get_parameter('path_back_window').value
        )
        self.path_fwd_window = float(
            self.get_parameter('path_fwd_window').value
        )

    def _validate_parameters(self) -> None:
        if self.csv_mode not in {'xy', 'llh'}:
            raise ValueError("csv_mode must be 'xy' or 'llh'")
        if self.csv_mode == 'llh' and not valid_lla(
            self.origin_lat,
            self.origin_lon,
            self.origin_alt,
        ):
            raise ValueError('LLH mode requires a valid fixed ENU origin')
        if not self.odom_topic:
            raise ValueError('odom_topic must not be empty')
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        if not self.active_speed_cap_topic:
            raise ValueError('active_speed_cap_topic must not be empty')

        positive = (
            self.linear_speed,
            self.vision_mission_speed,
            self.obstacle_mission_speed,
            self.parking_mission_speed,
            self.angular_speed,
            self.target_tolerance,
            self.path_resolution,
            self.lookahead_min,
            self.lookahead_max,
            self.lookahead_gain,
            self.a_lat_max,
            self.min_drive_speed,
            self.rotate_enter,
            self.rotate_exit,
            self.path_fwd_window,
            self.stopline_hold_sec,
            self.stopline_trigger_dist,
            self.stopline_stopped_speed,
            self.stopline_settle_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('speed/path parameters must be finite and positive')
        if self.lookahead_min > self.lookahead_max:
            raise ValueError('lookahead_min must not exceed lookahead_max')
        if self.rotate_exit >= self.rotate_enter:
            raise ValueError('rotate_exit_deg must be smaller than rotate_enter_deg')
        if self.min_drive_speed > self.linear_speed:
            raise ValueError('min_drive_speed must not exceed linear_speed')
        if not math.isfinite(self.path_back_window) or self.path_back_window < 0.0:
            raise ValueError('path_back_window must be finite and non-negative')

    def _load_waypoints(self) -> List[Waypoint]:
        path = os.path.expanduser(f'~/.ros/{self.file_name}')
        if not os.path.isfile(path):
            raise FileNotFoundError(f'waypoint CSV not found: {path}')

        raw_llh: List[Tuple[float, float, float, Optional[str]]] = []
        waypoints: List[Waypoint] = []
        with open(path, 'r', encoding='utf-8-sig', newline='') as handle:
            for line_number, row in enumerate(csv.reader(handle), start=1):
                if not row or row[0].strip().startswith('#'):
                    continue
                values = [value.strip() for value in row]
                try:
                    if self.csv_mode == 'llh':
                        lat = float(values[0])
                        lon = float(values[1])
                        alt = (
                            float(values[2])
                            if len(values) > 2 and values[2]
                            else 0.0
                        )
                        name = values[3] if len(values) > 3 and values[3] else None
                        if not valid_lla(lat, lon, alt):
                            raise ValueError('invalid LLH coordinate')
                        raw_llh.append((lat, lon, alt, name))
                    else:
                        x = float(values[0])
                        y = float(values[1])
                        yaw = (
                            float(values[2])
                            if len(values) > 2 and values[2]
                            else None
                        )
                        name = values[3] if len(values) > 3 and values[3] else None
                        if not valid_xy_waypoint(x, y, yaw):
                            raise ValueError('invalid XY waypoint')
                        waypoints.append((x, y, yaw, name))
                except (IndexError, TypeError, ValueError) as exc:
                    self.get_logger().warn(
                        f'skipping waypoint line {line_number}: {exc}'
                    )

        if self.csv_mode == 'llh':
            lat0_rad = math.radians(self.origin_lat)
            lon0_rad = math.radians(self.origin_lon)
            x0, y0, z0 = _lla_to_ecef(lat0_rad, lon0_rad, self.origin_alt)
            for lat, lon, alt, name in raw_llh:
                x, y, z = _lla_to_ecef(
                    math.radians(lat),
                    math.radians(lon),
                    alt,
                )
                east, north, _up = _ecef_to_enu(
                    x,
                    y,
                    z,
                    x0,
                    y0,
                    z0,
                    lat0_rad,
                    lon0_rad,
                )
                waypoints.append((east, north, None, name))

        if len(waypoints) < 2:
            raise RuntimeError(
                f'route needs at least two valid waypoints; loaded {len(waypoints)}'
            )
        return waypoints

    def _on_is_calibrated(self, msg: Bool) -> None:
        if msg.data and self._calibrated_since_ns is None:
            self._calibrated_since_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(
                f'IMU calibrated; settling {self._imu_settle_sec:.0f}s before drive.'
            )

    def _imu_ready(self) -> bool:
        # is_calibrated를 받았으면 그 시점, 못 받았으면(한 번만 발행되어 놓침 대비)
        # 첫 odom 시점 기준으로 settle_sec 경과 여부를 판정한다.
        ref = (
            self._calibrated_since_ns
            if self._calibrated_since_ns is not None
            else self._first_odom_ns
        )
        if ref is None:
            return False
        elapsed_ns = self.get_clock().now().nanoseconds - ref
        return elapsed_ns >= self._imu_settle_sec * 1e9

    def _calibration_step(self, x: float, y: float, yaw: float) -> bool:
        """출발 직진 헤딩 캘리브. 아직 진행 중이면 True(주행 보류)를 돌려준다.

        로봇이 바라보는 방향 그대로 저속 직진시키면서 (x, y, imu_yaw)를 모으고,
        목표 거리를 채우면 RTK 궤적의 코스와 IMU 헤딩의 차이를 offset으로 잡는다.
        실패하면 offset 없이(런치의 고정 yaw_offset 그대로) 주행을 계속한다.
        """
        now_ns = self.get_clock().now().nanoseconds

        # Global EKF는 초기화 전까지 원점(0, 0)을 그대로 내보낸다. 그 값을 기준점
        # 으로 잡으면 EKF가 실제 좌표로 점프하는 순간 이동거리가 한 번에 목표를
        # 넘겨 캘리브가 1초 만에 끝난다(2026-08-02 실측). 위치가 유효해진 뒤에만
        # 기준점을 잡는다.
        if abs(x) < 1e-6 and abs(y) < 1e-6:
            self._publish_command(Twist(), 'GPS_CALIB_WAIT')
            return True

        if self._calib_start_ns is None:
            self._calib_start_ns = now_ns
            self._calib_origin = (x, y)
            self.get_logger().info(
                f'Heading calibration: driving straight {self._calib_distance:.1f}m '
                f'at {self._calib_speed:.2f}m/s.'
            )

        # EKF 점프 방어: 직전 샘플 대비 한 스텝에 갈 수 없는 거리를 뛰면 그때까지
        # 모은 구간을 버리고 현재 위치에서 다시 시작한다.
        if self._calib_samples:
            px, py, _ = self._calib_samples[-1]
            if math.hypot(x - px, y - py) > self._calib_max_step:
                self.get_logger().warn(
                    'Heading calibration: position jump detected, restarting segment.'
                )
                self._calib_samples.clear()
                self._calib_origin = (x, y)
                self._calib_start_ns = now_ns

        self._calib_samples.append((x, y, yaw))
        ox, oy = self._calib_origin
        travelled = math.hypot(x - ox, y - oy)
        elapsed = (now_ns - self._calib_start_ns) * 1e-9

        if travelled < self._calib_distance and elapsed < self._calib_timeout:
            cmd = Twist()
            cmd.linear.x = self._calib_speed
            self._publish_command(cmd, 'GPS_CALIB')
            return True

        self._calib_done = True
        self._solve_calibration(travelled, elapsed)
        # 2026-08-09: 여기서 빈 Twist(v=0)를 내고 True를 돌려주던 것을 없앴다.
        #
        # 캘리브는 gps_calib_speed(1.5 m/s)로 달리는데, 종료 주기에 v=0을 한 번
        # 내면 main_controller의 linear_slew_rate(0.30 m/s^2)가 그 0에서부터
        # 다시 올린다. 21:47 주행 실측: 캘리브 종료 직후 v가 1.50 -> 0.30으로
        # 떨어졌고 2.50에 도달하기까지 4.2초가 걸렸다. 1.50에서 이어갔다면
        # 2.4초면 되는 구간이라, 정지했다 출발하는 것처럼 보였다.
        #
        # False를 돌려주면 _control_step이 같은 주기 안에서 곧바로 경로 추종
        # 으로 넘어간다(호출부의 not self._calib_done 은 호출 전에 평가되므로
        # 여기서 True로 바꾼 것과 무관하게 이번 주기는 그대로 진행된다).
        return False

    def _solve_calibration(self, travelled: float, elapsed: float) -> None:
        """모은 샘플로 heading offset을 계산한다. 실패 시 offset 없이 진행."""
        def give_up(reason: str) -> None:
            self.get_logger().warn(
                f'Heading calibration failed ({reason}); '
                'keeping the configured yaw offset.'
            )

        if len(self._calib_samples) < self._calib_min_samples:
            give_up(f'only {len(self._calib_samples)} samples')
            return
        if travelled < 0.5 * self._calib_distance:
            give_up(f'travelled only {travelled:.2f}m in {elapsed:.1f}s')
            return

        xs = np.array([s[0] for s in self._calib_samples], dtype=float)
        ys = np.array([s[1] for s in self._calib_samples], dtype=float)
        dx = xs - xs.mean()
        dy = ys - ys.mean()
        # 주성분(진행 방향)과 직선성. 회전이 섞였으면 직선성이 무너진다.
        cov = np.cov(np.vstack((dx, dy)))
        evals, evecs = np.linalg.eigh(cov)
        order = np.argsort(evals)[::-1]
        evals = evals[order]
        principal = evecs[:, order[0]]
        total = float(evals.sum())
        linearity = float(evals[0] / total) if total > 0.0 else 0.0
        if linearity < self._calib_linearity:
            give_up(f'path not straight (linearity {linearity:.3f})')
            return

        course = math.atan2(float(principal[1]), float(principal[0]))
        # 주성분은 부호가 임의라 실제 진행 방향으로 맞춘다.
        if (xs[-1] - xs[0]) * math.cos(course) + (ys[-1] - ys[0]) * math.sin(course) < 0.0:
            course = ang_norm(course + math.pi)

        # IMU 헤딩 평균은 원형 평균으로 낸다(±180도 경계 문제 회피).
        yaws = np.array([s[2] for s in self._calib_samples], dtype=float)
        imu_mean = math.atan2(float(np.sin(yaws).mean()), float(np.cos(yaws).mean()))
        # 런치의 고정 yaw_offset에 더해 어댑터로 보낸다. 어댑터가 이 값으로
        # /imu/heading_enu를 다시 만들면 EKF -> /odometry/global -> follower,
        # 회피, 주차까지 같은 보정 헤딩을 쓰게 된다.
        offset = ang_norm(course - imu_mean)
        self._heading_offset = offset
        self.pub_yaw_offset.publish(
            Float64(data=ang_norm(self._configured_yaw_offset + offset))
        )
        self.get_logger().info(
            f'Heading calibration done: offset={math.degrees(offset):+.1f} deg '
            f'(course={math.degrees(course):+.1f}, imu={math.degrees(imu_mean):+.1f}, '
            f'{travelled:.2f}m, {len(self._calib_samples)} samples, '
            f'linearity={linearity:.3f}).'
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self._first_odom_ns is None:
            self._first_odom_ns = self.get_clock().now().nanoseconds
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            return
        yaw = checked_yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if yaw is None:
            return

        # 정지선 정차는 "실제로 멈춘 뒤" 3초를 세야 하므로 측정 속도가 필요하다.
        speed = msg.twist.twist.linear.x
        self._odom_speed = abs(float(speed)) if math.isfinite(speed) else None


        x = float(position.x)
        y = float(position.y)
        self._publish_control_pose(msg, x, y, yaw)
        self._control_step(x, y, yaw)

    def _publish_control_pose(
        self,
        odom: Odometry,
        x: float,
        y: float,
        yaw: float,
    ) -> None:
        pose = PoseStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        self.pub_control_pose.publish(pose)

    def _control_step(self, x: float, y: float, yaw: float) -> None:
        active_speed = self._active_linear_speed()
        self.pub_active_speed_cap.publish(Float32(data=active_speed))
        # active_speed_cap과 함께, 조기 반환보다 먼저 발행한다.
        #
        # local_avoider/parking_controller는 이 토픽을 미션 구간 게이트인
        # 동시에 follower 생존 하트비트로 쓴다(active_waypoint_timeout 1.10s).
        # 예전에는 tracker 갱신 뒤에야 발행해서, DONE/IMU_WARMUP/GPS_CALIB
        # 구간에서는 한 번도 나가지 않았다. mission:=true로 띄우면 회피 노드가
        # "follower 상태 불명"으로 판단해 활성+속도0을 내고, main_controller가
        # 그것을 채택해 차가 아예 못 움직인다. 그러면 캘리브가 끝나지 않아
        # 이 토픽도 영영 발행되지 않는 교착이 된다
        # (2026-08-08 실측: winner=LOCAL_AVOIDANCE_EXACT_ZERO 34회).
        #
        # 여기서 발행하는 것은 아직 통과 판정 전의 현재 목표 웨이포인트라,
        # 인덱스 계약은 아래 _advance_waypoint_index 결과와 동일하다.
        self._publish_active_waypoint()

        if self.finished:
            self._publish_command(Twist(), 'DONE')
            return

        # IMU(자력계 AHRS) 헤딩이 안정화될 때까지 정지 유지.
        if not self._imu_ready():
            self._publish_command(Twist(), 'IMU_WARMUP')
            return

        # 출발 직진 캘리브가 끝나기 전에는 경로 추종을 시작하지 않는다.
        if not self._calib_done and self._calibration_step(x, y, yaw):
            return

        # 보정각은 imu_enu_adapter로 보내 /imu/heading_enu -> EKF 경로에서
        # 반영된다. 여기서 또 더하면 이중 보정이 되므로 yaw는 그대로 쓴다.

        state = self.tracker.update(x, y)
        # 진단용 CTE는 진행창과 무관하게 전체 경로 최근접으로 계산해 발행한다.
        # (state.cte는 진행창이 앞설 때 과대평가되므로 로깅/진단엔 부적합)
        diag_cte, _diag_dist = self.tracker.nearest_cte(x, y)
        self.pub_cte.publish(Float32(data=float(diag_cte)))
        # 인덱스가 넘어간 결과를 같은 주기 안에 반영한다. 위에서 이미 한 번
        # 발행했지만, 여기서 다시 내지 않으면 구간 전환이 한 주기 늦어진다.
        self._advance_waypoint_index(state.passed_wp_count, x, y)
        self._publish_active_waypoint()

        # 정지선 정차. 경로 추종보다 앞에 두어 정차 중에는 조향도 내지 않는다.
        if self._stopline_step(x, y):
            return

        last_x, last_y, _last_yaw, _last_name = self.waypoints[-1]
        distance_to_finish = math.hypot(last_x - x, last_y - y)
        if state.finished and distance_to_finish < self.target_tolerance:
            self.finished = True
            self.get_logger().info('All waypoints completed.')
            self._publish_command(Twist(), 'DONE')
            return

        lookahead = _clamp(
            self.lookahead_gain * active_speed,
            self.lookahead_min,
            self.lookahead_max,
        )
        target_x, target_y = self.tracker.lookahead(state.s_here, lookahead)
        alpha, curvature = pure_pursuit(
            x,
            y,
            yaw,
            target_x,
            target_y,
        )
        target_bearing = math.atan2(target_y - y, target_x - x)
        target_distance = math.hypot(target_x - x, target_y - y)

        if self.rotating:
            if abs(alpha) < self.rotate_exit:
                self.rotating = False
        elif abs(alpha) > self.rotate_enter:
            self.rotating = True

        command = Twist()
        state_name = 'ROTATE' if self.rotating else 'DRIVE'
        if self.rotating:
            command.angular.z = _clamp(
                2.0 * alpha,
                -self.angular_speed,
                self.angular_speed,
            )
        else:
            max_curvature = max(
                abs(curvature),
                self.tracker.max_kappa_ahead(state.s_here, lookahead),
            )
            speed = active_speed
            if max_curvature > 1e-6:
                speed = min(speed, math.sqrt(self.a_lat_max / max_curvature))
            speed = max(speed, min(self.min_drive_speed, active_speed))

            angular = speed * curvature
            if abs(angular) > self.angular_speed:
                angular = math.copysign(self.angular_speed, angular)
                speed = abs(angular) / max(abs(curvature), 1e-6)

            command.linear.x = float(speed)
            command.angular.z = float(angular)

        self._publish_target(target_x, target_y, target_bearing)
        self._publish_metrics(
            target_distance,
            alpha,
            command,
        )
        self._publish_command(command, state_name)
        self._log_state(
            state_name=state_name,
            command=command,
            cte=diag_cte,
            path_distance=_diag_dist,
            alpha=alpha,
            curvature=curvature,
            lookahead=lookahead,
        )

    def _stopline_step(self, x: float, y: float) -> bool:
        """정지선 웨이포인트 앞에서 정차한다. 정차 중이면 True를 돌려준다.

        판정은 웨이포인트 *이름*으로만 한다(흰 선을 인식하는 것이 아니다).
        따라서 정차 위치의 정확도는 그 웨이포인트를 어디서 찍었는지에 달려
        있다. 정지선 바로 앞에서 찍을 것.
        """
        now_ns = self.get_clock().now().nanoseconds

        if self._stopline_brake_ns is None:
            # 이미 정차를 마친 웨이포인트는 다시 세우지 않는다. 정차 직후
            # index가 아직 넘어가지 않은 동안 재발동하는 것을 막는다.
            if self.index in self._stopline_done:
                return False
            name = self.waypoints[self.index][3]
            if not is_stopline_waypoint(name):
                return False
            wp_x, wp_y, _yaw, _name = self.waypoints[self.index]
            if math.hypot(wp_x - x, wp_y - y) > self.stopline_trigger_dist:
                return False
            self._stopline_brake_ns = now_ns
            self.get_logger().info(
                f'stopline {self.index + 1}/{len(self.waypoints)} '
                f'(name={name}): braking to a full stop'
            )

        # 1단계: 감속. 실제 속도가 임계 아래로 내려가야 계시가 시작된다.
        if self._stopline_release_ns is None:
            stopped = (
                self._odom_speed is not None
                and self._odom_speed <= self.stopline_stopped_speed
            )
            timed_out = (
                now_ns - self._stopline_brake_ns
                >= int(self.stopline_settle_timeout_sec * 1e9)
            )
            if not stopped and not timed_out:
                self._publish_command(Twist(), 'STOPLINE_BRAKING')
                return True
            if not stopped:
                # 속도가 임계 아래로 안 떨어져도 코스 한가운데 영원히 서 있을
                # 수는 없다. 계시를 시작하되 관측 속도를 남겨 원인을 찾는다.
                self.get_logger().warn(
                    f'stopline: still moving after '
                    f'{self.stopline_settle_timeout_sec:.1f}s '
                    f'(v={self._odom_speed}), starting the hold anyway'
                )
            self._stopline_release_ns = now_ns + int(self.stopline_hold_sec * 1e9)
            self.get_logger().info(
                f'stopline: stopped, holding {self.stopline_hold_sec:.1f}s'
            )

        # 2단계: 완전 정지 상태로 계시.
        if now_ns < self._stopline_release_ns:
            self._publish_command(Twist(), 'STOPLINE')
            return True

        self._stopline_done.add(self.index)
        self._stopline_brake_ns = None
        self._stopline_release_ns = None
        self.get_logger().info('stopline hold complete, resuming')
        return False

    def _advance_waypoint_index(
        self,
        passed_count: int,
        x: float,
        y: float,
    ) -> None:
        projected_index = min(passed_count, len(self.waypoints) - 1)
        new_index = min(projected_index, self.index + 1)
        if new_index <= self.index:
            return
        reached_x, reached_y, _yaw, _name = self.waypoints[self.index]
        self.get_logger().info(
            f'waypoint {self.index + 1}/{len(self.waypoints)} passed '
            f'(distance={math.hypot(reached_x - x, reached_y - y):.2f}m)'
        )
        self.index = new_index

    def _active_linear_speed(self) -> float:
        name = self.waypoints[self.index][3]
        return waypoint_speed_cap(
            name,
            self.linear_speed,
            vision_speed=self.vision_mission_speed,
            obstacle_speed=self.obstacle_mission_speed,
            parking_speed=self.parking_mission_speed,
        )

    def _publish_active_waypoint(self) -> None:
        x, y, _yaw, name = self.waypoints[self.index]
        self.pub_active_wp_idx.publish(Int32(data=self.index + 1))
        text = f'{self.index + 1}/{len(self.waypoints)}'
        if name:
            text += f', name={name}'
        text += f', x={x:.3f}, y={y:.3f}'
        self.pub_active_wp.publish(String(data=text))

    def _publish_target(
        self,
        x: float,
        y: float,
        bearing: float,
    ) -> None:
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.frame_id
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.orientation.z = math.sin(0.5 * bearing)
        target.pose.orientation.w = math.cos(0.5 * bearing)
        self.pub_target.publish(target)

    def _publish_metrics(
        self,
        target_distance: float,
        alpha: float,
        command: Twist,
    ) -> None:
        speed = abs(float(command.linear.x))
        eta = target_distance / max(speed, 1e-3)
        self.pub_metrics.publish(
            Float32MultiArray(
                data=[
                    float(target_distance),
                    math.degrees(alpha),
                    float(command.linear.x),
                    float(command.angular.z),
                    float(eta),
                ]
            )
        )

    def _publish_command(self, command: Twist, state: str) -> None:
        self.pub_cmd.publish(command)
        self.pub_state.publish(String(data=state))

    def _log_state(
        self,
        *,
        state_name: str,
        command: Twist,
        cte: float,
        path_distance: float,
        alpha: float,
        curvature: float,
        lookahead: float,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns < 500_000_000:
            return
        self.last_log_ns = now_ns
        self.get_logger().info(
            f'wp={self.index + 1}/{len(self.waypoints)}, '
            f'state={state_name}, '
            f'v={command.linear.x:.2f}, w={command.angular.z:.2f}, '
            f'cte={cte:+.2f}m, path_dist={path_distance:.2f}m, '
            f'alpha={math.degrees(alpha):+.1f}deg, '
            f'kappa={curvature:+.3f}, Ld={lookahead:.2f}m'
        )


def main() -> None:
    rclpy.init()
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
