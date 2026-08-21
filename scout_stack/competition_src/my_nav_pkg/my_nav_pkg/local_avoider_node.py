"""Official two-layout LiDAR slalom node.

This competition build keeps only the active UPPER-LOWER-UPPER /
LOWER-UPPER-LOWER path. Legacy per-box waypoint selection, generic REJOIN,
ESTOP and SAFE_CREEP state machines were removed. The node deliberately
keeps the existing completion-priority policy: a transient rollout miss
continues the last committed path command instead of taking control with a
zero-speed fail-safe.
"""

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from .pose_utils import checked_yaw_from_quaternion, finite_planar_pose
from .speed_profile import is_obstacle_waypoint
from .vfh_core import (
    ConsecutiveScanGate,
    ObstaclePassTracker,
    PatternSlalomTarget,
    VFHConfig,
    VFHPlanner,
    ang_norm,
    assess_scan_quality,
    blind_zone_memory_points,
    clamp,
    cluster_scan_points,
    obstacle_clearance_from_geometry,
    path_yaw_in_pose_frame,
    scan_to_points,
)


class LocalAvoider(Node):
    FOLLOW = "FOLLOW"
    VFH_ZONE = "VFH_ZONE"
    STALE = "STALE"
    SCAN_INVALID = "SCAN_INVALID"
    TRIGGER_CONFIRM = "TRIGGER_CONFIRM"
    PATH_REFERENCE_WAIT = "PATH_REFERENCE_WAIT"
    SPEED_CAP_STALE = "SPEED_CAP_STALE"
    OUTSIDE_OBSTACLE_ZONE = "OUTSIDE_OBSTACLE_ZONE"

    PARAM_DEFAULTS = {
        "scan_topic": "/scan",
        "odom_topic": "/odometry/global",
        "pose_topic": "/follower/control_pose",
        "cmd_topic": "/cmd_vel/avoid",
        "active_topic": "/avoid_active",
        "hint_topic": "/follower/debug/metrics",
        "path_target_topic": "/follower/debug/target",
        "active_waypoint_topic": "/follower/active_wp",
        "active_waypoint_timeout": 1.10,
        "require_obstacle_zone": True,
        "active_speed_cap_topic": "/follower/active_speed_cap",
        "active_speed_cap_timeout": 0.35,
        "require_speed_cap": False,
        "rate_hz": 20.0,
        "scan_stale_sec": 0.35,
        "odom_stale_sec": 0.35,
        "hint_stale_sec": 0.50,
        "scan_min_beams": 100,
        "scan_min_angular_span_deg": 190.0,
        "scan_min_usable_ratio": 0.50,
        "scan_front_half_angle_deg": 30.0,
        "scan_min_front_usable_ratio": 0.80,
        "laser_expected_frame": "laser",
        "require_path_reference": False,
        "allow_single_target_fallback": True,
        "trigger_dist": 3.0,
        "trigger_confirm_scans": 3,
        "cluster_join_dist": 0.25,
        "cluster_min_points": 5,
        "cluster_min_extent": 0.08,
        "zone_exit_release_timeout_sec": 3.0,
        "pattern_slalom_detection_dist": 5.0,
        "pattern_slalom_split_lateral": 0.625,
        "pattern_slalom_upper_pass_lateral": 0.00,
        "pattern_slalom_lower_pass_lateral": 1.25,
        "pattern_slalom_obstacle_spacing": 3.00,
        "pattern_slalom_rejoin_distance": 3.00,
        "pattern_slalom_lookahead": 0.40,
        "pattern_slalom_curvature_preview": 0.10,
        "pattern_slalom_min_speed": 0.50,
        "pattern_slalom_max_speed": 0.70,
        "pattern_slalom_w_max": 1.00,
        "pattern_slalom_heading_gain": 2.50,
        "pattern_slalom_lateral_gain": 1.50,
        "pattern_slalom_yaw_response_gain": 1.31,
        "pattern_slalom_lateral_accel_limit": 0.60,
        "pattern_slalom_angular_utilization": 0.85,
        "pattern_slalom_linear_accel_limit": 0.70,
        "pattern_slalom_linear_decel_limit": 1.00,
        "pattern_slalom_verify_distance": 2.50,
        "pattern_slalom_verify_steps": 40,
        "avoid_unlocked_linear_cap": 0.50,
        "avoid_tracking_margin": 0.025,
        "obstacle_width": 0.90,
        "obstacle_depth": 0.50,
        "track_merge_dist": 1.15,
        "track_confirm_scans": 3,
        "track_unconfirmed_ttl_sec": 0.60,
        "track_confirmed_ttl_sec": 2.00,
        "track_pass_recent_sec": 1.00,
        "track_same_scan_forward_merge": 0.70,
        "track_same_scan_lateral_merge": 1.10,
        "track_rear_limit": 2.20,
        "pass_margin": 2.00,
        "rejoin_lateral_tol": 0.20,
        "rejoin_yaw_tol_deg": 5.0,
        "laser_x_offset": 0.14,
        "laser_y_offset": 0.0,
        "laser_yaw_offset": 3.1416,
        "laser_fov_deg": 200.0,
        "blind_zone_memory_ttl_sec": 60.0,
        "blind_zone_memory_range_m": 6.0,
        "avoid_target_slew_deg_per_s": 60.0,
        "avoid_near_obstacle_radius_m": 0.8,
        "avoid_max_target_angle_near_obstacle_deg": 30.0,
        "sector_deg": 5.0,
        "fov_deg": 100.0,
        "d_max": 2.20,
        "d_max_front": 3.50,
        "d_max_front_half_angle_deg": 40.0,
        "r_infl": 0.35,
        "hyst_w": 0.35,
        "corridor_weight": 1.20,
        "corridor_probe": 1.0,
        "usable_road_half": 1.35,
        "usable_road_left": 2.025,
        "usable_road_right": 0.675,
        "vehicle_length": 1.40,
        "vehicle_width": 0.65,
        "boundary_margin": 0.05,
        "safety_gap": 0.10,
        "verify_distance": 0.20,
        "verify_steps": 6,
        "max_retries": 12,
        "v_min": 0.08,
        "v_max": 0.54,
        "speed_distance_gain": 0.20,
        "w_max": 0.80,
    }

    def __init__(self):
        super().__init__("local_avoider_node")
        for name, default in self.PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self.active_speed_cap_topic = str(
            self.get_parameter("active_speed_cap_topic").value
        ).strip()
        self.require_speed_cap = bool(
            self.get_parameter("require_speed_cap").value
        )
        self.require_obstacle_zone = bool(
            self.get_parameter("require_obstacle_zone").value
        )
        self.require_path_reference = bool(
            self.get_parameter("require_path_reference").value
        )
        self.allow_single_target_fallback = bool(
            self.get_parameter("allow_single_target_fallback").value
        )

        self.active_speed_cap_timeout = self._float("active_speed_cap_timeout")
        self.active_waypoint_timeout = self._float("active_waypoint_timeout")
        self.scan_stale_sec = self._float("scan_stale_sec")
        self.odom_stale_sec = self._float("odom_stale_sec")
        self.hint_stale_sec = self._float("hint_stale_sec")
        self.scan_min_beams = int(self.get_parameter("scan_min_beams").value)
        self.scan_min_angular_span = math.radians(
            self._float("scan_min_angular_span_deg")
        )
        self.scan_min_usable_ratio = self._float("scan_min_usable_ratio")
        self.scan_front_half_angle = math.radians(
            self._float("scan_front_half_angle_deg")
        )
        self.scan_min_front_usable_ratio = self._float(
            "scan_min_front_usable_ratio"
        )
        self.laser_expected_frame = str(
            self.get_parameter("laser_expected_frame").value
        ).strip()

        self.trigger_dist = self._float("trigger_dist")
        self.trigger_confirm_scans = int(
            self.get_parameter("trigger_confirm_scans").value
        )
        self.cluster_join_dist = self._float("cluster_join_dist")
        self.cluster_min_points = int(
            self.get_parameter("cluster_min_points").value
        )
        self.cluster_min_extent = self._float("cluster_min_extent")
        self.zone_exit_release_timeout_sec = self._float(
            "zone_exit_release_timeout_sec"
        )

        for name in (
            "pattern_slalom_detection_dist",
            "pattern_slalom_split_lateral",
            "pattern_slalom_upper_pass_lateral",
            "pattern_slalom_lower_pass_lateral",
            "pattern_slalom_obstacle_spacing",
            "pattern_slalom_rejoin_distance",
            "pattern_slalom_lookahead",
            "pattern_slalom_curvature_preview",
            "pattern_slalom_min_speed",
            "pattern_slalom_max_speed",
            "pattern_slalom_w_max",
            "pattern_slalom_heading_gain",
            "pattern_slalom_lateral_gain",
            "pattern_slalom_yaw_response_gain",
            "pattern_slalom_lateral_accel_limit",
            "pattern_slalom_angular_utilization",
            "pattern_slalom_linear_accel_limit",
            "pattern_slalom_linear_decel_limit",
            "pattern_slalom_verify_distance",
            "avoid_unlocked_linear_cap",
            "avoid_tracking_margin",
            "obstacle_width",
            "obstacle_depth",
            "track_merge_dist",
            "track_unconfirmed_ttl_sec",
            "track_confirmed_ttl_sec",
            "track_pass_recent_sec",
            "track_same_scan_forward_merge",
            "track_same_scan_lateral_merge",
            "track_rear_limit",
            "pass_margin",
            "rejoin_lateral_tol",
            "laser_x_offset",
            "laser_y_offset",
            "laser_yaw_offset",
            "laser_fov_deg",
            "blind_zone_memory_ttl_sec",
            "blind_zone_memory_range_m",
        ):
            setattr(self, name, self._float(name))
        self.pattern_slalom_verify_steps = int(
            self.get_parameter("pattern_slalom_verify_steps").value
        )
        self.track_confirm_scans = int(
            self.get_parameter("track_confirm_scans").value
        )
        self.rejoin_yaw_tol = math.radians(self._float("rejoin_yaw_tol_deg"))

        usable_road_left = self._float("usable_road_left")
        usable_road_right = self._float("usable_road_right")
        if usable_road_left == 0.0 and usable_road_right == 0.0:
            usable_road_left = usable_road_right = None
        elif usable_road_left <= 0.0 or usable_road_right <= 0.0:
            raise ValueError(
                "usable_road_left/right must both be positive or both zero"
            )

        rate_hz = self._float("rate_hz")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.control_period_sec = 1.0 / rate_hz
        config = VFHConfig(
            sector_deg=self._float("sector_deg"),
            fov_deg=self._float("fov_deg"),
            d_max=self._float("d_max"),
            d_max_front=self._float("d_max_front"),
            d_max_front_half_angle_deg=self._float(
                "d_max_front_half_angle_deg"
            ),
            inflation_radius=self._float("r_infl"),
            hysteresis_weight=self._float("hyst_w"),
            corridor_weight=self._float("corridor_weight"),
            corridor_probe=self._float("corridor_probe"),
            usable_road_half=self._float("usable_road_half"),
            usable_road_left=usable_road_left,
            usable_road_right=usable_road_right,
            vehicle_length=self._float("vehicle_length"),
            vehicle_width=self._float("vehicle_width"),
            boundary_margin=self._float("boundary_margin"),
            safety_gap=self._float("safety_gap"),
            verify_distance=self._float("verify_distance"),
            verify_steps=int(self.get_parameter("verify_steps").value),
            max_retries=int(self.get_parameter("max_retries").value),
            v_min=self._float("v_min"),
            v_max=self._float("v_max"),
            speed_distance_gain=self._float("speed_distance_gain"),
            w_max=self._float("w_max"),
            max_target_slew_rad=(
                math.radians(self._float("avoid_target_slew_deg_per_s"))
                / rate_hz
            ),
            near_obstacle_radius=self._float(
                "avoid_near_obstacle_radius_m"
            ),
            max_target_angle_near_obstacle_rad=math.radians(
                self._float("avoid_max_target_angle_near_obstacle_deg")
            ),
        )
        self.planner = VFHPlanner(config)

        clearance = obstacle_clearance_from_geometry(
            config.vehicle_length,
            config.vehicle_width,
            self.obstacle_depth,
            self.obstacle_width,
            config.safety_gap,
        )
        self.required_lateral_clearance = (
            clearance.lateral + self.avoid_tracking_margin
        )
        self.required_longitudinal_clearance = (
            clearance.front_face_longitudinal
        )
        self.pass_margin = max(self.pass_margin, clearance.front_face_longitudinal)
        if self.track_rear_limit + 1e-9 < self.pass_margin:
            raise ValueError("track_rear_limit must cover pass_margin")

        upper_row = self.pattern_slalom_split_lateral + 0.5 * self.obstacle_width
        lower_row = self.pattern_slalom_split_lateral - 0.5 * self.obstacle_width
        if (
            abs(upper_row - self.pattern_slalom_upper_pass_lateral)
            + 1e-9 < self.required_lateral_clearance
            or abs(lower_row - self.pattern_slalom_lower_pass_lateral)
            + 1e-9 < self.required_lateral_clearance
        ):
            raise ValueError("configured slalom pass line lacks lateral clearance")
        if (
            self.pattern_slalom_min_speed <= 0.0
            or self.pattern_slalom_max_speed < self.pattern_slalom_min_speed
            or self.pattern_slalom_w_max <= 0.0
            or self.pattern_slalom_verify_steps < 4
            or self.track_confirm_scans < 1
            or self.trigger_confirm_scans < 1
            or self.cluster_min_points < 1
            or self.cluster_join_dist <= 0.0
            or self.cluster_min_extent < 0.0
            or not 0.0 <= self.scan_min_usable_ratio <= 1.0
            or not 0.0 <= self.scan_min_front_usable_ratio <= 1.0
        ):
            raise ValueError("invalid local avoider configuration")

        self.tracker = ObstaclePassTracker(
            self.track_merge_dist,
            self.pass_margin,
            self.track_confirm_scans,
            self.track_unconfirmed_ttl_sec,
            self.track_confirmed_ttl_sec,
            self.track_pass_recent_sec,
            self.track_same_scan_forward_merge,
            self.track_same_scan_lateral_merge,
        )
        self.blind_zone_tracker = ObstaclePassTracker(
            self.track_merge_dist,
            self.pass_margin,
            self.track_confirm_scans,
            self.blind_zone_memory_ttl_sec,
            self.blind_zone_memory_ttl_sec,
            self.track_pass_recent_sec,
            self.track_same_scan_forward_merge,
            self.track_same_scan_lateral_merge,
        )
        self.pattern_slalom_target = PatternSlalomTarget(
            self.pattern_slalom_split_lateral,
            self.pattern_slalom_upper_pass_lateral,
            self.pattern_slalom_lower_pass_lateral,
            self.pattern_slalom_obstacle_spacing,
            self.pattern_slalom_rejoin_distance,
            self.pattern_slalom_lookahead,
            self.track_confirm_scans,
            0.5 * self.obstacle_depth,
            config.left_center_limit,
            config.right_center_limit,
        )
        self.trigger_gate = ConsecutiveScanGate(self.trigger_confirm_scans)

        self.scan: Optional[LaserScan] = None
        self.scan_received_ns: Optional[int] = None
        self.scan_generation = 0
        self.last_scan_header_stamp_ns: Optional[int] = None
        self.trigger_scan_generation = -1
        self.track_scan_generation = -1
        self.last_scan_quality_reason = ""
        self.odom_received_ns: Optional[int] = None
        self.hint_received_ns: Optional[int] = None
        self.active_speed_cap: Optional[float] = None
        self.active_speed_cap_received_ns: Optional[int] = None
        self.active_waypoint_received_ns: Optional[int] = None
        self.obstacle_zone_active = False
        self.pose: Optional[Tuple[float, float, float]] = None
        self.pose_frame = ""
        self.target_direction = 0.0
        self.target_distance: Optional[float] = None
        self.path_reference: Optional[Tuple[float, float, float]] = None
        self.path_target_points: List[Tuple[float, float]] = []
        self.path_target_bearing: Optional[float] = None
        self.path_target_frame = ""
        self.path_target_received_ns: Optional[int] = None
        self.last_tick_ns: Optional[int] = None
        self.mode = self.FOLLOW
        self.entry_pose: Optional[Tuple[float, float, float]] = None
        self.zone_exit_since_ns: Optional[int] = None

        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value).strip()
        hint_topic = str(self.get_parameter("hint_topic").value)
        path_target_topic = str(self.get_parameter("path_target_topic").value)
        active_waypoint_topic = str(
            self.get_parameter("active_waypoint_topic").value
        ).strip()
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        active_topic = str(self.get_parameter("active_topic").value)
        if not active_waypoint_topic or not self.active_speed_cap_topic:
            raise ValueError("waypoint and speed-cap topics must not be empty")

        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        if pose_topic:
            self.create_subscription(
                PoseStamped, pose_topic, self._on_control_pose, 10
            )
        else:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            Float32MultiArray, hint_topic, self._on_hint, 10
        )
        self.create_subscription(
            Float32,
            self.active_speed_cap_topic,
            self._on_active_speed_cap,
            10,
        )
        self.create_subscription(
            PoseStamped, path_target_topic, self._on_path_target, 10
        )
        self.create_subscription(
            String, active_waypoint_topic, self._on_active_waypoint, 10
        )

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_active = self.create_publisher(Bool, active_topic, 10)
        self.pub_state = self.create_publisher(String, "/avoid/debug/state", 10)
        self.pub_selected = self.create_publisher(
            Float32, "/avoid/debug/selected_dir", 10
        )
        self.pub_closest = self.create_publisher(
            Float32, "/avoid/debug/closest", 10
        )
        self.pub_histogram = self.create_publisher(
            Float32MultiArray, "/avoid/debug/histogram", 10
        )
        self.timer = self.create_timer(self.control_period_sec, self._tick)
        self.get_logger().info(
            "Official slalom ready: vehicle=%.2fx%.2fm, speed=%.2f-%.2fm/s, "
            "required lateral clearance=%.3fm"
            % (
                config.vehicle_length,
                config.vehicle_width,
                self.pattern_slalom_min_speed,
                self.pattern_slalom_max_speed,
                self.required_lateral_clearance,
            )
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_scan(self, msg: LaserScan) -> None:
        now_ns = self._now_ns()
        if (
            self.scan_received_ns is not None
            and now_ns < self.scan_received_ns
        ):
            self._reset_time_epoch()
        stamp_ns = self._scan_stamp_ns(msg)
        if (
            stamp_ns > 0
            and self.last_scan_header_stamp_ns is not None
            and stamp_ns <= self.last_scan_header_stamp_ns
        ):
            # A driver that republishes one cached frame must not satisfy the
            # healthy/trigger/track consecutive-scan gates.
            return
        self.scan = msg
        self.scan_received_ns = now_ns
        if stamp_ns > 0:
            self.last_scan_header_stamp_ns = stamp_ns
        self.scan_generation += 1

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = checked_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if yaw is None or not finite_planar_pose(p.x, p.y, yaw):
            return
        self._accept_pose(
            float(p.x),
            float(p.y),
            yaw,
            msg.header.frame_id,
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec),
        )

    def _on_control_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = checked_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if yaw is None or not finite_planar_pose(p.x, p.y, yaw):
            return
        self._accept_pose(
            float(p.x),
            float(p.y),
            yaw,
            msg.header.frame_id,
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec),
        )

    def _accept_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        header_stamp_ns: int,
    ) -> None:
        now_ns = self._now_ns()
        frame = str(frame_id).strip().lstrip("/")
        if (
            not frame
            or not self._stamp_is_fresh(
                now_ns,
                header_stamp_ns if header_stamp_ns > 0 else None,
                self.odom_stale_sec,
            )
        ):
            return
        if (
            self.odom_received_ns is not None
            and now_ns < self.odom_received_ns
        ):
            self._reset_time_epoch()
        self.pose = (x, y, yaw)
        self.pose_frame = frame
        self.odom_received_ns = now_ns

    def _on_hint(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2 and math.isfinite(msg.data[1]):
            self.target_direction = math.radians(float(msg.data[1]))
            if math.isfinite(msg.data[0]) and msg.data[0] > 0.0:
                self.target_distance = float(msg.data[0])
            else:
                self.target_distance = None
            self.hint_received_ns = self._now_ns()

    def _on_active_speed_cap(self, msg: Float32) -> None:
        now_ns = self._now_ns()
        if (
            self.active_speed_cap_received_ns is not None
            and now_ns < self.active_speed_cap_received_ns
        ):
            self._reset_time_epoch()
        value = float(msg.data)
        self.active_speed_cap_received_ns = now_ns
        self.active_speed_cap = (
            value if math.isfinite(value) and value > 0.0 else None
        )

    def _on_active_waypoint(self, msg: String) -> None:
        now_ns = self._now_ns()
        if (
            self.active_waypoint_received_ns is not None
            and now_ns < self.active_waypoint_received_ns
        ):
            self._reset_time_epoch()
        self.obstacle_zone_active = is_obstacle_waypoint(msg.data)
        self.active_waypoint_received_ns = now_ns

    def _on_path_target(self, msg: PoseStamped) -> None:
        point = (float(msg.pose.position.x), float(msg.pose.position.y))
        if not all(math.isfinite(value) for value in point):
            return
        q = msg.pose.orientation
        target_bearing = checked_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if target_bearing is None:
            return
        if (
            not self.path_target_points
            or math.hypot(
                point[0] - self.path_target_points[-1][0],
                point[1] - self.path_target_points[-1][1],
            )
            > 0.20
        ):
            self.path_target_points.append(point)
            self.path_target_points = self.path_target_points[-2:]
        self.path_target_bearing = target_bearing
        self.path_target_frame = msg.header.frame_id
        self.path_target_received_ns = self._now_ns()

    @staticmethod
    def _scan_stamp_ns(msg: LaserScan) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _scan_quality(self, now_ns: int):
        assert self.scan is not None
        return assess_scan_quality(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.scan.range_min,
            self.scan.range_max,
            header_stamp_ns=self._scan_stamp_ns(self.scan),
            now_ns=now_ns,
            max_age_sec=self.scan_stale_sec,
            frame_id=self.scan.header.frame_id,
            expected_frame=self.laser_expected_frame,
            min_beams=self.scan_min_beams,
            min_angular_span=self.scan_min_angular_span,
            min_usable_ratio=self.scan_min_usable_ratio,
            front_half_angle=self.scan_front_half_angle,
            front_center_angle=-self.laser_yaw_offset,
            min_front_usable_ratio=self.scan_min_front_usable_ratio,
            angle_max=self.scan.angle_max,
        )

    def _reset_avoidance(self) -> None:
        self.mode = self.FOLLOW
        self.entry_pose = None
        self.path_reference = None
        self.zone_exit_since_ns = None
        self.tracker.reset()
        self.blind_zone_tracker.reset()
        self.pattern_slalom_target.reset()
        self.planner.reset()
        self.trigger_gate.reset()
        self.track_scan_generation = -1

    def _reset_time_epoch(self) -> None:
        """Discard state tied to a previous ROS clock epoch."""
        self.scan = None
        self.scan_received_ns = None
        self.last_scan_header_stamp_ns = None
        self.scan_generation = 0
        self.trigger_scan_generation = -1
        self.track_scan_generation = -1
        self.pose = None
        self.pose_frame = ""
        self.odom_received_ns = None
        self.hint_received_ns = None
        self.active_speed_cap = None
        self.active_speed_cap_received_ns = None
        self.active_waypoint_received_ns = None
        self.obstacle_zone_active = False
        self.target_distance = None
        self.path_target_points.clear()
        self.path_target_bearing = None
        self.path_target_frame = ""
        self.path_target_received_ns = None
        self._reset_avoidance()
        self.last_tick_ns = None

    def _start_zone(self) -> None:
        assert self.pose is not None
        self.mode = self.VFH_ZONE
        self.trigger_gate.reset()
        self.track_scan_generation = -1
        self.entry_pose = self.pose
        x, y, yaw = self.pose
        now_ns = self._now_ns()
        bearing_pose = ang_norm(yaw + self.target_direction)
        path_yaw = bearing_pose
        source = "single_target_bearing"
        if (
            self._stamp_is_fresh(
                now_ns, self.path_target_received_ns, self.hint_stale_sec
            )
            and len(self.path_target_points) >= 2
        ):
            first, second = self.path_target_points[-2:]
            source_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
            assert self.path_target_bearing is not None
            path_yaw = path_yaw_in_pose_frame(
                source_yaw,
                self.path_target_bearing,
                yaw,
                self.target_direction,
            )
            source = "two_waypoint_tangent"

        if (
            self._stamp_is_fresh(
                now_ns, self.hint_received_ns, self.hint_stale_sec
            )
            and self.target_distance is not None
        ):
            self.path_reference = (
                x + self.target_distance * math.cos(bearing_pose),
                y + self.target_distance * math.sin(bearing_pose),
                path_yaw,
            )
        else:
            self.path_reference = (x, y, path_yaw)
        self.tracker.reset()
        self.blind_zone_tracker.reset()
        self.pattern_slalom_target.reset()
        self.planner.reset()
        self.get_logger().info(
            "First obstacle detected: VFH_ZONE, "
            f"route_frame={self.path_target_frame or 'unknown'}, "
            f"path_reference={source}"
        )

    def _tick(self) -> None:
        now_ns = self._now_ns()
        if self.last_tick_ns is not None and now_ns < self.last_tick_ns:
            self._reset_time_epoch()
        self.last_tick_ns = now_ns

        if self.obstacle_zone_active:
            self.zone_exit_since_ns = None
        elif self.require_obstacle_zone:
            waypoint_fresh = self._stamp_is_fresh(
                now_ns,
                self.active_waypoint_received_ns,
                self.active_waypoint_timeout,
            )
            if self.mode == self.FOLLOW:
                self._reset_avoidance()
                self._publish(
                    False,
                    Twist(),
                    self.OUTSIDE_OBSTACLE_ZONE,
                    float("inf"),
                    0.0,
                    [],
                )
                return
            if waypoint_fresh:
                if self.zone_exit_since_ns is None:
                    self.zone_exit_since_ns = now_ns
                elif (
                    self.zone_exit_release_timeout_sec > 0.0
                    and now_ns - self.zone_exit_since_ns
                    >= int(self.zone_exit_release_timeout_sec * 1e9)
                ):
                    held = (now_ns - self.zone_exit_since_ns) / 1e9
                    self.get_logger().warn(
                        "Obstacle waypoint zone ended before the committed "
                        f"trajectory completed ({held:.1f}s); releasing FOLLOW"
                    )
                    self._reset_avoidance()
                    self._publish(
                        False,
                        Twist(),
                        self.OUTSIDE_OBSTACLE_ZONE,
                        float("inf"),
                        0.0,
                        [],
                    )
                    return

        speed_cap = (
            self._fresh_active_speed_cap(now_ns)
            if self.require_speed_cap
            else None
        )
        if self.pattern_slalom_target.pattern is None:
            speed_cap = (
                self.avoid_unlocked_linear_cap
                if speed_cap is None
                else min(speed_cap, self.avoid_unlocked_linear_cap)
            )
        if self.require_speed_cap and speed_cap is None:
            self.trigger_gate.reset()
            self._publish(
                False,
                Twist(),
                self.SPEED_CAP_STALE,
                float("inf"),
                0.0,
                [],
            )
            return
        if not self._inputs_fresh(now_ns):
            self.trigger_gate.reset()
            self._publish(False, Twist(), self.STALE, float("inf"), 0.0, [])
            return

        assert self.scan is not None
        assert self.pose is not None
        quality = self._scan_quality(now_ns)
        if not quality.ok and quality.reason != self.last_scan_quality_reason:
            self.last_scan_quality_reason = quality.reason
            self.get_logger().warning(
                "LaserScan warning (non-blocking): "
                f"{quality.reason} ({quality.usable_beams}/"
                f"{quality.total_beams} usable)"
            )
        elif quality.ok:
            self.last_scan_quality_reason = "OK"

        points = scan_to_points(
            self.scan.ranges,
            self.scan.angle_min,
            self.scan.angle_increment,
            self.scan.range_min,
            min(self.scan.range_max, 8.0),
            self.laser_yaw_offset,
            self.laser_x_offset,
            self.laser_y_offset,
            math.radians(self.laser_fov_deg) / 2.0,
        )
        front_min = self._sector_min(points, -30.0, 30.0)

        if self.require_obstacle_zone and not self.obstacle_zone_active:
            if self.mode == self.FOLLOW:
                self._publish(
                    False,
                    Twist(),
                    self.OUTSIDE_OBSTACLE_ZONE,
                    front_min,
                    0.0,
                    points,
                )
                return

        if self.mode == self.FOLLOW:
            trigger_seen = (
                front_min < self.pattern_slalom_detection_dist
                and self._has_road_trigger_cluster(
                    points, self.pattern_slalom_detection_dist
                )
            )
            if self.scan_generation != self.trigger_scan_generation:
                self.trigger_scan_generation = self.scan_generation
                self.trigger_gate.update(trigger_seen)
            if self.trigger_gate.count < self.trigger_gate.required_scans:
                state = (
                    self.TRIGGER_CONFIRM
                    if self.trigger_gate.count > 0
                    else self.FOLLOW
                )
                self._publish(False, Twist(), state, front_min, 0.0, points)
                return
            if not self._path_reference_ready(now_ns):
                self._publish(
                    False,
                    Twist(),
                    self.PATH_REFERENCE_WAIT,
                    front_min,
                    0.0,
                    points,
                )
                return
            self._start_zone()

        lateral, path_heading_error, forward = self._path_errors()
        self._update_obstacle_tracks(points, forward)
        assert self.entry_pose is not None
        assert self.path_reference is not None
        points = points + blind_zone_memory_points(
            self.blind_zone_tracker.tracks,
            self.pose[0],
            self.pose[1],
            self.pose[2],
            self.entry_pose[0],
            self.entry_pose[1],
            self.path_reference[0],
            self.path_reference[1],
            self.path_reference[2],
            math.radians(self.laser_fov_deg) / 2.0,
            0.5 * self.obstacle_depth,
            0.5 * self.obstacle_width,
            self.blind_zone_memory_range_m,
        )
        front_min = self._sector_min(points, -30.0, 30.0)

        if (
            self.pattern_slalom_target.pattern is not None
            and self.pattern_slalom_target.knots
        ):
            final_forward = self.pattern_slalom_target.knots[-1][0]
            if (
                forward >= final_forward
                and abs(lateral) <= self.rejoin_lateral_tol
                and abs(path_heading_error) <= self.rejoin_yaw_tol
                and front_min >= self.trigger_dist
            ):
                self.get_logger().info("Pattern slalom complete: FOLLOW")
                self._reset_avoidance()
                self._publish(
                    False, Twist(), self.FOLLOW, front_min, 0.0, points
                )
                return

        global_target = self._target_direction(now_ns, path_heading_error)
        target_direction = self._avoid_target_direction(
            lateral, path_heading_error, forward, global_target
        )
        if target_direction is None:
            target_direction = global_target

        pattern_locked = self.pattern_slalom_target.pattern is not None
        if pattern_locked:
            planned = self.planner.plan_pattern_trajectory(
                points,
                self.pattern_slalom_target.reference_state,
                self.pattern_slalom_target.max_abs_curvature,
                forward,
                lateral,
                path_heading_error,
                min_speed=self.pattern_slalom_min_speed,
                max_speed=self.pattern_slalom_max_speed,
                angular_limit=self.pattern_slalom_w_max,
                speed_cap=speed_cap,
                heading_gain=self.pattern_slalom_heading_gain,
                lateral_gain=self.pattern_slalom_lateral_gain,
                yaw_response_gain=self.pattern_slalom_yaw_response_gain,
                lateral_acceleration_limit=(
                    self.pattern_slalom_lateral_accel_limit
                ),
                angular_utilization=self.pattern_slalom_angular_utilization,
                linear_acceleration_limit=(
                    self.pattern_slalom_linear_accel_limit
                ),
                linear_deceleration_limit=(
                    self.pattern_slalom_linear_decel_limit
                ),
                control_period_sec=self.control_period_sec,
                verify_distance=self.pattern_slalom_verify_distance,
                verify_steps=self.pattern_slalom_verify_steps,
                curvature_preview=self.pattern_slalom_curvature_preview,
            )
        else:
            planned = self.planner.plan(
                points,
                target_direction,
                lateral,
                path_heading_error,
                speed_cap=speed_cap,
                allow_sector_fallback=False,
            )

        if planned is None:
            self._publish_continuation(
                points, target_direction, front_min, speed_cap
            )
            return
        linear, angular, selected, front_min = planned
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self._publish(True, cmd, self.mode, front_min, selected, points)

    def _inputs_fresh(self, now_ns: int) -> bool:
        if self.scan is None or self.pose is None:
            return False
        scan_age = (
            float("inf")
            if self.scan_received_ns is None
            else (now_ns - self.scan_received_ns) / 1e9
        )
        odom_age = (
            float("inf")
            if self.odom_received_ns is None
            else (now_ns - self.odom_received_ns) / 1e9
        )
        return (
            0.0 <= scan_age <= self.scan_stale_sec
            and 0.0 <= odom_age <= self.odom_stale_sec
        )

    def _fresh_active_speed_cap(self, now_ns: int) -> Optional[float]:
        if (
            self.active_speed_cap is None
            or not self._stamp_is_fresh(
                now_ns,
                self.active_speed_cap_received_ns,
                self.active_speed_cap_timeout,
            )
        ):
            return None
        return self.active_speed_cap

    @staticmethod
    def _stamp_is_fresh(
        now_ns: int,
        stamp_ns: Optional[int],
        timeout_sec: float,
    ) -> bool:
        if stamp_ns is None:
            return False
        age_sec = (now_ns - stamp_ns) / 1e9
        return 0.0 <= age_sec <= timeout_sec

    def _path_reference_ready(self, now_ns: int) -> bool:
        if not self.require_path_reference:
            return True
        hint_fresh = self._stamp_is_fresh(
            now_ns,
            self.hint_received_ns,
            self.hint_stale_sec,
        )
        target_fresh = self._stamp_is_fresh(
            now_ns,
            self.path_target_received_ns,
            self.hint_stale_sec,
        )
        minimum_target_points = (
            1 if self.allow_single_target_fallback else 2
        )
        return (
            hint_fresh
            and target_fresh
            and self.target_distance is not None
            and self.path_target_bearing is not None
            and len(self.path_target_points) >= minimum_target_points
            and bool(self.pose_frame)
            and self.pose_frame
            == str(self.path_target_frame).strip().lstrip("/")
        )

    def _path_errors(self) -> Tuple[float, float, float]:
        assert self.pose is not None
        assert self.entry_pose is not None
        assert self.path_reference is not None
        x, y, yaw = self.pose
        entry_x, entry_y, _entry_yaw = self.entry_pose
        reference_x, reference_y, path_yaw = self.path_reference
        entry_dx = x - entry_x
        entry_dy = y - entry_y
        forward = math.cos(path_yaw) * entry_dx + math.sin(path_yaw) * entry_dy
        path_dx = x - reference_x
        path_dy = y - reference_y
        lateral = -math.sin(path_yaw) * path_dx + math.cos(path_yaw) * path_dy
        return lateral, ang_norm(yaw - path_yaw), forward

    def _target_direction(self, now_ns: int, path_heading_error: float) -> float:
        if (
            self._stamp_is_fresh(
                now_ns,
                self.hint_received_ns,
                self.hint_stale_sec,
            )
        ):
            return clamp(
                self.target_direction,
                -math.radians(self.planner.cfg.fov_deg),
                math.radians(self.planner.cfg.fov_deg),
            )
        return -path_heading_error

    def _has_road_trigger_cluster(
        self,
        points: Sequence[Tuple[float, float]],
        max_distance: Optional[float] = None,
    ) -> bool:
        distance_limit = (
            self.trigger_dist if max_distance is None else max_distance
        )
        for cluster in self._clusters(points):
            center_x = sum(point[0] for point in cluster) / len(cluster)
            center_y = sum(point[1] for point in cluster) / len(cluster)
            if (
                center_x > 0.0
                and math.hypot(center_x, center_y) < distance_limit
                and self.planner.cfg.contains_lateral(center_y)
            ):
                return True
        return False

    def _update_obstacle_tracks(
        self,
        points: Sequence[Tuple[float, float]],
        vehicle_forward: float,
    ) -> None:
        assert self.pose is not None
        assert self.entry_pose is not None
        assert self.path_reference is not None
        now_ns = self._now_ns()
        detections: List[Tuple[float, float]] = []
        if self.scan_generation != self.track_scan_generation:
            self.track_scan_generation = self.scan_generation
            for cluster in self._clusters(
                points,
                max_abs_angle=math.pi,
            ):
                cx = sum(point[0] for point in cluster) / len(cluster)
                cy = sum(point[1] for point in cluster) / len(cluster)
                distance = math.hypot(cx, cy)
                if distance > 5.5 or cx < -self.track_rear_limit:
                    continue
                obs_forward, obs_lateral = self._vehicle_point_to_entry(cx, cy)
                # 주행 폭 밖 수풀/펜스는 회피 장애물로 추적하지 않는다.
                if not self.planner.cfg.contains_lateral(obs_lateral):
                    continue
                detections.append((obs_forward, obs_lateral))

        for _track_id in self.tracker.update(
            detections,
            vehicle_forward,
            now_ns,
        ):
            self.get_logger().info("Obstacle track passed")
        # Same detections, long-TTL tracker: see blind_zone_tracker's
        # construction comment. Only its remembered track geometry is used.
        self.blind_zone_tracker.update(detections, vehicle_forward, now_ns)

    def _vehicle_point_to_entry(
        self,
        x_vehicle: float,
        y_vehicle: float,
    ) -> Tuple[float, float]:
        assert self.pose is not None
        assert self.entry_pose is not None
        assert self.path_reference is not None
        x, y, yaw = self.pose
        world_x = x + math.cos(yaw) * x_vehicle - math.sin(yaw) * y_vehicle
        world_y = y + math.sin(yaw) * x_vehicle + math.cos(yaw) * y_vehicle
        entry_x, entry_y, _entry_yaw = self.entry_pose
        reference_x, reference_y, path_yaw = self.path_reference
        entry_dx = world_x - entry_x
        entry_dy = world_y - entry_y
        path_dx = world_x - reference_x
        path_dy = world_y - reference_y
        return (
            math.cos(path_yaw) * entry_dx + math.sin(path_yaw) * entry_dy,
            -math.sin(path_yaw) * path_dx + math.cos(path_yaw) * path_dy,
        )

    def _clusters(
        self,
        points: Sequence[Tuple[float, float]],
        max_abs_angle: float = math.radians(110.0),
    ) -> List[List[Tuple[float, float]]]:
        return cluster_scan_points(
            points,
            max_distance=5.5,
            max_abs_angle=max_abs_angle,
            join_distance=self.cluster_join_dist,
            min_points=self.cluster_min_points,
            min_extent=self.cluster_min_extent,
        )

    @staticmethod
    def _sector_min(
        points: Sequence[Tuple[float, float]],
        min_deg: float,
        max_deg: float,
    ) -> float:
        values = []
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)
        for x, y in points:
            angle = math.atan2(y, x)
            if min_angle <= angle <= max_angle:
                values.append(math.hypot(x, y))
        return min(values) if values else 9.0

    def _publish(
        self,
        active: bool,
        cmd: Twist,
        state: str,
        front_min: float,
        selected: float,
        points: Sequence[Tuple[float, float]],
    ) -> None:
        self.pub_active.publish(Bool(data=active))
        self.pub_cmd.publish(cmd)
        self.pub_state.publish(String(data=state))
        self.pub_selected.publish(Float32(data=float(selected)))
        self.pub_closest.publish(Float32(data=float(front_min)))
        blocked = self.planner._blocked_sectors(points)
        self.pub_histogram.publish(
            Float32MultiArray(data=[1.0 if value else 0.0 for value in blocked])
        )

    def _avoid_target_direction(
        self,
        lateral: float,
        path_heading_error: float,
        vehicle_forward: float,
        global_target: float,
    ) -> Optional[float]:
        previous_pattern = self.pattern_slalom_target.pattern
        direction = self.pattern_slalom_target.direction(
            self.tracker.tracks,
            lateral,
            path_heading_error,
            vehicle_forward,
            global_target,
        )
        if previous_pattern is None and self.pattern_slalom_target.pattern:
            self.get_logger().info(
                "Pattern slalom locked: "
                f"{self.pattern_slalom_target.pattern}"
            )
        return direction

    def _publish_continuation(
        self,
        points: Sequence[Tuple[float, float]],
        target_direction: float,
        front_min: float,
        speed_cap: Optional[float],
    ) -> None:
        """Keep the committed path through a transient rollout rejection."""
        continuation = self.planner.continuation_command()
        if continuation is None:
            linear = min(
                self.planner.cfg.v_min,
                speed_cap if speed_cap is not None else self.planner.cfg.v_min,
            )
            selected = target_direction
            angular = clamp(
                2.50 * selected,
                -self.planner.cfg.w_max,
                self.planner.cfg.w_max,
            )
            self.planner.previous_direction = selected
        else:
            linear, angular, selected, front_min = continuation
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self._publish(True, cmd, self.mode, front_min, selected, points)



def main(args=None):
    rclpy.init(args=args)
    node = LocalAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
