import math
import re
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from my_nav_pkg.vfh_core import (
    ObstaclePassTracker,
    PatternSlalomTarget,
    VFHConfig,
    VFHPlanner,
    ang_norm,
    blind_zone_memory_points,
    cluster_scan_points,
    obstacle_clearance_from_geometry,
)


Point = Tuple[float, float]
Pose = List[float]


@dataclass(frozen=True)
class SimulationResult:
    completed: bool
    side_none_count: int
    planner_none_count: int
    physical_collision_count: int
    inflated_collision_count: int
    corridor_violation_count: int
    parallel_recovery_count: int = 0
    continuation_count: int = 0
    pattern_min_speed: float = 0.0
    pattern_max_speed: float = 0.0
    pattern_average_speed: float = 0.0
    pattern_max_abs_angular: float = 0.0
    pattern_max_abs_lateral_error: float = 0.0
    minimum_inflated_sat_margin: float = float("inf")
    minimum_corridor_margin: float = float("inf")
    elapsed_sec: float = 0.0
    final_forward: float = 0.0
    final_lateral: float = 0.0
    final_yaw: float = 0.0
    failure_context: str = ""


def _load_local_avoider_parameters() -> Dict[str, object]:
    """Read the simple scalar ROS parameters without requiring PyYAML."""
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "local_avoider.yaml"
    )
    parameters: Dict[str, object] = {}
    in_local_avoider = False
    pattern = re.compile(r"^    ([A-Za-z0-9_]+):\s*([^#]*?)\s*$")

    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        if raw_line.startswith("local_avoider_node:"):
            in_local_avoider = True
            continue
        if raw_line.startswith("main_controller_node:"):
            in_local_avoider = False
        if not in_local_avoider:
            continue

        match = pattern.match(raw_line)
        if match is None:
            continue
        key, raw_value = match.groups()
        raw_value = raw_value.strip()
        if not raw_value:
            continue
        if raw_value.lower() in ("true", "false"):
            value: object = raw_value.lower() == "true"
        else:
            try:
                value = (
                    int(raw_value)
                    if re.fullmatch(r"[-+]?\d+", raw_value)
                    else float(raw_value)
                )
            except ValueError:
                value = raw_value
        parameters[key] = value
    return parameters


class ClosedLoopHarness:
    """Small deterministic 2-D plant around the real ROS-independent core."""

    dt = 0.1
    beam_count = 100
    max_scan_range = 5.5

    def __init__(
        self,
        fov_deg: float = 360.0,
        road_upper: float | None = None,
        road_lower: float | None = None,
        use_asymmetric_config: bool = False,
        use_pattern_slalom: bool = True,
    ) -> None:
        # fov_deg: total physical LiDAR field of view, centered on the
        # vehicle's forward axis. A value below 360 drops beams outside
        # +/-fov_deg/2 entirely (as if the sensor never scans that
        # direction), matching a rear-blocked mount rather than reporting
        # a fake no-return there.
        self.fov_half = math.radians(fov_deg) / 2.0
        self.parameters = _load_local_avoider_parameters()
        if not use_pattern_slalom:
            raise ValueError(
                "the slim competition harness supports only official ULU/LUL slalom"
            )
        p = self.parameters
        self.dt = 1.0 / float(p["rate_hz"])
        config_kwargs = {}
        if use_asymmetric_config:
            config_kwargs = {
                "usable_road_left": float(p["usable_road_left"]),
                "usable_road_right": float(p["usable_road_right"]),
            }
        self.config = VFHConfig(
            sector_deg=float(p["sector_deg"]),
            fov_deg=float(p["fov_deg"]),
            d_max=float(p["d_max"]),
            inflation_radius=float(p["r_infl"]),
            hysteresis_weight=float(p["hyst_w"]),
            corridor_weight=float(p["corridor_weight"]),
            corridor_probe=float(p["corridor_probe"]),
            usable_road_half=float(p["usable_road_half"]),
            vehicle_length=float(p["vehicle_length"]),
            vehicle_width=float(p["vehicle_width"]),
            boundary_margin=float(p["boundary_margin"]),
            safety_gap=float(p["safety_gap"]),
            verify_distance=float(p["verify_distance"]),
            verify_steps=int(p["verify_steps"]),
            max_retries=int(p["max_retries"]),
            v_min=float(p["v_min"]),
            v_max=float(p["v_max"]),
            speed_distance_gain=float(p["speed_distance_gain"]),
            w_max=float(p["w_max"]),
            max_target_slew_rad=(
                math.radians(float(p["avoid_target_slew_deg_per_s"]))
                * self.dt
            ),
            near_obstacle_radius=float(p["avoid_near_obstacle_radius_m"]),
            max_target_angle_near_obstacle_rad=math.radians(
                float(p["avoid_max_target_angle_near_obstacle_deg"])
            ),
            **config_kwargs,
        )
        # Physical bounds are judged independently from planner decisions.
        self.road_upper = (
            road_upper
            if road_upper is not None
            else self.config.road_left_limit
        )
        self.road_lower = (
            road_lower
            if road_lower is not None
            else -self.config.road_right_limit
        )
        clearance = obstacle_clearance_from_geometry(
            self.config.vehicle_length,
            self.config.vehicle_width,
            float(p["obstacle_depth"]),
            float(p["obstacle_width"]),
            self.config.safety_gap,
        )
        self.required_lateral = (
            clearance.lateral + float(p["avoid_tracking_margin"])
        )
        self.required_longitudinal = clearance.front_face_longitudinal

    @staticmethod
    def _sector_min(
        points: Sequence[Tuple[float, float]],
        min_deg: float,
        max_deg: float,
    ) -> float:
        """Mirrors LocalAvoider._sector_min exactly."""
        values = []
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)
        for x, y in points:
            angle = math.atan2(y, x)
            if min_angle <= angle <= max_angle:
                values.append(math.hypot(x, y))
        return min(values) if values else 9.0

    @staticmethod
    def _slab(
        origin: float,
        direction: float,
        lower: float,
        upper: float,
    ) -> Tuple[float, float] | None:
        if abs(direction) < 1e-12:
            return (-1e9, 1e9) if lower <= origin <= upper else None
        first = (lower - origin) / direction
        second = (upper - origin) / direction
        return min(first, second), max(first, second)

    def _scan(self, pose: Sequence[float], boxes: Sequence[Point]) -> List[Point]:
        """Ray-cast axis-aligned 0.5 x 0.9 m boxes in LaserScan order."""
        x, y, yaw = pose
        half_depth = 0.5 * float(self.parameters["obstacle_depth"])
        half_width = 0.5 * float(self.parameters["obstacle_width"])
        points: List[Point] = []

        for index in range(self.beam_count):
            angle = -math.pi + 2.0 * math.pi * index / self.beam_count
            if abs(angle) > self.fov_half + 1e-9:
                # Outside the physical FOV: the sensor never samples this
                # direction, so it must not appear as a point OR as a
                # healthy no-return. It simply contributes nothing.
                continue
            world_angle = yaw + angle
            dx = math.cos(world_angle)
            dy = math.sin(world_angle)
            best = self.max_scan_range + 1.0
            for box_x, box_y in boxes:
                x_hit = self._slab(
                    x, dx, box_x - half_depth, box_x + half_depth
                )
                y_hit = self._slab(
                    y, dy, box_y - half_width, box_y + half_width
                )
                if x_hit is None or y_hit is None:
                    continue
                near = max(0.0, x_hit[0], y_hit[0])
                far = min(x_hit[1], y_hit[1])
                if 0.02 < near <= far and near < best:
                    best = near
            if best <= self.max_scan_range:
                points.append(
                    (best * math.cos(angle), best * math.sin(angle))
                )
        return points

    @staticmethod
    def _projection(
        corners: Sequence[Point],
        axis: Point,
    ) -> Tuple[float, float]:
        values = [x * axis[0] + y * axis[1] for x, y in corners]
        return min(values), max(values)

    def _overlaps_box(
        self,
        pose: Sequence[float],
        box: Point,
        inflation: float,
    ) -> bool:
        x, y, yaw = pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        half_length = 0.5 * self.config.vehicle_length + inflation
        half_width = 0.5 * self.config.vehicle_width + inflation
        robot = [
            (
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
            )
            for local_x, local_y in (
                (-half_length, -half_width),
                (-half_length, half_width),
                (half_length, half_width),
                (half_length, -half_width),
            )
        ]
        half_depth = 0.5 * float(self.parameters["obstacle_depth"])
        obstacle_half_width = 0.5 * float(
            self.parameters["obstacle_width"]
        )
        box_x, box_y = box
        obstacle = [
            (box_x - half_depth, box_y - obstacle_half_width),
            (box_x - half_depth, box_y + obstacle_half_width),
            (box_x + half_depth, box_y + obstacle_half_width),
            (box_x + half_depth, box_y - obstacle_half_width),
        ]
        for axis in (
            (1.0, 0.0),
            (0.0, 1.0),
            (cosine, sine),
            (-sine, cosine),
        ):
            robot_min, robot_max = self._projection(robot, axis)
            box_min, box_max = self._projection(obstacle, axis)
            if robot_max < box_min - 1e-9 or box_max < robot_min - 1e-9:
                return False
        return True

    def _box_sat_margin(
        self,
        pose: Sequence[float],
        box: Point,
        inflation: float,
    ) -> float:
        """Signed SAT margin: positive separated, negative overlapping."""
        x, y, yaw = pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        half_length = 0.5 * self.config.vehicle_length + inflation
        half_width = 0.5 * self.config.vehicle_width + inflation
        robot = [
            (
                x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y,
            )
            for local_x, local_y in (
                (-half_length, -half_width),
                (-half_length, half_width),
                (half_length, half_width),
                (half_length, -half_width),
            )
        ]
        half_depth = 0.5 * float(self.parameters["obstacle_depth"])
        obstacle_half_width = 0.5 * float(
            self.parameters["obstacle_width"]
        )
        box_x, box_y = box
        obstacle = [
            (box_x - half_depth, box_y - obstacle_half_width),
            (box_x - half_depth, box_y + obstacle_half_width),
            (box_x + half_depth, box_y + obstacle_half_width),
            (box_x + half_depth, box_y - obstacle_half_width),
        ]
        separations = []
        for axis in (
            (1.0, 0.0),
            (0.0, 1.0),
            (cosine, sine),
            (-sine, cosine),
        ):
            robot_min, robot_max = self._projection(robot, axis)
            box_min, box_max = self._projection(obstacle, axis)
            separations.append(
                max(box_min - robot_max, robot_min - box_max)
            )
        return max(separations)

    def run(
        self,
        boxes: Sequence[Point],
        forced_rollout_miss_steps: Optional[Sequence[int]] = None,
        *,
        initial_pose: Sequence[float] = (0.0, 0.0, 0.0),
        actual_yaw_response_gain: Optional[float] = None,
        angular_time_constant_sec: float = 0.0,
        linear_time_constant_sec: float = 0.0,
        command_delay_sec: float = 0.0,
        linear_speed_scale: float = 1.0,
        scan_period_sec: float = 0.0,
        pose_lateral_bias: float = 0.0,
        pose_yaw_bias: float = 0.0,
        trajectory_log: Optional[List[Tuple[float, float, float]]] = None,
        max_duration_sec: float = 180.0,
    ) -> SimulationResult:
        p = self.parameters
        if len(initial_pose) != 3:
            raise ValueError("initial_pose must contain x, y, yaw")
        dynamic_values = (
            angular_time_constant_sec,
            linear_time_constant_sec,
            command_delay_sec,
            linear_speed_scale,
            scan_period_sec,
            pose_lateral_bias,
            pose_yaw_bias,
            max_duration_sec,
        )
        if not all(math.isfinite(value) for value in dynamic_values):
            raise ValueError("2-D plant parameters must be finite")
        if (
            angular_time_constant_sec < 0.0
            or linear_time_constant_sec < 0.0
            or command_delay_sec < 0.0
            or linear_speed_scale <= 0.0
            or scan_period_sec < 0.0
            or max_duration_sec <= 0.0
            or (
                actual_yaw_response_gain is not None
                and (
                    not math.isfinite(actual_yaw_response_gain)
                    or actual_yaw_response_gain <= 0.0
                )
            )
        ):
            raise ValueError("invalid 2-D plant parameter range")
        planner = VFHPlanner(self.config)
        pass_margin = max(
            float(p["pass_margin"]),
            self.required_longitudinal,
        )
        tracker = ObstaclePassTracker(
            float(p["track_merge_dist"]),
            pass_margin,
            int(p["track_confirm_scans"]),
            float(p["track_unconfirmed_ttl_sec"]),
            float(p["track_confirmed_ttl_sec"]),
            float(p["track_pass_recent_sec"]),
            float(p["track_same_scan_forward_merge"]),
            float(p["track_same_scan_lateral_merge"]),
        )
        # Mirrors LocalAvoider.blind_zone_tracker: same detections, a
        # much longer TTL, used only to feed blind_zone_memory_points()
        # so an obstacle that simply rotated out of a narrow FOV is not
        # dropped from safety memory on the pass-tracker's short TTL.
        blind_tracker = ObstaclePassTracker(
            float(p["track_merge_dist"]),
            pass_margin,
            int(p["track_confirm_scans"]),
            float(p["blind_zone_memory_ttl_sec"]),
            float(p["blind_zone_memory_ttl_sec"]),
            float(p["track_pass_recent_sec"]),
            float(p["track_same_scan_forward_merge"]),
            float(p["track_same_scan_lateral_merge"]),
        )
        target = PatternSlalomTarget(
            classification_lateral=float(
                p["pattern_slalom_split_lateral"]
            ),
            upper_pass_lateral=float(
                p["pattern_slalom_upper_pass_lateral"]
            ),
            lower_pass_lateral=float(
                p["pattern_slalom_lower_pass_lateral"]
            ),
            obstacle_spacing=float(
                p["pattern_slalom_obstacle_spacing"]
            ),
            rejoin_distance=float(
                p["pattern_slalom_rejoin_distance"]
            ),
            lookahead=float(p["pattern_slalom_lookahead"]),
            confirm_scans=int(p["track_confirm_scans"]),
            front_face_to_center=0.5 * float(p["obstacle_depth"]),
            road_left_center_limit=self.config.left_center_limit,
            road_right_center_limit=self.config.right_center_limit,
        )

        pose: Pose = [
            float(initial_pose[0]),
            float(initial_pose[1]),
            float(initial_pose[2]),
        ]
        actual_linear_speed = 0.0
        actual_angular_speed = 0.0
        delay_steps = max(0, round(command_delay_sec / self.dt))
        command_queue: List[Tuple[float, float]] = [
            (0.0, 0.0) for _ in range(delay_steps)
        ]
        scan_stride = max(
            1,
            round(scan_period_sec / self.dt)
            if scan_period_sec > 0.0
            else 1,
        )
        latest_scan_points: List[Point] = []
        side_none = planner_none = 0
        physical_collision = inflated_collision = corridor_violation = 0
        minimum_inflated_sat_margin = float("inf")
        minimum_corridor_margin = float("inf")
        parallel_recovery = 0
        continuation_count = 0
        pattern_speeds: List[float] = []
        pattern_angulars: List[float] = []
        pattern_lateral_errors: List[float] = []
        completed = False
        failure_context = ""
        forced_miss_steps = set(forced_rollout_miss_steps or ())

        # The official C2 pattern owns entry, all three passes, and the
        # return to lateral=0. There is no separate generic REJOIN state.
        rejoin_lateral_tol = float(p["rejoin_lateral_tol"])
        rejoin_yaw_tol = math.radians(float(p["rejoin_yaw_tol_deg"]))
        trigger_dist = float(p["trigger_dist"])

        step = 0
        for step in range(int(max_duration_sec / self.dt)):
            now_ns = round(step * self.dt * 1e9)
            control_pose = [
                pose[0],
                pose[1] + pose_lateral_bias,
                ang_norm(pose[2] + pose_yaw_bias),
            ]
            new_scan = step % scan_stride == 0 or not latest_scan_points
            if new_scan:
                latest_scan_points = self._scan(pose, boxes)
                clusters = cluster_scan_points(
                    latest_scan_points,
                    max_distance=self.max_scan_range,
                    max_abs_angle=math.pi,
                    join_distance=float(p["cluster_join_dist"]),
                    min_points=int(p["cluster_min_points"]),
                    min_extent=float(p["cluster_min_extent"]),
                )
                cosine = math.cos(control_pose[2])
                sine = math.sin(control_pose[2])
                detections: List[Point] = []
                for cluster in clusters:
                    local_x = sum(point[0] for point in cluster) / len(cluster)
                    local_y = sum(point[1] for point in cluster) / len(cluster)
                    world_x = (
                        control_pose[0]
                        + cosine * local_x
                        - sine * local_y
                    )
                    world_y = (
                        control_pose[1]
                        + sine * local_x
                        + cosine * local_y
                    )
                    if (
                        local_x >= -float(p["track_rear_limit"])
                        and self.config.contains_lateral(world_y)
                    ):
                        detections.append((world_x, world_y))
                tracker.update(detections, control_pose[0], now_ns)
                blind_tracker.update(detections, control_pose[0], now_ns)

            points = latest_scan_points
            forward = control_pose[0]
            # This harness has no separate entry/path-reference frame:
            # detections above are appended directly in world coordinates, so
            # entry == reference == world origin with zero path yaw.
            safety_points = points + blind_zone_memory_points(
                blind_tracker.tracks,
                control_pose[0], control_pose[1], control_pose[2],
                0.0, 0.0, 0.0, 0.0, 0.0,
                self.fov_half,
                0.5 * float(p["obstacle_depth"]),
                0.5 * float(p["obstacle_width"]),
                float(p["blind_zone_memory_range_m"]),
            )

            pattern_reference_active = (
                target.pattern is not None and bool(target.knots)
            )
            if pattern_reference_active:
                front_min_for_finish = VFHPlanner._front_min(safety_points)
                if (
                    control_pose[0] >= target.knots[-1][0]
                    and abs(control_pose[1]) <= rejoin_lateral_tol
                    and abs(control_pose[2]) <= rejoin_yaw_tol
                    and front_min_for_finish >= trigger_dist
                ):
                    completed = True
                    break

            direction = target.direction(
                tracker.tracks,
                control_pose[1],
                control_pose[2],
                control_pose[0],
                -control_pose[2],
            )
            if direction is None:
                side_none += 1
                break

            pattern_locked = target.pattern is not None
            original_trajectory_check = planner._trajectory_is_safe
            original_pattern_check = planner._pattern_rollout_is_safe
            if step in forced_miss_steps:
                if pattern_locked:
                    planner._pattern_rollout_is_safe = (
                        lambda *_args, **_kwargs: False
                    )
                else:
                    planner._trajectory_is_safe = (
                        lambda *_args, **_kwargs: False
                    )
            try:
                if pattern_locked:
                    command = planner.plan_pattern_trajectory(
                        safety_points,
                        target.reference_state,
                        target.max_abs_curvature,
                        control_pose[0],
                        control_pose[1],
                        control_pose[2],
                        min_speed=float(p["pattern_slalom_min_speed"]),
                        max_speed=float(p["pattern_slalom_max_speed"]),
                        angular_limit=float(p["pattern_slalom_w_max"]),
                        speed_cap=None,
                        heading_gain=float(p["pattern_slalom_heading_gain"]),
                        lateral_gain=float(p["pattern_slalom_lateral_gain"]),
                        yaw_response_gain=float(
                            p["pattern_slalom_yaw_response_gain"]
                        ),
                        lateral_acceleration_limit=float(
                            p["pattern_slalom_lateral_accel_limit"]
                        ),
                        angular_utilization=float(
                            p["pattern_slalom_angular_utilization"]
                        ),
                        linear_acceleration_limit=float(
                            p["pattern_slalom_linear_accel_limit"]
                        ),
                        linear_deceleration_limit=float(
                            p["pattern_slalom_linear_decel_limit"]
                        ),
                        control_period_sec=self.dt,
                        verify_distance=float(
                            p["pattern_slalom_verify_distance"]
                        ),
                        verify_steps=int(p["pattern_slalom_verify_steps"]),
                        curvature_preview=float(
                            p.get("pattern_slalom_curvature_preview", 0.0)
                        ),
                    )
                else:
                    command = planner.plan(
                        safety_points,
                        direction,
                        control_pose[1],
                        control_pose[2],
                        speed_cap=float(p["avoid_unlocked_linear_cap"]),
                        allow_sector_fallback=False,
                    )
            finally:
                planner._trajectory_is_safe = original_trajectory_check
                planner._pattern_rollout_is_safe = original_pattern_check

            if command is None:
                continuation = planner.continuation_command()
                if continuation is None:
                    planner_none += 1
                    failure_context = (
                        f"step={step}, pose={tuple(pose)}, "
                        f"control_pose={tuple(control_pose)}, "
                        f"target={direction}, points={len(safety_points)}"
                    )
                    break
                linear, angular = continuation[0], continuation[1]
                continuation_count += 1
            else:
                linear, angular = command[0], command[1]
            if pattern_locked:
                reference = target.reference_state(pose[0])
                pattern_speeds.append(linear)
                pattern_angulars.append(abs(angular))
                pattern_lateral_errors.append(abs(pose[1] - reference.lateral))

            command_queue.append((linear, angular))
            applied_linear, applied_angular = command_queue.pop(0)
            target_linear_speed = applied_linear * linear_speed_scale
            if linear_time_constant_sec <= 1e-9:
                actual_linear_speed = target_linear_speed
            else:
                alpha_linear = 1.0 - math.exp(
                    -self.dt / linear_time_constant_sec
                )
                actual_linear_speed += alpha_linear * (
                    target_linear_speed - actual_linear_speed
                )

            if actual_yaw_response_gain is None:
                yaw_gain = (
                    float(p["pattern_slalom_yaw_response_gain"])
                    if pattern_locked
                    else 1.0
                )
            else:
                yaw_gain = actual_yaw_response_gain
            target_angular_speed = applied_angular * yaw_gain
            if angular_time_constant_sec <= 1e-9:
                actual_angular_speed = target_angular_speed
            else:
                alpha_angular = 1.0 - math.exp(
                    -self.dt / angular_time_constant_sec
                )
                actual_angular_speed += alpha_angular * (
                    target_angular_speed - actual_angular_speed
                )

            pose[0] += actual_linear_speed * math.cos(pose[2]) * self.dt
            pose[1] += actual_linear_speed * math.sin(pose[2]) * self.dt
            pose[2] = ang_norm(
                pose[2] + actual_angular_speed * self.dt
            )
            if trajectory_log is not None:
                trajectory_log.append((pose[0], pose[1], pose[2]))

            for box in boxes:
                if self._overlaps_box(pose, box, 0.0):
                    physical_collision += 1
                if self._overlaps_box(
                    pose,
                    box,
                    self.config.safety_gap,
                ):
                    inflated_collision += 1
                minimum_inflated_sat_margin = min(
                    minimum_inflated_sat_margin,
                    self._box_sat_margin(
                        pose,
                        box,
                        self.config.safety_gap,
                    ),
                )

            lateral_extent = (
                abs(math.sin(pose[2])) * 0.5 * self.config.vehicle_length
                + abs(math.cos(pose[2])) * 0.5 * self.config.vehicle_width
            )
            upper_margin = (
                self.road_upper
                - pose[1]
                - lateral_extent
                - self.config.boundary_margin
            )
            lower_margin = (
                pose[1]
                - lateral_extent
                - self.config.boundary_margin
                - self.road_lower
            )
            minimum_corridor_margin = min(
                minimum_corridor_margin,
                upper_margin,
                lower_margin,
            )
            if upper_margin < -1e-9 or lower_margin < -1e-9:
                corridor_violation += 1


        return SimulationResult(
            completed=completed,
            side_none_count=side_none,
            planner_none_count=planner_none,
            physical_collision_count=physical_collision,
            inflated_collision_count=inflated_collision,
            corridor_violation_count=corridor_violation,
            parallel_recovery_count=parallel_recovery,
            continuation_count=continuation_count,
            pattern_min_speed=(min(pattern_speeds) if pattern_speeds else 0.0),
            pattern_max_speed=(max(pattern_speeds) if pattern_speeds else 0.0),
            pattern_average_speed=(
                sum(pattern_speeds) / len(pattern_speeds)
                if pattern_speeds
                else 0.0
            ),
            pattern_max_abs_angular=(
                max(pattern_angulars) if pattern_angulars else 0.0
            ),
            pattern_max_abs_lateral_error=(
                max(pattern_lateral_errors) if pattern_lateral_errors else 0.0
            ),
            minimum_inflated_sat_margin=minimum_inflated_sat_margin,
            minimum_corridor_margin=minimum_corridor_margin,
            elapsed_sec=step * self.dt,
            final_forward=pose[0],
            final_lateral=pose[1],
            final_yaw=pose[2],
            failure_context=failure_context,
        )



class OfficialPatternClosedLoopTest(unittest.TestCase):
    """Rear-blocked LiDAR over the measured lane-2 path-frame corridor."""

    FOV_DEG = 200.0
    ROAD_UPPER = 2.025
    ROAD_LOWER = -0.675
    LANE_DIVIDER_Y = 0.625
    BOX_HALF_WIDTH = 0.45
    # Harder nominal placement: boxes touch the lane divider. No extra
    # 0.20 m outward offset is assumed in either row.
    UPPER_Y = LANE_DIVIDER_Y + BOX_HALF_WIDTH
    LOWER_Y = LANE_DIVIDER_Y - BOX_HALF_WIDTH
    FIRST_X = 3.0
    SPACING = 3.0  # 2.50 m clear gap + 0.50 m box depth

    @classmethod
    def setUpClass(cls) -> None:
        cls.harness = ClosedLoopHarness(
            fov_deg=cls.FOV_DEG,
            road_upper=cls.ROAD_UPPER,
            road_lower=cls.ROAD_LOWER,
            use_asymmetric_config=True,
            use_pattern_slalom=True,
        )

    def _boxes(self, pattern: Sequence[float]) -> List[Point]:
        return [
            (self.FIRST_X + index * self.SPACING, y)
            for index, y in enumerate(pattern)
        ]

    def assert_safe_completion(self, result: SimulationResult) -> None:
        self.assertTrue(result.completed, result)
        self.assertEqual(result.side_none_count, 0, result)
        self.assertEqual(result.planner_none_count, 0, result)
        self.assertEqual(result.physical_collision_count, 0, result)
        self.assertEqual(result.inflated_collision_count, 0, result)
        self.assertEqual(result.corridor_violation_count, 0, result)
        self.assertGreaterEqual(result.pattern_min_speed, 0.50, result)
        self.assertGreaterEqual(result.pattern_max_speed, 0.68, result)
        self.assertGreaterEqual(result.pattern_average_speed, 0.65, result)
        self.assertLessEqual(result.pattern_max_abs_angular, 1.00, result)
        self.assertLessEqual(result.pattern_max_abs_lateral_error, 0.05, result)

    def test_nominal_rows_touch_lane_divider_without_200mm_gap(self) -> None:
        self.assertAlmostEqual(
            self.UPPER_Y - self.BOX_HALF_WIDTH,
            self.LANE_DIVIDER_Y,
        )
        self.assertAlmostEqual(
            self.LOWER_Y + self.BOX_HALF_WIDTH,
            self.LANE_DIVIDER_Y,
        )

    def test_case1_upper_lower_upper(self) -> None:
        result = self.harness.run(
            self._boxes([self.UPPER_Y, self.LOWER_Y, self.UPPER_Y])
        )
        self.assert_safe_completion(result)

    def test_case2_lower_upper_lower(self) -> None:
        result = self.harness.run(
            self._boxes([self.LOWER_Y, self.UPPER_Y, self.LOWER_Y])
        )
        self.assert_safe_completion(result)

    def test_optional_200mm_outward_offset_also_completes(self) -> None:
        upper = self.UPPER_Y + 0.20
        lower = self.LOWER_Y - 0.20
        for pattern in (
            [upper, lower, upper],
            [lower, upper, lower],
        ):
            with self.subTest(pattern=pattern):
                result = self.harness.run(self._boxes(pattern))
                self.assert_safe_completion(result)

    def test_transient_rollout_misses_keep_motion_and_finish(self) -> None:
        forced_steps = (20, 90, 170, 250)
        for pattern in (
            [self.UPPER_Y, self.LOWER_Y, self.UPPER_Y],
            [self.LOWER_Y, self.UPPER_Y, self.LOWER_Y],
        ):
            with self.subTest(pattern=pattern):
                result = self.harness.run(
                    self._boxes(pattern),
                    forced_rollout_miss_steps=forced_steps,
                )
                self.assert_safe_completion(result)
                self.assertEqual(
                    result.continuation_count,
                    len(forced_steps),
                    result,
                )


if __name__ == "__main__":
    unittest.main()
