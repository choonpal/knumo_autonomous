"""ROS에 의존하지 않는 VFH+ 계산부."""

import math
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ang_norm(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi




def slew_limited_angle(
    previous: float,
    desired: float,
    max_step: float,
) -> float:
    """Move ``previous`` toward ``desired`` by at most ``max_step`` radians.

    ``max_step <= 0.0`` disables limiting and returns ``desired``
    unchanged, so callers can wire this in as an opt-in without changing
    behaviour at the default of zero.
    """
    if not all(math.isfinite(value) for value in (previous, desired, max_step)):
        raise ValueError("slew_limited_angle inputs must be finite")
    if max_step <= 0.0:
        return desired
    delta = clamp(ang_norm(desired - previous), -max_step, max_step)
    return ang_norm(previous + delta)


def cap_target_near_obstacle(
    target_direction: float,
    path_heading_error: float,
    points: Sequence[Tuple[float, float]],
    near_obstacle_radius: float,
    max_angle: float,
) -> float:
    """Clamp how far off the path heading the vehicle may end up while
    any known point (live scan or remembered blind-zone memory) sits
    within ``near_obstacle_radius``.

    ``target_direction`` is a *relative* correction for this planning
    cycle, recomputed fresh every tick -- capping it alone (as an
    earlier version of this function did) only slows how fast each
    single tick chases the goal. It does not stop the vehicle's actual
    heading from still accumulating the full turn one still-large
    per-tick request at a time, since a fresh, still-uncapped-relative
    request keeps arriving every tick. What must be bounded is the
    *resulting absolute* deviation from the path heading,
    ``path_heading_error + target_direction``: this clamps that sum to
    +/-``max_angle`` and returns the target adjusted accordingly, so
    once the vehicle is already near the cap it can gain little to no
    further deviation regardless of how large the raw target keeps
    asking for.

    Rationale: a cross-lane target computed while still close to an
    obstacle just passed can point the vehicle to rotate far enough
    that its own body -- and its own sensor cone -- sweeps back toward
    that same obstacle from a different, more dangerous angle. This was
    observed directly in a 200-degree-FOV closed-loop simulation: the
    vehicle rotated toward 90 degrees off the road heading to cross
    from one side to the other between two alternating obstacles, and
    at that heading its own field of view rotated onto the box it had
    just cleared, well inside the vehicle's own rollout footprint.

    ``near_obstacle_radius <= 0`` disables this check entirely (the
    raw target direction is returned unchanged).
    """
    values = (target_direction, path_heading_error, near_obstacle_radius, max_angle)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("cap_target_near_obstacle inputs must be finite")
    if near_obstacle_radius <= 0.0:
        return target_direction
    if max_angle < 0.0:
        raise ValueError("max_angle must be >= 0")
    if not points:
        return target_direction
    nearest = min(math.hypot(x, y) for x, y in points)
    if nearest > near_obstacle_radius:
        return target_direction
    resulting_heading = clamp(
        path_heading_error + target_direction,
        -max_angle,
        max_angle,
    )
    return resulting_heading - path_heading_error






@dataclass(frozen=True)
class ObstacleClearance:
    """차량/장애물 중심 사이에 필요한 축별 최소 간격."""

    lateral: float
    longitudinal: float
    # LiDAR cluster는 접근 중 박스 중심보다 앞면을 관측한다. 앞면 좌표를
    # 기준으로 차량 후미가 박스 뒤쪽까지 완전히 통과하는 보수 거리다.
    front_face_longitudinal: float


@dataclass(frozen=True)
class PatternTrajectoryReference:
    """Reference state of the official slalom in the local path frame.

    ``forward`` and ``lateral`` use the same path-frame convention as the
    local avoider. ``heading`` is relative to the surveyed global-path
    heading, and ``curvature`` is signed left-positive curvature in 1/m.
    The two derivatives are retained so tests and diagnostics can verify the
    C2 boundary conditions directly instead of reconstructing them by finite
    differences.
    """

    forward: float
    lateral: float
    lateral_slope: float
    lateral_second_derivative: float
    heading: float
    curvature: float


def obstacle_clearance_from_geometry(
    vehicle_length: float,
    vehicle_width: float,
    obstacle_depth: float,
    obstacle_width: float,
    safety_gap: float,
) -> ObstacleClearance:
    values = (
        vehicle_length,
        vehicle_width,
        obstacle_depth,
        obstacle_width,
        safety_gap,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("vehicle/obstacle geometry must be finite")
    if min(vehicle_length, vehicle_width, obstacle_depth, obstacle_width) <= 0.0:
        raise ValueError("vehicle/obstacle dimensions must be positive")
    if safety_gap < 0.0:
        raise ValueError("safety_gap must be non-negative")
    return ObstacleClearance(
        lateral=0.5 * (vehicle_width + obstacle_width) + safety_gap,
        longitudinal=0.5 * (vehicle_length + obstacle_depth) + safety_gap,
        front_face_longitudinal=(
            0.5 * vehicle_length + obstacle_depth + safety_gap
        ),
    )


class ConsecutiveScanGate:
    """서로 다른 scan에서 조건이 연속 확인됐을 때만 열린다."""

    def __init__(self, required_scans: int) -> None:
        self.required_scans = max(1, int(required_scans))
        self.count = 0

    def reset(self) -> None:
        self.count = 0

    def update(self, detected: bool) -> bool:
        self.count = self.count + 1 if detected else 0
        return self.count >= self.required_scans


def path_yaw_in_pose_frame(
    path_yaw_source: float,
    target_bearing_source: float,
    pose_yaw: float,
    target_heading_error: float,
) -> float:
    """Follower 경로 방향을 현재 pose 좌표계의 방향으로 변환한다."""
    target_bearing_pose = ang_norm(pose_yaw + target_heading_error)
    source_to_pose_yaw = ang_norm(
        target_bearing_pose - target_bearing_source
    )
    path_yaw_pose = ang_norm(path_yaw_source + source_to_pose_yaw)

    # 최근 두 waypoint의 저장 순서가 반대로 들어와도 진행방향을 선택한다.
    if abs(ang_norm(path_yaw_pose - target_bearing_pose)) > math.pi / 2.0:
        path_yaw_pose = ang_norm(path_yaw_pose + math.pi)
    return path_yaw_pose


@dataclass
class VFHConfig:
    sector_deg: float = 5.0
    fov_deg: float = 100.0
    d_max: float = 3.5
    # 전방은 d_max보다 멀리 본다. 측면은 d_max 그대로 둔다.
    #
    # d_max가 trigger_dist(3.0)보다 작으면, 회피 구역이 시작되는 거리에서
    # 장애물이 아직 히스토그램에 안 들어와 계획이 불가능하다. 그러면 정지하고,
    # 정지하면 더 가까워지지 못해 영영 못 본다(교착).
    # 2026-08-10 20:48 주행 실측: 박스 2.45m 앞에서 정지 -> d_max 2.20 안의
    # 점 0개 -> 25.5초 고착.
    # None이면 d_max를 그대로 쓴다(기존 동작).
    d_max_front: Optional[float] = None
    d_max_front_half_angle_deg: float = 40.0
    inflation_radius: float = 0.35
    hysteresis_weight: float = 0.35
    corridor_weight: float = 1.20
    corridor_probe: float = 1.0
    usable_road_half: float = 1.35
    vehicle_length: float = 1.40
    vehicle_width: float = 0.60
    boundary_margin: float = 0.10
    safety_gap: float = 0.10
    verify_distance: float = 0.20
    verify_steps: int = 6
    max_retries: int = 12
    v_min: float = 0.08
    v_max: float = 0.12
    # linear = v_min_base + speed_distance_gain * (front_min - 0.5) 의 기울기.
    # 회피 구간 소요시간은 v_max보다 이 값이 훨씬 크게 좌우한다(실측: 거리의
    # 64%가 front_min < 5m 라 cap이 걸리지 않는다).
    speed_distance_gain: float = 0.10
    w_max: float = 0.80
    # Maximum change per plan() call (radians) allowed between the
    # previously executed heading and the target this planner will
    # actually chase. 0.0 disables slewing (legacy behaviour: chase the
    # raw target instantly). This exists so a sudden large target swap
    # -- e.g. the pre-lock target changing as the next obstacle becomes
    # dominant in a narrow sensor FOV -- ramps in
    # gradually instead of demanding an immediate sharp turn while still
    # right beside the obstacle just passed.
    max_target_slew_rad: float = 0.0
    # While any known point (live scan or blind-zone memory) is within
    # this radius, the raw target direction is clamped to
    # +/-max_target_angle_near_obstacle_rad before slewing. 0.0 (either
    # field) disables this cap entirely. See cap_target_near_obstacle().
    near_obstacle_radius: float = 0.0
    max_target_angle_near_obstacle_rad: float = 0.0
    # Optional asymmetric road extents measured from the path centre.
    # Positive lateral is left and negative lateral is right. Keeping both
    # unset preserves the legacy symmetric usable_road_half behaviour.
    usable_road_left: Optional[float] = None
    usable_road_right: Optional[float] = None

    def __post_init__(self) -> None:
        if (
            self.usable_road_left is None
            and self.usable_road_right is None
        ):
            self.usable_road_left = self.usable_road_half
            self.usable_road_right = self.usable_road_half
        elif (
            self.usable_road_left is None
            or self.usable_road_right is None
        ):
            raise ValueError(
                "usable_road_left and usable_road_right must be set together"
            )
        assert self.usable_road_left is not None
        assert self.usable_road_right is not None
        finite_values = (
            self.sector_deg,
            self.fov_deg,
            self.d_max,
            self.inflation_radius,
            self.hysteresis_weight,
            self.corridor_weight,
            self.corridor_probe,
            self.usable_road_half,
            self.vehicle_length,
            self.vehicle_width,
            self.boundary_margin,
            self.safety_gap,
            self.verify_distance,
            self.v_min,
            self.v_max,
            self.w_max,
            self.max_target_slew_rad,
            self.near_obstacle_radius,
            self.max_target_angle_near_obstacle_rad,
            self.usable_road_left,
            self.usable_road_right,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("VFH configuration must be finite")
        if self.max_target_slew_rad < 0.0:
            raise ValueError("max_target_slew_rad must be >= 0")
        if self.near_obstacle_radius < 0.0:
            raise ValueError("near_obstacle_radius must be >= 0")
        if self.max_target_angle_near_obstacle_rad < 0.0:
            raise ValueError(
                "max_target_angle_near_obstacle_rad must be >= 0"
            )
        if (
            self.near_obstacle_radius > 0.0
            and self.max_target_angle_near_obstacle_rad <= 0.0
        ):
            raise ValueError(
                "near_obstacle_radius requires a positive "
                "max_target_angle_near_obstacle_rad"
            )
        if (
            self.sector_deg <= 0.0
            or not 0.0 < self.fov_deg <= 180.0
            or self.d_max <= 0.0
            or self.inflation_radius < 0.0
            or self.hysteresis_weight < 0.0
            or self.corridor_weight < 0.0
            or self.corridor_probe <= 0.0
            or self.usable_road_half <= 0.0
            or self.usable_road_left <= 0.0
            or self.usable_road_right <= 0.0
            or self.vehicle_length <= 0.0
            or self.vehicle_width <= 0.0
            or self.boundary_margin < 0.0
            or self.safety_gap < 0.0
            or self.verify_distance <= 0.0
            or self.verify_steps < 2
            or self.max_retries < 1
            or self.v_min < 0.0
            or self.v_max <= 0.0
            or self.v_min > self.v_max
            or self.w_max <= 0.0
        ):
            raise ValueError("invalid VFH configuration range")
        if self.left_center_limit <= 0.0 or self.right_center_limit <= 0.0:
            raise ValueError(
                "vehicle does not fit inside both configured road sides"
            )

    @property
    def road_left_limit(self) -> float:
        assert self.usable_road_left is not None
        return self.usable_road_left

    @property
    def road_right_limit(self) -> float:
        assert self.usable_road_right is not None
        return self.usable_road_right

    @property
    def left_center_limit(self) -> float:
        return max(
            0.0,
            self.road_left_limit
            - 0.5 * self.vehicle_width
            - self.boundary_margin,
        )

    @property
    def right_center_limit(self) -> float:
        return max(
            0.0,
            self.road_right_limit
            - 0.5 * self.vehicle_width
            - self.boundary_margin,
        )

    @property
    def max_center_limit(self) -> float:
        return max(self.left_center_limit, self.right_center_limit)

    @property
    def center_limit(self) -> float:
        """Legacy symmetric limit; signed code uses side-specific limits."""
        return min(self.left_center_limit, self.right_center_limit)

    @property
    def steering_left_center_limit(self) -> float:
        return max(0.0, self.road_left_limit - 0.5 * self.vehicle_width)

    @property
    def steering_right_center_limit(self) -> float:
        return max(0.0, self.road_right_limit - 0.5 * self.vehicle_width)

    def contains_lateral(self, lateral: float) -> bool:
        """Return whether a path-frame point lies inside the raw road."""
        return (
            math.isfinite(lateral)
            and -self.road_right_limit <= lateral <= self.road_left_limit
        )

    @property
    def steering_center_limit(self) -> float:
        """섹터 사전 필터 한계. 최종 경계 여유는 rollout이 검사한다."""
        return min(
            self.steering_left_center_limit,
            self.steering_right_center_limit,
        )


class VFHPlanner:
    """LaserScan 점들과 목표 방향으로 안전한 (v, w)를 선택한다."""

    def __init__(self, config: VFHConfig):
        self.cfg = config
        count = int(round(2.0 * config.fov_deg / config.sector_deg)) + 1
        self.sector_angles = [
            math.radians(-config.fov_deg + i * config.sector_deg)
            for i in range(count)
        ]
        self.previous_direction = 0.0
        self.last_path_command: Optional[
            Tuple[float, float, float, float]
        ] = None
        self.last_pattern_speed: Optional[float] = None
        self.last_plan_failure: Optional[dict] = None

    def reset(self) -> None:
        self.previous_direction = 0.0
        self.last_path_command = None
        self.last_pattern_speed = None
        self.last_plan_failure = None

    def continuation_command(
        self,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Return and commit the latest path command after a rollout miss.

        The command stored in ``last_path_command`` is the continuous
        quintic/path-heading request calculated before VFH rollout checks.
        It is intentionally *not* advertised as collision-verified.  The
        local avoider uses this only in the completion-priority competition
        setting, which explicitly prioritises retaining motion and
        command ownership over stopping.

        Committing ``previous_direction`` is important: without it, every
        failed scan would restart the target-direction slew from the same old
        value, so the vehicle could repeat one small steering request forever
        instead of continuing along the locked curve.
        """
        if self.last_path_command is None:
            return None
        command = self.last_path_command
        self.previous_direction = command[2]
        return command

    def plan(
        self,
        points: Sequence[Tuple[float, float]],
        target_direction: float,
        lateral: float,
        path_heading_error: float,
        speed_cap: Optional[float] = None,
        allow_sector_fallback: bool = True,
    ) -> Optional[Tuple[float, float, float, float]]:
        """(linear, angular, selected_direction, front_min)를 반환한다."""
        if speed_cap is None:
            effective_max = self.cfg.v_max
        else:
            if not math.isfinite(float(speed_cap)) or float(speed_cap) <= 0.0:
                raise ValueError("speed_cap must be finite and positive")
            effective_max = min(self.cfg.v_max, float(speed_cap))
        # A mission cap may intentionally be lower than the normal VFH
        # anti-stall floor. In that case the cap wins.
        effective_min = min(self.cfg.v_min, effective_max)

        blocked = self._blocked_sectors(points)
        front_min = self._front_min(points)

        linear = clamp(
            0.08 + self.cfg.speed_distance_gain * max(0.0, front_min - 0.5),
            effective_min,
            effective_max,
        )
        # While anything is still close, refuse to even *aim* at a
        # near-perpendicular cross-lane target -- rotating that far
        # would sweep the vehicle body (and its own sensor cone) back
        # toward whatever is nearby. This is checked before slewing so
        # the ramped-toward goal itself is capped, not just its rate.
        target_direction = cap_target_near_obstacle(
            target_direction,
            path_heading_error,
            points,
            self.cfg.near_obstacle_radius,
            self.cfg.max_target_angle_near_obstacle_rad,
        )
        near_obstacle_active = (
            self.cfg.near_obstacle_radius > 0.0
            and bool(points)
            and min(math.hypot(x, y) for x, y in points)
            <= self.cfg.near_obstacle_radius
        )
        # The fallback candidate search below is cost-ranked, not
        # hard-bounded: a candidate beyond the cap can still win if
        # everything closer to the target scores worse. That let the
        # planner drift out to ~80 degrees one rejected candidate at a
        # time even with the cap above in place. Excluding those
        # candidates outright (not just de-prioritising them) is what
        # actually keeps the search inside the cap while close to
        # something.
        candidate_hard_limit = (
            self.cfg.max_target_angle_near_obstacle_rad
            if near_obstacle_active
            else math.pi
        )

        # A large instantaneous target swap (e.g. the active obstacle
        # target switching to the next box) is rate-limited here so the
        # planner chases a gradually advancing goal instead of demanding
        # a sharp turn on the very first tick after the swap, while
        # still right beside the obstacle it is coming off of.
        effective_target = slew_limited_angle(
            self.previous_direction,
            target_direction,
            self.cfg.max_target_slew_rad,
        )

        # 임시 local waypoint 방향을 우선 추종한다. 여러 곡률을 짧게
        # rollout하여 10 Hz 폐루프에서 실행할 안전 명령을 선택한다.
        desired_angular = clamp(
            2.50 * effective_target,
            -self.cfg.w_max,
            self.cfg.w_max,
        )
        # The node may be configured to prioritise completion over a stop.
        # Keep the path-following command that this cycle wanted before any
        # short rollout rejected it. This is not returned as a verified VFH
        # result; LocalAvoider uses it only in completion-priority mode so a
        # one-scan rejection does not release control
        # back to the follower and trigger repeated source-switch zero cycles.
        continuation_linear = clamp(
            linear if linear > 0.0 else effective_min,
            effective_min,
            effective_max,
        )
        self.last_path_command = (
            continuation_linear,
            desired_angular,
            effective_target,
            front_min,
        )
        self.last_plan_failure = None
        speed_candidates = (
            linear,
            max(effective_min, 0.75 * linear),
            effective_min,
        )
        for candidate_linear in speed_candidates:
            for angular_scale in (1.0, 0.8, 0.6, 0.4, 0.0):
                angular = desired_angular * angular_scale
                if self._trajectory_is_safe(
                    points,
                    candidate_linear,
                    angular,
                    lateral,
                    path_heading_error,
                    effective_target,
                ):
                    self.previous_direction = effective_target
                    return (
                        candidate_linear,
                        angular,
                        effective_target,
                        front_min,
                    )

        if not allow_sector_fallback:
            self.last_plan_failure = {
                "points": len(points),
                "front_min": front_min,
                "blocked": sum(1 for b in blocked if b),
                "sectors": len(blocked),
                "target_deg": math.degrees(effective_target),
                "linear": linear,
                "candidates": 0,
                "sector_fallback": False,
            }
            return None

        candidates = self._candidate_directions(
            blocked,
            effective_target,
            lateral,
            path_heading_error,
            hard_limit=candidate_hard_limit,
        )
        for selected in candidates[: max(1, self.cfg.max_retries)]:
            turn_scale = clamp(
                1.0 - 0.90 * abs(selected),
                0.35,
                1.0,
            )
            candidate_linear = clamp(
                linear * turn_scale,
                effective_min,
                linear,
            )
            base_angular = clamp(
                2.20 * selected,
                -self.cfg.w_max,
                self.cfg.w_max,
            )
            for angular_scale in (1.0, 0.8, 0.6, 0.4):
                angular = base_angular * angular_scale
                if self._trajectory_is_safe(
                    points,
                    candidate_linear,
                    angular,
                    lateral,
                    path_heading_error,
                    selected,
                ):
                    self.previous_direction = selected
                    return candidate_linear, angular, selected, front_min

        # 2026-08-10 진단: 왜 후보가 전부 탈락했는지 남긴다. 노드가
        # RECOVERY 로그에 붙여 출력한다(원인 특정 후 제거해도 된다).
        self.last_plan_failure = {
            "points": len(points),
            "front_min": front_min,
            "blocked": sum(1 for b in blocked if b),
            "sectors": len(blocked),
            "target_deg": math.degrees(effective_target),
            "linear": linear,
            "candidates": len(candidates),
        }
        return None

    @staticmethod
    def _pattern_angular_command(
        reference: PatternTrajectoryReference,
        linear: float,
        lateral: float,
        path_heading_error: float,
        heading_gain: float,
        lateral_gain: float,
        yaw_response_gain: float,
        w_max: float,
        feedforward_curvature: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """Return command angular speed and the two Frenet tracking errors.

        The feed-forward term follows the reference curvature, optionally a
        short distance ahead to compensate command/yaw-response lag. Lateral
        and heading errors always stay referenced to the current path station,
        so preview does not cut across the quintic. Dividing by the measured
        yaw response gain compensates the Scout skid-steer's
        command-to-gyro over/under-rotation without changing the geometric
        path that is being followed.
        """
        lateral_error = lateral - reference.lateral
        heading_error = ang_norm(path_heading_error - reference.heading)
        curvature = (
            reference.curvature
            if feedforward_curvature is None
            else float(feedforward_curvature)
        )
        if not math.isfinite(curvature):
            raise ValueError("feedforward curvature must be finite")
        denominator = 1.0 - reference.curvature * lateral_error
        if abs(denominator) < 0.25:
            denominator = math.copysign(0.25, denominator or 1.0)
        actual_angular = (
            linear
            * curvature
            * math.cos(heading_error)
            / denominator
            - lateral_gain * lateral_error
            - heading_gain * heading_error
        )
        command_angular = clamp(
            actual_angular / yaw_response_gain,
            -w_max,
            w_max,
        )
        return command_angular, lateral_error, heading_error

    def _pattern_rollout_is_safe(
        self,
        points: Sequence[Tuple[float, float]],
        reference_at: Callable[[float], PatternTrajectoryReference],
        vehicle_forward: float,
        lateral: float,
        path_heading_error: float,
        linear: float,
        heading_gain: float,
        lateral_gain: float,
        yaw_response_gain: float,
        angular_limit: float,
        curvature_preview: float,
        verify_distance: float,
        verify_steps: int,
    ) -> bool:
        """Roll the actual path controller along the committed reference.

        Unlike the legacy 0.20 m constant-(v,w) check, this follows the
        changing quintic curvature over the configured horizon. It therefore
        checks the same continuous S-path the vehicle will execute, including
        the current tracking error, rectangular body sweep and asymmetric
        road corridor.
        """
        if linear <= 0.0:
            return False
        steps = max(4, int(verify_steps))
        total_time = verify_distance / linear
        dt = total_time / steps

        start_forward = vehicle_forward
        start_lateral = lateral
        start_heading = path_heading_error
        forward = vehicle_forward
        path_lateral = lateral
        heading = path_heading_error

        for _ in range(steps):
            reference = reference_at(forward)
            preview_reference = reference_at(forward + curvature_preview)
            command_angular, _lat_error, _heading_error = (
                self._pattern_angular_command(
                    reference,
                    linear,
                    path_lateral,
                    heading,
                    heading_gain,
                    lateral_gain,
                    yaw_response_gain,
                    angular_limit,
                    feedforward_curvature=preview_reference.curvature,
                )
            )
            actual_angular = command_angular * yaw_response_gain
            forward += linear * math.cos(heading) * dt
            path_lateral += linear * math.sin(heading) * dt
            heading = ang_norm(heading + actual_angular * dt)

            lateral_extent = (
                abs(math.sin(heading)) * 0.5 * self.cfg.vehicle_length
                + abs(math.cos(heading)) * 0.5 * self.cfg.vehicle_width
            )
            if (
                path_lateral + lateral_extent + self.cfg.boundary_margin
                > self.cfg.road_left_limit
                or path_lateral - lateral_extent - self.cfg.boundary_margin
                < -self.cfg.road_right_limit
            ):
                return False

            delta_forward = forward - start_forward
            delta_lateral = path_lateral - start_lateral
            cos_start = math.cos(start_heading)
            sin_start = math.sin(start_heading)
            local_x = cos_start * delta_forward + sin_start * delta_lateral
            local_y = -sin_start * delta_forward + cos_start * delta_lateral
            local_yaw = ang_norm(heading - start_heading)
            cos_yaw = math.cos(local_yaw)
            sin_yaw = math.sin(local_yaw)
            half_length = 0.5 * self.cfg.vehicle_length + self.cfg.safety_gap
            half_width = 0.5 * self.cfg.vehicle_width + self.cfg.safety_gap
            for point_x, point_y in points:
                delta_x = point_x - local_x
                delta_y = point_y - local_y
                body_x = cos_yaw * delta_x + sin_yaw * delta_y
                body_y = -sin_yaw * delta_x + cos_yaw * delta_y
                if abs(body_x) <= half_length and abs(body_y) <= half_width:
                    return False
        return True

    def plan_pattern_trajectory(
        self,
        points: Sequence[Tuple[float, float]],
        reference_at: Callable[[float], PatternTrajectoryReference],
        max_abs_curvature: Callable[[float, float, int], float],
        vehicle_forward: float,
        lateral: float,
        path_heading_error: float,
        *,
        min_speed: float,
        max_speed: float,
        angular_limit: float,
        speed_cap: Optional[float],
        heading_gain: float,
        lateral_gain: float,
        yaw_response_gain: float,
        lateral_acceleration_limit: float,
        angular_utilization: float,
        linear_acceleration_limit: float,
        linear_deceleration_limit: float,
        control_period_sec: float,
        verify_distance: float,
        verify_steps: int,
        curvature_preview: float = 0.0,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Track the locked official slalom with coupled speed/curvature.

        The minimum speed is a trajectory constraint, not a blind clamp on
        the old heading controller. Speed is first limited by curvature,
        angular authority and lateral acceleration, then the angular command
        is recomputed from that exact speed so ``w = v*kappa`` remains
        consistent. ``curvature_preview`` anticipates only feed-forward
        curvature; current lateral/heading errors are not moved forward. No
        stop/ESTOP is produced here; a rejected tick returns ``None`` so the
        caller's completion-priority continuation policy can keep the last
        committed trajectory command.
        """
        values = (
            vehicle_forward,
            lateral,
            path_heading_error,
            min_speed,
            max_speed,
            angular_limit,
            heading_gain,
            lateral_gain,
            yaw_response_gain,
            lateral_acceleration_limit,
            angular_utilization,
            linear_acceleration_limit,
            linear_deceleration_limit,
            control_period_sec,
            verify_distance,
            curvature_preview,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pattern trajectory inputs must be finite")
        if (
            min_speed <= 0.0
            or max_speed < min_speed
            or angular_limit <= 0.0
            or heading_gain < 0.0
            or lateral_gain < 0.0
            or yaw_response_gain <= 0.0
            or lateral_acceleration_limit <= 0.0
            or not 0.0 < angular_utilization <= 1.0
            or linear_acceleration_limit <= 0.0
            or linear_deceleration_limit <= 0.0
            or control_period_sec <= 0.0
            or curvature_preview < 0.0
            or verify_distance <= 0.0
            or verify_steps < 4
        ):
            raise ValueError("invalid pattern trajectory range")

        effective_max = max_speed
        if speed_cap is not None:
            if not math.isfinite(float(speed_cap)) or float(speed_cap) <= 0.0:
                raise ValueError("speed_cap must be finite and positive")
            effective_max = min(effective_max, float(speed_cap))
        if effective_max + 1e-9 < min_speed:
            self.last_plan_failure = {
                "pattern_trajectory": True,
                "speed_cap_below_floor": effective_max,
                "required_min_speed": min_speed,
            }
            return None
        effective_min = min_speed

        front_min = self._front_min(points)
        reference = reference_at(vehicle_forward)
        preview_reference = reference_at(vehicle_forward + curvature_preview)
        curve_peak = max(
            abs(reference.curvature),
            max_abs_curvature(vehicle_forward, verify_distance, verify_steps),
            1e-6,
        )
        curvature_speed = math.sqrt(
            lateral_acceleration_limit / curve_peak
        )
        angular_speed = (
            angular_limit
            * yaw_response_gain
            * angular_utilization
            / curve_peak
        )
        desired_speed = min(effective_max, curvature_speed, angular_speed)

        # Tracking error may require additional angular authority. Reduce only
        # down to the requested 0.50 m/s floor, never below it.
        lateral_error = abs(lateral - reference.lateral)
        heading_error = abs(
            ang_norm(path_heading_error - reference.heading)
        )
        error_cap = effective_max / (
            1.0 + 1.2 * lateral_error + 0.8 * heading_error
        )
        desired_speed = max(effective_min, min(desired_speed, error_cap))

        previous_speed = (
            effective_min
            if self.last_pattern_speed is None
            else clamp(self.last_pattern_speed, effective_min, effective_max)
        )
        desired_speed = clamp(
            desired_speed,
            previous_speed - linear_deceleration_limit * control_period_sec,
            previous_speed + linear_acceleration_limit * control_period_sec,
        )
        desired_speed = clamp(desired_speed, effective_min, effective_max)

        candidate_speeds: List[float] = []
        for candidate in (
            desired_speed,
            max(effective_min, desired_speed - 0.10),
            effective_min,
        ):
            if all(abs(candidate - existing) > 1e-6 for existing in candidate_speeds):
                candidate_speeds.append(candidate)

        first_command: Optional[Tuple[float, float, float, float]] = None
        for candidate_speed in candidate_speeds:
            command_angular, _lateral_error, _heading_error = (
                self._pattern_angular_command(
                    reference,
                    candidate_speed,
                    lateral,
                    path_heading_error,
                    heading_gain,
                    lateral_gain,
                    yaw_response_gain,
                    angular_limit,
                    feedforward_curvature=preview_reference.curvature,
                )
            )
            selected_direction = ang_norm(
                reference.heading - path_heading_error
            )
            command = (
                candidate_speed,
                command_angular,
                selected_direction,
                front_min,
            )
            if first_command is None:
                first_command = command
                # Completion-priority fallback must retain the exact path
                # command requested this cycle, not an unrelated VFH sector.
                self.last_path_command = command
            if self._pattern_rollout_is_safe(
                points,
                reference_at,
                vehicle_forward,
                lateral,
                path_heading_error,
                candidate_speed,
                heading_gain,
                lateral_gain,
                yaw_response_gain,
                angular_limit,
                curvature_preview,
                verify_distance,
                verify_steps,
            ):
                self.last_pattern_speed = candidate_speed
                self.previous_direction = selected_direction
                self.last_path_command = command
                self.last_plan_failure = None
                return command

        self.last_plan_failure = {
            "points": len(points),
            "front_min": front_min,
            "target_deg": math.degrees(
                ang_norm(reference.heading - path_heading_error)
            ),
            "linear": first_command[0] if first_command else desired_speed,
            "pattern_trajectory": True,
            "sector_fallback": False,
        }
        return None

    def _blocked_sectors(
        self,
        points: Sequence[Tuple[float, float]],
    ) -> List[bool]:
        blocked = [False] * len(self.sector_angles)
        front_limit = (
            self.cfg.d_max
            if self.cfg.d_max_front is None
            else max(self.cfg.d_max, self.cfg.d_max_front)
        )
        front_half = math.radians(self.cfg.d_max_front_half_angle_deg)
        for x, y in points:
            distance = math.hypot(x, y)
            angle = math.atan2(y, x)
            # 전방은 front_limit, 그 밖은 d_max. 옆 벽이 히스토그램에 들어오는
            # 것을 늘리지 않으면서 정면 장애물만 더 일찍 본다.
            limit = front_limit if abs(angle) <= front_half else self.cfg.d_max
            if distance <= 0.02 or distance > limit:
                continue
            if abs(angle) > math.radians(self.cfg.fov_deg) + 0.7:
                continue
            ratio = min(0.95, self.cfg.inflation_radius / max(distance, 1e-3))
            half_angle = math.asin(ratio)
            for index, sector in enumerate(self.sector_angles):
                if abs(ang_norm(sector - angle)) <= half_angle:
                    blocked[index] = True
        return blocked

    @staticmethod
    def _front_min(points: Sequence[Tuple[float, float]]) -> float:
        distances = [
            math.hypot(x, y)
            for x, y in points
            if x > 0.0 and abs(math.atan2(y, x)) <= math.radians(30.0)
        ]
        return min(distances) if distances else 9.0

    def _select_direction(
        self,
        blocked: Sequence[bool],
        target_direction: float,
        lateral: float,
        path_heading_error: float,
    ) -> Optional[float]:
        candidates = self._candidate_directions(
            blocked,
            target_direction,
            lateral,
            path_heading_error,
        )
        return candidates[0] if candidates else None

    def _candidate_directions(
        self,
        blocked: Sequence[bool],
        target_direction: float,
        lateral: float,
        path_heading_error: float,
        hard_limit: float = math.pi,
    ) -> List[float]:
        """``hard_limit`` excludes any sector whose *resulting absolute*
        heading, ``path_heading_error + sector``, exceeds it -- not the
        sector's own relative value -- outright, rather than merely
        de-prioritising it by cost. Unlike the cost-based pull toward
        ``target_direction``, candidates beyond this bound are never
        returned, however poorly the closer ones score against the
        histogram/corridor checks below.
        """
        scored: List[Tuple[float, float]] = []
        left_soft_limit = max(0.0, self.cfg.left_center_limit - 0.10)
        right_soft_limit = max(0.0, self.cfg.right_center_limit - 0.10)

        for sector, is_blocked in zip(self.sector_angles, blocked):
            if is_blocked or abs(path_heading_error + sector) > hard_limit:
                continue
            predicted_lateral = lateral + self.cfg.corridor_probe * math.sin(
                path_heading_error + sector
            )
            # 여기서는 명백한 도로 밖 방향만 제거한다. 차체 회전까지
            # 엄격한 경계 검사는 아래의 직사각 footprint rollout이 담당한다.
            if (
                predicted_lateral > self.cfg.steering_left_center_limit
                or predicted_lateral
                < -self.cfg.steering_right_center_limit
            ):
                continue

            corridor_penalty = self.cfg.corridor_weight * max(
                0.0,
                predicted_lateral - left_soft_limit,
                -predicted_lateral - right_soft_limit,
            )
            cost = (
                abs(ang_norm(sector - target_direction))
                + self.cfg.hysteresis_weight
                * abs(ang_norm(sector - self.previous_direction))
                + corridor_penalty
            )
            scored.append((cost, sector))

        scored.sort(key=lambda item: item[0])
        separated: List[float] = []
        separation = math.radians(max(10.0, 2.0 * self.cfg.sector_deg))

        # 먼 장애물이 같은 sector에 있어도 짧은 rollout은 안전할 수 있다.
        # 목표 주변은 histogram을 advisory로만 쓰고 exact footprint로 판정한다.
        fov = math.radians(self.cfg.fov_deg)
        offsets = (0.0, -10.0, 10.0, -20.0, 20.0)
        for offset_deg in offsets:
            sector = clamp(
                target_direction + math.radians(offset_deg),
                -fov,
                fov,
            )
            if abs(path_heading_error + sector) > hard_limit:
                continue
            predicted_lateral = lateral + self.cfg.corridor_probe * math.sin(
                path_heading_error + sector
            )
            if (
                -self.cfg.steering_right_center_limit
                <= predicted_lateral
                <= self.cfg.steering_left_center_limit
            ):
                if all(
                    abs(ang_norm(sector - chosen)) >= separation
                    for chosen in separated
                ):
                    separated.append(sector)

        for _cost, sector in scored:
            if all(
                abs(ang_norm(sector - chosen)) >= separation
                for chosen in separated
            ):
                separated.append(sector)
        return separated

    def _trajectory_is_safe(
        self,
        points: Sequence[Tuple[float, float]],
        linear: float,
        angular: float,
        lateral: float,
        path_heading_error: float,
        target_direction: Optional[float] = None,
    ) -> bool:
        steps = max(2, int(self.cfg.verify_steps))
        if abs(linear) > 0.03:
            total_time = min(
                1.8,
                self.cfg.verify_distance / abs(linear),
            )
        else:
            total_time = 0.8
        dt = total_time / steps

        x = y = yaw = 0.0
        sim_angular = angular

        for _ in range(steps):
            if target_direction is not None:
                yaw_error = ang_norm(target_direction - yaw)
                if abs(yaw_error) <= math.radians(2.0):
                    sim_angular = 0.0
                elif sim_angular * yaw_error <= 0.0:
                    sim_angular = 0.0
                else:
                    sim_angular = math.copysign(
                        min(abs(angular), 2.20 * abs(yaw_error)),
                        yaw_error,
                    )
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw = ang_norm(yaw + sim_angular * dt)

            relative_yaw = ang_norm(path_heading_error + yaw)
            path_y = lateral + math.sin(path_heading_error) * x + math.cos(
                path_heading_error
            ) * y
            lateral_extent = (
                abs(math.sin(relative_yaw)) * 0.5 * self.cfg.vehicle_length
                + abs(math.cos(relative_yaw)) * 0.5 * self.cfg.vehicle_width
            )
            if (
                path_y + lateral_extent + self.cfg.boundary_margin
                > self.cfg.road_left_limit
                or path_y - lateral_extent - self.cfg.boundary_margin
                < -self.cfg.road_right_limit
            ):
                return False

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            half_length = 0.5 * self.cfg.vehicle_length + self.cfg.safety_gap
            half_width = 0.5 * self.cfg.vehicle_width + self.cfg.safety_gap
            for point_x, point_y in points:
                delta_x = point_x - x
                delta_y = point_y - y
                local_x = cos_yaw * delta_x + sin_yaw * delta_y
                local_y = -sin_yaw * delta_x + cos_yaw * delta_y
                if abs(local_x) <= half_length and abs(local_y) <= half_width:
                    return False

        return True

    def command_is_safe(
        self,
        points: Sequence[Tuple[float, float]],
        linear: float,
        angular: float,
        lateral: float,
        path_heading_error: float,
        target_direction: Optional[float] = None,
    ) -> bool:
        """복구 명령도 일반 후보와 같은 footprint/corridor로 검증한다."""
        return self._trajectory_is_safe(
            points,
            linear,
            angular,
            lateral,
            path_heading_error,
            target_direction,
        )



@dataclass
class ObstacleTrack:
    track_id: int
    forward: float
    lateral: float
    last_seen_ns: int
    seen_count: int
    was_ahead: bool
    passed: bool = False
    # Running observed envelope, distinct from the EMA-smoothed
    # forward/lateral used for association/pass-confirmation above.
    # forward/lateral tracks whichever surface point the sensor last
    # happened to see (often a near corner/edge, not the box centre),
    # so it is the wrong anchor for reconstructing a safety footprint.
    # These bounds accumulate everything actually observed of this
    # object and stay None until update() has real data for them, so
    # every existing/manual ObstacleTrack(...) call site is unaffected.
    min_forward: Optional[float] = None
    max_forward: Optional[float] = None
    min_lateral: Optional[float] = None
    max_lateral: Optional[float] = None


class ObstaclePassTracker:
    """스캔별 cluster 중심을 연결해 각 장애물의 통과 상태를 추적한다."""

    def __init__(
        self,
        merge_distance: float,
        pass_margin: float,
        confirm_scans: int,
        unconfirmed_ttl_sec: float = 0.60,
        confirmed_ttl_sec: float = 2.00,
        pass_recent_sec: float = 1.00,
        same_scan_forward_merge: float = 0.75,
        same_scan_lateral_merge: float = 1.10,
    ) -> None:
        values = (
            merge_distance,
            pass_margin,
            unconfirmed_ttl_sec,
            confirmed_ttl_sec,
            pass_recent_sec,
            same_scan_forward_merge,
            same_scan_lateral_merge,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("obstacle tracker configuration must be finite")
        if (
            merge_distance <= 0.0
            or pass_margin < 0.0
            or confirm_scans < 1
            or unconfirmed_ttl_sec <= 0.0
            or confirmed_ttl_sec <= 0.0
            or pass_recent_sec <= 0.0
            or same_scan_forward_merge < 0.0
            or same_scan_lateral_merge < 0.0
        ):
            raise ValueError("invalid obstacle tracker configuration")
        self.merge_distance = merge_distance
        self.pass_margin = pass_margin
        self.confirm_scans = max(1, int(confirm_scans))
        self.unconfirmed_ttl_ns = int(max(0.01, unconfirmed_ttl_sec) * 1e9)
        self.confirmed_ttl_ns = int(max(0.01, confirmed_ttl_sec) * 1e9)
        self.pass_recent_ns = int(max(0.01, pass_recent_sec) * 1e9)
        self.same_scan_forward_merge = same_scan_forward_merge
        self.same_scan_lateral_merge = same_scan_lateral_merge
        self.tracks: List[ObstacleTrack] = []
        self.next_track_id = 1

    def reset(self) -> None:
        self.tracks.clear()
        self.next_track_id = 1

    @staticmethod
    def _age_ns(track: ObstacleTrack, now_ns: int) -> int:
        if now_ns < track.last_seen_ns:
            # ROS/simulation clock epoch changed. A future-dated track must
            # never be reused or counted as freshly passed.
            return 2**63 - 1
        return now_ns - track.last_seen_ns

    def _coalesce_same_scan(
        self,
        detections: Sequence[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """Merge fragments of one physical box before scan confirmation.

        A box front/side can be split by invalid beams.  Counting those
        fragments independently would confirm one obstacle several times in
        one scan.  Connected components use separate forward/lateral limits
        derived from the configured box dimensions.
        """
        valid = [
            (float(forward), float(lateral))
            for forward, lateral in detections
            if math.isfinite(forward) and math.isfinite(lateral)
        ]
        if (
            len(valid) < 2
            or self.same_scan_forward_merge <= 0.0
            or self.same_scan_lateral_merge <= 0.0
        ):
            return valid

        remaining = set(range(len(valid)))
        merged: List[Tuple[float, float]] = []
        while remaining:
            component = {remaining.pop()}
            changed = True
            while changed:
                changed = False
                for candidate in tuple(remaining):
                    forward, lateral = valid[candidate]
                    if any(
                        abs(forward - valid[index][0])
                        <= self.same_scan_forward_merge
                        and abs(lateral - valid[index][1])
                        <= self.same_scan_lateral_merge
                        for index in component
                    ):
                        component.add(candidate)
                        remaining.remove(candidate)
                        changed = True
            merged.append(
                (
                    sum(valid[index][0] for index in component)
                    / len(component),
                    sum(valid[index][1] for index in component)
                    / len(component),
                )
            )
        return merged

    def _prune_stale(self, now_ns: int) -> None:
        fresh_tracks: List[ObstacleTrack] = []
        for track in self.tracks:
            # 통과 완료 기록은 같은 박스의 다른 면을 다시 세지 않도록
            # zone이 끝날 때까지 보존한다.
            if track.passed:
                fresh_tracks.append(track)
                continue
            ttl_ns = (
                self.confirmed_ttl_ns
                if track.seen_count >= self.confirm_scans
                else self.unconfirmed_ttl_ns
            )
            if self._age_ns(track, now_ns) <= ttl_ns:
                fresh_tracks.append(track)
        self.tracks = fresh_tracks

    def update(
        self,
        detections: Sequence[Tuple[float, float]],
        vehicle_forward: float,
        now_ns: int,
    ) -> List[int]:
        self._prune_stale(now_ns)
        detections = self._coalesce_same_scan(detections)
        if not math.isfinite(vehicle_forward):
            return []
        matched_ids = set()
        for forward, lateral in detections:
            candidates = [
                track
                for track in self.tracks
                # 한 scan의 서로 다른 cluster가 같은 track을 중복 갱신하지
                # 않게 association을 일대일로 제한한다.
                if track.track_id not in matched_ids
                # 이미 통과한 track은 차량 뒤쪽에 남은 같은 박스 면만
                # 흡수한다. 차량 앞의 다음 박스와 합치면 다음 회피 목표를
                # 잃을 수 있다.
                if not track.passed or forward <= vehicle_forward + 0.20
                if math.hypot(
                    track.forward - forward,
                    track.lateral - lateral,
                )
                <= self.merge_distance
            ]
            if candidates:
                track = min(
                    candidates,
                    key=lambda item: math.hypot(
                        item.forward - forward,
                        item.lateral - lateral,
                    ),
                )
                alpha = 0.3
                track.forward = (1.0 - alpha) * track.forward + alpha * forward
                track.lateral = (1.0 - alpha) * track.lateral + alpha * lateral
                track.last_seen_ns = now_ns
                track.seen_count += 1
                track.min_forward = (
                    forward
                    if track.min_forward is None
                    else min(track.min_forward, forward)
                )
                track.max_forward = (
                    forward
                    if track.max_forward is None
                    else max(track.max_forward, forward)
                )
                track.min_lateral = (
                    lateral
                    if track.min_lateral is None
                    else min(track.min_lateral, lateral)
                )
                track.max_lateral = (
                    lateral
                    if track.max_lateral is None
                    else max(track.max_lateral, lateral)
                )
                if forward > vehicle_forward + 0.2:
                    track.was_ahead = True
            else:
                track = ObstacleTrack(
                    track_id=self.next_track_id,
                    forward=forward,
                    lateral=lateral,
                    min_forward=forward,
                    max_forward=forward,
                    min_lateral=lateral,
                    max_lateral=lateral,
                    last_seen_ns=now_ns,
                    seen_count=1,
                    was_ahead=forward > vehicle_forward + 0.2,
                )
                self.next_track_id += 1
                self.tracks.append(track)
            matched_ids.add(track.track_id)

        newly_passed: List[int] = []
        for track in self.tracks:
            if (
                not track.passed
                and track.was_ahead
                and track.seen_count >= self.confirm_scans
                and self._age_ns(track, now_ns) <= self.pass_recent_ns
                and vehicle_forward > track.forward + self.pass_margin
            ):
                track.passed = True
                newly_passed.append(track.track_id)
        return newly_passed


def entry_frame_to_vehicle_local(
    forward: float,
    lateral: float,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    entry_x: float,
    entry_y: float,
    reference_x: float,
    reference_y: float,
    path_yaw: float,
) -> Tuple[float, float]:
    """Invert the avoider's entry/path-frame projection back to a
    current-tick vehicle-local (x, y).

    ``forward`` is measured from ``(entry_x, entry_y)`` and ``lateral``
    from ``(reference_x, reference_y)``, both along the same
    ``path_yaw`` axis -- this is the exact algebraic inverse of
    ``LocalAvoider._vehicle_point_to_entry``/``_path_errors`` in
    local_avoider_node.py, and must stay in sync with them. A stored
    obstacle position is static in the world; only the vehicle moves,
    so re-deriving its current bearing/range every tick is what lets a
    track that has left the sensor's field of view still be placed
    correctly for a blind-zone safety check.
    """
    values = (
        forward,
        lateral,
        pose_x,
        pose_y,
        pose_yaw,
        entry_x,
        entry_y,
        reference_x,
        reference_y,
        path_yaw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("entry_frame_to_vehicle_local inputs must be finite")
    cos_path = math.cos(path_yaw)
    sin_path = math.sin(path_yaw)
    # Solve the 2x2 linear system built from the two projections at once
    # (forward projects from entry_*, lateral projects from reference_*)
    # rather than assuming a single shared origin.
    a = forward + cos_path * entry_x + sin_path * entry_y
    b = lateral - sin_path * reference_x + cos_path * reference_y
    world_x = cos_path * a - sin_path * b
    world_y = sin_path * a + cos_path * b
    delta_x = world_x - pose_x
    delta_y = world_y - pose_y
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    return (
        cos_yaw * delta_x + sin_yaw * delta_y,
        -sin_yaw * delta_x + cos_yaw * delta_y,
    )


def _rectangle_perimeter_offsets(
    half_length: float,
    half_width: float,
    step: float,
) -> List[Tuple[float, float]]:
    """Sample points around a centred, path-aligned rectangle's edges.

    Always includes the four corners even if the requested step would
    otherwise skip them, so a degenerate/tiny box still yields a shape.
    """
    if half_length < 0.0 or half_width < 0.0 or step <= 0.0:
        raise ValueError("invalid rectangle sampling configuration")
    if half_length == 0.0 and half_width == 0.0:
        return [(0.0, 0.0)]

    def _samples(span: float) -> List[float]:
        # Points from -span to +span inclusive, spaced at most `step`
        # apart, always including both endpoints.
        if span <= 0.0:
            return [0.0]
        count = max(1, int(math.ceil((2.0 * span) / step)))
        return [-span + (2.0 * span) * i / count for i in range(count + 1)]

    offsets: set = set()
    for lon in _samples(half_length):
        offsets.add((lon, -half_width))
        offsets.add((lon, half_width))
    for lat in _samples(half_width):
        offsets.add((-half_length, lat))
        offsets.add((half_length, lat))
    return sorted(offsets)


def blind_zone_memory_points(
    tracks: Sequence[ObstacleTrack],
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
    entry_x: float,
    entry_y: float,
    reference_x: float,
    reference_y: float,
    path_yaw: float,
    fov_half_angle: float,
    obstacle_half_depth: float,
    obstacle_half_width: float,
    max_memory_range: float,
    sample_step: float = 0.15,
    envelope_margin: float = 0.0,
) -> List[Tuple[float, float]]:
    """Synthesize vehicle-local safety points for tracked obstacles that
    are currently outside the sensor's physical field of view.

    A rear-blocked or narrow-FOV LiDAR cannot refute an obstacle it
    never looks at. Without this, ``VFHPlanner._trajectory_is_safe``
    only ever sees this tick's raw scan and will happily plan straight
    through a box the instant it rotates past the visible cone -- most
    dangerously right after passing it, when a re-acquisition manoeuvre
    can swing the vehicle back toward an obstacle it just cleared. This
    function does not change tracking/pass-confirmation at all; it only
    adds phantom points to the *safety* check for still-tracked
    (unpassed, not yet TTL-pruned), still-nearby obstacles whose bearing
    currently falls outside +/-``fov_half_angle``. A part of a box that
    is currently visible is left to the live scan, not memory.
    """
    if not math.isfinite(fov_half_angle) or fov_half_angle <= 0.0:
        raise ValueError("fov_half_angle must be finite and positive")
    if fov_half_angle >= math.pi:
        # Full circle: nothing is ever out of view, so there is nothing
        # to remember.
        return []
    if (
        not math.isfinite(obstacle_half_depth)
        or not math.isfinite(obstacle_half_width)
        or obstacle_half_depth < 0.0
        or obstacle_half_width < 0.0
    ):
        raise ValueError("obstacle half-extents must be finite and >= 0")
    if not math.isfinite(max_memory_range) or max_memory_range <= 0.0:
        raise ValueError("max_memory_range must be finite and positive")
    if not math.isfinite(envelope_margin) or envelope_margin < 0.0:
        raise ValueError("envelope_margin must be finite and >= 0")

    points: List[Tuple[float, float]] = []
    for track in tracks:
        if track.passed:
            continue
        has_envelope = (
            track.min_forward is not None
            and track.max_forward is not None
            and track.min_lateral is not None
            and track.max_lateral is not None
        )
        if has_envelope:
            # track.forward/lateral is an EMA of whichever surface point
            # the sensor happened to see most recently -- often a near
            # corner, not the box centre -- so it is the wrong anchor
            # for a safety footprint. The accumulated observed envelope
            # is a direct lower bound on the physical extent instead;
            # pad it by envelope_margin for residual uncertainty.
            center_forward = 0.5 * (track.max_forward + track.min_forward)
            center_lateral = 0.5 * (track.max_lateral + track.min_lateral)
            half_depth = (
                0.5 * (track.max_forward - track.min_forward)
                + envelope_margin
            )
            half_width = (
                0.5 * (track.max_lateral - track.min_lateral)
                + envelope_margin
            )
        else:
            # No envelope yet (e.g. a hand-built track in a test, or a
            # track this tick hasn't matched a detection for): fall
            # back to the full configured obstacle footprint around the
            # single known point, which cannot under-cover the object.
            center_forward = track.forward
            center_lateral = track.lateral
            half_depth = obstacle_half_depth
            half_width = obstacle_half_width
        offsets = _rectangle_perimeter_offsets(
            half_depth,
            half_width,
            sample_step,
        )
        for delta_forward, delta_lateral in offsets:
            local_x, local_y = entry_frame_to_vehicle_local(
                center_forward + delta_forward,
                center_lateral + delta_lateral,
                pose_x,
                pose_y,
                pose_yaw,
                entry_x,
                entry_y,
                reference_x,
                reference_y,
                path_yaw,
            )
            distance = math.hypot(local_x, local_y)
            if distance > max_memory_range:
                continue
            bearing = math.atan2(local_y, local_x)
            if abs(bearing) <= fov_half_angle:
                # Currently inside the sensor cone: trust the live scan
                # for this part of the box rather than stale memory.
                continue
            points.append((local_x, local_y))
    return points




class PatternSlalomTarget:
    """Continuous quintic target for the two official three-box layouts.

    The first confirmed LiDAR track fixes the whole layout. The object exposes
    lateral position, heading and curvature of one C2 reference so the planner
    can couple linear and angular speed instead of chasing a new VFH sector on
    every scan. Rectangular footprint and asymmetric-corridor rollout remain
    the command authority.
    """

    UPPER_LOWER_UPPER = "UPPER_LOWER_UPPER"
    LOWER_UPPER_LOWER = "LOWER_UPPER_LOWER"

    def __init__(
        self,
        classification_lateral: float,
        upper_pass_lateral: float,
        lower_pass_lateral: float,
        obstacle_spacing: float,
        rejoin_distance: float,
        lookahead: float,
        confirm_scans: int,
        front_face_to_center: float,
        road_left_center_limit: float,
        road_right_center_limit: float,
    ) -> None:
        values = (
            classification_lateral,
            upper_pass_lateral,
            lower_pass_lateral,
            obstacle_spacing,
            rejoin_distance,
            lookahead,
            front_face_to_center,
            road_left_center_limit,
            road_right_center_limit,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pattern slalom configuration must be finite")
        if (
            obstacle_spacing <= 0.0
            or rejoin_distance <= 0.0
            or lookahead <= 0.0
            or confirm_scans < 1
            or front_face_to_center < 0.0
            or road_left_center_limit <= 0.0
            or road_right_center_limit <= 0.0
            or not (
                -road_right_center_limit
                <= upper_pass_lateral
                <= road_left_center_limit
            )
            or not (
                -road_right_center_limit
                <= lower_pass_lateral
                <= road_left_center_limit
            )
        ):
            raise ValueError("invalid pattern slalom configuration")
        self.classification_lateral = classification_lateral
        self.upper_pass_lateral = upper_pass_lateral
        self.lower_pass_lateral = lower_pass_lateral
        self.obstacle_spacing = obstacle_spacing
        self.rejoin_distance = rejoin_distance
        self.lookahead = lookahead
        self.confirm_scans = int(confirm_scans)
        self.front_face_to_center = front_face_to_center
        self.reset()

    def reset(self) -> None:
        self.pattern: Optional[str] = None
        self.knots: List[Tuple[float, float]] = []
        self.first_obstacle_forward: Optional[float] = None
        self.first_obstacle_lateral: Optional[float] = None
        self.pass_lateral: Optional[float] = None
        self.selection_blocked = False

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = clamp(value, 0.0, 1.0)
        return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5

    @staticmethod
    def _smoothstep_first(value: float) -> float:
        value = clamp(value, 0.0, 1.0)
        return 30.0 * value**2 - 60.0 * value**3 + 30.0 * value**4

    @staticmethod
    def _smoothstep_second(value: float) -> float:
        value = clamp(value, 0.0, 1.0)
        return 60.0 * value - 180.0 * value**2 + 120.0 * value**3

    def _first_confirmed_track(
        self,
        tracks: Sequence[ObstacleTrack],
        vehicle_forward: float,
    ) -> Optional[ObstacleTrack]:
        candidates = [
            track
            for track in tracks
            if (
                not track.passed
                and track.seen_count >= self.confirm_scans
                and math.isfinite(track.forward)
                and math.isfinite(track.lateral)
                and track.forward > vehicle_forward + 0.20
            )
        ]
        return min(candidates, key=lambda item: item.forward, default=None)

    def _lock_pattern(
        self,
        track: ObstacleTrack,
        vehicle_forward: float,
        lateral: float,
    ) -> bool:
        front_face = (
            track.min_forward
            if track.min_forward is not None
            and math.isfinite(track.min_forward)
            else track.forward
        )
        first_center = front_face + self.front_face_to_center
        if first_center <= vehicle_forward + 0.50:
            self.selection_blocked = True
            return False
        if track.lateral > self.classification_lateral:
            self.pattern = self.UPPER_LOWER_UPPER
            passes = (
                self.upper_pass_lateral,
                self.lower_pass_lateral,
                self.upper_pass_lateral,
            )
        else:
            self.pattern = self.LOWER_UPPER_LOWER
            passes = (
                self.lower_pass_lateral,
                self.upper_pass_lateral,
                self.lower_pass_lateral,
            )
        self.first_obstacle_forward = first_center
        self.first_obstacle_lateral = track.lateral
        self.knots = [
            (vehicle_forward, lateral),
            (first_center, passes[0]),
            (first_center + self.obstacle_spacing, passes[1]),
            (first_center + 2.0 * self.obstacle_spacing, passes[2]),
            (
                first_center
                + 2.0 * self.obstacle_spacing
                + self.rejoin_distance,
                0.0,
            ),
        ]
        return True

    def reference_state(self, forward: float) -> PatternTrajectoryReference:
        """Return the C2 slalom reference at ``forward``.

        Every adjacent segment uses the same zero-slope, zero-curvature
        boundary condition at its pass line.  Position, heading and curvature
        therefore remain continuous when the target changes from one box row
        to the other and again when it rejoins the surveyed waypoint path.
        """
        if not self.knots:
            raise RuntimeError("pattern slalom has not been locked")
        if not math.isfinite(forward):
            raise ValueError("forward must be finite")

        if forward <= self.knots[0][0]:
            lateral = self.knots[0][1]
            return PatternTrajectoryReference(
                forward=forward,
                lateral=lateral,
                lateral_slope=0.0,
                lateral_second_derivative=0.0,
                heading=0.0,
                curvature=0.0,
            )

        for (start_s, start_d), (end_s, end_d) in zip(
            self.knots,
            self.knots[1:],
        ):
            if forward <= end_s:
                span = end_s - start_s
                if span <= 1e-9:
                    lateral = end_d
                    slope = second = 0.0
                else:
                    unit = clamp((forward - start_s) / span, 0.0, 1.0)
                    delta = end_d - start_d
                    lateral = start_d + delta * self._smoothstep(unit)
                    slope = delta * self._smoothstep_first(unit) / span
                    second = (
                        delta * self._smoothstep_second(unit) / (span * span)
                    )
                heading = math.atan(slope)
                curvature = second / (1.0 + slope * slope) ** 1.5
                return PatternTrajectoryReference(
                    forward=forward,
                    lateral=lateral,
                    lateral_slope=slope,
                    lateral_second_derivative=second,
                    heading=heading,
                    curvature=curvature,
                )

        lateral = self.knots[-1][1]
        return PatternTrajectoryReference(
            forward=forward,
            lateral=lateral,
            lateral_slope=0.0,
            lateral_second_derivative=0.0,
            heading=0.0,
            curvature=0.0,
        )

    def reference_lateral(self, forward: float) -> float:
        return self.reference_state(forward).lateral

    def max_abs_curvature(
        self,
        start_forward: float,
        distance: float,
        samples: int,
    ) -> float:
        if not all(math.isfinite(value) for value in (start_forward, distance)):
            raise ValueError("curvature query must be finite")
        if distance < 0.0 or samples < 1:
            raise ValueError("invalid curvature query range")
        count = max(1, int(samples))
        return max(
            abs(
                self.reference_state(
                    start_forward + distance * index / count
                ).curvature
            )
            for index in range(count + 1)
        )

    def direction(
        self,
        tracks: Sequence[ObstacleTrack],
        lateral: float,
        path_heading_error: float,
        vehicle_forward: float,
        global_target: float,
    ) -> Optional[float]:
        self.selection_blocked = False
        if not all(
            math.isfinite(value)
            for value in (
                lateral,
                path_heading_error,
                vehicle_forward,
                global_target,
            )
        ):
            self.selection_blocked = True
            return None
        if self.pattern is None:
            track = self._first_confirmed_track(tracks, vehicle_forward)
            if track is None:
                return global_target
            if not self._lock_pattern(track, vehicle_forward, lateral):
                return None
        reference = self.reference_lateral(
            vehicle_forward + self.lookahead
        )
        self.pass_lateral = reference
        pursuit = math.atan2(reference - lateral, self.lookahead)
        return clamp(
            ang_norm(pursuit - path_heading_error),
            -math.radians(55.0),
            math.radians(55.0),
        )




# 이전 코드/테스트에서 사용한 이름과의 호환성.


@dataclass(frozen=True)
class ScanQuality:
    ok: bool
    reason: str
    usable_beams: int
    total_beams: int


def assess_scan_quality(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    header_stamp_ns: Optional[int] = None,
    now_ns: Optional[int] = None,
    max_age_sec: Optional[float] = None,
    frame_id: str = "",
    expected_frame: str = "",
    min_beams: int = 1,
    min_angular_span: float = 0.0,
    min_usable_ratio: float = 0.0,
    front_half_angle: float = 0.0,
    front_center_angle: float = 0.0,
    min_front_usable_ratio: float = 0.0,
    angle_max: Optional[float] = None,
    max_future_sec: float = 0.05,
) -> ScanQuality:
    """ROS 메시지에 의존하지 않고 LaserScan 메타데이터/beam을 검증한다.

    유한한 정상 거리와 +Inf(no return)는 usable beam이다. NaN, -Inf,
    0 및 센서 범위 밖의 유한값은 usable로 세지 않는다.
    """

    total = len(ranges)

    def bad(
        reason: str,
        usable: int = 0,
    ) -> ScanQuality:
        return ScanQuality(False, reason, usable, total)

    metadata = (angle_min, angle_increment, range_min, range_max)
    if not all(math.isfinite(value) for value in metadata):
        return bad("NONFINITE_METADATA")
    quality_parameters = (
        min_angular_span,
        min_usable_ratio,
        front_half_angle,
        front_center_angle,
        min_front_usable_ratio,
        max_future_sec,
    )
    if not all(math.isfinite(float(value)) for value in quality_parameters):
        return bad("NONFINITE_QUALITY_LIMIT")
    if max_age_sec is not None and not math.isfinite(float(max_age_sec)):
        return bad("NONFINITE_QUALITY_LIMIT")
    if abs(angle_increment) <= 1e-12:
        return bad("ZERO_ANGLE_INCREMENT")
    if range_min < 0.0 or range_max <= max(0.02, range_min):
        return bad("INVALID_RANGE_LIMITS")
    if total < max(1, int(min_beams)):
        return bad("TOO_FEW_BEAMS")
    angular_span = abs(angle_increment) * max(0, total - 1)
    if angular_span + 1e-9 < max(0.0, min_angular_span):
        return bad("INSUFFICIENT_ANGULAR_SPAN")
    if angle_max is not None:
        if not math.isfinite(angle_max):
            return bad("NONFINITE_METADATA")
        expected_angle_max = angle_min + (total - 1) * angle_increment
        tolerance = max(1e-4, 0.25 * abs(angle_increment))
        if abs(angle_max - expected_angle_max) > tolerance:
            return bad("INCONSISTENT_ANGLE_MAX")

    normalized_frame = str(frame_id).strip().lstrip("/")
    normalized_expected = str(expected_frame).strip().lstrip("/")
    if not normalized_frame:
        return bad("EMPTY_FRAME")
    if normalized_expected and normalized_frame != normalized_expected:
        return bad("UNEXPECTED_FRAME")

    if max_age_sec is not None:
        if (
            header_stamp_ns is None
            or now_ns is None
            or header_stamp_ns <= 0
        ):
            return bad("MISSING_STAMP")
        age_ns = now_ns - header_stamp_ns
        if age_ns < -int(max(0.0, max_future_sec) * 1e9):
            return bad("FUTURE_STAMP")
        if age_ns > int(max(0.0, max_age_sec) * 1e9):
            return bad("OLD_STAMP")

    lower = max(0.02, range_min)
    usable = 0
    for distance in ranges:
        if math.isinf(distance) and distance > 0.0:
            usable += 1
        elif math.isfinite(distance) and lower <= distance <= range_max:
            usable += 1
    if usable == 0:
        return bad("NO_USABLE_BEAMS")
    required_ratio = clamp(float(min_usable_ratio), 0.0, 1.0)
    if usable / total + 1e-12 < required_ratio:
        return bad("LOW_USABLE_RATIO", usable)
    if front_half_angle > 0.0:
        front_total = 0
        front_usable = 0
        for index, distance in enumerate(ranges):
            angle = angle_min + index * angle_increment
            relative = math.atan2(
                math.sin(angle - front_center_angle),
                math.cos(angle - front_center_angle),
            )
            if abs(relative) <= front_half_angle + 1e-12:
                front_total += 1
                if (
                    (math.isinf(distance) and distance > 0.0)
                    or (
                        math.isfinite(distance)
                        and lower <= distance <= range_max
                    )
                ):
                    front_usable += 1
        if front_total == 0:
            return bad("FRONT_SECTOR_MISSING", usable)
        required_front_ratio = clamp(
            float(min_front_usable_ratio),
            0.0,
            1.0,
        )
        if (
            front_usable / front_total + 1e-12
            < required_front_ratio
        ):
            return bad("LOW_FRONT_USABLE_RATIO", usable)
    return ScanQuality(True, "OK", usable, total)


def cluster_extent(points: Sequence[Tuple[float, float]]) -> float:
    if not points:
        return 0.0
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def cluster_scan_points(
    points: Sequence[Tuple[float, float]],
    *,
    max_distance: float = 5.5,
    max_abs_angle: float = math.radians(110.0),
    join_distance: float = 0.25,
    min_points: int = 3,
    min_extent: float = 0.0,
) -> List[List[Tuple[float, float]]]:
    """LaserScan 순서의 점들을 물리적 연속성과 최소 크기로 군집화한다."""

    clusters: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = []
    previous: Optional[Tuple[float, float]] = None

    def finish(candidate: Sequence[Tuple[float, float]]) -> None:
        if (
            len(candidate) >= max(1, int(min_points))
            and cluster_extent(candidate) + 1e-9 >= max(0.0, min_extent)
        ):
            clusters.append(list(candidate))

    for point in points:
        if (
            len(point) < 2
            or not math.isfinite(point[0])
            or not math.isfinite(point[1])
        ):
            finish(current)
            current = []
            previous = None
            continue
        distance = math.hypot(point[0], point[1])
        angle = math.atan2(point[1], point[0])
        if distance > max_distance or abs(angle) > max_abs_angle:
            finish(current)
            current = []
            previous = None
            continue
        if previous is None or math.hypot(
            point[0] - previous[0],
            point[1] - previous[1],
        ) <= join_distance:
            current.append(point)
        else:
            finish(current)
            current = [point]
        previous = point
    finish(current)
    return clusters


def scan_to_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    yaw_offset: float = 0.0,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    max_abs_angle: Optional[float] = None,
) -> List[Tuple[float, float]]:
    """LaserScan을 base_link 좌표점으로 바꾼다.

    max_abs_angle을 주면 base_link 기준 그 각도 밖의 점을 여기서 버린다.
    소비하는 쪽(_blocked_sectors / _trajectory_is_safe / _clusters)마다 각자
    각도 필터를 두면 하나만 놓쳐도 증상이 남기 때문에 한 곳에서 자른다.

    이 차량은 라이다가 차체를 본다. 필터가 없으면 다음이 벌어진다
    (2026-08-10 22:07 주행, 회피 구간 실측):
      통과점 2559개 중 |각도|>100도(차체 뒤쪽)가 835개,
      rollout footprint(+-0.70 x +-0.40m) 안에 1110개,
      가장 큰 클러스터가 808점 / 최근접 0.45m 로 차체 바로 위에 생긴다.
    그 결과 어떤 후보 궤적도 즉시 충돌 판정을 받아 RECOVERY[plan_failed]가
    매 틱 발생하고 회피가 한 번도 성립하지 못했다.

    2026-08-09 라이브 스택에서 같은 수정으로 차체 점 348 -> 0개가 되어 회피가
    처음 동작했다. hope/slalom zip 병합 때 두 번 덮여 재발했다.
    """
    metadata = (
        angle_min,
        angle_increment,
        range_min,
        range_max,
        yaw_offset,
        x_offset,
        y_offset,
    )
    if not all(math.isfinite(float(value)) for value in metadata):
        raise ValueError("scan geometry and static transform must be finite")
    if abs(angle_increment) <= 1e-12 or range_min < 0.0 or range_max <= range_min:
        raise ValueError("invalid scan geometry")
    if max_abs_angle is not None:
        if not math.isfinite(max_abs_angle) or max_abs_angle <= 0.0:
            raise ValueError("max_abs_angle must be finite and positive")
    points: List[Tuple[float, float]] = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < max(0.02, range_min) or distance > range_max:
            continue
        angle = angle_min + index * angle_increment + yaw_offset
        px = x_offset + distance * math.cos(angle)
        py = y_offset + distance * math.sin(angle)
        # 각도 판정은 오프셋을 반영한 base_link 좌표에서 한다. 라이다가 차량
        # 중심에서 떨어져 있으면 센서 각도와 base_link 각도가 다르다.
        if max_abs_angle is not None and abs(math.atan2(py, px)) > max_abs_angle:
            continue
        points.append((px, py))
    return points
