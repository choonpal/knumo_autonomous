"""ROS-independent velocity caps and slew-rate limiting."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional


FOLLOW_SOURCE = "WAYPOINT_FOLLOWER"
AVOID_SOURCE = "LOCAL_AVOIDANCE"
# 후진주차 전용 소스. AVOID와 달리 slew 제한을 그대로 받는다 - AVOID가 slew를
# 건너뛰는 건 VFH가 그 (v, w) 쌍을 footprint rollout으로 이미 검증했기 때문인데,
# 주차 명령은 그런 검증을 거치지 않으므로 같은 예외를 줄 근거가 없다.
PARKING_SOURCE = "REVERSE_PARKING"


def values_are_finite(values: Iterable[float]) -> bool:
    """Return True only when every supplied command component is finite."""
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def active_speed_cap_if_fresh(
    *,
    now_ns: int,
    received_ns: Optional[int],
    timeout_sec: float,
    value: Optional[float],
) -> Optional[float]:
    """Return a recent positive cap, otherwise fail closed with ``None``."""
    if received_ns is None or value is None:
        return None
    if (
        not values_are_finite((timeout_sec, value))
    ):
        return None
    timeout_sec = float(timeout_sec)
    value = float(value)
    if timeout_sec <= 0.0 or value <= 0.0:
        return None
    age_ns = int(now_ns) - int(received_ns)
    if not 0 <= age_ns <= int(timeout_sec * 1e9):
        return None
    return value


def forward_command_within_speed_cap(
    linear: float,
    speed_cap: float,
    *,
    tolerance: float = 1e-6,
) -> bool:
    """Check a positive command without clipping its verified curvature."""
    if not values_are_finite((linear, speed_cap, tolerance)):
        return False
    linear = float(linear)
    speed_cap = float(speed_cap)
    tolerance = float(tolerance)
    if speed_cap <= 0.0 or tolerance < 0.0:
        return False
    return linear <= 0.0 or linear <= speed_cap + tolerance


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class GuardLimits:
    """Hard limits and maximum normal-command rates of change."""

    follow_forward_cap: float = 0.15
    avoid_forward_cap: float = 0.12
    reverse_cap: float = 0.10
    # 후진주차 전용 상한. reverse_cap을 0.0으로 잠가 전 구간 후진을 막아둔
    # 상태에서, 주차 미션에 한해서만 후진을 허용하기 위한 별도 게이트다.
    # 기본값 0.0 = fail-closed.
    parking_forward_cap: float = 0.08
    parking_reverse_cap: float = 0.0
    angular_cap: float = 0.80
    linear_slew_rate: float = 0.30
    angular_slew_rate: float = 1.50
    nominal_dt: float = 0.05
    max_slew_dt: float = 0.20

    def __post_init__(self) -> None:
        values = (
            self.follow_forward_cap,
            self.avoid_forward_cap,
            self.reverse_cap,
            self.parking_forward_cap,
            self.parking_reverse_cap,
            self.angular_cap,
            self.linear_slew_rate,
            self.angular_slew_rate,
            self.nominal_dt,
            self.max_slew_dt,
        )
        if not values_are_finite(values):
            raise ValueError("all command guard limits must be finite")
        if self.reverse_cap < 0.0:
            raise ValueError("reverse_cap must be non-negative")
        if self.parking_reverse_cap < 0.0:
            raise ValueError("parking_reverse_cap must be non-negative")
        positive_values = (
            self.follow_forward_cap,
            self.avoid_forward_cap,
            self.parking_forward_cap,
            self.angular_cap,
            self.linear_slew_rate,
            self.angular_slew_rate,
            self.nominal_dt,
            self.max_slew_dt,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError("non-reverse command guard limits must be positive")
        if self.nominal_dt > self.max_slew_dt:
            raise ValueError("nominal_dt must not exceed max_slew_dt")


@dataclass(frozen=True)
class GuardedCommand:
    linear: float
    angular: float
    stopped: bool
    reason: str


class CommandGuard:
    """Apply source caps and slew limits while preserving immediate stops."""

    def __init__(self, limits: GuardLimits) -> None:
        self.limits = limits
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_time: Optional[float] = None
        self._last_source: Optional[str] = None

    def reset(self) -> None:
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_time = None
        self._last_source = None

    def _forward_cap(
        self,
        source: str,
        override: Optional[float] = None,
    ) -> Optional[float]:
        if source == FOLLOW_SOURCE:
            configured = self.limits.follow_forward_cap
        elif source == AVOID_SOURCE:
            configured = self.limits.avoid_forward_cap
        elif source == PARKING_SOURCE:
            configured = self.limits.parking_forward_cap
        else:
            return None
        if override is None:
            return configured
        return min(configured, float(override))

    def _reverse_cap(self, source: str) -> float:
        """소스별 후진 상한.

        일반 주행(FOLLOW/AVOID)은 reverse_cap을 쓰고, 운영 설정에서 이 값을
        0.0으로 잠가 전 구간 후진을 막아둔 상태다(후방 센서 커버리지 없음).
        후진주차는 슬롯 규격이 미리 알려져 있고 진입 전 라이다로 위치를
        확정하는 별도 미션이라, 그 구간에 한해 parking_reverse_cap으로만 연다.
        """
        if source == PARKING_SOURCE:
            return self.limits.parking_reverse_cap
        return self.limits.reverse_cap

    def _hard_cap(
        self,
        source: str,
        linear: float,
        angular: float,
        forward_cap_override: Optional[float] = None,
    ) -> tuple[float, float]:
        forward_cap = self._forward_cap(source, forward_cap_override)
        if forward_cap is None:
            raise ValueError(f"unknown motion source: {source}")
        return (
            _clamp(linear, -self._reverse_cap(source), forward_cap),
            _clamp(
                angular,
                -self.limits.angular_cap,
                self.limits.angular_cap,
            ),
        )

    def _stop(
        self,
        now_sec: float,
        source: Optional[str],
        reason: str,
    ) -> GuardedCommand:
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_source = source
        self._last_time = (
            float(now_sec) if math.isfinite(float(now_sec)) else None
        )
        return GuardedCommand(0.0, 0.0, True, reason)

    def apply(
        self,
        source: str,
        linear: float,
        angular: float,
        now_sec: float,
        *,
        force_stop: bool = False,
        forward_cap_override: Optional[float] = None,
    ) -> GuardedCommand:
        """Return a hard-capped and slew-limited planar command.

        Forced stops, non-finite inputs, and exact zero commands bypass slew
        limiting. A backwards/non-advancing clock also fails safe to zero.
        Large positive time gaps are capped so they cannot bypass slew limits.
        """
        if force_stop:
            return self._stop(now_sec, None, "FORCED_STOP")

        if not values_are_finite((linear, angular, now_sec)):
            return self._stop(now_sec, None, "NONFINITE")
        if (
            forward_cap_override is not None
            and (
                not values_are_finite((forward_cap_override,))
                or float(forward_cap_override) <= 0.0
            )
        ):
            return self._stop(now_sec, None, "DYNAMIC_CAP_INVALID")

        linear = float(linear)
        angular = float(angular)
        now_sec = float(now_sec)

        if linear == 0.0 and angular == 0.0:
            return self._stop(now_sec, source, "EXACT_ZERO")

        if self._forward_cap(source, forward_cap_override) is None:
            return self._stop(now_sec, None, "UNKNOWN_SOURCE")

        if self._last_time is None:
            dt = self.limits.nominal_dt
        else:
            elapsed = now_sec - self._last_time
            if elapsed <= 0.0:
                return self._stop(now_sec, None, "TIME_INVALID")
            dt = min(elapsed, self.limits.max_slew_dt)

        if (
            self._last_source is not None
            and self._last_source != source
        ):
            # Both takeover directions get one deterministic exact-zero cycle.
            # In particular FOLLOW -> AVOID must not jump directly from a
            # straight 0.15 m/s command to a verified high-curvature pair.
            return self._stop(now_sec, source, "SOURCE_SWITCH_STOP")

        if source == AVOID_SOURCE:
            # The local avoider has already footprint-rollout-verified this
            # exact (v, w) pair. Independently slewing or clipping either
            # component changes its curvature and invalidates that proof.
            within_limits = (
                -self._reverse_cap(source) <= linear
                <= self._forward_cap(source, forward_cap_override)
                and abs(angular) <= self.limits.angular_cap
            )
            if not within_limits:
                return self._stop(
                    now_sec,
                    source,
                    "AVOID_LIMIT_VIOLATION",
                )
            self._last_linear = linear
            self._last_angular = angular
            self._last_time = now_sec
            self._last_source = source
            return GuardedCommand(linear, angular, False, "AVOID_VERIFIED")

        target_linear, target_angular = self._hard_cap(
            source,
            linear,
            angular,
            forward_cap_override,
        )

        # A newly selected source's hard bounds take precedence over slew.
        # In particular, a follower output above the avoider cap must never
        # leak through for one cycle during the source transition.
        base_linear, base_angular = self._hard_cap(
            source,
            self._last_linear,
            self._last_angular,
            forward_cap_override,
        )

        linear_delta = self.limits.linear_slew_rate * dt
        angular_delta = self.limits.angular_slew_rate * dt
        output_linear = _clamp(
            target_linear,
            base_linear - linear_delta,
            base_linear + linear_delta,
        )
        output_angular = _clamp(
            target_angular,
            base_angular - angular_delta,
            base_angular + angular_delta,
        )
        output_linear, output_angular = self._hard_cap(
            source,
            output_linear,
            output_angular,
            forward_cap_override,
        )

        self._last_linear = output_linear
        self._last_angular = output_angular
        self._last_time = now_sec
        self._last_source = source
        return GuardedCommand(
            output_linear,
            output_angular,
            False,
            "OK",
        )
