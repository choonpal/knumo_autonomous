"""ROS-independent validation helpers for planar poses."""

import math
from typing import Optional, Sequence


def orientation_covariance_has_valid_yaw(
    covariance: Sequence[float],
    max_yaw_variance_rad2: float,
    zero_covariance_yaw_variance_rad2: Optional[float] = None,
) -> bool:
    """Validate a ROS Imu orientation covariance and its yaw variance.

    ROS ``sensor_msgs/Imu`` uses an all-zero orientation covariance to mean
    "unknown".  The Humble ``phidgets_spatial`` driver has that exact output
    even when a MOT0110 onboard AHRS quaternion is enabled.  Such a message is
    accepted only when the caller supplies a positive, field-characterized yaw
    variance.  A partially populated covariance never uses this exception.
    """
    try:
        values = tuple(float(value) for value in covariance)
        limit = float(max_yaw_variance_rad2)
    except (TypeError, ValueError):
        return False
    if (
        len(values) != 9
        or not all(math.isfinite(value) for value in values)
        or not math.isfinite(limit)
        or limit <= 0.0
    ):
        return False
    if any(values[index] < 0.0 for index in (0, 4, 8)):
        return False
    covariance_is_zero = all(abs(value) <= 1e-15 for value in values)
    if covariance_is_zero:
        try:
            fallback = float(zero_covariance_yaw_variance_rad2)
        except (TypeError, ValueError):
            return False
        return math.isfinite(fallback) and 0.0 < fallback <= limit

    scale = max(1.0, *(abs(value) for value in values))
    symmetry_tolerance = 1e-9 * scale
    for upper, lower in ((1, 3), (2, 6), (5, 7)):
        if abs(values[upper] - values[lower]) > symmetry_tolerance:
            return False
    return 0.0 < values[8] <= limit


def yaw_with_offset(
    yaw: float,
    offset_rad: float,
    yaw_sign: float = 1.0,
) -> Optional[float]:
    """Apply a measured heading transform and return normalized ENU yaw.

    Phidget compass-style heading and ROS ENU yaw may have opposite signs, so
    the field contract explicitly supplies ``yaw_sign`` as exactly +1 or -1
    as well as the constant offset.  The offset is deliberately restricted to
    one normalized revolution.  Invalid sentinels therefore keep the direct
    MOT0110 path fail closed until both parts are physically verified.
    """
    try:
        raw = float(yaw)
        offset = float(offset_rad)
        sign = float(yaw_sign)
    except (TypeError, ValueError):
        return None
    if (
        not math.isfinite(raw)
        or not math.isfinite(offset)
        or not math.isfinite(sign)
        or abs(offset) > math.pi
        or sign not in (-1.0, 1.0)
    ):
        return None
    return (sign * raw + offset + math.pi) % (2.0 * math.pi) - math.pi


def checked_yaw_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[float]:
    """Return yaw from a finite, non-degenerate normalized quaternion."""
    try:
        values = tuple(float(value) for value in (x, y, z, w))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None

    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw if math.isfinite(yaw) else None


def finite_planar_pose(x: float, y: float, yaw: float) -> bool:
    """Return True only for a finite planar pose."""
    try:
        return all(math.isfinite(float(value)) for value in (x, y, yaw))
    except (TypeError, ValueError):
        return False


def planar_position_step_is_plausible(
    *,
    previous_x: float,
    previous_y: float,
    previous_ns: int,
    next_x: float,
    next_y: float,
    next_ns: int,
    maximum_speed_mps: float,
    jump_tolerance_m: float,
    maximum_elapsed_sec: float,
) -> bool:
    """Reject a planar teleport that physical motion cannot explain.

    The elapsed interval is capped because a sensor outage must not grant an
    arbitrarily large jump allowance.  A recovered sensor can still be
    accepted after a long outage when it returns near the last trusted pose.
    """
    try:
        px = float(previous_x)
        py = float(previous_y)
        nx = float(next_x)
        ny = float(next_y)
        speed = float(maximum_speed_mps)
        tolerance = float(jump_tolerance_m)
        elapsed_limit = float(maximum_elapsed_sec)
        elapsed_ns = int(next_ns) - int(previous_ns)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        not all(
            math.isfinite(value)
            for value in (px, py, nx, ny, speed, tolerance, elapsed_limit)
        )
        or speed <= 0.0
        or tolerance < 0.0
        or elapsed_limit <= 0.0
        or elapsed_ns <= 0
    ):
        return False
    elapsed_sec = min(elapsed_ns * 1e-9, elapsed_limit)
    allowed_distance = tolerance + speed * elapsed_sec
    return math.hypot(nx - px, ny - py) <= allowed_distance


def angular_step_is_plausible(
    *,
    previous_yaw: float,
    previous_ns: int,
    next_yaw: float,
    next_ns: int,
    maximum_rate_rad_s: float,
    jump_tolerance_rad: float,
    maximum_elapsed_sec: float,
) -> bool:
    """Reject a wrapped-yaw jump that exceeds a physical rate budget."""
    try:
        previous = float(previous_yaw)
        following = float(next_yaw)
        maximum_rate = float(maximum_rate_rad_s)
        tolerance = float(jump_tolerance_rad)
        elapsed_limit = float(maximum_elapsed_sec)
        elapsed_ns = int(next_ns) - int(previous_ns)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        not all(
            math.isfinite(value)
            for value in (
                previous,
                following,
                maximum_rate,
                tolerance,
                elapsed_limit,
            )
        )
        or maximum_rate <= 0.0
        or tolerance < 0.0
        or elapsed_limit <= 0.0
        or elapsed_ns <= 0
    ):
        return False
    elapsed_sec = min(elapsed_ns * 1e-9, elapsed_limit)
    allowed_angle = tolerance + maximum_rate * elapsed_sec
    wrapped_delta = (
        following - previous + math.pi
    ) % (2.0 * math.pi) - math.pi
    return abs(wrapped_delta) <= allowed_angle
