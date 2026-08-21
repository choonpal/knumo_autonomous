"""Pure helpers for the measured Scout Mini localization contract.

This module deliberately has no ROS imports so the coordinate, heading and
freshness rules can be unit-tested on the review machine.
"""

import math
from typing import Optional, Tuple


GPS_WEEK_MS = 604_800_000
WGS84_A = 6_378_137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def normalize_angle(angle: float) -> float:
    """Wrap a finite angle to [-pi, pi)."""
    angle = float(angle)
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[float]:
    """Return planar yaw from a finite, approximately unit quaternion."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if not 0.95 <= norm <= 1.05:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def corrected_enu_yaw(
    raw_yaw: float,
    yaw_sign: float,
    yaw_offset_rad: float,
) -> float:
    """Apply the measured MOT0110 scalar-heading conversion."""
    raw_yaw = float(raw_yaw)
    yaw_sign = float(yaw_sign)
    yaw_offset_rad = float(yaw_offset_rad)
    if yaw_sign not in (-1.0, 1.0):
        raise ValueError("yaw_sign must be exactly +1 or -1")
    if (
        not math.isfinite(raw_yaw)
        or not math.isfinite(yaw_offset_rad)
        or abs(yaw_offset_rad) > math.pi
    ):
        raise ValueError("yaw and offset must be finite; offset must be in [-pi, pi]")
    return normalize_angle(yaw_sign * raw_yaw + yaw_offset_rad)


def planar_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return an x/y/z/w quaternion containing yaw only."""
    half = 0.5 * normalize_angle(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def itow_delta_ms(
    previous_itow_ms: Optional[int],
    current_itow_ms: int,
) -> Optional[int]:
    """Return forward GPS-time delta, including the weekly rollover."""
    current = int(current_itow_ms)
    if not 0 <= current < GPS_WEEK_MS:
        return None
    if previous_itow_ms is None:
        return 0
    previous = int(previous_itow_ms)
    if not 0 <= previous < GPS_WEEK_MS:
        return None
    return (current - previous) % GPS_WEEK_MS


def itow_progresses(
    previous_itow_ms: Optional[int],
    current_itow_ms: int,
    max_step_ms: int = 5_000,
) -> bool:
    """Accept the first sample or a bounded, strictly advancing NAV-PVT iTOW."""
    try:
        max_step = int(max_step_ms)
    except (TypeError, ValueError):
        return False
    if max_step <= 0 or max_step >= GPS_WEEK_MS:
        return False
    delta = itow_delta_ms(previous_itow_ms, current_itow_ms)
    if delta is None:
        return False
    if previous_itow_ms is None:
        return True
    return 0 < delta <= max_step


def lla_to_ecef(
    latitude_rad: float,
    longitude_rad: float,
    altitude_m: float,
) -> Tuple[float, float, float]:
    """Convert WGS84 geodetic coordinates to ECEF metres."""
    latitude_rad = float(latitude_rad)
    longitude_rad = float(longitude_rad)
    altitude_m = float(altitude_m)
    if not all(
        math.isfinite(value)
        for value in (latitude_rad, longitude_rad, altitude_m)
    ):
        raise ValueError("LLA must be finite")
    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    sin_lon = math.sin(longitude_rad)
    cos_lon = math.cos(longitude_rad)
    prime_vertical = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_lat * sin_lat
    )
    return (
        (prime_vertical + altitude_m) * cos_lat * cos_lon,
        (prime_vertical + altitude_m) * cos_lat * sin_lon,
        (
            prime_vertical * (1.0 - WGS84_E2) + altitude_m
        )
        * sin_lat,
    )


def lla_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> Tuple[float, float, float]:
    """Convert WGS84 LLA to the route's local tangent ENU frame."""
    values = tuple(
        float(value)
        for value in (
            latitude_deg,
            longitude_deg,
            altitude_m,
            origin_latitude_deg,
            origin_longitude_deg,
            origin_altitude_m,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("LLA and origin must be finite")
    lat, lon, alt, lat0, lon0, alt0 = values
    if not -90.0 <= lat <= 90.0 or not -90.0 <= lat0 <= 90.0:
        raise ValueError("latitude is outside WGS84 range")
    if not -180.0 <= lon <= 180.0 or not -180.0 <= lon0 <= 180.0:
        raise ValueError("longitude is outside WGS84 range")

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    x, y, z = lla_to_ecef(lat_rad, lon_rad, alt)
    x0, y0, z0 = lla_to_ecef(lat0_rad, lon0_rad, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    sin_lat0, cos_lat0 = math.sin(lat0_rad), math.cos(lat0_rad)
    sin_lon0, cos_lon0 = math.sin(lon0_rad), math.cos(lon0_rad)
    east = -sin_lon0 * dx + cos_lon0 * dy
    north = (
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz
    )
    up = (
        cos_lat0 * cos_lon0 * dx
        + cos_lat0 * sin_lon0 * dy
        + sin_lat0 * dz
    )
    return east, north, up


def antenna_to_base_xy(
    antenna_east_m: float,
    antenna_north_m: float,
    vehicle_yaw_rad: float,
    antenna_x_m: float,
    antenna_y_m: float,
) -> Tuple[float, float]:
    """Remove a base-frame horizontal GNSS lever arm from antenna ENU."""
    values = tuple(
        float(value)
        for value in (
            antenna_east_m,
            antenna_north_m,
            vehicle_yaw_rad,
            antenna_x_m,
            antenna_y_m,
        )
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("position, yaw and lever arm must be finite")
    east, north, yaw, lever_x, lever_y = values
    delta_east = math.cos(yaw) * lever_x - math.sin(yaw) * lever_y
    delta_north = math.sin(yaw) * lever_x + math.cos(yaw) * lever_y
    return east - delta_east, north - delta_north


def horizontal_variance(
    h_acc_m: float,
    variance_floor_m2: float,
) -> float:
    """Return a finite GNSS variance no smaller than a measured floor."""
    h_acc_m = float(h_acc_m)
    floor = float(variance_floor_m2)
    if (
        not math.isfinite(h_acc_m)
        or h_acc_m < 0.0
        or not math.isfinite(floor)
        or floor <= 0.0
    ):
        raise ValueError("GNSS accuracy and covariance floor are invalid")
    return max(h_acc_m * h_acc_m, floor)


def scale_wheel_twist(
    linear_x_mps: float,
    scale: float,
    variance_x: float,
) -> Tuple[float, float]:
    """Scale wheel longitudinal speed and its variance consistently."""
    linear_x_mps = float(linear_x_mps)
    scale = float(scale)
    variance_x = float(variance_x)
    if (
        not math.isfinite(linear_x_mps)
        or not math.isfinite(scale)
        or scale <= 0.0
        or not math.isfinite(variance_x)
        or variance_x < 0.0
    ):
        raise ValueError("wheel twist scale input is invalid")
    return linear_x_mps * scale, variance_x * scale * scale
