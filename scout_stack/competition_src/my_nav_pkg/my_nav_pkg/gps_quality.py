"""Pure helpers for GPS reception freshness and u-blox RTK quality."""

import math
from typing import Optional


RTK_FIXED = "FIXED"
RTK_FLOAT = "FLOAT"
RTK_NONE = "NONE"
RTK_UNKNOWN = "UNKNOWN"

# UBX-NAV-PVT constants.  They are kept here so this module remains usable in
# unit tests without importing ROS-generated ``ublox_msgs`` classes.
FLAGS_GNSS_FIX_OK = 0x01
FLAGS_DIFF_SOLN = 0x02
FLAGS_CARRIER_PHASE_MASK = 0xC0
CARRIER_PHASE_FLOAT = 0x40
CARRIER_PHASE_FIXED = 0x80

FIX_TYPE_2D = 2
FIX_TYPE_3D = 3
FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4
POSITION_FIX_TYPES = frozenset(
    (FIX_TYPE_2D, FIX_TYPE_3D, FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED)
)


def valid_lla(latitude: float, longitude: float, altitude: float) -> bool:
    """Validate a finite WGS84 position before it can seed an ENU origin."""
    try:
        latitude = float(latitude)
        longitude = float(longitude)
        altitude = float(altitude)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and math.isfinite(altitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


def valid_planar_position(x: float, y: float) -> bool:
    """Return whether both planar coordinates are finite."""
    try:
        return math.isfinite(float(x)) and math.isfinite(float(y))
    except (TypeError, ValueError):
        return False


def valid_xy_waypoint(
    x: float,
    y: float,
    yaw: Optional[float] = None,
) -> bool:
    """Validate finite XY waypoint fields before they reach control math."""
    if not valid_planar_position(x, y):
        return False
    if yaw is None:
        return True
    try:
        return math.isfinite(float(yaw))
    except (TypeError, ValueError):
        return False


def horizontal_accuracy_ok(
    h_acc_mm: Optional[int],
    max_h_acc_m: float,
) -> bool:
    """Validate UBX NAV-PVT horizontal accuracy against a metric limit."""
    if h_acc_mm is None:
        return False
    try:
        h_acc_m = float(h_acc_mm) / 1000.0
        max_h_acc_m = float(max_h_acc_m)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(h_acc_m)
        and math.isfinite(max_h_acc_m)
        and h_acc_m >= 0.0
        and max_h_acc_m > 0.0
        and h_acc_m <= max_h_acc_m
    )


def is_reception_fresh(
    now_ns: int,
    received_ns: Optional[int],
    timeout_sec: float,
) -> bool:
    """Return whether a message was received recently enough.

    Non-positive/non-finite timeouts and backwards clock jumps fail closed.
    """
    if received_ns is None:
        return False
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        return False

    age_ns = int(now_ns) - int(received_ns)
    if age_ns < 0:
        return False
    return age_ns <= int(timeout_sec * 1e9)


def classify_rtk_status(
    *,
    flags: Optional[int],
    fix_type: Optional[int],
    now_ns: int,
    received_ns: Optional[int],
    timeout_sec: float,
) -> str:
    """Classify a UBX-NAV-PVT sample as FIXED/FLOAT/NONE/UNKNOWN.

    UNKNOWN means that no current reception is available.  A current sample
    is FIXED or FLOAT only when it represents a valid position fix, reports a
    differential solution, and has the matching carrier-phase state.
    """
    if not is_reception_fresh(now_ns, received_ns, timeout_sec):
        return RTK_UNKNOWN
    if flags is None or fix_type not in POSITION_FIX_TYPES:
        return RTK_NONE

    flags = int(flags)
    if not (flags & FLAGS_GNSS_FIX_OK):
        return RTK_NONE
    if not (flags & FLAGS_DIFF_SOLN):
        return RTK_NONE

    carrier_phase = flags & FLAGS_CARRIER_PHASE_MASK
    if carrier_phase == CARRIER_PHASE_FIXED:
        return RTK_FIXED
    if carrier_phase == CARRIER_PHASE_FLOAT:
        return RTK_FLOAT
    return RTK_NONE


def _bounded_scale(value: float) -> float:
    """Clamp a configured speed scale to a safe finite [0, 1] range."""
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(0.0, min(1.0, value))


def speed_scale_for_rtk(
    status: str,
    *,
    float_scale: float,
    none_scale: float,
) -> float:
    """Return the bounded linear-speed scale for an RTK status."""
    if status == RTK_FIXED:
        return 1.0
    if status == RTK_FLOAT:
        return _bounded_scale(float_scale)
    # Archive behavior deliberately applies the NONE scale to UNKNOWN too.
    return _bounded_scale(none_scale)
