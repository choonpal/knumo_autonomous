"""ROS-independent waypoint speed-profile policy."""

from __future__ import annotations

import math


# 정지선(stopline/정지선)은 여기 들어 있지 않다. 예전에는 vision 감속을
# 같이 받았으나, 정지선은 그 자리에서 완전히 정차하는 웨이포인트라 접근
# 구간까지 늦출 이유가 없다. 코스 기록 단축을 위해 주행 속도를 정지선
# 직전까지 유지하고 거기서 급정지한다(2026-08-07 운영 결정).
_VISION_TOKENS = (
    'crosswalk',
    'traffic',
    'signal',
    '횡단보도',
    '신호등',
)
_OBSTACLE_TOKENS = ('obstacle', 'avoid', '장애물')
_PARKING_TOKENS = ('parking', 'park', '주차')
# 정지선은 _VISION_TOKENS의 부분집합이다. vision 감속은 그대로 받으면서,
# 추가로 "그 자리에 일정 시간 정차"까지 요구하는 웨이포인트만 골라낸다.
# 횡단보도/신호등은 감속만 하고 정차 의무가 없으므로 여기 넣지 않는다.
_STOPLINE_TOKENS = ('stopline', 'stop_line', '정지선')


def is_stopline_waypoint(name: object) -> bool:
    """Return whether the waypoint requires a timed full stop."""
    value = str(name or '').casefold()
    return any(token in value for token in _STOPLINE_TOKENS)


def is_obstacle_waypoint(name: object) -> bool:
    """Return whether the active waypoint explicitly enables lateral VFH."""
    value = str(name or '').casefold()
    return any(token in value for token in _OBSTACLE_TOKENS)


def is_parking_waypoint(name: object) -> bool:
    """Return whether the active waypoint enables the reverse-parking mission.

    This is the single gate that lets the parking controller command motion at
    all, and (through ``/parking_active``) the only condition under which the
    final selector opens its reverse cap. Reverse stays hard-blocked
    everywhere else, so the token match must stay as narrow as the label
    convention allows.
    """
    value = str(name or '').casefold()
    return any(token in value for token in _PARKING_TOKENS)


def waypoint_speed_cap(
    name: object,
    route_speed: float,
    *,
    vision_speed: float = 0.12,
    obstacle_speed: float = 0.12,
    parking_speed: float = 0.08,
) -> float:
    """Return the speed cap for the current named mission waypoint.

    The route speed is always the upper bound. Mission-specific values can
    therefore slow a vehicle for perception/braking, but can never
    accidentally raise the operator-selected route speed.
    """
    values = (
        float(route_speed),
        float(vision_speed),
        float(obstacle_speed),
        float(parking_speed),
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('all waypoint speed caps must be finite and positive')

    value = str(name or '').casefold()
    if any(token in value for token in _PARKING_TOKENS):
        mission_cap = values[3]
    elif is_obstacle_waypoint(value):
        mission_cap = values[2]
    elif any(token in value for token in _VISION_TOKENS):
        mission_cap = values[1]
    else:
        mission_cap = values[0]
    return min(values[0], mission_cap)
