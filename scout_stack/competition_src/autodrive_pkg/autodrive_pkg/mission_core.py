"""ROS-independent regulation-aware mission state machine."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


NAVIGATION = 'NAVIGATION'
CROSSWALK = 'CROSSWALK'
SIGNAL = 'SIGNAL'
OBSTACLE = 'OBSTACLE'
PARKING = 'PARKING'


def signed_along_track_progress(
    start_x: float,
    start_y: float,
    current_x: float,
    current_y: float,
    path_yaw: float,
) -> Optional[float]:
    """Project odometry displacement onto the release path direction."""
    values = (start_x, start_y, current_x, current_y, path_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        return None
    return (
        (float(current_x) - float(start_x)) * math.cos(float(path_yaw))
        + (float(current_y) - float(start_y)) * math.sin(float(path_yaw))
    )


def source_stamp_is_fresh(
    *,
    now_ns: int,
    stamp_ns: Optional[int],
    timeout_sec: float,
) -> bool:
    """Accept only a current, nonzero source timestamp."""
    if stamp_ns is None:
        return False
    try:
        now_ns = int(now_ns)
        stamp_ns = int(stamp_ns)
        timeout_sec = float(timeout_sec)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        stamp_ns <= 0
        or not math.isfinite(timeout_sec)
        or timeout_sec <= 0.0
    ):
        return False
    age_ns = now_ns - stamp_ns
    return 0 <= age_ns <= int(timeout_sec * 1e9)


def source_stamp_advances(
    previous_ns: Optional[int],
    stamp_ns: int,
) -> bool:
    """Reject a cached or reordered sample from a stamped source."""
    try:
        stamp_ns = int(stamp_ns)
        previous = None if previous_ns is None else int(previous_ns)
    except (TypeError, ValueError, OverflowError):
        return False
    return stamp_ns > 0 and (previous is None or stamp_ns > previous)


def normalized_frame_id(frame_id: object) -> str:
    """Normalize the spelling of a ROS frame without inventing a frame."""
    return str(frame_id or '').strip().lstrip('/')


def same_reference_frame(first: object, second: object) -> bool:
    """Return true only for two explicit, matching ROS frame identifiers."""
    first_frame = normalized_frame_id(first)
    second_frame = normalized_frame_id(second)
    return bool(first_frame) and first_frame == second_frame


@dataclass
class SignalCommitProgress:
    """Track signed signal-release progress in one explicit coordinate frame."""

    release_x: Optional[float] = None
    release_y: Optional[float] = None
    release_yaw: Optional[float] = None
    frame_id: str = ''

    @property
    def ready(self) -> bool:
        return (
            self.release_x is not None
            and self.release_y is not None
            and self.release_yaw is not None
            and bool(self.frame_id)
        )

    def clear(self) -> None:
        self.release_x = None
        self.release_y = None
        self.release_yaw = None
        self.frame_id = ''

    def capture(
        self,
        *,
        x: float,
        y: float,
        path_yaw: float,
        pose_frame: object,
        path_frame: object,
    ) -> bool:
        """Capture a release only when pose and path use the same frame."""
        try:
            values = tuple(float(value) for value in (x, y, path_yaw))
        except (TypeError, ValueError, OverflowError):
            self.clear()
            return False
        if (
            not all(math.isfinite(value) for value in values)
            or not same_reference_frame(pose_frame, path_frame)
        ):
            self.clear()
            return False
        self.release_x, self.release_y, self.release_yaw = values
        self.frame_id = normalized_frame_id(pose_frame)
        return True

    def progress(
        self,
        *,
        current_x: float,
        current_y: float,
        current_frame: object,
    ) -> Optional[float]:
        """Return progress only while the current pose stays in that frame."""
        if not self.ready or not same_reference_frame(
            self.frame_id,
            current_frame,
        ):
            return None
        assert self.release_x is not None
        assert self.release_y is not None
        assert self.release_yaw is not None
        return signed_along_track_progress(
            self.release_x,
            self.release_y,
            current_x,
            current_y,
            self.release_yaw,
        )


def mission_zone_from_waypoint(info: str) -> str:
    """Map a named waypoint contract to a mission zone."""
    value = str(info).casefold()
    tokens = {
        CROSSWALK: ('crosswalk', 'stopline', '횡단보도', '정지선'),
        SIGNAL: ('traffic', 'signal', '신호등'),
        OBSTACLE: ('obstacle', 'avoid', '장애물'),
        PARKING: ('parking', 'park', '주차'),
    }
    for zone, candidates in tokens.items():
        if any(candidate in value for candidate in candidates):
            return zone
    return NAVIGATION


@dataclass(frozen=True)
class MissionDecision:
    stop: bool
    state: str


class MissionSafety:
    """Latch braking/dwell decisions independently of detector flicker."""

    def __init__(
        self,
        *,
        green_confirm_sec: float = 0.5,
        observation_gap_timeout_sec: float = 0.35,
    ) -> None:
        timing_values = (
            green_confirm_sec,
            observation_gap_timeout_sec,
        )
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in timing_values
        ):
            raise ValueError('mission timing must be positive')
        self.green_confirm_sec = float(green_confirm_sec)
        self.observation_gap_timeout_sec = float(
            observation_gap_timeout_sec
        )

        self.signal_latched = False
        self.signal_released = False
        self.signal_done = False
        self.green_since: Optional[float] = None
        self._last_zone = NAVIGATION
        self._last_step_sec: Optional[float] = None

    def _reset_unverified_timing(self) -> None:
        """Discard elapsed time that was not continuously observed."""
        self.green_since = None
        if self.signal_released:
            self.signal_released = False
            self.signal_latched = True

    def _reset_after_exit(self, zone: str) -> None:
        if (
            zone != SIGNAL
            and not self.signal_latched
            and not self.signal_released
        ):
            self.signal_done = False
            self.green_since = None

    def step(
        self,
        *,
        now_sec: float,
        zone: str,
        odom_fresh: bool,
        traffic_state: str,
        traffic_fresh: bool,
        signal_committed: bool = False,
        signal_tracking_fresh: bool = True,
    ) -> MissionDecision:
        try:
            now_sec = float(now_sec)
        except (TypeError, ValueError, OverflowError):
            self._reset_unverified_timing()
            self._last_step_sec = None
            return MissionDecision(True, 'MISSION_INVALID_TIME')
        if not math.isfinite(now_sec):
            self._reset_unverified_timing()
            self._last_step_sec = None
            return MissionDecision(True, 'MISSION_INVALID_TIME')
        if (
            self._last_step_sec is not None
            and (
                now_sec < self._last_step_sec
                or now_sec - self._last_step_sec
                > self.observation_gap_timeout_sec
            )
        ):
            self._reset_unverified_timing()
        self._last_step_sec = now_sec
        zone = str(zone)
        self._reset_after_exit(zone)

        if zone == SIGNAL or self.signal_latched or self.signal_released:
            if self.signal_done and not self.signal_latched:
                self._last_zone = zone
                return MissionDecision(False, 'SIGNAL_GO')
            if self.signal_released:
                if signal_committed:
                    self.signal_released = False
                    self.signal_done = True
                    self._last_zone = zone
                    return MissionDecision(
                        False,
                        'SIGNAL_COMMITTED',
                    )
                if not odom_fresh or not signal_tracking_fresh:
                    self.signal_released = False
                    self.signal_latched = True
                    self.green_since = None
                    self._last_zone = zone
                    return MissionDecision(
                        True,
                        'SIGNAL_WAIT_COMMIT_POSE',
                    )
                state = str(traffic_state).strip().lower()
                if not traffic_fresh or state not in {
                    'green',
                    'blue',
                    'left',
                    'green_left',
                    'blue_left',
                }:
                    self.signal_released = False
                    self.signal_latched = True
                    self.green_since = None
                    self._last_zone = zone
                    return MissionDecision(
                        True,
                        (
                            'SIGNAL_WAIT_VISION'
                            if not traffic_fresh
                            else f'SIGNAL_STOP_{state.upper() or "UNKNOWN"}'
                        ),
                    )
                self._last_zone = zone
                return MissionDecision(
                    False,
                    'SIGNAL_RELEASE_PENDING_COMMIT',
                )
            state = str(traffic_state).strip().lower()
            if not traffic_fresh:
                self.signal_latched = True
                self.green_since = None
                self._last_zone = zone
                return MissionDecision(True, 'SIGNAL_WAIT_VISION')
            # The regulation calls the go indication "blue"; camera models
            # and road terminology may call the same indication green.
            if state not in {
                'green',
                'blue',
                'left',
                'green_left',
                'blue_left',
            }:
                self.signal_latched = True
                self.green_since = None
                self._last_zone = zone
                return MissionDecision(
                    True,
                    f'SIGNAL_STOP_{state.upper() or "UNKNOWN"}',
                )
            if self.green_since is None:
                self.green_since = now_sec
            if (
                now_sec - self.green_since + 1e-9
                < self.green_confirm_sec
            ):
                self._last_zone = zone
                return MissionDecision(True, 'SIGNAL_CONFIRM_GREEN')
            if not odom_fresh or not signal_tracking_fresh:
                self.signal_latched = True
                self._last_zone = zone
                return MissionDecision(
                    True,
                    'SIGNAL_WAIT_COMMIT_POSE',
                )

            self.signal_latched = False
            self.signal_released = True
            self._last_zone = zone
            return MissionDecision(
                False,
                'SIGNAL_RELEASE_PENDING_COMMIT',
            )

        self._last_zone = zone
        return MissionDecision(False, zone)
