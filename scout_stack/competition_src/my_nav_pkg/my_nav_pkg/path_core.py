#!/usr/bin/env python3
"""path_core.py — spline 경로 + pure pursuit + cross-track error 계산 모듈.

이 모듈은 ROS에 의존하지 않는다. `vfh_core.py`, `gps_quality.py`와 같은
검증 패턴을 따르며 단위시험은 ROS 없이 실행된다.

제공 기능
---------
build_path(waypoints, ds)
    waypoint 열을 centripetal Catmull-Rom spline으로 보간해 등간격에 가까운
    샘플열 + 누적 호길이 + 곡률을 담은 `Path`를 만든다.

PathTracker
    현재 위치를 경로에 투영해 진행 호길이, 부호 있는 cross-track error,
    통과한 waypoint 수를 계산한다. lookahead 목표점과 전방 최대 곡률도 제공한다.

pure_pursuit(x, y, yaw, tx, ty)
    lookahead 목표점에 대한 heading 오차 alpha와 명령 곡률 kappa를 만든다.

validate_waypoints(waypoints, ...)
    간격/꺾임각 이상치를 경고 문자열 목록으로 돌려준다. 자동 수정은 하지 않는다.

좌표계는 ENU(x=east, y=north)를 가정하고, 각도는 라디안,
cross-track error는 미터이며 **양수 = 경로 기준 왼쪽 이탈**이다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

__all__ = [
    "PathBuildError",
    "Path",
    "TrackState",
    "PathTracker",
    "build_path",
    "pure_pursuit",
    "validate_waypoints",
    "ang_norm",
]

# 수치 하한. 0 나눗셈을 막기 위한 값이며 물리적 의미는 없다.
_EPS = 1e-9
# lookahead 직선거리 하한 [m]. 목표점이 로봇과 겹칠 때 곡률이 발산하지 않게 한다.
_MIN_LOOKAHEAD_DIST = 0.05


class PathBuildError(ValueError):
    """경로를 만들 수 없을 때 발생한다 (유효 waypoint 2개 미만 등)."""


def ang_norm(a: float) -> float:
    """각도를 (-pi, pi] 범위로 정규화한다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Path:
    """보간된 경로.

    xs, ys  : 샘플 좌표
    s       : 누적 호길이. s[0] == 0.0, 단조 증가
    kappa   : 부호 있는 곡률 [1/m]. 왼쪽으로 휘면 양수
    wp_s    : 원본 waypoint i가 놓인 호길이. len(wp_s) == len(입력 waypoints)
    ds      : 요청 샘플 간격
    """

    xs: np.ndarray
    ys: np.ndarray
    s: np.ndarray
    kappa: np.ndarray
    wp_s: np.ndarray
    ds: float

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def __len__(self) -> int:
        return int(self.xs.size)

    def point_at(self, s_query: float) -> Tuple[float, float]:
        """호길이 s_query에 가장 가까운 샘플 좌표를 반환한다 (양끝 클램프)."""
        idx = self.index_at(s_query)
        return float(self.xs[idx]), float(self.ys[idx])

    def index_at(self, s_query: float) -> int:
        """호길이 s_query 이상인 첫 샘플 index (양끝 클램프)."""
        if not math.isfinite(s_query):
            return 0
        idx = int(np.searchsorted(self.s, s_query, side="left"))
        return max(0, min(idx, len(self) - 1))


@dataclass
class TrackState:
    """PathTracker.update()의 결과."""

    s_here: float
    cte: float
    distance_to_path: float
    passed_wp_count: int
    finished: bool
    nearest_index: int


# ---------------------------------------------------------------------------
# 경로 생성
# ---------------------------------------------------------------------------
def _dedupe(
    waypoints: Sequence[Tuple[float, float]],
    ds: float,
) -> Tuple[List[Tuple[float, float]], List[int]]:
    """연속 중복점(간격 < ds)을 제거하고 원본 index -> 유지 index 사상을 만든다.

    중복으로 제거된 원본 waypoint는 자신이 겹친 유지점의 호길이를 물려받는다.
    따라서 follower가 발행하는 1-based 원본 waypoint index 계약이 유지된다.
    """
    kept: List[Tuple[float, float]] = []
    index_map: List[int] = []
    for x, y in waypoints:
        fx = float(x)
        fy = float(y)
        if not (math.isfinite(fx) and math.isfinite(fy)):
            raise PathBuildError("waypoint contains a non-finite coordinate")
        if kept and math.hypot(fx - kept[-1][0], fy - kept[-1][1]) < ds:
            index_map.append(len(kept) - 1)
            continue
        kept.append((fx, fy))
        index_map.append(len(kept) - 1)
    return kept, index_map


def _catmull_rom_samples(
    control: np.ndarray,
    ds: float,
    alpha: float = 0.5,
) -> Tuple[np.ndarray, List[int]]:
    """centripetal Catmull-Rom으로 control 점열을 샘플링한다.

    control: (n, 2) 배열. 반환은 (samples, control_sample_indices).

    uniform 파라미터화는 간격이 불균일한 waypoint에서 loop/overshoot를
    만들므로 사용하지 않는다 (alpha=0.5, centripetal).

    양 끝은 첫/마지막 chord 방향으로 control 점을 선형 외삽해 클램프한다.
    끝점을 그대로 복제하면 centripetal knot 간격이 0이 되어 계산이
    정의되지 않으므로 외삽을 쓴다. 두 방법 모두 끝 접선이 끝 chord 방향과
    같으므로 의도한 클램프 거동은 동일하다.
    """
    n = int(control.shape[0])
    if n < 2:
        raise PathBuildError("at least 2 distinct waypoints are required")

    first = 2.0 * control[0] - control[1]
    last = 2.0 * control[-1] - control[-2]
    ext = np.vstack([first[None, :], control, last[None, :]])

    samples: List[Tuple[float, float]] = []
    control_indices: List[int] = []

    for i in range(n - 1):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]

        t0 = 0.0
        t1 = t0 + max(float(np.linalg.norm(p1 - p0)), _EPS) ** alpha
        t2 = t1 + max(float(np.linalg.norm(p2 - p1)), _EPS) ** alpha
        t3 = t2 + max(float(np.linalg.norm(p3 - p2)), _EPS) ** alpha

        chord = float(np.linalg.norm(p2 - p1))
        count = max(2, int(math.ceil(chord / ds)))

        control_indices.append(len(samples))
        for t in np.linspace(t1, t2, count, endpoint=False):
            # Barry-Goldman pyramidal formulation
            a1 = ((t1 - t) * p0 + (t - t0) * p1) / (t1 - t0)
            a2 = ((t2 - t) * p1 + (t - t1) * p2) / (t2 - t1)
            a3 = ((t3 - t) * p2 + (t - t2) * p3) / (t3 - t2)
            b1 = ((t2 - t) * a1 + (t - t0) * a2) / (t2 - t0)
            b2 = ((t3 - t) * a2 + (t - t1) * a3) / (t3 - t1)
            c = ((t2 - t) * b1 + (t - t1) * b2) / (t2 - t1)
            samples.append((float(c[0]), float(c[1])))

    # 마지막 control 점은 어떤 세그먼트의 시작도 아니므로 따로 붙인다.
    control_indices.append(len(samples))
    samples.append((float(control[-1][0]), float(control[-1][1])))

    return np.asarray(samples, dtype=float), control_indices


def _curvature(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """부호 있는 곡률. 중앙차분 기반이며 파라미터화에 불변인 형태를 쓴다.

    kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^1.5
    """
    count = xs.size
    if count < 3:
        return np.zeros(count, dtype=float)

    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    denom = np.power(dx * dx + dy * dy, 1.5)
    kappa = np.zeros(count, dtype=float)
    good = denom > _EPS
    kappa[good] = (dx[good] * ddy[good] - dy[good] * ddx[good]) / denom[good]
    kappa[~np.isfinite(kappa)] = 0.0

    # 끝점은 중앙차분이 한쪽만 보므로 이웃값을 복사한다.
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]

    # 3-샘플 이동평균 1회 평활.
    smoothed = kappa.copy()
    smoothed[1:-1] = (kappa[:-2] + kappa[1:-1] + kappa[2:]) / 3.0
    return smoothed


def build_path(
    waypoints: Sequence[Tuple[float, float]],
    ds: float = 0.05,
) -> Path:
    """waypoint (x, y) 열을 보간해 Path를 만든다.

    ds는 샘플 간격 [m]이며 0보다 커야 한다.
    유효 waypoint가 2개 미만이면 PathBuildError를 던진다.
    """
    if not math.isfinite(ds) or ds <= 0.0:
        raise PathBuildError("ds must be a positive finite number")
    if waypoints is None or len(waypoints) < 2:
        raise PathBuildError("at least 2 waypoints are required")

    kept, index_map = _dedupe(waypoints, ds)
    if len(kept) < 2:
        raise PathBuildError(
            "waypoints collapse to a single point after duplicate removal"
        )

    control = np.asarray(kept, dtype=float)
    samples, control_indices = _catmull_rom_samples(control, ds)

    xs = samples[:, 0]
    ys = samples[:, 1]

    seg = np.hypot(np.diff(xs), np.diff(ys))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    # 동일 좌표 샘플이 생기면 s가 평평해져 searchsorted가 흔들린다.
    # 단조 증가를 강제한다.
    for i in range(1, s.size):
        if s[i] <= s[i - 1]:
            s[i] = s[i - 1] + _EPS

    kappa = _curvature(xs, ys)

    kept_wp_s = np.asarray(
        [float(s[idx]) for idx in control_indices],
        dtype=float,
    )
    wp_s = np.asarray(
        [kept_wp_s[k] for k in index_map],
        dtype=float,
    )

    return Path(xs=xs, ys=ys, s=s, kappa=kappa, wp_s=wp_s, ds=float(ds))


# ---------------------------------------------------------------------------
# 진행 추적 + cross-track error
# ---------------------------------------------------------------------------
class PathTracker:
    """경로 진행 상태를 유지하며 CTE와 lookahead 목표점을 계산한다.

    최근접 샘플 탐색을 진행 위치 주변 윈도우로 제한한다. 경로가 교차하거나
    스스로 근접하는 구간(주차 미션의 왕복 등)에서 전 구간 최근접 탐색은
    엉뚱한 구간으로 점프해 진행이 뛰거나 역주행 판정이 나올 수 있다.

    back_window_m 만큼의 후퇴는 허용한다. 회피 중 일시 후진(REV)이 진행
    하한에 걸려 CTE가 잘못 계산되는 것을 막기 위한 여유다.
    """

    def __init__(
        self,
        path: Path,
        back_window_m: float = 2.0,
        fwd_window_m: float = 5.0,
    ) -> None:
        if len(path) < 2:
            raise PathBuildError("path must contain at least 2 samples")
        self.path = path
        self.back_window_m = max(float(back_window_m), 0.0)
        self.fwd_window_m = max(float(fwd_window_m), path.ds)
        self._max_s = 0.0
        self._floor_s = 0.0
        self._last_index: int = 0
        self._initialized = False

    # -- 내부 helper ------------------------------------------------------
    def _search_bounds(self) -> Tuple[int, int]:
        path = self.path
        count = len(path)
        if not self._initialized:
            # Competition courses commonly finish near the start. A global
            # first search can therefore snap a newly booted vehicle straight
            # to the finish branch and mark every checkpoint as passed.
            # Bootstrap only in the same bounded forward window used after
            # initialization; explicit skip_to() remains available for a
            # deliberate recovery/resume operation.
            lo = path.index_at(self._floor_s)
            hi = path.index_at(self._floor_s + self.fwd_window_m)
            return lo, max(lo, hi)
        lo_s = max(self._floor_s, self._max_s - self.back_window_m)
        hi_s = path.s[self._last_index] + self.fwd_window_m
        lo = path.index_at(lo_s)
        hi = path.index_at(hi_s)
        if hi < lo:
            hi = lo
        return lo, hi

    def _project(self, x: float, y: float, nearest: int):
        """Project on adjacent segments.

        ``cte`` is the signed lateral component used for steering diagnostics.
        ``distance_to_path`` is the full Euclidean distance to the clamped
        segment.  They differ beyond a path endpoint or a bounded search-window
        edge, where a point may lie on the tangent extension and have cte=0
        despite being metres away from the actual route.
        """
        path = self.path
        count = len(path)
        best = None
        for j in (nearest - 1, nearest):
            if j < 0 or j >= count - 1:
                continue
            ax = float(path.xs[j])
            ay = float(path.ys[j])
            vx = float(path.xs[j + 1]) - ax
            vy = float(path.ys[j + 1]) - ay
            len2 = vx * vx + vy * vy
            if len2 <= _EPS:
                continue
            t = ((x - ax) * vx + (y - ay) * vy) / len2
            t = max(0.0, min(1.0, t))
            px = ax + t * vx
            py = ay + t * vy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0]:
                best = (d2, j, t, vx, vy, px, py)

        if best is None:
            # 세그먼트를 못 찾는 경우(샘플 1개짜리 경로 등)의 방어 코드.
            idx = max(0, min(nearest, count - 1))
            distance = math.hypot(
                x - float(path.xs[idx]),
                y - float(path.ys[idx]),
            )
            return float(path.s[idx]), 0.0, float(distance)

        d2, j, t, vx, vy, px, py = best
        s_here = float(path.s[j] + t * (path.s[j + 1] - path.s[j]))
        norm = math.hypot(vx, vy)
        if norm <= _EPS:
            return s_here, 0.0, float(math.sqrt(max(d2, 0.0)))
        tx = vx / norm
        ty = vy / norm
        # cross product z 성분: 양수면 로봇이 경로 진행방향 왼쪽에 있다.
        cte = tx * (y - py) - ty * (x - px)
        return (
            s_here,
            float(cte),
            float(math.sqrt(max(d2, 0.0))),
        )

    # -- 공개 API ---------------------------------------------------------
    def update(self, x: float, y: float) -> TrackState:
        """현재 위치로 진행 상태를 갱신한다."""
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("position must be finite")

        path = self.path
        lo, hi = self._search_bounds()
        xs = path.xs[lo : hi + 1]
        ys = path.ys[lo : hi + 1]
        d2 = (xs - x) ** 2 + (ys - y) ** 2
        nearest = lo + int(np.argmin(d2))

        s_here, cte, distance_to_path = self._project(x, y, nearest)
        if s_here < self._floor_s:
            s_here = self._floor_s

        self._last_index = nearest
        self._initialized = True
        if s_here > self._max_s:
            self._max_s = s_here

        passed = int(np.searchsorted(path.wp_s, s_here, side="right"))
        finished = s_here >= (path.length - path.ds)

        return TrackState(
            s_here=float(s_here),
            cte=float(cte),
            distance_to_path=float(distance_to_path),
            passed_wp_count=passed,
            finished=bool(finished),
            nearest_index=int(nearest),
        )

    def nearest_cte(self, x: float, y: float) -> Tuple[float, float]:
        """진단 전용 CTE.

        update()의 진행창(``_max_s`` 기반 [뒤 2 m, 앞 5 m])은 단조 진행 보장을
        위해 로봇보다 앞설 수 있고, 그러면 로봇의 실제 최근접 세그먼트가 창 밖으로
        밀려 cte/거리가 과대평가된다(진단값만 왜곡, 제어엔 무관).

        이 메서드는 진행 상태를 건드리지 않고 **전체 경로에서 최근접 세그먼트**를
        찾아 부호있는 CTE와 실제 거리를 반환한다. 자기교차 경로에서 반대편 가지로
        스냅될 수 있으므로 오직 진단·로깅에만 쓸 것.
        """
        path = self.path
        d2 = (path.xs - x) ** 2 + (path.ys - y) ** 2
        nearest = int(np.argmin(d2))
        _, cte, distance = self._project(x, y, nearest)
        return float(cte), float(distance)

    def lookahead(self, s_here: float, lookahead_m: float) -> Tuple[float, float]:
        """호길이 s_here + lookahead_m 지점 좌표. 경로 끝은 마지막 점으로 클램프."""
        target = float(s_here) + max(float(lookahead_m), 0.0)
        return self.path.point_at(target)

    def max_kappa_ahead(self, s_here: float, dist: float) -> float:
        """[s_here, s_here + dist] 구간의 최대 |kappa|."""
        path = self.path
        i0 = path.index_at(s_here)
        i1 = path.index_at(float(s_here) + max(float(dist), 0.0))
        if i1 <= i0:
            i1 = min(i0 + 1, len(path) - 1)
        window = np.abs(path.kappa[i0 : i1 + 1])
        if window.size == 0:
            return 0.0
        return float(np.max(window))

    def skip_to(self, s_target: float) -> None:
        """진행 하한을 s_target까지 전진시킨다 (후퇴는 무시)."""
        if not math.isfinite(s_target):
            return
        value = min(float(s_target), self.path.length)
        if value > self._floor_s:
            self._floor_s = value
        if value > self._max_s:
            self._max_s = value
        self._last_index = self.path.index_at(self._floor_s)
        self._initialized = True

    @property
    def progress_s(self) -> float:
        return float(self._max_s)


# ---------------------------------------------------------------------------
# Pure pursuit
# ---------------------------------------------------------------------------
def pure_pursuit(
    x: float,
    y: float,
    yaw: float,
    tx: float,
    ty: float,
) -> Tuple[float, float]:
    """lookahead 목표점에 대한 (alpha, kappa)를 반환한다.

    alpha : 목표점 방향과 현재 heading의 차 [rad]
    kappa : 명령 곡률 = 2 sin(alpha) / Ld_eff [1/m]

    Ld_eff는 요청한 호길이가 아니라 목표점까지의 **실제 직선거리**다.
    경로 끝에서 lookahead가 클램프되어도 곡률이 과대평가되지 않는다.
    """
    dx = tx - x
    dy = ty - y
    alpha = ang_norm(math.atan2(dy, dx) - yaw)
    ld_eff = max(math.hypot(dx, dy), _MIN_LOOKAHEAD_DIST)
    kappa = 2.0 * math.sin(alpha) / ld_eff
    return float(alpha), float(kappa)


# ---------------------------------------------------------------------------
# waypoint 사전 검증
# ---------------------------------------------------------------------------
def validate_waypoints(
    waypoints: Sequence[Tuple[float, float]],
    min_spacing: float = 0.1,
    max_spacing: float = 5.0,
    max_turn_deg: float = 100.0,
) -> List[str]:
    """waypoint 이상치를 경고 문자열 목록으로 반환한다 (빈 목록 = 이상 없음).

    자동 수정이나 제거는 하지 않는다. 잘못 찍힌 점인지, 실제 코스 형상인지는
    사람이 판단해야 하기 때문이다.
    """
    warnings: List[str] = []
    if waypoints is None or len(waypoints) < 2:
        warnings.append("waypoint count < 2; a path cannot be built")
        return warnings

    points = [(float(x), float(y)) for x, y in waypoints]
    for i, (x, y) in enumerate(points):
        if not (math.isfinite(x) and math.isfinite(y)):
            warnings.append(f"wp {i + 1}: non-finite coordinate")

    for i in range(len(points) - 1):
        d = math.hypot(
            points[i + 1][0] - points[i][0],
            points[i + 1][1] - points[i][1],
        )
        if d < min_spacing:
            warnings.append(
                f"wp {i + 1}->{i + 2}: spacing {d:.3f}m < {min_spacing:.2f}m"
                " (duplicate or noisy point)"
            )
        elif d > max_spacing:
            warnings.append(
                f"wp {i + 1}->{i + 2}: spacing {d:.2f}m > {max_spacing:.2f}m"
                " (curve may be cut; record a denser path)"
            )

    limit = math.radians(max_turn_deg)
    for i in range(1, len(points) - 1):
        ax = points[i][0] - points[i - 1][0]
        ay = points[i][1] - points[i - 1][1]
        bx = points[i + 1][0] - points[i][0]
        by = points[i + 1][1] - points[i][1]
        if math.hypot(ax, ay) <= _EPS or math.hypot(bx, by) <= _EPS:
            continue
        turn = abs(ang_norm(math.atan2(by, bx) - math.atan2(ay, ax)))
        if turn > limit:
            warnings.append(
                f"wp {i + 1}: turn {math.degrees(turn):.0f}deg >"
                f" {max_turn_deg:.0f}deg (likely a mis-recorded point)"
            )

    return warnings
