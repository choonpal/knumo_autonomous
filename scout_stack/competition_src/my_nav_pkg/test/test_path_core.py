"""path_core 단위시험 + unicycle 폐루프 시험 (ROS 없이 실행).

폐루프 시험은 `waypoint_follower_node._path_control`의 제어식을 그대로 옮긴
`_follow()` 헬퍼로 수행한다. 제어식을 바꾸면 이 헬퍼도 함께 고쳐야 한다.
"""

import math
import unittest

import numpy as np

from my_nav_pkg.path_core import (
    PathBuildError,
    PathTracker,
    ang_norm,
    build_path,
    pure_pursuit,
    validate_waypoints,
)

DS = 0.05

# follower 기본 파라미터와 같은 값
V_NOM = 0.15
W_MAX = 0.25
A_LAT_MAX = 0.05
LD_MIN = 0.6
LD_MAX = 1.6
LD_GAIN = 4.0
ROT_ENTER = math.radians(60.0)
ROT_EXIT = math.radians(20.0)
MIN_DRIVE_SPEED = 0.05
TARGET_TOL = 0.30
DT = 0.1


def _straight(length=10.0, spacing=1.0):
    n = int(length / spacing) + 1
    return [(i * spacing, 0.0) for i in range(n)]


def _arc(radius=2.0, sweep_deg=90.0, count=10):
    pts = []
    for i in range(count):
        th = math.radians(sweep_deg) * i / (count - 1)
        pts.append((radius * math.sin(th), radius * (1.0 - math.cos(th))))
    return pts


class _Result:
    def __init__(self):
        self.done = False
        self.steps = 0
        self.cte = []
        self.w = []
        self.v = []
        self.traj = []
        self.states = []


def _follow(
    waypoints,
    start,
    yaw0,
    max_steps=4000,
    v_nom=V_NOM,
    ld_min=LD_MIN,
):
    """§5의 제어식으로 unicycle을 굴린다."""
    path = build_path(waypoints, ds=DS)
    tracker = PathTracker(path)
    x, y = start
    yaw = yaw0
    rotating = False
    res = _Result()

    last_x, last_y = waypoints[-1]

    for _ in range(max_steps):
        st = tracker.update(x, y)
        res.cte.append(st.cte)
        res.traj.append((x, y))
        res.steps += 1

        if st.finished and math.hypot(last_x - x, last_y - y) < TARGET_TOL:
            res.done = True
            break

        ld = min(ld_max_clamp(v_nom, ld_min), LD_MAX)
        tx, ty = tracker.lookahead(st.s_here, ld)
        alpha, kappa_cmd = pure_pursuit(x, y, yaw, tx, ty)

        if rotating:
            if abs(alpha) < ROT_EXIT:
                rotating = False
        elif abs(alpha) > ROT_ENTER:
            rotating = True

        if rotating:
            res.states.append("ROTATE")
            v = 0.0
            w = max(-W_MAX, min(W_MAX, 2.0 * alpha))
        else:
            res.states.append("DRIVE")
            kap = max(abs(kappa_cmd), tracker.max_kappa_ahead(st.s_here, ld))
            v = v_nom
            if kap > 1e-6:
                v = min(v, math.sqrt(A_LAT_MAX / kap))
            v = max(v, MIN_DRIVE_SPEED)
            w = v * kappa_cmd
            if abs(w) > W_MAX:
                w = math.copysign(W_MAX, w)
                v = abs(w) / max(abs(kappa_cmd), 1e-6)

        res.v.append(v)
        res.w.append(w)

        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = ang_norm(yaw + w * DT)

    return res, path


def ld_max_clamp(v_nom, ld_min):
    return max(ld_min, LD_GAIN * v_nom)


class TestBuildPath(unittest.TestCase):
    def test_spline_passes_through_every_waypoint(self):
        wps = _arc(radius=3.0, sweep_deg=120.0, count=8)
        path = build_path(wps, ds=DS)
        for i, (wx, wy) in enumerate(wps):
            d = np.min(np.hypot(path.xs - wx, path.ys - wy))
            self.assertLess(
                d,
                DS,
                f"waypoint {i} not on the path (d={d:.4f})",
            )
            # wp_s가 가리키는 지점도 그 waypoint여야 한다.
            px, py = path.point_at(path.wp_s[i])
            self.assertLess(math.hypot(px - wx, py - wy), 2.0 * DS)

    def test_arclength_monotonic_and_curvature_finite(self):
        path = build_path(_arc(), ds=DS)
        self.assertTrue(np.all(np.diff(path.s) > 0.0))
        self.assertEqual(path.s[0], 0.0)
        self.assertTrue(np.all(np.isfinite(path.kappa)))

    def test_straight_path_has_near_zero_curvature(self):
        path = build_path(_straight(), ds=DS)
        self.assertLess(float(np.max(np.abs(path.kappa))), 0.05)
        self.assertAlmostEqual(path.length, 10.0, delta=0.05)

    def test_arc_curvature_matches_radius(self):
        radius = 2.0
        path = build_path(_arc(radius=radius, sweep_deg=90.0, count=12), ds=DS)
        mid = path.kappa[len(path) // 4 : 3 * len(path) // 4]
        measured = float(np.mean(np.abs(mid)))
        expected = 1.0 / radius
        self.assertAlmostEqual(measured, expected, delta=0.2 * expected)

    def test_duplicate_waypoints_keep_original_indexing(self):
        wps = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        path = build_path(wps, ds=DS)
        self.assertEqual(len(path.wp_s), len(wps))
        # 중복점은 자신이 겹친 유지점의 호길이를 물려받는다.
        self.assertAlmostEqual(path.wp_s[1], path.wp_s[2], delta=1e-6)
        self.assertTrue(path.wp_s[3] > path.wp_s[2])

    def test_degenerate_inputs_raise(self):
        with self.assertRaises(PathBuildError):
            build_path([(0.0, 0.0)], ds=DS)
        with self.assertRaises(PathBuildError):
            build_path([(0.0, 0.0), (0.001, 0.0)], ds=DS)
        with self.assertRaises(PathBuildError):
            build_path([(0.0, 0.0), (float("nan"), 1.0)], ds=DS)
        with self.assertRaises(PathBuildError):
            build_path(_straight(), ds=0.0)


class TestTracker(unittest.TestCase):
    def test_cte_sign_and_magnitude(self):
        tracker = PathTracker(build_path(_straight(), ds=DS))
        left = tracker.update(3.0, 0.3)
        self.assertAlmostEqual(left.cte, 0.3, delta=0.02)
        self.assertAlmostEqual(left.distance_to_path, 0.3, delta=0.02)
        self.assertAlmostEqual(left.s_here, 3.0, delta=0.05)

        tracker2 = PathTracker(build_path(_straight(), ds=DS))
        right = tracker2.update(3.0, -0.3)
        self.assertAlmostEqual(right.cte, -0.3, delta=0.02)
        self.assertAlmostEqual(right.distance_to_path, 0.3, delta=0.02)

    def test_endpoint_tangent_extension_is_not_considered_on_route(self):
        tracker = PathTracker(build_path(_straight(), ds=DS))
        state = tracker.update(-2.0, 0.0)
        self.assertAlmostEqual(state.cte, 0.0, delta=0.02)
        self.assertAlmostEqual(state.distance_to_path, 2.0, delta=0.02)
        self.assertEqual(state.passed_wp_count, 1)

    def test_cte_sign_is_left_positive_for_any_heading(self):
        # 남향(-y) 경로에서도 "왼쪽 양수"가 유지되어야 한다.
        wps = [(0.0, 0.0), (0.0, -5.0), (0.0, -10.0)]
        tracker = PathTracker(build_path(wps, ds=DS))
        # -y 진행 방향의 왼쪽은 +x 쪽이다 (heading을 +90도 회전).
        st = tracker.update(0.25, -5.0)
        self.assertAlmostEqual(st.cte, 0.25, delta=0.02)
        tracker2 = PathTracker(build_path(wps, ds=DS))
        self.assertAlmostEqual(
            tracker2.update(-0.25, -5.0).cte, -0.25, delta=0.02
        )

    def test_passed_waypoint_count_advances_with_progress(self):
        wps = _straight(length=5.0, spacing=1.0)
        tracker = PathTracker(build_path(wps, ds=DS))
        self.assertEqual(tracker.update(0.0, 0.0).passed_wp_count, 1)
        self.assertEqual(tracker.update(2.5, 0.0).passed_wp_count, 3)
        # 경로에서 0.6m 벗어나 있어도 진행 기준으로 통과 판정된다.
        self.assertEqual(tracker.update(4.2, 0.6).passed_wp_count, 5)

    def test_lookahead_and_end_clamp(self):
        path = build_path(_straight(), ds=DS)
        tracker = PathTracker(path)
        tx, ty = tracker.lookahead(2.0, 0.6)
        self.assertAlmostEqual(tx, 2.6, delta=DS)
        self.assertAlmostEqual(ty, 0.0, delta=0.01)
        # 경로 끝을 넘기면 마지막 점으로 클램프된다.
        tx, ty = tracker.lookahead(9.9, 1.6)
        self.assertAlmostEqual(tx, path.xs[-1], delta=1e-6)

    def test_max_kappa_ahead_sees_upcoming_curve(self):
        wps = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)] + [
            (4.0 + 1.0 * math.sin(math.radians(30 * i)),
             1.0 * (1.0 - math.cos(math.radians(30 * i))))
            for i in range(1, 4)
        ]
        tracker = PathTracker(build_path(wps, ds=DS))
        near_start = tracker.max_kappa_ahead(0.5, 0.6)
        approaching = tracker.max_kappa_ahead(3.6, 1.6)
        self.assertLess(near_start, approaching)

    def test_windowed_search_does_not_jump_on_self_approaching_path(self):
        # U자 경로: 나가는 구간과 돌아오는 구간이 0.5m 간격으로 근접한다.
        out = [(i * 0.5, 0.0) for i in range(13)]
        turn = [(6.0 + 0.25 * math.sin(math.radians(30 * i)),
                 0.25 * (1.0 - math.cos(math.radians(30 * i))))
                for i in range(1, 6)]
        back = [(6.0 - i * 0.5, 0.5) for i in range(13)]
        wps = out + turn + back
        path = build_path(wps, ds=DS)
        tracker = PathTracker(path, back_window_m=2.0, fwd_window_m=5.0)

        prev_s = -1.0
        for i in range(len(path)):
            st = tracker.update(float(path.xs[i]), float(path.ys[i]))
            self.assertGreaterEqual(
                st.s_here,
                prev_s - 2.0,
                "progress retreated further than back_window",
            )
            prev_s = st.s_here
        # 경로 끝까지 진행이 이어져야 한다 (반대 차선으로 점프하지 않음).
        self.assertGreater(prev_s, path.length - 0.5)

    def test_initial_search_cannot_snap_to_nearby_finish(self):
        # Start and finish are physically close but far apart in path order.
        waypoints = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 5.0),
            (0.0, 5.0),
            (0.10, 0.0),
        ]
        path = build_path(waypoints, ds=DS)
        tracker = PathTracker(
            path,
            back_window_m=2.0,
            fwd_window_m=3.0,
        )
        state = tracker.update(0.08, 0.0)
        self.assertLess(state.s_here, 1.0)
        self.assertFalse(state.finished)
        self.assertLessEqual(state.passed_wp_count, 1)

    def test_skip_to_advances_floor_and_never_retreats(self):
        path = build_path(_straight(), ds=DS)
        tracker = PathTracker(path)
        tracker.update(0.0, 0.0)
        tracker.skip_to(4.0)
        st = tracker.update(0.0, 0.0)
        self.assertGreaterEqual(st.s_here, 4.0)
        tracker.skip_to(1.0)  # 후퇴 요청은 무시
        st = tracker.update(0.0, 0.0)
        self.assertGreaterEqual(st.s_here, 4.0)

    def test_skip_to_marks_waypoint_passed(self):
        wps = _straight(length=5.0, spacing=1.0)
        path = build_path(wps, ds=DS)
        tracker = PathTracker(path)
        tracker.update(0.0, 0.0)
        tracker.skip_to(float(path.wp_s[2]))
        st = tracker.update(0.0, 0.0)
        self.assertEqual(st.passed_wp_count, 3)


class TestPurePursuit(unittest.TestCase):
    def test_straight_ahead_gives_zero_curvature(self):
        alpha, kappa = pure_pursuit(0.0, 0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(alpha, 0.0, delta=1e-9)
        self.assertAlmostEqual(kappa, 0.0, delta=1e-9)

    def test_left_target_gives_positive_curvature(self):
        _alpha, kappa = pure_pursuit(0.0, 0.0, 0.0, 1.0, 0.5)
        self.assertGreater(kappa, 0.0)

    def test_geometric_curvature_matches_circle(self):
        # 반지름 r 원 위의 lookahead 점이면 kappa = 1/r 이어야 한다.
        r = 2.0
        ld = 1.0
        th = 2.0 * math.asin(ld / (2.0 * r))
        tx = r * math.sin(th)
        ty = r * (1.0 - math.cos(th))
        _alpha, kappa = pure_pursuit(0.0, 0.0, 0.0, tx, ty)
        self.assertAlmostEqual(kappa, 1.0 / r, delta=1e-6)

    def test_coincident_target_does_not_blow_up(self):
        _alpha, kappa = pure_pursuit(1.0, 1.0, 0.0, 1.0, 1.0)
        self.assertTrue(math.isfinite(kappa))


class TestClosedLoop(unittest.TestCase):
    def test_lateral_offset_converges_on_straight_path(self):
        wps = _straight(length=10.0, spacing=1.0)
        res, path = _follow(wps, start=(0.0, 2.0), yaw0=0.0)
        self.assertTrue(res.done, "did not finish the straight path")
        half = len(res.cte) // 2
        tail = [abs(c) for c in res.cte[half:]]
        self.assertLess(max(tail), 0.05, f"max|cte| in second half={max(tail):.3f}")

    def test_l_curve_completes_without_stopping(self):
        wps = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0),
               (4.0, 2.0), (4.0, 4.0), (4.0, 6.0)]
        res, _path = _follow(wps, start=(0.0, 0.0), yaw0=0.0)
        self.assertTrue(res.done, "did not finish the L curve")
        self.assertLess(max(abs(c) for c in res.cte), 0.15)
        # DRIVE 중 선속이 0으로 내려가 교착되지 않는다.
        drive_v = [
            v for v, s in zip(res.v, res.states) if s == "DRIVE"
        ]
        self.assertGreaterEqual(min(drive_v), MIN_DRIVE_SPEED - 1e-9)
        # 각속도가 연속적이다 (스텝당 변화 제한).
        dw = [abs(b - a) for a, b in zip(res.w, res.w[1:])]
        self.assertLess(max(dw), 0.10, f"max dw={max(dw):.3f}")

    def test_s_curve_completes_without_stopping(self):
        wps = [(0.0, 0.0), (1.5, 0.0), (3.0, 1.2), (4.5, 2.4),
               (6.0, 2.4), (7.5, 1.2), (9.0, 0.0), (10.5, 0.0)]
        res, _path = _follow(wps, start=(0.0, 0.0), yaw0=0.0)
        self.assertTrue(res.done, "did not finish the S curve")
        self.assertLess(max(abs(c) for c in res.cte), 0.15)
        dw = [abs(b - a) for a, b in zip(res.w, res.w[1:])]
        self.assertLess(max(dw), 0.10)

    def test_no_in_place_rotation_while_tracking_curves(self):
        wps = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0),
               (4.0, 2.0), (4.0, 4.0)]
        res, _path = _follow(wps, start=(0.0, 0.0), yaw0=0.0)
        self.assertNotIn(
            "ROTATE",
            res.states,
            "ROTATE was entered while following a curve",
        )

    def test_rotate_recovers_from_reversed_start_heading(self):
        wps = _straight(length=6.0, spacing=1.0)
        res, _path = _follow(wps, start=(0.0, 0.0), yaw0=math.pi)
        self.assertTrue(res.done)
        self.assertEqual(res.states[0], "ROTATE")
        self.assertEqual(res.states[-1], "DRIVE")

    def test_single_bad_waypoint_is_smoothed(self):
        """잘못 찍힌 waypoint 1개가 있어도 완주하고, 이상점을 그대로 따라가지 않는다.

        CTE는 오염된 경로 기준이므로 작게 나온다. 이상점 완화 여부는
        '이상적인 직선 대비 실제 궤적 이탈량'으로 확인한다.
        """
        offset = 0.5
        wps = _straight(length=10.0, spacing=1.0)
        polluted = list(wps)
        polluted[5] = (polluted[5][0], offset)

        res, _path = _follow(polluted, start=(0.0, 0.0), yaw0=0.0)
        self.assertTrue(res.done, "did not finish the polluted path")
        max_dev = max(abs(y) for (_x, y) in res.traj)
        self.assertLess(
            max_dev,
            offset,
            f"trajectory followed the outlier (max deviation={max_dev:.3f}m)",
        )


class TestValidateWaypoints(unittest.TestCase):
    def test_clean_waypoints_have_no_warning(self):
        self.assertEqual(validate_waypoints(_straight()), [])

    def test_spacing_and_turn_outliers_are_reported(self):
        tight = [(0.0, 0.0), (0.01, 0.0), (1.0, 0.0)]
        self.assertTrue(
            any("spacing" in w for w in validate_waypoints(tight))
        )
        far = [(0.0, 0.0), (9.0, 0.0)]
        self.assertTrue(any("spacing" in w for w in validate_waypoints(far)))
        hairpin = [(0.0, 0.0), (1.0, 0.0), (0.1, 0.2), (1.0, 1.0)]
        self.assertTrue(any("turn" in w for w in validate_waypoints(hairpin)))

    def test_short_input_is_reported(self):
        self.assertTrue(validate_waypoints([(0.0, 0.0)]))


if __name__ == "__main__":
    unittest.main()
