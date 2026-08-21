import math
import unittest

from my_nav_pkg.speed_profile import (
    is_obstacle_waypoint,
    is_stopline_waypoint,
    waypoint_speed_cap,
)


class SpeedProfileTests(unittest.TestCase):
    def test_only_explicit_obstacle_names_enable_lateral_avoidance(self):
        self.assertTrue(is_obstacle_waypoint('name=obstacle_entry'))
        self.assertTrue(is_obstacle_waypoint('장애물 통과'))
        self.assertFalse(is_obstacle_waypoint('crosswalk approach'))
        self.assertFalse(is_obstacle_waypoint('finish'))

    def test_normal_route_uses_requested_route_speed(self):
        self.assertAlmostEqual(
            waypoint_speed_cap('straight_04', 0.35),
            0.35,
        )

    def test_vision_mission_tokens_slow_the_approach(self):
        self.assertAlmostEqual(
            waypoint_speed_cap('crosswalk approach', 0.35),
            0.12,
        )
        self.assertAlmostEqual(
            waypoint_speed_cap('신호등 대기', 0.35),
            0.12,
        )

    def test_obstacle_and_parking_use_separate_caps(self):
        self.assertAlmostEqual(
            waypoint_speed_cap('obstacle entry', 0.35),
            0.12,
        )
        self.assertAlmostEqual(
            waypoint_speed_cap('후진 주차', 0.35),
            0.08,
        )

    def test_mission_profile_cannot_raise_route_speed(self):
        self.assertAlmostEqual(
            waypoint_speed_cap('parking', 0.05),
            0.05,
        )

    def test_only_stopline_names_require_a_timed_stop(self):
        # 리코더의 'w' 키가 붙이는 이름이 실제로 정차를 발동해야 한다.
        self.assertTrue(is_stopline_waypoint('wp007_stopline'))
        self.assertTrue(is_stopline_waypoint('wp007_정지선'))
        # 감속만 하면 되는 vision 웨이포인트는 세우지 않는다.
        self.assertFalse(is_stopline_waypoint('crosswalk approach'))
        self.assertFalse(is_stopline_waypoint('traffic signal'))
        self.assertFalse(is_stopline_waypoint('wp007'))
        self.assertFalse(is_stopline_waypoint(None))

    def test_stopline_waypoint_keeps_the_full_route_speed(self):
        # 정지선은 그 자리에서 완전히 정차하므로 접근까지 늦추지 않는다.
        # 주행 속도를 유지하다 트리거 지점에서 급정지한다.
        self.assertAlmostEqual(
            waypoint_speed_cap('wp007_stopline', 2.50, vision_speed=1.50),
            2.50,
        )
        self.assertAlmostEqual(
            waypoint_speed_cap('wp007_정지선', 2.50, vision_speed=1.50),
            2.50,
        )
        # 횡단보도/신호등은 정차 의무가 없으므로 감속은 그대로 유지한다.
        self.assertAlmostEqual(
            waypoint_speed_cap('crosswalk', 2.50, vision_speed=1.50),
            1.50,
        )

    def test_stopline_and_obstacle_labels_stay_independent(self):
        self.assertFalse(is_obstacle_waypoint('wp007_stopline'))
        self.assertFalse(is_stopline_waypoint('wp007_obstacle'))

    def test_invalid_speed_is_rejected(self):
        for bad in (0.0, -0.1, math.inf, math.nan):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    waypoint_speed_cap('route', bad)


if __name__ == '__main__':
    unittest.main()
