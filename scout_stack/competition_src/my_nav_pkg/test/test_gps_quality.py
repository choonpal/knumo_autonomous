import math
import unittest

from my_nav_pkg.gps_quality import (
    CARRIER_PHASE_FIXED,
    CARRIER_PHASE_FLOAT,
    FLAGS_DIFF_SOLN,
    FLAGS_GNSS_FIX_OK,
    FIX_TYPE_3D,
    RTK_FIXED,
    RTK_FLOAT,
    RTK_NONE,
    RTK_UNKNOWN,
    classify_rtk_status,
    horizontal_accuracy_ok,
    is_reception_fresh,
    speed_scale_for_rtk,
    valid_lla,
    valid_planar_position,
    valid_xy_waypoint,
)


NOW_NS = 10_000_000_000
RECENT_NS = NOW_NS - 250_000_000
VALID_BASE_FLAGS = FLAGS_GNSS_FIX_OK | FLAGS_DIFF_SOLN


class ReceptionFreshnessTests(unittest.TestCase):
    def test_missing_reception_is_not_fresh(self):
        self.assertFalse(is_reception_fresh(NOW_NS, None, 1.0))

    def test_recent_reception_is_fresh(self):
        self.assertTrue(is_reception_fresh(NOW_NS, RECENT_NS, 1.0))

    def test_timeout_boundary_is_fresh(self):
        self.assertTrue(
            is_reception_fresh(NOW_NS, NOW_NS - 1_000_000_000, 1.0)
        )

    def test_stale_reception_is_not_fresh(self):
        self.assertFalse(
            is_reception_fresh(NOW_NS, NOW_NS - 1_000_000_001, 1.0)
        )

    def test_backwards_clock_jump_fails_closed(self):
        self.assertFalse(is_reception_fresh(NOW_NS, NOW_NS + 1, 1.0))

    def test_non_positive_or_non_finite_timeout_fails_closed(self):
        self.assertFalse(is_reception_fresh(NOW_NS, RECENT_NS, 0.0))
        self.assertFalse(is_reception_fresh(NOW_NS, RECENT_NS, math.inf))


class PositionValidationTests(unittest.TestCase):
    def test_valid_wgs84_and_planar_positions(self):
        self.assertTrue(valid_lla(37.0, 127.0, 42.0))
        self.assertTrue(valid_planar_position(1.0, -2.0))
        self.assertTrue(valid_xy_waypoint(1.0, -2.0, 0.5))
        self.assertTrue(valid_xy_waypoint(1.0, -2.0, None))

    def test_invalid_or_nonfinite_positions_are_rejected(self):
        self.assertFalse(valid_lla(91.0, 127.0, 0.0))
        self.assertFalse(valid_lla(37.0, 181.0, 0.0))
        self.assertFalse(valid_lla(math.nan, 127.0, 0.0))
        self.assertFalse(valid_planar_position(math.inf, 0.0))
        self.assertFalse(valid_xy_waypoint(math.nan, 0.0, None))
        self.assertFalse(valid_xy_waypoint(0.0, 0.0, math.inf))

    def test_horizontal_accuracy_limit(self):
        self.assertTrue(horizontal_accuracy_ok(500, 0.5))
        self.assertFalse(horizontal_accuracy_ok(501, 0.5))
        self.assertFalse(horizontal_accuracy_ok(None, 0.5))
        self.assertFalse(horizontal_accuracy_ok(50, math.nan))


class RtkClassificationTests(unittest.TestCase):
    def classify(self, flags, fix_type=FIX_TYPE_3D, received_ns=RECENT_NS):
        return classify_rtk_status(
            flags=flags,
            fix_type=fix_type,
            now_ns=NOW_NS,
            received_ns=received_ns,
            timeout_sec=1.0,
        )

    def test_missing_or_stale_reception_is_unknown(self):
        fixed_flags = VALID_BASE_FLAGS | CARRIER_PHASE_FIXED
        self.assertEqual(
            self.classify(fixed_flags, received_ns=None),
            RTK_UNKNOWN,
        )
        self.assertEqual(
            self.classify(
                fixed_flags,
                received_ns=NOW_NS - 1_000_000_001,
            ),
            RTK_UNKNOWN,
        )

    def test_valid_fixed_solution(self):
        self.assertEqual(
            self.classify(VALID_BASE_FLAGS | CARRIER_PHASE_FIXED),
            RTK_FIXED,
        )

    def test_valid_float_solution(self):
        self.assertEqual(
            self.classify(VALID_BASE_FLAGS | CARRIER_PHASE_FLOAT),
            RTK_FLOAT,
        )

    def test_missing_gnss_fix_ok_is_none(self):
        flags = FLAGS_DIFF_SOLN | CARRIER_PHASE_FIXED
        self.assertEqual(self.classify(flags), RTK_NONE)

    def test_missing_differential_solution_is_none(self):
        flags = FLAGS_GNSS_FIX_OK | CARRIER_PHASE_FIXED
        self.assertEqual(self.classify(flags), RTK_NONE)

    def test_no_carrier_solution_is_none(self):
        self.assertEqual(self.classify(VALID_BASE_FLAGS), RTK_NONE)

    def test_non_position_fix_type_is_none(self):
        flags = VALID_BASE_FLAGS | CARRIER_PHASE_FIXED
        self.assertEqual(self.classify(flags, fix_type=0), RTK_NONE)


class RtkSpeedScaleTests(unittest.TestCase):
    def scale(self, status, float_scale=0.5, none_scale=0.25):
        return speed_scale_for_rtk(
            status,
            float_scale=float_scale,
            none_scale=none_scale,
        )

    def test_archive_status_mapping(self):
        self.assertEqual(self.scale(RTK_FIXED), 1.0)
        self.assertEqual(self.scale(RTK_FLOAT), 0.5)
        self.assertEqual(self.scale(RTK_NONE), 0.25)
        self.assertEqual(self.scale(RTK_UNKNOWN), 0.25)

    def test_scales_are_bounded(self):
        self.assertEqual(self.scale(RTK_FLOAT, float_scale=2.0), 1.0)
        self.assertEqual(self.scale(RTK_FLOAT, float_scale=-0.1), 0.0)
        self.assertEqual(self.scale(RTK_NONE, none_scale=math.nan), 0.0)


if __name__ == "__main__":
    unittest.main()
