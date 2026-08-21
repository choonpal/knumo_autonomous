import math
import unittest

from my_nav_pkg.pose_utils import (
    angular_step_is_plausible,
    checked_yaw_from_quaternion,
    finite_planar_pose,
    orientation_covariance_has_valid_yaw,
    planar_position_step_is_plausible,
    yaw_with_offset,
)


class PoseUtilsTest(unittest.TestCase):
    def test_scaled_valid_quaternion_is_normalized(self):
        half = 0.25 * math.pi
        yaw = checked_yaw_from_quaternion(
            0.0,
            0.0,
            2.0 * math.sin(half),
            2.0 * math.cos(half),
        )
        self.assertIsNotNone(yaw)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_zero_or_nonfinite_quaternion_is_rejected(self):
        self.assertIsNone(checked_yaw_from_quaternion(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(
            checked_yaw_from_quaternion(0.0, 0.0, math.nan, 1.0)
        )

    def test_planar_pose_requires_all_finite_values(self):
        self.assertTrue(finite_planar_pose(1.0, -2.0, 0.3))
        self.assertFalse(finite_planar_pose(math.inf, 0.0, 0.0))
        self.assertFalse(finite_planar_pose(0.0, 0.0, math.nan))

    def test_yaw_covariance_is_finite_and_bounded(self):
        valid = [0.0] * 9
        valid[8] = 0.25
        self.assertTrue(orientation_covariance_has_valid_yaw(valid, 0.25))

        bad_yaw = list(valid)
        bad_yaw[8] = 0.251
        bad_off_diagonal = list(valid)
        bad_off_diagonal[3] = math.nan
        unavailable = list(valid)
        unavailable[0] = -1.0
        self.assertFalse(
            orientation_covariance_has_valid_yaw(bad_yaw, 0.25)
        )
        self.assertFalse(
            orientation_covariance_has_valid_yaw(
                bad_off_diagonal,
                0.25,
            )
        )
        self.assertFalse(
            orientation_covariance_has_valid_yaw(unavailable, 0.25)
        )

    def test_zero_covariance_requires_characterized_phidget_variance(self):
        unknown = [0.0] * 9
        self.assertFalse(
            orientation_covariance_has_valid_yaw(unknown, 0.25)
        )
        self.assertTrue(
            orientation_covariance_has_valid_yaw(
                unknown,
                0.25,
                0.04,
            )
        )
        for invalid_fallback in (-1.0, 0.0, 0.251, math.nan):
            self.assertFalse(
                orientation_covariance_has_valid_yaw(
                    unknown,
                    0.25,
                    invalid_fallback,
                )
            )

    def test_partial_covariance_cannot_use_phidget_fallback(self):
        partial = [0.0] * 9
        partial[0] = 0.01
        self.assertFalse(
            orientation_covariance_has_valid_yaw(
                partial,
                0.25,
                0.04,
            )
        )
        asymmetric = [0.0] * 9
        asymmetric[1] = 0.01
        asymmetric[8] = 0.04
        self.assertFalse(
            orientation_covariance_has_valid_yaw(
                asymmetric,
                0.25,
            )
        )

    def test_measured_mot0110_yaw_offset_is_normalized(self):
        corrected = yaw_with_offset(
            math.radians(170.0),
            math.radians(30.0),
        )
        self.assertIsNotNone(corrected)
        self.assertAlmostEqual(corrected, math.radians(-160.0))
        self.assertIsNone(yaw_with_offset(0.0, 1000.0))
        self.assertIsNone(yaw_with_offset(math.nan, 0.0))
        # A compass-style North=0, East=+90 heading requires a sign reversal
        # plus +90 degrees to become ROS ENU East=0, North=+90.
        self.assertAlmostEqual(
            yaw_with_offset(
                math.radians(90.0),
                math.radians(90.0),
                -1.0,
            ),
            0.0,
        )
        self.assertIsNone(yaw_with_offset(0.0, 0.0, 0.0))

    def test_plausible_position_step_uses_speed_and_tolerance_budget(self):
        common = {
            'previous_x': 1.0,
            'previous_y': 2.0,
            'previous_ns': 1_000_000_000,
            'next_ns': 1_200_000_000,
            'maximum_speed_mps': 0.60,
            'jump_tolerance_m': 0.15,
            'maximum_elapsed_sec': 1.0,
        }
        self.assertTrue(
            planar_position_step_is_plausible(
                next_x=1.26,
                next_y=2.0,
                **common,
            )
        )
        self.assertFalse(
            planar_position_step_is_plausible(
                next_x=1.28,
                next_y=2.0,
                **common,
            )
        )

    def test_long_outage_does_not_grant_unbounded_jump(self):
        self.assertFalse(
            planar_position_step_is_plausible(
                previous_x=0.0,
                previous_y=0.0,
                previous_ns=1_000_000_000,
                next_x=1.0,
                next_y=0.0,
                next_ns=11_000_000_000,
                maximum_speed_mps=0.60,
                jump_tolerance_m=0.15,
                maximum_elapsed_sec=1.0,
            )
        )
        self.assertTrue(
            planar_position_step_is_plausible(
                previous_x=0.0,
                previous_y=0.0,
                previous_ns=1_000_000_000,
                next_x=0.10,
                next_y=0.0,
                next_ns=11_000_000_000,
                maximum_speed_mps=0.60,
                jump_tolerance_m=0.15,
                maximum_elapsed_sec=1.0,
            )
        )

    def test_nonadvancing_time_or_invalid_limits_rejects_step(self):
        base = {
            'previous_x': 0.0,
            'previous_y': 0.0,
            'previous_ns': 1_000_000_000,
            'next_x': 0.0,
            'next_y': 0.0,
            'maximum_speed_mps': 0.60,
            'jump_tolerance_m': 0.15,
            'maximum_elapsed_sec': 1.0,
        }
        self.assertFalse(
            planar_position_step_is_plausible(
                next_ns=1_000_000_000,
                **base,
            )
        )
        bad = dict(base)
        bad['maximum_speed_mps'] = math.nan
        self.assertFalse(
            planar_position_step_is_plausible(
                next_ns=1_100_000_000,
                **bad,
            )
        )

    def test_wrapped_yaw_step_uses_rate_budget(self):
        common = {
            'previous_yaw': math.radians(179.0),
            'previous_ns': 1_000_000_000,
            'next_ns': 1_100_000_000,
            'maximum_rate_rad_s': 2.0,
            'jump_tolerance_rad': 0.05,
            'maximum_elapsed_sec': 0.60,
        }
        self.assertTrue(
            angular_step_is_plausible(
                next_yaw=math.radians(-179.0),
                **common,
            )
        )
        self.assertFalse(
            angular_step_is_plausible(
                next_yaw=0.0,
                **common,
            )
        )

    def test_long_imu_outage_does_not_allow_orientation_reset(self):
        self.assertFalse(
            angular_step_is_plausible(
                previous_yaw=0.0,
                previous_ns=1_000_000_000,
                next_yaw=math.pi,
                next_ns=11_000_000_000,
                maximum_rate_rad_s=2.0,
                jump_tolerance_rad=0.35,
                maximum_elapsed_sec=0.60,
            )
        )


if __name__ == "__main__":
    unittest.main()
