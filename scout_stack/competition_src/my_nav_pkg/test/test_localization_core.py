import math
import unittest

from my_nav_pkg.localization_core import (
    GPS_WEEK_MS,
    corrected_enu_yaw,
    horizontal_variance,
    itow_delta_ms,
    itow_progresses,
    lla_to_enu,
    normalize_angle,
    planar_quaternion,
    scale_wheel_twist,
    yaw_from_quaternion,
)


class HeadingContractTest(unittest.TestCase):
    def test_normalize_angle_uses_half_open_ros_range(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(
            normalize_angle(math.radians(181.0)),
            math.radians(-179.0),
        )

    def test_measured_heading_sign_and_offset_are_scalar_only(self):
        offset = 1.764222
        raw_yaw = 2.90
        self.assertAlmostEqual(
            corrected_enu_yaw(raw_yaw, -1.0, offset),
            normalize_angle(-raw_yaw + offset),
        )

    def test_heading_correction_wraps_across_pi(self):
        corrected = corrected_enu_yaw(
            raw_yaw=math.radians(-170.0),
            yaw_sign=-1.0,
            yaw_offset_rad=math.radians(30.0),
        )
        self.assertAlmostEqual(corrected, math.radians(-160.0))

    def test_planar_quaternion_round_trip_preserves_wrapped_yaw(self):
        source_yaw = math.radians(225.0)
        quaternion = planar_quaternion(source_yaw)
        recovered = yaw_from_quaternion(*quaternion)
        self.assertIsNotNone(recovered)
        self.assertAlmostEqual(recovered, math.radians(-135.0))

    def test_invalid_heading_contract_fails_closed(self):
        with self.assertRaises(ValueError):
            corrected_enu_yaw(0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            normalize_angle(math.nan)


class NavPvtTimeContractTest(unittest.TestCase):
    def test_first_and_bounded_forward_samples_progress(self):
        self.assertTrue(itow_progresses(None, 123_000))
        self.assertTrue(itow_progresses(123_000, 123_200))
        self.assertEqual(itow_delta_ms(123_000, 123_200), 200)

    def test_duplicate_and_regressed_samples_are_rejected(self):
        self.assertFalse(itow_progresses(123_000, 123_000))
        self.assertFalse(itow_progresses(123_000, 122_800))

    def test_gps_week_rollover_is_forward_progress(self):
        previous = GPS_WEEK_MS - 200
        current = 300
        self.assertEqual(itow_delta_ms(previous, current), 500)
        self.assertTrue(itow_progresses(previous, current))

    def test_discontinuous_or_out_of_range_itow_is_rejected(self):
        self.assertFalse(itow_progresses(100_000, 105_001))
        self.assertFalse(itow_progresses(100_000, -1))
        self.assertFalse(itow_progresses(100_000, GPS_WEEK_MS))


class EnuCoordinateContractTest(unittest.TestCase):
    ORIGIN = (35.888387529, 128.606716704, 66.936)

    def test_route_origin_is_exactly_local_zero(self):
        east, north, up = lla_to_enu(*self.ORIGIN, *self.ORIGIN)
        self.assertAlmostEqual(east, 0.0, places=9)
        self.assertAlmostEqual(north, 0.0, places=9)
        self.assertAlmostEqual(up, 0.0, places=9)

    def test_small_longitude_offset_points_east(self):
        latitude, longitude, altitude = self.ORIGIN
        east, north, up = lla_to_enu(
            latitude,
            longitude + 0.00001,
            altitude,
            *self.ORIGIN,
        )
        self.assertAlmostEqual(east, 0.9029151, places=5)
        self.assertAlmostEqual(north, 0.0, places=5)
        self.assertAlmostEqual(up, 0.0, places=5)

    def test_small_latitude_offset_points_north(self):
        latitude, longitude, altitude = self.ORIGIN
        east, north, up = lla_to_enu(
            latitude + 0.00001,
            longitude,
            altitude,
            *self.ORIGIN,
        )
        self.assertAlmostEqual(east, 0.0, places=5)
        self.assertAlmostEqual(north, 1.1095810, places=5)
        self.assertAlmostEqual(up, 0.0, places=5)


class MeasurementScalingContractTest(unittest.TestCase):
    def test_gnss_covariance_never_drops_below_measured_floor(self):
        self.assertAlmostEqual(
            horizontal_variance(0.010, 0.000225),
            0.000225,
        )
        self.assertAlmostEqual(
            horizontal_variance(0.200, 0.000225),
            0.040,
        )

    def test_wheel_speed_and_variance_use_same_scale(self):
        speed, variance = scale_wheel_twist(
            linear_x_mps=-0.40,
            scale=1.01595,
            variance_x=0.010,
        )
        self.assertAlmostEqual(speed, -0.40638)
        self.assertAlmostEqual(variance, 0.010 * 1.01595**2)

    def test_invalid_measurement_scaling_fails_closed(self):
        with self.assertRaises(ValueError):
            horizontal_variance(0.010, 0.0)
        with self.assertRaises(ValueError):
            scale_wheel_twist(0.1, 0.0, 0.01)


if __name__ == "__main__":
    unittest.main()
