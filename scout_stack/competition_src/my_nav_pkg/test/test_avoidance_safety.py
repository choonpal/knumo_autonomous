import math
import unittest

from my_nav_pkg.vfh_core import (
    ConsecutiveScanGate,
    ObstaclePassTracker,
    ObstacleTrack,
    VFHConfig,
    assess_scan_quality,
    cluster_scan_points,
    obstacle_clearance_from_geometry,
    scan_to_points,
)


NANOSECONDS = 1_000_000_000


def seconds(value):
    return int(value * NANOSECONDS)






def quality_kwargs():
    return {
        "angle_min": -math.pi,
        "angle_increment": math.radians(1.0),
        "range_min": 0.05,
        "range_max": 12.0,
        "header_stamp_ns": seconds(10.0),
        "now_ns": seconds(10.0),
        "max_age_sec": 0.30,
        "frame_id": "laser",
        "expected_frame": "laser",
        "min_beams": 300,
        "min_angular_span": math.radians(300.0),
        "min_usable_ratio": 0.50,
        "front_half_angle": math.radians(30.0),
        "min_front_usable_ratio": 0.80,
        "max_future_sec": 0.05,
    }


class GeometrySafetyTest(unittest.TestCase):
    def test_clearance_uses_full_finite_dimensions(self):
        clearance = obstacle_clearance_from_geometry(
            vehicle_length=1.20,
            vehicle_width=0.60,
            obstacle_depth=0.50,
            obstacle_width=0.90,
            safety_gap=0.10,
        )
        self.assertTrue(math.isfinite(clearance.lateral))
        self.assertTrue(math.isfinite(clearance.longitudinal))
        self.assertAlmostEqual(clearance.lateral, 0.85)
        self.assertAlmostEqual(clearance.longitudinal, 0.95)

    def test_front_face_clearance_includes_hidden_obstacle_depth(self):
        clearance = obstacle_clearance_from_geometry(
            vehicle_length=1.20,
            vehicle_width=0.60,
            obstacle_depth=0.50,
            obstacle_width=0.90,
            safety_gap=0.10,
        )
        self.assertTrue(
            hasattr(clearance, "front_face_longitudinal"),
            "LiDAR가 본 앞면을 박스 중심으로 오인하지 않도록 "
            "front_face_longitudinal clearance가 필요하다",
        )
        self.assertAlmostEqual(
            clearance.front_face_longitudinal,
            0.5 * 1.20 + 0.50 + 0.10,
        )

    def test_nonfinite_or_invalid_geometry_is_rejected(self):
        valid = [1.20, 0.60, 0.50, 0.90, 0.10]
        for index in range(len(valid)):
            with self.subTest(nonfinite_index=index):
                values = list(valid)
                values[index] = math.nan
                with self.assertRaises(ValueError):
                    obstacle_clearance_from_geometry(*values)

        for index in range(4):
            with self.subTest(nonpositive_dimension_index=index):
                values = list(valid)
                values[index] = 0.0
                with self.assertRaises(ValueError):
                    obstacle_clearance_from_geometry(*values)

        with self.assertRaises(ValueError):
            obstacle_clearance_from_geometry(
                1.20,
                0.60,
                0.50,
                0.90,
                -0.01,
            )

        with self.assertRaises(ValueError):
            VFHConfig(w_max=math.nan)
        with self.assertRaises(ValueError):
            VFHConfig(vehicle_width=3.0, usable_road_half=1.0)
        with self.assertRaises(ValueError):
            VFHConfig(usable_road_left=2.025)
        with self.assertRaises(ValueError):
            VFHConfig(
                usable_road_left=2.025,
                usable_road_right=0.39,
                vehicle_width=0.60,
                boundary_margin=0.10,
            )


class ConsecutiveScanGateTest(unittest.TestCase):
    def test_gate_requires_distinct_consecutive_positive_updates(self):
        gate = ConsecutiveScanGate(required_scans=3)
        self.assertFalse(gate.update(True))
        self.assertFalse(gate.update(True))
        self.assertFalse(gate.update(False))
        self.assertFalse(gate.update(True))
        self.assertFalse(gate.update(True))
        self.assertTrue(gate.update(True))
        self.assertTrue(gate.update(True))

        gate.reset()
        self.assertEqual(gate.count, 0)
        self.assertFalse(gate.update(True))

    def test_gate_never_allows_zero_required_scans(self):
        gate = ConsecutiveScanGate(required_scans=0)
        self.assertTrue(gate.update(True))
        self.assertEqual(gate.required_scans, 1)


class ObstaclePassTrackerSafetyTest(unittest.TestCase):
    def test_unconfirmed_and_confirmed_tracks_have_separate_ttls(self):
        tracker = ObstaclePassTracker(
            merge_distance=0.75,
            pass_margin=0.95,
            confirm_scans=3,
            unconfirmed_ttl_sec=0.60,
            confirmed_ttl_sec=2.00,
            pass_recent_sec=0.50,
        )
        tracker.update([(2.0, 0.0)], 0.0, seconds(1.0))
        tracker.update([], 0.0, seconds(1.61))
        self.assertEqual(tracker.tracks, [])

        for now in (3.0, 3.1, 3.2):
            tracker.update([(4.0, 0.0)], 0.0, seconds(now))
        tracker.update([], 0.0, seconds(4.0))
        self.assertEqual(len(tracker.tracks), 1)
        tracker.update([], 0.0, seconds(5.21))
        self.assertEqual(tracker.tracks, [])

    def test_pass_requires_a_recent_confirmed_observation(self):
        tracker = ObstaclePassTracker(
            merge_distance=0.75,
            pass_margin=0.95,
            confirm_scans=3,
            confirmed_ttl_sec=5.0,
            pass_recent_sec=0.25,
        )
        for now in (1.0, 1.1, 1.2):
            tracker.update([(2.0, 0.0)], 0.0, seconds(now))

        newly_passed = tracker.update([], 3.0, seconds(1.46))
        self.assertEqual(newly_passed, [])
        self.assertFalse(tracker.tracks[0].passed)

    def test_recent_confirmed_track_can_be_marked_passed_once(self):
        tracker = ObstaclePassTracker(
            merge_distance=0.75,
            pass_margin=0.95,
            confirm_scans=3,
            pass_recent_sec=0.50,
        )
        for now in (1.0, 1.1, 1.2):
            tracker.update([(2.0, 0.0)], 0.0, seconds(now))

        self.assertEqual(
            tracker.update([], 3.0, seconds(1.40)),
            [1],
        )
        self.assertTrue(tracker.tracks[0].passed)
        self.assertEqual(
            tracker.update([], 3.5, seconds(1.50)),
            [],
        )
        self.assertTrue(tracker.tracks[0].passed)

    def test_clock_regression_invalidates_tracks_instead_of_marking_passed(self):
        tracker = ObstaclePassTracker(
            merge_distance=0.75,
            pass_margin=0.95,
            confirm_scans=3,
            confirmed_ttl_sec=5.0,
            pass_recent_sec=1.0,
        )
        for now in (10.0, 10.1, 10.2):
            tracker.update([(2.0, 0.0)], 0.0, seconds(now))

        newly_passed = tracker.update([], 3.0, seconds(9.0))
        self.assertEqual(newly_passed, [])
        self.assertEqual(
            tracker.tracks,
            [],
            "ROS clock가 뒤로 이동하면 미래 timestamp track을 재사용하면 안 된다",
        )

    def test_same_scan_duplicate_faces_coalesce_without_fake_confirmation(self):
        tracker = ObstaclePassTracker(
            merge_distance=0.75,
            pass_margin=0.95,
            confirm_scans=3,
        )
        tracker.update(
            [(2.00, 0.00), (2.04, 0.03), (1.97, -0.02)],
            vehicle_forward=0.0,
            now_ns=seconds(1.0),
        )
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(tracker.tracks[0].seen_count, 1)

        tracker.update(
            [(2.02, 0.01), (1.99, -0.01)],
            vehicle_forward=0.0,
            now_ns=seconds(1.1),
        )
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(tracker.tracks[0].seen_count, 2)
        self.assertFalse(tracker.tracks[0].passed)




class ScanQualitySafetyTest(unittest.TestCase):
    def assess(self, ranges, **overrides):
        kwargs = quality_kwargs()
        kwargs.update(overrides)
        return assess_scan_quality(ranges, **kwargs)

    def test_empty_nan_and_zero_scans_are_not_open_space(self):
        cases = (
            ([], "TOO_FEW_BEAMS"),
            ([math.nan] * 360, "NO_USABLE_BEAMS"),
            ([0.0] * 360, "NO_USABLE_BEAMS"),
        )
        for ranges, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                quality = self.assess(ranges)
                self.assertFalse(quality.ok)
                self.assertEqual(quality.reason, expected_reason)

    def test_positive_infinity_is_a_healthy_no_return_beam(self):
        quality = self.assess([math.inf] * 360)
        self.assertTrue(quality.ok)
        self.assertEqual(quality.reason, "OK")
        self.assertEqual(quality.usable_beams, 360)

    def test_invalid_front_sector_is_never_treated_as_open(self):
        ranges = [math.inf] * 360
        for index in range(360):
            angle = -math.pi + math.radians(index)
            if abs(angle) <= math.radians(30.0):
                ranges[index] = math.nan
        quality = self.assess(ranges)
        self.assertFalse(quality.ok)
        self.assertEqual(quality.reason, "LOW_FRONT_USABLE_RATIO")

    def test_old_and_future_header_stamps_are_rejected(self):
        old = self.assess(
            [1.0] * 360,
            header_stamp_ns=seconds(9.69),
        )
        self.assertFalse(old.ok)
        self.assertEqual(old.reason, "OLD_STAMP")

        future = self.assess(
            [1.0] * 360,
            header_stamp_ns=seconds(10.06),
        )
        self.assertFalse(future.ok)
        self.assertEqual(future.reason, "FUTURE_STAMP")

    def test_frame_is_required_and_must_match_after_slash_normalization(self):
        empty = self.assess([1.0] * 360, frame_id="")
        self.assertFalse(empty.ok)
        self.assertEqual(empty.reason, "EMPTY_FRAME")

        wrong = self.assess([1.0] * 360, frame_id="laser_wrong")
        self.assertFalse(wrong.ok)
        self.assertEqual(wrong.reason, "UNEXPECTED_FRAME")

        normalized = self.assess([1.0] * 360, frame_id="/laser")
        self.assertTrue(normalized.ok)

    def test_invalid_metadata_is_rejected(self):
        cases = (
            ({"angle_min": math.nan}, "NONFINITE_METADATA"),
            ({"angle_increment": 0.0}, "ZERO_ANGLE_INCREMENT"),
            ({"range_min": -0.01}, "INVALID_RANGE_LIMITS"),
            ({"range_min": 1.0, "range_max": 1.0}, "INVALID_RANGE_LIMITS"),
            (
                {"angle_increment": math.radians(0.5)},
                "INSUFFICIENT_ANGULAR_SPAN",
            ),
        )
        for overrides, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                quality = self.assess([1.0] * 360, **overrides)
                self.assertFalse(quality.ok)
                self.assertEqual(quality.reason, expected_reason)

        inconsistent = self.assess(
            [1.0] * 360,
            angle_max=0.0,
        )
        self.assertFalse(inconsistent.ok)
        self.assertEqual(inconsistent.reason, "INCONSISTENT_ANGLE_MAX")


class ScanGeometryTest(unittest.TestCase):
    def test_cluster_requires_both_minimum_points_and_extent(self):
        too_few = [(1.0, index * 0.03) for index in range(4)]
        too_small = [(1.0, index * 0.01) for index in range(5)]
        valid = [(1.0, index * 0.03) for index in range(5)]

        self.assertEqual(
            cluster_scan_points(
                too_few,
                join_distance=0.05,
                min_points=5,
                min_extent=0.08,
            ),
            [],
        )
        self.assertEqual(
            cluster_scan_points(
                too_small,
                join_distance=0.05,
                min_points=5,
                min_extent=0.08,
            ),
            [],
        )
        clusters = cluster_scan_points(
            valid,
            join_distance=0.05,
            min_points=5,
            min_extent=0.08,
        )
        self.assertEqual(len(clusters), 1)
        self.assertEqual(clusters[0], valid)

    def test_scan_xy_and_yaw_static_transform_is_applied(self):
        points = scan_to_points(
            [1.0],
            angle_min=0.0,
            angle_increment=1.0,
            range_min=0.05,
            range_max=12.0,
            yaw_offset=math.pi / 2.0,
            x_offset=0.20,
            y_offset=-0.10,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.20)
        self.assertAlmostEqual(points[0][1], 0.90)

    def test_nonfinite_static_transform_is_rejected(self):
        for name in ("yaw_offset", "x_offset", "y_offset"):
            with self.subTest(name=name):
                kwargs = {
                    "yaw_offset": 0.0,
                    "x_offset": 0.0,
                    "y_offset": 0.0,
                }
                kwargs[name] = math.nan
                with self.assertRaises(ValueError):
                    scan_to_points(
                        [1.0],
                        angle_min=0.0,
                        angle_increment=1.0,
                        range_min=0.05,
                        range_max=12.0,
                        **kwargs,
                    )


if __name__ == "__main__":
    unittest.main()
