import math
import unittest

from my_nav_pkg.vfh_core import (
    ObstaclePassTracker,
    ObstacleTrack,
    PatternSlalomTarget,
    VFHConfig,
    VFHPlanner,
    ang_norm,
    blind_zone_memory_points,
    cap_target_near_obstacle,
    entry_frame_to_vehicle_local,
    path_yaw_in_pose_frame,
    slew_limited_angle,
)


class VFHCoreTest(unittest.TestCase):





    def test_locked_path_can_disable_vfh_sector_reselection(self):
        planner = VFHPlanner(VFHConfig())
        planner._trajectory_is_safe = lambda *_args, **_kwargs: False

        def unexpected_sector_search(*_args, **_kwargs):
            self.fail("locked path must not search a replacement VFH sector")

        planner._candidate_directions = unexpected_sector_search
        result = planner.plan(
            [],
            target_direction=0.20,
            lateral=0.0,
            path_heading_error=0.0,
            allow_sector_fallback=False,
        )
        self.assertIsNone(result)
        self.assertIsNotNone(planner.last_path_command)
        self.assertFalse(planner.last_plan_failure["sector_fallback"])

    def test_completion_command_commits_locked_path_slew_history(self):
        planner = VFHPlanner(
            VFHConfig(max_target_slew_rad=0.05, v_min=0.08, v_max=0.40)
        )
        planner._trajectory_is_safe = lambda *_args, **_kwargs: False
        result = planner.plan(
            [],
            target_direction=0.40,
            lateral=0.0,
            path_heading_error=0.0,
            allow_sector_fallback=False,
        )
        self.assertIsNone(result)
        continuation = planner.continuation_command()
        self.assertIsNotNone(continuation)
        self.assertAlmostEqual(continuation[2], 0.05)
        self.assertAlmostEqual(planner.previous_direction, 0.05)

        # The next failed scan must advance to the next slew step instead of
        # repeating 0.05 rad forever.
        planner.plan(
            [],
            target_direction=0.40,
            lateral=0.0,
            path_heading_error=0.0,
            allow_sector_fallback=False,
        )
        continuation = planner.continuation_command()
        self.assertIsNotNone(continuation)
        self.assertAlmostEqual(continuation[2], 0.10)

    def test_close_point_does_not_force_zero_in_completion_mode(self):
        planner = VFHPlanner(
            VFHConfig(v_min=0.08, v_max=0.40)
        )
        planner._trajectory_is_safe = lambda *_args, **_kwargs: True
        result = planner.plan(
            [(0.30, 0.0)],
            target_direction=0.0,
            lateral=0.0,
            path_heading_error=0.0,
        )
        self.assertIsNotNone(result)
        self.assertGreater(result[0], 0.0)


    def test_path_yaw_is_unchanged_when_frames_match(self):
        result = path_yaw_in_pose_frame(
            path_yaw_source=0.20,
            target_bearing_source=0.30,
            pose_yaw=0.10,
            target_heading_error=0.20,
        )
        self.assertTrue(math.isclose(result, 0.20, abs_tol=1e-9))

    def test_path_yaw_is_rotated_from_gps_local_to_odom(self):
        source_vehicle_yaw = -0.10
        frame_rotation = math.pi / 2.0
        result = path_yaw_in_pose_frame(
            path_yaw_source=0.0,
            target_bearing_source=0.20,
            pose_yaw=source_vehicle_yaw + frame_rotation,
            target_heading_error=0.30,
        )
        self.assertTrue(
            math.isclose(result, frame_rotation, abs_tol=1e-9)
        )

    def test_reversed_waypoint_segment_uses_forward_direction(self):
        result = path_yaw_in_pose_frame(
            path_yaw_source=math.pi,
            target_bearing_source=0.0,
            pose_yaw=0.0,
            target_heading_error=0.0,
        )
        self.assertTrue(math.isclose(result, 0.0, abs_tol=1e-9))

    def test_vehicle_width_sets_hard_center_limit(self):
        config = VFHConfig(
            usable_road_half=1.35,
            vehicle_width=0.80,
            boundary_margin=0.15,
        )
        self.assertTrue(math.isclose(config.center_limit, 0.80))

    def test_asymmetric_road_limits_are_signed_from_path_centre(self):
        config = VFHConfig(
            usable_road_left=2.025,
            usable_road_right=0.675,
            vehicle_width=0.60,
            boundary_margin=0.10,
        )
        self.assertAlmostEqual(config.road_left_limit, 2.025)
        self.assertAlmostEqual(config.road_right_limit, 0.675)
        self.assertAlmostEqual(config.left_center_limit, 1.625)
        self.assertAlmostEqual(config.right_center_limit, 0.275)
        self.assertAlmostEqual(config.steering_left_center_limit, 1.725)
        self.assertAlmostEqual(config.steering_right_center_limit, 0.375)
        self.assertAlmostEqual(config.center_limit, 0.275)
        self.assertTrue(config.contains_lateral(2.025))
        self.assertTrue(config.contains_lateral(-0.675))
        self.assertFalse(config.contains_lateral(2.026))
        self.assertFalse(config.contains_lateral(-0.676))

    def test_default_width_and_inflation_match_060m_vehicle(self):
        config = VFHConfig()
        self.assertTrue(math.isclose(config.vehicle_width, 0.60))
        self.assertTrue(math.isclose(config.inflation_radius, 0.35))

    def test_open_space_follows_goal_direction(self):
        result = VFHPlanner(VFHConfig()).plan([], 0.0, 0.0, 0.0)
        self.assertIsNotNone(result)
        linear, angular, selected, _front = result
        self.assertGreater(linear, 0.0)
        self.assertTrue(math.isclose(angular, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(selected, 0.0, abs_tol=1e-9))

    def test_dynamic_speed_cap_wins_over_normal_vfh_minimum(self):
        result = VFHPlanner(VFHConfig(v_min=0.08, v_max=0.12)).plan(
            [],
            0.0,
            0.0,
            0.0,
            speed_cap=0.05,
        )
        self.assertIsNotNone(result)
        linear, _angular, _selected, _front = result
        self.assertAlmostEqual(linear, 0.05)

    def test_dynamic_speed_cap_rejects_nonpositive_or_nonfinite_values(self):
        planner = VFHPlanner(VFHConfig())
        for value in (0.0, -0.01, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    planner.plan([], 0.0, 0.0, 0.0, speed_cap=value)

    def test_local_waypoint_direction_is_executed_when_rollout_is_safe(self):
        box_face = [(2.00, index * 0.05) for index in range(-5, 6)]
        target = math.radians(-25.0)
        result = VFHPlanner(VFHConfig()).plan(box_face, target, 0.0, 0.0)
        self.assertIsNotNone(result)
        _linear, _angular, selected, _front = result
        self.assertTrue(math.isclose(selected, target, abs_tol=1e-9))

    def test_open_side_command_keeps_full_footprint_in_corridor(self):
        config = VFHConfig()
        planner = VFHPlanner(config)
        result = planner.plan(
            [],
            target_direction=math.radians(90.0),
            lateral=0.79,
            path_heading_error=0.0,
        )
        self.assertIsNotNone(result)
        linear, angular, selected, _front = result
        self.assertTrue(
            planner.command_is_safe(
                [],
                linear,
                angular,
                lateral=0.79,
                path_heading_error=0.0,
                target_direction=selected,
            )
        )

    def test_command_crossing_lane_is_rejected_by_rectangular_footprint(self):
        planner = VFHPlanner(VFHConfig())
        safe = planner.command_is_safe(
            [],
            linear=0.30,
            angular=0.80,
            lateral=0.78,
            path_heading_error=0.0,
            target_direction=math.radians(60.0),
        )
        self.assertFalse(safe)

    def test_asymmetric_rollout_accepts_exact_edges_and_rejects_overrun(self):
        planner = VFHPlanner(
            VFHConfig(
                usable_road_left=2.025,
                usable_road_right=0.675,
                vehicle_width=0.60,
                boundary_margin=0.10,
            )
        )
        for lateral in (-0.275, 1.625):
            with self.subTest(lateral=lateral):
                self.assertTrue(
                    planner.command_is_safe(
                        [],
                        linear=0.05,
                        angular=0.0,
                        lateral=lateral,
                        path_heading_error=0.0,
                        target_direction=0.0,
                    )
                )
        for lateral in (-0.275001, 1.625001):
            with self.subTest(lateral=lateral):
                self.assertFalse(
                    planner.command_is_safe(
                        [],
                        linear=0.05,
                        angular=0.0,
                        lateral=lateral,
                        path_heading_error=0.0,
                        target_direction=0.0,
                    )
                )

    def test_low_speed_command_does_not_bypass_footprint_collision_check(self):
        planner = VFHPlanner(VFHConfig())
        safe = planner.command_is_safe(
            [(0.64, 0.0)],
            linear=0.05,
            angular=0.0,
            lateral=0.0,
            path_heading_error=0.0,
            target_direction=0.0,
        )
        self.assertFalse(safe)

    def test_low_speed_command_is_allowed_only_in_verified_open_corridor(self):
        planner = VFHPlanner(VFHConfig())
        safe = planner.command_is_safe(
            [],
            linear=0.05,
            angular=0.0,
            lateral=0.0,
            path_heading_error=0.0,
            target_direction=0.0,
        )
        self.assertTrue(safe)

    def test_fully_blocked_scan_has_no_motion_solution(self):
        points = [
            (
                0.50 * math.cos(math.radians(degree)),
                0.50 * math.sin(math.radians(degree)),
            )
            for degree in range(-100, 101, 5)
        ]
        result = VFHPlanner(VFHConfig()).plan(points, 0.0, 0.0, 0.0)
        self.assertIsNone(result)

    def test_noise_track_is_not_marked_passed(self):
        tracker = ObstaclePassTracker(0.75, 0.55, confirm_scans=3)
        tracker.update([(2.0, 0.0)], vehicle_forward=0.0, now_ns=1)
        tracker.update([], vehicle_forward=3.0, now_ns=2)
        self.assertFalse(any(track.passed for track in tracker.tracks))

    def test_three_confirmed_obstacles_are_marked_passed(self):
        tracker = ObstaclePassTracker(0.75, 0.55, confirm_scans=3)
        obstacle_positions = (2.0, 4.5, 7.0)
        now_ns = 0

        for obstacle in obstacle_positions:
            for scan in range(3):
                now_ns += 100_000_000
                tracker.update(
                    [(obstacle + 0.01 * scan, 0.25)],
                    vehicle_forward=obstacle - 1.0,
                    now_ns=now_ns,
                )
            now_ns += 100_000_000
            tracker.update(
                [],
                vehicle_forward=obstacle + 0.60,
                now_ns=now_ns,
            )

        passed = [track for track in tracker.tracks if track.passed]
        self.assertEqual(len(passed), 3)

    def test_passed_obstacle_surface_does_not_create_a_duplicate_track(self):
        tracker = ObstaclePassTracker(1.15, 0.55, confirm_scans=3)
        for scan in range(3):
            tracker.update(
                [(2.0, 0.45)],
                vehicle_forward=0.0,
                now_ns=scan + 1,
            )
        tracker.update([], vehicle_forward=2.60, now_ns=4)
        self.assertTrue(tracker.tracks[0].passed)

        tracker.update(
            [(2.10, 0.50)],
            vehicle_forward=2.70,
            now_ns=5,
        )
        self.assertEqual(len(tracker.tracks), 1)
        self.assertTrue(tracker.tracks[0].passed)

    def test_passed_track_does_not_absorb_next_obstacle_ahead(self):
        tracker = ObstaclePassTracker(1.15, 0.55, confirm_scans=3)
        for scan in range(3):
            tracker.update(
                [(2.0, 0.45)],
                vehicle_forward=0.0,
                now_ns=scan + 1,
            )
        tracker.update([], vehicle_forward=2.60, now_ns=4)
        self.assertTrue(tracker.tracks[0].passed)

        # 다음 박스가 merge_distance 안에 있어도 차량 앞에 있으면 새 track이다.
        tracker.update(
            [(3.0, -0.45)],
            vehicle_forward=2.60,
            now_ns=5,
        )
        self.assertEqual(len(tracker.tracks), 2)
        self.assertFalse(tracker.tracks[-1].passed)








class PatternSlalomTargetTest(unittest.TestCase):
    @staticmethod
    def _target():
        return PatternSlalomTarget(
            classification_lateral=0.625,
            upper_pass_lateral=0.00,
            lower_pass_lateral=1.25,
            obstacle_spacing=3.00,
            rejoin_distance=3.00,
            lookahead=0.40,
            confirm_scans=3,
            front_face_to_center=0.25,
            road_left_center_limit=1.625,
            road_right_center_limit=0.275,
        )

    @staticmethod
    def _track(lateral):
        return ObstacleTrack(
            track_id=1,
            forward=2.75,
            lateral=lateral,
            last_seen_ns=1,
            seen_count=3,
            was_ahead=True,
            min_forward=2.75,
            max_forward=2.75,
            min_lateral=lateral,
            max_lateral=lateral,
        )

    def test_upper_first_locks_all_three_centres_once(self):
        target = self._target()
        track = self._track(1.075)
        self.assertIsNotNone(
            target.direction([track], 0.0, 0.0, 0.0, 0.0)
        )
        self.assertEqual(target.pattern, target.UPPER_LOWER_UPPER)
        self.assertEqual(
            target.knots,
            [
                (0.0, 0.0),
                (3.0, 0.00),
                (6.0, 1.25),
                (9.0, 0.00),
                (12.0, 0.0),
            ],
        )
        frozen = list(target.knots)
        track.forward = 4.0
        track.lateral = 0.175
        target.direction([track], 0.2, 0.0, 1.0, 0.0)
        self.assertEqual(target.knots, frozen)

    def test_lower_first_selects_opposite_pattern_and_reset_unlocks(self):
        target = self._target()
        target.direction([self._track(0.175)], 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(target.pattern, target.LOWER_UPPER_LOWER)
        self.assertEqual(
            [lateral for _forward, lateral in target.knots[1:4]],
            [1.25, 0.00, 1.25],
        )
        for forward, expected in target.knots:
            self.assertAlmostEqual(
                target.reference_lateral(forward),
                expected,
            )
        target.reset()
        self.assertIsNone(target.pattern)
        self.assertEqual(target.knots, [])


    def test_reference_is_c2_at_every_pass_line(self):
        target = self._target()
        target.direction([self._track(1.075)], 0.0, 0.0, 0.0, 0.0)
        for forward, lateral in target.knots:
            with self.subTest(forward=forward):
                reference = target.reference_state(forward)
                self.assertAlmostEqual(reference.lateral, lateral, places=9)
                self.assertAlmostEqual(reference.lateral_slope, 0.0, places=9)
                self.assertAlmostEqual(
                    reference.lateral_second_derivative,
                    0.0,
                    places=9,
                )
                self.assertAlmostEqual(reference.heading, 0.0, places=9)
                self.assertAlmostEqual(reference.curvature, 0.0, places=9)

    def test_reference_curvature_is_finite_and_below_070_speed_limit(self):
        target = self._target()
        target.direction([self._track(1.075)], 0.0, 0.0, 0.0, 0.0)
        peak = target.max_abs_curvature(0.0, 12.0, 480)
        self.assertGreater(peak, 0.0)
        # At 0.70 m/s this is comfortably below the configured 1.00 rad/s
        # pattern angular limit, before feedback correction.
        self.assertLess(0.70 * peak, 1.00)



class BlindZoneMemoryTest(unittest.TestCase):
    """A rear-blocked/narrow-FOV LiDAR must not let an obstacle it can no
    longer see be treated as open space by the safety rollout."""

    def _vehicle_point_to_entry(self, x_vehicle, y_vehicle, pose, entry_pose, path_reference):
        # Mirrors LocalAvoider._vehicle_point_to_entry exactly, so the
        # inverse under test is checked against the real forward
        # transform rather than against itself.
        x, y, yaw = pose
        world_x = x + math.cos(yaw) * x_vehicle - math.sin(yaw) * y_vehicle
        world_y = y + math.sin(yaw) * x_vehicle + math.cos(yaw) * y_vehicle
        entry_x, entry_y, _entry_yaw = entry_pose
        reference_x, reference_y, path_yaw = path_reference
        entry_dx = world_x - entry_x
        entry_dy = world_y - entry_y
        path_dx = world_x - reference_x
        path_dy = world_y - reference_y
        return (
            math.cos(path_yaw) * entry_dx + math.sin(path_yaw) * entry_dy,
            -math.sin(path_yaw) * path_dx + math.cos(path_yaw) * path_dy,
        )

    def test_entry_frame_inverse_matches_forward_projection(self):
        pose = (1.7, -0.4, math.radians(35.0))
        entry_pose = (0.2, -0.1, 0.0)
        path_reference = (0.5, 0.3, math.radians(12.0))
        for x_vehicle, y_vehicle in [
            (0.0, 0.0), (3.0, 0.0), (-2.0, 0.6), (1.0, -0.9), (5.5, 1.2),
        ]:
            forward, lateral = self._vehicle_point_to_entry(
                x_vehicle, y_vehicle, pose, entry_pose, path_reference
            )
            recovered_x, recovered_y = entry_frame_to_vehicle_local(
                forward, lateral, *pose,
                entry_pose[0], entry_pose[1],
                path_reference[0], path_reference[1], path_reference[2],
            )
            self.assertAlmostEqual(recovered_x, x_vehicle, places=9)
            self.assertAlmostEqual(recovered_y, y_vehicle, places=9)

    def test_entry_frame_inverse_rejects_nonfinite_input(self):
        with self.assertRaises(ValueError):
            entry_frame_to_vehicle_local(
                float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            )

    def test_obstacle_behind_the_fov_yields_phantom_points_behind_vehicle(self):
        # Vehicle at the origin facing +x; a still-tracked box sits 2 m
        # behind it (already passed laterally, not yet marked passed).
        track = ObstacleTrack(
            track_id=1, forward=-2.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=True, passed=False,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.radians(100.0),
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertTrue(points)
        for x, _y in points:
            self.assertLess(x, 0.0)
            bearing = math.atan2(_y, x)
            self.assertGreater(abs(bearing), math.radians(100.0))

    def test_obstacle_still_in_fov_is_left_to_the_live_scan(self):
        track = ObstacleTrack(
            track_id=1, forward=3.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=False, passed=False,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.radians(100.0),
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertEqual(points, [])

    def test_passed_tracks_are_never_remembered(self):
        track = ObstacleTrack(
            track_id=1, forward=-2.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=True, passed=True,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.radians(100.0),
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertEqual(points, [])

    def test_far_beyond_memory_range_is_dropped(self):
        track = ObstacleTrack(
            track_id=1, forward=-50.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=True, passed=False,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.radians(100.0),
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertEqual(points, [])

    def test_full_circle_fov_never_needs_memory(self):
        track = ObstacleTrack(
            track_id=1, forward=-2.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=True, passed=False,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.pi,
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertEqual(points, [])

    def test_vehicle_yaw_rotates_remembered_bearing_correctly(self):
        # Same physical obstacle (2 m behind in world terms, at world
        # (-2, 0)), but the vehicle has yawed +90 deg (turned left, now
        # facing world +y). What used to be directly behind is now to
        # the vehicle's left (local +y): the phantom points must follow
        # the vehicle's own heading, not stay pinned to world axes.
        track = ObstacleTrack(
            track_id=1, forward=-2.0, lateral=0.0,
            last_seen_ns=0, seen_count=5, was_ahead=True, passed=False,
        )
        points = blind_zone_memory_points(
            [track], 0.0, 0.0, math.radians(90.0), 0.0, 0.0, 0.0, 0.0, 0.0,
            fov_half_angle=math.radians(100.0),
            obstacle_half_depth=0.25,
            obstacle_half_width=0.45,
            max_memory_range=6.0,
        )
        self.assertTrue(points)
        centroid_x, centroid_y = entry_frame_to_vehicle_local(
            track.forward, track.lateral, 0.0, 0.0, math.radians(90.0),
            0.0, 0.0, 0.0, 0.0, 0.0,
        )
        self.assertAlmostEqual(centroid_x, 0.0, places=9)
        self.assertAlmostEqual(centroid_y, 2.0, places=9)
        for _x, y in points:
            self.assertGreater(y, 0.0)


class SlewLimitedAngleTest(unittest.TestCase):
    def test_zero_max_step_disables_limiting(self):
        self.assertAlmostEqual(
            slew_limited_angle(0.0, math.radians(49.0), 0.0),
            math.radians(49.0),
        )

    def test_large_jump_is_capped_to_max_step(self):
        result = slew_limited_angle(0.0, math.radians(49.0), math.radians(6.0))
        self.assertAlmostEqual(result, math.radians(6.0))

    def test_small_change_within_budget_passes_through(self):
        result = slew_limited_angle(
            math.radians(10.0), math.radians(12.0), math.radians(6.0)
        )
        self.assertAlmostEqual(result, math.radians(12.0))

    def test_negative_direction_is_capped_symmetrically(self):
        result = slew_limited_angle(0.0, math.radians(-49.0), math.radians(6.0))
        self.assertAlmostEqual(result, math.radians(-6.0))

    def test_wraparound_across_pi_takes_the_short_way(self):
        # From 179 deg toward -179 deg is a 2 deg step the short way
        # around, not a ~358 deg step the long way.
        previous = math.radians(179.0)
        desired = math.radians(-179.0)
        result = slew_limited_angle(previous, desired, math.radians(10.0))
        self.assertAlmostEqual(ang_norm(result - previous), math.radians(2.0), places=6)

    def test_rejects_nonfinite_input(self):
        with self.assertRaises(ValueError):
            slew_limited_angle(0.0, float("nan"), 0.1)


class CapTargetNearObstacleTest(unittest.TestCase):
    def test_disabled_when_radius_is_zero(self):
        result = cap_target_near_obstacle(
            math.radians(80.0), 0.0, [(0.3, 0.0)], 0.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(80.0))

    def test_unclamped_when_nothing_is_nearby(self):
        result = cap_target_near_obstacle(
            math.radians(80.0), 0.0, [(5.0, 0.0)], 2.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(80.0))

    def test_unclamped_with_no_points_at_all(self):
        result = cap_target_near_obstacle(
            math.radians(80.0), 0.0, [], 2.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(80.0))

    def test_clamped_when_a_point_is_within_radius(self):
        result = cap_target_near_obstacle(
            math.radians(80.0), 0.0, [(0.4, 0.3)], 2.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(35.0))

    def test_small_target_within_cap_passes_through_unchanged(self):
        result = cap_target_near_obstacle(
            math.radians(10.0), 0.0, [(0.4, 0.3)], 2.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(10.0))

    def test_negative_target_is_capped_symmetrically(self):
        result = cap_target_near_obstacle(
            math.radians(-80.0), 0.0, [(0.4, 0.3)], 2.0, math.radians(35.0)
        )
        self.assertAlmostEqual(result, math.radians(-35.0))

    def test_uses_the_nearest_point_not_the_first(self):
        result = cap_target_near_obstacle(
            math.radians(80.0),
            0.0,
            [(5.0, 0.0), (0.2, 0.1)],
            2.0,
            math.radians(35.0),
        )
        self.assertAlmostEqual(result, math.radians(35.0))

    def test_bounds_the_resulting_absolute_heading_not_just_the_step(self):
        # Vehicle already sitting 30 deg off path heading (accumulated
        # from earlier ticks) asks for another 20 deg this tick. Capping
        # the raw 20 deg step would let it through unchanged (20 < 35),
        # but the *resulting* heading (30+20=50) must still be capped to
        # 35, so the returned target is only enough to reach 35 total.
        result = cap_target_near_obstacle(
            math.radians(20.0),
            math.radians(30.0),
            [(0.4, 0.3)],
            2.0,
            math.radians(35.0),
        )
        self.assertAlmostEqual(result, math.radians(5.0))

    def test_already_past_the_cap_pulls_back_toward_it(self):
        # If accumulated heading error already exceeds the cap (e.g. the
        # vehicle got there before anything was nearby), a further
        # positive request is turned into a negative (pull-back) one
        # rather than merely zeroed, since the resulting heading is
        # clamped to exactly max_angle.
        result = cap_target_near_obstacle(
            math.radians(10.0),
            math.radians(50.0),
            [(0.4, 0.3)],
            2.0,
            math.radians(35.0),
        )
        self.assertAlmostEqual(result, math.radians(-15.0))

    def test_rejects_negative_max_angle(self):
        with self.assertRaises(ValueError):
            cap_target_near_obstacle(0.0, 0.0, [(0.1, 0.1)], 2.0, -0.1)

    def test_rejects_nonfinite_input(self):
        with self.assertRaises(ValueError):
            cap_target_near_obstacle(
                float("nan"), 0.0, [(0.1, 0.1)], 2.0, 0.5
            )

    def test_vfhconfig_rejects_a_radius_with_no_angle_cap(self):
        with self.assertRaises(ValueError):
            VFHConfig(near_obstacle_radius=2.0, max_target_angle_near_obstacle_rad=0.0)

    def test_vfhconfig_accepts_radius_with_angle_cap(self):
        config = VFHConfig(
            near_obstacle_radius=2.0,
            max_target_angle_near_obstacle_rad=math.radians(35.0),
        )
        self.assertAlmostEqual(config.near_obstacle_radius, 2.0)


if __name__ == "__main__":
    unittest.main()
