import unittest
import math

from autodrive_pkg.mission_core import (
    CROSSWALK,
    NAVIGATION,
    SIGNAL,
    MissionSafety,
    SignalCommitProgress,
    mission_zone_from_waypoint,
    same_reference_frame,
    signed_along_track_progress,
    source_stamp_advances,
    source_stamp_is_fresh,
)


class MissionZoneTests(unittest.TestCase):
    def test_named_waypoints_select_mission_zone(self):
        self.assertEqual(
            mission_zone_from_waypoint('idx=10, name=crosswalk_approach'),
            CROSSWALK,
        )
        self.assertEqual(
            mission_zone_from_waypoint('idx=20, name=traffic_signal'),
            SIGNAL,
        )
        self.assertEqual(
            mission_zone_from_waypoint('idx=1, name=start'),
            NAVIGATION,
        )

    def test_signal_commit_progress_is_signed_along_track(self):
        self.assertAlmostEqual(
            signed_along_track_progress(0.0, 0.0, 1.0, 2.0, 0.0),
            1.0,
        )
        self.assertAlmostEqual(
            signed_along_track_progress(
                0.0,
                0.0,
                1.0,
                2.0,
                math.pi / 2.0,
            ),
            2.0,
        )
        self.assertLess(
            signed_along_track_progress(0.0, 0.0, -0.5, 0.0, 0.0),
            0.0,
        )

    def test_signal_commit_progress_rejects_mixed_frames(self):
        tracker = SignalCommitProgress()
        self.assertFalse(
            tracker.capture(
                x=0.0,
                y=0.0,
                path_yaw=0.0,
                pose_frame='odom',
                path_frame='gps_local',
            )
        )
        self.assertFalse(tracker.ready)

        self.assertTrue(
            tracker.capture(
                x=1.0,
                y=2.0,
                path_yaw=0.0,
                pose_frame='/gps_local',
                path_frame='gps_local',
            )
        )
        self.assertAlmostEqual(
            tracker.progress(
                current_x=1.6,
                current_y=3.0,
                current_frame='gps_local',
            ),
            0.6,
        )
        self.assertIsNone(
            tracker.progress(
                current_x=2.0,
                current_y=2.0,
                current_frame='odom',
            )
        )
        self.assertTrue(same_reference_frame('/gps_local', 'gps_local'))
        self.assertFalse(same_reference_frame('', 'gps_local'))

    def test_source_stamp_must_be_current_nonzero_and_not_future(self):
        now_ns = 10_000_000_000
        self.assertTrue(
            source_stamp_is_fresh(
                now_ns=now_ns,
                stamp_ns=9_800_000_000,
                timeout_sec=0.35,
            )
        )
        self.assertFalse(
            source_stamp_is_fresh(
                now_ns=now_ns,
                stamp_ns=0,
                timeout_sec=0.35,
            )
        )
        self.assertTrue(source_stamp_advances(None, 100))
        self.assertTrue(source_stamp_advances(100, 101))
        self.assertFalse(source_stamp_advances(100, 100))
        self.assertFalse(source_stamp_advances(100, 99))
        self.assertFalse(
            source_stamp_is_fresh(
                now_ns=now_ns,
                stamp_ns=9_600_000_000,
                timeout_sec=0.35,
            )
        )
        self.assertFalse(
            source_stamp_is_fresh(
                now_ns=now_ns,
                stamp_ns=10_000_000_001,
                timeout_sec=0.35,
            )
        )


class MissionSafetyTests(unittest.TestCase):
    def setUp(self):
        self.core = MissionSafety(
            green_confirm_sec=0.5,
            # Most tests intentionally jump between state-transition
            # boundaries instead of simulating every 10 Hz observation.
            observation_gap_timeout_sec=10.0,
        )

    def step(self, now, **overrides):
        args = {
            'now_sec': now,
            'zone': NAVIGATION,
            'odom_fresh': True,
            'traffic_state': 'unknown',
            'traffic_fresh': True,
        }
        args.update(overrides)
        return self.core.step(**args)

    def test_crosswalk_zone_is_a_no_op_for_mission_control(self):
        # Timed stopline dwell now lives entirely in
        # waypoint_follower_node (GPS distance + hold timer); mission
        # control no longer imposes a stop for CROSSWALK.
        decision = self.step(0.0, zone=CROSSWALK)
        self.assertFalse(decision.stop)
        self.assertEqual(decision.state, CROSSWALK)

    def test_yellow_never_releases_and_green_must_be_stable(self):
        yellow = self.step(
            0.0,
            zone=SIGNAL,
            traffic_state='yellow',
        )
        self.assertTrue(yellow.stop)
        first_green = self.step(
            1.0,
            zone=SIGNAL,
            traffic_state='green',
        )
        self.assertTrue(first_green.stop)
        released = self.step(
            1.51,
            zone=SIGNAL,
            traffic_state='green',
        )
        self.assertFalse(released.stop)

    def test_stale_signal_vision_is_fail_closed(self):
        stale = self.step(
            0.0,
            zone=SIGNAL,
            traffic_state='green',
            traffic_fresh=False,
        )
        self.assertTrue(stale.stop)

    def test_regulation_blue_go_indication_is_accepted(self):
        first_blue = self.step(
            0.0,
            zone=SIGNAL,
            traffic_state='blue',
        )
        self.assertTrue(first_blue.stop)
        released = self.step(
            0.51,
            zone=SIGNAL,
            traffic_state='blue',
        )
        self.assertFalse(released.stop)
        self.assertEqual(
            released.state,
            'SIGNAL_RELEASE_PENDING_COMMIT',
        )

    def test_left_arrow_go_indication_is_accepted(self):
        self.assertTrue(
            self.step(
                0.0,
                zone=SIGNAL,
                traffic_state='green_left',
            ).stop
        )
        released = self.step(
            0.51,
            zone=SIGNAL,
            traffic_state='green_left',
        )
        self.assertFalse(released.stop)

    def test_signal_rechecks_colour_until_odom_commit(self):
        self.step(0.0, zone=SIGNAL, traffic_state='green')
        self.assertFalse(
            self.step(0.51, zone=SIGNAL, traffic_state='green').stop
        )
        red_before_commit = self.step(
            0.6,
            zone=SIGNAL,
            traffic_state='red',
        )
        self.assertTrue(red_before_commit.stop)

        self.step(1.0, zone=SIGNAL, traffic_state='green')
        self.assertFalse(
            self.step(1.51, zone=SIGNAL, traffic_state='green').stop
        )
        committed = self.step(
            1.6,
            zone=SIGNAL,
            traffic_state='green',
            signal_committed=True,
        )
        self.assertFalse(committed.stop)
        self.assertEqual(committed.state, 'SIGNAL_COMMITTED')
        # Once physically committed, braking mid-intersection is avoided.
        self.assertFalse(
            self.step(
                1.7,
                zone=SIGNAL,
                traffic_state='unknown',
            ).stop
        )

        self.step(2.0, zone=NAVIGATION)
        second_signal = self.step(
            3.0,
            zone=SIGNAL,
            traffic_state='green',
        )
        self.assertTrue(second_signal.stop)

    def test_signal_release_fails_closed_if_odom_becomes_stale(self):
        self.step(0.0, zone=SIGNAL, traffic_state='green')
        self.assertFalse(
            self.step(0.51, zone=SIGNAL, traffic_state='green').stop
        )
        stale = self.step(
            0.6,
            zone=SIGNAL,
            traffic_state='green',
            odom_fresh=False,
        )
        self.assertTrue(stale.stop)
        self.assertEqual(stale.state, 'SIGNAL_WAIT_COMMIT_POSE')

    def test_clock_regression_relocks_signal_release(self):
        core = MissionSafety(
            green_confirm_sec=0.5,
            observation_gap_timeout_sec=1.0,
        )
        args = {
            'zone': SIGNAL,
            'odom_fresh': True,
            'traffic_state': 'green',
            'traffic_fresh': True,
            'signal_tracking_fresh': True,
        }
        self.assertTrue(core.step(now_sec=10.0, **args).stop)
        self.assertFalse(core.step(now_sec=10.51, **args).stop)
        after_regression = core.step(now_sec=1.0, **args)
        self.assertTrue(after_regression.stop)
        self.assertEqual(after_regression.state, 'SIGNAL_CONFIRM_GREEN')

    def test_unobserved_gap_does_not_confirm_green(self):
        core = MissionSafety(
            green_confirm_sec=0.5,
            observation_gap_timeout_sec=0.35,
        )
        args = {
            'zone': SIGNAL,
            'odom_fresh': True,
            'traffic_state': 'green',
            'traffic_fresh': True,
            'signal_tracking_fresh': True,
        }
        self.assertTrue(core.step(now_sec=0.0, **args).stop)
        after_gap = core.step(now_sec=1.0, **args)
        self.assertTrue(after_gap.stop)
        self.assertEqual(after_gap.state, 'SIGNAL_CONFIRM_GREEN')

    def test_invalid_mission_clock_is_fail_closed(self):
        invalid = self.step(float('nan'))
        self.assertTrue(invalid.stop)
        self.assertEqual(invalid.state, 'MISSION_INVALID_TIME')


if __name__ == '__main__':
    unittest.main()
