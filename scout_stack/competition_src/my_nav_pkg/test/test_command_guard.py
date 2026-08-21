import math
import unittest

from my_nav_pkg.command_guard import (
    AVOID_SOURCE,
    FOLLOW_SOURCE,
    PARKING_SOURCE,
    CommandGuard,
    GuardLimits,
    active_speed_cap_if_fresh,
    forward_command_within_speed_cap,
    values_are_finite,
)


class CommandGuardTest(unittest.TestCase):
    @staticmethod
    def _fast_limits() -> GuardLimits:
        return GuardLimits(
            follow_forward_cap=0.30,
            avoid_forward_cap=0.12,
            reverse_cap=0.10,
            angular_cap=1.00,
            linear_slew_rate=100.0,
            angular_slew_rate=100.0,
            nominal_dt=0.10,
            max_slew_dt=0.10,
        )

    def test_follower_is_capped_but_verified_avoid_pair_is_unchanged(self):
        guard = CommandGuard(self._fast_limits())

        follow = guard.apply(FOLLOW_SOURCE, 9.0, 9.0, 1.0)
        self.assertAlmostEqual(follow.linear, 0.30)
        self.assertAlmostEqual(follow.angular, 1.00)

        handoff = guard.apply(AVOID_SOURCE, 0.12, -0.80, 1.1)
        self.assertTrue(handoff.stopped)
        self.assertEqual(handoff.reason, "SOURCE_SWITCH_STOP")

        avoid = guard.apply(AVOID_SOURCE, 0.12, -0.80, 1.2)
        self.assertAlmostEqual(avoid.linear, 0.12)
        self.assertAlmostEqual(avoid.angular, -0.80)
        self.assertEqual(avoid.reason, "AVOID_VERIFIED")

        reverse = guard.apply(AVOID_SOURCE, -0.10, 0.2, 1.3)
        self.assertAlmostEqual(reverse.linear, -0.10)
        self.assertAlmostEqual(reverse.angular, 0.20)

    def test_out_of_contract_avoid_pair_stops_instead_of_changing_curvature(self):
        for linear, angular in (
            (0.121, 0.0),
            (-0.101, 0.0),
            (0.08, 1.001),
        ):
            with self.subTest(linear=linear, angular=angular):
                guard = CommandGuard(self._fast_limits())
                result = guard.apply(
                    AVOID_SOURCE,
                    linear,
                    angular,
                    1.0,
                )
                self.assertTrue(result.stopped)
                self.assertEqual(result.reason, "AVOID_LIMIT_VIOLATION")
                self.assertEqual((result.linear, result.angular), (0.0, 0.0))

    def test_nonfinite_input_bypasses_slew_and_stops(self):
        guard = CommandGuard(self._fast_limits())
        moving = guard.apply(FOLLOW_SOURCE, 0.30, 0.5, 1.0)
        self.assertGreater(moving.linear, 0.0)

        for linear, angular, now_sec in (
            (math.nan, 0.0, 1.1),
            (0.1, math.inf, 1.2),
            (0.1, 0.0, math.nan),
        ):
            stopped = guard.apply(
                FOLLOW_SOURCE,
                linear,
                angular,
                now_sec,
            )
            self.assertTrue(stopped.stopped)
            self.assertEqual(stopped.reason, "NONFINITE")
            self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))

    def test_all_twist_components_can_be_checked_for_finiteness(self):
        self.assertTrue(values_are_finite((0.0, 0.1, -0.2, 0.3, 0.0, 0.4)))
        self.assertFalse(
            values_are_finite((0.0, 0.1, math.nan, 0.3, 0.0, 0.4))
        )
        self.assertFalse(values_are_finite((0.0, "not-a-number")))

    def test_active_speed_cap_requires_fresh_positive_value(self):
        kwargs = {
            "now_ns": 2_000_000_000,
            "timeout_sec": 0.35,
        }
        self.assertAlmostEqual(
            active_speed_cap_if_fresh(
                received_ns=1_800_000_000,
                value=0.08,
                **kwargs,
            ),
            0.08,
        )
        for received_ns, value in (
            (None, 0.08),
            (1_649_999_999, 0.08),
            (2_000_000_001, 0.08),
            (1_800_000_000, 0.0),
            (1_800_000_000, math.nan),
        ):
            with self.subTest(received_ns=received_ns, value=value):
                self.assertIsNone(
                    active_speed_cap_if_fresh(
                        received_ns=received_ns,
                        value=value,
                        **kwargs,
                    )
                )

    def test_positive_command_above_active_cap_is_rejected_not_clipped(self):
        self.assertTrue(forward_command_within_speed_cap(0.08, 0.08))
        self.assertTrue(forward_command_within_speed_cap(-0.10, 0.08))
        self.assertFalse(forward_command_within_speed_cap(0.081, 0.08))
        self.assertFalse(
            forward_command_within_speed_cap(math.nan, 0.08)
        )

    def test_falling_dynamic_cap_also_limits_post_slew_output(self):
        guard = CommandGuard(self._fast_limits())
        fast = guard.apply(
            FOLLOW_SOURCE,
            0.30,
            0.60,
            1.0,
            forward_cap_override=0.30,
        )
        self.assertAlmostEqual(fast.linear, 0.30)

        slowed = guard.apply(
            FOLLOW_SOURCE,
            0.08,
            0.16,
            1.1,
            forward_cap_override=0.08,
        )
        self.assertLessEqual(slowed.linear, 0.08)

    def test_verified_avoid_pair_cannot_exceed_dynamic_cap(self):
        guard = CommandGuard(self._fast_limits())
        result = guard.apply(
            AVOID_SOURCE,
            0.10,
            0.40,
            1.0,
            forward_cap_override=0.08,
        )
        self.assertTrue(result.stopped)
        self.assertEqual(result.reason, "AVOID_LIMIT_VIOLATION")

    def test_invalid_dynamic_cap_fails_closed(self):
        guard = CommandGuard(self._fast_limits())
        for cap in (0.0, -0.1, math.nan, math.inf):
            with self.subTest(cap=cap):
                result = guard.apply(
                    FOLLOW_SOURCE,
                    0.05,
                    0.0,
                    1.0,
                    forward_cap_override=cap,
                )
                self.assertTrue(result.stopped)
                self.assertEqual(result.reason, "DYNAMIC_CAP_INVALID")

    def test_zero_reverse_cap_rejects_reverse_commands(self):
        limits = GuardLimits(
            follow_forward_cap=0.15,
            avoid_forward_cap=0.12,
            reverse_cap=0.0,
            angular_cap=0.8,
            linear_slew_rate=0.3,
            angular_slew_rate=1.5,
            nominal_dt=0.05,
            max_slew_dt=0.2,
        )
        result = CommandGuard(limits).apply(
            AVOID_SOURCE,
            -0.1,
            0.0,
            1.0,
        )
        self.assertTrue(result.stopped)
        self.assertEqual(result.reason, "AVOID_LIMIT_VIOLATION")

    def test_normal_commands_obey_linear_and_angular_slew(self):
        guard = CommandGuard(
            GuardLimits(
                linear_slew_rate=0.50,
                angular_slew_rate=1.00,
                nominal_dt=0.10,
                max_slew_dt=0.20,
            )
        )

        first = guard.apply(FOLLOW_SOURCE, 0.30, 1.0, 1.0)
        self.assertAlmostEqual(first.linear, 0.05)
        self.assertAlmostEqual(first.angular, 0.10)

        second = guard.apply(FOLLOW_SOURCE, 0.30, 1.0, 1.1)
        self.assertAlmostEqual(second.linear, 0.10)
        self.assertAlmostEqual(second.angular, 0.20)

    def test_exact_zero_command_bypasses_slew(self):
        guard = CommandGuard(self._fast_limits())
        guard.apply(FOLLOW_SOURCE, 0.30, 0.5, 1.0)

        stopped = guard.apply(FOLLOW_SOURCE, 0.0, 0.0, 1.001)
        self.assertTrue(stopped.stopped)
        self.assertEqual(stopped.reason, "EXACT_ZERO")
        self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))

    def test_forced_emergency_stop_bypasses_slew(self):
        guard = CommandGuard(self._fast_limits())
        guard.apply(FOLLOW_SOURCE, 0.30, 0.5, 1.0)

        stopped = guard.apply(
            "MISSION_STOP",
            0.30,
            0.5,
            1.001,
            force_stop=True,
        )
        self.assertTrue(stopped.stopped)
        self.assertEqual(stopped.reason, "FORCED_STOP")
        self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))

    def test_follow_to_avoid_handoff_has_an_exact_zero_cycle(self):
        guard = CommandGuard(self._fast_limits())
        follow = guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 1.0)
        self.assertAlmostEqual(follow.linear, 0.30)

        handoff = guard.apply(AVOID_SOURCE, 0.12, 0.0, 1.1)
        self.assertTrue(handoff.stopped)
        self.assertEqual(handoff.reason, "SOURCE_SWITCH_STOP")
        self.assertEqual((handoff.linear, handoff.angular), (0.0, 0.0))

        avoid = guard.apply(AVOID_SOURCE, 0.12, 0.0, 1.2)
        self.assertAlmostEqual(avoid.linear, 0.12)
        self.assertLessEqual(avoid.linear, guard.limits.avoid_forward_cap)

    def test_avoid_pair_bypasses_slew_because_exact_curve_was_verified(self):
        guard = CommandGuard(
            GuardLimits(
                linear_slew_rate=0.01,
                angular_slew_rate=0.01,
                nominal_dt=0.10,
                max_slew_dt=0.10,
            )
        )
        result = guard.apply(AVOID_SOURCE, 0.08, 0.70, 1.0)
        self.assertAlmostEqual(result.linear, 0.08)
        self.assertAlmostEqual(result.angular, 0.70)

    def test_avoid_to_follow_handoff_has_an_exact_zero_cycle(self):
        guard = CommandGuard(self._fast_limits())
        guard.apply(AVOID_SOURCE, 0.08, 0.60, 1.0)

        handoff = guard.apply(FOLLOW_SOURCE, 0.15, 0.0, 1.1)
        self.assertTrue(handoff.stopped)
        self.assertEqual(handoff.reason, "SOURCE_SWITCH_STOP")
        self.assertEqual((handoff.linear, handoff.angular), (0.0, 0.0))

    def test_forward_to_reverse_transition_slews_through_zero(self):
        guard = CommandGuard(
            GuardLimits(
                linear_slew_rate=0.50,
                angular_slew_rate=1.00,
                nominal_dt=0.10,
                max_slew_dt=0.10,
            )
        )
        forward = guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 1.0)
        self.assertAlmostEqual(forward.linear, 0.05)

        zero_crossing = guard.apply(FOLLOW_SOURCE, -0.10, 0.0, 1.1)
        self.assertAlmostEqual(zero_crossing.linear, 0.0)

        reverse = guard.apply(FOLLOW_SOURCE, -0.10, 0.0, 1.2)
        self.assertAlmostEqual(reverse.linear, -0.05)

        capped = guard.apply(FOLLOW_SOURCE, -0.50, 0.0, 1.3)
        self.assertAlmostEqual(capped.linear, -0.10)

    def test_bad_or_large_time_delta_cannot_bypass_slew(self):
        guard = CommandGuard(
            GuardLimits(
                linear_slew_rate=0.50,
                angular_slew_rate=1.00,
                nominal_dt=0.10,
                max_slew_dt=0.10,
            )
        )
        guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 10.0)

        backwards = guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 9.9)
        self.assertTrue(backwards.stopped)
        self.assertEqual(backwards.reason, "TIME_INVALID")

        after_long_gap = guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 100.0)
        self.assertAlmostEqual(after_long_gap.linear, 0.05)

        same_time = guard.apply(FOLLOW_SOURCE, 0.30, 0.0, 100.0)
        self.assertTrue(same_time.stopped)
        self.assertEqual(same_time.reason, "TIME_INVALID")

    def test_unknown_motion_source_fails_safe(self):
        guard = CommandGuard(self._fast_limits())
        stopped = guard.apply("UNEXPECTED_SOURCE", 0.2, 0.1, 1.0)
        self.assertTrue(stopped.stopped)
        self.assertEqual(stopped.reason, "UNKNOWN_SOURCE")
        self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))

    @staticmethod
    def _reverse_locked_limits(parking_reverse_cap: float) -> GuardLimits:
        """운영 설정 재현: 일반 주행 후진은 완전히 잠그고 주차만 별도 게이트."""
        return GuardLimits(
            follow_forward_cap=0.15,
            avoid_forward_cap=0.12,
            reverse_cap=0.0,
            parking_forward_cap=0.08,
            parking_reverse_cap=parking_reverse_cap,
            angular_cap=0.80,
            linear_slew_rate=100.0,
            angular_slew_rate=100.0,
            nominal_dt=0.10,
            max_slew_dt=0.10,
        )

    def test_parking_reverse_is_closed_until_explicitly_opened(self):
        # parking_reverse_cap 기본값 0.0이면 주차 소스로 와도 후진 못 한다.
        guard = CommandGuard(self._reverse_locked_limits(0.0))
        parked = guard.apply(PARKING_SOURCE, -0.15, 0.0, 1.0)
        self.assertEqual(parked.linear, 0.0)

    def test_only_parking_source_may_reverse_when_opened(self):
        limits = self._reverse_locked_limits(0.15)
        for source in (FOLLOW_SOURCE, AVOID_SOURCE):
            guard = CommandGuard(limits)
            blocked = guard.apply(source, -0.15, 0.0, 1.0)
            self.assertEqual(
                blocked.linear,
                0.0,
                msg=f"{source} must stay reverse-locked",
            )
        guard = CommandGuard(limits)
        allowed = guard.apply(PARKING_SOURCE, -0.15, 0.0, 1.0)
        self.assertAlmostEqual(allowed.linear, -0.15)

    def test_parking_reverse_is_capped_at_its_own_limit(self):
        guard = CommandGuard(self._reverse_locked_limits(0.10))
        capped = guard.apply(PARKING_SOURCE, -0.40, 0.0, 1.0)
        self.assertAlmostEqual(capped.linear, -0.10)

    def test_parking_forward_uses_its_own_cap(self):
        guard = CommandGuard(self._reverse_locked_limits(0.10))
        capped = guard.apply(PARKING_SOURCE, 0.30, 0.0, 1.0)
        self.assertAlmostEqual(capped.linear, 0.08)

    def test_parking_commands_are_slew_limited_unlike_avoid(self):
        # AVOID는 rollout 검증된 쌍이라 통과시키지만, 주차 명령은 그런 검증이
        # 없으므로 slew 제한을 그대로 받아야 한다.
        limits = GuardLimits(
            follow_forward_cap=0.15,
            avoid_forward_cap=0.12,
            reverse_cap=0.0,
            parking_forward_cap=0.08,
            parking_reverse_cap=0.15,
            angular_cap=0.80,
            linear_slew_rate=0.30,
            angular_slew_rate=1.50,
            nominal_dt=0.10,
            max_slew_dt=0.10,
        )
        guard = CommandGuard(limits)
        first = guard.apply(PARKING_SOURCE, -0.15, 0.0, 1.0)
        self.assertGreater(
            first.linear,
            -0.15,
            msg="parking must not jump straight to full reverse",
        )

    def test_invalid_limits_are_rejected(self):
        with self.assertRaises(ValueError):
            GuardLimits(linear_slew_rate=0.0)
        with self.assertRaises(ValueError):
            GuardLimits(angular_cap=math.nan)
        with self.assertRaises(ValueError):
            GuardLimits(reverse_cap=-0.01)
        with self.assertRaises(ValueError):
            GuardLimits(parking_reverse_cap=-0.01)
        with self.assertRaises(ValueError):
            GuardLimits(parking_forward_cap=0.0)
        with self.assertRaises(ValueError):
            GuardLimits(nominal_dt=0.30, max_slew_dt=0.20)


if __name__ == "__main__":
    unittest.main()
