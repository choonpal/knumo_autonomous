#!/usr/bin/env python3
"""Follower/회피/미션 명령 중 하나를 최종 /cmd_vel로 선택한다."""

from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

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


class MainController(Node):
    def __init__(self) -> None:
        super().__init__("main_controller_node")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("debug", True)
        self.declare_parameter("cmd_out", "/cmd_vel")
        self.declare_parameter("follow_topic", "/cmd_vel/follow")
        self.declare_parameter("avoid_topic", "/cmd_vel/avoid")
        self.declare_parameter("avoid_active_topic", "/avoid_active")
        # 후진주차 채널. 후진은 reverse_linear_cap=0.0으로 전 구간 잠겨
        # 있고, 이 소스로 들어온 명령만 parking_reverse_cap으로 열린다.
        self.declare_parameter("parking_topic", "/cmd_vel/parking")
        self.declare_parameter("parking_active_topic", "/parking_active")
        self.declare_parameter("mission_stop_topic", "/control/state")
        self.declare_parameter(
            "active_speed_cap_topic",
            "/follower/active_speed_cap",
        )
        self.declare_parameter("command_timeout", 0.40)
        self.declare_parameter("state_timeout", 0.60)
        self.declare_parameter("active_speed_cap_timeout", 0.35)
        self.declare_parameter("require_mission_heartbeat", False)
        self.declare_parameter("require_speed_cap", False)
        self.declare_parameter("rate_hz", 20.0)
        # mission 모드에서도 follower가 계산한 0.25 m/s 기준 명령을
        # selector가 임의로 0.15 m/s로 잘라 경로추종 특성을 바꾸지 않는다.
        self.declare_parameter("follow_linear_cap", 0.25)
        self.declare_parameter("avoid_linear_cap", 0.12)
        self.declare_parameter("reverse_linear_cap", 0.0)
        self.declare_parameter("parking_linear_cap", 0.15)
        # fail-closed: 설정에서 명시적으로 열어주지 않으면 주차 소스도 후진 못 함
        self.declare_parameter("parking_reverse_cap", 0.0)
        self.declare_parameter("angular_cap", 0.80)
        self.declare_parameter("linear_slew_rate", 0.30)
        self.declare_parameter("angular_slew_rate", 1.50)
        self.declare_parameter("slew_max_dt", 0.20)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.debug = bool(self.get_parameter("debug").value)
        self.cmd_out = str(self.get_parameter("cmd_out").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.active_speed_cap_timeout = float(
            self.get_parameter("active_speed_cap_timeout").value
        )
        self.require_mission_heartbeat = bool(
            self.get_parameter("require_mission_heartbeat").value
        )
        self.require_speed_cap = bool(
            self.get_parameter("require_speed_cap").value
        )
        rate_hz = float(self.get_parameter("rate_hz").value)
        if (
            not values_are_finite(
                (rate_hz, self.command_timeout, self.state_timeout)
            )
            or not values_are_finite((self.active_speed_cap_timeout,))
            or min(
                rate_hz,
                self.command_timeout,
                self.state_timeout,
                self.active_speed_cap_timeout,
            ) <= 0.0
        ):
            raise ValueError("controller rates/timeouts must be finite and positive")
        self.command_guard = CommandGuard(
            GuardLimits(
                follow_forward_cap=float(
                    self.get_parameter("follow_linear_cap").value
                ),
                avoid_forward_cap=float(
                    self.get_parameter("avoid_linear_cap").value
                ),
                reverse_cap=float(
                    self.get_parameter("reverse_linear_cap").value
                ),
                parking_forward_cap=float(
                    self.get_parameter("parking_linear_cap").value
                ),
                parking_reverse_cap=float(
                    self.get_parameter("parking_reverse_cap").value
                ),
                angular_cap=float(self.get_parameter("angular_cap").value),
                linear_slew_rate=float(
                    self.get_parameter("linear_slew_rate").value
                ),
                angular_slew_rate=float(
                    self.get_parameter("angular_slew_rate").value
                ),
                nominal_dt=1.0 / rate_hz,
                max_slew_dt=float(self.get_parameter("slew_max_dt").value),
            )
        )

        self.follow_cmd = Twist()
        self.avoid_cmd = Twist()
        self.avoid_active = False
        self.parking_cmd = Twist()
        self.parking_active = False
        self.parking_stamp_ns = 0
        self.parking_active_stamp_ns = 0
        self.mission_stop = False
        self.active_speed_cap = None
        self.follow_stamp_ns = 0
        self.avoid_stamp_ns = 0
        self.avoid_active_stamp_ns = 0
        self.mission_stamp_ns = 0
        self.active_speed_cap_stamp_ns = 0
        self.last_log_ns = 0
        self.last_source = ""

        self.create_subscription(
            Twist,
            str(self.get_parameter("follow_topic").value),
            self._on_follow,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("avoid_topic").value),
            self._on_avoid_cmd,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("avoid_active_topic").value),
            self._on_avoid_active,
            10,
        )
        parking_topic = str(self.get_parameter("parking_topic").value)
        parking_active_topic = str(
            self.get_parameter("parking_active_topic").value
        )
        if parking_topic and parking_active_topic:
            self.create_subscription(
                Twist, parking_topic, self._on_parking_cmd, 10
            )
            self.create_subscription(
                Bool, parking_active_topic, self._on_parking_active, 10
            )
        mission_topic = str(self.get_parameter("mission_stop_topic").value)
        if mission_topic:
            self.create_subscription(
                Bool,
                mission_topic,
                self._on_mission_stop,
                10,
            )
        active_speed_cap_topic = str(
            self.get_parameter("active_speed_cap_topic").value
        ).strip()
        if not active_speed_cap_topic:
            raise ValueError("active_speed_cap_topic must not be empty")
        self.create_subscription(
            Float32,
            active_speed_cap_topic,
            self._on_active_speed_cap,
            10,
        )

        self.pub_final = self.create_publisher(Twist, self.cmd_out, 10)
        self.pub_preview = self.create_publisher(
            Twist,
            "/control/debug/cmd_vel_preview",
            10,
        )
        self.pub_sel = self.create_publisher(
            String,
            "/control/debug/selected_source",
            10,
        )

        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            "MainController ready: follower < avoid < parking < mission-stop, "
            f"dry_run={self.dry_run}, out={self.cmd_out}, "
            "guard=enabled"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    @staticmethod
    def _age(now_ns: int, stamp_ns: int) -> float:
        if stamp_ns == 0:
            return float("inf")
        age = (now_ns - stamp_ns) / 1e9
        return age if age >= 0.0 else float("inf")

    def _on_follow(self, msg: Twist) -> None:
        self.follow_cmd = msg
        self.follow_stamp_ns = self._now_ns()

    def _on_avoid_cmd(self, msg: Twist) -> None:
        self.avoid_cmd = msg
        self.avoid_stamp_ns = self._now_ns()

    def _on_avoid_active(self, msg: Bool) -> None:
        self.avoid_active = bool(msg.data)
        self.avoid_active_stamp_ns = self._now_ns()

    def _on_parking_cmd(self, msg: Twist) -> None:
        self.parking_cmd = msg
        self.parking_stamp_ns = self._now_ns()

    def _on_parking_active(self, msg: Bool) -> None:
        self.parking_active = bool(msg.data)
        self.parking_active_stamp_ns = self._now_ns()

    def _on_mission_stop(self, msg: Bool) -> None:
        self.mission_stop = bool(msg.data)
        self.mission_stamp_ns = self._now_ns()

    def _on_active_speed_cap(self, msg: Float32) -> None:
        value = float(msg.data)
        self.active_speed_cap = (
            value if values_are_finite((value,)) and value > 0.0 else None
        )
        self.active_speed_cap_stamp_ns = self._now_ns()

    def _fresh_active_speed_cap(self, now_ns: int):
        return active_speed_cap_if_fresh(
            now_ns=now_ns,
            received_ns=(
                self.active_speed_cap_stamp_ns
                if self.active_speed_cap_stamp_ns
                else None
            ),
            timeout_sec=self.active_speed_cap_timeout,
            value=self.active_speed_cap,
        )

    def _select(self, now_ns: int) -> Tuple[str, Twist]:
        stop = Twist()
        if (
            self.require_speed_cap
            and self._fresh_active_speed_cap(now_ns) is None
        ):
            return "ACTIVE_SPEED_CAP_INTERLOCK", stop
        if (
            self.require_mission_heartbeat
            and self._age(now_ns, self.mission_stamp_ns) > self.state_timeout
        ):
            return "MISSION_HEARTBEAT_LOST", stop
        if self.mission_stop:
            return "MISSION_STOP", stop
        # 후진주차는 회피보다 우선한다. 슬롯 안으로 들어가는 기동 중에는
        # 옆 차가 가까운 게 정상이라 VFH 회피가 개입하면 안 된다.
        if (
            self.parking_active
            and self._age(now_ns, self.parking_active_stamp_ns)
            <= self.state_timeout
        ):
            if self._age(now_ns, self.parking_stamp_ns) > self.command_timeout:
                return "PARKING_COMMAND_STALE", stop
            return PARKING_SOURCE, self.parking_cmd
        follow_fresh = (
            self._age(now_ns, self.follow_stamp_ns) <= self.command_timeout
        )
        if self._age(now_ns, self.avoid_active_stamp_ns) > self.state_timeout:
            if follow_fresh:
                return FOLLOW_SOURCE, self.follow_cmd
            return "AVOIDER_HEARTBEAT_LOST", stop
        if self.avoid_active:
            if self._age(now_ns, self.avoid_stamp_ns) > self.command_timeout:
                if follow_fresh:
                    return FOLLOW_SOURCE, self.follow_cmd
                return "AVOID_COMMAND_STALE", stop
            return AVOID_SOURCE, self.avoid_cmd
        if not follow_fresh:
            return "FOLLOW_COMMAND_STALE", stop
        return FOLLOW_SOURCE, self.follow_cmd

    @staticmethod
    def _twist_is_finite(command: Twist) -> bool:
        return values_are_finite(
            (
                command.linear.x,
                command.linear.y,
                command.linear.z,
                command.angular.x,
                command.angular.y,
                command.angular.z,
            )
        )

    @staticmethod
    def _planar_twist(linear: float, angular: float) -> Twist:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        return command

    def _tick(self) -> None:
        now_ns = self._now_ns()
        source, command = self._select(now_ns)
        motion_source = source in (FOLLOW_SOURCE, AVOID_SOURCE, PARKING_SOURCE)
        command_finite = self._twist_is_finite(command)
        active_speed_cap = (
            self._fresh_active_speed_cap(now_ns)
            if self.require_speed_cap
            else None
        )
        speed_cap_violation = (
            motion_source
            and active_speed_cap is not None
            and not forward_command_within_speed_cap(
                command.linear.x,
                active_speed_cap,
            )
        )
        guarded = self.command_guard.apply(
            source,
            command.linear.x,
            command.angular.z,
            now_ns / 1e9,
            force_stop=(
                not motion_source
                or not command_finite
                or speed_cap_violation
            ),
            forward_cap_override=(
                active_speed_cap if motion_source else None
            ),
        )
        if motion_source and not command_finite:
            source = f"{source}_NONFINITE"
        elif speed_cap_violation:
            source = f"{source}_ACTIVE_SPEED_CAP_EXCEEDED"
        elif (
            motion_source
            and guarded.reason not in ("OK", "AVOID_VERIFIED")
        ):
            source = f"{source}_{guarded.reason}"
        safe_command = self._planar_twist(
            guarded.linear,
            guarded.angular,
        )

        self.pub_preview.publish(safe_command)
        self.pub_sel.publish(String(data=source))
        if not self.dry_run:
            self.pub_final.publish(safe_command)

        if self.debug and (
            source != self.last_source or now_ns - self.last_log_ns >= int(1e9)
        ):
            self.last_source = source
            self.last_log_ns = now_ns
            self.get_logger().info(
                f"winner={source}, v={safe_command.linear.x:.2f}, "
                f"w={safe_command.angular.z:.2f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
