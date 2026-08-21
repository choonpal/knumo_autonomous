#!/usr/bin/env python3
"""Publish the regulation-aware mission stop heartbeat on /control/state."""

from __future__ import annotations

import math
import re
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .mission_core import (
    NAVIGATION,
    SIGNAL,
    MissionSafety,
    SignalCommitProgress,
    mission_zone_from_waypoint,
    same_reference_frame,
    source_stamp_advances,
    source_stamp_is_fresh,
)


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_control')

        self.declare_parameter(
            'odom_topic',
            '/scout_mini_base_controller/odom',
        )
        self.declare_parameter('vision_timeout_sec', 0.50)
        self.declare_parameter('traffic_timeout_sec', 1.00)
        self.declare_parameter('odom_timeout_sec', 0.35)
        self.declare_parameter('green_confirm_sec', 0.50)
        self.declare_parameter('observation_gap_timeout_sec', 0.35)
        # 0.50 -> 0.15 (2026-08-17). CommandGuard 가감속 0.60 m/s^2 에서 0.50 m 를
        # 채우려면 해제 후 약 1.5 초를 더 달려야 하는데, 실측 초록 검출 지속은
        # 1.2~1.4 초뿐이라 커밋 전에 매번 재정지했다(4/4 실패, 실이동 0.03~0.10 m).
        # 0.15 m 는 약 0.7 초 주행이면 확정된다.
        self.declare_parameter('signal_commit_distance_m', 0.15)
        # 신호 웨이포인트까지 이 거리 안으로 들어와야 정지를 요구한다.
        # waypoint_follower_node._stopline_step 의 stopline_trigger_dist 와 같은
        # 형태의 트리거다(이름으로 구역을 고르고, 거리로 발동 시점을 정한다).
        #
        # 이 값이 없을 때는 팔로워가 신호점을 "목표로 삼는 순간" 정지가 걸려서
        # 직전 웨이포인트 위치 + 제동거리에 섰다. 실측(bag run_20260817_195906):
        # wp003 통과 0.18 m 지점에서 정지요구 -> 1.62 m 더 가서 정지 ->
        # wp004_signal 보다 0.65 m 앞. 즉 정지 위치가 직전 점 간격에 좌우됐다.
        #
        # 1.80 = 실측 제동거리 1.62 m(1.5 m/s, CommandGuard 0.60 m/s^2) + 여유.
        # 속도 상한(SIGNAL 구역 1.50 m/s)을 바꾸면 이 값도 같이 조정할 것.
        self.declare_parameter('signal_trigger_dist', 1.80)
        self.declare_parameter('path_reference_timeout_sec', 0.50)
        self.declare_parameter(
            'tracking_pose_topic',
            '/follower/control_pose',
        )
        self.declare_parameter('rate_hz', 10.0)

        self.vision_timeout = float(
            self.get_parameter('vision_timeout_sec').value
        )
        self.traffic_timeout = float(
            self.get_parameter('traffic_timeout_sec').value
        )
        self.odom_timeout = float(
            self.get_parameter('odom_timeout_sec').value
        )
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.signal_commit_distance = float(
            self.get_parameter('signal_commit_distance_m').value
        )
        self.signal_trigger_dist = float(
            self.get_parameter('signal_trigger_dist').value
        )
        self.path_reference_timeout = float(
            self.get_parameter('path_reference_timeout_sec').value
        )
        self.tracking_pose_topic = str(
            self.get_parameter('tracking_pose_topic').value
        ).strip()
        positive_limits = (
            self.vision_timeout,
            self.traffic_timeout,
            self.odom_timeout,
            self.signal_commit_distance,
            self.signal_trigger_dist,
            self.path_reference_timeout,
            rate_hz,
        )
        if (
            not all(
                math.isfinite(value) and value > 0.0
                for value in positive_limits
            )
            or not self.tracking_pose_topic
        ):
            raise ValueError('mission control limits must be positive')

        self.core = MissionSafety(
            green_confirm_sec=float(
                self.get_parameter('green_confirm_sec').value
            ),
            observation_gap_timeout_sec=float(
                self.get_parameter(
                    'observation_gap_timeout_sec'
                ).value
            ),
        )

        self.waypoint_info = ''
        self.waypoint_x: Optional[float] = None
        self.waypoint_y: Optional[float] = None
        self.vision_healthy = False
        self.traffic_state = 'unknown'
        self.odom_source_stamp_ns: Optional[int] = None
        # Scout odom only proves the odometry source is alive here; Signal
        # progress uses the follower pose and path in one common frame.
        self.tracking_x: Optional[float] = None
        self.tracking_y: Optional[float] = None
        self.tracking_frame = ''
        self.tracking_stamp_ns: Optional[int] = None
        self.tracking_source_stamp_ns: Optional[int] = None
        self.signal_progress = SignalCommitProgress()
        self.path_yaw: Optional[float] = None
        self.path_frame = ''
        self.path_stamp_ns: Optional[int] = None
        self.path_source_stamp_ns: Optional[int] = None
        self.vision_stamp_ns: Optional[int] = None
        self.traffic_stamp_ns: Optional[int] = None
        self.odom_stamp_ns: Optional[int] = None
        self.last_state = ''

        self.pub_stop = self.create_publisher(
            Bool,
            '/control/state',
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            '/control/mission_state',
            10,
        )
        self.create_subscription(
            String,
            '/follower/active_wp',
            self._on_waypoint,
            10,
        )
        self.create_subscription(
            String,
            '/traffic_light/state',
            self._on_traffic,
            10,
        )
        self.create_subscription(
            Bool,
            '/vision/health',
            self._on_vision_health,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.tracking_pose_topic,
            self._on_tracking_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/follower/debug/target',
            self._on_path_target,
            10,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _fresh(self, stamp_ns: Optional[int], timeout: float) -> bool:
        if stamp_ns is None:
            return False
        age_ns = self._now_ns() - stamp_ns
        return 0 <= age_ns <= int(timeout * 1e9)

    @staticmethod
    def _header_stamp_ns(msg) -> int:
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    @staticmethod
    def _yaw_from_pose(msg: PoseStamped) -> Optional[float]:
        q = msg.pose.orientation
        values = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-9:
            return None
        x, y, z, w = (value / norm for value in values)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return yaw if math.isfinite(yaw) else None

    def _on_waypoint(self, msg: String) -> None:
        self.waypoint_info = msg.data
        # waypoint_follower_node 가 "i/n, name=..., x=..., y=..." 형식으로 낸다.
        # 좌표는 팔로워가 추종에 쓰는 프레임(=/follower/control_pose 프레임)과
        # 같으므로 그대로 거리 계산에 쓸 수 있다. 형식이 바뀌어 파싱이 실패하면
        # 좌표를 버리고, 거리 게이트는 발동하지 않는다(= 종전처럼 즉시 정지).
        match = re.search(
            r'x=(-?\d+(?:\.\d+)?),\s*y=(-?\d+(?:\.\d+)?)',
            msg.data,
        )
        if match is None:
            self.waypoint_x = None
            self.waypoint_y = None
            return
        values = (float(match.group(1)), float(match.group(2)))
        if not all(math.isfinite(value) for value in values):
            self.waypoint_x = None
            self.waypoint_y = None
            return
        self.waypoint_x, self.waypoint_y = values

    def _signal_distance(self) -> Optional[float]:
        """현재 위치에서 활성 신호 웨이포인트까지의 거리."""
        if (
            self.waypoint_x is None
            or self.waypoint_y is None
            or self.tracking_x is None
            or self.tracking_y is None
        ):
            return None
        if not self._fresh(self.tracking_stamp_ns, self.path_reference_timeout):
            return None
        return math.hypot(
            self.waypoint_x - self.tracking_x,
            self.waypoint_y - self.tracking_y,
        )

    def _armed_zone(self, zone: str) -> str:
        """거리 트리거. stopline 과 같은 형태로 발동 시점을 늦춘다.

        신호 구역이라는 것은 이름으로 정하고(zone), 실제로 정지를 요구할지는
        신호점까지의 거리로 정한다. 아직 멀면 NAVIGATION 으로 넘겨 주행을
        이어가고, signal_trigger_dist 안으로 들어와야 SIGNAL 을 넘긴다.

        이미 latch/release/done 된 뒤에는 게이트를 걸지 않는다. 연속된
        _signal 점(wp004~wp007 같은) 사이에서 zone 이 NAVIGATION 으로
        떨어지면 MissionSafety._reset_after_exit 이 signal_done 을 지워
        다음 점에서 또 정지하기 때문이다.
        """
        if zone != SIGNAL:
            return zone
        if (
            self.core.signal_latched
            or self.core.signal_released
            or self.core.signal_done
        ):
            return zone
        distance = self._signal_distance()
        if distance is None:
            # 좌표나 위치를 못 믿는 상황에서는 늦추지 않는다(fail-closed).
            return zone
        return zone if distance <= self.signal_trigger_dist else NAVIGATION

    def _on_traffic(self, msg: String) -> None:
        state = msg.data.strip().lower()
        self.traffic_state = (
            state
            if state in {
                'red',
                'yellow',
                'green',
                'blue',
                'left',
                'green_left',
                'blue_left',
            }
            else 'unknown'
        )
        self.traffic_stamp_ns = self._now_ns()

    def _on_vision_health(self, msg: Bool) -> None:
        self.vision_healthy = bool(msg.data)
        self.vision_stamp_ns = self._now_ns()

    def _on_odom(self, msg: Odometry) -> None:
        now_ns = self._now_ns()
        if (
            self.odom_stamp_ns is not None
            and now_ns < self.odom_stamp_ns
        ):
            self.odom_stamp_ns = None
            self.odom_source_stamp_ns = None
        source_stamp_ns = self._header_stamp_ns(msg)
        if not source_stamp_is_fresh(
            now_ns=now_ns,
            stamp_ns=source_stamp_ns,
            timeout_sec=self.odom_timeout,
        ):
            return
        if not source_stamp_advances(
            self.odom_source_stamp_ns,
            source_stamp_ns,
        ):
            return
        # Odometry no longer drives any decision beyond liveness, but a
        # corrupt (non-finite) message must still not count as fresh.
        values = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if not all(math.isfinite(value) for value in values):
            return
        self.odom_source_stamp_ns = source_stamp_ns
        self.odom_stamp_ns = now_ns

    def _on_tracking_pose(self, msg: PoseStamped) -> None:
        now_ns = self._now_ns()
        if (
            self.tracking_stamp_ns is not None
            and now_ns < self.tracking_stamp_ns
        ):
            self.tracking_stamp_ns = None
            self.tracking_source_stamp_ns = None
        source_stamp_ns = self._header_stamp_ns(msg)
        if not source_stamp_is_fresh(
            now_ns=now_ns,
            stamp_ns=source_stamp_ns,
            timeout_sec=self.path_reference_timeout,
        ):
            return
        if not source_stamp_advances(
            self.tracking_source_stamp_ns,
            source_stamp_ns,
        ):
            return
        position = msg.pose.position
        values = (float(position.x), float(position.y))
        frame = str(msg.header.frame_id).strip()
        if (
            not all(math.isfinite(value) for value in values)
            or not frame
            or self._yaw_from_pose(msg) is None
        ):
            return
        self.tracking_x, self.tracking_y = values
        self.tracking_frame = frame
        self.tracking_source_stamp_ns = source_stamp_ns
        self.tracking_stamp_ns = now_ns

    def _on_path_target(self, msg: PoseStamped) -> None:
        now_ns = self._now_ns()
        if (
            self.path_stamp_ns is not None
            and now_ns < self.path_stamp_ns
        ):
            self.path_stamp_ns = None
            self.path_source_stamp_ns = None
        source_stamp_ns = self._header_stamp_ns(msg)
        if not source_stamp_is_fresh(
            now_ns=now_ns,
            stamp_ns=source_stamp_ns,
            timeout_sec=self.path_reference_timeout,
        ):
            return
        if not source_stamp_advances(
            self.path_source_stamp_ns,
            source_stamp_ns,
        ):
            return
        frame = str(msg.header.frame_id).strip()
        yaw = self._yaw_from_pose(msg)
        if not frame or yaw is None:
            return
        self.path_yaw = yaw
        self.path_frame = frame
        self.path_source_stamp_ns = source_stamp_ns
        self.path_stamp_ns = now_ns

    def _signal_tracking_fresh(self) -> bool:
        return (
            self._fresh(
                self.path_stamp_ns,
                self.path_reference_timeout,
            )
            and self._fresh(
                self.tracking_stamp_ns,
                self.path_reference_timeout,
            )
            and same_reference_frame(
                self.path_frame,
                self.tracking_frame,
            )
        )

    def _tick(self) -> None:
        named_zone = mission_zone_from_waypoint(self.waypoint_info)
        zone = self._armed_zone(named_zone)
        vision_fresh = (
            self.vision_healthy
            and self._fresh(self.vision_stamp_ns, self.vision_timeout)
        )
        traffic_fresh = (
            vision_fresh
            and self._fresh(
                self.traffic_stamp_ns,
                self.traffic_timeout,
            )
        )
        odom_fresh = self._fresh(self.odom_stamp_ns, self.odom_timeout)
        signal_tracking_fresh = self._signal_tracking_fresh()
        signal_committed = False
        if (
            odom_fresh
            and signal_tracking_fresh
            and self.tracking_x is not None
            and self.tracking_y is not None
        ):
            progress = self.signal_progress.progress(
                current_x=self.tracking_x,
                current_y=self.tracking_y,
                current_frame=self.tracking_frame,
            )
            signal_committed = (
                progress is not None
                and progress >= self.signal_commit_distance
            )

        decision = self.core.step(
            now_sec=self._now_ns() / 1e9,
            zone=zone,
            odom_fresh=odom_fresh,
            traffic_state=self.traffic_state,
            traffic_fresh=traffic_fresh,
            signal_committed=signal_committed,
            signal_tracking_fresh=signal_tracking_fresh,
        )
        if decision.state == 'SIGNAL_RELEASE_PENDING_COMMIT':
            if (
                not self.signal_progress.ready
                and odom_fresh
                and signal_tracking_fresh
                and self.tracking_x is not None
                and self.tracking_y is not None
                and self.path_yaw is not None
            ):
                self.signal_progress.capture(
                    x=self.tracking_x,
                    y=self.tracking_y,
                    path_yaw=self.path_yaw,
                    pose_frame=self.tracking_frame,
                    path_frame=self.path_frame,
                )
        elif decision.state != 'SIGNAL_COMMITTED':
            self.signal_progress.clear()
        self.pub_stop.publish(Bool(data=decision.stop))
        self.pub_state.publish(String(data=decision.state))

        if decision.state != self.last_state:
            self.last_state = decision.state
            distance = self._signal_distance()
            gated = '' if named_zone == zone else f' (named={named_zone}, 대기중)'
            wp_dist = '?' if distance is None else f'{distance:.2f}m'
            self.get_logger().info(
                f'zone={zone}, state={decision.state}, '
                f'stop={decision.stop}, wp_dist={wp_dist}{gated}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
