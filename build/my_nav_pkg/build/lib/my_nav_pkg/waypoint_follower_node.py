import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import os

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        self.declare_parameter('file_name', 'waypoints.csv')
        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        path = os.path.expanduser(f'~/.ros/' + file_name)
        self.waypoints = self._load_waypoints(path)

        self.pub = self.create_publisher(Twist, '/cmd_vel/follow', 10)
        self.sub = self.create_subscription(Odometry, '/scout_mini_base_controller/odom', self._odom_callback, 10)
        self.timer = self.create_timer(0.1, self._control_loop)

        self.linear_speed     = 0.2
        self.angular_speed    = 0.3
        self.yaw_threshold    = math.radians(3)
        self.target_tolerance = 0.3

        self.Kp_ang           = 1.0
        self.min_ang_speed    = math.radians(2)

        self.curr_x = None
        self.curr_y = None
        self.current_yaw = None
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.yaw_offset = 0.0
        self.offset_initialized = False

        self.index = 0
        self.state = 'rotate'

        self.get_logger().info(f"📍 Waypoint Follower Initialized. 총 웨이포인트: {len(self.waypoints)}")

    def _load_waypoints(self, path):
        waypoints = []
        try:
            with open(path, 'r') as f:
                for x_str, y_str in csv.reader(f):
                    waypoints.append((float(x_str), float(y_str)))
        except Exception as e:
            self.get_logger().error(f"❌ 웨이포인트 로드 실패: {e}")
        return waypoints

    def rotate_all_waypoints(self, angle):
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        rotated = []
        for x, y in self.waypoints:
            x_r = cos_theta * x + sin_theta * y
            y_r = -sin_theta * x + cos_theta * y
            rotated.append((x_r, y_r))
        return rotated

    def _odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.offset_initialized:
            self.start_x = self.curr_x
            self.start_y = self.curr_y
            self.start_yaw = self.current_yaw

            dx = self.waypoints[0][0]
            dy = self.waypoints[0][1]
            target_yaw = math.atan2(dy, dx)

            self.yaw_offset = self.start_yaw - target_yaw
            self.waypoints = self.rotate_all_waypoints(-self.yaw_offset)

            self.offset_initialized = True
            self.get_logger().info(f"🧭 좌표계 회전 적용: offset={math.degrees(self.yaw_offset):.1f}°")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _control_loop(self):
        if not self.offset_initialized or self.curr_x is None:
            return

        rel_x = self.curr_x - self.start_x
        rel_y = self.curr_y - self.start_y

        if self.index >= len(self.waypoints):
            self.pub.publish(Twist())
            return

        tgt_x, tgt_y = self.waypoints[self.index]
        dx = tgt_x - rel_x
        dy = tgt_y - rel_y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        yaw_diff = self.normalize_angle(target_yaw - self.current_yaw)

        twist = Twist()

        if self.state == 'rotate':
            if abs(yaw_diff) > self.yaw_threshold:
                raw = self.Kp_ang * yaw_diff
                speed = max(-self.angular_speed, min(self.angular_speed, raw))
                if abs(speed) < self.min_ang_speed:
                    speed = self.min_ang_speed * (1.0 if speed > 0 else -1.0)
                twist.angular.z = speed
                self.get_logger().info(f"🔄 회전 중: yaw_diff={math.degrees(yaw_diff):.1f}°")
            else:
                self.state = 'forward'
                self.get_logger().info("✅ 회전 완료 → 직진 모드")
        elif self.state == 'forward':
            if distance > self.target_tolerance:
                twist.linear.x = self.linear_speed
                self.get_logger().info(f"➡️ 직진 중: 거리={distance:.2f}m")
            else:
                self.get_logger().info(f"🎯 웨이포인트 {self.index} 도달")
                self.index += 1
                self.state = 'rotate'

        self.pub.publish(twist)

        if self.index >= len(self.waypoints):
            self.get_logger().info("🎉 모든 웨이포인트 완료 → 정지")
            self.pub.publish(Twist())
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

