import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Bool
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.pub      = self.create_publisher(Twist, '/cmd_vel/avoid', 10)
        self.flag_pub = self.create_publisher(Bool,  '/avoid_active', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.front_angle    = math.radians(30.0)
        self.stop_threshold = 0.3

    def lidar_callback(self, scan: LaserScan):
        # 앞쪽 ±30° 범위 필터
        angles = [scan.angle_min + i*scan.angle_increment for i in range(len(scan.ranges))]
        front_ranges = [r for r,a in zip(scan.ranges, angles)
                        if not math.isinf(r) and not math.isnan(r) and abs(a)<=self.front_angle]

        if not front_ranges:
            return

        min_dist = min(front_ranges)
        twist    = Twist()
        flag     = Bool()

        if min_dist < self.stop_threshold:
            # 0.3m 이내 → 정지 플래그=True, /cmd_vel/avoid에는 정지 twist 전송
            flag.data = True
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn(f"🚧 장애물 {min_dist:.2f}m 이내 → 정지")
        else:
            # 안전 거리 → 정지 플래그=False, /cmd_vel/avoid는 (0,0)도 OK
            flag.data = False
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"✅ 안전 거리({min_dist:.2f}m) 확보")

        self.pub.publish(twist)
        self.flag_pub.publish(flag)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

