import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(Twist, '/cmd_vel/follow', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.get_logger().info("🔁 Publishing test Twist")
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPub()
    rclpy.spin(node)
    rclpy.shutdown()
