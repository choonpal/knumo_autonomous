import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # follow/avoid 토픽과 avoid_active 플래그 구독
        self.create_subscription(Twist, '/cmd_vel/follow', self.follow_cb, 10)
        self.create_subscription(Twist, '/cmd_vel/avoid',  self.avoid_cb,  10)
        self.create_subscription(Bool,  '/avoid_active', self.flag_cb,   10)

        self.latest_follow = Twist()
        self.latest_avoid  = Twist()
        self.avoid_active  = False

        self.timer = self.create_timer(0.1, self.timer_cb)

    def follow_cb(self, msg: Twist):
        self.latest_follow = msg

    def avoid_cb(self, msg: Twist):
        self.latest_avoid = msg

    def flag_cb(self, msg: Bool):
        self.avoid_active = msg.data

    def timer_cb(self):
        # 1) 정지 플래그 우선
        if self.avoid_active:
            output = Twist()  # 완전 정지
            self.get_logger().info("🔁 장애물 정지 모드 적용")
        # 2) 회피 명령(회전/후진) 우선
        elif abs(self.latest_avoid.angular.z) > 0.01 or abs(self.latest_avoid.linear.x) > 0.01:
            output = self.latest_avoid
            self.get_logger().info("🔁 회피 명령 우선 적용")
        # 3) 그 외는 웨이포인트 주행
        else:
            output = self.latest_follow
            self.get_logger().info("➡️ 주행 명령 적용")

        # 최종 퍼블리시
        self.pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

