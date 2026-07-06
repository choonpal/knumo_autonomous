import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import utm
import csv
import os
import sys
import termios
import tty

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('file_name', 'waypoints.csv')
        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.csv_file_path = os.path.expanduser(f'~/.ros/{file_name}')
        if os.path.exists(self.csv_file_path):
            os.remove(self.csv_file_path)
            self.get_logger().info("🗑 이전 웨이포인트 파일 삭제, 새로 시작합니다.")

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        self.start_x = None
        self.start_y = None
        self.latest_fix = None
        self.latest_status = -1

        self.get_logger().info(f"📡 Waypoint Recorder Node started. Saving to {file_name}")
        self.get_logger().info("⌨️  Press 's' to save, 'r' to reset origin, 'q' to quit.")

    def gps_callback(self, msg):
        self.latest_status = msg.status.status  # status 저장
        if msg.status.status == -1:
            return  # 위치 fix 안 되었으면 무시

        try:
            x, y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            self.latest_fix = (x, y)
            if self.start_x is None:
                self.set_start(x, y)
        except Exception as e:
            self.get_logger().warn(f"❌ UTM 변환 실패: {e}")

    def set_start(self, x, y):
        self.start_x = x
        self.start_y = y
        self.get_logger().info(f"📍 기준 위치 설정됨: UTM({x:.2f}, {y:.2f})")

    def save_waypoint(self):
        if self.latest_fix is None or self.start_x is None:
            self.get_logger().warn('⚠️ 위치 정보 없음 또는 기준점 미설정.')
            return

        if self.latest_status == -1:
            self.get_logger().warn('⛔️ 현재 GPS 위치 fix되지 않음 (status = -1), 저장 취소.')
            return

        rel_x = self.latest_fix[0] - self.start_x
        rel_y = self.latest_fix[1] - self.start_y

        try:
            with open(self.csv_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([rel_x, rel_y])
                self.get_logger().info(f"✅ 저장됨: ({rel_x:.2f}, {rel_y:.2f})")
        except Exception as e:
            self.get_logger().error(f"❌ 저장 실패: {e}")

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = get_key()
            if key == 's':
                node.save_waypoint()
            elif key == 'r':
                if node.latest_fix:
                    node.set_start(*node.latest_fix)
                    node.get_logger().info("🔁 기준 위치 초기화 완료.")
                else:
                    node.get_logger().warn("⚠️ 아직 GPS fix 없음.")
            elif key == 'q':
                node.get_logger().info("👋 종료합니다.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

