#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import utm
import csv
import os
import sys
import termios
import tty
import math

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        # 저장할 파일명 파라미터 (기본: waypoints.csv)
        self.declare_parameter('file_name', 'waypoints.csv')
        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.csv_file_path = os.path.expanduser(f'~/.ros/{file_name}')

        # 시작 시 기존 파일 제거 후 헤더 생성
        if os.path.exists(self.csv_file_path):
            os.remove(self.csv_file_path)
            self.get_logger().info("🗑 이전 웨이포인트 파일 삭제, 새로 시작합니다.")

        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        try:
            with open(self.csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['lat', 'lon', 'alt', 'status', 'utm_zone', 'easting', 'northing'])
        except Exception as e:
            self.get_logger().error(f"❌ 파일 헤더 작성 실패: {e}")

        # GPS 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        self.latest_fix_msg = None  # NavSatFix 원본 메시지 저장
        self.get_logger().info(f"📡 Waypoint Recorder Node started. Saving to {file_name}")
        self.get_logger().info("⌨️  Press 's' to save, 'q' to quit. (※ 기준점/상대좌표는 사용하지 않습니다)")

    def gps_callback(self, msg: NavSatFix):
        self.latest_fix_msg = msg

    def save_waypoint(self):
        if self.latest_fix_msg is None:
            self.get_logger().warn('⚠️ 최신 GPS 메시지가 아직 없습니다.')
            return

        status = self.latest_fix_msg.status.status
        if status == -1:
            self.get_logger().warn('⛔️ 현재 GPS 위치 fix되지 않음 (status = -1), 저장 취소.')
            return

        lat = self.latest_fix_msg.latitude
        lon = self.latest_fix_msg.longitude
        alt = self.latest_fix_msg.altitude
        alt_out = "" if (alt is None or (isinstance(alt, float) and math.isnan(alt))) else alt

        try:
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            zone_str = f"{zone_number}{zone_letter}"
            with open(self.csv_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([lat, lon, alt_out, status, zone_str, f"{easting:.2f}", f"{northing:.2f}"])
            self.get_logger().info(
                f"✅ 저장됨: lat={lat:.8f}, lon={lon:.8f}, UTM={zone_str} ({easting:.2f}, {northing:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f"❌ 저장 실패(UTM 변환/파일): {e}")

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
            elif key == 'q':
                node.get_logger().info("👋 종료합니다.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
