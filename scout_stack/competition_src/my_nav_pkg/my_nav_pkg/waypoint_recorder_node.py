#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT
import utm
import csv
import os
import shutil
import sys
import termios
import threading
import time
import tty
import math

# 장애물 구간 라벨. local_avoider 는 현재 active 웨이포인트의 *이름*만 보고
# 횡방향 VFH 회피를 켠다(local_avoider_node.py:853 -> speed_profile.py 의
# is_obstacle_waypoint). 그래서 이 접미사는 _OBSTACLE_TOKENS 와 반드시
# 일치해야 하며, 구간 시작/끝만이 아니라 그 사이 모든 점에 붙어야 한다.
OBSTACLE_SUFFIX = '_obstacle'
# 정지선 라벨. follower 가 이 이름을 만나면 그 자리에서
# stopline_hold_sec(기본 3초) 동안 정차한다(speed_profile.py 의
# is_stopline_waypoint). 정차 위치는 이 점을 찍은 위치 그대로이므로
# 흰 선 바로 앞에서 찍어야 한다.
STOPLINE_SUFFIX = '_stopline'
# 후진주차 구간 라벨. parking_controller 는 현재 active 웨이포인트의 *이름*만
# 보고 동작한다(parking_controller_node.py:534 -> speed_profile.py 의
# is_parking_waypoint). 그래서 이 접미사는 _PARKING_TOKENS 와 반드시
# 일치해야 하며, 주차 구역을 지나는 동안의 점에 전부 붙어야 한다.
PARKING_SUFFIX = '_parking'
# 신호등 라벨. mission_control(autodrive_pkg)이 현재 active 웨이포인트의
# *이름*만 보고 신호 구역을 판정한다(mission_core.py 의 SIGNAL 토큰
# ('traffic', 'signal', '신호등')). 구역에 들어가면 /control/state 로
# 정지를 걸고, 초록이 green_confirm_sec 동안 유지되면 푼다.
# speed_profile 의 _VISION_TOKENS 에도 걸려 접근 구간 감속까지 함께 적용된다.
SIGNAL_SUFFIX = '_signal'


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        # 저장할 파일명 파라미터 (기본: waypoints.csv)
        self.declare_parameter('file_name', 'waypoints.csv')
        # True면 RTK Fixed가 아닐 때 's'로 저장이 안 되고 'f'로 강제 저장해야 함
        self.declare_parameter('require_rtk_fixed', True)
        # 직전 저장점과 이 거리(m) 미만이면 중복으로 보고 저장하지 않는다.
        # 0 이하로 두면 거리 검사를 끄고 동일 GPS 메시지만 걸러낸다.
        self.declare_parameter('min_separation_m', 0.05)
        # /fix 가 이보다 오래됐으면 GPS 가 끊긴 것으로 보고 저장하지 않는다.
        # 0 이하면 검사하지 않는다.
        self.declare_parameter('max_fix_age_s', 2.0)

        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.require_rtk_fixed = bool(self.get_parameter('require_rtk_fixed').value)
        self.min_separation_m = float(self.get_parameter('min_separation_m').value)
        self.max_fix_age_s = float(self.get_parameter('max_fix_age_s').value)
        self.csv_file_path = os.path.expanduser(f'~/.ros/{file_name}')

        # 중복 판정용 상태
        self.seq = 0                  # 저장된 웨이포인트 개수
        self.last_saved = None        # (lat, lon)
        self.last_stamp = None        # 마지막으로 저장한 /fix 의 header.stamp
        self._last_move_m = 0.0       # 직전 저장점에서 이동한 거리

        # 시작 시 기존 파일을 지우기 전에 타임스탬프 사본을 남긴다.
        if os.path.exists(self.csv_file_path):
            backup = f"{self.csv_file_path}.{time.strftime('%Y%m%d_%H%M%S')}.bak"
            try:
                shutil.copy2(self.csv_file_path, backup)
                self.get_logger().info(f"🗂 이전 파일 백업: {os.path.basename(backup)}")
            except Exception as e:
                self.get_logger().error(f"❌ 백업 실패, 중단합니다: {e}")
                raise
            os.remove(self.csv_file_path)
            self.get_logger().info("🗑 이전 웨이포인트 파일 삭제, 새로 시작합니다.")

        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
        try:
            with open(self.csv_file_path, 'w', newline='') as f:
                # follower 는 lat,lon,alt,name 을 위치로 읽고 '#' 줄은 건너뛴다.
                # 그래서 헤더는 주석 처리하고 4번째 칸을 name(=번호)으로 둔다.
                f.write('# lat,lon,alt,name,status,utm_zone,easting,northing,'
                        'rtk_status,h_acc_m\n')
        except Exception as e:
            self.get_logger().error(f"❌ 파일 헤더 작성 실패: {e}")

        # GPS 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        # RTK Fixed/Float 판단용 (u-blox NAV-PVT의 carrier phase 플래그)
        self.navpvt_sub = self.create_subscription(
            NavPVT,
            '/navpvt',
            self.navpvt_callback,
            10
        )

        self.latest_fix_msg = None    # NavSatFix 원본 메시지 저장
        self.latest_navpvt_msg = None
        self.get_logger().info(f"📡 Waypoint Recorder Node started. Saving to {file_name}")
        self.get_logger().info(
            "⌨️  's' = RTK Fixed일 때만 저장 / 'f' = 품질 무시하고 강제 저장 / 'q' = 종료 "
            "(※ 기준점/상대좌표는 사용하지 않습니다)"
        )
        self.get_logger().info(
            "🚧 'o' = 장애물 구간 웨이포인트 (이름에 _obstacle 이 붙어 "
            "local_avoider 의 횡방향 회피가 그 구간에서만 켜진다). "
            "구간 시작/끝만이 아니라 장애물을 지나는 동안의 점을 전부 'o' 로 찍을 것."
        )
        self.get_logger().info(
            "🛑 'w' = 정지선 웨이포인트 (이름에 _stopline 이 붙어 그 자리에서 "
            "3초 정차한다). 흰 선 바로 앞에서 딱 한 점만 찍을 것 - 연속으로 찍으면 "
            "점마다 3초씩 선다.  ※ 'O'/'W' = 각각 품질 무시 강제 저장."
        )
        self.get_logger().info(
            "🅿️  'p' = 후진주차 구간 웨이포인트 (이름에 _parking 이 붙어 "
            "parking_controller 가 그 구간에서만 켜진다). 주차 구역을 지나는 "
            "동안의 점을 전부 'p' 로 찍을 것.  ※ 'P' = 품질 무시 강제 저장."
        )
        self.get_logger().info(
            "🚦 't' = 신호등 웨이포인트 (이름에 _signal 이 붙어 그 자리에서 "
            "정차하고, 카메라가 초록을 확인할 때까지 기다렸다가 출발한다). "
            "정지선 위치에 찍을 것.  ※ 'T' = 품질 무시 강제 저장."
        )
        self.get_logger().info(
            f"🔢 저장 순서대로 wp001, wp002 ... 로 이름이 붙습니다 "
            f"(4번째 칸 = follower 의 name).  "
            f"중복 방지: 같은 GPS 측정이거나 직전에서 "
            f"{self.min_separation_m * 100:.0f} cm 미만 이동이면 저장하지 않습니다."
        )

    def gps_callback(self, msg: NavSatFix):
        self.latest_fix_msg = msg

    def navpvt_callback(self, msg: NavPVT):
        self.latest_navpvt_msg = msg

    def _rtk_state(self):
        """(carrier_phase_code, 'FIXED'/'FLOAT'/'NONE'/'UNKNOWN', h_acc_m) 반환"""
        if self.latest_navpvt_msg is None:
            return None, 'UNKNOWN', None
        m = self.latest_navpvt_msg
        carr = m.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK
        if carr == NavPVT.CARRIER_PHASE_FIXED:
            label = 'FIXED'
        elif carr == NavPVT.CARRIER_PHASE_FLOAT:
            label = 'FLOAT'
        else:
            label = 'NONE'
        return carr, label, m.h_acc / 1000.0

    def save_waypoint(self, force=False, suffix='', force_key='f'):
        if self.latest_fix_msg is None:
            self.get_logger().warn('⚠️ 최신 GPS 메시지가 아직 없습니다.')
            return

        status = self.latest_fix_msg.status.status
        if status == -1:
            self.get_logger().warn('⛔️ 현재 GPS 위치 fix되지 않음 (status = -1), 저장 취소.')
            return

        # GPS 가 끊긴 채로 마지막 값을 계속 저장하는 것을 막는다.
        stamp = self.latest_fix_msg.header.stamp
        age_s = (self.get_clock().now().nanoseconds
                 - (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9
        if self.max_fix_age_s > 0.0 and age_s > self.max_fix_age_s:
            self.get_logger().warn(
                f"⛔️ 마지막 /fix 가 {age_s:.1f} 초 전 값입니다 "
                f"(한계 {self.max_fix_age_s:.1f} 초). GPS 수신이 끊겼는지 확인하세요. "
                f"저장하지 않습니다."
            )
            return

        carr, rtk_label, h_acc_m = self._rtk_state()
        is_fixed = (carr == NavPVT.CARRIER_PHASE_FIXED)
        acc_str = f"{h_acc_m:.2f}m" if h_acc_m is not None else "알수없음"

        if self.require_rtk_fixed and not is_fixed and not force:
            self.get_logger().warn(
                f"🟡 RTK {rtk_label} 상태입니다 (정확도 ≈{acc_str}). 이대로 저장하면 오차가 클 수 있어요. "
                f"그래도 저장하려면 '{force_key}'를 누르세요."
            )
            return

        lat = self.latest_fix_msg.latitude
        lon = self.latest_fix_msg.longitude
        alt = self.latest_fix_msg.altitude
        alt_out = "" if (alt is None or (isinstance(alt, float) and math.isnan(alt))) else alt

        # --- 중복 검사 -------------------------------------------------
        # 1) 같은 /fix 메시지를 두 번 저장하는 경우. GPS 는 4~5 Hz 라
        #    키를 연달아 누르면 새 측정 없이 같은 값이 다시 들어온다.
        stamp_key = (stamp.sec, stamp.nanosec)
        if self.last_stamp is not None and stamp_key == self.last_stamp:
            self.get_logger().warn(
                f"🔁 [{self.seq:03d}]번과 같은 GPS 측정입니다 (새 측정이 아직 안 왔음). "
                f"저장하지 않습니다. 잠시 후 다시 누르세요."
            )
            return

        # 2) 위치가 사실상 그대로인 경우.
        if self.last_saved is not None:
            self._last_move_m = self._haversine_m(
                self.last_saved[0], self.last_saved[1], lat, lon
            )
            if (self.min_separation_m > 0.0
                    and self._last_move_m < self.min_separation_m):
                self.get_logger().warn(
                    f"🔁 직전 [{self.seq:03d}]번에서 {self._last_move_m * 100:.1f} cm "
                    f"밖에 안 움직였습니다 (기준 {self.min_separation_m * 100:.0f} cm). "
                    f"중복으로 보고 저장하지 않습니다."
                )
                return
        # ---------------------------------------------------------------

        try:
            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
            zone_str = f"{zone_number}{zone_letter}"
            h_acc_str = f"{h_acc_m:.3f}" if h_acc_m is not None else ""
            name = f"wp{self.seq + 1:03d}{suffix}"
            with open(self.csv_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([lat, lon, alt_out, name, status, zone_str,
                                 f"{easting:.2f}", f"{northing:.2f}",
                                 rtk_label, h_acc_str])
        except Exception as e:
            self.get_logger().error(f"❌ 저장 실패(UTM 변환/파일): {e}")
            return

        # 파일에 실제로 쓴 뒤에만 상태를 갱신한다.
        self.seq += 1
        self.last_saved = (lat, lon)
        self.last_stamp = stamp_key
        mark = "🟢" if is_fixed else "🟡(강제저장)"
        moved = ""
        if self.seq > 1:
            moved = f", 직전에서 {self._last_move_m:.2f} m"
        self.get_logger().info(
            f"✅ {mark} [{self.seq:03d}] {name} 저장됨: lat={lat:.8f}, lon={lon:.8f}, "
            f"RTK={rtk_label}, acc≈{acc_str}, UTM={zone_str} "
            f"({easting:.2f}, {northing:.2f}){moved}"
        )

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        r = 6371000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = (math.sin(dp / 2) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
        return 2 * r * math.asin(min(1.0, math.sqrt(a)))

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

    # 콜백은 별도 스레드에서 쉬지 않고 돌린다.
    #
    # 예전에는 `spin_once()` 한 번 -> `get_key()` 블록 을 반복했다. 키를 누를
    # 때까지 콜백이 전혀 실행되지 않고, 키 하나당 콜백이 딱 하나만 처리됐다.
    # 구독이 /fix 와 /navpvt 둘이라 번갈아 실행되는 바람에, 두 번에 한 번은
    # /fix 가 갱신되지 않은 채로 저장을 시도해 '같은 GPS 측정' 으로 거부됐다.
    # (초당 1 m 로 움직이는 가짜 GPS 로 재현 확인: 8회 중 3회 오거부)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = get_key()
            if key == 's':
                node.save_waypoint(force=False)
            elif key == 'f':
                node.save_waypoint(force=True)
            elif key == 'o':
                node.save_waypoint(force=False, suffix=OBSTACLE_SUFFIX, force_key='O')
            elif key == 'O':
                node.save_waypoint(force=True, suffix=OBSTACLE_SUFFIX, force_key='O')
            elif key == 'w':
                node.save_waypoint(force=False, suffix=STOPLINE_SUFFIX, force_key='W')
            elif key == 'W':
                node.save_waypoint(force=True, suffix=STOPLINE_SUFFIX, force_key='W')
            elif key == 'p':
                node.save_waypoint(force=False, suffix=PARKING_SUFFIX, force_key='P')
            elif key == 'P':
                node.save_waypoint(force=True, suffix=PARKING_SUFFIX, force_key='P')
            elif key == 't':
                node.save_waypoint(force=False, suffix=SIGNAL_SUFFIX, force_key='T')
            elif key == 'T':
                node.save_waypoint(force=True, suffix=SIGNAL_SUFFIX, force_key='T')
            elif key in ('q', '\x03'):  # 'q' 또는 Ctrl+C(raw 모드라 SIGINT로 안 잡히고 문자로 들어옴)
                node.get_logger().info("👋 종료합니다.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
