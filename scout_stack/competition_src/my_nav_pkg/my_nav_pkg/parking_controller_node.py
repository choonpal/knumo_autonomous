"""
LiDAR 기반 후진주차 제어 노드 (v7, 차동구동 AMR - 차선 주행 + 편측 갭 탐지 + 후방
사각지대 대응).

실제 시나리오: 로봇은 주차구역 앞 차선(아일)을 따라 똑바로 직진하다가, 이미 주차된
차량(장애물) 사이의 빈자리를 찾으면 그 중앙에 멈추고, 개활지에서 제자리 회전으로
자세를 슬롯에 대해 수직으로 돌린 뒤, 직진으로 후진해 들어간다. 자동차식 전륜 조향이
아니라 "제자리 회전이 되는 AMR"(차동구동)이므로 Ackermann 자전거 모델 없이
geometry_msgs/Twist(linear.x, angular.z)를 직접 발행해 Gazebo DiffDrive 시스템
플러그인에 명령한다.

통합 차량/라이다 규약:
- local_avoider와 동일하게 base_link 기준 lidar_offset_x=0.60,
  vehicle_length=1.40m를 쓴다. 스캔 범위는 전방+좌/우 약 200도이며 정후방은
  직접 관측하지 못한다.
- 따라서 "뒤쪽 장애물까지 남은 거리"를 라이다로 재서 후진을 멈추는 방식은 쓸 수
  없다. 대신 슬롯 규격(치수는 이미 알려진 상수: 후진주차장.png 기준 입구 y=0 ~
  안쪽 정지선 y=1.6)과 통합 차체 길이(1.40m)로 "목표
  정차 위치"를 미리 계산해 두고, 오도메트리로 그 위치에 도달했는지만 확인한다
  (= 뒤를 못 보는 대신 "얼마나 왔는지"로 판단).
- 좌/우 라이다 거리는 SEARCH/BACK_TO_CENTER/RETURN_SCAN(회전 전, 아직 헤딩이
  차선 방향) 동안 "옆 장애물과의 안전거리 유지(row-hold)"에 쓴다: 헤딩(각도)만
  잡으면 몇 미터를 주행하는 동안 미세한 드리프트가 누적돼도 못 잡기 때문이다.
  회전 후 STRAIGHT_REVERSE에서는 좌우 거리차 기반 실시간 정렬을 시도했었지만
  결과가 더 나빠서 뺐다(아래 STRAIGHT_REVERSE 항목 참고) - 대신 회전 전 위치
  자체를 정확히 맞추는 쪽(RETURN_SCAN)으로 해결했다.

주차 완료 조건: 차체 전체(앞범퍼~뒷범퍼)가 슬롯(y=[0, 1.6]) 안에 들어와야 한다.
슬롯 깊이 1.6m에서 차체 길이 1.40m를 빼면 총 여유가 0.20m다. 이를 앞뒤
0.10m씩 나누기 위해 차체 중심을 슬롯 중앙(y=0.8)에 맞추는 것을 목표 정차
위치로 삼는다.

상태머신:
- `SEARCH` (차선 직진): 직진 헤딩 + 옆 장애물과의 목표거리(row-hold)를 함께 유지하며
  로봇 왼쪽(정측면, 90도) LiDAR 섹터 하나만으로 "가깝다(장애물 옆)/멀다(빈 공간)"를
  판정한다. 이 판정을 BEFORE_BOX1 -> AT_BOX1 -> IN_GAP 순으로 추적하다가, IN_GAP
  상태에서 다시 "가깝다"(다음 장애물, box2)로 바뀌는 순간 갭의 진입/이탈 지점을
  기록하고 `RETURN_SCAN`으로 전환한다.
- `RETURN_SCAN` (반대 방향 재측정): SEARCH의 측정은 항상 전진(+s) 중에만
  이뤄져서, 디바운스 지연이나 라이다 섹터가 모서리를 비스듬히 붙잡는 정도 같은
  "이동 방향에 따라 달라지는" 오차가 상쇄되지 않고 그대로 slot_center_s에 남는다.
  box2를 지나친 지점에서 살짝만 후진해 "이격"을 반대 방향(-s)에서 다시 재면
  충분하다 - box1까지 다시 갈 필요는 없다:
    gap_entry_s = 실제 box1 경계 + 편향   (전진 중 "이격" 전이, +s 방향 지연)
    return_exit_s = 실제 box2 경계 - 편향  (후진 중 "이격" 전이, -s 방향 지연)
  둘 다 같은 종류의 전이(근접->이격)라 편향 크기가 같고 부호만 반대이므로,
  slot_center = (gap_entry_pos + return_exit_pos) / 2 로 계산하면 편향 항이
  대수적으로 정확히 상쇄되어 box1/box2 각각의 실제 경계 평균이 그대로 나온다.
  재측정이 끝나면 `BACK_TO_CENTER`로 전환한다(실패 시 SEARCH의 편측 값으로 폴백).
- `BACK_TO_CENTER` (슬롯 중심 복귀): 기록해둔 슬롯 중심 진행거리 s까지 헤딩 + row-hold를
  유지한 채 이동해 되돌아온다 - 이 row-hold가 곧 "제자리 회전 전 위치 조정" 과정이다
  (헤딩만 잡으면 SEARCH부터 누적된 횡방향 드리프트가 그대로 남아 회전/후진이
  삐뚤어짐). box1/box2 감지는 차체 중심이 아니라 그보다 앞(LIDAR_OFFSET_X) 달린
  라이다 위치에서 일어나므로, 감지 시점의 차체 위치가 목표(slot_center_s)보다 이미
  넘어가 있을 수도(후진 필요), 못 미쳐 있을 수도(전진 필요) 있다 - 그래서 방향을
  후진으로 고정하지 않고 전환 시점에 계산한다(self.back_direction).
- `ROTATE_IN_PLACE` (제자리 회전): 반드시 장애물 사이로 들어가기 **전**, 아직 차선
  위(개활지)에 있을 때 끝내야 한다. 통합 차량 envelope를 고려하면 제자리 회전은
  시 대각선 반경이 약 0.8m인데, 좌우 장애물 사이 빈 폭은 1.2m뿐이라 그 사이에 낀
  채로 돌면 반드시 스친다. 스폰 시 차선 y옵셋(-0.6m)을 충분히 둔 것도 이 회전
  반경을 위한 여유다. 진입 시 경로 헤딩에서 슬롯 쪽으로 상대 90도 회전한다.
  실제로는 완전한 점 회전이 아니라 바퀴가 살짝 미끄러지며(스크럽) 회전 중 차체
  중심이 진행방향으로 밀릴 수 있다. 다른 차량에서 얻은 고정 피드포워드는 적용하지
  않고 통합 설정의 rotate_scrub_drift(기본 0.0)로만 명시적으로 보정한다.
- `STRAIGHT_REVERSE` (직진 후진): 회전이 끝나면 상대 목표 헤딩을 유지하며 직진
  후진한다. 정지는 후방 감지가 아니라 "차체 중심이 목표 정차 위치(슬롯 중앙,
  오도메트리 기준)에 도달했는가"로 판단한다. 회전 시작 위치 자체가 RETURN_SCAN +
  ROTATE_SCRUB_DRIFT 보정으로 이미 정확하므로, 후진 중 추가 정렬 없이도 최종
  좌우 간극이 거의 대칭으로 나온다(실측: 오차 약 1cm 수준).
  후방이 물리적으로 안 보이므로 정지 판정은 회전 종료 뒤 오도메트리 이동량으로
  한다. 라이다로 계산한 목표 후진거리 도달이 정상 조건이고, 그 계산값이 비정상적으로
  커져도 MAX_REVERSE_TRAVEL에서 끝내는 주행거리 상한을 별도로 둔다.
- 최상위 긴급회피: 대응이 상태에 따라 갈린다.
  * 차선 직진 중(SEARCH): 가장 가까운 장애물의 방위 반대쪽으로 단순 조향(후퇴
    없이 전진 유지 + 각속도만으로 회피). 앞이 열려 있는 구간이라 유효하다.
  * 그 외 정밀 기동 중(RETURN_SCAN/BACK_TO_CENTER/ROTATE_IN_PLACE/
    STRAIGHT_REVERSE): 즉시 정지. 슬롯에 절반 들어간 상태에서 전진 탈출을 하면
    슬롯 밖으로 튀어나오며 옆 차를 긁고, 후진 중에는 각속도 부호의 의미가
    뒤집혀(차체 뒤쪽이 반대로 스윙) 오히려 장애물 쪽으로 붙는다.
  어느 쪽이든 이 회피는 라이다가 보는 전방+좌우 200도 범위 안에서만 동작한다
  (정후방 160도는 애초에 감지 대상이 아님).

장애물 배치 (worlds/parking_lot.sdf 기준):
  - 슬롯 좌/우 옆 칸 2개 : SEARCH/RETURN_SCAN의 편측 갭 탐지 대상(box1 -> 갭 -> box2).
  - 슬롯 자체(y=[0, 1.6])는 완전히 비어 있다. 보도블록 쪽에 있던 장애물은 슬롯
    경계(정지선, y=1.6) 밖 - 정지선에서 200mm 더 간 뒤에야 있으므로 정상 주차
    동작에는 관여하지 않는다.

좌표계/센서 사용 원칙:
- 통합 실행에서는 waypoint_follower가 실제 제어에 쓴 `/follower/control_pose`의
  x/y/yaw를 그대로 사용한다. pose_topic을 비운 단독 실행에서만
  `/odometry/global` 위치와 보정된 `/imu/heading_enu` 자세를 쓴다.
- 절대 map x축을 코스 진행축으로 가정하지 않는다. 주차 진입 시점의 follower
  헤딩으로 위치를 투영한 진행거리 s를 사용한다. 지금은
    * 슬롯 중심 s  : 갭 진입/이탈 위치를 경로 진행축에 투영한 값의 평균
    * 후진 정지    : 회전 종료 지점 기준 상대 후진거리(_plan_reverse)
  로 모두 상대량이며, 슬롯 기준 위치가 필요한 곳은 라이다로 옆 차 면까지의
  거리를 재서 역산한다(_slot_frame_y).
- 상태 전이의 1차 판단 기준은 항상 LiDAR다.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, String

from .speed_profile import is_parking_waypoint

# ==========================================================================
# 튜닝 상수 (실제 Gazebo에서 여러 번 구동/계측하며 조정함 - 최종 확인된 정밀도:
# 회전 종료 시 요(yaw) 오차 0도, 최종 좌우 중심 오차 약 1cm)
# ==========================================================================
SPEED_SEARCH = 0.15          # follower 주차 미션 상한과 같은 SEARCH 속도 (m/s)
SPEED_BACK_TO_CENTER = -0.15 # BACK_TO_CENTER 속도 크기(부호는 self.back_direction이
                             # 정함, abs()로 사용) - 슬롯 중심으로 복귀
SPEED_STRAIGHT = -0.15       # STRAIGHT_REVERSE 상태 후진 속도
SPEED_ESCAPE = 0.15          # 긴급 회피 시 전진 속도

# --- BACK_TO_CENTER 최종 접근 감속 ---
# BACK_TO_CENTER는 "목표를 지났는가"라는 임계값 통과로 정지한다. 접근 속도가
# 0.20m/s면 제어주기 0.1s당 2cm를 이동하므로 정지 위치가 2cm 단위로 양자화되고,
# 그 양자화가 그대로 최종 좌우 정렬 오차로 남는다(실측: 10회 중 최종 x가 한
# 지점에 뭉치다가 딱 2cm 떨어진 곳에 찍히는 패턴이 반복됨 - 한 틱 이동거리와
# 정확히 일치). 목표 근처에서 속도를 줄이면 틱당 이동거리가 줄어 양자화도
# 비례해서 작아진다: 0.04m/s면 틱당 0.4cm.
BACK_TO_CENTER_SLOW_DIST = 0.15   # 목표까지 이 거리 안으로 들어오면 감속 시작(m)
BACK_TO_CENTER_SLOW_SPEED = 0.04  # 감속 구간 속도 크기(m/s) - 틱당 0.4cm

ROTATE_SPEED = 0.6          # ROTATE_IN_PLACE 최대 각속도 (rad/s) - 0.25로 늦춰봤더니
                             # 회전 스크럽 드리프트(ROTATE_SCRUB_DRIFT 참고)가 오히려
                             # 더 커져서(+0.05m, 원래보다 나쁨) 되돌림 - 느리게 돈다고
                             # 슬립이 줄어들지 않고, 오히려 회전에 걸리는 총 시간이
                             # 길어질수록 누적 드리프트가 커지는 쪽으로 보인다.
K_ROTATE = 1.5               # 회전 목표각 오차 -> 각속도 비례 게인 (위와 동일 이유로 되돌림)
K_HEADING_HOLD = 1.0         # SEARCH/BACK_TO_CENTER/STRAIGHT_REVERSE 중 직진 유지용 헤딩 P게인

# --- 최상위 긴급회피 ---
# 라이다 FOV가 전방+좌우 200도뿐이라, 이 회피도 그 범위 안의 장애물에만 반응한다.
EMERGENCY_DIST = 0.12      # 전방위 어디서든 이 거리 안으로 들어오면 최우선 회피
# 긴급회피로 "전진 탈출"을 허용하는 상태. 차선(아일)을 직진 중이라 앞이 열려 있는
# SEARCH에서만 유효하다.
#
# 그 외 상태(RETURN_SCAN/BACK_TO_CENTER/ROTATE_IN_PLACE/STRAIGHT_REVERSE)에서
# 전진 탈출을 그대로 쓰면 위험하다:
#  - STRAIGHT_REVERSE는 슬롯에 절반쯤 들어간 상태라, 옆 박스가 EMERGENCY_DIST
#    안에 잡히는 순간 차가 SPEED_ESCAPE로 슬롯 밖으로 튀어나오면서
#    옆 차를 긁는다.
#  - 후진 중에는 angular.z 부호의 의미도 뒤집힌다. 같은 각속도라도 차체 뒤쪽은
#    반대로 스윙하므로 "가장 가까운 장애물 반대쪽으로 돈다"는 의도가 정반대로
#    작동한다.
# 좁은 슬롯 안/옆에서의 정밀 기동 중에는 탈출 기동보다 즉시 정지가 안전하다.
EMERGENCY_FORWARD_ESCAPE_STATES = ('SEARCH',)

# --- 센서 신선도 (끊긴 센서의 마지막 값을 계속 쓰지 않기 위한 최소 방어) ---
# IMU는 50Hz, LiDAR는 10Hz로 발행된다. 값이 이 시간보다 오래됐으면 "살아있지
# 않다"고 보고, IMU는 odom으로 폴백하며 LiDAR는 즉시 정지한다.
# 특히 IMU는 회전/후진 정확도의 기준이라(odom yaw는 슬립 때문에 7~9도 틀림)
# 끊긴 걸 모르고 옛 자세로 계속 제어하면 그대로 오차가 된다.
IMU_STALE_SEC = 0.30
SCAN_STALE_SEC = 0.50

# --- 명령은 내는데 실제로 안 움직이는 상태(stall) 감지 ---
# 이 노드는 후진 명령을 /cmd_vel/parking으로 낼 뿐, 실제 모터까지 갈지는
# 최종 selector(main_controller_node)의 parking_reverse_cap이 정한다. 그 값이
# 0.0이면 후진 명령이 조용히 0으로 잘리는데, 이 노드의 종료조건이 전부
# "얼마나 이동했는가" 기준이라 주행거리가 안 늘어 어떤 조건도 성립하지 않고
# 그 상태로 영구히 멈춘다. 로그도 안 남아 원인 파악이 어렵다.
# 그래서 "의미 있는 속도를 명령했는데 실제 변위가 없음"을 직접 감지해
# 명시적으로 FAILED로 떨어뜨린다.
STALL_CHECK_SEC = 2.0       # 이 시간 동안
STALL_MIN_TRAVEL = 0.02     # 이 거리도 못 움직였으면 stall (0.15m/s면 30cm 가야 정상)
STALL_MIN_COMMAND = 0.02    # 이 크기 이상을 명령했을 때만 판정 대상

# --- 좌측 정측면 섹터 : 라이다 FOV가 -100~+100도(200도)라, 90도 정중앙 +/-5도 =
#     85~95도로 FOV 안에 들어오긴 하지만 경계(100도)까지 여유가 5도뿐이다(예전
#     250도 FOV일 땐 15도 여유, 그보다 전엔 180도라 90도가 경계값이어서 75도로
#     물러나야 했음). 라이다 장착각 오차나 차체 롤이 5도를 넘으면 섹터 끝이
#     잘리므로 실차 이식 시 실측 재확인 필요. SEARCH/RETURN_SCAN의 편측 갭 탐지,
#     row-hold(횡방향 위치 유지) 모두 이 섹터를 공용으로 쓴다. ---
SIDE_SECTOR_HALF_WIDTH = math.radians(5.0)  # 좁을수록 정확 - 섹터가 넓으면(과거 15도)
                             # 장애물 모서리를 실제 경계를 지난 뒤에도(box1) 또는 도달
                             # 하기 전에도(box2) 비스듬한 각도로 계속 붙잡아, 갭 진입/이탈
                             # 감지가 진행방향 기준 각각 다르게(비대칭으로) 밀린다(실측:
                             # box1 이탈 +0.30m 지연, box2 진입 -0.21m 조기 감지 -> 평균
                             # slot_center_s가 진행방향으로 약 4.5cm 밀림). 5도로 좁히면
                             # 이 비스듬한 모서리 포착 여유가 tan(5)/tan(15)=0.32배로
                             # 줄어 편향도 비례해서 작아진다.
LEFT_SECTOR_CENTER = math.pi / 2.0          # 로봇 좌측 정측면(장애물 행이 있는 쪽) - 90도가
                                             # 아닌 다른 각을 중심으로 쓰면 갭 진입/이탈 감지
                                             # 위치가 기하학적으로 밀려서(전방으로 치우쳐서)
                                             # slot_center_s가 크게 틀어지는 버그가 생긴다
                                             # (실측으로 확인: 75도 중심 사용 시 실제 중심
                                             # 0.0m 대신 1.20m로 계산됨).
ROW_NEAR_DIST = 1.2        # 이 값(m)보다 가까우면 "장애물 옆", 멀면 "빈 공간"으로 판정
                            # (스폰 y옵셋 -0.6m 기준: 장애물 옆 ~0.75m, 갭 구간 ~1.15~2.2m
                            # 이므로 그 사이인 1.0m을 경계로 삼음 - 차선 y옵셋을 바꾸면
                            # 이 값도 같이 재계산해야 함)
ROW_DEBOUNCE_COUNT = 3      # 이 횟수(제어주기)만큼 연속으로 같은 판정이 나와야 확정
                            # (라이다 노이즈로 인한 순간적 오판정 필터링)

# --- SEARCH/BACK_TO_CENTER 중 옆 장애물과의 거리 유지(row-hold) : 헤딩만 잡으면
#     회전 전까지 몇 미터를 주행하는 동안 횡방향 드리프트가 누적되어 회전/후진이
#     삐뚤어지므로, 편측 라이다 거리를 목표값에 붙잡아 실제 위치도 함께 보정한다 ---
# 허용 옆거리 밴드. 이 안에 있으면 아무 보정도 하지 않는다.
#   row_dist < ROW_HOLD_MIN  -> 너무 붙었다, 밀어낸다
#   row_dist > ROW_HOLD_MAX  -> 너무 멀다, 끌어당긴다 (단 ROW_NEAR_DIST 안에서만)
#   그 사이                   -> 무보정
# 예전에는 단일 임계값(0.75)에서 "가까우면 밀어내기만" 하는 편측 보정이었다.
# 그러면 0.75보다 가깝게 찍힌 경로는 실효 옆거리가 결국 0.75로 밀려나고,
# 그 과정에서 차가 비스듬히 달린다(0.5m 에서 8.6도). 밴드로 바꾸면 0.6~1.0
# 어디에 찍혀도 그대로 유지된다.
#
# 끌어당김을 ROW_NEAR_DIST 안으로 제한하는 게 중요하다. 갭 구간에서는 측면이
# 빈 공간(또는 안쪽 박스까지 2.4m)이라, 제한이 없으면 갭에서 차를 옆으로
# 끌어당겨 통과선이 무너진다.
ROW_HOLD_MIN = 0.6
ROW_HOLD_MAX = 1.0
# 예전 단일 임계값. 지금은 쓰지 않지만, 이 키가 들어 있는 기존 yaml 이
# 그대로 로드되도록 선언만 남긴다(선언 안 된 파라미터가 오면 노드가 죽는다).
ROW_HOLD_DIST = 0.75

K_ROW_HOLD = 0.6            # 목표거리 오차 -> 각속도 보정 비례 게인
MAX_ROW_CORRECTION = 0.25   # 위 보정 각속도 상한(rad/s) - 헤딩 유지 항과 합산되므로 제한

ANGLE_TOL = math.radians(0.3)  # 예전엔 3도라 STRAIGHT_REVERSE 시작 시점에 잔여 각오차가
                             # 남아있었고, 그게 후진 중 옆으로 밀리는 변위(약 3.5cm)로
                             # 누적되는 원인 중 하나로 추정됨 - 훨씬 좁혀서 회전을 더
                             # 정밀하게 끝낸다.

# --- 슬롯/차체 규격 상수 (후진주차장.png, 차량규격.png 기준 - 센서로 재는 값이
#     아니라 이미 알려진 환경/설계 상수) ---
LIDAR_OFFSET_X = 0.60       # local_avoider의 laser_x_offset과 같은 base_link 기준값.
                             # box1/box2 감지는 "차체 중심"이 아니라 "라이다"가 그
                             # 위치에 왔을 때 일어나므로, gap_entry/gap_exit에 기록되는
                             # self.x(차체 중심)는 실제 물리적 경계보다 항상 이만큼
                             # (라이다가 차체보다 앞서 있는 만큼) 덜 온 값이다. 이 보정을
                             # 빼먹으면 entry/exit 평균(slot_center_s)이 실제 갭 중심에서
                             # 이 오프셋만큼 그대로 밀려서 나온다.
CONTROL_PERIOD = 0.1        # 10 Hz
ROTATE_SCRUB_DRIFT = 0.0    # 다른 parking_bot에서 측정한 피드포워드는 Scout에
                             # 이식하지 않는다. 필요할 때 통합 설정에서만 조정한다.
SLOT_ENTRY_Y = 0.0          # 슬롯 입구 (world/슬롯 좌표계)
# 옆 칸 주차 차량의 "슬롯 입구쪽 면" y좌표(후진주차장.png: 입구에서 100mm 띄움).
# 이건 스폰 위치 같은 실행 환경 값이 아니라 코스 도면에서 오는 기하 상수다.
# SEARCH 중 좌측 90도 섹터가 재는 거리가 바로 "로봇 중심 ~ 이 면"까지의 수직
# 거리이므로, 로봇의 슬롯좌표 y = SIDE_BOX_FRONT_Y - (좌측 섹터 거리)로 역산된다.
# 덕분에 odom 원점이 어디든(실차에서는 부팅 지점) 무관하게 슬롯 기준 위치를
# 알 수 있다 - 아래 _slot_frame_y() 참고.
SIDE_BOX_FRONT_Y = 0.10
SLOT_BACK_BOUNDARY_Y = 1.6  # 슬롯 안쪽 경계(정지선) - 이 너머는 슬롯 밖
TARGET_CENTER_WORLD_Y = (SLOT_ENTRY_Y + SLOT_BACK_BOUNDARY_Y) / 2.0  # 0.8m
CENTER_Y_TOL = 0.03         # 목표 정차 위치 도달 허용 오차(m)
VEHICLE_LENGTH = 1.40       # local_avoider가 쓰는 동일 차량 외곽 길이
# --- 안전 폴백 (LiDAR/오도메트리가 예상과 다를 때를 대비한 최후 방어선, 1차 기준 아님) ---
MAX_SEARCH_TRAVEL = 6.0     # 이만큼 이동해도 갭(box1->빈공간->box2)을 다 못 찾으면
                             # SEARCH 중단(FAILED)
MAX_RETURN_SCAN_TRAVEL = 0.5  # RETURN_SCAN(box2에서 살짝 후진)이 이만큼 이동해도
                             # "이격"을 못 찾으면 SEARCH의 편측 측정값만으로 폴백
                             # (정상 상황에서는 디바운스 지연 정도인 몇 cm면 충분함)
MAX_BACK_TRAVEL = 2.0       # BACK_TO_CENTER 진입 후 이만큼 이동해도 슬롯 중심에
                             # 못 이르면 그 자리에서 회전으로 넘어감(안전 폴백)
# 계산된 목표를 이만큼 넘어서면 강제 정지한다. 예전에는 고정 상한(1.50)이라
# 옆거리가 커지면(후진거리 = 0.70 + 옆거리) 목표에 닿기도 전에 상한이 먼저
# 걸려 얕게 주차됐다. 옆거리 1.0m 면 후진 1.70m 라 1.50 으로는 불가능하다.
# 목표 대비 상대값으로 두면 옆거리와 무관하게 "0.1m 초과 금지" 라는 원래
# 안전 의도가 유지된다.
REVERSE_OVERSHOOT_LIMIT = 0.10
MAX_REVERSE_TRAVEL = 2.00   # 라이다 측정 실패 시의 폴백 겸 절대 상한
                             # 위치(오도메트리 기준)에 못 이르면 강제 정지(FINISHED) -
                             # 오도메트리 드리프트 등에 대비한 최후 방어선.
                             # 회전 종료 시 차체 중심이 world y=-0.6 부근이고 목표가
                             # y=0.8이므로 정상 소요 후진거리는 약 1.40m다. 예전 값
                             # 1.6은 목표를 0.2m 지나칠 때까지 계속 후진을 허용했고,
                             # 그 경우 차체 후단이 y=1.7까지 가서 안쪽 장애물(앞면
                             # y=1.80)과 10cm밖에 안 남았다 - "최후 방어선"이라기엔
                             # 여유가 없어서 1.50(목표 초과 0.1m)으로 조인다.


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, lo, hi):
    return max(lo, min(hi, value))


class ParkingController(Node):
    """후진주차 미션 노드.

    integrated_stack 통합 규약:
      - `/follower/active_wp` 라벨이 주차 구역일 때만 동작한다(그 밖에서는
        parking_active=False + 0속도). local_avoider의 require_obstacle_zone과
        같은 방식이며, 이 게이트가 곧 main_controller의 PARKING 소스 선택 조건이다.
      - 명령은 `/cmd_vel/parking`, 활성 신호는 `/parking_active`로 낸다.
        후진은 main_controller의 parking_reverse_cap으로만 열린다
        (reverse_linear_cap은 전 구간 0.0으로 잠긴 채 유지).
      - follower가 주는 동적 속도상한(active_speed_cap)을 넘지 않는다.
    """

    def __init__(self):
        super().__init__('parking_controller_node')

        # --- 토픽/게이팅 파라미터 (메인 스택 규약) ---
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('pose_topic', '/follower/control_pose')
        self.declare_parameter('imu_topic', '/imu/heading_enu')
        self.declare_parameter('cmd_topic', '/cmd_vel/parking')
        self.declare_parameter('active_topic', '/parking_active')
        self.declare_parameter('active_waypoint_topic', '/follower/active_wp')
        self.declare_parameter('active_waypoint_timeout', 1.10)
        self.declare_parameter('require_parking_zone', True)
        self.declare_parameter(
            'active_speed_cap_topic', '/follower/active_speed_cap'
        )
        self.declare_parameter('active_speed_cap_timeout', 0.35)
        self.declare_parameter('require_speed_cap', True)

        # --- 차량/코스 규격 ---
        # 기본값은 이 통합 스택의 local_avoider 차량 기준과 일치시킨다.
        # spec_confirmed는 배포 설정에서 동일 값을 명시적으로 확인하는 게이트다.
        self.declare_parameter('spec_confirmed', True)
        self.declare_parameter('lidar_offset_x', LIDAR_OFFSET_X)
        # 라이다 장착 각도(rad). local_avoider의 laser_yaw_offset과 같은 값이어야
        # 한다. 이 차량은 라이다가 정반대라 3.1416이다.
        self.declare_parameter('laser_yaw_offset', 0.0)
        # 슬롯이 좌우 어느 쪽인지 주차 구간 진입 후 라이다로 판정한다.
        # false면 slot_on_left 파라미터를 그대로 쓴다(기존 동작).
        self.declare_parameter('detect_slot_side', True)
        # 판정 섹터 중심(rad, base_link 기준 좌/우 정측면)과 확정에 필요한 연속 관측 수.
        self.declare_parameter('slot_detect_confirm_scans', 5)
        self.declare_parameter('vehicle_length', VEHICLE_LENGTH)
        self.declare_parameter('slot_back_boundary_y', SLOT_BACK_BOUNDARY_Y)
        self.declare_parameter('side_box_front_y', SIDE_BOX_FRONT_Y)
        # 진행방향 기준 슬롯이 어느 쪽인가. 갭 탐지 섹터 방향과 회전 목표
        # 자세가 통째로 뒤집히므로 코스에 맞게 반드시 확인할 것.
        self.declare_parameter('slot_on_left', True)
        self.declare_parameter('row_near_dist', ROW_NEAR_DIST)
        self.declare_parameter('row_hold_dist', ROW_HOLD_DIST)  # 미사용(호환용)
        self.declare_parameter('row_hold_min', ROW_HOLD_MIN)
        self.declare_parameter('row_hold_max', ROW_HOLD_MAX)
        self.declare_parameter('rotate_scrub_drift', ROTATE_SCRUB_DRIFT)
        self.declare_parameter('speed_search', SPEED_SEARCH)
        self.declare_parameter('speed_back_to_center', abs(SPEED_BACK_TO_CENTER))
        self.declare_parameter('speed_straight', abs(SPEED_STRAIGHT))
        self.declare_parameter('speed_escape', SPEED_ESCAPE)

        self.spec_confirmed = bool(self.get_parameter('spec_confirmed').value)
        self.lidar_offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.laser_yaw_offset = float(
            self.get_parameter('laser_yaw_offset').value
        )
        self.detect_slot_side = bool(
            self.get_parameter('detect_slot_side').value
        )
        self.slot_detect_confirm_scans = int(
            self.get_parameter('slot_detect_confirm_scans').value
        )
        self.vehicle_length = float(self.get_parameter('vehicle_length').value)
        self.slot_back_boundary_y = float(
            self.get_parameter('slot_back_boundary_y').value
        )
        self.side_box_front_y = float(
            self.get_parameter('side_box_front_y').value
        )
        self.slot_on_left = bool(self.get_parameter('slot_on_left').value)
        self.row_near_dist = float(self.get_parameter('row_near_dist').value)
        self.row_hold_dist = float(self.get_parameter('row_hold_dist').value)
        self.row_hold_min = float(self.get_parameter('row_hold_min').value)
        self.row_hold_max = float(self.get_parameter('row_hold_max').value)
        self.rotate_scrub_drift = float(
            self.get_parameter('rotate_scrub_drift').value
        )
        self.speed_search = abs(float(self.get_parameter('speed_search').value))
        self.speed_back_to_center = abs(
            float(self.get_parameter('speed_back_to_center').value)
        )
        self.speed_straight = abs(
            float(self.get_parameter('speed_straight').value)
        )
        self.speed_escape = abs(float(self.get_parameter('speed_escape').value))

        # 슬롯 방향에서 파생되는 값들.
        #  - 갭 탐지/row-hold 섹터: 슬롯이 있는 쪽 정측면
        #  - 회전 목표 자세: 차체 뒤가 슬롯을 향하도록(= 앞은 반대편)
        self._apply_slot_side(self.slot_on_left)
        # 실제 목표각은 주차 진입 시 follower 경로 진행방향을 기준으로 정한다.
        self.target_yaw = 0.0
        self.target_center_y = (
            SLOT_ENTRY_Y + self.slot_back_boundary_y
        ) / 2.0
        self.active_waypoint_timeout = float(
            self.get_parameter('active_waypoint_timeout').value
        )
        self.require_parking_zone = bool(
            self.get_parameter('require_parking_zone').value
        )
        self.active_speed_cap_timeout = float(
            self.get_parameter('active_speed_cap_timeout').value
        )
        self.require_speed_cap = bool(
            self.get_parameter('require_speed_cap').value
        )

        self.parking_zone_active = False
        self.parking_mission_latched = False
        self.active_waypoint_stamp_sec = None
        self.active_speed_cap = None
        self.active_speed_cap_stamp_sec = None
        self.zone_logged = False

        # 슬롯 방향 자동 판정 상태. detect_slot_side가 false면 곧바로 잠근
        # 것으로 취급해 기존 동작(slot_on_left 파라미터 고정)을 유지한다.
        self.slot_side_locked = not self.detect_slot_side
        self.slot_detect_pending = None
        self.slot_detect_votes = 0
        self.detect_heading = 0.0
        self.detect_heading_set = False

        self.state = 'DETECT_SIDE' if self.detect_slot_side else 'SEARCH'
        self.x = 0.0
        self.y = 0.0
        # self.yaw는 제어에 실제로 쓰는 자세다. 통합 실행에서는 follower의
        # control_pose, 단독 fallback에서는 보정 IMU/odom을 사용한다.
        self.yaw = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw = None
        self.imu_stamp_sec = None
        self.scan_stamp_sec = None
        self.scan_stale_warned = False
        self.yaw_source = 'odom'
        self.have_odom = False
        self.scan = None
        # 정밀 기동 상태에서 긴급정지가 걸려 있는 동안 로그를 한 번만 남기기 위한 래치
        self.emergency_hold = False
        # stall 감지용 기준점
        self.stall_ref_x = None
        self.stall_ref_y = None
        self.stall_ref_sec = None

        self.search_start_x = 0.0
        self.search_start_y = 0.0
        self.search_start_set = False
        self.search_heading = 0.0

        # 편측 갭 탐지 상태
        self.row_phase = 'BEFORE_BOX1'   # BEFORE_BOX1 -> AT_BOX1 -> IN_GAP
        self.row_near = False
        self.row_pending = None
        self.row_pending_count = 0
        self.gap_entry_pos = None
        self.gap_exit_pos = None
        self.slot_center_s = 0.0

        # RETURN_SCAN: box2에서 살짝 후진해 "이격"을 반대 방향에서 재측정하는 상태
        # (전진 중 측정과 평균 내면 디바운스/센서 각도 등 방향 의존적 편향이 상쇄됨)
        self.return_near = True
        self.return_pending = None
        self.return_pending_count = 0
        self.return_exit_pos = None
        self.return_start_x = 0.0
        self.return_start_y = 0.0

        self.back_start_x = 0.0
        self.back_start_y = 0.0
        self.back_direction = -1.0
        self.straight_start_x = 0.0
        self.straight_start_y = 0.0
        # 박스 옆을 지날 때 좌측 섹터가 잰 마지막 거리. 슬롯 기준 위치를
        # 절대좌표 없이 역산하는 데 쓴다(_slot_frame_y 참고).
        self.side_dist_alongside = None
        # ROTATE_IN_PLACE 종료 시점에 확정하는 후진 계획(전부 상대 거리)
        self.reverse_target_dist = None   # 목표 정차까지 후진해야 할 거리

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10
        )
        self.active_pub = self.create_publisher(
            Bool, str(self.get_parameter('active_topic').value), 10
        )
        self.state_pub = self.create_publisher(
            String, '/parking/debug/state', 10
        )
        pose_topic = str(self.get_parameter('pose_topic').value).strip()
        if pose_topic:
            # 회피 노드와 동일하게 follower가 실제 제어에 쓴 pose를 사용한다.
            self.create_subscription(
                PoseStamped,
                pose_topic,
                self.control_pose_cb,
                10,
            )
        else:
            # 단독 실행용 fallback. 통합 설정에서는 이 경로를 사용하지 않는다.
            self.create_subscription(
                Odometry,
                str(self.get_parameter('odom_topic').value),
                self.odom_cb,
                10,
            )
            self.create_subscription(
                Imu,
                str(self.get_parameter('imu_topic').value),
                self.imu_cb,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('active_waypoint_topic').value),
            self.active_waypoint_cb,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('active_speed_cap_topic').value),
            self.active_speed_cap_cb,
            10,
        )

        self.timer = self.create_timer(CONTROL_PERIOD, self.control_step)
        if not self.spec_confirmed:
            self.get_logger().error(
                'spec_confirmed=false - 주차 노드가 비활성 상태로 시작한다. '
                '차량/코스 규격값을 확인하고 설정에서 활성화해야 한다 '
                f'(lidar_offset_x={self.lidar_offset_x:.3f}, '
                f'vehicle_length={self.vehicle_length:.3f}, '
                f'slot_back_boundary_y={self.slot_back_boundary_y:.3f}, '
                f"slot_on_left={self.slot_on_left}). "
                '실측값을 넣고 spec_confirmed를 true로 바꾸기 전까지 '
                '어떤 명령도 내지 않는다.'
            )
        else:
            self.get_logger().info(
                'parking_controller_node 시작 - 주차 waypoint 구역에서만 동작 '
                f'(require_parking_zone={self.require_parking_zone}, '
                f'lidar_offset_x={self.lidar_offset_x:.3f}, '
                f'vehicle_length={self.vehicle_length:.3f}, '
                f'slot_on_left={self.slot_on_left})'
            )

    # ---- 미션 게이팅 -----------------------------------------------------------

    def active_waypoint_cb(self, msg: String) -> None:
        self.parking_zone_active = is_parking_waypoint(msg.data)
        if self.parking_zone_active:
            # SEARCH가 진행되며 follower의 active waypoint가 다음 점으로 넘어가도
            # 시작한 주차 상태머신은 FINISHED/FAILED까지 이어가야 한다.
            self.parking_mission_latched = True
        self.active_waypoint_stamp_sec = self._now_sec()

    def active_speed_cap_cb(self, msg: Float32) -> None:
        self.active_speed_cap = float(msg.data)
        self.active_speed_cap_stamp_sec = self._now_sec()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _fresh(self, stamp_sec, timeout_sec) -> bool:
        if stamp_sec is None:
            return False
        age = self._now_sec() - stamp_sec
        return 0.0 <= age <= timeout_sec

    def _in_parking_zone(self) -> bool:
        """주차 미션 구역 안이며 그 신호가 신선한가."""
        if not self.require_parking_zone:
            return True
        # 마지막 waypoint가 주차 지점이면 follower는 완료 판정 뒤에도 속도
        # 상한은 계속 발행하지만 active_wp는 더 발행하지 않는다. 주차가 한 번
        # 시작된 뒤에는 waypoint heartbeat 만료로 상태머신을 중간에 끊지 않는다.
        if self.parking_mission_latched:
            return True
        return (
            self.parking_zone_active
            and self._fresh(
                self.active_waypoint_stamp_sec,
                self.active_waypoint_timeout,
            )
        )

    def _fresh_speed_cap(self):
        """follower가 준 동적 속도상한. 없거나 오래되면 None."""
        if not self._fresh(
            self.active_speed_cap_stamp_sec, self.active_speed_cap_timeout
        ):
            return None
        cap = self.active_speed_cap
        if cap is None or not math.isfinite(cap) or cap <= 0.0:
            return None
        return cap

    def _check_stall(self, commanded_linear: float, now_sec: float) -> bool:
        """의미 있는 속도를 명령했는데 실제로 안 움직이면 True.

        가장 흔한 원인은 최종 selector의 parking_reverse_cap이 0.0이라 후진
        명령이 0으로 잘리는 경우다(STALL_* 상수 주석 참고). 그 외에 바퀴가
        물리적으로 걸렸거나 다른 노드가 cmd_vel을 덮어쓰는 경우도 잡힌다.
        """
        if abs(commanded_linear) < STALL_MIN_COMMAND:
            # 정지를 의도한 구간(긴급정지 등)은 판정 대상이 아니다.
            self.stall_ref_x = None
            self.stall_ref_y = None
            self.stall_ref_sec = None
            return False

        if self.stall_ref_sec is None:
            self.stall_ref_x, self.stall_ref_y = self.x, self.y
            self.stall_ref_sec = now_sec
            return False

        moved = math.hypot(self.x - self.stall_ref_x, self.y - self.stall_ref_y)
        if moved >= STALL_MIN_TRAVEL:
            # 정상 이동 - 기준점을 현재 위치로 당긴다.
            self.stall_ref_x, self.stall_ref_y = self.x, self.y
            self.stall_ref_sec = now_sec
            return False

        return (now_sec - self.stall_ref_sec) >= STALL_CHECK_SEC

    def _publish(self, linear: float, angular: float, active: bool) -> None:
        """명령/활성신호/상태를 한 번에 낸다.

        active=False면 main_controller가 PARKING 소스를 고르지 않으므로,
        이 노드가 주차 구역 밖에서 차량을 움직일 수 없다.
        """
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        self.active_pub.publish(Bool(data=active))
        self.state_pub.publish(String(data=self.state))

    def control_pose_cb(self, msg: PoseStamped):
        """waypoint_follower가 사용한 map pose를 주차 제어에도 그대로 쓴다."""
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.yaw = quaternion_to_yaw(msg.pose.orientation)
        self.odom_yaw = self.yaw
        self.yaw_source = 'follower/control_pose'
        self.have_odom = True

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True
        self._refresh_yaw()

    def imu_cb(self, msg: Imu):
        self.imu_yaw = quaternion_to_yaw(msg.orientation)
        self.imu_stamp_sec = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self._refresh_yaw()

    def scan_cb(self, msg: LaserScan):
        self.scan = msg
        self.scan_stamp_sec = self.get_clock().now().nanoseconds / 1e9

    def _refresh_yaw(self):
        """제어에 쓸 자세를 고른다: IMU 우선, 끊기면 odom 폴백.

        바퀴 적분 오도메트리는 제자리 회전에서 바퀴가 미끄러지면 그 슬립을
        알 수 없어 "실제보다 더 돌았다"고 착각한다. Gazebo 실측으로 확인한
        결과 ROTATE_IN_PLACE 종료 시점에 odom은 -90.00도를 가리켰지만
        실제 자세는 -82.83도로 **7.17도**가 틀렸고, 그 자세 오차가 이어지는
        1.4m 후진에서 증폭되어 최종 횡방향 오차 19cm(차체 좌측이 좌측 박스
        뒤 모서리를 사실상 스치는 수준)로 커졌다. 같은 시점 IMU는 -82.833도로
        실제값과 소수점 둘째 자리까지 일치했다.

        위치(x, y)는 odom이 충분히 정확하다(회전 종료 시점 실측 오차 약
        1.7cm) - 문제는 자세뿐이라 자세만 IMU로 바꾼다.

        실차 주의: 시뮬레이터 IMU는 절대 자세를 그대로 주지만, 실제 IMU는
        자이로 적분 드리프트가 있다. 주차 기동은 수십 초 단위로 짧아 드리프트가
        누적될 시간이 없지만, 장시간 주행 후 곧바로 주차에 들어간다면 진입
        직전에 yaw 기준을 다시 잡아주는 게 안전하다.

        IMU가 끊긴 경우(IMU_STALE_SEC 초과) 마지막 값을 계속 쓰면 고정된 옛
        자세로 제어하게 되므로, 명시적으로 odom 폴백으로 내려간다. odom yaw는
        슬립 오차가 있지만 "멈춰버린 값"보다는 낫다.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        imu_fresh = (
            self.imu_yaw is not None
            and self.imu_stamp_sec is not None
            and 0.0 <= (now - self.imu_stamp_sec) <= IMU_STALE_SEC
        )
        if imu_fresh:
            self.yaw = self.imu_yaw
            if self.yaw_source != 'imu':
                self.yaw_source = 'imu'
                self.get_logger().info('자세 소스: IMU 사용(바퀴 슬립 영향 없음)')
        else:
            self.yaw = self.odom_yaw
            if self.yaw_source != 'odom':
                self.yaw_source = 'odom'
                self.get_logger().warn(
                    ' 자세 소스: IMU 끊김/없음 - odom 폴백'
                    '(회전 슬립으로 7~9도 오차 발생 가능)'
                )

    # ---- LiDAR 기반 거리 계산 (섹터 기반, 상태전이 트리거) --------------

    def _apply_slot_side(self, on_left):
        """슬롯 방향과 거기서 파생되는 값들을 한 번에 정한다.

        갭 탐지/row-hold 섹터는 슬롯이 있는 쪽 정측면이고, 회전 목표 자세는
        차체 뒤가 슬롯을 향하도록(= 앞은 반대편) 잡는다. 셋이 따로 놀면
        엉뚱한 쪽으로 기동하므로 반드시 같이 바꾼다.
        """
        self.slot_on_left = bool(on_left)
        self.slot_side_sign = 1.0 if self.slot_on_left else -1.0
        self.side_sector_center = (
            math.pi / 2.0 if self.slot_on_left else -math.pi / 2.0
        )

    def detect_slot_side_once(self):
        """좌우 정측면을 동시에 재서 박스 행이 있는 쪽을 슬롯 쪽으로 확정한다.

        코스는 한쪽에만 박스가 놓이고 좌우는 매번 바뀐다. 그래서 slot_on_left를
        런치 파라미터로 고정할 수 없다. 판정은 단순하다 - row_near_dist 안에
        반사가 잡히는 쪽이 박스 행이다. 반대쪽은 빈 공간이라 훨씬 멀거나 inf다.

        한 스캔의 노이즈로 뒤집히지 않도록 slot_detect_confirm_scans 회 연속으로
        같은 결론이 나와야 확정한다. 확정 뒤에는 주차가 끝날 때까지 다시 보지
        않는다 - 기동 중에 방향이 바뀌면 회전 목표와 슬롯 좌표가 통째로
        어긋난다.

        확정하면 True.
        """
        left = self._sector_min_range(math.pi / 2.0, SIDE_SECTOR_HALF_WIDTH)
        right = self._sector_min_range(-math.pi / 2.0, SIDE_SECTOR_HALF_WIDTH)
        left_near = left < self.row_near_dist
        right_near = right < self.row_near_dist

        if left_near == right_near:
            # 둘 다 비었거나(아직 박스 행에 도달 전) 둘 다 가깝다(모호).
            # 어느 쪽도 확정하지 않고 관측을 리셋한다.
            self.slot_detect_votes = 0
            self.slot_detect_pending = None
            return False

        candidate = left_near
        if self.slot_detect_pending is not candidate:
            self.slot_detect_pending = candidate
            self.slot_detect_votes = 1
        else:
            self.slot_detect_votes += 1

        if self.slot_detect_votes < self.slot_detect_confirm_scans:
            return False

        self._apply_slot_side(candidate)
        self.slot_side_locked = True
        self.get_logger().info(
            f"SLOT SIDE = {'LEFT' if candidate else 'RIGHT'} 확정 "
            f"(left={left:.2f}m, right={right:.2f}m, "
            f"row_near_dist={self.row_near_dist:.2f}m, "
            f"{self.slot_detect_votes}회 연속)"
        )
        return True

    def _beam_angle(self, scan, index):
        """스캔 인덱스를 base_link 기준 각도로 바꾼다.

        2026-08-09: 예전에는 scan.angle_min + i*inc 를 그대로 썼다. 그런데 이
        차량은 라이다가 정반대로 장착돼 있어(local_avoider.yaml
        laser_yaw_offset=3.1416, 2026-08-06 실측 "차량 정면 = laser 180도")
        원본 각도의 +90도는 실제로는 차량 오른쪽이다. 그 상태로 slot_on_left를
        true로 두면 주차 로직이 반대쪽을 뒤진다.
        """
        return wrap_to_pi(
            scan.angle_min + index * scan.angle_increment + self.laser_yaw_offset
        )

    def _sector_min_range(self, center_angle, half_width):
        scan = self.scan
        best = float('inf')
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            angle = self._beam_angle(scan, i)
            diff = wrap_to_pi(angle - center_angle)
            if abs(diff) <= half_width:
                best = min(best, r)
        return best

    def compute_row_distance(self):
        # 로봇 좌표계 기준 좌측 전방-측면 섹터의 최소 거리 - 장애물 행이 있는 쪽.
        return self._sector_min_range(self.side_sector_center, SIDE_SECTOR_HALF_WIDTH)

    def compute_nearest_obstacle(self):
        scan = self.scan
        best_r = float('inf')
        best_angle = 0.0
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < best_r:
                best_r = r
                best_angle = self._beam_angle(scan, i)
        return best_angle, best_r

    def compute_emergency(self):
        _, best_r = self.compute_nearest_obstacle()
        return best_r < EMERGENCY_DIST

    def _lane_hold_angular(self, target_heading):
        """헤딩을 target_heading 으로 유지하면서, 옆 라이다 거리가 허용 밴드
        [row_hold_min, row_hold_max] 를 벗어났을 때만 되돌리는 보정을 더한다.

        밴드 안이면 아무 것도 하지 않는다. 예전에는 단일 임계값(0.75)에서
        "가까우면 밀어내기만" 하는 편측 보정이었는데, 그러면 0.75 보다 가깝게
        찍힌 경로는 실효 옆거리가 결국 0.75 로 밀려나고 그 과정에서 차가
        비스듬히 달린다(옆거리 0.5m 에서 정상상태 편향 8.6도).

        끌어당김(너무 멀 때)은 row_near_dist 안에서만 한다. 갭 구간은 측면이
        빈 공간이라 제한이 없으면 갭에서 차를 옆으로 끌어당겨 통과선이 무너진다.
        예전 주석이 경고하던 "양방향 보정이 slot_center_s 를 장애물 쪽으로
        민다"는 문제도 이 제한과 밴드(무보정 구간)로 함께 막는다.
        """
        heading_term = -K_HEADING_HOLD * wrap_to_pi(self.yaw - target_heading)
        row_dist = self.compute_row_distance()
        row_term = 0.0
        if not math.isinf(row_dist):
            if row_dist < self.row_hold_min:
                # 너무 붙었다 -> 멀어지는 쪽으로만
                row_term = self.slot_side_sign * clip(
                    K_ROW_HOLD * (row_dist - self.row_hold_min),
                    -MAX_ROW_CORRECTION,
                    0.0,
                )
            elif self.row_hold_max < row_dist < self.row_near_dist:
                # 너무 멀다(단 아직 박스 옆으로 인정되는 범위) -> 붙는 쪽으로만
                row_term = self.slot_side_sign * clip(
                    K_ROW_HOLD * (row_dist - self.row_hold_max),
                    0.0,
                    MAX_ROW_CORRECTION,
                )
        return clip(heading_term + row_term, -0.5, 0.5)

    def _route_progress(self, position=None):
        """주차 진입 시점부터 follower 진행방향으로 투영한 거리."""
        px, py = (self.x, self.y) if position is None else position
        dx = px - self.search_start_x
        dy = py - self.search_start_y
        return (
            math.cos(self.search_heading) * dx
            + math.sin(self.search_heading) * dy
        )

    def _slot_frame_y(self):
        """회전 직전 로봇 중심의 슬롯 좌표 y를 라이다 측정으로 역산한다.

        예전에는 `SPAWN_Y + self.y`로 구했는데, 이건 "odom 원점 == 스폰 지점 ==
        슬롯 좌표계의 특정 지점"을 전제한다. 시뮬레이터에서는 맞지만 실차에서는
        odom 원점이 로봇이 부팅된 임의의 위치라 이 식 자체가 성립하지 않는다.

        대신 옆 칸 주차 차량의 입구쪽 면(self.side_box_front_y)을 기준으로 삼는다:
        SEARCH 중 좌측 90도 섹터가 재는 값이 곧 "로봇 중심 ~ 그 면"까지의 수직
        거리이므로(라이다 y offset 0, 섹터가 정측면이라 종방향 오프셋은 무관),

            로봇 슬롯좌표 y = self.side_box_front_y - (좌측 섹터 거리)

        로 역산된다. 좌표 원점이 어디든 무관한 순수 상대 측정이다.

        박스 옆을 지날 때 실측한 마지막 값(self.side_dist_alongside)을 쓴다 -
        갭 구간에서는 섹터가 빈 공간을 보므로 그때 값은 쓰면 안 된다.
        """
        if self.side_dist_alongside is None:
            return None
        return self.side_box_front_y - self.side_dist_alongside

    def _plan_reverse(self):
        """회전 종료 시점에 "얼마나 후진할지"를 상대 거리로 확정한다.

        후방은 라이다 사각이라 직접 못 재므로, 회전 **전에** 라이다로 잰
        슬롯 기준 위치(_slot_frame_y)와 이미 알고 있는 코스 치수로 계산한다:

            필요 후진거리 = 목표 정차 y(0.8) - 현재 슬롯좌표 y
        이 값은 "회전 종료 지점 기준 상대 거리"라 odom 원점이 어디든 무관하다.
        라이다 측정을 못 얻은 경우에만 주행거리 폴백(MAX_REVERSE_TRAVEL)에
        의존한다.
        """
        slot_y = self._slot_frame_y()
        if slot_y is None:
            self.reverse_target_dist = None
            self.get_logger().warn(
                '후진 계획: 옆 박스 측정값이 없어 슬롯 기준 위치를 못 구했다 - '
                f'주행거리 폴백({MAX_REVERSE_TRAVEL:.2f}m)만으로 정지 판정'
            )
            return

        self.reverse_target_dist = self.target_center_y - slot_y
        self.get_logger().info(
            f'후진 계획: 슬롯기준 현재 y={slot_y:+.3f}m '
            f'(옆박스 측정 {self.side_dist_alongside:.3f}m로 역산) -> '
            f'목표까지 {self.reverse_target_dist:.3f}m'
        )

    # ---- SEARCH: 편측 갭 탐지 (box1 -> 빈공간 -> box2) ----------------------

    def update_row_gap_detection(self):
        """매 제어주기 로봇 왼쪽(전방-측면) 섹터를 보고 BEFORE_BOX1 -> AT_BOX1 ->
        IN_GAP 순으로 진행시킨다. IN_GAP에서 다시 장애물이 잡히면(box2) 갭 중심을
        계산해 True를 반환(BACK_TO_CENTER로 전환하라는 신호), 그 전까지는 False."""
        row_dist = self.compute_row_distance()
        raw_near = row_dist < self.row_near_dist
        # 박스 옆을 지나는 동안의 측정값만 저장한다(갭 구간 값은 빈 공간이라 무의미).
        # 이 값으로 슬롯 기준 위치를 절대좌표 없이 역산한다(_slot_frame_y).
        if raw_near and math.isfinite(row_dist):
            self.side_dist_alongside = row_dist

        if raw_near == self.row_near:
            self.row_pending_count = 0
            self.row_pending = None
            return False

        if self.row_pending != raw_near:
            self.row_pending = raw_near
            self.row_pending_count = 1
        else:
            self.row_pending_count += 1

        if self.row_pending_count < ROW_DEBOUNCE_COUNT:
            return False

        # 확정된 전이
        self.row_near = raw_near
        self.row_pending_count = 0

        if self.row_phase == 'BEFORE_BOX1' and self.row_near:
            self.row_phase = 'AT_BOX1'
            self.get_logger().info('SEARCH: 첫 번째 장애물(box1) 옆 통과 중')
        elif self.row_phase == 'AT_BOX1' and not self.row_near:
            self.row_phase = 'IN_GAP'
            self.gap_entry_pos = (self.x, self.y)
            self.get_logger().info(
                f'SEARCH: 갭 진입 감지 (s={self._route_progress():.2f})'
            )
        elif self.row_phase == 'IN_GAP' and self.row_near:
            self.gap_exit_pos = (self.x, self.y)
            self.get_logger().info(
                f'SEARCH: 갭 이탈 감지 (s={self._route_progress():.2f}) '
                '- box2 도달'
            )
            return True

        return False

    # ---- RETURN_SCAN: 후진하며 box2/box1 경계 반대 방향 재측정 ----------------

    def update_return_scan(self):
        """SEARCH가 box1을 "이탈"(근접->이격, +s 방향 이동 중)한 지점(gap_entry_pos)에는
        방향 의존적 편향(디바운스 지연, 라이다 섹터가 모서리를 비스듬히 붙잡는 정도)이
        그대로 남아있다. box2에서 살짝만 후진해 같은 종류의 전이(근접->이격, 이번엔
        -s 방향 이동 중)를 한 번 더 재면, 같은 종류의 전이는 방향만 반대이므로 편향도
        부호만 반대로 걸린다 - box1 쪽까지 다시 갈 필요 없이 이 두 값만 평균 내도
        슬롯 중심이 나온다(자세한 유도는 모듈 docstring 참고). box2에서 "이격"으로
        바뀌는 순간(단 한 번의 전이) True를 반환한다."""
        raw_near = self.compute_row_distance() < self.row_near_dist

        if raw_near == self.return_near:
            self.return_pending_count = 0
            self.return_pending = None
            return False

        if self.return_pending != raw_near:
            self.return_pending = raw_near
            self.return_pending_count = 1
        else:
            self.return_pending_count += 1

        if self.return_pending_count < ROW_DEBOUNCE_COUNT:
            return False

        self.return_near = raw_near
        self.return_pending_count = 0

        if not self.return_near:
            self.return_exit_pos = (self.x, self.y)
            self.get_logger().info(
                f'RETURN_SCAN: box2 이탈 재감지 '
                f'(s={self._route_progress():.2f})'
            )
            return True

        return False

    # ---- 제어 루프 -------------------------------------------------------------

    def control_step(self):
        # 배포 설정에서 차량/코스 규격 확인이 해제되어 있으면 동작하지 않는다.
        if not self.spec_confirmed:
            self._publish(0.0, 0.0, False)
            return

        # 주차 waypoint 구역 밖에서는 어떤 명령도 내지 않는다. active=False라
        # main_controller가 PARKING 소스를 고르지 않으므로 후진 게이트도 닫힌다.
        if not self._in_parking_zone():
            if self.zone_logged:
                self.zone_logged = False
                self.get_logger().info('주차 구역 이탈 - 대기')
            self._publish(0.0, 0.0, False)
            return
        if not self.zone_logged:
            self.zone_logged = True
            self.get_logger().info(f'주차 구역 진입 - 상태: {self.state}')

        # follower의 동적 속도상한은 이 미션에도 그대로 적용된다.
        speed_cap = self._fresh_speed_cap()
        if self.require_speed_cap and speed_cap is None:
            self._publish(0.0, 0.0, True)
            return

        if not self.have_odom or self.scan is None:
            self._publish(0.0, 0.0, True)
            return

        # LiDAR가 끊기면 마지막 스캔을 계속 재사용하지 않는다. 갭 탐지와
        # 긴급회피가 모두 스캔에 걸려 있으므로, 옛 데이터로 주행을 이어가는
        # 것보다 정지가 안전하다(FINISHED/FAILED는 이미 정지 상태라 제외).
        now_sec = self.get_clock().now().nanoseconds / 1e9
        scan_fresh = (
            self.scan_stamp_sec is not None
            and 0.0 <= (now_sec - self.scan_stamp_sec) <= SCAN_STALE_SEC
        )
        if not scan_fresh and self.state not in ('FINISHED', 'FAILED'):
            if not self.scan_stale_warned:
                self.scan_stale_warned = True
                age = (
                    now_sec - self.scan_stamp_sec
                    if self.scan_stamp_sec is not None else float('inf')
                )
                self.get_logger().error(
                    f'LiDAR 끊김({age:.2f}s 경과) - {self.state} 중 정지'
                )
            self._publish(0.0, 0.0, True)
            return
        if scan_fresh and self.scan_stale_warned:
            self.scan_stale_warned = False
            self.get_logger().info(f'LiDAR 복구 - {self.state} 재개')

        linear = 0.0
        angular = 0.0

        # [최상위 알고리즘] FINISHED/FAILED 가 아닌 한 항상 최우선으로 검사.
        # 충돌 일보 직전의 최후 방어선이므로 가장 단순하고 예측 가능한 규칙을 쓴다.
        # 단, 대응 방식은 상태에 따라 다르다(EMERGENCY_FORWARD_ESCAPE_STATES 주석 참고):
        #  - 차선 직진 중(SEARCH): 가장 가까운 장애물 반대쪽으로 회전하며 전진 탈출.
        #  - 그 외 정밀 기동 중: 즉시 정지. 좁은 슬롯 안/옆에서 전진 탈출을 하면
        #    슬롯 밖으로 튀어나오며 옆 차를 긁고, 후진 중에는 각속도 부호의 의미까지
        #    뒤집혀 오히려 장애물 쪽으로 스윙한다.
        # (라이다가 못 보는 후방 장애물은 어느 쪽이든 이 회피의 대상이 될 수 없다 -
        #  애초에 감지 불가능한 영역이다.)
        if self.compute_emergency() and self.state not in ('FINISHED', 'FAILED'):
            if self.state in EMERGENCY_FORWARD_ESCAPE_STATES:
                angle, _ = self.compute_nearest_obstacle()
                linear = self.speed_escape
                angular = -ROTATE_SPEED if 0.0 < wrap_to_pi(angle) <= math.pi else ROTATE_SPEED
            else:
                linear = 0.0
                angular = 0.0
                if not self.emergency_hold:
                    self.emergency_hold = True
                    _, near_r = self.compute_nearest_obstacle()
                    self.get_logger().warn(
                        f'긴급정지: {self.state} 중 장애물 {near_r:.3f}m 근접 - '
                        f'정밀 기동 상태라 전진 탈출 대신 정지(장애물이 물러나면 재개)'
                    )

        else:
            if self.emergency_hold:
                self.emergency_hold = False
                self.get_logger().info(f'긴급정지 해제 - {self.state} 재개')

            if self.state == 'DETECT_SIDE':
                # 슬롯 방향이 정해지기 전에는 SEARCH의 파생값(회전 목표 자세,
                # 갭 탐지 섹터)을 계산할 수 없다. 차선을 따라 그대로 전진하며
                # 좌우를 재고, 확정되는 즉시 SEARCH로 넘어간다.
                #
                # 헤딩 유지는 _lane_hold_angular를 쓰지 않는다. 그 보정이
                # side_sector_center(=아직 미확정)에 의존하기 때문이다.
                # 2026-08-13: 확정 전에는 active=False로 두어 follower가 계속
                # 웨이포인트를 추종하게 한다. 예전에는 이 단계에서 이 노드가
                # speed_search로 직진했는데, 슬롯이 끝내 확정되지 않으면
                # (주차 칸이 row_near_dist 밖이면) follower를 132초간 밀어내
                # 마지막 웨이포인트에서도 정차하지 못했다.
                if not self.detect_slot_side_once():
                    self._publish(0.0, 0.0, False)
                    return
                self.state = 'SEARCH'
                self._publish(0.0, 0.0, False)
                return

            elif self.state == 'SEARCH':
                if not self.search_start_set:
                    self.search_start_x, self.search_start_y = self.x, self.y
                    self.search_heading = self.yaw
                    # 차량 뒤가 슬롯 쪽을 향하도록 현재 경로 진행방향에서
                    # 좌/우 90도를 상대 회전한다. map 절대축에는 의존하지 않는다.
                    self.target_yaw = wrap_to_pi(
                        self.search_heading
                        - self.slot_side_sign * math.pi / 2.0
                    )
                    self.search_start_set = True

                # 차선을 따라 직진 헤딩 유지(스폰 시의 헤딩을 기준으로 삼는다) +
                # 옆 장애물과의 거리 유지(row-hold)로 횡방향 드리프트 보정
                linear = self.speed_search
                angular = self._lane_hold_angular(self.search_heading)

                if self.update_row_gap_detection():
                    self.state = 'RETURN_SCAN'
                    self.return_start_x, self.return_start_y = self.x, self.y
                    self.get_logger().info(
                        f'SEARCH -> RETURN_SCAN '
                        f'(s={self._route_progress():.2f})'
                    )
                elif math.hypot(self.x - self.search_start_x,
                                 self.y - self.search_start_y) > MAX_SEARCH_TRAVEL:
                    self.state = 'FAILED'
                    self.get_logger().error('SEARCH 실패: 빈자리를 찾지 못하고 MAX_SEARCH_TRAVEL 초과')

            elif self.state == 'RETURN_SCAN':
                # box2를 막 지나친 상태에서 살짝만 후진해 "box2에서 벗어남"을
                # 반대 방향(-s)에서 다시 재는, 아주 짧은 재측정이다(모듈 docstring/
                # update_return_scan 참고 - box1까지 다시 갈 필요 없음).
                linear = -self.speed_back_to_center
                angular = self._lane_hold_angular(self.search_heading)

                if self.update_return_scan():
                    # true_center = (gap_entry_pos + return_exit_pos) / 2 - 두 값 모두
                    # "근접->이격" 전이(하나는 +s, 하나는 -s 방향)라 방향 의존적 편향이
                    # 부호만 반대로 걸려서 평균에서 상쇄된다(유도는 모듈 docstring 참고).
                    self.slot_center_s = (
                        (
                            self._route_progress(self.gap_entry_pos)
                            + self._route_progress(self.return_exit_pos)
                        )
                        / 2.0
                        + self.lidar_offset_x - self.rotate_scrub_drift
                    )
                    self.state = 'BACK_TO_CENTER'
                    self.back_start_x, self.back_start_y = self.x, self.y
                    # 방향에 따른 편향은 상쇄됐지만, self.lidar_offset_x 보정 때문에
                    # 목표(slot_center_s)가 현재 위치보다 앞(+s)일 수도 뒤(-s)일
                    # 수도 있으므로 방향을 가정하지 않고 매번 계산한다.
                    self.back_direction = (
                        1.0
                        if self.slot_center_s - self._route_progress() >= 0.0
                        else -1.0
                    )
                    self.get_logger().info(
                        f'RETURN_SCAN -> BACK_TO_CENTER '
                        f'(슬롯 중심 s={self.slot_center_s:.2f}, '
                        f'방향={"+" if self.back_direction > 0 else "-"})'
                    )
                elif math.hypot(self.x - self.return_start_x,
                                 self.y - self.return_start_y) > MAX_RETURN_SCAN_TRAVEL:
                    # 재측정 실패 - SEARCH의 편측(전진) 측정값만으로 폴백
                    self.slot_center_s = (
                        (
                            self._route_progress(self.gap_entry_pos)
                            + self._route_progress(self.gap_exit_pos)
                        )
                        / 2.0
                        + self.lidar_offset_x - self.rotate_scrub_drift
                    )
                    self.state = 'BACK_TO_CENTER'
                    self.back_start_x, self.back_start_y = self.x, self.y
                    self.back_direction = (
                        1.0
                        if self.slot_center_s - self._route_progress() >= 0.0
                        else -1.0
                    )
                    self.get_logger().warn(
                        f'RETURN_SCAN -> BACK_TO_CENTER (box2 재이탈 감지 실패, 편측 측정값 폴백, '
                        f'슬롯 중심 s={self.slot_center_s:.2f})'
                    )

            elif self.state == 'BACK_TO_CENTER':
                # 슬롯 중심 s까지 헤딩 유지한 채 이동(방향은 SEARCH -> BACK_TO_CENTER
                # 전환 시점에 계산해둔 self.back_direction).
                # 목표 근처에서는 감속한다 - 정지가 임계값 통과 방식이라 접근 속도가
                # 그대로 정지 위치의 양자화 폭이 되기 때문(BACK_TO_CENTER_SLOW_* 주석).
                current_s = self._route_progress()
                remaining = abs(self.slot_center_s - current_s)
                speed = (
                    BACK_TO_CENTER_SLOW_SPEED
                    if remaining < BACK_TO_CENTER_SLOW_DIST
                    else self.speed_back_to_center
                )
                linear = self.back_direction * speed
                angular = self._lane_hold_angular(self.search_heading)

                traveled = math.hypot(self.x - self.back_start_x, self.y - self.back_start_y)
                reached_center = (
                    (current_s - self.slot_center_s) * self.back_direction
                    >= 0.0
                )

                if reached_center:
                    self.state = 'ROTATE_IN_PLACE'
                    self.get_logger().info(
                        f'BACK_TO_CENTER -> ROTATE_IN_PLACE '
                        f'(s={current_s:.2f}, 목표 {self.slot_center_s:.2f})'
                    )
                elif traveled > MAX_BACK_TRAVEL:
                    self.state = 'ROTATE_IN_PLACE'
                    self.get_logger().warn(
                        'BACK_TO_CENTER -> ROTATE_IN_PLACE (슬롯 중심 미도달, 안전거리 폴백으로 전환)'
                    )

            elif self.state == 'ROTATE_IN_PLACE':
                # 위치는 그대로, 목표 자세(슬롯에 대해 수직)까지 제자리 회전만 한다.
                # 반드시 장애물 사이로 들어가기 전(개활지)에 끝나야 한다 - 모듈
                # docstring의 ROTATE_IN_PLACE 설명 참고.
                linear = 0.0
                angle_error = wrap_to_pi(self.target_yaw - self.yaw)
                angular = clip(K_ROTATE * angle_error, -ROTATE_SPEED, ROTATE_SPEED)
                if abs(angle_error) < ANGLE_TOL:
                    self.state = 'STRAIGHT_REVERSE'
                    self.straight_start_x, self.straight_start_y = self.x, self.y
                    # 자세 소스별 값을 같이 남긴다. 두 값의 차이가 곧 이번 회전에서
                    # 바퀴 슬립이 오도메트리에 누적시킨 오차다(실측 약 7도).
                    self.get_logger().info(
                        f'회전 종료 자세: 사용({self.yaw_source})='
                        f'{math.degrees(self.yaw):+.2f}deg, '
                        f'odom={math.degrees(self.odom_yaw):+.2f}deg, '
                        f'슬립오차='
                        f'{math.degrees(wrap_to_pi(self.odom_yaw - self.yaw)):+.2f}deg'
                    )
                    self.get_logger().info(
                        f'ROTATE_IN_PLACE -> STRAIGHT_REVERSE '
                        f'(s={self._route_progress():.4f}, '
                        f'중심 오차='
                        f'{self._route_progress() - self.slot_center_s:+.4f})'
                    )
                    self._plan_reverse()

            elif self.state == 'STRAIGHT_REVERSE':
                # 직진 후진하며 상대 목표 헤딩만 유지한다. 후방은 못 보므로 정지는
                # 오도메트리 기준 목표 정차 위치 도달 여부로 판단한다.
                #
                # 좌우 라이다 거리차 기반 중앙 정렬을 추가해봤지만(각속도에 직접
                # 더하는 방식, 이후 목표각 오프셋으로 변환하는 방식 둘 다) 오히려
                # 결과가 나빠져서 뺐다: 좌/우 라이다가 후진 시작 시점부터 동시에
                # 잡히지 않고(한쪽은 초반부터, 반대쪽은 도착 직전에야 잡힘) 대부분의
                # 구간에서 보정이 꺼져있다가 막판에만 급하게 작동해 오히려 흔들렸다.
                # 대신 BACK_TO_CENTER/RETURN_SCAN에서 회전 시작 위치 자체를 정확히
                # 맞추는 쪽으로 해결했다 - 위치가 정확하면 순수 헤딩 유지만으로
                # 충분하다(모듈 docstring 참고).
                linear = -self.speed_straight
                angular = clip(-K_HEADING_HOLD * wrap_to_pi(self.yaw - self.target_yaw), -0.5, 0.5)

                # 정지 판정은 전부 "회전 종료 지점부터 얼마나 후진했는가"라는
                # 상대 거리로 한다. 예전에는 절대 좌표(world_y)로 판정했는데,
                # 그건 odom 원점이 슬롯 좌표계의 특정 지점이라는 전제를 깔고
                # 있어서 실차에서는 성립하지 않는다(_plan_reverse 참고).
                traveled = math.hypot(self.x - self.straight_start_x,
                                       self.y - self.straight_start_y)
                target_dist = (
                    self.reverse_target_dist
                    if self.reverse_target_dist is not None
                    else MAX_REVERSE_TRAVEL
                )
                # 목표를 아는 경우엔 "목표 + 0.1m" 를 상한으로 쓴다. 고정
                # 상한이면 옆거리가 커질수록(후진거리 = 0.70 + 옆거리) 목표에
                # 닿기 전에 상한이 먼저 걸린다.
                travel_limit = (
                    target_dist + REVERSE_OVERSHOOT_LIMIT
                    if self.reverse_target_dist is not None
                    else MAX_REVERSE_TRAVEL
                )
                if traveled >= target_dist - CENTER_Y_TOL:
                    self.state = 'FINISHED'
                    self.get_logger().info(
                        f'STRAIGHT_REVERSE -> FINISHED (목표 정차 위치 도달, '
                        f'후진 {traveled:.3f}m / 목표 {target_dist:.3f}m)'
                    )
                elif traveled > travel_limit:
                    self.state = 'FINISHED'
                    self.get_logger().warn(
                        f'STRAIGHT_REVERSE -> FINISHED (목표 정차 위치 미도달, '
                        f'주행거리 폴백 {traveled:.2f}m > {travel_limit:.2f}m로 정지)'
                    )

            else:  # FAILED, FINISHED
                linear, angular = 0.0, 0.0

        # follower가 준 동적 속도상한을 전/후진 양방향 크기에 적용한다.
        # (최종 selector에도 parking_forward_cap/parking_reverse_cap이 있지만,
        #  거기서 잘리면 검증된 곡률이 깨지므로 여기서 미리 맞춰서 낸다.)
        if speed_cap is not None:
            linear = clip(linear, -speed_cap, speed_cap)

        # 명령은 내는데 실제로 안 움직이면 조용히 굳지 말고 명시적으로 실패시킨다.
        if (
            self.state not in ('FINISHED', 'FAILED')
            and self._check_stall(linear, now_sec)
        ):
            direction = '후진' if linear < 0.0 else '전진'
            self.get_logger().error(
                f'{self.state} 중 {direction} {abs(linear):.3f}m/s를 '
                f'{STALL_CHECK_SEC:.0f}초간 명령했으나 실제 이동이 '
                f'{STALL_MIN_TRAVEL * 100:.0f}cm 미만이다. '
                + (
                    'main_controller의 parking_reverse_cap이 0.0이면 후진 명령이 '
                    '0으로 잘린다 - 이 값부터 확인할 것. '
                    if linear < 0.0 else ''
                )
                + '(그 외 바퀴 구속/다른 노드의 cmd_vel 덮어쓰기 가능성)'
            )
            self.state = 'FAILED'
            self._publish(0.0, 0.0, True)
            return

        self._publish(linear, angular, True)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
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
