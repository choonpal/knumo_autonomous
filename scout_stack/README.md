# Scout Mini 자율주행 스택 (2026-08-17)

경기 코스: 3박스 슬라럼 회피 + 신호등 + 후진주차. ROS2 Humble.
2026-08-15 번들(`scout_stack_20260815`)의 후속본이다. 아래 "2026-08-17 변경 이력"만
새로 읽으면 되고, 나머지 구조/운용은 이전 README와 같다.

## 구성

```
competition_src/       주행 스택 src 전체
  my_nav_pkg/            follower, local_avoider, parking_controller, main_controller
  autodrive_pkg/         vision(YOLO 신호등), mission_control
    models/              YOLO26n.engine / YOLO26n.onnx
  competition_bringup/   런치
sensor_ws_src/         RPLIDAR S2 드라이버 (max_distance_override 추가본)
ros_home/waypoints.csv 현재 웨이포인트
can_up.sh              USB CAN 어댑터를 can0 으로 올리는 스크립트 (아래 참고)
```

## 설치

```bash
mkdir -p ~/competition/integrated_stack_cog_heading_2026-08-04
cp -a competition_src ~/competition/integrated_stack_cog_heading_2026-08-04/src
cd ~/competition/integrated_stack_cog_heading_2026-08-04
colcon build --symlink-install

mkdir -p ~/sensor_ws/src
cp -a sensor_ws_src/sllidar_ros2 ~/sensor_ws/src/
cd ~/sensor_ws && colcon build --symlink-install

cp ros_home/waypoints.csv ~/.ros/
cp can_up.sh ~/
```

## 실행

### 0) CAN — 부팅할 때마다 필요하다

```bash
sudo bash ~/can_up.sh
candump can0,211:7FF     # 둘째 바이트 01 = CAN 모드
```

젯슨 온보드 CAN(mttcan)이 부팅 때 `can0` 을 선점하므로, 이 스크립트가 그것을
`canhw0` 으로 밀어내고 USB candleLight 어댑터(1d50:606f)를 `can0` 으로 개명한다.
`scout_mini_description/urdf/scout_mini.urdf.xacro:147` 이 `can0` 을 하드코딩하고
있어서 이름이 반드시 `can0` 이어야 한다. gs_usb 는 JetPack 커널에 없으므로
`~/gs_usb.ko` 를 insmod 한다(스크립트가 알아서 판단).

**리모컨을 끄지 않으면 차가 움직이지 않는다.** `0x211` 의 control_mode 가
`0x03`(REMOTE) 이면 `scout_mini_hardware.cpp:424` 의 헬스 게이트가 닫혀
`health gate is closed; forcing zero command` 로 모든 명령이 0이 된다.

### 1~4) 노드 (터미널 4개)

```bash
# 1) IMU
source ~/competition/integrated_stack_cog_heading_2026-08-04/install/setup.bash
ros2 launch my_nav_pkg imu_receiver_launch.py

# 2) GPS (NTRIP 은 systemd user service: ntrip-ros.service)
source ~/competition/integrated_stack_cog_heading_2026-08-04/install/setup.bash
ros2 launch my_nav_pkg gnss_receiver_launch.py

# 3) 라이다
source ~/sensor_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py

# 4) 주행
source ~/competition/integrated_stack_cog_heading_2026-08-04/install/setup.bash
ros2 launch competition_bringup knu_mission_drive_vision.launch.py   # 신호등 포함
ros2 launch competition_bringup knu_mission_drive.launch.py          # 회피+주차만
```

### 5) bag 녹화 (주행 분석용)

```bash
cd ~/bags && ros2 bag record -o "run_$(date +%Y%m%d_%H%M%S)" \
  /navpvt /fix /rtcm /imu/data /imu/heading_enu /imu/is_calibrated \
  /localization/imu_status /localization/yaw_offset \
  /odometry/local /odometry/global /cmd_vel /cmd_vel/follow \
  /traffic_light/state /control/state /control/mission_state /follower/active_wp \
  /avoid/debug/state /avoid_active /follower/cte /scan \
  /scout_mini/robot_state /scout_mini/hardware_ready \
  /scout_mini/driver_state /scout_mini/motor_state /tf /tf_static /diagnostics
```

`/traffic_light/state`, `/control/state`, `/follower/active_wp`, `/avoid/debug/state`,
`/scan` 은 2026-08-17 에 추가했다. 이게 없으면 신호등/회피 이상을 사후에
확정할 수 없다(08-15 목록에는 빠져 있었다).

## 2026-08-17 변경 이력

### autodrive_pkg 가 아예 설치되지 않고 있었다 (최우선 수정)

`src/autodrive_pkg/autodrive_pkg/__init__.py` 가 없어서 setuptools 가 패키지를
설치하지 못했다. install 에는 `autodrive-pkg.egg-link`(85바이트)만 남고 실제
모듈이 없어, python 이 `~/.bashrc:127` 이 source 하는 **옛 `~/ros2_ws` 의
autodrive_pkg** 로 넘어갔다. 결과:

- `vision_node` 즉사 — 옛 패키지에 `vision.py` 가 없다
  (`ModuleNotFoundError: No module named 'autodrive_pkg.vision'`)
- `mission_control_node` 는 **옛 154줄 `control.py`** 로 실행 — `_signal` 구역
  로직도, 비전 끊김 시 fail-closed 도 없는 정지선 방식 코드다.

2026-08-17 19:35 주행에서 빨간불을 1.50 m/s 로 그대로 통과한 원인이 이것이다.
빈 `__init__.py` 추가 + 재빌드로 해결했다. 검증 방법:

```bash
python3 -c "import autodrive_pkg.vision; print(autodrive_pkg.vision.__file__)"
# 경로에 competition 이 나와야 한다. ros2_ws 가 나오면 아직 옛것이다.
```

### 신호등: 커밋 거리 0.50 -> 0.15 m (`control.py`)

`signal_commit_distance_m`. 초록 확인 후 정지해제는 이 거리를 실제로 주행해야
확정되는데, CommandGuard 가감속 0.60 m/s^2 에서 0.50 m 는 약 1.5초가 필요하다.
실측 초록 검출 지속은 1.18~1.40초뿐이라 4회 해제가 4회 모두 재정지했다
(실이동 0.031~0.101 m). 0.15 m 는 약 0.7초면 확정된다.

### 신호등: 거리 트리거 신설 (`control.py`, `signal_trigger_dist` 기본 1.80 m)

`waypoint_follower_node._stopline_step` 의 `stopline_trigger_dist` 와 같은 형태다.
구역은 이름으로 고르고, 정지 요구 시점은 신호점까지의 거리로 정한다.

종전에는 팔로워가 신호점을 목표로 삼는 순간(=직전 점 통과 시) 정지가 걸려,
정지 위치가 "직전 점 + 제동거리"로 결정됐다. 실측: wp003 통과 0.18 m 지점에서
정지요구 -> 1.62 m 더 가서 정지 -> wp004_signal 보다 0.65 m 앞.
같은 궤적에 새 로직을 얹어 역산하면 신호점 0.12 m 앞이다.

**주의: 이 값은 접근 속도에 묶인다.** SIGNAL 구역 속도 cap(1.50 m/s)을 바꾸면
같이 조정해야 한다. 2.5 m/s 로 들어오면 제동거리가 5 m 를 넘는다.

`_armed_zone()` 은 latch/release/done 이후에는 게이트를 걸지 않는다. 연속된
`_signal` 점 사이에서 zone 이 NAVIGATION 으로 떨어지면
`MissionSafety._reset_after_exit` 가 `signal_done` 을 지워 매 점마다 재정지하기
때문이다. 좌표/위치를 못 믿으면 게이트 미적용(fail-closed).

### 회피: 공식 2개 배치 전용 연속 궤적 슬림화 (`local_avoider_node.py`)

현재 대회용 회피는 `UPPER-LOWER-UPPER` / `LOWER-UPPER-LOWER` 두 배치만
사용한다. 첫 박스 row를 확정하면 세 박스와 원경로 복귀까지 하나의 C² quintic
경로로 잠그고, `0.50~0.70 m/s`에서 곡률과 각속도를 함께 계산한다. 별도
`ObstacleWaypointTarget`, 일반 `REJOIN`, ESTOP, SAFE_CREEP 상태기는 제거했다.

`zone_exit_release_timeout_sec`(기본 3.0초)은 장애물 waypoint 라벨이 먼저 끝난
경우 회피권 고착을 풀기 위한 timeout이다. 장애물 판단 실패 ESTOP은 아니며, 한
주기의 rollout 거부에는 직전 committed trajectory 명령을 유지한다. 노드 또는
명령 발행 자체가 끊겼을 때의 main-controller command timeout은 그대로 남아 있다.

현재 검증 기준은 차량 `1.40 x 0.65 m`, 박스 `0.50 x 0.90 m`, 박스 면 사이
`2.50 m`, 차선 분리선 이격 `0 m`, 회피 상한 `0.70 m/s`다. 상세 결과는
`SLALOM_SLIM_REFACTOR_VALIDATION.md`를 본다.

### 후진주차: 옆거리 밴드 + 후진 상한 상대화 (`parking_controller_node.py`)

08-16 작업분을 가져왔다. 회피 쪽은 건드리지 않았다.

```
row_near_dist   1.0 -> 1.2      # 이 안이면 "박스 옆"으로 인정
row_hold_min    (신규) 0.6      ┐ 허용 밴드. 이 안이면 무보정
row_hold_max    (신규) 1.0      ┘
row_hold_dist   0.75            # 미사용, 기존 yaml 호환용으로 선언만 유지
MAX_REVERSE_TRAVEL       1.50 -> 2.00
REVERSE_OVERSHOOT_LIMIT  (신규) 0.10
```

**옆거리 보정을 밴드로 바꿨다.** 예전에는 단일 임계값 0.75 에서 "가까우면
밀어내기만" 하는 편측 보정이라, 0.75 보다 가깝게 찍힌 경로는 실효 옆거리가
결국 0.75 로 밀려나고 그 과정에서 차가 비스듬히 달렸다(옆거리 0.5 m 에서
정상상태 편향 8.6도). 밴드로 두면 0.6~1.0 어디에 찍어도 그대로 유지된다.
Gazebo 검증에서 옆거리 0.60 / 0.70 / 1.00 의 폭오차가 모두 +28 mm 로 같았다.

끌어당김(너무 멀 때)은 `row_near_dist` 안에서만 건다. 갭 구간은 측면이 빈
공간이라 제한이 없으면 갭에서 차를 옆으로 끌어당겨 통과선이 무너진다.

**후진 상한을 목표 대비 상대값으로 바꿨다.** 후진거리 = 0.70 + 옆거리 인데
고정 상한 1.50 이면 옆거리 1.0 m(=후진 1.70 m)에서 목표에 닿기도 전에 상한이
먼저 걸려 얕게 주차됐다. 이제 목표를 아는 동안에는 `목표 + 0.10 m` 를 쓰고,
라이다 측정이 실패해 목표를 모를 때만 `MAX_REVERSE_TRAVEL` 을 절대 상한으로
쓴다.

## 알려진 미해결 사항

1. **초록 검출률이 낮다 (최대 과제).** 2026-08-17 19:59 주행 실측:
   UNKNOWN 96.1% / RED 2.4% / GREEN 1.5%. 초록이 한 번에 1.18~1.40초밖에
   안 붙는다. 커밋 거리를 0.15로 낮춰 당장은 출발하겠지만, 대회에서 초록을
   잠깐 놓치면 또 멈춘다. 카메라 화각/거리/역광 점검이 필요하다.

2. **회피 고착의 근본 원인 미확정.** 08-17 20:49 주행에서 마지막 장애물 점
   진입 후 14.5초를 더 붙잡았는데, 스캔 여유 조건으로는 설명되지 않았다
   (노드의 `scan_to_points` 로 재현해 측방 임계를 0.50까지 낮춰도 릴리즈
   시점 동일). 남은 후보는 `confirmed_ahead`(트래커가 박스를 "통과함"으로
   안 지움) 과 블라인드존 메모리(TTL 60초/6 m)인데, **둘 다 토픽으로 안 나가
   bag 에 안 남는다.** 진단하려면 트래커 상태를 발행하거나
   `_ready_to_rejoin` 에 차단 사유 로그를 넣어야 한다.
   21:10 주행에서는 구간 이탈 0.2초 후 정상 복귀해 재현되지 않았다.

3. **CAN 설정이 재부팅으로 초기화된다.** `can_up.sh` 를 매번 실행해야 한다.
   부팅 자동화(mttcan 블랙리스트 + gs_usb 자동 로드)는 아직 안 했다.

4. **ENU 원점이 코스에서 15.1 km 떨어져 있다.** launch ORIGIN
   35.8245366/128.7539002 vs 코스 35.888/128.606. 주행 시작 직후 1회
   15 km 점프가 관측된다(08-17 21:10 주행 1.8초 지점). 웨이포인트끼리는 같은
   프레임을 공유해 주행 자체에는 영향이 없었다.

5. **주차는 실차에서 슬롯 진입까지 성공한 적이 없다.**

6. 신호등 정지/재출발은 **08-17 변경 이후 실차 재검증 전이다.** 커밋 거리
   0.15 와 거리 트리거 1.80 둘 다 bag 역산으로만 확인했다.

7. USB 재열거가 잦다. 프로세스는 살아 있는데 발행만 멈추므로
   `ros2 topic hz` 로 확인해야 한다.

8. IMU 런치를 두 번 띄우면 phidget_container 가 중복되고 `/navpvt` 가 절반
   거부된다. GPS 노드를 재시작하면 NTRIP 도 같이 재시작할 것
   (`systemctl --user restart ntrip-ros.service`).

## 테스트

```bash
cd ~/competition/integrated_stack_cog_heading_2026-08-04
colcon test --packages-select my_nav_pkg autodrive_pkg competition_bringup
colcon test-result --all
```
현재 257개 전부 통과 (my_nav_pkg 238 / autodrive_pkg 14 / competition_bringup 5).

테스트는 yaml 을 텍스트로 읽어 플래너를 직접 생성하므로 노드를 띄우지 않는다.
설정을 바꾼 뒤에는 노드 기동까지 반드시 확인할 것.
```bash
ros2 run my_nav_pkg local_avoider_node --ros-args --params-file <yaml>
```

## 되돌리기

08-17 변경분의 원본 백업과 복원 절차는 `~/restore/README.md` 에 있다.
- `competition_control_signal_commit.py.orig` — control.py (신호등 변경 2건 이전)
- `competition_local_avoider_node.py.orig` — local_avoider_node.py (강제 해제 이전)
- `competition_local_avoider.yaml.orig` — local_avoider.yaml
