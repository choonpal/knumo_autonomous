# 공식 2개 장애물 배치용 0.50–0.70 m/s 연속 궤적 회피 수정

## 기준과 해석

- 원본: `scout_stack_20260817.zip`
- 박스 사이 2.50 m는 **박스 면과 면 사이의 빈 간격**으로 반영했다.
- 박스 진행방향 깊이가 0.50 m이므로 궤적 knot에 사용하는 박스 중심 간격은 `2.50 + 0.50 = 3.00 m`이다.
- 장애물 공식 배치 두 가지인 `UPPER-LOWER-UPPER`와 `LOWER-UPPER-LOWER`를 대상으로 한다.
- 기존 waypoint recorder, waypoint CSV 형식, global spline 및 Pure Pursuit 흐름은 변경하지 않았다. 장애물 구간은 기존대로 `o` waypoint로 기록한다.

## 회피 알고리즘 변경

기존 패턴 분류와 장애물 tracker는 유지하고, 패턴이 확정된 뒤의 제어를 다음 구조로 바꿨다.

```text
LiDAR 첫 박스 검출
→ ULU/LUL 패턴 고정
→ 현재 경로좌표계에서 C² quintic S-trajectory 생성
→ d, d', d''로 reference heading/curvature 계산
→ 곡률·횡가속도·각속도 한계로 0.50–0.70 m/s 속도 결정
→ ω = vκ feed-forward + heading/lateral error feedback
→ 회전된 1.20 × 0.65 m 차체를 2.50 m 전방까지 rollout 검증
→ 세 박스 통과 후 같은 C² 궤적으로 d=0 global path 재합류
```

패턴 통과선은 현재 측정된 두 줄 중심에 대해 다음처럼 균형화했다.

```yaml
pattern_slalom_upper_pass_lateral: 0.00
pattern_slalom_lower_pass_lateral: 1.25
```

두 통과선 모두 인접 박스 중심과 명목상 1.075 m 떨어진다. 설정된 차량 폭·박스 폭·안전 간격·tracking margin으로 요구되는 0.900 m보다 0.175 m 여유가 있다.

## 속도와 각속도

```yaml
pattern_slalom_min_speed: 0.50
pattern_slalom_max_speed: 0.70
pattern_slalom_w_max: 1.00
pattern_slalom_yaw_response_gain: 1.31
pattern_slalom_lateral_accel_limit: 0.60
pattern_slalom_verify_distance: 2.50
rate_hz: 20.0
```

- 패턴 확정 전 follower 접근속도는 0.50 m/s이다.
- 패턴 확정 뒤에는 0.50–0.70 m/s 범위에서 곡률과 추종 오차를 반영한다.
- 일반 VFH fallback의 기존 `v_max=0.54`, `w_max=0.80`은 다른 모드와 패턴 확정 전 호환성을 위해 유지했다.
- main controller의 회피 상한은 `0.70 m/s`, 각속도 상한은 `1.00 rad/s`로 맞췄다.

## ESTOP 정책

`estop_enabled: false`를 유지했다. 장애물 회피 궤적의 한 주기 rollout이 거부됐다고 새 ESTOP 상태나 정지 상태기를 만들지 않는다. 직전 committed trajectory 명령을 유지하면서 다음 주기에 다시 검증한다.

단, 노드 자체가 명령 발행을 중단했을 때의 기존 main-controller `command_timeout`은 통신 계약이므로 삭제하지 않았다. 또한 fail-open 정책은 계획 실패 시 전진을 이어가므로 안전을 보장하는 정책이 아니라 대회 완주 우선 정책이다.

## 수정 파일

- `competition_src/my_nav_pkg/my_nav_pkg/vfh_core.py`
- `competition_src/my_nav_pkg/my_nav_pkg/local_avoider_node.py`
- `competition_src/my_nav_pkg/config/local_avoider.yaml`
- `competition_src/competition_bringup/launch/knu_waypoint_drive.launch.py`
- `competition_src/my_nav_pkg/test/test_vfh_core.py`
- `competition_src/my_nav_pkg/test/test_vfh_closed_loop.py`
- `competition_src/my_nav_pkg/test/test_bundle_contract.py`
- `competition_src/my_nav_pkg/tools/validate_pattern_slalom_070.py`
- `competition_src/my_nav_pkg/validation/*`

## 자동 테스트 결과

```text
240 passed, 66 subtests passed
```

2D 검증은 200° LiDAR FOV, scan 10 Hz, controller 20 Hz, 회전된 직사각형 footprint, 비대칭 도로 경계, actuator lag와 yaw response 오차를 포함한다.

| 그룹 | 시나리오 | 성공 | 최소 inflated 박스 여유 | 최소 도로 여유 | 평균 명령속도 |
|---|---:|---:|---:|---:|---:|
| 명목 두 배치 | 2 | 2 | 0.198 m | 0.176 m | 0.680 m/s |
| 정확 배치 + 차량/추정 오차 | 60 | 60 | 0.159 m | 0.127 m | 0.674 m/s |
| 박스 위치 random stress | 60 | 60 | 0.118 m | 0.104 m | 0.666 m/s |
| 결정론적 boundary stress | 36 | 36 | 0.051 m | 0.064 m | 0.653 m/s |

총 158개 구성 시나리오에서 158개가 통과했다. 이 수치는 해당 시뮬레이션 집합의 통과율이지 실차 성공확률은 아니다.

## 한계 탐색 결과

가혹한 차량 지연·초기오차 프로파일을 함께 준 별도 탐색에서는 다음 경계가 나타났다.

- 박스가 각 통과선 쪽으로 8 cm 이동: 선택된 4개 가혹 시나리오 모두 통과, 최소 inflated 여유 약 1.3 cm.
- 박스가 각 통과선 쪽으로 10 cm 이동: 4개 중 1개 inflated 충돌.
- 실제 도로/waypoint frame이 설정 대비 불리한 방향으로 6 cm 이동: 모두 통과, 최소 도로 여유 약 0.5 cm.
- 같은 오차가 8 cm이면 4개 중 2개 corridor 이탈.
- 설정된 yaw gain 1.31과 달리 실제 gain이 0.75까지 낮아진 가혹 조건은 통과했지만 도로 여유가 약 0.2 cm까지 감소했다. 0.70에서는 corridor 이탈이 발생했다.

따라서 0.70 m/s 투입 전에 가장 중요한 실측은 박스 횡위치, waypoint 기준 도로 좌우폭, 실제 yaw-response gain이다.

## 실차 단계

1. 공식 두 배치를 각각 0.50 m/s로 연속 5회 통과한다.
2. 같은 pass line과 controller gain을 유지한 채 0.60 m/s로 각각 5회 통과한다.
3. 접촉·경계 이탈·반복 planner rejection이 모두 없을 때만 0.70 m/s로 각각 10회 시험한다.
4. `cmd_vel`, gyro yaw rate, control pose, pattern lateral error, LiDAR track와 road/box 최소여유를 rosbag으로 기록한다.
5. 실제 `gyro yaw-rate / commanded yaw-rate`가 1.31과 크게 다르면 속도보다 먼저 `pattern_slalom_yaw_response_gain`을 다시 맞춘다.

## 2026-08-21 geometry correction and curvature preview

- Vehicle envelope length was corrected from `1.20 m` to the measured `1.40 m`
  in local avoidance, parking configuration, planner defaults, and validation.
- The nominal two-layout validation uses the harder lane-divider-flush placement:
  the `0.90 m` boxes have centres `1.075 m` and `0.175 m` around divider
  `d=0.625 m`. No extra `0.20 m` outward gap is assumed.
- Added `pattern_slalom_curvature_preview: 0.10`. Only feed-forward curvature is
  read 10 cm ahead; lateral and heading errors remain at the current path
  station, avoiding the corner-cutting caused by moving the whole target ahead.
- `estop_enabled: false` remains unchanged. No obstacle-planning failure ESTOP
  or new stop latch was added.

