# 공식 두 배치 장애물 회피 슬림화 및 재검증

## 기준

- 입력 기준: `scout_stack_20260817_slalom070_flush140_preview010.zip`
- 차량 footprint: `1.40 × 0.65 m`
- 박스: `0.50 × 0.90 m`
- 박스 면 사이 빈 간격: `2.50 m` → 중심 간격 `3.00 m`
- 박스와 차선 분리선 사이 추가 간격: `0 m`
- 공식 배치: `UPPER-LOWER-UPPER`, `LOWER-UPPER-LOWER`
- 패턴 잠금 후 속도: `0.50–0.70 m/s`
- 각속도 상한: `1.00 rad/s`
- 곡률 preview: `0.10 m`

Waypoint recorder, waypoint CSV 형식, `s/o/w/p` 기록 방식, global spline 및
일반 Pure Pursuit는 수정하지 않았다.

## 제거한 생산 코드

- 기존 박스별 `ObstacleWaypointTarget`
- 별도 `SCurveRejoin` 및 일반 REJOIN 상태기
- 장애물 판단용 ESTOP / EMERGENCY_STOP / SAFE_CREEP / PARALLEL_ALIGN
- 동적 ESTOP 거리 및 transition 검증 경로
- 위 기능만을 위한 ROS 파라미터와 중복 reset/publish 분기

공식 패턴 잠금 전의 짧은 VFH 명령 생성, ULU/LUL 분류, obstacle tracker,
200° LiDAR blind-zone memory, C² quintic reference, 곡률 연동 제어,
1.40 × 0.65 m 회전 footprint rollout, 비대칭 도로 경계 검사는 유지했다.

장애물 경로 rollout이 한 주기 거부되면 정지하지 않고 직전 committed trajectory
명령을 이어간다. 다만 노드나 `/cmd_vel/avoid` 발행 자체가 끊겼을 때의 기존
main-controller command timeout은 유지한다.

## 코드 규모

| 파일 | 기존 | 슬림화 | 감소 |
|---|---:|---:|---:|
| `vfh_core.py` | 2,982 | 2,188 | 794 |
| `local_avoider_node.py` | 2,497 | 1,258 | 1,239 |
| `local_avoider.yaml` | 422 | 242 | 180 |
| **합계** | **5,901** | **3,688** | **2,213 (37.5%)** |

삭제된 기능의 테스트도 함께 제거했으므로 전체 pytest 개수는 기존
`243 passed, 68 subtests`에서 `221 passed, 59 subtests`로 줄었다. 남아 있는
테스트는 전부 통과했다.

## 성능 보존 비교

같은 seed와 같은 78개 2D 시나리오를 기존판과 슬림판에 각각 실행했다.

- 모든 그룹의 성공/실패 개수 동일
- 평균 명령속도 차이: 최대 약 `0.00010 m/s`
- 최소 inflated 장애물 여유 차이: 최대 약 `0.00009 m`
- 최소 도로 여유 차이: 최대 약 `0.00039 m`

즉 삭제한 부분은 현재 공식 패턴의 핵심 궤적 계산이 아니며, 남겨 둔
`PatternSlalomTarget`과 `plan_pattern_trajectory()`의 AST는 기존판과 동일하다.

## 확대 2D 재검증

4개 seed에서 무작위 시나리오를 나눠 실행하고, 명목/결정론적 경계조건은 중복 없이
합쳐 총 518개 서로 다른 실행조건을 검증했다.

| 그룹 | 시나리오 | 성공 | 최소 inflated 박스 여유 | 최소 도로 여유 | 평균 명령속도 |
|---|---:|---:|---:|---:|---:|
| 명목 공식 두 배치 | 2 | 2 | 0.192 m | 0.163 m | 0.693 m/s |
| 정확 배치 + 차량/추정 오차 | 240 | 240 | 0.143 m | 0.099 m | 0.682 m/s |
| 박스 위치 변동 포함 | 240 | 240 | 0.083 m | 0.081 m | 0.677 m/s |
| 결정론적 경계 stress | 36 | 36 | 0.058 m | 0.057 m | 0.661 m/s |
| **합계** | **518** | **518** |  |  |  |

포함 조건은 200° LiDAR, scan 10 Hz, 제어 20 Hz, 초기 횡/yaw 오차,
실제 yaw-response 변화, 선·각속도 응답지연, 0–100 ms 명령지연, 속도 scale,
pose bias, 박스 종방향 ±0.08 m 및 횡방향 ±0.04 m 변동이다.

## ROS 노드 경로 smoke test

ROS 메시지/Node API를 mock하여 실제 슬림 `LocalAvoider`의 다음 흐름도 직접 실행했다.

```text
FOLLOW → trigger confirm → VFH_ZONE → ULU/LUL lock
→ /avoid_active=true → 세 박스 통과 → d=0 복귀
→ /avoid_active=false, FOLLOW
```

두 배치 모두 inflated 충돌 0회, 도로 경계 이탈 0회로 완료했다. 이 검사는
리팩터링한 ROS 상태 연결의 누락을 찾기 위한 offline smoke test다.

## 판단과 한계

현재 모델과 설정 안에서는 이 차량이 두 공식 배치를 `0.50–0.70 m/s`로 회피할 수
있다는 결과다. 하지만 이는 실차 성공확률이 아니다. 이 실행환경에는 ROS 2/colcon과
실제 Scout Mini가 없어 실차 CAN, timestamp, 타이어 scrub, 노면 마찰, 실제 yaw gain,
LiDAR cluster bias, EKF 횡/yaw 오차를 직접 검증하지 못했다.

가혹조건의 남은 최소 여유가 약 5.7–5.8 cm이므로 여러 실차 오차가 같은 방향으로
누적되면 실패할 수 있다. 실차에서는 같은 경로와 gain을 유지하고 `0.50 → 0.60 →
0.70 m/s` 순으로 두 배치를 반복하며 `/cmd_vel`, gyro yaw rate, pose, `/scan`,
패턴 횡오차를 rosbag으로 확인해야 한다.
