"""KNU EKF -> spline -> Pure Pursuit 주행 스택 (+ 선택적 미션 스택).

mission 인자로 두 가지 구성을 낸다. 실측 캘리브레이션 상수(ORIGIN_*, IMU_*,
GPS_*, YAW_*, WHEEL_LINEAR_SCALE)를 한 파일에만 두기 위해 launch를 나누지
않고 인자로 분기한다 - 파일을 복제하면 차량 재튜닝 때 한쪽만 갱신되어
헤딩/스케일이 조용히 어긋난다.

  mission:=false (기본, 기존 동작 그대로)
      waypoint_follower -> /cmd_vel  (직결, 중재자 없음)

  mission:=true (장애물 회피 + 후진주차 포함)
      waypoint_follower  -> /cmd_vel/follow   ┐
      local_avoider      -> /cmd_vel/avoid    ├-> main_controller -> /cmd_vel
      parking_controller -> /cmd_vel/parking  ┘

  명령 소스가 셋이라 중재자(main_controller)가 반드시 필요하다. 그래서 이
  구성에서는 follower의 /cmd_vel 직결 리맵을 쓰지 않는다.

  주의: main_controller가 경로에 들어오면 CommandGuard의 상한과 slew 제한,
  그리고 남아 있는 인터락(mission_stop, require_speed_cap, heartbeat 타임아웃)이
  다시 적용된다. 속도 상한 자체는 현재 follower 설정(linear 0.10 < cap 0.15,
  angular 0.25 < cap 0.80)에 걸리지 않지만 slew(0.30 m/s^2, 1.50 rad/s^2)는
  적용되므로 가감속 감각이 달라진다. 처음 주행 시 config/local_avoider.yaml의
  main_controller_node 값부터 확인할 것.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# ENU 로컬 좌표계의 기준점. EKF/GNSS 변환/follower가 모두 이 값을 공유하므로
# 코스를 옮기면 반드시 같이 갱신해야 한다. 원점에서 멀수록 좌표값이 커지고
# 평면 근사 오차가 붙는다.
#
# 2026-08-09: 대구 성서(35.888/128.607) -> 현재 코스로 갱신.
# 이전 값은 새 웨이포인트에서 15.0 km 떨어져 있었고(동 +13.3 km, 북 -7.1 km),
# 그 상태로 띄우니 EKF 수렴 과정에서 위치가 크게 튀어 캘리브가 계속
# "position jump detected, restarting segment" 로 리셋됐다(6회 연속).
# 값은 waypoints.csv 첫 점(wp001)이다.
ORIGIN_LAT = 35.8245366
ORIGIN_LON = 128.7539002
ORIGIN_ALT = 88.665

IMU_SERIAL = 807529
IMU_X = 0.124
IMU_Y = 0.0
IMU_Z = 0.3765
GPS_X = 0.0
GPS_Y = 0.0
GPS_Z = 0.420

YAW_SIGN = -1.0
YAW_OFFSET_RAD = -1.4197  # 2026-07-31 GNSS 4방향 재측정 (구 1.764222는 ~180° 어긋남). 방향편차 24°(자력계 잔여왜곡) 남음 → 자력계 재보정 시 갱신
# 자력계 절대헤딩의 실측 오차는 방향에 따라 18~20도 남았다. 이 covariance는
# heading_selector가 첫 RTK COG를 얻기 전 초기화 구간에서만 사용한다. 첫 COG lock
# 이후에는 자력계 yaw를 Global EKF에 다시 넣지 않고, 코너는 gyro-z, 직선은 raw
# GPS ENU 1.5m COG가 절대 yaw를 담당한다.
YAW_VARIANCE_RAD2 = 0.12
WHEEL_LINEAR_SCALE = 1.01595


def _config_file(package: str, filename: str) -> str:
    return os.path.join(
        get_package_share_directory(package),
        'config',
        filename,
    )


def _launch_file(package: str, filename: str) -> str:
    return os.path.join(
        get_package_share_directory(package),
        'launch',
        filename,
    )


def generate_launch_description() -> LaunchDescription:
    mission = LaunchConfiguration('mission')
    dry_run = LaunchConfiguration('dry_run')
    # mission=false면 예전처럼 follower를 /cmd_vel에 직결하고, true면
    # /cmd_vel/follow 그대로 두어 main_controller가 중재하게 한다
    # (자기 자신으로의 리맵은 무해하다).
    follow_out = PythonExpression(
        ["'/cmd_vel/follow' if '", mission, "' == 'true' else '/cmd_vel'"]
    )

    bringup_share = get_package_share_directory('competition_bringup')
    scout_config = os.path.join(
        bringup_share,
        'config',
        'scout_mini_waypoint_ekf.yaml',
    )

    scout = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('scout_mini_base', 'base_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'enable_drive': 'true',
            'robot_config_dir': scout_config,
        }.items(),
    )

    # GPS는 기본적으로 별도 터미널에서 gnss_receiver_launch.py 로 먼저 띄운다
    # (RTK를 Fixed까지 수렴시킨 뒤 주행을 시작하기 위해서).
    # 한 번에 같이 띄우려면 run_gnss:=true 로 실행한다.
    gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('my_nav_pkg', 'gnss_receiver_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('run_gnss')),
        launch_arguments={
            'device': '/dev/ublox',
            'provision_usb_rtcm3': 'true',
        }.items(),
    )

    # IMU도 GPS와 같이 기본적으로 별도 터미널에서 띄운다
    # (imu_receiver_launch.py). 주행 스택을 재시작해도 IMU 캘리브/헤딩 기준이
    # 새로 잡히지 않게 하기 위해서다. 한 번에 같이 띄우려면 run_imu:=true.
    phidget = ComposableNodeContainer(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        condition=IfCondition(LaunchConfiguration('run_imu')),
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial_node',
                parameters=[{
                    'serial': IMU_SERIAL,
                    'hub_port': 0,
                    'frame_id': 'imu_link',
                    'use_orientation': True,
                    'spatial_algorithm': 'ahrs',
                    'publish_rate': 0.0,
                    # 자력계 하드아이언 보정 (2026-07-26 측정). 차체 자화(46.5uT)로
                    # 인한 헤딩 왜곡(회전후 드리프트 55.9->0.22도) 제거 → 출발 헤딩 안정.
                    # 차체 구성(자석 등) 바뀌면 반드시 재측정.
                    'cc_mag_field': 0.534836,
                    'cc_offset0': 0.153731,
                    'cc_offset1': 0.459185,
                    'cc_offset2': 0.130169,
                    'cc_gain0': 1.869731,
                    'cc_gain1': 1.869731,
                    'cc_gain2': 1.869731,
                    'cc_t0': 0.0, 'cc_t1': 0.0, 'cc_t2': 0.0,
                    'cc_t3': 0.0, 'cc_t4': 0.0, 'cc_t5': 0.0,
                }],
                remappings=[
                    ('imu/data_raw', '/imu/data'),
                    ('imu/is_calibrated', '/imu/is_calibrated'),
                ],
            ),
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('my_nav_pkg', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'run_global': 'true',
            'publish_tf': 'true',
            'publish_sensor_static_tf': 'true',
            'raw_odom_topic': '/scout_mini_base_controller/odom',
            'imu_topic': '/imu/data',
            'imu_frame': 'imu_link',
            'imu_x_m': str(IMU_X),
            'imu_y_m': str(IMU_Y),
            'imu_z_m': str(IMU_Z),
            'gps_x_m': str(GPS_X),
            'gps_y_m': str(GPS_Y),
            'gps_z_m': str(GPS_Z),
            'yaw_sign': str(YAW_SIGN),
            'yaw_offset_rad': str(YAW_OFFSET_RAD),
            'yaw_variance_rad2': str(YAW_VARIANCE_RAD2),
            'wheel_linear_scale': str(WHEEL_LINEAR_SCALE),
            'origin_lat': str(ORIGIN_LAT),
            'origin_lon': str(ORIGIN_LON),
            'origin_alt': str(ORIGIN_ALT),
            'antenna_lever_arm_verified': 'true',
            'required_rtk': 'float_or_fixed',
            'max_h_acc_m': '0.05',
            # RTK Float도 COG 헤딩에 쓴다. Float 위치 공분산은 h_acc와 무관하게
            # 0.102m std로 깔리므로 0.05로 두면 COG가 한 번도 안 걸린다
            # (2026-08-05 주행: COG_STRAIGHT 0%, 26초 내내 자력계).
            'cog_max_position_std_m': '0.12',
        }.items(),
    )

    follower = Node(
        package='my_nav_pkg',
        executable='waypoint_follower_node',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'file_name': 'waypoints.csv',
            'csv_mode': 'llh',
            'origin_lat': ORIGIN_LAT,
            'origin_lon': ORIGIN_LON,
            'origin_alt': ORIGIN_ALT,
            'odom_topic': '/odometry/global',
            'frame_id': 'map',
            'linear_speed': 2.50,          # 시범주행 요청 (검증된 값은 0.35)
            'vision_mission_speed': 1.50,
            # Pattern lock 전에는 0.50m/s로 접근한다. Lock 뒤에는 local
            # avoider가 연속 quintic 곡률을 따라 0.50~0.70m/s를 직접 결정한다.
            # mission:=false에서는 기존 linear_speed 2.50m/s를 유지한다.
            'obstacle_mission_speed': ParameterValue(
                PythonExpression(
                    ["0.50 if '", mission, "' == 'true' else 2.50"]
                ),
                value_type=float,
            ),
            'parking_mission_speed': 0.40,
            # 2026-08-08: 0.35 -> 0.80.
            # 0.35 rad/s는 좌우 바퀴 속도차 0.17 m/s(바퀴당 8.6 cm/s)라
            # 스키드 스티어가 정지 마찰을 못 이긴다. ROTATE는 선속도를 0으로
            # 두고 각속도만 내는데, 실측에서 0.350 rad/s를 130초간 명령했는데
            # 실제 각속도가 0.0014 rad/s(명령의 0.4%)로 전혀 돌지 못했다.
            # 헤딩이 튀어 ROTATE에 들어가면 빠져나올 수 없는 교착이 된다
            # (차가 안 돌면 COG 갱신 불가 -> 헤딩 그대로 -> ROTATE 유지).
            # main_controller angular_cap 0.80, 섀시 상한 1.0 안에 든다.
            'angular_speed': 0.80,
            'target_tolerance': 0.30,
            # 흰 정지선 앞 3초 정차. 리코더의 'w' 키로 찍은 웨이포인트
            # (이름에 _stopline)에 이 거리까지 오면 그 자리에서 정차한다.
            # 규정이 "완전히 멈춘 상태로 3초"라, 측정 속도가 stopped_speed
            # 아래로 내려간 뒤부터 3초를 센다(감속 시간은 포함하지 않는다).
            # 미리 감속하지 않고 'w'로 찍은 지점에서 바로 0을 낸다(급정지).
            # 주의: mission:=true 에서는 CommandGuard linear_slew_rate(0.60)가
            # 걸려 1.5m/s 접근 시 제동거리가 약 1.9m다. 즉 실제 정차 위치는
            # 찍은 점보다 그만큼 앞이 된다. 흰 선에 맞추려면 'w'를 그만큼
            # 앞에서 찍거나 slew를 올려야 한다.
            'stopline_hold_sec': 3.0,
            # 2026-08-08: 0.50 -> 1.30. 이 반경에 들어오면 0을 내지만 슬루
            # 0.60 m/s^2 때문에 관성으로 더 나간다. 접근 2.5m/s에서 세 번의
            # 주행 모두 0.75~0.86m 오버슈트했다(재현성 높음). 0.50 + 0.80 으로
            # 잡는다. 접근 속도가 크게 낮아지면 이번엔 반대로 선 앞에서 서므로
            # 그때는 다시 줄일 것.
            'stopline_trigger_dist': 1.30,
            'stopline_stopped_speed': 0.03,
            'stopline_settle_timeout_sec': 5.0,
            'path_resolution': 0.05,
            'lookahead_min': 0.40,        # 저속 하한 (너무 짧으면 직진 진동)
            'lookahead_max': 2.20,        # 직선(1.5m/s)에서 조향 진동 방지용 상한
            # 코너에서 lookahead가 회전반경(이 코스 최소 2.3m)에 가까우면 안쪽으로
            # 질러가 경로를 벗어난다. gain을 낮춰 감속된 코너 구간에서 lookahead도
            # 같이 짧아지게 한다(0.5m/s에서 0.75m).
            'lookahead_gain': 1.5,
            # 곡률 큰 구간에서 자동 감속시키는 핵심 파라미터. 허용 횡가속도[m/s^2]로,
            # v_max = sqrt(a_lat_max * r) 로 속도를 거꾸로 정한다.
            #
            # 2026-08-08: 0.10 -> 0.50.
            # 여기 들어가는 곡률은 경로 곡률(max_kappa_ahead)뿐 아니라 Pure Pursuit의
            # 조향 명령 곡률(2*sin(alpha)/Ld)도 포함한다. 0.10에서는 2.5m/s를 유지하는
            # 조건이 alpha < 1.0도라, 경로가 직선이어도 차가 1도만 틀어지면 즉시
            # 감속이 걸렸다. 실제로 직선 구간에서 명령이 2.50 <-> 1.50 을 계속
            # 오갔다(2026-08-08 주행).
            # 실측 근거: 반경 3.0m 코너를 1.53m/s로 돌 때 횡가속 0.79m/s^2였고
            # CTE는 7.8cm에 그쳤다. 즉 차량이 감당하는 값은 0.10의 8배다.
            # 0.50은 그 실측보다 여전히 보수적이면서, 2.5m/s 유지 조건을
            # alpha < 2.3도로 완화한다.
            'a_lat_max': 0.50,
            # 주행 하한(2026-08-05 요청: 최저 1.5 / 최고 2.5). 이 값이면
            # sqrt(a_lat_max*r) = 1.5가 되는 반경 22.5m보다 급한 코너는 전부
            # 하한에 걸리므로, 이 코스(최소 반경 2.3m)에서는 a_lat_max 감속이
            # 사실상 동작하지 않는다. 반경 2.3m 코너의 횡가속은 0.98 m/s^2로
            # a_lat_max(0.10)의 약 10배. 코너 이탈이 보이면 여기부터 되돌린다.
            # 주차 모드는 clamp가 min(min_drive_speed, active_speed)라
            # 0.15가 그대로 유지된다.
            'min_drive_speed': 1.50,
            'rotate_enter_deg': 60.0,
            'rotate_exit_deg': 20.0,
            'path_back_window': 2.0,
            'path_fwd_window': 5.0,
            # IMU는 imu_receiver_launch.py 로 미리 띄워 수렴시킨 뒤 주행하므로
            # 주행 시작 시 추가 대기는 두지 않는다(0). 한 터미널에서 run_imu:=true 로
            # 함께 띄울 때는 12.0 정도로 되돌릴 것.
            'imu_settle_sec': 0.0,
            # 출발 직진은 한 번만 수행한다. 이 구간에서 heading_selector가 raw
            # GPS ENU 1.5m COG를 여러 번 발행해 Global EKF의 절대 yaw를 초기화한다.
            # 기존 offset 계산은 COG lock이 실패할 때의 자력계 fallback으로 남긴다.
            'gps_calib_enabled': True,
            # 2026-08-05: 8.0m/0.7m/s -> 15.0m/2.5m/s.
            # 직선 잔차 RMS는 4~5cm로 속도와 무관한 GPS/EKF 노이즈이므로
            # 헤딩 정밀도는 베이스라인이 결정한다(sigma ~= RMS/거리).
            # 실측: 6m -> 0.376deg, 8m -> 0.301deg. 15m면 0.16deg 수준.
            # 소요 6초, 샘플 ~170개로 gps_calib_min_samples(20) 대비 여유.
            # 가속 전이는 0.7 도달에 0.44s/0.15m였고 2.5도 2m 안쪽이라
            # 15m 중 오염 구간은 13% 이하.
            # 주의 1: 출발점 앞에 직선 15m가 확보되어야 한다.
            # 주의 2: 이 구간은 헤딩이 아직 확정되기 전의 개루프 직진이다.
            #         무보정 주행이므로 진행 방향에 장애물이 없는지 반드시
            #         확인하고 출발한다.
            #
            # 2026-08-08: 15.0 -> 8.0.
            # 개루프라 차량 방향이 경로와 조금만 어긋나도 그 거리에 비례해
            # 코스를 벗어난 채 주행이 시작된다(실측 출발 CTE 최대 1.10m).
            # 8m면 같은 각도 오차에서 이탈이 절반 가까이 준다.
            # 정확도 sigma ~ 횡편차RMS / 거리 이므로 8m에서는 실측 RMS
            # 2.5~5.6cm 기준 sigma 0.18~0.40도. 잡으려는 오차(자력계 18~20도,
            # COG 미확보 시 offset 편차 13도)에 비하면 충분하다.
            'gps_calib_distance': 8.0,
            # 2026-08-08: 2.5 -> 1.5. 섀시 리밋이 1.5였을 때 캘리브도 실제로는
            # 1.5로 달렸고(실측 wheel 1.53 / GPS 1.52), 그 조건에서 횡편차 RMS
            # 5.46cm였다. 리밋을 2.5로 푼 뒤 캘리브까지 빨라지면 개루프 직진의
            # S자가 더 커진다(0.7 -> 1.5에서 이미 RMS 1.7배). 검증된 조건을
            # 유지하기 위해 캘리브만 1.5로 고정한다.
            'gps_calib_speed': 1.5,
            # imu_enu_adapter가 쓰는 고정 offset. 캘리브 결과를 여기에 더해
            # 절대 offset으로 만들어 어댑터에 돌려준다(같은 값을 써야 한다).
            'configured_yaw_offset_rad': YAW_OFFSET_RAD,
        }],
        remappings=[('/cmd_vel/follow', follow_out)],
    )

    avoider_config = _config_file('my_nav_pkg', 'local_avoider.yaml')
    mission_if = IfCondition(mission)

    local_avoider = Node(
        package='my_nav_pkg',
        executable='local_avoider_node',
        name='local_avoider_node',
        output='screen',
        condition=mission_if,
        parameters=[avoider_config, {'odom_topic': '/odometry/global'}],
    )

    parking_controller = Node(
        package='my_nav_pkg',
        executable='parking_controller_node',
        name='parking_controller_node',
        output='screen',
        condition=mission_if,
        # spec_confirmed=false인 동안 이 노드는 어떤 명령도 내지 않는다.
        # 실차 규격(lidar_offset_x/vehicle_length/slot_on_left 등)을 넣고
        # config에서 spec_confirmed를 true로 바꿔야 동작한다.
        parameters=[avoider_config, {'odom_topic': '/odometry/global'}],
    )

    main_controller = Node(
        package='my_nav_pkg',
        executable='main_controller_node',
        name='main_controller_node',
        output='screen',
        condition=mission_if,
        # config의 dry_run 기본값은 true(벤치 테스트용)라 그대로 두면
        # main_controller가 /cmd_vel을 발행하지 않아 차가 전혀 움직이지
        # 않는다. 주행 전용 구성에서는 follower가 /cmd_vel로 직결이라
        # 이 값이 무관했지만, selector가 경로에 들어오는 mission 구성에서는
        # 하드 블로커가 된다. 그래서 여기서 명시적으로 덮어쓴다.
        # 명령만 확인하고 차를 세워두려면 dry_run:=true 로 실행할 것
        # (그때도 /control/debug/cmd_vel_preview 로는 계속 볼 수 있다).
        parameters=[
            avoider_config,
            {'dry_run': ParameterValue(dry_run, value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission',
            default_value='false',
            description=(
                'true면 장애물 회피/후진주차/최종 selector를 함께 띄운다. '
                'false(기본)면 기존 주행 전용 구성 그대로.'
            ),
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description=(
                'mission:=true 일 때만 의미가 있다. true면 main_controller가 '
                '/cmd_vel을 발행하지 않고 /control/debug/cmd_vel_preview 로만 '
                '내보낸다(차가 움직이지 않음).'
            ),
        ),
        DeclareLaunchArgument(
            'run_imu',
            default_value='false',
            description=(
                'false(기본)면 IMU를 띄우지 않는다. 별도 터미널에서 '
                'imu_receiver_launch.py 를 먼저 실행해 자이로 캘리브와 '
                '자력계 헤딩 수렴을 끝낸 뒤 주행하는 운용 방식이다. '
                '주행 스택을 재시작해도 헤딩 기준이 유지된다. '
                'true면 이 launch가 IMU까지 함께 띄운다(예전 동작).'
            ),
        ),
        DeclareLaunchArgument(
            'run_gnss',
            default_value='false',
            description=(
                'false(기본)면 GPS를 띄우지 않는다. 별도 터미널에서 '
                'gnss_receiver_launch.py 를 먼저 실행해 RTK를 Fixed까지 '
                '수렴시킨 뒤 주행하는 운용 방식이다. true면 이 launch가 '
                'GPS까지 함께 띄운다.'
            ),
        ),
        scout,
        gnss,
        phidget,
        localization,
        follower,
        local_avoider,
        parking_controller,
        main_controller,
    ])
