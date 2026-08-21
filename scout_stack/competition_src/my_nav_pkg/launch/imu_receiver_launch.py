"""Phidget MOT0110 IMU 단독 실행 (주행 스택과 분리).

GPS(gnss_receiver_launch.py)와 같은 방식으로 IMU도 별도 터미널에서 한 번만
띄워 두고, 주행 스택(knu_waypoint_drive.launch.py)은 껐다 켜도 IMU는 건드리지
않는다.

분리하는 이유:
  - phidgets_spatial은 기동할 때마다 자이로 0점 캘리브(약 2초)를 다시 하고,
    자력계 AHRS 절대헤딩도 처음부터 다시 수렴한다. 주행 스택이 IMU를 같이
    띄우면 주행을 한 번 재시작할 때마다 헤딩 기준이 새로 잡혀서, 한 번 측정한
    yaw_offset이 다음 주행에 맞지 않는다(2026-07-31 실측: 세션 간 약 83도 이동).
  - IMU를 따로 띄워 두면 캘리브/수렴이 세션당 한 번이라 기준이 유지되고,
    드라이버가 죽어도 이 터미널만 다시 띄우면 된다.

운용:
    ros2 launch my_nav_pkg imu_receiver_launch.py
  차량을 정지시킨 상태로 실행하고 'Calibrating IMU done' 이후 자력계 헤딩이
  수렴하도록 10~20초 더 기다린 뒤 주행 스택을 띄운다.

주행 스택은 run_imu:=false(기본)로 두면 IMU를 띄우지 않는다. 한 터미널에서
전부 띄우려면 knu_waypoint_drive.launch.py 를 run_imu:=true 로 실행한다.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# 주행 launch(knu_waypoint_drive.launch.py)와 반드시 같은 값을 쓴다.
IMU_SERIAL = 807529


def generate_launch_description() -> LaunchDescription:
    phidget = ComposableNodeContainer(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
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

    return LaunchDescription([phidget])
