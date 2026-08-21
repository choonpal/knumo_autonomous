"""knu_mission_drive + YOLO 신호등 비전 (2026-08-15 추가).

기존 런치를 복제하지 않고 include 만 한다. 실측 캘리브레이션 상수
(ORIGIN_*, IMU_*, YAW_*)와 회피/주차 설정은 knu_waypoint_drive.launch.py
한 곳에만 두어야 재튜닝 때 한쪽만 갱신되어 조용히 어긋나는 일이 없다.

  ros2 launch competition_bringup knu_mission_drive_vision.launch.py

기본 모델은 autodrive_pkg/models/YOLO26n.engine 이다. TensorRT 엔진은
빌드한 GPU/드라이버/TensorRT 버전에 묶이므로 다른 장비에서 로드가 실패하면
이식 가능한 onnx 로 바꿔서 실행한다.

  ros2 launch competition_bringup knu_mission_drive_vision.launch.py \
      vision_model:=YOLO26n.onnx

비전만 빼고 주행만 하려면 run_vision:=false, 또는 기존 런치를 그대로 쓴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mission_launch = os.path.join(
        get_package_share_directory('competition_bringup'),
        'launch',
        'knu_mission_drive.launch.py',
    )
    model_dir = os.path.join(
        get_package_share_directory('autodrive_pkg'),
        'models',
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_imu', default_value='false'),
        DeclareLaunchArgument('run_gnss', default_value='false'),
        DeclareLaunchArgument(
            'run_vision',
            default_value='true',
            description='false 면 주행만 하고 비전 노드를 띄우지 않는다.',
        ),
        DeclareLaunchArgument(
            'vision_model',
            default_value='YOLO26n.engine',
            description=(
                'autodrive_pkg/models 안의 파일 이름. TensorRT 엔진 로드가 '
                '실패하면 YOLO26n.onnx 로 바꿀 것.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='/dev/videoN 의 N.',
        ),
        DeclareLaunchArgument(
            'green_confirm_sec',
            default_value='0.50',
            description=(
                '초록을 이 시간 동안 연속으로 봐야 출발한다. 깜빡임/오검출로 '
                '먼저 나가는 것을 막는다.'
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'run_imu': LaunchConfiguration('run_imu'),
                'run_gnss': LaunchConfiguration('run_gnss'),
            }.items(),
        ),

        Node(
            package='autodrive_pkg',
            executable='vision_node',
            name='vision',
            output='screen',
            # 시스템 python3의 torch가 CPU 전용 빌드(2.7.1+cpu, CUDA 불가)라
            # .engine 로드 시 AssertionError로 즉사한다(torch.md 3장 실측).
            # yolo11_env는 CUDA torch(2.3.0)를 갖춘 검증된 실행 환경이라
            # (run_vision.sh / run_trafficlight.sh와 동일한 방식) 이 인터프리터로 강제한다.
            # vision.py 자체는 이미 sys.path.append로 시스템 TensorRT를 보충하도록 되어 있다.
            prefix='/home/knumo/yolo11_env/bin/python3',
            condition=IfCondition(LaunchConfiguration('run_vision')),
            parameters=[{
                'model_path': PathJoinSubstitution([
                    model_dir,
                    LaunchConfiguration('vision_model'),
                ]),
                'camera_index': ParameterValue(
                    LaunchConfiguration('camera_index'),
                    value_type=int,
                ),
            }],
        ),

        # 신호등 게이트. /follower/active_wp 가 _signal 웨이포인트면
        # /control/state 로 정지를 걸고, /traffic_light/state 가
        # green_confirm_sec 동안 초록이면 푼다. main_controller 는 이
        # mission_stop 을 회피/주차보다 우선해서 처리한다.
        # 비전이 끊기면 fail-closed 로 정지를 유지한다.
        Node(
            package='autodrive_pkg',
            executable='mission_control_node',
            name='mission_control',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_vision')),
            parameters=[{
                'odom_topic': '/odometry/global',
                'tracking_pose_topic': '/follower/control_pose',
                'green_confirm_sec': ParameterValue(
                    LaunchConfiguration('green_confirm_sec'),
                    value_type=float,
                ),
            }],
        ),
    ])
