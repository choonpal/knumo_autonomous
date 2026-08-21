"""장애물 회피 + 후진주차 포함 주행 런치 (mission 구성 기본값).

knu_waypoint_drive.launch.py 를 mission:=true, dry_run:=false 로 감싼 것뿐이다.
파라미터를 복제하지 않는다 - 실측 캘리브레이션 상수(ORIGIN_*, IMU_*, YAW_*)와
속도/정지선 설정은 전부 원본 런치 한 곳에만 두어야, 차량 재튜닝 때 한쪽만
갱신되어 조용히 어긋나는 일이 없다.

  ros2 launch competition_bringup knu_mission_drive.launch.py

원본과 달리 아래 두 값은 이 파일에서 고정한다.

  mission := true     local_avoider / parking_controller / main_controller 기동
  dry_run := false    main_controller가 실제로 /cmd_vel 을 발행

나머지 인자(run_imu, run_gnss)는 그대로 전달되므로 필요하면 덧붙이면 된다.

  ros2 launch competition_bringup knu_mission_drive.launch.py run_imu:=true

주의: mission 구성에서는 CommandGuard(main_controller)가 명령 경로에 들어온다.
linear_slew_rate 0.60 m/s^2 가 걸려 가감속이 느려지므로, mission:=false 에서
맞춘 stopline_trigger_dist(1.30)는 이 구성에서 그대로 쓰면 정지선을 지나친다.
2.5 m/s 기준 제동거리가 약 5.2 m다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    drive_launch = os.path.join(
        get_package_share_directory('competition_bringup'),
        'launch',
        'knu_waypoint_drive.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_imu', default_value='false'),
        DeclareLaunchArgument('run_gnss', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch),
            launch_arguments={
                'mission': 'true',
                'dry_run': 'false',
                'run_imu': LaunchConfiguration('run_imu'),
                'run_gnss': LaunchConfiguration('run_gnss'),
            }.items(),
        ),
    ])
