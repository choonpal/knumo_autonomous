import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mission_launch = os.path.join(
        get_package_share_directory('autodrive_pkg'),
        'launch',
        'mission_control.launch.py',
    )
    run_vision = LaunchConfiguration('run_vision')
    odom_topic = LaunchConfiguration('odom_topic')
    vision_model_path = LaunchConfiguration('vision_model_path')
    camera_index = LaunchConfiguration('camera_index')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_vision',
            default_value='false',
            description=(
                'Deprecated wrapper: start vision only when its CUDA/YOLO '
                'dependencies and calibrated model are available.'
            ),
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/scout_mini_base_controller/odom',
        ),
        DeclareLaunchArgument('vision_model_path', default_value=''),
        DeclareLaunchArgument('camera_index', default_value='0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'run_vision': run_vision,
                'odom_topic': odom_topic,
                'vision_model_path': vision_model_path,
                'camera_index': camera_index,
            }.items(),
        ),
    ])
