from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    tracking_pose_topic = LaunchConfiguration('tracking_pose_topic')
    run_vision = LaunchConfiguration('run_vision')
    vision_model_path = LaunchConfiguration('vision_model_path')
    camera_index = LaunchConfiguration('camera_index')
    signal_commit_distance = LaunchConfiguration(
        'signal_commit_distance_m'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/scout_mini_base_controller/odom',
        ),
        DeclareLaunchArgument(
            'tracking_pose_topic',
            default_value='/follower/control_pose',
            description=(
                'Fresh follower pose in the same frame as its path target; '
                'used only for signed signal commit progress.'
            ),
        ),
        DeclareLaunchArgument(
            'run_vision',
            default_value='false',
            description=(
                'Start vision with the current Python environment. '
                'Leave false when the TensorRT/YOLO venv runs it separately.'
            ),
        ),
        DeclareLaunchArgument(
            'vision_model_path',
            default_value='',
            description=(
                'Absolute calibrated .engine/.pt path. Required when '
                'run_vision is true.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
        ),
        DeclareLaunchArgument(
            'signal_commit_distance_m',
            default_value='0.50',
        ),
        Node(
            package='autodrive_pkg',
            executable='mission_control_node',
            name='mission_control',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'tracking_pose_topic': tracking_pose_topic,
                'signal_commit_distance_m': ParameterValue(
                    signal_commit_distance,
                    value_type=float,
                ),
            }],
        ),
        Node(
            package='autodrive_pkg',
            executable='vision_node',
            name='vision',
            output='screen',
            condition=IfCondition(run_vision),
            parameters=[{
                'model_path': vision_model_path,
                'camera_index': ParameterValue(
                    camera_index,
                    value_type=int,
                ),
            }],
        ),
    ])
