from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_nav_pkg',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            output='screen'
        ),
        Node(
            package='my_nav_pkg',
            executable='obstacle_avoider_node',
            name='obstacle_avoider',
            output='screen'
        ),
        Node(
            package='my_nav_pkg',
            executable='main_controller_node',
            name='main_controller',
            output='screen'
        ),
    ])
