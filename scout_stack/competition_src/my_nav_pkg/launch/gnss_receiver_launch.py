import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    safe_runtime_config = os.path.join(
        get_package_share_directory('my_nav_pkg'),
        'config',
        'zed_f9p.yaml',
    )
    params_file = LaunchConfiguration('params_file')
    device = LaunchConfiguration('device')
    provision_usb_rtcm3 = LaunchConfiguration('provision_usb_rtcm3')
    provision_timeout = LaunchConfiguration('provision_timeout')

    receiver_after_provisioning = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',
        parameters=[params_file, {'device': device}],
    )
    receiver_without_provisioning = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',
        parameters=[params_file, {'device': device}],
        condition=UnlessCondition(provision_usb_rtcm3),
    )
    provisioning = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'run',
            'my_nav_pkg',
            'zed_f9p_usb_rtcm',
            '--device',
            device,
            '--timeout',
            provision_timeout,
        ],
        output='both',
        condition=IfCondition(provision_usb_rtcm3),
    )

    def start_receiver_only_after_success(event, _context):
        if event.returncode != 0:
            return [
                # ROS 2 Humble's launch.actions has LogInfo but no LogError.
                # Keep the failure visible and preserve the fail-closed
                # Shutdown event instead of making the launch file fail to
                # import before the provisioning can run.
                LogInfo(
                    msg=(
                        'ERROR: ZED-F9P USB RTCM3 provisioning failed; receiver launch '
                        'is stopped. Fix the exclusive device/rate/firmware issue '
                        'or explicitly use provision_usb_rtcm3:=false only '
                        'after verifying RTCM3 input.'
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason='ZED-F9P USB RTCM3 provisioning failed'
                    )
                ),
            ]
        return [
            LogInfo(
                msg=(
                    'ZED-F9P minimal RAM provisioning passed; starting '
                    'receiver without broad startup reconfiguration.'
                )
            ),
            receiver_after_provisioning,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=safe_runtime_config,
            description='ZED-F9P ROS parameters matched to minimal RAM provisioning.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ublox',
            description='Exclusive ZED-F9P USB serial device.',
        ),
        DeclareLaunchArgument(
            'provision_usb_rtcm3',
            default_value='true',
            description=(
                'Provision and verify only CFG-USBINPROT-RTCM3X, '
                'CFG-RATE-MEAS=200 ms, and CFG-RATE-NAV=1 in RAM before '
                'starting ublox_gps.'
            ),
        ),
        DeclareLaunchArgument(
            'provision_timeout',
            default_value='1.5',
            description='Per-attempt UBX provisioning timeout in seconds.',
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=provisioning,
                on_exit=start_receiver_only_after_success,
            )
        ),
        # Register the exit handler before starting the short-lived process;
        # otherwise a fast "already enabled" result can race the handler.
        provisioning,
        receiver_without_provisioning,
    ])
