import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('my_nav_pkg')
    local_config = os.path.join(share, 'config', 'ekf_local.yaml')
    global_config = os.path.join(share, 'config', 'ekf_global.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_global = LaunchConfiguration('run_global')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_sensor_static_tf = LaunchConfiguration(
        'publish_sensor_static_tf'
    )
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_frame = LaunchConfiguration('imu_frame')
    imu_x = LaunchConfiguration('imu_x_m')
    imu_y = LaunchConfiguration('imu_y_m')
    imu_z = LaunchConfiguration('imu_z_m')
    gps_x = LaunchConfiguration('gps_x_m')
    gps_y = LaunchConfiguration('gps_y_m')
    gps_z = LaunchConfiguration('gps_z_m')
    yaw_sign = LaunchConfiguration('yaw_sign')
    yaw_offset = LaunchConfiguration('yaw_offset_rad')
    yaw_variance = LaunchConfiguration('yaw_variance_rad2')
    wheel_scale = LaunchConfiguration('wheel_linear_scale')
    origin_lat = LaunchConfiguration('origin_lat')
    origin_lon = LaunchConfiguration('origin_lon')
    origin_alt = LaunchConfiguration('origin_alt')
    antenna_verified = LaunchConfiguration('antenna_lever_arm_verified')
    required_rtk = LaunchConfiguration('required_rtk')
    max_h_acc = LaunchConfiguration('max_h_acc_m')
    cog_max_std = LaunchConfiguration('cog_max_position_std_m')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'run_global',
            default_value='false',
            description='Start RTK-gated map EKF in addition to local EKF.',
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description=(
                'Publish EKF TF. Keep false for side-by-side bag/RC trials; '
                'when true Scout enable_odom_tf must be false.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_sensor_static_tf',
            default_value='false',
            description=(
                'Publish the measured base_link->imu_link and base_link->gps '
                'transforms from this launch. Never run duplicate publishers.'
            ),
        ),
        DeclareLaunchArgument(
            'raw_odom_topic',
            default_value='/scout_mini_base_controller/odom',
        ),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('imu_x_m', default_value='1000.0'),
        DeclareLaunchArgument('imu_y_m', default_value='1000.0'),
        DeclareLaunchArgument('imu_z_m', default_value='1000.0'),
        DeclareLaunchArgument('gps_x_m', default_value='1000.0'),
        DeclareLaunchArgument('gps_y_m', default_value='1000.0'),
        DeclareLaunchArgument('gps_z_m', default_value='1000.0'),
        DeclareLaunchArgument('yaw_sign', default_value='0.0'),
        DeclareLaunchArgument('yaw_offset_rad', default_value='1000.0'),
        DeclareLaunchArgument('yaw_variance_rad2', default_value='-1.0'),
        DeclareLaunchArgument(
            'wheel_linear_scale',
            default_value='1.0',
            description=(
                'Trial-only longitudinal scale. KNU tile measurement gives '
                '1.01595; keep 1.0 until explicitly selected.'
            ),
        ),
        DeclareLaunchArgument('origin_lat', default_value='91.0'),
        DeclareLaunchArgument('origin_lon', default_value='181.0'),
        DeclareLaunchArgument('origin_alt', default_value='0.0'),
        DeclareLaunchArgument(
            'antenna_lever_arm_verified',
            default_value='false',
        ),
        DeclareLaunchArgument('required_rtk', default_value='fixed'),
        DeclareLaunchArgument('max_h_acc_m', default_value='0.05'),
        # 헤딩 셀렉터가 COG 구간을 받아들일 때 보는 위치 표준편차 상한.
        # max_h_acc_m(실측 h_acc 게이트)과 의미가 다르므로 분리한다:
        # gnss_enu_odometry_node는 RTK Float 위치에 float_variance_floor_m2
        # =0.0104(std 0.102m)를 하한으로 깔아 두므로, 0.05를 쓰면 h_acc가
        # 2cm대로 좋아도 Float은 무조건 탈락해 COG가 영영 안 걸린다.
        # 2026-08-05 실측(Float, h_acc 2.5cm): 1.5m 베이스라인 PCA 횡방향
        # 잔차 std 0.23cm, 선형도 중앙 0.99991, COG 헤딩 창간 변동 1.37도.
        # 자력계 절대헤딩 오차(18~20도)보다 한 자릿수 낫다. 0.12는 Float
        # 하한(0.102)을 통과시키되, h_acc가 12cm 넘게 나빠진 Float은 여전히
        # 거른다(공분산 = max(h_acc^2, floor)).
        DeclareLaunchArgument('cog_max_position_std_m', default_value='0.12'),
    ]

    wheel_adapter = Node(
        package='my_nav_pkg',
        executable='wheel_twist_adapter_node',
        name='wheel_twist_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'input_topic': raw_odom_topic,
            'output_topic': '/wheel/twist',
            'expected_header_frame': 'odom',
            'expected_child_frame': 'base_link',
            'output_frame': 'base_link',
            'linear_scale': ParameterValue(wheel_scale, value_type=float),
        }],
    )
    heading_adapter = Node(
        package='my_nav_pkg',
        executable='imu_enu_adapter_node',
        name='imu_enu_adapter',
        output='screen',
        condition=IfCondition(run_global),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'input_topic': imu_topic,
            'output_topic': '/imu/heading_enu',
            'expected_input_frame': imu_frame,
            'output_frame': 'base_link',
            'yaw_sign': ParameterValue(yaw_sign, value_type=float),
            'yaw_offset_rad': ParameterValue(yaw_offset, value_type=float),
            'yaw_variance_rad2': ParameterValue(
                yaw_variance,
                value_type=float,
            ),
        }],
    )
    heading_selector = Node(
        package='my_nav_pkg',
        executable='heading_selector_node',
        name='heading_selector',
        output='screen',
        condition=IfCondition(run_global),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'mag_heading_topic': '/imu/heading_enu',
            'gps_odom_topic': '/odometry/gps_enu',
            'gyro_topic': imu_topic,
            'wheel_twist_topic': '/wheel/twist',
            'output_topic': '/imu/heading_selected',
            'status_topic': '/localization/heading_source',
            'output_frame': 'base_link',
            'min_forward_speed_mps': 0.20,
            'max_abs_gyro_rad_s': 0.05235987755982989,  # 3 deg/s
            'min_cog_baseline_m': 1.50,
            'min_cog_samples': 6,
            'min_linearity': 0.995,
            'max_position_std_m': ParameterValue(cog_max_std, value_type=float),
            'max_position_step_m': 0.75,
            'input_timeout_sec': 0.50,
            'cog_variance_floor_rad2': 0.002741556778080377,
        }],
    )
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[
            local_config,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
            },
        ],
        remappings=[('odometry/filtered', '/odometry/local')],
    )
    gnss_adapter = Node(
        package='my_nav_pkg',
        executable='gnss_enu_odometry_node',
        name='gnss_enu_odometry',
        output='screen',
        condition=IfCondition(run_global),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'fix_topic': '/fix',
            'navpvt_topic': '/navpvt',
            'rtcm_status_topic': '/rxmrtcm',
            'heading_topic': '/imu/heading_enu',
            'output_topic': '/odometry/gps_enu',
            'origin_lat': ParameterValue(origin_lat, value_type=float),
            'origin_lon': ParameterValue(origin_lon, value_type=float),
            'origin_alt': ParameterValue(origin_alt, value_type=float),
            'antenna_x_m': ParameterValue(gps_x, value_type=float),
            'antenna_y_m': ParameterValue(gps_y, value_type=float),
            'antenna_z_m': ParameterValue(gps_z, value_type=float),
            'antenna_lever_arm_verified': ParameterValue(
                antenna_verified,
                value_type=bool,
            ),
            'required_rtk': required_rtk,
            'max_h_acc_m': ParameterValue(max_h_acc, value_type=float),
            # Humble ublox_gps may stamp NavSatFix in GNSS UTC rather than
            # the ROS wall clock. Progression, matching NavPVT and callback
            # reception freshness remain enforced.
            'require_fix_stamp_age': False,
        }],
    )
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        condition=IfCondition(run_global),
        parameters=[
            global_config,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'publish_tf': ParameterValue(publish_tf, value_type=bool),
            },
        ],
        remappings=[('odometry/filtered', '/odometry/global')],
    )
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_static_tf',
        condition=IfCondition(publish_sensor_static_tf),
        arguments=[
            '--x', imu_x,
            '--y', imu_y,
            '--z', imu_z,
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', imu_frame,
        ],
    )
    gps_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_static_tf',
        condition=IfCondition(publish_sensor_static_tf),
        arguments=[
            '--x', gps_x,
            '--y', gps_y,
            '--z', gps_z,
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gps',
        ],
    )

    return LaunchDescription(
        declarations
        + [
            imu_static_tf,
            gps_static_tf,
            wheel_adapter,
            heading_adapter,
            heading_selector,
            local_ekf,
            gnss_adapter,
            global_ekf,
        ]
    )
