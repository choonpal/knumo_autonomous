from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


class BundleContractTest(unittest.TestCase):
    def test_setup_keeps_navigation_entry_points(self):
        setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
        for entry_point in (
            'waypoint_follower_node',
            'local_avoider_node',
            'main_controller_node',
            'imu_enu_adapter_node',
            'heading_selector_node',
            'wheel_twist_adapter_node',
            'gnss_enu_odometry_node',
        ):
            self.assertIn(entry_point, setup_source)

    def test_package_dependencies_match_remaining_nodes(self):
        package = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
        dependencies = {
            (element.text or '').strip()
            for tag in ('depend', 'exec_depend')
            for element in package.findall(tag)
        }
        required = {
            'rclpy',
            'geometry_msgs',
            'sensor_msgs',
            'std_msgs',
            'nav_msgs',
            'python3-numpy',
            'ublox_msgs',
            'robot_localization',
            'tf2_ros',
        }
        self.assertTrue(required.issubset(dependencies))
        self.assertNotIn('std_srvs', dependencies)
        self.assertNotIn('visualization_msgs', dependencies)

    def test_follower_has_one_global_ekf_pure_pursuit_path(self):
        follower = (
            PACKAGE_ROOT / 'my_nav_pkg/waypoint_follower_node.py'
        ).read_text(encoding='utf-8')

        for required in (
            "'/odometry/global'",
            'build_path(',
            'PathTracker(',
            'pure_pursuit(',
            "'/cmd_vel/follow'",
            "'/follower/control_pose'",
            "'/follower/debug/metrics'",
            "'/follower/debug/target'",
            "'/follower/cte'",
        ):
            self.assertIn(required, follower)

        for removed in (
            'calibrate_on_start',
            '_legacy_control',
            'cog_blend',
            'required_route_tokens',
            'expected_route_sha256',
            'require_route_sha256',
            'position_source',
            'yaw_source',
            'NavPVT',
            'Imu',
            'SetBool',
            'Trigger',
            'cte_stop',
            'cte_slowdown_scale',
            'route_valid',
        ):
            self.assertNotIn(removed, follower)

    def test_heading_selector_uses_raw_gps_and_gyro_gate(self):
        selector = (
            PACKAGE_ROOT / 'my_nav_pkg/heading_selector_node.py'
        ).read_text(encoding='utf-8')
        self.assertIn("'/odometry/gps_enu'", selector)
        self.assertNotIn("'/odometry/global'", selector)
        self.assertIn('max_abs_gyro_rad_s', selector)
        self.assertIn('min_cog_baseline_m', selector)
        self.assertIn('if self._cog_locked:', selector)

    def test_cte_is_observation_only(self):
        follower = (
            PACKAGE_ROOT / 'my_nav_pkg/waypoint_follower_node.py'
        ).read_text(encoding='utf-8')
        self.assertIn('self.pub_cte.publish', follower)
        self.assertNotIn('distance_to_path >', follower)
        self.assertNotIn('distance_to_path >=', follower)

    def test_local_avoider_can_still_consume_follower_contract(self):
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        follower = (
            PACKAGE_ROOT / 'my_nav_pkg/waypoint_follower_node.py'
        ).read_text(encoding='utf-8')
        self.assertIn('pose_topic: /follower/control_pose', config)
        self.assertIn('hint_topic: /follower/debug/metrics', config)
        self.assertIn('path_target_topic: /follower/debug/target', config)
        self.assertIn("'/follower/active_wp'", follower)
        self.assertIn("'/follower/active_speed_cap'", follower)

    def test_official_pattern_slalom_is_enabled_with_tested_geometry(self):
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        avoider = (
            PACKAGE_ROOT / 'my_nav_pkg/local_avoider_node.py'
        ).read_text(encoding='utf-8')
        core = (PACKAGE_ROOT / 'my_nav_pkg/vfh_core.py').read_text(
            encoding='utf-8'
        )
        for setting in (
            'pattern_slalom_detection_dist: 5.0',
            'pattern_slalom_split_lateral: 0.625',
            'pattern_slalom_upper_pass_lateral: 0.00',
            'pattern_slalom_lower_pass_lateral: 1.25',
            'pattern_slalom_obstacle_spacing: 3.00',
            'pattern_slalom_rejoin_distance: 3.00',
            'pattern_slalom_lookahead: 0.40',
            'pattern_slalom_curvature_preview: 0.10',
            'pattern_slalom_min_speed: 0.50',
            'pattern_slalom_max_speed: 0.70',
            'pattern_slalom_w_max: 1.00',
            'vehicle_length: 1.40',
        ):
            self.assertIn(setting, config)
        self.assertIn('PatternSlalomTarget', avoider)
        self.assertIn('class PatternSlalomTarget:', core)
        self.assertIn('def plan_pattern_trajectory(', core)
        self.assertIn('self.planner.plan_pattern_trajectory(', avoider)

    def test_obstacle_mission_preconditions_are_fail_open(self):
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        avoider = (
            PACKAGE_ROOT / 'my_nav_pkg/local_avoider_node.py'
        ).read_text(encoding='utf-8')
        controller = (
            PACKAGE_ROOT / 'my_nav_pkg/main_controller_node.py'
        ).read_text(encoding='utf-8')

        self.assertIn('require_path_reference: false', config)
        self.assertIn('dry_run: false', config)
        self.assertGreaterEqual(config.count('require_speed_cap: false'), 2)
        self.assertIn(
            'self._publish(False, Twist(), self.STALE',
            avoider,
        )
        self.assertIn('self.avoid_active = False', controller)


    def test_legacy_avoidance_paths_are_removed(self):
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        avoider = (
            PACKAGE_ROOT / 'my_nav_pkg/local_avoider_node.py'
        ).read_text(encoding='utf-8')
        core = (PACKAGE_ROOT / 'my_nav_pkg/vfh_core.py').read_text(
            encoding='utf-8'
        )
        for removed in (
            'ObstacleWaypointTarget',
            'SCurveRejoin',
            'transition_command_is_safe',
            'dynamic_estop_distance',
            'safe_creep',
            'estop_enabled',
            'EMERGENCY_STOP',
            'PARALLEL_ALIGN',
        ):
            self.assertNotIn(removed, avoider)
            self.assertNotIn(removed, config)
            self.assertNotIn(removed, core)
        self.assertIn('def _publish_continuation(', avoider)
        self.assertIn('allow_sector_fallback=False', avoider)

    def test_measured_lidar_offset_is_shared_by_mission_nodes(self):
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        # 2026-08-08 줄자 실측값. 이 테스트의 취지대로 회피/주차가 같은 값을
        # 써야 하며, zip 병합 때도 이 값만은 실측치로 유지한다.
        self.assertIn('laser_x_offset: 0.14', config)
        self.assertIn('lidar_offset_x: 0.14', config)

    def test_rear_blind_avoider_has_no_reverse_recovery_branch(self):
        avoider = (
            PACKAGE_ROOT / 'my_nav_pkg/local_avoider_node.py'
        ).read_text(encoding='utf-8')
        config = (PACKAGE_ROOT / 'config/local_avoider.yaml').read_text(
            encoding='utf-8'
        )
        for removed in (
            'allow_reverse_escape',
            'rev_max_sec',
            'rear_clear_dist',
            '_rear_sector_min',
        ):
            self.assertNotIn(removed, avoider)
            self.assertNotIn(removed, config)

    def test_parking_has_only_effective_reverse_stop_conditions(self):
        parking = (
            PACKAGE_ROOT / 'my_nav_pkg/parking_controller_node.py'
        ).read_text(encoding='utf-8')
        self.assertIn('reverse_target_dist', parking)
        self.assertIn('MAX_REVERSE_TRAVEL', parking)
        self.assertNotIn('reverse_limit_dist', parking)
        self.assertNotIn('REVERSE_HARD_LIMIT_Y', parking)

    def test_removed_preflight_and_route_contract_are_not_present(self):
        self.assertFalse(
            (
                WORKSPACE_SRC
                / 'competition_bringup/competition_bringup/system_preflight_node.py'
            ).exists()
        )
        self.assertFalse(
            (
                WORKSPACE_SRC
                / 'competition_bringup/competition_bringup/preflight_core.py'
            ).exists()
        )
        self.assertFalse((PACKAGE_ROOT / 'my_nav_pkg/route_contract.py').exists())

    def test_path_core_remains_ros_independent(self):
        path_core = (PACKAGE_ROOT / 'my_nav_pkg/path_core.py').read_text(
            encoding='utf-8'
        )
        self.assertNotIn('import rclpy', path_core)


if __name__ == '__main__':
    unittest.main()
