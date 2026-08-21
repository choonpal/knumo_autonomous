from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class WaypointDriveLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = (
            PACKAGE_ROOT / 'launch/knu_waypoint_drive.launch.py'
        ).read_text(encoding='utf-8')

    def test_launch_runs_the_global_ekf_follower_path(self):
        self.assertIn("'run_global': 'true'", self.launch)
        self.assertIn("'publish_tf': 'true'", self.launch)
        self.assertIn("'odom_topic': '/odometry/global'", self.launch)
        # follower 출력은 mission 인자로 갈린다: 기본(false)이면 예전처럼
        # /cmd_vel 직결, true면 /cmd_vel/follow를 남겨 selector가 중재한다.
        self.assertIn("('/cmd_vel/follow', follow_out)", self.launch)
        self.assertIn("else '/cmd_vel'", self.launch)

    def test_launch_does_not_start_removed_interlocks(self):
        # local_avoider_node / main_controller_node는 이 목록에서 뺐다.
        # 장애물 회피와 후진주차를 통합하면서 다시 들어왔기 때문이다.
        # 다만 기본 주행 경로는 그대로 유지된다 - 아래
        # test_mission_nodes_are_opt_in 이 그 조건을 지킨다.
        for removed in (
            'system_preflight_node',
            'mission_control',
            'vision.py',
            'software_runstop',
            'route_sha256',
            'calibrate_on_start',
        ):
            self.assertNotIn(removed, self.launch)

    def test_mission_nodes_are_opt_in(self):
        """회피/주차/selector는 mission:=true 일 때만 뜬다.

        기본 주행 구성을 건드리지 않는 것이 통합의 전제였다. 세 노드가
        조건 없이 선언되면 주행 전용 실행에서도 selector가 끼어들어
        명령 경로와 가감속 특성이 바뀐다.
        """
        self.assertIn("DeclareLaunchArgument(", self.launch)
        self.assertIn("'mission',", self.launch)
        self.assertIn("default_value='false'", self.launch)
        self.assertIn('mission_if = IfCondition(mission)', self.launch)
        for node in (
            'local_avoider_node',
            'parking_controller_node',
            'main_controller_node',
        ):
            block = self.launch[self.launch.index(f"executable='{node}'"):]
            block = block[:block.index('    )')]
            self.assertIn(
                'condition=mission_if',
                block,
                msg=f'{node} must be gated behind mission:=true',
            )

    def test_mission_mode_actually_publishes_cmd_vel(self):
        """mission 구성에서 main_controller가 실제로 /cmd_vel을 낸다.

        config의 dry_run 기본값은 true(벤치 테스트용)라, 그대로 두면
        main_controller가 /cmd_vel 발행을 건너뛰어 차가 전혀 움직이지 않는다.
        주행 전용 구성은 follower가 /cmd_vel로 직결이라 이 값이 무관하지만,
        selector가 경로에 들어오는 mission 구성에서는 하드 블로커가 된다.
        """
        self.assertIn("'dry_run',", self.launch)
        self.assertIn("default_value='false'", self.launch)
        block = self.launch[self.launch.index("executable='main_controller_node'"):]
        block = block[:block.index('    )')]
        self.assertIn(
            "'dry_run': ParameterValue(dry_run, value_type=bool)",
            block,
            msg='main_controller must override the bench-test dry_run default',
        )

    def test_only_one_bringup_launch_profile_remains(self):
        """실측 상수/속도 설정을 가진 런치 프로파일은 하나뿐이어야 한다.

        복제된 런치가 생기면 차량 재튜닝 때 한쪽만 갱신되어 헤딩/스케일이
        조용히 어긋난다. knu_mission_drive는 원본을 include 해 mission/dry_run
        기본값만 바꾸는 얇은 래퍼라 이 위험이 없다. 아래에서 파라미터를
        복제하지 않았다는 것과 mission 실행값이 고정됐는지까지 검사한다.
        """
        launch_files = sorted(
            path.name for path in (PACKAGE_ROOT / 'launch').glob('*.py')
        )
        self.assertEqual(
            launch_files,
            [
                'knu_mission_drive.launch.py',
                'knu_mission_drive_vision.launch.py',
                'knu_waypoint_drive.launch.py',
            ],
        )

        duplicated_tokens = (
            'ORIGIN_LAT',
            'YAW_OFFSET_RAD',
            'WHEEL_LINEAR_SCALE',
            "'linear_speed'",
            "'a_lat_max'",
            "'gps_calib_distance'",
            "'stopline_trigger_dist'",
        )

        wrapper = (
            PACKAGE_ROOT / 'launch/knu_mission_drive.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('knu_waypoint_drive.launch.py', wrapper)
        self.assertIn("'mission': 'true'", wrapper)
        self.assertIn("'dry_run': 'false'", wrapper)
        for duplicated in duplicated_tokens:
            self.assertNotIn(duplicated, wrapper)

        # 2026-08-15: YOLO 신호등 비전을 붙인 래퍼. knu_mission_drive를 그대로
        # include 하고 vision_node만 덧붙이므로 실측 상수를 복제하지 않는다.
        # 이 테스트의 취지(프로파일 복제 금지)는 아래 검사로 유지한다.
        vision_wrapper = (
            PACKAGE_ROOT / 'launch/knu_mission_drive_vision.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn('knu_mission_drive.launch.py', vision_wrapper)
        self.assertIn('vision_node', vision_wrapper)
        for duplicated in duplicated_tokens:
            self.assertNotIn(duplicated, vision_wrapper)


if __name__ == '__main__':
    unittest.main()
