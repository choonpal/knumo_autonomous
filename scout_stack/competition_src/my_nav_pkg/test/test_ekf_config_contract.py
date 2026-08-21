from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read_config(name: str) -> str:
    return (PACKAGE_ROOT / "config" / name).read_text(encoding="utf-8")


def _scalar(config: str, key: str) -> str:
    match = re.search(
        rf"(?m)^\s+{re.escape(key)}:\s*([^\s#]+)",
        config,
    )
    if match is None:
        raise AssertionError(f"missing YAML scalar: {key}")
    return match.group(1)


def _boolean_mask(config: str, key: str):
    match = re.search(
        rf"(?ms)^\s+{re.escape(key)}:\s*\[(.*?)\]",
        config,
    )
    if match is None:
        raise AssertionError(f"missing YAML sensor mask: {key}")
    tokens = re.findall(r"\b(?:true|false)\b", match.group(1))
    return [token == "true" for token in tokens]


class EkfYamlContractTest(unittest.TestCase):
    def setUp(self):
        self.local = _read_config("ekf_local.yaml")
        self.global_ = _read_config("ekf_global.yaml")

    def test_every_robot_localization_mask_has_15_states(self):
        expected_masks = {
            "ekf_local.yaml": (
                self.local,
                ("twist0_config", "imu0_config"),
            ),
            "ekf_global.yaml": (
                self.global_,
                (
                    "twist0_config",
                    "imu0_config",
                    "imu1_config",
                    "odom0_config",
                ),
            ),
        }
        for filename, (config, keys) in expected_masks.items():
            for key in keys:
                with self.subTest(filename=filename, key=key):
                    self.assertEqual(len(_boolean_mask(config, key)), 15)

    def test_global_gps_position_is_absolute_not_differential(self):
        self.assertEqual(_scalar(self.global_, "odom0"), "/odometry/gps_enu")
        self.assertEqual(_scalar(self.global_, "odom0_differential"), "false")
        self.assertEqual(_scalar(self.global_, "odom0_relative"), "false")
        self.assertEqual(
            _boolean_mask(self.global_, "odom0_config"),
            [
                True,
                True,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
                False,
            ],
        )

    def test_global_absolute_yaw_uses_selected_heading(self):
        self.assertEqual(_scalar(self.global_, "imu1"), "/imu/heading_selected")
        self.assertNotIn("imu1: /imu/heading_enu", self.global_)

    def test_side_by_side_filters_do_not_publish_tf_by_default(self):
        self.assertEqual(_scalar(self.local, "publish_tf"), "false")
        self.assertEqual(_scalar(self.global_, "publish_tf"), "false")

    def test_local_filter_stays_gps_free_and_global_filter_uses_map(self):
        self.assertNotRegex(self.local, r"(?m)^\s+odom0:")
        self.assertEqual(_scalar(self.local, "world_frame"), "odom")
        self.assertEqual(_scalar(self.global_, "world_frame"), "map")


class GnssAdapterStaticContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = (
            PACKAGE_ROOT
            / "my_nav_pkg"
            / "gnss_enu_odometry_node.py"
        ).read_text(encoding="utf-8")
        cls.launch = (
            PACKAGE_ROOT / "launch" / "localization_launch.py"
        ).read_text(encoding="utf-8")

    def test_either_navpvt_fix_callback_order_can_publish(self):
        self.assertIn("self._pending_fix", self.node)
        self.assertIn("self._try_publish_fix(self._pending_fix)", self.node)
        self.assertIn("self._try_publish_fix(message)", self.node)

    def test_gnss_utc_stamp_is_not_compared_to_ros_wall_clock(self):
        self.assertIn("'require_fix_stamp_age': False", self.launch)
        self.assertIn("REJECT_FIX_NONADVANCING_STAMP", self.node)
        self.assertIn("REJECT_FIX_RECEPTION_STALE", self.node)

    def test_gated_fix_header_is_not_aliased_to_map_odom_header(self):
        self.assertNotIn("output.header = message.header", self.node)
        self.assertIn(
            "output.header.stamp.sec = message.header.stamp.sec",
            self.node,
        )


if __name__ == "__main__":
    unittest.main()
