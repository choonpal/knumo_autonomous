from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
UBLOX_ROOT = PACKAGE_ROOT.parent / "ublox_gps"


class ZedF9pConfigContractTest(unittest.TestCase):
    def test_competition_config_pins_verified_rate_and_bounded_rtcm_input(self):
        config = (PACKAGE_ROOT / "config/zed_f9p.yaml").read_text(
            encoding="utf-8"
        )
        for required_line in (
            "    config_on_startup: false",
            "    rate: 5.0",
            "    nav_rate: 1",
            "      enabled: true",
            "      topic: /rtcm",
            "      queue_depth: 5",
            "      max_batch_bytes: 8192",
            "      require_header_stamp: true",
            "      max_age_sec: 2.0",
            "      max_future_sec: 0.5",
            "      status_timeout_sec: 3.0",
        ):
            self.assertIn(required_line, config)

        # Broad startup configuration and persistent save blocks stay absent.
        self.assertNotIn("\n    usb:", config)
        self.assertNotIn("\n    uart1:", config)
        self.assertNotIn("\n    gnss:", config)
        self.assertNotIn("\n    save:", config)

    def test_package_config_has_the_same_rtcm_validation_contract(self):
        config = (UBLOX_ROOT / "config/zed_f9p.yaml").read_text(
            encoding="utf-8"
        )
        for required_line in (
            "    rate: 5.0",
            "    nav_rate: 1",
            "      queue_depth: 5",
            "      max_batch_bytes: 8192",
            "      require_header_stamp: true",
            "      max_age_sec: 2.0",
            "      max_future_sec: 0.5",
            "      status_timeout_sec: 3.0",
        ):
            self.assertIn(required_line, config)

    def test_ram_provisioning_pins_only_usb_input_and_navigation_rate_keys(self):
        provisioner = (
            PACKAGE_ROOT / "my_nav_pkg/zed_f9p_usb_rtcm.py"
        ).read_text(encoding="utf-8")
        self.assertIn("CFG_USBINPROT_RTCM3X = 0x10770004", provisioner)
        self.assertIn("CFG_RATE_MEAS = 0x30210001", provisioner)
        self.assertIn("CFG_RATE_NAV = 0x30210002", provisioner)
        self.assertIn("TARGET_MEASUREMENT_PERIOD_MS = 200", provisioner)
        self.assertIn("TARGET_NAVIGATION_RATE = 1", provisioner)
        self.assertIn("VALSET_LAYER_RAM = 0x01", provisioner)
        self.assertIn("_claim_exclusive_serial(fd)", provisioner)


if __name__ == "__main__":
    unittest.main()
