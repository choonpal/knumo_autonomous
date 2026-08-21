import struct
import unittest
from types import SimpleNamespace
from unittest import mock

import my_nav_pkg.zed_f9p_usb_rtcm as provisioner
from my_nav_pkg.zed_f9p_usb_rtcm import (
    ACK_ACK_ID,
    ACK_CLASS,
    CFG_CLASS,
    CFG_RATE_MEAS,
    CFG_RATE_NAV,
    CFG_USBINPROT_RTCM3X,
    CFG_VALGET_ID,
    TARGET_MEASUREMENT_PERIOD_MS,
    TARGET_NAVIGATION_RATE,
    UbxMessage,
    UbxStreamDecoder,
    _claim_exclusive_serial,
    _valget_payload,
    _valset_bool_payload,
    _valset_u2_payload,
    build_ubx,
    parse_valget_bool,
    parse_valget_u2,
    ubx_checksum,
)


class UbxEncodingTest(unittest.TestCase):
    def test_checksum_matches_known_empty_poll_shape(self):
        body = bytes((CFG_CLASS, CFG_VALGET_ID, 0, 0))
        packet = build_ubx(CFG_CLASS, CFG_VALGET_ID, b"")
        self.assertEqual(packet[:2], b"\xb5\x62")
        self.assertEqual(packet[2:-2], body)
        self.assertEqual(packet[-2:], ubx_checksum(body))

    def test_valget_queries_only_usb_rtcm3_ram_key(self):
        payload = _valget_payload(CFG_USBINPROT_RTCM3X)
        self.assertEqual(payload[:4], b"\x00\x00\x00\x00")
        self.assertEqual(
            struct.unpack_from("<I", payload, 4)[0],
            CFG_USBINPROT_RTCM3X,
        )

    def test_valset_is_ram_only_and_changes_one_boolean_key(self):
        payload = _valset_bool_payload(CFG_USBINPROT_RTCM3X, True)
        self.assertEqual(payload[:4], b"\x00\x01\x00\x00")
        self.assertEqual(
            struct.unpack_from("<I", payload, 4)[0],
            CFG_USBINPROT_RTCM3X,
        )
        self.assertEqual(payload[8:], b"\x01")
        self.assertEqual(len(payload), 9)

    def test_rate_valset_is_ram_only_u2(self):
        payload = _valset_u2_payload(
            CFG_RATE_MEAS,
            TARGET_MEASUREMENT_PERIOD_MS,
        )
        self.assertEqual(payload[:4], b"\x00\x01\x00\x00")
        self.assertEqual(struct.unpack_from("<I", payload, 4)[0], CFG_RATE_MEAS)
        self.assertEqual(
            struct.unpack_from("<H", payload, 8)[0],
            TARGET_MEASUREMENT_PERIOD_MS,
        )
        self.assertEqual(len(payload), 10)


class UbxStreamDecoderTest(unittest.TestCase):
    def test_decoder_skips_noise_and_accepts_fragmented_packet(self):
        packet = build_ubx(
            ACK_CLASS,
            ACK_ACK_ID,
            bytes((CFG_CLASS, 0x8A)),
        )
        decoder = UbxStreamDecoder()
        self.assertEqual(decoder.feed(b"$GPGGA,noise\r\n" + packet[:3]), [])
        messages = decoder.feed(packet[3:])
        self.assertEqual(
            messages,
            [UbxMessage(ACK_CLASS, ACK_ACK_ID, bytes((CFG_CLASS, 0x8A)))],
        )

    def test_decoder_rejects_bad_checksum(self):
        packet = bytearray(build_ubx(CFG_CLASS, CFG_VALGET_ID, b"abc"))
        packet[-1] ^= 0xFF
        self.assertEqual(UbxStreamDecoder().feed(bytes(packet)), [])

    def test_valget_parser_requires_expected_key(self):
        payload = (
            b"\x00\x00\x00\x00"
            + struct.pack("<I", CFG_USBINPROT_RTCM3X)
            + b"\x01"
        )
        message = UbxMessage(CFG_CLASS, CFG_VALGET_ID, payload)
        self.assertTrue(parse_valget_bool(message, CFG_USBINPROT_RTCM3X))
        with self.assertRaises(ValueError):
            parse_valget_bool(message, CFG_USBINPROT_RTCM3X + 1)

    def test_u2_valget_parser_requires_expected_key(self):
        payload = (
            b"\x00\x00\x00\x00"
            + struct.pack("<I", CFG_RATE_NAV)
            + struct.pack("<H", TARGET_NAVIGATION_RATE)
        )
        message = UbxMessage(CFG_CLASS, CFG_VALGET_ID, payload)
        self.assertEqual(
            parse_valget_u2(message, CFG_RATE_NAV),
            TARGET_NAVIGATION_RATE,
        )
        with self.assertRaises(ValueError):
            parse_valget_u2(message, CFG_RATE_MEAS)


class RuntimeProvisioningTest(unittest.TestCase):
    def _serial_mocks(self):
        fake_termios = SimpleNamespace(
            TIOCEXCL=0x540C,
            TCSANOW=0,
            TCIOFLUSH=2,
            TCIFLUSH=0,
            tcgetattr=mock.Mock(return_value=[]),
            tcsetattr=mock.Mock(),
            tcflush=mock.Mock(),
        )
        fake_tty = SimpleNamespace(setraw=mock.Mock())
        fake_fcntl = SimpleNamespace(ioctl=mock.Mock())
        return fake_termios, fake_tty, fake_fcntl

    def test_claims_kernel_serial_exclusivity(self):
        fake_termios, _fake_tty, fake_fcntl = self._serial_mocks()
        with (
            mock.patch.object(provisioner, "termios", fake_termios),
            mock.patch.object(provisioner, "fcntl", fake_fcntl),
        ):
            _claim_exclusive_serial(17)
        fake_fcntl.ioctl.assert_called_once_with(17, fake_termios.TIOCEXCL)

    def test_conditionally_sets_and_reverifies_all_three_ram_keys(self):
        fake_termios, fake_tty, fake_fcntl = self._serial_mocks()
        with (
            mock.patch.object(provisioner, "termios", fake_termios),
            mock.patch.object(provisioner, "tty", fake_tty),
            mock.patch.object(provisioner, "fcntl", fake_fcntl),
            mock.patch.object(provisioner.os, "O_NOCTTY", 0, create=True),
            mock.patch.object(provisioner.os, "O_NONBLOCK", 0, create=True),
            mock.patch.object(provisioner.os, "open", return_value=17),
            mock.patch.object(provisioner.os, "close") as close,
            mock.patch.object(
                provisioner,
                "query_usb_rtcm3",
                side_effect=[False, True],
            ),
            mock.patch.object(provisioner, "set_usb_rtcm3_ram") as set_usb,
            mock.patch.object(
                provisioner,
                "query_u2_config",
                side_effect=[1000, 200, 2, 1],
            ),
            mock.patch.object(provisioner, "set_u2_config_ram") as set_u2,
        ):
            self.assertTrue(
                provisioner.ensure_usb_rtcm3(
                    "/dev/ublox",
                    timeout=1.5,
                    retries=1,
                )
            )

        set_usb.assert_called_once()
        self.assertEqual(
            set_u2.call_args_list,
            [
                mock.call(
                    17,
                    mock.ANY,
                    1.5,
                    CFG_RATE_MEAS,
                    TARGET_MEASUREMENT_PERIOD_MS,
                ),
                mock.call(
                    17,
                    mock.ANY,
                    1.5,
                    CFG_RATE_NAV,
                    TARGET_NAVIGATION_RATE,
                ),
            ],
        )
        fake_fcntl.ioctl.assert_called_once_with(17, fake_termios.TIOCEXCL)
        close.assert_called_once_with(17)


if __name__ == "__main__":
    unittest.main()
