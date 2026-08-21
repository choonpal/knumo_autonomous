#!/usr/bin/env python3
"""Provision the minimal ZED-F9P RAM settings used by competition bringup.

The gokulp01 ``ublox_gps`` fork places USB configuration behind the broad
``config_on_startup`` switch.  Enabling that switch also writes navigation,
rate, UART and constellation defaults.  This small pre-flight utility instead
uses the Gen-9 configuration database and changes only three RAM keys:

    CFG-USBINPROT-RTCM3X (0x10770004) = true
    CFG-RATE-MEAS         (0x30210001) = 200 ms
    CFG-RATE-NAV          (0x30210002) = 1 measurement cycle

Each current value is polled first and re-verified after any change. USB output
protocols and all other receiver settings are untouched, and changes are
deliberately not persisted to BBR/flash. Run it before the ROS driver while no
other process has the serial device open.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

try:
    import fcntl
    import termios
    import tty
except ImportError:  # Unit tests and offline review may run on Windows.
    fcntl = None
    termios = None
    tty = None


SYNC = b"\xb5\x62"
CFG_CLASS = 0x06
CFG_VALSET_ID = 0x8A
CFG_VALGET_ID = 0x8B
ACK_CLASS = 0x05
ACK_NAK_ID = 0x00
ACK_ACK_ID = 0x01

CFG_USBINPROT_RTCM3X = 0x10770004
CFG_RATE_MEAS = 0x30210001
CFG_RATE_NAV = 0x30210002
TARGET_MEASUREMENT_PERIOD_MS = 200
TARGET_NAVIGATION_RATE = 1
VAL_LAYER_RAM = 0x00
VALSET_LAYER_RAM = 0x01


@dataclass(frozen=True)
class UbxMessage:
    message_class: int
    message_id: int
    payload: bytes


def ubx_checksum(body: bytes) -> bytes:
    checksum_a = 0
    checksum_b = 0
    for value in body:
        checksum_a = (checksum_a + value) & 0xFF
        checksum_b = (checksum_b + checksum_a) & 0xFF
    return bytes((checksum_a, checksum_b))


def build_ubx(message_class: int, message_id: int, payload: bytes) -> bytes:
    if not 0 <= message_class <= 0xFF or not 0 <= message_id <= 0xFF:
        raise ValueError("UBX class and id must fit in one byte")
    if len(payload) > 0xFFFF:
        raise ValueError("UBX payload is too large")
    body = bytes((message_class, message_id)) + struct.pack(
        "<H",
        len(payload),
    ) + payload
    return SYNC + body + ubx_checksum(body)


class UbxStreamDecoder:
    """Incremental UBX decoder that safely skips interleaved NMEA/noise."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[UbxMessage]:
        self._buffer.extend(data)
        messages: List[UbxMessage] = []
        while True:
            sync_index = self._buffer.find(SYNC)
            if sync_index < 0:
                # Retain a trailing 0xb5 because it may be half of the sync.
                self._buffer[:] = (
                    self._buffer[-1:]
                    if self._buffer.endswith(SYNC[:1])
                    else b""
                )
                break
            if sync_index:
                del self._buffer[:sync_index]
            if len(self._buffer) < 6:
                break
            payload_length = struct.unpack_from("<H", self._buffer, 4)[0]
            total_length = 8 + payload_length
            if len(self._buffer) < total_length:
                break
            packet = bytes(self._buffer[:total_length])
            del self._buffer[:total_length]
            body = packet[2:-2]
            if ubx_checksum(body) != packet[-2:]:
                # The consumed sync was false/corrupt. Continue scanning the
                # remaining stream rather than interpreting damaged data.
                continue
            messages.append(
                UbxMessage(
                    message_class=packet[2],
                    message_id=packet[3],
                    payload=packet[6:-2],
                )
            )
        return messages


def _write_all(fd: int, data: bytes, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    view = memoryview(data)
    while view:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("timed out writing UBX packet")
        _readable, writable, _exceptional = select.select(
            [],
            [fd],
            [],
            remaining,
        )
        if not writable:
            continue
        written = os.write(fd, view)
        if written <= 0:
            raise OSError("serial device accepted no UBX bytes")
        view = view[written:]


def _read_matching(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
    accepted: Iterable[tuple[int, int]],
) -> UbxMessage:
    deadline = time.monotonic() + timeout
    accepted_set = set(accepted)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("timed out waiting for UBX response")
        readable, _writable, _exceptional = select.select(
            [fd],
            [],
            [],
            remaining,
        )
        if not readable:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        for message in decoder.feed(chunk):
            if (message.message_class, message.message_id) in accepted_set:
                return message


def _valget_payload(key: int) -> bytes:
    return struct.pack("<BBHI", 0, VAL_LAYER_RAM, 0, key)


def _valset_bool_payload(key: int, value: bool) -> bytes:
    # version, RAM-only layer bitmask, transaction, reserved, key, U1 value
    return struct.pack(
        "<BBBBIB",
        0,
        VALSET_LAYER_RAM,
        0,
        0,
        key,
        int(bool(value)),
    )


def _valset_u2_payload(key: int, value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("U2 configuration value must fit in two bytes")
    return struct.pack(
        "<BBBBIH",
        0,
        VALSET_LAYER_RAM,
        0,
        0,
        key,
        value,
    )


def parse_valget_bool(message: UbxMessage, expected_key: int) -> bool:
    if (
        message.message_class != CFG_CLASS
        or message.message_id != CFG_VALGET_ID
        or len(message.payload) < 9
    ):
        raise ValueError("not a boolean UBX-CFG-VALGET response")
    key = struct.unpack_from("<I", message.payload, 4)[0]
    if key != expected_key:
        raise ValueError(
            f"unexpected VALGET key 0x{key:08x}; "
            f"wanted 0x{expected_key:08x}"
        )
    return bool(message.payload[8])


def parse_valget_u2(message: UbxMessage, expected_key: int) -> int:
    if (
        message.message_class != CFG_CLASS
        or message.message_id != CFG_VALGET_ID
        or len(message.payload) < 10
    ):
        raise ValueError("not a U2 UBX-CFG-VALGET response")
    key = struct.unpack_from("<I", message.payload, 4)[0]
    if key != expected_key:
        raise ValueError(
            f"unexpected VALGET key 0x{key:08x}; "
            f"wanted 0x{expected_key:08x}"
        )
    return struct.unpack_from("<H", message.payload, 8)[0]


def query_usb_rtcm3(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
) -> bool:
    _write_all(
        fd,
        build_ubx(
            CFG_CLASS,
            CFG_VALGET_ID,
            _valget_payload(CFG_USBINPROT_RTCM3X),
        ),
        timeout,
    )
    response = _read_matching(
        fd,
        decoder,
        timeout,
        [(CFG_CLASS, CFG_VALGET_ID)],
    )
    return parse_valget_bool(response, CFG_USBINPROT_RTCM3X)


def query_u2_config(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
    key: int,
) -> int:
    _write_all(
        fd,
        build_ubx(CFG_CLASS, CFG_VALGET_ID, _valget_payload(key)),
        timeout,
    )
    response = _read_matching(
        fd,
        decoder,
        timeout,
        [(CFG_CLASS, CFG_VALGET_ID)],
    )
    return parse_valget_u2(response, key)


def _set_ram_value(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
    payload: bytes,
) -> None:
    _write_all(
        fd,
        build_ubx(CFG_CLASS, CFG_VALSET_ID, payload),
        timeout,
    )
    response = _read_matching(
        fd,
        decoder,
        timeout,
        [
            (ACK_CLASS, ACK_ACK_ID),
            (ACK_CLASS, ACK_NAK_ID),
        ],
    )
    if response.payload[:2] != bytes((CFG_CLASS, CFG_VALSET_ID)):
        raise RuntimeError("receiver acknowledged a different UBX command")
    if response.message_id == ACK_NAK_ID:
        raise RuntimeError("receiver rejected RAM-only configuration VALSET")


def set_usb_rtcm3_ram(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
) -> None:
    _set_ram_value(
        fd,
        decoder,
        timeout,
        _valset_bool_payload(CFG_USBINPROT_RTCM3X, True),
    )


def set_u2_config_ram(
    fd: int,
    decoder: UbxStreamDecoder,
    timeout: float,
    key: int,
    value: int,
) -> None:
    _set_ram_value(
        fd,
        decoder,
        timeout,
        _valset_u2_payload(key, value),
    )


def _claim_exclusive_serial(fd: int) -> None:
    if (
        fcntl is None
        or termios is None
        or not hasattr(termios, "TIOCEXCL")
    ):
        raise RuntimeError(
            "ZED-F9P provisioning requires POSIX TIOCEXCL support"
        )
    fcntl.ioctl(fd, termios.TIOCEXCL)


def ensure_usb_rtcm3(
    device: str,
    timeout: float = 1.5,
    retries: int = 3,
) -> bool:
    """Return True when any required RAM change was made."""
    if not device:
        raise ValueError("device path must not be empty")
    if not 0.1 <= timeout <= 30.0:
        raise ValueError("timeout must be between 0.1 and 30 seconds")
    if not 1 <= retries <= 20:
        raise ValueError("retries must be between 1 and 20")
    if fcntl is None or termios is None or tty is None:
        raise RuntimeError("ZED-F9P provisioning requires a POSIX serial host")

    fd = os.open(
        device,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )
    previous_attributes: Optional[list] = None
    try:
        _claim_exclusive_serial(fd)
        previous_attributes = termios.tcgetattr(fd)
        tty.setraw(fd, when=termios.TCSANOW)
        termios.tcflush(fd, termios.TCIOFLUSH)
        decoder = UbxStreamDecoder()

        last_error: Optional[Exception] = None
        changed = False
        for _attempt in range(retries):
            try:
                if not query_usb_rtcm3(fd, decoder, timeout):
                    set_usb_rtcm3_ram(fd, decoder, timeout)
                    changed = True
                if not query_usb_rtcm3(fd, decoder, timeout):
                    raise RuntimeError("USB RTCM3 remained disabled after VALSET")

                if query_u2_config(
                    fd,
                    decoder,
                    timeout,
                    CFG_RATE_MEAS,
                ) != TARGET_MEASUREMENT_PERIOD_MS:
                    set_u2_config_ram(
                        fd,
                        decoder,
                        timeout,
                        CFG_RATE_MEAS,
                        TARGET_MEASUREMENT_PERIOD_MS,
                    )
                    changed = True
                if query_u2_config(
                    fd,
                    decoder,
                    timeout,
                    CFG_RATE_MEAS,
                ) != TARGET_MEASUREMENT_PERIOD_MS:
                    raise RuntimeError(
                        "measurement period remained different from 200 ms"
                    )

                if query_u2_config(
                    fd,
                    decoder,
                    timeout,
                    CFG_RATE_NAV,
                ) != TARGET_NAVIGATION_RATE:
                    set_u2_config_ram(
                        fd,
                        decoder,
                        timeout,
                        CFG_RATE_NAV,
                        TARGET_NAVIGATION_RATE,
                    )
                    changed = True
                if query_u2_config(
                    fd,
                    decoder,
                    timeout,
                    CFG_RATE_NAV,
                ) != TARGET_NAVIGATION_RATE:
                    raise RuntimeError(
                        "navigation rate remained different from one cycle"
                    )
                return changed
            except (OSError, TimeoutError, RuntimeError, ValueError) as error:
                last_error = error
                termios.tcflush(fd, termios.TCIFLUSH)
        raise RuntimeError(
            f"could not verify USB RTCM3 input after {retries} attempts"
        ) from last_error
    finally:
        try:
            if previous_attributes is not None:
                termios.tcsetattr(fd, termios.TCSANOW, previous_attributes)
        finally:
            os.close(fd)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Provision only ZED-F9P USB RTCM3 input and 5 Hz navigation "
            "rate keys in RAM; do not rewrite other receiver settings."
        )
    )
    parser.add_argument("--device", default="/dev/ublox")
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--retries", type=int, default=3)
    args = parser.parse_args(argv)

    try:
        changed = ensure_usb_rtcm3(
            args.device,
            timeout=args.timeout,
            retries=args.retries,
        )
    except Exception as error:
        print(f"ZED-F9P USB RTCM3 provisioning failed: {error}", file=sys.stderr)
        return 2

    if changed:
        print(
            "ZED-F9P USB RTCM3/5 Hz navigation RAM settings applied and "
            "verified; all other receiver settings preserved."
        )
    else:
        print(
            "ZED-F9P USB RTCM3/5 Hz navigation RAM settings already match; "
            "receiver unchanged."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
