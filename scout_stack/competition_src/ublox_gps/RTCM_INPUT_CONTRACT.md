# ZED-F9P RTCM input contract

This integration subscribes to `rtcm_msgs/msg/Message` on `/rtcm` and writes
validated RTCM3 bytes through the existing bounded `Gps/Worker` output queue.
No second serial-forwarder is permitted to open `/dev/ublox`.

`/rxmrtcm` is receiver-generated `UBX-RXM-RTCM` input status. Publishing to
`/rxmrtcm` does not send corrections to the receiver.

## Receiver port and rate provisioning

The competition device is the ZED-F9P USB CDC endpoint, not UART1. Before
starting `ublox_gps_node`, `gnss_receiver_launch.py` opens the device, claims
`TIOCEXCL`, then polls and conditionally changes exactly these Gen-9 RAM keys:

- `CFG-USBINPROT-RTCM3X` (`0x10770004`, L) = `true`
- `CFG-RATE-MEAS` (`0x30210001`, U2) = `200` ms
- `CFG-RATE-NAV` (`0x30210002`, U2) = `1` measurement cycle

Every value is re-read before the driver is launched. No BBR or flash layer is
written. Therefore a receiver power cycle requires the full GNSS launch
RAM provisioning step again. Starting only the package-local u-blox launch does not apply
these keys.

After the provisioning process closes its descriptor, `ublox_gps` opens the same endpoint
and claims `TIOCEXCL` again for its entire lifetime. `TIOCEXCL` prevents later
opens; starting another serial process before the driver remains an operator/configuration error.

An installation wired through UART1 has a different contract: it must
explicitly provision the UART1 RTCM3 input protocol and matching baud rate.
Do not use this USB provisioning launch unchanged for a UART1 receiver.

`config_on_startup: false` disables the driver's broad USB/UART, navigation,
rate, and constellation rewrite. It is not a read-only mode: the driver still
sends volatile `CFG-MSG` commands required to obtain NAV-PVT/RXM-RTCM and
volatile `CFG-INF` output settings. `rate: 5.0` and `nav_rate: 1` describe the
rate verified by the RAM provisioning step and are used by ROS diagnostics.

## `/rtcm` byte and QoS contract

The standard NTRIP client must publish a complete RTCM3 frame, or a
concatenation of complete frames, in each `rtcm_msgs/msg/Message`. The driver
does not accumulate partial frames across ROS messages. For every batch it:

1. requires `0xD3` and zero reserved header bits;
2. reads the 10-bit payload length and requires exactly enough remaining bytes
   for the `3 + payload + 3` byte frame;
3. verifies CRC-24Q for every frame;
4. rejects trailing, truncated, corrupt, or non-RTCM bytes; and
5. bounds the whole batch to `8192` bytes before enqueueing it.

The competition subscription is volatile BEST_EFFORT with depth 5. It accepts
BEST_EFFORT or RELIABLE publishers while preferring a fresh correction over
retransmission of an old one. A full worker queue drops the entire batch and
logs a throttled error; it never enqueues only part of a batch.

The competition config requires a non-zero publisher header stamp and accepts
only ages from `-0.5` to `2.0` seconds. The standard LORD MicroStrain NTRIP ROS
client stamps its published messages. A custom publisher must do the same.

## Runtime output policy

The GNSS adapter publishes `/odometry/gps_enu` only while NAV-PVT, horizontal
accuracy, RTCM reception and carrier-phase requirements are satisfied. Invalid
or stale GNSS data is not forwarded as a new absolute-position measurement.
The waypoint follower itself consumes `/odometry/global`; it does not contain a
second RTK lock or a system-wide preflight state machine.

NTRIP credentials and correction-service configuration do not belong in this
package.
