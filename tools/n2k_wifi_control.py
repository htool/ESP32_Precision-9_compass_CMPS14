#!/usr/bin/env python3
"""Send the compass magic NMEA 2000 frame that turns Wi‑Fi on or off.

In N2K output mode the SH-ESP32 keeps Wi‑Fi off until it receives proprietary
PGN 61184 with manufacturer 275 and ASCII tag WIFI:

  data[0..1] = 0x13 0x01   # mfr 275 LE
  data[2..5] = W I F I
  data[6]    = 0 off / 1 on
  data[7]    = duration minutes (0 = device default wifi_on_duration_s)

Requires a SocketCAN interface on the same NMEA 2000 bus (e.g. boatnet can0).

Examples:
  python3 tools/n2k_wifi_control.py on
  python3 tools/n2k_wifi_control.py off
  python3 tools/n2k_wifi_control.py on --iface can0 --minutes 20 --sa 0
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# ISO 11783 / NMEA 2000 proprietary single-frame (PDU1, dest=255).
PGN = 61184
PF = 0xEF
PS_BROADCAST = 0xFF
PRIORITY = 6
MANUFACTURER = 275  # matches firmware N2kIdentity default / kWifiMagicManufacturer
TAG = b"WIFI"

CAN_EFF_FLAG = 0x80000000
# Linux can_frame: canid(u32) + dlc(u8) + pad(3) + data(8)
CAN_FRAME_FMT = "=IB3x8s"


def n2k_can_id(priority: int, pf: int, ps: int, sa: int) -> int:
    return ((priority & 0x7) << 26) | ((pf & 0xFF) << 16) | ((ps & 0xFF) << 8) | (
        sa & 0xFF
    )


def build_payload(cmd: int, minutes: int) -> bytes:
    mfr_lo = MANUFACTURER & 0xFF
    mfr_hi = (MANUFACTURER >> 8) & 0xFF
    return bytes([mfr_lo, mfr_hi]) + TAG + bytes([cmd & 0xFF, minutes & 0xFF])


def send_frame(iface: str, sa: int, payload: bytes, count: int, gap_s: float) -> None:
    can_id = n2k_can_id(PRIORITY, PF, PS_BROADCAST, sa) | CAN_EFF_FLAG
    frame = struct.pack(CAN_FRAME_FMT, can_id, len(payload), payload.ljust(8, b"\x00"))
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((iface,))
        for i in range(count):
            sock.send(frame)
            if i + 1 < count and gap_s > 0:
                time.sleep(gap_s)
    finally:
        sock.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("command", choices=("on", "off"), help="Wi‑Fi on or off")
    p.add_argument(
        "--iface",
        "-i",
        default="can0",
        help="SocketCAN interface (default: can0)",
    )
    p.add_argument(
        "--sa",
        type=int,
        default=0,
        help="NMEA 2000 source address of this sender (default: 0)",
    )
    p.add_argument(
        "--minutes",
        "-m",
        type=int,
        default=0,
        help="ON duration minutes (0 = device default; ignored for off)",
    )
    p.add_argument(
        "--count",
        "-n",
        type=int,
        default=3,
        help="Repeat the frame this many times (default: 3)",
    )
    p.add_argument(
        "--gap",
        type=float,
        default=0.05,
        help="Seconds between repeated frames (default: 0.05)",
    )
    args = p.parse_args()

    if not 0 <= args.sa <= 251:
        print("sa must be 0..251", file=sys.stderr)
        return 2
    if not 0 <= args.minutes <= 255:
        print("minutes must be 0..255", file=sys.stderr)
        return 2

    cmd = 1 if args.command == "on" else 0
    minutes = args.minutes if cmd == 1 else 0
    payload = build_payload(cmd, minutes)
    can_id = n2k_can_id(PRIORITY, PF, PS_BROADCAST, args.sa)

    print(
        f"Sending PGN {PGN} WIFI {args.command.upper()} "
        f"on {args.iface} (id=0x{can_id:08X} data={payload.hex()} ×{args.count})"
    )
    try:
        send_frame(args.iface, args.sa, payload, args.count, args.gap)
    except OSError as e:
        print(f"CAN send failed on {args.iface}: {e}", file=sys.stderr)
        print(
            "Is the interface up? e.g. sudo ip link set can0 up type can bitrate 250000",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
