#!/usr/bin/env python3
"""Live 3D cube + guided calibration for WitMotion HWT901B (USB-TTL UART).

Usage:
  python3 tools/witmotion_cube.py
  python3 tools/witmotion_cube.py --port /dev/ttyUSB0 --baud 57600

Calibration uses WitMotion CALSW register (same as Minimu / WitMotion.exe):
  1 = accelerometer, 2 = magnetometer, 0 = exit, then SAVE.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Deque, List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial required: sudo apt install python3-serial") from exc


FRAME_LEN = 11
ACC_TYPE = 0x51
GYRO_TYPE = 0x52
ANGLE_TYPE = 0x53
MAG_TYPE = 0x54

REG_SAVE = 0x00
REG_CALSW = 0x01
REG_RSW = 0x02
REG_HXOFFSET = 0x0B
REG_DIRECTION = 0x23  # 0 = horizontal, 1 = vertical (90° about X)
REG_AXIS6 = 0x24  # 0 = 9-axis (mag heading), 1 = 6-axis (integrates → drifts)
REG_READ = 0x27
REG_RESP_TYPE = 0x5F  # reply to FF AA 27 …
CAL_NORMAL = 0
CAL_ACCEL = 1
# WIT Standard Protocol CALSW (0x01):
#   0x07 = Magnetic Field Calibration (Spherical Fitting)  ← HWT901B / industrial
#   0x09 = Magnetic Field Calibration (Dual Plane Mode)   ← yaw-only / boat swing
# Classic JY901 TTL docs used 2 for mag; on this unit 2 freezes mag output.
CAL_MAG = 7
CAL_MAG_DUAL = 9
CAL_MAG_ALT = 2  # older JY901-style modules only

# RSW bits: TIME|ACC|GYRO|ANGLE|MAG|PORT|PRESS|GPS ≈ enable common set + mag
RSW_WITH_MAG = 0x1F

CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])

ASSETS_DIR = Path(__file__).resolve().parent / "assets"
CALIBRATIONS_DIR = Path(__file__).resolve().parent / "calibrations"


def s16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 65536 if v >= 32768 else v


def checksum_ok(buf: bytes) -> bool:
    return (sum(buf[:10]) & 0xFF) == buf[10]


def wit_write(reg: int, low: int, high: int = 0) -> bytes:
    return bytes([0xFF, 0xAA, reg & 0xFF, low & 0xFF, high & 0xFF])


@dataclass
class ImuState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    linked: bool = False
    frames: int = 0
    baud: int = 0
    port: str = ""
    last_error: str = ""
    mag_norm_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=200))
    # Mag samples collected during active magnetic calibration
    cal_mag_samples: List[Tuple[float, float, float]] = field(default_factory=list)
    cal_collecting: bool = False
    # Set when mag cal wizard finishes & saves (UI hint only; sensor flash is source of truth)
    mag_cal_saved: bool = False
    mag_cal_coverage: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    nine_axis: Optional[bool] = None  # True after we force AXIS6=0
    vertical_mount: Optional[bool] = None  # True after DIRECTION=1
    # Last register readback (None until 0x5F reply)
    reg_direction: Optional[int] = None
    reg_axis6: Optional[int] = None
    mag_bias: Optional[Tuple[int, int, int]] = None  # HX/HY/HZOFFSET
    mag_bias_ts: float = 0.0


class WitMotionReader(threading.Thread):
    def __init__(self, state: ImuState, port: Optional[str], baud: Optional[int]):
        super().__init__(daemon=True)
        self.state = state
        self.port_arg = port
        self.baud_arg = baud
        self._stop = threading.Event()
        self._cmd_q: "queue.Queue[bytes]" = queue.Queue()
        self._pending_reads: "queue.Queue[int]" = queue.Queue()
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()

    def stop(self) -> None:
        self._stop.set()

    def send_raw(self, payload: bytes, pause_s: float = 0.05) -> None:
        self._cmd_q.put((payload, pause_s))

    def read_reg(self, addr: int, pause_s: float = 0.15) -> None:
        """Request register block starting at addr (reply frame type 0x5F)."""
        self._pending_reads.put(addr & 0xFF)
        self.send_raw(wit_write(REG_READ, addr & 0xFF, 0x00), pause_s)

    def write_mag_bias(
        self, hx: int, hy: int, hz: int, save: bool = True
    ) -> None:
        """Write HX/HY/HZOFFSET (hard-iron) and optionally SAVE to flash."""
        def _u16(v: int) -> Tuple[int, int]:
            v = int(v) & 0xFFFF
            return v & 0xFF, (v >> 8) & 0xFF

        for reg, val in (
            (REG_HXOFFSET, hx),
            (REG_HXOFFSET + 1, hy),
            (REG_HXOFFSET + 2, hz),
        ):
            lo, hi = _u16(val)
            self.send_raw(CMD_UNLOCK, 0.15)
            self.send_raw(wit_write(reg, lo, hi), 0.15)
        if save:
            self.send_raw(CMD_UNLOCK, 0.2)
            self.send_raw(wit_write(REG_SAVE, 0x00), 0.2)
        with self.state.lock:
            self.state.mag_bias = (int(hx), int(hy), int(hz))
            self.state.mag_bias_ts = time.time()

    def send_calsw(self, mode: int, save: bool = False) -> None:
        """Unlock, set CALSW, optionally save config to flash.

        Timing mirrors SensESP/WitMotion PC: unlock → wait → cmd → (unlock → save).
        """
        self.send_raw(CMD_UNLOCK, 0.2)
        self.send_raw(wit_write(REG_CALSW, mode), 0.2)
        if save:
            self.send_raw(CMD_UNLOCK, 0.2)
            self.send_raw(wit_write(REG_SAVE, 0x00), 0.2)

    def ensure_mag_output(self) -> None:
        """Make sure continuous MAG frames (0x54) are enabled."""
        self.send_raw(CMD_UNLOCK, 0.2)
        self.send_raw(wit_write(REG_RSW, RSW_WITH_MAG), 0.2)

    def ensure_nine_axis(self, save: bool = True) -> None:
        """Force 9-axis algorithm — mag-referenced heading (much less post-move drift)."""
        self.send_raw(CMD_UNLOCK, 0.2)
        self.send_raw(wit_write(REG_AXIS6, 0x00), 0.2)
        if save:
            self.send_raw(CMD_UNLOCK, 0.2)
            self.send_raw(wit_write(REG_SAVE, 0x00), 0.2)
        with self.state.lock:
            self.state.nine_axis = True

    def set_install_direction(self, vertical: bool, save: bool = True) -> None:
        """DIRECTION: vertical = cable down / sticker forward (90° about X)."""
        self.send_raw(CMD_UNLOCK, 0.2)
        self.send_raw(wit_write(REG_DIRECTION, 0x01 if vertical else 0x00), 0.2)
        if save:
            self.send_raw(CMD_UNLOCK, 0.2)
            self.send_raw(wit_write(REG_SAVE, 0x00), 0.2)
        with self.state.lock:
            self.state.vertical_mount = vertical
        # Confirm what the sensor actually stored
        self.read_reg(REG_DIRECTION)
        self.read_reg(REG_HXOFFSET)

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                port, baud = self._open_params()
                with self.state.lock:
                    self.state.port = port
                    self.state.baud = baud
                    self.state.last_error = ""
                self._read_loop(port, baud)
            except Exception as exc:  # noqa: BLE001
                msg = str(exc)
                if "readiness to read" in msg or "multiple access" in msg:
                    msg = (
                        "Serial port busy or CH340 glitch — is another "
                        "witmotion_cube / serial tool using the port? "
                        f"({exc})"
                    )
                with self.state.lock:
                    self.state.linked = False
                    self.state.last_error = msg
                # Back off; avoid spamming reconnect on contested ports
                time.sleep(2.0)

    def _open_params(self) -> Tuple[str, int]:
        port = self.port_arg or self._autodetect_port()
        if self.baud_arg:
            return port, self.baud_arg
        baud = detect_baud(port)
        if baud is None:
            raise RuntimeError(f"No WitMotion frames on {port}")
        return port, baud

    @staticmethod
    def _autodetect_port() -> str:
        ports = list(list_ports.comports())
        for p in ports:
            hwid = (p.hwid or "").lower()
            desc = (p.description or "").lower()
            if "1a86" in hwid or "ch340" in desc or "usb serial" in desc:
                return p.device
        for p in ports:
            if "USB" in p.device or "ACM" in p.device:
                return p.device
        raise RuntimeError("No serial port found (plug in USB-TTL)")

    def _read_loop(self, port: str, baud: int) -> None:
        # exclusive=True fails fast if another process holds the port
        # (avoids the CH340 "device reports readiness…" loop).
        open_kwargs = dict(
            port=port,
            baudrate=baud,
            timeout=0.05,
            dsrdtr=False,
            rtscts=False,
            write_timeout=1.0,
        )
        try:
            ser = serial.Serial(**open_kwargs, exclusive=True)
        except TypeError:
            ser = serial.Serial(**open_kwargs)
        except serial.SerialException as exc:
            raise serial.SerialException(
                f"Cannot open {port} exclusively — close other cube/serial "
                f"apps first ({exc})"
            ) from exc

        with self._ser_lock:
            self._ser = ser
        buf = bytearray()
        last_frame = 0.0
        try:
            # Queue config once connected (drains inside the loop below).
            # Do NOT overwrite DIRECTION — forcing horizontal here was wiping
            # vertical boat installs (roll≈90°, drifting HDG, crazy boat mesh).
            self.send_calsw(CAL_NORMAL, save=False)  # leave any stuck mag/accel cal
            self.ensure_mag_output()
            self.ensure_nine_axis(save=True)  # persist 9-axis — 6-axis = yaw drift
            self.read_reg(REG_DIRECTION)
            self.read_reg(REG_AXIS6)
            self.read_reg(REG_HXOFFSET)
            while not self._stop.is_set():
                while True:
                    try:
                        item = self._cmd_q.get_nowait()
                    except queue.Empty:
                        break
                    if isinstance(item, tuple):
                        cmd, pause_s = item
                    else:
                        cmd, pause_s = item, 0.05
                    try:
                        ser.write(cmd)
                        ser.flush()
                    except serial.SerialException:
                        raise
                    time.sleep(pause_s)

                try:
                    waiting = ser.in_waiting
                    chunk = ser.read(waiting or 1)
                except serial.SerialException:
                    # CH340 often throws "readiness to read but returned no
                    # data" when the peer vanishes or the port is shared.
                    raise
                if chunk:
                    buf.extend(chunk)
                    self._consume(buf)
                    last_frame = time.time()
                linked = (time.time() - last_frame) < 1.0
                with self.state.lock:
                    self.state.linked = linked and self.state.frames > 0
        finally:
            with self._ser_lock:
                self._ser = None
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _consume(self, buf: bytearray) -> None:
        while True:
            if not buf:
                return
            try:
                start = buf.index(0x55)
            except ValueError:
                buf.clear()
                return
            if start:
                del buf[:start]
            if len(buf) < FRAME_LEN:
                return
            frame = bytes(buf[:FRAME_LEN])
            if not checksum_ok(frame):
                del buf[0]
                continue
            del buf[:FRAME_LEN]
            self._handle(frame)

    def _handle(self, frame: bytes) -> None:
        typ = frame[1]
        d = frame[2:10]
        with self.state.lock:
            self.state.frames += 1
            if typ == ANGLE_TYPE:
                self.state.roll = s16(d[0], d[1]) * 180.0 / 32768.0
                self.state.pitch = s16(d[2], d[3]) * 180.0 / 32768.0
                self.state.yaw = s16(d[4], d[5]) * 180.0 / 32768.0
            elif typ == ACC_TYPE:
                # raw / 32768 * 16 g
                self.state.acc = tuple(
                    s16(d[j], d[j + 1]) * 16.0 / 32768.0 for j in (0, 2, 4)
                )
            elif typ == GYRO_TYPE:
                scale = 2000.0 / 32768.0
                self.state.gyro_dps = tuple(
                    s16(d[j], d[j + 1]) * scale for j in (0, 2, 4)
                )
            elif typ == MAG_TYPE:
                mx = float(s16(d[0], d[1]))
                my = float(s16(d[2], d[3]))
                mz = float(s16(d[4], d[5]))
                self.state.mag = (mx, my, mz)
                self.state.mag_norm_hist.append(math.sqrt(mx * mx + my * my + mz * mz))
                if self.state.cal_collecting:
                    # Keep a bound so long cal sessions stay light.
                    if len(self.state.cal_mag_samples) < 8000:
                        self.state.cal_mag_samples.append((mx, my, mz))
                    elif len(self.state.cal_mag_samples) % 4 == 0:
                        self.state.cal_mag_samples.append((mx, my, mz))
                        del self.state.cal_mag_samples[:2]
            elif typ == REG_RESP_TYPE:
                try:
                    addr = self._pending_reads.get_nowait()
                except queue.Empty:
                    addr = None
                v0 = s16(d[0], d[1])
                v1 = s16(d[2], d[3])
                v2 = s16(d[4], d[5])
                if addr == REG_HXOFFSET:
                    self.state.mag_bias = (v0, v1, v2)
                    self.state.mag_bias_ts = time.time()
                elif addr == REG_DIRECTION:
                    self.state.reg_direction = v0 & 0xFFFF
                    self.state.reg_axis6 = v1 & 0xFFFF
                    self.state.vertical_mount = bool(v0 & 0xFF)
                    self.state.nine_axis = (v1 & 0xFF) == 0
                elif addr == REG_AXIS6:
                    self.state.reg_axis6 = v0 & 0xFFFF
                    self.state.nine_axis = (v0 & 0xFF) == 0


def detect_baud(port: str, candidates=(57600, 9600, 115200, 38400)) -> Optional[int]:
    for baud in candidates:
        try:
            ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=False, rtscts=False)
        except serial.SerialException:
            return None
        try:
            time.sleep(0.15)
            ser.reset_input_buffer()
            t0 = time.time()
            buf = b""
            while time.time() - t0 < 0.8:
                buf += ser.read(512)
            good = 0
            i = 0
            while i + FRAME_LEN <= len(buf):
                if buf[i] == 0x55 and buf[i + 1] in (0x51, 0x52, 0x53, 0x54):
                    if checksum_ok(buf[i : i + FRAME_LEN]):
                        good += 1
                        i += FRAME_LEN
                        continue
                i += 1
            if good >= 5:
                return baud
        finally:
            ser.close()
    return None


def calibration_report(state: ImuState) -> str:
    with state.lock:
        hist = list(state.mag_norm_hist)
        mag = state.mag
        linked = state.linked
        frames = state.frames
        cal_saved = state.mag_cal_saved
        cal_cov = state.mag_cal_coverage
    if not linked or frames < 20:
        return "Mag status: waiting for data…"
    if len(hist) < 30:
        return "Mag status: collecting samples…"
    mean = sum(hist) / len(hist)
    var = sum((x - mean) ** 2 for x in hist) / len(hist)
    stdev = math.sqrt(var)
    mx, my, mz = mag
    # WitMotion mag LSB is often 500–4000; ‖H‖>2500 is normal, not a failure.
    if mean < 40:
        return f"Mag status: WEAK ‖H‖≈{mean:.0f} — check wiring / run mag cal."
    if stdev > max(80.0, 0.35 * mean):
        return (f"Mag status: MOVING/NOISY ‖H‖≈{mean:.0f}±{stdev:.0f} "
                f"({mx:.0f},{my:.0f},{mz:.0f}) — hold still to assess.")
    if cal_saved:
        cx, cy, cz = cal_cov
        return (f"Mag status: OK after cal — stable ‖H‖≈{mean:.0f}±{stdev:.0f} "
                f"({mx:.0f},{my:.0f},{mz:.0f}) "
                f"(coverage {cx:.0%}/{cy:.0%}/{cz:.0%})")
    if stdev <= max(40.0, 0.20 * mean):
        return (f"Mag status: STABLE ‖H‖≈{mean:.0f}±{stdev:.0f} "
                f"({mx:.0f},{my:.0f},{mz:.0f}). "
                f"Use Calibrate… if heading drifts when you turn.")
    return (f"Mag status: ‖H‖≈{mean:.0f}±{stdev:.0f} "
            f"({mx:.0f},{my:.0f},{mz:.0f}).")


MAG_ANGLE_BINS = 36  # 10° sectors around each 2D ellipse


def mag_plane_ellipse(
    samples: List[Tuple[float, float, float]],
    ax0: int,
    ax1: int,
) -> dict:
    """MiniIMU-style plane stats: angular ring fill + outlier fraction.

    Span/max-min bars lie with hard-iron offset (a short arc looks “full”).
    Edge fill asks: have we walked around the ellipse circumference?
    """
    empty = {
        "fill": 0.0, "outliers": 0.0, "n": 0, "r": 0.0,
        "cx": 0.0, "cy": 0.0, "pts": [], "ok": [],
    }
    if len(samples) < 40:
        return empty
    pts = [(p[ax0], p[ax1]) for p in samples]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    radii = sorted(math.hypot(x - cx, y - cy) for x, y in pts)
    r_med = radii[len(radii) // 2]
    if r_med < 40:
        return {**empty, "n": len(pts), "cx": cx, "cy": cy, "pts": pts}
    bins = [False] * MAG_ANGLE_BINS
    n_out = 0
    ok_flags = []
    for x, y in pts:
        rr = math.hypot(x - cx, y - cy)
        if rr > 2.0 * r_med or rr < 0.25 * r_med:
            n_out += 1
            ok_flags.append(False)
            continue
        ok_flags.append(True)
        # Only count the ring band toward edge fill
        if 0.55 * r_med <= rr <= 1.55 * r_med:
            ang = math.atan2(y - cy, x - cx)
            bi = int((ang + math.pi) / (2 * math.pi) * MAG_ANGLE_BINS) % MAG_ANGLE_BINS
            bins[bi] = True
    fill = sum(1 for b in bins if b) / MAG_ANGLE_BINS
    return {
        "fill": fill,
        "outliers": n_out / len(pts),
        "n": len(pts),
        "r": r_med,
        "cx": cx,
        "cy": cy,
        "pts": pts,
        "ok": ok_flags,
    }


def mag_ellipse_coverage(
    samples: List[Tuple[float, float, float]],
) -> Tuple[float, float, float]:
    """XY / XZ / YZ angular edge-fill 0..1."""
    xy = mag_plane_ellipse(samples, 0, 1)["fill"]
    xz = mag_plane_ellipse(samples, 0, 2)["fill"]
    yz = mag_plane_ellipse(samples, 1, 2)["fill"]
    return xy, xz, yz


def mag_coverage(samples: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
    return mag_ellipse_coverage(samples)


def _bar(frac: float, width: int = 24) -> str:
    n = max(0, min(width, int(round(frac * width))))
    return "█" * n + "·" * (width - n)


class CalibWizard(tk.Toplevel):
    """Guided accel + mag cal using real HWT901 product photos.

    Mag procedure follows WitMotion HWT901B manuals + WIT Standard Protocol:
    CALSW=0x07 spherical fitting, slow full coverage of the field sphere,
    ≥20 cm (ideally ~1 m) from iron / speakers / laptop magnets, then SAVE
    and keep AXIS6=0 (9-axis). See witmotion-sensor.com FAQ and
    open-boat-projects.org HWT901B Signal K guide.
    """

    # phase → asset filename under tools/assets/
    _PHOTOS = {
        "intro": "hwt901.png",
        "accel_wait": "cal_flat.png",
        "accel_run": "cal_flat.png",
        "mag_run": "cal_rot_x.png",
        "settle": "hwt901.png",
        "done": "hwt901.png",
    }
    _MAG_TIP_PHOTOS = ("cal_rot_x.png", "cal_rot_y.png", "cal_rot_z.png")
    # Official + marine guides: several minutes of free motion beats 3 quick rolls
    _MAG_MIN_S = 90.0
    _MAG_WARMUP_S = 3.0  # drop samples while CALSW mode settles
    _SETTLE_S = 8.0

    def __init__(
        self,
        master: tk.Tk,
        reader: WitMotionReader,
        state: ImuState,
        app: Optional["CubeApp"] = None,
        want_vertical: bool = False,
    ):
        super().__init__(master)
        self.reader = reader
        self.state = state
        self.app = app
        # Boat mount: accel still needs a flat/horizontal pass, then mag with
        # DIRECTION=vertical (WitMotion: set vertical, then calibrate).
        self.want_vertical = want_vertical
        kind = "VERTICAL boat mount" if want_vertical else "HORIZONTAL / table"
        self.title(f"HWT901 calibration — {kind}")
        self.geometry("720x720")
        self.minsize(640, 640)
        # No transient(master): with transient, withdraw/iconify of the main
        # window also unmaps this dialog on X11 (Calibrate… looked like a crash).
        self.grab_set()
        self._master_iconified = False
        try:
            master.iconify()
            self._master_iconified = True
        except tk.TclError:
            pass
        try:
            self.attributes("-topmost", True)
            self.lift()
            self.focus_force()
            self.after(200, lambda: self.attributes("-topmost", False))
        except tk.TclError:
            pass

        self.phase = "intro"
        self._accel_end = 0.0
        self._mag_start = 0.0
        self._mag_warmed = False
        self._settle_end = 0.0
        self._settle_hdg: List[float] = []
        self._tip_i = 0
        self._mag_stats: Tuple = (0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        self._mag_skipped = False
        self._photo_cache: dict = {}
        self._tk_photo = None
        self._ellipse_draw_i = 0
        self._verify_notes: List[str] = []

        # Pack buttons first (BOTTOM) so they never fall off-screen when
        # the intro text or photo grows.
        btns = tk.Frame(self)
        btns.pack(side=tk.BOTTOM, fill=tk.X, padx=12, pady=10)
        self.btn_next = ttk.Button(btns, text="Start", command=self._next)
        self.btn_next.pack(side=tk.LEFT, padx=4)
        self.btn_skip = ttk.Button(btns, text="Skip accel → mag",
                                   command=self._skip_to_mag)
        self.btn_skip.pack(side=tk.LEFT, padx=4)
        self.btn_skip_mag = ttk.Button(
            btns, text="Skip mag → align", command=self._skip_mag
        )
        self.btn_skip_mag.pack(side=tk.LEFT, padx=4)
        self.btn_cancel = ttk.Button(btns, text="Cancel", command=self._cancel)
        self.btn_cancel.pack(side=tk.RIGHT, padx=4)

        body = tk.Frame(self)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(8, 0))

        if want_vertical:
            banner_bg, banner_fg, banner_txt = (
                "#5c2b0a", "#ffd166",
                "CAL MODE: VERTICAL  ·  cable down · sticker bow · −X starboard",
            )
        else:
            banner_bg, banner_fg, banner_txt = (
                "#0a3a4a", "#9ee7ff",
                "CAL MODE: HORIZONTAL / table  ·  sensor flat, label up",
            )
        self.mount_banner = tk.Label(
            body, text=banner_txt, bg=banner_bg, fg=banner_fg,
            font=("Helvetica", 12, "bold"), pady=6,
        )
        self.mount_banner.pack(fill=tk.X, padx=4, pady=(0, 4))

        self.progress = tk.StringVar(value="")
        # Scrollable instructions — fixed height so buttons stay on-screen
        info_frame = tk.Frame(body, height=110)
        info_frame.pack(fill=tk.X, padx=4, pady=(4, 6))
        info_frame.pack_propagate(False)
        self.info_text = tk.Text(
            info_frame, wrap=tk.WORD, height=6, relief=tk.FLAT,
            background=info_frame.cget("background"), cursor="arrow")
        info_scroll = ttk.Scrollbar(info_frame, command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=info_scroll.set, state=tk.DISABLED)
        info_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.info_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Photo (intro / accel / settle) — swapped for ellipse plots during mag
        self.img_slot = tk.Frame(body, height=200, bg="#121a24")
        self.img_slot.pack(fill=tk.X, padx=4, pady=4)
        self.img_slot.pack_propagate(False)
        self.img_label = tk.Label(self.img_slot, bg="#121a24")
        self.img_label.pack(expand=True)

        # MiniIMU-style XY / XZ / YZ mag ellipses (shown only in mag_run)
        self.ellipse_frame = tk.Frame(body, height=220, bg="#0b1c2c")
        self.ellipse_canvases = []
        for title in ("XY", "XZ", "YZ"):
            col = tk.Frame(self.ellipse_frame, bg="#0b1c2c")
            col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)
            tk.Label(col, text=title, fg="#ffd166", bg="#0b1c2c",
                     font=("TkFixedFont", 10, "bold")).pack()
            cv = tk.Canvas(col, bg="#121a24", highlightthickness=1,
                           highlightbackground="#3a5166", height=190)
            cv.pack(fill=tk.BOTH, expand=True)
            self.ellipse_canvases.append(cv)

        self.progress_label = tk.Label(
            body, textvariable=self.progress, justify=tk.LEFT, anchor="nw",
            fg="#333", font=("TkFixedFont", 10))
        self.progress_label.pack(fill=tk.BOTH, expand=True, padx=4, pady=(6, 4))
        self._body = body

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self._set_cal_banner()
        self._set_intro()
        self.after(100, self._tick)
        self.after(50, self._fit_window)

    def _set_cal_banner(
        self, temporary_horizontal: bool = False, detail: str = ""
    ) -> None:
        """Keep the cal window banner in sync with the active DIRECTION step."""
        if temporary_horizontal and self.want_vertical:
            bg, fg = "#0a3a4a", "#9ee7ff"
            text = "CAL: VERTICAL session — accel uses HORIZONTAL (lay flat)"
        elif self.want_vertical:
            bg, fg = "#5c2b0a", "#ffd166"
            text = "CAL MODE: VERTICAL  ·  cable down · sticker bow · −X starboard"
        else:
            bg, fg = "#0a3a4a", "#9ee7ff"
            text = "CAL MODE: HORIZONTAL / table  ·  sensor flat, label up"
        if detail:
            text = f"{text}  ·  {detail}"
        self.mount_banner.configure(text=text, bg=bg, fg=fg)
        kind = "VERTICAL" if self.want_vertical else "HORIZONTAL"
        self.title(f"HWT901 calibration — {kind}")

    def _show_media_photo(self) -> None:
        self.ellipse_frame.pack_forget()
        self.img_slot.pack(fill=tk.X, padx=4, pady=4, before=self.progress_label)

    def _show_media_ellipses(self) -> None:
        self.img_slot.pack_forget()
        self.ellipse_frame.pack(
            fill=tk.BOTH, expand=True, padx=4, pady=4, before=self.progress_label)

    def _fit_window(self) -> None:
        """Ensure default geometry keeps the button bar on-screen."""
        if not self.winfo_exists():
            return
        self.update_idletasks()
        req_h = self.winfo_reqheight()
        cur_h = self.winfo_height()
        if cur_h < req_h:
            self.geometry(f"{max(self.winfo_width(), 560)}x{min(req_h + 8, 780)}")

    def _load_photo_file(self, name: str) -> None:
        path = ASSETS_DIR / name
        if not path.exists():
            self.img_label.configure(image="", text=f"(missing {name})")
            return
        if name not in self._photo_cache:
            img = tk.PhotoImage(file=str(path))
            # Fit into ~500×220 slot (subsample keeps PhotoImage-only, no Pillow)
            max_w, max_h = 500, 220
            fx = max(1, (img.width() + max_w - 1) // max_w)
            fy = max(1, (img.height() + max_h - 1) // max_h)
            f = max(fx, fy)
            if f > 1:
                img = img.subsample(f, f)
            self._photo_cache[name] = img
        self._tk_photo = self._photo_cache[name]
        self.img_label.configure(image=self._tk_photo, text="")

    def _show_photo(self, key: str) -> None:
        self._show_media_photo()
        self._load_photo_file(self._PHOTOS.get(key) or "hwt901.png")

    def _set_info(self, text: str) -> None:
        self.info_text.configure(state=tk.NORMAL)
        self.info_text.delete("1.0", tk.END)
        self.info_text.insert("1.0", text)
        self.info_text.configure(state=tk.DISABLED)
        self.info_text.yview_moveto(0)

    def _set_intro(self) -> None:
        self.phase = "intro"
        self._show_photo("intro")
        if self.want_vertical:
            # Keep DIRECTION=vertical for the whole vertical session unless
            # the user explicitly chooses the one-time flat accel step.
            self.reader.set_install_direction(vertical=True, save=True)
            self._set_cal_banner(detail="recommended: Skip accel → mag")
            mount = (
                "Mode: VERTICAL boat mount\n\n"
                "We do NOT remap X/Y/Z in software — WitMotion DIRECTION=vertical "
                "rotates the algorithm 90° about X inside the sensor. After that, "
                "reported roll/pitch/yaw are already vehicle axes (heel/trim/HDG).\n\n"
                "Recommended: Skip accel → mag (keeps VERTICAL the whole time).\n"
                "Accel is optional and only needed once on a flat table (WitMotion "
                "requires 1g on Z for that step — that is why “Start” briefly uses "
                "horizontal).\n\n"
                "Mag pose: cable down, sticker bow, −X starboard — tumble in that "
                "orientation until raw R/P ≈ 0° when still.\n\n"
                "Already mag-calibrated? Skip mag → align.\n\n"
            )
            self.btn_next.configure(text="Accel first (flat)…", state=tk.NORMAL)
            self.btn_skip.configure(text="Skip accel → mag (keep VERTICAL)",
                                    state=tk.NORMAL)
        else:
            self.reader.set_install_direction(vertical=False, save=True)
            self._set_cal_banner()
            mount = (
                "Mode: HORIZONTAL / table\n"
                "Keep the sensor flat (label up) for accel + mag.\n"
                "For the boat bracket, click Vertical mount first, then Calibrate.\n"
                "Already mag-calibrated? Skip mag → align.\n\n"
            )
            self.btn_next.configure(text="Start", state=tk.NORMAL)
            self.btn_skip.configure(text="Skip accel → mag", state=tk.NORMAL)
        self.btn_skip_mag.configure(text="Skip mag → align", state=tk.NORMAL)
        self._set_info(
            "HWT901 guided calibration (WitMotion spherical mag)\n\n"
            f"{mount}"
            "Flow: (optional accel) → mag tumble → hold still → auto level "
            "roll/pitch → point bow north for yaw→0 → left-90° invert checks "
            "on roll / pitch / yaw.\n\n"
            "Calibrate away from the boat’s steel. Soft-iron at the mount is "
            "fixed later with heading offset / GPS deviation table."
        )
        self.progress.set("")

    def _skip_to_mag(self) -> None:
        self._begin_mag()

    def _skip_mag(self) -> None:
        """Keep existing mag bias; jump to stillness → align → invert checks."""
        try:
            with self.state.lock:
                self.state.cal_collecting = False
            self.reader.send_calsw(CAL_NORMAL, save=False)
            self.reader.ensure_nine_axis(save=True)
            self.reader.set_install_direction(vertical=self.want_vertical, save=True)
        except Exception:  # noqa: BLE001
            pass
        self._mag_skipped = True
        self._mag_stats = (0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        self._mag_summary = "Mag calibration skipped (kept existing sensor bias)."
        self._begin_settle()

    def _next(self) -> None:
        if self.phase == "intro":
            self._begin_accel()
        elif self.phase == "accel_wait":
            self._run_accel()
        elif self.phase == "mag_run":
            self._finish_mag()
        elif self.phase == "yaw_north":
            self._finish_yaw_north()
        elif self.phase == "verify_roll":
            self._advance_verify("roll", "pitch")
        elif self.phase == "verify_pitch":
            self._advance_verify("pitch", "yaw")
        elif self.phase == "verify_yaw":
            self._advance_verify("yaw", None)
        elif self.phase == "done":
            self.destroy()

    def _begin_accel(self) -> None:
        self.phase = "accel_wait"
        self._show_photo("accel_wait")
        self.reader.ensure_nine_axis(save=True)
        # WitMotion: accel cal needs the module flat with 1g on Z. That is the
        # only reason we briefly switch DIRECTION=horizontal — not for mag.
        self.reader.set_install_direction(vertical=False, save=True)
        self._set_cal_banner(
            temporary_horizontal=True,
            detail="accel only — lay flat (label up); mag will restore VERTICAL",
        )
        self._set_info(
            "① Accelerometer (optional one-time table step)\n\n"
            "WitMotion requires this with the sensor flat (1g on Z), so "
            "DIRECTION is temporarily HORIZONTAL. Lay flat, label up, still.\n\n"
            "Then press “Calibrate accel”."
            + (
                "\n\nNext: we restore DIRECTION=VERTICAL before mag — you will "
                "stand the sensor on end again."
                if self.want_vertical else ""
            )
        )
        self.progress.set("")
        self.btn_next.configure(text="Calibrate accel")
        self.btn_skip.configure(
            text=("Skip → mag (VERTICAL)" if self.want_vertical else "Skip accel → mag"),
            state=tk.NORMAL,
        )
        self.btn_skip_mag.configure(text="Skip mag → align", state=tk.NORMAL)

    def _run_accel(self) -> None:
        self.phase = "accel_run"
        self._show_photo("accel_run")
        self.btn_next.configure(state=tk.DISABLED)
        self.btn_skip.configure(state=tk.DISABLED)
        self.btn_skip_mag.configure(state=tk.DISABLED)
        self._set_info("Accelerometer calibration running… keep still (label up).")
        self.reader.send_calsw(CAL_ACCEL, save=False)
        self._accel_end = time.time() + 3.2

    def _finish_accel(self) -> None:
        self.reader.send_calsw(CAL_NORMAL, save=True)
        time.sleep(0.3)
        self._begin_mag()

    def _begin_mag(self) -> None:
        self.phase = "mag_run"
        self._mag_start = time.time()
        self._mag_warmed = False
        self._tip_i = 0
        self._show_media_ellipses()
        with self.state.lock:
            self.state.cal_mag_samples.clear()
            self.state.cal_collecting = True
        self.reader.ensure_mag_output()
        self.reader.ensure_nine_axis(save=False)
        if self.want_vertical:
            self.reader.set_install_direction(vertical=True, save=True)
            if self.app is not None:
                self.app.mount_vertical_intent = True
                self.app._apply_inverts_for_mount(True, force=True)  # fresh mag session
            self._set_cal_banner(detail="mag tumble — cable down / sticker bow")
            mag_pose = (
                "DIRECTION=vertical is ON. Hold the housing as installed: "
                "cable down, sticker toward bow, −X to starboard — then tumble "
                "slowly about that pose (not flat on the table)."
            )
        else:
            self.reader.set_install_direction(vertical=False, save=True)
            self._set_cal_banner(detail="mag tumble — free in space, not on iron")
            mag_pose = (
                "Keep DIRECTION=horizontal: tumble with the sensor free in "
                "space (~50 cm above the table)."
            )
        # Spherical fitting (0x07) — same as WitMotion MagCal / Modbus example
        self.reader.send_calsw(CAL_MAG, save=False)
        self._set_info(
            "② Magnetometer — elliptical fit (like MiniIMU)\n\n"
            f"{mag_pose}\n\n"
            "≥1 m from laptop speakers / iron. Tumble slowly 3–5 min until "
            "XY / XZ / YZ rings are sharp and filled. Red dots = outliers "
            "(interference) — move elsewhere and restart mag. Finish when "
            "all three edge-fills are high."
        )
        self.btn_next.configure(text="Finish & save", state=tk.DISABLED)
        self.btn_skip.configure(state=tk.DISABLED)
        self.btn_skip_mag.configure(text="Skip mag → align", state=tk.NORMAL)

    def _finish_mag(self) -> None:
        with self.state.lock:
            self.state.cal_collecting = False
            n = len(self.state.cal_mag_samples)
            cov = mag_ellipse_coverage(self.state.cal_mag_samples)
            self.state.mag_cal_saved = True
            self.state.mag_cal_coverage = cov
            self.state.mag_norm_hist.clear()
            samples = list(self.state.cal_mag_samples)
        outs = (
            mag_plane_ellipse(samples, 0, 1)["outliers"],
            mag_plane_ellipse(samples, 0, 2)["outliers"],
            mag_plane_ellipse(samples, 1, 2)["outliers"],
        )
        self.reader.send_calsw(CAL_NORMAL, save=True)
        self.reader.ensure_nine_axis(save=True)
        # Keep the installation direction that matches the mag we just did.
        self.reader.set_install_direction(vertical=self.want_vertical, save=True)
        self._mag_stats = (n, outs, cov, 0.0)
        self._begin_settle()

    def _begin_settle(self) -> None:
        self.phase = "settle"
        self._show_photo("settle")
        self._settle_end = time.time() + self._SETTLE_S
        self._settle_hdg.clear()
        self._ensure_cal_direction()
        if self.want_vertical:
            self._set_cal_banner(detail="stillness ON END — not flat")
            self._set_info(
                "③ Stillness check (vertical)\n\n"
                "Stand ON END (not on the table): cable down, sticker bow. "
                "Hold still — vehicle roll/pitch should be near 0° and HDG "
                "should not wander. If R/P ≈ ±90°, you are flat or DIRECTION "
                "is wrong — tip to the install pose."
            )
        else:
            self._set_cal_banner(detail="stillness — lay flat")
            self._set_info(
                "③ Stillness check\n\n"
                "Lay the sensor flat, label up, and hold perfectly still.\n"
                "Heading should stay put. Wandering here means mag cal failed "
                "or a nearby magnet is still pulling the field."
            )
        self.progress.set("")
        self.btn_next.configure(state=tk.DISABLED)
        self.btn_skip.configure(state=tk.DISABLED)
        self.btn_skip_mag.configure(state=tk.DISABLED)
        self.btn_cancel.configure(state=tk.DISABLED)

    def _finish_settle(self) -> None:
        n, outs, cov, _ = self._mag_stats
        cx, cy, cz = cov
        ox, oy, oz = outs
        hdgs = self._settle_hdg
        if len(hdgs) >= 5:
            lo, hi = min(hdgs), max(hdgs)
            span = hi - lo
            if span > 180:
                adj = sorted(((h + 180) % 360) for h in hdgs)
                span = adj[-1] - adj[0]
            mean = sum(hdgs) / len(hdgs)
            settle_txt = f"still heading span ≈ {span:.1f}° (mean {mean:.1f}°)"
            if span > 3.0:
                settle_txt += " — ⚠ too much wander; redo mag farther from metal"
            else:
                settle_txt += " — ✓ stable"
        else:
            settle_txt = "stillness sample thin (USB drop?)"
        self._settle_txt = settle_txt
        if self._mag_skipped:
            self._mag_summary = (
                f"Mag calibration skipped (kept existing sensor bias).\n"
                f"{settle_txt}"
            )
        else:
            self._mag_summary = (
                f"Mag samples: {n}\n"
                f"Ellipse edge fill XY/XZ/YZ: {cx:.0%} / {cy:.0%} / {cz:.0%}\n"
                f"Outliers XY/XZ/YZ: {ox:.0%} / {oy:.0%} / {oz:.0%}\n"
                f"{settle_txt}"
            )
        # Hold-still pose already defines level roll/pitch — apply offsets now.
        # Mount defaults for Invert_* once here; verify may toggle them later.
        self._ensure_cal_direction(reset_inverts=True)
        self._auto_level_rp()
        self._begin_yaw_north()

    def _ensure_cal_direction(self, reset_inverts: bool = False) -> None:
        """Keep DIRECTION matched to this wizard session (especially vertical).

        reset_inverts=True only at session start / after accel→mag — never during
        invert checks (that would undo auto Invert_* toggles).
        """
        self.reader.set_install_direction(vertical=self.want_vertical, save=True)
        if self.app is not None:
            self.app.mount_vertical_intent = self.want_vertical
            if reset_inverts:
                self.app._apply_inverts_for_mount(self.want_vertical, force=True)

    def _auto_level_rp(self) -> None:
        """Zero heel/trim from the stillness pose (leave yaw for north align)."""
        if self.app is None:
            return
        self.app._zero_rp_axes()
        br, bp, _, _ = self._wizard_boat_rpy()
        self._rp_level_note = (
            f"Auto level roll/pitch applied → boat heel/trim≈{br:+.1f}/{bp:+.1f}°"
        )

    def _begin_yaw_north(self) -> None:
        """Only yaw still needs a known reference: point bow at north (HDG 0)."""
        self.phase = "yaw_north"
        self._show_photo("settle")
        self._ensure_cal_direction()
        if self.want_vertical:
            self._set_cal_banner(detail="point bow NORTH — then yaw→0")
            pose = (
                "Stay ON END (cable down / sticker bow). Roll & pitch are already "
                "zeroed from the hold-still pose."
            )
        else:
            self._set_cal_banner(detail="point bow NORTH — then yaw→0")
            pose = "Stay flat, label up. Roll & pitch are already zeroed."
        self._set_info(
            "④ Point bow due NORTH (HDG → 0°)\n\n"
            f"{pose}\n\n"
            "Turn the housing in yaw only until HDG is near 0° (north), then "
            "press “Yaw→0 & invert checks”. That sets the yaw offset only.\n\n"
            f"{getattr(self, '_rp_level_note', '')}"
        )
        self.btn_cancel.configure(state=tk.NORMAL)
        self.btn_next.configure(
            text="Yaw→0 & invert checks →", state=tk.NORMAL
        )
        self.btn_skip.configure(state=tk.DISABLED, command=self._skip_to_mag)
        self.btn_skip_mag.configure(state=tk.DISABLED)
        self.progress.set("Turn yaw until HDG≈0°…")

    def _finish_yaw_north(self) -> None:
        self._ensure_cal_direction()
        if self.app is not None:
            self.app._zero_yaw_axis()
        _, _, by, hdg = self._wizard_boat_rpy()
        self._yaw_north_note = f"Yaw→0 at HDG was {hdg:.1f}° → boat yaw≈{by:+.1f}°"
        self._verify_notes = [
            getattr(self, "_rp_level_note", "R/P leveled"),
            self._yaw_north_note,
        ]
        self._begin_verify("roll")

    def _begin_verify(self, which: str) -> None:
        """Left-~90° motion on one axis — detect whether Invert_* is needed."""
        self.phase = f"verify_{which}"
        self._verify_which = which
        br, bp, by, _ = self._wizard_boat_rpy()
        self._verify_base = {"roll": br, "pitch": bp, "yaw": by}
        self._verify_signed = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self._show_photo("settle")
        self._ensure_cal_direction()
        if self.want_vertical:
            stay = "Stay ON END (cable down / sticker bow). "
        else:
            stay = "Stay in table pose. "
        tips = {
            "roll": (
                "⑤ Invert check — ROLL left ~90°\n\n"
                f"{stay}"
                "Heel the housing to PORT (mast tips left / port) about 90°, "
                "then return.\n\n"
                "We expect boat heel to go negative. If it goes the wrong way, "
                "Invert Roll is toggled automatically."
            ),
            "pitch": (
                "⑥ Invert check — PITCH / tilt left ~90°\n\n"
                f"{stay}"
                "Tilt BOW DOWN about 90°, then return.\n\n"
                "We expect boat trim to go negative. Wrong sign → Invert Pitch "
                "toggles automatically."
            ),
            "yaw": (
                "⑦ Invert check — YAW swing left ~90°\n\n"
                f"{stay}"
                "Swing the bow LEFT (port) about 90°, then return.\n\n"
                "We expect boat yaw / HDG to decrease. Wrong sign → Invert Yaw "
                "toggles automatically."
            ),
        }
        next_lbl = {
            "roll": "Next — pitch invert →",
            "pitch": "Next — yaw invert →",
            "yaw": "Finish",
        }
        detail = (
            f"invert {which} LEFT ~90°"
            + (" ON END" if self.want_vertical else "")
        )
        self._set_cal_banner(detail=detail)
        self._set_info(tips[which])
        self.btn_next.configure(text=next_lbl[which], state=tk.NORMAL)
        self.btn_skip.configure(state=tk.DISABLED)
        self.btn_skip_mag.configure(state=tk.DISABLED)
        self.btn_cancel.configure(state=tk.NORMAL)
        self.progress.set("Move LEFT ~90° on this axis…")

    def _toggle_invert(self, which: str) -> None:
        if self.app is None:
            return
        var = {
            "roll": self.app.invert_roll,
            "pitch": self.app.invert_pitch,
            "yaw": self.app.invert_yaw,
        }[which]
        var.set(not bool(var.get()))
        self.app._invert_manual = True

    def _analyze_invert(self, which: str) -> str:
        """Expect left/port/bow-down motion → negative delta on that axis."""
        signed = getattr(self, "_verify_signed", {}) or {}
        d = float(signed.get(which, 0.0))
        others = [
            abs(float(signed.get(k, 0.0)))
            for k in ("roll", "pitch", "yaw")
            if k != which
        ]
        expect_sign = -1.0
        label = {"roll": "heel", "pitch": "trim", "yaw": "yaw"}[which]
        if abs(d) < 25.0:
            return (
                f"{which}: weak motion ({label} Δ{d:+.0f}°) — "
                "could not judge invert (move farther next time)"
            )
        parts = [f"{which}: {label} Δ{d:+.0f}°"]
        if d * expect_sign < 0:
            self._toggle_invert(which)
            on = False
            if self.app is not None:
                on = {
                    "roll": self.app.invert_roll,
                    "pitch": self.app.invert_pitch,
                    "yaw": self.app.invert_yaw,
                }[which].get()
            parts.append(f"mirrored → Invert {which.title()} now {'ON' if on else 'OFF'}")
        else:
            parts.append("direction OK")
        if others and max(others) > abs(d) * 0.85:
            parts.append("⚠ other axes moved a lot")
        return " — ".join(parts)

    def _advance_verify(self, which: str, nxt: Optional[str]) -> None:
        note = self._analyze_invert(which)
        self._verify_notes.append(note)
        if nxt is None:
            self._finish_verify()
        else:
            self._begin_verify(nxt)

    def _finish_verify(self) -> None:
        self.phase = "done"
        self._show_photo("done")
        profile_note = ""
        if self.app is not None:
            path = self.app._save_mount_profile(
                vertical=self.want_vertical, quiet=True
            )
            if path:
                kind = "vertical" if self.want_vertical else "horizontal"
                profile_note = (
                    f"\n\n{kind.capitalize()} profile saved → {path.name}\n"
                    f"(also tools/calibrations/{kind}_latest.json)"
                )
        mount_txt = (
            "DIRECTION=vertical left ON (boat mount)."
            if self.want_vertical
            else "DIRECTION=horizontal left ON (table)."
        )
        notes = "\n".join(f"• {n}" for n in self._verify_notes) or "• (no notes)"
        inv = ""
        if self.app is not None:
            inv = (
                f"\nInvert roll/pitch/yaw = "
                f"{self.app.invert_roll.get()}/"
                f"{self.app.invert_pitch.get()}/"
                f"{self.app.invert_yaw.get()}"
            )
        self._set_info(
            f"Calibration complete + 9-axis locked.\n{mount_txt}\n\n"
            f"{getattr(self, '_mag_summary', '')}\n\n"
            f"Align / invert checks:\n{notes}"
            f"{inv}"
            f"{profile_note}\n\n"
            "Boat soft-iron still needs a GPS deviation table underway."
        )
        self.progress.set("")
        self.btn_next.configure(text="Close", state=tk.NORMAL)
        self.btn_skip.configure(state=tk.DISABLED)
        self.btn_skip_mag.configure(state=tk.DISABLED)

    def _draw_ellipse_plane(self, canvas: tk.Canvas, stats: dict, title: str) -> None:
        canvas.delete("all")
        w = max(canvas.winfo_width(), 120)
        h = max(canvas.winfo_height(), 120)
        pad = 12
        pts = stats.get("pts") or []
        if len(pts) < 8 or stats["r"] < 40:
            canvas.create_text(w / 2, h / 2, text="collecting…", fill="#9eb6c9",
                               font=("TkFixedFont", 9))
            return
        cx, cy, r = stats["cx"], stats["cy"], stats["r"]
        # Fit view to centre ± 2.2 r
        half = max(r * 2.2, 80.0)
        def to_screen(x: float, y: float) -> Tuple[float, float]:
            sx = pad + (x - cx + half) / (2 * half) * (w - 2 * pad)
            sy = pad + (half - (y - cy)) / (2 * half) * (h - 2 * pad)
            return sx, sy

        # Guide circle at median radius (ideal hard-iron sphere → circle in plane)
        steps = 48
        ring = []
        for i in range(steps + 1):
            a = 2 * math.pi * i / steps
            ring.extend(to_screen(cx + r * math.cos(a), cy + r * math.sin(a)))
        canvas.create_line(*ring, fill="#3a5166", width=1)

        ok_flags = stats.get("ok") or [True] * len(pts)
        # Decimate for speed
        step = max(1, len(pts) // 400)
        for i in range(0, len(pts), step):
            x, y = pts[i]
            sx, sy = to_screen(x, y)
            if ok_flags[i]:
                canvas.create_oval(sx - 1.5, sy - 1.5, sx + 1.5, sy + 1.5,
                                   fill="#7fdbca", outline="")
            else:
                canvas.create_oval(sx - 2, sy - 2, sx + 2, sy + 2,
                                   fill="#ff6b6b", outline="")
        canvas.create_text(
            6, 6, anchor="nw",
            text=f"{title} edge {stats['fill']:.0%}  out {stats['outliers']:.0%}",
            fill="#e8f1f8", font=("TkFixedFont", 8),
        )

    def _restore_master(self) -> None:
        if not getattr(self, "_master_iconified", False):
            return
        self._master_iconified = False
        try:
            self.master.deiconify()
            self.master.lift()
        except tk.TclError:
            pass

    def destroy(self) -> None:  # noqa: A003 — tk API
        self._restore_master()
        super().destroy()

    def _cancel(self) -> None:
        try:
            with self.state.lock:
                self.state.cal_collecting = False
            self.reader.send_calsw(CAL_NORMAL, save=False)
        except Exception:  # noqa: BLE001
            pass
        self.destroy()

    @staticmethod
    def _delta_signed(cur: float, base: float, circular: bool = False) -> float:
        d = cur - base
        if circular:
            d = (d + 180.0) % 360.0 - 180.0
        return d

    def _tick(self) -> None:
        if not self.winfo_exists():
            return
        if self.phase == "accel_run":
            left = self._accel_end - time.time()
            if left <= 0:
                self._finish_accel()
            else:
                self.progress.set(f"Hold still… {left:0.1f}s")
        elif self.phase == "mag_run":
            elapsed = time.time() - self._mag_start
            # Discard first seconds — CALSW entry + offset updates junk the cloud
            if not self._mag_warmed:
                if elapsed < self._MAG_WARMUP_S:
                    self.btn_next.configure(state=tk.DISABLED)
                    self.progress.set(
                        f"warming up… {self._MAG_WARMUP_S - elapsed:0.1f}s "
                        f"(ignoring samples while cal mode settles)"
                    )
                    self.after(100, self._tick)
                    return
                with self.state.lock:
                    self.state.cal_mag_samples.clear()
                self._mag_warmed = True
            with self.state.lock:
                samples = list(self.state.cal_mag_samples)
                mag = self.state.mag
            xy = mag_plane_ellipse(samples, 0, 1)
            xz = mag_plane_ellipse(samples, 0, 2)
            yz = mag_plane_ellipse(samples, 1, 2)
            # Redraw ellipses ~5 Hz
            self._ellipse_draw_i += 1
            if self._ellipse_draw_i % 2 == 0:
                self._draw_ellipse_plane(self.ellipse_canvases[0], xy, "XY")
                self._draw_ellipse_plane(self.ellipse_canvases[1], xz, "XZ")
                self._draw_ellipse_plane(self.ellipse_canvases[2], yz, "YZ")
            fx, fy, fz = xy["fill"], xz["fill"], yz["fill"]
            ox, oy, oz = xy["outliers"], xz["outliers"], yz["outliers"]
            uniq = len(set(samples)) if samples else 0
            ready = (
                elapsed >= self._MAG_MIN_S
                and min(fx, fy, fz) >= 0.75
                and max(ox, oy, oz) <= 0.20
                and uniq > 120
                and len(samples) > 250
            )
            tip = ""
            if len(samples) > 40 and uniq < 3:
                tip = "⚠ mag frozen — cancel & retry"
            elif max(ox, oy, oz) > 0.25 and len(samples) > 80:
                tip = "⚠ many red outliers — move away from metal / speakers"
            elif ready:
                tip = "✓ rings filled — Finish & save"
            elif elapsed < self._MAG_MIN_S:
                tip = f"keep tumbling… {self._MAG_MIN_S - elapsed:0.0f}s left"
            else:
                tip = "need edge fill ≥75% on all planes, outliers ≤20%"
            self.btn_next.configure(state=tk.NORMAL if ready else tk.DISABLED)
            self.progress.set(
                f"t={elapsed:0.0f}s  n={len(samples)} uniq={uniq}\n"
                f"XY [{_bar(fx)}] edge {fx:3.0%}  out {ox:3.0%}\n"
                f"XZ [{_bar(fy)}] edge {fy:3.0%}  out {oy:3.0%}\n"
                f"YZ [{_bar(fz)}] edge {fz:3.0%}  out {oz:3.0%}\n"
                f"mag=({mag[0]:.0f},{mag[1]:.0f},{mag[2]:.0f})  {tip}"
            )
        elif self.phase == "settle":
            left = self._settle_end - time.time()
            with self.state.lock:
                yaw = self.state.yaw
                sr, sp = self.state.roll, self.state.pitch
            self._settle_hdg.append(yaw_to_heading_deg(yaw))
            if left <= 0:
                self._finish_settle()
            else:
                self.progress.set(
                    f"Hold still… {left:0.1f}s   "
                    f"vehicle R/P={sr:+.1f}/{sp:+.1f}  "
                    f"HDG={yaw_to_heading_deg(yaw):.1f}°\n"
                    f"(roll/pitch offsets will be taken from this pose)"
                )
        elif self.phase == "yaw_north":
            if int(time.time() * 2) % 5 == 0:
                self._ensure_cal_direction()
            with self.state.lock:
                sr, sp = self.state.roll, self.state.pitch
                vert = self.state.vertical_mount
            br, bp, by, hdg = self._wizard_boat_rpy()
            mode = "VERT" if vert else ("HORIZ" if vert is False else "?")
            near = hdg < 8.0 or hdg > 352.0
            hint = "✓ near north — press Yaw→0" if near else "turn yaw until HDG≈0°"
            self.progress.set(
                f"DIRECTION={mode}  vehicle R/P={sr:+.1f}/{sp:+.1f}  "
                f"boat heel/trim/yaw={br:+.1f}/{bp:+.1f}/{by:+.1f}\n"
                f"HDG={hdg:06.1f}°   {hint}"
            )
        elif self.phase.startswith("verify_"):
            if int(time.time() * 2) % 7 == 0:
                self._ensure_cal_direction()
            br, bp, by, hdg = self._wizard_boat_rpy()
            which = getattr(self, "_verify_which", "roll")
            base = getattr(self, "_verify_base", None)
            signed = getattr(self, "_verify_signed", None)
            if base is not None and signed is not None:
                for key, val, circ in (
                    ("roll", br, False),
                    ("pitch", bp, False),
                    ("yaw", by, True),
                ):
                    d = self._delta_signed(val, base[key], circular=circ)
                    if abs(d) > abs(signed.get(key, 0.0)):
                        signed[key] = d
            expect = {"roll": "heel−", "pitch": "trim−", "yaw": "yaw−"}[which]
            d_pri = (signed or {}).get(which, 0.0)
            stay = "ON END" if self.want_vertical else "flat"
            self.progress.set(
                f"stay {stay}  heel/trim/yaw={br:+.1f}/{bp:+.1f}/{by:+.1f}  "
                f"HDG={hdg:.1f}°\n"
                f"expect {expect} on LEFT move   Δ={d_pri:+.0f}°   "
                f"Δr/p/y="
                f"{(signed or {}).get('roll', 0):+.0f}/"
                f"{(signed or {}).get('pitch', 0):+.0f}/"
                f"{(signed or {}).get('yaw', 0):+.0f}"
            )
        self.after(100, self._tick)

    def _wizard_boat_rpy(self) -> Tuple[float, float, float, float]:
        with self.state.lock:
            roll, pitch, yaw = self.state.roll, self.state.pitch, self.state.yaw
            vert = bool(self.state.vertical_mount) if self.state.vertical_mount is not None \
                else self.want_vertical
        if self.app is not None:
            return self.app._boat_pose(roll, pitch, yaw, vert)
        return roll, pitch, yaw, yaw_to_heading_deg(yaw)


def yaw_to_heading_deg(yaw: float) -> float:
    """WitMotion yaw (−180…180) → compass heading 0…360 (0 = North / bow ref)."""
    h = yaw % 360.0
    if h < 0:
        h += 360.0
    return h


def build_boat_mesh():
    """Sailboat in WitMotion vehicle axes: +Y bow, +X starboard, +Z up.

    After DIRECTION=vertical the sensor still reports that frame (sticker /
    physical +Z becomes forward = +Y in the angle output).
    """
    # Authored as (athwart, up, forward) then swapped to (X, Y=bow, Z=up).
    raw = [
        # Hull
        (0.0, -0.15, 1.35),   # 0 bow tip (low)
        (-0.55, -0.25, -1.0), # 1 port stern keel
        (0.55, -0.25, -1.0),  # 2 stbd stern keel
        (-0.65, 0.15, -1.05), # 3 port stern deck
        (0.65, 0.15, -1.05),  # 4 stbd stern deck
        (-0.50, 0.18, 0.35),  # 5 port mid gunwale
        (0.50, 0.18, 0.35),   # 6 stbd mid gunwale
        (0.0, 0.12, 1.15),    # 7 bow deck
        # Cabin
        (-0.28, 0.18, -0.15), # 8
        (0.28, 0.18, -0.15),  # 9
        (-0.28, 0.18, -0.75), # 10
        (0.28, 0.18, -0.75),  # 11
        (-0.28, 0.48, -0.20), # 12
        (0.28, 0.48, -0.20),  # 13
        (-0.28, 0.48, -0.70), # 14
        (0.28, 0.48, -0.70),  # 15
        # Mast + sail
        (0.0, 0.18, 0.25),    # 16 mast foot
        (0.0, 1.35, 0.25),    # 17 mast top
        (0.0, 1.20, 0.95),    # 18 sail forward tip (bow)
        (0.0, 0.25, 0.20),    # 19 sail tack
    ]
    # (x, up, fwd) → (x, fwd=+Y, up=+Z)
    verts = [(x, fwd, up) for (x, up, fwd) in raw]
    hull = "#c45c26"
    deck = "#e8d5b5"
    bottom = "#5c4033"
    cabin = "#f4f1ea"
    cabin_top = "#d4c4a8"
    sail = "#f7f7f2"
    mast = "#888888"
    faces = [
        # Bottom
        ((0, 1, 2), bottom),
        # Port hull
        ((0, 1, 5, 7), hull),
        # Stbd hull
        ((0, 7, 6, 2), hull),
        # Stern transom
        ((1, 2, 4, 3), hull),
        # Deck
        ((7, 5, 3, 4, 6), deck),
        # Port mid→stern gunwale fill
        ((5, 1, 3), hull),
        ((6, 4, 2), hull),
        # Cabin sides
        ((8, 9, 13, 12), cabin),
        ((10, 11, 15, 14), cabin),
        ((8, 12, 14, 10), cabin),
        ((9, 11, 15, 13), cabin),
        ((12, 13, 15, 14), cabin_top),
        # Sail (thin triangle, both sides same)
        ((17, 18, 19), sail),
    ]
    edges = [
        (0, 7), (7, 5), (5, 3), (3, 4), (4, 6), (6, 7),
        (1, 2), (1, 3), (2, 4),
        (16, 17), (17, 18), (18, 19), (19, 17),
        (8, 9), (9, 11), (11, 10), (10, 8),
        (12, 13), (13, 15), (15, 14), (14, 12),
    ]
    labels = {
        # face index → label (bow deck-ish)
    }
    return verts, faces, edges, labels


class CubeApp:
    def __init__(self, state: ImuState, reader: WitMotionReader):
        self.state = state
        self.reader = reader
        self.root = tk.Tk()
        self.root.title("WitMotion compass — 3D boat")
        self.root.geometry("960x720")

        # Vehicle-frame flips AFTER WitMotion DIRECTION (not PCB chip axes).
        # Vertical/−X=starboard defaults: invert roll + yaw (see _apply_inverts_for_mount).
        self.invert_roll = tk.BooleanVar(master=self.root, value=False)
        self.invert_pitch = tk.BooleanVar(master=self.root, value=False)
        self.invert_yaw = tk.BooleanVar(master=self.root, value=False)
        # Display-only Euler offsets (°) — align boat; applied before invert.
        self.offset_x = tk.DoubleVar(master=self.root, value=0.0)  # roll
        self.offset_y = tk.DoubleVar(master=self.root, value=0.0)  # pitch
        self.offset_z = tk.DoubleVar(master=self.root, value=0.0)  # yaw
        # Which mount Calibrate… / Save profile should use (set by mount buttons).
        self.mount_vertical_intent = False
        self._invert_manual = False  # True after user toggles a checkbox
        self._last_vert_for_invert: Optional[bool] = None

        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=8, pady=6)

        tk.Label(top, text="Invert (vehicle):").pack(side=tk.LEFT, padx=(0, 2))
        for text, var in (
            ("Roll", self.invert_roll),
            ("Pitch", self.invert_pitch),
            ("Yaw", self.invert_yaw),
        ):
            tk.Checkbutton(
                top, text=text, variable=var,
                command=self._on_invert_toggled,
            ).pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="Calibrate…", command=self._open_calib).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(top, text="Fix drift…", command=self._fix_drift).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(top, text="Vertical mount", command=self._set_vertical).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(top, text="Horizontal (table)", command=self._set_horizontal).pack(
            side=tk.RIGHT, padx=4
        )

        self.mount_mode = tk.StringVar(value="MOUNT: …")
        self.mount_banner = tk.Label(
            self.root, textvariable=self.mount_mode,
            font=("Helvetica", 13, "bold"), pady=5, anchor="center",
            bg="#1a2330", fg="#cfd8e3",
        )
        self.mount_banner.pack(fill=tk.X, padx=8, pady=(0, 2))

        off = tk.Frame(self.root)
        off.pack(fill=tk.X, padx=8, pady=(0, 4))
        # Offsets are on vehicle-frame roll/pitch/yaw AFTER WitMotion DIRECTION
        # (and viewer invert) — i.e. the compass orientation, not raw PCB X/Y/Z.
        tk.Label(off, text="Zero boat:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(off, text="Roll→0", command=lambda: self._zero_axis("x")).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(off, text="Pitch→0", command=lambda: self._zero_axis("y")).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(off, text="Yaw→0", command=lambda: self._zero_axis("z")).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(off, text="All→0", command=self._zero_all_axes).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(off, text="Clear offs", command=self._clear_offsets).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(off, text="Save profile", command=self._save_profile_clicked).pack(
            side=tk.RIGHT, padx=2)
        ttk.Button(off, text="Load profile", command=self._load_profile_clicked).pack(
            side=tk.RIGHT, padx=2)

        self.status = tk.StringVar(value="Starting…")
        tk.Label(self.root, textvariable=self.status, anchor="w", justify=tk.LEFT).pack(
            fill=tk.X, padx=8
        )

        self.canvas = tk.Canvas(self.root, bg="#0b1c2c", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.corners, face_defs, self.edges, self.face_labels = build_boat_mesh()
        self.faces = [(idxs, color) for idxs, color in face_defs]
        self._hdg_filt: Optional[float] = None
        self._hdg_still: Deque[float] = deque(maxlen=150)  # ~5s at 30 Hz
        self._autosaved_link = False
        self._mount_ui_synced = False

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._tick()
        self.root.after(1500, self._sync_mount_when_linked)

    def _require_link(self) -> bool:
        with self.state.lock:
            linked = self.state.linked
        if not linked:
            messagebox.showwarning(
                "No link",
                "Sensor not linked. Check USB wiring (TX↔RX) and baud, then retry.",
            )
            return False
        return True

    def _raw_rpy(self) -> Tuple[float, float, float]:
        """Sensor roll/pitch/yaw before offset and invert."""
        with self.state.lock:
            return self.state.roll, self.state.pitch, self.state.yaw

    def _warn_if_zeroing_high_tilt(self, roll: float, pitch: float) -> None:
        """Zeroing with |raw|≫0 only paints over a bad pose / wrong DIRECTION."""
        if abs(roll) < 45.0 and abs(pitch) < 45.0:
            return
        vert = self._current_vertical()
        mode = "VERTICAL" if vert else "HORIZONTAL"
        messagebox.showwarning(
            "Large raw tilt",
            f"Raw roll/pitch ≈ {roll:.0f}°/{pitch:.0f}° with DIRECTION={mode}.\n\n"
            "Zero boat only shifts the 3D mesh — it does not fix the sensor.\n"
            "With VERTICAL: hold cable-down / sticker-bow until raw R/P are "
            "near 0°, then Clear offs (or Zero again).\n"
            "If raw stays ~90° on end: DIRECTION may still be horizontal — "
            "click Vertical mount.",
        )

    def _zero_axis(self, axis: str) -> None:
        """Zero nautical Roll/Pitch/Yaw (heel/trim/HDG), not raw WitMotion names.

        WitMotion is Y-forward: heel = WM pitch, trim = WM roll, yaw = WM yaw.
        """
        wm_r, wm_p, wm_y = self._raw_rpy()
        self._warn_if_zeroing_high_tilt(wm_r, wm_p)
        if axis == "x":  # Roll→0 = heel → offset on WM pitch
            self.offset_x.set(-wm_p)
        elif axis == "y":  # Pitch→0 = trim → offset on WM roll
            self.offset_y.set(-wm_r)
        else:
            self.offset_z.set(-wm_y)
        self._save_mount_profile(vertical=self._current_vertical(), quiet=True)

    def _zero_rp_axes(self) -> None:
        """Zero heel/trim only (used after cal hold-still). Leaves yaw offset."""
        wm_r, wm_p, _ = self._raw_rpy()
        self.offset_x.set(-wm_p)  # heel
        self.offset_y.set(-wm_r)  # trim
        self._save_mount_profile(vertical=self._current_vertical(), quiet=True)

    def _zero_yaw_axis(self) -> None:
        """Zero yaw/HDG only (used when bow is pointed north)."""
        _, _, wm_y = self._raw_rpy()
        self.offset_z.set(-wm_y)
        self._save_mount_profile(vertical=self._current_vertical(), quiet=True)

    def _zero_all_axes(self) -> None:
        wm_r, wm_p, wm_y = self._raw_rpy()
        self._warn_if_zeroing_high_tilt(wm_r, wm_p)
        self.offset_x.set(-wm_p)  # heel
        self.offset_y.set(-wm_r)  # trim
        self.offset_z.set(-wm_y)
        self._save_mount_profile(vertical=self._current_vertical(), quiet=True)

    def _clear_offsets(self) -> None:
        self.offset_x.set(0.0)
        self.offset_y.set(0.0)
        self.offset_z.set(0.0)
        self._save_mount_profile(vertical=self._current_vertical(), quiet=True)

    def _current_vertical(self) -> bool:
        with self.state.lock:
            if self.state.vertical_mount is not None:
                return bool(self.state.vertical_mount)
        return bool(self.mount_vertical_intent)

    def _on_invert_toggled(self) -> None:
        """User overrode mount defaults — stop auto-reset until mount button."""
        self._invert_manual = True

    @staticmethod
    def _mount_invert_defaults(vertical: bool) -> Tuple[bool, bool, bool]:
        """Vehicle-frame invert defaults for DIRECTION (+ −X=starboard).

        Horizontal/table: identity.
        Vertical boat (−X starboard): invert roll+pitch+yaw (verified on desk).
        """
        if vertical:
            return True, True, True  # roll, pitch, yaw
        return False, False, False

    def _apply_inverts_for_mount(self, vertical: bool, force: bool = False) -> None:
        """Set Invert roll/pitch/yaw from the active sensor mount."""
        if self._invert_manual and not force:
            return
        ir, ip, iy = self._mount_invert_defaults(vertical)
        self.invert_roll.set(ir)
        self.invert_pitch.set(ip)
        self.invert_yaw.set(iy)
        self._last_vert_for_invert = bool(vertical)
        if force:
            self._invert_manual = False

    def _boat_pose(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        vertical: bool,
    ) -> Tuple[float, float, float, float]:
        """Map WitMotion angles → nautical heel/trim/yaw + HDG.

        WitMotion / boat mesh are +Y bow, +X starboard, +Z up:
          WM roll  (about X) = trim (bow up/down)
          WM pitch (about Y) = heel (port/starboard)  ← “Roll” in the UI
          WM yaw   (about Z) = heading
        UI Roll/Pitch/Yaw and Invert_* refer to these nautical axes.
        Returns (heel, trim, yaw, heading_deg) for status + verify.
        """
        del vertical  # mapping is the same; DIRECTION already applied in sensor
        ox = float(self.offset_x.get())  # heel offset (Roll→0)
        oy = float(self.offset_y.get())  # trim offset (Pitch→0)
        oz = float(self.offset_z.get())
        heel = pitch + ox
        trim = roll + oy
        yaw_b = yaw + oz
        if self.invert_roll.get():
            heel = -heel
        if self.invert_pitch.get():
            trim = -trim
        if self.invert_yaw.get():
            yaw_b = -yaw_b
        heading = yaw_to_heading_deg(yaw_b)
        return heel, trim, yaw_b, heading

    def _mesh_euler_args(
        self, heel: float, trim: float, yaw_b: float
    ) -> Tuple[float, float, float]:
        """Euler args for Y-forward mesh: Rx(trim), Ry(heel), Rz(yaw)."""
        # _euler_matrix(a,b,c) = Rz(c)*Ry(b)*Rx(a) — pass trim as first arg.
        return trim, heel, -yaw_b

    def _wait_mag_bias(self, timeout_s: float = 1.2) -> Optional[Tuple[int, int, int]]:
        """Request HX/HY/HZOFFSET and wait briefly for the 0x5F reply."""
        before = time.time()
        self.reader.read_reg(REG_HXOFFSET)
        self.reader.read_reg(REG_DIRECTION)
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self.state.lock:
                bias = self.state.mag_bias
                ts = self.state.mag_bias_ts
            if bias is not None and ts >= before:
                return bias
            time.sleep(0.05)
            self.root.update_idletasks()
        with self.state.lock:
            return self.state.mag_bias

    def _profile_path_latest(self, vertical: bool) -> Path:
        name = "vertical_latest.json" if vertical else "horizontal_latest.json"
        return CALIBRATIONS_DIR / name

    def _apply_viewer_profile(self, profile: dict, vertical: Optional[bool] = None) -> None:
        viewer = profile.get("viewer") or {}
        install = profile.get("install") or {}
        if vertical is None:
            vertical = bool(install.get("vertical_mount", False))
        # Prefer saved inverts when present; else mount defaults for this DIRECTION.
        if "invert_roll" in viewer or "invert_yaw" in viewer or "invert_pitch" in viewer:
            self.invert_roll.set(bool(viewer.get("invert_roll", False)))
            self.invert_pitch.set(bool(viewer.get("invert_pitch", False)))
            self.invert_yaw.set(bool(viewer.get("invert_yaw", False)))
            self._invert_manual = True  # profile is an explicit choice
        else:
            self._apply_inverts_for_mount(bool(vertical), force=True)
        self.offset_x.set(float(viewer.get("offset_x_deg", 0.0)))
        self.offset_y.set(float(viewer.get("offset_y_deg", 0.0)))
        self.offset_z.set(float(viewer.get("offset_z_deg", 0.0)))
        self._last_vert_for_invert = bool(vertical)

    def _restore_viewer_for_mount(self, vertical: bool) -> None:
        """Apply saved invert/offset for the active DIRECTION only."""
        latest = self._profile_path_latest(vertical)
        if not latest.exists():
            self._apply_inverts_for_mount(vertical, force=True)
            return
        try:
            profile = json.loads(latest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            self._apply_inverts_for_mount(vertical, force=True)
            return
        self._apply_viewer_profile(profile, vertical=vertical)

    def _tilt_issue(
        self, vert: Optional[bool], roll: float, pitch: float, linked: bool
    ) -> Optional[str]:
        """Classify high raw tilt. Only HORIZONTAL+on-end is a DIRECTION mismatch."""
        if not linked or (abs(pitch) <= 55.0 and abs(roll) <= 55.0):
            return None
        if vert is False:
            return "direction"  # on-end while algorithm still horizontal
        if vert is True:
            return "pose"  # DIRECTION ok; housing not upright in vertical frame
        return None

    def _update_mount_banner(self, vert: Optional[bool], roll: float, pitch: float,
                             linked: bool) -> None:
        """Big always-visible DIRECTION mode + mismatch hint."""
        issue = self._tilt_issue(vert, roll, pitch, linked)
        if vert is True:
            text = "DIRECTION: VERTICAL  ·  cable down · sticker bow · −X starboard"
            bg, fg = "#5c2b0a", "#ffd166"
            if issue == "pose":
                text += "  ⚠ raw tilt high — align mount (Zero only hides it)"
                bg, fg = "#6b4a1c", "#ffe0a0"
        elif vert is False:
            text = "DIRECTION: HORIZONTAL / table  ·  sensor flat, label up"
            bg, fg = "#0a3a4a", "#9ee7ff"
            if issue == "direction":
                text += "  ⚠ DIRECTION mismatch — lay flat, or click Vertical mount"
                bg, fg = "#6b1c1c", "#ffb4b4"
        else:
            text = "DIRECTION: unknown (waiting for sensor…)"
            bg, fg = "#1a2330", "#cfd8e3"
        self.mount_mode.set(text)
        self.mount_banner.configure(bg=bg, fg=fg)
        kind = "vertical" if vert else ("horizontal" if vert is False else "?")
        self.root.title(f"WitMotion compass — {kind}")

    def _sync_mount_when_linked(self) -> None:
        """After link: read DIRECTION, restore matching viewer profile."""
        if self._mount_ui_synced:
            return
        with self.state.lock:
            linked = self.state.linked
            vert = self.state.vertical_mount
        if not linked or vert is None:
            self.root.after(500, self._sync_mount_when_linked)
            return
        self._mount_ui_synced = True
        self.mount_vertical_intent = bool(vert)
        # Always restore the saved mount profile (inverts + offsets), including
        # large yaw/trim zeros from a real level/north align — do not discard
        # |offset|>45° (that wiped the working vertical cal on reconnect).
        latest = self._profile_path_latest(bool(vert))
        if latest.exists():
            self._restore_viewer_for_mount(bool(vert))
        else:
            self._apply_inverts_for_mount(bool(vert), force=True)
            if not self._autosaved_link:
                self._autosaved_link = True
                self._save_mount_profile(vertical=bool(vert), quiet=True)

    def _save_profile_clicked(self) -> None:
        self._save_mount_profile(vertical=self._current_vertical(), quiet=False)

    def _load_profile_clicked(self) -> None:
        self._load_mount_profile(vertical=self._current_vertical())

    def _save_mount_profile(
        self, vertical: bool, quiet: bool = False
    ) -> Optional[Path]:
        """Snapshot viewer offsets + sensor mag bias for horizontal or vertical."""
        CALIBRATIONS_DIR.mkdir(parents=True, exist_ok=True)
        bias = None
        with self.state.lock:
            linked = self.state.linked
        if linked:
            bias = self._wait_mag_bias()
        with self.state.lock:
            roll, pitch, yaw = self.state.roll, self.state.pitch, self.state.yaw
            mag = self.state.mag
            cov = self.state.mag_cal_coverage
            port, baud = self.state.port, self.state.baud
            nine = self.state.nine_axis
            direction = self.state.reg_direction
            axis6 = self.state.reg_axis6
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        kind = "vertical" if vertical else "horizontal"
        profile = {
            "name": f"{kind}_mount",
            "saved_at_utc": stamp,
            "note": (
                f"DIRECTION={kind}, 9-axis. Sensor flash also holds mag bias after SAVE."
            ),
            "install": {
                "direction": kind,
                "vertical_mount": bool(vertical),
                "nine_axis": True if nine is None else bool(nine),
                "reg_direction": direction,
                "reg_axis6": axis6,
            },
            "viewer": {
                "invert_roll": bool(self.invert_roll.get()),
                "invert_pitch": bool(self.invert_pitch.get()),
                "invert_yaw": bool(self.invert_yaw.get()),
                "offset_x_deg": float(self.offset_x.get()),
                "offset_y_deg": float(self.offset_y.get()),
                "offset_z_deg": float(self.offset_z.get()),
            },
            "sensor": {
                "port": port,
                "baud": baud,
                "mag_bias_hx_hy_hz": list(bias) if bias is not None else None,
                "mag_cal_coverage_xy_xz_yz": list(cov),
                "sample_rpy_deg": [roll, pitch, yaw],
                "sample_mag": list(mag),
            },
        }
        stamped = CALIBRATIONS_DIR / f"{kind}_{stamp}.json"
        latest = self._profile_path_latest(vertical)
        text = json.dumps(profile, indent=2) + "\n"
        stamped.write_text(text, encoding="utf-8")
        latest.write_text(text, encoding="utf-8")
        if not quiet:
            bias_txt = (
                f"HX/HY/HZOFFSET={bias}" if bias is not None
                else "mag bias not read (still on sensor flash if you SAVEd)"
            )
            messagebox.showinfo(
                f"{kind.capitalize()} profile saved",
                f"Wrote:\n{stamped}\n{latest}\n\n"
                f"DIRECTION={kind}\n"
                f"Viewer offs X/Y/Z="
                f"{profile['viewer']['offset_x_deg']:.1f}/"
                f"{profile['viewer']['offset_y_deg']:.1f}/"
                f"{profile['viewer']['offset_z_deg']:.1f}\n"
                f"{bias_txt}",
            )
        return stamped

    def _load_mount_profile(self, vertical: bool) -> None:
        latest = self._profile_path_latest(vertical)
        kind = "vertical" if vertical else "horizontal"
        if not latest.exists():
            messagebox.showwarning(
                "No profile",
                f"No saved {kind} profile at\n{latest}\n\n"
                f"Use “Save profile” while in {kind} mode first.",
            )
            return
        try:
            profile = json.loads(latest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            messagebox.showerror("Load failed", str(exc))
            return
        self.mount_vertical_intent = vertical
        self._apply_viewer_profile(profile, vertical=vertical)
        wrote_bias = False
        sensor = profile.get("sensor") or {}
        bias = sensor.get("mag_bias_hx_hy_hz")
        if self._require_link():
            self.reader.set_install_direction(vertical=vertical, save=True)
            self.reader.ensure_nine_axis(save=True)
            if isinstance(bias, list) and len(bias) == 3:
                self.reader.write_mag_bias(int(bias[0]), int(bias[1]), int(bias[2]),
                                           save=True)
                wrote_bias = True
        msg = (
            f"Loaded {latest.name}\n"
            f"DIRECTION={kind}\n"
            f"offs X/Y/Z={self.offset_x.get():.1f}/"
            f"{self.offset_y.get():.1f}/{self.offset_z.get():.1f}"
        )
        if wrote_bias:
            msg += f"\nRestored HX/HY/HZOFFSET={tuple(bias)}"
        elif bias:
            msg += "\n(mag bias in file but sensor not linked — not written)"
        messagebox.showinfo(f"{kind.capitalize()} profile loaded", msg)

    def _set_horizontal(self) -> None:
        if not self._require_link():
            return
        self.mount_vertical_intent = False
        self.reader.set_install_direction(vertical=False, save=True)
        self._apply_inverts_for_mount(False, force=True)
        self.offset_x.set(0.0)
        self.offset_y.set(0.0)
        self.offset_z.set(0.0)
        messagebox.showinfo(
            "Horizontal (table)",
            "DIRECTION=horizontal saved. Invert roll/pitch/yaw reset for "
            "table mount (vehicle = sensor frame).\n\n"
            "Use flat, label up. Banner should read HORIZONTAL.",
        )

    def _set_vertical(self) -> None:
        if not self._require_link():
            return
        self.mount_vertical_intent = True
        self.reader.set_install_direction(vertical=True, save=True)
        # DIRECTION remaps axes in the sensor; then −X=starboard needs
        # invert roll + yaw in the vehicle frame (same as ESP32 web UI).
        self._apply_inverts_for_mount(True, force=True)
        self.offset_x.set(0.0)
        self.offset_y.set(0.0)
        self.offset_z.set(0.0)
        messagebox.showinfo(
            "Vertical mount",
            "DIRECTION=vertical saved. Invert Yaw ON for −X=starboard "
            "(vehicle frame; matches ESP32). Invert Roll/Pitch off.\n\n"
            "Mount: cable down, sticker toward bow, −X to starboard.\n"
            "Stand on end until vehicle R/P ≈ 0°, then Calibrate… "
            "(prefer Skip accel → mag so VERTICAL stays on).",
        )

    def _fix_drift(self) -> None:
        """Exit cal mode, lock 9-axis, measure still HDG span, guide next step."""
        if not self._require_link():
            return
        self.reader.send_calsw(CAL_NORMAL, save=False)
        self.reader.ensure_nine_axis(save=True)
        self.reader.ensure_mag_output()
        with self.state.lock:
            vert = self.state.vertical_mount
        if vert is None:
            self.reader.read_reg(REG_DIRECTION)
            time.sleep(0.3)
            with self.state.lock:
                vert = self.state.vertical_mount
        self.reader.read_reg(REG_AXIS6)
        self.reader.read_reg(REG_HXOFFSET)
        # Collect ~5s of still samples
        self._hdg_still.clear()
        t_end = time.time() + 5.0
        rolls: List[float] = []
        pitches: List[float] = []
        yaws: List[float] = []
        while time.time() < t_end:
            with self.state.lock:
                r, p, y = self.state.roll, self.state.pitch, self.state.yaw
                nine = self.state.nine_axis
                bias = self.state.mag_bias
            rolls.append(r)
            pitches.append(p)
            yaws.append(y)
            self.root.update()
            time.sleep(0.05)
        if not yaws:
            messagebox.showerror("Fix drift", "No angle samples — check USB link.")
            return

        def _circ_span(vals: List[float]) -> float:
            if len(vals) < 5:
                return 0.0
            lo, hi = min(vals), max(vals)
            span = hi - lo
            if span > 180:
                adj = sorted(((v + 180) % 360) for v in vals)
                span = adj[-1] - adj[0]
            return span

        r_m = sum(rolls) / len(rolls)
        p_m = sum(pitches) / len(pitches)
        y_span = _circ_span(yaws)
        pose_ok = abs(r_m) < 15.0 and abs(p_m) < 15.0
        mode = "VERTICAL" if vert else ("HORIZONTAL" if vert is False else "?")
        axis = "9-axis" if nine else "NOT 9-axis (will drift!)"
        lines = [
            f"DIRECTION={mode}  algorithm={axis}",
            f"HX/HY/HZOFFSET={bias}",
            f"Mean raw R/P ≈ {r_m:.1f}° / {p_m:.1f}°",
            f"HDG wander while (hopefully) still ≈ {y_span:.1f}° over 5s",
            "",
        ]
        if nine is False:
            lines.append("→ 9-axis was off; it is forced ON and saved now.")
        if not pose_ok:
            lines.append(
                "→ Pose is the problem: raw tilt is far from 0°. "
                "With VERTICAL, tip the housing (cable down / sticker bow / "
                "−X starboard) until raw R/P ≈ 0. Zero boat will NOT fix drift."
            )
        elif y_span > 3.0:
            lines.append(
                "→ Pose looks OK but HDG still wanders — re-run Calibrate… "
                "magnetometer ≥1 m from iron/laptop, with this DIRECTION left on."
            )
        else:
            lines.append(
                "→ Stillness looks good. If HDG only goes wrong after you turn "
                "and stop, redo mag cal farther from metal / speakers."
            )
        messagebox.showinfo("Fix drift — diagnosis", "\n".join(lines))
        if not pose_ok or y_span > 3.0:
            if messagebox.askyesno(
                "Calibrate now?",
                "Open the calibration wizard now?\n"
                "(Accel lays flat briefly; mag uses your current mount mode.)",
            ):
                self._open_calib()

    def _open_calib(self) -> None:
        if not self._require_link():
            return
        want_vertical = self._current_vertical() or self.mount_vertical_intent
        # Do NOT force horizontal here — wizard handles accel (flat) then
        # restores vertical for mag when want_vertical.
        try:
            CalibWizard(
                self.root, self.reader, self.state,
                app=self, want_vertical=want_vertical,
            )
        except Exception as exc:  # noqa: BLE001
            try:
                self.root.deiconify()
            except tk.TclError:
                pass
            messagebox.showerror(
                "Calibration",
                f"Could not open calibration wizard:\n{exc}",
            )

    def on_close(self) -> None:
        self.root.destroy()

    def _tick(self) -> None:
        self._draw()
        self.root.after(33, self._tick)

    def _euler_matrix(self, roll: float, pitch: float, yaw: float):
        r, p, y = map(math.radians, (roll, pitch, yaw))
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    def _rotate(self, v, R):
        x, y, z = v
        return (
            R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z,
        )

    def _project(self, v, w, h):
        # Stern camera: look forward along +Y (bow into scene), +Z up, +X starboard
        x, y, z = v
        dist = 4.5
        depth = y + dist
        if depth < 0.2:
            depth = 0.2
        scale = min(w, h) * 0.34 / (depth / dist)
        # Slight elevated viewpoint so deck/sail read clearly
        return w * 0.50 + x * scale, h * 0.62 - z * scale - y * scale * 0.08, depth

    def _draw_compass(self, c: tk.Canvas, heading: float, w: int, h: int) -> None:
        """Fixed north-up rose; boat heading as rotating lubber line + big digits."""
        cx = w - 110
        cy = 110
        r = 78
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#dce7f2", width=2,
                      fill="#132433")
        c.create_oval(cx - r + 10, cy - r + 10, cx + r - 10, cy + r - 10,
                      outline="#3a5166", width=1)
        for deg in range(0, 360, 10):
            rad = math.radians(deg)
            # Screen: 0° = up (north), clockwise
            outer = r - 4
            inner = r - (16 if deg % 30 == 0 else 10)
            x1 = cx + outer * math.sin(rad)
            y1 = cy - outer * math.cos(rad)
            x2 = cx + inner * math.sin(rad)
            y2 = cy - inner * math.cos(rad)
            c.create_line(x1, y1, x2, y2, fill="#9eb6c9",
                          width=2 if deg % 30 == 0 else 1)
        for deg, lab in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            rad = math.radians(deg)
            tx = cx + (r - 28) * math.sin(rad)
            ty = cy - (r - 28) * math.cos(rad)
            c.create_text(tx, ty, text=lab, fill="#ff6b6b" if lab == "N" else "#e8f1f8",
                          font=("Helvetica", 11, "bold"))
        # Heading needle (from centre toward heading)
        rad = math.radians(heading)
        nx = cx + (r - 22) * math.sin(rad)
        ny = cy - (r - 22) * math.cos(rad)
        bx = cx - 14 * math.sin(rad)
        by = cy + 14 * math.cos(rad)
        c.create_line(bx, by, nx, ny, fill="#ffd166", width=3, arrow=tk.LAST)
        c.create_text(cx, cy + r + 22, text=f"{heading:05.1f}°",
                      fill="#ffd166", font=("Helvetica", 18, "bold"))
        c.create_text(cx, cy + r + 42, text="HDG", fill="#9eb6c9",
                      font=("Helvetica", 10))

    def _draw(self) -> None:
        with self.state.lock:
            roll, pitch, yaw = self.state.roll, self.state.pitch, self.state.yaw
            linked = self.state.linked
            port, baud = self.state.port, self.state.baud
            err = self.state.last_error
            frames = self.state.frames
            mag = self.state.mag
            gx, gy, gz = self.state.gyro_dps
            nine = self.state.nine_axis
            vert = self.state.vertical_mount

        # DIRECTION change always resets Invert_* to that mount’s vehicle defaults.
        if vert is not None and vert != self._last_vert_for_invert:
            self._apply_inverts_for_mount(bool(vert), force=True)

        heel, trim, yaw_b, heading_raw = self._boat_pose(
            roll, pitch, yaw, vert is True
        )
        # Keep names roll_b/pitch_b as nautical heel/trim for status + verify.
        roll_b, pitch_b = heel, trim
        ox = float(self.offset_x.get())
        oy = float(self.offset_y.get())
        oz = float(self.offset_z.get())
        rate = math.sqrt(gx * gx + gy * gy + gz * gz)
        settling = rate > 8.0  # °/s — still moving
        # Track unfiltered HDG when nearly still — exposes real sensor drift.
        if not settling:
            self._hdg_still.append(heading_raw)
        else:
            self._hdg_still.clear()
        hdg_wander = 0.0
        if len(self._hdg_still) >= 30:
            hs = list(self._hdg_still)
            lo, hi = min(hs), max(hs)
            hdg_wander = hi - lo
            if hdg_wander > 180:
                adj = sorted(((h + 180) % 360) for h in hs)
                hdg_wander = adj[-1] - adj[0]
        # Light display filter when nearly still (does not change sensor).
        if self._hdg_filt is None:
            self._hdg_filt = heading_raw
        else:
            # shortest-path blend on the circle
            d = (heading_raw - self._hdg_filt + 540.0) % 360.0 - 180.0
            # Less smoothing when still so drift is visible, not hidden.
            alpha = 0.45 if settling else 0.25
            self._hdg_filt = (self._hdg_filt + alpha * d) % 360.0
        heading = self._hdg_filt
        cal = calibration_report(self.state)
        link = "LINK OK" if linked else "NO LINK"
        axis = "9-axis" if nine else "⚠ 6-axis"
        mount = "VERTICAL" if vert else ("HORIZONTAL" if vert is False else "mount?")
        tilt_issue = self._tilt_issue(vert, roll, pitch, linked)
        mount_warn = ""
        if nine is False:
            mount_warn = "  ⚠ 6-axis ON — yaw will drift; click Fix drift…"
        elif tilt_issue == "direction":
            mount_warn = "  ⚠ DIRECTION mismatch — lay flat, or Vertical mount"
        elif tilt_issue == "pose" or (linked and (abs(roll) > 20.0 or abs(pitch) > 20.0)):
            mount_warn = (
                "  ⚠ raw tilt not ~0° — mag HDG will be messy; align mount "
                "(Fix drift…)"
            )
        elif linked and hdg_wander > 3.0:
            mount_warn = (
                f"  ⚠ HDG wanders ±{hdg_wander/2:.1f}° while still — "
                "re-cal mag away from iron (Fix drift…)"
            )
        elif linked and (abs(ox) > 45.0 or abs(oy) > 45.0):
            mount_warn = "  ⚠ large Zero offset — boat looks level but raw tilt is not"
        self._update_mount_banner(vert, roll, pitch, linked)
        settle_txt = "  settling…" if settling else ""
        wander_txt = (
            f"  stillΔ={hdg_wander:.1f}°" if len(self._hdg_still) >= 30 else ""
        )
        # After DIRECTION=vertical, R/P/Y are already vehicle-frame (firmware).
        frame = "vehicle" if vert else "sensor"
        self.status.set(
            f"{link}  {port or '?'} @ {baud or '?'} baud  {axis}  DIRECTION={mount}  "
            f"frames={frames}{mount_warn}\n"
            f"HDG {heading:06.2f}°{settle_txt}{wander_txt}   "
            f"WM R/P/Y={roll:6.1f}/{pitch:6.1f}/{yaw:6.1f}  "
            f"boat heel/trim/yaw={roll_b:5.1f}/{pitch_b:5.1f}/{yaw_b:5.1f}  "
            f"off={ox:.1f}/{oy:.1f}/{oz:.1f}  "
            f"mag=({mag[0]:.0f},{mag[1]:.0f},{mag[2]:.0f})\n"
            f"{cal}"
            + (f"\nError: {err}" if err and not linked else "")
        )

        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or 800
        h = c.winfo_height() or 600

        # Soft water horizon band
        c.create_rectangle(0, h * 0.55, w, h, fill="#0e3048", outline="")
        c.create_rectangle(0, 0, w, h * 0.55, fill="#0b1c2c", outline="")

        # Persistent mode chip on the 3D view (matches banner).
        if vert is True:
            chip, chip_fg = "VERTICAL", "#ffd166"
        elif vert is False:
            chip, chip_fg = "HORIZONTAL", "#9ee7ff"
        else:
            chip, chip_fg = "MOUNT?", "#cfd8e3"
        c.create_rectangle(16, 16, 16 + 11 * len(chip) + 28, 48,
                           fill="#121a24", outline=chip_fg, width=2)
        c.create_text(30, 32, anchor="w", text=chip, fill=chip_fg,
                      font=("Helvetica", 14, "bold"))
        if tilt_issue == "direction":
            c.create_text(30, 58, anchor="w", text="DIRECTION mismatch",
                          fill="#ff6b6b", font=("Helvetica", 11, "bold"))
        elif tilt_issue == "pose":
            c.create_text(30, 58, anchor="w", text="align mount (raw tilt)",
                          fill="#ffd166", font=("Helvetica", 11, "bold"))
        elif abs(ox) > 45.0 or abs(oy) > 45.0:
            c.create_text(30, 58, anchor="w", text="large Zero hides tilt",
                          fill="#ffb347", font=("Helvetica", 11, "bold"))

        # Compass HDG increases clockwise (nautical); RH Euler yaw is CCW from
        # above — negate so the boat turns the same way as the rose needle.
        # Nautical heel/trim → mesh Rx(trim), Ry(heel), Rz(-yaw)
        ea, eb, ec = self._mesh_euler_args(roll_b, pitch_b, yaw_b)
        R = self._euler_matrix(ea, eb, ec)
        pts3 = [self._rotate(p, R) for p in self.corners]
        pts2 = [self._project(p, w, h) for p in pts3]

        face_draw = []
        for idxs, color in self.faces:
            depth = sum(pts2[i][2] for i in idxs) / len(idxs)
            face_draw.append((depth, idxs, color))
        face_draw.sort(key=lambda t: t[0])

        for _, idxs, color in face_draw:
            coords = []
            for i in idxs:
                coords.extend(pts2[i][:2])
            c.create_polygon(coords, fill=color, outline="#1a1a1a", width=1)

        for a, b in self.edges:
            c.create_line(pts2[a][0], pts2[a][1], pts2[b][0], pts2[b][1],
                          fill="#f0e6d8", width=1)

        # Bow label in world after project of bow tip (vert 0)
        c.create_text(pts2[0][0], pts2[0][1] - 14, text="BOW", fill="#ffd166",
                      font=("Helvetica", 11, "bold"))

        self._draw_compass(c, heading, w, h)

        # Large heading readout (left, below mode chip)
        c.create_text(24, 88, anchor="w", text=f"{heading:05.1f}°",
                      fill="#ffd166", font=("Helvetica", 36, "bold"))
        c.create_text(28, 120, anchor="w", text="compass 0–360°",
                      fill="#9eb6c9", font=("Helvetica", 11))

    def run(self) -> None:
        self.root.mainloop()


def save_horizontal_profile_headless(
    port: Optional[str],
    baud: Optional[int],
    offset_x: float = 0.0,
    offset_y: float = 0.0,
    offset_z: float = 0.0,
    invert_roll: bool = False,
    invert_pitch: bool = False,
    invert_yaw: bool = False,
    timeout_s: float = 8.0,
) -> Path:
    """Read HX/HY/HZOFFSET from the sensor and write tools/calibrations/*.json."""
    CALIBRATIONS_DIR.mkdir(parents=True, exist_ok=True)
    # Merge viewer fields from an existing profile if present (keep offs).
    latest = CALIBRATIONS_DIR / "horizontal_latest.json"
    if latest.exists():
        try:
            prev = json.loads(latest.read_text(encoding="utf-8"))
            v = prev.get("viewer") or {}
            offset_x = float(v.get("offset_x_deg", offset_x))
            offset_y = float(v.get("offset_y_deg", offset_y))
            offset_z = float(v.get("offset_z_deg", offset_z))
            invert_roll = bool(v.get("invert_roll", invert_roll))
            invert_pitch = bool(v.get("invert_pitch", invert_pitch))
            invert_yaw = bool(v.get("invert_yaw", invert_yaw))
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            pass

    state = ImuState()
    reader = WitMotionReader(state, port, baud)
    reader.start()
    try:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with state.lock:
                if state.linked:
                    break
            time.sleep(0.05)
        with state.lock:
            if not state.linked:
                raise SystemExit(
                    f"No sensor link within {timeout_s:.0f}s "
                    f"(port={state.port!r} err={state.last_error!r})"
                )
        reader.read_reg(REG_HXOFFSET)
        reader.read_reg(REG_DIRECTION)
        bias = None
        direction = None
        axis6 = None
        roll = pitch = yaw = 0.0
        mag = (0.0, 0.0, 0.0)
        t1 = time.time()
        while time.time() - t1 < 2.0:
            with state.lock:
                if state.mag_bias is not None:
                    bias = state.mag_bias
                direction = state.reg_direction
                axis6 = state.reg_axis6
                roll, pitch, yaw = state.roll, state.pitch, state.yaw
                mag = state.mag
                port_s, baud_s = state.port, state.baud
            if bias is not None and direction is not None:
                break
            time.sleep(0.05)
        with state.lock:
            port_s, baud_s = state.port, state.baud
            if bias is None:
                bias = state.mag_bias
            if direction is None:
                direction = state.reg_direction
            if axis6 is None:
                axis6 = state.reg_axis6
            roll, pitch, yaw = state.roll, state.pitch, state.yaw
            mag = state.mag
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        profile = {
            "name": "horizontal_table",
            "saved_at_utc": stamp,
            "note": (
                "Good flat-table calibration: DIRECTION=horizontal, 9-axis, "
                "stable HDG when level. Sensor flash also holds offsets after SAVE."
            ),
            "install": {
                "direction": "horizontal",
                "vertical_mount": False,
                "nine_axis": True if axis6 is None else (int(axis6) & 0xFF) == 0,
                "reg_direction": direction,
                "reg_axis6": axis6,
            },
            "viewer": {
                "invert_roll": invert_roll,
                "invert_pitch": invert_pitch,
                "invert_yaw": invert_yaw,
                "offset_x_deg": offset_x,
                "offset_y_deg": offset_y,
                "offset_z_deg": offset_z,
            },
            "sensor": {
                "port": port_s,
                "baud": baud_s,
                "mag_bias_hx_hy_hz": list(bias) if bias is not None else None,
                "mag_cal_coverage_xy_xz_yz": None,
                "sample_rpy_deg": [roll, pitch, yaw],
                "sample_mag": list(mag),
            },
        }
        stamped = CALIBRATIONS_DIR / f"horizontal_{stamp}.json"
        text = json.dumps(profile, indent=2) + "\n"
        stamped.write_text(text, encoding="utf-8")
        latest.write_text(text, encoding="utf-8")
        print(f"Wrote {stamped}")
        print(f"Wrote {latest}")
        print(f"mag_bias={bias} viewer_offs={offset_x}/{offset_y}/{offset_z}")
        return stamped
    finally:
        reader.stop()
        reader.join(timeout=1.0)


def main() -> None:
    ap = argparse.ArgumentParser(description="WitMotion 3D boat + calibration")
    ap.add_argument("--port", default=None, help="Serial port (default: auto)")
    ap.add_argument("--baud", type=int, default=None,
                    help="Baud rate (default: auto-detect)")
    ap.add_argument(
        "--save-horizontal-profile",
        action="store_true",
        help="Headless: read mag bias + write tools/calibrations/horizontal_*.json",
    )
    ap.add_argument("--offset-x", type=float, default=0.0)
    ap.add_argument("--offset-y", type=float, default=0.0)
    ap.add_argument("--offset-z", type=float, default=0.0)
    args = ap.parse_args()

    if args.save_horizontal_profile:
        save_horizontal_profile_headless(
            args.port, args.baud,
            offset_x=args.offset_x,
            offset_y=args.offset_y,
            offset_z=args.offset_z,
        )
        return

    state = ImuState()
    reader = WitMotionReader(state, args.port, args.baud)
    reader.start()
    try:
        CubeApp(state, reader).run()
    finally:
        reader.stop()
        reader.join(timeout=1.0)


if __name__ == "__main__":
    main()
