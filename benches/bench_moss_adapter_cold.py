"""Cold bench: MossBaseAdapter on the fake MOSS firmware. Known in -> known out.

No hardware. dimOS only for section 0's gamepad clamp (skipped without it).
The fake runs the firmware's own state machine (Control.h, qtpy-mdd3a-v3:
arm / drive / stop / ramp, 300 ms drive deadman, 10 s arm expiry, +-100 %),
so what is proven here is the line protocol the Jetson speaks to the QT Py
over USB CDC, and the adapter's deadman on top of it.

Known input -> known output in percent, metres, radians, seconds.

Section 0's gamepad half needs dimOS (moss_dimos.gamepad imports it) and is
SKIPPED with a line where dimOS is not installed; any other import failure of
moss_dimos.gamepad is a KO.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_moss_adapter_cold.py
"""
import importlib.util
import math
import os
import re
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from moss_dimos.moss_adapter import (FIRMWARE_MAX_PCT, KEEPALIVE_S, STALE_S,
                                     MossBaseAdapter, MossGeometry,
                                     teleop_envelope, teleop_speeds, tracks_to_twist,
                                     twist_to_tracks)
from moss_dimos.moss_mock import DRIVE, OFF, READY, FakeMossFirmware

OK = KO = 0


def check(cond, label):
    global OK, KO
    if cond:
        OK += 1
    else:
        KO += 1
    print(("  OK  " if cond else "  KO  ") + label)


def settle(s=0.06):
    time.sleep(s)


# ── 0. gamepad scale: full stick = the base's limit, not the generic 0.6 m/s ──
os.environ["MOSS_PULSE_MS"] = "0"   # sections 0-12: the plain mapping and keepalive; pulsing has its own section
os.environ["MOSS_BUS_REF_MV"] = "0"   # the fake reports 11.6 V: sections 0-14 compare raw percents; the bus compensation has its own section (15)
print("0. teleop_speeds / teleop_envelope (track 0.30 m, 0.45 m/s at 100 %, turn gain 2)")
os.environ.pop("MOSS_TURN_GAIN", None)
G100 = MossGeometry(track_m=0.30, mps_at_limit=0.45, max_pct=100, min_pct=0)   # floor off: the mapping itself
_lin, _ang = teleop_speeds(G100)
check(abs(_lin - 0.45) < 1e-9, "full stick forward -> 0.45 m/s (= 100/100)")
check(abs(_ang - 6.0) < 1e-9, "full stick sideways -> 6.0 rad/s (gain 2 x the 3.0 rad/s spin)")
check(twist_to_tracks(_lin, 0.0, G100) == (100, 100), "that twist IS 100/100")
check(twist_to_tracks(0.0, _ang / 2, G100) == (-100, 100), "half turn stick alone = full spin -100/+100")
check(teleop_speeds(MossGeometry(track_m=0.0, mps_at_limit=0.45, max_pct=100)) == (0.45, 0.0), "track 0 -> no spin, no crash")
env = teleop_envelope(G100)
check(all(abs(env[k] - v) < 1e-9 for k, v in {"linear": 0.45, "angular": 6.0, "lever": 0.15, "rim": 1.35}.items()), f"envelope: {env}")
HAVE_DIMOS = False
SKIP_GAMEPAD = None
try:
    from moss_dimos.gamepad import TwistEnvelope, clamp_twist   # gamepad.py imports dimOS
    HAVE_DIMOS = True
except ImportError as exc:
    if importlib.util.find_spec("dimos") is None:
        SKIP_GAMEPAD = f"no dimOS here ({exc}): run this on the robot"
    else:
        check(False, f"moss_dimos.gamepad imports (TwistEnvelope, clamp_twist): {type(exc).__name__}: {exc}")
if HAVE_DIMOS:
    E = TwistEnvelope(**env)
    def tracks(vx, wz):
        c = clamp_twist(vx, 0.0, wz, E); return twist_to_tracks(c[0], c[2], G100)
    check(tracks(0.45, 0.0) == (100, 100), "full forward through the gamepad clamp -> 100/100")
    check(tracks(0.0, 6.0) == (-100, 100), "full turn alone -> full spin -100/+100")
    check(tracks(0.45, 3.0) == (0, 100), "full forward + HALF turn -> inner track stops (0/100)")
    check(tracks(0.45, 6.0) == (-100, 100), "full forward + FULL turn -> inner track full reverse (-100/100): the tank pivot")
    check(tracks(0.45, 1.92) == (36, 100), f"full forward + turn just past the deadzone -> inner slows (36/100): {tracks(0.45, 1.92)}")
    os.environ["MOSS_TURN_GAIN"] = "1"
    _l1, _a1 = teleop_speeds(G100); E1 = TwistEnvelope(**teleop_envelope(G100))
    _c = clamp_twist(0.45, 0.0, _a1, E1)
    check(abs(_a1 - 3.0) < 1e-9 and twist_to_tracks(_c[0], _c[2], G100) == (0, 100), "MOSS_TURN_GAIN=1 -> full forward + full turn only stops the inner track (0/100)")
    os.environ.pop("MOSS_TURN_GAIN", None)
    check(clamp_twist(9.0, 0.0, 0.0) == (0.45, 0.0, 0.0), "no envelope given -> the gamepad's conservative default ceiling (0.45 m/s)")
elif SKIP_GAMEPAD:
    print(f"  -- SKIP the gamepad clamp half: {SKIP_GAMEPAD}")


# ── 1. pure mapping: twist -> track percent ─────────────────────────────
G = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=None, min_pct=0)   # floor off: the mapping itself
print("1. twist -> tracks (track 0.30 m, 0.15 m/s at 25 %)")
check(twist_to_tracks(0.15, 0.0, G) == (25, 25), "vx=0.15 -> L=25 R=25 (the limit)")
check(twist_to_tracks(0.06, 0.0, G) == (10, 10), "vx=0.06 -> L=10 R=10")
check(twist_to_tracks(-0.06, 0.0, G) == (-10, -10), "vx=-0.06 -> -10/-10 (reverse)")
check(twist_to_tracks(0.0, 1.0, G) == (-25, 25), "wz=+1 rad/s -> L=-25 R=+25 (spin left)")
check(twist_to_tracks(0.30, 0.0, G) == (25, 25), "vx=0.30 (2x the limit) -> clamped 25/25")
check(twist_to_tracks(0.001, 0.0, G) == (0, 0), "vx=0.001 rounds to 0 -> a stop")
from moss_dimos.moss_adapter import CONNECT_PROBE_S, REPLY_TIMEOUT_S as _RT  # noqa: E402
check(CONNECT_PROBE_S >= 1.0 and _RT < CONNECT_PROBE_S,
      f"connect probe waits >= 1 s for the first status (reply tick {_RT} s): the first status can take longer than one reply tick")


# ... and connect() really uses that window, not the reply tick (a cold reader
# thread on the robot's computer answers the first 'status' late).
class _MuteFirmware(FakeMossFirmware):
    """Takes every line, never answers (a board with no firmware, or the wrong port)."""

    def write(self, data):
        return len(data)


class _LateStatus(FakeMossFirmware):
    """Answers the first 'status' only after `delay` s, like a cold reader thread."""

    def __init__(self, delay):
        super().__init__()
        self.delay, self._first = delay, True

    def write(self, data):
        if self._first and data.strip() == b"status":
            self._first = False
            threading.Timer(self.delay, FakeMossFirmware.write, (self, data)).start()
            return len(data)
        return super().write(data)


def _factory(fw_):
    f = lambda: fw_  # noqa: E731
    f.is_mock = True
    return f


_a = MossBaseAdapter(dof=3, port_factory=_factory(_MuteFirmware()), geometry=G)
_t0 = time.monotonic()
_ok = _a.connect()
_dt = time.monotonic() - _t0
check(_ok is False and 0.9 * CONNECT_PROBE_S <= _dt < CONNECT_PROBE_S + 0.5,
      f"mute firmware: connect() waits the {CONNECT_PROBE_S} s probe window, then refuses "
      f"(took {_dt:.2f} s, connect() is {_ok}; the reply tick alone would be {_RT} s)")
_a.disconnect()
_a = MossBaseAdapter(dof=3, port_factory=_factory(_LateStatus(0.5)), geometry=G)
_ok = _a.connect()
check(_ok is True, f"first status 0.5 s late (over the {_RT} s reply tick, inside the "
                   f"{CONNECT_PROBE_S} s window) -> connect() is True ({_ok})")
_a.disconnect()
GF = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=None, min_pct=12)
check(twist_to_tracks(0.06, 0.0, GF) == (12, 12), "floor 12: vx=0.06 (10 %) -> raised to L=12 R=12")
check(twist_to_tracks(-0.06, 0.0, GF) == (-12, -12), "floor 12: vx=-0.06 -> -12/-12 (sign kept)")
check(twist_to_tracks(0.0, 0.3, GF) == (-12, 12), "floor 12: tiny spin (-5/+5) -> -12/+12")
check(twist_to_tracks(0.001, 0.0, GF) == (0, 0), "floor 12: a twist that rounds to 0 is STILL a stop")
check(twist_to_tracks(0.15, 0.0, GF) == (25, 25), "floor 12: the limit is untouched")
check(twist_to_tracks(0.09, 0.0, GF) == (15, 15), "floor 12: 15 % above the floor passes as is")
check(twist_to_tracks(0.06, 0.0, G) == (10, 10), "floor 0 (explicit): L=10 R=10 unchanged")
GD = MossGeometry()   # the MOSS default: the tracks only turn at full duty
check(GD.min_pct == 50, f"the default floor is 50: {GD.min_pct}")
check(twist_to_tracks(0.02, 0.0, GD) == (50, 50), f"default: a slow forward twist -> 50/50: {twist_to_tracks(0.02, 0.0, GD)}")
check(twist_to_tracks(0.0, 0.2, GD) == (-50, 50), f"default: the planner's slowest pivot (0.2 rad/s) -> -50/+50 before pivot_tracks: {twist_to_tracks(0.0, 0.2, GD)}")
check(twist_to_tracks(-0.02, 0.0, GD) == (-50, -50), "default: slow reverse -> -50/-50 (sign kept)")
check(twist_to_tracks(0.0001, 0.0, GD) == (0, 0), "default: a twist that rounds to 0 is still a stop")
check(twist_to_tracks(0.06, 0.0, MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, min_pct=99)) == (25, 25),
      "a floor above the limit is clamped to the limit")
check(twist_to_tracks(0.15, 0.0, MossGeometry(mps_at_limit=0.15, max_pct=10)) == (10, 10),
      "max_pct=10 -> the same full-scale twist lands on L=10 R=10")
vx, vy, wz = tracks_to_twist(-0.15, 0.15, G)
check(abs(vx) < 1e-9 and abs(wz - 1.0) < 1e-9 and vy == 0.0,
      "tracks (-0.15, +0.15) m/s -> vx=0 wz=+1.0 (inverse of the above)")

# ── 2. the adapter drives the fake firmware ────────────────────────────
print("2. adapter on the fake firmware: connect, arm, drive, stop")
fw = FakeMossFirmware()
factory = lambda: fw
factory.is_mock = True
a = MossBaseAdapter(dof=3, port_factory=factory, geometry=G)
check(a.connect() is True, "connect() probes with 'status' and gets telemetry")
check(fw.commands[:1] == ["status"], f"first line on the wire is 'status': {fw.commands[:1]}")
check(a.get_dof() == 3, "get_dof() == 3 (vy ignored at the tracks)")
check(a.read_odometry() is None, "no ticks_per_m -> read_odometry() is None (honest)")
check(a.write_velocities([0.15, 0.0, 0.0]) is False and "drive 25 25" not in fw.commands,
      "drive before enable is refused and never reaches the wire")
check(a.write_enable(True) and a.read_enabled() and fw.mode == READY,
      "write_enable(True) = 'arm' -> firmware READY")
check(a.write_velocities([0.15, 0.4, 0.0]) is True,
      "write_velocities(vx=0.15, vy=0.4 ignored) accepted")
check(fw.commands[-2:] == ["arm", "drive 25 25"] and fw.mode == DRIVE and (fw.left, fw.right) == (25, 25),
      f"-> 'arm' then 'drive 25 25' on the wire, nothing else, firmware DRIVE 25/25: {fw.commands[-2:]}")
check(a.read_velocities() == [0.15, 0.0, 0.0],
      "read_velocities() = the commanded twist (open loop, no encoder calibration)")
n_before = fw.commands.count("drive 25 25")
settle(KEEPALIVE_S * 3.5)
n_after = fw.commands.count("drive 25 25")
check(n_after - n_before >= 2,
      f"keepalive re-sends the drive every {KEEPALIVE_S*1000:.0f} ms: +{n_after - n_before} in 350 ms")
check(a.write_stop() is True and fw.commands[-1] == "stop" and fw.mode == OFF,
      "write_stop() = 'stop' -> firmware OFF")
check(a.read_velocities() == [0.0, 0.0, 0.0], "after stop, commanded velocity is zero")

# ── 3. the adapter deadman: a commander that goes quiet ────────────────
print("3. deadman: no fresh twist -> one 'stop', tracks held")
check(a.write_velocities([0.06, 0.0, 0.0]) and fw.mode == DRIVE, "drive L=10 R=10 again")
settle(STALE_S + 0.25)
tail = fw.commands[-3:]
check(fw.commands[-1] == "stop" and fw.mode == OFF,
      f"after {STALE_S}s without a write: 'stop' sent, firmware OFF (tail {tail})")
n_stop = fw.commands.count("stop")
settle(KEEPALIVE_S * 3)
check(fw.commands.count("stop") == n_stop, "the stop is sent ONCE, not hammered")
check(a.target_pct == (0, 0), "target zeroed")

# ── 4. arm expiry in the firmware, seen through telemetry ──────────────
print("4. the firmware disarms itself (10 s idle) -> 'ERR not_armed' -> re-arm, retry")
check(a.write_enable(True) and fw.mode == READY, "armed again")
fw.advance(10_001)          # firmware: READY for 10 s -> OFF 'arm_expired', telemetry queued
settle(0.1)
check(fw.mode == OFF and fw.reason == "arm_expired", f"firmware OFF ({fw.reason})")
check(a.firmware_mode == OFF, "adapter read mode OFF from telemetry")
before = len(fw.commands)
check(a.write_velocities([0.06, 0.0, 0.0]) is True, "a new drive is accepted")
check(fw.commands[before:] == ["drive 10 10", "arm", "drive 10 10"] and fw.mode == DRIVE,
      f"-> refused, 'arm', resent (never an 'arm' on a guess): {fw.commands[before:]}")

# ── 5. 'ERR not_armed' without telemetry -> re-arm and retry once ───────
print("5. firmware stopped silently (no telemetry) -> the same path, no guessing")
fw._stop("timeout")         # no telemetry line emitted: the adapter cannot know
a.write_stop()              # adapter-side reset to a clean known state
check(a.write_enable(True) and fw.mode == READY, "armed")
fw._stop("timeout")
before = len(fw.commands)
check(a.write_velocities([0.06, 0.0, 0.0]) is True, "drive accepted after the retry")
check(fw.commands[before:] == ["drive 10 10", "arm", "drive 10 10"] and fw.mode == DRIVE,
      f"-> refused, 'arm', resent: {fw.commands[before:]}")
a.write_stop()

# ── 6. odometry from ticks when calibrated ─────────────────────────────
print("6. ticks_per_m set -> velocity and pose from encoder ticks")
G2 = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=1000.0)
fw2 = FakeMossFirmware()
f2 = lambda: fw2
f2.is_mock = True
b = MossBaseAdapter(dof=3, port_factory=f2, geometry=G2)
check(b.connect(), "connect (calibrated geometry)")
settle()
fw2.ticks = [100, 100]
fw2.emit_telemetry()
settle()
x, y, th = b.read_odometry()
check(abs(x - 0.10) < 1e-6 and abs(y) < 1e-6 and abs(th) < 1e-6,
      f"+100 ticks both sides (1000/m) -> x=+0.100 m, y=0, th=0: {x:.3f} {y:.3f} {th:.3f}")
fw2.ticks = [50, 150]
fw2.emit_telemetry()
settle()
x, y, th = b.read_odometry()
check(abs(x - 0.10) < 1e-6 and abs(th - (0.10 / 0.30)) < 1e-6,
      f"then -50/+50 ticks -> x unchanged, th=+0.333 rad (0.1 m over 0.30 m track): th={th:.3f}")
vx, vy, wz = b.read_velocities()
check(abs(vx) < 1e-9 and wz > 0, f"read_velocities() from ticks: vx=0, wz>0 ({wz:+.2f})")
b.disconnect()
check(fw2.commands[-1] == "stop" and not b.is_connected(), "disconnect() sends 'stop'")

# ── 7. geometry from env: bounds and mistyped values ───────────────────
print("7. MossGeometry.from_env")
import os
os.environ.update({"MOSS_MAX_PCT": "140", "MOSS_TICKS_PER_M": "abc", "MOSS_TRACK_M": "0.28"})
g = MossGeometry.from_env()
check(g.max_pct == FIRMWARE_MAX_PCT, f"MOSS_MAX_PCT=140 clamped to the firmware's {FIRMWARE_MAX_PCT}")
check(g.ticks_per_m is None, "MOSS_TICKS_PER_M=abc -> None (no odometry, not a random scale)")
check(g.track_m == 0.28, "MOSS_TRACK_M=0.28 taken")
for k in ("MOSS_MAX_PCT", "MOSS_TICKS_PER_M", "MOSS_TRACK_M"):
    os.environ.pop(k, None)
os.environ["MOSS_MIN_PCT"] = "40"
check(MossGeometry.from_env().min_pct == 40, "MOSS_MIN_PCT=40 -> min_pct 40")
os.environ["MOSS_MIN_PCT"] = "150"
check(MossGeometry.from_env().min_pct == MossGeometry.from_env().max_pct, "MOSS_MIN_PCT above the ceiling -> clamped to max_pct")
os.environ["MOSS_MIN_PCT"] = "-5"
check(MossGeometry.from_env().min_pct == 0, "MOSS_MIN_PCT negative -> 0")
os.environ.pop("MOSS_MIN_PCT", None)
check(MossGeometry.from_env().min_pct == 50, "MOSS_MIN_PCT unset -> 50 (measured floor 40 on a full battery, 10 of margin)")

# ── 8. the fake is the firmware: its own refusals ──────────────────────
print("8. fake firmware fidelity (Control.h)")
fw3 = FakeMossFirmware()
fw3.write(b"drive 10 10\n")
check(fw3.sent[-1] == "ERR not_armed" and fw3.mode == OFF, "drive before arm -> ERR not_armed")
fw3.write(b"arm\n")
fw3.write(b"drive 26 0\n")
check(fw3.sent[-1] == "OK" and fw3.mode == DRIVE and (fw3.left, fw3.right) == (26, 0),
      "26 % is valid since v3 (limit 100) -> OK, DRIVE 26/0")
fw3.write(b"drive 101 0\n")
check(fw3.sent[-1] == "ERR invalid_command" and fw3.mode == OFF and fw3.reason == "invalid",
      "101 % -> ERR invalid_command, OFF 'invalid'")
fw3.write(b"arm\ndrive -100 100\n")
check(fw3.sent[-1] == "OK" and (fw3.left, fw3.right) == (-100, 100), "arm + drive -100 100 (the full pivot) -> OK")
fw3.write(b"stop\n")      # 'arm' while DRIVE is arm_while_moving (a stop), so stop first
fw3.write(b"arm\ndrive 5 -5\n")
check(fw3.mode == DRIVE and (fw3.left, fw3.right) == (5, -5), "arm + drive 5 -5 -> DRIVE 5/-5")
fw3.advance(301)
check(fw3.mode == OFF and fw3.reason == "timeout", "301 ms without a drive -> OFF 'timeout'")
a.disconnect()

# ── 9. v3 firmware surface: ramp command and telemetry keys ────────────
print("9. fake firmware v3: 'ramp UP DOWN' and the telemetry keys")
import json
fw4 = FakeMossFirmware()
check("limit 100%" in fw4.sent[0] and "qtpy-mdd3a-v3" in fw4.sent[0], f"banner: {fw4.sent[0]!r}")
fw4.write(b"status\n")
tel = json.loads(fw4.sent[-1])
check(tel.get("fw") == "qtpy-mdd3a-v3" and tel.get("ramp_ms") == [30, 15],
      f"boot telemetry: fw qtpy-mdd3a-v3, ramp 30 / 15 ms per % ({tel.get('fw')}, {tel.get('ramp_ms')})")
check(tel.get("ina_ok") is True and tel.get("bus_mV") == 11600 and tel.get("shunt_uV") == 0
      and tel.get("current_mA") is None, "battery: ina_ok, 11600 mV (11.6 V), shunt 0, current null")
check("driver" not in tel, "no 'driver' key (only the MDD10A build sends it)")
fw4.write(b"ramp 5 5\n")
check(fw4.sent[-1] == "OK" and fw4.ramp_ms == [5, 5], "ramp 5 5 -> OK, 5/5 ms per %")
fw4.write(b"arm\ndrive 40 40\nramp 10 500\n")
check(fw4.sent[-1] == "OK" and fw4.mode == DRIVE and fw4.ramp_ms == [10, 500],
      "ramp 10 500 while driving -> OK, still DRIVE (a ramp change never stops)")
fw4.emit_telemetry()
tel = json.loads(fw4.sent[-1])
check(tel["ramp_ms"] == [10, 500] and tel["target_pct"] == [40, 40] and tel["output_pct"] == [40, 40],
      "telemetry reports ramp 10/500; output_pct == target_pct (ramp not simulated)")
for bad in ("ramp 0 5", "ramp 5 501", "ramp x 5"):
    fw4.write(b"arm\n")
    fw4.write((bad + "\n").encode())
    check(fw4.sent[-1] == "ERR invalid_command" and fw4.mode == OFF and fw4.ramp_ms == [10, 500],
          f"'{bad}' -> ERR invalid_command, OFF, ramp unchanged")

# ── 10. adapter: telemetry age, default port, geometry default ────────
print("10. adapter telemetry_age_s, default_port (QT Py by-id, never ttyUSB0), track 0.266")
import tempfile
from moss_dimos import robot_profile as rp
from moss_dimos.moss_adapter import default_port
fw5 = FakeMossFirmware()
f5 = lambda: fw5
f5.is_mock = True
c = MossBaseAdapter(dof=3, port_factory=f5, geometry=G)
check(c.telemetry_age_s is None, "before connect: telemetry_age_s is None")
check(c.connect(), "connect")
age0 = c.telemetry_age_s
settle(0.25)
age1 = c.telemetry_age_s
check(age0 is not None and 0.0 <= age0 < 0.2 and 0.2 <= age1 - age0 < 0.5,
      f"age right after connect {age0:.3f} s, +0.25 s later {age1:.3f} s")
fw5.emit_telemetry()
settle()
check(c.telemetry_age_s < 0.2, f"a new line resets the age ({c.telemetry_age_s:.3f} s)")
check(c.telemetry.get("bus_mV") == 11600 and c.telemetry is not c.telemetry, "telemetry is a copy of the last line")
c.disconnect()
saved_root, saved_port = rp.BYID_ROOT, os.environ.pop("MOSS_PORT", None)
with tempfile.TemporaryDirectory() as d:
    rp.BYID_ROOT = d
    check(default_port() == "/dev/serial/by-id/MISSING-QT-PY",
          f"no QT Py, MOSS_PORT unset -> the sentinel: {default_port()}")
    lidar = os.path.join(d, "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_x-if00-port0")
    open(lidar, "w").close()
    check(default_port() == "/dev/serial/by-id/MISSING-QT-PY", "a CP2102N alone is never taken for the base")
    qt = os.path.join(d, "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00")
    open(qt, "w").close()
    check(default_port() == qt, f"QT Py present -> its by-id path: {os.path.basename(default_port())}")
    check(MossBaseAdapter(port_factory=f5)._port == qt, "an adapter built without address takes it")
    os.environ["MOSS_PORT"] = "/dev/ttyACM9"
    check(default_port() == "/dev/ttyACM9", "MOSS_PORT wins")
    os.environ.pop("MOSS_PORT", None)
rp.BYID_ROOT = saved_root
if saved_port is not None:
    os.environ["MOSS_PORT"] = saved_port
check("ttyUSB0" not in default_port(), "the default is never /dev/ttyUSB0 (the lidar on MOSS)")
os.environ.pop("MOSS_TRACK_M", None)
check(MossGeometry().track_m == 0.266 and MossGeometry.from_env().track_m == 0.266,
      "MossGeometry track default 0.266 m (robot_profile.MOSS, CAD placeholder)")

# ── 11. the mock bus on the wall clock (MOSS_MOCK_BUS runs) ──────────────
print("11. MOSS_MOCK_BUS: live 200 ms telemetry and a firmware deadman that fires")
fw6 = FakeMossFirmware(auto_time=True)
fw6.write(b"arm\ndrive 10 10\n")
check(fw6.mode == DRIVE, "auto-time fake: arm + drive 10 10 -> DRIVE")
t_end = time.monotonic() + 0.45
while time.monotonic() < t_end:
    fw6.readline()
check(fw6.mode == OFF and fw6.reason == "timeout", f"no refresh for 0.45 s -> OFF '{fw6.reason}' (300 ms deadman)")
n_tel = sum(1 for line in fw6.sent if line.startswith("{"))
check(n_tel >= 2, f"telemetry every 200 ms while polled: {n_tel} lines in 0.45 s")
os.environ["MOSS_MOCK_BUS"] = "1"
m = MossBaseAdapter(dof=3, geometry=G)
check(m.connect() and m.mock_bus, "MOSS_MOCK_BUS=1 -> the adapter connects to the fake, mock_bus True")
settle(0.6)
check(m.telemetry_age_s is not None and m.telemetry_age_s < 0.35,
      f"telemetry stays fresh on the mock bus: age {m.telemetry_age_s:.3f} s after 0.6 s")
check(m.write_enable(True) and m.write_velocities([0.06, 0.0, 0.0]), "arm + drive through the mock bus")
settle(0.35)
check(m.telemetry.get("mode") == DRIVE and m.telemetry.get("target_pct") == [10, 10],
      f"keepalive holds DRIVE L=10 R=10 past the 300 ms firmware deadman: {m.telemetry.get('mode')} {m.telemetry.get('target_pct')}")
m.disconnect()
# the switch is parsed by robot_profile.env_on, like MOSS_CAMERA / MOSS_RECORD
os.environ["MOSS_MOCK_BUS"] = " Y "
m = MossBaseAdapter(dof=3, geometry=G)
check(m.connect() and m.mock_bus, "MOSS_MOCK_BUS=' Y ' -> the fake too (any value but 0/false/no/off)")
m.disconnect()
_saved_port = os.environ.get("MOSS_PORT")
os.environ["MOSS_PORT"] = "/dev/serial/by-id/MISSING-QT-PY"
for _off in ("0", "OFF", " false ", "No"):
    os.environ["MOSS_MOCK_BUS"] = _off
    m = MossBaseAdapter(dof=3, geometry=G)
    check(m.connect() is False and not m.mock_bus,
          f"MOSS_MOCK_BUS={_off!r} -> no fake: the real port is opened (and fails here, no board)")
if _saved_port is None:
    os.environ.pop("MOSS_PORT", None)
else:
    os.environ["MOSS_PORT"] = _saved_port
os.environ.pop("MOSS_MOCK_BUS", None)

# ── 12. public-repo hygiene: the adapter and the fake carry no journal ──
print("12. moss_adapter.py / moss_mock.py: comments explain the code, no dated journal")
for _name in ("moss_adapter.py", "moss_mock.py"):
    _src = (Path(__file__).resolve().parent.parent / "moss_dimos" / _name).read_text()
    _hits = (re.findall(r"\b[0-3][0-9]/[01][0-9]\b", _src)            # day/month dates
             + re.findall(r"\b[0-9]{1,2}h[0-5][0-9]\b", _src)          # clock stamps
             + re.findall(r"\b20[0-9]{2}-[01][0-9]-[0-3][0-9]\b", _src))  # ISO dates
    check(_hits == [], f"{_name}: no dated journal comment (day/month, clock stamp, ISO date): hits {_hits}")

print()

# ── 13. full-duty pulses: a percent becomes a duty of 100 % slices ─────
print("13. pulse_tracks + the adapter pulsing on the fake (MOSS_PULSE_MS=500 -> 5 slices of 100 ms)")
from moss_dimos.moss_adapter import pulse_tracks, pulse_ms, pulse_min_pct, pivot_tracks, pivot_pct, PULSE_RAMP  # noqa: E402
check(pivot_tracks((-8, 8), 100) == (-100, 100), "a planner pivot at 8 % -> -100/+100 (as the pad does it)")
check(pivot_tracks((8, -8), 100) == (100, -100), "...the other way round")
check(pivot_tracks((52, 40), 100) == (52, 40), "an arc (same signs) passes unchanged")
check(pivot_tracks((0, 40), 100) == (0, 40), "one track stopped: not a pivot, unchanged")
check(pivot_tracks((-8, 8), 0) == (-8, 8), "MOSS_PIVOT_PCT=0 -> as asked")
os.environ.pop("MOSS_PIVOT_PCT", None)
check(pivot_pct() == 100, "MOSS_PIVOT_PCT unset -> 100")
check([pulse_tracks((100, 100), k, 5) for k in range(5)] == [(100, 100)] * 5, "100 % -> on every slice")
check([pulse_tracks((20, 20), k, 5) for k in range(5)] == [(100, 100), (0, 0), (0, 0), (0, 0), (0, 0)], "20 % -> one slice of 100 % then four off")
check([pulse_tracks((50, -50), k, 5) for k in range(5)] == [(100, -100)] * 3 + [(0, 0)] * 2, "50 % -> three slices on (ceil of 2.5), sign kept per track")
check([pulse_tracks((1, 0), k, 5) for k in range(5)] == [(100, 0), (0, 0), (0, 0), (0, 0), (0, 0)], "1 % -> still one slice on; a 0 track stays off")
check(pulse_tracks((0, 0), 3, 5) == (0, 0), "(0, 0) -> (0, 0): a stop is never pulsed")
check(pulse_tracks((60, 40), 7, 5) == (100, 0), "tick 7 of a 5-slice period = slice 2: 60 % on (3 slices), 40 % off (2 slices)")
check(pulse_tracks((30, 30), 2, 0) == (30, 30), "n_ticks 0 -> pass-through (pulsing off)")
check(pulse_tracks((80, 80), 0, 5, full=25) == (25, 25), "full = the geometry's ceiling (25 here)")
os.environ["MOSS_PULSE_MS"] = "500"
check(pulse_ms() == 500, "MOSS_PULSE_MS=500 -> 500")
os.environ["MOSS_PULSE_MS"] = "9000"
check(pulse_ms() == 2000, "MOSS_PULSE_MS above the cap -> 2000")
os.environ.pop("MOSS_PULSE_MS", None)
check(pulse_ms() == 0, "MOSS_PULSE_MS unset -> 0 (the MOSS default: pulsing off, the tracks turn continuously from 40 %)")
os.environ["MOSS_PULSE_MS"] = "500"
os.environ.pop("MOSS_RAMP_MS", None)
os.environ.pop("MOSS_PULSE_MIN_PCT", None)
check(pulse_min_pct() == 40, "MOSS_PULSE_MIN_PCT unset -> 40 (a planner pivot at 8 % is raised to 40 % of the period)")
pd = MossBaseAdapter(dof=3, port_factory=(lambda: FakeMossFirmware()), geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100))
check(pd._map_geometry.min_pct == 40, f"adapter: the mapping floor is the duty floor 40 while pulsing ({pd._map_geometry.min_pct})")
os.environ["MOSS_PULSE_MIN_PCT"] = "0"   # the wire test below counts the slices of a bare 20 %
fw7 = FakeMossFirmware()
f7 = lambda: fw7  # noqa: E731
f7.is_mock = True
pa = MossBaseAdapter(dof=3, port_factory=f7, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100))
check(pa._pulse_n == 5 and pa._map_geometry.min_pct == 0, f"adapter: 5 slices, mapping floor off while pulsing ({pa._pulse_n}, {pa._map_geometry.min_pct})")
check(pa.connect() and pa._ramp == PULSE_RAMP, f"connect picks 'ramp {PULSE_RAMP}' when MOSS_RAMP_MS is unset (sent on a real board, not on the fake): {pa._ramp!r}")
check(pa.write_enable(True), "armed")
n0 = len(fw7.commands)
check(pa.write_velocities([0.03, 0.0, 0.0]), "vx 0.03 m/s (20 %) accepted")
settle(0.62)
pa.write_stop()
drives = [c for c in fw7.commands[n0:] if c.startswith("drive")]
on = sum(1 for c in drives if c == "drive 100 100")
off = sum(1 for c in drives if c == "drive 0 0")
check(on >= 1 and off >= 3 and on + off == len(drives) and set(drives) <= {"drive 100 100", "drive 0 0"},
      f"on the wire: full-duty slices and off slices only, 20 % = {on} on / {off} off over {len(drives)} ticks: {drives[:8]}")
check(pa._want == (0, 0) and fw7.mode == OFF, "stop clears the duty and the fake is OFF")
check(pa.write_velocities([0.15, 0.0, 0.0]) and fw7.commands[-1] == "drive 100 100", "100 % -> every slice on (first sent now)")
pa.write_stop()
pa.disconnect()
os.environ["MOSS_PULSE_MS"] = "0"
fw8 = FakeMossFirmware()
f8 = lambda: fw8  # noqa: E731
f8.is_mock = True
pb = MossBaseAdapter(dof=3, port_factory=f8, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=100))
check(pb._pulse_n == 0 and pb._map_geometry.min_pct == 100, "MOSS_PULSE_MS=0 -> no pulsing, the geometry's floor applies instead (100 here)")
check(pb.connect() and not any(c.startswith("ramp") for c in fw8.commands), "...and no ramp line without MOSS_RAMP_MS")
pb.disconnect()
os.environ.pop("MOSS_PULSE_MS", None)
os.environ.pop("MOSS_PULSE_MIN_PCT", None)


# ── 14. the stall guard: the encoders are the bumper ───────────────────
print("14. stall guard: a track commanded for STALL_S whose encoder did not move -> stop, counted (the encoders are the bumper)")
from moss_dimos.moss_adapter import STALL_S, STALL_MIN_TICKS, telemetry_payload as _tp  # noqa: E402
os.environ["MOSS_PULSE_MS"] = "0"
fw9 = FakeMossFirmware(auto_time=True)        # the stock fake never turns its encoders: every drive is a push against a wall
f9 = lambda: fw9  # noqa: E731
f9.is_mock = True
sa = MossBaseAdapter(dof=3, port_factory=f9, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=0))
check(sa.connect() and sa.write_enable(True), "connected and armed on a fake with frozen encoders")
from moss_dimos.moss_adapter import STALL_PUSH_PCT  # noqa: E402
t_start = time.monotonic(); pushed = False
while time.monotonic() - t_start < 2 * STALL_S + 1.0 and not sa.stalled:
    sa.write_velocities([0.06, 0.0, 0.0])      # 40 %, refreshed like a planner would
    pushed = pushed or sa.target_pct == (STALL_PUSH_PCT, STALL_PUSH_PCT)
    settle(0.1)
check(pushed, f"frozen at 40 %: the guard first pushes at {STALL_PUSH_PCT} % for one window")
check(sa.stalled and sa.stall_count == 1, f"still frozen at {STALL_PUSH_PCT} %: a stall, once (stalled {sa.stalled}, count {sa.stall_count})")
check(fw9.commands[-1] == "stop" and fw9.mode == OFF, f"'stop' was sent and the fake is OFF: last {fw9.commands[-1]!r}, mode {fw9.mode}")
check(sa.target_pct == (0, 0), "the command is dropped (no keepalive pushes on)")
doc = _tp(sa, 0.0)
check(doc is not None and doc.get("stalled") is True and doc.get("stall_count") == 1, f"telemetry_payload carries stalled/stall_count: {None if doc is None else (doc.get('stalled'), doc.get('stall_count'))}")
check(sa.write_velocities([-0.06, 0.0, 0.0]) and sa.stalled is False, "the next accepted command (the escape, in reverse) clears the flag")
sa.write_stop(); sa.disconnect()
# a fake whose encoders turn never stalls, however long the command is held
class Turning(FakeMossFirmware):
    def _telemetry_line(self):
        if self.mode == DRIVE:
            self.ticks[0] += 80; self.ticks[1] += 80
        return super()._telemetry_line()


# one track frozen while the other turns: an arc's inner track under its floor is not a stall,
# a track frozen at the HIGHER command (something holds it) is
class OneSided(FakeMossFirmware):
    def _telemetry_line(self):
        if self.mode == DRIVE:
            self.ticks[1] += 80            # only the right track turns
        return super()._telemetry_line()


for twist, label, want_stall in (([0.06, 0.0, 0.1], "left frozen at the lower command (an arc turning left): not a stall", False),
                                 ([0.06, 0.0, -0.1], "left frozen at the HIGHER command (something holds it): a stall", True)):
    fw11 = OneSided(auto_time=True)
    f11 = lambda fw=fw11: fw  # noqa: E731
    f11.is_mock = True
    sc = MossBaseAdapter(dof=3, port_factory=f11, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=0))
    sc.connect(); sc.write_enable(True)
    t_start = time.monotonic()
    while time.monotonic() - t_start < 2 * STALL_S + 1.0 and not sc.stalled:
        sc.write_velocities(twist); settle(0.1)
    check((sc.stall_count == 1) == want_stall, f"{label} (count {sc.stall_count}, L/R {sc._want})")
    sc.write_stop(); sc.disconnect()
fw10 = Turning(auto_time=True)
f10 = lambda: fw10  # noqa: E731
f10.is_mock = True
sb = MossBaseAdapter(dof=3, port_factory=f10, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=0))
check(sb.connect() and sb.write_enable(True), "a fake whose encoders turn")
t_start = time.monotonic()
while time.monotonic() - t_start < STALL_S + 0.6:
    sb.write_velocities([0.06, 0.0, 0.0]); settle(0.1)
check(not sb.stalled and sb.stall_count == 0 and fw10.mode == DRIVE, f"no stall while the encoders move: count {sb.stall_count}, mode {fw10.mode}")
sb.write_stop(); sb.disconnect()
os.environ.pop("MOSS_PULSE_MS", None)

# ── 15. the floor keeps the curvature, the bus compensation, the one-track pivot ─────
print("15. floor_tracks keeps the curvature; bus_gain / bus_scaled; a pivot with one frozen track turns on the other")
from moss_dimos.moss_adapter import (floor_tracks, bus_gain, bus_scaled, bus_ref_mv, one_track_pivot_on,  # noqa: E402
                                     BUS_GAIN_MAX, BUS_REF_MV_DEFAULT)
check(floor_tracks((57, 32), 50, 100) == (89, 50), f"arc 57/32 under a floor of 50 -> 89/50 (scaled up whole, ratio kept): {floor_tracks((57, 32), 50, 100)}")
check(floor_tracks((32, 57), 50, 100) == (50, 89), "...and mirrored 32/57 -> 50/89")
check(floor_tracks((-57, -32), 50, 100) == (-89, -50), "...in reverse -89/-50 (signs kept)")
check(floor_tracks((57, 10), 50, 100) == (100, 0), f"arc 57/10: the outer track would pass 100 -> 100/0, a one-track turn in the asked direction: {floor_tracks((57, 10), 50, 100)}")
check(floor_tracks((4, 4), 50, 100) == (50, 50), "a slow straight 4/4 -> 50/50")
check(floor_tracks((0, 30), 50, 100) == (0, 50) and floor_tracks((-30, 30), 50, 100) == (-50, 50), "a lone track and a pivot are raised to the floor as before")
check(floor_tracks((60, 55), 50, 100) == (60, 55) and floor_tracks((57, 32), 0, 100) == (57, 32) and floor_tracks((0, 0), 50, 100) == (0, 0),
      "above the floor, floor off, or a stop: unchanged")
G50 = MossGeometry(track_m=0.30, mps_at_limit=0.45, max_pct=100, min_pct=50)
check(twist_to_tracks(0.2, -0.385, G50) == (89, 50), f"the planner's slow right arc vx=0.2 wz=-0.385 (57/32 raw) -> 89/50: {twist_to_tracks(0.2, -0.385, G50)}")
check(twist_to_tracks(0.15, -0.5, G50) == (100, 0), f"a tighter one vx=0.15 wz=-0.5 (50/17 raw) -> 100/0: {twist_to_tracks(0.15, -0.5, G50)}")
check(abs(bus_gain(10500, 12400) - 12400 / 10500) < 1e-9 and bus_gain(12600, 12400) == 1.0 and bus_gain(7000, 12400) == BUS_GAIN_MAX,
      f"bus_gain: 10.5 V -> x{bus_gain(10500, 12400):.3f}, a full pack -> x1, 7 V -> capped x{BUS_GAIN_MAX}")
check(bus_gain(None, 12400) == 1.0 and bus_gain("n/a", 12400) == 1.0 and bus_gain(10500, 0) == 1.0 and bus_gain(30000, 12400) == 1.0 and bus_gain(0, 12400) == 1.0,
      "bus_gain: no reading, text, no reference, or an insane reading -> x1")
check(bus_scaled((56, 50), 12400 / 10500, 100) == (66, 59) and bus_scaled((100, -100), 1.2, 100) == (100, -100) and bus_scaled((0, 50), 1.5, 100) == (0, 75),
      f"bus_scaled: 56/50 at 10.5 V -> {bus_scaled((56, 50), 12400 / 10500, 100)}, a pivot stays at the limit, a 0 stays 0")
check(bus_ref_mv() == 0, "MOSS_BUS_REF_MV=0 (this bench's prelude) -> 0")
os.environ["MOSS_BUS_REF_MV"] = "12400"
check(bus_ref_mv() == 12400 and one_track_pivot_on() is True, "MOSS_BUS_REF_MV=12400 -> 12400; MOSS_ONE_TRACK_PIVOT unset -> on")
os.environ.pop("MOSS_BUS_REF_MV", None)
check(bus_ref_mv() == BUS_REF_MV_DEFAULT == 12400, "MOSS_BUS_REF_MV unset -> 12400 (a full 3S pack, where the floor was measured)")
os.environ["MOSS_BUS_REF_MV"] = "12400"
fw15 = Turning(auto_time=True)
f15 = lambda: fw15  # noqa: E731
f15.is_mock = True
se = MossBaseAdapter(dof=3, port_factory=f15, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.45, max_pct=100, min_pct=0))
check(se.connect() and se.write_enable(True) and se.telemetry.get("bus_mV") == 11600, "adapter on the fake (11.6 V on the bus), armed")
check(se.write_velocities([0.252, 0.0, 0.0]) and se._want == (56, 56) and se.target_pct == (60, 60) and fw15.commands[-1] == "drive 60 60",
      f"vx=0.252 (56 %) at 11.6 V with a 12.4 V reference -> drive 60 60 on the wire, the commander's 56/56 kept: want {se._want}, wire {se.target_pct}, {fw15.commands[-1]}")
check(se.write_velocities([0.45, 0.0, 0.0]) and se.target_pct == (100, 100), "100 % stays 100 % (capped)")
se.write_stop(); se.disconnect()
os.environ["MOSS_BUS_REF_MV"] = "0"

# a fake whose LEFT track turns and whose right track never does: a pivot to the right (right in reverse)
class LeftOnly(FakeMossFirmware):
    def _telemetry_line(self):
        if self.mode == DRIVE:
            self.ticks[0] += 80
        return super()._telemetry_line()


for env_on, label in (("1", "MOSS_ONE_TRACK_PIVOT on"), ("0", "MOSS_ONE_TRACK_PIVOT=0")):
    os.environ["MOSS_ONE_TRACK_PIVOT"] = env_on
    fw16 = LeftOnly(auto_time=True)
    f16 = lambda fw=fw16: fw  # noqa: E731
    f16.is_mock = True
    sf = MossBaseAdapter(dof=3, port_factory=f16, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.45, max_pct=100, min_pct=0))
    sf.connect(); sf.write_enable(True)
    t_start = time.monotonic()
    while time.monotonic() - t_start < 2 * STALL_S + 1.0 and not sf.stalled and not sf._dead_pivots:
        sf.write_velocities([0.0, 0.0, -2.0]); settle(0.1)      # a pivot to the right: L=+100 R=-100
    if env_on == "1":
        check(sf._dead_pivots == {("R", -1)} and sf.stall_count == 0 and not sf.stalled,
              f"{label}: the right track frozen in reverse while the left turns -> remembered as dead (R, reverse), no stall (count {sf.stall_count}, dead {sf._dead_pivots})")
        check(sf._want == (100, 0) and fw16.commands[-1] == "drive 100 0",
              f"...the pivot goes on on the left track, the right braked: want {sf._want}, wire {fw16.commands[-1]}")
        check(sf.write_velocities([0.0, 0.0, -2.0]) and sf._want == (100, 0) and sf.target_pct == (100, 0),
              f"...the planner's next pivot to the right is rewritten at once: {sf._want}")
        check(sf.write_velocities([0.0, 0.0, 2.0]) and sf._want == (-100, 100),
              f"...a pivot to the LEFT (right track forward) is untouched: {sf._want}")
        check(sf.write_velocities([0.2, 0.0, 0.0]) and sf._want == (44, 44), f"...a straight drive is untouched: {sf._want}")
    else:
        check(sf.stalled and sf.stall_count == 1 and not sf._dead_pivots,
              f"{label}: the same pivot is a stall, as before (count {sf.stall_count})")
    sf.write_stop(); sf.disconnect()
os.environ.pop("MOSS_ONE_TRACK_PIVOT", None)
os.environ.pop("MOSS_BUS_REF_MV", None)

# ── 16. feedback loss: logged once, rate-limited, with a count and a recovery line ──
print("16. feedback loss: telemetry silent while driving -> one warning, a counted repeat per READ_WARN_PERIOD_S, "
      "a recovery line; a failing serial read the same way")
import moss_dimos.moss_adapter as _MA  # noqa: E402
from moss_dimos.moss_adapter import READ_WARN_PERIOD_S, TELEMETRY_FRESH_S  # noqa: E402


class _CapLog:
    """Stands in for the adapter's logger: every line, by level."""

    def __init__(self):
        self.lines = []

    def _add(self, level, msg):
        self.lines.append((level, str(msg)))

    def info(self, msg, *a, **k):
        self._add("info", msg)

    def warning(self, msg, *a, **k):
        self._add("warning", msg)

    def error(self, msg, *a, **k):
        self._add("error", msg)

    def of(self, level, text):
        return [m for lv, m in self.lines if lv == level and text in m]


class _MutedTelemetry(FakeMossFirmware):
    """The firmware answers every command, but its telemetry can be cut (the CDC
    FIFO guard dropping the lines): the deadman and the arm expiry still run."""
    mute = False

    def _auto_tick(self):
        if not self.mute:
            return super()._auto_tick()
        self.now()
        self._poll()


class _FlakyReads(FakeMossFirmware):
    """readline() raises `fail` times (an I/O error on an open port), then reads again."""
    fail = 0

    def readline(self):
        if self.fail > 0:
            self.fail -= 1
            raise OSError(5, "Input/output error")
        return super().readline()


_saved_logger = _MA._LOGGER
_cap = _CapLog()
_MA._LOGGER = _cap
_saved_period = _MA.READ_WARN_PERIOD_S
try:
    check(READ_WARN_PERIOD_S == 5.0 and TELEMETRY_FRESH_S == 1.0,
          f"the rate limit is 5 s, the telemetry is stale past 1 s ({READ_WARN_PERIOD_S}, {TELEMETRY_FRESH_S})")
    os.environ["MOSS_PULSE_MS"] = "0"
    os.environ["MOSS_BUS_REF_MV"] = "0"
    fwm = _MutedTelemetry(auto_time=True)   # the stock encoders never turn: with telemetry, a long drive would be a stall
    fm = lambda: fwm  # noqa: E731
    fm.is_mock = True
    ma = MossBaseAdapter(dof=3, port_factory=fm, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=0))
    check(ma.connect() and ma.write_enable(True), "connected and armed on a fake whose telemetry can be cut")

    def _drive_for(seconds, twist=(0.06, 0.0, 0.0)):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            ma.write_velocities(list(twist)); settle(0.1)

    _drive_for(0.4)
    check(_cap.of("warning", "blind") == [] and ma.blind_count == 0,
          f"telemetry fresh while driving 40 %: no warning, blind count 0 ({ma.blind_count})")
    fwm.mute = True
    _drive_for(TELEMETRY_FRESH_S + 0.8)
    blind = _cap.of("warning", "stall guard and IMU bump blind")
    check(len(blind) == 1, f"telemetry silent for {TELEMETRY_FRESH_S + 0.8:.1f} s while driving: exactly ONE warning, "
                           f"not one per tick ({len(blind)}): {blind[:1]}")
    check(blind and "no telemetry from the motor board for" in blind[0] and "L=+40% R=+40%" in blind[0]
          and "(1 consecutive ticks)" in blind[0], f"it says what is blind, the command held and the count: {blind[:1]}")
    n_blind = ma.blind_count
    check(n_blind >= 4, f"every blind keepalive tick is counted: {n_blind} ticks past the 1 s mark (10 per second)")
    check(ma.stall_count == 0 and not ma.stalled and fwm.mode == DRIVE,
          f"meanwhile the stall guard cannot see the frozen encoders: no stall, the fake still DRIVEs "
          f"(count {ma.stall_count}, mode {fwm.mode}) - why the line matters")
    _MA.READ_WARN_PERIOD_S = 0.3
    _drive_for(0.75)
    blind = _cap.of("warning", "stall guard and IMU bump blind")
    counts = [int(re.search(r"\((\d+) consecutive", m).group(1)) for m in blind]
    check(2 <= len(blind) - 1 <= 4 and counts == sorted(counts) and len(set(counts)) == len(counts),
          f"period shortened to 0.3 s: the warning repeats about every 0.3 s ({len(blind) - 1} more lines over "
          f"0.75 s more of driving), each with a growing count {counts}")
    _MA.READ_WARN_PERIOD_S = _saved_period
    before_back = ma.blind_count
    fwm.mute = False
    _drive_for(0.5)
    back = _cap.of("info", "telemetry recovered after")
    check(len(back) == 1 and f"after {before_back} failures" in back[0] and "s without a line" in back[0],
          f"telemetry back: exactly one recovery line with the count ({before_back}): {back}")
    check(ma.blind_count == 0, f"the count resets on recovery ({ma.blind_count})")
    ma.write_stop()
    n_w = len(_cap.of("warning", "blind"))
    fwm.mute = True
    settle(TELEMETRY_FRESH_S + 0.5)
    check(len(_cap.of("warning", "blind")) == n_w and ma.blind_count == 0,
          "telemetry silent while the tracks are at rest: nothing is blind, nothing is said")
    fwm.mute = False
    ma.disconnect()

    fwf = _FlakyReads(auto_time=True)
    ff = lambda: fwf  # noqa: E731
    ff.is_mock = True
    mf = MossBaseAdapter(dof=3, port_factory=ff, geometry=MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=100, min_pct=0))
    check(mf.connect(), "connected on a fake whose reads can fail")
    n_err = len(_cap.lines)
    fwf.fail = 30
    t_end = time.monotonic() + 4.0
    peak = 0
    while time.monotonic() < t_end and (fwf.fail > 0 or mf.read_failure_count):
        peak = max(peak, mf.read_failure_count); settle(0.02)
    errs = [m for lv, m in _cap.lines[n_err:] if lv == "error" and "serial read failed" in m]
    check(len(errs) == 1 and "(1 consecutive)" in errs[0] and "Input/output error" in errs[0],
          f"30 failed reads in ~1.5 s (inside the 5 s period): ONE error line, not 30 ({len(errs)}): {errs[:1]}")
    check(peak >= 25, f"the failures are counted ({peak} at the peak, 30 thrown)")
    rec = [m for lv, m in _cap.lines[n_err:] if lv == "info" and "serial reads recovered" in m]
    check(rec == ["MOSS base: serial reads recovered after 30 failures"] and mf.read_failure_count == 0,
          f"reads work again: one recovery line with the count, the count reset: {rec}")
    mf.disconnect()
finally:
    _MA._LOGGER = _saved_logger
    _MA.READ_WARN_PERIOD_S = _saved_period
    os.environ.pop("MOSS_PULSE_MS", None)
    os.environ.pop("MOSS_BUS_REF_MV", None)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
