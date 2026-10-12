"""Cold bench: contacts from the IMU (moss_dimos/bump_imu.py) and the camera
that opens its motion module in-process (moss_dimos/realsense_imu.py).

Known input -> known output in physical units. Synthetic accel at 200 Hz in
the frame the camera publishes (x right, y down, z forward), gravity on the
down axis (y = -9.81 m/s^2), a little fixed-seed noise:

  A. the rule      - a 6 m/s^2 deceleration pulse of 60 ms while the tracks are
                     commanded (70, 60) -> exactly one contact at the pulse, its
                     high-pass about -6.0 m/s^2, in front; no command, a (0, 0)
                     command or a stale one -> none; a 1.5 m/s^2 wobble -> none;
                     two pulses 0.3 s apart -> one, 1.0 s apart -> two; an
                     acceleration (wrong sign) while driving forward -> none, the
                     same kick while reversing -> one, behind; a pivot takes
                     either sign, the sign giving the side; a 2 m/s^2 tilt held
                     3 s, a camera pitched 10 or 30 deg, a slope of 5 m/s^2
                     reached over 2 s -> none; a pulse before 0.25 s of history
                     -> none
  F. the sides     - a front contact, then the command reversed and a kick
                     behind 0.4 s later (inside the 0.7 s debounce) -> two
                     contacts, front then rear (a rear contact is not a repeat
                     of a front one); the same kick under the SAME pivot
                     command -> one (the rebound of the impact); a reverse
                     take-off while the telemetry still says forward -> none
  B. the knobs     - MOSS_BUMP_ACCEL=8 -> the 6 pulse is not a contact, a 9 one
                     is; MOSS_BUMP_AXIS=-z on a camera turned round -> the
                     contact again; bad values refused, defaults kept
  C. the module    - base_telemetry then the pulse sample by sample -> one
                     Bool(True) on bump and the log line with its numbers and
                     side; reversing then the kick -> one Bool(True) on
                     bump_rear and nothing on bump; no command, a stale firmware
                     line, an unwired Out, a NaN sample, a bad telemetry line
                     -> no bump and no exception
  D. the CSV       - MOSS_IMU_LOG writes a header and one row per sample with
                     the high-pass and the command; an unwritable path warns
                     once and never raises
  E. the camera    - MossRealSense on a fake pyrealsense2: the motion sensor
                     is found by name on the pipeline's own device, opened with
                     accel at its highest rate + gyro at imu_hz, started on the
                     parent's handler; stop() stops and closes it; no motion
                     module / a gyro rate not offered / a start that fails ->
                     a warning and the camera goes on; the parent publishes the
                     motion axes unrotated in <camera>_accel_optical_frame
  G. the watchdog  - ImuWatchdog on a fake clock: no sample for 10 s after start
                     -> one "waiting" status line; 200 Hz for 10 s -> a status
                     line "alive ... (200 Hz)" with gravity on y; samples stop
                     -> exactly one "MOSS IMU: no sample for 1 s - IMU contacts
                     and gyro prior OFF" at 1 s, nothing more while quiet but a
                     QUIET status every 10 s; samples resume -> "MOSS IMU:
                     samples back (200 Hz) after 3.2 s quiet" once the resumed
                     stream ran 1 s; a stream dropping out every 2 s -> at most
                     one "no sample" line per 5 s, the spells in between
                     counted; the module's own thread says it on the real clock,
                     stop() ends it; a failing look warns once, never raises

No hardware, no pyrealsense2 (a fake stands in), no bus.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_bump_imu_cold.py
"""
import asyncio
import json
import math
import os
import random
import sys
import tempfile
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OK = KO = 0


def check(label, ok, detail=""):
    global OK, KO
    if ok:
        OK += 1
    else:
        KO += 1
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + str(detail) if detail and not ok else ''}")


def finish():
    print()
    print(f"{OK} OK, {KO} KO")
    print("TEST PASSED" if KO == 0 else "TEST FAILED")
    sys.exit(1 if KO else 0)


for var in [k for k in os.environ if k.startswith("MOSS_")]:
    os.environ.pop(var, None)

try:
    from moss_dimos import bump_imu as bi
except Exception as exc:  # noqa: BLE001
    check("moss_dimos.bump_imu imports (needs dimOS: run in the robot's venv)", False, f"{type(exc).__name__}: {exc}")
    finish()

HZ = 200.0
G = 9.81
T0 = 1_760_000_000.0           # an epoch second: the camera's timestamps are on the host clock


def trace(duration_s=3.0, pulses=(), offset=None, wobble=None, pitch_deg=0.0, flip=False, seed=7):
    """[(ts, (ax, ay, az))] at 200 Hz. pulses: (start s, length s, m/s^2 on the
    forward axis); offset: (start s, length s, m/s^2) held; wobble: (amplitude,
    Hz) on the forward axis; pitch_deg: camera nose down (gravity leaks onto
    z); flip: the camera turned round (forward = -z)."""
    rng = random.Random(seed)
    out = []
    n = int(round(duration_s * HZ))
    pitch = math.radians(pitch_deg)
    for i in range(n):
        t = i / HZ
        fwd = 0.0
        for start, length, amp in pulses:
            if start <= t < start + length - 1e-9:
                fwd += amp
        if offset and offset[0] <= t < offset[0] + offset[1]:
            fwd += offset[2]
        if wobble:
            fwd += wobble[0] * math.sin(2.0 * math.pi * wobble[1] * t)
        ay = -G * math.cos(pitch) + rng.uniform(-0.05, 0.05)
        az = -G * math.sin(pitch) + fwd + rng.uniform(-0.05, 0.05)
        ax = rng.uniform(-0.05, 0.05)
        out.append((T0 + t, (ax, ay, -az if flip else az)))
    return out


def run(samples, commanded=(70, 60), age=0.0, det=None):
    """Feed a trace; -> (detector, [(ts, hp)] of the contacts)."""
    det = det or bi.ImuContactDetector()
    hits = []
    for ts, acc in samples:
        if det.feed(ts, acc, commanded, command_age_s=age):
            hits.append((ts, det.last_hp))
    return det, hits


PULSE = (1.5, 0.06, -6.0)       # 60 ms of -6 m/s^2 on the forward axis, 1.5 s in

# ── A. the rule ──────────────────────────────────────────────────────────
print("A. ImuContactDetector on known traces (200 Hz, gravity on y)")
det, hits = run(trace(pulses=[PULSE]))
check("(a) a 6 m/s^2 deceleration pulse of 60 ms while commanded (70, 60) -> exactly one contact",
      len(hits) == 1, f"{len(hits)} contacts")
check("    ... at the first sample of the pulse (t = 1.500 s)",
      len(hits) == 1 and abs(hits[0][0] - (T0 + 1.5)) < 1e-6, f"{[round(h[0] - T0, 3) for h in hits]}")
check("    ... its high-pass is the pulse: -6.0 m/s^2 within 0.15",
      len(hits) == 1 and abs(hits[0][1] - (-6.0)) <= 0.15, f"{[round(h[1], 3) for h in hits]}")
check("    ... the detector counts one contact and remembers the command and its side (in front)",
      det.contacts == 1 and det.last_contact["commanded"] == (70, 60) and det.last_contact["side"] == "front",
      f"{det.contacts} {det.last_contact}")
check("(b) the same pulse with commanded (0, 0) -> none", run(trace(pulses=[PULSE]), commanded=(0, 0))[1] == [])
check("    ... with no command at all -> none", run(trace(pulses=[PULSE]), commanded=None)[1] == [])
check("    ... with a command 1.5 s old (older than 1 s) -> none", run(trace(pulses=[PULSE]), age=1.5)[1] == [])
check("    ... with a command 0.9 s old -> one", len(run(trace(pulses=[PULSE]), age=0.9)[1]) == 1)
check("(c) a 1.5 m/s^2 wobble at 8 Hz for 3 s, commanded -> none",
      run(trace(wobble=(1.5, 8.0)))[1] == [])
check("    ... at 2 Hz -> none", run(trace(wobble=(1.5, 2.0)))[1] == [])
two_close = trace(pulses=[PULSE, (1.8, 0.06, -6.0)])
check("(d) two pulses 0.3 s apart -> one contact (debounce 0.7 s)", len(run(two_close)[1]) == 1,
      f"{len(run(two_close)[1])}")
two_far = trace(pulses=[PULSE, (2.5, 0.06, -6.0)])
far_hits = run(two_far)[1]
check("    ... 1.0 s apart -> two, at 1.5 s and 2.5 s",
      [round(h[0] - T0, 3) for h in far_hits] == [1.5, 2.5], f"{[round(h[0] - T0, 3) for h in far_hits]}")
kick = trace(pulses=[(1.5, 0.06, +6.0)])
check("(e) a +6 m/s^2 pulse (an acceleration) while commanded forward -> none", run(kick)[1] == [])
rev_det, rev_hits = run(kick, commanded=(-70, -60))
check("    ... the same +6 kick while reversing (-70, -60) -> one, a contact behind (side rear)",
      len(rev_hits) == 1 and rev_det.last_contact["side"] == "rear", f"{rev_det.last_contact}")
check("    ... a -6 pulse while reversing -> none (that is the reverse taking off)",
      run(trace(pulses=[PULSE]), commanded=(-70, -60))[1] == [])
piv_dec, piv_dec_hits = run(trace(pulses=[PULSE]), commanded=(70, -70))
piv_kick, piv_kick_hits = run(kick, commanded=(70, -70))
check("    ... a pivot (70, -70) takes either sign: -6 -> one in front, +6 -> one behind",
      len(piv_dec_hits) == 1 and len(piv_kick_hits) == 1
      and (piv_dec.last_contact["side"], piv_kick.last_contact["side"]) == ("front", "rear"),
      f"{piv_dec.last_contact} {piv_kick.last_contact}")
check("    ... an arc with one track stopped (70, 0) is forward: -6 -> one, +6 -> none",
      len(run(trace(pulses=[PULSE]), commanded=(70, 0))[1]) == 1 and run(kick, commanded=(70, 0))[1] == [])
check("(f) tilt: +2 m/s^2 held on the forward axis for 3 s -> none (the running mean eats it)",
      run(trace(duration_s=5.0, offset=(1.0, 3.0, 2.0)))[1] == [])
check("    ... -2 m/s^2 held 3 s -> none", run(trace(duration_s=5.0, offset=(1.0, 3.0, -2.0)))[1] == [])
check("    ... a camera pitched 10 deg nose down (1.70 m/s^2 of gravity on z), no pulse -> none",
      run(trace(pitch_deg=10.0))[1] == [])
check("    ... and the pulse on that pitched camera -> one",
      len(run(trace(pitch_deg=10.0, pulses=[PULSE]))[1]) == 1)
check("    ... a camera pitched 30 deg (4.9 m/s^2 of gravity on z, above the threshold raw) -> none",
      run(trace(pitch_deg=30.0))[1] == [])
ramp = [(ts, (a[0], a[1], a[2] - 5.0 * min(1.0, max(0.0, (ts - T0 - 0.5) / 2.0)))) for ts, a in trace(duration_s=4.0)]
check("    ... onto a slope: -5 m/s^2 on the forward axis reached over 2 s and held -> none",
      run(ramp)[1] == [])
check("a pulse 0.1 s after the first sample (less than 0.25 s of history) -> none",
      run(trace(pulses=[(0.1, 0.06, -6.0)]))[1] == [])
d = bi.ImuContactDetector()
check("a NaN sample or a short tuple is ignored, never raises",
      d.feed(T0, (0.0, -G, float("nan")), (70, 60)) is False and d.feed(T0, (0.0,), (70, 60)) is False)
check("command_direction: (70, 60) +1, (-70, -60) -1, (70, -70) 0, (0, 0) None, None None",
      [bi.command_direction(c) for c in ((70, 60), (-70, -60), (70, -70), (0, 0), None)] == [1, -1, 0, None, None])
check("contact_side: forward front, reversing rear, a pivot by the sign (-6 front, +6 rear); its stream bump / bump_rear",
      [bi.contact_side(d, hp) for d, hp in ((1, -6.0), (-1, 6.0), (0, -6.0), (0, 6.0))] == ["front", "rear", "front", "rear"]
      and (bi.contact_stream("front"), bi.contact_stream("rear")) == ("bump", "bump_rear"))

# ── F. the sides: one debounce per side ──────────────────────────────────
print("F. the sides: a rear contact is not a repeat of a front one; a kick within 0.1 s of a contact is its rebound")


def run_cmds(samples, command_at):
    """Feed a trace with the command each sample sees: command_at(t) -> pair;
    -> (detector, [(t, hp, side)] of the contacts)."""
    det = bi.ImuContactDetector()
    hits = []
    for ts, acc in samples:
        if det.feed(ts, acc, command_at(ts - T0), command_age_s=0.1):
            hits.append((round(ts - T0, 3), det.last_hp, det.last_contact["side"]))
    return det, hits


wedge = trace(pulses=[PULSE, (1.9, 0.06, +6.0)])
_, wedge_hits = run_cmds(wedge, lambda t: (70, 60) if t < 1.7 else (-70, -60))
check("(j) a front contact at 1.5 s, the escape reversing from 1.7 s, a kick behind at 1.9 s (0.4 s later, inside "
      "the 0.7 s debounce) -> two contacts: front then rear",
      [(t, side) for t, _, side in wedge_hits] == [(1.5, "front"), (1.9, "rear")], f"{wedge_hits}")
_, same_hits = run_cmds(wedge, lambda t: (70, 60))
check("    ... the same +6 kick with the command still forward -> no second contact (forward takes no kick)",
      [(t, side) for t, _, side in same_hits] == [(1.5, "front")], f"{same_hits}")
check("the rebound window: REBOUND_S 0.1 s, the detector's default",
      bi.REBOUND_S == 0.1 and bi.ImuContactDetector().rebound_s == 0.1)
_, corner_hits = run_cmds(trace(pulses=[PULSE, (1.68, 0.06, +6.0)]), lambda t: (70, -70))
check("(j2) a rover wedging into a corner during ONE pivot (70, -70): -6 at 1.5 s, +6 at 1.68 s (0.18 s later, the "
      "same command) -> two contacts: front then rear (one debounce per side, past the rebound)",
      [(t, side) for t, _, side in corner_hits] == [(1.5, "front"), (1.68, "rear")], f"{corner_hits}")
_, corner_hits2 = run_cmds(trace(pulses=[PULSE, (1.9, 0.06, +6.0)]), lambda t: (70, -70))
check("    ... the same pivot with the rear contact 0.4 s later -> two contacts as well",
      [(t, side) for t, _, side in corner_hits2] == [(1.5, "front"), (1.9, "rear")], f"{corner_hits2}")
rebound = trace(pulses=[PULSE, (1.56, 0.04, +6.0)])
reb_det, reb_hits = run_cmds(rebound, lambda t: (70, -70))
check("(k) a pivot (70, -70): -6 at 1.5 s then a +6 rebound at 1.56 s -> one contact, in front",
      [(t, side) for t, _, side in reb_hits] == [(1.5, "front")], f"{reb_hits}")
check("    ... the rebound is counted ONCE (8 samples past the threshold), behind, 0.06 s after the contact",
      reb_det.rebounds == 1 and reb_det.last_rebound["side"] == "rear"
      and abs(reb_det.last_rebound["after_s"] - 0.06) < 1e-6 and reb_det.last_rebound["hp"] >= 4.0,
      f"{reb_det.rebounds} {reb_det.last_rebound}")
ring_det, ring_hits = run_cmds(trace(pulses=[(1.5, 0.04, -6.0), (1.56, 0.03, +6.0)]),
                               lambda t: (70, 60) if t < 1.55 else (-30, -30))
check("(k2) driving forward: -6 at 1.5 s, the telemetry flips to the escape's reverse at 1.55 s, +6 ringing at "
      "1.56 s -> one contact, in front: the ringing is the rebound whatever the command, never a rear contact",
      [(t, side) for t, _, side in ring_hits] == [(1.5, "front")] and ring_det.rebounds == 1,
      f"{ring_hits} {ring_det.rebounds}")
long_det, long_hits = run_cmds(trace(pulses=[(1.5, 0.02, -6.0), (1.58, 0.04, +6.0)]), lambda t: (70, -70))
check("    ... a rebound that starts 0.08 s after the contact and runs past 0.1 s stays the rebound while it is past "
      "the threshold -> one contact, one rebound",
      [(t, side) for t, _, side in long_hits] == [(1.5, "front")] and long_det.rebounds == 1,
      f"{long_hits} {long_det.rebounds}")
_, flip_hits = run_cmds(trace(pulses=[PULSE, (2.0, 0.06, +6.0)]), lambda t: (70, -70) if t < 1.8 else (-70, -60))
check("    ... a +6 kick 0.5 s later once the command reverses (-70, -60) -> a new contact, behind",
      [(t, side) for t, _, side in flip_hits] == [(1.5, "front"), (2.0, "rear")], f"{flip_hits}")
_, again_hits = run_cmds(trace(pulses=[PULSE, (1.9, 0.06, -6.0)]), lambda t: (70, 60) if t < 1.7 else (70, -70))
check("    ... a second deceleration in front 0.4 s later under another command -> none (the same side debounces)",
      [(t, side) for t, _, side in again_hits] == [(1.5, "front")], f"{again_hits}")
_, takeoff_hits = run_cmds(trace(pulses=[PULSE, (1.8, 0.10, -6.0)]), lambda t: (70, 60))
check("(l) the escape taking off in reverse (-6 at 1.8 s) while the telemetry still says forward -> no second contact",
      [(t, side) for t, _, side in takeoff_hits] == [(1.5, "front")], f"{takeoff_hits}")
_, late_hits = run_cmds(trace(pulses=[PULSE, (2.3, 0.06, -6.0)]), lambda t: (70, 60))
check("    ... a second deceleration 0.8 s later (past the debounce) -> a second contact in front",
      [(t, side) for t, _, side in late_hits] == [(1.5, "front"), (2.3, "front")], f"{late_hits}")

# ── B. the knobs ─────────────────────────────────────────────────────────
print("B. the knobs: MOSS_BUMP_ACCEL, MOSS_BUMP_AXIS, MOSS_BUMP_DEBOUNCE_S, MOSS_IMU_LOG")


class LogSpy:
    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **kwargs):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = exception = _record


real_logger = bi.logger
spy = LogSpy()
bi.logger = spy
s = bi.bump_settings({})
check("defaults: axis +z, 4.0 m/s^2, debounce 0.7 s, no CSV",
      s == {"axis": "+z", "threshold": 4.0, "debounce_s": 0.7, "log_path": None}, f"{s}")
eight = bi.ImuContactDetector.from_env({"MOSS_BUMP_ACCEL": "8"})
check("(g) MOSS_BUMP_ACCEL=8 -> threshold 8.0 and the 6 m/s^2 pulse is not a contact",
      eight.threshold == 8.0 and run(trace(pulses=[PULSE]), det=eight)[1] == [])
check("    ... a 9 m/s^2 pulse is", len(run(trace(pulses=[(1.5, 0.06, -9.0)]),
                                         det=bi.ImuContactDetector.from_env({"MOSS_BUMP_ACCEL": "8"}))[1]) == 1)
flipped = bi.ImuContactDetector.from_env({"MOSS_BUMP_AXIS": "-z"})
check("MOSS_BUMP_AXIS=-z on a camera turned round (forward = -z) -> the pulse is one contact again",
      flipped.axis == "-z" and len(run(trace(pulses=[PULSE], flip=True), det=flipped)[1]) == 1)
check("    ... and the default +z on that camera sees an acceleration instead -> none",
      run(trace(pulses=[PULSE], flip=True))[1] == [])
check("MOSS_BUMP_AXIS=x -> +x, Y -> +y", bi.bump_settings({"MOSS_BUMP_AXIS": "x"})["axis"] == "+x"
      and bi.bump_settings({"MOSS_BUMP_AXIS": "Y"})["axis"] == "+y")
spy.lines.clear()
bad = bi.bump_settings({"MOSS_BUMP_AXIS": "w", "MOSS_BUMP_ACCEL": "-1", "MOSS_BUMP_DEBOUNCE_S": "abc"})
check("MOSS_BUMP_AXIS=w, MOSS_BUMP_ACCEL=-1, MOSS_BUMP_DEBOUNCE_S=abc -> defaults kept, three warnings",
      bad["axis"] == "+z" and bad["threshold"] == 4.0 and bad["debounce_s"] == 0.7 and len(spy.lines) == 3, f"{bad} {spy.lines}")
check("MOSS_BUMP_ACCEL=0 and =1e9 refused (the threshold is above 0, up to 100)",
      bi.bump_settings({"MOSS_BUMP_ACCEL": "0"})["threshold"] == 4.0 and bi.bump_settings({"MOSS_BUMP_ACCEL": "1e9"})["threshold"] == 4.0)
check("MOSS_BUMP_DEBOUNCE_S=0 accepted (no debounce), =0.3 -> 0.3",
      bi.bump_settings({"MOSS_BUMP_DEBOUNCE_S": "0"})["debounce_s"] == 0.0
      and bi.bump_settings({"MOSS_BUMP_DEBOUNCE_S": "0.3"})["debounce_s"] == 0.3)
nodeb = bi.ImuContactDetector(debounce_s=0.0)
check("    ... with debounce 0 the 60 ms pulse is still one contact per sample above threshold (12 samples)",
      len(run(trace(pulses=[PULSE]), det=nodeb)[1]) == 12, f"{len(run(trace(pulses=[PULSE]), det=bi.ImuContactDetector(debounce_s=0.0))[1])}")
check("MOSS_IMU_LOG=~/imu.csv -> an absolute path under the home",
      bi.bump_settings({"MOSS_IMU_LOG": "~/imu.csv"})["log_path"] == os.path.join(os.path.expanduser("~"), "imu.csv"))

# ── C. the module ────────────────────────────────────────────────────────
print("C. BumpImu: base_telemetry, then the samples -> bump and the log line")
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # noqa: E402
from dimos.msgs.sensor_msgs.Imu import Imu  # noqa: E402
from dimos.msgs.std_msgs.String import String  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402


class BumpOut:
    """The `bump` Out, recording what the module publishes; transport None =
    a module built outside a blueprint."""

    def __init__(self, wired=True):
        self.transport = object() if wired else None
        self.sent = []

    def publish(self, msg):
        self.sent.append((type(msg), bool(getattr(msg, "data", False))))


class RefusingOut(BumpOut):
    def publish(self, msg):
        raise RuntimeError("transport closed")


def imu_msgs(samples, gyro=(0.01, -0.02, 0.03)):
    return [Imu(angular_velocity=Vector3(*gyro), linear_acceleration=Vector3(*acc),
                frame_id="camera_accel_optical_frame", ts=ts) for ts, acc in samples]


def telemetry(target, age=0.1):
    return String(data=json.dumps({"ts": 0.0, "target_pct": list(target), "firmware_age_s": age, "stall_count": 0}))


MODULES = []


def module(wired=True, **kwargs):
    m = bi.BumpImu(**kwargs)
    m.bump = BumpOut(wired)
    m.bump_rear = BumpOut(wired)
    MODULES.append(m)
    return m


check("streams: In imu (Imu), In base_telemetry (String), Out bump and Out bump_rear (Bool), nothing else",
      set(bi.BumpImu.__annotations__) == {"imu", "base_telemetry", "bump", "bump_rear"}, f"{sorted(bi.BumpImu.__annotations__)}")
import moss_dimos.blueprints as _bp_mod  # noqa: E402
import moss_dimos.recovering_planner as _rp_mod  # noqa: E402
check("bump and bump_rear carry the SAME Bool class as the coordinator's and the planner's (one stream type on the bus)",
      bi.Bool is _bp_mod.Bool is _rp_mod.Bool)
check("no async handle_imu: every sample goes through a plain subscription (the async handler keeps only the latest)",
      not hasattr(bi.BumpImu, "handle_imu") and callable(getattr(bi.BumpImu, "on_imu", None)))
check("start/stop keep @rpc", {"start", "stop"} <= set(bi.BumpImu.rpcs))

spy.lines.clear()
mod = module()
check("a module built bare reads the environment's defaults: +z, 4.0, 0.7, no CSV",
      (mod.detector.axis, mod.detector.threshold, mod.detector.debounce_s, mod._csv) == ("+z", 4.0, 0.7, None))
asyncio.run(mod.handle_base_telemetry(telemetry((70, 60))))
cmd, age = mod.command_now()
check("(h) base_telemetry target_pct [70, 60], firmware age 0.1 s -> command (70, 60), about 0.1 s old",
      cmd == (70, 60) and 0.09 <= age < 0.5, f"{cmd} {age:.3f}")
results = [mod.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))]
check("    ... then the pulse sample by sample -> on_imu True exactly once", results.count(True) == 1, f"{results.count(True)}")
check("    ... one Bool(True) published on bump, nothing on bump_rear", mod.bump.sent == [(Bool, True)]
      and mod.bump_rear.sent == [], f"{mod.bump.sent} {mod.bump_rear.sent}")
contact_lines = [ln for ln in spy.lines if ln.startswith("IMU contact:")]
check("    ... the log line: 'IMU contact: forward -6.0 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%, "
      "front contact -> bump'",
      contact_lines == ["IMU contact: forward -6.0 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%, "
                        "front contact -> bump"], f"{contact_lines}")
check("    ... and the IMU is announced alive at its first sample (gravity on y)",
      any(ln.startswith("MOSS IMU: first sample") and "-9.8" in ln for ln in spy.lines), f"{spy.lines[:3]}")
check("every other line of the module starts with 'MOSS IMU:' (the launcher's log filter keeps MOSS lines), "
      "none of them with 'IMU contact'",
      all(ln.startswith("MOSS IMU:") for ln in spy.lines if not ln.startswith("IMU contact:"))
      and len([ln for ln in spy.lines if ln.startswith("IMU contact")]) == 1, f"{spy.lines}")
check("contact_line on a reverse contact: 'forward +5.2 ... L=-70% R=-60%, rear contact -> bump_rear'",
      bi.contact_line(5.24, 4.0, (-70, -60), "rear")
      == "IMU contact: forward +5.2 m/s^2 (threshold 4.0) while commanded L=-70% R=-60%, rear contact -> bump_rear")
spy.lines.clear()
backing = module()
asyncio.run(backing.handle_base_telemetry(telemetry((-70, -60))))
back_results = [backing.on_imu(m) for m in imu_msgs(trace(pulses=[(1.5, 0.06, +6.0)]))]
check("(m) reversing (-70, -60) then a +6 kick sample by sample -> one Bool(True) on bump_rear, nothing on bump",
      back_results.count(True) == 1 and backing.bump_rear.sent == [(Bool, True)] and backing.bump.sent == [],
      f"{backing.bump_rear.sent} {backing.bump.sent}")
check("    ... its line ends 'rear contact -> bump_rear'",
      [ln for ln in spy.lines if ln.startswith("IMU contact:")]
      == ["IMU contact: forward +6.0 m/s^2 (threshold 4.0) while commanded L=-70% R=-60%, rear contact -> bump_rear"],
      f"{[ln for ln in spy.lines if ln.startswith('IMU contact:')]}")
spy.lines.clear()
pivoting = module()
asyncio.run(pivoting.handle_base_telemetry(telemetry((70, -70))))
pivot_results = [pivoting.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE, (1.56, 0.04, +6.0)]))]
rebound_lines = [ln for ln in spy.lines if "rebound" in ln]
check("(m2) a pivot (70, -70), a contact and its rebound 0.06 s later -> one Bool(True) on bump, nothing on bump_rear",
      pivot_results.count(True) == 1 and pivoting.bump.sent == [(Bool, True)] and pivoting.bump_rear.sent == [],
      f"{pivoting.bump.sent} {pivoting.bump_rear.sent}")
check("    ... and ONE line for the rebound, never silent, not an 'IMU contact' line: 'MOSS IMU: forward +6.x m/s^2 "
      "while commanded L=+70% R=-70%, 0.06 s after the front contact: taken as its rebound, not published on bump_rear'",
      len(rebound_lines) == 1 and rebound_lines[0].startswith("MOSS IMU: forward +6.")
      and rebound_lines[0].endswith("while commanded L=+70% R=-70%, 0.06 s after the front contact: taken as its "
                                    "rebound, not published on bump_rear")
      and len([ln for ln in spy.lines if ln.startswith("IMU contact")]) == 1, f"{spy.lines}")
spy.lines.clear()
refused = module()
refused.bump = RefusingOut()
asyncio.run(refused.handle_base_telemetry(telemetry((70, 60))))
refused_results = [refused.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))]
check("a refused publish: the contact is logged first, then one warning naming bump, nothing raised",
      refused_results.count(True) == 1 and any(ln.startswith("IMU contact:") for ln in spy.lines)
      and len([ln for ln in spy.lines if "not published on bump " in ln]) == 1, f"{spy.lines}")

idle = module()
asyncio.run(idle.handle_base_telemetry(telemetry((0, 0))))
check("target_pct [0, 0] (tracks stopped) then the pulse -> no bump",
      not any(idle.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))) and idle.bump.sent == [])
silent = module()
check("no telemetry at all then the pulse -> no bump",
      not any(silent.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))) and silent.bump.sent == [])
stale = module()
asyncio.run(stale.handle_base_telemetry(telemetry((70, 60), age=2.0)))
check("a firmware line 2 s old -> no command (age >= 2 s), no bump",
      stale.command_now()[1] >= 2.0 and not any(stale.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))) and stale.bump.sent == [])
garbled = module()
asyncio.run(garbled.handle_base_telemetry(telemetry((70, 60))))
for bad_line in ("not json", json.dumps({"target_pct": "fast"}), json.dumps({"target_pct": [1]}), json.dumps([1, 2])):
    asyncio.run(garbled.handle_base_telemetry(String(data=bad_line)))
check("bad telemetry lines (not JSON, no pair, a list) leave the last good command in force",
      garbled.command_now()[0] == (70, 60))
unwired = module(wired=False)
asyncio.run(unwired.handle_base_telemetry(telemetry((70, 60))))
check("an unwired bump (no transport): the contact is detected and logged, nothing published, nothing raised",
      [unwired.on_imu(m) for m in imu_msgs(trace(pulses=[PULSE]))].count(True) == 1 and unwired.bump.sent == []
      and unwired.bump_rear.sent == [])
weird = module()
check("a sample with NaN or a message without fields -> False, never raises",
      weird.on_imu(Imu(linear_acceleration=Vector3(float("nan"), 0.0, 0.0), ts=T0)) is False
      and weird.on_imu(types.SimpleNamespace(ts="x")) is False)

# ── D. the CSV ───────────────────────────────────────────────────────────
print("D. MOSS_IMU_LOG: every sample to a CSV")
tmp = tempfile.mkdtemp(prefix="imu_log_")
csv_path = os.path.join(tmp, "sub", "imu.csv")
os.environ["MOSS_IMU_LOG"] = csv_path
logged = module()
os.environ.pop("MOSS_IMU_LOG")
check("(i) MOSS_IMU_LOG set -> the module built bare logs to that path", logged._csv is not None and logged._csv.path == csv_path)
asyncio.run(logged.handle_base_telemetry(telemetry((70, 60))))
pulse_msgs = imu_msgs(trace(pulses=[PULSE]))
for m in pulse_msgs:
    logged.on_imu(m)
logged.stop()
rows = Path(csv_path).read_text().splitlines() if os.path.exists(csv_path) else []
check("    ... header ts,ax,ay,az,gx,gy,gz,hp,cmd_l,cmd_r then one row per sample (600)",
      rows[:1] == ["ts,ax,ay,az,gx,gy,gz,hp,cmd_l,cmd_r"] and len(rows) == 1 + len(pulse_msgs), f"{len(rows)} lines")
cells = [r.split(",") for r in rows[1:]]
check("    ... ten cells per row", bool(cells) and all(len(c) == 10 for c in cells))
pulse_row = next((c for c in cells if abs(float(c[0]) - (T0 + 1.5)) < 1e-3), None)
check("    ... the row at 1.500 s: hp about -6.0, gyro (0.01, -0.02, 0.03), command 70 / 60",
      pulse_row is not None and abs(float(pulse_row[7]) + 6.0) <= 0.15 and pulse_row[4:7] == ["0.01000", "-0.02000", "0.03000"]
      and pulse_row[8:] == ["70", "60"], f"{pulse_row}")
check("    ... the first rows (window warming up) leave hp empty", cells[0][7] == "", f"{cells[0]}")
flushes = bi.ImuCsvLog(os.path.join(tmp, "every.csv"), flush_s=0.0)
flushes.add(T0, (0.0, -G, 0.0), (0.0, 0.0, 0.0), None, None)
check("a flush period of 0 writes at once; no command -> empty command cells",
      Path(tmp, "every.csv").read_text().splitlines()[1].endswith(",,,"), Path(tmp, "every.csv").read_text())
spy.lines.clear()
blocked = bi.ImuCsvLog(os.path.join(tmp, "every.csv", "cannot", "be.csv"), flush_s=0.0)   # a file stands where a folder must be
for i in range(3):
    blocked.add(T0 + i, (0.0, -G, 0.0), (0.0, 0.0, 0.0), 0.1, (70, 60))
blocked.close()
check("an unwritable path: no exception, ONE warning naming MOSS_IMU_LOG",
      len([ln for ln in spy.lines if "MOSS_IMU_LOG" in ln]) == 1 and blocked.rows_written == 0, f"{spy.lines}")
bi.logger = real_logger

# ── E. the camera: MossRealSense on a fake pyrealsense2 ──────────────────
print("E. MossRealSense: the motion module on the pipeline's own device (fake pyrealsense2)")


class _Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


rs = types.ModuleType("pyrealsense2")
rs.stream = types.SimpleNamespace(accel=_Enum("accel"), gyro=_Enum("gyro"), depth=_Enum("depth"), color=_Enum("color"))
rs.format = types.SimpleNamespace(motion_xyz32f=_Enum("motion_xyz32f"), motion_raw=_Enum("motion_raw"), z16=_Enum("z16"))
rs.camera_info = types.SimpleNamespace(name=_Enum("name"))


class FakeProfile:
    def __init__(self, kind, fps, fmt=None):
        self.kind, self.rate, self.fmt = kind, fps, fmt or rs.format.motion_xyz32f

    def stream_type(self):
        return self.kind

    def format(self):
        return self.fmt

    def fps(self):
        return self.rate

    def is_motion_stream_profile(self):
        return self.kind in (rs.stream.accel, rs.stream.gyro)

    def as_motion_stream_profile(self):
        return self

    def __repr__(self):
        return f"{self.kind}@{self.rate}"


class FakeSensor:
    def __init__(self, name, profiles, fail_start=False):
        self.name, self.profiles, self.fail_start = name, profiles, fail_start
        self.opened, self.callback, self.calls = None, None, []

    def get_info(self, key):
        return self.name

    def get_stream_profiles(self):
        return list(self.profiles)

    def open(self, profiles):
        self.calls.append("open")
        self.opened = list(profiles)

    def start(self, callback):
        self.calls.append("start")
        if self.fail_start:
            raise RuntimeError("failed to set power state")
        self.callback = callback

    def stop(self):
        self.calls.append("stop")

    def close(self):
        self.calls.append("close")


class FakeDevice:
    def __init__(self, sensors):
        self.sensors = sensors

    def query_sensors(self):
        return list(self.sensors)


class FakePipelineProfile:
    def __init__(self, device):
        self.device = device

    def get_device(self):
        return self.device


def d455(fail_start=False, gyro_rates=(200, 400), accel_rates=(100, 200)):
    motion = FakeSensor("Motion Module",
                        [FakeProfile(rs.stream.accel, r) for r in accel_rates]
                        + [FakeProfile(rs.stream.accel, 400, rs.format.motion_raw)]       # a raw format is never picked
                        + [FakeProfile(rs.stream.gyro, r) for r in gyro_rates], fail_start=fail_start)
    stereo = FakeSensor("Stereo Module", [FakeProfile(rs.stream.depth, 30, rs.format.z16)])
    rgb = FakeSensor("RGB Camera", [FakeProfile(rs.stream.color, 30)])
    return FakeDevice([stereo, rgb, motion]), motion


sys.modules["pyrealsense2"] = rs
from moss_dimos import realsense_imu as ri  # noqa: E402
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera  # noqa: E402

dev, motion = d455()
check("find_motion_sensor: the sensor named 'Motion Module' among stereo / RGB / motion",
      ri.find_motion_sensor(dev, rs) is motion)
check("... and None on a camera without one", ri.find_motion_sensor(FakeDevice(dev.sensors[:2]), rs) is None)
acc, gyr = ri.motion_profiles(motion, rs, 200)
check("motion_profiles(imu_hz=200): accel at its highest float rate (200, not the raw 400), gyro at 200",
      (acc.kind, acc.rate, acc.fmt, gyr.kind, gyr.rate) == (rs.stream.accel, 200, rs.format.motion_xyz32f, rs.stream.gyro, 200),
      f"{acc} {gyr}")
try:
    ri.motion_profiles(motion, rs, 300)
    check("imu_hz=300 -> ValueError naming the offered gyro rates [200, 400]", False)
except ValueError as exc:
    check("imu_hz=300 -> ValueError naming the offered gyro rates [200, 400]", "[200, 400]" in str(exc), str(exc))

spy2 = LogSpy()
real_ri_logger = ri.logger
ri.logger = spy2
cam = ri.MossRealSense(enable_imu=True, imu_hz=200)
MODULES.append(cam)
from dimos.core.coordination.blueprints import BlueprintAtom  # noqa: E402
_streams = lambda cls: {(r.name, r.type, r.direction) for r in BlueprintAtom.create(cls, {}).streams}  # noqa: E731
check("MossRealSense is a RealSenseCamera with exactly the parent's streams (imu among them), no new one",
      issubclass(ri.MossRealSense, RealSenseCamera) and not vars(ri.MossRealSense).get("__annotations__")
      and _streams(ri.MossRealSense) == _streams(RealSenseCamera) and ("imu", Imu, "out") in _streams(RealSenseCamera))
check("its stop keeps @rpc", "stop" in ri.MossRealSense.rpcs)
dev, motion = d455()
cam._profile = FakePipelineProfile(dev)
cam._start_imu()
check("_start_imu: the motion sensor of the pipeline's own device is opened with [accel 200 Hz, gyro 200 Hz]",
      cam._imu_sensor is motion and [(p.kind, p.rate) for p in motion.opened] == [(rs.stream.accel, 200), (rs.stream.gyro, 200)],
      f"{motion.opened}")
check("... and started on the parent's own handler (_on_motion_frame), no second pipeline",
      motion.calls == ["open", "start"] and motion.callback == cam._on_motion_frame and cam._imu_pipeline is None)
check("... announced in the log with the rates and the axes",
      any("accel 200 Hz, gyro 200 Hz" in ln and "z forward" in ln for ln in spy2.lines), f"{spy2.lines}")


class FakeMotionFrame:
    def __init__(self, kind, ms, xyz):
        self.kind, self.ms, self.xyz = kind, ms, xyz

    def as_motion_frame(self):
        return self

    def get_motion_data(self):
        return types.SimpleNamespace(x=self.xyz[0], y=self.xyz[1], z=self.xyz[2])

    def get_timestamp(self):
        return self.ms

    def get_profile(self):
        return FakeProfile(self.kind, 200)


published = []
cam.imu = types.SimpleNamespace(publish=published.append)
base_ms = T0 * 1000.0
motion.callback(FakeMotionFrame(rs.stream.accel, base_ms, (0.10, -9.81, 0.20)))
motion.callback(FakeMotionFrame(rs.stream.accel, base_ms + 10.0, (0.30, -9.81, 0.40)))
motion.callback(FakeMotionFrame(rs.stream.gyro, base_ms + 5.0, (0.01, 0.02, 0.03)))
imu = published[0] if published else None
check("the parent publishes one Imu per gyro sample, accel interpolated to it: (0.20, -9.81, 0.30) at +5 ms",
      imu is not None and len(published) == 1
      and all(abs(a - b) < 1e-4 for a, b in zip((imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z),
                                                 (0.20, -9.81, 0.30)))
      and abs(imu.ts - (T0 + 0.005)) < 1e-6, f"{imu!r}")
check("... unrotated (librealsense's axes as they come: y down = -9.81) in camera_accel_optical_frame",
      imu is not None and imu.frame_id == "camera_accel_optical_frame"
      and (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) == (0.01, 0.02, 0.03))
cam.stop()
check("stop(): the motion sensor is stopped then closed, and forgotten",
      motion.calls == ["open", "start", "stop", "close"] and cam._imu_sensor is None, f"{motion.calls}")

spy2.lines.clear()
bare = ri.MossRealSense(enable_imu=True, imu_hz=200)
MODULES.append(bare)
bare._profile = FakePipelineProfile(FakeDevice(d455()[0].sensors[:2]))
bare._start_imu()
bare._start_imu()
check("a camera without a motion module: ONE warning for two starts, no sensor, no exception",
      bare._imu_sensor is None and len([ln for ln in spy2.lines if "no motion module" in ln]) == 1, f"{spy2.lines}")
spy2.lines.clear()
odd = ri.MossRealSense(enable_imu=True, imu_hz=300)
MODULES.append(odd)
odd._profile = FakePipelineProfile(d455()[0])
odd._start_imu()
check("imu_hz=300 (not offered): a warning naming the rates, the camera goes on (no exception)",
      odd._imu_sensor is None and any("[200, 400]" in ln and "colour and depth go on" in ln for ln in spy2.lines), f"{spy2.lines}")
spy2.lines.clear()
fdev, fmotion = d455(fail_start=True)
failing = ri.MossRealSense(enable_imu=True, imu_hz=200)
MODULES.append(failing)
failing._profile = FakePipelineProfile(fdev)
failing._start_imu()
check("a start that fails ('failed to set power state'): the sensor is closed again, a warning, the camera goes on",
      failing._imu_sensor is None and fmotion.calls == ["open", "start", "close"]
      and any("failed to set power state" in ln for ln in spy2.lines), f"{fmotion.calls} {spy2.lines}")
ri.logger = real_ri_logger
del sys.modules["pyrealsense2"]

# ── G. the watchdog: a contact sensor never goes quiet unnoticed ─────────
print("G. ImuWatchdog: quiet, back, the status line (fake clock), then the module's own thread")
GRAVITY = (0.0, -G, 0.0)


def feed_watch(wd, t0, t1, hz=200.0):
    """Samples arriving at hz from t0 (included) to t1 (excluded), on the fake clock."""
    n = int(round((t1 - t0) * hz))
    for i in range(n):
        wd.sample(t0 + i / hz, GRAVITY)


def tick_until(wd, t0, t1, step=0.1, contacts=0):
    """tick() every step from t0 (included) to t1 (included): -> [(t, level, text)]."""
    out, n = [], int(round((t1 - t0) / step))
    for i in range(n + 1):
        t = round(t0 + i * step, 6)
        out += [(t, lvl, txt) for lvl, txt in wd.tick(t, contacts)]
    return out


wd = bi.ImuWatchdog()
check("defaults: quiet after 1 s, at most one 'no sample' line per 5 s, rate over 1 s, a status line every 10 s",
      (wd.quiet_s, wd.repeat_s, wd.rate_window_s, wd.status_s) == (1.0, 5.0, 1.0, 10.0)
      and (bi.QUIET_S, bi.STATUS_PERIOD_S) == (1.0, 10.0))
check("tick() before start() says nothing", wd.tick(5.0) == [])
wd.start(0.0)
waiting = tick_until(wd, 0.1, 10.0)
check("(n) no sample at all: nothing for 10 s (the camera takes seconds to start), then ONE status line at warning: "
      "'waiting, no sample 10 s after start - IMU contacts and gyro prior OFF'",
      [(t, lvl) for t, lvl, _ in waiting] == [(10.0, "warning")]
      and waiting[0][2] == "MOSS IMU: status waiting, no sample 10 s after start - IMU contacts and gyro prior OFF",
      f"{waiting}")
feed_watch(wd, 10.0, 20.0)
alive = wd.tick(20.0, 3)
check("200 Hz for 10 s -> the status line at info: 'alive, 2000 samples in 10 s (200 Hz)', gravity on y, 3 contacts, "
      "0 quiet spells",
      alive == [("info", "MOSS IMU: status alive, 2000 samples in 10 s (200 Hz), mean accel (+0.00, -9.81, +0.00) m/s^2, "
                          "3 contact(s) so far, 0 quiet spell(s)")], f"{alive}")
lost = tick_until(wd, 20.1, 23.0)
check("(o) the samples stop (last at 19.995 s): exactly one line, at warning, at 21.0 s: "
      "'MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF'",
      lost == [(21.0, "warning", "MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF")], f"{lost}")
check("    ... the watchdog is quiet and counts one spell", wd.quiet is True and wd.spells == 1)
feed_watch(wd, 23.2, 25.0)
back = tick_until(wd, 23.1, 25.0)
check("(p) samples again from 23.2 s -> one line once the resumed stream ran 1 s: "
      "'MOSS IMU: samples back (200 Hz) after 3.2 s quiet'",
      back == [(24.2, "info", "MOSS IMU: samples back (200 Hz) after 3.2 s quiet")], f"{back}")
check("    ... no longer quiet", wd.quiet is False)
feed_watch(wd, 25.0, 30.0)
status2 = wd.tick(30.0, 3)
check("    ... the next status line counts the samples of its 10 s (1360) and 1 quiet spell",
      len(status2) == 1 and status2[0][0] == "info" and "1360 samples in 10 s" in status2[0][1]
      and status2[0][1].endswith("3 contact(s) so far, 1 quiet spell(s)"), f"{status2}")
quiet_status = tick_until(wd, 30.1, 40.0, contacts=3)
check("a status line while quiet: warning 'status QUIET, no sample for 10 s - IMU contacts and gyro prior OFF'",
      [ln for ln in quiet_status if "status" in ln[2]]
      == [(40.0, "warning", "MOSS IMU: status QUIET, no sample for 10 s - IMU contacts and gyro prior OFF, "
                            "3 contact(s) so far, 2 quiet spell(s)")],
      f"{[ln for ln in quiet_status if 'status' in ln[2]]}")

flap = bi.ImuWatchdog()
flap.start(0.0)
lines = []
t = 0.0
for k in range(6):                       # 1.5 s of samples, then 2 s of silence, six times
    feed_watch(flap, t, t + 1.5)
    lines += [ln for ln in tick_until(flap, t + 0.1, t + 3.5) if "status" not in ln[2]]
    t += 3.5
lost_lines = [ln for ln in lines if "no sample" in ln[2]]
back_lines = [ln for ln in lines if "samples back" in ln[2]]
check("(q) a stream dropping out every 3.5 s, six times -> six spells, a 'no sample' line at most once per 5 s: "
      "spells 1, 3 and 5 said, spells 2, 4 and 6 counted",
      flap.spells == 6 and [round(ln[0], 1) for ln in lost_lines] == [2.5, 9.5, 16.5], f"{flap.spells} {lost_lines}")
check("    ... the next line counts the spell it did not say: '(1 more quiet spell(s) since the last line)'",
      len(lost_lines) == 3 and lost_lines[0][2].endswith("gyro prior OFF")
      and all(ln[2].endswith("OFF (1 more quiet spell(s) since the last line)") for ln in lost_lines[1:]),
      f"{lost_lines}")
check("    ... and a spell that was not said gets no 'back' line either (3 back lines, after spells 1, 3, 5)",
      len(back_lines) == 3, f"{back_lines}")

burst = bi.ImuWatchdog()
burst.start(0.0)
feed_watch(burst, 0.0, 2.0)
burst_lines = tick_until(burst, 0.1, 4.0)
feed_watch(burst, 4.0, 4.4)                  # a 0.4 s burst, then nothing again
burst_lines += tick_until(burst, 4.1, 11.9)
check("(s) samples to 2 s, then a 0.4 s burst at 4.0 s and nothing more: the 'no sample' line at 3.0 s, and NO "
      "'samples back' line (the burst stopped before its rate window ended): the watchdog stays quiet, one spell",
      [(t, txt) for t, _, txt in burst_lines if "status" not in txt]
      == [(3.0, "MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF")]
      and burst.quiet is True and burst.spells == 1, f"{burst_lines} {burst.quiet} {burst.spells}")
check("    ... the status line at 10 s says QUIET, no sample for 6 s",
      [txt for _, _, txt in burst_lines if "status" in txt]
      == ["MOSS IMU: status QUIET, no sample for 6 s - IMU contacts and gyro prior OFF, 0 contact(s) so far, "
          "1 quiet spell(s)"], f"{burst_lines}")
feed_watch(burst, 12.0, 14.0)
resumed = tick_until(burst, 12.0, 14.0)
check("    ... a stream that resumes for good at 12.0 s -> 'samples back (200 Hz) after 10.0 s quiet' at 13.0 s, "
      "the same spell",
      resumed == [(13.0, "info", "MOSS IMU: samples back (200 Hz) after 10.0 s quiet")]
      and burst.quiet is False and burst.spells == 1, f"{resumed}")

spy.lines.clear()
bi.logger = spy
wmod = module()
asyncio.run(wmod.handle_base_telemetry(telemetry((70, 60))))
for m in imu_msgs(trace(duration_s=0.5)):
    wmod.on_imu(m)
check("on_imu feeds the watchdog with the arrival time (monotonic), whatever the sample's own stamp",
      wmod.watchdog._last is not None and abs(wmod.watchdog._last - time.monotonic()) < 1.0, f"{wmod.watchdog._last}")
wmod.watchdog.start(time.monotonic())
said = wmod.watch(time.monotonic() + 1.5)
check("watch(): the lines go through the module's logger and come back as texts",
      said == ["MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF"]
      and "MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF" in spy.lines, f"{said} {spy.lines[-3:]}")


class BrokenWatchdog(bi.ImuWatchdog):
    def tick(self, now, contacts=0):
        raise ZeroDivisionError("float division by zero")


spy.lines.clear()
wmod._watchdog = BrokenWatchdog()
check("a look that fails: watch() returns [], never raises, ONE warning for two failures",
      wmod.watch(1.0) == [] and wmod.watch(2.0) == []
      and len([ln for ln in spy.lines if "watchdog look failed" in ln]) == 1, f"{spy.lines}")

spy.lines.clear()
live = module()
for m in imu_msgs(trace(duration_s=0.2)):
    live.on_imu(m)
live._start_watchdog()
thread = live._watch_thread
for m in imu_msgs(trace(duration_s=0.2)):
    live.on_imu(m)
t_stop = time.monotonic()
seen = None
while time.monotonic() - t_stop < 3.0:
    if any("no sample for 1 s" in ln for ln in spy.lines):
        seen = time.monotonic() - t_stop
        break
    time.sleep(0.02)
check("(r) the module's own thread, on the real clock: the samples stop -> the 'no sample' line within 1.0 to 1.3 s",
      seen is not None and 1.0 <= seen <= 1.3 and thread is not None and thread.name == "moss-imu-watchdog",
      f"{seen} {spy.lines}")
live.stop()
check("    ... stop() ends the thread", thread is not None and not thread.is_alive() and live._watch_thread is None)

# start() itself starts the watchdog: the dimOS base start is stood in for and
# the imu stream stubbed, so the module's own start() runs bare.
spy.lines.clear()
base_start = next((k, vars(k)["start"]) for k in bi.BumpImu.__mro__[1:] if "start" in vars(k))
started = module()
subscribed = []
started.imu = types.SimpleNamespace(subscribe=lambda cb: subscribed.append(cb) or (lambda: None))
setattr(base_start[0], "start", lambda self: None)
try:
    started.start()
    start_thread = started._watch_thread
    alive_after_start = start_thread is not None and start_thread.is_alive()
finally:
    setattr(base_start[0], "start", base_start[1])
check("(t) BumpImu.start() subscribes on_imu and starts the watchdog thread 'moss-imu-watchdog'",
      alive_after_start and start_thread.name == "moss-imu-watchdog"
      and len(subscribed) == 1 and subscribed[0] == started.on_imu, f"{start_thread} {subscribed}")
check("    ... its start line says the watchdog: 'watchdog: quiet after 1 s, status every 10 s'",
      any(ln.startswith("MOSS IMU: contacts on") and ln.endswith("watchdog: quiet after 1 s, status every 10 s")
          for ln in spy.lines), f"{spy.lines}")
started.stop()
check("    ... stop() ends that thread", start_thread is not None and not start_thread.is_alive()
      and started._watch_thread is None)
bi.logger = real_logger

MODULES.clear()
import shutil  # noqa: E402
shutil.rmtree(tmp, ignore_errors=True)
finish()
