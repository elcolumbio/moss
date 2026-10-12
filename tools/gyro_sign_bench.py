"""Gyro sign bench: does the RealSense gyro agree with the lidar about turning?

The lidar odometry's rotation prior reads the yaw rate on one axis of the
camera's frame (LidarOdometry gyro_axis "-y": the level, upright mount on the
nose). A wrong-sign rotation prior is worse than none, so the prior stays off
(MOSS_GYRO unset) until this bench has measured the mapping instead of
guessing it:

  1. commands a pivot, left then right, until the LIDAR yaw has turned by
     TURN_DEG (30 deg), the gyro by GYRO_STOP_RATIO x TURN_DEG, or MAX_RUNG_S
     passed
  2. integrates the gyro (mapped axis) over each move
  3. reads the lidar odometry yaw over the same window
  4. verdict: SAME sign and similar magnitude -> the mapping is right
     (then MOSS_GYRO=1); OPPOSITE sign -> flip it; garbage -> the mapping
     axis is wrong.

A rung only counts once the LIDAR SAW the body turn (MIN_TURN_DEG). With no
such rung the bench measured nothing and refuses to conclude: a run where the
rover stood still (tracks off the ground, base not armed, the pad's deadman)
must never validate the mapping.

A rung the lidar did not end, on which the gyro says the body turned past
TURN_DEG and more than LOST_RATIO times what the lidar saw, is a lidar that
lost the pivot (or a gyro that over-reads): the bench says INCONCLUSIVE (exit
3) and never blames the gyro's axes for it. The gyro stop keeps such a rung to
about GYRO_STOP_RATIO x TURN_DEG of body turn instead of MAX_RUNG_S of pivot.

The lidar yaw must be the lidar's own: with the prior on (MOSS_GYRO=1) every
registration starts from the gyro under test. Before turning, the bench reads
the newest run's log; a "prior: gyro=True" line there is a refusal (exit 3).

The rung ends on the lidar's own yaw, not on a clock: the tracks are open loop
and every pivot goes out at MOSS_PIVOT_PCT (100 % by default) whatever rate is
asked, so a rung timed for 30 deg at TURN_RAD_S would turn several times that.
At full power (about 250 deg/s on the floor) the lidar odometry without the
prior may not follow the turn; launch the stack with a slower pivot for this
bench (MOSS_PIVOT_PCT=50) and read the deg/s each rung prints.

The gyro is integrated on the samples' own stamps, as the odometry's prior
integrates them (lidar_odometry handle_imu: 0 < dt < 0.1 s), so the bench
measures the integral the prior would use.

Known input -> known output, in degrees.

Run on the robot WITH the stack up (imu + odom on the bus, the base armed,
nothing else commanding: exploration not armed, the pad untouched; MOSS_GYRO
unset), with room to turn:

    MOSS_PIVOT_PCT=50 tools/moss-fly.sh map
    .venv/bin/python tools/gyro_sign_bench.py
"""
from __future__ import annotations

import glob
import json
import math
import os
import re
import threading
import time

TURN_RAD_S = 0.3
TURN_DEG = 30.0
MAX_RUNG_S = 6.0          # a rung that has not turned TURN_DEG by then ends anyway (the verdict then says why)
GYRO_STOP_RATIO = 3.0     # the gyro has turned this many times TURN_DEG and the lidar has not seen TURN_DEG: stop
LOST_RATIO = 2.0          # a rung the lidar did not end, gyro past TURN_DEG and over this x the lidar: the lidar lost it
GYRO_AXIS = "-y"          # the mapping under test (LidarOdometry gyro_axis)
MIN_TURN_DEG = 5.0        # below this the lidar saw no turn: the rung measures nothing
GYRO_MAX_DT_S = 0.1       # the prior's own gap limit: a longer gap between two samples adds nothing


ENDED = {"lidar": "the lidar yaw", "gyro": "the gyro integral", "time": "the time limit"}


def verdict(results: list[tuple[str, float, float, str]]) -> tuple[str, int]:
    """(message, exit code) from the rungs [(label, gyro deg, lidar deg, ended), ...],
    `ended` being what stopped the rung ("lidar", "gyro" or "time").

    Needs at least ONE rung the lidar credits with a real turn - all() over an
    empty set is True and used to congratulate a rover that never moved. A rung
    the lidar lost (see the module docstring) makes the whole run inconclusive.
    """
    for label, g, l, ended in results:
        if ended != "lidar" and abs(g) > TURN_DEG and abs(g) > LOST_RATIO * abs(l):
            return (f"VERDICT: INCONCLUSIVE - rung {label}: the gyro says {g:+.0f} deg, the lidar {l:+.0f} deg, "
                    f"and the rung ended on {ENDED.get(ended, ended)} before the lidar saw {TURN_DEG:.0f} deg: "
                    f"the lidar odometry did not follow the pivot (relaunch the stack with MOSS_PIVOT_PCT=50) "
                    f"or the gyro over-reads - nothing concluded about the axes", 3)
    turned = [(g, l) for _, g, l, _ended in results if abs(l) > MIN_TURN_DEG]
    if not turned:
        return (f"VERDICT: INCONCLUSIVE - no rung turned the body by more than "
                f"{MIN_TURN_DEG:.0f} deg of lidar yaw, so nothing was measured "
                f"(tracks off the ground? base not armed? something else commanding?)", 3)
    if all(g * l > 0 and 0.5 < abs(g / l) < 2.0 for g, l in turned):
        return (f"VERDICT: mapping '{GYRO_AXIS}' CORRECT - the gyro may serve as "
                f"rotation prior ({len(turned)}/{len(results)} rungs measured): MOSS_GYRO=1", 0)
    if all(g * l < 0 for g, l in turned):
        flip = GYRO_AXIS[1:] if GYRO_AXIS.startswith("-") else "-" + GYRO_AXIS
        return (f"VERDICT: SIGN FLIPPED - use '{flip}' (LidarOdometry gyro_axis); keep MOSS_GYRO off until then", 2)
    return ("VERDICT: axis mapping WRONG or gyro not delivering - check the other axes", 2)


def axis_rate(v) -> float:
    """The mapped yaw rate (rad/s) from an angular velocity with .x .y .z."""
    val = {"x": v.x, "y": v.y, "z": v.z}[GYRO_AXIS[-1]]
    return -val if GYRO_AXIS.startswith("-") else val


def wrap_deg(a: float) -> float:
    """An angle difference in radians -> degrees in (-180, 180]."""
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


PRIOR_RE = re.compile(r"prior: gyro=(True|False)")


def newest_run_log(env=None) -> str | None:
    """The newest dimOS run's main.jsonl, whatever its blueprint: the log root's
    run dirs plus the log_dir of every run registry entry (a dimOS from a git
    clone logs under the clone)."""
    env = os.environ if env is None else env
    state = os.path.join(env.get("XDG_STATE_HOME") or os.path.expanduser("~/.local/state"), "dimos")
    hits = glob.glob(os.path.join(state, "logs", "*", "main.jsonl"))
    for entry in glob.glob(os.path.join(state, "runs", "*.json")):
        try:
            with open(entry) as f:
                log_dir = json.load(f).get("log_dir")
        except (OSError, ValueError, AttributeError):
            continue
        if isinstance(log_dir, str) and os.path.isfile(os.path.join(log_dir, "main.jsonl")):
            hits.append(os.path.join(log_dir, "main.jsonl"))
    hits = sorted(set(hits), key=os.path.getmtime)
    return hits[-1] if hits else None


def prior_check(log: str | None) -> tuple[str, int | None]:
    """(message, exit code or None to go on) from what the run's log says of the
    gyro prior: its last "prior: gyro=" line (the blueprint's and the odometry's
    start lines both carry one)."""
    if not log:
        return ("WARNING: no dimOS run log to read the gyro prior's state from - make sure the stack runs "
                "without MOSS_GYRO", None)
    state = None
    try:
        with open(log, errors="replace") as f:
            for line in f:
                m = PRIOR_RE.search(line)
                if m:
                    state = m.group(1) == "True"
    except OSError as exc:
        return (f"WARNING: cannot read {log} ({exc}) - make sure the stack runs without MOSS_GYRO", None)
    if state is None:
        return (f"WARNING: {log} says nothing of the gyro prior - make sure the stack runs without MOSS_GYRO", None)
    if state:
        return (f"KO: {log} says the gyro prior is ON (MOSS_GYRO=1): every lidar registration starts from the "
                f"gyro under test, so the lidar yaw is not a witness - relaunch the stack without MOSS_GYRO", 3)
    return (f"gyro prior OFF in {log}: the lidar yaw is the lidar's own", None)


class Bench:
    """The bus side: the gyro integral and the lidar yaw, fed by LCM callbacks.

    `lc` and `twist` are injectable (the cold bench feeds recorded messages
    through on_imu / on_odom and records the twists); None = the real bus."""

    def __init__(self, lc=None, twist=None, start: bool = True) -> None:
        self.gyro_int = 0.0
        self.last_imu_ts: float | None = None
        self.yaw: float | None = None
        self.lock = threading.Lock()
        self.running = True
        if lc is None:
            import lcm as lcmlib
            lc = lcmlib.LCM()
        if twist is None:
            from dimos.core.transport_factory import make_transport
            from dimos.msgs.geometry_msgs.Twist import Twist
            twist = make_transport("/cmd_vel", Twist)
        self.lc = lc
        self.twist = twist
        self.lc.subscribe("/imu#sensor_msgs.Imu", self._on_imu)
        self.lc.subscribe("/odom#geometry_msgs.PoseStamped", self._on_odom)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        if start:
            self.thread.start()

    def _spin(self) -> None:
        while self.running:
            self.lc.handle_timeout(100)

    def _on_imu(self, _ch: str, data: bytes) -> None:
        from dimos.msgs.sensor_msgs.Imu import Imu
        msg = Imu.lcm_decode(data)
        self.on_imu(axis_rate(msg.angular_velocity), float(getattr(msg, "ts", 0.0) or time.time()))

    def _on_odom(self, _ch: str, data: bytes) -> None:
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        msg = PoseStamped.lcm_decode(data)
        self.on_odom(msg.orientation.euler[2])

    def on_imu(self, rate: float, ts: float) -> None:
        """One gyro sample: the mapped rate (rad/s) at its own stamp (s)."""
        with self.lock:
            if self.last_imu_ts is not None:
                dt = ts - self.last_imu_ts
                if 0.0 < dt < GYRO_MAX_DT_S:
                    self.gyro_int += rate * dt
            self.last_imu_ts = ts

    def on_odom(self, yaw: float) -> None:
        """One lidar odometry pose: its yaw (rad)."""
        with self.lock:
            self.yaw = yaw

    def reset(self) -> None:
        with self.lock:
            self.gyro_int = 0.0
            self.last_imu_ts = None

    def snapshot(self) -> tuple[float, float | None]:
        with self.lock:
            return self.gyro_int, self.yaw

    def _send(self, wz: float) -> None:
        from dimos.msgs.geometry_msgs.Twist import Twist
        self.twist.broadcast(None, Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, wz]))

    def turn(self, wz: float, deg: float = TURN_DEG, max_s: float = MAX_RUNG_S,
             clock=time.monotonic, sleep=time.sleep) -> tuple[float, str]:
        """Pivot at wz (the command, every 0.1 s) until the lidar yaw has turned
        by `deg` from where the rung started ("lidar"), the gyro integral by
        GYRO_STOP_RATIO x `deg` ("gyro": the lidar is not following), or
        `max_s` passed ("time"); then zero. Returns (seconds, what ended it)."""
        gyro0, yaw0 = self.snapshot()
        t0 = clock()
        ended = "time"
        while clock() - t0 < max_s:
            gyro, yaw = self.snapshot()
            if yaw0 is not None and yaw is not None and abs(wrap_deg(yaw - yaw0)) >= deg:
                ended = "lidar"
                break
            if abs(math.degrees(gyro - gyro0)) >= GYRO_STOP_RATIO * deg:
                ended = "gyro"
                break
            self._send(wz)
            sleep(0.1)
        self._send(0.0)
        return clock() - t0, ended


def run_rungs(b: Bench, settle_s: float = 1.0, sleep=time.sleep, **turn_kw) -> list[tuple[str, float, float, str]]:
    """Left then right: [(label, gyro deg, lidar deg, ended)], each rung measured
    from its start to settle_s after its stop (the tracks coast, both sensors
    see it); `ended` is what stopped the pivot (Bench.turn)."""
    results = []
    for direction, label in ((+1.0, f"+{TURN_DEG:.0f} deg (left)"),
                             (-1.0, f"-{TURN_DEG:.0f} deg (right)")):
        b.reset()
        _, yaw_before = b.snapshot()
        took, ended = b.turn(direction * TURN_RAD_S, sleep=sleep, **turn_kw)
        sleep(settle_s)
        gyro, yaw_after = b.snapshot()
        lidar = wrap_deg(yaw_after - yaw_before) if yaw_before is not None and yaw_after is not None else 0.0
        gyro_deg = math.degrees(gyro)
        results.append((label, gyro_deg, lidar, ended))
        print(f"{label}: gyro({GYRO_AXIS}) = {gyro_deg:+.1f} deg | lidar yaw = {lidar:+.1f} deg "
              f"| {abs(lidar) / max(took, 1e-3):.0f} deg/s over {took:.1f} s of command, "
              f"ended on {ENDED.get(ended, ended)}")
    return results


def main() -> int:
    message, code = prior_check(newest_run_log())
    print(message)
    if code is not None:
        return code
    b = Bench()
    time.sleep(1.5)
    _, yaw0 = b.snapshot()
    if yaw0 is None:
        print("KO: no odom on the bus - is the stack up?")
        return 1
    if b.last_imu_ts is None:
        print("KO: no imu on the bus - is the camera up with its IMU (MOSS_IMU)?")
        return 1

    results = run_rungs(b)
    message, code = verdict(results)
    print(message)
    b.running = False
    return code


if __name__ == "__main__":
    raise SystemExit(main())
