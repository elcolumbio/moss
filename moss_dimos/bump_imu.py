"""BumpImu: contacts felt by the RealSense's accelerometer, published on
`bump` (a contact in front) or `bump_rear` (a contact behind).

The encoders are a poor bumper: a track that slips keeps turning against a
wall, an oblique contact leaves one track free, and the stall guard needs a
full second of frozen ticks before it calls a stall. The chassis itself
says it at once: hitting something while driving is a short, hard
deceleration along the direction of travel. This module reads it on the
camera's IMU (`imu`, published by moss_dimos.realsense_imu.MossRealSense) and
publishes Bool(True) on the side of the contact:

  * `bump`       in front: the planner's escape backs off 0.20 m in reverse.
                 The explorer's back-off publishes on the same stream; the
                 costmap, the sounds and the lidar odometry listen to it.
  * `bump_rear`  behind: the planner's escape moves 0.20 m FORWARD. A rear
                 contact is not a front wall: the costmap and the sounds do
                 not listen to it.

The two are opposite reflexes. The escape moves away from the contact; a
contact felt while reversing and answered with another reverse drives the
rover back into what it just hit. A stall of the base is not published here
or on either stream: it is inferred, not felt (the coordinator's `stall`).

The rule (ImuContactDetector, pure, benched with known traces):
  * forward = one signed accel axis of the frame the camera publishes in
    (MOSS_BUMP_AXIS, default "+z": librealsense's optical axes, z out of the
    lens, the rover's forward; docs/imu.md);
  * high-pass: the sample's forward component minus the mean of the samples
    of the last 0.5 s before it, so gravity, a tilted camera or a slope cancel;
  * a contact is |high-pass| >= MOSS_BUMP_ACCEL (default 4.0 m/s^2), with a
    non-zero track command younger than 1 s, decelerating AGAINST that command
    (driving forward: <= -threshold, in front; reversing: >= +threshold,
    behind; pivoting, the tracks of opposite signs: either sign, and the sign
    gives the side: a deceleration is in front, a forward kick behind);
  * once per contact, one debounce per side: nothing more on the same side
    for MOSS_BUMP_DEBOUNCE_S (default 0.7 s). A front contact and a rear one
    are opposite reflexes, not repeats of each other: the other side counts
    at once. One exception, which an accelerometer has and a switch would
    not: a kick of the other sign within REBOUND_S (0.1 s) of a contact is
    that impact's rebound, whatever the command says by then. It is logged
    once ("MOSS IMU: ... taken as its rebound"), never published.

The command is the firmware's target_pct from the coordinator's
`base_telemetry` (2 Hz), aged by the time since it arrived plus the
firmware's own age of that line.

  In  imu             : Imu      the camera's accel + gyro (every sample: a plain subscription, not the
                                 latest-only async handler, so a burst of samples is never thinned)
  In  base_telemetry  : String   JSON with target_pct [l, r] (percent, + = forward) and firmware_age_s
  Out bump            : Bool     True on a contact in front
  Out bump_rear       : Bool     True on a contact behind

Each contact is logged as one line, e.g.
    IMU contact: forward -4.8 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%, front contact -> bump
(the cockpit's decision feed matches its start, "IMU contact"). The other
lines start with "MOSS IMU:": the knobs at start, the first sample, and the
watchdog's (ImuWatchdog). A contact sensor must never go quiet unnoticed:
when no sample has come for 1 s while the stack runs, one warning says so,
    MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF
and one line says when the samples resume, at what rate,
    MOSS IMU: samples back (199 Hz) after 3.2 s quiet
(a stream that keeps dropping out says it at most once every 5 s, with a
count of the spells in between). Every 10 s a status line gives the state
(alive, QUIET, or waiting for the first sample), the sample rate, where
gravity sits and the contacts so far. All of it goes through the dimOS
logger, into the run's log.

MOSS_IMU_LOG=<path> appends every sample to a CSV (ts, ax, ay, az, gx, gy,
gz, hp, cmd_l, cmd_r), flushed every 2 s: the signature of real contacts,
real braking and real floors, to read before tightening the threshold.
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from collections import deque
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool   # the SAME Bool class as the explorer's bump and the coordinator's stall

logger = setup_logger()

DEFAULT_AXIS = "+z"             # librealsense optical frame: x right, y down, z forward (docs/imu.md)
DEFAULT_THRESHOLD = 4.0         # m/s^2 of high-passed forward deceleration
DEFAULT_DEBOUNCE_S = 0.7        # one contact, then silence for this long
WINDOW_S = 0.5                  # the running mean the high-pass subtracts
WARMUP_S = 0.25                 # history the window needs before a high-pass value is trusted
COMMAND_MAX_AGE_S = 1.0         # a track command older than this is no command
CSV_FLUSH_S = 2.0
CSV_HEADER = "ts,ax,ay,az,gx,gy,gz,hp,cmd_l,cmd_r\n"
QUIET_S = 1.0                   # no sample for this long while the stack runs: the stream is quiet
QUIET_REPEAT_S = 5.0            # at most one "no sample" line this often; the spells in between are counted
RATE_WINDOW_S = 1.0             # "samples back" measures the rate over this much of the resumed stream
STATUS_PERIOD_S = 10.0          # one status line this often, quiet or not
WATCH_PERIOD_S = 0.1            # how often the watchdog thread looks
FLOWING_S = 2 * WATCH_PERIOD_S  # a resumed stream whose last sample is older than this has stopped again
# An impact rings: right after the hit the accelerometer can feel a kick of the
# other sign, and by then the telemetry may already carry the escape's
# reverse. The contact measured on this robot lasted 20 ms (four samples past
# -4 m/s^2, docs/imu.md): 0.1 s covers it five times over, while two real
# contacts on opposite sides (a rover wedging into a corner) benched 0.18 s
# apart stay two. The figure comes from that impact length, not from a
# recorded rebound: a MOSS_IMU_LOG pivot contact is the recording that sets it.
REBOUND_S = 0.1
THRESHOLD_MAX = 100.0
DEBOUNCE_MAX_S = 10.0
_AXIS_RE = re.compile(r"^([+-]?)([xyz])$")


def parse_axis(text: str) -> tuple[int, float]:
    """"+z" / "z" / "-y" -> (axis index, sign). Raises ValueError otherwise."""
    m = _AXIS_RE.match(str(text).strip().lower())
    if not m:
        raise ValueError(f"not an axis: {text!r} (x, y or z with an optional sign, e.g. +z)")
    return "xyz".index(m.group(2)), (-1.0 if m.group(1) == "-" else 1.0)


def bump_settings(env: dict | None = None) -> dict:
    """The detector's knobs from the environment: {"axis", "threshold",
    "debounce_s", "log_path"}. A value that does not parse, or is out of range,
    is refused with a warning and the default is kept."""
    env = os.environ if env is None else env
    out = {"axis": DEFAULT_AXIS, "threshold": DEFAULT_THRESHOLD,
           "debounce_s": DEFAULT_DEBOUNCE_S, "log_path": None}
    raw = str(env.get("MOSS_BUMP_AXIS", "")).strip()
    if raw:
        try:
            parse_axis(raw)
            out["axis"] = raw.lower() if raw[0] in "+-" else "+" + raw.lower()
        except ValueError as exc:
            logger.warning(f"MOSS_BUMP_AXIS: {exc}; keeping {DEFAULT_AXIS}")
    for name, key, low, high in (("MOSS_BUMP_ACCEL", "threshold", 0.0, THRESHOLD_MAX),
                                 ("MOSS_BUMP_DEBOUNCE_S", "debounce_s", 0.0, DEBOUNCE_MAX_S)):
        raw = str(env.get(name, "")).strip()
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError:
            value = math.nan
        positive = key == "threshold"
        if not math.isfinite(value) or value > high or (value <= low if positive else value < low):
            bound = f"{'above' if positive else 'from'} {low:g} {'and up to' if positive else 'to'} {high:g}"
            logger.warning(f"{name}={raw!r} is not a number {bound}: keeping {out[key]:g}")
            continue
        out[key] = value
    path = str(env.get("MOSS_IMU_LOG", "")).strip()
    if path:
        out["log_path"] = os.path.abspath(os.path.expanduser(path))
    return out


def contact_side(direction: int, hp: float) -> str:
    """"front" or "rear" for a contact against the command: reversing is
    behind, driving forward in front, and on a pivot the sign of the
    high-pass decides (a deceleration of the forward axis is in front)."""
    if direction < 0 or (direction == 0 and hp > 0):
        return "rear"
    return "front"


def command_direction(commanded: tuple[int, int] | None) -> int | None:
    """+1 forward (no track backward), -1 reverse (no track forward), 0 a
    pivot (tracks of opposite signs), None no command (None or (0, 0))."""
    if commanded is None:
        return None
    try:
        left, right = float(commanded[0]), float(commanded[1])
    except (TypeError, ValueError, IndexError):
        return None
    if left == 0 and right == 0:
        return None
    if left >= 0 and right >= 0:
        return 1
    if left <= 0 and right <= 0:
        return -1
    return 0


class ImuContactDetector:
    """The contact rule of the module docstring, on raw samples. Pure: no
    clock, no I/O. feed() returns True once per contact, its side in
    last_contact["side"]; last_hp is the high-pass of the last sample (None
    while the window warms up). A rebound is counted in `rebounds` once per
    contact, described in last_rebound, and feed() returns False for it."""

    def __init__(self, axis: str = DEFAULT_AXIS, threshold: float = DEFAULT_THRESHOLD,
                 debounce_s: float = DEFAULT_DEBOUNCE_S, window_s: float = WINDOW_S,
                 warmup_s: float = WARMUP_S, command_max_age_s: float = COMMAND_MAX_AGE_S,
                 rebound_s: float = REBOUND_S) -> None:
        self.axis = axis
        self._index, self._sign = parse_axis(axis)
        self.threshold = float(threshold)
        self.debounce_s = float(debounce_s)
        self.window_s = float(window_s)
        self.warmup_s = float(warmup_s)
        self.command_max_age_s = float(command_max_age_s)
        self.rebound_s = float(rebound_s)
        self._window: deque[tuple[float, float]] = deque()
        self._sum = 0.0
        self._appends = 0
        self.last_hp: float | None = None
        self.last_contact_ts: float | None = None
        self.last_contact: dict | None = None
        self._last_by_side: dict[str, float] = {}   # side -> ts of its last contact
        self.contacts = 0
        self.rebounds = 0
        self.last_rebound: dict | None = None
        self._rebound_side: str | None = None       # a rebound in progress: its samples are past the threshold
        self._rebound_of: float | None = None       # ts of the contact whose rebound was counted

    @classmethod
    def from_env(cls, env: dict | None = None) -> "ImuContactDetector":
        s = bump_settings(env)
        return cls(axis=s["axis"], threshold=s["threshold"], debounce_s=s["debounce_s"])

    def forward(self, accel_xyz: Any) -> float:
        """The forward component of an accel sample (the signed axis)."""
        return self._sign * float(accel_xyz[self._index])

    def _high_pass(self, ts: float, value: float) -> float | None:
        """value minus the mean of the samples of the last window_s before ts;
        None while the window holds less than warmup_s of history. Then the
        sample joins the window."""
        if self._window and ts < self._window[-1][0]:
            self._window.clear()          # the clock went back: start over
            self._sum = 0.0
        while self._window and self._window[0][0] < ts - self.window_s:
            self._sum -= self._window.popleft()[1]
        hp = None
        if self._window and ts - self._window[0][0] >= self.warmup_s:
            hp = value - self._sum / len(self._window)
        self._window.append((ts, value))
        self._sum += value
        self._appends += 1
        if self._appends % 4096 == 0:       # no float drift over hours of samples
            self._sum = math.fsum(v for _, v in self._window)
        return hp

    def feed(self, ts: float, accel_xyz: Any, commanded: tuple[int, int] | None,
             command_age_s: float = 0.0) -> bool:
        """One accel sample (m/s^2, the camera's frame) at ts (s), with the
        track command in force (percent pair, or None) and its age. True when
        this sample is a new contact."""
        try:
            ts = float(ts)
            value = self.forward(accel_xyz)
        except (TypeError, ValueError, IndexError):
            return False
        if not (math.isfinite(ts) and math.isfinite(value)):
            return False
        hp = self._high_pass(ts, value)
        self.last_hp = hp
        if hp is None or abs(hp) < self.threshold:
            self._rebound_side = None     # whatever rang has ended
            return False
        try:
            fresh = float(command_age_s) <= self.command_max_age_s
        except (TypeError, ValueError):
            fresh = False
        direction = command_direction(commanded) if fresh else None
        if direction is None:
            return False
        if direction > 0 and hp > -self.threshold:
            return False                  # driving forward: only a deceleration is a contact
        if direction < 0 and hp < self.threshold:
            return False                  # reversing: only a forward kick is a contact
        side = contact_side(direction, hp)
        of = self._rebound_target(side, ts)
        if of is not None:
            self._rebound_side = side
            if of != self._rebound_of:    # counted once per contact, however many samples it lasts
                self._rebound_of = of
                self.rebounds += 1
                self.last_rebound = {"ts": ts, "hp": hp, "commanded": (int(commanded[0]), int(commanded[1])),
                                     "side": side, "after_s": ts - of}
            return False
        self._rebound_side = None
        if self._repeats(side, ts):
            return False
        self._last_by_side[side] = ts
        self.last_contact_ts = ts
        self.contacts += 1
        self.last_contact = {"ts": ts, "hp": hp, "commanded": (int(commanded[0]), int(commanded[1])),
                             "side": side}
        return True

    def _rebound_target(self, side: str, ts: float) -> float | None:
        """The ts of the contact this sample is the rebound of, or None. A
        rebound is a sample of the other side within rebound_s of that side's
        last contact, whatever the command; once one has started, its samples
        stay the rebound while they are past the threshold."""
        other = "front" if side == "rear" else "rear"
        last = self._last_by_side.get(other)
        if last is None:
            return None
        if 0.0 <= ts - last < self.rebound_s or (side == self._rebound_side and self._rebound_of == last):
            return last
        return None

    def _repeats(self, side: str, ts: float) -> bool:
        """True when a contact at ts repeats the last one of the SAME side
        within debounce_s. One debounce per side: a contact on the other side
        is never a repeat (past the rebound, it is a new contact)."""
        last = self._last_by_side.get(side)
        return last is not None and 0.0 <= ts - last < self.debounce_s


def contact_stream(side: str) -> str:
    """The stream a contact on that side is published on."""
    return "bump_rear" if side == "rear" else "bump"


def contact_line(hp: float, threshold: float, commanded: tuple[int, int], side: str) -> str:
    """The log line of one contact (the cockpit's decision feed reads it)."""
    return (f"IMU contact: forward {hp:+.1f} m/s^2 (threshold {threshold:.1f}) "
            f"while commanded L={int(commanded[0]):+d}% R={int(commanded[1]):+d}%, "
            f"{side} contact -> {contact_stream(side)}")


def rebound_line(hp: float, commanded: tuple[int, int], side: str, after_s: float) -> str:
    """The log line of a kick taken as the rebound of the last contact: said
    once, so no felt kick is silent, and kept off the "IMU contact" lines the
    cockpit reads as contacts."""
    other = "front" if side == "rear" else "rear"
    return (f"MOSS IMU: forward {hp:+.1f} m/s^2 while commanded L={int(commanded[0]):+d}% "
            f"R={int(commanded[1]):+d}%, {after_s:.2f} s after the {other} contact: taken as its rebound, "
            f"not published on {contact_stream(side)}")


class ImuWatchdog:
    """Says when the imu stream goes quiet and when it comes back, and gives a
    status line every status_s (module docstring). Pure: the caller passes a
    monotonic clock. sample() runs on the subscription's thread, tick() on the
    watchdog's; one lock between them. tick() returns the lines to log, as
    (level, text) pairs.

    Quiet means no sample for quiet_s after at least one arrived: before the
    first sample the status line says "waiting" instead (the camera takes a
    few seconds to start). A quiet spell is said once, at most once per
    repeat_s (the spells in between are counted in the next line), and its
    end once the resumed stream has run for rate_window_s and still flows
    (its last sample younger than flowing_s), with its rate. A burst that has
    stopped again by then ends with no line: the spell goes on."""

    def __init__(self, quiet_s: float = QUIET_S, repeat_s: float = QUIET_REPEAT_S,
                 rate_window_s: float = RATE_WINDOW_S, status_s: float = STATUS_PERIOD_S,
                 flowing_s: float = FLOWING_S) -> None:
        self.quiet_s = float(quiet_s)
        self.repeat_s = float(repeat_s)
        self.rate_window_s = float(rate_window_s)
        self.status_s = float(status_s)
        self.flowing_s = float(flowing_s)
        self._lock = threading.Lock()
        self._started: float | None = None
        self._last: float | None = None          # arrival of the last sample
        self.quiet = False
        self.spells = 0                          # quiet spells since start
        self._quiet_from: float | None = None    # arrival of the last sample before the spell
        self._spell_said = False                 # this spell's "no sample" line was logged
        self._unsaid = 0                         # spells not logged since the last "no sample" line
        self._said_t = -math.inf                 # when the last "no sample" line was logged
        self._back: list[float] | None = None    # [first, last arrival, count] of the resumed stream
        self._status_t: float | None = None
        self._n = 0                              # samples since the last status line
        self._n_mean = 0                         # ... of them finite, in the mean accel
        self._sum = [0.0, 0.0, 0.0]

    def start(self, now: float) -> None:
        with self._lock:
            self._started = self._status_t = float(now)

    def sample(self, now: float, accel: tuple) -> None:
        """One sample arrived at `now`."""
        with self._lock:
            self._last = now
            self._n += 1
            if all(math.isfinite(v) for v in accel):
                self._n_mean += 1
                for i in range(3):
                    self._sum[i] += accel[i]
            if self.quiet:
                if self._back is None:
                    self._back = [now, now, 1]
                else:
                    self._back[1] = now
                    self._back[2] += 1

    def tick(self, now: float, contacts: int = 0) -> list[tuple[str, str]]:
        lines: list[tuple[str, str]] = []
        with self._lock:
            if self._started is None:
                return lines
            if not self.quiet and self._last is not None and now - self._last >= self.quiet_s:
                self.quiet, self._quiet_from, self._back = True, self._last, None
                self.spells += 1
                self._spell_said = now - self._said_t >= self.repeat_s
                if self._spell_said:
                    more = (f" ({self._unsaid} more quiet spell(s) since the last line)" if self._unsaid else "")
                    lines.append(("warning", f"MOSS IMU: no sample for {self.quiet_s:g} s - "
                                             f"IMU contacts and gyro prior OFF{more}"))
                    self._said_t, self._unsaid = now, 0
                else:
                    self._unsaid += 1
            if self.quiet and self._back is not None and now - self._back[0] >= self.rate_window_s:
                first, last, count = self._back
                if now - last > self.flowing_s:
                    self._back = None             # a burst that stopped again: the same spell, nothing said
                else:
                    rate = (count - 1) / (last - first) if last > first else 0.0
                    if self._spell_said:
                        lines.append(("info", f"MOSS IMU: samples back ({rate:.0f} Hz) after "
                                              f"{first - self._quiet_from:.1f} s quiet"))
                    self.quiet, self._back = False, None
            if now - self._status_t >= self.status_s:
                lines.append(self._status(now, contacts))
                self._status_t, self._n, self._n_mean, self._sum = now, 0, 0, [0.0, 0.0, 0.0]
        return lines

    def _status(self, now: float, contacts: int) -> tuple[str, str]:
        """The periodic line (under the lock): alive while samples arrive."""
        tail = f"{contacts} contact(s) so far, {self.spells} quiet spell(s)"
        if self._last is None:
            return ("warning", f"MOSS IMU: status waiting, no sample {now - self._started:.0f} s after start "
                               "- IMU contacts and gyro prior OFF")
        if now - self._last >= self.quiet_s:
            return ("warning", f"MOSS IMU: status QUIET, no sample for {now - self._last:.0f} s "
                               f"- IMU contacts and gyro prior OFF, {tail}")
        span, n = now - self._status_t, self._n
        mean = [v / self._n_mean for v in self._sum] if self._n_mean else [math.nan] * 3
        return ("info", f"MOSS IMU: status alive, {n} samples in {span:.0f} s ({n / span:.0f} Hz), mean accel "
                        f"({mean[0]:+.2f}, {mean[1]:+.2f}, {mean[2]:+.2f}) m/s^2, {tail}")


class ImuCsvLog:
    """Every sample appended to a CSV, flushed every CSV_FLUSH_S of wall
    time and on close. Never raises: an I/O error is logged once and the rows
    of that flush are dropped."""

    def __init__(self, path: str, flush_s: float = CSV_FLUSH_S) -> None:
        self.path = path
        self.flush_s = float(flush_s)
        self._rows: list[str] = []
        self._lock = threading.Lock()        # the row buffer
        self._io_lock = threading.Lock()     # one writer at a time (the sample thread, or close at stop)
        self._last_flush = time.monotonic()
        self._error_logged = False
        self.rows_written = 0

    @staticmethod
    def _cell(v: Any, fmt: str) -> str:
        return "" if v is None else format(v, fmt)

    def add(self, ts: float, accel: tuple, gyro: tuple, hp: float | None,
            commanded: tuple[int, int] | None) -> None:
        cmd = commanded if commanded is not None else (None, None)
        row = ",".join([format(ts, ".4f")] + [format(float(v), ".4f") for v in accel]
                       + [format(float(v), ".5f") for v in gyro]
                       + [self._cell(hp, ".4f"), self._cell(cmd[0], "d"), self._cell(cmd[1], "d")])
        with self._lock:
            self._rows.append(row + "\n")
        if time.monotonic() - self._last_flush >= self.flush_s:
            self.flush()

    def flush(self) -> None:
        with self._lock:
            rows, self._rows = self._rows, []
            self._last_flush = time.monotonic()
        if not rows:
            return
        with self._io_lock:
            self._write(rows)

    def _write(self, rows: list[str]) -> None:
        try:
            folder = os.path.dirname(self.path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            with open(self.path, "a", encoding="ascii") as fh:
                if new:
                    fh.write(CSV_HEADER)
                fh.writelines(rows)
            self.rows_written += len(rows)
        except Exception as exc:  # noqa: BLE001 - the log is a study, never a reason to stop
            if not self._error_logged:
                self._error_logged = True
                logger.warning(f"MOSS_IMU_LOG: cannot write {self.path} ({type(exc).__name__}: {exc}); "
                               "samples are dropped, the contacts go on")

    def close(self) -> None:
        self.flush()


class BumpImu(Module):
    """IMU contacts on `bump` (in front) and `bump_rear` (behind), and the
    watchdog on the imu stream; see the module docstring."""

    imu: In[Imu]
    base_telemetry: In[String]
    bump: Out[Bool]
    bump_rear: Out[Bool]

    def __init__(self, axis: str | None = None, threshold: float | None = None,
                 debounce_s: float | None = None, log_path: str | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # the blueprint passes the four values read when it was built; a module built
        # without them (all None) reads the same environment itself
        if axis is None and threshold is None and debounce_s is None and log_path is None:
            env = bump_settings()
            axis, threshold, debounce_s, log_path = env["axis"], env["threshold"], env["debounce_s"], env["log_path"]
        self._detector = ImuContactDetector(
            axis=DEFAULT_AXIS if axis is None else axis,
            threshold=DEFAULT_THRESHOLD if threshold is None else threshold,
            debounce_s=DEFAULT_DEBOUNCE_S if debounce_s is None else debounce_s)
        self._csv = ImuCsvLog(log_path) if log_path else None
        self._command: tuple[tuple[int, int], float] | None = None   # (target_pct, monotonic instant it was true)
        self._unsubscribe: Any = None
        self._samples = 0
        self._error: str | None = None
        self._watchdog = ImuWatchdog()
        self._watch_stop = threading.Event()
        self._watch_thread: threading.Thread | None = None
        self._watch_error: str | None = None

    @property
    def detector(self) -> ImuContactDetector:
        return self._detector

    @property
    def watchdog(self) -> ImuWatchdog:
        return self._watchdog

    @rpc
    def start(self) -> None:
        super().start()
        try:
            self._unsubscribe = self.imu.subscribe(self.on_imu)
        except Exception as exc:  # noqa: BLE001 - no imu stream: no IMU contacts, the stack goes on
            logger.warning(f"MOSS IMU: cannot subscribe to imu ({type(exc).__name__}: {exc}); no IMU contacts")
        d = self._detector
        logger.info(f"MOSS IMU: contacts on (front on bump, rear on bump_rear), forward axis {d.axis}, "
                    f"threshold {d.threshold:.1f} m/s^2, debounce {d.debounce_s:.1f} s, command younger than "
                    f"{d.command_max_age_s:.1f} s, csv {self._csv.path if self._csv else 'off'}, "
                    f"watchdog: quiet after {self._watchdog.quiet_s:g} s, status every {self._watchdog.status_s:g} s")
        self._start_watchdog()

    def _start_watchdog(self) -> None:
        """The watchdog's clock starts now, and its thread unless it runs."""
        self._watchdog.start(time.monotonic())
        if self._watch_thread is None:
            self._watch_stop.clear()
            self._watch_thread = threading.Thread(target=self._watch_loop, name="moss-imu-watchdog", daemon=True)
            self._watch_thread.start()

    @rpc
    def stop(self) -> None:
        self._watch_stop.set()
        thread, self._watch_thread = self._watch_thread, None
        if thread is not None:
            thread.join(timeout=1.0)
        unsubscribe, self._unsubscribe = self._unsubscribe, None
        if unsubscribe is not None:
            try:
                unsubscribe()
            except Exception:  # noqa: BLE001
                pass
        if self._csv is not None:
            self._csv.close()
        super().stop()

    def _watch_loop(self) -> None:
        while not self._watch_stop.wait(WATCH_PERIOD_S):
            self.watch(time.monotonic())

    def watch(self, now: float) -> list[str]:
        """One look of the watchdog at `now` (monotonic): its lines are logged
        through the dimOS logger and returned. Never raises: the thread that
        says a sensor went quiet must not go quiet itself."""
        try:
            lines = self._watchdog.tick(now, self._detector.contacts)
        except Exception as exc:  # noqa: BLE001
            cause = f"{type(exc).__name__}: {exc}"
            if cause != self._watch_error:
                self._watch_error = cause
                logger.warning(f"MOSS IMU: watchdog look failed ({cause})")
            return []
        for level, text in lines:
            (logger.warning if level == "warning" else logger.info)(text)
        return [text for _, text in lines]

    async def handle_base_telemetry(self, msg: String) -> None:
        try:
            doc = json.loads(getattr(msg, "data", "") or "{}")
            target = doc.get("target_pct")
            pair = (int(target[0]), int(target[1]))
        except (TypeError, ValueError, AttributeError, IndexError, KeyError):
            return
        try:
            age = max(0.0, float(doc.get("firmware_age_s") or 0.0))
        except (TypeError, ValueError):
            age = 0.0
        self._command = (pair, time.monotonic() - age)

    def command_now(self) -> tuple[tuple[int, int] | None, float]:
        """(the last target_pct, its age in s), or (None, inf) before any."""
        cmd = self._command
        if cmd is None:
            return None, math.inf
        return cmd[0], max(0.0, time.monotonic() - cmd[1])

    def on_imu(self, msg: Imu) -> bool:
        """Every sample (the subscription's callback). True on a contact."""
        try:
            ts = float(msg.ts)
            a, g = msg.linear_acceleration, msg.angular_velocity
            accel = (float(a.x), float(a.y), float(a.z))
            gyro = (float(g.x), float(g.y), float(g.z))
            self._watchdog.sample(time.monotonic(), accel)
            self._first_sample(accel, msg)
            commanded, age = self.command_now()
            rebounds = self._detector.rebounds
            contact = self._detector.feed(ts, accel, commanded, command_age_s=age)
            if self._csv is not None:
                self._csv.add(ts, accel, gyro, self._detector.last_hp,
                              commanded if age <= self._detector.command_max_age_s else None)
            if contact:
                self._publish_contact()
            elif self._detector.rebounds != rebounds:
                r = self._detector.last_rebound or {}
                logger.info(rebound_line(r.get("hp", 0.0), r.get("commanded", (0, 0)), r.get("side", "rear"),
                                         r.get("after_s", 0.0)))
            return contact
        except Exception as exc:  # noqa: BLE001 - one bad sample never kills the subscription
            cause = f"{type(exc).__name__}: {exc}"
            if cause != self._error:
                self._error = cause
                logger.warning(f"MOSS IMU: sample skipped ({cause})")
            return False

    def _publish_contact(self) -> None:
        """Log first, publish second: a contact that was felt is on the record
        even when its stream refuses it."""
        c = self._detector.last_contact or {}
        side = c.get("side", "front")
        logger.warning(contact_line(c.get("hp", 0.0), self._detector.threshold, c.get("commanded", (0, 0)), side))
        name = contact_stream(side)
        out = getattr(self, name)
        if getattr(out, "transport", None) is None:
            return
        try:
            out.publish(Bool(data=True))
        except Exception as exc:  # noqa: BLE001 - said, not swallowed; the next contact tries again
            logger.warning(f"MOSS IMU: the contact was not published on {name} ({type(exc).__name__}: {exc})")

    def _first_sample(self, accel: tuple, msg: Imu) -> None:
        """The IMU announced alive at its first sample, with where gravity sits."""
        self._samples += 1
        if self._samples == 1:
            logger.info(f"MOSS IMU: first sample in {getattr(msg, 'frame_id', '?')}: accel "
                        f"({accel[0]:+.2f}, {accel[1]:+.2f}, {accel[2]:+.2f}) m/s^2")
