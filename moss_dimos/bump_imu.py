"""BumpImu: contacts felt by the RealSense's accelerometer, published on `bump`.

The encoders are a poor bumper: a track that slips keeps turning against a
wall, an oblique contact leaves one track free, and the stall guard needs a
full second of frozen ticks before it calls a contact. The chassis itself
says it at once: hitting something while driving is a short, hard
deceleration along the direction of travel. This module reads it on the
camera's IMU (`imu`, published by moss_dimos.realsense_imu.MossRealSense) and
publishes Bool(True) on `bump`, the stream the coordinator's stall guard and
the explorer's back-off already publish on: the planner's escape, the
costmap's contact memory and the sounds react to it the same way.

The rule (ImuContactDetector, pure, benched with known traces):
  * forward = one signed accel axis of the frame the camera publishes in
    (MOSS_BUMP_AXIS, default "+z": librealsense's optical axes, z out of the
    lens, the rover's forward; docs/imu.md);
  * high-pass: the sample's forward component minus the mean of the samples
    of the last 0.5 s before it, so gravity, a tilted camera or a slope cancel;
  * a contact is |high-pass| >= MOSS_BUMP_ACCEL (default 4.0 m/s^2), with a
    non-zero track command younger than 1 s, decelerating AGAINST that command
    (driving forward: <= -threshold; reversing: >= +threshold; pivoting, the
    tracks of opposite signs: either sign);
  * once per contact: nothing for MOSS_BUMP_DEBOUNCE_S (default 0.7 s) after one.

The command is the firmware's target_pct from the coordinator's
`base_telemetry` (2 Hz), aged by the time since it arrived plus the
firmware's own age of that line.

  In  imu             : Imu      the camera's accel + gyro (every sample: a plain subscription, not the
                                 latest-only async handler, so a burst of samples is never thinned)
  In  base_telemetry  : String   JSON with target_pct [l, r] (percent, + = forward) and firmware_age_s
  Out bump            : Bool     True on a contact

Each contact is logged as one line, e.g.
    IMU contact: forward -4.8 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%
(the cockpit's decision feed matches its start, "IMU contact"). The other
lines start with "MOSS IMU:": the knobs at start, the first sample, then one
line a minute with the sample rate and where gravity sits.

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
from dimos_lcm.std_msgs import Bool   # the SAME Bool class as the coordinator's and the explorer's bump

logger = setup_logger()

DEFAULT_AXIS = "+z"             # librealsense optical frame: x right, y down, z forward (docs/imu.md)
DEFAULT_THRESHOLD = 4.0         # m/s^2 of high-passed forward deceleration
DEFAULT_DEBOUNCE_S = 0.7        # one contact, then silence for this long
WINDOW_S = 0.5                  # the running mean the high-pass subtracts
WARMUP_S = 0.25                 # history the window needs before a high-pass value is trusted
COMMAND_MAX_AGE_S = 1.0         # a track command older than this is no command
CSV_FLUSH_S = 2.0
CSV_HEADER = "ts,ax,ay,az,gx,gy,gz,hp,cmd_l,cmd_r\n"
HEARTBEAT_S = 60.0              # one "IMU alive" line per minute of samples
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
    clock, no I/O. feed() returns True once per contact; last_hp is the
    high-pass of the last sample (None while the window warms up)."""

    def __init__(self, axis: str = DEFAULT_AXIS, threshold: float = DEFAULT_THRESHOLD,
                 debounce_s: float = DEFAULT_DEBOUNCE_S, window_s: float = WINDOW_S,
                 warmup_s: float = WARMUP_S, command_max_age_s: float = COMMAND_MAX_AGE_S) -> None:
        self.axis = axis
        self._index, self._sign = parse_axis(axis)
        self.threshold = float(threshold)
        self.debounce_s = float(debounce_s)
        self.window_s = float(window_s)
        self.warmup_s = float(warmup_s)
        self.command_max_age_s = float(command_max_age_s)
        self._window: deque[tuple[float, float]] = deque()
        self._sum = 0.0
        self._appends = 0
        self.last_hp: float | None = None
        self.last_contact_ts: float | None = None
        self.last_contact: dict | None = None
        self.contacts = 0

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
        if self.last_contact_ts is not None and 0.0 <= ts - self.last_contact_ts < self.debounce_s:
            return False
        self.last_contact_ts = ts
        self.contacts += 1
        self.last_contact = {"ts": ts, "hp": hp, "commanded": (int(commanded[0]), int(commanded[1]))}
        return True


def contact_line(hp: float, threshold: float, commanded: tuple[int, int]) -> str:
    """The log line of one contact (the cockpit's decision feed reads it)."""
    return (f"IMU contact: forward {hp:+.1f} m/s^2 (threshold {threshold:.1f}) "
            f"while commanded L={int(commanded[0]):+d}% R={int(commanded[1]):+d}%")


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
    """IMU contacts on `bump`; see the module docstring."""

    imu: In[Imu]
    base_telemetry: In[String]
    bump: Out[Bool]

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
        self._beat_ts: float | None = None
        self._beat_samples = 0
        self._beat_sum = [0.0, 0.0, 0.0]
        self._error: str | None = None

    @property
    def detector(self) -> ImuContactDetector:
        return self._detector

    @rpc
    def start(self) -> None:
        super().start()
        try:
            self._unsubscribe = self.imu.subscribe(self.on_imu)
        except Exception as exc:  # noqa: BLE001 - no imu stream: no IMU contacts, the stack goes on
            logger.warning(f"MOSS IMU: cannot subscribe to imu ({type(exc).__name__}: {exc}); no IMU contacts")
        d = self._detector
        logger.info(f"MOSS IMU: contacts on, forward axis {d.axis}, threshold {d.threshold:.1f} m/s^2, "
                    f"debounce {d.debounce_s:.1f} s, command younger than {d.command_max_age_s:.1f} s, "
                    f"csv {self._csv.path if self._csv else 'off'}")

    @rpc
    def stop(self) -> None:
        unsubscribe, self._unsubscribe = self._unsubscribe, None
        if unsubscribe is not None:
            try:
                unsubscribe()
            except Exception:  # noqa: BLE001
                pass
        if self._csv is not None:
            self._csv.close()
        super().stop()

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
            commanded, age = self.command_now()
            contact = self._detector.feed(ts, accel, commanded, command_age_s=age)
            if self._csv is not None:
                self._csv.add(ts, accel, gyro, self._detector.last_hp,
                              commanded if age <= self._detector.command_max_age_s else None)
            self._heartbeat(ts, accel, msg)
            if contact:
                self._publish_contact()
            return contact
        except Exception as exc:  # noqa: BLE001 - one bad sample never kills the subscription
            cause = f"{type(exc).__name__}: {exc}"
            if cause != self._error:
                self._error = cause
                logger.warning(f"MOSS IMU: sample skipped ({cause})")
            return False

    def _publish_contact(self) -> None:
        c = self._detector.last_contact or {}
        logger.warning(contact_line(c.get("hp", 0.0), self._detector.threshold, c.get("commanded", (0, 0))))
        if getattr(self.bump, "transport", None) is None:
            return
        try:
            self.bump.publish(Bool(data=True))
        except Exception as exc:  # noqa: BLE001 - said, not swallowed; the next contact tries again
            logger.warning(f"MOSS IMU: the contact was not published on bump ({type(exc).__name__}: {exc})")

    def _heartbeat(self, ts: float, accel: tuple, msg: Imu) -> None:
        """The first sample, then one line per HEARTBEAT_S of samples: the IMU
        is alive, at what rate, and where gravity sits."""
        self._samples += 1
        if self._samples == 1:
            logger.info(f"MOSS IMU: first sample in {getattr(msg, 'frame_id', '?')}: accel "
                        f"({accel[0]:+.2f}, {accel[1]:+.2f}, {accel[2]:+.2f}) m/s^2")
            self._beat_ts = ts
            return
        self._beat_samples += 1
        for i in range(3):
            self._beat_sum[i] += accel[i]
        if self._beat_ts is not None and ts - self._beat_ts >= HEARTBEAT_S:
            span, n = ts - self._beat_ts, self._beat_samples
            mean = [s / n for s in self._beat_sum]
            logger.info(f"MOSS IMU: {n} samples in {span:.0f} s ({n / span:.0f} Hz), mean accel "
                        f"({mean[0]:+.2f}, {mean[1]:+.2f}, {mean[2]:+.2f}) m/s^2, "
                        f"{self._detector.contacts} contact(s) so far")
            self._beat_ts, self._beat_samples, self._beat_sum = ts, 0, [0.0, 0.0, 0.0]
