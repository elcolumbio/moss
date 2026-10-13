"""MOSS tracked base for dimOS: a TwistBaseAdapter over the QT Py motor firmware.

MOSS drives two geared DC motors through a Cytron MDD3A dual driver from an
Adafruit QT Py ESP32-S3 running `firmware/MOSS_QTPY_MDD3A` (this repo, fw
"qtpy-mdd3a-v3"; an MDD10A build adds a "driver" key to the telemetry). The
firmware speaks a line protocol on the QT Py's native USB CDC port (115200
nominal; the host must hold DTR high to receive anything):

    arm                 -> "OK"      READY for 10 s, then it disarms itself
    drive L R           -> "OK"      L, R in percent, -100..+100 (firmware limit)
                                     must be REFRESHED within 300 ms or the
                                     firmware stops ("timeout") - a deadman
    stop                -> "OK"
    ramp UP DOWN        -> "OK"      ms per 1 % rise / fall, 1..500 each (RAM only)
    status              -> one telemetry JSON line
    (unsolicited)          one telemetry JSON line every 200 ms:
        {"ms":..,"fw":"qtpy-mdd3a-v3","mode":0-3,"reason":"..","target_pct":[l,r],
         "output_pct":[l,r],"ramp_ms":[rise,fall],"ticks":[l,r],"invalid_edges":[l,r],
         "ina_ok":..,"bus_mV":..,"shunt_uV":..,"current_mA":null}
    errors:                "ERR not_armed" | "ERR invalid_command" | "ERR stopped_busy"

Modes: 0 OFF, 1 READY (armed, idle), 2 DRIVE, 3 TEST.

What this adapter does with it:

- write_enable(True)  = "arm"; write_enable(False) = "stop".
- write_velocities([vx, vy, wz]) maps the body twist to left/right track
  speeds (differential drive: vy is IGNORED, a tracked base cannot strafe),
  then to percent of the firmware limit, and sends "drive L R". The firmware
  is open loop (PWM percent): the metres-per-second scale is a CALIBRATION
  number (MossGeometry.mps_at_limit), measured on the floor, not derived.
- a keepalive thread re-sends the last non-zero drive every KEEPALIVE_S
  while the last command is fresher than STALE_S; past that it sends "stop"
  ONCE and holds. So a dead commander stops the tracks in <= STALE_S here
  and, if this process dies too, in 300 ms in the firmware. Nothing here
  ever prolongs a command nobody is refreshing.
- the firmware disarms itself 10 s after "arm" with no drive, and after any
  invalid line. The adapter does NOT guess that from telemetry (a line up to
  200 ms old can say OFF while the firmware is already in DRIVE, and an "arm"
  sent in DRIVE is "arm_while_moving": a STOP). The firmware is the source of
  truth: a drive answered "ERR not_armed" re-arms and retries once.
- odometry: the firmware counts quadrature ticks; with MossGeometry
  .ticks_per_m set (measured), read_velocities()/read_odometry() come from
  tick deltas. Without it (the default), read_velocities() returns the COMMANDED
  twist (open loop, low confidence) and read_odometry() returns None -
  an honest None beats a pose made of guesses.
- telemetry: the last firmware JSON line is kept (`telemetry`, a copy, and
  `telemetry_age_s`); the coordinator republishes it on `base_telemetry`
  (blueprints.MossControlCoordinator, payload = telemetry_payload below).

dof: reported as 3 so the stock dimOS twist joints (vx, vy, wz) and the
gamepad module work unchanged; vy is dropped at the tracks.

Calibration (env, all optional):
    MOSS_PORT           serial port (default: the QT Py by-id path,
                        /dev/serial/by-id/*QT_Py*; when absent the sentinel
                        /dev/serial/by-id/MISSING-QT-PY, which fails loudly -
                        never /dev/ttyUSB0, the lidar on MOSS)
    MOSS_TRACK_M        distance between the two track centrelines, metres
                        (default robot_profile.MOSS.track_m, 0.266 CAD, placeholder)
    MOSS_MPS_AT_LIMIT   floor speed in m/s at MOSS_MAX_PCT, measured
    MOSS_MAX_PCT        percent ceiling, <= the firmware's 100
    MOSS_MIN_PCT        static-friction floor in percent (0 = off, default 50: measured 40 on a full battery, 10 of margin for the sag): a non-zero track command below it is raised to it, sign kept; an arc is scaled up whole so its curvature survives (floor_tracks)
    MOSS_BUS_REF_MV     the bus voltage the floor and the speeds were measured at (default 12400): every command is scaled by ref/bus so a sagging pack puts the same voltage on the motors (never below 1, at most x1.5, capped at the limit); 0 = off
    MOSS_ONE_TRACK_PIVOT a pivot whose one track freezes while the other turns is not a contact: that track is braked and the pivot turns on the other, for the rest of the run (default on; 0 = off, such a pivot is a stall)
    MOSS_TICKS_PER_M    encoder ticks per metre of track travel, measured
    MOSS_TURN_GAIN      turn authority vs forward speed (teleop_speeds), 0.5..4, default 2
    MOSS_RAMP_MS        "UP DOWN" sent as 'ramp UP DOWN' after connect (while pulsing, unset = "1 1")
    MOSS_PULSE_MS       period of the full-duty time slicing (default 0 = off; 500 = five slices of 100 ms): a percent becomes a duty of
                        full-power slices over the period (pulse_tracks), the way MOSS's tracks can be steered
    MOSS_PULSE_MIN_PCT  duty floor of a non-zero track while pulsing (default 40)
    MOSS_PIVOT_PCT      every pivot (tracks in opposite directions) is sent at this percent, both tracks (default 100, 0 = as asked)
    (no variable)       stall guard: a commanded track whose encoder moved < STALL_MIN_TICKS over STALL_S -> 'stop', logged, counted
    (no variable)       feedback loss: telemetry older than TELEMETRY_FRESH_S while a non-zero command is held blinds the
                        stall guard and the IMU contacts (both read the telemetry): warned once, then once per
                        READ_WARN_PERIOD_S with a count, and a recovery line when telemetry comes back; a failing serial
                        read is logged the same way, never once per read
    MOSS_MOCK_BUS       any value but 0/false/no/off: drive the fake firmware (no motors)
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, replace

from moss_dimos import robot_profile

BAUDRATE = 115200


def default_port() -> str:
    """MOSS_PORT, else the QT Py by-id path, else a sentinel that cannot open
    (robot_profile.moss_qtpy_port). Resolved when an adapter is built, so a
    board plugged after import is still found."""
    return robot_profile.moss_qtpy_port()


DEFAULT_PORT = default_port()   # import-time snapshot, kept for callers that read it
FIRMWARE_MAX_PCT = 100         # Control.h MAX_PERCENT: the firmware refuses more
FIRMWARE_COMMAND_TIMEOUT_S = 0.300   # Control.h COMMAND_TIMEOUT_MS
FIRMWARE_ARM_TIMEOUT_S = 10.0        # Control.h ARM_TIMEOUT_MS
KEEPALIVE_S = 0.100            # re-send period, 3x inside the firmware deadman
STALE_S = 0.500                # no fresh write for this long -> "stop", once
REPLY_TIMEOUT_S = 0.150        # the firmware answers within a loop tick (ms)
CONNECT_PROBE_S = 1.000        # the first "status" of a process: reader thread start + first parse on a Jetson take longer than a reply tick
TELEMETRY_FRESH_S = 1.0        # mode read from telemetry older than this = unknown; older while driving = feedback lost
READ_WARN_PERIOD_S = 5.0       # a lost feedback (telemetry silent while driving, a failing serial read) is warned on
                               # its first occurrence, then once per this period with a count, and its recovery is logged
MOCK_BUS_ENV = "MOSS_MOCK_BUS"
STALL_S = 1.0                  # every commanded track frozen for this long = STALLED: the rover is pushing against
                               # something (no bumper on MOSS: the encoders are the bumper)
STALL_MIN_TICKS = 20           # ticks over STALL_S that count as motion (a track at 40 % makes ~800 per second)
STALL_WINDOW_S = 2.0           # how long the tick history is kept for the comparison
STALL_PUSH_PCT = 75            # before a stall below this command becomes a contact, the tracks get one window at this
                               # percent: a half-full battery sags and a track at the floor just creeps, which is not a wall
PULSE_MS_DEFAULT = 0           # MOSS_PULSE_MS: period of the full-duty time slicing, 0 = off (then MOSS_MIN_PCT applies).
                               # Off on MOSS: with the drive wheels seated on their axles the tracks turn continuously
                               # from 40 % (floor probe), so slices are only for a base that moves at full duty alone
PULSE_MS_MAX = 2000
PULSE_RAMP = "1 1"             # firmware ramp while pulsing: 1 ms per %, so a 100 ms slice reaches full duty
PIVOT_PCT_DEFAULT = 100        # MOSS_PIVOT_PCT: a pivot (tracks in opposite directions) is sent at this percent on both
                               # tracks whatever the planner asked, 0 = off. The pad pivots at 100 % continuous and the
                               # tracks turn without jolts there; the planner's slow pivots (8 %) can only be jolts
BUS_REF_MV_DEFAULT = 12400     # MOSS_BUS_REF_MV: the bus voltage the floor and the speeds were measured at (a full 3S pack).
                               # A percent is a duty of the bus: at 10.5 V under load the same 50 % puts 15 % less voltage on
                               # the motor and the tracks creep or freeze, so every command is scaled by ref/bus (capped at
                               # BUS_GAIN_MAX, never below 1, never above the limit). 0 = off.
BUS_GAIN_MAX = 1.5             # the scale never goes past this (a 3S pack at 8.3 V is a pack to stop on, not to compensate)
BUS_SANE_MV = (6000, 20000)    # a bus reading outside this is a broken sensor: no compensation
ONE_TRACK_PIVOT_DEFAULT = True # MOSS_ONE_TRACK_PIVOT: a pivot whose one track does not turn (the other does) is not a contact:
                               # the frozen track is braked (0) and the pivot turns on the other track, for the rest of the run
PULSE_MIN_PCT_DEFAULT = 40     # MOSS_PULSE_MIN_PCT: the lowest duty of a non-zero track while pulsing; a planner pivot at
                               # 8 % would be one 100 ms slice per second (seen on the floor: jerky and slow), 40 % moves the rover


def pulse_ms() -> int:
    """MOSS_PULSE_MS clamped to 0..PULSE_MS_MAX; 0 = no pulsing."""
    return max(0, min(PULSE_MS_MAX, int(_env_float("MOSS_PULSE_MS", PULSE_MS_DEFAULT))))


def pulse_min_pct() -> int:
    """MOSS_PULSE_MIN_PCT clamped to 0..100: the duty floor of the pulses."""
    return max(0, min(100, int(_env_float("MOSS_PULSE_MIN_PCT", PULSE_MIN_PCT_DEFAULT))))


def pivot_pct() -> int:
    """MOSS_PIVOT_PCT clamped to 0..100: the percent every pivot is sent at (0 = as asked)."""
    return max(0, min(100, int(_env_float("MOSS_PIVOT_PCT", PIVOT_PCT_DEFAULT))))


def bus_ref_mv() -> int:
    """MOSS_BUS_REF_MV clamped to 0..20000 (0 = no compensation)."""
    return max(0, min(20000, int(_env_float("MOSS_BUS_REF_MV", BUS_REF_MV_DEFAULT))))


def bus_gain(bus_mv, ref_mv: int) -> float:
    """The scale that puts the reference voltage back on the motors: ref/bus,
    never below 1 (a full pack is never slowed down), never above BUS_GAIN_MAX;
    1 without a reference, without a reading, or with a reading outside BUS_SANE_MV."""
    try:
        bus = int(bus_mv)
    except (TypeError, ValueError):
        return 1.0
    if ref_mv <= 0 or not BUS_SANE_MV[0] <= bus <= BUS_SANE_MV[1]:
        return 1.0
    return max(1.0, min(BUS_GAIN_MAX, ref_mv / bus))


def bus_scaled(want: tuple[int, int], gain: float, lim: int) -> tuple[int, int]:
    """Each track scaled by the gain, sign kept, capped at the limit; a 0 stays 0."""
    def one(p: int) -> int:
        if p == 0 or gain == 1.0:
            return p
        return int(min(lim, round(abs(p) * gain))) * (1 if p > 0 else -1)
    return one(want[0]), one(want[1])


def one_track_pivot_on() -> bool:
    """MOSS_ONE_TRACK_PIVOT: on unless 0 / false / no / off."""
    return robot_profile.env_on("MOSS_ONE_TRACK_PIVOT", ONE_TRACK_PIVOT_DEFAULT)


def floor_tracks(want: tuple[int, int], floor: int, lim: int) -> tuple[int, int]:
    """The static-friction floor, the curvature kept. A non-zero track below
    the floor does not turn, so a lone track (the other at 0) and a pivot
    (opposite signs) are raised to the floor, sign kept. An arc (same signs)
    is scaled up until its inner track reaches the floor, so the ratio the
    commander asked for survives: raising only the inner track straightens
    the arc (57/32 asked, 56/50 sent: the rover goes nearly straight and the
    planner insists on the turn for twenty seconds). When the outer track
    would pass the limit it is capped there, and the inner track, below the
    floor again, goes to 0: a one-track turn, tighter than asked but in the
    asked direction, which the planner corrects at its next tick."""
    left, right = want
    if floor <= 0 or (left == 0 and right == 0):
        return want
    if left == 0 or right == 0 or (left > 0) != (right > 0):
        def raise_(p: int) -> int:
            return p if p == 0 or abs(p) >= floor else (floor if p > 0 else -floor)
        return raise_(left), raise_(right)
    inner, outer = min(abs(left), abs(right)), max(abs(left), abs(right))
    if inner >= floor:
        return want
    sign = 1 if left > 0 else -1
    k = floor / inner
    if outer * k > lim + 0.5:
        inner_capped = int(round(lim * inner / outer))
        inner_out = inner_capped if inner_capped >= floor else 0
        return (sign * lim, sign * inner_out) if abs(left) >= abs(right) else (sign * inner_out, sign * lim)
    return sign * int(round(abs(left) * k)), sign * int(round(abs(right) * k))


def pivot_tracks(want: tuple[int, int], pct: int) -> tuple[int, int]:
    """A pivot (left and right non-zero, opposite signs) is sent at +-pct on both
    tracks, the sign of each kept; anything else passes unchanged."""
    left, right = want
    if pct <= 0 or left == 0 or right == 0 or (left > 0) == (right > 0):
        return want
    return (pct if left > 0 else -pct, pct if right > 0 else -pct)


def pulse_tracks(want: tuple[int, int], tick: int, n_ticks: int, full: int = 100) -> tuple[int, int]:
    """Time-slice a (left, right) percent pair into full-duty pulses.

    The tracks of MOSS only turn at full duty (the geared motors behind the
    MDD3A), so a percent is delivered as a DUTY over a period of n_ticks
    keepalive ticks: a track at p % is ON (+-full) for ceil(|p| * n / 100)
    ticks from the start of the period and OFF (0) for the rest. Any non-zero
    p is on for at least one tick; |p| >= 100 is always on; 0 is always off.
    Both tracks share the period, so a straight command pulses both together."""
    if n_ticks <= 0:
        return want
    k = tick % n_ticks
    out = []
    for p in want:
        if p == 0:
            out.append(0)
            continue
        on = math.ceil(min(100, abs(p)) * n_ticks / 100 - 1e-9)
        out.append((full if p > 0 else -full) if k < on else 0)
    return out[0], out[1]

MODE_OFF, MODE_READY, MODE_DRIVE, MODE_TEST = 0, 1, 2, 3


_LOGGER = None


def _log():
    """dimOS's structlog logger when dimOS is there (that is what `dimos log`
    shows - a plain logging.getLogger() has no handler under the daemon, so
    every line of this adapter would be lost), the stdlib with a handler
    otherwise. Resolved on first use so that importing this
    module never imports dimos (the cold bench)."""
    global _LOGGER
    if _LOGGER is None:
        try:
            from dimos.utils.logging_config import setup_logger
            _LOGGER = setup_logger()
        except Exception:
            lg = logging.getLogger(__name__)
            if not lg.handlers and not logging.getLogger().handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(
                    logging.Formatter("%(levelname)s %(name)s: %(message)s"))
                lg.addHandler(handler)
            lg.setLevel(logging.INFO)
            _LOGGER = lg
    return _LOGGER


def _env_float(name: str, default):
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        _log().warning(f"MOSS base: {name}={raw!r} is not a number - using {default}")
        return default


@dataclass
class MossGeometry:
    """Everything the twist <-> tracks mapping needs, measured on the floor
    with the 90:1 motors (tools/moss_floor_probe.py, tools/moss_speed_profile.py;
    the track centre distance is still the CAD's). Each field has an env
    override so a flight can be re-scaled without a code change."""
    track_m: float = robot_profile.MOSS.track_m   # PLACEHOLDER 0.266 (CAD track centre distance), to measure
    mps_at_limit: float = 0.32     # measured: the plateau at 100 % is 5 420 ticks/s = 0.32 m/s (it takes 3.5 s to get there on the 3 A driver)
    max_pct: int = FIRMWARE_MAX_PCT
    ticks_per_m: float | None = 17190.0   # measured: 1.00 m in 5 s at 100 % = 17 234 / 17 144 ticks (left / right)
    min_pct: int = 20    # static-friction floor: a non-zero track command below this percent is raised to it (sign kept). 0 = off. MOSS_MIN_PCT. 20 with the 90:1 motors: both tracks turn from 20 % (the earlier motors needed 40, plus margin)

    @classmethod
    def from_env(cls) -> "MossGeometry":
        pct = int(_env_float("MOSS_MAX_PCT", FIRMWARE_MAX_PCT))
        tpm = _env_float("MOSS_TICKS_PER_M", None)
        max_pct = max(1, min(FIRMWARE_MAX_PCT, pct))
        floor = int(_env_float("MOSS_MIN_PCT", cls.min_pct))
        return cls(track_m=_env_float("MOSS_TRACK_M", cls.track_m),
                   mps_at_limit=_env_float("MOSS_MPS_AT_LIMIT", cls.mps_at_limit),
                   max_pct=max_pct,
                   ticks_per_m=(tpm if tpm and tpm > 0 else None),
                   min_pct=max(0, min(max_pct, floor)))


def twist_to_tracks(vx: float, wz: float, g: MossGeometry) -> tuple[int, int]:
    """Body twist -> (left, right) percent, clamped to +-g.max_pct.

    Differential drive: v_left = vx - wz*track/2, v_right = vx + wz*track/2.
    Percent is proportional to speed with g.mps_at_limit <-> g.max_pct.
    A twist too small to round to 1 % IS a stop (0, 0). A non-zero percent
    below g.min_pct is raised by floor_tracks (static-friction floor: the
    tracks do not turn below it, so the alternative is a stall), which keeps
    the curvature of an arc instead of straightening it."""
    half = g.track_m / 2.0
    v_l = vx - wz * half
    v_r = vx + wz * half
    scale = g.max_pct / g.mps_at_limit if g.mps_at_limit > 0 else 0.0
    lim = int(g.max_pct)
    floor = max(0, min(lim, int(g.min_pct)))

    def pct(v: float) -> int:
        return max(-lim, min(lim, int(round(v * scale))))

    return floor_tracks((pct(v_l), pct(v_r)), floor, lim)


TURN_GAIN_DEFAULT = 2.0   # MOSS_TURN_GAIN: turn authority vs forward speed, see teleop_speeds


def turn_gain() -> float:
    return max(0.5, min(4.0, _env_float("MOSS_TURN_GAIN", TURN_GAIN_DEFAULT)))


def teleop_envelope(g: MossGeometry) -> dict:
    """The gamepad's last gate (clamp_twist), sized to the tracks rather
    than to the gamepad's generic default. Per-axis ceilings = teleop_speeds; the rim is set
    so that a mixed command is NEVER scaled down as a whole: the adapter
    clamps each track on its own (twist_to_tracks), which is what gives the
    tank feel below. Keys match gamepad.TwistEnvelope."""
    lin, ang = teleop_speeds(g)
    lever = g.track_m / 2.0
    return {"linear": lin, "angular": ang, "lever": lever, "rim": lin + ang * lever}


def teleop_speeds(g: MossGeometry) -> tuple[float, float]:
    """(linear m/s, angular rad/s) at FULL stick for the gamepad module, sized
    to the base: full stick forward = g.max_pct on both tracks. Full stick
    sideways = turn_gain() times the spin-in-place rate (v = wz * track/2):
    with the default gain 2, at full forward the inner track STOPS at half
    turn stick and REVERSES beyond it, full turn = a full pivot (the inner
    track reverses past a turn threshold instead of only stopping); gain 1 =
    the inner track only ever stops.
    The gamepad module's own defaults (0.6 m/s, 1.2 rad/s) would saturate at
    25 % from the deadzone on, an on/off throttle; this keeps the stick
    proportional."""
    lin = float(g.mps_at_limit)
    ang = turn_gain() * 2.0 * lin / g.track_m if g.track_m > 0 else 0.0
    return lin, ang


def tracks_to_twist(v_l: float, v_r: float, g: MossGeometry) -> tuple[float, float, float]:
    """(left, right) track speeds in m/s -> (vx, vy=0, wz)."""
    vx = (v_l + v_r) / 2.0
    wz = (v_r - v_l) / g.track_m if g.track_m > 0 else 0.0
    return vx, 0.0, wz


class MossBaseAdapter:
    """TwistBaseAdapter for MOSS (two tracks, QT Py ESP32-S3 + MDD3A firmware, USB CDC).

    Args:
        dof: reported DOF (3, see module docstring).
        address: serial port; None = default_port() (MOSS_PORT, the QT Py by-id, or the sentinel).
        port_factory: injectable callable returning a serial-like object
            (write / readline / in_waiting / close / is_open). Benches inject
            the fake firmware; MOSS_MOCK_BUS does the same in a real dimOS run.
        geometry: MossGeometry; None = MossGeometry.from_env().
    """

    def __init__(self, dof: int = 3, address: str | None = None,
                 hardware_id: str = "base", port_factory=None,
                 geometry: MossGeometry | None = None, **_ignored):
        self._dof = dof
        self._port = address or default_port()
        self._hardware_id = hardware_id
        self._port_factory = port_factory
        self._geometry = geometry or MossGeometry.from_env()
        self._ser = None
        self._mock = False
        self._lock = threading.RLock()
        self._reply_cv = threading.Condition(self._lock)
        self._replies: list[str] = []
        self._telemetry: dict | None = None
        self._tel_logged_t = 0.0
        self._telemetry_t: float | None = None
        self._connected = False
        self._enabled = False
        self._target = (0, 0)              # the last pair sent to the firmware (a pulse value when pulsing)
        self._want = (0, 0)                # the last pair asked by the commander (the duty when pulsing)
        self._target_twist = (0.0, 0.0, 0.0)
        self._target_t = 0.0
        self._stopped_for_stale = True
        # full-duty time slicing: n keepalive ticks per period, 0 = off (floor mode)
        self._pulse_n = int(round(pulse_ms() / 1000.0 / KEEPALIVE_S)) if pulse_ms() > 0 else 0
        self._ramp = ""                    # the ramp sent at connect ("" = the firmware default)
        self._tick = 0
        self._pivot_pct = pivot_pct()      # pivots sent at this percent on both tracks (0 = as asked)
        self._tick_hist: list[tuple[float, int, int]] = []   # (monotonic, ticks L, ticks R) from telemetry, STALL_WINDOW_S deep
        self._drive_since: float | None = None               # when the current non-zero command started
        self._stalled = False                                # True from a stall until the next accepted command
        self._stall_count = 0                                # stalls since connect (telemetry_payload, the coordinator's bump)
        self._pushing: tuple[int, int] | None = None         # the command being pushed harder before a contact is declared
        self._push_origin: tuple[int, int] | None = None     # the commander's own pair while the push runs (its repeats keep the push)
        self._bus_ref_mv = bus_ref_mv()                      # the voltage the percents were measured at (0 = no compensation)
        self._one_track = one_track_pivot_on()               # a pivot with one frozen track turns on the other track
        self._dead_pivots: set[tuple[str, int]] = set()      # (side, sign) that did not turn in a pivot: braked in such pivots from then on
        # the mapping geometry: when pulsing, the static-friction floor becomes the duty floor of the pulses
        self._map_geometry = replace(self._geometry, min_pct=pulse_min_pct()) if self._pulse_n else self._geometry
        self._last_logged = None
        # feedback loss, logged once then rate-limited (see _note_read_failure, _note_blind)
        self._read_failures = 0                              # consecutive serial reads that raised
        self._last_read_warn_t = 0.0
        self._blind_ticks = 0                                # consecutive keepalive ticks driving on stale telemetry
        self._last_blind_warn_t = 0.0
        self._reader = None
        self._keepalive = None
        self._stop_threads = threading.Event()
        # odometry from ticks
        self._pose = [0.0, 0.0, 0.0]
        self._last_ticks: tuple[int, int] | None = None
        self._last_ticks_t: float | None = None
        self._velocity = (0.0, 0.0, 0.0)

    # ── properties for benches ─────────────────────────────────────────
    @property
    def geometry(self) -> MossGeometry:
        return self._geometry

    @property
    def mock_bus(self) -> bool:
        return self._mock

    @property
    def target_pct(self) -> tuple[int, int]:
        return self._target

    @property
    def firmware_mode(self) -> int | None:
        """Mode from the last telemetry line, None if none or stale."""
        with self._lock:
            if self._telemetry is None or self._telemetry_t is None:
                return None
            if time.monotonic() - self._telemetry_t > TELEMETRY_FRESH_S:
                return None
            try:
                return int(self._telemetry.get("mode"))
            except (TypeError, ValueError):
                return None

    @property
    def telemetry(self) -> dict | None:
        with self._lock:
            return dict(self._telemetry) if self._telemetry else None

    @property
    def read_failure_count(self) -> int:
        """Consecutive serial reads that raised (0 = healthy)."""
        return self._read_failures

    @property
    def blind_count(self) -> int:
        """Consecutive keepalive ticks that held a non-zero command on telemetry
        older than TELEMETRY_FRESH_S (0 = the guards can see)."""
        return self._blind_ticks

    @property
    def telemetry_age_s(self) -> float | None:
        """Seconds since the last telemetry line arrived (same monotonic stamp
        firmware_mode uses), None before the first one."""
        with self._lock:
            if self._telemetry_t is None:
                return None
            return max(0.0, time.monotonic() - self._telemetry_t)

    # ── connection ─────────────────────────────────────────────────────
    def _open_port(self):
        if self._port_factory is not None:
            self._mock = getattr(self._port_factory, "is_mock", False)
            return self._port_factory()
        if robot_profile.env_on(MOCK_BUS_ENV, False):   # the one switch parser for MOSS_* on/off
            from .moss_mock import FakeMossFirmware
            self._mock = True
            _log().info(f"MOSS base: MOCK BUS ({MOCK_BUS_ENV} set) - no motors will move")
            return FakeMossFirmware(auto_time=True)   # wall clock: live telemetry, real deadmen
        import serial
        return serial.Serial(self._port, BAUDRATE, timeout=0.05)

    def connect(self) -> bool:
        with self._lock:
            if self._connected:
                return True
            try:
                self._ser = self._open_port()
            except Exception as exc:
                _log().error(f"MOSS base: cannot open {self._port} @{BAUDRATE}: {exc!r}")
                self._ser = None
                return False
            self._stop_threads.clear()
            self._read_failures = self._blind_ticks = 0
            self._reader = threading.Thread(target=self._read_loop, name="moss-rx", daemon=True)
            self._reader.start()
            # probe: the firmware answers "status" with one telemetry line
            self._replies.clear()
            if not self._send("status", expect_status=True, timeout=CONNECT_PROBE_S):
                _log().error(f"MOSS base: no telemetry from the firmware on "
                             f"{self._port} @{BAUDRATE} (is it the QT Py by-id port, "
                             "and is the firmware flashed?)")
                self._teardown()
                return False
            self._connected = True
            self._keepalive = threading.Thread(target=self._keepalive_loop,
                                               name="moss-keepalive", daemon=True)
            self._keepalive.start()
            ramp = os.environ.get("MOSS_RAMP_MS", "").strip()   # "10 5" = ms per % up / down
            if self._pulse_n and not ramp:
                ramp = PULSE_RAMP                                 # a slice must reach full duty inside a keepalive tick
            if self._pulse_n:
                _log().info(f"MOSS base: full-duty pulses, period {self._pulse_n * KEEPALIVE_S * 1000:.0f} ms "
                            f"({self._pulse_n} slices of {KEEPALIVE_S * 1000:.0f} ms), ramp {ramp} ms per %")
            self._ramp = ramp
            if ramp and not self._mock:
                if self._send(f"ramp {ramp}"):
                    _log().info(f"MOSS base: firmware ramp set to {ramp} ms per % (rise fall)")
                else:
                    _log().warning(f"MOSS base: 'ramp {ramp}' refused ({self._last_err}) - firmware default kept")
            _log().info(f"MOSS base: connected on {self._port}, firmware mode="
                        f"{self.firmware_mode} geometry={self._geometry}")
            return True

    def _teardown(self) -> None:
        self._stop_threads.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self._connected = False
        self._enabled = False

    def disconnect(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self._send("stop")
                except Exception:
                    pass
            self._teardown()

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    # ── serial plumbing ────────────────────────────────────────────────
    def _read_loop(self) -> None:
        while not self._stop_threads.is_set():
            ser = self._ser
            if ser is None:
                return
            try:
                raw = ser.readline()
            except Exception as exc:
                # the port closed under us (stack stopping): not an error
                if self._stop_threads.is_set() or not getattr(ser, "is_open", True):
                    return
                self._note_read_failure(exc)
                time.sleep(0.05)
                continue
            self._note_read_success()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            with self._lock:
                if line.startswith("{"):
                    try:
                        self._telemetry = json.loads(line)
                        now = time.monotonic()
                        self._note_telemetry_back(now)
                        self._telemetry_t = now
                        self._ingest_ticks(self._telemetry)
                    except (ValueError, TypeError):
                        pass
                    self._replies.append("STATUS")
                elif line == "OK" or line.startswith("ERR"):
                    self._replies.append(line)
                else:
                    # boot banner and the like
                    _log().info(f"MOSS firmware: {line}")
                    continue
                self._reply_cv.notify_all()

    def _note_read_failure(self, exc: Exception) -> None:
        """A serial read raised (the port is open, the stack is not stopping):
        counted, logged on the first failure and then once per
        READ_WARN_PERIOD_S - a dead port would otherwise write 20 lines a second."""
        self._read_failures += 1
        now = time.monotonic()
        if self._read_failures == 1 or now - self._last_read_warn_t >= READ_WARN_PERIOD_S:
            self._last_read_warn_t = now
            _log().error(f"MOSS base: serial read failed on {self._port} "
                         f"({self._read_failures} consecutive): {exc!r}")

    def _note_read_success(self) -> None:
        if self._read_failures:
            _log().info(f"MOSS base: serial reads recovered after {self._read_failures} failures")
            self._read_failures = 0

    def _note_blind(self, now: float) -> None:
        """Called under the lock every keepalive tick while a non-zero command is
        held. Telemetry older than TELEMETRY_FRESH_S means the stall guard has no
        encoder ticks to compare and the IMU contacts no command to qualify a
        deceleration with: both are blind while the tracks are driven. Counted,
        warned on the first tick and then once per READ_WARN_PERIOD_S."""
        age = None if self._telemetry_t is None else now - self._telemetry_t
        if age is not None and age <= TELEMETRY_FRESH_S:
            return
        self._blind_ticks += 1
        if self._blind_ticks == 1 or now - self._last_blind_warn_t >= READ_WARN_PERIOD_S:
            self._last_blind_warn_t = now
            since = "since the connect" if age is None else f"for {age:.1f} s"
            _log().warning(f"MOSS base: no telemetry from the motor board {since} on {self._port} while "
                           f"driving L={self._want[0]:+d}% R={self._want[1]:+d}% ({self._blind_ticks} consecutive "
                           "ticks) - stall guard and IMU bump blind")

    def _note_telemetry_back(self, now: float) -> None:
        """Called under the lock when a telemetry line arrives, before its stamp."""
        if self._blind_ticks:
            gap = now - self._telemetry_t if self._telemetry_t is not None else 0.0
            _log().info(f"MOSS base: telemetry recovered after {self._blind_ticks} failures "
                        f"({gap:.1f} s without a line)")
            self._blind_ticks = 0

    def _send(self, line: str, expect_status: bool = False, timeout: float | None = None) -> bool:
        """Write one command line and wait for its reply. Called with the lock
        held (RLock). True on "OK" (or a telemetry line when expect_status)."""
        ser = self._ser
        if ser is None:
            return False
        before = len(self._replies)
        try:
            ser.write((line + "\n").encode())
        except Exception as exc:
            _log().error(f"MOSS base: serial write failed ({line!r}): {exc!r}")
            return False
        deadline = time.monotonic() + (REPLY_TIMEOUT_S if timeout is None else timeout)
        while True:
            for reply in self._replies[before:]:
                if expect_status and reply == "STATUS":
                    return True
                if not expect_status and reply == "OK":
                    return True
                if reply.startswith("ERR"):
                    self._last_err = reply
                    return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._last_err = "timeout"
                return False
            self._reply_cv.wait(remaining)

    _last_err = ""

    # ── enable ─────────────────────────────────────────────────────────
    def write_enable(self, enable: bool) -> bool:
        with self._lock:
            if not self._connected:
                return False
            if not enable:
                ok = self._send("stop")
                self._enabled = False
                self._target = (0, 0)
                self._stopped_for_stale = True
                return ok
            if not self._arm():
                self._enabled = False
                return False
            self._enabled = True
            return True

    def _arm(self) -> bool:
        ok = self._send("arm")
        if not ok:
            _log().error(f"MOSS base: 'arm' refused ({self._last_err})")
        return ok

    def read_enabled(self) -> bool:
        return self._enabled

    # ── commands ───────────────────────────────────────────────────────
    def write_velocities(self, velocities: list[float]) -> bool:
        with self._lock:
            if not self._connected:
                return False
            try:
                vx, vy, wz = (list(velocities) + [0.0, 0.0, 0.0])[:3]
                vx, vy, wz = float(vx), float(vy), float(wz)
            except Exception:
                return False
            want = self._pivot_fallback(pivot_tracks(twist_to_tracks(vx, wz, self._map_geometry), self._pivot_pct))
            if want != (0, 0) and not self._enabled:
                return False
            if self._pushing is not None:
                if want == self._push_origin:
                    want = self._pushing            # the commander repeats itself: the push goes on, and is what is sent
                else:
                    self._pushing = self._push_origin = None   # a new order: the push is over
            left, right = self._to_wire(want)
            ok = self._drive(left, right)
            if ok:
                if want == (0, 0):
                    self._drive_since = None
                elif self._want == (0, 0) or self._drive_since is None:
                    self._drive_since = time.monotonic()
                self._stalled = False
                self._want = want
                self._target = (left, right)
                self._target_twist = (vx, 0.0, wz)
                self._target_t = time.monotonic()
                self._stopped_for_stale = want == (0, 0)
                self._velocity = self._velocity if self._geometry.ticks_per_m else (vx, 0.0, wz)
                self._log_cmd((vx, vy, wz), want)
            return ok

    def _slice(self, want: tuple[int, int]) -> tuple[int, int]:
        """The pair to send now: the pulse value of this tick when pulsing, else the pair itself."""
        if not self._pulse_n:
            return want
        return pulse_tracks(want, self._tick, self._pulse_n, int(self._geometry.max_pct))

    def _gain(self) -> float:
        """The bus compensation of the moment, from the last telemetry line."""
        return bus_gain((self._telemetry or {}).get("bus_mV"), self._bus_ref_mv)

    def _to_wire(self, want: tuple[int, int]) -> tuple[int, int]:
        """What goes on the wire for the commander's pair: the bus compensation,
        then the pulse slice of this tick when pulsing. The commander's pair
        itself (`_want`) stays as asked: the stall guard and the push compare
        percents between themselves, which the gain does not change."""
        return self._slice(bus_scaled(want, self._gain(), int(self._geometry.max_pct)))

    def _pivot_fallback(self, want: tuple[int, int]) -> tuple[int, int]:
        """A pivot whose one track is known not to turn in that direction (it froze
        while the other turned, see _stall_check) turns on the other track: the
        dead side is sent 0 (braked). Anything else passes unchanged."""
        left, right = want
        if not self._dead_pivots or left == 0 or right == 0 or (left > 0) == (right > 0):
            return want
        if ("L", 1 if left > 0 else -1) in self._dead_pivots:
            return 0, right
        if ("R", 1 if right > 0 else -1) in self._dead_pivots:
            return left, 0
        return want

    def _drive(self, left: int, right: int) -> bool:
        """Send 'drive L R'; on 'ERR not_armed' (arm expiry, invalid line,
        the firmware's own deadman) re-arm and retry ONCE. Never pre-arm
        from telemetry: see the module docstring. A (0, 0) drive is sent as
        'stop' when the firmware is not armed: stopping never needs an arm."""
        if (left, right) == (0, 0) and self.firmware_mode in (MODE_OFF, None):
            return self._send("stop")
        if self._send(f"drive {left} {right}"):
            return True
        if self._last_err == "ERR not_armed" and self._enabled:
            reason = (self._telemetry or {}).get("reason")
            _log().warning(f"MOSS base: 'drive' refused not_armed (firmware reason: "
                           f"{reason}) - re-arming once")
            if self._arm() and self._send(f"drive {left} {right}"):
                return True
        _log().error(f"MOSS base: 'drive {left} {right}' refused ({self._last_err})")
        return False

    def write_stop(self) -> bool:
        with self._lock:
            if not self._connected:
                return False
            ok = self._send("stop")
            if ok:
                self._target = (0, 0)
                self._want = (0, 0)
                self._drive_since = None
                self._target_twist = (0.0, 0.0, 0.0)
                self._target_t = time.monotonic()
                self._stopped_for_stale = True
                if not self._geometry.ticks_per_m:
                    self._velocity = (0.0, 0.0, 0.0)
                self._log_cmd((0.0, 0.0, 0.0), (0, 0))
            return ok

    def _keepalive_loop(self) -> None:
        """Refresh the last drive inside the firmware deadman while it is
        fresh; past STALE_S send one 'stop' and hold until a new write."""
        while not self._stop_threads.is_set():
            time.sleep(KEEPALIVE_S)
            with self._lock:
                self._tick += 1
                if not self._connected or self._stopped_for_stale:
                    continue
                age = time.monotonic() - self._target_t
                if age > STALE_S:
                    _log().warning(f"MOSS base: no twist for {age:.2f}s - stop (deadman)")
                    self._send("stop")
                    self._target = (0, 0)
                    self._want = (0, 0)
                    self._target_twist = (0.0, 0.0, 0.0)
                    self._stopped_for_stale = True
                    if not self._geometry.ticks_per_m:
                        self._velocity = (0.0, 0.0, 0.0)
                    continue
                if self._want != (0, 0):
                    self._note_blind(time.monotonic())
                    if self._stall_check():
                        continue
                    # pulsing: the slice of this tick, 'drive 0 0' included (an OFF slice
                    # keeps the firmware armed and its 300 ms deadman fed)
                    self._target = self._to_wire(self._want)
                    self._send(f"drive {self._target[0]} {self._target[1]}")
                    # what the firmware actually applies, and the battery
                    # voltage under load: one line per 2 s
                    tel = self._telemetry
                    now = time.monotonic()
                    if tel and now - self._tel_logged_t > 2.0:
                        self._tel_logged_t = now
                        _log().info(f"MOSS telemetry: out={tel.get('output_pct')} target={tel.get('target_pct')} "
                                    f"bus={tel.get('bus_mV')} mV ticks={tel.get('ticks')} mode={tel.get('mode')}")

    def _stall_check(self) -> bool:
        """Called under the lock, every keepalive tick while a non-zero command
        is held. Samples the encoder ticks from telemetry; once the command has
        been held for STALL_S, a commanded track whose ticks moved less than
        STALL_MIN_TICKS over the last STALL_S is a stall: 'stop' is sent, the
        command dropped, the stall counted and logged. Returns True on a stall
        (the caller then sends nothing more this tick). The next accepted
        command clears the flag - the planner's escape drives away."""
        tel = self._telemetry
        now = time.monotonic()
        if tel and self._telemetry_t is not None and now - self._telemetry_t < TELEMETRY_FRESH_S:
            try:
                ticks = (int(tel["ticks"][0]), int(tel["ticks"][1]))
            except (KeyError, TypeError, ValueError, IndexError):
                ticks = None
            if ticks is not None and (not self._tick_hist or self._tick_hist[-1][0] < self._telemetry_t):
                self._tick_hist.append((self._telemetry_t, ticks[0], ticks[1]))
                while self._tick_hist and now - self._tick_hist[0][0] > STALL_WINDOW_S:
                    self._tick_hist.pop(0)
        if self._drive_since is None or now - self._drive_since < STALL_S or len(self._tick_hist) < 2:
            return False
        old = [h for h in self._tick_hist if now - h[0] >= STALL_S]
        if not old:
            return False
        t_old, l_old, r_old = old[-1]
        t_new, l_new, r_new = self._tick_hist[-1]
        if t_new - t_old < STALL_S * 0.8:
            return False
        want_l, want_r = self._want
        stalled_l = want_l != 0 and abs(l_new - l_old) < STALL_MIN_TICKS
        stalled_r = want_r != 0 and abs(r_new - r_old) < STALL_MIN_TICKS
        if not (stalled_l or stalled_r):
            self._pushing = self._push_origin = None
            return False
        # A pivot (opposite signs) with ONE frozen track while the other turns is not a
        # contact: a track that does not turn in that direction under the pivot's load (the
        # right track in reverse on a sagging pack, every pivot to the right became a stall,
        # a back-off and an abandoned goal). From now on such a pivot turns on the other
        # track, the dead one braked at 0; both tracks frozen in that turn is a contact.
        pivot = want_l != 0 and want_r != 0 and (want_l > 0) != (want_r > 0)
        if pivot and stalled_l != stalled_r and self._one_track:
            side = "R" if stalled_r else "L"
            sign = 1 if (want_r if stalled_r else want_l) > 0 else -1
            if (side, sign) not in self._dead_pivots:
                self._dead_pivots.add((side, sign))
                _log().warning(f"MOSS base: pivot L={want_l:+d}% R={want_r:+d}%: the {'right' if stalled_r else 'left'} track "
                               f"did not turn (encoders L={l_new - l_old:+d} R={r_new - r_old:+d} ticks in {t_new - t_old:.1f} s) "
                               f"- not a contact: from now on this pivot turns on the {'left' if stalled_r else 'right'} track "
                               f"only, the {'right' if stalled_r else 'left'} track braked")
            new_want = (want_l, 0) if stalled_r else (0, want_r)
            self._pushing = self._push_origin = None
            self._want = new_want
            self._drive_since = now
            self._tick_hist = self._tick_hist[-1:]
            self._target = self._to_wire(new_want)
            self._send(f"drive {self._target[0]} {self._target[1]}")
            return True
        # Both commanded tracks frozen = the rover pushes against something. One track frozen
        # while the other turns is a contact only when the frozen track was commanded at least
        # as hard as the moving one (it should have turned at least as much: something holds
        # it); frozen at the LOWER command it is an arc's inner track under its friction floor,
        # which still moves the rover and is the planner's stuck detection's case, not a bump.
        if stalled_l != stalled_r:
            frozen_cmd, moving_cmd = (abs(want_l), abs(want_r)) if stalled_l else (abs(want_r), abs(want_l))
            if frozen_cmd < moving_cmd:
                return False
        # Frozen below STALL_PUSH_PCT: push harder for one more window before calling it a
        # contact (the tracks creep at the floor on a sagging battery). The pushed command
        # keeps each track's sign; a track at 0 stays at 0.
        if max(abs(want_l), abs(want_r)) < STALL_PUSH_PCT and self._pushing is None:
            push = (0 if want_l == 0 else (STALL_PUSH_PCT if want_l > 0 else -STALL_PUSH_PCT),
                    0 if want_r == 0 else (STALL_PUSH_PCT if want_r > 0 else -STALL_PUSH_PCT))
            self._pushing = push
            self._push_origin = (want_l, want_r)
            self._want = push
            self._drive_since = now
            self._tick_hist = self._tick_hist[-1:]
            _log().info(f"MOSS base: tracks frozen at L={want_l:+d}% R={want_r:+d}% - pushing at {STALL_PUSH_PCT} % for "
                        f"{STALL_S:.0f} s before calling it a contact")
            return False
        self._pushing = self._push_origin = None
        self._send("stop")
        self._stall_count += 1
        self._stalled = True
        self._want = (0, 0)
        self._target = (0, 0)
        self._drive_since = None
        self._stopped_for_stale = True
        if not self._geometry.ticks_per_m:
            self._velocity = (0.0, 0.0, 0.0)
        _log().warning(f"MOSS base: STALL #{self._stall_count}: tracks commanded L={want_l:+d}% R={want_r:+d}% "
                       f"but the encoders moved L={l_new - l_old:+d} R={r_new - r_old:+d} ticks in "
                       f"{t_new - t_old:.1f} s - stopped (the encoders are the bumper)")
        return True

    @property
    def stalled(self) -> bool:
        """True from a stall until the next accepted command."""
        return self._stalled

    @property
    def stall_count(self) -> int:
        return self._stall_count

    def _log_cmd(self, twist, pct) -> None:
        if pct == self._last_logged:
            return
        self._last_logged = pct
        head = "MOSS base MOCK:" if self._mock else "MOSS base:"
        vx, vy, wz = twist
        gain = self._gain()
        wire = bus_scaled(pct, gain, int(self._geometry.max_pct))
        comp = "" if gain == 1.0 else f" x{gain:.2f} for the bus at {int((self._telemetry or {}).get('bus_mV') or 0) / 1000:.1f} V = L={wire[0]:+d}% R={wire[1]:+d}%"
        _log().info(f"{head} twist vx={vx:+.3f} vy={vy:+.3f}(ignored) wz={wz:+.3f} "
                    f"-> tracks L={pct[0]:+d}% R={pct[1]:+d}%{comp} (limit {self._geometry.max_pct}%)")

    # ── feedback ───────────────────────────────────────────────────────
    def _ingest_ticks(self, tel: dict) -> None:
        """Called with the lock held, on every telemetry line."""
        tpm = self._geometry.ticks_per_m
        if not tpm:
            return
        try:
            l, r = tel["ticks"]
            l, r = int(l), int(r)
        except (KeyError, TypeError, ValueError):
            return
        now = time.monotonic()
        if self._last_ticks is not None and self._last_ticks_t is not None:
            dt = now - self._last_ticks_t
            if dt > 0:
                v_l = (l - self._last_ticks[0]) / tpm / dt
                v_r = (r - self._last_ticks[1]) / tpm / dt
                vx, _, wz = tracks_to_twist(v_l, v_r, self._geometry)
                self._velocity = (vx, 0.0, wz)
                x, y, th = self._pose
                d = (v_l + v_r) / 2.0 * dt
                dth = wz * dt
                x += d * math.cos(th + dth / 2.0)
                y += d * math.sin(th + dth / 2.0)
                th += dth
                self._pose = [x, y, th]
        self._last_ticks = (l, r)
        self._last_ticks_t = now

    def read_velocities(self) -> list[float]:
        with self._lock:
            return list(self._velocity)

    def read_odometry(self) -> list[float] | None:
        with self._lock:
            if not self._geometry.ticks_per_m:
                return None
            return list(self._pose)


# ── telemetry payload for the bus (MossControlCoordinator.base_telemetry) ──

TELEMETRY_KEYS = ("fw", "driver", "mode", "reason", "bus_mV", "ina_ok", "ticks",
                  "target_pct", "output_pct", "ramp_ms")


def telemetry_payload(adapter, now: float) -> dict | None:
    """One base_telemetry document from anything shaped like MossBaseAdapter
    (`telemetry` dict or None, `telemetry_age_s`, `mock_bus`); None when the
    adapter has no telemetry (an adapter without a `telemetry` property, or
    MOSS before its first line), so the coordinator publishes nothing.

    Firmware keys are copied as they are; a key the flashed build does not
    send (driver before the MDD10A build) is None, never invented. `ts` is
    the caller's wall clock, `firmware_age_s` the adapter's monotonic age.
    """
    tel = getattr(adapter, "telemetry", None)
    if not isinstance(tel, dict) or not tel:
        return None
    age = getattr(adapter, "telemetry_age_s", None)
    doc = {"ts": float(now)}
    for key in TELEMETRY_KEYS:
        doc[key] = tel.get(key)
    doc["firmware_age_s"] = None if age is None else round(float(age), 3)
    doc["mock"] = bool(getattr(adapter, "mock_bus", False))
    doc["stalled"] = bool(getattr(adapter, "stalled", False))
    doc["stall_count"] = int(getattr(adapter, "stall_count", 0) or 0)
    return doc
