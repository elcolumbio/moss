"""MOSS geometry: one place for the body numbers the nav stack reads.

The lidar driver, the lidar odometry, the costmap and the planner do not bake
in the rover's dimensions: they read a RobotProfile at call time through
``current()``. The values below are MOSS's, and every one of them can be
overridden by a MOSS_* environment variable (read at call time too, so a
flight can be re-scaled without a code change).

Frames: base_link IS the lidar's floor projection. The odometry publishes
world->base_link at the lidar's x, y; the costmap body footprint and the
planner clearances centre on it. A lidar mounted off the rotation centre
would need that offset applied in lidar_odometry first.

This module is pure (no dimOS import): the benches, the launcher's preflight
and the cockpit import it on any machine. It also holds MOSS's device and
state-dir resolution and the on/off switch parser (env_on), for the same
reason.
"""
from __future__ import annotations

import glob
import logging
import math
import os
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple

# Read only to warn: this package knows one robot, MOSS.
PROFILE_ENV = "ROBOT_PROFILE"

_STDLIB_LOG = logging.getLogger(__name__)
_DIMOS_LOG = None
_DIMOS_LOGGING = "dimos.utils.logging_config"


def _log():
    """Where this module's warnings go.

    Inside a dimOS process (the blueprint build and every worker: dimOS's
    logging is already loaded there), dimOS's own logger, so the line lands
    in the run's main.jsonl - a plain logging.getLogger() has no handler
    under dimOS, and a worker's stderr is /dev/null. Anywhere else (the
    preflight, the cockpit, a bench), the stdlib logger, whose warnings reach
    stderr. Never imports dimOS itself: this module stays pure."""
    global _DIMOS_LOG
    if _DIMOS_LOG is None:
        mod = sys.modules.get(_DIMOS_LOGGING)
        if mod is None:
            return _STDLIB_LOG      # not cached: dimOS may load later in this process
        try:
            _DIMOS_LOG = mod.setup_logger()
        except Exception:  # noqa: BLE001 - a broken dimOS logger must not stop the profile
            return _STDLIB_LOG
    return _DIMOS_LOG


def env_on(name: str, default: bool = True, env: dict | None = None) -> bool:
    """An on/off switch from the environment (MOSS_CAMERA, MOSS_WRIST,
    MOSS_RECORD, MOSS_RERUN, MOSS_MOCK_BUS), parsed ONE way everywhere:
    unset or empty -> default; 0 / false / no / off (any case, spaces
    stripped) -> False; anything else -> True."""
    env = os.environ if env is None else env
    raw = str(env.get(name, "")).strip().lower()
    if not raw:
        return default
    return raw not in ("0", "false", "no", "off")


@dataclass(frozen=True)
class RobotProfile:
    name: str
    lidar_height_m: float                       # lidar_link above base_link
    lidar_yaw_offset_deg: float                 # lidar 0-deg mark vs the body's +x
    scan_min_range_m: float                     # returns closer than this, any direction, are dropped
    scan_mask_sectors_deg: tuple[tuple[float, float], ...]   # (start, end) sectors dropped under scan_mask_range_m; may wrap past 360
    scan_mask_range_m: float
    body_box_m: tuple[float, float]             # half extents |x|, |y| of returns that are the robot itself
    body_clear_m: tuple[float, float]           # half extents of the footprint certainly free under the body
    robot_width_m: float
    pivot_diameter_m: float
    control_margin_m: float
    track_m: float                              # track centre distance
    camera_xyz_base_m: tuple[float, float, float]   # depth camera in base_link (lidar_odometry's projection)
    camera_pitch_deg: float                     # pitched DOWN by this much


# EVERY value below is a PLACEHOLDER until measured on the rover (CAD nominal
# or a guess with margin). Each one has an env override, read at call time:
#   MOSS_LIDAR_HEIGHT_M       0.20          lidar scan plane above the floor
#   MOSS_LIDAR_YAW_DEG        0.0           lidar 0-deg mark vs the nose
#   MOSS_SCAN_MIN_RANGE_M     0.15
#   MOSS_SCAN_MASK_DEG        ""            sectors "a b, c d" (deg); empty = none
#   MOSS_SCAN_MASK_RANGE_M    0.30
#   MOSS_BODY_BOX_M           "0.22 0.18"   returns inside are the robot
#   MOSS_BODY_CLEAR_M         "0.20 0.16"   cleared under the body
#   MOSS_ROBOT_WIDTH_M        0.34
#   MOSS_PIVOT_DIAMETER_M     0.50
#   MOSS_CONTROL_MARGIN_M     0.05
#   MOSS_TRACK_M              0.266         CAD centre distance (moss_adapter reads the same variable)
#   MOSS_CAMERA_XYZ_M         "0.156 0 0.075"  D455F face on the nose, CAD nominal
#   MOSS_CAMERA_PITCH_DEG     0.0
MOSS = RobotProfile(
    name="moss",
    lidar_height_m=0.20,
    lidar_yaw_offset_deg=0.0,
    scan_min_range_m=0.15,
    scan_mask_sectors_deg=(),
    scan_mask_range_m=0.30,
    body_box_m=(0.22, 0.18),
    body_clear_m=(0.20, 0.16),
    robot_width_m=0.34,
    pivot_diameter_m=0.50,
    control_margin_m=0.05,
    track_m=0.266,
    camera_xyz_base_m=(0.156, 0.0, 0.075),
    camera_pitch_deg=0.0,
)

# field -> (env var, number of floats; 0 = list of (start, end) pairs)
_MOSS_ENV = {
    "lidar_height_m": ("MOSS_LIDAR_HEIGHT_M", 1),
    "lidar_yaw_offset_deg": ("MOSS_LIDAR_YAW_DEG", 1),
    "scan_min_range_m": ("MOSS_SCAN_MIN_RANGE_M", 1),
    "scan_mask_sectors_deg": ("MOSS_SCAN_MASK_DEG", 0),
    "scan_mask_range_m": ("MOSS_SCAN_MASK_RANGE_M", 1),
    "body_box_m": ("MOSS_BODY_BOX_M", 2),
    "body_clear_m": ("MOSS_BODY_CLEAR_M", 2),
    "robot_width_m": ("MOSS_ROBOT_WIDTH_M", 1),
    "pivot_diameter_m": ("MOSS_PIVOT_DIAMETER_M", 1),
    "control_margin_m": ("MOSS_CONTROL_MARGIN_M", 1),
    "track_m": ("MOSS_TRACK_M", 1),
    "camera_xyz_base_m": ("MOSS_CAMERA_XYZ_M", 3),
    "camera_pitch_deg": ("MOSS_CAMERA_PITCH_DEG", 1),
}

_warned: set[tuple[str, str]] = set()


def _warn_once(var: str, raw: str, why: str) -> None:
    # call-time parsing runs every lidar revolution: say a bad value once
    if (var, raw) not in _warned:
        _warned.add((var, raw))
        _log().warning("%s=%r ignored (%s) - placeholder kept", var, raw, why)


def _floats(raw: str) -> list[float]:
    vals = [float(t) for t in raw.replace(",", " ").replace(";", " ").split()]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("not finite")
    return vals


def _parse(var: str, raw: str, count: int, default):
    try:
        if count == 0:
            if raw.strip().lower() in ("", "none", "-"):
                return ()
            vals = _floats(raw)
            if len(vals) % 2:
                raise ValueError("sectors come in (start, end) pairs")
            return tuple((vals[i] % 360.0, vals[i + 1] % 360.0) for i in range(0, len(vals), 2))
        vals = _floats(raw)
        if len(vals) != count:
            raise ValueError(f"expected {count} number(s)")
        return vals[0] if count == 1 else tuple(vals)
    except ValueError as exc:
        _warn_once(var, raw, str(exc))
        return default


def moss_profile(env: dict | None = None) -> RobotProfile:
    """MOSS with the MOSS_* env overrides applied (env defaults to os.environ)."""
    env = os.environ if env is None else env
    changes = {}
    for field_name, (var, count) in _MOSS_ENV.items():
        raw = env.get(var)
        if raw is None or (raw.strip() == "" and count != 0):
            continue
        value = _parse(var, raw, count, getattr(MOSS, field_name))
        if value != getattr(MOSS, field_name):
            changes[field_name] = value
    return replace(MOSS, **changes) if changes else MOSS


def current(env: dict | None = None) -> RobotProfile:
    """The active profile, read NOW (not at import): MOSS with its MOSS_*
    overrides. ROBOT_PROFILE is not read; set to anything but "moss" it
    earns a single warning per process, so a stray export is visible: on
    the preflight's terminal, and in the run log (main.jsonl) of a flight."""
    env = os.environ if env is None else env
    name = env.get(PROFILE_ENV, "").strip().lower()
    if name and name != "moss" and (PROFILE_ENV, name) not in _warned:
        _warned.add((PROFILE_ENV, name))
        _log().warning("ROBOT_PROFILE is ignored: this package only knows MOSS (got %r)", name)
    return moss_profile(env)


class Derived(NamedTuple):
    lethal_clearance_m: float
    pivot_clearance_m: float


def derived(profile: RobotProfile | None = None) -> Derived:
    """Planner/explorer clearances from the body: lethal = width/2 + margin,
    pivot = pivot diameter / 2 (MOSS defaults: 0.22 and 0.25 m)."""
    p = profile or current()
    return Derived(p.robot_width_m / 2 + p.control_margin_m, p.pivot_diameter_m / 2)


# ── devices and state (pure: the preflight and the cockpit use them) ──

BYID_ROOT = "/dev/serial/by-id"
MOSS_QTPY_GLOB = "*QT_Py*"
MOSS_LIDAR_GLOB = "*CP2102N*"
# A missing device resolves to a path that cannot exist: the module that opens
# it fails loudly with this name in the log. Never a fallback to another
# device - on MOSS /dev/ttyUSB0 is the lidar, not the motor board.
MOSS_QTPY_MISSING = "/dev/serial/by-id/MISSING-QT-PY"
MOSS_LIDAR_MISSING = "/dev/serial/by-id/MISSING-C1"


def resolve_by_id(pattern: str, missing: str, root: str | None = None) -> str:
    """First match (sorted, so the choice is stable) of root/pattern, else
    the sentinel. The path returned keeps the by-id form: it survives a
    re-enumeration that renumbers ttyACM/ttyUSB."""
    base = root if root is not None else BYID_ROOT
    hits = sorted(glob.glob(os.path.join(base, pattern)))
    return hits[0] if hits else missing


def moss_qtpy_port(env: dict | None = None) -> str:
    """MOSS motor board: MOSS_PORT if set, else the QT Py by-id, else the sentinel."""
    env = os.environ if env is None else env
    return env.get("MOSS_PORT", "").strip() or resolve_by_id(MOSS_QTPY_GLOB, MOSS_QTPY_MISSING)


def moss_lidar_port(env: dict | None = None) -> str:
    """MOSS lidar: MOSS_LIDAR_PORT if set, else the CP2102N by-id, else the sentinel."""
    env = os.environ if env is None else env
    return env.get("MOSS_LIDAR_PORT", "").strip() or resolve_by_id(MOSS_LIDAR_GLOB, MOSS_LIDAR_MISSING)


def moss_state_dir(env: dict | None = None) -> Path:
    """MOSS_STATE_DIR, default ~/.local/state/moss (expanded, absolute)."""
    env = os.environ if env is None else env
    raw = env.get("MOSS_STATE_DIR", "").strip()
    # absolute, always: dimOS resolves a relative recorder path into its own
    # site-packages, and a cwd-relative state dir depends on who launched
    return Path(raw or "~/.local/state/moss").expanduser().absolute()


def moss_recording_path(env: dict | None = None) -> Path:
    """Where MossRecorder writes: <state>/recordings/moss.db (absolute - a
    relative db_path lands in dimOS's site-packages)."""
    return moss_state_dir(env) / "recordings" / "moss.db"
