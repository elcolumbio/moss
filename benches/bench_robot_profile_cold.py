"""Cold bench for moss_dimos.robot_profile: MOSS's body, read at call time.

Known input -> known output in physical units. Groups:

  A. the profile - MOSS's placeholders (lidar 0.20 m up, body box 0.22 x 0.18,
                   body clear 0.20 x 0.16, width 0.34, pivot 0.50 -> lethal
                   0.22 m, pivot clearance 0.25 m), exactly the fields MOSS reads
  B. selection   - current() is MOSS whatever ROBOT_PROFILE says (one warning
                   for a foreign value), the env overrides read at call time
                   (MOSS_ROBOT_WIDTH_M=0.40 -> lethal 0.25 m), a malformed value
                   keeps the placeholder
  C. devices     - by-id resolution of the QT Py and the CP2102N lidar bridge,
                   the sentinels, the env overrides, the absolute state dir;
                   env_on, the one parser of the MOSS_* on/off switches
  S. the speed   - tools/speed_guard.py: the speed comes from the LOG's own
     guard         timestamps, one interval over the profile's 0.45 m/s ceiling
                   is a kill (exploration stop, then the stack down), the
                   planner's 0.385 m/s cruise is not, a ceiling at or under that
                   cruise is refused, a run that is over makes the watchdog stand
                   down, the pid file is ours
  T. arming      - tools/arm_speed_guard.sh REPLACES the previous run's
                   watchdog (killed by its pid, the new one proven born after the
                   launch), and refuses without that proof or below the cruise
  L. the log     - outside dimOS the warnings go to the stdlib logger (stderr)
                   and dimOS is never imported; inside a dimOS process they
                   land in the run's main.jsonl (subprocess; needs dimOS)
  D. the lidar   - rplidar_c1: a return at 0.30 m dead ahead is kept, one inside
                   MOSS's body box is dropped, polar_to_xy(90 deg, 1000 mm) =
                   (0, -1.0) m
  E. the map     - costmap2d tags lidar returns at the profile's height (0.20 m),
                   body_clear clears the profile's footprint, a stop never
                   freezes the map; the planner's clearances and the odometry's
                   lidar lift and depth projection follow the profile

A-C need nothing. L needs dimOS for its main.jsonl half (SKIPPED with a line
without it). S and T need dimos_lcm (tools/explore_ctl.py imports it; SKIPPED
with a line without it) and Linux's /proc. D and E import the modules: with dimOS when it is there, else
behind a stand-in for every dimOS name - the functions under test are pure
numpy and never touch dimOS objects. A reader module that does not import,
or lacks a function under test, is a KO.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_robot_profile_cold.py
"""
import ast
import asyncio
import copy
import dataclasses
import importlib
import importlib.abc
import importlib.machinery
import logging
import math
import os
import re
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from moss_dimos import robot_profile as rp  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def clear_env():
    for var in [k for k in os.environ if k.startswith("MOSS_")] + ["ROBOT_PROFILE"]:
        os.environ.pop(var, None)


def load(name):
    """moss_dimos.<name>, or None after a KO when it does not import."""
    try:
        return importlib.import_module(f"moss_dimos.{name}")
    except Exception as exc:  # noqa: BLE001
        check(f"moss_dimos.{name} imports", False, f"{type(exc).__name__}: {exc}")
        return None


class _Capture(logging.Handler):
    def __init__(self):
        super().__init__(logging.WARNING)
        self.lines = []

    def emit(self, record):
        self.lines.append(record.getMessage())


clear_env()

# ── A. the MOSS profile ──────────────────────────────────────────────────
print("A. the MOSS profile: placeholders, derived clearances, the fields MOSS reads")
M = rp.MOSS
check("name 'moss'", M.name == "moss")
check("lidar 0.20 m up, yaw 0 deg",
      (M.lidar_height_m, M.lidar_yaw_offset_deg) == (0.20, 0.0))
check("min range 0.15 m, no mask sector, mask range 0.30 m",
      (M.scan_min_range_m, M.scan_mask_sectors_deg, M.scan_mask_range_m) == (0.15, (), 0.30))
check("body box 0.22 x 0.18, body clear 0.20 x 0.16",
      M.body_box_m == (0.22, 0.18) and M.body_clear_m == (0.20, 0.16))
check("width 0.34, pivot 0.50, margin 0.05, track 0.266 m",
      (M.robot_width_m, M.pivot_diameter_m, M.control_margin_m, M.track_m) == (0.34, 0.50, 0.05, 0.266))
check("camera (0.156, 0, 0.075) m, pitch 0",
      M.camera_xyz_base_m == (0.156, 0.0, 0.075) and M.camera_pitch_deg == 0.0)
check("speed ceiling 0.45 m/s (tools/speed_guard.py's limit)", M.max_speed_mps == 0.45)
dm = rp.derived(M)
check("lethal clearance 0.22 m, pivot clearance 0.25 m",
      close(dm.lethal_clearance_m, 0.22) and close(dm.pivot_clearance_m, 0.25),
      f"{dm.lethal_clearance_m:.4f} / {dm.pivot_clearance_m:.4f}")
check("derived == width / 2 + margin and diameter / 2, bit for bit",
      dm.lethal_clearance_m == 0.34 / 2 + 0.05 and dm.pivot_clearance_m == 0.50 / 2)
fields = {f.name for f in dataclasses.fields(rp.RobotProfile)}
want = {"name", "lidar_height_m", "lidar_yaw_offset_deg", "scan_min_range_m",
        "scan_mask_sectors_deg", "scan_mask_range_m", "body_box_m", "body_clear_m",
        "robot_width_m", "pivot_diameter_m", "control_margin_m", "track_m",
        "camera_xyz_base_m", "camera_pitch_deg", "max_speed_mps"}
check("RobotProfile holds exactly the 15 fields MOSS reads", fields == want,
      f"extra {sorted(fields - want)}, missing {sorted(want - fields)}")
check("every numeric field has its MOSS_* override",
      set(rp._MOSS_ENV) == want - {"name"}, f"{sorted(set(rp._MOSS_ENV) ^ (want - {'name'}))}")
check("one robot only: no profile table, no second profile",
      not hasattr(rp, "PROFILES") and [n for n in vars(rp)
                                       if isinstance(getattr(rp, n), rp.RobotProfile)] == ["MOSS"])
try:
    M.lidar_height_m = 1.0   # type: ignore[misc]
    check("assigning a field raises", False, "no exception")
except Exception as exc:  # dataclasses.FrozenInstanceError
    check("assigning a field raises", True, type(exc).__name__)


# ── B. selection and env overrides, at call time ─────────────────────────
print("B. current(): MOSS, read now; env overrides; ROBOT_PROFILE ignored")
cap = _Capture()
check("outside dimOS the warnings go to the stdlib logger 'moss_dimos.robot_profile'",
      rp._log() is logging.getLogger("moss_dimos.robot_profile"))
rp._log().addHandler(cap)
check("nothing set -> MOSS", rp.current() is M)
os.environ["ROBOT_PROFILE"] = "moss"
check("ROBOT_PROFILE=moss -> MOSS, no warning", rp.current() == M and cap.lines == [])
os.environ["ROBOT_PROFILE"] = " MOSS "
check("ROBOT_PROFILE=' MOSS ' (case, spaces) -> no warning", rp.current() == M and cap.lines == [])
os.environ["ROBOT_PROFILE"] = "rover9"
p1, p2 = rp.current(), rp.current()
check("ROBOT_PROFILE=rover9 -> still MOSS", p1 == M and p2 == M)
check("... and ONE warning saying it is ignored, not one per call",
      len(cap.lines) == 1 and "ROBOT_PROFILE is ignored" in cap.lines[0], f"{cap.lines}")
os.environ.pop("ROBOT_PROFILE", None)
os.environ["MOSS_ROBOT_WIDTH_M"] = "0.40"
check("MOSS_ROBOT_WIDTH_M=0.40 -> lethal 0.25 m (set after import, read now)",
      close(rp.derived().lethal_clearance_m, 0.25), f"{rp.derived().lethal_clearance_m:.4f}")
check("... the MOSS constant itself is untouched (0.34 m)", rp.MOSS.robot_width_m == 0.34)
os.environ["MOSS_BODY_BOX_M"] = "0.25 0.20"
os.environ["MOSS_SCAN_MASK_DEG"] = "350 10, 170 190"
os.environ["MOSS_CAMERA_XYZ_M"] = "0.16,0,0.08"
p = rp.current()
check("MOSS_BODY_BOX_M='0.25 0.20' -> (0.25, 0.20)", p.body_box_m == (0.25, 0.20))
check("MOSS_SCAN_MASK_DEG='350 10, 170 190' -> two sectors", p.scan_mask_sectors_deg == ((350.0, 10.0), (170.0, 190.0)))
check("MOSS_CAMERA_XYZ_M='0.16,0,0.08' -> (0.16, 0.0, 0.08)", p.camera_xyz_base_m == (0.16, 0.0, 0.08))
os.environ["MOSS_SCAN_MASK_DEG"] = "-30 30"
check("MOSS_SCAN_MASK_DEG='-30 30' -> (330, 30): wrapped into 0..360",
      rp.current().scan_mask_sectors_deg == ((330.0, 30.0),))
os.environ["MOSS_LIDAR_HEIGHT_M"] = "twenty"
os.environ["MOSS_BODY_CLEAR_M"] = "0.3"
os.environ["MOSS_SCAN_MASK_DEG"] = "350"
p = rp.current()
check("MOSS_LIDAR_HEIGHT_M=twenty -> placeholder 0.20 kept", p.lidar_height_m == 0.20)
check("MOSS_BODY_CLEAR_M with one number -> placeholder kept", p.body_clear_m == (0.20, 0.16))
check("MOSS_SCAN_MASK_DEG with an odd count -> no sector (placeholder)", p.scan_mask_sectors_deg == ())
os.environ["MOSS_LIDAR_HEIGHT_M"] = "nan"
check("MOSS_LIDAR_HEIGHT_M=nan -> placeholder kept (finite numbers only)", rp.current().lidar_height_m == 0.20)
os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.215"
check("MOSS_LIDAR_HEIGHT_M=0.215 -> 0.215 m", rp.current().lidar_height_m == 0.215)
os.environ["MOSS_MAX_SPEED_MPS"] = "0.5"
check("MOSS_MAX_SPEED_MPS=0.5 -> a 0.50 m/s ceiling, read now", rp.current().max_speed_mps == 0.5)
os.environ["MOSS_MAX_SPEED_MPS"] = "fast"
check("MOSS_MAX_SPEED_MPS=fast -> placeholder 0.45 kept", rp.current().max_speed_mps == 0.45)
os.environ.pop("MOSS_MAX_SPEED_MPS", None)
check("an explicit env dict wins over os.environ",
      rp.current({"MOSS_LIDAR_HEIGHT_M": "0.30"}).lidar_height_m == 0.30)
clear_env()
check("env cleared -> the plain MOSS placeholders again", rp.current() == M)
logging.getLogger("moss_dimos.robot_profile").removeHandler(cap)


# ── C. devices and state ─────────────────────────────────────────────────
print("C. devices (by-id) and the state dir")
QTPY = "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00"
C1 = "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0000-if00-port0"
OTHER_UART = "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
saved_root = rp.BYID_ROOT
with tempfile.TemporaryDirectory() as d:
    rp.BYID_ROOT = d     # read at call time: patching the module variable is enough
    check("nothing plugged -> both sentinels",
          rp.moss_qtpy_port({}) == rp.MOSS_QTPY_MISSING and rp.moss_lidar_port({}) == rp.MOSS_LIDAR_MISSING,
          f"{rp.moss_qtpy_port({})}, {rp.moss_lidar_port({})}")
    open(os.path.join(d, OTHER_UART), "w").close()
    check("a CP2102 (no N) bridge is never taken for the lidar", rp.moss_lidar_port({}) == rp.MOSS_LIDAR_MISSING)
    open(os.path.join(d, C1), "w").close()
    check("the CP2102N bridge -> the lidar, by-id path kept", rp.moss_lidar_port({}) == os.path.join(d, C1))
    check("... and never for the motor board", rp.moss_qtpy_port({}) == rp.MOSS_QTPY_MISSING)
    open(os.path.join(d, QTPY), "w").close()
    check("the QT Py -> the motor board", rp.moss_qtpy_port({}) == os.path.join(d, QTPY))
    check("MOSS_PORT / MOSS_LIDAR_PORT win over the by-id scan",
          rp.moss_qtpy_port({"MOSS_PORT": "/dev/ttyACM9"}) == "/dev/ttyACM9"
          and rp.moss_lidar_port({"MOSS_LIDAR_PORT": "/dev/ttyUSB7"}) == "/dev/ttyUSB7")
    check("a blank MOSS_PORT falls back to the by-id scan", rp.moss_qtpy_port({"MOSS_PORT": "  "}) == os.path.join(d, QTPY))
    check("resolve_by_id with an explicit root and no match -> the sentinel given",
          rp.resolve_by_id("*nothing*", "/dev/null/x", root=d) == "/dev/null/x")
rp.BYID_ROOT = saved_root
check("the sentinels cannot be opened (no such path)",
      not os.path.exists(rp.MOSS_QTPY_MISSING) and not os.path.exists(rp.MOSS_LIDAR_MISSING))
check("state dir default ~/.local/state/moss, recordings/moss.db under it",
      rp.moss_recording_path({}) == Path.home() / ".local/state/moss/recordings/moss.db",
      str(rp.moss_recording_path({})))
rel = rp.moss_state_dir({"MOSS_STATE_DIR": "rel/state"})
check("a relative MOSS_STATE_DIR is made absolute (cwd-based)",
      rel.is_absolute() and rel == Path.cwd() / "rel/state", str(rel))
check("MOSS_STATE_DIR=/srv/moss -> /srv/moss/recordings/moss.db",
      rp.moss_recording_path({"MOSS_STATE_DIR": "/srv/moss"}) == Path("/srv/moss/recordings/moss.db"))

# env_on: MOSS_CAMERA / MOSS_WRIST / MOSS_RECORD / MOSS_RERUN / MOSS_MOCK_BUS,
# one parser for the blueprint, the adapter and the preflight
check("env_on: unset or blank -> the default given",
      rp.env_on("MOSS_X", True, {}) is True and rp.env_on("MOSS_X", False, {}) is False
      and rp.env_on("MOSS_X", True, {"MOSS_X": "  "}) is True and rp.env_on("MOSS_X", False, {"MOSS_X": ""}) is False)
check("env_on: the default is True when none is given", rp.env_on("MOSS_X", env={}) is True)
_offs = ["0", "false", "no", "off", " OFF ", "False", "NO", "\t0\n"]
check(f"env_on: {_offs} -> off (any case, spaces stripped)",
      all(rp.env_on("MOSS_X", True, {"MOSS_X": v}) is False for v in _offs))
_ons = ["1", "true", "yes", "on", "y", "2", "enabled", " On "]
check(f"env_on: {_ons} -> on (anything else)",
      all(rp.env_on("MOSS_X", False, {"MOSS_X": v}) is True for v in _ons))
os.environ["MOSS_CAMERA"] = "off"
check("env_on reads os.environ by default: MOSS_CAMERA=off -> off", rp.env_on("MOSS_CAMERA", True) is False)
clear_env()


# ── L. where the warnings go ─────────────────────────────────────────────
print("L. the warnings: stderr outside dimOS, the run's main.jsonl inside a dimOS process")
_PROBE = """
import sys
if sys.argv[1] == "dimos":
    try:
        import dimos.utils.logging_config  # loaded in every dimOS process (CLI and workers)
    except ImportError as exc:
        print("NODIMOS", exc)
        raise SystemExit(0)
from moss_dimos import robot_profile as rp
p = rp.current({"ROBOT_PROFILE": "rover9", "MOSS_LIDAR_HEIGHT_M": "0,25"})
print("HEIGHT", p.lidar_height_m)
print("DIMOS_LOADED", "dimos" in sys.modules)
"""
_WANT = ("ROBOT_PROFILE is ignored: this package only knows MOSS (got 'rover9')",
         "MOSS_LIDAR_HEIGHT_M='0,25' ignored (expected 1 number(s)) - placeholder kept")


def _probe(mode, run_dir):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith("MOSS_") and k not in ("ROBOT_PROFILE", "DIMOS_RUN_LOG_DIR")}
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["DIMOS_RUN_LOG_DIR"] = run_dir      # what a dimOS worker inherits
    return subprocess.run([sys.executable, "-c", _PROBE, mode], env=env,
                          capture_output=True, text=True, timeout=300)


with tempfile.TemporaryDirectory() as _run:
    _r = _probe("plain", _run)
    _ok = all(w in _r.stderr for w in _WANT) and "HEIGHT 0.2" in _r.stdout
    check("outside dimOS: both warnings on stderr, the placeholder kept (0.2 m)",
          _ok, "" if _ok else f"stdout {_r.stdout.strip()[-200:]!r} stderr {_r.stderr.strip()[-300:]!r}")
    _ok = "DIMOS_LOADED False" in _r.stdout and not os.path.exists(os.path.join(_run, "main.jsonl"))
    check("... and robot_profile never imports dimOS (pure: the preflight, the cockpit)",
          _ok, "" if _ok else f"stdout {_r.stdout.strip()[-200:]!r}")
with tempfile.TemporaryDirectory() as _run:
    _r = _probe("dimos", _run)
    if "NODIMOS" in _r.stdout:
        print(f"  -- SKIP the main.jsonl half: no dimOS here ({_r.stdout.strip()}): run this on the robot")
    else:
        _log_file = Path(_run) / "main.jsonl"
        _events = []
        if _log_file.exists():
            import json
            for _line in _log_file.read_text().splitlines():
                try:
                    _events.append(json.loads(_line).get("event", ""))
                except ValueError:
                    pass
        _ok = all(w in _events for w in _WANT)
        check("inside a dimOS process: both warnings land in the run's main.jsonl",
              _ok, "" if _ok else f"events {_events} stderr {_r.stderr.strip()[-300:]!r}")
        _ok = "HEIGHT 0.2" in _r.stdout
        check("... with the placeholder kept (0.2 m)", _ok, "" if _ok else f"stdout {_r.stdout.strip()[-200:]!r}")


# ── S. the speed guard: tools/speed_guard.py ─────────────────────────────
print("S. the speed guard: speed from the log's own timestamps, one interval over the profile's ceiling is a kill")
import importlib.util  # noqa: E402
import io  # noqa: E402
import json  # noqa: E402
from contextlib import redirect_stdout  # noqa: E402
from datetime import datetime, timezone  # noqa: E402

clear_env()
SG = EC = None
SKIP_GUARD = None
sys.path.insert(0, str(ROOT / "tools"))
try:
    import explore_ctl as EC  # noqa: E402
    import speed_guard as SG  # noqa: E402
except ImportError as exc:
    if importlib.util.find_spec("dimos_lcm") is None:
        SKIP_GUARD = f"no dimos_lcm here ({exc}): run this where dimOS is installed"
    else:
        check("tools/speed_guard.py imports", False, f"{type(exc).__name__}: {exc}")
if SKIP_GUARD:
    print(f"  -- SKIP S and T: {SKIP_GUARD}")

T0 = 1_800_000_000.25           # a known epoch, in seconds
STEP_S = 2.0                    # LidarOdometry's log_every_s: the cadence of the odom lines
ODOM_LOGGER = "/home/x/moss/moss_dimos/lidar_odometry.py"


def iso(t):
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def odom_line(n, t, x, y):
    """One line shaped like the robot's own: structlog jsonl, %+.3f positions."""
    return json.dumps({
        "event": f"lidar odom #{n}: x={x:+.3f} y={y:+.3f} yaw=+0.0deg (380 pts, 31.0 ms, prior gyro, "
                 "depth cloud 274 pts) | gyro integral x=+0.0 y=-1.1 z=-0.2deg | wheels none",
        "level": "info", "logger": ODOM_LOGGER, "timestamp": iso(t),
        "func_name": "handle_pointcloud", "lineno": 535,
    })


def straight_run(speed, steps=5, step_s=STEP_S):
    """Lines for a body going straight at `speed` m/s, one line every `step_s`."""
    return [odom_line(20 * i + 1, T0 + i * step_s, i * speed * step_s, 0.0) for i in range(steps + 1)]


def one_line(out) -> str:
    """Captured output on one line, for a check's detail."""
    text = out.getvalue() if hasattr(out, "getvalue") else str(out)
    return " / ".join(t for t in text.strip().splitlines() if t)


def lines_with(out: str, text: str) -> str:
    return " / ".join(t for t in out.splitlines() if text in t) or "(none)"


if SG is not None:
    check("the limit is the profile's max_speed_mps: 0.45 m/s", SG.LIMIT_MS == 0.45 == rp.MOSS.max_speed_mps,
          f"{SG.LIMIT_MS}")
    check("limit_ms() reads MOSS_MAX_SPEED_MPS (0.5 -> 0.50 m/s)", SG.limit_ms({"MOSS_MAX_SPEED_MPS": "0.5"}) == 0.5)

    # the ceiling against the speed the stack itself commands
    check("cruise = 0.55 m/s x the launcher's 0.7 = 0.385 m/s (MOSS_NERF_SPEED unset)",
          abs(SG.cruise_mps({}) - 0.385) < 1e-12, f"{SG.cruise_mps({}):.4f}")
    check("MOSS_NERF_SPEED=0.5 -> 0.275 m/s; 1.0 (and above, and junk -> the default) as dimOS applies it",
          abs(SG.cruise_mps({"MOSS_NERF_SPEED": "0.5"}) - 0.275) < 1e-12
          and SG.cruise_mps({"MOSS_NERF_SPEED": "1.0"}) == 0.55 and SG.cruise_mps({"MOSS_NERF_SPEED": "2"}) == 0.55
          and abs(SG.cruise_mps({"MOSS_NERF_SPEED": "fast"}) - 0.385) < 1e-12)
    check("the profile's 0.45 m/s sits above that cruise: armed", SG.ceiling_refusal(0.45, SG.cruise_mps({})) is None)
    why = SG.ceiling_refusal(0.30, SG.cruise_mps({}))
    check("a 0.30 m/s ceiling under the 0.385 m/s cruise is refused, and says what to raise",
          why is not None and "0.385" in why and "MOSS_MAX_SPEED_MPS" in why, str(why))
    check("a ceiling equal to the cruise is refused too",
          SG.ceiling_refusal(0.55, SG.cruise_mps({"MOSS_NERF_SPEED": "1"})) is not None)
    _fly = (ROOT / "tools" / "moss-fly.sh").read_text()
    _m = re.search(r'NERF_SPEED="\$\{MOSS_NERF_SPEED:-([0-9.]+)\}"', _fly)
    check("the cruise's default scale is tools/moss-fly.sh's own (NERF_SPEED default)",
          _m is not None and float(_m.group(1)) == SG.NERF_SPEED_DEFAULT,
          f"launcher {_m.group(1) if _m else None}, guard {SG.NERF_SPEED_DEFAULT}")
    _spec = importlib.util.find_spec("dimos")
    _lp = next((p for d in ("go2", ".") if (p := Path(_spec.origin).parent / "navigation" / d / "replanning_a_star" / "local_planner.py").is_file()), None) if _spec and _spec.origin else None  # main: navigation/go2/ (e5679069d)
    if _lp is None or not _lp.is_file():
        print("  -- SKIP the planner-speed check: no dimOS source here")
    else:
        _speed = None
        for _node in ast.walk(ast.parse(_lp.read_text())):
            if isinstance(_node, ast.ClassDef) and _node.name == "LocalPlanner":
                for _st in _node.body:
                    if (isinstance(_st, ast.AnnAssign) and getattr(_st.target, "id", "") == "_speed"
                            and isinstance(_st.value, ast.Constant)):
                        _speed = _st.value.value
        check("the cruise's 0.55 m/s is dimOS's own LocalPlanner._speed", _speed == SG.PLANNER_SPEED_MPS,
              f"dimOS {_speed}, guard {SG.PLANNER_SPEED_MPS}")

    # dating the lines
    sample = SG.parse_odom(odom_line(1, T0, 0.0, 0.0))
    check("a real-shaped odom line parses", sample is not None)
    check(f"its stamp comes back as {T0} s", sample is not None and abs(sample[0] - T0) < 1e-6,
          f"{sample[0]:.6f}" if sample else "")
    later = SG.parse_odom(odom_line(21, T0 + STEP_S, 1.0, 0.0))
    check("two lines 2 s apart -> 2.000 s of log time", abs((later[0] - sample[0]) - 2.0) < 1e-6,
          f"{later[0] - sample[0]:.6f} s")

    # runaway at 0.5 m/s
    guard = SG.SpeedGuard()
    speeds, kills = [], []
    for i, line in enumerate(straight_run(0.5)):
        v = guard.feed(line)
        if v is not None:
            speeds.append(v)
            if v > SG.LIMIT_MS:
                kills.append(i)
    check("6 lines -> 5 intervals (the first line closes none)", len(speeds) == 5, f"{len(speeds)}")
    check("every interval measures 0.50 m/s", all(abs(v - 0.5) < 1e-9 for v in speeds),
          ", ".join(f"{v:.3f}" for v in speeds))
    check("the SECOND line already breaches (1 interval = 1 kill, ~1 m of runaway)", kills[:1] == [1], f"{kills}")
    check("no strike counter: all 5 intervals breach", len(kills) == 5, f"{kills}")

    # inside the 0.45 m/s ceiling
    guard = SG.SpeedGuard()
    measured = [v for v in (guard.feed(line) for line in straight_run(0.25)) if v is not None]
    check("0.500 m per 2 s line -> every interval 0.25 m/s", len(measured) == 5 and all(abs(v - 0.25) < 1e-9 for v in measured))
    guard = SG.SpeedGuard()
    check("the planner's cruise, 0.385 m/s (0.770 m per line), stays silent",
          all(guard.breach(line) is None for line in straight_run(0.385)))
    guard = SG.SpeedGuard()
    check("0.44 m/s (0.880 m per line) stays silent", all(guard.breach(line) is None for line in straight_run(0.44)))
    guard = SG.SpeedGuard()
    check("0.46 m/s (0.920 m per line) kills", any(guard.breach(line) is not None for line in straight_run(0.46)))
    guard = SG.SpeedGuard(limit=0.50)
    check("with MOSS_MAX_SPEED_MPS=0.5 the same 0.46 m/s stays silent",
          all(guard.breach(line) is None for line in straight_run(0.46)))

    # geometry: the chord, sign-free
    guard = SG.SpeedGuard()
    guard.feed(odom_line(1, T0, 0.0, 0.0))
    v = guard.feed(odom_line(21, T0 + STEP_S, 0.6, 0.8))
    check("dx 0.600 m, dy 0.800 m in 2 s -> 0.50 m/s (the chord, not one axis)", abs(v - 0.5) < 1e-9, f"{v:.6f}")
    guard = SG.SpeedGuard()
    guard.feed(odom_line(1, T0, 5.0, -3.0))
    v = guard.feed(odom_line(21, T0 + STEP_S, 5.0, -3.5))
    check("0.500 m backwards on y in 2 s -> 0.25 m/s (sign-free)", abs(v - 0.25) < 1e-9, f"{v:.6f}")

    # the cadence: the log's own interval jitters around 2 s
    gaps = [1.699, 2.4, 3.071, 1.9, 2.0, 2.65, 1.75, 3.0, 2.2, 2.05]
    lines, t, x = [], T0, 0.0
    lines.append(odom_line(1, t, x, 0.0))
    for k, g in enumerate(gaps):
        t += g; x += 0.149 * g
        lines.append(odom_line(20 * (k + 1), t, x, 0.0))
    guard = SG.SpeedGuard()
    crawl = [v for v in (guard.feed(line) for line in lines) if v is not None]
    check("a 0.149 m/s crawl with gaps of 1.699 to 3.071 s: all 10 intervals measured",
          len(crawl) == 10 and all(abs(v - 0.149) < 1e-3 for v in crawl), f"{len(crawl)}: {[round(v, 3) for v in crawl]}")
    check("(a wall-clock window 0.02 < dt < 2.0 would keep only 3 of them)", sum(1 for g in gaps if 0.02 < g < 2.0) == 3)
    guard = SG.SpeedGuard()
    check("and the crawl never trips the 0.45 m/s ceiling", all(guard.breach(line) is None for line in lines))

    # the kill: exploration stop on the bus, then the stack down
    TMPS = tempfile.TemporaryDirectory()
    STOP_LOG = Path(TMPS.name) / "stack_stop.log"
    RECORDER = Path(TMPS.name) / "stack_stop.py"
    RECORDER.write_text("import os, sys\n"
                        "open(sys.argv[1], 'a').write(os.getcwd() + '\\n')\n"
                        "print('stack down')\n")
    published = []
    saved_pub, saved_stop = EC.publish, SG.STACK_STOP
    try:
        EC.publish = lambda cmd: published.append(cmd) or f"/{cmd}_explore_cmd#std_msgs.Bool"
        SG.STACK_STOP = [sys.executable, str(RECORDER), str(STOP_LOG)]
        out = io.StringIO()
        with redirect_stdout(out):
            SG.stop_all()
        ran = STOP_LOG.read_text().splitlines() if STOP_LOG.exists() else []
        check("a breach publishes exactly one 'stop' through explore_ctl (the run's bus)",
              published == ["stop"] and "STOP EXPLORE published on /stop_explore_cmd" in out.getvalue(), one_line(out))
        check("then runs the stack stop once, from the checkout root (tools/moss-fly.sh stop by default)",
              ran == [str(ROOT)] and "STACK STOP" in out.getvalue() and "exit 0: stack down" in out.getvalue(),
              f"{ran}")
        check("the default stack stop is tools/moss-fly.sh stop",
              saved_stop == ["tools/moss-fly.sh", "stop"] if "SPEED_GUARD_STACK_STOP" not in os.environ else True,
              f"{saved_stop}")

        def sick(cmd):
            raise RuntimeError("bus down")

        EC.publish = sick
        out = io.StringIO()
        with redirect_stdout(out):
            SG.stop_all()   # must not raise: the watchdog keeps watching
        ran = STOP_LOG.read_text().splitlines()
        check("a failing bus is logged, not fatal, and the stack still goes down",
              "STOP EXPLORE FAILED: bus down" in out.getvalue() and len(ran) == 2, one_line(out))
        SG.STACK_STOP = [str(Path(TMPS.name) / "no_such_launcher")]
        out = io.StringIO()
        with redirect_stdout(out):
            SG.stop_all()
        check("a missing launcher is logged, not fatal", "STACK STOP FAILED" in out.getvalue(), one_line(out)[-120:])
    finally:
        EC.publish, SG.STACK_STOP = saved_pub, saved_stop

    # junk invents nothing
    guard = SG.SpeedGuard()
    guard.feed(odom_line(1, T0, 0.0, 0.0))
    check("a non-odom line -> None", guard.feed('{"event": "BUMP received", "timestamp": "' + iso(T0 + 1) + '"}') is None)
    check("an odom line with no timestamp -> None (no wall clock fallback)",
          guard.feed("lidar odom #7: x=+9.000 y=+9.000 yaw=+0.0deg") is None)
    check("tail's '==> file <==' header -> None", guard.feed("==> /tmp/moss_fly_launch.log <==\n") is None)
    check("a line dated BEFORE the previous one -> None", guard.feed(odom_line(41, T0 - 5.0, 9.0, 9.0)) is None)
    guard = SG.SpeedGuard()
    guard.feed(odom_line(1, T0, 0.0, 0.0))
    check("a repeated line (same stamp, dt = 0) -> None, no division", guard.feed(odom_line(1, T0, 0.0, 0.0)) is None)
    guard = SG.SpeedGuard()
    lines = straight_run(0.5)
    lines.insert(3, '{"event": "costmap tick", "timestamp": "' + iso(T0 + 2.5) + '"}')
    check("junk between odom lines does not break the chain", sum(1 for line in lines if guard.breach(line) is not None) == 5)

    # standing down (fake clock, fake log tree, fake registry)
    LOGS = Path(TMPS.name) / "state" / "dimos" / "logs"
    RUNS = Path(TMPS.name) / "state" / "dimos" / "runs"
    LOGS.mkdir(parents=True); RUNS.mkdir(parents=True)
    GLOB, RGLOB = str(LOGS / "*"), str(RUNS / "*.json")

    class Clock:
        def __init__(self, t=1_000_000.0):
            self.t = t

        def __call__(self):
            return self.t

    def make_run(parent, name, lines=1, mtime=None):
        run = Path(parent) / name
        run.mkdir(parents=True, exist_ok=True)
        (run / "main.jsonl").write_text("".join(odom_line(i, T0 + i * STEP_S, 0.0, 0.0) + "\n" for i in range(lines)))
        if mtime:
            os.utime(run, (mtime, mtime))
        return run

    RUN_A = make_run(LOGS, "run-a-moss-dimos-moss-explore", lines=2, mtime=T0)
    clk = Clock()
    watch = SG.RunWatch(str(RUN_A), idle_s=30.0, glob_pat=GLOB, clock=clk, runs_glob=RGLOB)
    check("a fresh watchdog on the live run stays on watch", watch.reason() is None)
    clk.t += 10.0
    with open(RUN_A / "main.jsonl", "a") as f:
        f.write(odom_line(9, T0 + 20.0, 0.1, 0.0) + "\n")
    check("the log grew: still on watch, the idle clock restarts", watch.reason() is None)
    clk.t += 29.0
    check("29 s of silence: still on watch (limit 30 s)", watch.reason() is None)
    clk.t += 2.0
    why = watch.reason()
    check("31 s of silence: stands down, and says so in seconds",
          why is not None and "has not grown for 31 s (limit 30 s)" in why, str(why))
    clk = Clock()
    edge = SG.RunWatch(str(RUN_A), idle_s=30.0, glob_pat=GLOB, clock=clk, runs_glob=RGLOB)
    clk.t += 29.999
    check("29.999 s: not yet", edge.reason() is None)
    clk.t += 0.001
    check("exactly 30.000 s: retired", edge.reason() is not None)
    orphan = SG.RunWatch(str(RUN_A), idle_s=1e6, glob_pat=GLOB, clock=Clock(), runs_glob=RGLOB)
    check("still the newest run: on watch", orphan.reason() is None)
    RUN_B = make_run(LOGS, "run-b-moss-dimos-moss-explore", lines=1, mtime=T0 + 3600)
    why = orphan.reason()
    check("a newer run dir retires the orphan at once (idle limit not even reached)",
          why is not None and RUN_B.name in why and "newest run" in why, str(why))
    live = SG.RunWatch(str(RUN_B), idle_s=1e6, glob_pat=GLOB, clock=Clock(), runs_glob=RGLOB)
    check("the watchdog of the NEWEST run keeps watching", live.reason() is None)
    # a dimOS from a git clone logs under the clone: only the registry names the dir
    CLONE = Path(TMPS.name) / "clone" / "logs"
    RUN_C = make_run(CLONE, "run-c-moss-dimos-moss-explore", lines=1, mtime=T0 + 7200)
    check("a run dir outside the log root is not seen without its registry entry",
          str(RUN_C.resolve()) not in SG.run_dirs(GLOB, RGLOB))
    (RUNS / "run-c.json").write_text(json.dumps({"pid": 1, "blueprint": "moss_dimos.moss-explore", "log_dir": str(RUN_C)}))
    (RUNS / "half-written.json").write_text('{"pid": 1, "log_')
    check("with its registry entry it is the newest (a half-written entry is skipped)",
          SG.run_dirs(GLOB, RGLOB)[-1] == str(RUN_C.resolve()), f"{SG.run_dirs(GLOB, RGLOB)[-1:]}")
    why = live.reason()
    check("so a run started from a clone retires the previous watchdog too",
          why is not None and RUN_C.name in why, str(why))
    (RUNS / "run-c.json").unlink()
    mine = SG.RunWatch(str(RUN_C), idle_s=1e6, glob_pat=GLOB, clock=Clock(), runs_glob=RGLOB)
    check("a watchdog given its run dir (SPEED_GUARD_RUN_DIR) whose registry entry is gone does not retire "
          "for an OLDER dir", mine.reason() is None)
    CRASHED = LOGS / "run-d-moss-dimos-moss-explore"     # newest run, but the stack died before writing
    CRASHED.mkdir()
    os.utime(CRASHED, (T0 + 9000, T0 + 9000))
    clk = Clock()
    dead = SG.RunWatch(str(CRASHED), idle_s=30.0, glob_pat=GLOB, clock=clk, runs_glob=RGLOB)
    check("a newest run dir with no main.jsonl: on watch while the limit runs", dead.reason() is None)
    clk.t += 30.0
    why = dead.reason()
    check("... and retired by the same silence, not left tailing nothing",
          why is not None and "has not grown for 30 s" in why, str(why))
    check("defaults: 30 s idle, /tmp/moss_speed_guard.pid",
          (SG.IDLE_S, SG.PID_FILE) == (30.0, "/tmp/moss_speed_guard.pid")
          if not (os.environ.get("SPEED_GUARD_IDLE_S") or os.environ.get("SPEED_GUARD_PID_FILE")) else True,
          f"{SG.IDLE_S} s, {SG.PID_FILE}")

    # the pid file the launcher kills by
    PID_PATH = str(Path(TMPS.name) / "guard.pid")
    SG.write_pid_file(PID_PATH)
    check("write_pid_file stamps our own pid", Path(PID_PATH).read_text().strip() == str(os.getpid()))
    Path(PID_PATH).write_text("999999\n")
    SG.clear_pid_file(PID_PATH)
    check("clear_pid_file leaves a file that names ANOTHER watchdog", Path(PID_PATH).exists())
    Path(PID_PATH).write_text(f"{os.getpid()}\n")
    SG.clear_pid_file(PID_PATH)
    check("and removes it when it names us", not Path(PID_PATH).exists())
    SG.clear_pid_file(PID_PATH)
    check("clearing a pid file that is already gone is harmless", not Path(PID_PATH).exists())

    # the real script
    def guard_env(home, **extra):
        env = {k: v for k, v in os.environ.items()
               if not k.startswith(("MOSS_", "SPEED_GUARD_")) and k != "XDG_STATE_HOME"}
        env.update({"HOME": str(home), "PYTHONPATH": str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""),
                    "PYTHONDONTWRITEBYTECODE": "1", "SPEED_GUARD_PID_FILE": str(Path(home) / "guard.pid"),
                    "SPEED_GUARD_STACK_STOP": f"{sys.executable} {RECORDER} {Path(home) / 'stack_stop.log'}"})
        env.update({k: str(v) for k, v in extra.items()})
        return env

    def run_guard(home, timeout_s=30, after_start=None, **extra):
        """Launch tools/speed_guard.py for real against a fake HOME; returns
        (returncode, seconds, stdout, stamped pid, pid, pid file)."""
        env = guard_env(home, **extra)
        pid_file = env["SPEED_GUARD_PID_FILE"]
        t0 = time.monotonic()
        proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "speed_guard.py")],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, cwd=str(ROOT))
        stamped = None
        while time.monotonic() - t0 < timeout_s and proc.poll() is None:
            if Path(pid_file).exists():
                stamped = Path(pid_file).read_text().strip()
                break
            time.sleep(0.05)
        if after_start:
            after_start()
        try:
            out = proc.communicate(timeout=timeout_s)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            out = proc.communicate()[0]
        return proc.returncode, time.monotonic() - t0, out, stamped, proc.pid, pid_file

    HOME1 = Path(TMPS.name) / "orphan_home"
    LOGS1 = HOME1 / ".local" / "state" / "dimos" / "logs" / "run-a-moss-dimos-moss-explore"
    LOGS1.mkdir(parents=True)
    (LOGS1 / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")   # written once, never again
    rc, took, out, stamped, pid, pid_file = run_guard(HOME1, SPEED_GUARD_IDLE_S=1.0, SPEED_GUARD_POLL_S=0.2)
    check("the real script stamped its own pid while running", stamped == str(pid), f"{stamped} vs {pid}")
    check("it exits by itself on a dead log, rc 0", rc == 0, f"rc {rc} after {took:.1f} s")
    check("in seconds (idle limit 1 s here, 30 s on the robot)", took < 35.0, f"{took:.1f} s")
    check("with a clear message naming the silent log", "standing down" in out and "has not grown" in out,
          out.strip().splitlines()[-1] if out.strip() else "(no output)")
    check("and it watched with the profile's 0.45 m/s limit, the 0.385 m/s cruise said beside it",
          "(limit 0.45 m/s, cruise 0.385 m/s commanded" in out, out.strip().splitlines()[0] if out.strip() else "")
    check("the pid file is cleaned up behind it", not Path(pid_file).exists())

    HOME2 = Path(TMPS.name) / "second_run_home"
    LOGS2 = HOME2 / ".local" / "state" / "dimos" / "logs"
    (LOGS2 / "run-a-moss-dimos-moss-explore").mkdir(parents=True)
    (LOGS2 / "run-a-moss-dimos-moss-explore" / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
    NEXT_RUN = LOGS2 / "run-b-moss-dimos-moss-explore"

    def start_second_run():
        NEXT_RUN.mkdir()
        (NEXT_RUN / "main.jsonl").write_text(odom_line(1, T0 + 3600, 0.0, 0.0) + "\n")

    rc, took, out, stamped, pid, pid_file = run_guard(HOME2, after_start=start_second_run,
                                                      SPEED_GUARD_IDLE_S=1e6, SPEED_GUARD_POLL_S=0.2)
    check("the previous run's watchdog retires when the next run starts", rc == 0, f"rc {rc} after {took:.1f} s")
    check("it names the newer run (idle limit 1e6 s: it cannot be the silence)",
          NEXT_RUN.name in out and "newest run" in out, out.strip().splitlines()[-1] if out.strip() else "(no output)")

    # a runaway on a live log, end to end: the bus refused (not LCM here), the stack still stopped
    HOME3 = Path(TMPS.name) / "runaway_home"
    RUN3 = HOME3 / ".local" / "state" / "dimos" / "logs" / "run-a-moss-dimos-moss-explore"
    RUN3.mkdir(parents=True)
    (RUN3 / "main.jsonl").write_text("")

    def runaway():
        time.sleep(0.5)                               # the tail is up
        with open(RUN3 / "main.jsonl", "a") as f:
            for line in straight_run(0.5, steps=1):   # 1.000 m in 2 s of log time
                f.write(line + "\n")
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not (HOME3 / "stack_stop.log").exists():
            time.sleep(0.05)
        time.sleep(0.3)

    rc, took, out, stamped, pid, pid_file = run_guard(HOME3, after_start=runaway, SPEED_GUARD_IDLE_S=2.0,
                                                      SPEED_GUARD_POLL_S=0.2, SPEED_GUARD_RUN_DIR=RUN3,
                                                      DIMOS_TRANSPORT="zenoh")
    stopped = (HOME3 / "stack_stop.log").read_text().splitlines() if (HOME3 / "stack_stop.log").exists() else []
    check("a 0.50 m/s interval in the run's log -> the real watchdog says so",
          "0.50 m/s over the 0.45 m/s envelope" in out, lines_with(out, "envelope"))
    check("its bus stop is refused off LCM and logged, not fatal",
          "STOP EXPLORE FAILED" in out and "LCM only" in out, lines_with(out, "STOP EXPLORE")[:160])
    check("and the stack stop ran, once, from the checkout root", stopped == [str(ROOT)], f"{stopped}")

    HOME4 = Path(TMPS.name) / "killed_home"
    RUN4 = HOME4 / ".local" / "state" / "dimos" / "logs" / "run-a-moss-dimos-moss-explore"
    RUN4.mkdir(parents=True)
    (RUN4 / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
    env4 = guard_env(HOME4, SPEED_GUARD_IDLE_S="1e6", SPEED_GUARD_POLL_S="0.2", MOSS_MAX_SPEED_MPS="0.5")
    killed = subprocess.Popen([sys.executable, str(ROOT / "tools" / "speed_guard.py")],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env4, cwd=str(ROOT))
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and not Path(env4["SPEED_GUARD_PID_FILE"]).exists():
        time.sleep(0.05)
    stamped = Path(env4["SPEED_GUARD_PID_FILE"]).read_text().strip() if Path(env4["SPEED_GUARD_PID_FILE"]).exists() else None
    if stamped:
        os.kill(int(stamped), 15)      # exactly what the arm script does: kill the pid in the file
    try:
        out4 = killed.communicate(timeout=15)[0]
        dead_ok = True
    except subprocess.TimeoutExpired:
        killed.kill()
        out4 = killed.communicate()[0]
        dead_ok = False
    check("killed by the pid in its file, it dies, rc 0", dead_ok and killed.returncode == 0, f"rc {killed.returncode}")
    check("and takes its pid file with it", not Path(env4["SPEED_GUARD_PID_FILE"]).exists())
    check("MOSS_MAX_SPEED_MPS=0.5 in its environment -> it watched with 0.5 m/s", "(limit 0.5 m/s" in out4,
          out4.strip().splitlines()[0] if out4.strip() else "(no output)")

    # a ceiling the stack's own cruise would cross: the real script refuses to watch
    HOME5 = Path(TMPS.name) / "low_ceiling_home"
    RUN5 = HOME5 / ".local" / "state" / "dimos" / "logs" / "run-a-moss-dimos-moss-explore"
    RUN5.mkdir(parents=True)
    (RUN5 / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
    rc, took, out, stamped, pid, pid_file = run_guard(HOME5, timeout_s=20, MOSS_MAX_SPEED_MPS="0.30")
    check("MOSS_MAX_SPEED_MPS=0.30 (under the 0.385 m/s cruise): exit 1, no pid file ever, the reason said",
          rc == 1 and stamped is None and not Path(pid_file).exists() and "not above the planner's cruise 0.385" in out,
          out.strip()[-160:])
    rc, took, out, stamped, pid, pid_file = run_guard(HOME5, timeout_s=20, MOSS_NERF_SPEED="1.0")
    check("MOSS_NERF_SPEED=1.0 (a 0.55 m/s cruise) under the 0.45 m/s ceiling: refused the same way",
          rc == 1 and stamped is None and "cruise 0.550" in out, out.strip()[-160:])


# ── T. arming: tools/arm_speed_guard.sh ──────────────────────────────────
if SG is not None:
    print("T. tools/arm_speed_guard.sh REPLACES the previous run's watchdog, by pid, and proves the new one")
    ARM = ROOT / "tools" / "arm_speed_guard.sh"
    CLK_TCK = os.sysconf("SC_CLK_TCK")
    SPAWNED = []

    def uptime_s() -> int:
        return int(float(Path("/proc/uptime").read_text().split()[0]))

    def born_s(pid) -> int:
        raw = Path(f"/proc/{pid}/stat").read_text()
        return int(raw.split(") ", 1)[1].split()[19]) // CLK_TCK

    def alive(pid) -> bool:
        try:
            raw = Path(f"/proc/{int(pid)}/stat").read_text()
        except (OSError, ValueError, TypeError):
            return False
        return raw.split(") ", 1)[1].split()[0] != "Z"

    def wait_dead(pid, timeout_s=5.0) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and alive(pid):
            time.sleep(0.05)
        return not alive(pid)

    TMPT = tempfile.TemporaryDirectory()
    FAKE_DIR = Path(TMPT.name) / "fake"
    FAKE_DIR.mkdir()
    FAKE = FAKE_DIR / "speed_guard.py"      # same name: /proc/<pid>/cmdline says speed_guard.py
    FAKE.write_text("import os, time\n"
                    "open(os.environ['SPEED_GUARD_PID_FILE'], 'w').write(f'{os.getpid()}\\n')\n"
                    "time.sleep(600)\n")
    DEAD = FAKE_DIR / "dies_at_once.py"
    DEAD.write_text("import sys\nsys.exit('no dimos run logs found')\n")
    LIAR = FAKE_DIR / "stamps_an_old_pid.py"
    LIAR.write_text("import os\nopen(os.environ['SPEED_GUARD_PID_FILE'], 'w').write(os.environ['OLD_PID'] + '\\n')\n")
    INNOCENT = FAKE_DIR / "innocent_worker.py"
    INNOCENT.write_text("import time\ntime.sleep(600)\n")

    def spawn(script, env_extra=None):
        p = subprocess.Popen([sys.executable, str(script)], env={**os.environ, **(env_extra or {})},
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        SPAWNED.append(p)
        return p

    def run_arm(pid_file, guard_py=None, extra=None, args=(), timeout_s=60):
        env = {k: v for k, v in os.environ.items() if not k.startswith(("MOSS_", "SPEED_GUARD_"))}
        env.update({"SPEED_GUARD_PID_FILE": str(pid_file), "SPEED_GUARD_PYTHON": sys.executable,
                    "SPEED_GUARD_LOG": str(Path(pid_file).with_suffix(".log")), "SPEED_GUARD_ARM_DEADLINE_S": "20",
                    "PYTHONPATH": str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""),
                    "PYTHONDONTWRITEBYTECODE": "1"})
        if guard_py:
            env["SPEED_GUARD_PY"] = str(guard_py)
        env.update(extra or {})
        mark = uptime_s()
        t0 = time.monotonic()
        r = subprocess.run(["bash", str(ARM), *args], env=env, cwd=str(ROOT), text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout_s)
        return r.returncode, r.stdout, time.monotonic() - t0, mark

    def wait_pid_file(path, timeout_s=20.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            try:
                return int(Path(path).read_text().strip())
            except (OSError, ValueError):
                time.sleep(0.05)
        return None

    def first(out):
        return out.strip().splitlines()[0] if out.strip() else "(silence)"

    def last(out):
        return out.strip().splitlines()[-1] if out.strip() else "(silence)"

    check("the arm script parses", subprocess.run(["bash", "-n", str(ARM)]).returncode == 0)
    PID_A = Path(TMPT.name) / "a.pid"
    spawn(FAKE, {"SPEED_GUARD_PID_FILE": str(PID_A)})
    OLD_A = wait_pid_file(PID_A)
    OLD_BORN = born_s(OLD_A)
    time.sleep(1.2)                      # so the two births land in different seconds
    rc, out, took, mark = run_arm(PID_A, guard_py=FAKE)
    NEW_A = int(PID_A.read_text().strip())
    check("the previous run's watchdog alive in the pid file: arming exits 0", rc == 0, f"rc {rc}")
    check("the previous watchdog is DEAD, and the script says so", wait_dead(OLD_A) and "retired" in out, first(out))
    check("the pid file names a DIFFERENT, live pid", NEW_A != OLD_A and alive(NEW_A), f"{OLD_A} -> {NEW_A}")
    check("born AFTER the launch mark, the retired one BEFORE it", born_s(NEW_A) >= mark > OLD_BORN,
          f"new born {born_s(NEW_A)} s, mark {mark} s, old born {OLD_BORN} s (since boot)")
    check("it announces the armed pid and the profile's ceiling",
          f"pid {NEW_A}" in out and "armed (stops the run beyond 0.45 m/s)" in out, last(out))
    check("arming takes seconds", took < 10.0, f"{took:.1f} s")

    PID_B = Path(TMPT.name) / "b.pid"
    rc, out, took, mark = run_arm(PID_B, guard_py=FAKE)
    NEW_B = int(PID_B.read_text().strip())
    check("no pid file at all (first run after a boot): armed, rc 0, born after the mark",
          rc == 0 and alive(NEW_B) and born_s(NEW_B) >= mark and "retired" not in out, f"rc {rc}, pid {NEW_B}")

    DEAD_PID = subprocess.Popen([sys.executable, "-c", "pass"])
    DEAD_PID.wait()
    PID_C = Path(TMPT.name) / "c.pid"
    PID_C.write_text(f"{DEAD_PID.pid}\n")
    rc, out, took, mark = run_arm(PID_C, guard_py=FAKE)
    NEW_C = int(PID_C.read_text().strip())
    check("a stale pid file (its pid is gone): named stale, armed anyway",
          rc == 0 and "stale pid file" in out and alive(NEW_C) and NEW_C != DEAD_PID.pid, first(out))

    PID_D = Path(TMPT.name) / "d.pid"
    victim = spawn(INNOCENT)
    PID_D.write_text(f"{victim.pid}\n")
    rc, out, took, mark = run_arm(PID_D, guard_py=FAKE)
    NEW_D = int(PID_D.read_text().strip())
    check("a pid file naming a live process that is NOT a watchdog: it is left alone",
          rc == 0 and alive(victim.pid) and "not a speed guard" in out, first(out))
    check("and a watchdog is armed beside it", alive(NEW_D) and NEW_D != victim.pid and born_s(NEW_D) >= mark)

    PID_E = Path(TMPT.name) / "e.pid"
    rc, out, took, mark = run_arm(PID_E, guard_py=DEAD, extra={"SPEED_GUARD_ARM_DEADLINE_S": "2"})
    check("a watchdog that never comes up -> rc 1, the reason on screen, never 'armed'",
          rc == 1 and "DID NOT COME UP" in out and "armed" not in out, out.strip()[-160:])
    check("it refuses within its deadline", took < 8.0, f"{took:.1f} s")
    PID_E2 = Path(TMPT.name) / "e2.pid"
    elder = spawn(INNOCENT)
    time.sleep(1.2)
    rc, out, took, mark = run_arm(PID_E2, guard_py=LIAR, extra={"SPEED_GUARD_ARM_DEADLINE_S": "2", "OLD_PID": str(elder.pid)})
    check("a pid file naming a LIVE process born BEFORE the launch -> rc 1 (liveness alone proves nothing)",
          rc == 1 and born_s(elder.pid) < mark and alive(elder.pid), f"rc {rc}")
    rc, out, took, mark = run_arm(Path(TMPT.name) / "f0.pid", guard_py=FAKE, args=(str(Path(TMPT.name) / "no_such_run"),))
    check("a RUN_DIR that is not a directory -> rc 1, nothing launched", rc == 1 and "IS NOT A DIRECTORY" in out, last(out))

    HOME_F = Path(TMPT.name) / "rover_home"
    LOGS_F = HOME_F / ".local" / "state" / "dimos" / "logs"
    RUN_OLD = LOGS_F / "run-a-moss-dimos-moss-explore"
    RUN_NEW = LOGS_F / "run-b-moss-dimos-moss-explore"
    for d in (RUN_OLD, RUN_NEW):
        d.mkdir(parents=True)
        (d / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
    os.utime(RUN_OLD, (T0, T0))
    os.utime(RUN_NEW, (T0 + 60, T0 + 60))
    PID_F = Path(TMPT.name) / "f.pid"
    env_f = {"HOME": str(HOME_F), "SPEED_GUARD_IDLE_S": "1e6", "SPEED_GUARD_POLL_S": "0.5"}
    rc, out, took, mark = run_arm(PID_F, extra=env_f, args=(str(RUN_NEW),))
    FIRST = int(PID_F.read_text().strip()) if PID_F.exists() else None
    check("the REAL watchdog arms with the run dir given, rc 0", rc == 0 and FIRST is not None and alive(FIRST),
          f"rc {rc}, pid {FIRST}")
    guard_log = Path(str(PID_F.with_suffix(".log")))
    for _ in range(40):
        if "watching" in guard_log.read_text():
            break
        time.sleep(0.1)
    check("and it tails THAT run's log", str(RUN_NEW / "main.jsonl") in guard_log.read_text(), first(guard_log.read_text()))
    time.sleep(1.2)
    rc, out, took, mark = run_arm(PID_F, extra=env_f, args=(str(RUN_NEW),))
    SECOND = int(PID_F.read_text().strip()) if PID_F.exists() else None
    check("armed again: rc 0 and the PID CHANGED (the arm replaced it)",
          rc == 0 and SECOND is not None and SECOND != FIRST, f"{FIRST} -> {SECOND}")
    check("the first real watchdog is dead, the second born after the second mark",
          wait_dead(FIRST) and SECOND is not None and born_s(SECOND) >= mark)
    if SECOND:
        os.kill(SECOND, 15)
        wait_dead(SECOND)
    PID_G = Path(TMPT.name) / "g.pid"
    rc, out, took, mark = run_arm(PID_G, extra={**env_f, "MOSS_MAX_SPEED_MPS": "0.30",
                                                "SPEED_GUARD_ARM_DEADLINE_S": "3"}, args=(str(RUN_NEW),))
    check("the REAL watchdog under a 0.30 m/s ceiling (below the cruise): arming fails, rc 1, the reason on screen",
          rc == 1 and "DID NOT COME UP" in out and "not above the planner's cruise" in out and "armed (" not in out,
          out.strip()[-200:])

    for p in SPAWNED:
        p.kill()
    for p in SPAWNED:
        p.wait()
    for leftover in (PID_A, PID_B, PID_C, PID_D):
        try:
            os.kill(int(leftover.read_text().strip()), 9)
        except (OSError, ValueError):
            pass
clear_env()


# ── D/E need the modules: real dimOS, or a stand-in for every dimOS name ──

class _Stub:
    """Any dimOS class / function / decorator: constructible, subscriptable,
    subclassable; never called by the functions under test."""

    def __init__(self, *args, **kwargs):
        pass

    def __class_getitem__(cls, item):
        return cls

    def __call__(self, *args, **kwargs):
        return args[0] if len(args) == 1 and callable(args[0]) else self

    def __getattr__(self, name):
        return lambda *a, **k: None


class _StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = type(name, (_Stub,), {})
        setattr(self, name, cls)
        return cls


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def __init__(self, roots):
        self.roots = roots

    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in self.roots:
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        mod = _StubModule(spec.name)
        mod.__path__ = []
        return mod

    def exec_module(self, module):
        pass


def _install_stubs():
    """Stand-ins only for what does not import here; returns what was stubbed."""
    stubbed = []
    for root, probe in (("dimos", "dimos.core.module"), ("dimos_lcm", "dimos_lcm.std_msgs")):
        try:
            __import__(probe)
        except Exception:  # noqa: BLE001 - absent, or present without its deps
            for name in [n for n in sys.modules if n == root or n.startswith(root + ".")]:
                del sys.modules[name]
            stubbed.append(root)
    if stubbed:
        sys.meta_path.insert(0, _StubFinder(set(stubbed)))
    return stubbed


stubbed = _install_stubs()
if stubbed:
    print(f"  (stand-ins for {', '.join(stubbed)}: not importable here)")


def bare(cls, **kwargs):
    """An instance with the class's own __init__ run and dimOS's base
    __init__ skipped: the fields as the module lays them, no runtime."""
    base = cls.__mro__[1]
    saved = base.__dict__.get("__init__")
    base.__init__ = lambda self, *a, **k: None
    try:
        return cls(**kwargs)
    finally:
        if saved is None:
            del base.__init__
        else:
            base.__init__ = saved


print("D. rplidar_c1 reads the MOSS profile")
rpl = load("rplidar_c1")
if rpl is not None:
    x, y = rpl.polar_to_xy(90.0, 1000.0)
    check("polar_to_xy(90 deg, 1000 mm), yaw 0 -> (0, -1.0) m (SLAMTEC clockwise: y negated)",
          abs(x) < 1e-6 and abs(y + 1.0) < 1e-6, f"({x:.9f}, {y:.9f})")
    x, y = rpl.polar_to_xy(0.0, 2000.0)
    check("polar_to_xy(0 deg, 2000 mm) -> (2.0, 0) m: no mounting term", close(x, 2.0) and close(y, 0.0, 1e-9))
    os.environ["MOSS_LIDAR_YAW_DEG"] = "90"
    x, y = rpl.polar_to_xy(270.0, 1000.0)
    check("MOSS_LIDAR_YAW_DEG=90: 270 deg raw + 90 = the nose, (1.0, 0) m (read at call time)",
          close(x, 1.0, 1e-6) and abs(y) < 1e-6, f"({x:.6f}, {y:.6f})")
    os.environ.pop("MOSS_LIDAR_YAW_DEG", None)
    scan = [(15, 0.0, 300.0),     # 0.30 m dead ahead
            (15, 0.0, 250.0),     # 0.25 m dead ahead
            (15, 0.0, 200.0),     # 0.20 m dead ahead: inside MOSS's body box (0.22)
            (15, 90.0, 100.0),    # 0.10 m: under the 0.15 m floor everywhere
            (15, 180.0, 1500.0)]  # 1.5 m behind: a wall
    pts_m = rpl.scan_to_points(scan, 0)
    ahead = sorted(round(p[0], 3) for p in pts_m if abs(p[1]) < 1e-6 and p[0] > 0)
    check("the 0.30 m and 0.25 m returns ahead are kept", ahead == [0.25, 0.3], f"kept ahead {ahead}")
    check("the 0.20 m return (inside the 0.22 x 0.18 body box) and the 0.10 m one are dropped",
          len(pts_m) == 3, f"{len(pts_m)} points: {[(round(a, 3), round(b, 3)) for a, b, _ in pts_m]}")
    check("the 1.5 m wall behind -> (-1.5, 0) m", any(close(p[0], -1.5, 1e-6) and abs(p[1]) < 1e-6 for p in pts_m))
    check("an explicit profile argument gives the same 3 points",
          len(rpl.scan_to_points(scan, 0, profile=rp.MOSS)) == 3)
    os.environ["MOSS_BODY_BOX_M"] = "0.18 0.18"
    n_small = len(rpl.scan_to_points(scan, 0))
    os.environ.pop("MOSS_BODY_BOX_M", None)
    check("MOSS_BODY_BOX_M='0.18 0.18' -> the 0.20 m return is kept too (4 points, env read per scan)",
          n_small == 4, f"{n_small} points")
    os.environ["MOSS_SCAN_MASK_DEG"] = "348 12"
    n_mask = len(rpl.scan_to_points(scan, 0))
    os.environ.pop("MOSS_SCAN_MASK_DEG", None)
    check("MOSS_SCAN_MASK_DEG='348 12' (under 0.30 m) -> the 0.25 m return is dropped too (2 points)",
          n_mask == 2, f"{n_mask} points")
    check("an explicit mask wins over the profile (348..12 deg under 0.30 m drops the 0.25 m return)",
          len(rpl.scan_to_points(scan, 0, mask=((348.0, 12.0),), mask_range_m=0.30, profile=rp.MOSS)) == 2)
    src = (ROOT / "moss_dimos/rplidar_c1.py").read_text()
    check("scan_to_points reads the body box from the profile, no literal",
          "body_box_m" in src and "> 0.33 or" not in src)


print("E. costmap2d / recovering_planner / lidar_odometry read the profile")
cmod = load("costmap2d")
if cmod is not None:
    cloud = np.array([[1.0, 0.0, np.float32(0.37)], [2.0, 0.0, np.float32(0.20)],
                      [3.0, 0.0, 0.25]], dtype=np.float32)
    check("the z=0.20 point (float32) is the lidar return, 0.37 and 0.25 are not",
          list(cmod.lidar_returns(cloud)) == [False, True, False])
    os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.37"
    check("MOSS_LIDAR_HEIGHT_M=0.37 -> now the z=0.37 point is the lidar return (read at call time)",
          list(cmod.lidar_returns(cloud)) == [True, False, False])
    os.environ.pop("MOSS_LIDAR_HEIGHT_M", None)

    def cleared(hx=None, hy=None):
        """Cells body_clear marks at pose (0, 0, 0) on a 4 m grid; with hx, hy
        given, the reference formula with those literals instead
        (arange(-hx, hx + 1e-9, res / 2) x arange(-hy, hy + 1e-9, res / 2))."""
        g = cmod.ScoredGrid(resolution=0.05, span_m=4.0)
        if hx is None:
            g.body_clear((0.0, 0.0, 0.0))
            return g.seen
        lx = np.arange(-hx, hx + 1e-9, g.res / 2)
        ly = np.arange(-hy, hy + 1e-9, g.res / 2)
        bx, by = np.meshgrid(lx, ly)
        gx, gy = g.cell(bx.ravel(), by.ravel())
        seen = np.zeros_like(g.seen)
        seen[gy, gx] = True
        return seen

    moss_cells = cleared()
    check("body_clear = a 0.20 x 0.16 footprint, cell for cell",
          np.array_equal(moss_cells, cleared(0.20, 0.16)), f"{int(moss_cells.sum())} cells")
    os.environ["MOSS_BODY_CLEAR_M"] = "0.31 0.23"
    big_cells = cleared()
    os.environ.pop("MOSS_BODY_CLEAR_M", None)
    check("MOSS_BODY_CLEAR_M='0.31 0.23' -> a 0.31 x 0.23 footprint, cell for cell",
          np.array_equal(big_cells, cleared(0.31, 0.23)), f"{int(big_cells.sum())} cells")
    check("the default body clears less floor than the 0.31 x 0.23 override",
          0 < moss_cells.sum() < big_cells.sum(), f"{int(moss_cells.sum())} < {int(big_cells.sum())}")

    def _pose(x, y):
        return types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0),
                                     orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    def after_stop(handler, open_first=True):
        """The costmap as its __init__ lays it (no dimOS runtime): first odom
        at (0, 0), optionally the explore order, then `handler` (Bool True)
        when the module still has it, then the rover at (1.0, 0). Returns
        (cells written by that move, the 0.20 x 0.16 footprint at (1.0, 0))."""
        vc = bare(cmod.MossCostMap)
        asyncio.run(vc.handle_odom(_pose(0.0, 0.0)))
        if vc._grid is None:
            return None, None
        if open_first:
            asyncio.run(vc.handle_explore_cmd(types.SimpleNamespace(data=True)))
        if handler and hasattr(vc, handler):
            asyncio.run(getattr(vc, handler)(types.SimpleNamespace(data=True)))
        before = int(vc._grid.seen.sum())
        ref = copy.deepcopy(vc._grid)
        ref.body_clear((1.0, 0.0, 0.0))
        asyncio.run(vc.handle_odom(_pose(1.0, 0.0)))
        return int(vc._grid.seen.sum()) - before, int(ref.seen.sum()) - before

    if hasattr(cmod, "MossCostMap"):
        n, fp = after_stop(None)
        check("the grid is created on the first odom (no frame precondition)", n is not None)
        if n is not None:
            check("explore_cmd, then the rover at (1.0, 0) -> body_clear writes its 0.20 x 0.16 footprint",
                  n == fp and n > 0, f"{n} cells written, footprint {fp}")
            for h in ("handle_stop_explore_cmd", "handle_explore_done"):
                if not hasattr(cmod.MossCostMap, h):
                    check(f"MossCostMap.{h} exists", False)
                    continue
                n, fp = after_stop(h)
                check(f"{h.removeprefix('handle_')} -> the map keeps writing: the move writes the footprint",
                      n == fp and n > 0, f"{n} cells, footprint {fp}")
            n, fp = after_stop("handle_stop_explore_cmd", open_first=False)
            check("a stop BEFORE the map opened -> the 5 cm gate still opens it (a 1.0 m move writes)",
                  n is not None and n > 0, f"{n} cells")
    else:
        check("costmap2d defines MossCostMap", False, "class not found")

rpm = load("recovering_planner")
if rpm is not None:
    lth, pv = rpm.clearances()
    check("clearances() = (0.22, 0.25) m", close(lth, 0.22) and close(pv, 0.25), f"({lth:.4f}, {pv:.4f})")
    os.environ["MOSS_ROBOT_WIDTH_M"] = "0.40"
    check("MOSS_ROBOT_WIDTH_M=0.40 -> planner lethal 0.25 m", close(rpm.clearances()[0], 0.25))
    os.environ.pop("MOSS_ROBOT_WIDTH_M", None)

    # clearance_cost_map itself, as _find_wide_path calls it (no radii): one
    # obstacle cell on a 41 x 41 grid at 0.05 m, cells read along +x. The
    # dimOS grid types and the Voronoi field get minimal stand-ins (the
    # Voronoi term is not under test: a zero field keeps lethal = 100 exact).
    class _Grid:
        def __init__(self, grid, resolution, origin=None, frame_id="world", ts=0.0):
            self.grid, self.resolution, self.origin, self.frame_id, self.ts = grid, resolution, origin, frame_id, ts

    saved_pl = (rpm.OccupancyGrid, rpm.CostValues, rpm.voronoi_gradient)
    rpm.OccupancyGrid = _Grid
    rpm.CostValues = types.SimpleNamespace(FREE=0, OCCUPIED=100, UNKNOWN=-1)
    rpm.voronoi_gradient = lambda g, max_distance: _Grid(g.grid.copy(), g.resolution)
    try:
        raw = np.zeros((41, 41), dtype=np.int8)
        raw[20, 20] = 100
        binary = _Grid(raw, 0.05)

        def row(**kw):
            return [int(v) for v in rpm.clearance_cost_map(binary, **kw).grid[20, 24:27]]  # 0.20, 0.25, 0.30 m

        r = row()
        check("clearance_cost_map(): 0.20 m lethal, 0.25 m passable (lethal 0.22 m)",
              r[0] == 100 and r[1] < 100, f"costs at 0.20/0.25/0.30 m: {r}")
        r = row(lethal_m=0.30, pivot_m=0.39)
        check("explicit 0.30/0.39 radii -> 0.25 m lethal, 0.30 m passable (explicit wins)",
              r[1] == 100 and r[2] < 100, f"{r}")
    finally:
        rpm.OccupancyGrid, rpm.CostValues, rpm.voronoi_gradient = saved_pl

    # the soft STOP (stop_movement) vs a running contact escape
    class _Odom:
        position = types.SimpleNamespace(distance=lambda other: 0.0)   # never moves: only abort/timeout end it

    sent = []
    g = rpm.RecoveringGlobalPlanner.__new__(rpm.RecoveringGlobalPlanner)
    g._lock = threading.Lock()
    g._escape_abort = threading.Event()
    g._escaping, g._escape_id, g._held_goal = False, 0, None
    g._current_odom = _Odom()
    g._local_planner = types.SimpleNamespace(stop_planning=lambda: None,
                                             cmd_vel=types.SimpleNamespace(on_next=sent.append))
    g._position_tracker = types.SimpleNamespace(reset_data=lambda: None)
    g.escape_timeout_s = 3.0
    cancels, planned = [], []
    g.cancel_goal = lambda *a, **k: cancels.append(1)
    g.handle_goal_request = planned.append
    module = types.SimpleNamespace(_planner=g)

    def wait_idle(limit_s):
        t0 = time.monotonic()
        while g._escaping and time.monotonic() - t0 < limit_s:
            time.sleep(0.01)
        return time.monotonic() - t0

    check("bump -> escape starts", g.escape(-1.0, trigger="bump") is True)
    time.sleep(0.25)
    rpm.RecoveringGlobalPlanner.handle_goal_request(g, "goal-A")     # held: the escape owns the tracks
    n_before = len(sent)
    check("0.25 s into the escape: reversing (twists sent), a goal held",
          g._escaping and n_before >= 3 and g._held_goal == "goal-A", f"{n_before} twists")
    rpm.RecoveringPlanner._on_stop_movement(module, types.SimpleNamespace(data=True))
    dt = wait_idle(1.0)
    time.sleep(0.2)
    check("stop_movement -> the escape ends within 0.3 s (not its 3 s timeout)", not g._escaping and dt < 0.3,
          f"{dt:.2f} s")
    check("... at most one more reverse twist + the zero twist after the STOP, none 0.2 s later",
          len(sent) - n_before <= 2, f"{len(sent) - n_before} twists after STOP")
    check("... the held goal is dropped, never planned; the goal is cancelled",
          planned == [] and g._held_goal is None and len(cancels) >= 1, f"planned {planned}")
    sent.clear()
    check("a new contact after the STOP still escapes (abort flag cleared)", g.escape(-1.0, trigger="bump") is True)
    time.sleep(0.2)
    check("... and reverses", g._escaping and len(sent) >= 2, f"{len(sent)} twists")
    check("a second contact during an escape aborts it (returns False)",
          g.escape(+1.0, trigger="bump") is False)
    check("... and the escape stops", wait_idle(1.0) < 0.3 and not g._escaping)

lom = load("lidar_odometry")
if lom is not None:
    if hasattr(lom, "_profile"):
        check("lidar_odometry._profile(): MOSS at 0.20 m", lom._profile().lidar_height_m == 0.20)
        os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.215"
        check("MOSS_LIDAR_HEIGHT_M=0.215 -> _profile() at 0.215 m (read at call time)",
              lom._profile().lidar_height_m == 0.215)
        os.environ.pop("MOSS_LIDAR_HEIGHT_M", None)
    else:
        check("lidar_odometry reads the profile through _profile()", False, "no _profile()")

    check("LidarOdometry has its depth handler (handle_depth_image)", hasattr(lom.LidarOdometry, "handle_depth_image"))
    if hasattr(lom.LidarOdometry, "handle_depth_image"):
        # handle_depth_image on one pixel: depth 1.000 m at the principal point
        # (optical (0, 0, 1)), rover at (0, 0, 0). Base point = (cos p + cx, 0,
        # -sin p + cz): MOSS (pitch 0, cam 0.156/0/0.075) -> (1.156, 0, 0.075).
        def depth_point():
            me = types.SimpleNamespace(_depth_n=lom.DEPTH_EVERY - 1, _K=(600.0, 600.0, 0.0, 0.0),
                                       _searching=False, pose2d=(0.0, 0.0, 0.0), _pose_hist=[],
                                       camera_floor=types.SimpleNamespace(publish=lambda m: None),
                                       world_frame="world", _depth_pts_last=0, _pending_cam_pts=None)
            msg = types.SimpleNamespace(data=np.array([[1000]], dtype=np.uint16), ts=0.0)
            saved_pc = lom.PointCloud2
            lom.PointCloud2 = types.SimpleNamespace(from_numpy=lambda arr, **k: arr)
            try:
                asyncio.run(lom.LidarOdometry.handle_depth_image(me, msg))
            finally:
                lom.PointCloud2 = saved_pc
            return None if me._pending_cam_pts is None else [float(v) for v in me._pending_cam_pts[0][0]]

        pt = depth_point()
        check("depth projection: 1 m at the principal point -> (1.156, 0, 0.075) m",
              pt is not None and all(abs(a - b) < 1e-5 for a, b in zip(pt, (1.156, 0.0, 0.075))), f"{pt}")
        os.environ["MOSS_CAMERA_PITCH_DEG"] = "6.3"
        os.environ["MOSS_CAMERA_XYZ_M"] = "0.14 0 0.32"
        pt = depth_point()
        clear_env()
        vp = math.radians(6.3)
        want_pt = (math.cos(vp) + 0.14, 0.0, -math.sin(vp) + 0.32)
        check("MOSS_CAMERA_PITCH_DEG=6.3, MOSS_CAMERA_XYZ_M='0.14 0 0.32' -> (1.1340, 0, 0.2103) m",
              pt is not None and all(abs(a - b) < 1e-5 for a, b in zip(pt, want_pt)), f"{pt}")

    # the lidar lift (world cloud z and base_link->lidar_link tf) sits inside
    # handle_pointcloud next to kiss-icp, too heavy for a cold call: pin the
    # source instead - both use sites read the ONE profile value
    lo_src = (ROOT / "moss_dimos/lidar_odometry.py").read_text()
    seg = ast.get_source_segment(lo_src, next(n for n in ast.walk(ast.parse(lo_src))
                                              if isinstance(n, ast.AsyncFunctionDef) and n.name == "handle_pointcloud"))
    check("handle_pointcloud: lidar_h = _profile().lidar_height_m feeds world_pts z AND the lidar_link tf",
          "lidar_h = _profile().lidar_height_m" in seg and "np.array([x, y, lidar_h])" in seg
          and "Vector3(0.0, 0.0, lidar_h)" in seg)

clear_env()
print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
