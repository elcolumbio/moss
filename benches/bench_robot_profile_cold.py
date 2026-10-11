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
without it). D and E import the modules: with dimOS when it is there, else
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
        "camera_xyz_base_m", "camera_pitch_deg"}
check("RobotProfile holds exactly the 14 fields MOSS reads", fields == want,
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
