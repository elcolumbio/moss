"""Cold bench for moss_dimos.blueprints: devices, the import, the four
blueprints, the coordinator in a fresh worker.

Known input -> known output in physical units. Groups:

  A. devices      - moss_ports() on a fake /dev/serial/by-id: the QT Py and the
                    C1 resolved by glob, the sentinel when one is missing (never
                    another device, never /dev/ttyUSB0), MOSS_PORT /
                    MOSS_LIDAR_PORT win
  B. the import   - importing the module in a clean interpreter builds no
                    blueprint, loads no camera / lidar / pad module, sets no
                    environment variable, registers 'moss' with the exact
                    factory path and installs the LCM queue clamp
  C. state dir    - recordings at ~/.local/state/moss/recordings/moss.db,
                    MOSS_STATE_DIR moves it, always absolute
  D. telemetry    - telemetry_payload(adapter_like, now) -> the known document
                    (driver None when the build does not send it, the age in
                    seconds, the mock flag), None for an adapter without
                    telemetry; the real adapter on the fake firmware
                    -> 11.6 V, fw qtpy-mdd3a-v3, JSON round trip
  E. the camera   - the base_link -> camera_link mount from the profile
                    (pitch -> quaternion)
  H. wrist index  - wrist_camera_index() on a fake /dev/v4l/by-id
  F. entry points - pyproject: distribution moss-dimos, the four entry points
                    on the four lazy builders; then each one resolved through
                    dimOS's own `dimos run` lookup (get_by_name) from a
                    temporary dist-info built from pyproject.toml
  G. blueprints   - moss-base, moss-gamepad, moss-nav and moss-explore build:
                    module names, remaps, the pad's MOSS envelope, JPEG camera
                    transport, recorder path, explorer clearances from the MOSS
                    profile, no gamepad -> cmd_vel remap in explore; switches
                    MOSS_CAMERA=0 / MOSS_RECORD=0 / MOSS_WRIST=0 / MOSS_RERUN=1
                    (and off / no / false: robot_profile.env_on);
                    MossMemoryConfig defaults; base_telemetry on the coordinator
  I. the worker   - what a dimOS forkserver worker does: unpickle the
                    coordinator class in a clean interpreter -> the module is
                    imported, 'moss' is registered, the registry builds a
                    MossBaseAdapter from the factory string, the LCM clamp is
                    on, no other moss_dimos module came along. WristCamera
                    pickles by reference both ways. A conflicting 'moss'
                    registration is reported with both paths, not swallowed.

Needs dimOS (the module imports it). No hardware.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_moss_blueprints_cold.py
"""
import importlib
import json
import math
import os
import pickle
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

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


def finish():
    print()
    print(f"{OK} OK, {KO} KO")
    print("TEST PASSED" if KO == 0 else "TEST FAILED")
    sys.exit(1 if KO else 0)


CLEARED = [k for k in os.environ if k.startswith("MOSS_")] + ["ROBOT_PROFILE", "PERSISTENT_MAP"]
for var in CLEARED:
    os.environ.pop(var, None)

try:
    import dimos.core.coordination.blueprints  # noqa: F401
except Exception as exc:  # noqa: BLE001
    check("dimOS importable (moss_dimos.blueprints needs it: run this in the robot's venv)", False,
          f"{type(exc).__name__}: {exc}")
    finish()

from moss_dimos import robot_profile as rp  # noqa: E402
from moss_dimos import blueprints as mb  # noqa: E402

# Device names as /dev/serial/by-id spells them, with zeroed identifiers.
QTPY = "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00"
C1 = "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_" + "0" * 32 + "-if00-port0"
FACTORY = "moss_dimos.moss_adapter:MossBaseAdapter"


def child_env(**extra):
    """A clean environment for a fresh interpreter: no MOSS_* / ROBOT_PROFILE,
    this checkout first on PYTHONPATH (the rest kept: dimOS may come from it)."""
    env = {k: v for k, v in os.environ.items() if k not in CLEARED and not k.startswith("MOSS_")}
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", "")
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env.update(extra)
    return env


def run_child(code, stdin=None, **extra):
    return subprocess.run([sys.executable, "-c", textwrap.dedent(code)], input=stdin,
                          env=child_env(**extra), capture_output=True, timeout=180)


def last_line(proc):
    lines = proc.stdout.decode(errors="replace").strip().splitlines()
    return lines[-1] if lines else ""


# ── A. devices ────────────────────────────────────────────────────────────
print("A. moss_ports() on a fake /dev/serial/by-id")
saved_root = rp.BYID_ROOT
byid = tempfile.mkdtemp(prefix="byid_")
rp.BYID_ROOT = byid
ports = mb.moss_ports()
check("empty by-id -> both sentinels",
      ports == {"base": "/dev/serial/by-id/MISSING-QT-PY", "lidar": "/dev/serial/by-id/MISSING-C1"}, f"{ports}")
open(os.path.join(byid, C1), "w").close()
ports = mb.moss_ports()
check("C1 only -> lidar resolved, base still the sentinel (the CP2102N is never the base)",
      ports["lidar"] == os.path.join(byid, C1) and ports["base"] == rp.MOSS_QTPY_MISSING, f"{ports}")
open(os.path.join(byid, QTPY), "w").close()
ports = mb.moss_ports()
check("both plugged -> both by-id paths",
      ports == {"base": os.path.join(byid, QTPY), "lidar": os.path.join(byid, C1)},
      f"base {os.path.basename(ports['base'])}, lidar {os.path.basename(ports['lidar'])}")
os.environ["MOSS_PORT"] = "/dev/ttyACM7"
os.environ["MOSS_LIDAR_PORT"] = "/dev/ttyUSB7"
check("MOSS_PORT / MOSS_LIDAR_PORT win over the glob",
      mb.moss_ports() == {"base": "/dev/ttyACM7", "lidar": "/dev/ttyUSB7"})
os.environ.pop("MOSS_PORT")
os.environ.pop("MOSS_LIDAR_PORT")
check("explicit env dict is honoured (no os.environ read)",
      mb.moss_ports({"MOSS_PORT": "/x", "MOSS_LIDAR_PORT": "/y"}) == {"base": "/x", "lidar": "/y"})
check("no resolution ever lands on /dev/ttyUSB0",
      all("ttyUSB0" not in v for v in mb.moss_ports().values()))


def refused(env):
    try:
        mb.moss_ports(env)
    except ValueError as exc:
        return str(exc)
    return None


c1_path = os.path.join(byid, C1)
msg = refused({"MOSS_PORT": c1_path})
check("MOSS_PORT = the C1's by-id path (lidar from the glob) -> ValueError naming the device",
      msg is not None and "same device" in msg and C1 in msg, f"{msg}")
alias = os.path.join(byid, "alias-to-c1")
os.symlink(c1_path, alias)
check("MOSS_PORT = a symlink to the C1, MOSS_LIDAR_PORT = the C1 -> refused (realpath compare)",
      refused({"MOSS_PORT": alias, "MOSS_LIDAR_PORT": c1_path}) is not None)
check("the plugged QT Py + C1 -> no refusal", refused({}) is None)

# ── B. the import ────────────────────────────────────────────────────────
print("B. importing moss_dimos.blueprints in a clean interpreter")
probe = run_child("""
    import json, os, sys
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry as r
    before = r._factory_paths.get("moss")
    import moss_dimos.blueprints as b
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM
    heavy = ["moss_dimos.gamepad", "moss_dimos.costmap2d", "moss_dimos.rplidar_c1",
             "moss_dimos.lidar_odometry", "moss_dimos.explorer2", "moss_dimos.recovering_planner",
             "moss_dimos.memory", "moss_dimos.c1_serial", "dimos.hardware.sensors.camera.module",
             "pygame", "kiss_icp"]
    print(json.dumps({
        "before": before, "after": r._factory_paths.get("moss"),
        "built": [k for k in ("moss_base_blueprint", "moss_gamepad_blueprint",
                              "moss_nav_blueprint", "moss_explore_blueprint") if k in vars(b)],
        "heavy": [m for m in heavy if m in sys.modules],
        "env": {k: os.environ.get(k) for k in ("ROBOT_PROFILE", "PERSISTENT_MAP")},
        "clamp": hasattr(LCM.subscribe, "__wrapped__"),
    }))
""")
try:
    got = json.loads(last_line(probe))
except ValueError:
    got = None
    print(probe.stderr.decode(errors="replace")[-600:])
check("the import runs in a clean interpreter", got is not None, f"rc {probe.returncode}")
got = got or {}
check("'moss' unknown to the registry before the import", got.get("before") is None, f"{got.get('before')}")
check(f"after it: 'moss' -> {FACTORY} (the factory path is a string)", got.get("after") == FACTORY, f"{got.get('after')}")
check("no blueprint built at import (all four are lazy)", got.get("built") == [], f"{got.get('built')}")
check("no pad, lidar, map, planner, explorer, recorder or camera module imported with it",
      got.get("heavy") == [], f"{got.get('heavy')}")
check("no environment variable set by the import (ROBOT_PROFILE, PERSISTENT_MAP untouched)",
      got.get("env") == {"ROBOT_PROFILE": None, "PERSISTENT_MAP": None}, f"{got.get('env')}")
check("the LCM queue clamp is installed by the import (LCM.subscribe wrapped)", got.get("clamp") is True)
other = run_child("""
    import moss_dimos.robot_profile as rp
    print(rp.current().name)
""", ROBOT_PROFILE="other")
check("ROBOT_PROFILE=other is ignored: the profile is still MOSS, with a warning",
      last_line(other) == "moss" and b"ROBOT_PROFILE is ignored" in other.stderr + other.stdout,
      (other.stdout + other.stderr).decode(errors="replace").strip()[-200:])
try:
    mb.no_such_blueprint
    check("unknown attribute raises AttributeError", False)
except AttributeError:
    check("unknown attribute raises AttributeError", True)
try:
    mb._coordinator_blueprint()
    check("_coordinator_blueprint() without a base -> TypeError (no default base)", False)
except TypeError:
    check("_coordinator_blueprint() without a base -> TypeError (no default base)", True)

# ── C. state dir ─────────────────────────────────────────────────────────
print("C. the recording path")
home_db = Path.home() / ".local/state/moss/recordings/moss.db"
check("default ~/.local/state/moss/recordings/moss.db", rp.moss_recording_path() == home_db, str(rp.moss_recording_path()))
os.environ["MOSS_STATE_DIR"] = "/data/moss_state"
check("MOSS_STATE_DIR=/data/moss_state -> /data/moss_state/recordings/moss.db",
      rp.moss_recording_path() == Path("/data/moss_state/recordings/moss.db"))
os.environ["MOSS_STATE_DIR"] = "~/elsewhere"
check("MOSS_STATE_DIR=~/elsewhere -> expanded", rp.moss_state_dir() == Path.home() / "elsewhere")
os.environ["MOSS_STATE_DIR"] = "relative/state"
check("a relative MOSS_STATE_DIR is made absolute (never site-packages)",
      rp.moss_recording_path().is_absolute(), str(rp.moss_recording_path()))
os.environ.pop("MOSS_STATE_DIR")

# ── D. telemetry payload ─────────────────────────────────────────────────
print("D. telemetry_payload(adapter_like, now)")
from moss_dimos.moss_adapter import (MossBaseAdapter, MossGeometry, telemetry_payload,  # noqa: E402
                                     teleop_envelope, teleop_speeds)
from moss_dimos.moss_mock import FakeMossFirmware  # noqa: E402


class FakeAdapter:
    def __init__(self, telemetry, age, mock=False):
        self.telemetry, self.telemetry_age_s, self.mock_bus = telemetry, age, mock


v3_line = {"ms": 1234, "fw": "qtpy-mdd3a-v3", "mode": 2, "reason": "drive",
           "target_pct": [40, -40], "output_pct": [35, -35], "ramp_ms": [5, 5],
           "ticks": [160585, 158263], "invalid_edges": [0, 0], "ina_ok": True,
           "bus_mV": 11208, "shunt_uV": 0, "current_mA": None}
doc = telemetry_payload(FakeAdapter(v3_line, 0.3), 1760000000.0)
expected = {"ts": 1760000000.0, "fw": "qtpy-mdd3a-v3", "driver": None, "mode": 2, "reason": "drive",
            "bus_mV": 11208, "ina_ok": True, "ticks": [160585, 158263], "target_pct": [40, -40],
            "output_pct": [35, -35], "ramp_ms": [5, 5], "firmware_age_s": 0.3, "mock": False}
check("v3 line, age 0.3 s, now 1760000000 -> the known document", doc == expected,
      "" if doc == expected else f"{doc}")
check("exactly the cockpit's keys, in order", list(doc) == ["ts", "fw", "driver", "mode", "reason", "bus_mV", "ina_ok",
                                                             "ticks", "target_pct", "output_pct", "ramp_ms",
                                                             "firmware_age_s", "mock"])
mdd10a = dict(v3_line, driver="mdd10a")
check("an MDD10A build's driver key is carried", telemetry_payload(FakeAdapter(mdd10a, 0.0), 0.0)["driver"] == "mdd10a")
check("no telemetry yet -> None (nothing published)", telemetry_payload(FakeAdapter(None, None), 0.0) is None)
check("an adapter without the attribute -> None", telemetry_payload(object(), 0.0) is None)
check("no adapter at all -> None", telemetry_payload(None, 0.0) is None)
check("age unknown -> firmware_age_s None", telemetry_payload(FakeAdapter(v3_line, None), 0.0)["firmware_age_s"] is None)

fw = FakeMossFirmware()
factory = lambda: fw  # noqa: E731
factory.is_mock = True
a = MossBaseAdapter(port_factory=factory, geometry=MossGeometry())
check("real adapter on the fake firmware connects", a.connect())
time.sleep(0.12)
t0 = time.time()
doc = telemetry_payload(a, t0)
check("-> fw qtpy-mdd3a-v3, 11600 mV (11.6 V), ina_ok, mode 0, mock True",
      doc is not None and doc["fw"] == "qtpy-mdd3a-v3" and doc["bus_mV"] == 11600 and doc["ina_ok"] is True
      and doc["mode"] == 0 and doc["mock"] is True and doc["driver"] is None, f"{doc}")
check("firmware_age_s ~0.12 s after the connect probe", doc is not None and 0.05 <= doc["firmware_age_s"] < 0.5,
      f"{doc and doc['firmware_age_s']}")
check("JSON round trip keeps every value", json.loads(json.dumps(doc)) == doc)
a.disconnect()

# ── E. camera mount ──────────────────────────────────────────────────────
print("E. camera_mount_xyzq(profile)")
m = mb.camera_mount_xyzq(rp.MOSS)
check("MOSS: (0.156, 0, 0.075) m, identity rotation (pitch 0)", m == (0.156, 0.0, 0.075, 0.0, 0.0, 0.0, 1.0), f"{m}")
from dataclasses import replace  # noqa: E402
m = mb.camera_mount_xyzq(replace(rp.MOSS, camera_pitch_deg=1.1))
check("1.1 deg nose down -> qy 0.00959916, qw 0.99995393 (sin / cos of 0.55 deg)",
      close(m[4], 0.00959916, 1e-8) and close(m[6], 0.99995393, 1e-8), f"qy {m[4]:.8f} qw {m[6]:.8f}")
check("unit quaternion", close(math.hypot(m[4], m[6]), 1.0, 1e-12))

# ── H. wrist camera index ────────────────────────────────────────────────
print("H. wrist_camera_index(env, by_id_dir)")
v4l = tempfile.mkdtemp(prefix="v4l_")
check("empty by-id dir, no MOSS_WRIST_DEV -> None (no wrist camera)", mb.wrist_camera_index({}, v4l) is None)
os.symlink("/dev/video6", os.path.join(v4l, "usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0"))
os.symlink("/dev/video7", os.path.join(v4l, "usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index1"))
check("KD-USB by-id video-index0 -> /dev/video6 -> 6 (index1 ignored)", mb.wrist_camera_index({}, v4l) == 6)
check("MOSS_WRIST_DEV=3 -> 3", mb.wrist_camera_index({"MOSS_WRIST_DEV": "3"}, v4l) == 3)
check("MOSS_WRIST_DEV=/dev/video9 -> 9", mb.wrist_camera_index({"MOSS_WRIST_DEV": "/dev/video9"}, v4l) == 9)
check("MOSS_WRIST_DEV=/dev/ttyUSB0 (not a video device) -> None", mb.wrist_camera_index({"MOSS_WRIST_DEV": "/dev/ttyUSB0"}, v4l) is None)
v4l2 = tempfile.mkdtemp(prefix="v4l_other_")
os.symlink("/dev/video2", os.path.join(v4l2, "usb-Intel_R_RealSense_TM_Depth_Camera_455f_Intel_R_RealSense_TM_Depth_Camera_455f_000000000000-video-index0"))
check("only the RealSense under by-id -> None (the nose camera is never the wrist)", mb.wrist_camera_index({}, v4l2) is None)
os.symlink("/dev/video8", os.path.join(v4l2, "usb-Generic_Vendor_USB_Camera_XYZ-video-index0"))
os.symlink("/dev/video9", os.path.join(v4l2, "usb-Generic_Vendor_USB_Camera_XYZ-video-index1"))
check("an unknown UVC camera next to the RealSense -> it is the wrist (/dev/video8), index1 ignored", mb.wrist_camera_index({}, v4l2) == 8)
os.symlink("/dev/video6", os.path.join(v4l2, "usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0"))
check("a known name wins over the unknown one", mb.wrist_camera_index({}, v4l2) == 6)
mb.V4L_BY_ID = v4l          # F and G build the blueprints against this fake by-id
print()

# ── F. entry points ──────────────────────────────────────────────────────
print("F. pyproject entry points")
try:
    import tomllib
    meta = tomllib.loads((ROOT / "pyproject.toml").read_text())
except Exception as exc:  # noqa: BLE001
    meta = {}
    check("pyproject.toml parses", False, f"{type(exc).__name__}: {exc}")
project = meta.get("project", {})
eps = project.get("entry-points", {}).get("dimos.blueprints", {})
TARGETS = {"moss-base": "moss_dimos.blueprints:moss_base_blueprint",
           "moss-gamepad": "moss_dimos.blueprints:moss_gamepad_blueprint",
           "moss-nav": "moss_dimos.blueprints:moss_nav_blueprint",
           "moss-explore": "moss_dimos.blueprints:moss_explore_blueprint"}
check("distribution name moss-dimos (the dimOS namespace)", project.get("name") == "moss-dimos", f"{project.get('name')}")
check("exactly the four MOSS entry points, on moss_dimos.blueprints", eps == TARGETS, f"{eps}")
check("every target has a lazy builder", {t.split(":")[1] for t in eps.values()} == set(mb._BUILDERS),
      f"{sorted(mb._BUILDERS)}")
check("names match dimOS's ^[a-z0-9]+(-[a-z0-9]+)*$", bool(eps) and all(re.match(r"^[a-z0-9]+(-[a-z0-9]+)*$", k) for k in eps))
check("packages moss_dimos and moss_pi", meta.get("tool", {}).get("setuptools", {}).get("packages") == ["moss_dimos", "moss_pi"])
check("pytest collects tests/ only (the benches are scripts)",
      meta.get("tool", {}).get("pytest", {}).get("ini_options", {}).get("testpaths") == ["tests"])

# dimOS reads entry points from installed metadata. A dist-info built from
# pyproject.toml on sys.path is that metadata, without installing anything.
dist_root = tempfile.mkdtemp(prefix="moss_dist_")
dist = Path(dist_root) / f"moss_dimos-{project.get('version', '0')}.dist-info"
dist.mkdir()
(dist / "METADATA").write_text(f"Metadata-Version: 2.1\nName: {project.get('name')}\nVersion: {project.get('version')}\n")
(dist / "entry_points.txt").write_text("[dimos.blueprints]\n" + "".join(f"{k} = {v}\n" for k, v in eps.items()))
sys.path.append(dist_root)
importlib.invalidate_caches()
from dimos.core.coordination.blueprints import Blueprint  # noqa: E402
from dimos.robot.external_blueprints import list_external_blueprint_names  # noqa: E402
from dimos.robot.get_all_blueprints import get_by_name  # noqa: E402

listed = [n for n in list_external_blueprint_names() if n.startswith("moss-dimos.")]
check("dimOS lists moss-dimos.moss-base / moss-explore / moss-gamepad / moss-nav",
      listed == ["moss-dimos.moss-base", "moss-dimos.moss-explore", "moss-dimos.moss-gamepad", "moss-dimos.moss-nav"],
      f"{listed}")
resolved = {}
for local in TARGETS:
    name = f"moss-dimos.{local}"
    try:
        resolved[local] = get_by_name(name)
        n = len(resolved[local].blueprints)
        check(f"get_by_name({name!r}) -> a Blueprint ({n} module(s))", isinstance(resolved[local], Blueprint))
    except Exception as exc:  # noqa: BLE001
        check(f"get_by_name({name!r})", False, f"{type(exc).__name__}: {exc}")
check("resolution caches the built blueprint on the module (built once per process)",
      resolved.get("moss-nav") is not None and resolved.get("moss-nav") is mb.moss_nav_blueprint)
sys.path.remove(dist_root)
shutil.rmtree(dist_root, ignore_errors=True)
importlib.invalidate_caches()

# ── G. blueprints (dimOS) ────────────────────────────────────────────────
print("G. moss-base / moss-gamepad / moss-nav / moss-explore blueprints")
from dimos.core.coordination.blueprints import TransportSpec  # noqa: E402
from dimos.core.transport import JpegLcmTransport  # noqa: E402
from dimos.msgs.sensor_msgs.Image import Image  # noqa: E402
from moss_dimos.blueprints import MossControlCoordinator  # noqa: E402
from moss_dimos.gamepad import GamepadTeleop  # noqa: E402
from moss_dimos.memory import MossMemoryConfig, MossRecorder  # noqa: E402


def names(bp):
    return {b.module.__name__ for b in bp.active_blueprints}


def atom(bp, cls_name):
    return next(b for b in bp.active_blueprints if b.module.__name__ == cls_name)


base = mb.build_moss_base()
check("moss-base: the coordinator alone, served as 'ControlCoordinator'",
      [x.name for x in base.blueprints] == ["ControlCoordinator"], f"{[x.name for x in base.blueprints]}")
coord_b = base._instance_key(MossControlCoordinator)
check("moss-base: twist_command -> cmd_vel", base.remapping_map.get((coord_b, "twist_command")) == "cmd_vel")
hw = atom(base, "MossControlCoordinator").kwargs["hardware"][0]
check("moss-base: adapter 'moss', port left to the adapter (MOSS_PORT, by-id, sentinel)",
      hw.adapter_type == "moss" and hw.address is None, f"{hw.adapter_type} {hw.address}")
task = atom(base, "MossControlCoordinator").kwargs["tasks"][0]
check("moss-base: task vel_base, velocity, priority 10, on the base joints",
      (task.name, task.type, task.priority) == ("vel_base", "velocity", 10) and list(task.joint_names) == list(mb._base_joints))

pad_bp = mb.build_moss_gamepad()
check("moss-gamepad: coordinator + gamepad", names(pad_bp) == {"MossControlCoordinator", "GamepadTeleop"}, f"{sorted(names(pad_bp))}")
coord_g = pad_bp._instance_key(MossControlCoordinator)
pad_g = pad_bp._instance_key(GamepadTeleop)
check("moss-gamepad: coordinator instance ControlCoordinator", coord_g == "ControlCoordinator")
check("moss-gamepad: twist_command -> cmd_vel and tele_cmd_vel -> cmd_vel (direct drive)",
      pad_bp.remapping_map.get((coord_g, "twist_command")) == "cmd_vel"
      and pad_bp.remapping_map.get((pad_g, "tele_cmd_vel")) == "cmd_vel")
gk = atom(pad_bp, "GamepadTeleop").kwargs
g = MossGeometry.from_env()
lin, ang = teleop_speeds(g)
check("moss-gamepad: stick scale = the base's: 0.45 m/s, 6.7669 rad/s (default geometry)",
      close(gk.get("linear_speed"), 0.45) and close(gk.get("angular_speed"), 2.0 * 2.0 * 0.45 / 0.266)
      and (gk.get("linear_speed"), gk.get("angular_speed")) == (lin, ang),
      f"{gk.get('linear_speed')} m/s, {gk.get('angular_speed')} rad/s")
env_d = gk.get("envelope")
check("moss-gamepad: the envelope rides as a plain dict: linear 0.45, lever 0.133, rim 1.35",
      isinstance(env_d, dict) and env_d == teleop_envelope(g)
      and close(env_d["lever"], 0.133) and close(env_d["rim"], 1.35), f"{env_d}")
os.environ["MOSS_MPS_AT_LIMIT"] = "0.30"
slow = atom(mb.build_moss_gamepad(), "GamepadTeleop").kwargs
os.environ.pop("MOSS_MPS_AT_LIMIT")
check("MOSS_MPS_AT_LIMIT=0.30 read when the blueprint is built -> 0.30 m/s, rim 0.90",
      close(slow.get("linear_speed"), 0.30) and close(slow["envelope"]["rim"], 0.30 + 0.30 * 4.0 / 0.266 * 0.133),
      f"{slow.get('linear_speed')} {slow['envelope']}")

nav = mb.build_moss_nav()
n = names(nav)
want = {"MossControlCoordinator", "GamepadTeleop", "RPLidarC1", "LidarOdometry",
        "MossCostMap", "RealSenseCamera", "MossRecorder"}
check("moss-nav modules", want <= n, f"{sorted(n)}")
check("moss-nav has no planner, no explorer, no MovementManager, no viewer",
      not ({"RecoveringPlanner", "Explorer2", "MovementManager", "WebsocketVisModule"} & n))
coord = nav._instance_key(MossControlCoordinator)
pad = nav._instance_key(GamepadTeleop)
check("coordinator instance name ControlCoordinator", coord == "ControlCoordinator")
check("remap twist_command -> cmd_vel", nav.remapping_map.get((coord, "twist_command")) == "cmd_vel")
check("remap pad tele_cmd_vel -> cmd_vel (direct drive)", nav.remapping_map.get((pad, "tele_cmd_vel")) == "cmd_vel")
spec = nav.transport_map.get(("color_image", Image))
check("color_image crosses the bus as JPEG on /color_image",
      isinstance(spec, TransportSpec) and spec.cls is JpegLcmTransport and spec.args[0] == "/color_image")
lid = atom(nav, "RPLidarC1").kwargs
check("lidar on the CP2102N by-id at 460800, frame lidar_link",
      lid.get("port") == os.path.join(byid, C1) and lid.get("baudrate") == 460800 and lid.get("frame_id") == "lidar_link")
hw = atom(nav, "MossControlCoordinator").kwargs["hardware"][0]
check("base: adapter 'moss' on the QT Py by-id", hw.adapter_type == "moss" and hw.address == os.path.join(byid, QTPY))
check("pad: the same MOSS envelope as moss-gamepad", atom(nav, "GamepadTeleop").kwargs.get("envelope") == teleop_envelope(g))
check("odometry without the gyro prior", atom(nav, "LidarOdometry").kwargs.get("use_gyro_prior") is False)
cam = atom(nav, "RealSenseCamera").kwargs
check("RealSense 640x480 @ 5 fps, colour + depth (aligned), no point cloud, no IMU",
      (cam.get("width"), cam.get("height"), cam.get("fps"), cam.get("enable_depth"),
       cam.get("enable_pointcloud"), cam.get("enable_imu")) == (640, 480, 5, True, False, False))
t = cam["base_transform"]
check("camera mount from the profile: (0.156, 0, 0.075) m",
      close(t.translation.x, 0.156) and close(t.translation.z, 0.075))
check("recorder at the MOSS state dir", atom(nav, "MossRecorder").kwargs.get("db_path") == str(home_db))
check("wrist camera module WristCamera in moss-nav (fake by-id -> /dev/video6)", "WristCamera" in n, f"{sorted(n)}")
wc = atom(nav, "WristCamera").kwargs
check("wrist: Webcam hardware factory on index 6, 640x480 @ 5 fps, frame prefix 'wrist', no TF, 5 Hz barrier",
      getattr(wc.get("hardware"), "keywords", {}).get("camera_index") == 6
      and getattr(wc.get("hardware"), "keywords", {}).get("width") == 640
      and getattr(wc.get("hardware"), "keywords", {}).get("fps") == 5.0
      and getattr(wc.get("hardware"), "keywords", {}).get("frame_id_prefix") == "wrist"
      and wc.get("transform") is None and wc.get("frequency") == 5.0, f"{wc}")
wk = mb.wrist_instance(nav)
check("wrist instance name found (dimOS lowercases it)", wk == "wristcamera", f"{wk}")
check("wrist outputs remapped: color_image -> wrist_image, camera_info -> wrist_camera_info, tf -> wrist_tf",
      nav.remapping_map.get((wk, "color_image")) == "wrist_image"
      and nav.remapping_map.get((wk, "camera_info")) == "wrist_camera_info"
      and nav.remapping_map.get((wk, "tf")) == "wrist_tf", f"{dict(nav.remapping_map)}")
wspec = nav.transport_map.get(("wrist_image", Image))
check("wrist_image crosses the bus as JPEG on /wrist_image",
      isinstance(wspec, TransportSpec) and wspec.cls is JpegLcmTransport and wspec.args[0] == "/wrist_image")
check("the RealSense keeps its own names (no remap on RealSenseCamera)",
      not any(k[0] == "RealSenseCamera" for k in nav.remapping_map))

exp = mb.build_moss_explore()
e = names(exp)
check("moss-explore adds RecoveringPlanner, MovementManager, Explorer2",
      {"RecoveringPlanner", "MovementManager", "Explorer2"} <= e and want <= e, f"{sorted(e)}")
pad_e = exp._instance_key(GamepadTeleop)
check("moss-explore: NO pad -> cmd_vel remap (MovementManager arbitrates)",
      (pad_e, "tele_cmd_vel") not in exp.remapping_map)
check("moss-explore: twist_command -> cmd_vel", exp.remapping_map.get(("ControlCoordinator", "twist_command")) == "cmd_vel")
ex = atom(exp, "Explorer2").kwargs
check("explorer clearances = derived(MOSS): 0.22 / 0.25 m",
      close(ex.get("lethal_clearance_m"), 0.22) and close(ex.get("pivot_clearance_m"), 0.25), f"{ex}")
check("explorer frontier floor 0.3 m, goal timeout 45 s",
      ex.get("min_frontier_perimeter") == 0.3 and ex.get("goal_timeout") == 45.0)
pl = atom(exp, "RecoveringPlanner").kwargs
check("planner footprint = MOSS width 0.34, pivot 0.50",
      pl.get("robot_width") == 0.34 and pl.get("robot_rotation_diameter") == 0.50)

os.environ["MOSS_CAMERA"] = "0"
os.environ["MOSS_RECORD"] = "0"
bare = mb.build_moss_nav()
check("MOSS_CAMERA=0 MOSS_RECORD=0 -> no RealSense, no recorder, no JPEG transport",
      not ({"RealSenseCamera", "MossRecorder"} & names(bare)) and ("color_image", Image) not in bare.transport_map)
os.environ["MOSS_WRIST"] = "0"
bare2 = mb.build_moss_nav()
check("MOSS_WRIST=0 -> no WristCamera, no wrist remap, no wrist transport",
      "WristCamera" not in names(bare2) and mb.wrist_instance(bare2) is None
      and ("wrist_image", Image) not in bare2.transport_map)
os.environ.pop("MOSS_WRIST")
os.environ.update(MOSS_CAMERA="OFF", MOSS_RECORD=" no ", MOSS_WRIST="False")
spelled = mb.build_moss_nav()
check("MOSS_CAMERA=OFF, MOSS_RECORD=' no ', MOSS_WRIST=False -> the same bare build "
      "(robot_profile.env_on, the one switch parser)",
      not ({"RealSenseCamera", "MossRecorder", "WristCamera"} & names(spelled))
      and ("color_image", Image) not in spelled.transport_map and ("wrist_image", Image) not in spelled.transport_map)
check("the blueprint has no switch parser of its own", not hasattr(mb, "_env_on"))
os.environ.update(MOSS_CAMERA="0", MOSS_RECORD="0")
os.environ.pop("MOSS_WRIST")
mb.V4L_BY_ID = tempfile.mkdtemp(prefix="v4l_empty_")
nowrist = mb.build_moss_nav()
check("wrist camera absent from by-id -> module skipped, blueprint still builds",
      "WristCamera" not in names(nowrist) and ("wrist_image", Image) not in nowrist.transport_map)
mb.V4L_BY_ID = v4l
os.environ["MOSS_RERUN"] = "1"
check("MOSS_RERUN=1 -> the stock viewers ride along", "WebsocketVisModule" in names(mb.build_moss_nav()))
for k in ("MOSS_CAMERA", "MOSS_RECORD", "MOSS_RERUN"):
    os.environ.pop(k, None)
empty = tempfile.mkdtemp(prefix="byid_empty_")
rp.BYID_ROOT = empty
missing = mb.build_moss_nav()
shutil.rmtree(empty, ignore_errors=True)
check("devices absent -> the sentinels reach the modules (fail loudly, no fallback)",
      atom(missing, "RPLidarC1").kwargs["port"] == rp.MOSS_LIDAR_MISSING
      and atom(missing, "MossControlCoordinator").kwargs["hardware"][0].address == rp.MOSS_QTPY_MISSING)

cfg = MossMemoryConfig()
check("MossMemoryConfig default db_path ~/.local/state/moss/recordings/moss.db", Path(cfg.db_path) == home_db, str(cfg.db_path))
check("poseless: global_costmap, cmd_vel, base_telemetry, episode_marker; record_tf False",
      cfg.poseless_streams == ["global_costmap", "cmd_vel", "base_telemetry", "episode_marker"] and cfg.record_tf is False)
os.environ["MOSS_STATE_DIR"] = "/data/moss_state"
check("MOSS_STATE_DIR read when the config is built", Path(MossMemoryConfig().db_path) == Path("/data/moss_state/recordings/moss.db"))
os.environ.pop("MOSS_STATE_DIR")
ins = set(MossRecorder.__annotations__) | set(MossRecorder.__mro__[1].__annotations__)
check("MossRecorder records odom, global_costmap, lidar, color_image, wrist_image, cmd_vel, base_telemetry, episode_marker",
      {"odom", "global_costmap", "lidar", "color_image", "wrist_image", "cmd_vel", "base_telemetry", "episode_marker"} <= ins)
check("MossControlCoordinator has the base_telemetry Out, and no other stream of its own",
      set(MossControlCoordinator.__annotations__) == {"base_telemetry"}, f"{sorted(MossControlCoordinator.__annotations__)}")
check("its start/stop overrides keep @rpc (else dimOS pickles the module to call them)",
      {"start", "stop"} <= set(MossControlCoordinator.rpcs))
check("it runs in its own worker (dedicated_worker)", MossControlCoordinator.dedicated_worker is True)

# ── I. the coordinator and the wrist camera in a fresh worker ───────────
print("I. what a dimOS worker does: unpickle in a clean interpreter")
# dimOS deploys modules into forkserver workers: a fresh interpreter that
# imports dimos but not this package. It only imports what it needs to
# UNPICKLE the module class it was told to deploy. So the pickled coordinator
# has to drag moss_dimos.blueprints in with it - otherwise _setup_hardware
# dies with KeyError: "Unknown twist base adapter: moss".
payload = pickle.dumps(MossControlCoordinator)
check("the coordinator pickles by reference to moss_dimos.blueprints",
      b"moss_dimos.blueprints" in payload and b"MossControlCoordinator" in payload)
worker = run_child("""
    import json, pickle, sys
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry as r
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM
    before = "moss" in r.available()
    cls = pickle.loads(sys.stdin.buffer.read())
    after = "moss" in r.available()
    adapter = r.create("moss", dof=3, address="/dev/serial/by-id/MISSING-QT-PY", hardware_id="base")
    print(json.dumps({
        "before": before, "after": after, "cls": cls.__name__,
        "path": r._factory_paths.get("moss"),
        "adapter": f"{type(adapter).__module__}.{type(adapter).__name__}",
        "port": adapter._port, "clamp": hasattr(LCM.subscribe, "__wrapped__"),
        "moss_modules": sorted(m for m in sys.modules if m.startswith("moss_dimos")),
    }))
""", stdin=payload)
try:
    w = json.loads(last_line(worker))
except ValueError:
    w = {}
    print(worker.stderr.decode(errors="replace")[-600:])
check("the worker unpickles MossControlCoordinator", w.get("cls") == "MossControlCoordinator", f"rc {worker.returncode}")
check("a clean interpreter does not know 'moss' before unpickling", w.get("before") is False)
check("'moss' registered after unpickling, path " + FACTORY, w.get("after") is True and w.get("path") == FACTORY)
check("the registry builds the adapter from that string (as _setup_hardware does): MossBaseAdapter",
      w.get("adapter") == "moss_dimos.moss_adapter.MossBaseAdapter" and w.get("port") == "/dev/serial/by-id/MISSING-QT-PY",
      f"{w.get('adapter')} on {w.get('port')}")
check("the LCM queue clamp is on in the worker", w.get("clamp") is True)
check("the worker loads only the package, robot_profile, the adapter, the clamp and the wrapper",
      w.get("moss_modules") == ["moss_dimos", "moss_dimos.blueprints", "moss_dimos.latest_only", "moss_dimos.lcm_latest",
                                "moss_dimos.moss_adapter", "moss_dimos.robot_profile"],
      f"{w.get('moss_modules')}")

WristCamera = mb.WristCamera
wpayload = pickle.dumps(WristCamera)
check("WristCamera: module moss_dimos.blueprints, qualname WristCamera",
      (WristCamera.__module__, WristCamera.__qualname__) == ("moss_dimos.blueprints", "WristCamera"))
check("pickle.dumps(WristCamera) round-trips to the same class (served by the module __getattr__)",
      pickle.loads(wpayload) is WristCamera and mb.WristCamera is WristCamera)
wchild = run_child("""
    import json, pickle, sys
    from dimos.hardware.sensors.camera.module import CameraModule
    cls = pickle.loads(sys.stdin.buffer.read())
    import moss_dimos.blueprints as b
    print(json.dumps({"name": cls.__name__, "module": cls.__module__, "camera": issubclass(cls, CameraModule),
                      "same": b.WristCamera is cls}))
""", stdin=wpayload)
try:
    wc_got = json.loads(last_line(wchild))
except ValueError:
    wc_got = {}
    print(wchild.stderr.decode(errors="replace")[-600:])
check("... and unpickles in a clean interpreter to a CameraModule named WristCamera",
      wc_got == {"name": "WristCamera", "module": "moss_dimos.blueprints", "camera": True, "same": True}, f"{wc_got}")

from dimos.hardware.drive_trains.registry import twist_base_adapter_registry  # noqa: E402
try:
    twist_base_adapter_registry.register_path("moss", mb.MOSS_ADAPTER_PATH)
    check("registering the same path again is a no-op (re-import is safe)", True)
except ValueError as exc:
    check("registering the same path again is a no-op (re-import is safe)", False, str(exc))
clash = run_child("""
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry as r
    r.register_path("moss", "other_pkg.adapter:OtherAdapter")
    import moss_dimos.blueprints
    print("REGISTRY", r._factory_paths.get("moss"))
""")
out = clash.stdout.decode(errors="replace") + clash.stderr.decode(errors="replace")
warn = [ln for ln in out.splitlines() if "already registered" in ln]
check("a conflicting 'moss' registration: the import still succeeds, the first path stays",
      clash.returncode == 0 and last_line(clash) == "REGISTRY other_pkg.adapter:OtherAdapter", out.strip()[-300:])
check("... and a warning names both paths (not swallowed)",
      bool(warn) and "other_pkg.adapter:OtherAdapter" in warn[0] and FACTORY in warn[0], f"{warn[:1]}")

rp.BYID_ROOT = saved_root
shutil.rmtree(byid, ignore_errors=True)
shutil.rmtree(v4l, ignore_errors=True)
finish()
