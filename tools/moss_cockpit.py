#!/usr/bin/env python3
"""MOSS cockpit: one small web server on the rover, one page for a whole run.

    http://<rover>:8904/            the page (tools/moss_cockpit/index.html)
    .venv/bin/python tools/moss_cockpit.py            live: listens to the stack's LCM bus
    python3 tools/moss_cockpit.py --demo --port 8905  synthetic rover, no bus, no dimOS

Why a separate process instead of a dimOS web module: dimOS's own viewers either
need a secure context, git LFS (the command center) or cost a core with no
client connected (inflate + distance transform on every costmap). This server
imports lcm, dimos_lcm, numpy and moss_dimos.robot_profile (pure python) only -
never dimos, torch or open3d - and does nothing until a page asks: LCM callbacks keep the latest bytes and a timestamp,
decoding happens on request and is cached by that timestamp (map at most 2 Hz,
scan at most 4 Hz). A camera channel is subscribed only while an MJPEG client
is watching it.

What it touches: the LCM bus (explicit channels, listen; publish only on a POST
/cmd), ~/.local/state/dimos/runs/*.json, the running stack's main.jsonl and
/proc/<stack pid>/environ (read: the bus the stack chose), $MOSS_STATE_DIR
(default ~/.local/state/moss: preflight.json and the recording, read;
last_launch.log, written by the launcher it starts). It NEVER
opens a serial port: battery and arming state come from /base_telemetry, which
the stack's coordinator publishes.

It stops an idle moss-nav or moss-explore stack by itself: no commanded track, no
non-zero cmd_vel, no explore order, no goal, no episode, no move of the pose (2 cm or
2 deg over 10 s) and no command from the page for MOSS_IDLE_STOP_S seconds (default
300, at least 30, 0 = never) runs `tools/moss-fly.sh stop` with the message
"stopping the stack to preserve battery". A cockpit that hears nothing on the bus,
or a stack on another bus, is never stopped; moss-gamepad and the other blueprints
stay up.

Endpoints (all Cache-Control: no-store; a failing request never kills the server):
    GET  /  /static/<file>      the page and its assets (tools/moss_cockpit/)
    GET  /state.json            stack, pose, topic rates, base, nav, recording, system
    GET  /map.png  /map_meta    latest /global_costmap as a PNG + the pixel <-> metre contract
    GET  /scan.json             latest lidar points, world frame (fallback: lidar frame), and
                                apart the depth camera's obstacle points the same cloud carries;
                                plus the pose read at the request, as /state.json's (the page
                                polls the scan at 3 Hz, so its robot marker moves at 3 Hz)
    GET  /path.json             planner path
    GET  /log.json?n=40         tail of the running stack's main.jsonl
    GET  /decisions.json?n=40   what the stack decided, read from the same main.jsonl: the last n
                                decision events (goals, obstacles, replans, contacts, waits, the
                                mission) as short English lines, and the state word now
                                (HEADING, OBSTACLE, BACKING OFF...), at most 2 Hz of file reads;
                                after a stop, the last run's events until the next run
    GET  /preflight.json        the last saved flight check
    GET  /launch.json           tail of the last launcher output
    GET  /camera.mjpg           nose camera (RealSense colour), multipart JPEG, at most 5 fps
    GET  /camera.mjpg?cam=wrist the arm's camera, same framing (cam=nose is the default)
    POST /cmd/explore|goal|stop|episode|stack   publish on the bus / run tools/moss-fly.sh
                                (header 'X-Requested-With: moss-cockpit' required, else 403;
                                explore and goal only on a moss-* stack, else 409;
                                bus commands only on an LCM run, else 503: the stack's own
                                DIMOS_TRANSPORT, then TRANSPORT, as dimOS reads them)
    POST /cmd/keepalive         the idle stop's count starts again: the stack stays up (same
                                header; the answer carries /state.json's "idle")

Standard library + numpy (+ lcm and dimos_lcm in live mode). No build step,
no CDN: the field has no internet.
"""

from __future__ import annotations

import argparse
import base64
import datetime as _dt
import glob
import json
import math
import os
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)      # explore_ctl, the shared out-of-stack publisher
if REPO_DIR not in sys.path:
    sys.path.append(REPO_DIR)          # moss_dimos, when the venv does not install it already

try:
    from moss_dimos import robot_profile   # pure: the geometry the stack flies (lidar tag height)
except Exception:  # noqa: BLE001 - tools/ copied without the package: no lidar/camera split
    robot_profile = None

try:
    import lcm  # the bus; absent on a laptop: live mode says so, --demo does not need it
except ImportError:
    lcm = None

try:
    from dimos_lcm.geometry_msgs import Point as LcmPoint
    from dimos_lcm.geometry_msgs import Pose as LcmPose
    from dimos_lcm.geometry_msgs import PoseStamped as LcmPoseStamped
    from dimos_lcm.geometry_msgs import Quaternion as LcmQuaternion
    from dimos_lcm.geometry_msgs import Twist as LcmTwist
    from dimos_lcm.geometry_msgs import Vector3 as LcmVector3
    from dimos_lcm.nav_msgs import OccupancyGrid as LcmOccupancyGrid
    from dimos_lcm.nav_msgs import Path as LcmPath
    from dimos_lcm.sensor_msgs import Image as LcmImage
    from dimos_lcm.sensor_msgs import PointCloud2 as LcmPointCloud2
    from dimos_lcm.std_msgs import Bool as LcmBool
    from dimos_lcm.std_msgs import Header as LcmHeader
    from dimos_lcm.std_msgs import String as LcmString
    from dimos_lcm.std_msgs import Time as LcmTime
    HAVE_MSGS = True
except Exception:  # noqa: BLE001 - pure-python codecs, but not installed everywhere
    HAVE_MSGS = False

DEFAULT_PORT = 8904
DEFAULT_BIND = "0.0.0.0"
PAGE_DIR = os.path.join(TOOLS_DIR, "moss_cockpit")
FLY_SCRIPT = os.path.join(TOOLS_DIR, "moss-fly.sh")

# LCM channels as the stack names them: "/<stream>#<dimOS msg_name>". Spelled
# out instead of derived from dimos_lcm classes, whose msg_name is not always
# the one dimOS uses (dimos_lcm's PoseStamped says just "PoseStamped").
CHANNELS = {
    "global_costmap": "/global_costmap#nav_msgs.OccupancyGrid",
    "odom": "/odom#geometry_msgs.PoseStamped",
    "lidar": "/lidar#sensor_msgs.PointCloud2",            # world frame, from LidarOdometry
    "pointcloud": "/pointcloud#sensor_msgs.PointCloud2",  # lidar frame, straight from the C1 driver
    "path": "/path#nav_msgs.Path",
    "goal_request": "/goal_request#geometry_msgs.PoseStamped",
    "goal_reached": "/goal_reached#std_msgs.Bool",
    "explore_done": "/explore_done#std_msgs.Bool",
    "explore_cmd": "/explore_cmd#std_msgs.Bool",
    "stop_explore_cmd": "/stop_explore_cmd#std_msgs.Bool",
    "stop_movement": "/stop_movement#std_msgs.Bool",       # MovementManager on a pad touch: ends exploration
    "cmd_vel": "/cmd_vel#geometry_msgs.Twist",
    "base_telemetry": "/base_telemetry#std_msgs.String",
    "episode_marker": "/episode_marker#std_msgs.String",
}
CAMERAS = {                     # cockpit name -> (bus topic, LCM channel); subscribed only while watched
    "nose": ("color_image", "/color_image#sensor_msgs.Image"),
    "wrist": ("wrist_image", "/wrist_image#sensor_msgs.Image"),
}
IMAGE_NAME, IMAGE_CHANNEL = CAMERAS["nose"]
STATE_TOPICS = ("odom", "global_costmap", "lidar", "pointcloud", "path", "cmd_vel",
                "base_telemetry") + tuple(name for name, _ in CAMERAS.values())

MAP_MIN_INTERVAL_S = 0.5      # decode + PNG the costmap at most 2 Hz, whoever asks
SCAN_MIN_INTERVAL_S = 0.25    # scan at most 4 Hz
SCAN_FALLBACK_S = 5.0         # /lidar silent this long -> show /pointcloud (lidar frame)
MAX_SCAN_POINTS = 1500        # per list: the lidar returns, and apart the depth camera's points
LIDAR_Z_TOL_M = 1e-6          # costmap2d's rule: a lidar return sits at EXACTLY the profile's lidar_height_m
HZ_WINDOW_S = 5.0
MJPEG_PERIOD_S = 0.2          # 5 fps: enough to see where the nose points, cheap on a small board
MAX_CAMERA_CLIENTS = 4
FIRST_FRAME_WAIT_S = 2.0
LCM_HANDLE_MS = 250           # the listener loop also (un)subscribes the camera between handles
SYSTEM_CACHE_S = 1.0
MAX_BODY = 65536
GOAL_LIMIT_M = 500.0          # MovementManager ignores clicks beyond 500 m: refuse them here, loudly
LAUNCH_TAIL = 60
LAUNCH_KILL_GRACE_S = 3.0     # 'stop' during a launch: SIGTERM its process group, SIGKILL after this
LAUNCH_ENV_DROP = ("EXPLORE", "MOCK", "MOSS_MOCK_BUS")   # per-run switches of a CLI run: never inherited
GOAL_AFTER_STOP_S = 3.0       # the explorer's own 'stay here' goal lands ~2 s after its stop: ours goes after
MJPEG_WRITE_TIMEOUT_S = 5.0   # a viewer that vanished without FIN/RST frees its slot after this
CMD_HEADER = ("X-Requested-With", "moss-cockpit")   # every POST /cmd/* carries it: a cross-site form cannot
IDLE_STOP_DEFAULT_S = 300.0   # MOSS_IDLE_STOP_S unset: a MOSS stack idle this long is stopped (0 = never)
IDLE_STOP_MIN_S = 30.0        # a shorter limit is refused (the default applies): it would stop a stack in use
IDLE_STOP_BLUEPRINTS = ("moss-nav", "moss-explore")   # the stacks it stops: odometry + costmap load the
                                                      # board; moss-gamepad and the others stay up
IDLE_TICK_S = 1.0             # the idle watchdog looks once per second, from the bus tap's loop
IDLE_MOVE_M = 0.02            # the pose moved more than this over IDLE_WINDOW_S: activity
IDLE_TURN_RAD = math.radians(2.0)   # ... or turned more than this
IDLE_WINDOW_S = 10.0          # the pose now against the pose this long ago: drift at rest never adds up
IDLE_DEAF_S = 10.0            # a MOSS stack up and nothing heard on any tapped channel this long: deaf
IDLE_FIRMWARE_AGE_S = 1.0     # telemetry whose firmware line is older: the coordinator repeating its last one
IDLE_BUSY_MAX_S = 120.0       # a launcher busy longer than this no longer holds the stop
IDLE_STOP_TEXT = "stopping the stack to preserve battery"
IDLE_KEYS = ("enabled", "idle_s", "stop_in_s", "limit_s", "last_source", "blind")   # /state.json "idle"
IDLE_SOURCES = ("start", "telemetry", "cmd_vel", "odom", "explore_cmd", "goal_request", "episode",
                "post", "keepalive", "demo")   # idle.last_source: what restarted the count last

# 3S pack: 12.6 V full, the BMS cuts at 10.2 V (linear, clamped; a voltage
# gauge, not a coulomb counter).
BATTERY_EMPTY_V = 10.2
BATTERY_FULL_V = 12.6
MODE_NAMES = {0: "off", 1: "ready", 2: "drive", 3: "test"}

# Map colours, UI-only (the brand has no occupancy palette): unknown = STUDIO,
# free = a darker studio, cost 1..99 ramps from free to a light slate, lethal =
# brand WHITE, so obstacles read first on the dark page.
COLOR_UNKNOWN = (0x3D, 0x57, 0x6A)
COLOR_FREE = (0x2E, 0x45, 0x56)
COLOR_COST_MAX = (0x8F, 0xA6, 0xB5)
COLOR_OCCUPIED = (0xEA, 0xF0, 0xEF)

_ISO = "%Y-%m-%dT%H:%M:%S"


def state_dir() -> str:
    """MOSS's own state dir (recordings, preflight.json, last_launch.log)."""
    return os.path.expanduser(os.environ.get("MOSS_STATE_DIR") or "~/.local/state/moss")


def runs_dir() -> str:
    """dimOS's run registry, resolved the way dimos.constants does on Linux."""
    base = os.environ.get("XDG_STATE_HOME") or os.path.expanduser("~/.local/state")
    return os.path.join(base, "dimos", "runs")


# --- pictures ----------------------------------------------------------------

def png_bytes(rgb: np.ndarray, level: int = 3) -> bytes:
    """An (h, w, 3) uint8 array as a PNG, with zlib and struct only (level 3 instead
    of 6: ~2x faster on a 480x480 costmap, a few % bigger)."""
    h, w, _ = rgb.shape
    rows = np.zeros((h, w * 3 + 1), dtype=np.uint8)
    rows[:, 1:] = rgb.reshape(h, w * 3)          # filter byte 0 = None, per row
    raw = zlib.compress(rows.tobytes(), level)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", raw)
            + chunk(b"IEND", b""))


def _build_lut() -> np.ndarray:
    """256-entry colour table indexed by the int8 cell value seen as uint8."""
    lut = np.empty((256, 3), dtype=np.uint8)
    lut[:] = COLOR_UNKNOWN                       # -128..-1 (as 128..255): unknown
    lut[0] = COLOR_FREE
    free = np.array(COLOR_FREE, dtype=float)
    top = np.array(COLOR_COST_MAX, dtype=float)
    for v in range(1, 100):
        lut[v] = np.rint(free + (top - free) * (v / 99.0)).astype(np.uint8)
    lut[100:128] = COLOR_OCCUPIED                # 100 lethal (and anything above)
    return lut


MAP_LUT = _build_lut()


def map_meta(width: int, height: int, res: float, ox: float, oy: float,
             stamp: float, frame: str) -> dict:
    """The /map_meta contract: extent and pixel <-> metre formulas, plus the costmap stamp and frame."""
    return {
        "res": res, "ox": ox, "oy": oy,
        "width": int(width), "height": int(height),
        "x_min": ox, "x_max": ox + width * res,
        "y_min": oy, "y_max": oy + height * res,
        "stamp": stamp, "frame": frame,
        # px -> metres and back. Row 0 of the PNG is the highest y.
        "x_of_px": "x = ox + (px + 0.5) * res",
        "y_of_py": "y = oy + (height - py - 0.5) * res",
        "px_of_x": "px = (x - ox) / res",
        "py_of_y": "py = height - (y - oy) / res",
    }


def render_grid(cells: np.ndarray, res: float, ox: float, oy: float,
                stamp: float = 0.0, frame: str = "world") -> tuple[bytes, dict]:
    """An OccupancyGrid's cells (rows = y from min y up, int8 -1/0/1..99/100) as (PNG, meta).
    Flipped so PNG row 0 is the highest y."""
    grid = np.ascontiguousarray(cells, dtype=np.int8)
    h, w = grid.shape
    rgb = MAP_LUT[grid.view(np.uint8)]
    return png_bytes(np.flipud(rgb)), map_meta(w, h, float(res), float(ox), float(oy), stamp, frame)


# --- message decoding (bytes off the bus -> plain values) -----------------------

def _stamp(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nsec) * 1e-9


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading (rad) of a quaternion, z-up."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def decode_costmap(raw: bytes) -> tuple[bytes, dict]:
    m = LcmOccupancyGrid.lcm_decode(raw)
    w, h, n = int(m.info.width), int(m.info.height), int(m.data_length)
    # the int8 cells are the message's last field: view them straight off the
    # raw bytes instead of rebuilding a 230k-element tuple
    cells = np.frombuffer(raw, dtype=np.int8, count=n, offset=len(raw) - n)
    if n != w * h:
        raise ValueError(f"costmap {w}x{h} carries {n} cells")
    o = m.info.origin.position
    return render_grid(cells.reshape(h, w), float(m.info.resolution), float(o.x), float(o.y),
                       _stamp(m.header), m.header.frame_id or "world")


def decode_pose(raw: bytes) -> dict:
    m = LcmPoseStamped.lcm_decode(raw)
    p, q = m.pose.position, m.pose.orientation
    return {"x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw": yaw_of(q.x, q.y, q.z, q.w), "stamp": _stamp(m.header),
            "frame": m.header.frame_id}


def cloud_xyz(raw: bytes) -> tuple[str, float, np.ndarray]:
    """(frame, stamp, Nx3 float32 x/y/z) of a PointCloud2, rows with a NaN/inf
    coordinate dropped. Field offsets read from the message (dimOS writes x,y,z
    float32 at 0/4/8 with a 16-byte step, more when per-point extras are attached);
    a cloud without a z field reads z = 0."""
    m = LcmPointCloud2.lcm_decode(raw)
    n = int(m.width) * int(m.height)
    step = int(m.point_step)
    offs = {f.name: int(f.offset) for f in m.fields}
    if n == 0 or step < 8:
        return m.header.frame_id, _stamp(m.header), np.zeros((0, 3), np.float32)
    end = ">" if m.is_bigendian else "<"
    zoff = offs.get("z", None if offs else 8)
    names, where = ["x", "y"], [offs.get("x", 0), offs.get("y", 4)]
    if zoff is not None and zoff + 4 <= step:
        names.append("z")
        where.append(zoff)
    dt = np.dtype({"names": names, "formats": [end + "f4"] * len(names), "offsets": where, "itemsize": step})
    data = m.data if isinstance(m.data, (bytes, bytearray)) else bytes(m.data)
    arr = np.frombuffer(data, dtype=dt, count=min(n, len(data) // step))
    z = arr["z"] if "z" in names else np.zeros(len(arr), np.float32)
    xyz = np.column_stack((arr["x"], arr["y"], z)).astype(np.float32)
    return m.header.frame_id, _stamp(m.header), xyz[np.isfinite(xyz).all(axis=1)]


def lidar_tag_height(pid=None) -> float | None:
    """The z every lidar return carries in the `lidar` cloud: the profile's
    lidar_height_m as the running stack reads it, with the MOSS_* overrides of
    its own environment (/proc/<pid>/environ, as bus_env reads the bus); with no
    stack, or one whose environment cannot be read, the cockpit's own. None
    when moss_dimos does not import here."""
    if robot_profile is None:
        return None
    env = proc_environ(pid) if pid else None
    return float(robot_profile.current(os.environ if env is None else env).lidar_height_m)


def scan_points(xyz: np.ndarray, tag_z: float | None) -> dict:
    """The point lists of /scan.json for one cloud (Nx3). With tag_z (a `lidar`
    cloud), the lidar returns - z within LIDAR_Z_TOL_M of tag_z, the rule of
    costmap2d.lidar_returns - go to `points` and every other point (a depth-camera
    obstacle, at its true height) to `camera`. Without tag_z (the C1 driver's
    flat cloud, or no profile) every point is a lidar return and `camera` is left
    out: this cloud says nothing about the depth camera. n and camera_n count the
    points in the cloud, before each list is capped at MAX_SCAN_POINTS."""
    if tag_z is None:
        lidar, camera = xyz[:, :2], None
    else:
        is_lidar = np.abs(xyz[:, 2].astype(np.float64) - tag_z) < LIDAR_Z_TOL_M
        lidar, camera = xyz[is_lidar, :2], xyz[~is_lidar, :2]
    doc = {"points": np.round(downsample(lidar).astype(float), 3).tolist(), "n": int(len(lidar))}
    if camera is not None:
        doc["camera"] = np.round(downsample(camera).astype(float), 3).tolist()
        doc["camera_n"] = int(len(camera))
    return doc


def downsample(points: np.ndarray, limit: int = MAX_SCAN_POINTS) -> np.ndarray:
    """At most `limit` rows, taken evenly across the array (keeps the angular spread)."""
    if len(points) <= limit:
        return points
    return points[np.linspace(0, len(points) - 1, limit).astype(int)]


def decode_path(raw: bytes) -> list[list[float]]:
    m = LcmPath.lcm_decode(raw)
    return [[round(float(p.pose.position.x), 3), round(float(p.pose.position.y), 3)] for p in m.poses]


def decode_twist(raw: bytes) -> tuple[float, float]:
    m = LcmTwist.lcm_decode(raw)
    return float(m.linear.x), float(m.angular.z)


def decode_bool(raw: bytes) -> bool:
    return bool(LcmBool.lcm_decode(raw).data)


def decode_string(raw: bytes) -> str:
    return str(LcmString.lcm_decode(raw).data)


def decode_json(raw: bytes):
    """A String message that carries JSON (the base telemetry), parsed."""
    return json.loads(decode_string(raw))


# --- messages to publish ----------------------------------------------------------
# Every nested field is built fresh: the generated dimos_lcm constructors take
# their defaults as shared instances (`pose=Pose()` evaluated once), so mutating
# a default-built message would silently change every later one in the process.

def goal_message(x: float, y: float, now: float | None = None):
    """A navigation goal the way dimOS's own web module sends a click: frame world,
    z 0, identity orientation, stamped now."""
    now = time.time() if now is None else now
    header = LcmHeader(seq=0, stamp=LcmTime(sec=int(now), nsec=int((now % 1.0) * 1e9)), frame_id="world")
    pose = LcmPose(position=LcmPoint(x=float(x), y=float(y), z=0.0),
                   orientation=LcmQuaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    return LcmPoseStamped(header=header, pose=pose)


def zero_twist():
    return LcmTwist(linear=LcmVector3(x=0.0, y=0.0, z=0.0), angular=LcmVector3(x=0.0, y=0.0, z=0.0))


# --- the base, from its telemetry JSON ------------------------------------------

def battery_pct(volts: float | None) -> int | None:
    """3S linear gauge: 10.2 V -> 0 %, 12.6 V -> 100 %, clamped."""
    if volts is None:
        return None
    frac = (volts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)
    return int(round(100.0 * min(1.0, max(0.0, frac))))


def base_view(t: dict, age_s: float | None) -> dict:
    """The /base_telemetry JSON (MossControlCoordinator, every 0.5 s) as the page shows it."""
    mv = t.get("bus_mV")
    volts = round(mv / 1000.0, 2) if isinstance(mv, (int, float)) else None
    mode = t.get("mode")
    stalled, stalls = t.get("stalled"), t.get("stall_count")
    return {
        "fw": t.get("fw"), "driver": t.get("driver"),
        "mode": mode, "mode_name": MODE_NAMES.get(mode, "unknown") if mode is not None else None,
        "reason": t.get("reason"),
        "bus_mV": mv, "battery_v": volts,
        "battery_pct": battery_pct(volts) if t.get("ina_ok", True) else None,
        "ina_ok": t.get("ina_ok"),
        "ticks": t.get("ticks"), "target_pct": t.get("target_pct"), "output_pct": t.get("output_pct"),
        "ramp_ms": t.get("ramp_ms"),
        "firmware_age_s": t.get("firmware_age_s"), "mock": t.get("mock"),
        # the base's stall guard (a commanded track whose encoder did not move for 1 s: stop,
        # counted); telemetry published before the guard existed has neither key -> None
        "stalled": None if stalled is None else bool(stalled),
        "stall_count": stalls if isinstance(stalls, int) and not isinstance(stalls, bool) else None,
        "age_s": None if age_s is None else round(age_s, 2),
    }


# --- the machine (stdlib, sysfs, /proc) ---------------------------------------------

_prev_cpu = None
_prev_cores = None
_gpu_samples: list = []
_sys_lock = threading.Lock()
_sys_cache = [0.0, None]
GPU_LOAD_PATHS = ("/sys/devices/platform/bus@0/17000000.gpu/load", "/sys/devices/gpu.0/load")


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:  # noqa: BLE001 - sysfs raises odd errors on absent sensors
        return None


def cpu_percent():
    """CPU usage since the previous call (first call: 0.2 s sample)."""
    global _prev_cpu

    def sample():
        vals = [int(x) for x in _read("/proc/stat").splitlines()[0].split()[1:]]
        return vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)

    if _prev_cpu is None:
        _prev_cpu = sample()
        time.sleep(0.2)
    idle0, total0 = _prev_cpu
    idle1, total1 = sample()
    _prev_cpu = (idle1, total1)
    dt = total1 - total0
    return round(100.0 * (1 - (idle1 - idle0) / dt), 1) if dt > 0 else 0.0


def cpu_per_core():
    """Per-core usage since the previous call: one saturated worker reads differently
    from a spread load."""
    global _prev_cores

    def sample():
        out = []
        for line in _read("/proc/stat").splitlines()[1:]:
            if not line.startswith("cpu"):
                break
            vals = [int(x) for x in line.split()[1:]]
            out.append((vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)))
        return out

    if _prev_cores is None:
        _prev_cores = sample()
        time.sleep(0.2)
    prev, cur = _prev_cores, sample()
    _prev_cores = cur
    res = []
    for (i0, t0), (i1, t1) in zip(prev, cur):
        dt = t1 - t0
        res.append(round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0)
    return res


def meminfo():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        k, v = line.split(":", 1)
        info[k] = int(v.strip().split()[0])  # kB
    total = info["MemTotal"]
    used = total - info.get("MemAvailable", info.get("MemFree", 0))
    return {"ram_used_gb": round(used / 1024**2, 1), "ram_total_gb": round(total / 1024**2, 1),
            "ram_percent": round(100.0 * used / total, 1)}


def _gpu_read_once():
    """Jetson Orin GPU load (sysfs gives tenths of a percent); None on a board without that file."""
    for path in GPU_LOAD_PATHS:
        raw = _read(path)
        if raw is not None:
            try:
                return int(raw) / 10.0
            except ValueError:
                pass
    return None


def _gpu_sampler():
    """10 Hz, 3 s average: the mapper works in 4 ms CUDA bursts that a single
    read misses (10 Hz is plenty for a page that polls at 1 Hz)."""
    while True:
        v = _gpu_read_once()
        now = time.time()
        if v is not None:
            _gpu_samples.append((now, v))
            while _gpu_samples and now - _gpu_samples[0][0] > 3.0:
                _gpu_samples.pop(0)
        time.sleep(0.1)


def gpu_percent():
    if not _gpu_samples:
        return _gpu_read_once()
    return round(sum(v for _, v in _gpu_samples) / len(_gpu_samples), 1)


def temps():
    zones = {}
    for z in glob.glob("/sys/class/thermal/thermal_zone*"):
        name = _read(os.path.join(z, "type")) or "?"
        raw = _read(os.path.join(z, "temp"))
        if raw is None:
            continue
        try:
            t = int(raw) / 1000.0
        except ValueError:
            continue
        if t <= 0:
            continue
        zones[name.replace("-thermal", "").replace("_thermal", "")] = round(t, 1)
    return zones


def disks():
    seen, out = set(), []
    for line in (_read("/proc/mounts") or "").splitlines():
        dev, mnt, fs = line.split()[:3]
        if fs not in ("ext4", "vfat", "xfs", "btrfs", "f2fs") or mnt in seen or mnt.startswith("/boot"):
            continue
        seen.add(mnt)
        try:
            st = os.statvfs(mnt)
        except OSError:
            continue
        total = st.f_blocks * st.f_frsize
        free = st.f_bavail * st.f_frsize
        if total == 0:
            continue
        out.append({"mountpoint": mnt, "fstype": fs,
                    "used_gb": round((total - free) / 1024**3, 1), "total_gb": round(total / 1024**3, 1),
                    "free_gb": round(free / 1024**3, 1), "percent": round(100.0 * (total - free) / total, 1)})
    return sorted(out, key=lambda d: d["mountpoint"])


def system_metrics() -> dict:
    """The machine, cached 1 s: several pages polling at 1 Hz cost one /proc pass."""
    with _sys_lock:
        stamp, cached = _sys_cache
        if cached is not None and time.monotonic() - stamp < SYSTEM_CACHE_S:
            return cached
        out = {"cpu_percent": cpu_percent(), "cpu_per_core": cpu_per_core(), "gpu_percent": gpu_percent()}
        try:
            out.update(meminfo())
        except Exception:  # noqa: BLE001
            pass
        out["temps"] = temps()
        out["disks"] = disks()
        out["load_1m"] = round(os.getloadavg()[0], 2)
        out["uptime_s"] = int(float((_read("/proc/uptime") or "0 0").split()[0]))
        _sys_cache[:] = [time.monotonic(), out]
        return out


# --- the stack, without asking dimOS -----------------------------------------------

def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True             # exists, not ours to signal
    except (OSError, TypeError, ValueError):
        return False


def _parse_iso(text: str) -> float | None:
    try:
        return _dt.datetime.fromisoformat(str(text).replace("Z", "+00:00")).timestamp()
    except (TypeError, ValueError):
        return None


def _parse_utc(text) -> float | None:
    """A registry time (started_at) as epoch seconds; one written without a timezone is UTC,
    as dimOS writes it, never this machine's local time."""
    try:
        t = _dt.datetime.fromisoformat(str(text).replace("Z", "+00:00"))
    except (TypeError, ValueError):
        return None
    return (t if t.tzinfo is not None else t.replace(tzinfo=_dt.timezone.utc)).timestamp()


def stack_status(now: float | None = None) -> dict:
    """The live run from dimOS's registry (dead pids skipped, never `dimos status`:
    every dimOS CLI call writes a log file and it would also prune the registry)."""
    now = time.time() if now is None else now
    best = None
    for path in glob.glob(os.path.join(runs_dir(), "*.json")):
        try:
            with open(path) as fh:
                entry = json.load(fh)
            pid = int(entry.get("pid"))
        except Exception:  # noqa: BLE001 - half-written or foreign file
            continue
        if not _pid_alive(pid):
            continue
        started = _parse_utc(entry.get("started_at"))
        if best is None or (started or 0) > (best[0] or 0):
            best = (started, pid, entry)
    if best is None:
        return {"running": False}
    started, pid, entry = best
    blueprint = str(entry.get("blueprint", ""))
    return {"running": True, "run_id": entry.get("run_id"),
            "blueprint": blueprint.rsplit(".", 1)[-1], "pid": pid,
            "started_at": entry.get("started_at"),
            "uptime_s": round(now - started, 1) if started else None,
            "log_dir": entry.get("log_dir")}


def proc_cmdline(pid) -> list[bytes] | None:
    """The arguments process `pid` runs with (/proc/<pid>/cmdline), None when it is gone."""
    try:
        with open(f"/proc/{int(pid)}/cmdline", "rb") as fh:
            return fh.read().split(b"\0")
    except (OSError, TypeError, ValueError):
        return None


def is_dimos_run(argv) -> bool:
    """argv (a /proc cmdline) is a `dimos run`: an argument ending in bin/dimos, then "run" among
    the arguments after it. tools/moss-fly.sh's STACK_RE ('[b]in/dimos( [^ ]+)* run( |$)') reads
    a command line the same way."""
    argv = list(argv or [])
    return any(a.endswith(b"bin/dimos") and b"run" in argv[i + 1:] for i, a in enumerate(argv))


def _dimos_run_pid() -> int | None:
    """A `dimos run` process even without a registry entry (a foreground run, a
    registry swept by someone's `dimos status`). TOCTOU-safe: a process that exits
    between the listing and the read is skipped."""
    try:
        pids = os.listdir("/proc")
    except OSError:
        return None
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                argv = fh.read().split(b"\0")
        except OSError:
            continue
        if is_dimos_run(argv):
            return int(pid)
    return None


def proc_environ(pid) -> dict | None:
    """The environment process `pid` started with (/proc/<pid>/environ), or None
    when it cannot be read (gone, another user's, no pid)."""
    try:
        with open(f"/proc/{int(pid)}/environ", "rb") as fh:
            raw = fh.read()
    except (OSError, TypeError, ValueError):
        return None
    env = {}
    for item in raw.split(b"\0"):
        key, sep, value = item.partition(b"=")
        if sep:
            env[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    return env


BUS_VARS = ("DIMOS_TRANSPORT", "TRANSPORT",   # what dimOS picks its bus from (explore_ctl reads them)
            "LCM_DEFAULT_URL")                # and the LCM group: the cockpit and the stack share all three


def bus_env(pid) -> tuple[dict, str]:
    """The variables that chose the bus, and whose they are. A running stack read
    DIMOS_TRANSPORT / TRANSPORT from its own environment when it started, so that
    one decides: a cockpit run by systemd shares nothing with a stack started from
    a shell. With no stack, or one whose environment cannot be read, the cockpit's
    own: what a stack launched from the page inherits."""
    env = proc_environ(pid) if pid else None
    whose = f"the running stack's environment, pid {pid}"
    if env is None:
        env = os.environ
        whose = "the cockpit's own environment" + (", the stack's is unreadable" if pid else "")
    return {k: env[k] for k in BUS_VARS if k in env}, whose


def bus_transport(env) -> str:
    """The bus a stack with environment `env` runs on, in dimOS's order (explore_ctl's rule)."""
    try:
        import explore_ctl
        return explore_ctl.transport(env)
    except Exception:  # noqa: BLE001 - no dimos_lcm: the same order as explore_ctl.transport
        return (env.get("DIMOS_TRANSPORT") or env.get("TRANSPORT") or "lcm").strip().lower()


# --- the idle stop: a stack nobody uses drains the battery ------------------------------
#
# With the explore stack up and the rover still, the lidar odometry, the costmap and the
# camera keep one core of the board at 100 %. The cockpit stops a MOSS stack that saw no
# activity for MOSS_IDLE_STOP_S seconds, as the page's Stop stack button does.

def idle_limit_s(env=None) -> float:
    """MOSS_IDLE_STOP_S from the cockpit's environment (the service's), read once at start:
    the seconds a MOSS stack may stay idle, 0 = never stopped. Unset: the default; not a
    number of seconds >= 0, or under IDLE_STOP_MIN_S: the default, and the cockpit says so."""
    env = os.environ if env is None else env
    raw = (env.get("MOSS_IDLE_STOP_S") or "").strip()
    if not raw:
        return IDLE_STOP_DEFAULT_S
    try:
        value = float(raw)
    except ValueError:
        value = -1.0
    if not (math.isfinite(value) and value >= 0):
        print(f"moss_cockpit: MOSS_IDLE_STOP_S={raw!r} is not a number of seconds >= 0: "
              f"{IDLE_STOP_DEFAULT_S:g} s", file=sys.stderr, flush=True)
        return IDLE_STOP_DEFAULT_S
    if 0 < value < IDLE_STOP_MIN_S:
        print(f"moss_cockpit: MOSS_IDLE_STOP_S={raw!r} is under {IDLE_STOP_MIN_S:g} s (0 turns the stop off): "
              f"{IDLE_STOP_DEFAULT_S:g} s", file=sys.stderr, flush=True)
        return IDLE_STOP_DEFAULT_S
    return value


def is_moss_stack(stack) -> bool:
    """A running stack whose blueprint is a MOSS one: the only kind the cockpit commands or stops."""
    return bool(stack and stack.get("running")) and str(stack.get("blueprint") or "").startswith("moss")


def idle_stoppable(stack) -> bool:
    """A running stack the idle stop may stop: moss-nav or moss-explore, whose lidar odometry and
    costmap keep the board busy. moss-gamepad stays up with the pad on the table, as
    tools/moss-teleop.sh promises, and so does any other blueprint."""
    return is_moss_stack(stack) and str(stack.get("blueprint") or "") in IDLE_STOP_BLUEPRINTS


_FROM_STACK = object()      # idle_state: the start is the stack's started_at


def idle_state(stack, activity_ts, now: float, limit_s: float, started=_FROM_STACK) -> dict:
    """The idle rule, pure. idle_s: seconds since the last activity, or since the stack's start
    when nothing happened since (activity before the start never counts); None with no running
    stack or no time to count from. `started`: the stack's start on the clock of `now` and
    `activity_ts` (the cockpit passes the monotonic clock); left out, the stack's started_at
    as epoch seconds (UTC when it has no timezone). stop_in_s: limit_s - idle_s floored at 0,
    for a stack the stop may stop (idle_stoppable) with the rule on, else None. due: the rule
    is on (limit_s > 0), the stack is one of those and idle_s >= limit_s."""
    enabled = limit_s > 0
    running = bool(stack and stack.get("running"))
    if started is _FROM_STACK:
        started = _parse_utc(stack.get("started_at")) if running else None
    marks = [t for t in (activity_ts, started)
             if isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t)]
    idle = max(0.0, now - max(marks)) if running and marks else None
    counted = enabled and idle is not None and idle_stoppable(stack)
    return {"enabled": enabled, "idle_s": None if idle is None else round(idle, 1),
            "stop_in_s": round(max(0.0, limit_s - idle), 1) if counted else None,
            "limit_s": limit_s, "due": counted and idle >= limit_s}


def idle_stop_message(limit_s: float) -> str:
    """The line the cockpit says when it stops an idle stack: the limit in minutes, in seconds under one."""
    span = f"{limit_s:g} s" if limit_s < 60 else f"{round(limit_s / 60.0, 1):g} min"
    return f"{IDLE_STOP_TEXT} (idle {span})"


def motors_commanded(output_pct) -> bool:
    """Telemetry output_pct [left, right] with a track commanded (anything but [0, 0]): the pad,
    the planner and the explorer all end there."""
    return isinstance(output_pct, (list, tuple)) and any(
        isinstance(v, (int, float)) and not isinstance(v, bool) and v != 0 for v in output_pct)


def telemetry_fresh(t: dict) -> bool:
    """A telemetry line whose firmware line is at most IDLE_FIRMWARE_AGE_S old. With the board
    lost, the coordinator keeps publishing its last line (firmware_age_s grows): its output_pct
    is old news. No age at all counts as fresh: a commanded track is never missed for it."""
    age = t.get("firmware_age_s")
    return not (isinstance(age, (int, float)) and not isinstance(age, bool) and age > IDLE_FIRMWARE_AGE_S)


def pose_moved(ref: dict, pose: dict, move_m: float = IDLE_MOVE_M, turn_rad: float = IDLE_TURN_RAD) -> bool:
    """The pose is more than move_m from ref, or turned more than turn_rad from it."""
    turned = abs(math.remainder(pose["yaw"] - ref["yaw"], math.tau))
    return math.hypot(pose["x"] - ref["x"], pose["y"] - ref["y"]) > move_m or turned > turn_rad


def log_tail(log_dir: str | None, n: int) -> list[dict]:
    """The last n records of <log_dir>/main.jsonl as {ts, level, event, logger}."""
    if not log_dir:
        return []
    path = os.path.join(log_dir, "main.jsonl")
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - 262144))
            chunk = fh.read()
    except OSError:
        return []
    out = []
    for line in reversed(chunk.splitlines()):     # newest first, until n records parse
        if len(out) >= n:
            break
        try:
            rec = json.loads(line)
        except ValueError:
            continue                              # a half-written last line, a stray print
        if isinstance(rec, dict):
            out.append({"ts": rec.get("timestamp"), "level": rec.get("level"),
                        "event": rec.get("event"), "logger": rec.get("logger")})
    return out[::-1]


# --- what it decides: the stack's own log lines, read as decisions --------------------
#
# The explorer, the planners, the base adapter, the costmap and the contact sources
# all log through dimOS's logger into the run's main.jsonl. Each line below is one
# of their decisions, matched on its real text; every other line (twists,
# telemetry, camera counts, transport, workers) is ignored. The page shows the
# result as a feed and one state word.

DECISIONS_MIN_INTERVAL_S = 0.5   # main.jsonl read for decisions at most 2 Hz, whoever asks
DECISIONS_TAIL_BYTES = 1 << 20   # a first read takes the last MiB: thousands of records, past the twists
DECISIONS_KEEP = 500             # decision rows kept per run (the page shows 30)
DECISION_ECHO_S = 1.0            # a generic line this soon after the specific one it repeats is not a new decision
AFTER_STOP_ECHO_S = 0.2          # the explorer's stay-here goal and its arrival land within ~60 ms of its stop
REASON_CONFIRM_S = 0.5           # a replan reason with no consequence logged this soon was not acted on
OBSTACLE_HOLD_S = 3.0            # OBSTACLE (a stop on an obstacle, a replan) and the planner's STUCK stay up this long
CONTACT_HOLD_S = 4.0             # STUCK for frozen tracks (the base pushing harder) stays up this long
ESCAPE_TIMEOUT_S = 5.0           # recovering_planner's: an escape that never covers its distance ends after this
ESCAPE_DISTANCE_M = 0.20         # recovering_planner's: an escape that ended short of this timed out
ESCAPE_HOLD_S = ESCAPE_TIMEOUT_S + 1.0   # BACKING OFF lasts until the escape's end line, at most this (a lost line)
STALL_MIN_TICKS = 20             # moss_adapter's: fewer ticks over the stall window = that track did not move
DECISION_WORDS = ("IDLE", "READY", "HEADING", "OBSTACLE", "BACKING OFF", "STUCK", "WAITING", "DONE", "STOPPED")
HOLD_RANK = {"OBSTACLE": 1, "STUCK": 2, "BACKING OFF": 3}   # several holds running: the highest rank shows
RECORD_FIELDS = ("acted", "aborted", "travelled_m", "seconds", "trigger", "x", "y")   # what a decision reads
                                                                                     # beyond the event text

_N = r"([-+]?(?:\d+(?:\.\d+)?|nan|inf))"     # a number as the line prints it, kept as printed
_P = r"([-+]?\d+)"                           # a signed track percent or tick count

TXT_ESCAPE = "Escape "                       # every end of an escape: done, timed out, aborted
TXT_ESCAPE_DONE = "Escape done"
TXT_STARTED = "Exploration started"
TXT_STOPPED = "Exploration stopped"
TXT_COMPLETE = "Exploration complete"
TXT_REPLAN = "Replanning"
TXT_STUCK = "Stuck: replanning"
TXT_OFF_TRACK = "Off track: replanning"
TXT_NAV_FAILED = "Navigation failed: replanning"
QUIET_REPLANS = (TXT_OFF_TRACK, TXT_NAV_FAILED)   # the rover keeps heading for its goal: no hold
TXT_ABANDONED = "Stuck: goal abandoned"
TXT_NO_PATH = "Goal dropped: no path"
TXT_NO_SAFE = "Goal dropped: no safe spot near it"
TXT_PLANNER_DROPS = "Goal dropped: no "              # the planner's own reasons for dropping a goal
TXT_CORNERED = "Cornered"
TXT_PLANNER_ECHOES = ("Planner: new goal", "Planner: safe goal found")   # the detail keeps the explorer's goal line
TXT_NO_LOG = "Stack running, its log is not found"
_FOLD = re.compile(r" \(x\d+\)$")             # the count a folded run of identical rows carries


def _num(rec, key) -> float | None:
    """A record field as a finite number, else None."""
    value = rec.get(key) if isinstance(rec, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return float(value)


def _stall_text(m, rec=None) -> str:
    n, cmd_l, cmd_r, enc_l, enc_r, secs = m.groups()
    blocked = [side for side, cmd, enc in (("left", cmd_l, enc_l), ("right", cmd_r, enc_r))
               if int(cmd) != 0 and abs(int(enc)) < STALL_MIN_TICKS]
    which = "both tracks" if len(blocked) == 2 else f"{blocked[0]} track" if blocked else "tracks"
    return (f"Contact #{n}: {which} blocked (commanded L={cmd_l}% R={cmd_r}%, "
            f"encoders moved L={enc_l} R={enc_r} ticks in {secs} s)")


def _pivot_text(m, rec=None) -> str:
    cmd_l, cmd_r, side = m.groups()
    cmd = cmd_r if side == "right" else cmd_l
    other = "left" if side == "right" else "right"
    way = "reverse" if int(cmd) < 0 else "forward"
    return (f"{side.capitalize()} track did not turn in {way} (L={cmd_l}% R={cmd_r}%): "
            f"pivoting on the {other} track from now on")


def _bump_text(m, rec) -> str | None:
    """'BUMP received' with acted=False is a contact during an escape: the line before it
    ('Contact during an escape: stopping dead') already says so, and nothing backs off."""
    return None if rec.get("acted") is False else "Bump: backing off"


def _escape_text(m, rec) -> str:
    """The end of an escape, from the record's fields (aborted, travelled_m, seconds)."""
    travelled, seconds = _num(rec, "travelled_m"), _num(rec, "seconds")
    took = f" in {seconds:.1f} s" if seconds is not None else ""
    if rec.get("aborted") is True:
        return "Escape aborted" + (f" after {travelled:.2f} m" if travelled is not None else "")
    if travelled is None:
        return TXT_ESCAPE_DONE
    if travelled < ESCAPE_DISTANCE_M - 0.005:
        return f"Escape timed out: only {travelled:.2f} m{took}"
    return f"{TXT_ESCAPE_DONE}: {travelled:.2f} m{took}"


def _abandon_text(m, rec) -> str:
    trigger = rec.get("trigger")
    if trigger is None or trigger == "stuck_detector":
        return TXT_ABANDONED
    return f"Goal abandoned ({str(trigger)[:40]})"


def _no_path_text(m, rec) -> str:
    x, y = _num(rec, "x"), _num(rec, "y")
    return TXT_NO_PATH + (f" to ({x:.2f}, {y:.2f})" if x is not None and y is not None else "")


def _metres(text: str) -> str:
    try:
        return f"{float(text):.2f}"
    except ValueError:
        return text


# (kind, src, pattern on the event text, the short English text or a function of the
# match and the record (None: no event), echo, role).
#
# `echo` lists (kind, text prefix or None[, window s]) entries: when the previous
# decision is one of them and at most the window (DECISION_ECHO_S by default) older,
# this line only repeats it and is not a new event (the planner taking the
# explorer's goal, the second of the two stop lines a STOP produces, the explorer's
# give-up after the planner's own reason, the explorer's "stay here" goal right
# after it stops).
#
# `role` ties the global planner's replan lines together. Its monitor and its stop
# handler log a REASON ("Robot is stuck. Replanning.", "Replanning path due to
# obstacle found.", "Robot veered off track. Replanning.", "Failure in
# navigation.") and then call _replan_path, which decides: "Replanning." when it
# really replans (logged by dimOS's _replan_path), "Goal abandoned, no recovery
# motion" when MOSS's planner gives the goal up, "Replan requested without a goal;
# ignoring.", or nothing at all (MOSS's planner ignores a stuck verdict while the
# follower turns in place, is idle or a contact escape owns the wheels). So a
# reason is held, not shown; the "Replanning." that follows within
# REASON_CONFIRM_S ("confirm") shows it, an abandon ("resolve") replaces it, and
# anything else ("clear", or nothing in time) drops it: the planner did not act on it.
_AFTER_STOP = ("mission", TXT_STOPPED)
_AFTER_STOP_GOAL = ("mission", TXT_STOPPED, AFTER_STOP_ECHO_S)
DECISION_CATALOGUE = (
    ("goal", "explorer",
     r"goal (\d+): \(" + _N + r", " + _N + r"\) " + _N + r" m away( \(beyond the cap[^)]*\))?, (\d+) frontier cells",
     lambda m, r: f"Goal {m[1]}: {m[4]} m away ({m[6]} cells{', beyond the cap' if m[5] else ''})", None, None),
    ("goal_dropped", "explorer", r"planner gave up on that goal\b", "Goal dropped: the planner gave up",
     (("goal_dropped", TXT_PLANNER_DROPS), ("goal_dropped", TXT_ABANDONED)), None),
    ("goal_dropped", "explorer", r"goal timeout after " + _N + r" s having closed " + _N + r" m of " + _N + r" m",
     lambda m, r: f"Goal dropped: {m[2]} of {m[3]} m closed in {m[1]} s", None, None),
    ("waiting", "explorer", r"(\d+) of (\d+) clusters are on a recently failed goal",
     lambda m, r: f"Waiting: {m[1]} of {m[2]} frontiers are failed goals", None, None),
    ("retry", "explorer", r"starvation escape:.*reopening the oldest failed goal at \(" + _N + r", " + _N + r"\)",
     lambda m, r: f"Retrying an old goal at ({m[1]}, {m[2]})", None, None),
    ("obstacle", "explorer",
     r"born cornered: " + _N + r" m2 of reachable floor and no frontier - one " + _N + r" m back-off",
     lambda m, r: f"{TXT_CORNERED} ({m[1]} m2 of floor, no frontier): backing off {m[2]} m", None, None),
    ("obstacle", "planner", r"Obstacle detected ahead, stopping local planner", "Obstacle ahead: stop", None, None),
    ("replan", "planner", r"Replanning path due to obstacle found", TXT_REPLAN, None, "reason"),
    ("replan", "planner", r"Robot is stuck\. Replanning", TXT_STUCK, None, "reason"),
    ("replan", "planner", r"Robot veered off track\. Replanning", TXT_OFF_TRACK, None, "reason"),
    ("replan", "planner", r"Failure in navigation\b", TXT_NAV_FAILED, None, "reason"),
    ("replan", "planner", r"Replanning\.?$", TXT_REPLAN, (("obstacle", None), ("replan", None)), "confirm"),
    (None, None, r"Replan requested without a goal", None, None, "clear"),
    ("goal_dropped", "recovery", r"Goal abandoned, no recovery motion", _abandon_text, None, "resolve"),
    ("goal_dropped", "planner", r"No path found to the goal\b", _no_path_text, None, None),
    ("goal_dropped", "planner", r"No safe goal found\b", TXT_NO_SAFE, (("goal_dropped", TXT_NO_SAFE),), None),
    ("new_goal", "planner", r"Got new goal\b", TXT_PLANNER_ECHOES[0],
     (("goal", None), ("retry", None), _AFTER_STOP_GOAL), None),
    ("new_goal", "planner", r"Found safe goal\b", TXT_PLANNER_ECHOES[1],
     (("goal", None), ("new_goal", None), ("obstacle", None), ("replan", None), _AFTER_STOP_GOAL), None),
    ("new_goal", "planner", r"Travelling to goal " + _N + r" ?m away from requested goal",
     lambda m, r: f"Planner: safe goal {_metres(m[1])} m from the requested one", None, None),
    ("reached", "planner", r"Reached goal position\b", "Goal reached", (_AFTER_STOP_GOAL,), None),
    ("reached", "planner", r"Close enough to goal\. Accepting as arrived", "Goal reached (close enough)",
     (_AFTER_STOP_GOAL,), None),
    ("contact", "base",
     r"(?:MOSS base: )?STALL #(\d+): tracks commanded L=" + _P + r"% R=" + _P + r"% but the encoders moved L="
     + _P + r" R=" + _P + r" ticks in " + _N + r" s", _stall_text, None, None),
    ("contact", "recovery", r"BUMP received\b", _bump_text, (("obstacle", TXT_CORNERED),), None),
    ("contact", "recovery", r"Contact during an escape: stopping dead", "Contact during an escape: stop dead",
     None, None),
    ("contact", "recovery", r"Contact escape$", _escape_text, None, None),
    ("contact", "costmap",
     r"costmap: contact at \(" + _N + r", " + _N + r"\) heading " + _N + r" deg: (\d+) cells ahead marked as a wall",
     lambda m, r: f"Wall remembered ahead ({m[4]} cells)", None, None),
    ("push", "base", r"(?:MOSS base: )?tracks frozen at L=" + _P + r"% R=" + _P + r"% - pushing at (\d+) %",
     lambda m, r: f"Tracks frozen at L={m[1]}% R={m[2]}%: pushing at {m[3]} %", None, None),
    ("pivot_fallback", "base",
     r"(?:MOSS base: )?pivot L=" + _P + r"% R=" + _P + r"%: the (right|left) track did not turn", _pivot_text,
     None, None),
    ("imu", "imu", r"IMU contact\b.*", lambda m, r: m[0][:120], None, None),
    ("mission", "mission", r"exploration start: .*?cap " + _N + r" m, radius " + _N + r" m",
     lambda m, r: f"{TXT_STARTED} (cap {m[1]} m, radius {m[2]} m)", None, None),
    ("mission", "mission", r"exploration start\b", TXT_STARTED, None, None),
    ("mission", "mission", r"(?:\w+: )?stop_movement received, stopping exploration",
     f"{TXT_STOPPED} (stop button or pad)", (_AFTER_STOP,), None),
    ("mission", "mission", r"Received exploration stop command\b", f"{TXT_STOPPED} (stop command)",
     (_AFTER_STOP,), None),
    ("mission", "mission", r"Stopped autonomous frontier exploration", TXT_STOPPED, (_AFTER_STOP,), None),
    ("mission", "mission", r"exploration complete: no reachable frontier left.*?\((\d+) targets",
     lambda m, r: f"{TXT_COMPLETE}: no reachable frontier left ({m[1]} goals)", None, None),
    ("mission", "mission", r"[Ee]xploration complete after (\d+) goals",
     lambda m, r: f"{TXT_COMPLETE} ({m[1]} goals)", None, None),
    ("mission", "mission", r"exploration complete\b", TXT_COMPLETE, None, None),
)
_CATALOGUE = tuple((kind, src, re.compile(pattern), text, echo, role)
                   for kind, src, pattern, text, echo, role in DECISION_CATALOGUE)


def _epoch(ts) -> float | None:
    """A record's time as epoch seconds: a number as is, an ISO string (dimOS writes
    UTC with a Z) through fromisoformat; anything else None."""
    if isinstance(ts, bool):
        return None
    if isinstance(ts, (int, float)):
        return float(ts) if math.isfinite(ts) else None
    if isinstance(ts, str):
        t = _parse_iso(ts)
        if t is None:
            try:
                t = float(ts)
            except ValueError:
                return None
        return t if math.isfinite(t) else None
    return None


def _record_ts(rec: dict):
    """The record's own time as given (ISO string or epoch), None when it is not a time."""
    ts = rec.get("ts", rec.get("timestamp"))
    if (isinstance(ts, bool) or not isinstance(ts, (int, float, str))
            or (isinstance(ts, float) and not math.isfinite(ts))):
        return None
    return ts


def decide(record) -> tuple | None:
    """(kind, src, text, echo, role) for one log record whose event is a decision line,
    else None. A "clear" line has no kind, src or text."""
    if not isinstance(record, dict):
        return None
    event = record.get("event")
    if not isinstance(event, str):
        return None
    event = event.strip()
    for kind, src, rx, text, echo, role in _CATALOGUE:
        m = rx.match(event)
        if m:
            said = text(m, record) if callable(text) else text
            if said is None and role != "clear":
                return None
            return kind, src, said, echo, role
    return None


def _is_echo(echo, last: dict | None, ts) -> bool:
    if not echo or last is None:
        return False
    a, b = _epoch(last.get("ts")), _epoch(ts)
    if a is None or b is None:
        return False
    text = str(last.get("text", ""))
    for item in echo:
        kind, prefix = item[0], item[1]
        window = item[2] if len(item) > 2 else DECISION_ECHO_S
        if last.get("kind") == kind and (not prefix or text.startswith(prefix)) and abs(b - a) <= window:
            return True
    return False


def decisions_from_records(records, seq0: int = 0, prev: dict | None = None, carry: dict | None = None) -> list[dict]:
    """Log records ({ts, level, event, logger} plus any RECORD_FIELDS, oldest first, as
    log_tail returns them) -> decision events {ts, kind, text, src, seq}, in the same
    order. `ts` is the record's own (an ISO string or epoch seconds); `seq` counts from
    seq0 + 1; `prev` is the last event already emitted (for the echo rule across reads);
    `carry` is a dict the caller keeps between reads: a replan reason still waiting for
    its consequence when the records ran out stays in it (read and updated in place).
    Never raises: a record that is not a dict, has no event or an unknown one is skipped."""
    out: list[dict] = []
    last = prev if isinstance(prev, dict) else None
    pending = carry.get("pending") if isinstance(carry, dict) else None
    seq = seq0
    try:
        items = list(records or ())
    except Exception:  # noqa: BLE001 - not a list of records at all
        items = []
    for rec in items:
        try:
            if pending is not None and isinstance(rec, dict):
                t = _epoch(rec.get("ts", rec.get("timestamp")))
                if t is not None and pending["t"] is not None and t - pending["t"] > REASON_CONFIRM_S:
                    pending = None               # the planner did not act on that reason
            hit = decide(rec)
            if hit is None:
                continue
            kind, src, text, echo, role = hit
            ts = _record_ts(rec)
            if role == "clear":
                pending = None
                continue
            if role == "reason":
                pending = {"ts": ts, "t": _epoch(ts), "kind": kind, "src": src, "text": text}
                continue
            if role == "confirm" and pending is not None:
                kind, src, text, ts = pending["kind"], pending["src"], pending["text"], pending["ts"]
                pending = None
            else:
                if role == "resolve":
                    pending = None
                if _is_echo(echo, last, ts):
                    continue
        except Exception:  # noqa: BLE001 - one odd record never costs the feed
            continue
        seq += 1
        last = {"ts": ts, "kind": kind, "text": text, "src": src, "seq": seq}
        out.append(last)
    if isinstance(carry, dict):
        carry["pending"] = pending
    return out


def now_state(events, stack_running: bool, exploring: bool | None, now_ts: float) -> dict:
    """The state word for the page, from the decision events (oldest first).

    The last decision wins, with holds. OBSTACLE_HOLD_S: an obstacle stop or a replan
    around an obstacle -> OBSTACLE; the planner's stuck replan or its giving the goal up
    when stuck -> STUCK. Frozen tracks (the base pushing harder) -> STUCK for
    CONTACT_HOLD_S. A contact (a stall, the bump, the wall it leaves, an IMU contact) or
    the explorer backing out of a corner -> BACKING OFF until the escape's end line,
    at most ESCAPE_HOLD_S. While several holds run, the highest rank shows (BACKING OFF
    over STUCK over OBSTACLE, the latest among equals); the end of an escape or of the
    mission ends them all. Outside a hold the word is what the other events said: goal
    / retry / new goal / a replan that keeps the goal (off track, a navigation failure)
    -> HEADING; waiting / the explorer dropping a goal -> WAITING; exploration started
    -> WAITING (the explorer picks a goal), stopped -> STOPPED, complete -> DONE. A goal
    reached, the planner dropping a goal and the end of an escape -> HEADING / WAITING /
    WAITING while the explorer runs (its own lines since the last start say so, or
    `exploring` from the bus), READY otherwise: a tapped goal is over. An escape that
    ends after a STOP leaves STOPPED. No stack -> IDLE; a stack and no decision yet ->
    READY. Returns {word, detail, since}: since = the ts of the event that set the
    word, detail = the goal line or that event's text."""
    if not stack_running:
        return {"word": "IDLE", "detail": "No stack running", "since": None}
    base = None                      # (word, detail, ts) said by the last event that is not a hold
    holds: list = []                 # (rank, start epoch, end epoch, word, detail, ts)
    last_hold = None                 # the last hold event, for its text once every hold ran out
    goal_text = None                 # the explorer's line for the goal being driven to
    explorer_on = False              # the explorer's own lines say it runs
    seen = False
    for ev in events or ():
        if not isinstance(ev, dict):
            continue
        seen = True
        kind, ts, src = ev.get("kind"), ev.get("ts"), ev.get("src")
        shown = str(ev.get("text") or "")
        text = _FOLD.sub("", shown)
        on = explorer_on or exploring is True
        new_hold = None
        if kind == "obstacle":
            if text.startswith(TXT_CORNERED):
                explorer_on = True
                new_hold = ("BACKING OFF", ESCAPE_HOLD_S)
            else:
                new_hold = ("OBSTACLE", OBSTACLE_HOLD_S)
        elif kind == "replan":
            if text in QUIET_REPLANS:
                base = ("HEADING", shown, ts)
            else:
                new_hold = ("STUCK" if text.startswith(TXT_STUCK) else "OBSTACLE", OBSTACLE_HOLD_S)
        elif kind == "contact" and text.startswith(TXT_ESCAPE):
            holds, goal_text = [], None
            if base is None or base[0] not in ("STOPPED", "DONE"):
                base = ("WAITING" if on else "READY", shown, ts)
        elif kind in ("contact", "imu"):
            new_hold = ("BACKING OFF", ESCAPE_HOLD_S)
        elif kind == "push":
            new_hold = ("STUCK", CONTACT_HOLD_S)
        elif kind in ("goal", "retry"):
            explorer_on = True
            goal_text = shown if kind == "goal" else goal_text
            base = ("HEADING", shown, ts)
        elif kind == "new_goal":
            base = ("HEADING", goal_text if goal_text and text in TXT_PLANNER_ECHOES else shown, ts)
        elif kind == "reached":
            goal_text = None
            base = ("HEADING" if on else "READY", shown, ts)
        elif kind == "waiting":
            explorer_on, goal_text = True, None
            base = ("WAITING", shown, ts)
        elif kind == "goal_dropped":
            goal_text = None
            if src in ("planner", "recovery"):
                base = ("WAITING" if on else "READY", shown, ts)
                if text.startswith(TXT_ABANDONED):
                    new_hold = ("STUCK", OBSTACLE_HOLD_S)
            else:
                explorer_on = True
                base = ("WAITING", shown, ts)
        elif kind == "mission":
            holds, goal_text = [], None
            explorer_on = text.startswith(TXT_STARTED)
            base = ("STOPPED" if text.startswith(TXT_STOPPED) else "DONE" if text.startswith(TXT_COMPLETE)
                    else "WAITING", shown, ts)
        if new_hold is not None:
            word, dur = new_hold
            last_hold = (word, shown, ts)
            t = _epoch(ts)
            if t is not None:            # a hold needs a time to end
                holds.append((HOLD_RANK[word], t, t + dur, word, shown, ts))
    if not seen:
        return {"word": "READY", "detail": "Stack running, no decision yet", "since": None}
    active = [h for h in holds if h[1] <= now_ts < h[2]]
    if active:
        _, _, _, word, detail, since = max(active, key=lambda h: (h[0], h[1]))
        return {"word": word, "detail": detail, "since": since}
    if base is not None:
        return {"word": base[0], "detail": base[1], "since": base[2]}
    return {"word": "WAITING" if explorer_on or exploring is True else "READY",
            "detail": last_hold[1] if last_hold else "Stack running, no decision yet",
            "since": last_hold[2] if last_hold else None}


def decisions_doc(events, n: int, stack_running: bool, exploring: bool | None, now_ts: float, run) -> dict:
    """/decisions.json: the state word now, the last n events, the run they belong to.
    With no stack running, `run` is the last run whose events are kept."""
    events = list(events or ())
    now = now_state(events, stack_running, exploring, now_ts)
    if not stack_running and run:
        now["detail"] = f"No stack running (last run: {run})"
    return {"now": now, "events": events[-n:] if n > 0 else [], "run": run}


class DecisionLog:
    """Decision rows in arrival order; `seq` never goes back, even across a restart.
    Identical events one after the other (same kind and text: the explorer saying it
    waits every second) make one row whose text ends in ' (xN)', with the last one's
    ts and seq, so a long wait never pushes the contacts out of the page's 30 rows."""

    def __init__(self, keep: int = DECISIONS_KEEP):
        self.events: deque = deque(maxlen=keep)
        self.seq = 0
        self._carry: dict = {}           # a replan reason waiting for its consequence across reads
        self._fold = None                # (kind, text, count) of the last row

    def add(self, records) -> None:
        new = decisions_from_records(records, self.seq, self.events[-1] if self.events else None, self._carry)
        for ev in new:
            self.seq = ev["seq"]
            fold = self._fold
            if fold is not None and self.events and fold[:2] == (ev["kind"], ev["text"]):
                count = fold[2] + 1
                self.events[-1] = dict(ev, text=f"{ev['text']} (x{count})")   # a new dict: readers hold the old one
                self._fold = (ev["kind"], ev["text"], count)
            else:
                self.events.append(ev)
                self._fold = (ev["kind"], ev["text"], 1)

    def clear(self) -> None:
        self.events.clear()
        self._carry, self._fold = {}, None


class DecisionFeed(DecisionLog):
    """The decision events of one run's main.jsonl, read incrementally and at most
    every DECISIONS_MIN_INTERVAL_S. The first read takes the last DECISIONS_TAIL_BYTES;
    later reads only the bytes appended since (a half-written last line waits for the
    next read). Another file (a new run, a rotation, a truncation) or more than the
    window appended since the last read starts over from the tail: the feed is always
    one unbroken stretch of the log. A read without a log (the stack stopped) keeps the
    last run's events and `run`; the next run's log starts a new feed."""

    def __init__(self, tail_bytes: int = DECISIONS_TAIL_BYTES, keep: int = DECISIONS_KEEP,
                 min_interval_s: float = DECISIONS_MIN_INTERVAL_S):
        super().__init__(keep)
        self._tail_bytes = tail_bytes
        self._min_interval_s = min_interval_s
        self._lock = threading.Lock()
        self._path = None
        self._ident = None
        self._offset = None
        self._partial = b""
        self._read_at = None
        self.run = None                  # the run the events belong to

    def _restart(self, path, ident=None) -> None:
        self.clear()
        self._path, self._ident, self._offset, self._partial = path, ident, None, b""

    def read(self, log_dir: str | None, run=None) -> list[dict]:
        """The events of <log_dir>/main.jsonl (oldest first); without a log_dir, the
        events already read, as they are."""
        path = os.path.join(log_dir, "main.jsonl") if log_dir else None
        with self._lock:
            if path is None:
                return list(self.events)
            if path != self._path:
                self._restart(path)
                self._read_at = None
                self.run = run
            now = time.monotonic()
            if self._read_at is not None and now - self._read_at < self._min_interval_s:
                return list(self.events)
            self._read_at = now
            try:
                self._read_new(path)
            except OSError:
                pass                     # gone or unreadable for now: keep what was read
            return list(self.events)

    def _read_new(self, path: str) -> None:
        with open(path, "rb") as fh:
            st = os.fstat(fh.fileno())
            ident = (st.st_dev, st.st_ino)
            size = st.st_size
            if (ident != self._ident or self._offset is None or size < self._offset
                    or size - self._offset > self._tail_bytes):
                self._restart(path, ident)
                start = max(0, size - self._tail_bytes)
                drop_first = start > 0           # the window starts inside a line
            else:
                start, drop_first = self._offset, False
            fh.seek(start)
            chunk = fh.read(size - start)
        self._offset = start + len(chunk)
        lines = (self._partial + chunk).split(b"\n")
        self._partial = lines.pop()              # b"" after a final newline, else a line still being written
        if len(self._partial) > self._tail_bytes:
            self._partial = b""
        if drop_first and lines:
            lines = lines[1:]
        records = []
        for line in lines:
            try:
                rec = json.loads(line)
            except ValueError:
                continue                         # a stray print, a torn line
            if isinstance(rec, dict):
                out = {"ts": rec.get("timestamp"), "level": rec.get("level"),
                       "event": rec.get("event"), "logger": rec.get("logger")}
                out.update((k, rec[k]) for k in RECORD_FIELDS if k in rec)
                records.append(out)
        self.add(records)


def preflight_doc() -> dict:
    """The last flight check moss-fly.sh saved (moss_preflight.py --json)."""
    path = os.path.join(state_dir(), "preflight.json")
    try:
        with open(path) as fh:
            doc = json.load(fh)
    except FileNotFoundError:
        return {"available": False}
    except (OSError, ValueError) as exc:
        return {"available": False, "error": f"{path}: {exc}"}
    if not isinstance(doc, dict):
        return {"available": False, "error": f"{path}: not a JSON object"}
    doc["available"] = True
    doc["mtime"] = os.path.getmtime(path)
    return doc


def recording_view(episode: dict | None, closed: dict | None = None) -> dict:
    """`closed`: the episode the cockpit closed itself because the stack stopped under it."""
    db = os.path.join(state_dir(), "recordings", "moss.db")
    size = 0
    for p in (db, db + "-wal", db + "-shm"):
        try:
            size += os.path.getsize(p)
        except OSError:
            pass
    return {"db": db, "exists": os.path.isfile(db), "size_mb": round(size / 1e6, 1), "episode": episode,
            "closed": closed}


_EPISODE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 ._-]{0,59}$")


def episode_name(raw) -> str:
    """A safe episode name: letters, digits, space . _ -, 60 max; default ep-HHMMSS."""
    name = str(raw or "").strip()
    if not name:
        return time.strftime("ep-%H%M%S")
    if not _EPISODE_NAME.match(name):
        raise ValueError("episode name: letters, digits, space . _ - only, 60 characters max")
    return name


def _finite_xy(body: dict) -> tuple[float, float]:
    try:
        x, y = float(body.get("x")), float(body.get("y"))
    except (TypeError, ValueError):
        raise ValueError("goal needs numbers x and y (metres, world frame)") from None
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("goal x/y must be finite")
    if abs(x) > GOAL_LIMIT_M or abs(y) > GOAL_LIMIT_M:
        raise ValueError(f"goal beyond {GOAL_LIMIT_M:.0f} m: the movement manager would drop it")
    return x, y


def _later(delay_s: float, fn) -> threading.Timer:
    """Run fn once, delay_s from now, on a daemon thread (the bench swaps this out)."""
    def run():
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - a timer thread has nobody to raise to
            print(f"moss_cockpit: deferred command failed: {exc!r}", file=sys.stderr, flush=True)
    timer = threading.Timer(delay_s, run)
    timer.daemon = True
    timer.start()
    return timer


# --- the launcher, as a child process -----------------------------------------------

class Launcher:
    """Runs tools/moss-fly.sh <action> without waiting for it; output appended to
    $MOSS_STATE_DIR/last_launch.log (a `stop` may run while a `map` is still in
    its gates, so both write to the same file, each under a header line)."""

    def __init__(self, script: str = FLY_SCRIPT):
        self.script = script
        self._lock = threading.Lock()
        self._procs: list[tuple[str, float, subprocess.Popen]] = []
        self.last: dict = {}
        self.note: dict | None = None    # {text, ts}: why the cockpit itself ran the last action

    def log_path(self) -> str:
        return os.path.join(state_dir(), "last_launch.log")

    def _running(self) -> list:
        return [(a, t, p) for a, t, p in self._procs if p.poll() is None]

    def _cancel(self, fh) -> None:
        """Stop pressed during a launch: end every launcher still in its gates first,
        or it would bring the stack up after the operator's stop. Each one leads its
        own process group (start_new_session), so its preflight child goes too."""
        for action, _, proc in self._running():
            if action == "stop":
                continue
            how = "SIGTERM"
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=LAUNCH_KILL_GRACE_S)
                except subprocess.TimeoutExpired:
                    how = "SIGKILL"
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=LAUNCH_KILL_GRACE_S)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            fh.write(f"=== moss-fly.sh {action} (pid {proc.pid}) cancelled by stop: {how} ===\n".encode())

    def start(self, action: str, note: str | None = None) -> dict:
        """`note`: why the cockpit runs it by itself (the idle stop): a line in the log above the
        action's header, and status()'s "note" until the next start."""
        with self._lock:
            busy = self._running()
            if busy and action != "stop":
                return {"ok": False, "error": f"moss-fly.sh {busy[-1][0]} is still running"}
            if not os.path.isfile(self.script):
                return {"ok": False, "error": f"launcher missing: {self.script}"}
            path = self.log_path()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            # a cockpit started by `EXPLORE=1 moss-fly.sh explore` must not arm the next page launch
            env = {k: v for k, v in os.environ.items() if k not in LAUNCH_ENV_DROP}
            with open(path, "ab" if busy else "wb") as fh:
                if action == "stop":
                    self._cancel(fh)
                if note:
                    fh.write(f"=== {note}  {time.strftime(_ISO)} ===\n".encode())
                fh.write(f"=== moss-fly.sh {action}  {time.strftime(_ISO)} ===\n".encode())
                fh.flush()
                proc = subprocess.Popen(["bash", self.script, action], cwd=REPO_DIR, env=env,
                                        stdin=subprocess.DEVNULL, stdout=fh, stderr=subprocess.STDOUT,
                                        start_new_session=True)   # its own process group: Stop can end it
            self._procs = self._running() + [(action, time.time(), proc)]
            self.last = {"action": action, "started_at": time.time(), "pid": proc.pid}
            self.note = {"text": note, "ts": time.time()} if note else None
            return {"ok": True, "sent": f"moss-fly.sh {action}", "pid": proc.pid}

    def status(self, lines: bool = True) -> dict:
        with self._lock:
            running = self._running()
            last = self._procs[-1] if self._procs else None
            rc = last[2].poll() if last else None
            note = self.note
        head = {"running": bool(running), "action": last[0] if last else None,
                "started_at": last[1] if last else None, "returncode": rc, "note": note}
        if not lines:
            return head
        try:
            with open(self.log_path(), "rb") as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - 65536))
                tail = fh.read().decode("utf-8", "replace").splitlines()[-LAUNCH_TAIL:]
        except OSError:
            tail = []
        return dict(head, lines=tail)


# --- bus bookkeeping ---------------------------------------------------------------

class _Latest:
    __slots__ = ("raw", "mono", "arrivals")

    def __init__(self):
        self.raw = None
        self.mono = None
        self.arrivals = deque(maxlen=64)


class BusCache:
    """Latest bytes + arrival times per stream. Callbacks only store; decoding is
    the request's job, cached by the arrival stamp."""

    def __init__(self):
        self._lock = threading.Lock()
        self._topics = {name: _Latest() for name in list(CHANNELS) + [n for n, _ in CAMERAS.values()]}
        self._by_channel = {ch: name for name, ch in CHANNELS.items()}
        for name, ch in CAMERAS.values():
            self._by_channel[ch] = name
        self._decoded: dict = {}

    def ingest(self, channel: str, data: bytes, mono: float | None = None) -> None:
        name = self._by_channel.get(channel)
        if name is None:
            return
        mono = time.monotonic() if mono is None else mono
        with self._lock:
            t = self._topics[name]
            t.raw, t.mono = data, mono
            t.arrivals.append(mono)

    def clear(self) -> None:
        """Forget every message: a new run must not show the previous run's map, scan or goal."""
        with self._lock:
            self._topics = {name: _Latest() for name in self._topics}
            self._decoded = {}

    def forget(self, name: str) -> None:
        """Drop one topic's last message (a cleared map must not be served from the cache)."""
        with self._lock:
            if name in self._topics:
                self._topics[name] = _Latest()
            self._decoded.pop(name, None)

    def latest(self, name: str):
        with self._lock:
            t = self._topics[name]
            return t.raw, t.mono

    def stats(self, name: str, now: float) -> dict:
        with self._lock:
            t = self._topics[name]
            recent = [a for a in t.arrivals if now - a <= HZ_WINDOW_S]
            mono = t.mono
        hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and recent[-1] > recent[0] else 0.0
        return {"age_s": None if mono is None else round(now - mono, 2), "hz": round(hz, 1)}

    def last_arrival(self) -> float | None:
        """The monotonic time of the newest message on any tapped channel, None before the first."""
        with self._lock:
            arrivals = [t.mono for t in self._topics.values() if t.mono is not None]
        return max(arrivals) if arrivals else None

    def decoded(self, name: str, fn):
        """fn(raw) for the latest message of `name`, computed once per arrival.
        Returns (value, mono) or (None, None); a decode error caches None."""
        raw, mono = self.latest(name)
        if raw is None:
            return None, None
        hit = self._decoded.get(name)
        if hit is not None and hit[0] == mono:
            return hit[1], mono
        try:
            value = fn(raw)
        except Exception:  # noqa: BLE001 - one bad message must not poison the page
            value = None
        self._decoded[name] = (mono, value)
        return value, mono


# --- the cockpit: shared part ---------------------------------------------------------

class CameraUnavailable(RuntimeError):
    """The camera stream cannot be served (and why, for a 503)."""


class Cockpit:
    mode = "live"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        self.launcher = Launcher(fly_script)
        self.episode: dict | None = None
        self.episode_closed: dict | None = None   # closed by the cockpit when the stack stopped under it
        self._ep_lock = threading.Lock()
        self._cam_lock = threading.Lock()
        self._cam_clients = {cam: 0 for cam in CAMERAS}
        self.started = time.time()
        self.idle_limit_s = idle_limit_s()
        # The idle count runs on the monotonic clock: a clock step never stops a stack nor holds
        # one. The cockpit's own start counts as activity: a cockpit restarted under a running
        # stack gives it the whole limit, it never stops it at once for time it did not see.
        self.activity_mono = time.monotonic()
        self.activity_source = "start"
        self._idle_lock = threading.Lock()
        self._idle_stopped = None                 # the run the idle stop stopped: never twice
        self._idle_start = (None, None)           # (the run, its start on the monotonic clock): converted once
        self._idle_said: set = set()              # (run, what) journal lines already said for a run

    def note_activity(self, source: str, mono: float | None = None) -> None:
        """Something happened (`source`, one of IDLE_SOURCES) at `mono` (the monotonic clock, now by
        default): the idle count restarts from there."""
        mono = time.monotonic() if mono is None else mono
        with self._idle_lock:
            if mono >= self.activity_mono:
                self.activity_mono, self.activity_source = mono, source

    def _run_start_mono(self, stack: dict) -> float | None:
        """The stack's started_at (wall clock, UTC) on the monotonic clock, converted once per run."""
        if not (stack and stack.get("running")):
            return None
        key = (stack.get("run_id"), stack.get("pid"), stack.get("started_at"))
        if self._idle_start[0] != key:
            t = _parse_utc(stack.get("started_at"))
            self._idle_start = (key, None if t is None else time.monotonic() - (time.time() - t))
        return self._idle_start[1]

    def _idle_rule(self, stack: dict) -> dict:
        """idle_state on the monotonic clock, plus "source": what restarted the count last."""
        started = self._run_start_mono(stack)
        with self._idle_lock:
            act, source = self.activity_mono, self.activity_source
        st = idle_state(stack, act, time.monotonic(), self.idle_limit_s, started=started)
        st["source"] = "start" if started is not None and started > act else source
        return st

    def _idle_blind(self, stack: dict) -> str | None:
        """Why this cockpit cannot see the stack's activity, or None. A blind cockpit never stops a stack."""
        return None

    def idle_doc(self, stack: dict) -> dict:
        """/state.json "idle": {enabled, idle_s, stop_in_s, limit_s, last_source, blind}. last_source:
        what restarted the count last (IDLE_SOURCES; "start" = the stack's or the cockpit's start);
        blind: why the cockpit cannot see the run, and then nothing is counted (enabled false)."""
        st = self._idle_rule(stack)
        blind = self._idle_blind(stack)
        if blind:
            return {"enabled": False, "idle_s": None, "stop_in_s": None, "limit_s": self.idle_limit_s,
                    "last_source": st["source"], "blind": blind}
        return {"enabled": st["enabled"], "idle_s": st["idle_s"], "stop_in_s": st["stop_in_s"],
                "limit_s": st["limit_s"], "last_source": st["source"], "blind": None}

    def keepalive(self, stack: dict) -> dict:
        """POST /cmd/keepalive: the operator keeps the stack up, the idle count starts again."""
        self.note_activity("keepalive")
        return {"ok": True, "idle": self.idle_doc(stack)}

    def _say_once(self, run, what: str, level: str, event: str) -> None:
        """A journal line said once per run: a held stop says why without repeating it every second."""
        if (run, what) not in self._idle_said:
            self._idle_said.add((run, what))
            self._say(level, event)

    def _say(self, level: str, event: str) -> None:
        """The cockpit's own message line: its standard output, the service's journal."""
        print(f"moss_cockpit: {level}: {event}", flush=True)

    @property
    def camera_clients(self) -> int:
        """Viewers on all cameras together."""
        return sum(self._cam_clients.values())

    def camera_viewers(self, cam: str) -> int:
        return self._cam_clients.get(cam, 0)

    def camera_state(self) -> dict:
        doc = {"clients": self.camera_clients, "subscribed": self.camera_clients > 0}
        for cam in CAMERAS:
            n = self._cam_clients[cam]
            doc[cam] = {"clients": n, "subscribed": n > 0}
        return doc

    # episode bookkeeping, shared by both modes
    def _episode(self, body: dict) -> dict:
        action = str(body.get("action", ""))
        if action not in ("start", "stop"):
            raise ValueError('episode needs "action": "start" or "stop"')
        with self._ep_lock:
            if action == "start":
                name = episode_name(body.get("name"))
                self.episode = {"name": name, "started_at": time.time()}
                self.episode_closed = None
            else:
                name = (self.episode or {}).get("name") or episode_name(body.get("name"))
                self.episode = None
        return {"action": action, "name": name, "ts": time.time()}

    def camera_open(self, cam: str = "nose") -> None:
        if cam not in CAMERAS:
            raise CameraUnavailable(f"no camera called {cam!r} (nose or wrist)")
        with self._cam_lock:
            if self._cam_clients[cam] >= MAX_CAMERA_CLIENTS:
                raise CameraUnavailable(f"{MAX_CAMERA_CLIENTS} viewers already on the {cam} camera")
            self._cam_clients[cam] += 1

    def camera_close(self, cam: str = "nose") -> None:
        with self._cam_lock:
            if cam in self._cam_clients:
                self._cam_clients[cam] = max(0, self._cam_clients[cam] - 1)

    def launch(self) -> dict:
        return self.launcher.status()

    def scan_doc(self) -> dict:
        """/scan.json: the latest scan (an empty one without a cloud) plus the pose read now,
        the same document /state.json gives under "pose". A copy: the cached scan never
        carries a pose, so a scan served again from the cache still gets the newest pose."""
        doc = self.scan()
        doc = dict(doc) if doc is not None else {"frame": None, "stamp": None, "points": []}
        doc["pose"] = self.pose()
        return doc


# --- live: the stack's bus ----------------------------------------------------------

class LiveCockpit(Cockpit):
    mode = "live"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        super().__init__(fly_script)
        self.bus = BusCache()
        self.bus_error: str | None = None if lcm is not None else "python module lcm not importable"
        if not HAVE_MSGS:
            self.bus_error = "python module dimos_lcm not importable"
        self._pub_lock = threading.Lock()
        self._pub_lcm = None
        self._map_cache = (None, None, 0.0)     # (mono, (png, meta), decoded_at)
        self._scan_cache = (None, None, 0.0)
        self._cam_cache = {cam: (None, None) for cam in CAMERAS}   # cam -> (mono, jpeg)
        self._encoder = None
        self._ep_seen = None
        self._run_id = None                     # the run the bus cache belongs to
        self._goal_lock = threading.Lock()
        self._goal_gen = 0                      # a stop, an explore or a newer goal cancels a queued goal
        self._feed = DecisionFeed()             # the decisions in the running stack's main.jsonl
        self.listening = False                  # the bus tap is subscribed: the idle watchdog sees the run
        self._idle_next = 0.0                   # monotonic time of the watchdog's next look
        self._idle_seen: dict = {}              # topic -> arrival of the last message the watchdog read
        self._idle_run = None                   # (run_id, pid) the watchdog's pose samples belong to
        self._idle_poses: deque = deque(maxlen=64)   # (arrival, pose) per look: the last IDLE_WINDOW_S and one older

    # the listener ------------------------------------------------------------
    def start_listener(self) -> None:
        if lcm is None:
            return
        threading.Thread(target=self._listen, name="lcm-listener", daemon=True).start()

    def _listen(self) -> None:
        """One LCM handle, one thread, explicit channels only; a camera is
        subscribed while someone watches it."""
        while True:
            try:
                lc = lcm.LCM()
                for ch in CHANNELS.values():
                    lc.subscribe(ch, self._on_message)
                image_subs = {cam: None for cam in CAMERAS}
                self.bus_error = None
                self.listening = True
                while True:
                    for cam, (_, channel) in CAMERAS.items():
                        want = self._cam_clients[cam] > 0
                        if want and image_subs[cam] is None:
                            image_subs[cam] = lc.subscribe(channel, self._on_message)
                        elif not want and image_subs[cam] is not None:
                            lc.unsubscribe(image_subs[cam])
                            image_subs[cam] = None
                    lc.handle_timeout(LCM_HANDLE_MS)
                    if time.monotonic() >= self._idle_next:
                        self._idle_next = time.monotonic() + IDLE_TICK_S
                        try:
                            self.idle_tick()
                        except Exception as exc:  # noqa: BLE001 - the watchdog never takes the bus tap down
                            print(f"moss_cockpit: idle watchdog: {exc!r}", file=sys.stderr, flush=True)
            except Exception as exc:  # noqa: BLE001 - multicast not up yet, etc.: retry
                self.listening = False
                self.bus_error = f"{type(exc).__name__}: {exc}"
                time.sleep(3.0)

    def _on_message(self, channel: str, data: bytes) -> None:
        self.bus.ingest(channel, data)

    # reads -------------------------------------------------------------------
    def _pose(self, now: float) -> dict | None:
        pose, mono = self.bus.decoded("odom", decode_pose)
        if pose is None:
            return None
        return {"x": round(pose["x"], 3), "y": round(pose["y"], 3), "yaw": round(pose["yaw"], 4),
                "age_s": round(now - mono, 2)}

    def pose(self) -> dict | None:
        return self._pose(time.monotonic())

    def _base(self, now: float) -> dict | None:
        t, mono = self.bus.decoded("base_telemetry", decode_json)
        if not isinstance(t, dict):
            return None
        return base_view(t, now - mono)

    def _bool(self, name: str):
        value, mono = self.bus.decoded(name, decode_bool)
        return value, mono

    def _nav(self, now: float, stack: dict | None = None) -> dict:
        start, t_start = self._bool("explore_cmd")
        _, t_stop = self._bool("stop_explore_cmd")
        done, t_done = self._bool("explore_done")
        # a pad touch on moss-explore: MovementManager publishes stop_movement and the
        # explorer stops on it (WavefrontFrontierExplorer._on_stop_movement)
        halt, t_halt = self._bool("stop_movement")
        t_start = t_start if start else None
        exploring = t_start is not None and t_start > max(t_stop or -1.0, (t_done if done else None) or -1.0,
                                                          (t_halt if halt else None) or -1.0)
        # moss-nav has no explorer: an explore_cmd there only opens map writing (the costmap
        # mission gate), nothing drives. Only an explore stack can be exploring.
        if stack is not None and stack.get("running") and "explore" not in str(stack.get("blueprint") or ""):
            exploring = False
        explore_done = bool(done) and (t_start is None or (t_done or 0) > t_start)
        goal_pose, t_goal = self.bus.decoded("goal_request", decode_pose)
        goal = None
        if goal_pose is not None and math.isfinite(goal_pose["x"]) and math.isfinite(goal_pose["y"]):
            goal = {"x": round(goal_pose["x"], 3), "y": round(goal_pose["y"], 3)}
        reached, t_reached = self._bool("goal_reached")
        goal_reached = reached if (t_reached is not None and (t_goal is None or t_reached > t_goal)) else None
        tw, t_tw = self.bus.decoded("cmd_vel", decode_twist)
        last_cmd = None if tw is None else {"vx": round(tw[0], 3), "wz": round(tw[1], 3),
                                            "age_s": round(now - t_tw, 2)}
        return {"exploring": exploring, "explore_done": explore_done, "goal": goal,
                "goal_reached": goal_reached, "last_cmd_vel": last_cmd}

    def _episode_from_bus(self) -> None:
        """Markers published by anyone (another page, a script) move the episode too."""
        marker, mono = self.bus.decoded("episode_marker", lambda raw: json.loads(decode_string(raw)))
        if not isinstance(marker, dict) or mono == self._ep_seen:
            return
        self._ep_seen = mono
        with self._ep_lock:
            if marker.get("action") == "start":
                self.episode = {"name": marker.get("name"), "started_at": marker.get("ts")}
                self.episode_closed = None
            elif marker.get("action") == "stop":
                self.episode = None

    def _sync_run(self, stack: dict) -> None:
        """A new run_id in the registry: drop the previous run's messages (its costmap
        is in another odometry frame). A stop alone keeps them: the last map stays readable."""
        run_id = stack.get("run_id") if stack.get("running") else None
        if run_id is None or run_id == self._run_id:
            return
        if self._run_id is not None:
            self.bus.clear()
            self._map_cache = (None, None, 0.0)
            self._scan_cache = (None, None, 0.0)
            self._cam_cache = {cam: (None, None) for cam in CAMERAS}
        self._run_id = run_id

    def _close_orphan_episode(self) -> None:
        """The stack stopped with an episode open: nothing records it any more, close it here."""
        with self._ep_lock:
            if self.episode is None:
                return
            self.episode_closed = dict(self.episode, ts=time.time(),
                                       reason="the stack stopped: the episode was closed")
            self.episode = None

    def _moss_stack(self) -> dict | None:
        """The running stack if it is a MOSS one. Another dimOS stack may share the
        computer and the bus: it never takes a goal or an explore_cmd from this page."""
        st = stack_status()
        return st if is_moss_stack(st) else None

    def state(self) -> dict:
        now = time.monotonic()
        stack = stack_status()
        self._sync_run(stack)
        self._episode_from_bus()
        if not stack["running"]:
            pid = _dimos_run_pid()
            if pid:
                stack = {"running": True, "run_id": None, "blueprint": None, "pid": pid,
                         "started_at": None, "uptime_s": None, "log_dir": None}
        if not stack["running"]:
            self._close_orphan_episode()
        env, _ = bus_env(stack.get("pid") if stack["running"] else None)
        transport = bus_transport(env)
        ts = time.time()
        return {
            "ts": ts, "mode": self.mode,
            "stack": stack,
            "idle": self.idle_doc(stack),
            "pose": self._pose(now),
            "topics": {name: self.bus.stats(name, now) for name in STATE_TOPICS},
            "base": self._base(now),
            "nav": self._nav(now, stack),
            "recording": recording_view(self.episode, self.episode_closed),
            "system": system_metrics(),
            "bus": {"transport": transport, "listening": lcm is not None and HAVE_MSGS,
                    "error": self.bus_error},
            "camera": self.camera_state(),
            "launch": self.launcher.status(lines=False),
        }

    def map(self):
        self._sync_run(stack_status())
        raw, mono = self.bus.latest("global_costmap")
        if raw is None:
            return None
        cached_mono, cached, at = self._map_cache
        if cached is not None and (cached_mono == mono or time.monotonic() - at < MAP_MIN_INTERVAL_S):
            return cached
        result = decode_costmap(raw)
        self._map_cache = (mono, result, time.monotonic())
        return result

    def scan(self) -> dict | None:
        stack = stack_status()
        self._sync_run(stack)
        now = time.monotonic()
        raw, mono = self.bus.latest("lidar")
        name = "lidar"
        if raw is None or now - mono > SCAN_FALLBACK_S:
            fb_raw, fb_mono = self.bus.latest("pointcloud")
            if fb_raw is not None:
                raw, mono, name = fb_raw, fb_mono, "pointcloud"
        if raw is None:
            return None
        cached_key, cached, at = self._scan_cache
        if cached is not None and (cached_key == (name, mono) or now - at < SCAN_MIN_INTERVAL_S):
            return cached
        frame, stamp, xyz = cloud_xyz(raw)
        # only the `lidar` cloud carries depth-camera points; /pointcloud is the C1's own scan
        tag = None
        if name == "lidar":
            tag = lidar_tag_height(stack.get("pid") if stack.get("running") else _dimos_run_pid())
        doc = {"frame": frame or ("world" if name == "lidar" else "lidar_link"), "stamp": stamp,
               "source": name, "age_s": round(now - mono, 2)}
        doc.update(scan_points(xyz, tag))
        self._scan_cache = ((name, mono), doc, now)
        return doc

    def path(self) -> dict:
        self._sync_run(stack_status())
        pts, mono = self.bus.decoded("path", decode_path)
        return {"points": pts or [], "age_s": None if mono is None else round(time.monotonic() - mono, 2)}

    def log(self, n: int) -> list[dict]:
        return log_tail(stack_status().get("log_dir"), n)

    def decisions(self, n: int) -> dict:
        """The running stack's decisions (its main.jsonl, read incrementally) and the word now.
        `exploring` comes from the bus when it is listened to, else it is unknown (None).
        Once the stack stops, the last run's events stay (IDLE, "last run: ..."), as its map
        does, until a new run's log replaces them. A `dimos run` with no registry entry (a
        foreground run, a swept registry) has no known log: READY, and the detail says so."""
        stack = stack_status()
        self._sync_run(stack)
        now = time.time()
        if stack.get("running") and stack.get("log_dir"):
            events = self._feed.read(stack["log_dir"], stack.get("run_id"))
            exploring = self._nav(time.monotonic(), stack)["exploring"] if lcm is not None and HAVE_MSGS else None
            return decisions_doc(events, n, True, exploring, now, stack.get("run_id"))
        if stack.get("running") or _dimos_run_pid() is not None:
            return {"now": {"word": "READY", "detail": TXT_NO_LOG, "since": None}, "events": [], "run": None}
        return decisions_doc(self._feed.read(None), n, False, None, now, self._feed.run)

    def preflight(self) -> dict:
        return preflight_doc()

    # the idle stop -----------------------------------------------------------
    def _idle_blind(self, stack: dict) -> str | None:
        if not (self.listening and HAVE_MSGS):
            return "the cockpit does not listen to the bus"
        if not stack.get("running"):
            return None
        env, _ = bus_env(stack.get("pid"))
        if bus_transport(env) != "lcm":
            return "the stack is not on LCM, the bus the cockpit listens to"
        if (env.get("LCM_DEFAULT_URL") or "") != (os.environ.get("LCM_DEFAULT_URL") or ""):
            return "the stack's LCM_DEFAULT_URL is not the cockpit's: another LCM bus"
        if is_moss_stack(stack):
            argv = proc_cmdline(stack.get("pid"))
            if argv is None or not is_dimos_run(argv):
                return f"pid {stack.get('pid')} is not a dimos run (a stale registry entry)"
            heard = self.bus.last_arrival()
            if heard is None or time.monotonic() - heard > IDLE_DEAF_S:
                # every MOSS stack publishes base_telemetry at 2 Hz: silence here means this
                # cockpit is deaf (another multicast route, a firewall), not that the rover is idle
                return f"nothing heard on the bus for {IDLE_DEAF_S:g} s"
        return None

    def idle_tick(self) -> dict | None:
        """The watchdog, once a second from the bus tap's loop: note what the bus shows happened
        since its last look, then stop a moss-nav or moss-explore stack idle for idle_limit_s,
        once per run. Returns the launcher's answer when it asked for a stop, else None."""
        stack = stack_status()
        self._episode_from_bus()                   # a marker from a script opens the episode too
        if self.episode is not None and not stack.get("running") and _dimos_run_pid() is None:
            self._close_orphan_episode()           # no stack left to record it: it holds nothing
        self._idle_scan(stack)
        return self._idle_fire(stack)

    def _idle_scan(self, stack: dict) -> None:
        """Activity on the bus since the last look, each at its arrival (monotonic). Only messages
        that arrived since then count: a planner that went silent on a non-zero twist is not driving."""
        run = (stack.get("run_id"), stack.get("pid")) if stack.get("running") else None
        if run != self._idle_run:
            self._idle_run = run
            self._idle_poses.clear()                       # a new run: its odometry starts a new frame

        def new(name, fn):
            value, m = self.bus.decoded(name, fn)
            if m is None or m == self._idle_seen.get(name):
                return None, None
            self._idle_seen[name] = m
            return value, m

        t, at = new("base_telemetry", decode_json)
        if isinstance(t, dict) and telemetry_fresh(t) and motors_commanded(t.get("output_pct")):
            self.note_activity("telemetry", at)
        tw, at = new("cmd_vel", decode_twist)
        if tw is not None and (tw[0] != 0.0 or tw[1] != 0.0):
            self.note_activity("cmd_vel", at)
        order, at = new("explore_cmd", decode_bool)
        if order:
            self.note_activity("explore_cmd", at)
        _, at = new("goal_request", decode_pose)
        if at is not None:
            self.note_activity("goal_request", at)         # a goal from the page, a script or the explorer
        _, at = new("episode_marker", decode_json)
        if at is not None:
            self.note_activity("episode", at)
        if self.episode is not None:
            self.note_activity("episode")                  # an open episode holds the stop
        pose, at = new("odom", decode_pose)
        if pose is not None and self._pose_moved(pose, at):
            self.note_activity("odom", at)

    def _pose_moved(self, pose: dict, at: float) -> bool:
        """The pose now against the pose IDLE_WINDOW_S ago (early in a run, its first one): moved
        more than IDLE_MOVE_M or turned more than IDLE_TURN_RAD. A window, not a sum since the last
        move: odometry drifting a few mm a minute at rest never adds up to a move."""
        ring = self._idle_poses
        ring.append((at, pose))
        while len(ring) > 1 and ring[1][0] <= at - IDLE_WINDOW_S:
            ring.popleft()
        return pose_moved(ring[0][1], pose)

    def _idle_fire(self, stack: dict) -> dict | None:
        """Stop a moss-nav or moss-explore stack that is due, the way the page's Stop stack button
        does. Held while the cockpit is blind (the journal says why, once per run); tried again
        at the next look when the launcher fails."""
        st = self._idle_rule(stack)
        if not st["due"]:
            return None
        run = (stack.get("run_id"), stack.get("pid"))
        if run == self._idle_stopped:
            return None
        blind = self._idle_blind(stack)
        if blind:
            self._say_once(run, "blind: " + blind, "warning", f"idle {st['idle_s']:g} s, not stopped: {blind}")
            return None
        busy = self.launcher.status(lines=False)
        if busy.get("running"):
            began = busy.get("started_at")
            age = time.time() - began if isinstance(began, (int, float)) else 0.0
            if age <= IDLE_BUSY_MAX_S:
                return None                  # a launch, a flight check or a stop is running: never over it
            self._say_once(run, "busy", "warning", f"moss-fly.sh {busy.get('action')} has run {age:.0f} s: "
                                                   "the idle stop no longer waits for it")
        msg = idle_stop_message(self.idle_limit_s)
        try:
            answer = self.launcher.start("stop", note=msg)
        except Exception as exc:  # noqa: BLE001 - an unwritable state dir...: tried again at the next look
            answer = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        if not answer.get("ok"):
            self._say_once(run, f"failed: {answer.get('error')}", "warning",
                           f"{msg}: the stop did not start ({answer.get('error')}), tried again each second")
            return answer
        self._idle_stopped = run
        self._say("info", msg)
        return answer

    # camera ------------------------------------------------------------------
    def _encode(self, arr: np.ndarray, encoding: str) -> bytes:
        if self._encoder is None:
            try:
                from turbojpeg import TJPF_BGR, TJPF_GRAY, TJPF_RGB, TurboJPEG
                tj = TurboJPEG()
                fmts = {"rgb8": TJPF_RGB, "bgr8": TJPF_BGR, "mono8": TJPF_GRAY}

                def enc(a, e):
                    return tj.encode(a, quality=70, pixel_format=fmts[e])
                self._encoder = ("turbojpeg", enc)
            except Exception:  # noqa: BLE001
                try:
                    import cv2

                    def enc(a, e):
                        a = a[..., ::-1] if e == "rgb8" else a
                        ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(a), [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if not ok:
                            raise CameraUnavailable("cv2 could not encode the frame")
                        return buf.tobytes()
                    self._encoder = ("cv2", enc)
                except Exception:  # noqa: BLE001
                    self._encoder = ("none", None)
        name, enc = self._encoder
        if enc is None:
            raise CameraUnavailable("raw camera frames and no JPEG encoder here (turbojpeg or cv2); "
                                    "put the stack's color_image on JpegLcmTransport")
        return enc(arr, encoding)

    def camera_frame(self, cam: str = "nose"):
        """(key, jpeg) of the latest frame of that camera, or None before the first one."""
        topic = CAMERAS[cam][0]
        raw, mono = self.bus.latest(topic)
        if raw is None:
            return None
        cached = self._cam_cache[cam]
        if cached[0] == mono:
            return mono, cached[1]
        m = LcmImage.lcm_decode(raw)
        enc = str(m.encoding).lower()
        data = m.data if isinstance(m.data, (bytes, bytearray)) else bytes(m.data)
        if enc == "jpeg":
            jpeg = bytes(data)        # JpegLcmTransport: the bus already carries the JPEG
        else:
            chans = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}.get(enc)
            if chans is None:
                raise CameraUnavailable(f"camera encoding {m.encoding!r} not handled")
            arr = np.frombuffer(data, np.uint8)[: m.height * m.width * chans]
            arr = arr.reshape(m.height, m.width, chans) if chans > 1 else arr.reshape(m.height, m.width)
            if chans == 4:
                arr, enc = arr[..., :3], ("rgb8" if enc == "rgba8" else "bgr8")
            jpeg = self._encode(arr, enc)
        self._cam_cache[cam] = (mono, jpeg)
        return mono, jpeg

    # commands ----------------------------------------------------------------
    def _publisher(self):
        """One LCM handle for every command publish (None: explore_ctl opens its own)."""
        if lcm is None:
            return None
        with self._pub_lock:
            if self._pub_lcm is None:
                self._pub_lcm = lcm.LCM()
            return self._pub_lcm

    def command(self, name: str, body: dict) -> dict:
        if name == "stack":
            return self._stack(body)
        if name == "keepalive":
            return self.keepalive(stack_status())
        try:
            import explore_ctl                   # needs dimos_lcm: imported where it is used
            from dimos_lcm.std_msgs import Bool, String
        except ImportError as exc:
            return {"ok": False, "error": f"cannot publish on the bus here: {exc}", "_code": 503}
        # the bus is the one the running stack chose (the listener is LCM-only too): say so, publish nothing
        st = stack_status()
        env, whose = bus_env(st.get("pid") if st.get("running") else _dimos_run_pid())
        try:
            explore_ctl.require_lcm(env)
        except explore_ctl.TransportError as exc:
            return {"ok": False, "error": f"cannot publish on the bus here: {exc} (read from {whose})",
                    "_code": 503}
        bus = self._publisher()

        def pub(stream, msg, msg_name=None):
            with self._pub_lock:
                return explore_ctl.publish_msg(stream, msg, msg_name=msg_name, bus=bus, env=env)

        def not_moss():
            st = stack_status()
            what = f"the running stack is {st.get('blueprint')!r}" if st.get("running") else "no stack is running"
            return {"ok": False, "error": f"refused: {what}, not a MOSS stack", "_code": 409}

        if name == "explore":
            action = body.get("action")
            if action not in ("start", "stop"):
                raise ValueError('explore needs "action": "start" or "stop"')
            if self._moss_stack() is None:
                return not_moss()
            with self._goal_lock:
                self._goal_gen += 1
                return {"ok": True, "sent": pub(explore_ctl.CHANNELS[action], Bool(data=True))}
        if name == "map":
            if body.get("action") != "clear":
                raise ValueError('map needs "action": "clear"')
            if self._moss_stack() is None:
                return not_moss()
            sent = pub(explore_ctl.CHANNELS["clear"], Bool(data=True))
            # the last map must not be served again: the next one comes from the fresh grid
            self.bus.forget("global_costmap")
            self._map_cache = (None, None, 0.0)
            return {"ok": True, "sent": sent}
        if name == "goal":
            x, y = _finite_xy(body)
            st = self._moss_stack()
            if st is None:
                return not_moss()
            exploring = self._nav(time.monotonic(), st)["exploring"]
            with self._goal_lock:
                self._goal_gen += 1
                gen = self._goal_gen
                if not exploring:
                    sent = pub("goal_request", goal_message(x, y), msg_name="geometry_msgs.PoseStamped")
                    return {"ok": True, "sent": sent, "goal": {"x": x, "y": y}}
                # Explorer2 publishes its own goal_request: stop it first. Its stop ends with a
                # 'stay here' goal about 2 s later, which would replace ours: ours goes after it.
                first = pub(explore_ctl.CHANNELS["stop"], Bool(data=True))

            def deferred():
                with self._goal_lock:
                    if gen != self._goal_gen or self._moss_stack() is None:
                        return                    # a stop, an explore, a newer goal or no stack: drop it
                    pub("goal_request", goal_message(x, y), msg_name="geometry_msgs.PoseStamped")
            _later(GOAL_AFTER_STOP_S, deferred)
            return {"ok": True, "sent": first,
                    "queued": explore_ctl.channel("goal_request", "geometry_msgs.PoseStamped"),
                    "queued_s": GOAL_AFTER_STOP_S, "goal": {"x": x, "y": y}}
        if name == "stop":
            with self._goal_lock:
                self._goal_gen += 1               # a goal still queued behind an explore stop is dropped
            # soft stop: the firmware deadman (300 ms) and `moss-fly.sh stop` are the hard ones
            a = pub(explore_ctl.CHANNELS["stop"], Bool(data=True))
            b = pub("stop_movement", Bool(data=True))
            c = pub("tele_cmd_vel", zero_twist())
            also = [a, b]
            if stack_status().get("blueprint") != "moss-explore":
                # moss-nav has no MovementManager: the pad's tele_cmd_vel is remapped
                # onto cmd_vel and nobody listens on /tele_cmd_vel, so zero cmd_vel itself
                also.append(c)
                c = pub("cmd_vel", zero_twist())
            return {"ok": True, "sent": c, "also": also}
        if name == "episode":
            marker = self._episode(body)
            sent = pub("episode_marker", String(data=json.dumps(marker)))
            return {"ok": True, "sent": sent, "episode": self.episode}
        return {"ok": False, "error": f"unknown command {name!r}", "_code": 404}

    def _stack(self, body: dict) -> dict:
        action = body.get("action")
        if action not in ("map", "explore", "stop", "check"):
            raise ValueError('stack needs "action": "map", "explore", "stop" or "check"')
        if action in ("map", "explore"):
            st = stack_status()
            if st["running"] and not str(st.get("blueprint") or "").startswith("moss"):
                return {"ok": False, "error": f"a dimOS stack is already running ({st.get('blueprint')!r}, "
                                              "not MOSS): the MOSS cockpit leaves it alone", "_code": 409}
            if st["running"] or _dimos_run_pid():
                return {"ok": False, "error": "a dimOS stack is already running: stop it first"}
        return self.launcher.start(action)


# --- demo: a synthetic rover, for the page without a robot ------------------------------

# 320x240 JPEG (made once with cv2; studio-to-navy gradient, a mint square, "DEMO CAMERA")
_DEMO_JPEG_B64 = (
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA0JCgsKCA0LCgsODg0PEyAVExISEyccHhcgLikxMC4pLSwzOko+MzZGNywtQFdB"
    "RkxOUlNSMj5aYVpQYEpRUk//2wBDAQ4ODhMREyYVFSZPNS01T09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09P"
    "T09PT09PT09PT09PT0//wAARCADwAUADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA"
    "AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6"
    "Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXG"
    "x8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA"
    "AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5"
    "OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPE"
    "xcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDBopM0ZroOcWikzRmgBaKTNGaAFopM0ZoA"
    "WikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikz"
    "RmgBaKTNGaAFopM0ZoAWikzRmgBM0ZpuaM0ASxxSy58uN3x12qTikkjkjOJI2Qn+8MV13geRo9O1ORcbkAYfUBqk0DX5tau2"
    "07U4YZI5EJGFxyOx/DNTcqxx0cUsufLjd8ddqk4pxtrgAkwSgDqdhrtvDVu9lLrdvbDLRSYiB78Nt/pVS6v/ABYtrK09nGsQ"
    "Ql22rwMcnrRzBY5ExyCMSFGCHo2OD+NIqs7BUUsx6ADJr0DTLSG/8G21nOwXzlKoT2bJI/lVTwfpDWU0l3fLsmLtDCp9RncR"
    "+R/I0uYOU4pgysVYEEdQe1PaGZYxI0UgQ9GKnB/Guz0uxiuvGWpzzIri3YFQezHof0NQ6V4pur/XFtZ44jazsUCbeQOcfWnz"
    "BY47NKAWICgknoBWj4ks47DXLiCFQsWQyqOwIzit7QPL0nwpcawsSPcMSFZh0GQoH0zzRfQVjkpYZYcebE6Z6blIpgySAAST"
    "0Arq7PxVDeW09tr8QkjcfKY0/wA/gap+DrJbnWvPbmG1UyEn16D/AB/Ci/cdjCkjkjOJEZCezDFAjkMZkCMUHBbHA/Guv8RS"
    "Ra54dTU7dfmtpWUjuFzj/wCJNWvCMcE3heWK5x5UkrI2ffA/rRzaBy6nCDJIABJPQCn+VL5nl+W+/wDu7Tn8q6zw9oDWetTz"
    "X2BHauEiY9HZvun9R+JFXJP+SiR/9cP/AGU0cwcpxX2W5/595v8Avg01YJmZlWKQleoCniu5v7vxQl9MtnaI1uHPlsQOR+dR"
    "+F57uQ6zNOuLsONygfxANxj6ilzBynEyRSx/6yN0/wB5SKZmvQdBudV1Lz4NbsgLcpwZIimT6YPWuU0WwjvfEq28fzW8cpcn"
    "1RTx+fA/GmpBYzHiljAMkboD03AjNMzXfa1LFrukalFAMy2EuV99o5P4/MPwrz/NCdxNWHZozTc0ZqhDs0ZpuaM0AOzRmm5o"
    "zQA7NGabmjNADs0ZpuaM0AOzRmm5ozQA7NGabmjNADs0ZpuaM0AJmjNNzRmkM7TwMhm0/U4lIDOFUZ9w1S6LoX9gXjX+p3ts"
    "qxodoVj3+oFcNmjNKw7nfeGJ21B9bmhbY00mYyeNuQ2P6VWn0DxK0Egl1dHjKncpnfBGOR0ris0ZpWC52dxO9v4E06eFtskc"
    "ysp9wWpdO1+XWvFGnBohDHEH+QNnLFDzXF5ozT5Qudzp2owWXjLUobiRUS4bCsem4dB+ppdO8MHTdYGoT3UH2OJi6fNg98Z7"
    "cVwuaXJxjPApWC5qeIr6PUNbuLiE5iJCocdQBjNb3h6W31TwzPorzpFPk7A3GRkMD781xmaM07aBc7K38NWWl201z4gmicAf"
    "u0jcjP8AIk+1T6O1povhVru+jcreuQUT7xU5AHUdsn8a4csT1OaTNFguegaHfaHfLcaVYWs1us6MWWQ8Nxg45POP5VTtontP"
    "BGpwucSQzspx6grXF5ozS5QudUniS41S70m0kjEYS4jMrBs+YQRzjtWrIf8Ai4sf/XD/ANlNcBmjNPlC56BqGj+Ip7+eW11U"
    "Rwu5KJ5zjaPTAFM8NxXFgNbjnlD3ERDM4OcttJzzXBZozS5QudzoWpzeINNvNMvbgi5ZSUkGFJH4eh/MGmeF4F0XTtQ1O/Uq"
    "Y2MWB1+U8gfU8fhXE5ozRyhc73QdX0E6h9nsLOeCS5+UtIcq3U4PzGuR1mz/ALP1a5tf4Uf5f908j9DVDNGaaVgbuOzRmm5o"
    "zTEOzRmm5ozQA7NGabmjNADs0ZpuaM0AOzRmm5ozQA7NGabmjNADs0ZpuaM0AOzRmm5ozQAlFJRQAtFJRQAtFJRQAtFJRQAt"
    "FJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQA3"
    "NGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRm"
    "kzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBtFJmjN"
    "IYtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtF"
    "JmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNACZozTaKAHZozTaKAHZozTaKAHZozTaKAHZozTaKAHZozT"
    "aKAHZozTaKAHZozTaKAHZozWhaaFqd5bJcW1tvifO1vMUZwcdz7VN/wjOs/8+f8A5FT/ABpXRDqwWjkjJzRmtb/hGdZ/58//"
    "ACKn+NH/AAjOs/8APn/5FT/Gi6F7an/MvvMnNGa1v+EZ1n/nz/8AIqf40f8ACM6z/wA+f/kVP8aLoPbU/wCZfeZOaM1rf8Iz"
    "rP8Az5/+RU/xpk3h3VoIXmltNscalmPmIcAcnvRdB7Wn/MvvMzNGabRTNB2aM02igB2aM02igB2aM02igB2aM02igB2aM02i"
    "gB2aM02igB2aM02igBKKSigYtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQB6R4U/5Fy0/4H/6G1a9Y/hP/kW7T/gf"
    "/obVsVi9zwq38SXqwooopGYUUUUAFU9X/wCQNff9e8n/AKCauVT1j/kDX3/XvJ/6CaaKh8SPLaKSitj3xaKSigBaKSigBaKS"
    "igBaKSigBaKSigBaKSigBaKSigBuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjN"
    "JmjNAHpXhL/kW7T/AIH/AOhtWxWN4S/5Fq0/4H/6G1bNYvc8Kt/El6sKKKKRmFFFFABVPWP+QLff9e8n/oJq5VPWf+QLff8A"
    "XvJ/6CaaKh8SPKs0ZpM0ZrY98XNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAG5ozSZ"
    "ozSAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAPTPCP8AyLVp/wAD/wDQ2rZrG8If"
    "8izaf8D/APQ2rZrJ7nhVv4kvVhRRRSMwooooAKpaz/yBb/8A69pP/QTV2qWs/wDIEv8A/r2k/wDQTTRUPiR5RmjNJmjNanvi"
    "5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAmaM1e/tG1/6Ath/33P/APHKP7Rtf+gL"
    "Yf8Afc//AMcoGUc0Zq9/aNr/ANAWw/77n/8AjlH9o2v/AEBbD/vuf/45QBRzRmr39o2v/QFsP++5/wD45R/aNr/0BbD/AL7n"
    "/wDjlAFHNGavf2ja/wDQFsP++5//AI5R/aNr/wBAWw/77n/+OUAUc0Zq9/aNr/0BbD/vuf8A+OUf2ja/9AWw/wC+5/8A45QB"
    "RzRmr39o2v8A0BbD/vuf/wCOUf2ja/8AQFsP++5//jlAFHNGavf2ja/9AWw/77n/APjlH9o2v/QFsP8Avuf/AOOUAUc0Zq9/"
    "aNr/ANAWw/77n/8AjlH9o2v/AEBbD/vuf/45QB33hD/kWbT/AIH/AOhtW1XnFp4vvLK2S3tLOyihTO1AHOMnJ5LE9TU3/Cc6"
    "p/zws/8Avhv/AIqocWeXUwlSU213PQaK8+/4TnVP+eFn/wB8N/8AFUf8Jzqn/PCz/wC+G/8AiqXKyPqdU9Borz7/AITnVP8A"
    "nhZ/98N/8VR/wnOqf88LP/vhv/iqOVh9TqnoNUta/wCQJf8A/XtJ/wCgmuL/AOE51T/nhZ/98N/8VTJvGmoTwvDNa2bxyKVd"
    "drjIPBHDUcrHHB1U0znc0Zq9/aNr/wBAWw/77n/+OUf2ja/9AWw/77n/APjlaHrFHNGavf2ja/8AQFsP++5//jlH9o2v/QFs"
    "P++5/wD45QBRzRmr39o2v/QFsP8Avuf/AOOUf2ja/wDQFsP++5//AI5QBRzRmr39o2v/AEBbD/vuf/45R/aNr/0BbD/vuf8A"
    "+OUAUc0Zq9/aNr/0BbD/AL7n/wDjlH9o2v8A0BbD/vuf/wCOUAUc0Zq9/aNr/wBAWw/77n/+OUf2ja/9AWw/77n/APjlAFHN"
    "Gavf2ja/9AWw/wC+5/8A45R/aNr/ANAWw/77n/8AjlAFHNGavf2ja/8AQFsP++5//jlH9o2v/QFsP++5/wD45QBnZozTc0Zo"
    "AdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRm"
    "gB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgBM0ZpKKB"
    "i5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5oz"
    "SUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUANopM0ZpALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNA"
    "C0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0Um"
    "aM0ALRSZozQAmaM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzR"
    "mgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03N"
    "GaAHZozTc0ZoA//Z"
)
DEMO_JPEG = base64.b64decode("".join(_DEMO_JPEG_B64))


class DemoWorld:
    """A 12 x 12 m square corridor around a central block, mapped as the pose drives
    its loop at 0.2 m/s. Lethal walls, a cost ramp near them, unknown outside."""

    RES = 0.05
    N = 240
    OX = OY = -6.0
    SPEED = 0.2
    LOOP = ((-4.25, -4.25), (4.25, -4.25), (4.25, 4.25), (-4.25, 4.25))
    SEEN_RADIUS_M = 3.0

    def __init__(self, now: float):
        c = self.OX + (np.arange(self.N) + 0.5) * self.RES
        X, Y = np.meshgrid(c, c)                        # rows = y, from min y up
        ax, ay = np.abs(X), np.abs(Y)
        inside = (ax < 5.6) & (ay < 5.6)
        outer = inside & ((ax >= 5.4) | (ay >= 5.4))
        block = (ax <= 3.0) & (ay <= 3.0)
        block_wall = block & ((ax >= 2.85) | (ay >= 2.85))
        bin_box = (X >= -5.25) & (X <= -4.95) & (Y >= 0.3) & (Y <= 0.7)   # off the loop line
        truth = np.full((self.N, self.N), -1, dtype=np.int8)
        free = inside & ~outer & ~block
        truth[free] = 0
        d_outer = 5.4 - np.maximum(ax, ay)
        d_block = np.hypot(np.maximum(ax - 3.0, 0), np.maximum(ay - 3.0, 0))
        d_bin = np.hypot(np.maximum(np.abs(X + 5.1) - 0.15, 0), np.maximum(np.abs(Y - 0.5) - 0.2, 0))
        d = np.minimum(np.minimum(d_outer, d_block), d_bin)
        ramp = free & (d < 0.4)
        truth[ramp] = np.clip(np.rint(99 * (1 - d[ramp] / 0.4)), 1, 99).astype(np.int8)
        truth[outer | block_wall | bin_box] = 100
        self.X, self.Y, self.truth = X, Y, truth
        self.seen = np.zeros_like(truth, dtype=bool)
        self.x, self.y, self.yaw = self.LOOP[0][0], self.LOOP[0][1], 0.0
        self.corner = 1
        self.goal = None
        self.goal_active = False
        self.goal_reached = None
        self.hold_until = 0.0
        self.moving = False
        self.travelled = 0.0
        self.last = now
        self.map_stamp = now
        self._reveal(now)

    def _reveal(self, now: float) -> None:
        self._revealed_at = (self.x, self.y)
        near = np.hypot(self.X - self.x, self.Y - self.y) <= self.SEEN_RADIUS_M
        if not (near & ~self.seen).any():
            return
        self.seen |= near
        self.map_stamp = now

    def cells(self) -> np.ndarray:
        return np.where(self.seen, self.truth, np.int8(-1)).astype(np.int8)

    def target(self):
        return self.goal if self.goal_active else self.LOOP[self.corner]

    def step(self, now: float, may_patrol: bool) -> None:
        dt = min(max(now - self.last, 0.0), 1.0)
        self.last = now
        if not self.goal_active and (not may_patrol or now < self.hold_until):
            self.moving = False
            return
        tx, ty = self.target()
        dist = math.hypot(tx - self.x, ty - self.y)
        stride = self.SPEED * dt
        if dist > 1e-9:
            self.yaw = math.atan2(ty - self.y, tx - self.x)
        if dist <= stride:
            self.x, self.y = tx, ty
            if self.goal_active:
                self.goal_active, self.goal_reached = False, True
                self.hold_until = now + 5.0
                self.corner = min(range(4), key=lambda i: math.hypot(self.LOOP[i][0] - tx, self.LOOP[i][1] - ty))
            else:
                self.corner = (self.corner + 1) % 4
        else:
            self.x += stride * (tx - self.x) / dist
            self.y += stride * (ty - self.y) / dist
        self.travelled += min(dist, stride)
        self.moving = stride > 0
        if math.hypot(self.x - self._revealed_at[0], self.y - self._revealed_at[1]) > 0.25:
            self._reveal(now)

    def scan(self, max_range_m: float = 8.0) -> np.ndarray:
        """360 rays at 1 deg, ray-marched on the true walls (what a lidar would see)."""
        a = np.radians(np.arange(0.0, 360.0, 1.0))
        r = np.arange(1, int(max_range_m / self.RES)) * self.RES
        px = self.x + np.cos(a)[:, None] * r
        py = self.y + np.sin(a)[:, None] * r
        col = np.floor((px - self.OX) / self.RES).astype(int)
        row = np.floor((py - self.OY) / self.RES).astype(int)
        inb = (col >= 0) & (col < self.N) & (row >= 0) & (row < self.N)
        occ = np.zeros(px.shape, dtype=bool)
        occ[inb] = self.truth[row[inb], col[inb]] == 100
        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)
        idx = np.nonzero(hit)[0]
        return np.column_stack((px[idx, first[idx]], py[idx, first[idx]])).astype(np.float32)


# The demo's decisions: one exploration cycle as the stack's modules log it, in the
# order a real run logs them (goal, obstacle and replan, contact, bump and wall,
# escape, goal dropped, next goal, arrival). The lines go into the demo's log and
# through decisions_from_records, like a real main.jsonl.
DEMO_DECISION_CYCLE_S = 16.0
DEMO_DECISION_SCRIPT = (   # (seconds into the cycle, level, logger, line)
    (0.0, "info", "explorer2.py", "goal {goal}: ({gx:.2f}, {gy:.2f}) {dist:.1f} m away, {cells} frontier cells, "
                                  "14 clusters, chosen in 120 ms"),
    (0.1, "info", "global_planner.py", "Got new goal"),
    (0.2, "info", "global_planner.py", "Found safe goal."),
    (4.0, "info", "local_planner.py", "Obstacle detected ahead, stopping local planner."),
    (4.1, "info", "global_planner.py", "Replanning path due to obstacle found."),
    (4.2, "info", "global_planner.py", "Replanning."),
    (4.3, "info", "global_planner.py", "Found safe goal."),
    (8.0, "warning", "moss_adapter.py", "MOSS base: STALL #{stall}: tracks commanded L=+60% R=+60% but the encoders "
                                        "moved L=+2 R=+1 ticks in 1.0 s - stopped (the encoders are the bumper)"),
    (8.3, "warning", "recovering_planner.py", "BUMP received"),
    (8.3, "info", "costmap2d.py", "costmap: contact at ({x:+.2f}, {y:+.2f}) heading {deg:+.0f} deg: "
                                  "32 cells ahead marked as a wall"),
    (10.4, "warning", "recovering_planner.py", "Contact escape"),
    (10.5, "info", "explorer2.py", "planner gave up on that goal: excluded until the map or the viewpoint changes"),
    (10.7, "info", "explorer2.py", "goal {goal2}: ({gx:.2f}, {gy:.2f}) {dist:.1f} m away, {cells} frontier cells, "
                                   "14 clusters, chosen in 130 ms"),
    (14.0, "info", "local_planner.py", "Reached goal position, starting final rotation"),
)
DEMO_LINE_FIELDS = {   # the fields a real record carries with its line, where a decision reads them
    "Contact escape": {"trigger": "bump", "direction": "reverse", "travelled_m": 0.201, "seconds": 2.1,
                       "aborted": False},
    "BUMP received": {"reflex": "escape reverse 0.20 m", "acted": True},
}
DEMO_EXPLORE_START = ("exploration start: distance exp 2, cap 3 m, radius 4 m "
                      "(0 = no limit; the radius is centred on the first pose)")
DEMO_STOP_LINES = {   # how the explorer logs each way of being stopped
    "cmd": ("Received exploration stop command via LCM", "Stopped autonomous frontier exploration"),
    "stop": ("WavefrontFrontierExplorer: stop_movement received, stopping exploration",
             "Stopped autonomous frontier exploration"),
}


def _iso_ms(t: float) -> str:
    """Epoch seconds as dimOS stamps its records: ISO, UTC, milliseconds, a Z."""
    return _dt.datetime.fromtimestamp(t, _dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class DemoCockpit(Cockpit):
    """Same endpoints, synthetic sources, never touches LCM or the launcher; the
    machine metrics are real."""

    mode = "demo"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        super().__init__(fly_script)
        now = time.time()
        self._lock = threading.Lock()
        self.world = DemoWorld(now)
        self.stack = {"running": True, "blueprint": "moss-explore", "started": now}
        self.exploring = True
        self.explore_done = False
        self.halted = False
        self.stack_stopped_at = None
        self._map_cache = (None, None)
        self._log: deque = deque(maxlen=200)
        self._launch_lines: deque = deque(maxlen=LAUNCH_TAIL)
        self._launch_action = None
        self._launch_note = None               # {text, ts}: why the demo stopped its own stack
        self._preflight_ts = now
        self._decisions = DecisionLog()
        self._script_t0 = None                 # the scripted exploration's start; None = nothing scripted runs
        self._script_pos = (0, 0)              # (cycle, step) of the next scripted line
        self._goal_said = True                 # the arrival at a tapped goal is logged once
        self._pending: list = []               # (at, level, logger, line) logged once their time comes
        self._say("info", "demo cockpit: synthetic MOSS, nothing on the bus")
        self._say("info", "RPLIDAR C1 up (demo)")
        self._say("info", "lidar odometry up (demo)")
        self._start_script(now)

    def _say(self, level: str, event: str) -> None:
        self._log.append({"ts": time.strftime(_ISO), "level": level, "event": event, "logger": "demo"})

    # the decisions, scripted (called with self._lock held) ---------------------
    def _line(self, at: float, level: str, logger: str, event: str) -> None:
        """One line as a stack module logs it: into the demo log and the decision feed."""
        rec = {"ts": _iso_ms(at), "level": level, "event": event, "logger": logger}
        rec.update(DEMO_LINE_FIELDS.get(event, {}))
        self._log.append(rec)
        self._decisions.add([rec])

    def _start_script(self, now: float) -> None:
        self._line(now, "info", "explorer2.py", DEMO_EXPLORE_START)
        self._script_t0, self._script_pos = now + 0.5, (0, 0)

    def _stop_script(self, now: float, how: str) -> None:
        if self._script_t0 is None:
            return
        for event in DEMO_STOP_LINES[how]:
            self._line(now, "info", "wavefront_frontier_goal_selector.py", event)
        self._script_t0 = None

    def _script_values(self, cycle: int) -> dict:
        w = self.world
        tx, ty = w.target()
        return {"goal": 2 * cycle + 1, "goal2": 2 * cycle + 2, "stall": cycle + 1, "gx": tx, "gy": ty,
                "dist": max(0.1, math.hypot(tx - w.x, ty - w.y)), "cells": 20 + (7 * cycle) % 40,
                "x": w.x, "y": w.y, "deg": math.degrees(w.yaw)}

    def _script_tick(self, now: float) -> None:
        """Log every scripted line that is due; a page away for long skips to the last cycle."""
        if self._script_t0 is None:
            return
        cycle, step = self._script_pos
        if now - (self._script_t0 + cycle * DEMO_DECISION_CYCLE_S) > 2 * DEMO_DECISION_CYCLE_S:
            cycle, step = int((now - self._script_t0) // DEMO_DECISION_CYCLE_S) - 1, 0
        while True:
            offset, level, logger, line = DEMO_DECISION_SCRIPT[step]
            at = self._script_t0 + cycle * DEMO_DECISION_CYCLE_S + offset
            if at > now:
                break
            self._line(at, level, logger, line.format(**self._script_values(cycle)))
            step += 1
            if step == len(DEMO_DECISION_SCRIPT):
                cycle, step = cycle + 1, 0
        self._script_pos = (cycle, step)

    def _run_id(self) -> str:
        return (time.strftime("%Y%m%d-%H%M%S-demo-", time.localtime(self.stack["started"]))
                + self.stack["blueprint"])

    def _tick(self) -> float:
        now = time.time()
        with self._lock:
            if not self.stack["running"]:            # no stack, no motion, not even toward a goal
                self.world.last, self.world.moving = now, False
                return now
            may = not self.halted and (self.exploring or self.stack["blueprint"] == "moss-nav")
            self.world.step(now, may)
            self._script_tick(now)
            for item in [p for p in self._pending if p[0] <= now]:
                self._pending.remove(item)
                self._line(*item)
            if not self._goal_said and not self._pending and self.world.goal_reached:
                self._goal_said = True
                self._line(now, "info", "local_planner.py", "Reached goal position, starting final rotation")
            if self.world.moving:
                self.note_activity("demo")            # the demo rover drives: it is not idle
            msg = self._idle_due(now)
        if msg:
            # the page's Stop stack path, then the note that says why
            self.command("stack", {"action": "stop"})
            self._launch_note = {"text": msg, "ts": now}
        return now

    def _idle_due(self, now: float) -> str | None:
        """The idle rule on the demo stack (self._lock held): its message once per run when due.
        The demo has no thread of its own: the stop lands on the first request past the limit."""
        if not self._idle_rule(self._stack_doc(now))["due"]:
            return None
        if self._idle_stopped == self._run_id():
            return None
        self._idle_stopped = self._run_id()
        msg = idle_stop_message(self.idle_limit_s)
        self._say("info", msg)
        self._launch_lines.append(f"=== {msg}  {time.strftime(_ISO)} === (demo)")
        return msg

    def _stack_doc(self, now: float) -> dict:
        """The demo stack as /state.json gives it."""
        if not self.stack["running"]:
            return {"running": False}
        return {"running": True, "run_id": self._run_id(), "blueprint": self.stack["blueprint"], "pid": os.getpid(),
                "started_at": _dt.datetime.fromtimestamp(self.stack["started"], _dt.timezone.utc).isoformat(),
                "uptime_s": round(now - self.stack["started"], 1), "log_dir": None}

    def state(self) -> dict:
        now = self._tick()
        w = self.world
        running = self.stack["running"]
        up = now - self.stack["started"]
        stack = self._stack_doc(now)
        silent = 0.0 if running else now - (self.stack_stopped_at or now)
        nominal = {"odom": 10.0, "global_costmap": 1.0, "lidar": 10.0, "pointcloud": 10.0, "path": 1.0,
                   "cmd_vel": 10.0, "base_telemetry": 2.0}
        for topic, _ in CAMERAS.values():
            nominal[topic] = 5.0
        topics = {k: {"age_s": round(silent + 1.0 / hz / 2, 2), "hz": hz if running else 0.0}
                  for k, hz in nominal.items()}
        volts = max(11.0, 11.6 - 0.0005 * (now - self.started))
        drive = w.moving and running
        pct = 40 if drive else 0
        telemetry = {"fw": "qtpy-mdd3a-v3", "driver": "mdd3a", "mode": 2 if drive else 0,
                     "reason": "drive" if drive else "boot", "bus_mV": int(volts * 1000), "ina_ok": True,
                     "ticks": [int(w.travelled * 1000), int(w.travelled * 1000)],
                     "target_pct": [pct, pct], "output_pct": [pct, pct], "ramp_ms": [5, 5],
                     "firmware_age_s": 0.1, "mock": True, "stalled": False, "stall_count": 0}
        return {
            "ts": now, "mode": self.mode, "stack": stack,
            "idle": self.idle_doc(stack),
            "pose": self._pose_doc(),
            "topics": topics,
            "base": base_view(telemetry, 0.3) if running else None,
            "nav": {"exploring": self.exploring and running, "explore_done": self.explore_done,
                    "goal": None if w.goal is None else {"x": round(w.goal[0], 3), "y": round(w.goal[1], 3)},
                    "goal_reached": w.goal_reached,
                    "last_cmd_vel": {"vx": w.SPEED if drive else 0.0, "wz": 0.0, "age_s": 0.1}},
            "recording": {"db": os.path.join(state_dir(), "recordings", "moss.db") + " (demo)",
                          "exists": True, "size_mb": round(0.5 + 0.01 * up, 1), "episode": self.episode},
            "system": system_metrics(),
            "bus": {"transport": "demo", "listening": False, "error": None},
            "camera": self.camera_state(),
            "launch": {"running": False, "action": self._launch_action, "note": self._launch_note},
        }

    def _pose_doc(self) -> dict | None:
        w = self.world
        return ({"x": round(w.x, 3), "y": round(w.y, 3), "yaw": round(w.yaw, 4), "age_s": 0.05}
                if self.stack["running"] else None)

    def pose(self) -> dict | None:
        """The pose as of the last tick: scan_doc ticks through scan(), so the scan's
        points and this pose come from the same instant."""
        with self._lock:
            return self._pose_doc()

    def map(self):
        self._tick()
        with self._lock:
            stamp = self.world.map_stamp
            if self._map_cache[0] != stamp:
                self._map_cache = (stamp, render_grid(self.world.cells(), DemoWorld.RES, DemoWorld.OX,
                                                      DemoWorld.OY, stamp, "world"))
            return self._map_cache[1]

    def scan(self) -> dict | None:
        now = self._tick()
        if not self.stack["running"]:
            return None
        with self._lock:
            xy = self.world.scan()
        # the demo world is lidar only: its depth camera sees no obstacle
        return {"frame": "world", "stamp": now, "source": "demo", "age_s": 0.05,
                "points": np.round(downsample(xy).astype(float), 3).tolist(), "n": int(len(xy)),
                "camera": [], "camera_n": 0}

    def path(self) -> dict:
        self._tick()
        w = self.world
        if not self.stack["running"] or not (w.goal_active or w.moving):
            return {"points": [], "age_s": 0.5}
        tx, ty = w.target()
        return {"points": [[round(w.x, 3), round(w.y, 3)], [round(tx, 3), round(ty, 3)]], "age_s": 0.5}

    def log(self, n: int) -> list[dict]:
        return list(self._log)[-n:]

    def decisions(self, n: int) -> dict:
        now = self._tick()
        with self._lock:
            running = self.stack["running"]
            # a stopped stack keeps its run's events, as live mode does
            return decisions_doc(list(self._decisions.events), n, running, running and self.exploring, now,
                                 self._run_id() if running or self._decisions.events else None)

    def preflight(self) -> dict:
        checks = [("stack", "stack: none", ""), ("ports", "QT Py present", "by-id"),
                  ("ports", "C1 lidar present", "by-id"), ("firmware", "telemetry", "qtpy-mdd3a-v3, mode 0 (off)"),
                  ("firmware", "battery", "11.62 V (INA219)"), ("lidar", "C1 spins", "1480 pts/s, 36/36 sectors"),
                  ("camera", "RealSense D455 on USB 3", "demo"), ("camera", "wrist camera", "demo"),
                  ("gamepad", "gamepad js0", "present"), ("system", "free space", "79 GB"),
                  ("system", "memory available", "3.1 GB"), ("software", "entry points", "moss-nav, moss-explore")]
        rows = [{"ok": True, "label": label, "detail": detail, "section": section}
                for section, label, detail in checks]
        return {"available": True, "demo": True, "ts": self._preflight_ts, "checks": rows,
                "ok_count": len(rows), "ko_count": 0, "summary": f"{len(rows)}/{len(rows)} OK - ready to fly"}

    def launch(self) -> dict:
        return {"running": False, "action": self._launch_action, "started_at": None, "returncode": 0,
                "note": self._launch_note, "lines": list(self._launch_lines)}

    def camera_frame(self, cam: str = "nose"):
        # a new key every period, so the stream really runs at 5 fps; both demo cameras
        # show the demo frame
        return int(time.time() / MJPEG_PERIOD_S), DEMO_JPEG

    def command(self, name: str, body: dict) -> dict:
        self._tick()
        with self._lock:
            w = self.world
            if name == "keepalive":
                return self.keepalive(self._stack_doc(time.time()))
            if name == "explore":
                action = body.get("action")
                if action not in ("start", "stop"):
                    raise ValueError('explore needs "action": "start" or "stop"')
                was = self.exploring
                self.exploring = action == "start"
                self.halted = False if self.exploring else self.halted
                self._say("info", f"demo: exploration {'started' if self.exploring else 'stopped'}")
                if self.exploring and not was and self.stack["running"] and self.stack["blueprint"] == "moss-explore":
                    self._start_script(time.time())
                elif not self.exploring:
                    self._stop_script(time.time(), "cmd")
                return {"ok": True, "sent": "/explore_cmd#std_msgs.Bool" if self.exploring
                        else "/stop_explore_cmd#std_msgs.Bool"}
            if name == "map":
                if body.get("action") != "clear":
                    raise ValueError('map needs "action": "clear"')
                w.seen[:] = False                       # the demo forgets what it mapped and maps it again
                self._map_cache = (None, None)
                self._say("info", "demo: map cleared")
                return {"ok": True, "sent": "/clear_map_cmd#std_msgs.Bool"}
            if name == "goal":
                x, y = _finite_xy(body)
                self.exploring = False
                self.halted = False
                w.goal, w.goal_active, w.goal_reached = (x, y), True, None
                self._say("info", f"demo: goal ({x:+.2f}, {y:+.2f})")
                # exploring: the page stops the explorer first and the goal goes out GOAL_AFTER_STOP_S later
                now = time.time()
                at = now + GOAL_AFTER_STOP_S if self._script_t0 is not None else now
                self._stop_script(now, "cmd")
                self._pending = [(at, "info", "global_planner.py", "Got new goal"),
                                 (at, "info", "global_planner.py", "Found safe goal.")]
                self._goal_said = False
                return {"ok": True, "sent": "/goal_request#geometry_msgs.PoseStamped",
                        "also": ["/stop_explore_cmd#std_msgs.Bool"], "goal": {"x": x, "y": y}}
            if name == "stop":
                self.exploring, self.halted, w.goal_active = False, True, False
                self._say("warning", "demo: soft stop")
                self._stop_script(time.time(), "stop")
                self._pending, self._goal_said = [], True
                return {"ok": True, "sent": "/tele_cmd_vel#geometry_msgs.Twist",
                        "also": ["/stop_explore_cmd#std_msgs.Bool", "/stop_movement#std_msgs.Bool"]}
            if name == "episode":
                marker = self._episode(body)
                self._say("info", f"demo: episode {marker['action']} {marker['name']}")
                return {"ok": True, "sent": "/episode_marker#std_msgs.String", "episode": self.episode}
            if name == "stack":
                action = body.get("action")
                if action not in ("map", "explore", "stop", "check"):
                    raise ValueError('stack needs "action": "map", "explore", "stop" or "check"')
                now = time.time()
                self._launch_action = action
                self._launch_note = None
                self._launch_lines.append(f"=== moss-fly.sh {action}  {time.strftime(_ISO)} === (demo)")
                if action != "check" and not (action in ("map", "explore") and self.stack["running"]):
                    self._script_t0, self._pending, self._goal_said = None, [], True
                    if action != "stop":
                        self._decisions.clear()        # a new stack: a fresh feed; a stop keeps the last run's
                if action in ("map", "explore"):
                    if self.stack["running"]:
                        return {"ok": False, "error": "a dimOS stack is already running: stop it first"}
                    self.stack = {"running": True, "started": now,
                                  "blueprint": "moss-nav" if action == "map" else "moss-explore"}
                    self.exploring, self.halted = False, False
                elif action == "stop":
                    self.stack = dict(self.stack, running=False)
                    self.stack_stopped_at = now
                    self.exploring = False
                else:
                    self._preflight_ts = now
                self._say("info", f"demo: moss-fly.sh {action}")
                return {"ok": True, "sent": f"moss-fly.sh {action}"}
        return {"ok": False, "error": f"unknown command {name!r}", "_code": 404}


# --- HTTP ---------------------------------------------------------------------------

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8", ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
    ".ttf": "font/ttf", ".otf": "font/otf", ".woff": "font/woff", ".woff2": "font/woff2",
    ".txt": "text/plain; charset=utf-8", ".ico": "image/x-icon",
}
PLACEHOLDER_PAGE = (b"<!doctype html><meta charset=utf-8><title>MOSS cockpit</title>"
                    b"<p>MOSS cockpit server is up; the page (tools/moss_cockpit/index.html) is not installed."
                    b" Try <a href=/state.json>/state.json</a>.</p>")


def json_safe(obj):
    """NaN/inf -> null (JSON.parse rejects NaN; a NaN goal is how dimOS cancels one),
    numpy scalars -> Python numbers."""
    if isinstance(obj, dict):
        return {str(k): json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    if isinstance(obj, np.generic):
        obj = obj.item()
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


class Handler(BaseHTTPRequestHandler):
    server_version = "moss-cockpit/1.0"
    protocol_version = "HTTP/1.1"

    @property
    def cockpit(self) -> Cockpit:
        return self.server.cockpit

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code: int, payload) -> None:
        self._send(code, json.dumps(json_safe(payload), allow_nan=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _static(self, rel: str) -> None:
        root = os.path.realpath(self.server.page_dir)
        full = os.path.realpath(os.path.join(root, unquote(rel)))
        if not full.startswith(root + os.sep) or not os.path.isfile(full):
            self._json(404, {"error": "nothing at this address"})
            return
        ctype = CONTENT_TYPES.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
        with open(full, "rb") as fh:
            self._send(200, fh.read(), ctype)

    def do_GET(self) -> None:  # noqa: N802
        url = urlsplit(self.path)
        path = url.path
        try:
            if path in ("/", "/index.html"):
                index = os.path.join(self.server.page_dir, "index.html")
                if os.path.isfile(index):
                    with open(index, "rb") as fh:
                        self._send(200, fh.read(), "text/html; charset=utf-8")
                else:
                    self._send(200, PLACEHOLDER_PAGE, "text/html; charset=utf-8")
            elif path.startswith("/static/"):
                self._static(path[len("/static/"):])
            elif path == "/state.json":
                self._json(200, self.cockpit.state())
            elif path in ("/map.png", "/map_meta"):
                got = self.cockpit.map()
                if got is None:
                    self._json(404, {"error": "no costmap yet: is a mapping stack running?"})
                elif path == "/map.png":
                    self._send(200, got[0], "image/png")
                else:
                    self._json(200, got[1])
            elif path == "/scan.json":
                self._json(200, self.cockpit.scan_doc())
            elif path == "/path.json":
                self._json(200, self.cockpit.path())
            elif path == "/log.json":
                try:
                    n = int(parse_qs(url.query).get("n", ["40"])[0])
                except ValueError:
                    n = 40
                self._json(200, {"lines": self.cockpit.log(max(1, min(n, 500)))})
            elif path == "/decisions.json":
                try:
                    n = int(parse_qs(url.query).get("n", ["40"])[0])
                except ValueError:
                    n = 40
                self._json(200, self.cockpit.decisions(max(1, min(n, DECISIONS_KEEP))))
            elif path == "/preflight.json":
                self._json(200, self.cockpit.preflight())
            elif path == "/launch.json":
                self._json(200, self.cockpit.launch())
            elif path == "/camera.mjpg":
                cam = parse_qs(url.query).get("cam", ["nose"])[0]
                if cam not in CAMERAS:
                    self._json(404, {"error": f"no camera called {cam!r}: " + " or ".join(CAMERAS)})
                else:
                    self._mjpeg(cam)
            else:
                self._json(404, {"error": f"nothing at this address: {path}"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:  # noqa: BLE001 - a page reload must never kill the server
            self.log_error("GET %s failed: %r", path, exc)
            try:
                self._json(500, {"error": f"{type(exc).__name__}: {exc}"})
            except OSError:
                pass

    do_HEAD = do_GET

    def _peer_gone(self) -> bool:
        """The viewer closed its socket. Checked while no frame is due: with a silent
        camera nothing is written, so a write error would never tell us, and the
        camera would stay subscribed for a viewer long gone."""
        try:
            readable, _, _ = select.select([self.connection], [], [], 0)
            return bool(readable) and self.connection.recv(1, socket.MSG_PEEK) == b""
        except (OSError, ValueError):
            return True

    def _mjpeg(self, cam: str = "nose") -> None:
        c = self.cockpit
        try:
            c.camera_open(cam)
        except CameraUnavailable as exc:
            self._json(503, {"error": str(exc)})
            return
        try:
            # a viewer that vanished without FIN/RST (phone off the WiFi, laptop asleep) blocks
            # the write once the buffers fill: time out and free its slot instead of waiting for TCP
            self.connection.settimeout(MJPEG_WRITE_TIMEOUT_S)
            frame = None
            deadline = time.monotonic() + FIRST_FRAME_WAIT_S
            try:
                while frame is None and time.monotonic() < deadline and not self.server.stopping:
                    frame = c.camera_frame(cam)
                    if frame is None:
                        time.sleep(0.05)
            except CameraUnavailable as exc:
                self._json(503, {"error": str(exc)})
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            if self.command == "HEAD":
                return
            sent_key, next_at = None, 0.0
            while not self.server.stopping:
                if frame is None or frame[0] == sent_key or time.monotonic() < next_at:
                    if self._peer_gone():
                        return
                    time.sleep(0.03)
                    try:
                        frame = c.camera_frame(cam)
                    except CameraUnavailable:
                        return
                    continue
                key, jpeg = frame
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
                sent_key, next_at = key, time.monotonic() + MJPEG_PERIOD_S
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                                  # the viewer left
        finally:
            c.camera_close(cam)

    def do_POST(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if not path.startswith("/cmd/"):
            self._json(404, {"ok": False, "error": f"nothing at this address: {path}"})
            return
        name = path[len("/cmd/"):]
        if self.headers.get(CMD_HEADER[0]) != CMD_HEADER[1]:
            # a cross-site page cannot set this header without a CORS preflight, which this
            # server never grants (no OPTIONS): only the cockpit page and deliberate scripts reach here
            self.close_connection = True          # the body is not read: do not reuse the connection
            self._json(403, {"ok": False, "error": f"refused: POST /cmd/* needs the header "
                                                   f"'{CMD_HEADER[0]}: {CMD_HEADER[1]}'"})
            return
        self.cockpit.note_activity("post")        # the operator is there: the idle count restarts
        try:
            length = int(self.headers.get("Content-Length") or "0")
            if length < 0 or length > MAX_BODY:
                raise ValueError(f"body of {length} bytes refused")
            raw = self.rfile.read(length) if length else b""
            body = json.loads(raw.decode("utf-8")) if raw.strip() else {}
            if not isinstance(body, dict):
                raise ValueError("the body must be a JSON object")
            answer = self.cockpit.command(name, body)
            code = answer.pop("_code", 200)
            self._json(code, answer)
        except ValueError as exc:                 # includes a JSON decode error
            self._json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self.log_error("POST %s failed: %r", path, exc)
            self._json(500, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    def log_message(self, *args) -> None:
        pass                                      # quiet: errors still go through log_error


class CockpitServer(ThreadingHTTPServer):
    daemon_threads = True                         # an open MJPEG stream must not block exit
    allow_reuse_address = True

    def __init__(self, addr, cockpit: Cockpit, page_dir: str = PAGE_DIR):
        super().__init__(addr, Handler)
        self.cockpit = cockpit
        self.page_dir = page_dir
        self.stopping = False

    def shutdown(self) -> None:
        self.stopping = True
        super().shutdown()


def make_server(bind: str = DEFAULT_BIND, port: int = DEFAULT_PORT, demo: bool = False,
                page_dir: str = PAGE_DIR, fly_script: str = FLY_SCRIPT, listen: bool = True) -> CockpitServer:
    """The server, built but not serving (port 0 = any free port: the bench uses that)."""
    cockpit = DemoCockpit(fly_script) if demo else LiveCockpit(fly_script)
    if not demo and listen:
        cockpit.start_listener()
    if any(os.path.exists(p) for p in GPU_LOAD_PATHS):
        threading.Thread(target=_gpu_sampler, name="gpu-sampler", daemon=True).start()
    return CockpitServer((bind, port), cockpit, page_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MOSS cockpit web server")
    ap.add_argument("--port", type=int, default=int(os.environ.get("MOSS_COCKPIT_PORT", DEFAULT_PORT)))
    ap.add_argument("--bind", default=DEFAULT_BIND)
    ap.add_argument("--demo", action="store_true", help="synthetic rover: no bus, no stack, no launcher")
    args = ap.parse_args(argv)
    server = make_server(args.bind, args.port, demo=args.demo)
    cockpit = server.cockpit
    if not args.demo and isinstance(cockpit, LiveCockpit) and cockpit.bus_error:
        print(f"warning: {cockpit.bus_error} - the page will show no live data", flush=True)
    print(f"MOSS cockpit ({cockpit.mode}) on http://{args.bind}:{server.server_address[1]}/", flush=True)
    limit = cockpit.idle_limit_s
    print(f"idle stop: a moss-nav or moss-explore stack idle {limit:g} s is stopped" if limit > 0 else
          "idle stop: off (MOSS_IDLE_STOP_S=0)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
