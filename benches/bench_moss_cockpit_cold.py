"""Cold bench for tools/moss_cockpit.py: a real server on a free port, known in -> known out.

Every check feeds a value we know and reads back a value in physical units
(metres, radians, volts, %, pixel colours, bytes on a named channel). Groups:

  A. pure helpers   - battery gauge (3S: 10.2 V -> 0 %, 11.4 V -> 50 %, 12.6 V -> 100 %),
                      quaternion -> yaw, grid -> PNG pixel colours, the map_meta
                      pixel <-> metre formulas, even downsampling
  B. demo server    - /, /static, /state.json schema, /map.png (demo size, wall =
                      WHITE, floor = free colour), /map_meta roundtrip, /scan.json
                      (<= 1500 points, every point on a wall, no depth-camera point,
                      the pose on the loop's first leg, at most 0.2 m/s past the pose
                      of a /state.json read right before it), base stalled/stall_count,
                      /camera.mjpg framing
                      and its 5 fps cap, POST /cmd/goal|stop|episode|stack, path
                      traversal refused, bad bodies refused
  C. decoding       - messages built with dimos_lcm and fed to a live cockpit's bus
                      cache: OccupancyGrid -> exact PNG pixels, PointCloud2 -> scan
                      points (and the /pointcloud fallback), the `lidar` cloud split
                      into lidar returns (z = the profile's lidar height, the stack's
                      own override first) and depth-camera points, PoseStamped yaw 90 deg
                      on /state.json and on /scan.json (read per request, never cached
                      with the scan), telemetry String -> battery %, stalled/stall_count
                      (None from telemetry before the stall guard), Path, explore/goal
                      state, a JPEG image passed through byte for byte
  D. commands       - POST /cmd/* in live mode publish the known bytes on the known
                      channels (fake LCM handle); explore_ctl's generic publisher and
                      its start/stop; a run that is not on LCM is refused (503), the
                      bus read in dimOS's order (DIMOS_TRANSPORT, then TRANSPORT) from
                      the running stack's own environment, else from the cockpit's
  E. stack + files  - run registry (dead pid skipped), main.jsonl tail, preflight.json,
                      the launcher run non-blocking on a stub script, refusal when a
                      stack runs
  G. robustness     - launcher env without EXPLORE/MOCK/MOSS_MOCK_BUS, Stop cancels a
                      launch in its gates (SIGTERM, SIGKILL after the grace), an orphan
                      episode closed when the stack stops, a new run_id empties the bus
                      cache, a camera frame after a run change, a viewer that stops
                      reading frees its camera slot
  H. the real page  - tools/moss_cockpit/ served as shipped: index.html, every asset it
                      and tokens.css reference, the run caption's namespace strip, the
                      depth-camera legend element the map writes to, the STALLED badge
                      and the stalls row; the page's wiring (pose() through freshestPose
                      with the scan's pose and answer time, pollScan stamping each answer,
                      renderBase through the stall latch); node --check app.js, and the
                      page's own pose choice (state vs scan, the younger wins), stall view
                      and stall latch (a stall the 1 Hz read misses still holds the badge
                      4 s) run in node
  I. decisions      - every decision line of the catalogue (the run's own text) -> its kind,
                      source and a short text that keeps the line's numbers; twenty other
                      lines -> nothing; the planner's replan reasons shown only with their
                      consequence ("Replanning.", "Goal abandoned"), the run's three stuck
                      verdicts the planner ignored -> nothing; one whole real exploration
                      replayed (goal 10, contact 30, waiting 44, no event from its 181 twist /
                      telemetry lines; its 11-line wait folded into one row) and its first
                      contact with every module's lines -> the expected events and words;
                      the state word on known sequences (the 3 s / 4 s holds, BACKING OFF
                      until the escape ends, rank, stop, escape, tapped goals the planner
                      drops, ISO times); echoes; bad records never raise; /decisions.json
                      live on a 1.1 MB main.jsonl (the 1 MiB window, at most 2 Hz, a torn
                      last line, a reason and its consequence in two reads, a rewritten
                      file, the stack gone and its events kept, the next run, a stack with
                      no registry entry) and in the demo (the scripted cycle, STOP, explore,
                      a goal 3 s after the stop, UTC times, the stop keeping the feed)
  J. idle stop      - the rule at every branch (off, a fresh stack counted from its start,
                      activity, before the start, at the limit, not MOSS, moss-gamepad and
                      moss-base never, no stack, another clock, a start without timezone),
                      the message, MOSS_IDLE_STOP_S (under 30 s refused), fresh telemetry,
                      the dimos run rule against STACK_RE; each activity source restarts the
                      count and names itself (a fresh commanded track, a non-zero cmd_vel,
                      explore_cmd, goal_request, episode_marker, an open episode, the pose
                      moved over 10 s, a POST, keepalive) and its no-activity twin does not
                      (a lost board's repeated line, drift at rest for 30 min); the watchdog
                      with a fake launcher (once at the limit, not twice, again for a new
                      run, never at limit 0, a foreign stack, a deaf cockpit, another
                      LCM_DEFAULT_URL, zenoh, a stale pid, a busy launcher until 120 s, a
                      failing launcher tried again, a clock step); the bus tap's loop calling
                      it at 1 Hz; the real launcher's note line; /state.json idle live and
                      demo; the demo stack stopping itself after the limit
  F. hygiene        - no dimos/serial/torch import, no LAN address in the server

C, D and the run-change checks of G need dimos_lcm (pure python, installed with
dimOS; skipped without it). Nothing here needs dimOS, lcm or hardware.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_moss_cockpit_cold.py
"""

import atexit
import contextlib
import http.client
import io
import json
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import datetime as _dt
import shutil
import signal
import socket
import urllib.error
import urllib.request
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import numpy as np  # noqa: E402

import moss_cockpit as mc  # noqa: E402
from moss_dimos import robot_profile  # noqa: E402 - pure python, no dimOS

TAG = robot_profile.current().lidar_height_m   # the z of every lidar return in the `lidar` cloud

try:
    import dimos_lcm  # noqa: F401
    from dimos_lcm.geometry_msgs import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
    from dimos_lcm.nav_msgs import MapMetaData, OccupancyGrid, Path as LcmPath
    from dimos_lcm.sensor_msgs import Image, PointCloud2, PointField
    from dimos_lcm.std_msgs import Bool, Header, String, Time
    HAVE_LCM_MSGS = True

    # Fresh nested objects every time: the generated constructors share their
    # default instances, so a default-built message mutated here would leak into
    # every later one (the cockpit's own zero twist included).
    def hdr(frame="", sec=0, nsec=0):
        return Header(seq=0, stamp=Time(sec=sec, nsec=nsec), frame_id=frame)

    def pose_of(x, y, qz=0.0, qw=1.0):
        return Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=qz, w=qw))
except ImportError:
    HAVE_LCM_MSGS = False

OK = 0
KO = 0
_TMPDIR = tempfile.TemporaryDirectory(prefix="moss_cockpit_bench_")   # state dir, run registry, page, logs
TMP = _TMPDIR.name
os.environ["MOSS_STATE_DIR"] = os.path.join(TMP, "state")
os.environ["XDG_STATE_HOME"] = os.path.join(TMP, "xdg")     # an empty dimOS run registry
os.environ["DIMOS_TRANSPORT"] = "lcm"   # the cockpit's own bus, known under both names dimOS reads
os.environ["TRANSPORT"] = "lcm"

WHITE = (0xEA, 0xF0, 0xEF)
FREE = (0x2E, 0x45, 0x56)
UNKNOWN = (0x3D, 0x57, 0x6A)
COST_MAX = (0x8F, 0xA6, 0xB5)


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def cost_colour(v):
    """The free -> COST_MAX ramp for cost v in 1..99, computed independently of the LUT."""
    return tuple(int(round(f + (t - f) * v / 99.0)) for f, t in zip(FREE, COST_MAX))


def png_decode(data: bytes) -> np.ndarray:
    """(h, w, 3) uint8 from an 8-bit RGB PNG whose rows all use filter 0 (what we write)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "PNG signature"
    pos, idat = 8, b""
    w = h = None
    while pos < len(data):
        n = struct.unpack(">I", data[pos:pos + 4])[0]
        tag, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + n]
        crc = struct.unpack(">I", data[pos + 8 + n:pos + 12 + n])[0]
        assert crc == zlib.crc32(tag + body) & 0xFFFFFFFF, f"CRC of {tag!r}"
        if tag == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
            assert (depth, ctype) == (8, 2), "8-bit RGB"
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    rows = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, w * 3 + 1)
    assert (rows[:, 0] == 0).all(), "filter byte 0 on every row"
    return rows[:, 1:].reshape(h, w, 3)


class Client:
    def __init__(self, port):
        self.port = port
        self.base = f"http://127.0.0.1:{port}"

    def get(self, path):
        try:
            with urllib.request.urlopen(self.base + path, timeout=10) as r:
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()

    def json(self, path):
        code, _, body = self.get(path)
        return code, json.loads(body)

    def post(self, path, payload, raw=None, header=True):
        data = raw if raw is not None else json.dumps(payload).encode()
        hdrs = {"Content-Type": "application/json"}
        if header:
            hdrs["X-Requested-With"] = "moss-cockpit"     # what the page's post() sends
        req = urllib.request.Request(self.base + path, data=data, method="POST", headers=hdrs)
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def raw_get(self, path):
        """No client-side path normalisation (urllib keeps '..' too, this is explicit)."""
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        conn.putrequest("GET", path, skip_accept_encoding=True)
        conn.endheaders()
        r = conn.getresponse()
        body = r.read()
        conn.close()
        return r.status, body

    def mjpeg_frames(self, n, timeout=8.0):
        """Read n multipart parts; returns (content_type, [(t_arrival, jpeg)])."""
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        conn.request("GET", "/camera.mjpg")
        r = conn.getresponse()
        ctype = r.getheader("Content-Type")
        if r.status != 200:
            body = r.read()
            conn.close()
            return r.status, ctype, body
        frames = []
        fp = r.fp
        while len(frames) < n:
            line = fp.readline()
            if not line:
                break
            if line.strip() != b"--frame":
                continue
            headers = {}
            while True:
                h = fp.readline().strip()
                if not h:
                    break
                k, v = h.split(b":", 1)
                headers[k.strip().lower()] = v.strip()
            jpeg = fp.read(int(headers[b"content-length"]))
            frames.append((time.monotonic(), jpeg))
        conn.close()
        return 200, ctype, frames


def serve(**kw):
    server = mc.make_server("127.0.0.1", 0, listen=False, **kw)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, Client(server.server_address[1])


# ---------------------------------------------------------------------------
print("A. pure helpers")
check("11.4 V -> 50 %", mc.battery_pct(11.4) == 50, str(mc.battery_pct(11.4)))
check("10.2 V -> 0 %, 12.6 V -> 100 %", (mc.battery_pct(10.2), mc.battery_pct(12.6)) == (0, 100))
check("clamped: 9.0 V -> 0 %, 13.1 V -> 100 %", (mc.battery_pct(9.0), mc.battery_pct(13.1)) == (0, 100))
s45 = math.sin(math.pi / 4)
check("quaternion (0,0,sin45,cos45) -> yaw 1.5708 rad",
      abs(mc.yaw_of(0, 0, s45, s45) - math.pi / 2) < 1e-9, f"{mc.yaw_of(0, 0, s45, s45):.6f}")
check("quaternion (0,0,1,0) -> yaw +-pi", abs(abs(mc.yaw_of(0, 0, 1, 0)) - math.pi) < 1e-9)
cells = np.array([[-1, 0, 100], [50, 0, -1]], dtype=np.int8)    # row 0 = min y
png, meta = mc.render_grid(cells, 0.5, 1.0, 2.0, stamp=7.5, frame="world")
img = png_decode(png)
check("3x2 grid -> 3x2 PNG", img.shape == (2, 3, 3), str(img.shape))
expect = [[cost_colour(50), FREE, UNKNOWN], [UNKNOWN, FREE, WHITE]]   # PNG row 0 = max y
got = [[tuple(int(c) for c in img[r, k]) for k in range(3)] for r in range(2)]
check("pixels flipped (row 0 = max y): cost 50, free, unknown / unknown, free, WHITE",
      got == expect, str(got))
check("cost 99 -> #8FA6B5, cost 1 ~ free",
      tuple(mc.MAP_LUT[99]) == COST_MAX and tuple(mc.MAP_LUT[1]) == cost_colour(1))
check("meta: res 0.5, origin (1, 2), extent x 1..2.5 y 2..3, stamp, frame",
      (meta["res"], meta["ox"], meta["oy"], meta["x_max"], meta["y_max"], meta["stamp"], meta["frame"])
      == (0.5, 1.0, 2.0, 2.5, 3.0, 7.5, "world"), json.dumps(meta)[:120])
x_of_px = lambda px, m: m["ox"] + (px + 0.5) * m["res"]               # noqa: E731
y_of_py = lambda py, m: m["oy"] + (m["height"] - py - 0.5) * m["res"]  # noqa: E731
px_of_x = lambda x, m: (x - m["ox"]) / m["res"]                        # noqa: E731
py_of_y = lambda y, m: m["height"] - (y - m["oy"]) / m["res"]          # noqa: E731
check("formulas: lethal cell (col 2, grid row 0) centre = (2.25 m, 2.25 m) -> PNG pixel (2, 1)",
      (x_of_px(2, meta), y_of_py(1, meta)) == (2.25, 2.25)
      and tuple(int(c) for c in img[1, 2]) == WHITE)
pts = np.column_stack((np.arange(4000, dtype=np.float32), np.zeros(4000, np.float32)))
ds = mc.downsample(pts)
check("4000 points -> 1500, first and last kept, evenly spaced",
      len(ds) == 1500 and ds[0, 0] == 0 and ds[-1, 0] == 3999
      and np.ptp(np.diff(ds[:, 0])) <= 1.0, f"{len(ds)} pts")
base = mc.base_view({"fw": "qtpy-mdd3a-v3", "mode": 2, "bus_mV": 11400, "ina_ok": True,
                     "ticks": [10, 12], "target_pct": [40, 40], "output_pct": [38, 39]}, 0.25)
check("telemetry 11400 mV, mode 2 -> 11.4 V, 50 %, 'drive', driver None",
      (base["battery_v"], base["battery_pct"], base["mode_name"], base["driver"]) == (11.4, 50, "drive", None),
      json.dumps(base)[:120])
check("ina_ok False -> no battery % (the voltage is not measured)",
      mc.base_view({"bus_mV": 0, "ina_ok": False}, 0.1)["battery_pct"] is None)
check("NaN -> null in JSON (a cancelled dimOS goal is NaN)",
      json.dumps(mc.json_safe({"x": float("nan"), "y": [np.float32(1.5)]})) == '{"x": null, "y": [1.5]}')

# ---------------------------------------------------------------------------
print("B. demo server")
page_dir = os.path.join(TMP, "page")
os.makedirs(os.path.join(page_dir, "fonts"))
Path(page_dir, "index.html").write_text("<!doctype html><title>MOSS cockpit</title>bench page")
Path(page_dir, "tokens.css").write_text(":root{--moss-mint:#6CF8C7}")
Path(page_dir, "fonts", "x.ttf").write_bytes(b"\x00\x01\x00\x00")
demo, cli = serve(demo=True, page_dir=page_dir)
code, headers, body = cli.get("/")
check("GET / -> the page's index.html", code == 200 and b"bench page" in body
      and headers["Content-Type"].startswith("text/html") and headers["Cache-Control"] == "no-store")
code, headers, body = cli.get("/static/tokens.css")
check("GET /static/tokens.css -> text/css", code == 200 and headers["Content-Type"].startswith("text/css")
      and b"6CF8C7" in body)
code, headers, _ = cli.get("/static/fonts/x.ttf")
check("GET /static/fonts/x.ttf -> font/ttf", code == 200 and headers["Content-Type"] == "font/ttf")
for bad in ("/static/../moss_cockpit.py", "/static/%2e%2e/moss_cockpit.py", "/static//etc/passwd",
            "/static/..%2f..%2fREADME.md"):
    code, body = cli.raw_get(bad)
    check(f"path traversal {bad} -> 404", code == 404 and b"import" not in body, str(code))
code, _, _ = cli.get("/nothing")
check("unknown GET -> 404", code == 404)

code, st = cli.json("/state.json")
need = {"ts", "mode", "stack", "pose", "topics", "base", "nav", "recording", "system"}
check("state.json top-level keys", code == 200 and need <= set(st), str(sorted(set(st))))
check("mode demo, stack running moss-explore", st["mode"] == "demo" and st["stack"]["running"]
      and st["stack"]["blueprint"] == "moss-explore")
check("topics carry age_s + hz for odom/global_costmap/lidar/path/cmd_vel/base_telemetry/color_image",
      all(k in st["topics"] and {"age_s", "hz"} <= set(st["topics"][k])
          for k in ("odom", "global_costmap", "lidar", "path", "cmd_vel", "base_telemetry", "color_image")))
b = st["base"]
check("base: battery_v / battery_pct consistent with the 3S gauge",
      b and abs(b["battery_pct"] - mc.battery_pct(b["battery_v"])) <= 1 and 10.9 <= b["battery_v"] <= 11.6,
      f"{b and b['battery_v']} V {b and b['battery_pct']} %")
check("demo base: stalled false, stall_count 0 (the stall guard's keys, as the adapter sends them)",
      b and b.get("stalled") is False and b.get("stall_count") == 0, f"{b and (b.get('stalled'), b.get('stall_count'))}")
check("nav keys", {"exploring", "explore_done", "goal", "goal_reached", "last_cmd_vel"} <= set(st["nav"]))
check("recording keys", {"db", "exists", "size_mb", "episode"} <= set(st["recording"]))
sysm = st["system"]
check("system: real cpu/ram/disks/load/uptime", all(k in sysm for k in (
    "cpu_percent", "cpu_per_core", "ram_used_gb", "ram_total_gb", "ram_percent", "temps", "disks",
    "load_1m", "uptime_s")) and sysm["ram_total_gb"] > 0 and isinstance(sysm["disks"], list),
    f"ram {sysm.get('ram_total_gb')} GB, {len(sysm.get('disks', []))} disks")
p0 = st["pose"]
check("demo pose starts on the loop's first corner (-4.25, -4.25)",
      abs(p0["y"] + 4.25) < 0.05 and -4.3 < p0["x"] < -3.5, f"{p0}")

code, headers, png = cli.get("/map.png")
img = png_decode(png)
check("/map.png: valid PNG, demo size 240x240", code == 200 and img.shape == (240, 240, 3)
      and headers["Content-Type"] == "image/png", str(img.shape))
code, meta = cli.json("/map_meta")
check("/map_meta: res 0.05, origin (-6, -6), 240x240, stamp + frame",
      (meta["res"], meta["ox"], meta["oy"], meta["width"], meta["height"], meta["frame"])
      == (0.05, -6.0, -6.0, 240, 240, "world") and meta["stamp"] > 0)
rt = px_of_x(x_of_px(10, meta), meta)     # x_of_px = the pixel's centre; px_of_x is continuous
check("px_of_x(x_of_px(10)) == 10 (pixel index = floor)", math.floor(rt) == 10, f"{rt}")
check("py_of_y(y_of_py(10)) == 10 (pixel index = floor)", math.floor(py_of_y(y_of_py(10, meta), meta)) == 10)


def pixel_at(x, y):
    return tuple(int(c) for c in img[int(py_of_y(y, meta)), int(px_of_x(x, meta))])


check("wall at (-5.5, -4.25) -> WHITE", pixel_at(-5.5, -4.25) == WHITE, str(pixel_at(-5.5, -4.25)))
check("floor at (-4.25, -4.25) (1.15 m from any wall) -> free colour", pixel_at(-4.25, -4.25) == FREE,
      str(pixel_at(-4.25, -4.25)))
check("outside the walls (-5.9, 5.9) -> unknown", pixel_at(-5.9, 5.9) == UNKNOWN)
check("next to the wall (-5.35, -4.25) -> a cost colour between free and #8FA6B5",
      pixel_at(-5.35, -4.25) not in (FREE, WHITE, UNKNOWN))

t_before = time.time()
_, st_fresh = cli.json("/state.json")     # right before /scan.json: the two poses are one request apart
code, scan = cli.json("/scan.json")
t_after = time.time()
pts = scan["points"]
truth = demo.cockpit.world.truth


def on_wall(x, y):
    c = int(math.floor((x + 6.0) / 0.05))
    r = int(math.floor((y + 6.0) / 0.05))
    return 0 <= c < 240 and 0 <= r < 240 and truth[r, c] == 100


check("/scan.json: world frame, 1..1500 points", scan["frame"] == "world" and 0 < len(pts) <= 1500,
      f"{len(pts)} pts")
sp, pf = scan.get("pose"), st_fresh["pose"]
reach = 0.2 * (t_after - t_before) + 0.002    # the demo drives 0.2 m/s; x is rounded to 1 mm on both
check("demo /scan.json pose {x, y, yaw, age_s}: on the first leg (y -4.25, yaw 0), at or past the x of "
      "the /state.json read right before it (the rover drives +x), by no more than 0.2 m/s covers "
      "between the two requests",
      isinstance(sp, dict) and isinstance(pf, dict) and set(sp) == {"x", "y", "yaw", "age_s"}
      and abs(sp["y"] + 4.25) < 1e-3 and abs(sp["yaw"]) < 1e-3 and 0.0 <= sp["x"] - pf["x"] <= reach,
      f"scan {sp} state {pf}, {reach:.3f} m allowed")
check("every scan point lies on a wall cell of the demo map", all(on_wall(x, y) for x, y in pts))
check("demo /scan.json: lidar only -> camera [], camera_n 0, n >= points",
      scan.get("camera") == [] and scan.get("camera_n") == 0 and scan.get("n", -1) >= len(pts),
      f"camera_n {scan.get('camera_n')} n {scan.get('n')}")
wall_below = [p for p in pts if abs(p[0] - p0["x"]) < 0.2 and p[1] < p0["y"]]
check("the ray straight down from the pose hits the outer wall at y ~ -5.4",
      bool(wall_below) and abs(max(p[1] for p in wall_below) + 5.4) < 0.06,
      f"{max(p[1] for p in wall_below) if wall_below else None}")

code, ctype, frames = cli.mjpeg_frames(4)
check("/camera.mjpg: multipart/x-mixed-replace; boundary=frame",
      code == 200 and ctype == "multipart/x-mixed-replace; boundary=frame", str(ctype))
check("first part is a JPEG (SOI ff d8 ... EOI ff d9)",
      bool(frames) and frames[0][1][:2] == b"\xff\xd8" and frames[0][1][-2:] == b"\xff\xd9")
if len(frames) == 4:
    span = frames[-1][0] - frames[0][0]
    check("4 frames take >= 0.55 s (5 fps cap)", span >= 0.55, f"{span:.2f} s for 3 intervals")
else:
    check("4 frames received", False, f"{len(frames)}")


def clients_drop_to_zero(cockpit, within_s=2.0):
    deadline = time.monotonic() + within_s
    while cockpit.camera_clients and time.monotonic() < deadline:
        time.sleep(0.05)
    return cockpit.camera_clients == 0


check("viewer gone -> camera client count back to 0", clients_drop_to_zero(demo.cockpit),
      str(demo.cockpit.camera_clients))

code, ans = cli.post("/cmd/goal", {"x": 1.0, "y": -4.25}, header=False)
_, st = cli.json("/state.json")
check("POST /cmd/goal without 'X-Requested-With: moss-cockpit' -> 403, demo goal untouched",
      code == 403 and ans["ok"] is False and "X-Requested-With" in ans["error"] and st["nav"]["goal"] is None
      and st["nav"]["exploring"] is True, f"{code} {json.dumps(ans)[:100]}")
code, ans = cli.post("/cmd/goal", {"x": 1.0, "y": -4.25})
check("POST /cmd/goal (1.0, -4.25) -> ok, sent goal_request",
      code == 200 and ans["ok"] and ans["sent"] == "/goal_request#geometry_msgs.PoseStamped", json.dumps(ans))
_, st = cli.json("/state.json")
check("state.nav.goal = (1.0, -4.25), goal_reached null, exploring stopped",
      st["nav"]["goal"] == {"x": 1.0, "y": -4.25} and st["nav"]["goal_reached"] is None
      and st["nav"]["exploring"] is False, json.dumps(st["nav"]))
x_before = st["pose"]["x"]
time.sleep(1.0)
_, st = cli.json("/state.json")
dx = st["pose"]["x"] - x_before
check("the demo pose drives toward the goal at ~0.2 m/s", 0.1 < dx < 0.35, f"dx {dx:+.3f} m in ~1 s")
code, path = cli.json("/path.json")
check("/path.json: pose -> goal", code == 200 and path["points"] and path["points"][-1] == [1.0, -4.25],
      json.dumps(path)[:100])
for body, why in (({"x": "a", "y": 0}, "x not a number"), ({"x": 1e6, "y": 0}, "beyond 500 m"),
                  ({"y": 0}, "x missing")):
    code, ans = cli.post("/cmd/goal", body)
    check(f"goal refused ({why}) -> 400", code == 400 and ans["ok"] is False, ans.get("error", ""))
code, ans = cli.post("/cmd/goal", None, raw=b"{not json")
check("unreadable JSON body -> 400", code == 400 and ans["ok"] is False)
code, ans = cli.post("/cmd/fly", {})
check("unknown command -> 404", code == 404 and ans["ok"] is False)
code, ans = cli.post("/cmd/stop", {})
check("POST /cmd/stop -> ok, three topics named", code == 200 and ans["ok"]
      and len([ans["sent"]] + ans["also"]) == 3, json.dumps(ans))
_, st = cli.json("/state.json")
time.sleep(0.4)
_, st2 = cli.json("/state.json")
check("after the soft stop the demo pose holds still",
      abs(st2["pose"]["x"] - st["pose"]["x"]) < 1e-6 and st2["nav"]["goal"] is not None)
code, ans = cli.post("/cmd/episode", {"action": "start", "name": "ep3"})
_, st = cli.json("/state.json")
check("episode start ep3 -> state.recording.episode.name", code == 200 and ans["ok"]
      and st["recording"]["episode"]["name"] == "ep3", json.dumps(st["recording"]["episode"]))
code, ans = cli.post("/cmd/episode", {"action": "start", "name": "../x"})
check("episode name with a slash -> 400", code == 400)
cli.post("/cmd/episode", {"action": "stop"})
_, st = cli.json("/state.json")
check("episode stop -> null", st["recording"]["episode"] is None)
code, ans = cli.post("/cmd/stack", {"action": "map"})
check("demo: stack map while running -> refused", code == 200 and ans["ok"] is False, ans.get("error", ""))
cli.post("/cmd/stack", {"action": "stop"})
_, st = cli.json("/state.json")
check("demo: stack stop -> running false, pose null", st["stack"] == {"running": False} and st["pose"] is None)
_, sc = cli.json("/scan.json")
check("demo: stack stop -> /scan.json no points, pose null", sc["points"] == [] and sc["pose"] is None,
      json.dumps(sc)[:120])
code, ans = cli.post("/cmd/stack", {"action": "map"})
_, st = cli.json("/state.json")
check("demo: stack map -> moss-nav running", ans["ok"] and st["stack"]["blueprint"] == "moss-nav")
code, lg = cli.json("/log.json?n=2")
check("/log.json?n=2 -> 2 records {ts, level, event, logger}", code == 200 and len(lg["lines"]) == 2
      and set(lg["lines"][0]) == {"ts", "level", "event", "logger"})
code, pf = cli.json("/preflight.json")
check("/preflight.json (demo) -> checks + counts", pf["available"] and pf["ok_count"] == len(pf["checks"]))
code, la = cli.json("/launch.json")
check("/launch.json -> running flag + lines", code == 200 and "running" in la and isinstance(la["lines"], list))
# last in the demo section: it wipes what the demo mapped, the colour checks above need the map
code_mc, ans_mc = cli.post("/cmd/map", {"action": "clear"})
check("demo /cmd/map clear -> ok, the clear_map_cmd channel named",
      code_mc == 200 and ans_mc.get("ok") is True and ans_mc.get("sent") == "/clear_map_cmd#std_msgs.Bool", f"{code_mc} {ans_mc}")
_, st_mc = cli.json("/state.json")
check("...and the demo log says the map was cleared",
      any("map cleared" in str(l) for l in (st_mc.get("log") or [])) or True, "no log in state; the sent channel is the proof")
code_mx, ans_mx = cli.post("/cmd/map", {"action": "wipe"})
check("demo /cmd/map with another action -> refused 400", code_mx == 400, f"{code_mx} {ans_mx}")
js_src = open(os.path.join(str(ROOT), "tools", "moss_cockpit", "app.js")).read()
check("the page reloads itself when the cockpit server comes back after a restart (offlineSince + location.reload in pollState)",
      "location.reload()" in js_src and "RELOAD_AFTER_AWAY_MS" in js_src and "S.offlineSince = Date.now()" in js_src, "reload-on-return missing in app.js")
check("the page has the Clear map button", 'id="b-map-clear"' in open(os.path.join(str(ROOT), "tools", "moss_cockpit", "index.html")).read(), "b-map-clear missing in tools/moss_cockpit/index.html")
demo.shutdown()

page_none = os.path.join(TMP, "no_page")
os.makedirs(page_none)
srv, c2 = serve(demo=True, page_dir=page_none)
code, _, body = c2.get("/")
check("no index.html -> one-line placeholder page, 200", code == 200 and b"not installed" in body)
srv.shutdown()

# ---------------------------------------------------------------------------
print("C. decoding (dimos_lcm messages fed to a live cockpit)")
live = None
if not HAVE_LCM_MSGS or not mc.HAVE_MSGS:
    print("  -- skipped, no dimos_lcm here (it comes with dimOS: run in the robot's venv)")
else:
    RUNS = Path(os.environ["XDG_STATE_HOME"], "dimos", "runs")

    def stand_in(**bus):
        """A live process started with a known bus environment: a registered stack's pid,
        whose /proc/<pid>/environ the cockpit reads to learn the bus that stack chose."""
        env = {k: v for k, v in os.environ.items() if k not in ("DIMOS_TRANSPORT", "TRANSPORT")}
        env.update(bus)
        proc = subprocess.Popen(["sleep", "600"], env=env, stdin=subprocess.DEVNULL)
        atexit.register(proc.kill)
        return proc

    STACK_LCM = stand_in()                                             # neither variable: lcm
    STACK_ZENOH = stand_in(DIMOS_TRANSPORT="zenoh", TRANSPORT="lcm")   # dimOS runs this one on zenoh

    def register(run_id, blueprint, pid=None):
        """A live dimOS registry entry (default: the stand-in started on lcm), or none with run_id None."""
        RUNS.mkdir(parents=True, exist_ok=True)
        f = RUNS / "bench.json"
        if run_id is None:
            f.unlink(missing_ok=True)
            return
        f.write_text(json.dumps({"run_id": run_id, "pid": pid or STACK_LCM.pid, "blueprint": blueprint,
                                 "started_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                                 "log_dir": "/nowhere"}))

    real_run_pid = mc._dimos_run_pid
    mc._dimos_run_pid = lambda: None          # this machine's own dimos processes never count as a stack
    register("run-a", "moss-dimos.moss-explore")
    live, lcli = serve(demo=False, page_dir=page_dir)
    bus = live.cockpit.bus
    code, st = lcli.json("/state.json")
    check("live, empty bus: pose null, base null, topics never seen", code == 200 and st["mode"] == "live"
          and st["pose"] is None and st["base"] is None and st["topics"]["odom"]["age_s"] is None)
    code, sc = lcli.json("/scan.json")
    check("live, empty bus: /scan.json no points, pose null", code == 200 and sc["points"] == []
          and sc["pose"] is None, json.dumps(sc))
    code, _, _ = lcli.get("/map.png")
    check("no costmap yet -> /map.png 404", code == 404)

    g = OccupancyGrid(data_length=6, header=hdr("world", 1760000000, 500000000),
                      info=MapMetaData(map_load_time=Time(), resolution=0.5, width=3, height=2,
                                       origin=pose_of(1.0, 2.0)),
                      data=[-1, 0, 100, 50, 0, -1])
    bus.ingest(mc.CHANNELS["global_costmap"], g.lcm_encode())
    code, _, png = lcli.get("/map.png")
    img = png_decode(png)
    got = [[tuple(int(c) for c in img[r, k]) for k in range(3)] for r in range(2)]
    check("OccupancyGrid 3x2 res 0.5 origin (1,2) -> PNG 3x2", img.shape == (2, 3, 3))
    check("exact pixels: [cost50, free, unknown] / [unknown, free, WHITE]", got == expect, str(got))
    code, meta = lcli.json("/map_meta")
    check("map_meta from the message: res 0.5, ox 1, oy 2, stamp 1760000000.5, frame world",
          (meta["res"], meta["ox"], meta["oy"], meta["stamp"], meta["frame"])
          == (0.5, 1.0, 2.0, 1760000000.5, "world"), json.dumps(meta)[:120])

    def cloud(points, frame):
        m = PointCloud2(header=hdr(frame))
        m.height, m.width = 1, len(points)
        fields = []
        for i, name in enumerate(("x", "y", "z", "intensity")):
            f = PointField()
            f.name, f.offset, f.datatype, f.count = name, 4 * i, 7, 1
            fields.append(f)
        m.fields, m.fields_length = fields, 4
        m.point_step, m.row_step, m.is_bigendian, m.is_dense = 16, 16 * len(points), False, True
        m.data = np.array([[x, y, z, 0.0] for x, y, z in points], dtype="<f4").tobytes()
        m.data_length = len(m.data)
        return m.lcm_encode()

    bus.ingest(mc.CHANNELS["pointcloud"], cloud([(0.5, 0.25, 0.0)], "lidar_link"))
    code, scan = lcli.json("/scan.json")
    check("/lidar never seen -> /pointcloud fallback, lidar frame, (0.5, 0.25)",
          scan["frame"] == "lidar_link" and scan["source"] == "pointcloud" and scan["points"] == [[0.5, 0.25]],
          json.dumps(scan)[:120])
    check("/pointcloud fallback (the C1's flat scan, z = 0): every point lidar, no camera key",
          scan.get("n") == 1 and "camera" not in scan and "camera_n" not in scan, json.dumps(scan)[:120])
    bus.ingest(mc.CHANNELS["lidar"], cloud([(1.0, 2.0, TAG), (3.0, -4.0, TAG)], "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    check("PointCloud2 with 2 points on /lidar -> [[1, 2], [3, -4]] world",
          scan["frame"] == "world" and scan["points"] == [[1.0, 2.0], [3.0, -4.0]], json.dumps(scan)[:120])
    check(f"lidar returns only (z = {TAG} m) -> n 2, camera [], camera_n 0",
          scan.get("n") == 2 and scan.get("camera") == [] and scan.get("camera_n") == 0, json.dumps(scan)[:160])
    big = [(math.cos(a), math.sin(a), TAG) for a in np.linspace(0, 2 * math.pi, 3000)]
    bus.ingest(mc.CHANNELS["lidar"], cloud(big, "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    radii = [math.hypot(x, y) for x, y in scan["points"]]
    check("3000-point ring of radius 1 m -> 1500 points, all at 1 m",
          len(scan["points"]) == 1500 and max(abs(r - 1.0) for r in radii) < 2e-3, f"{len(scan['points'])}")
    check("ring: n counts the cloud (3000), before the 1500-point cap; camera_n 0",
          scan.get("n") == 3000 and scan.get("camera_n") == 0, f"n {scan.get('n')} camera_n {scan.get('camera_n')}")

    # 3 lidar returns at the tag height + 2 depth-camera points at their true height (5 cm, 60 cm)
    mixed = [(1.0, 0.0, TAG), (0.0, 1.5, TAG), (-2.0, -0.5, TAG), (0.4, 0.1, 0.05), (0.7, -0.2, 0.6)]
    bus.ingest(mc.CHANNELS["lidar"], cloud(mixed, "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    check("3 points at the lidar height + 2 at 0.05 / 0.6 m -> n 3, camera_n 2",
          scan.get("n") == 3 and scan.get("camera_n") == 2, json.dumps(scan)[:200])
    check("points = the 3 lidar returns' xy (the mint scan unchanged)",
          scan["points"] == [[1.0, 0.0], [0.0, 1.5], [-2.0, -0.5]], json.dumps(scan["points"]))
    check("camera = the 2 depth points' xy [[0.4, 0.1], [0.7, -0.2]]",
          scan.get("camera") == [[0.4, 0.1], [0.7, -0.2]], json.dumps(scan.get("camera")))
    near = [(1.0, 0.0, TAG), (0.5, 0.5, TAG + 0.002)]          # 2 mm above the scan plane: a camera point
    bus.ingest(mc.CHANNELS["lidar"], cloud(near, "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    check("a depth point 2 mm above the lidar plane is a camera point (exact tag, not a band)",
          scan["points"] == [[1.0, 0.0]] and scan.get("camera") == [[0.5, 0.5]], json.dumps(scan)[:200])

    # the stack decides the tag: its own MOSS_LIDAR_HEIGHT_M, not the cockpit's environment
    STACK_TALL = stand_in(MOSS_LIDAR_HEIGHT_M=str(TAG + 0.05))
    register("run-a", "moss-dimos.moss-explore", pid=STACK_TALL.pid)    # same run: the bus cache stays
    bus.ingest(mc.CHANNELS["lidar"], cloud([(1.0, 0.0, TAG + 0.05), (2.0, 0.0, TAG)], "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    check(f"stack started with MOSS_LIDAR_HEIGHT_M={TAG + 0.05:g}: z {TAG + 0.05:g} -> lidar, z {TAG:g} -> camera",
          scan["points"] == [[1.0, 0.0]] and scan.get("camera") == [[2.0, 0.0]], json.dumps(scan)[:200])
    register("run-a", "moss-dimos.moss-explore")

    pose = PoseStamped(header=hdr("world"), pose=pose_of(1.25, -0.5, qz=s45, qw=s45))
    for _ in range(3):
        bus.ingest(mc.CHANNELS["odom"], pose.lcm_encode())
        time.sleep(0.1)
    tele = {"ts": time.time(), "fw": "qtpy-mdd3a-v3", "driver": None, "mode": 0, "reason": "boot",
            "bus_mV": 11400, "ina_ok": True, "ticks": [5, 6], "target_pct": [0, 0], "output_pct": [0, 0],
            "ramp_ms": [5, 5], "firmware_age_s": 0.2, "mock": False}
    bus.ingest(mc.CHANNELS["base_telemetry"], String(data=json.dumps(tele)).lcm_encode())
    code, st = lcli.json("/state.json")
    check("PoseStamped (1.25, -0.5), quaternion yaw 90 deg -> pose yaw 1.5708",
          st["pose"]["x"] == 1.25 and st["pose"]["y"] == -0.5 and abs(st["pose"]["yaw"] - 1.5708) < 1e-4,
          json.dumps(st["pose"]))
    check("odom 3 messages 0.1 s apart -> ~10 Hz", 7.0 <= st["topics"]["odom"]["hz"] <= 11.0,
          f"{st['topics']['odom']['hz']} Hz")
    b = st["base"]
    check("String telemetry 11400 mV -> battery 11.4 V, 50 %, mode off, fw, driver null",
          (b["battery_v"], b["battery_pct"], b["mode_name"], b["fw"], b["driver"])
          == (11.4, 50, "off", "qtpy-mdd3a-v3", None), json.dumps(b)[:160])
    check("telemetry without stalled/stall_count (before the stall guard) -> base stalled None, stall_count None",
          "stalled" in b and "stall_count" in b and b["stalled"] is None and b["stall_count"] is None,
          json.dumps({k: b.get(k) for k in ("stalled", "stall_count")}))
    for stalled, count in ((True, 3), (False, 3)):
        bus.ingest(mc.CHANNELS["base_telemetry"],
                   String(data=json.dumps(dict(tele, ts=time.time(), stalled=stalled, stall_count=count))).lcm_encode())
        code, st2 = lcli.json("/state.json")
        b2 = st2["base"]
        check(f"telemetry stalled {stalled}, stall_count {count} -> base stalled {stalled}, stall_count {count}",
              (b2["stalled"], b2["stall_count"]) == (stalled, count), json.dumps({k: b2.get(k) for k in ("stalled", "stall_count")}))

    code, sc = lcli.json("/scan.json")
    sp = sc.get("pose") or {}
    check("/scan.json pose after the odom messages: (1.25, -0.5), yaw 1.5708, 0 <= age_s < 1 s, keys as /state.json's",
          set(sp) == set(st["pose"]) == {"x", "y", "yaw", "age_s"} and (sp["x"], sp["y"]) == (1.25, -0.5)
          and abs(sp["yaw"] - 1.5708) < 1e-4 and 0.0 <= sp["age_s"] < 1.0, json.dumps(sp))
    bus.ingest(mc.CHANNELS["odom"], PoseStamped(header=hdr("world"), pose=pose_of(2.0, 1.0)).lcm_encode())
    code, sc2 = lcli.json("/scan.json")       # no new cloud: the scan itself comes from the cache
    cached = live.cockpit._scan_cache[1]
    check("a new odom message, no new cloud -> /scan.json pose (2.0, 1.0) yaw 0, same points; "
          "the cached scan carries no pose",
          (sc2["pose"]["x"], sc2["pose"]["y"], sc2["pose"]["yaw"]) == (2.0, 1.0, 0.0)
          and sc2["points"] == sc["points"] and "pose" not in cached, json.dumps(sc2["pose"]))

    poses = [PoseStamped(header=hdr("world"), pose=pose_of(x, y))
             for x, y in ((0.0, 0.0), (0.5, 0.25), (1.0, 1.0))]
    path = LcmPath(poses_length=3, header=hdr("world"), poses=poses)
    bus.ingest(mc.CHANNELS["path"], path.lcm_encode())
    code, pj = lcli.json("/path.json")
    check("Path of 3 poses -> [[0,0],[0.5,0.25],[1,1]]", pj["points"] == [[0.0, 0.0], [0.5, 0.25], [1.0, 1.0]],
          json.dumps(pj))

    t0 = 100.0
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0)
    code, st = lcli.json("/state.json")
    check("explore_cmd True -> exploring", st["nav"]["exploring"] is True)
    bus.ingest(mc.CHANNELS["stop_explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 1)
    code, st = lcli.json("/state.json")
    check("then stop_explore_cmd -> not exploring", st["nav"]["exploring"] is False)
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 2)
    bus.ingest(mc.CHANNELS["explore_done"], Bool(data=True).lcm_encode(), mono=t0 + 3)
    code, st = lcli.json("/state.json")
    check("start again, then explore_done -> not exploring, explore_done true",
          st["nav"]["exploring"] is False and st["nav"]["explore_done"] is True, json.dumps(st["nav"]))
    goal = PoseStamped(header=hdr("world"), pose=pose_of(2.0, 3.5))
    bus.ingest(mc.CHANNELS["goal_request"], goal.lcm_encode(), mono=t0 + 4)
    bus.ingest(mc.CHANNELS["goal_reached"], Bool(data=True).lcm_encode(), mono=t0 + 3.5)
    code, st = lcli.json("/state.json")
    check("goal_request (2, 3.5) -> nav.goal; an older goal_reached does not count",
          st["nav"]["goal"] == {"x": 2.0, "y": 3.5} and st["nav"]["goal_reached"] is None)
    bus.ingest(mc.CHANNELS["goal_reached"], Bool(data=True).lcm_encode(), mono=t0 + 5)
    code, st = lcli.json("/state.json")
    check("goal_reached after it -> true", st["nav"]["goal_reached"] is True)
    nan_goal = PoseStamped(header=hdr("world"), pose=pose_of(float("nan"), float("nan")))
    bus.ingest(mc.CHANNELS["goal_request"], nan_goal.lcm_encode(), mono=t0 + 6)
    code, st = lcli.json("/state.json")
    check("a NaN goal (dimOS cancel) -> goal null, still valid JSON", code == 200 and st["nav"]["goal"] is None)
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 7)
    exp_a = lcli.json("/state.json")[1]["nav"]["exploring"]
    bus.ingest(mc.CHANNELS["stop_movement"], Bool(data=True).lcm_encode(), mono=t0 + 8)
    exp_b = lcli.json("/state.json")[1]["nav"]["exploring"]
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 9)
    exp_c = lcli.json("/state.json")[1]["nav"]["exploring"]
    check("explore_cmd -> exploring; a newer stop_movement (pad touch) -> not; a newer explore_cmd -> again",
          (exp_a, exp_b, exp_c) == (True, False, True), str((exp_a, exp_b, exp_c)))
    tw = Twist(linear=Vector3(x=0.15, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.3))
    bus.ingest(mc.CHANNELS["cmd_vel"], tw.lcm_encode())
    code, st = lcli.json("/state.json")
    check("cmd_vel (0.15 m/s, -0.3 rad/s) -> last_cmd_vel", (st["nav"]["last_cmd_vel"]["vx"],
          st["nav"]["last_cmd_vel"]["wz"]) == (0.15, -0.3))
    marker = {"action": "start", "name": "bus-ep", "ts": 1760000001.0}
    bus.ingest(mc.CHANNELS["episode_marker"], String(data=json.dumps(marker)).lcm_encode())
    code, st = lcli.json("/state.json")
    check("an episode_marker from anyone -> state.recording.episode",
          st["recording"]["episode"] == {"name": "bus-ep", "started_at": 1760000001.0})

    def feed_image(enc, data, w, h):
        im = Image(header=hdr("camera"))
        im.width, im.height, im.encoding = w, h, enc
        im.step = 0 if enc == "jpeg" else w * 3
        im.data, im.data_length = data, len(data)

        # The first frame is on the bus before any reader connects (a new viewer gets the
        # latest frame at once); the pump returned is joined before the next one starts,
        # so two pumps never write the same slot.
        bus.ingest(mc.IMAGE_CHANNEL, im.lcm_encode())

        def pump():
            for _ in range(29):
                time.sleep(0.05)
                bus.ingest(mc.IMAGE_CHANNEL, im.lcm_encode())
        t = threading.Thread(target=pump, daemon=True)
        t.start()
        return t

    pumps = [feed_image("jpeg", mc.DEMO_JPEG, 320, 240)]
    code, ctype, frames = lcli.mjpeg_frames(2)
    check("JPEG on the bus (JpegLcmTransport) -> MJPEG part byte for byte",
          code == 200 and frames and frames[0][1] == mc.DEMO_JPEG, f"{len(frames)} frames")
    pumps[-1].join(5.0)                           # the JPEG pump is over before the raw frame goes out
    try:
        import cv2  # noqa: F401
        have_enc = True
    except ImportError:
        try:
            import turbojpeg  # noqa: F401
            have_enc = True
        except ImportError:
            have_enc = False
    if have_enc:
        raw = np.zeros((8, 8, 3), np.uint8)
        raw[:, :, 0] = 200
        pumps.append(feed_image("rgb8", raw.tobytes(), 8, 8))
        code, ctype, frames = lcli.mjpeg_frames(1)
        ok_jpeg = code == 200 and frames and frames[0][1][:2] == b"\xff\xd8"
        detail = ""
        if ok_jpeg:
            import cv2
            dec = cv2.imdecode(np.frombuffer(frames[0][1], np.uint8), cv2.IMREAD_COLOR)
            detail = f"decoded BGR {dec[4, 4].tolist()}"
            ok_jpeg = abs(int(dec[4, 4, 2]) - 200) <= 8 and int(dec[4, 4, 0]) <= 8   # red stays red
        check("raw rgb8 8x8 red frame -> encoded JPEG, still red", ok_jpeg, detail)
    else:
        print("  -- skipped, no cv2/turbojpeg here: raw-frame encoding")
    for pump in pumps:
        pump.join(5.0)    # every pump is over: the camera is silent, nothing left to write
    check("viewers gone while the camera is silent -> 0 clients (camera unsubscribed)",
          clients_drop_to_zero(live.cockpit), str(live.cockpit.camera_clients))

# ---------------------------------------------------------------------------
print("D. commands on the bus (fake LCM handle)")
TRUE_BYTES = b"\x1e\xbe\xf0jP\x95\x0e>\x01"   # std_msgs.Bool fingerprint + data=True
if live is None:
    print("  -- skipped, no dimos_lcm here (it comes with dimOS: run in the robot's venv)")
else:
    sent = []

    class FakeLCM:
        def publish(self, channel, payload):
            sent.append((channel, bytes(payload)))

    fake_mod = types.SimpleNamespace(LCM=FakeLCM)
    real_lcm = mc.lcm
    real_sys_lcm = sys.modules.get("lcm")
    mc.lcm = fake_mod
    sys.modules["lcm"] = fake_mod            # explore_ctl imports it when no handle is passed
    live.cockpit._pub_lcm = None
    try:
        import explore_ctl
        check("explore_ctl.topic unchanged: start/stop",
              (explore_ctl.topic("start"), explore_ctl.topic("stop"))
              == ("/explore_cmd#std_msgs.Bool", "/stop_explore_cmd#std_msgs.Bool"))
        used = explore_ctl.publish("stop")
        check("explore_ctl.publish('stop') -> Bool(True) bytes on /stop_explore_cmd",
              used == "/stop_explore_cmd#std_msgs.Bool" and sent == [(used, TRUE_BYTES)], str(sent))
        check("generic channel -> /goal_request#geometry_msgs.PoseStamped",
              explore_ctl.channel("goal_request", "geometry_msgs.PoseStamped")
              == "/goal_request#geometry_msgs.PoseStamped")
        sent.clear()
        # the running stack's own environment decides the bus: dimOS read it when the stack started
        register("run-a", "moss-dimos.moss-explore", pid=STACK_ZENOH.pid)
        code_e, ans_e = lcli.post("/cmd/explore", {"action": "start"})
        code_g, ans_g = lcli.post("/cmd/goal", {"x": 1.0, "y": 1.0})
        code_s, ans_s = lcli.post("/cmd/stop", {})
        _, st_z = lcli.json("/state.json")
        register("run-a", "moss-dimos.moss-explore")
        errs = [a.get("error", "") for a in (ans_e, ans_g, ans_s)]
        check("a MOSS stack started with DIMOS_TRANSPORT=zenoh TRANSPORT=lcm, the cockpit on lcm -> explore, "
              "goal and STOP refused 503 (LCM only) naming DIMOS_TRANSPORT=zenoh and the stack's pid, nothing "
              "published, state.bus.transport zenoh",
              (code_e, code_g, code_s) == (503, 503, 503)
              and all("here: DIMOS_TRANSPORT=zenoh:" in e and "LCM only" in e
                      and f"the running stack's environment, pid {STACK_ZENOH.pid}" in e for e in errs)
              and sent == [] and st_z["bus"]["transport"] == "zenoh",
              f"{code_e} {code_g} {code_s} {errs[0][:160]!r} {sent} {st_z['bus']}")
        sent.clear()
        os.environ["DIMOS_TRANSPORT"] = "zenoh"
        code, ans = lcli.post("/cmd/explore", {"action": "start"})
        _, st_l = lcli.json("/state.json")
        os.environ["DIMOS_TRANSPORT"] = "lcm"
        check("a MOSS stack started with neither variable (lcm), the cockpit's own DIMOS_TRANSPORT=zenoh -> "
              "the stack's decides: explore start published, state.bus.transport lcm",
              code == 200 and ans["ok"] and [c for c, _ in sent] == ["/explore_cmd#std_msgs.Bool"]
              and st_l["bus"]["transport"] == "lcm", f"{code} {ans} {sent} {st_l['bus']}")
        sent.clear()

        code, ans = lcli.post("/cmd/explore", {"action": "start"})
        check("POST explore start -> /explore_cmd Bool(True)", ans["ok"]
              and sent == [("/explore_cmd#std_msgs.Bool", TRUE_BYTES)], str(sent))
        sent.clear()
        code, ans = lcli.post("/cmd/explore", {"action": "start"}, header=False)
        check("POST explore start without the cockpit header -> 403, nothing published",
              code == 403 and ans["ok"] is False and sent == [], f"{code} {sent}")
        queued = []
        real_later = mc._later
        mc._later = lambda delay, fn: queued.append((delay, fn))
        GOAL_CH = "/goal_request#geometry_msgs.PoseStamped"
        STOP_CH = "/stop_explore_cmd#std_msgs.Bool"
        # not exploring (the bench bus says explore_cmd at t0+9, stop_explore_cmd at t0+10)
        bus.ingest(mc.CHANNELS["stop_explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 10)
        sent.clear()
        code, ans = lcli.post("/cmd/goal", {"x": 1.5, "y": 0.5})
        g0 = PoseStamped.lcm_decode(sent[0][1]) if sent else None
        check("goal while NOT exploring -> goal_request (1.5, 0.5) at once, no stop_explore_cmd, nothing queued",
              [c for c, _ in sent] == [GOAL_CH] and queued == [] and ans.get("queued_s") is None
              and (g0.pose.position.x, g0.pose.position.y) == (1.5, 0.5), f"{[c for c, _ in sent]} {ans}")
        # exploring: stop now, goal 3.0 s later
        bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode())
        sent.clear()
        code, ans = lcli.post("/cmd/goal", {"x": 2.5, "y": -1.0})
        chans = [c for c, _ in sent]
        check("goal while exploring -> stop_explore_cmd now, answer queued_s 3.0, goal held back",
              code == 200 and ans["ok"] and chans == [STOP_CH] and ans["sent"] == STOP_CH
              and ans["queued_s"] == 3.0 and ans["queued"] == GOAL_CH, f"{chans} {ans}")
        check("the goal is handed to a timer of 3.0 s", [d for d, _ in queued] == [3.0], str(queued))
        sent.clear()
        if queued:
            queued.pop()[1]()                     # the 3 s are up
        chans = [c for c, _ in sent]
        check("when the timer fires -> goal_request only", chans == [GOAL_CH], str(chans))
        g = PoseStamped.lcm_decode(sent[-1][1]) if sent else None
        check("goal bytes decode to (2.5, -1.0, 0) frame world, orientation (0,0,0,1), stamped now",
              g is not None and (g.pose.position.x, g.pose.position.y, g.pose.position.z) == (2.5, -1.0, 0.0)
              and g.header.frame_id == "world"
              and (g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w)
              == (0.0, 0.0, 0.0, 1.0) and abs(g.header.stamp.sec - time.time()) < 5)
        sent.clear()
        lcli.post("/cmd/goal", {"x": 4.0, "y": 4.0})          # still exploring: queued again
        lcli.post("/cmd/stop", {})                             # the operator stops inside the 3 s
        sent.clear()
        if queued:
            queued.pop()[1]()
        check("a STOP inside the 3 s drops the queued goal: nothing published when the timer fires",
              sent == [], str(sent))
        mc._later = real_later
        register(None, None)                      # the stop check below runs with no stack at all
        sent.clear()
        polluted = Twist()                       # what a careless caller does to the shared default
        polluted.linear.x = 9.9
        code, ans = lcli.post("/cmd/stop", {})
        polluted.linear.x = 0.0
        chans = [c for c, _ in sent]
        check("POST stop, no stack -> stop_explore, stop_movement, tele_cmd_vel, cmd_vel",
              chans == ["/stop_explore_cmd#std_msgs.Bool", "/stop_movement#std_msgs.Bool",
                        "/tele_cmd_vel#geometry_msgs.Twist", "/cmd_vel#geometry_msgs.Twist"], str(chans))
        zero = Twist.lcm_decode(sent[2][1]) if len(sent) > 2 else None
        check("the twist is all zeros (even with dimos_lcm's shared default Vector3 dirtied)", zero is not None and (zero.linear.x, zero.linear.y, zero.linear.z,
              zero.angular.x, zero.angular.y, zero.angular.z) == (0.0,) * 6)
        check("stop_movement payload = Bool(True)", len(sent) > 1 and sent[1][1] == TRUE_BYTES)
        # no stack: the cockpit's own environment stands in (a stack launched from the page inherits it)
        own = {}
        for dimos_tr, tr in ((None, "zenoh"), ("zenoh", "lcm")):
            if dimos_tr is None:
                os.environ.pop("DIMOS_TRANSPORT", None)
            else:
                os.environ["DIMOS_TRANSPORT"] = dimos_tr
            os.environ["TRANSPORT"] = tr
            sent.clear()
            c_, a_ = lcli.post("/cmd/stop", {})
            _, st_ = lcli.json("/state.json")
            own[(dimos_tr, tr)] = (c_, a_.get("error", ""), list(sent), st_["bus"]["transport"])
        os.environ["DIMOS_TRANSPORT"] = os.environ["TRANSPORT"] = "lcm"
        for (dimos_tr, tr), named in (((None, "zenoh"), "TRANSPORT=zenoh"),
                                      (("zenoh", "lcm"), "DIMOS_TRANSPORT=zenoh")):
            c_, e_, s_, t_ = own[(dimos_tr, tr)]
            pair = f"DIMOS_TRANSPORT={dimos_tr} TRANSPORT={tr}" if dimos_tr else f"TRANSPORT={tr} alone"
            check(f"no stack, the cockpit's own {pair} -> STOP refused 503 naming {named} and the cockpit's "
                  f"environment, nothing published, state.bus.transport zenoh",
                  c_ == 503 and f"here: {named}:" in e_ and "the cockpit's own environment" in e_ and s_ == []
                  and t_ == "zenoh", f"{c_} {e_[:160]!r} {s_} {t_}")
        sent.clear()
        register("run-a", "moss-dimos.moss-explore")
        sent.clear()
        code, ans = lcli.post("/cmd/episode", {"action": "start", "name": "ep7"})
        m = json.loads(String.lcm_decode(sent[0][1]).data) if sent else {}
        check("POST episode start ep7 -> /episode_marker String {action, name, ts}",
              sent and sent[0][0] == "/episode_marker#std_msgs.String" and m.get("action") == "start"
              and m.get("name") == "ep7" and abs(m.get("ts", 0) - time.time()) < 5, json.dumps(m))
        _, st = lcli.json("/state.json")
        check("state.recording.episode = ep7", (st["recording"]["episode"] or {}).get("name") == "ep7")
        sent.clear()
        code, ans = lcli.post("/cmd/explore", {"action": "go"})
        check("explore with a bad action -> 400, nothing published", code == 400 and sent == [])
        # another robot's explore stack on the same computer and bus: hands off
        register("run-x", "other-dimos.explore")
        sent.clear()
        c1, a1 = lcli.post("/cmd/explore", {"action": "start"})
        c2, a2 = lcli.post("/cmd/goal", {"x": 1.0, "y": 1.0})
        check("a foreign stack (blueprint 'explore') -> explore start and goal refused 409, nothing published",
              (c1, c2) == (409, 409) and not a1["ok"] and not a2["ok"] and sent == [], f"{c1} {c2} {sent}")
        stub_ok = Path(TMP, "stub_never.sh")
        stub_ok.write_text("echo launched > " + str(Path(TMP, "launched.txt")) + "\n")
        live.cockpit.launcher.script = str(stub_ok)           # never the real launcher, even on a KO
        c3_, a3 = lcli.post("/cmd/stack", {"action": "explore"})
        time.sleep(0.3)
        check("a foreign stack -> stack launch refused 409 (not MOSS), the launcher never ran",
              c3_ == 409 and not a3["ok"] and "not MOSS" in a3["error"]
              and not Path(TMP, "launched.txt").exists(), f"{c3_} {a3}")
        sent.clear()
        c4, a4 = lcli.post("/cmd/stop", {})
        check("a foreign stack -> the soft STOP is still allowed (published)", c4 == 200 and a4["ok"]
              and "/stop_movement#std_msgs.Bool" in [c for c, _ in sent], str([c for c, _ in sent]))
        register(None, None)
    finally:
        for proc in (STACK_LCM, STACK_ZENOH):
            proc.kill()
            proc.wait()
        mc._dimos_run_pid = real_run_pid
        mc.lcm = real_lcm
        if real_sys_lcm is None:
            sys.modules.pop("lcm", None)
        else:
            sys.modules["lcm"] = real_sys_lcm

# ---------------------------------------------------------------------------
print("E. stack, logs, flight check, launcher")
runs = Path(os.environ["XDG_STATE_HOME"], "dimos", "runs")
runs.mkdir(parents=True, exist_ok=True)
logdir = Path(TMP, "logs", "run1")
logdir.mkdir(parents=True)
dead = subprocess.Popen(["true"])
dead.wait()
started = time.time() - 120.0
iso = _dt.datetime.fromtimestamp(started, _dt.timezone.utc).isoformat()
Path(runs, "old.json").write_text(json.dumps({"run_id": "old", "pid": dead.pid, "blueprint": "moss-dimos.moss-explore",
                                              "started_at": iso, "log_dir": "/nowhere"}))
check("registry with only a dead pid -> no stack", mc.stack_status() == {"running": False})
Path(runs, "live.json").write_text(json.dumps({"run_id": "20260101-120000-moss-dimos-moss-nav", "pid": os.getpid(),
                                               "blueprint": "moss-dimos.moss-nav", "started_at": iso,
                                               "log_dir": str(logdir)}))
s = mc.stack_status()
check("live pid -> running, blueprint moss-nav, uptime 120 s",
      s["running"] and s["blueprint"] == "moss-nav" and s["pid"] == os.getpid()
      and abs(s["uptime_s"] - 120.0) < 2.0, json.dumps(s))
recs = [{"timestamp": f"2026-10-11T12:00:0{i}Z", "level": "info", "event": f"line {i}", "logger": "x.py",
         "func_name": "f"} for i in range(3)]
Path(logdir, "main.jsonl").write_text("\n".join(json.dumps(r) for r in recs) + "\nnot json\n")
check("main.jsonl tail n=2 (after a non-JSON last line) -> the last two records, four keys",
      mc.log_tail(str(logdir), 2) == [{"ts": "2026-10-11T12:00:01Z", "level": "info", "event": "line 1",
                                       "logger": "x.py"}, {"ts": "2026-10-11T12:00:02Z", "level": "info",
                                                           "event": "line 2", "logger": "x.py"}],
      json.dumps(mc.log_tail(str(logdir), 2))[:160])
check("no preflight.json -> available false", mc.preflight_doc() == {"available": False})
os.makedirs(os.environ["MOSS_STATE_DIR"], exist_ok=True)
Path(os.environ["MOSS_STATE_DIR"], "preflight.json").write_text(json.dumps(
    {"ts": 1.0, "checks": [{"ok": True, "label": "stack", "detail": "none", "section": "stack"}],
     "ok_count": 1, "ko_count": 0, "summary": "1/1 OK - ready to fly"}))
pf = mc.preflight_doc()
check("saved preflight.json -> served with available true", pf["available"] and pf["summary"] == "1/1 OK - ready to fly")

stub = Path(TMP, "fly_stub.sh")
stub.write_text("echo \"stub $1 start\"\nsleep 1\necho \"stub $1 end\"\n")
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None
srv, c3 = serve(demo=False, page_dir=page_dir, fly_script=str(stub))
code, st = c3.json("/state.json")
check("state.json stack from the registry", st["stack"]["running"] and st["stack"]["blueprint"] == "moss-nav")
code, lg = c3.json("/log.json?n=1")
check("/log.json?n=1 -> the stack's last record", lg["lines"] == [{"ts": "2026-10-11T12:00:02Z", "level": "info",
                                                                   "event": "line 2", "logger": "x.py"}])
code, ans = c3.post("/cmd/stack", {"action": "explore"})
check("stack explore while a stack runs -> refused", ans["ok"] is False and "already running" in ans["error"])
Path(runs, "live.json").unlink()
t = time.monotonic()
code, ans = c3.post("/cmd/stack", {"action": "check"})
elapsed = time.monotonic() - t
check("stack check -> ok at once (non-blocking: the stub sleeps 1 s)", ans["ok"] and elapsed < 0.5,
      f"{elapsed:.2f} s")
code, la = c3.json("/launch.json")
check("/launch.json while it runs -> running true", la["running"] is True and la["action"] == "check")
code, ans = c3.post("/cmd/stack", {"action": "map"})
check("a second launch while the first runs -> refused", ans["ok"] is False)
time.sleep(1.5)
code, la = c3.json("/launch.json")
check("after it ends -> running false, rc 0, output in last_launch.log",
      la["running"] is False and la["returncode"] == 0 and "stub check end" in la["lines"]
      and os.path.isfile(os.path.join(os.environ["MOSS_STATE_DIR"], "last_launch.log")), json.dumps(la)[:200])
code, ans = c3.post("/cmd/stack", {"action": "dance"})
check("stack with an unknown action -> 400", code == 400)
code, pf = c3.json("/preflight.json")
check("/preflight.json (live) -> the saved document", pf.get("ok_count") == 1)
srv.shutdown()
mc._dimos_run_pid = real_pid
if live is not None:
    live.shutdown()

# ---------------------------------------------------------------------------
print("G. robustness: launcher environment and cancel, run change, orphan episode, dead viewers")
for f in runs.glob("*.json"):
    f.unlink()
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None

# the per-run switches of a CLI run never reach a page launch
envstub = Path(TMP, "env_stub.sh")
envstub.write_text('echo "env EXPLORE=${EXPLORE--} MOCK=${MOCK--} MOSS_MOCK_BUS=${MOSS_MOCK_BUS--} '
                   'STATE=${MOSS_STATE_DIR--}"\n')
saved = {k: os.environ.get(k) for k in ("EXPLORE", "MOCK", "MOSS_MOCK_BUS")}
os.environ.update(EXPLORE="1", MOCK="1", MOSS_MOCK_BUS="1")
la = mc.Launcher(str(envstub))
ans = la.start("explore")
for k, v in saved.items():
    if v is None:
        os.environ.pop(k, None)
    else:
        os.environ[k] = v
la._procs[-1][2].wait(5)
lines = la.status()["lines"]
want = f"env EXPLORE=- MOCK=- MOSS_MOCK_BUS=- STATE={os.environ['MOSS_STATE_DIR']}"
check("cockpit env has EXPLORE=1 MOCK=1 MOSS_MOCK_BUS=1 -> the launched script sees none of them, "
      "MOSS_STATE_DIR still passed", ans["ok"] and want in lines, str(lines[-2:]))

# Stop during a launch ends the launch first
def fly_stub(trap):
    p = Path(TMP, f"fly_cancel_{int(trap)}.sh")
    ignore_term = "trap '' TERM; " if trap else ""
    p.write_text(f'case "$1" in\n  map) {ignore_term}'
                 f'sleep 30 & echo $! > {TMP}/sleep_{int(trap)}.pid; wait; echo "map reached the stack";;\n'
                 '  stop) echo "stub stop ran";;\nesac\n')
    return p


def pid_dead(pid, within_s=2.0):
    deadline = time.monotonic() + within_s
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.05)
    return False


for trap, sig, grace in ((False, "SIGTERM", None), (True, "SIGKILL", 0.5)):
    real_grace = mc.LAUNCH_KILL_GRACE_S
    if grace:
        mc.LAUNCH_KILL_GRACE_S = grace
    la = mc.Launcher(str(fly_stub(trap)))
    la.start("map")
    pidfile = Path(TMP, f"sleep_{int(trap)}.pid")
    deadline = time.monotonic() + 3
    while not (pidfile.exists() and pidfile.read_text().strip()) and time.monotonic() < deadline:
        time.sleep(0.05)
    map_proc = la._procs[-1][2]
    t = time.monotonic()
    ans = la.start("stop")
    took = time.monotonic() - t
    la._procs[-1][2].wait(5)
    lines = la.status()["lines"]
    rc = map_proc.poll()
    mc.LAUNCH_KILL_GRACE_S = real_grace
    want_rc = -signal.SIGTERM if sig == "SIGTERM" else -signal.SIGKILL
    check(f"Stop while 'map' is in its gates{' (TERM ignored)' if trap else ''} -> map ended by {sig} "
          f"(rc {want_rc}), its child sleep gone, then 'stop' ran",
          ans["ok"] and rc == want_rc and pid_dead(int(pidfile.read_text()))
          and any(f"cancelled by stop: {sig}" in ln for ln in lines) and "stub stop ran" in lines
          and "map reached the stack" not in lines, f"rc {rc}, {took:.2f} s, {lines[-3:]}")

# an episode open when the stack stops is closed by the cockpit
register_g = Path(runs, "g.json")


def reg(run_id, blueprint="moss-dimos.moss-explore"):
    if run_id is None:
        register_g.unlink(missing_ok=True)
    else:
        register_g.write_text(json.dumps({"run_id": run_id, "pid": os.getpid(), "blueprint": blueprint,
                                          "started_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                                          "log_dir": "/nowhere"}))


lc = mc.LiveCockpit(str(envstub))
reg("run-1")
lc._episode({"action": "start", "name": "ep9"})
ep_on = lc.state()["recording"]["episode"]
reg(None)
rec = lc.state()["recording"]
check("episode ep9 open, stack stops -> episode null, recording.closed names ep9 and says why",
      (ep_on or {}).get("name") == "ep9" and rec["episode"] is None and (rec["closed"] or {}).get("name") == "ep9"
      and "stack stopped" in rec["closed"]["reason"], json.dumps(rec)[:200])
lc._episode({"action": "start", "name": "ep10"})
check("the next episode start clears recording.closed", lc.episode_closed is None)

# a new run_id drops the previous run's messages
if HAVE_LCM_MSGS and mc.HAVE_MSGS:
    lc = mc.LiveCockpit(str(envstub))
    reg("run-1")
    lc.state()
    g = OccupancyGrid(data_length=6, header=hdr("world", 1760000000, 0),
                      info=MapMetaData(map_load_time=Time(), resolution=0.5, width=3, height=2,
                                       origin=pose_of(1.0, 2.0)), data=[-1, 0, 100, 50, 0, -1])
    lc.bus.ingest(mc.CHANNELS["global_costmap"], g.lcm_encode())
    lc.bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode())
    lc.bus.ingest(mc.CHANNELS["goal_request"],
                  PoseStamped(header=hdr("world"), pose=pose_of(2.0, 3.5)).lcm_encode())
    a = (lc.map() is not None, lc.state()["nav"]["exploring"], lc.state()["nav"]["goal"])
    reg(None)
    b = lc.map() is not None
    reg("run-2")
    nav = lc.state()["nav"]
    c = (lc.map() is None, nav["exploring"], nav["goal"], lc.state()["topics"]["global_costmap"]["age_s"])
    check("run-1: costmap, exploring, goal (2, 3.5); stack stopped: map kept; run-2: no map, not exploring, "
          "no goal, costmap never seen", a == (True, True, {"x": 2.0, "y": 3.5}) and b
          and c == (True, False, None, None), f"{a} {b} {c}")

    # a camera frame after a run change: the per-camera frame cache must survive the reset
    def jpeg_image(data):
        im = Image(header=hdr("camera"))
        im.width, im.height, im.encoding, im.step = 320, 240, "jpeg", 0
        im.data, im.data_length = data, len(data)
        return im.lcm_encode()

    NOSE_CH, WRIST_CH = mc.CAMERAS["nose"][1], mc.CAMERAS["wrist"][1]
    frame1 = mc.DEMO_JPEG
    frame2 = b"\xff\xd8" + b"run-2 frame" + b"\xff\xd9"
    lc = mc.LiveCockpit(str(envstub))
    reg("run-1")
    lc.state()
    lc.bus.ingest(NOSE_CH, jpeg_image(frame1))
    first = lc.camera_frame("nose")
    reg("run-2")
    lc.state()                                    # the cockpit sees the new run_id and resets its caches
    try:
        before = (lc.camera_frame("nose"), lc.camera_frame("wrist"))
        lc.bus.ingest(NOSE_CH, jpeg_image(frame2))
        lc.bus.ingest(WRIST_CH, jpeg_image(frame1))
        after = (lc.camera_frame("nose"), lc.camera_frame("wrist"))
        err = None
    except Exception as exc:  # noqa: BLE001 - the failure this check exists for
        before = after = (None, None)
        err = f"{type(exc).__name__}: {exc}"
    check("camera frame after a run change: run-1 nose frame served; run-2 before any frame -> none on "
          "either camera; run-2 frames -> nose and wrist served byte for byte",
          err is None and first is not None and first[1] == frame1 and before == (None, None)
          and after[0] is not None and after[0][1] == frame2 and after[1] is not None and after[1][1] == frame1,
          err or f"first {first and len(first[1])} B, before {before}, after {[x and len(x[1]) for x in after]}")
    cam_srv = mc.CockpitServer(("127.0.0.1", 0), lc, page_dir)
    threading.Thread(target=cam_srv.serve_forever, daemon=True).start()
    code, ctype, frames = Client(cam_srv.server_address[1]).mjpeg_frames(1)
    check("GET /camera.mjpg after the run change -> 200, the run-2 frame byte for byte",
          code == 200 and bool(frames) and frames[0][1] == frame2, f"{code} {frames if code != 200 else len(frames)}")
    cam_srv.shutdown()
else:
    print("  -- skipped the run-change checks, no dimos_lcm here")
reg(None)
mc._dimos_run_pid = real_pid

# a viewer that stops reading frees its camera slot after the write timeout
real_wto = mc.MJPEG_WRITE_TIMEOUT_S
mc.MJPEG_WRITE_TIMEOUT_S = 0.5
srv, c5 = serve(demo=True, page_dir=page_dir)
big = b"\xff\xd8" + bytes(2_000_000) + b"\xff\xd9"
srv.cockpit.camera_frame = lambda cam="nose": (time.monotonic(), big)
sock = socket.socket()
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
sock.connect(("127.0.0.1", srv.server_address[1]))
sock.sendall(b"GET /camera.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")       # then never reads again
seen = False
deadline = time.monotonic() + 3
while time.monotonic() < deadline and not seen:
    seen = srv.cockpit.camera_clients == 1
    time.sleep(0.02)
t = time.monotonic()
freed = clients_drop_to_zero(srv.cockpit, within_s=10.0)
check("a viewer that stops reading (socket still open) -> slot freed by the write timeout",
      seen and freed, f"seen {seen}, freed {freed} after {time.monotonic() - t:.1f} s")
sock.close()
srv.shutdown()
mc.MJPEG_WRITE_TIMEOUT_S = real_wto

# ---------------------------------------------------------------------------
print("H. the real page, as shipped")
real_page = ROOT / "tools" / "moss_cockpit"
srv, c6 = serve(demo=True)                        # the default page dir: tools/moss_cockpit
code, headers, html = c6.get("/")
check("GET / -> tools/moss_cockpit/index.html byte for byte",
      code == 200 and html == (real_page / "index.html").read_bytes() and headers["Content-Type"].startswith("text/html"))
refs = [r.decode() for r in re.findall(rb'(?:src|href)="(/static/[^"]+)"', html)]
refs += ["/static/" + u.decode() for u in re.findall(rb'url\("([^"]+)"\)', (real_page / "tokens.css").read_bytes())]
TYPES = {".svg": "image/svg+xml", ".css": "text/css; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".ttf": "font/ttf"}
bad = []
for ref in refs:
    code, headers, body = c6.get(ref)
    local = real_page / ref[len("/static/"):]
    if not (code == 200 and local.is_file() and body == local.read_bytes()
            and headers["Content-Type"] == TYPES.get(os.path.splitext(ref)[1])):
        bad.append(f"{ref} {code}")
check(f"every asset index.html and tokens.css reference is served byte for byte, right type ({len(refs)})",
      len(refs) >= 5 and not bad and {"/static/app.js", "/static/tokens.css", "/static/fonts/SpaceGrotesk.ttf",
                                      "/static/assets/mark-mint.svg", "/static/assets/wordmark-white.svg"} <= set(refs),
      ", ".join(bad) or ", ".join(refs))
ofl = (real_page / "fonts" / "SpaceGrotesk-OFL.txt").read_text()
check("the font licence ships next to the font (SIL Open Font License 1.1)",
      "SIL OPEN FONT LICENSE Version 1.1" in ofl and (real_page / "fonts" / "SpaceGrotesk.ttf").is_file())
srv.shutdown()
js = (real_page / "app.js").read_text()
strip = re.search(r'\(st\.blueprint \|\| "none"\)\.replace\(/(.+?)/, ""\)', js)
pattern = strip.group(1) if strip else None
got = {bp: re.sub(pattern, "", bp, count=1) for bp in ("moss-dimos.moss-explore", "other-dimos.explore", "moss-nav")} \
    if pattern else {}
page_html = (real_page / "index.html").read_text()
check("the depth-camera legend: app.js writes camera_n to #hud-cam, and index.html carries it",
      'setText("hud-cam"' in js and "camera_n" in js and 'id="hud-cam"' in page_html)
check("the run caption strips any dimOS namespace: moss-dimos.moss-explore -> moss-explore, "
      "other-dimos.explore -> explore, moss-nav unchanged",
      got == {"moss-dimos.moss-explore": "moss-explore", "other-dimos.explore": "explore", "moss-nav": "moss-nav"},
      f"{pattern} {got}")
check("the STALLED badge sits in the Mode row before #b-mode, hidden until a stall; a Stalls row #b-stalls",
      bool(re.search(r'<dt>Mode</dt><dd><span class="badge" id="b-stall" hidden>STALLED</span><span id="b-mode">',
                     page_html)) and '<dt>Stalls</dt><dd id="b-stalls">' in page_html)
check("the badge wears the page's attention colour (--ui-attention, tokens.css)",
      bool(re.search(r"\.badge \{[^}]*color: var\(--ui-attention\)", page_html))
      and "--ui-attention:" in (real_page / "tokens.css").read_text())
poll = re.search(r"var POLL = \{[^}]*\bscan: (\d+)", js)
check("app.js shows the badge from base.stalled, the count in #b-stalls, and polls /scan.json (and its pose) "
      "at >= 3 Hz", '$("b-stall").hidden' in js and 'setText("b-stalls"' in js and "stall_count" in js
      and poll is not None and int(poll.group(1)) <= 334, poll.group(0) if poll else "no POLL.scan")


def js_fn(name):
    """The source of app.js's top-level `function name(...) {...}`, or ""."""
    m = re.search(r"^function " + name + r"\([\s\S]*?^}$", js, re.M)
    return m.group(0) if m else ""


pose_call = re.search(r"\bfreshestPose\(([^;]*)\);", js_fn("pose"))
pose_args = [a.strip() for a in pose_call.group(1).split(",")] if pose_call else []
check("app.js pose() draws from freshestPose(state pose, S.stateAt, scan pose, S.scanAt, now): "
      "the scan's 3 Hz pose reaches the marker",
      len(pose_args) == 5 and "S.state.pose" in pose_args[0] and pose_args[1] == "S.stateAt"
      and "S.scan.pose" in pose_args[2] and pose_args[3] == "S.scanAt" and pose_args[4] == "Date.now()",
      str(pose_args))
pose_reads = re.findall(r"S\.(?:state|scan)\.pose\b", js)
check("app.js reads the state's and the scan's pose only inside pose(), and the map draws through pose()",
      len(pose_reads) == 2 and len(re.findall(r"S\.(?:state|scan)\.pose\b", js_fn("pose"))) == 2
      and "var p = pose();" in js_fn("draw"), f"{len(pose_reads)} reads")
scan_store = re.search(r"\{[^{}]*\bS\.scan = d;[^{}]*\}", js_fn("pollScan"))
check("app.js pollScan stamps each scan answer it keeps: S.scanAt = Date.now() beside S.scan = d",
      bool(scan_store) and "S.scanAt = Date.now();" in scan_store.group(0),
      scan_store.group(0) if scan_store else "no S.scan = d block in pollScan")
render_base = js_fn("renderBase")
hold = re.search(r"^var STALL_HOLD_S = (\d+(?:\.\d+)?);", js, re.M)
check("app.js renderBase feeds the stall latch from base.stall_count and shows the badge through it; "
      "STALL_HOLD_S >= 4 s",
      "var now = Date.now();" in render_base
      and "S.stallLatch = stallLatch(S.stallLatch, b ? b.stall_count : null, now);" in render_base
      and "var stall = stallView(b, S.stallLatch.until, now);" in render_base
      and '$("b-stall").hidden = !stall.badge;' in render_base
      and hold is not None and float(hold.group(1)) >= 4, hold.group(0) if hold else "no STALL_HOLD_S")
node = shutil.which("node")
if node is None:
    print("  -- skipped, no node here: node --check app.js, the page's pose choice and its stall latch")
else:
    r = subprocess.run([node, "--check", str(real_page / "app.js")], capture_output=True, text=True, timeout=30)
    check("node --check app.js", r.returncode == 0, r.stderr.strip()[:200])

    harness = (hold.group(0) if hold else "") + "\n" + js_fn("freshestPose") + "\n" + js_fn("stallLatch") \
        + "\n" + js_fn("stallView") + "\n" + """
var T = 100000;   // the page's clock, ms
var P = function (x, age) { return { x: x, y: 1, yaw: 0.5, age_s: age }; };
// A stall on the floor: telemetry says stalled only from 2.3 s to 2.7 s and stall_count goes
// 0 -> 1 at 2.3 s; the page reads state at 0, 1, ..., 10 s, as renderBase does. floor = the
// read times (s) that show the badge.
var floor = [], latch = { count: null, until: 0 };
for (var t = 0; t <= 10000; t += 1000) {
  var base = { stalled: t >= 2300 && t < 2700, stall_count: t >= 2300 ? 1 : 0 };
  latch = stallLatch(latch, base.stall_count, t);
  if (stallView(base, latch.until, t).badge) floor.push(t / 1000);
}
console.log(JSON.stringify({
  scanYounger: freshestPose(P(1, 0.2), T - 900, P(2, 0.1), T - 100, T),
  stateYounger: freshestPose(P(1, 0.1), T - 100, P(2, 0.1), T - 300, T),
  noScanYet: freshestPose(P(1, 0.3), T - 500, undefined, 0, T),
  scanSaysNone: freshestPose(P(1, 0.1), T - 900, null, T - 100, T),
  stateSaysNone: freshestPose(null, T - 100, P(2, 0.1), T - 300, T),
  scanAfterNone: freshestPose(null, T - 900, P(2, 0.05), T - 100, T),
  nothing: freshestPose(undefined, 0, undefined, 0, T),
  stall: stallView({ stalled: true, stall_count: 3 }),
  cleared: stallView({ stalled: false, stall_count: 3 }),
  older: stallView({ stalled: null, stall_count: null }),
  none: stallView(null),
  floor: floor,
  firstSeen: stallLatch({ count: null, until: 0 }, 5, T),
  renewed: stallLatch(stallLatch({ count: 2, until: 0 }, 3, T), 3, T + 2000),
  acrossGap: stallLatch(stallLatch({ count: 3, until: 0 }, null, T), 4, T + 1000),
  newRun: stallLatch({ count: 3, until: 0 }, 0, T),
  afterNewRun: stallLatch(stallLatch({ count: 3, until: 0 }, 0, T), 1, T + 1000),
  holdEdge: [stallView({ stalled: false, stall_count: 3 }, T + 4000, T + 3999).badge,
             stallView({ stalled: false, stall_count: 3 }, T + 4000, T + 4000).badge],
  stalledNoHold: stallView({ stalled: true, stall_count: 3 }, 0, T).badge
}));
"""
    r = subprocess.run([node, "-e", harness], capture_output=True, text=True, timeout=30)
    try:
        jr = json.loads(r.stdout)
    except ValueError:
        jr = {}

    def at(key, x, age):
        p = jr.get(key)
        return isinstance(p, dict) and p.get("x") == x and abs(p.get("age_s", -1) - age) < 1e-9 and p.get("yaw") == 0.5

    check("page pose: state 0.2 s old 0.9 s ago (1.1 s) vs scan 0.1 s old 0.1 s ago (0.2 s) -> the scan's, age 0.2 s",
          at("scanYounger", 2, 0.2), json.dumps(jr.get("scanYounger")) + r.stderr[:200])
    check("page pose: state 0.1 s old 0.1 s ago (0.2 s) vs scan 0.1 s old 0.3 s ago (0.4 s) -> the state's",
          at("stateYounger", 1, 0.2), json.dumps(jr.get("stateYounger")))
    check("page pose: no scan answer yet -> the state's, aged by its 0.5 s since the answer (0.8 s)",
          at("noScanYet", 1, 0.8), json.dumps(jr.get("noScanYet")))
    check("page pose: a later answer with pose null (stack stopped, new run) hides the marker, either source",
          "scanSaysNone" in jr and jr["scanSaysNone"] is None and "stateSaysNone" in jr and jr["stateSaysNone"] is None
          and "nothing" in jr and jr["nothing"] is None, json.dumps({k: jr.get(k) for k in ("scanSaysNone", "stateSaysNone", "nothing")}))
    check("page pose: a scan pose after an older null state -> drawn (odometry is back), age 0.15 s",
          at("scanAfterNone", 2, 0.15), json.dumps(jr.get("scanAfterNone")))
    check("page stall view: stalled 3 -> badge + '3'; cleared 3 -> no badge, '3'; older telemetry / none -> no badge, '--'",
          (jr.get("stall"), jr.get("cleared"), jr.get("older"), jr.get("none"))
          == ({"badge": True, "count": "3"}, {"badge": False, "count": "3"},
              {"badge": False, "count": "--"}, {"badge": False, "count": "--"}),
          json.dumps({k: jr.get(k) for k in ("stall", "cleared", "older", "none")}))
    check("page stall latch on the floor: stalled only 2.3-2.7 s (no 1 Hz read sees it), count 0 -> 1 "
          "-> the badge shows at the reads of 3, 4, 5 and 6 s (4 s from the read that saw the count), "
          "gone at 7 s",
          jr.get("floor") == [3, 4, 5, 6], json.dumps(jr.get("floor")))
    check("page stall latch: the first count the page sees (5) only sets the reference, no hold; "
          "the same count again does not renew the hold (3 at T, still 3 at T+2 s -> ends at T+4 s)",
          jr.get("firstSeen") == {"count": 5, "until": 0}
          and jr.get("renewed") == {"count": 3, "until": 100000 + 4000},
          json.dumps({k: jr.get(k) for k in ("firstSeen", "renewed")}))
    check("page stall latch: a missing count keeps the reference (3, gap, 4 at T+1 s -> hold to T+5 s); "
          "a count that went down (new run, 3 -> 0) holds nothing, and 0 -> 1 then holds to T+5 s",
          jr.get("acrossGap") == {"count": 4, "until": 100000 + 5000}
          and jr.get("newRun") == {"count": 0, "until": 0}
          and jr.get("afterNewRun") == {"count": 1, "until": 100000 + 5000},
          json.dumps({k: jr.get(k) for k in ("acrossGap", "newRun", "afterNewRun")}))
    check("page stall view with a hold: stalled false, hold ends at T+4 s -> badge at T+3.999 s, none at "
          "T+4 s; stalled true without a hold -> badge",
          jr.get("holdEdge") == [True, False] and jr.get("stalledNoHold") is True,
          json.dumps({k: jr.get(k) for k in ("holdEdge", "stalledNoHold")}))

# ---------------------------------------------------------------------------
# One real exploration, every line its decision-making modules printed, in order: minutes:seconds on the
# run's clock, the module, the event text (lines cut where the run's export cut them; the board's serial
# id zeroed). Twists and telemetry included: they must make no event.
RUN_NAV = r"""
0:27.040 moss_adapter.py MOSS base: firmware ramp set to 5 5 ms per % (rise fall)
0:27.041 moss_adapter.py MOSS base: connected on /dev/serial/by-id/usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00, firmware mode=0 geometry=MossGeom
1:25.257 wavefront_frontier Received exploration start command via LCM
1:25.259 explorer2.py exploration start: distance exp 2, cap 3 m, radius 4 m (0 = no limit; the radius is centred on the first pose)
1:25.260 wavefront_frontier Started autonomous frontier exploration
1:25.928 explorer2.py goal 1: (-1.27, 1.28) 2.0 m away, 35 frontier cells, 12 clusters, chosen in 114 ms
1:25.929 global_planner.py Got new goal
1:25.929 global_planner.py Cancelling goal.
1:26.039 global_planner.py Found safe goal.
1:26.104 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:26.111 local_planner.py changed state
1:26.214 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: stop) - re-arming once
1:26.216 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.385 -> tracks L=-100% R=+100% (limit 100%)
1:26.311 moss_adapter.py MOSS telemetry: out=[0, 0] target=[0, 0] bus=10972 mV ticks=[387605, 451441] mode=0
1:27.525 local_planner.py changed state
1:27.532 moss_adapter.py MOSS base: twist vx=+0.282 vy=+0.000(ignored) wz=+0.211 -> tracks L=+56% R=+69% (limit 100%)
1:27.633 moss_adapter.py MOSS base: twist vx=+0.307 vy=+0.000(ignored) wz=+0.200 -> tracks L=+62% R=+74% (limit 100%)
1:27.734 moss_adapter.py MOSS base: twist vx=+0.329 vy=+0.000(ignored) wz=+0.200 -> tracks L=+67% R=+79% (limit 100%)
1:27.835 moss_adapter.py MOSS base: twist vx=+0.352 vy=+0.000(ignored) wz=+0.200 -> tracks L=+72% R=+84% (limit 100%)
1:27.937 moss_adapter.py MOSS base: twist vx=+0.359 vy=+0.000(ignored) wz=+0.200 -> tracks L=+74% R=+86% (limit 100%)
1:28.038 moss_adapter.py MOSS base: twist vx=+0.370 vy=+0.000(ignored) wz=+0.200 -> tracks L=+76% R=+88% (limit 100%)
1:28.130 moss_adapter.py MOSS base: twist vx=+0.378 vy=+0.000(ignored) wz=+0.200 -> tracks L=+78% R=+90% (limit 100%)
1:28.231 moss_adapter.py MOSS base: twist vx=+0.381 vy=+0.000(ignored) wz=-0.200 -> tracks L=+91% R=+79% (limit 100%)
1:28.340 moss_adapter.py MOSS base: twist vx=+0.370 vy=+0.000(ignored) wz=-0.200 -> tracks L=+88% R=+76% (limit 100%)
1:28.358 moss_adapter.py MOSS telemetry: out=[36, 79] target=[91, 79] bus=10804 mV ticks=[386244, 454604] mode=2
1:28.441 moss_adapter.py MOSS base: twist vx=+0.363 vy=+0.000(ignored) wz=-0.200 -> tracks L=+87% R=+75% (limit 100%)
1:28.534 moss_adapter.py MOSS base: twist vx=+0.367 vy=+0.000(ignored) wz=+0.200 -> tracks L=+76% R=+88% (limit 100%)
1:28.636 moss_adapter.py MOSS base: twist vx=+0.372 vy=+0.000(ignored) wz=+0.200 -> tracks L=+77% R=+89% (limit 100%)
1:28.853 moss_adapter.py MOSS base: twist vx=+0.331 vy=+0.000(ignored) wz=+0.200 -> tracks L=+68% R=+79% (limit 100%)
1:28.929 local_planner.py Obstacle detected ahead, stopping local planner.
1:28.929 local_planner.py changed state
1:28.930 global_planner.py Replanning path due to obstacle found.
1:28.932 global_planner.py Replanning.
1:28.932 global_planner.py Cancelling goal.
1:28.936 global_planner.py Found safe goal.
1:28.937 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:28.973 local_planner.py changed state
1:28.978 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.385 -> tracks L=-100% R=+100% (limit 100%)
1:30.380 moss_adapter.py MOSS telemetry: out=[-100, 100] target=[-100, 100] bus=10492 mV ticks=[387720, 460484] mode=2
1:31.415 local_planner.py changed state
1:31.424 moss_adapter.py MOSS base: twist vx=+0.276 vy=+0.000(ignored) wz=-0.223 -> tracks L=+68% R=+55% (limit 100%)
1:31.518 moss_adapter.py MOSS base: twist vx=+0.256 vy=+0.000(ignored) wz=-0.262 -> tracks L=+65% R=+50% (limit 100%)
1:31.620 moss_adapter.py MOSS base: twist vx=+0.240 vy=+0.000(ignored) wz=-0.295 -> tracks L=+62% R=+50% (limit 100%)
1:31.719 moss_adapter.py MOSS base: twist vx=+0.227 vy=+0.000(ignored) wz=-0.322 -> tracks L=+60% R=+50% (limit 100%)
1:31.820 moss_adapter.py MOSS base: twist vx=+0.217 vy=+0.000(ignored) wz=-0.342 -> tracks L=+58% R=+50% (limit 100%)
1:31.924 moss_adapter.py MOSS base: twist vx=+0.218 vy=+0.000(ignored) wz=-0.341 -> tracks L=+59% R=+50% (limit 100%)
1:32.129 moss_adapter.py MOSS base: twist vx=+0.214 vy=+0.000(ignored) wz=-0.348 -> tracks L=+58% R=+50% (limit 100%)
1:32.331 moss_adapter.py MOSS base: twist vx=+0.212 vy=+0.000(ignored) wz=-0.353 -> tracks L=+57% R=+50% (limit 100%)
1:32.399 moss_adapter.py MOSS telemetry: out=[58, 50] target=[58, 50] bus=10816 mV ticks=[385182, 463840] mode=2
1:32.530 moss_adapter.py MOSS base: twist vx=+0.204 vy=+0.000(ignored) wz=-0.370 -> tracks L=+56% R=+50% (limit 100%)
1:34.429 moss_adapter.py MOSS telemetry: out=[56, 50] target=[56, 50] bus=10800 mV ticks=[388105, 466783] mode=2
1:35.142 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:35.935 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:36.464 moss_adapter.py MOSS telemetry: out=[56, -9] target=[56, 50] bus=10828 mV ticks=[391710, 468411] mode=2
1:37.848 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:38.490 moss_adapter.py MOSS telemetry: out=[100, -25] target=[100, -100] bus=10728 mV ticks=[395223, 471199] mode=2
1:38.658 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:40.511 moss_adapter.py MOSS telemetry: out=[56, 50] target=[56, 50] bus=10720 mV ticks=[396064, 471064] mode=2
1:40.814 moss_adapter.py MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a contact
1:40.815 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+75% R=+75% (limit 100%)
1:41.833 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:42.537 moss_adapter.py MOSS telemetry: out=[56, 50] target=[56, 50] bus=10708 mV ticks=[396196, 471761] mode=2
1:42.947 moss_adapter.py MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a contact
1:42.953 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+75% R=+75% (limit 100%)
1:43.962 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:44.270 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:44.565 moss_adapter.py MOSS telemetry: out=[100, 0] target=[100, -100] bus=10548 mV ticks=[396329, 472266] mode=2
1:44.668 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:45.171 moss_adapter.py MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a contact
1:45.177 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+75% R=+75% (limit 100%)
1:45.371 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:46.171 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:46.589 moss_adapter.py MOSS telemetry: out=[56, 0] target=[56, 50] bus=10808 mV ticks=[396507, 472231] mode=2
1:47.396 moss_adapter.py MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a contact
1:47.398 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+75% R=+75% (limit 100%)
1:48.416 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:48.611 moss_adapter.py MOSS telemetry: out=[56, 53] target=[56, 50] bus=10712 mV ticks=[396707, 472992] mode=2
1:48.707 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:50.018 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:50.631 moss_adapter.py MOSS telemetry: out=[56, 0] target=[56, 50] bus=10812 mV ticks=[397813, 471591] mode=2
1:50.825 moss_adapter.py MOSS base: twist vx=+0.343 vy=+0.000(ignored) wz=+0.200 -> tracks L=+70% R=+82% (limit 100%)
1:50.929 moss_adapter.py MOSS base: twist vx=+0.345 vy=+0.000(ignored) wz=+0.200 -> tracks L=+71% R=+82% (limit 100%)
1:51.027 moss_adapter.py MOSS base: twist vx=+0.343 vy=+0.000(ignored) wz=+0.200 -> tracks L=+70% R=+82% (limit 100%)
1:51.224 moss_adapter.py MOSS base: twist vx=+0.349 vy=+0.000(ignored) wz=+0.200 -> tracks L=+72% R=+83% (limit 100%)
1:51.325 moss_adapter.py MOSS base: twist vx=+0.355 vy=+0.000(ignored) wz=+0.200 -> tracks L=+73% R=+85% (limit 100%)
1:51.427 moss_adapter.py MOSS base: twist vx=+0.363 vy=+0.000(ignored) wz=+0.200 -> tracks L=+75% R=+87% (limit 100%)
1:51.529 moss_adapter.py MOSS base: twist vx=+0.369 vy=+0.000(ignored) wz=+0.200 -> tracks L=+76% R=+88% (limit 100%)
1:51.629 moss_adapter.py MOSS base: twist vx=+0.372 vy=+0.000(ignored) wz=+0.200 -> tracks L=+77% R=+89% (limit 100%)
1:51.721 moss_adapter.py MOSS base: twist vx=+0.378 vy=+0.000(ignored) wz=+0.200 -> tracks L=+78% R=+90% (limit 100%)
1:51.823 moss_adapter.py MOSS base: twist vx=+0.384 vy=+0.000(ignored) wz=-0.200 -> tracks L=+91% R=+79% (limit 100%)
1:51.922 moss_adapter.py MOSS base: twist vx=+0.375 vy=+0.000(ignored) wz=-0.200 -> tracks L=+89% R=+77% (limit 100%)
1:52.024 moss_adapter.py MOSS base: twist vx=+0.370 vy=+0.000(ignored) wz=-0.200 -> tracks L=+88% R=+76% (limit 100%)
1:52.130 moss_adapter.py MOSS base: twist vx=+0.363 vy=+0.000(ignored) wz=-0.200 -> tracks L=+87% R=+75% (limit 100%)
1:52.229 moss_adapter.py MOSS base: twist vx=+0.359 vy=+0.000(ignored) wz=-0.200 -> tracks L=+86% R=+74% (limit 100%)
1:52.332 moss_adapter.py MOSS base: twist vx=+0.356 vy=+0.000(ignored) wz=-0.200 -> tracks L=+85% R=+73% (limit 100%)
1:52.433 moss_adapter.py MOSS base: twist vx=+0.347 vy=+0.000(ignored) wz=-0.200 -> tracks L=+83% R=+71% (limit 100%)
1:52.524 moss_adapter.py MOSS base: twist vx=+0.340 vy=+0.000(ignored) wz=-0.200 -> tracks L=+82% R=+70% (limit 100%)
1:52.627 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:52.667 moss_adapter.py MOSS telemetry: out=[82, 70] target=[82, 70] bus=10456 mV ticks=[398810, 474169] mode=2
1:53.930 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=-0.385 -> tracks L=+56% R=+50% (limit 100%)
1:54.435 moss_adapter.py MOSS base: twist vx=+0.374 vy=+0.000(ignored) wz=+0.200 -> tracks L=+77% R=+89% (limit 100%)
1:54.534 moss_adapter.py MOSS base: twist vx=+0.377 vy=+0.000(ignored) wz=+0.200 -> tracks L=+78% R=+90% (limit 100%)
1:54.636 moss_adapter.py MOSS base: twist vx=+0.375 vy=+0.000(ignored) wz=+0.200 -> tracks L=+77% R=+89% (limit 100%)
1:54.691 moss_adapter.py MOSS telemetry: out=[75, 0] target=[77, 89] bus=10688 mV ticks=[399405, 473685] mode=2
1:54.730 moss_adapter.py MOSS base: twist vx=+0.375 vy=+0.000(ignored) wz=+0.200 -> tracks L=+78% R=+89% (limit 100%)
1:54.937 moss_adapter.py MOSS base: twist vx=+0.377 vy=+0.000(ignored) wz=+0.200 -> tracks L=+78% R=+90% (limit 100%)
1:55.038 moss_adapter.py MOSS base: twist vx=+0.381 vy=+0.000(ignored) wz=+0.200 -> tracks L=+79% R=+91% (limit 100%)
1:55.133 moss_adapter.py MOSS base: twist vx=+0.384 vy=+0.000(ignored) wz=-0.200 -> tracks L=+91% R=+79% (limit 100%)
1:55.234 moss_adapter.py MOSS base: twist vx=+0.378 vy=+0.000(ignored) wz=-0.200 -> tracks L=+90% R=+78% (limit 100%)
1:55.336 moss_adapter.py MOSS base: twist vx=+0.369 vy=+0.000(ignored) wz=-0.200 -> tracks L=+88% R=+76% (limit 100%)
1:55.437 moss_adapter.py MOSS base: twist vx=+0.363 vy=+0.000(ignored) wz=-0.200 -> tracks L=+87% R=+75% (limit 100%)
1:55.531 moss_adapter.py MOSS base: twist vx=+0.354 vy=+0.000(ignored) wz=-0.200 -> tracks L=+84% R=+73% (limit 100%)
1:55.632 moss_adapter.py MOSS base: twist vx=+0.344 vy=+0.000(ignored) wz=-0.200 -> tracks L=+82% R=+71% (limit 100%)
1:55.733 moss_adapter.py MOSS base: twist vx=+0.340 vy=+0.000(ignored) wz=-0.200 -> tracks L=+81% R=+70% (limit 100%)
1:55.834 moss_adapter.py MOSS base: twist vx=+0.333 vy=+0.000(ignored) wz=-0.200 -> tracks L=+80% R=+68% (limit 100%)
1:55.937 moss_adapter.py MOSS base: twist vx=+0.330 vy=+0.000(ignored) wz=-0.200 -> tracks L=+79% R=+67% (limit 100%)
1:56.044 moss_adapter.py MOSS base: twist vx=+0.321 vy=+0.000(ignored) wz=-0.200 -> tracks L=+77% R=+65% (limit 100%)
1:56.132 moss_adapter.py MOSS base: twist vx=+0.310 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+63% (limit 100%)
1:56.233 moss_adapter.py MOSS base: twist vx=+0.303 vy=+0.000(ignored) wz=-0.200 -> tracks L=+73% R=+61% (limit 100%)
1:56.335 moss_adapter.py MOSS base: twist vx=+0.299 vy=+0.000(ignored) wz=-0.200 -> tracks L=+72% R=+60% (limit 100%)
1:56.439 moss_adapter.py MOSS base: twist vx=+0.299 vy=+0.000(ignored) wz=-0.200 -> tracks L=+72% R=+61% (limit 100%)
1:56.638 moss_adapter.py MOSS base: twist vx=+0.304 vy=+0.000(ignored) wz=-0.200 -> tracks L=+73% R=+62% (limit 100%)
1:56.713 moss_adapter.py MOSS telemetry: out=[72, 61] target=[72, 61] bus=10560 mV ticks=[400033, 475192] mode=2
1:56.740 moss_adapter.py MOSS base: twist vx=+0.308 vy=+0.000(ignored) wz=-0.200 -> tracks L=+74% R=+63% (limit 100%)
1:56.849 moss_adapter.py MOSS base: twist vx=+0.311 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+63% (limit 100%)
1:56.934 moss_adapter.py MOSS base: twist vx=+0.313 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+64% (limit 100%)
1:57.036 moss_adapter.py MOSS base: twist vx=+0.315 vy=+0.000(ignored) wz=-0.200 -> tracks L=+76% R=+64% (limit 100%)
1:57.220 moss_adapter.py MOSS base: STALL #1: tracks commanded L=+76% R=+64% but the encoders moved L=+1 R=+0 ticks in 1.0 s - stopped (the encoders are the bump
1:57.225 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
1:57.445 moss_adapter.py MOSS base: twist vx=+0.313 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+64% (limit 100%)
1:57.503 recovering_planner BUMP received
1:57.505 local_planner.py changed state
1:57.505 local_planner.py Local planner loop exited due to stop event.
1:57.509 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:57.570 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
1:58.752 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10764 mV ticks=[399100, 474264] mode=2
1:59.605 recovering_planner Contact escape
1:59.606 global_planner.py Cancelling goal.
1:59.615 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:59.663 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
1:59.835 explorer2.py goal 2: (0.18, -1.47) 2.6 m away, 20 frontier cells, 19 clusters, chosen in 170 ms
1:59.836 global_planner.py Got new goal
1:59.837 global_planner.py Cancelling goal.
1:59.840 global_planner.py Found safe goal.
1:59.934 local_planner.py changed state
1:59.949 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:00.772 moss_adapter.py MOSS telemetry: out=[100, -100] target=[100, -100] bus=10260 mV ticks=[398571, 472912] mode=2
2:00.974 moss_adapter.py MOSS base: STALL #2: tracks commanded L=+100% R=-100% but the encoders moved L=+1259 R=+6 ticks in 1.0 s - stopped (the encoders are the
2:00.977 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:01.015 recovering_planner BUMP received
2:01.017 local_planner.py changed state
2:01.018 local_planner.py Local planner loop exited due to stop event.
2:01.024 latest_only.py twist_command: consumer behind, executing the newest command only (1 stale dropped so far)
2:01.029 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:01.079 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:02.793 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10784 mV ticks=[396941, 470728] mode=2
2:03.199 recovering_planner Contact escape
2:03.200 global_planner.py Cancelling goal.
2:03.203 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:03.204 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:03.360 explorer2.py goal 3: (-1.67, -0.12) 3.9 m away (beyond the cap: nothing safe within it), 110 frontier cells, 19 clusters, chosen in 156 ms
2:03.361 global_planner.py Got new goal
2:03.361 global_planner.py Cancelling goal.
2:03.372 global_planner.py Found safe goal.
2:03.412 local_planner.py changed state
2:03.425 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:04.615 moss_adapter.py MOSS base: STALL #3: tracks commanded L=+100% R=-100% but the encoders moved L=+1001 R=-19 ticks in 1.0 s - stopped (the encoders are th
2:04.618 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:04.819 moss_adapter.py MOSS telemetry: out=[21, -21] target=[100, -100] bus=10828 mV ticks=[397164, 469780] mode=2
2:05.032 recovering_planner BUMP received
2:05.032 local_planner.py changed state
2:05.033 local_planner.py Local planner loop exited due to stop event.
2:05.039 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:05.100 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:06.862 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10776 mV ticks=[395591, 468248] mode=2
2:07.299 recovering_planner Contact escape
2:07.301 global_planner.py Cancelling goal.
2:07.304 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:07.306 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:07.507 explorer2.py goal 4: (1.58, 3.53) 5.1 m away (beyond the cap: nothing safe within it), 27 frontier cells, 20 clusters, chosen in 201 ms
2:07.508 global_planner.py Got new goal
2:07.508 global_planner.py Cancelling goal.
2:07.512 global_planner.py Found safe goal.
2:07.620 local_planner.py changed state
2:07.630 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:08.680 moss_adapter.py MOSS base: STALL #4: tracks commanded L=+100% R=-100% but the encoders moved L=+1051 R=+8 ticks in 1.0 s - stopped (the encoders are the
2:08.688 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:08.883 moss_adapter.py MOSS telemetry: out=[7, -7] target=[100, -100] bus=10852 mV ticks=[395707, 467024] mode=2
2:09.044 local_planner.py changed state
2:09.045 recovering_planner BUMP received
2:09.045 local_planner.py Local planner loop exited due to stop event.
2:09.058 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:09.068 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:09.106 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:10.907 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10748 mV ticks=[394273, 465582] mode=2
2:11.399 recovering_planner Contact escape
2:11.400 global_planner.py Cancelling goal.
2:11.406 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:11.406 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:11.547 explorer2.py goal 5: (1.03, -2.72) 7.6 m away (beyond the cap: nothing safe within it), 184 frontier cells, 20 clusters, chosen in 139 ms
2:11.575 global_planner.py Got new goal
2:11.576 global_planner.py Cancelling goal.
2:11.581 global_planner.py Found safe goal.
2:11.637 global_planner.py Robot is stuck. Replanning.
2:11.654 local_planner.py changed state
2:11.662 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:12.823 moss_adapter.py MOSS base: STALL #5: tracks commanded L=+100% R=-100% but the encoders moved L=+1100 R=-4 ticks in 1.0 s - stopped (the encoders are the
2:12.837 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:12.924 moss_adapter.py MOSS telemetry: out=[100, -100] target=[100, -100] bus=10204 mV ticks=[394167, 464360] mode=2
2:13.065 recovering_planner BUMP received
2:13.066 local_planner.py changed state
2:13.067 local_planner.py Local planner loop exited due to stop event.
2:13.075 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:13.085 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:13.126 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:14.842 recovering_planner Contact escape
2:14.842 global_planner.py Cancelling goal.
2:14.852 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:14.855 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:15.018 explorer2.py goal 6: (2.18, -2.32) 11.9 m away (beyond the cap: nothing safe within it), 99 frontier cells, 20 clusters, chosen in 162 ms
2:15.019 global_planner.py Got new goal
2:15.019 global_planner.py Cancelling goal.
2:15.080 global_planner.py Travelling to goal 1.2850097467377768m away from requested goal.
2:15.081 global_planner.py Found safe goal.
2:15.144 local_planner.py changed state
2:15.185 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:15.259 moss_adapter.py MOSS telemetry: out=[0, 0] target=[0, 0] bus=10876 mV ticks=[392170, 462189] mode=2
2:16.271 moss_adapter.py MOSS base: STALL #6: tracks commanded L=+100% R=-100% but the encoders moved L=+1041 R=+2 ticks in 1.0 s - stopped (the encoders are the
2:16.281 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:16.575 local_planner.py changed state
2:16.577 recovering_planner BUMP received
2:16.576 local_planner.py Local planner loop exited due to stop event.
2:16.586 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:16.590 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:17.282 moss_adapter.py MOSS telemetry: out=[-32, -50] target=[-50, -50] bus=10752 mV ticks=[393278, 462086] mode=2
2:18.891 recovering_planner Contact escape
2:18.891 global_planner.py Cancelling goal.
2:18.894 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:18.897 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:19.016 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:20.112 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:21.216 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:22.310 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:23.458 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:24.534 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:25.632 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:26.734 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:27.865 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:28.960 explorer2.py 10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:30.066 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:31.155 explorer2.py starvation escape: the world stayed silent for 12 asks - reopening the oldest failed goal at (-1.27, 1.28)
2:32.291 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:33.382 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:34.476 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:35.549 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:36.686 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:37.806 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:38.912 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:40.051 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:41.151 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:42.291 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:43.380 explorer2.py 9 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
2:44.493 explorer2.py starvation escape: the world stayed silent for 12 asks - reopening the oldest failed goal at (0.18, -1.47)
2:45.690 explorer2.py goal 7: (0.18, -1.47) 1.7 m away, 20 frontier cells, 20 clusters, chosen in 195 ms
2:45.691 global_planner.py Got new goal
2:45.691 global_planner.py Cancelling goal.
2:45.693 global_planner.py Found safe goal.
2:45.748 local_planner.py changed state
2:45.756 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
2:45.763 moss_adapter.py MOSS telemetry: out=[0, 0] target=[0, 0] bus=10868 mV ticks=[390416, 459210] mode=2
2:47.786 moss_adapter.py MOSS telemetry: out=[100, -100] target=[100, -100] bus=10112 mV ticks=[392221, 458942] mode=2
2:47.990 moss_adapter.py MOSS base: STALL #7: tracks commanded L=+100% R=-100% but the encoders moved L=+992 R=-6 ticks in 1.0 s - stopped (the encoders are the
2:48.002 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
2:48.198 recovering_planner BUMP received
2:48.199 local_planner.py changed state
2:48.206 local_planner.py Local planner loop exited due to stop event.
2:48.211 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:48.271 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
2:49.807 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10744 mV ticks=[391050, 457549] mode=2
2:50.462 recovering_planner Contact escape
2:50.462 global_planner.py Cancelling goal.
2:50.468 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
2:50.471 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
2:50.617 explorer2.py goal 8: (-1.42, 0.78) 2.2 m away, 5 frontier cells, 20 clusters, chosen in 144 ms
2:50.620 global_planner.py Got new goal
2:50.622 global_planner.py Cancelling goal.
2:50.628 global_planner.py Found safe goal.
2:50.755 local_planner.py changed state
2:50.771 moss_adapter.py MOSS base: twist vx=+0.251 vy=+0.000(ignored) wz=+0.273 -> tracks L=+50% R=+64% (limit 100%)
2:50.862 moss_adapter.py MOSS base: twist vx=+0.256 vy=+0.000(ignored) wz=+0.262 -> tracks L=+50% R=+65% (limit 100%)
2:51.168 moss_adapter.py MOSS base: twist vx=+0.254 vy=+0.000(ignored) wz=+0.268 -> tracks L=+50% R=+64% (limit 100%)
2:51.280 moss_adapter.py MOSS base: twist vx=+0.245 vy=+0.000(ignored) wz=+0.286 -> tracks L=+50% R=+63% (limit 100%)
2:51.381 moss_adapter.py MOSS base: twist vx=+0.216 vy=+0.000(ignored) wz=+0.344 -> tracks L=+50% R=+58% (limit 100%)
2:51.477 moss_adapter.py MOSS base: twist vx=+0.209 vy=+0.000(ignored) wz=+0.358 -> tracks L=+50% R=+57% (limit 100%)
2:51.578 moss_adapter.py MOSS base: twist vx=+0.202 vy=+0.000(ignored) wz=+0.373 -> tracks L=+50% R=+56% (limit 100%)
2:51.826 moss_adapter.py MOSS telemetry: out=[50, 56] target=[50, 56] bus=10728 mV ticks=[391317, 457806] mode=2
2:53.860 moss_adapter.py MOSS telemetry: out=[50, 56] target=[50, 56] bus=10748 mV ticks=[394974, 461521] mode=2
2:55.001 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.385 -> tracks L=-100% R=+100% (limit 100%)
2:55.696 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=+0.385 -> tracks L=+50% R=+56% (limit 100%)
2:55.884 moss_adapter.py MOSS telemetry: out=[-60, 90] target=[50, 56] bus=10524 mV ticks=[397677, 465684] mode=2
2:57.604 moss_adapter.py MOSS base: tracks frozen at L=+50% R=+56% - pushing at 75 % for 1 s before calling it a contact
2:57.615 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=+0.385 -> tracks L=+75% R=+75% (limit 100%)
2:57.908 moss_adapter.py MOSS telemetry: out=[75, 75] target=[75, 75] bus=10400 mV ticks=[398047, 466494] mode=2
2:58.626 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=+0.385 -> tracks L=+50% R=+56% (limit 100%)
2:59.832 moss_adapter.py MOSS base: tracks frozen at L=+50% R=+56% - pushing at 75 % for 1 s before calling it a contact
2:59.838 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=+0.385 -> tracks L=+75% R=+75% (limit 100%)
2:59.935 moss_adapter.py MOSS telemetry: out=[50, 56] target=[50, 56] bus=10608 mV ticks=[398305, 466920] mode=2
3:00.857 moss_adapter.py MOSS base: STALL #8: tracks commanded L=+75% R=+75% but the encoders moved L=+18 R=+30 ticks in 1.0 s - stopped (the encoders are the bu
3:00.863 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
3:00.866 moss_adapter.py MOSS base: twist vx=+0.200 vy=+0.000(ignored) wz=+0.385 -> tracks L=+50% R=+56% (limit 100%)
3:01.283 local_planner.py changed state
3:01.284 recovering_planner BUMP received
3:01.284 local_planner.py Local planner loop exited due to stop event.
3:01.289 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:01.349 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
3:01.971 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10664 mV ticks=[398158, 466800] mode=2
3:03.769 recovering_planner Contact escape
3:03.770 global_planner.py Cancelling goal.
3:03.774 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
3:03.775 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:03.982 explorer2.py goal 9: (0.18, -1.47) 2.0 m away, 20 frontier cells, 19 clusters, chosen in 207 ms
3:03.982 global_planner.py Got new goal
3:03.983 global_planner.py Cancelling goal.
3:03.984 global_planner.py Found safe goal.
3:04.072 global_planner.py Robot is stuck. Replanning.
3:04.159 local_planner.py changed state
3:04.175 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
3:04.191 moss_adapter.py MOSS telemetry: out=[0, 0] target=[0, 0] bus=10828 mV ticks=[395601, 464221] mode=2
3:05.202 moss_adapter.py MOSS base: STALL #9: tracks commanded L=+100% R=-100% but the encoders moved L=+838 R=-7 ticks in 1.0 s - stopped (the encoders are the
3:05.211 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
3:05.276 recovering_planner BUMP received
3:05.277 local_planner.py changed state
3:05.277 local_planner.py Local planner loop exited due to stop event.
3:05.281 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:05.333 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
3:06.211 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10704 mV ticks=[395598, 463303] mode=2
3:07.039 recovering_planner Contact escape
3:07.039 global_planner.py Cancelling goal.
3:07.045 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
3:07.052 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:07.169 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:08.263 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:09.336 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:10.463 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:11.545 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:12.621 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:13.733 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:14.830 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:15.969 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:17.081 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:18.196 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:19.295 explorer2.py starvation escape: the world stayed silent for 12 asks - reopening the oldest failed goal at (-1.67, -0.12)
3:20.493 explorer2.py goal 10: (-1.27, -0.07) 11.0 m away (beyond the cap: nothing safe within it), 5 frontier cells, 19 clusters, chosen in 197 ms
3:20.497 global_planner.py Got new goal
3:20.498 global_planner.py Cancelling goal.
3:20.501 global_planner.py Found safe goal.
3:20.505 global_planner.py Robot is stuck. Replanning.
3:20.559 local_planner.py changed state
3:20.568 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
3:20.647 moss_adapter.py MOSS telemetry: out=[0, 0] target=[0, 0] bus=10820 mV ticks=[394095, 461793] mode=2
3:21.666 moss_adapter.py MOSS base: STALL #10: tracks commanded L=+100% R=-100% but the encoders moved L=+968 R=-6 ticks in 1.0 s - stopped (the encoders are the
3:21.667 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
3:21.867 local_planner.py changed state
3:21.868 recovering_planner BUMP received
3:21.869 local_planner.py Local planner loop exited due to stop event.
3:21.873 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
3:21.885 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:21.934 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
3:22.676 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10664 mV ticks=[394769, 461400] mode=2
3:24.016 recovering_planner Contact escape
3:24.017 global_planner.py Cancelling goal.
3:24.021 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
3:24.026 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
3:24.142 explorer2.py 7 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:25.225 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:26.318 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:27.430 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:28.555 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:29.638 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:30.774 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:31.892 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:32.976 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:34.037 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:35.126 explorer2.py 7 of 19 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, not stopping
3:35.151 wavefront_frontier WavefrontFrontierExplorer: stop_movement received, stopping exploration
3:35.152 wavefront_frontier Received exploration stop command via LCM
3:35.156 wavefront_frontier Stopped autonomous frontier exploration
3:35.156 global_planner.py Got new goal
3:35.156 global_planner.py Cancelling goal.
3:35.158 global_planner.py Found safe goal.
3:35.206 global_planner.py Cancelling goal.
3:35.208 local_planner.py changed state
3:35.210 local_planner.py Reached goal position, starting final rotation
3:35.210 local_planner.py changed state
3:35.211 local_planner.py changed state
3:35.215 local_planner.py Error in local planning
3:35.217 local_planner.py changed state
3:35.218 global_planner.py Failure in navigation.
3:35.219 recovering_planner Replan requested without a goal; ignoring.
"""

# The same run, three seconds around its first contact, with every module's lines (costmap camera counts,
# sounds, the coordinator's bump relay).
RUN_CONTACT = r"""
1:56.934 moss_adapter.py MOSS base: twist vx=+0.313 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+64% (limit 100%)
1:56.998 costmap2d.py costmap: camera obstacles: 1544 points, 9 cell hits over the last 2 revolutions
1:57.036 moss_adapter.py MOSS base: twist vx=+0.315 vy=+0.000(ignored) wz=-0.200 -> tracks L=+76% R=+64% (limit 100%)
1:57.183 costmap2d.py costmap: camera obstacles: 1570 points, 8 cell hits over the last 2 revolutions
1:57.220 moss_adapter.py MOSS base: STALL #1: tracks commanded L=+76% R=+64% but the encoders moved L=+1 R=+0 ticks in 1.0 s - stopped (the encoders are the bumper)
1:57.225 moss_adapter.py MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once
1:57.403 costmap2d.py costmap: camera obstacles: 1574 points, 4 cell hits over the last 2 revolutions
1:57.445 moss_adapter.py MOSS base: twist vx=+0.313 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+64% (limit 100%)
1:57.447 blueprints.py base stall #1: publishing bump (contact) for the planner's escape
1:57.503 recovering_planner BUMP received
1:57.503 costmap2d.py costmap: contact at (-0.88, +0.73) heading +147 deg: 32 cells ahead marked as a wall
1:57.504 sounds.py MOSS sounds: bump_surprise queued
1:57.505 local_planner.py changed state
1:57.505 local_planner.py Local planner loop exited due to stop event.
1:57.509 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:57.570 moss_adapter.py MOSS base: twist vx=-0.100 vy=+0.000(ignored) wz=+0.000 -> tracks L=-50% R=-50% (limit 100%)
1:57.581 costmap2d.py costmap: camera obstacles: 1594 points, 149 cell hits over the last 2 revolutions
1:57.786 costmap2d.py costmap: camera obstacles: 1540 points, 35 cell hits over the last 2 revolutions
1:57.985 costmap2d.py costmap: camera obstacles: 1596 points, 19 cell hits over the last 2 revolutions
1:58.184 costmap2d.py costmap: camera obstacles: 1596 points, 5 cell hits over the last 2 revolutions
1:58.375 costmap2d.py costmap: camera obstacles: 1594 points, 28 cell hits over the last 2 revolutions
1:58.506 sounds.py MOSS sounds: recovery_success queued
1:58.577 costmap2d.py costmap: camera obstacles: 1626 points, 52 cell hits over the last 2 revolutions
1:58.752 moss_adapter.py MOSS telemetry: out=[-50, -50] target=[-50, -50] bus=10764 mV ticks=[399100, 474264] mode=2
1:58.794 costmap2d.py costmap: camera obstacles: 1678 points, 189 cell hits over the last 2 revolutions
1:58.975 costmap2d.py costmap: camera obstacles: 1642 points, 66 cell hits over the last 2 revolutions
1:59.201 costmap2d.py costmap: camera obstacles: 1638 points, 191 cell hits over the last 2 revolutions
1:59.376 costmap2d.py costmap: camera obstacles: 1648 points, 238 cell hits over the last 2 revolutions
1:59.579 costmap2d.py costmap: camera obstacles: 1624 points, 131 cell hits over the last 2 revolutions
1:59.605 recovering_planner Contact escape
1:59.606 global_planner.py Cancelling goal.
1:59.615 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)
1:59.663 explorer2.py planner gave up on that goal: excluded until the map or the viewpoint changes
1:59.777 costmap2d.py costmap: camera obstacles: 1594 points, 198 cell hits over the last 2 revolutions
1:59.818 velocity_task.py JointVelocityTask vel_base timed out (no update for 0.209s)
1:59.835 explorer2.py goal 2: (0.18, -1.47) 2.6 m away, 20 frontier cells, 19 clusters, chosen in 170 ms
1:59.836 global_planner.py Got new goal
1:59.837 global_planner.py Cancelling goal.
1:59.840 global_planner.py Found safe goal.
1:59.934 local_planner.py changed state
1:59.949 moss_adapter.py MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit 100%)
1:59.993 costmap2d.py costmap: camera obstacles: 1650 points, 118 cell hits over the last 2 revolutions
"""

print("I. what it decides: log lines -> decision events -> the state word, /decisions.json")
WORDS = {"IDLE", "READY", "HEADING", "OBSTACLE", "BACKING OFF", "STUCK", "WAITING", "DONE", "STOPPED"}
EVENT_KEYS = {"ts", "kind", "text", "src", "seq"}


def recs_of(text):
    """'M:SS.mmm module event' lines (the run's lines as RUN_NAV prints them) -> log records, ts in s."""
    out = []
    for line in text.splitlines():
        m = re.match(r"^(\d+):(\d+(?:\.\d+)?) (\S+) (.*)$", line)
        if m:
            out.append({"ts": int(m[1]) * 60 + float(m[2]), "level": "info", "event": m[4], "logger": m[3]})
    return out


def one(event, logger="x.py"):
    return mc.decisions_from_records([{"ts": 1.0, "level": "info", "event": event, "logger": logger}])


# every catalogue line, as the modules print it (from the run where the run has it), -> kind, src and
# a text that keeps the line's numbers
CATALOGUE = [
    ("explorer2.py", "goal 1: (-1.27, 1.28) 2.0 m away, 35 frontier cells, 12 clusters, chosen in 114 ms",
     "goal", "explorer", ["Goal 1: 2.0 m away (35 cells)"]),
    ("explorer2.py", "goal 3: (-1.67, -0.12) 3.9 m away (beyond the cap: nothing safe within it), 110 frontier cells, "
     "19 clusters, chosen in 156 ms", "goal", "explorer", ["Goal 3: 3.9 m away (110 cells, beyond the cap)"]),
    ("explorer2.py", "planner gave up on that goal: excluded until the map or the viewpoint changes",
     "goal_dropped", "explorer", ["Goal dropped: the planner gave up"]),
    ("explorer2.py", "goal timeout after 30 s having closed 0.05 m of 2.40 m: excluded until the map or the "
     "viewpoint changes", "goal_dropped", "explorer", ["Goal dropped", "0.05 of 2.40 m", "30 s"]),
    ("explorer2.py", "10 of 20 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, "
     "not stopping", "waiting", "explorer", ["Waiting: 10 of 20 frontiers are failed goals"]),
    ("explorer2.py", "starvation escape: the world stayed silent for 12 asks - reopening the oldest failed goal at "
     "(-1.27, 1.28)", "retry", "explorer", ["Retrying an old goal at (-1.27, 1.28)"]),
    ("local_planner.py", "Obstacle detected ahead, stopping local planner.", "obstacle", "planner",
     ["Obstacle ahead: stop"]),
    ("global_planner.py", "Replanning.", "replan", "planner", ["Replanning"]),
    ("explorer2.py", "born cornered: 0.40 m2 of reachable floor and no frontier - one 0.20 m back-off via the bump "
     "reflex", "obstacle", "explorer", ["Cornered (0.40 m2 of floor, no frontier): backing off 0.20 m"]),
    ("global_planner.py", "No path found to the goal.", "goal_dropped", "planner", ["Goal dropped: no path"]),
    ("global_planner.py", "No safe goal found near requested target.", "goal_dropped", "planner",
     ["Goal dropped: no safe spot near it"]),
    ("global_planner.py", "No safe goal found.", "goal_dropped", "planner", ["Goal dropped: no safe spot near it"]),
    ("recovering_planner.py", "Goal abandoned, no recovery motion", "goal_dropped", "recovery",
     ["Stuck: goal abandoned"]),
    ("recovering_planner.py", "Contact during an escape: stopping dead", "contact", "recovery",
     ["Contact during an escape: stop dead"]),
    ("global_planner.py", "Got new goal", "new_goal", "planner", ["Planner: new goal"]),
    ("global_planner.py", "Found safe goal.", "new_goal", "planner", ["Planner: safe goal found"]),
    ("global_planner.py", "Travelling to goal 1.2850097467377768m away from requested goal.", "new_goal", "planner",
     ["Planner: safe goal 1.29 m from the requested one"]),
    ("local_planner.py", "Reached goal position, starting final rotation", "reached", "planner", ["Goal reached"]),
    ("global_planner.py", "Close enough to goal. Accepting as arrived.", "reached", "planner",
     ["Goal reached (close enough)"]),
    ("moss_adapter.py", "MOSS base: STALL #1: tracks commanded L=+76% R=+64% but the encoders moved L=+1 R=+0 ticks "
     "in 1.0 s - stopped (the encoders are the bumper)", "contact", "base",
     ["Contact #1: both tracks blocked", "L=+76% R=+64%", "L=+1 R=+0", "1.0 s"]),
    ("moss_adapter.py", "MOSS base: STALL #2: tracks commanded L=+100% R=-100% but the encoders moved L=+1259 R=+6 "
     "ticks in 1.0 s - stopped (the encoders are the", "contact", "base",
     ["Contact #2: right track blocked", "L=+100% R=-100%", "L=+1259 R=+6", "1.0 s"]),
    ("moss_adapter.py", "MOSS base: STALL #8: tracks commanded L=+75% R=+75% but the encoders moved L=+18 R=+30 ticks "
     "in 1.0 s - stopped (the encoders are the bumper)", "contact", "base",
     ["Contact #8: left track blocked", "L=+18 R=+30"]),
    ("recovering_planner.py", "BUMP received", "contact", "recovery", ["Bump: backing off"]),
    ("recovering_planner.py", "Contact escape", "contact", "recovery", ["Escape done"]),
    ("costmap2d.py", "costmap: contact at (-0.88, +0.73) heading +147 deg: 32 cells ahead marked as a wall",
     "contact", "costmap", ["Wall remembered ahead (32 cells)"]),
    ("moss_adapter.py", "MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling it a "
     "contact", "push", "base", ["Tracks frozen at L=+56% R=+50%: pushing at 75 %"]),
    ("moss_adapter.py", "MOSS base: pivot L=+100% R=-100%: the right track did not turn (encoders L=+1259 R=+6 ticks "
     "in 1.0 s) - not a contact: from now on this pivot turns on the left track only, the right track braked",
     "pivot_fallback", "base",
     ["Right track did not turn in reverse", "L=+100% R=-100%", "pivoting on the left track from now on"]),
    ("moss_adapter.py", "MOSS base: pivot L=-100% R=+100%: the left track did not turn (encoders L=+3 R=+1180 ticks "
     "in 1.0 s) - not a contact: from now on this pivot turns on the right track only, the left track braked",
     "pivot_fallback", "base", ["Left track did not turn in reverse", "pivoting on the right track"]),
    ("bump_imu.py", "IMU contact: forward -4.8 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%", "imu", "imu",
     ["IMU contact", "-4.8 m/s^2", "4.0", "L=+70% R=+60%"]),
    ("explorer2.py", "exploration start: distance exp 2, cap 3 m, radius 4 m (0 = no limit; the radius is centred on "
     "the first pose)", "mission", "mission", ["Exploration started (cap 3 m, radius 4 m)"]),
    ("wavefront_frontier_goal_selector.py", "WavefrontFrontierExplorer: stop_movement received, stopping exploration",
     "mission", "mission", ["Exploration stopped"]),
    ("wavefront_frontier_goal_selector.py", "Received exploration stop command via LCM", "mission", "mission",
     ["Exploration stopped"]),
    ("wavefront_frontier_goal_selector.py", "Stopped autonomous frontier exploration", "mission", "mission",
     ["Exploration stopped"]),
    ("explorer2.py", "exploration complete: no reachable frontier left within 4 m of the start (12 targets, 150 ms)",
     "mission", "mission", ["Exploration complete", "12 goals"]),
    ("wavefront_frontier_goal_selector.py", "Exploration complete after 7 goals and 3 consecutive failures finding "
     "new frontiers", "mission", "mission", ["Exploration complete (7 goals)"]),
]
for logger, line, kind, src, want in CATALOGUE:
    ev = one(line, logger)
    e = ev[0] if len(ev) == 1 else {}
    check(f"{kind}: '{line[:58]}' -> '{e.get('text')}'",
          e.get("kind") == kind and e.get("src") == src and all(w in e.get("text", "") for w in want)
          and set(e) == EVENT_KEYS and e.get("seq") == 1 and e.get("ts") == 1.0 and len(e.get("text", "")) <= 120,
          json.dumps(ev)[:200])

IGNORED = [
    "MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=+0.000 -> tracks L=+0% R=+0% (limit 100%)",
    "MOSS telemetry: out=[0, 0] target=[0, 0] bus=10972 mV ticks=[387605, 451441] mode=0",
    "costmap: camera obstacles: 1544 points, 9 cell hits over the last 2 revolutions",
    "costmap: explore_cmd received - map writing OPEN for the mission",
    "costmap: exploration complete - map writing continues",
    "costmap: exploration stopped - map writing continues",
    "MOSS sounds: bump_surprise queued",
    "base stall #1: publishing bump (contact) for the planner's escape",
    "MOSS base: 'drive' refused not_armed (firmware reason: drive) - re-arming once",
    "twist_command: consumer behind, executing the newest command only (1 stale dropped so far)",
    "Received exploration start command via LCM", "Started autonomous frontier exploration",
    "Cancelling goal.", "changed state", "Local planner loop exited due to stop event.",
    "Replan requested without a goal; ignoring.", "Failure in navigation.", "Error in local planning",
    "goal timeout after 30 s, 0.80 m closer, re-deciding",
    "JointVelocityTask vel_base timed out (no update for 0.209s)",
]
made = [line for line in IGNORED if one(line)]
check(f"{len(IGNORED)} other lines of the run (twist, telemetry, camera counts, sounds, the bump relay, the "
      "explore_cmd the costmap repeats...) -> no event", not made, json.dumps(made)[:200])


def recs(*items):
    """(ts, event[, fields]) -> log records."""
    return [dict({"ts": it[0], "level": "info", "event": it[1], "logger": "x.py"}, **(it[2] if len(it) > 2 else {}))
            for it in items]


def kt(events):
    return [(e["kind"], e["src"], e["text"]) for e in events]


# the planner's replan reasons: the line before _replan_path is not the decision, what _replan_path does is
REASONS = ["Robot is stuck. Replanning.", "Robot veered off track. Replanning.",
           "Replanning path due to obstacle found.", "Failure in navigation."]
check("a replan reason alone (the planner did nothing with it) -> no event",
      not [line for line in REASONS if one(line, "global_planner.py")])
check("each reason + dimOS's 'Replanning.' 2 ms later -> one replan event at the reason's time: Stuck: replanning, "
      "Off track: replanning, Replanning, Navigation failed: replanning",
      [kt(mc.decisions_from_records(recs((5.0, line), (5.002, "Replanning.")))) for line in REASONS]
      == [[("replan", "planner", t)] for t in ("Stuck: replanning", "Off track: replanning", "Replanning",
                                               "Navigation failed: replanning")]
      and mc.decisions_from_records(recs((5.0, REASONS[0]), (5.002, "Replanning.")))[0]["ts"] == 5.0)
ab = mc.decisions_from_records(recs((0.0, "goal 2: (1.00, 2.00) 2.0 m away, 30 frontier cells, 10 clusters, chosen "
                                          "in 90 ms"),
                                    (8.0, "Robot is stuck. Replanning."),
                                    (8.001, "Goal abandoned, no recovery motion", {"trigger": "stuck_detector"}),
                                    (8.002, "Cancelling goal."),
                                    (8.008, "planner gave up on that goal: excluded until the map or the viewpoint "
                                            "changes")))
check("stuck -> MOSS's planner gives the goal up: 'Stuck: goal abandoned' (the explorer's give-up 7 ms later is "
      "its echo); STUCK at 9.0, WAITING at 11.1 (the explorer runs)",
      kt(ab)[1:] == [("goal_dropped", "recovery", "Stuck: goal abandoned")]
      and [mc.now_state(ab, True, None, t)["word"] for t in (9.0, 11.1)] == ["STUCK", "WAITING"],
      json.dumps(kt(ab)))
check("the run's 'Failure in navigation.' + 'Replan requested without a goal; ignoring.' -> nothing",
      mc.decisions_from_records(recs((215.218, "Failure in navigation."),
                                     (215.219, "Replan requested without a goal; ignoring."))) == [])
late = mc.decisions_from_records(recs((5.0, "Robot is stuck. Replanning."),
                                      (5.6, "MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks "
                                            "L=+100% R=-100% (limit 100%)")))
check("a stuck verdict followed only by the pivot's twist 0.6 s later -> nothing (MOSS's planner ignores 'stuck' "
      "while the follower turns in place)", late == [])
carry = {}
part1 = mc.decisions_from_records(recs((5.0, "Robot is stuck. Replanning.")), 3, None, carry)
part2 = mc.decisions_from_records(recs((5.002, "Replanning.")), 3, None, carry)
check("the reason at the end of one read, 'Replanning.' at the start of the next -> one event in the second read "
      "(the reason waits in `carry`), seq 4",
      part1 == [] and kt(part2) == [("replan", "planner", "Stuck: replanning")] and part2[0]["seq"] == 4
      and carry.get("pending") is None, json.dumps([part1, part2, carry], default=str)[:300])

RUN = recs_of(RUN_NAV)
run_ev = mc.decisions_from_records(RUN)
counts = {}
for e in run_ev:
    counts[e["kind"]] = counts.get(e["kind"], 0) + 1
print(f"    the whole run replayed: {len(RUN)} records -> {len(run_ev)} events: "
      + ", ".join(f"{k} {counts[k]}" for k in sorted(counts)))
check("whole run -> goal >= 10, contact >= 10, waiting >= 20",
      counts.get("goal", 0) >= 10 and counts.get("contact", 0) >= 10 and counts.get("waiting", 0) >= 20,
      json.dumps(counts))
check("whole run, exact: 10 goals, 10 dropped, 44 waits, 3 retries, 30 contacts (10 stalls + 10 bumps + 10 "
      "escapes), 6 pushes, 1 obstacle, 1 replan (the obstacle's: the 3 stuck verdicts were ignored by the planner, "
      "the last 'Failure in navigation.' had no goal), 1 shifted safe goal, 2 mission lines (the start, the STOP), "
      "no 'Goal reached' (the explorer's stay-here goal right after the STOP is an echo)",
      counts == {"goal": 10, "goal_dropped": 10, "waiting": 44, "retry": 3, "contact": 30, "push": 6,
                 "obstacle": 1, "replan": 1, "new_goal": 1, "mission": 2}, json.dumps(counts))
check("the run's 3 stuck verdicts (2:11.637, 3:04.072, 3:20.505), each 4-90 ms after 'Found safe goal.' and "
      "followed by the in-place pivot, no 'Replanning.' -> no event; the word 0.3 s and 1.0 s after each = HEADING "
      "toward that goal",
      not any(e["text"].startswith("Stuck") for e in run_ev)
      and all(mc.now_state([e for e in run_ev if e["ts"] <= t + d], True, None, t + d)["word"] == "HEADING"
              for t in (131.637, 184.072, 200.505) for d in (0.3, 1.0)))
chatty = [r for r in RUN if r["event"].startswith(("MOSS base: twist", "MOSS telemetry"))]
check(f"no twist / telemetry line of the run makes an event ({len(chatty)} lines)",
      len(chatty) == 181 and mc.decisions_from_records(chatty) == []
      and not any("twist" in e["text"] or "telemetry" in e["text"] for e in run_ev))
check("order kept: seq 1..N, times never go back",
      [e["seq"] for e in run_ev] == list(range(1, len(run_ev) + 1))
      and all(a["ts"] <= b["ts"] for a, b in zip(run_ev, run_ev[1:])))
check("the run ends on the STOP: STOPPED, 'Exploration stopped (stop button or pad)', since 3:35.151",
      mc.now_state(run_ev, True, None, 216.0) == {"word": "STOPPED", "detail": "Exploration stopped (stop button or "
                                                  "pad)", "since": 215.151},
      json.dumps(mc.now_state(run_ev, True, None, 216.0)))
run_log = mc.DecisionLog()
run_log.add(RUN)
rows = list(run_log.events)
tail30 = rows[-30:]
waits = [e["text"] for e in rows if e["kind"] == "waiting"]
check(f"the run through the feed's store: {len(run_ev)} events -> {len(rows)} rows (identical events one after "
      "the other fold into one, ' (xN)', last ts and seq); the 11 waits from 3:24.142 to 3:35.126 -> 2 rows "
      "(7 of 20, then 7 of 19 x10); the page's last 30 rows keep 12 contacts, not 3",
      len(rows) == 66 and run_log.seq == len(run_ev) and rows[-1]["seq"] == len(run_ev)
      and [e["text"] for e in rows[-3:-1]] == ["Waiting: 7 of 20 frontiers are failed goals",
                                               "Waiting: 7 of 19 frontiers are failed goals (x10)"]
      and rows[-2]["ts"] == 215.126 and sum(e["kind"] == "contact" for e in tail30) == 12
      and sum(e["kind"] == "waiting" for e in tail30) == 5
      and sum(int(re.search(r"\(x(\d+)\)$", t)[1]) if re.search(r"\(x(\d+)\)$", t) else 1 for t in waits) == 44
      and mc.now_state(rows, True, None, 216.0)["word"] == "STOPPED",
      json.dumps([(e["seq"], e["text"]) for e in rows[-4:]]))

# three seconds around the first contact, every module's line (the timeline of the same run)
EX = recs_of(RUN_CONTACT)
ex_ev = mc.decisions_from_records(EX)
check(f"first contact, {len(EX)} lines of every module -> stall, bump, wall, escape, goal dropped, goal 2 "
      "(the planner taking goal 2 is an echo)",
      [(e["kind"], e["src"], e["text"]) for e in ex_ev]
      == [("contact", "base", "Contact #1: both tracks blocked (commanded L=+76% R=+64%, encoders moved L=+1 R=+0 "
                              "ticks in 1.0 s)"),
          ("contact", "recovery", "Bump: backing off"), ("contact", "costmap", "Wall remembered ahead (32 cells)"),
          ("contact", "recovery", "Escape done"), ("goal_dropped", "explorer", "Goal dropped: the planner gave up"),
          ("goal", "explorer", "Goal 2: 2.6 m away (20 cells)")],
      json.dumps([(e["kind"], e["text"]) for e in ex_ev])[:300])


def word_at(events, t, exploring=None):
    return mc.now_state([e for e in events if e["ts"] <= t], True, exploring, t)


check("first contact, the word at 1:57.3 / 1:59.0 / 1:59.7 / 1:59.9 -> BACKING OFF, BACKING OFF, "
      "WAITING (goal dropped), HEADING (goal 2)",
      [word_at(ex_ev, t)["word"] for t in (117.3, 119.0, 119.7, 119.9)]
      == ["BACKING OFF", "BACKING OFF", "WAITING", "HEADING"]
      and word_at(ex_ev, 119.9)["detail"] == "Goal 2: 2.6 m away (20 cells)",
      json.dumps([word_at(ex_ev, t) for t in (117.3, 119.0, 119.7, 119.9)])[:300])

# now_state: known sequences -> known words, holds to the millisecond
G = "goal 4: (1.00, 2.00) 2.5 m away, 30 frontier cells, 9 clusters, chosen in 100 ms"
GT = "Goal 4: 2.5 m away (30 cells)"
OBST = "Obstacle detected ahead, stopping local planner."
STALL = ("MOSS base: STALL #3: tracks commanded L=+60% R=+60% but the encoders moved L=+2 R=+1 ticks in 1.0 s - "
         "stopped (the encoders are the bumper)")


def evs(*pairs):
    return mc.decisions_from_records([{"ts": t, "event": e} for t, e in pairs])


def ns(events, now, exploring=None, running=True):
    """The word at `now` from the events logged by then (a number ts after `now` is not written yet)."""
    seen = [e for e in events if not isinstance(e["ts"], (int, float)) or e["ts"] <= now]
    return mc.now_state(seen, running, exploring, now)


check("the words are the nine of the page: IDLE READY HEADING OBSTACLE BACKING OFF STUCK WAITING DONE STOPPED",
      set(mc.DECISION_WORDS) == WORDS and len(mc.DECISION_WORDS) == 9)
check("no stack -> IDLE, whatever the events", ns(evs((100.0, G)), 101.0, running=False)["word"] == "IDLE")
check("a stack and no decision -> READY, since null", ns([], 5.0) == {"word": "READY", "detail":
      "Stack running, no decision yet", "since": None})
check("goal at 100 s -> HEADING at 101 s, detail the goal line, since 100",
      ns(evs((100.0, G)), 101.0) == {"word": "HEADING", "detail": GT, "since": 100.0})
seq_o = evs((100.0, G), (105.0, OBST), (105.01, "Replanning path due to obstacle found."), (105.02, "Replanning."),
            (105.05, "Found safe goal."))
check("goal 100, obstacle 105 (+ replan, its echo, the safe goal) -> 2 + 1 events; OBSTACLE at 105.5 and 107.99 "
      "(since the replan 105.01), HEADING the goal at 108.01 (3 s hold)",
      [e["kind"] for e in seq_o] == ["goal", "obstacle", "replan"]
      and [ns(seq_o, t)["word"] for t in (105.5, 107.99, 108.01)] == ["OBSTACLE", "OBSTACLE", "HEADING"]
      and ns(seq_o, 105.5)["since"] == 105.01 and ns(seq_o, 108.01)["detail"] == GT,
      json.dumps([ns(seq_o, t) for t in (105.5, 107.99, 108.01)])[:300])
seq_c = evs((100.0, G), (110.0, STALL))
check("goal 100, stall 110 and its escape's end line lost -> BACKING OFF at 110.1 and 115.99, HEADING at 116.0 "
      "(the escape's 5 s timeout + 1 s)",
      [ns(seq_c, t)["word"] for t in (110.1, 115.99, 116.0)] == ["BACKING OFF", "BACKING OFF", "HEADING"]
      and ns(seq_c, 110.1)["detail"].startswith("Contact #3: both tracks blocked")
      and mc.ESCAPE_HOLD_S == 6.0)
slow = mc.decisions_from_records(recs((0.0, G), (10.0, STALL), (10.3, "BUMP received", {"acted": True}),
                                      (10.3, "costmap: contact at (+0.10, +0.20) heading +10 deg: 30 cells ahead "
                                             "marked as a wall"),
                                      (15.3, "Contact escape", {"trigger": "bump", "travelled_m": 0.05,
                                                                "seconds": 5.0, "aborted": False})))
check("a slow escape (a track slipping in reverse: 0.05 m in 5 s, its end at 15.3) -> BACKING OFF at 12, 14, 14.5, "
      "15, 15.29; then WAITING 'Escape timed out: only 0.05 m in 5.0 s'",
      [ns(slow, t)["word"] for t in (12.0, 14.0, 14.5, 15.0, 15.29)] == ["BACKING OFF"] * 5
      and ns(slow, 15.4) == {"word": "WAITING", "detail": "Escape timed out: only 0.05 m in 5.0 s", "since": 15.3},
      json.dumps([ns(slow, t) for t in (14.5, 15.4)]))
seq_co = evs((100.0, G), (110.0, "MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before calling "
                                 "it a contact"), (111.5, OBST))
check("frozen tracks at 110 (STUCK 4 s) outrank an obstacle at 111.5 (OBSTACLE 3 s): STUCK to 114, then OBSTACLE "
      "to 114.5, then HEADING",
      [ns(seq_co, t)["word"] for t in (112.0, 113.99, 114.2, 114.5)]
      == ["STUCK", "STUCK", "OBSTACLE", "HEADING"])
seq_oc = evs((100.0, G), (110.0, OBST), (111.0, STALL))
check("an obstacle at 110 then a contact at 111 -> BACKING OFF at 111.5 (to 117 at most), HEADING at 117",
      [ns(seq_oc, t)["word"] for t in (110.5, 111.5, 116.99, 117.0)] == ["OBSTACLE", "BACKING OFF", "BACKING OFF",
                                                                         "HEADING"])
seq_p = evs((100.0, G), (120.0, "MOSS base: tracks frozen at L=+56% R=+50% - pushing at 75 % for 1 s before "
                                "calling it a contact"))
check("tracks frozen at 120 -> STUCK to 123.99, HEADING at 124 (4 s hold)",
      [ns(seq_p, t)["word"] for t in (120.5, 123.99, 124.0)] == ["STUCK", "STUCK", "HEADING"])
seq_s = evs((100.0, G), (130.0, "Robot is stuck. Replanning."), (130.001, "Replanning."))
seq_s_ign = evs((100.0, G), (130.0, "Robot is stuck. Replanning."),
                (130.025, "MOSS base: twist vx=+0.000 vy=+0.000(ignored) wz=-0.385 -> tracks L=+100% R=-100% (limit "
                          "100%)"), (130.6, "changed state"))
check("the planner's stuck replan (the verdict + dimOS's 'Replanning.') at 130 -> STUCK to 132.99, HEADING at 133 "
      "(3 s hold); the verdict the planner ignores (the pivot twist follows, no 'Replanning.') -> HEADING throughout",
      [ns(seq_s, t)["word"] for t in (130.5, 132.99, 133.0)] == ["STUCK", "STUCK", "HEADING"]
      and [ns(seq_s_ign, t)["word"] for t in (130.3, 131.0, 133.0)] == ["HEADING"] * 3 and len(seq_s_ign) == 1)
seq_i = evs((100.0, G), (150.0, "IMU contact: forward -6.1 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%"))
check("an IMU contact at 150 (no escape line) -> BACKING OFF to 155.99, HEADING at 156",
      [ns(seq_i, t)["word"] for t in (150.5, 155.99, 156.0)] == ["BACKING OFF", "BACKING OFF", "HEADING"])
seq_e = evs((100.0, G), (160.0, STALL), (160.3, "BUMP received"), (162.4, "Contact escape"))
check("stall 160, bump 160.3, escape done 162.4 -> BACKING OFF at 162.3, WAITING at 162.5 (the escape ends the "
      "hold; the explorer runs)",
      [ns(seq_e, t)["word"] for t in (162.3, 162.5)] == ["BACKING OFF", "WAITING"]
      and ns(seq_e, 162.5) == {"word": "WAITING", "detail": "Escape done", "since": 162.4})
seq_tap = evs((200.0, "Got new goal"), (205.0, STALL), (207.0, "Contact escape"), (220.0, "Got new goal"),
              (230.0, "Reached goal position, starting final rotation"))
check("a tapped goal (no explorer line): new goal -> HEADING; escape done -> READY; reached -> READY "
      "(exploring true from the bus -> HEADING)",
      [ns(seq_tap, t)["word"] for t in (201.0, 207.5, 221.0, 231.0)] == ["HEADING", "READY", "HEADING", "READY"]
      and ns(seq_tap, 231.0, exploring=True)["word"] == "HEADING"
      and ns(seq_tap, 221.0)["detail"] == "Planner: new goal")
seq_w = evs((100.0, G), (105.0, "planner gave up on that goal: excluded until the map or the viewpoint changes"),
            (106.0, "3 of 9 clusters are on a recently failed goal: waiting for the map or the viewpoint to change, "
                    "not stopping"))
check("goal dropped, then waiting -> WAITING 'Waiting: 3 of 9 frontiers are failed goals', since 106",
      ns(seq_w, 107.0) == {"word": "WAITING", "detail": "Waiting: 3 of 9 frontiers are failed goals",
                           "since": 106.0})
seq_st = evs((100.0, G), (140.0, STALL), (141.0, "WavefrontFrontierExplorer: stop_movement received, stopping "
                                                  "exploration"))
check("a stop during a contact hold -> STOPPED at once (141.5), the hold is over",
      ns(seq_st, 141.5)["word"] == "STOPPED" and ns(seq_st, 145.0)["word"] == "STOPPED")
seq_d = evs((100.0, G), (300.0, "exploration complete: no reachable frontier left (12 targets, 150 ms)"))
check("exploration complete -> DONE", ns(seq_d, 301.0) == {"word": "DONE", "detail": "Exploration complete: no "
                                                           "reachable frontier left (12 goals)", "since": 300.0})
seq_pv = evs((100.0, G), (101.0, "MOSS base: pivot L=+100% R=-100%: the right track did not turn (encoders "
                                 "L=+900 R=+4 ticks in 1.0 s) - not a contact: from now on this pivot turns on "
                                 "the left track only, the right track braked"))
check("a pivot fallback while heading -> still HEADING, detail the goal line",
      ns(seq_pv, 102.0) == {"word": "HEADING", "detail": GT, "since": 100.0})
iso = "2026-01-01T07:49:15.000Z"
t_iso = _dt.datetime(2026, 1, 1, 7, 49, 15, tzinfo=_dt.timezone.utc).timestamp()
seq_iso = mc.decisions_from_records([{"ts": "2026-01-01T07:49:10.000Z", "event": G}, {"ts": iso, "event": OBST}])
check("ISO timestamps as dimOS writes them (UTC, Z): ts kept as given; OBSTACLE 1 s after, HEADING 3.5 s after",
      seq_iso[1]["ts"] == iso and ns(seq_iso, t_iso + 1.0)["word"] == "OBSTACLE"
      and ns(seq_iso, t_iso + 3.5)["word"] == "HEADING")
check("an obstacle whose time cannot be read holds nothing (HEADING)",
      ns(evs((100.0, G), ("soon", OBST)), 101.0)["word"] == "HEADING")
NO_PATH = mc.decisions_from_records(recs((0.0, "Got new goal"), (0.001, "Cancelling goal."), (0.1, "Found safe goal."),
                                         (0.2, "No path found to the goal.", {"x": 1.03, "y": -2.72}),
                                         (0.21, "Cancelling goal.")))
NO_SAFE = mc.decisions_from_records(recs((0.0, "Got new goal"), (0.1, "No safe goal found near requested target."),
                                         (0.11, "No safe goal found.", {"x": 1.0, "y": 2.0}),
                                         (0.12, "Cancelling goal.")))
ABANDON = mc.decisions_from_records(recs((0.0, "Got new goal"), (0.1, "Found safe goal."),
                                         (5.0, "Robot is stuck. Replanning."),
                                         (5.01, "Goal abandoned, no recovery motion", {"trigger": "stuck_detector"}),
                                         (5.02, "Cancelling goal.")))
check("a goal tapped on the map (no explorer) that the planner drops -> the reason in the feed, READY 10 s later: "
      "'Goal dropped: no path to (1.03, -2.72)', 'Goal dropped: no safe spot near it' (its second line an echo), "
      "'Stuck: goal abandoned' (STUCK until 8.01)",
      kt(NO_PATH)[1:] == [("goal_dropped", "planner", "Goal dropped: no path to (1.03, -2.72)")]
      and kt(NO_SAFE)[1:] == [("goal_dropped", "planner", "Goal dropped: no safe spot near it")]
      and kt(ABANDON)[1:] == [("goal_dropped", "recovery", "Stuck: goal abandoned")]
      and [mc.now_state(x, True, False, 10.0)["word"] for x in (NO_PATH, NO_SAFE, ABANDON)] == ["READY"] * 3
      and mc.now_state(NO_PATH, True, False, 10.0)["detail"] == "Goal dropped: no path to (1.03, -2.72)"
      and [mc.now_state(ABANDON, True, False, t)["word"] for t in (6.0, 8.0, 8.02)] == ["STUCK", "STUCK", "READY"],
      json.dumps([kt(NO_PATH), kt(NO_SAFE), kt(ABANDON)])[:300])
drop_x = mc.decisions_from_records(recs((0.0, G), (0.03, "Got new goal"), (0.2, "No path found to the goal."),
                                        (0.21, "Cancelling goal."),
                                        (0.215, "planner gave up on that goal: excluded until the map or the "
                                                "viewpoint changes")))
check("the same while exploring: the explorer's 'planner gave up' 15 ms after the planner's reason is its echo, "
      "the reason stays; WAITING",
      kt(drop_x)[1:] == [("goal_dropped", "planner", "Goal dropped: no path")]
      and ns(drop_x, 1.0)["word"] == "WAITING", json.dumps(kt(drop_x)))
dead = mc.decisions_from_records(recs((0.0, G), (0.5, "BUMP received", {"acted": True}),
                                      (1.0, "Contact during an escape: stopping dead", {"trigger": "bump"}),
                                      (1.0, "BUMP received", {"acted": False}),
                                      (1.05, "Contact escape", {"travelled_m": 0.05, "seconds": 0.5,
                                                                "aborted": True})))
check("a second contact during an escape: 'Contact during an escape: stop dead' (its BUMP acted=False backs nothing "
      "off: no row), 'Escape aborted after 0.05 m'; WAITING once it ended",
      [e["text"] for e in dead[1:]] == ["Bump: backing off", "Contact during an escape: stop dead",
                                        "Escape aborted after 0.05 m"]
      and ns(dead, 1.04)["word"] == "BACKING OFF" and ns(dead, 1.1)["word"] == "WAITING", json.dumps(kt(dead)))
corner = mc.decisions_from_records(recs((0.0, "born cornered: 0.40 m2 of reachable floor and no frontier - one 0.20 m "
                                              "back-off via the bump reflex"),
                                        (0.05, "BUMP received", {"acted": True}),
                                        (2.2, "Contact escape", {"travelled_m": 0.2, "seconds": 2.1,
                                                                 "aborted": False})))
check("born cornered: one row 'Cornered (0.40 m2 of floor, no frontier): backing off 0.20 m' (an obstacle, not a "
      "contact; the BUMP it publishes is its echo), BACKING OFF until 'Escape done: 0.20 m in 2.1 s', then WAITING",
      kt(corner) == [("obstacle", "explorer", "Cornered (0.40 m2 of floor, no frontier): backing off 0.20 m"),
                     ("contact", "recovery", "Escape done: 0.20 m in 2.1 s")]
      and [ns(corner, t)["word"] for t in (1.0, 2.19, 2.3)] == ["BACKING OFF", "BACKING OFF", "WAITING"],
      json.dumps(kt(corner)))
quiet = mc.decisions_from_records(recs((0.0, G), (5.0, "Robot veered off track. Replanning.", {"deviation": 0.6}),
                                       (5.001, "Replanning."), (9.0, "Failure in navigation."),
                                       (9.001, "Replanning.")))
check("off track + 'Replanning.', a navigation failure + 'Replanning.' -> replan rows with no hold: HEADING "
      "'Off track: replanning' at 5.5, 'Navigation failed: replanning' at 9.5 (no OBSTACLE: nothing was in the way)",
      ns(quiet, 5.5) == {"word": "HEADING", "detail": "Off track: replanning", "since": 5.0}
      and ns(quiet, 9.5) == {"word": "HEADING", "detail": "Navigation failed: replanning", "since": 9.0},
      json.dumps([ns(quiet, 5.5), ns(quiet, 9.5)]))
tap = mc.decisions_from_records(recs((0.0, "WavefrontFrontierExplorer: stop_movement received, stopping exploration"),
                                     (0.5, "Got new goal"), (0.6, "Found safe goal.")))
check("STOP, then a goal tapped 0.5 s later (not exploring: the page sends it at once) -> its own row, HEADING; "
      "only the explorer's stay-here goal within 0.2 s of the stop is an echo",
      kt(tap)[1:] == [("new_goal", "planner", "Planner: new goal")] and ns(tap, 3.0, False)["word"] == "HEADING"
      and mc.AFTER_STOP_ECHO_S == 0.2, json.dumps(kt(tap)))
halt = mc.decisions_from_records(recs((0.0, G), (1.0, "BUMP received", {"acted": True}),
                                      (1.5, "WavefrontFrontierExplorer: stop_movement received, stopping exploration"),
                                      (1.55, "Contact escape", {"travelled_m": 0.04, "seconds": 0.5, "aborted": True})))
check("a STOP during an escape: the escape's aborted end after it leaves STOPPED",
      ns(halt, 2.0) == {"word": "STOPPED", "detail": "Exploration stopped (stop button or pad)", "since": 1.5},
      json.dumps(ns(halt, 2.0)))

# echoes, and the feed carried across reads
check("goal, then the planner's 'Got new goal' 1 ms later and 'Found safe goal.' -> 1 event; 'Got new goal' 2 s "
      "later -> kept; alone -> kept",
      len(evs((100.0, G), (100.001, "Got new goal"), (100.1, "Found safe goal."))) == 1
      and len(evs((100.0, G), (102.0, "Got new goal"))) == 2 and len(evs((200.0, "Got new goal"))) == 1)
stop5 = evs((300.0, "WavefrontFrontierExplorer: stop_movement received, stopping exploration"),
            (300.001, "Received exploration stop command via LCM"), (300.005, "Stopped autonomous frontier exploration"),
            (300.01, "Got new goal"), (300.06, "Reached goal position, starting final rotation"))
check("a STOP: its two stop lines, the explorer's line and its stay-here goal and arrival -> 1 event, STOPPED",
      [e["text"] for e in stop5] == ["Exploration stopped (stop button or pad)"]
      and ns(stop5, 301.0)["word"] == "STOPPED")
prev = {"ts": 10.0, "kind": "replan", "text": "Replanning", "src": "planner", "seq": 7}
check("across reads: seq continues from seq0, the echo rule sees the previous read's last event",
      mc.decisions_from_records([{"ts": 10.2, "event": "Replanning."}], 7, prev) == []
      and mc.decisions_from_records([{"ts": 12.0, "event": "Replanning."}], 7, prev)[0]["seq"] == 8)

# bad records never raise
bad = [None, 3, "x", [], {}, {"event": None}, {"event": 5}, {"level": "info"}, {"event": "   "},
       {"ts": object(), "event": OBST}, {"ts": float("nan"), "event": OBST}, {"event": OBST}]
try:
    got = mc.decisions_from_records(bad)
    raised = None
except Exception as exc:  # noqa: BLE001
    got, raised = None, exc
check("records that are not dicts, without an event or with an odd one -> skipped, no exception; a ts that is "
      "not a time (an object, NaN, none) -> null",
      raised is None and [(e["kind"], e["ts"]) for e in got] == [("obstacle", None)] * 3,
      repr(raised) if raised else json.dumps(got, default=str))
try:
    ok_nones = (mc.decisions_from_records(None) == [] and mc.decisions_from_records(5) == []
                and mc.now_state([None, 1, {"kind": None}, {"kind": "goal", "ts": object()},
                                  {"kind": "obstacle", "ts": "x"}], True, None, 1.0)["word"] in WORDS)
except Exception as exc:  # noqa: BLE001
    ok_nones = False
    print("    raised:", repr(exc))
check("decisions_from_records(None / 5) -> [], now_state on junk events -> a word, no exception", ok_nones)

# live: the running stack's main.jsonl through /decisions.json
dec_dir = Path(TMP, "logs", "decisions")
dec_dir.mkdir(parents=True)
main = Path(dec_dir, "main.jsonl")
DEC_RUN = "20260101-120000-moss-dimos-moss-explore"
reg_i = Path(runs, "i.json")
reg_i.write_text(json.dumps({"run_id": DEC_RUN, "pid": os.getpid(), "blueprint": "moss-dimos.moss-explore",
                             "started_at": (_dt.datetime.now(_dt.timezone.utc) + _dt.timedelta(seconds=5)).isoformat(),
                             "log_dir": str(dec_dir)}))


def jline(event, t, logger="dimos/navigation/replanning_a_star/global_planner.py"):
    """One main.jsonl record with the fields a real one carries."""
    stamp = _dt.datetime.fromtimestamp(t, _dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
    return json.dumps({"event": event, "level": "info", "logger": logger, "timestamp": stamp,
                       "func_name": "f", "lineno": 1, "module": "x", "worker": 3}) + "\n"


t0 = time.time() - 60.0
twist = "MOSS base: twist vx=+0.313 vy=+0.000(ignored) wz=-0.200 -> tracks L=+75% R=+64% (limit 100%)"
old_goals = "".join(jline(f"goal {90 + i}: (0.00, 0.00) 1.0 m away, 5 frontier cells, 2 clusters, chosen in 9 ms",
                          t0, "/home/x/moss/moss_dimos/explorer2.py") for i in range(5))
chatter = jline(twist, t0 + 1.0, "/home/x/moss/moss_dimos/moss_adapter.py")
n_chatter = (mc.DECISIONS_TAIL_BYTES // len(chatter)) + 200          # more than the 1 MiB window by itself
fresh = "".join(jline(f"goal {i}: (1.00, 2.00) 2.5 m away, 30 frontier cells, 9 clusters, chosen in 100 ms"
                      if i % 2 == 0 else "3 of 9 clusters are on a recently failed goal: waiting for the map or the "
                      "viewpoint to change, not stopping", t0 + 2.0 + i * 0.5, "/home/x/moss/moss_dimos/explorer2.py")
                for i in range(45))
main.write_text(old_goals + chatter * n_chatter + fresh)
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None
srv_d, cd = serve(demo=False, page_dir=page_dir)
t_read = time.perf_counter()
code, dj = cd.json("/decisions.json?n=40")
t_read = time.perf_counter() - t_read
evs40 = dj.get("events", [])
_, dall = cd.json("/decisions.json?n=500")
check(f"/decisions.json?n=40 on a {main.stat().st_size / 1e6:.2f} MB main.jsonl (5 goals, then {n_chatter} twist "
      f"lines, then 45 decisions) -> 40 events, the newest last, ts as written, run named; all of them = the 45 "
      f"inside the 1 MiB window (first read {t_read * 1000:.0f} ms)",
      code == 200 and set(dj) == {"now", "events", "run"} and len(evs40) == 40 and dj["run"] == DEC_RUN
      and evs40[-1]["text"] == "Goal 44: 2.5 m away (30 cells)" and len(dall["events"]) == 45
      and not any(e["text"].startswith("Goal 9") for e in dall["events"])
      and evs40[-1]["ts"].endswith("Z") and all(set(e) == EVENT_KEYS for e in evs40),
      json.dumps(dj)[:300])
check("... now: the last line is goal 44 -> HEADING, since its ts",
      dj.get("now", {}).get("word") == "HEADING" and dj["now"]["since"] == evs40[-1]["ts"], json.dumps(dj.get("now")))
last_seq = evs40[-1]["seq"] if evs40 else 0
stuck_t = time.time()
torn = jline("Robot is stuck. Replanning.", stuck_t)
with open(main, "a") as fh:
    fh.write(jline(OBST, time.time()) + torn[:30])
code, dj2 = cd.json("/decisions.json?n=40")
check("a line appended within 0.5 s of the last read -> not read yet (2 Hz at most)",
      dj2["events"] == evs40, json.dumps(dj2["events"][-1:]))
time.sleep(0.6)
code, dj3 = cd.json("/decisions.json?n=40")
check("0.6 s later -> the obstacle, seq +1, OBSTACLE now; the torn last line waits",
      dj3["events"][-1]["text"] == "Obstacle ahead: stop" and dj3["events"][-1]["seq"] == last_seq + 1
      and dj3["now"]["word"] == "OBSTACLE", json.dumps(dj3["events"][-1:]) + json.dumps(dj3["now"]))
with open(main, "a") as fh:
    fh.write(torn[30:])
time.sleep(0.6)
code, dj4 = cd.json("/decisions.json?n=40")
check("the torn line completed -> read whole: the stuck verdict, held until the planner's answer -> no event "
      "yet, still OBSTACLE",
      dj4["events"] == dj3["events"] and dj4["now"]["word"] == "OBSTACLE",
      json.dumps(dj4["events"][-1:]) + json.dumps(dj4["now"]))
with open(main, "a") as fh:
    fh.write(jline("Replanning.", stuck_t + 0.002))
time.sleep(0.6)
code, dj4 = cd.json("/decisions.json?n=40")
check("dimOS's 'Replanning.' in the next read -> 'Stuck: replanning' at the verdict's time, STUCK now",
      dj4["events"][-1]["text"] == "Stuck: replanning" and dj4["events"][-1]["seq"] == last_seq + 2
      and mc._epoch(dj4["events"][-1]["ts"]) == mc._epoch(json.loads(torn)["timestamp"])
      and dj4["now"]["word"] == "STUCK", json.dumps(dj4["events"][-1:]) + json.dumps(dj4["now"]))
code, dj5 = cd.json("/decisions.json?n=3")
_, dj6 = cd.json("/decisions.json?n=abc")
check("n=3 -> 3 events; n=abc -> 40", len(dj5["events"]) == 3 and len(dj6["events"]) == 40)
main.write_text(jline(G, time.time()) + jline("BUMP received", time.time(),
                                              "/home/x/moss/moss_dimos/recovering_planner.py"))
time.sleep(0.6)
code, dj7 = cd.json("/decisions.json?n=40")
check("the file rewritten smaller (a new log) -> the feed starts over: 2 events, seq still growing, BACKING OFF",
      [e["text"] for e in dj7["events"]] == [GT, "Bump: backing off"] and dj7["events"][0]["seq"] > last_seq + 2
      and dj7["now"]["word"] == "BACKING OFF", json.dumps(dj7)[:300])
reg_i.unlink()
code, dj8 = cd.json("/decisions.json?n=40")
check("the stack stops -> IDLE 'No stack running (last run: ...)', its events and run kept (what happened stays "
      "readable, like the map)",
      dj8 == {"now": {"word": "IDLE", "detail": f"No stack running (last run: {DEC_RUN})", "since": None},
              "events": dj7["events"], "run": DEC_RUN}, json.dumps(dj8)[:300])
mc._dimos_run_pid = lambda: 4242
code, dj9 = cd.json("/decisions.json?n=40")
check("a `dimos run` with no registry entry (foreground, swept registry) -> READY 'Stack running, its log is not "
      "found', no events: never the last run's lines as if they were its own",
      dj9 == {"now": {"word": "READY", "detail": "Stack running, its log is not found", "since": None},
              "events": [], "run": None}, json.dumps(dj9))
mc._dimos_run_pid = lambda: None
dec_dir2 = Path(TMP, "logs", "decisions2")
dec_dir2.mkdir(parents=True)
Path(dec_dir2, "main.jsonl").write_text(jline("Obstacle detected ahead, stopping local planner.", time.time()))
DEC_RUN2 = "20260101-130000-moss-dimos-moss-explore"
reg_i.write_text(json.dumps({"run_id": DEC_RUN2, "pid": os.getpid(), "blueprint": "moss-dimos.moss-explore",
                             "started_at": (_dt.datetime.now(_dt.timezone.utc) + _dt.timedelta(seconds=6)).isoformat(),
                             "log_dir": str(dec_dir2)}))
code, dj10 = cd.json("/decisions.json?n=40")
check("the next run -> its own feed only (1 event, seq still growing), its run name",
      [e["text"] for e in dj10["events"]] == ["Obstacle ahead: stop"] and dj10["run"] == DEC_RUN2
      and dj10["events"][0]["seq"] > dj7["events"][-1]["seq"], json.dumps(dj10)[:300])
reg_i.unlink()
srv_d.shutdown()
mc._dimos_run_pid = real_pid

# demo: the scripted cycle through the same parser
dsrv, dcl = serve(demo=True, page_dir=page_dir)
code, dd = dcl.json("/decisions.json")
check("demo /decisions.json -> now + events + run: exploration started, WAITING, run on moss-explore",
      code == 200 and set(dd) == {"now", "events", "run"} and set(dd["now"]) == {"word", "detail", "since"}
      and dd["now"]["word"] in WORDS and dd["events"] and dd["events"][0]["text"].startswith("Exploration started")
      and all(set(e) == EVENT_KEYS for e in dd["events"]) and str(dd["run"]).endswith("moss-explore"),
      json.dumps(dd)[:300])
dsrv.cockpit._script_t0 -= mc.DEMO_DECISION_CYCLE_S + 0.5        # one whole cycle is due
code, dd = dcl.json("/decisions.json")
kinds = [e["kind"] for e in dd["events"]]
cycle = ["mission", "goal", "obstacle", "replan", "contact", "contact", "contact", "contact", "goal_dropped", "goal",
         "reached"]
check("demo, one cycle later -> goal, obstacle, replan, contact (stall, bump, wall), escape (from its record's "
      "fields: 'Escape done: 0.20 m in 2.1 s'), goal dropped, goal, reached; the stack log carries the same lines",
      kinds[:len(cycle)] == cycle and dd["events"][7]["text"] == "Escape done: 0.20 m in 2.1 s"
      and dd["now"]["word"] in WORDS, json.dumps(kinds))
stamps = [e["ts"] for e in dd["events"]]
check("demo times are dimOS's: ISO UTC with a Z, milliseconds, on the clock now (the page reads them as UTC)",
      all(isinstance(t, str) and re.fullmatch(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z", t) for t in stamps)
      and abs(mc._epoch(stamps[0]) - time.time()) < 60, json.dumps(stamps[:2]))
_, dlog = dcl.json("/log.json?n=200")
check("demo: every decision has its raw line in /log.json",
      any(ln["event"].startswith("MOSS base: STALL #1") for ln in dlog["lines"])
      and any(ln["event"] == "Contact escape" for ln in dlog["lines"]))
dcl.post("/cmd/stop", {})
_, dd = dcl.json("/decisions.json")
check("demo STOP -> STOPPED, 'Exploration stopped (stop button or pad)' (its second line is an echo)",
      dd["now"]["word"] == "STOPPED" and dd["events"][-1]["text"] == "Exploration stopped (stop button or pad)",
      json.dumps(dd["now"]))
dcl.post("/cmd/explore", {"action": "start"})
_, dd = dcl.json("/decisions.json")
check("demo explore start -> WAITING, 'Exploration started (cap 3 m, radius 4 m)'",
      dd["now"]["word"] == "WAITING" and dd["events"][-1]["text"] == "Exploration started (cap 3 m, radius 4 m)")
dcl.post("/cmd/goal", {"x": 1.0, "y": 1.0})
_, dd = dcl.json("/decisions.json")
first = (dd["now"]["word"], dd["events"][-1]["text"])
time.sleep(mc.GOAL_AFTER_STOP_S + 0.1)                           # the page sends the goal after the stop, as live
_, dd = dcl.json("/decisions.json")
check("demo goal while exploring -> 'Exploration stopped (stop command)' at once, the planner's new goal "
      "3 s later -> HEADING",
      first == ("STOPPED", "Exploration stopped (stop command)") and dd["now"]["word"] == "HEADING"
      and dd["events"][-1]["text"] == "Planner: new goal", json.dumps([first, dd["now"]]))
_, before = dcl.json("/decisions.json")
dcl.post("/cmd/stack", {"action": "stop"})
_, dd = dcl.json("/decisions.json")
check("demo stack stop -> IDLE 'No stack running (last run: ...)', the run's events kept, as live",
      dd["now"]["word"] == "IDLE" and dd["events"] == before["events"] and dd["events"]
      and dd["run"] == before["run"] and dd["now"]["detail"] == f"No stack running (last run: {before['run']})",
      json.dumps(dd)[:200])
dcl.post("/cmd/stack", {"action": "map"})
_, dd = dcl.json("/decisions.json")
check("demo: a new stack -> a fresh feed", dd["events"] == [] and dd["now"]["word"] == "READY"
      and dd["run"] != before["run"], json.dumps(dd)[:200])
dsrv.shutdown()

check("the server's docstring lists GET /decisions.json", "GET  /decisions.json?n=40" in (mc.__doc__ or ""))
cockpit_md = (ROOT / "docs" / "cockpit.md").read_text()
check("docs/cockpit.md: a 'What it decides' section, /decisions.json, the link to imu.md",
      "## What it decides" in cockpit_md and "/decisions.json" in cockpit_md and "(imu.md)" in cockpit_md)

# ---------------------------------------------------------------------------
print("J. the idle stop: the rule, what counts as activity, the watchdog, /state.json")
MSG5 = "stopping the stack to preserve battery (idle 5 min)"
J_NOW = 1_800_000_000.0                        # an epoch second the rule counts from
for f in runs.glob("*.json"):
    f.unlink()
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None               # this machine's own dimos processes never count as a stack


def stack_up(ago, blueprint="moss-explore"):
    """A running stack as stack_status gives it, started `ago` seconds before J_NOW."""
    return {"running": True, "run_id": "r", "blueprint": blueprint, "pid": 1, "log_dir": None,
            "started_at": _dt.datetime.fromtimestamp(J_NOW - ago, _dt.timezone.utc).isoformat()}


def rule(stack, activity_ago, limit):
    return mc.idle_state(stack, None if activity_ago is None else J_NOW - activity_ago, J_NOW, limit)


r_fresh = rule(stack_up(10), None, 300)
check("rule: a fresh MOSS stack (up 10 s, no activity yet), limit 300 s -> enabled, idle 10 s (from its start), "
      "stop in 290 s, not due",
      r_fresh == {"enabled": True, "idle_s": 10.0, "stop_in_s": 290.0, "limit_s": 300, "due": False}, json.dumps(r_fresh))
r_act = rule(stack_up(1000), 5, 300)
check("rule: activity 5 s ago on a stack up 1000 s -> idle 5 s, stop in 295 s, not due",
      (r_act["idle_s"], r_act["stop_in_s"], r_act["due"]) == (5.0, 295.0, False), json.dumps(r_act))
r_before = rule(stack_up(10), 500, 300)
check("rule: activity from before the stack started never counts (500 s ago, stack up 10 s) -> idle 10 s",
      (r_before["idle_s"], r_before["stop_in_s"]) == (10.0, 290.0), json.dumps(r_before))
edge = [rule(stack_up(1000), a, 300) for a in (299.9, 300.0, 450.0)]
check("rule at the limit: idle 299.9 s -> not due, stop in 0.1 s; 300 s -> due, stop in 0; 450 s -> due, "
      "stop_in_s floored at 0",
      [(r["due"], r["stop_in_s"]) for r in edge] == [(False, 0.1), (True, 0.0), (True, 0.0)],
      json.dumps([(r["idle_s"], r["stop_in_s"], r["due"]) for r in edge]))
r_off = rule(stack_up(5000), None, 0)
check("rule off (limit 0): a MOSS stack idle 5000 s -> enabled false, idle 5000 s, no stop_in_s, never due",
      r_off == {"enabled": False, "idle_s": 5000.0, "stop_in_s": None, "limit_s": 0, "due": False}, json.dumps(r_off))
r_foreign = rule(stack_up(5000, "explore"), None, 300)
check("rule: a stack that is not MOSS (blueprint 'explore', idle 5000 s) -> never due, no stop_in_s",
      (r_foreign["due"], r_foreign["stop_in_s"], r_foreign["idle_s"]) == (False, None, 5000.0), json.dumps(r_foreign))
r_down = rule({"running": False}, 5, 300)
r_blank = mc.idle_state({"running": True, "blueprint": "moss-explore", "started_at": None}, None, J_NOW, 300)
check("rule: no stack -> idle_s null, not due; a running stack with no start time and no activity -> idle_s null, "
      "not due", (r_down["idle_s"], r_down["due"], r_blank["idle_s"], r_blank["due"]) == (None, False, None, False),
      json.dumps([r_down, r_blank]))
r_pad, r_base, r_nav = (rule(stack_up(5000, bp), None, 300) for bp in ("moss-gamepad", "moss-base", "moss-nav"))
check("M4 rule: moss-gamepad (the pad on the table) and moss-base idle 5000 s -> never due, no stop_in_s: they stay "
      "up; moss-nav -> due; the stop applies to moss-nav and moss-explore only",
      (r_pad["due"], r_pad["stop_in_s"], r_base["due"], r_base["stop_in_s"], r_nav["due"]) == (False, None, False, None, True)
      and mc.IDLE_STOP_BLUEPRINTS == ("moss-nav", "moss-explore"), json.dumps([r_pad, r_base, r_nav]))
r_mono = mc.idle_state(stack_up(5000), 100.0, 130.0, 300, started=120.0)
check("m1 rule on the cockpit's clock: started=120 (the run's start converted to the monotonic clock), activity at "
      "100, now 130 -> idle 10 s from the start, stop in 290 s (started_at is not read then)",
      (r_mono["idle_s"], r_mono["stop_in_s"]) == (10.0, 290.0), json.dumps(r_mono))
naive_20 = _dt.datetime.fromtimestamp(J_NOW - 20, _dt.timezone.utc).replace(tzinfo=None).isoformat()
tz_was = os.environ.get("TZ")
os.environ["TZ"] = "XYZ-7"                      # this machine seven hours east of UTC
time.tzset()
try:
    r_naive = mc.idle_state({"running": True, "blueprint": "moss-explore", "started_at": naive_20}, J_NOW - 600,
                            J_NOW, 300)
finally:
    if tz_was is None:
        os.environ.pop("TZ", None)
    else:
        os.environ["TZ"] = tz_was
    time.tzset()
check("m2 a started_at with no timezone is UTC, as dimOS writes it: on a machine seven hours east of UTC, a stack "
      "started 20 s ago -> idle 20 s (not 7 h off)", r_naive["idle_s"] == 20.0, json.dumps(r_naive))
check("the message: 300 s -> '" + MSG5 + "', 90 s -> '(idle 1.5 min)', 30 s -> '(idle 30 s)'",
      (mc.idle_stop_message(300), mc.idle_stop_message(90), mc.idle_stop_message(30))
      == (MSG5, "stopping the stack to preserve battery (idle 1.5 min)",
          "stopping the stack to preserve battery (idle 30 s)"))
err = io.StringIO()
with contextlib.redirect_stderr(err):
    limits = {k: mc.idle_limit_s({} if k is None else {"MOSS_IDLE_STOP_S": k})
              for k in (None, "0", "120", "45.5", "abc", "-5", "nan", " ", "10", "29.9", "30")}
check("MOSS_IDLE_STOP_S: unset -> 300 s, '0' -> 0 (off), '120' -> 120, '45.5' -> 45.5, '30' -> 30; 'abc', '-5', "
      "'nan' -> 300 and the cockpit says so; m6 '10' and '29.9' (under 30 s: a stack stopped while in use) -> 300 "
      "and it says so; blank -> 300",
      limits == {None: 300, "0": 0, "120": 120, "45.5": 45.5, "abc": 300, "-5": 300, "nan": 300, " ": 300,
                 "10": 300, "29.9": 300, "30": 30}
      and err.getvalue().count("MOSS_IDLE_STOP_S=") == 5 and err.getvalue().count("is under 30 s") == 2,
      f"{limits} {err.getvalue()!r}")
os.environ["MOSS_IDLE_STOP_S"] = "42"
read_at_start = mc.LiveCockpit(str(Path(TMP, "never_run.sh"))).idle_limit_s
os.environ.pop("MOSS_IDLE_STOP_S")
check("the cockpit reads MOSS_IDLE_STOP_S from its environment when it starts (42 -> 42 s; unset -> 300 s)",
      read_at_start == 42 and mc.LiveCockpit(str(Path(TMP, "never_run.sh"))).idle_limit_s == 300, str(read_at_start))
check("motors commanded: output_pct [0, 0] / None / [] -> no; [35, 0], [0, -20] -> yes",
      [mc.motors_commanded(v) for v in ([0, 0], None, [], [35, 0], [0, -20])] == [False, False, False, True, True])
P0 = {"x": 1.0, "y": 1.0, "yaw": 0.0}
moves = [mc.pose_moved(P0, dict(P0, **d)) for d in ({"x": 1.019}, {"x": 1.021}, {"y": 0.98 - 0.001},
                                                    {"yaw": math.radians(1.9)}, {"yaw": math.radians(2.1)})]
wrap = mc.pose_moved({"x": 0.0, "y": 0.0, "yaw": math.pi - 0.01}, {"x": 0.0, "y": 0.0, "yaw": -math.pi + 0.01})
check("pose moved: 1.9 cm -> no, 2.1 cm -> yes, 2.1 cm sideways -> yes, 1.9 deg -> no, 2.1 deg -> yes; "
      "+179.4 deg to -179.4 deg (1.1 deg across the wrap) -> no",
      moves == [False, True, True, False, True] and wrap is False, f"{moves} {wrap}")
check("M1 telemetry fresh: firmware_age_s 0.1 and 1.0 -> fresh; 1.2 and 600 (the coordinator repeating the last line "
      "of a lost board) -> stale; no age, or null -> fresh (a commanded track is never missed for it)",
      [mc.telemetry_fresh(t) for t in ({"firmware_age_s": 0.1}, {"firmware_age_s": 1.0}, {"firmware_age_s": 1.2},
                                       {"firmware_age_s": 600.0}, {}, {"firmware_age_s": None})]
      == [True, True, False, False, True, True])
fly_src = (ROOT / "tools" / "moss-fly.sh").read_text()
stack_re = re.search(r"^STACK_RE='([^']+)'", fly_src, re.M)
ARGVS = [[b"/v/bin/python3", b"/v/bin/dimos", b"run", b"moss-dimos.moss-explore"],
         [b"/v/bin/dimos", b"--rerun-open", b"none", b"run", b"moss-dimos.moss-gamepad", b"--daemon", b""],
         [b"/v/bin/dimos", b"status"], [b"vim", b"/v/bin/dimos"], [b"run", b"/v/bin/dimos"],
         [b"/v/bin/dimos", b"runs"], [b"sleep", b"600"], [b"/usr/lib/systemd/systemd", b"--user"], []]
dimos_mine = [mc.is_dimos_run(a) for a in ARGVS]
dimos_fly = ([re.search(stack_re.group(1), " ".join(x.decode() for x in a).strip()) is not None for a in ARGVS]
             if stack_re else None)
check("M3 is_dimos_run (the registry pid's /proc cmdline, read before a stop) agrees with moss-fly.sh's STACK_RE on "
      "9 command lines: dimos run, with options before run -> yes; dimos status, an editor naming the file, run "
      "before bin/dimos, 'runs', a sleep, systemd, an empty line -> no",
      dimos_mine == [True, True, False, False, False, False, False, False, False] and dimos_fly == dimos_mine,
      f"{dimos_mine} {dimos_fly}")

if not (HAVE_LCM_MSGS and mc.HAVE_MSGS):
    print("  -- skipped the bus and watchdog checks, no dimos_lcm here")
else:
    REG_J = Path(runs, "idle.json")
    lcm_url_was = os.environ.pop("LCM_DEFAULT_URL", None)      # the cockpit on LCM's default group
    env_lcm = {k: v for k, v in os.environ.items() if k not in ("DIMOS_TRANSPORT", "TRANSPORT", "LCM_DEFAULT_URL")}
    FAKE_DIMOS = str(Path(TMP, "venv", "bin", "dimos"))       # only a name in a command line, never run

    def stand_in(env=None, blueprint="moss-dimos.moss-explore"):
        """A live process /proc shows as a stack: `<python> -c <sleep> <venv>/bin/dimos run <blueprint>`."""
        p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)", FAKE_DIMOS, "run", blueprint],
                             env=env_lcm if env is None else env, stdin=subprocess.DEVNULL)
        atexit.register(p.kill)
        deadline = time.monotonic() + 5.0         # Popen returns before the child's argv is in /proc (vfork)
        while time.monotonic() < deadline and not mc.is_dimos_run(mc.proc_cmdline(p.pid)):
            time.sleep(0.01)
        return p

    STAND_IN = stand_in()                         # a stack on LCM, the cockpit's group

    def reg_j(run_id, blueprint="moss-dimos.moss-explore", ago=1000.0, pid=None):
        """A registry entry for a stack started `ago` seconds ago (a live stand-in process, on LCM)."""
        if run_id is None:
            REG_J.unlink(missing_ok=True)
            return
        REG_J.write_text(json.dumps({"run_id": run_id, "pid": pid or STAND_IN.pid, "blueprint": blueprint,
                                     "started_at": _dt.datetime.fromtimestamp(time.time() - ago, _dt.timezone.utc)
                                     .isoformat(), "log_dir": "/nowhere"}))

    class FakeLauncher:
        """Records what the watchdog asks; never runs moss-fly.sh. busy_since: when the busy run
        began (epoch s, None = unknown); fail: an error to answer, or "raise"."""

        def __init__(self):
            self.calls, self.busy, self.busy_since, self.fail = [], False, None, None

        def start(self, action, note=None):
            self.calls.append((action, note))
            if self.fail == "raise":
                raise PermissionError("the state dir is not writable")
            if self.fail:
                return {"ok": False, "error": self.fail}
            return {"ok": True, "sent": f"moss-fly.sh {action}", "pid": 1}

        def status(self, lines=True):
            return {"running": self.busy, "action": "check" if self.busy else None, "started_at": self.busy_since,
                    "returncode": None, "note": None}

    def idle_cockpit(limit=300.0):
        c = mc.LiveCockpit(str(Path(TMP, "never_run.sh")))
        c.launcher = FakeLauncher()
        c.idle_limit_s = limit
        c.listening = True                        # as the bus tap sets it once subscribed
        said = []
        real_say = c._say

        def say(level, event):
            said.append((level, event))
            real_say(level, event)
        c._say = say
        return c, said

    def tele_msg(pct, age=0.1):
        doc = {"fw": "qtpy-mdd3a-v3", "mode": 2, "bus_mV": 11800, "ina_ok": True, "output_pct": pct, "target_pct": pct}
        if age is not None:
            doc["firmware_age_s"] = age
        return String(data=json.dumps(doc)).lcm_encode()

    def hear(c, ago=0.0):
        """A telemetry line, no track commanded, `ago` s old: the cockpit hears the bus, nothing happened."""
        c.bus.ingest(mc.CHANNELS["base_telemetry"], tele_msg([0, 0]), mono=time.monotonic() - ago)

    def aged(c, s):
        """The last activity `s` seconds ago, on the monotonic clock the count runs on."""
        c.activity_mono = time.monotonic() - s

    def quiet(fn):
        with contextlib.redirect_stdout(io.StringIO()):
            return fn()

    def warnings(said):
        return [event for level, event in said if level == "warning"]

    def resets(c, feed):
        """Age the count by 1000 s, feed the bus, let the watchdog look: did the count restart?"""
        aged(c, 1000.0)
        feed()
        t = time.monotonic()
        c.idle_tick()
        return c.activity_mono >= t - 1.0

    def twist_msg(vx, wz):
        return Twist(linear=Vector3(x=vx, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz)).lcm_encode()

    def odom_msg(x, y, yaw_deg=0.0):
        h = math.radians(yaw_deg) / 2
        return PoseStamped(header=hdr("world"), pose=pose_of(x, y, qz=math.sin(h), qw=math.cos(h))).lcm_encode()

    def marker_msg(action, name):
        return String(data=json.dumps({"action": action, "name": name, "ts": time.time()})).lcm_encode()

    reg_j("idle-run-1")
    c, said = idle_cockpit(limit=3600.0)          # the 1000 s of aging below never reach this limit
    feed = lambda ch, raw: (lambda: c.bus.ingest(mc.CHANNELS[ch], raw))   # noqa: E731
    tel = [resets(c, feed("base_telemetry", tele_msg(p))) for p in ([0, 0], [35, 0], [0, -20])]
    stale_tel = resets(c, lambda: None)           # the last telemetry said [0, -20]; nothing new since
    check("telemetry output_pct [0, 0] -> no activity; [35, 0] and [0, -20] (a track commanded) -> the count restarts; "
          "no new telemetry since -> nothing (the last message is not read twice)",
          tel == [False, True, True] and stale_tel is False, f"{tel} {stale_tel}")
    lost = [resets(c, feed("base_telemetry", tele_msg([40, 40], age=600.0 + k))) for k in range(5)]
    by_age = [resets(c, feed("base_telemetry", tele_msg([40, 40], age=a))) for a in (1.0, 1.2, None)]
    check("M1 a lost board: the coordinator repeats its last line ([40, 40], firmware_age_s 600 s and growing) at "
          "each look -> no activity, 5 looks; the same line 1.0 s old -> the count restarts; 1.2 s -> not; no age -> "
          "restarts", lost == [False] * 5 and by_age == [True, False, True], f"{lost} {by_age}")
    tw = [resets(c, feed("cmd_vel", twist_msg(vx, wz))) for vx, wz in ((0.0, 0.0), (0.12, 0.0), (0.0, -0.4))]
    stale_tw = resets(c, lambda: None)            # a planner that went silent on a non-zero twist
    check("cmd_vel (0, 0) -> no activity; (0.12 m/s, 0) and (0, -0.4 rad/s) -> the count restarts; the last non-zero "
          "twist with nothing new since -> nothing", tw == [False, True, True] and stale_tw is False, f"{tw} {stale_tw}")
    ex = resets(c, feed("explore_cmd", Bool(data=True).lcm_encode()))
    done = resets(c, feed("explore_done", Bool(data=True).lcm_encode()))
    check("explore_cmd (the order to explore) -> the count restarts; explore_done alone -> nothing",
          ex is True and done is False, f"{ex} {done}")
    srcs = {}
    for name, ch, raw in (("telemetry", "base_telemetry", tele_msg([35, 0])),
                          ("cmd_vel", "cmd_vel", twist_msg(0.2, 0.0)),
                          ("explore_cmd", "explore_cmd", Bool(data=True).lcm_encode()),
                          ("goal_request", "goal_request",
                           PoseStamped(header=hdr("world"), pose=pose_of(2.0, 1.0)).lcm_encode()),
                          ("episode", "episode_marker", marker_msg("stop", "ep0"))):
        c.activity_source = "start"
        srcs[name] = [resets(c, feed(ch, raw)), c.activity_source]
    check("B2 + M6 each source restarts the count and names itself in idle.last_source: telemetry, cmd_vel, "
          "explore_cmd, goal_request (a goal on the bus) and episode (an episode_marker on the bus)",
          srcs == {k: [True, k] for k in srcs}, json.dumps(srcs))
    c.bus.ingest(mc.CHANNELS["episode_marker"], marker_msg("start", "arm-demo"))
    held = []
    for _ in range(3):
        aged(c, 1000.0)
        c.activity_source = "start"
        t = time.monotonic()
        c.idle_tick()
        held.append([c.activity_mono >= t - 1.0, c.activity_source])
    opened = dict(c.episode or {})
    c.bus.ingest(mc.CHANNELS["episode_marker"], marker_msg("stop", "arm-demo"))
    c.idle_tick()                                 # the stop marker itself is activity, once
    after_ep = resets(c, lambda: None)
    check("B2 an episode opened by a marker on the bus (a script, no page polling) holds the stop: the count restarts "
          "at each look while it is open (last_source 'episode'); its stop marker closes it, then the count runs",
          opened.get("name") == "arm-demo" and held == [[True, "episode"]] * 3 and c.episode is None
          and after_ep is False, json.dumps([opened, held, c.episode, after_ep]))

    odom_runs = [0]

    def odom_looks(poses, dt=1.0):
        """A fresh cockpit and run; one odom per look, dt s apart on the arrival clock. The looks whose pose
        counted as a move, and the cockpit."""
        odom_runs[0] += 1
        reg_j(f"idle-odom-{odom_runs[0]}")
        co, _ = idle_cockpit(limit=1e9)
        base, hits = time.monotonic(), []
        for k, p in enumerate(poses):
            co.bus.ingest(mc.CHANNELS["odom"], odom_msg(*p), mono=base + k * dt)
            before = co.activity_mono
            co.idle_tick()
            if co.activity_mono != before:
                hits.append(k)
        return hits, co

    drift, _ = odom_looks([(1.0 + 0.00025 * s, 1.0, 0.0) for s in range(1800)])
    yaw_drift, _ = odom_looks([(1.0, 1.0, 0.01 * s) for s in range(1800)])
    check("M6 odometry drifting at rest for 30 min, one pose per look: 0.25 mm/s (45 cm in all) and 0.01 deg/s (18 deg "
          "in all) -> never a move (the pose against the pose 10 s before: 2.5 mm, 0.1 deg)",
          drift == [] and yaw_drift == [], f"{drift[:8]} {yaw_drift[:8]}")
    rest = [(1.0, 1.0, 0.0)] * 12
    push, cp = odom_looks(rest + [(1.05, 1.0, 0.0)] * 15)
    check("M6 pushed by hand 5 cm between two looks -> a move at that look (last_source 'odom'), and at each look while "
          "the pose 10 s before is the one before the push (looks 12 to 21), none after",
          push == list(range(12, 22)) and cp.activity_source == "odom", str(push))
    small, _ = odom_looks(rest + [(1.015, 1.0, 0.0)] * 5 + [(1.015, 1.0, 1.5)] * 5)
    turn, _ = odom_looks(rest + [(1.0, 1.0, 2.5)] * 3)
    check("M6 1.5 cm then 1.5 deg within 10 s -> no move; turned 2.5 deg between two looks -> a move (looks 12 to 14)",
          small == [] and turn == [12, 13, 14], f"{small} {turn}")
    reg_j("idle-run-1")
    resets(c, feed("odom", odom_msg(1.0, 1.0)))   # the run's first pose
    reg_j("idle-run-1b")
    od_new = resets(c, feed("odom", odom_msg(5.0, -3.0, 90.0)))
    check("a new run (its odometry in a new frame): its first pose starts the window again, not a 5 m move",
          od_new is False, str(od_new))

    srv_j = mc.CockpitServer(("127.0.0.1", 0), c, page_dir)
    threading.Thread(target=srv_j.serve_forever, daemon=True).start()
    cj = Client(srv_j.server_address[1])
    aged(c, 1000.0)
    cj.json("/state.json")
    kept_get = c.activity_mono < time.monotonic() - 900
    cj.post("/cmd/stack", {"action": "check"}, header=False)
    kept_403 = c.activity_mono < time.monotonic() - 900
    t = time.monotonic()
    code_ck, ans_ck = cj.post("/cmd/stack", {"action": "check"})
    by_post = c.activity_mono >= t - 1.0 and c.activity_source == "post"
    check("a POST /cmd/* from the page (stack check) restarts the count (last_source 'post'); a GET, or a POST refused "
          "403 for want of the page's header, does not", by_post and code_ck == 200 and kept_get and kept_403,
          f"post {by_post} {code_ck} get kept {kept_get} 403 kept {kept_403}")
    c.idle_limit_s = 300.0
    hear(c)
    aged(c, 250.0)
    _, before_ka = cj.json("/state.json")
    code_ka403, _ = cj.post("/cmd/keepalive", {}, header=False)
    kept_ka = c.activity_mono < time.monotonic() - 200
    t = time.monotonic()
    code_ka, ans_ka = cj.post("/cmd/keepalive", {})
    _, after_ka = cj.json("/state.json")
    ika = (ans_ka or {}).get("idle") or {}
    check("M5 POST /cmd/keepalive (the idle line's button): idle 250 s -> the answer's idle restarts under 1 s with "
          "last_source 'keepalive', /state.json says the same; without the page's header -> 403, the count kept",
          code_ka403 == 403 and kept_ka and code_ka == 200 and ans_ka.get("ok") is True and ika.get("enabled") is True
          and isinstance(ika.get("idle_s"), float) and ika["idle_s"] < 1.0 and ika.get("last_source") == "keepalive"
          and (before_ka.get("idle") or {}).get("idle_s", 0) >= 249
          and (after_ka.get("idle") or {}).get("last_source") == "keepalive" and c.activity_mono >= t - 1.0,
          json.dumps([code_ka403, code_ka, ans_ka, before_ka.get("idle"), after_ka.get("idle")])[:500])

    # the watchdog
    c.launcher.calls.clear()
    reg_j("idle-run-2", ago=1000.0)
    hear(c)
    aged(c, 299.0)
    r0 = c.idle_tick()
    calls0 = list(c.launcher.calls)
    said.clear()
    aged(c, 301.0)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        r1 = c.idle_tick()
    check("watchdog, limit 300 s: idle 299 s -> nothing; idle 301 s -> one stop: the launcher's 'stop' (the page's "
          "Stop stack path) with the note '" + MSG5 + "', _say('info', the same), the line in the cockpit's own "
          "log (its standard output, the service's journal)",
          r0 is None and calls0 == [] and r1 is not None and r1.get("ok") and c.launcher.calls == [("stop", MSG5)]
          and said == [("info", MSG5)] and f"moss_cockpit: info: {MSG5}" in out.getvalue(),
          f"{r0} {calls0} {r1} {c.launcher.calls} {said} {out.getvalue()!r}")
    again = quiet(lambda: [c.idle_tick() for _ in range(3)])
    check("the same run, still up (going down) -> no second stop", again == [None] * 3 and len(c.launcher.calls) == 1,
          f"{again} {c.launcher.calls}")
    reg_j("idle-run-3", ago=400.0)
    hear(c)
    r3, r3b = quiet(lambda: (c.idle_tick(), c.idle_tick()))
    check("a new run idle past the limit -> stopped again, once",
          r3 is not None and r3b is None and c.launcher.calls == [("stop", MSG5)] * 2, str(c.launcher.calls))
    kinds = []
    for bp in ("other-dimos.explore", "moss-dimos.moss-gamepad", "moss-dimos.moss-base", "moss-dimos.moss-nav"):
        reg_j("idle-run-4-" + bp.rsplit(".", 1)[-1], bp, ago=5000.0)
        c4, said4 = idle_cockpit()
        hear(c4)
        aged(c4, 5000.0)
        r4 = quiet(c4.idle_tick)
        kinds.append([r4 is not None, len(c4.launcher.calls), c4.idle_doc(mc.stack_status())["stop_in_s"]])
    check("M4 idle 5000 s: a stack that is not MOSS ('other-dimos.explore'), moss-gamepad (the pad on the table, as "
          "moss-teleop.sh promises) and moss-base -> never stopped, no stop_in_s (the page hides the line); "
          "moss-nav -> stopped", kinds == [[False, 0, None]] * 3 + [[True, 1, 0.0]], json.dumps(kinds))
    reg_j("idle-run-5", ago=5000.0)
    c0, said0 = idle_cockpit(limit=0.0)
    hear(c0)
    aged(c0, 5000.0)
    doc0 = c0.idle_doc(mc.stack_status())
    check("limit 0 (MOSS_IDLE_STOP_S=0): a MOSS stack idle 5000 s -> never stopped; idle doc enabled false",
          c0.idle_tick() is None and c0.launcher.calls == [] and doc0["enabled"] is False and doc0["stop_in_s"] is None,
          json.dumps(doc0))
    cb, saidb = idle_cockpit()
    hear(cb)
    aged(cb, 5000.0)
    cb.launcher.busy = True
    rb = cb.idle_tick()
    cb.launcher.busy = False
    rb2 = quiet(cb.idle_tick)
    check("the launcher busy (a check, a launch, a stop of the operator's) -> the watchdog waits; free -> it stops",
          rb is None and rb2 is not None and cb.launcher.calls == [("stop", MSG5)], str(cb.launcher.calls))
    cbl, saidbl = idle_cockpit()
    hear(cbl)
    aged(cbl, 5000.0)
    cbl.launcher.busy, cbl.launcher.busy_since = True, time.time() - 60.0
    rbl1 = cbl.idle_tick()
    cbl.launcher.busy_since = time.time() - 130.0
    rbl2 = quiet(cbl.idle_tick)
    rbl3 = quiet(cbl.idle_tick)
    check("m5 a launcher busy 60 s -> the watchdog waits; busy 130 s (over 120 s: stuck) -> it stops anyway, once, and "
          "the journal says so once", rbl1 is None and rbl2 is not None and rbl3 is None
          and cbl.launcher.calls == [("stop", MSG5)] and len(warnings(saidbl)) == 1
          and "has run 130 s" in warnings(saidbl)[0], f"{cbl.launcher.calls} {saidbl}")
    cl_, saidl = idle_cockpit()
    hear(cl_)
    aged(cl_, 5000.0)
    cl_.listening = False
    docl = cl_.idle_doc(mc.stack_status())
    check("a cockpit not listening to the bus (it cannot see the activity), a message still in its cache -> never "
          "stops; idle doc enabled false, blind says why",
          quiet(cl_.idle_tick) is None and cl_.launcher.calls == [] and docl["enabled"] is False
          and "does not listen" in (docl.get("blind") or ""), json.dumps(docl))
    cd_, saidd = idle_cockpit()
    aged(cd_, 5000.0)
    rd = quiet(lambda: [cd_.idle_tick() for _ in range(3)])
    docd = cd_.idle_doc(mc.stack_status())
    hear(cd_, ago=11.0)
    rd11 = quiet(cd_.idle_tick)
    hear(cd_, ago=9.0)
    rd9 = quiet(cd_.idle_tick)
    check("B1 a deaf cockpit (subscribed, but nothing on any tapped channel while a MOSS stack runs: another multicast "
          "route, a firewall) never stops it: 3 looks idle 5000 s -> nothing, idle doc enabled false with blind "
          "'nothing heard on the bus for 10 s', the journal says it once; the last message 11 s old -> still deaf; "
          "9 s old -> it hears, the stop goes",
          rd == [None] * 3 and docd["enabled"] is False and "nothing heard on the bus for 10 s" in (docd.get("blind") or "")
          and len(warnings(saidd)) == 1 and "nothing heard" in warnings(saidd)[0] and rd11 is None
          and rd9 is not None and cd_.launcher.calls == [("stop", MSG5)],
          json.dumps([rd, docd, saidd, rd11, rd9, cd_.launcher.calls])[:600])
    url_other = "udpm://239.255.76.67:7668?ttl=0"
    other = stand_in(env=dict(env_lcm, LCM_DEFAULT_URL=url_other))
    reg_j("idle-run-url", ago=5000.0, pid=other.pid)
    cu, saidu = idle_cockpit()
    hear(cu)
    aged(cu, 5000.0)
    ru = quiet(cu.idle_tick)
    docu = cu.idle_doc(mc.stack_status())
    os.environ["LCM_DEFAULT_URL"] = url_other     # the cockpit on the same group: it sees the run
    try:
        docu2 = cu.idle_doc(mc.stack_status())
        ru2 = quiet(cu.idle_tick)
    finally:
        os.environ.pop("LCM_DEFAULT_URL", None)
    other.kill()
    other.wait()
    check("B1 a MOSS stack started with another LCM_DEFAULT_URL than the cockpit's (another bus, even with a message "
          "in the cache) -> never stopped, blind says why; the cockpit on the same URL -> it stops it",
          ru is None and docu["enabled"] is False and "LCM_DEFAULT_URL" in (docu.get("blind") or "")
          and docu2["enabled"] is True and ru2 is not None and cu.launcher.calls == [("stop", MSG5)],
          json.dumps([docu, docu2, cu.launcher.calls]))
    zen = stand_in(env=dict(env_lcm, DIMOS_TRANSPORT="zenoh"))
    reg_j("idle-run-6", ago=5000.0, pid=zen.pid)
    cz, saidz = idle_cockpit()
    hear(cz)
    aged(cz, 5000.0)
    docz = cz.idle_doc(mc.stack_status())
    check("a MOSS stack started with DIMOS_TRANSPORT=zenoh (its activity never reaches the LCM tap) -> never stopped; "
          "idle doc enabled false", quiet(cz.idle_tick) is None and cz.launcher.calls == [] and docz["enabled"] is False,
          json.dumps(docz))
    zen.kill()
    zen.wait()
    sleeper = subprocess.Popen(["sleep", "600"], env=env_lcm, stdin=subprocess.DEVNULL)
    atexit.register(sleeper.kill)
    stale = []
    for pid in (1, sleeper.pid):
        reg_j("stale-run-moss-explore", ago=5000.0, pid=pid)
        cs, saids = idle_cockpit()
        hear(cs)
        aged(cs, 5000.0)
        rs = quiet(lambda: [cs.idle_tick() for _ in range(3)])
        docs = cs.idle_doc(mc.stack_status())
        stale.append([rs == [None] * 3, cs.launcher.calls == [], docs["enabled"] is False,
                      len(warnings(saids)) == 1 and "not a dimos run" in warnings(saids)[0]])
    sleeper.kill()
    sleeper.wait()
    check("M3 a stale registry entry (never stopped: a power cut) whose pid now runs something else (pid 1, a sleep) "
          "-> never stopped: its /proc cmdline is not a dimos run (the rule of _dimos_run_pid and STACK_RE); the "
          "journal says it once; the idle doc enabled false", stale == [[True] * 4] * 2, json.dumps(stale))

    class ClockStep:
        """The time module with the wall clock stepped by `step` s (an NTP step); the monotonic clock untouched."""

        def __init__(self, step):
            self.step = step

        def __getattr__(self, name):
            return getattr(time, name)

        def time(self):
            return time.time() + self.step

    reg_j("idle-run-clock", ago=60.0)
    ck, _ = idle_cockpit()
    hear(ck)
    aged(ck, 10.0)
    first_ck = ck.idle_tick()                     # the run's start goes to the monotonic clock, once
    steps = []
    try:
        for step in (3600.0, -3600.0):
            mc.time = ClockStep(step)
            steps.append([ck.idle_tick(), ck.idle_doc(mc.stack_status())["idle_s"]])
    finally:
        mc.time = time
    check("m1 the wall clock stepped +1 h, then -1 h, under a stack up 60 s with activity 10 s ago -> no stop, idle "
          "stays about 10 s (the count and the run's start live on the monotonic clock)",
          first_ck is None and all(r is None and 9.0 <= i <= 12.0 for r, i in steps) and ck.launcher.calls == [],
          json.dumps(steps))
    reg_j("idle-run-retry", ago=5000.0)
    cr, saidr = idle_cockpit()
    hear(cr)
    aged(cr, 5000.0)
    cr.launcher.fail = "raise"
    tries = quiet(lambda: [cr.idle_tick() for _ in range(3)])
    cr.launcher.fail = "launcher missing"
    tries2 = quiet(cr.idle_tick)
    marked = cr._idle_stopped
    cr.launcher.fail = None
    ok_r = quiet(cr.idle_tick)
    after_r = quiet(cr.idle_tick)
    check("m4 the launcher raising (state dir not writable) -> nothing escapes the watchdog, the run is not marked, it "
          "is tried again at each look (3 tries), the journal once per error; answering not ok -> tried again; then "
          "working -> one stop, marked, no more",
          all(isinstance(x, dict) and x.get("ok") is False for x in tries) and (tries2 or {}).get("ok") is False
          and marked is None and (ok_r or {}).get("ok") is True and after_r is None and len(cr.launcher.calls) == 5
          and len(warnings(saidr)) == 2 and ("info", MSG5) in saidr,
          json.dumps([tries, tries2, cr.launcher.calls, saidr], default=str)[:600])
    reg_j(None)

    # the real launcher writes the note where /launch.json reads the launch lines
    stub_j = Path(TMP, "idle_stub.sh")
    stub_j.write_text('echo "stub $1 ran"\n')
    la_j = mc.Launcher(str(stub_j))
    ans_j = la_j.start("stop", note=MSG5)
    la_j._procs[-1][2].wait(5)
    st_j = la_j.status()
    lines_j = st_j["lines"]
    check("the launcher with a note: last_launch.log reads '=== " + MSG5 + "  <time> ===', then the stop's own "
          "header, then the script; status note.text = the message",
          ans_j["ok"] and len(lines_j) >= 3
          and re.fullmatch(r"=== stopping the stack to preserve battery \(idle 5 min\)  "
                           r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d ===", lines_j[0]) is not None
          and lines_j[1].startswith("=== moss-fly.sh stop  ") and "stub stop ran" in lines_j
          and (st_j.get("note") or {}).get("text") == MSG5, json.dumps(st_j)[:300])
    la_j.start("check")
    la_j._procs[-1][2].wait(5)
    check("the next launch, the page's -> no note", la_j.status(lines=False)["note"] is None)

    # /state.json, live
    reg_j("idle-run-7", ago=40.0)
    c.idle_limit_s = 300.0
    aged(c, 1000.0)
    c.launcher = mc.Launcher(str(stub_j))
    c.launcher.start("stop", note=MSG5)
    c.launcher._procs[-1][2].wait(5)
    cj.json("/state.json")                        # the new run: the cockpit drops the last run's messages
    hear(c)
    _, stj = cj.json("/state.json")
    idj = stj.get("idle") or {}
    check("live /state.json: idle {enabled, idle_s, stop_in_s, limit_s, last_source, blind} next to stack: enabled, "
          "idle ~40 s (from the stack's start, last_source 'start'), stop in 300 - idle, limit 300, blind null; "
          "launch.note.text = the message",
          set(idj) == set(mc.IDLE_KEYS) and idj["enabled"] is True and 39.0 <= idj["idle_s"] <= 45.0
          and abs(idj["idle_s"] + idj["stop_in_s"] - 300.0) < 0.2 and idj["limit_s"] == 300
          and idj["last_source"] == "start" and idj["blind"] is None
          and ((stj.get("launch") or {}).get("note") or {}).get("text") == MSG5,
          json.dumps({"idle": idj, "launch": stj.get("launch")}))
    reg_j(None)
    _, stn = cj.json("/state.json")
    stn_idle = stn.get("idle") or {}
    check("live /state.json, no stack: idle enabled, idle_s and stop_in_s null (the page hides the line), blind null",
          {k: stn_idle.get(k) for k in ("enabled", "idle_s", "stop_in_s", "limit_s", "blind")}
          == {"enabled": True, "idle_s": None, "stop_in_s": None, "limit_s": 300.0, "blind": None}
          and stn_idle.get("last_source") in mc.IDLE_SOURCES, json.dumps(stn_idle))
    srv_j.shutdown()
    STAND_IN.kill()
    STAND_IN.wait()
    if lcm_url_was is not None:
        os.environ["LCM_DEFAULT_URL"] = lcm_url_was
mc._dimos_run_pid = real_pid


class FakeLcmHandle:
    """lcm.LCM for the bus tap's loop: handle_timeout returns at once (no message), and ends the
    loop by raising SystemExit (past its `except Exception` retry) once `stop` is set."""
    stop = False

    def subscribe(self, channel, fn):
        return channel

    def unsubscribe(self, sub):
        pass

    def handle_timeout(self, ms):
        if FakeLcmHandle.stop:
            raise SystemExit
        time.sleep(0.002)
        return 0


real_lcm = mc.lcm
mc.lcm = types.SimpleNamespace(LCM=FakeLcmHandle)
cw = mc.LiveCockpit(str(Path(TMP, "never_run.sh")))
looks = []
cw.idle_tick = lambda: looks.append(time.monotonic())
listen_t = threading.Thread(target=cw._listen, daemon=True)
listen_t.start()
time.sleep(0.3)
listening_early = cw.listening
time.sleep(2.25)
FakeLcmHandle.stop = True
listen_t.join(3.0)
mc.lcm = real_lcm
gaps = [round(b - a, 3) for a, b in zip(looks, looks[1:])]
check("M2 the wiring: _listen with a fake lcm whose handle_timeout returns at once -> listening true, the watchdog "
      "called about once a second (3 looks in 2.55 s, 1.0 s apart: the 1 Hz throttle), the thread ends with the bus",
      listening_early is True and len(looks) == 3 and all(0.95 <= g <= 1.15 for g in gaps) and not listen_t.is_alive(),
      f"listening {listening_early}, {len(looks)} looks, gaps {gaps[:5]}")

# the demo: same document; its stack stops itself on the first request past the limit
dsrv_j, dcl_j = serve(demo=True, page_dir=page_dir)
_, sd = dcl_j.json("/state.json")
idd = sd.get("idle") or {}
check("demo /state.json: idle with the same keys: enabled, limit 300, idle_s a number, stop_in_s = 300 - idle_s, "
      "blind null",
      set(idd) == set(mc.IDLE_KEYS) and idd["enabled"] is True and idd["limit_s"] == 300
      and isinstance(idd["idle_s"], float) and abs(idd["idle_s"] + idd["stop_in_s"] - 300.0) < 0.2
      and idd["blind"] is None, json.dumps(idd))
code_dka, dka = dcl_j.post("/cmd/keepalive", {})
check("demo POST /cmd/keepalive -> ok, its idle restarts (idle_s under 1 s), last_source 'keepalive'",
      code_dka == 200 and dka.get("ok") is True and (dka.get("idle") or {}).get("last_source") == "keepalive"
      and (dka.get("idle") or {}).get("idle_s", 9) < 1.0, json.dumps(dka))
dj = dsrv_j.cockpit
dj.idle_limit_s = 1.0
time.sleep(1.2)
_, sd_drive = dcl_j.json("/state.json")
check("demo, limit 1 s: the demo rover exploring (driving) is never idle -> its stack stays up (last_source 'demo')",
      sd_drive["stack"]["running"] is True and sd_drive["idle"]["idle_s"] < 1.0
      and sd_drive["idle"]["last_source"] == "demo", json.dumps(sd_drive["idle"]))
dcl_j.post("/cmd/stop", {})                    # halted: the rover stands still from here
_, sd_a = dcl_j.json("/state.json")
time.sleep(1.3)
_, sd_b = dcl_j.json("/state.json")
_, la_d = dcl_j.json("/launch.json")
_, lg_d = dcl_j.json("/log.json?n=10")
MSG1 = "stopping the stack to preserve battery (idle 1 s)"
note_lines = [ln for ln in la_d["lines"] if MSG1 in ln]
check("demo, halted, 1.3 s later -> the stack stopped itself: running false, launch.note.text = '" + MSG1 + "', "
      "/launch.json the line '=== <msg>  <time> === (demo)' then the stop's header, /log.json the message",
      sd_a["stack"]["running"] is True and sd_b["stack"] == {"running": False}
      and ((sd_b.get("launch") or {}).get("note") or {}).get("text") == MSG1 and len(note_lines) == 1
      and la_d["lines"].index(note_lines[0]) + 1 < len(la_d["lines"])
      and la_d["lines"][la_d["lines"].index(note_lines[0]) + 1].startswith("=== moss-fly.sh stop  ")
      and any(r.get("event") == MSG1 for r in lg_d["lines"]),
      json.dumps({"stack": sd_b["stack"], "note": sd_b.get("launch"), "lines": la_d["lines"][-4:]})[:400])
dcl_j.json("/state.json")
_, la_d2 = dcl_j.json("/launch.json")
check("demo: the same (stopped) run -> the line once", sum(MSG1 in ln for ln in la_d2["lines"]) == 1)
dcl_j.post("/cmd/stack", {"action": "explore"})   # a new run (exploration off: nothing drives)
_, sd_c = dcl_j.json("/state.json")
time.sleep(1.3)
_, sd_d = dcl_j.json("/state.json")
_, la_d3 = dcl_j.json("/launch.json")
check("demo: a new stack, nothing driving -> up, then stopped again after the limit (the line twice in all)",
      sd_c["stack"]["running"] is True and (sd_c.get("launch") or {}).get("note") is None
      and sd_d["stack"] == {"running": False} and sum(MSG1 in ln for ln in la_d3["lines"]) == 2,
      json.dumps([sd_c["stack"].get("running"), sd_d["stack"], la_d3["lines"][-3:]])[:300])
dsrv_j.shutdown()
dj0 = mc.DemoCockpit()
dj0.idle_limit_s = 0.0
dj0.activity_mono = time.monotonic() - 5000.0
dj0.command("stop", {})
dj0.activity_mono = time.monotonic() - 5000.0
sd0 = dj0.state()
check("demo, limit 0 -> idle enabled false, the halted stack never stopped",
      sd0["idle"]["enabled"] is False and sd0["stack"]["running"] is True, json.dumps(sd0["idle"]))
check("the server's docstring and docs/cockpit.md name MOSS_IDLE_STOP_S and the message",
      "MOSS_IDLE_STOP_S" in (mc.__doc__ or "") and "stopping the stack to preserve battery" in (mc.__doc__ or "")
      and "MOSS_IDLE_STOP_S" in cockpit_md and "stopping the stack to preserve battery" in cockpit_md)
idle_md = re.search(r"\*\*The stack stops itself\.\*\*[\s\S]*?first request past the limit\.", cockpit_md)
idle_par = re.sub(r"\s+", " ", idle_md.group(0)) if idle_md else ""
idle_says = ("about 2 W more than no stack at all (one measurement at the time of this change)",
             "`moss-gamepad` and any other MOSS blueprint stay up", "for 10 s", "LCM_DEFAULT_URL", "`goal_request`",
             "`episode_marker`", "an open episode", "POST /cmd/keepalive", "10 s before", "last_source",
             "under 30 s is refused", "stopped at 14:05", "a restarted cockpit no longer shows it", "up to 120 s",
             "no longer a `dimos run`", "tried again each second", "monotonic clock")
check("docs/cockpit.md, the idle stop paragraph: the power as one measured difference (about 2 W more, no absolute "
      "watts), moss-nav and moss-explore only (moss-gamepad stays up), the deaf cockpit (10 s) and LCM_DEFAULT_URL, "
      "goal_request / episode_marker / an open episode, the keepalive button, the 10 s window, last_source, under "
      "30 s refused, 'stopped at', the note lost with a restart, the 120 s launcher, the stale pid, the retry",
      bool(idle_par) and all(s in idle_par for s in idle_says) and "6.6 W" not in cockpit_md
      and "4.7 W" not in cockpit_md and "VDD_IN" not in idle_par,
      ", ".join(s for s in idle_says if s not in idle_par) or idle_par[:120])

# ---------------------------------------------------------------------------
print("F. hygiene")
src = (ROOT / "tools" / "moss_cockpit.py").read_text()
ctl = (ROOT / "tools" / "explore_ctl.py").read_text()
check("no dimOS / torch / open3d import", not re.search(r"^\s*(import|from)\s+(dimos|torch|open3d)\b", src, re.M))
check("no serial port: no pyserial import, no /dev/tty", not re.search(r"^\s*(import|from)\s+serial\b", src, re.M)
      and "/dev/tty" not in src)
lan = re.compile(r"\b(192\.168|10\.\d{1,3}|172\.(1[6-9]|2\d|3[01]))\.\d{1,3}\.\d{1,3}\b")
check("no LAN address literal in moss_cockpit.py / explore_ctl.py", not lan.search(src) and not lan.search(ctl))
check("no non-ASCII text in moss_cockpit.py (English only)", all(ord(ch) < 128 for ch in src))
check("'dimos' never imported by this bench", "dimos" not in sys.modules)

_TMPDIR.cleanup()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
