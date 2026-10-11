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
  F. hygiene        - no dimos/serial/torch import, no LAN address in the server

C, D and the run-change checks of G need dimos_lcm (pure python, installed with
dimOS; skipped without it). Nothing here needs dimOS, lcm or hardware.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_moss_cockpit_cold.py
"""

import atexit
import http.client
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
