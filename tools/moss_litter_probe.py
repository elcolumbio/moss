#!/usr/bin/env python3
"""Measure the litter detector on the real floor: does it see the litter, and where?

The mission trusts two things it has never been measured on in this room:
that the YOLO weights see our litter on our floor, and that the box lifted
through the depth image lands where the object is. This tool runs the nose
camera and the detector alone, without the stack, and writes one CSV line per
detection with the object's position in base_link (x forward, y left, z up,
the lidar's floor projection), then a summary: how many frames saw the
litter, where it was seen, how far from where the tape measure says it is.

Run it on the rover with the stack DOWN (it opens the RealSense itself; two
readers of one camera do not work). Put the litter on the floor, measure its
position from the lidar axis with a tape, and:

    tools/moss_litter_probe.py --seconds 10                      # the nose camera, the weights, one CSV
    tools/moss_litter_probe.py --seconds 10 --expect 1.00 0.00   # the can is 1.00 m ahead on the axis
    tools/moss_litter_probe.py --fake --frames 20                # no camera, no torch: a synthetic frame, a fake detector

Options: --weights (MOSS_LITTER_WEIGHTS, ~/.local/state/moss/models/litter.pt),
--classes "Metal,Plastic" (MOSS_LITTER_CLASSES: the ones the mission picks up),
--device cpu|cuda (MOSS_LITTER_DEVICE), --conf 0.5 --iou 0.6 (the detector's own
defaults in the mission), --width 640 --height 480 --fps 15 (the camera),
--camera-xyz "x y z" and --pitch-deg (the robot profile's MOSS_CAMERA_XYZ_M and
MOSS_CAMERA_PITCH_DEG otherwise), --out <csv> (default
<MOSS_STATE_DIR>/litter_probe/litter_probe_<stamp>.csv). It writes the CSV and
nothing else: no motor, no bus.

The geometry: the colour pixel and the aligned depth give a point in the
camera's optical frame (x right, y down, z forward); the mechanical camera
frame is x forward, y left, z up (x = Z, y = -X, z = -Y); the mount is a pure
pitch about +y, positive = nose down (blueprints.camera_mount_xyzq), then the
camera's position in base_link. The depth of a box is the median of the
non-zero depths in its central third (edges and holes left out).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos.robot_profile import moss_profile, moss_state_dir  # noqa: E402

try:  # the mission's own names and defaults; dimOS is not needed here, so fall back without it
    from moss_dimos.litter_detect import LITTER_CLASSES_DEFAULT, litter_classes, weights_path
except Exception:  # noqa: BLE001  (ImportError, or dimos' own import errors)
    LITTER_CLASSES_DEFAULT = "Glass,Metal,Paper,Plastic,Waste"

    def weights_path(env=None):
        env = os.environ if env is None else env
        return Path(env.get("MOSS_LITTER_WEIGHTS", "").strip() or "~/.local/state/moss/models/litter.pt").expanduser()

    def litter_classes(env=None):
        env = os.environ if env is None else env
        raw = env.get("MOSS_LITTER_CLASSES", "").strip() or LITTER_CLASSES_DEFAULT
        return {c.strip() for c in raw.split(",") if c.strip()}

CONF_DEFAULT = 0.5          # the detector's own thresholds in litter_detect
IOU_DEFAULT = 0.6
CSV_FIELDS = ("frame", "t_s", "latency_ms", "cls", "conf", "u1", "v1", "u2", "v2", "depth_m", "x_m", "y_m", "z_m")

# the fake camera: one object at a known place, so the whole chain has a known answer
FAKE_INTR = (400.0, 400.0, 320.0, 240.0)       # fx, fy, cx, cy
FAKE_PIXEL = (360, 300)                        # the object's centre: 40 px right, 60 px down of the centre
FAKE_DEPTH_M = 1.0
FAKE_CLASS = "Metal"
FAKE_CONF = 0.83
FAKE_BOX_HALF = 20                             # px


# ---------------------------------------------------------------- geometry (pure)

def deproject(u: float, v: float, depth_m: float, intr) -> tuple[float, float, float]:
    """Pixel (u, v) at depth_m -> the point in the camera's optical frame (X right, Y down, Z forward)."""
    fx, fy, cx, cy = intr
    return ((u - cx) / fx * depth_m, (v - cy) / fy * depth_m, depth_m)


def optical_to_base(p, camera_xyz, pitch_deg: float = 0.0) -> tuple[float, float, float]:
    """Optical frame -> base_link: mechanical camera frame (x = Z, y = -X, z = -Y), the mount's pitch
    about +y (positive = nose down, the convention of blueprints.camera_mount_xyzq), then the camera's
    position in base_link."""
    xc, yc, zc = p[2], -p[0], -p[1]
    th = math.radians(pitch_deg)
    xr = math.cos(th) * xc + math.sin(th) * zc
    zr = -math.sin(th) * xc + math.cos(th) * zc
    return (camera_xyz[0] + xr, camera_xyz[1] + yc, camera_xyz[2] + zr)


def box_centre(box) -> tuple[float, float]:
    u1, v1, u2, v2 = box
    return ((u1 + u2) / 2.0, (v1 + v2) / 2.0)


def box_depth_m(depth, box, depth_scale: float) -> float | None:
    """The median of the non-zero depths in the box's central third, in metres; None when the
    patch has no depth (a hole, or the box out of the image)."""
    import numpy as np

    h, w = depth.shape[:2]
    u1, v1, u2, v2 = box
    du, dv = (u2 - u1) / 3.0, (v2 - v1) / 3.0
    a, b = int(max(0, math.floor(u1 + du))), int(min(w, math.ceil(u2 - du)))
    c, d = int(max(0, math.floor(v1 + dv))), int(min(h, math.ceil(v2 - dv)))
    if b <= a or d <= c:
        return None
    patch = np.asarray(depth[c:d, a:b], dtype=np.float64)
    vals = patch[patch > 0]
    if vals.size == 0:
        return None
    return float(np.median(vals)) * float(depth_scale)


def parse_xyz(raw: str) -> tuple[float, float, float]:
    vals = [float(t) for t in raw.replace(",", " ").split()]
    if len(vals) != 3:
        raise ValueError(f"three numbers expected, got {raw!r}")
    return (vals[0], vals[1], vals[2])


# ---------------------------------------------------------------- the summary (pure)

def _median(xs):
    return statistics.median(xs) if xs else None


def _spread(xs):
    """Half the 16-84 percentile span: one sigma for a normal spread, robust to a stray box."""
    if len(xs) < 2:
        return 0.0
    s = sorted(xs)
    lo = s[int(round(0.16 * (len(s) - 1)))]
    hi = s[int(round(0.84 * (len(s) - 1)))]
    return (hi - lo) / 2.0


def summarise(rows, frames: int, expect=None) -> dict:
    """rows: dicts with the CSV fields (one per detection); frames: frames looked at.
    -> {frames, hit_frames, hit_rate, latency_ms, classes: {name: {n, conf, x, y, z, sx, sy}}, error}"""
    hit = {r["frame"] for r in rows if r.get("x_m") is not None}
    by = {}
    for r in rows:
        by.setdefault(r["cls"], []).append(r)
    classes = {}
    for name, rs in by.items():
        xs = [r["x_m"] for r in rs if r.get("x_m") is not None]
        ys = [r["y_m"] for r in rs if r.get("y_m") is not None]
        zs = [r["z_m"] for r in rs if r.get("z_m") is not None]
        classes[name] = {"n": len(rs), "no_depth": sum(1 for r in rs if r.get("x_m") is None),
                         "conf": _median([r["conf"] for r in rs]),
                         "x": _median(xs), "y": _median(ys), "z": _median(zs), "sx": _spread(xs), "sy": _spread(ys)}
    lat = _median([r["latency_ms"] for r in rows]) if rows else None
    out = {"frames": frames, "hit_frames": len(hit), "hit_rate": (len(hit) / frames) if frames else 0.0,
           "latency_ms": lat, "classes": classes, "error": None}
    if expect is not None and classes:
        # the error of the most seen class against the tape measure
        best = max(classes.values(), key=lambda c: c["n"])
        if best["x"] is not None and best["y"] is not None:
            dx, dy = best["x"] - expect[0], best["y"] - expect[1]
            out["error"] = {"dx": dx, "dy": dy, "dist": math.hypot(dx, dy)}
    return out


def summary_lines(s: dict, expect=None) -> list[str]:
    lines = [f"frames {s['frames']}, with a detection {s['hit_frames']} (hit rate {100 * s['hit_rate']:.0f} %)"
             + (f", detector {s['latency_ms']:.0f} ms a frame (median)" if s["latency_ms"] is not None else "")]
    for name, c in sorted(s["classes"].items(), key=lambda kv: -kv[1]["n"]):
        if c["x"] is None:
            lines.append(f"  {name}: {c['n']} boxes, no depth under any of them")
            continue
        lines.append(f"  {name}: {c['n']} boxes, conf {c['conf']:.2f}, at x {c['x']:+.3f} y {c['y']:+.3f} z {c['z']:+.3f} m"
                     f" (spread {c['sx']:.3f} / {c['sy']:.3f})" + (f", {c['no_depth']} without depth" if c["no_depth"] else ""))
    if expect is not None:
        e = s["error"]
        if e is None:
            lines.append(f"  expected x {expect[0]:+.3f} y {expect[1]:+.3f}: nothing to compare")
        else:
            lines.append(f"  against the tape (x {expect[0]:+.3f} y {expect[1]:+.3f}): dx {e['dx']:+.3f} dy {e['dy']:+.3f}"
                         f" m, {e['dist'] * 100:.1f} cm off")
    return lines


# ---------------------------------------------------------------- cameras and detectors

class FakeCamera:
    """One synthetic 640x480 frame: a grey floor, a box of another grey at FAKE_PIXEL, depth FAKE_DEPTH_M
    there and a farther floor elsewhere."""

    def __init__(self):
        self.intr = FAKE_INTR
        self.depth_scale = 0.001

    def frames(self, n: int):
        import numpy as np

        for _ in range(n):
            color = np.full((480, 640, 3), 120, dtype=np.uint8)
            depth = np.full((480, 640), 2500, dtype=np.uint16)
            u, v = FAKE_PIXEL
            h = FAKE_BOX_HALF
            color[v - h:v + h, u - h:u + h] = 40
            depth[v - h:v + h, u - h:u + h] = int(round(FAKE_DEPTH_M / self.depth_scale))
            depth[v - 2:v + 2, u - 2:u + 2] = 0        # a hole in the middle: the median must survive it
            yield color, depth

    def close(self):
        pass


class FakeDetector:
    names = {0: FAKE_CLASS}

    def detect(self, color):
        u, v = FAKE_PIXEL
        h = FAKE_BOX_HALF
        return [(FAKE_CLASS, FAKE_CONF, (u - h, v - h, u + h, v + h))]


class RealSenseCamera:
    """Colour and depth aligned to colour, from the nose camera, via pyrealsense2."""

    def __init__(self, width: int, height: int, fps: int):
        import pyrealsense2 as rs

        self._rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = (i.fx, i.fy, i.ppx, i.ppy)
        self.align = rs.align(rs.stream.color)

    def frames(self, n: int):
        import numpy as np

        for _ in range(n):
            fs = self.align.process(self.pipe.wait_for_frames(5000))
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if not c or not d:
                continue
            yield np.asanyarray(c.get_data()), np.asanyarray(d.get_data())

    def close(self):
        try:
            self.pipe.stop()
        except Exception:  # noqa: BLE001
            pass


class YoloDetector:
    """The mission's detector, standalone: ultralytics YOLO on the weights, the mission's thresholds."""

    def __init__(self, weights: Path, device: str | None, conf: float, iou: float):
        from ultralytics import YOLO

        self.model = YOLO(str(weights))
        self.names = self.model.names
        self.device, self.conf, self.iou = device, conf, iou

    def detect(self, color):
        kw = {"conf": self.conf, "iou": self.iou, "verbose": False}
        if self.device:
            kw["device"] = self.device
        res = self.model.predict(color, **kw)
        out = []
        for r in res:
            b = r.boxes
            if b is None or len(b) == 0:
                continue
            xyxy = b.xyxy.cpu().numpy()
            confs = b.conf.cpu().numpy()
            clss = b.cls.cpu().numpy()
            for i in range(len(confs)):
                name = self.names.get(int(clss[i]), str(int(clss[i]))) if isinstance(self.names, dict) else str(int(clss[i]))
                out.append((str(name), float(confs[i]), tuple(float(x) for x in xyxy[i])))
        return out


# ---------------------------------------------------------------- the run

def default_out(env=None) -> Path:
    return moss_state_dir(env) / "litter_probe" / ("litter_probe_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")


def run(camera, detector, wanted: set[str], camera_xyz, pitch_deg: float, out: Path, frames: int,
        seconds: float, expect=None, say=print) -> dict:
    """Frames through the detector, one CSV row per wanted detection, the summary printed and returned."""
    wanted_l = {w.lower() for w in wanted}
    out.parent.mkdir(parents=True, exist_ok=True)
    rows, n_frames, t0 = [], 0, time.monotonic()
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        w.writeheader()
        for color, depth in camera.frames(frames):
            t = time.monotonic() - t0
            if seconds and t > seconds:
                break
            n_frames += 1
            ta = time.monotonic()
            dets = detector.detect(color)
            lat = (time.monotonic() - ta) * 1000.0
            seen = []
            for name, conf, box in dets:
                if wanted_l and name.lower() not in wanted_l:
                    continue
                d = box_depth_m(depth, box, camera.depth_scale)
                x = y = z = None
                if d is not None:
                    u, v = box_centre(box)
                    x, y, z = optical_to_base(deproject(u, v, d, camera.intr), camera_xyz, pitch_deg)
                row = {"frame": n_frames, "t_s": round(t, 3), "latency_ms": round(lat, 1), "cls": name,
                       "conf": round(conf, 3), "u1": round(box[0], 1), "v1": round(box[1], 1),
                       "u2": round(box[2], 1), "v2": round(box[3], 1),
                       "depth_m": None if d is None else round(d, 3),
                       "x_m": None if x is None else round(x, 3), "y_m": None if y is None else round(y, 3),
                       "z_m": None if z is None else round(z, 3)}
                rows.append(row)
                w.writerow(row)
                seen.append(f"{name} {conf:.2f}" + (f" at x {x:+.2f} y {y:+.2f}" if x is not None else " (no depth)"))
            say(f"frame {n_frames:3d}  t {t:5.1f} s  {lat:5.0f} ms  " + ("; ".join(seen) if seen else "nothing"))
    s = summarise(rows, n_frames, expect)
    say("")
    for line in summary_lines(s, expect):
        say(line)
    say(f"csv {out}")
    return s


def load_robot_env(text: str) -> dict[str, str]:
    """KEY=VALUE lines of a robot.env (optional `export `, # comments, quotes stripped). Pure."""
    out: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        if "=" not in line:
            continue
        key, val = line.split("=", 1)
        key, val = key.strip(), val.strip()
        if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        if key:
            out[key] = val
    return out


def apply_robot_env(env: dict | None = None) -> tuple[Path, list[str]]:
    """The launchers source <MOSS_STATE_DIR>/robot.env before the stack starts; this tool reads the
    same file so its camera geometry is the stack's (the flight check does the same). The shell's
    own values win. Returns the path and the keys taken from the file."""
    e = os.environ if env is None else env
    path = moss_state_dir(e) / "robot.env"
    try:
        text = path.read_text()
    except OSError:
        return path, []
    taken = []
    for key, val in load_robot_env(text).items():
        if key not in e:
            e[key] = val
            taken.append(key)
    return path, taken


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Measure the litter detector on the floor: hits and positions.")
    ap.add_argument("--fake", action="store_true", help="no camera, no torch: a synthetic frame and a fake detector")
    ap.add_argument("--frames", type=int, default=0, help="frames to look at (default: until --seconds)")
    ap.add_argument("--seconds", type=float, default=10.0, help="stop after this long (default 10 s; 0 = frames only)")
    ap.add_argument("--expect", type=float, nargs=2, metavar=("X", "Y"),
                    help="where the tape says the litter is, in base_link metres (x forward, y left)")
    ap.add_argument("--weights", default=None, help=f"YOLO weights (default {weights_path()})")
    ap.add_argument("--classes", default=None, help="class names the mission picks up (default MOSS_LITTER_CLASSES)")
    ap.add_argument("--device", default=None, help="cpu or cuda (default MOSS_LITTER_DEVICE, else the detector's choice)")
    ap.add_argument("--conf", type=float, default=CONF_DEFAULT)
    ap.add_argument("--iou", type=float, default=IOU_DEFAULT)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--camera-xyz", default=None, help='camera in base_link, "x y z" m (default: the robot profile)')
    ap.add_argument("--pitch-deg", type=float, default=None, help="camera pitch, + = nose down (default: the profile)")
    ap.add_argument("--out", default=None, help="CSV path")
    a = ap.parse_args(argv)

    env_path, taken = apply_robot_env()
    if taken:
        print(f"robot.env {env_path}: {', '.join(sorted(taken))}")
    else:
        print(f"robot.env {env_path}: {'nothing taken (the shell sets everything)' if env_path.is_file() else 'absent'}"
              " - the camera geometry is the profile's default unless MOSS_CAMERA_XYZ_M / --camera-xyz says otherwise")
    profile = moss_profile()
    camera_xyz = parse_xyz(a.camera_xyz) if a.camera_xyz else profile.camera_xyz_base_m
    pitch = profile.camera_pitch_deg if a.pitch_deg is None else a.pitch_deg
    wanted = {c.strip() for c in a.classes.split(",") if c.strip()} if a.classes else litter_classes()
    out = Path(a.out).expanduser() if a.out else default_out()
    frames = a.frames if a.frames > 0 else (10 ** 9 if a.seconds else 0)
    if frames == 0:
        print("nothing to do: give --frames or --seconds", file=sys.stderr)
        return 2

    if a.fake:
        camera, detector = FakeCamera(), FakeDetector()
        print(f"fake camera and detector: one {FAKE_CLASS} at pixel {FAKE_PIXEL}, depth {FAKE_DEPTH_M} m")
    else:
        weights = Path(a.weights).expanduser() if a.weights else weights_path()
        if not weights.is_file():
            print(f"no weights at {weights} (MOSS_LITTER_WEIGHTS; docs/litter.md says where to get them)", file=sys.stderr)
            return 2
        try:
            detector = YoloDetector(weights, a.device or os.environ.get("MOSS_LITTER_DEVICE", "").strip() or None,
                                    a.conf, a.iou)
        except ImportError as exc:
            print(f"ultralytics not importable: {exc} (docs/litter.md: torch + ultralytics in the robot's venv)", file=sys.stderr)
            return 2
        try:
            camera = RealSenseCamera(a.width, a.height, a.fps)
        except ImportError as exc:
            print(f"pyrealsense2 not importable: {exc} (Software.md)", file=sys.stderr)
            return 2
        except Exception as exc:  # noqa: BLE001
            print(f"the RealSense did not open: {exc} (is the stack up? tools/moss-fly.sh stop first)", file=sys.stderr)
            return 2
        print(f"nose camera {a.width}x{a.height} at {a.fps} fps, weights {weights.name}, classes {sorted(wanted)}")
    print(f"camera in base_link {camera_xyz}, pitch {pitch:.1f} deg (+ = nose down)")
    try:
        run(camera, detector, wanted, camera_xyz, pitch, out, frames, a.seconds, a.expect)
    finally:
        camera.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
