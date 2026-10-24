"""Cold bench: tools/moss_litter_probe.py, the litter detector measured on the floor.

Known input -> known output, in metres: a pixel and a depth give a known point in the
camera's optical frame; that point lands at a known place in base_link through the
mechanical frame, the mount's pitch (+ = nose down) and the camera's position; a depth
patch with a hole in the middle gives the median of what is left; ten frames with eight
hits give an 80 % hit rate and a median position 2 cm from the tape; and the whole tool,
run with --fake, writes a CSV whose rows put the fake object where the geometry says
(x 1.156, y -0.100, z -0.075 with the profile's camera at (0.156, 0, 0.075)). No camera,
no torch, no dimOS.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_moss_litter_probe_cold.py
"""
import csv
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

spec = importlib.util.spec_from_file_location("moss_litter_probe", ROOT / "tools" / "moss_litter_probe.py")
lp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lp)

OK = KO = 0


def check(cond, label, detail=""):
    global OK, KO
    print(f"  {'OK ' if cond else 'KO '} {label}{' - ' + str(detail) if detail and not cond else ''}")
    if cond:
        OK += 1
    else:
        KO += 1


def near(a, b, tol=1e-6):
    return a is not None and abs(a - b) <= tol


print("A. geometry")
INTR = (400.0, 400.0, 320.0, 240.0)
p = lp.deproject(320, 240, 1.0, INTR)
check(all(near(a, b) for a, b in zip(p, (0.0, 0.0, 1.0))), "the centre pixel at 1 m -> (0, 0, 1) in the optical frame", p)
p = lp.deproject(720, 240, 2.0, INTR)
check(all(near(a, b) for a, b in zip(p, (2.0, 0.0, 2.0))), "400 px right of the centre (one focal length) at 2 m -> X = 2 m", p)
p = lp.deproject(320, 440, 1.0, INTR)
check(all(near(a, b) for a, b in zip(p, (0.0, 0.5, 1.0))), "200 px below the centre at 1 m -> Y = 0.5 m (down)", p)
CAM = (0.156, 0.0, 0.075)
b = lp.optical_to_base((0.0, 0.0, 1.0), CAM, 0.0)
check(all(near(a, c) for a, c in zip(b, (1.156, 0.0, 0.075))), "straight ahead at 1 m, level camera -> base (1.156, 0, 0.075)", b)
b = lp.optical_to_base((0.5, 0.0, 1.0), CAM, 0.0)
check(all(near(a, c) for a, c in zip(b, (1.156, -0.5, 0.075))), "0.5 m to the RIGHT of the optical axis -> base y = -0.5 (y is left)", b)
b = lp.optical_to_base((0.0, 0.3, 1.0), CAM, 0.0)
check(all(near(a, c) for a, c in zip(b, (1.156, 0.0, -0.225))), "0.3 m DOWN in the image -> base z = 0.075 - 0.3", b)
b = lp.optical_to_base((0.0, 0.0, 1.0), CAM, 90.0)
check(all(near(a, c) for a, c in zip(b, (0.156, 0.0, -0.925))), "camera pitched 90 deg nose down: the axis point is straight below the camera", b)
b = lp.optical_to_base((0.0, 0.0, 1.0), CAM, 30.0)
check(near(b[0], 0.156 + math.cos(math.radians(30))) and near(b[2], 0.075 - math.sin(math.radians(30))),
      "30 deg nose down: x = 0.156 + cos 30, z = 0.075 - sin 30 (the point is lower than the camera)", b)
check(lp.parse_xyz("0.026 -0.084 0.075") == (0.026, -0.084, 0.075) and lp.parse_xyz("1,2,3") == (1.0, 2.0, 3.0),
      "parse_xyz reads 'x y z' and 'x,y,z'")
try:
    lp.parse_xyz("1 2")
    check(False, "parse_xyz refuses two numbers")
except ValueError:
    check(True, "parse_xyz refuses two numbers")

print("B. the depth under a box")
depth = np.full((100, 100), 1000, dtype=np.uint16)
depth[45:55, 45:55] = 0                                    # a hole in the middle
depth[0:10, :] = 3000                                      # far edges the central third must not see
check(near(lp.box_depth_m(depth, (20, 20, 80, 80), 0.001), 1.0), "a 60x60 box over 1000 mm with a hole in the middle -> 1.000 m")
depth2 = np.zeros((100, 100), dtype=np.uint16)
check(lp.box_depth_m(depth2, (20, 20, 80, 80), 0.001) is None, "no depth under the box -> None")
check(lp.box_depth_m(depth, (200, 200, 260, 260), 0.001) is None, "a box outside the image -> None")
depth3 = np.full((100, 100), 2000, dtype=np.uint16)
depth3[30:70, 30:70] = 800                                 # the object fills the central third, the edges are floor
check(near(lp.box_depth_m(depth3, (10, 10, 90, 90), 0.001), 0.8), "edges at 2 m, central third at 0.8 m -> 0.800 m (the object, not the floor)")
check(lp.box_centre((10, 20, 30, 60)) == (20.0, 40.0), "box centre")

print("C. the summary")
rows = []
hits = [1, 2, 4, 5, 6, 8, 9, 10]                           # frames 3 and 7 saw nothing
for k, i in enumerate(hits):                               # x alternates 1.03 / 1.01: median 1.02, spread 0.01
    rows.append({"frame": i, "t_s": i * 0.2, "latency_ms": 180 + i, "cls": "Metal", "conf": 0.8,
                 "u1": 0, "v1": 0, "u2": 10, "v2": 10, "depth_m": 1.0,
                 "x_m": 1.02 + (0.01 if k % 2 == 0 else -0.01), "y_m": 0.00, "z_m": -0.07})
rows.append({"frame": 5, "t_s": 1.0, "latency_ms": 185, "cls": "Plastic", "conf": 0.55,
             "u1": 0, "v1": 0, "u2": 10, "v2": 10, "depth_m": None, "x_m": None, "y_m": None, "z_m": None})
s = lp.summarise(rows, 10, expect=(1.0, 0.0))
check(s["frames"] == 10 and s["hit_frames"] == 8 and near(s["hit_rate"], 0.8), "10 frames, 8 with a located detection -> hit rate 80 %", s)
m = s["classes"]["Metal"]
check(m["n"] == 8 and near(m["x"], 1.02, 1e-9) and near(m["y"], 0.0) and near(m["sx"], 0.01, 1e-9),
      "Metal: 8 boxes, median x 1.02, spread 0.01", m)
check(s["classes"]["Plastic"]["x"] is None and s["classes"]["Plastic"]["no_depth"] == 1, "a box without depth counts, with no position")
check(near(s["error"]["dx"], 0.02, 1e-9) and near(s["error"]["dy"], 0.0) and near(s["error"]["dist"], 0.02, 1e-9),
      "against the tape at (1.00, 0.00): the most seen class is 2 cm off", s["error"])
check(s["latency_ms"] is not None and 180 < s["latency_ms"] < 191, "median latency of the rows")
lines = lp.summary_lines(s, (1.0, 0.0))
check(lines[0].startswith("frames 10, with a detection 8 (hit rate 80 %)") and any("Metal: 8 boxes" in l for l in lines)
      and any("2.0 cm off" in l for l in lines), "the printed summary says the hit rate, the class and the error", lines)
e = lp.summarise([], 5, expect=(1.0, 0.0))
check(e["hit_rate"] == 0.0 and e["error"] is None and lp.summary_lines(e, (1.0, 0.0))[-1].endswith("nothing to compare"),
      "no detection at all: hit rate 0, nothing to compare")
check(lp.summarise([], 0)["hit_rate"] == 0.0, "zero frames do not divide by zero")

print("D. the whole tool on the fake camera")
with tempfile.TemporaryDirectory(prefix="moss_litter_probe_") as td:
    out = Path(td) / "probe.csv"
    env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1", "MOSS_STATE_DIR": td}
    for k in ("MOSS_CAMERA_XYZ_M", "MOSS_CAMERA_PITCH_DEG"):
        env.pop(k, None)
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "moss_litter_probe.py"), "--fake", "--frames", "6",
                        "--seconds", "0", "--out", str(out), "--expect", "1.156", "-0.100"],
                       capture_output=True, text=True, timeout=120, env=env)
    check(r.returncode == 0, "exit 0", r.stderr[-400:])
    rows = list(csv.DictReader(out.open())) if out.is_file() else []
    check(len(rows) == 6 and sorted({int(x["frame"]) for x in rows}) == [1, 2, 3, 4, 5, 6], "6 frames -> 6 rows, one Metal each", len(rows))
    if rows:
        x, y, z, d = (float(rows[0][k]) for k in ("x_m", "y_m", "z_m", "depth_m"))
        check(near(d, 1.0, 1e-3) and near(x, 1.156, 2e-3) and near(y, -0.100, 2e-3) and near(z, -0.075, 2e-3),
              "the fake object (40 px right, 60 px down, 1 m) lands at base (1.156, -0.100, -0.075): depth survives the hole",
              (x, y, z, d))
        check(rows[0]["cls"] == "Metal" and near(float(rows[0]["conf"]), 0.83, 1e-6), "class and confidence from the detector")
    check("hit rate 100 %" in r.stdout and "Metal: 6 boxes" in r.stdout and "0.0 cm off" in r.stdout and "csv " in r.stdout,
          "the summary: hit rate 100 %, Metal 6 boxes, 0.0 cm off the expected point, the csv path", r.stdout[-500:])
    r2 = subprocess.run([sys.executable, str(ROOT / "tools" / "moss_litter_probe.py"), "--fake", "--frames", "2", "--seconds", "0",
                         "--out", str(Path(td) / "p2.csv"), "--camera-xyz", "0.026 -0.084 0.075", "--pitch-deg", "0"],
                        capture_output=True, text=True, timeout=120, env=env)
    rows2 = list(csv.DictReader((Path(td) / "p2.csv").open())) if (Path(td) / "p2.csv").is_file() else []
    check(r2.returncode == 0 and rows2 and near(float(rows2[0]["x_m"]), 1.026, 2e-3) and near(float(rows2[0]["y_m"]), -0.184, 2e-3),
          "--camera-xyz moves the answer with the camera: (0.026, -0.084, 0.075) -> x 1.026, y -0.184",
          rows2[:1] if rows2 else r2.stderr[-300:])
    r3 = subprocess.run([sys.executable, str(ROOT / "tools" / "moss_litter_probe.py"), "--fake", "--frames", "0", "--seconds", "0"],
                        capture_output=True, text=True, timeout=60, env=env)
    check(r3.returncode == 2 and "nothing to do" in r3.stderr, "no frames and no seconds -> exit 2 with a reason")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
