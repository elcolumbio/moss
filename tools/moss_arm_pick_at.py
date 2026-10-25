#!/usr/bin/env python3
"""A pick-and-drop cycle aimed at a point on the floor: the arm goes to the litter,
the base does not. From the floor-reach table (tools/moss_arm_reach.py), the pose
whose jaws sit on the point and a pose above it become `pick` and `travel`/`lift`
of a copy of config/arm_drop_poses.json; the rest of the cycle (clear, drop, the
return) is the delivered one. The file is then checked like any cycle:

    python3 tools/moss_arm_pick_at.py 0.30 -0.05                       # base_link metres (x forward, y left)
    python3 tools/moss_arm_drop.py plan --poses config/arm_pick_at.json --mujoco auto
    .venv/bin/python tools/moss_arm_direct.py --poses config/arm_pick_at.json --pick-pose pick   # on the robot

Nothing here moves the arm. The vias of the delivered approach and lift are
dropped (they belonged to the old poses); `plan --suggest` proposes new ones
when a straight move is refused.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import moss_arm_reach as R  # noqa: E402

POSES_IN = ROOT / "config" / "arm_drop_poses.json"
POSES_OUT = ROOT / "config" / "arm_pick_at.json"
ABOVE_DZ_M = (0.05, 0.12)       # the pre-pick pose: this much higher than the pick, same spot
ABOVE_XY_TOL_M = 0.03           # ... within this of the pick's floor point
PAN_STEP_TOL = 1                # ... and the same pan grid cell (or its neighbour)


def pick_poses(doc: dict, x: float, y: float):
    """(floor pose, above pose, floor error m) for a base_link point, or (None, None, error)."""
    f, err = R.nearest(doc, x, y)
    fx, fy = f["xy_base_m"]
    fz = f["tcp_chassis_m"][2]
    best = None
    for a in doc.get("above", []):
        if abs(a["idx"][0] - f["idx"][0]) > PAN_STEP_TOL:
            continue
        dz = a["tcp_chassis_m"][2] - fz
        if not ABOVE_DZ_M[0] <= dz <= ABOVE_DZ_M[1]:
            continue
        dxy = math.hypot(a["xy_base_m"][0] - fx, a["xy_base_m"][1] - fy)
        if dxy > ABOVE_XY_TOL_M:
            continue
        # prefer the pose closest in joint space to the pick (a short, straight approach)
        dj = max(abs(p - q) for p, q in zip(a["ticks"], f["ticks"]))
        score = (dj, dxy)
        if best is None or score < best[0]:
            best = (score, a)
    return f, (best[1] if best else None), err


def write_cycle(f: dict, a: dict, out: Path = POSES_OUT, src: Path = POSES_IN) -> dict:
    doc = json.loads(src.read_text())
    def pose(p, note):
        return {"rad": p["rad"], "ticks": p["ticks"], "tcp_m": p["tcp_chassis_m"], "note": note}
    doc["poses"]["pick"] = pose(f, f"aimed pick: jaws {f['jaw_floor_m'] * 1000:.0f} mm above the floor at base_link {f['xy_base_m']} (tools/moss_arm_pick_at.py)")
    doc["poses"]["pick_low"] = doc["poses"]["pick"]
    doc["poses"]["travel"] = pose(a, f"above the aimed pick: jaws {a['jaw_floor_m'] * 1000:.0f} mm above the floor")
    doc["poses"]["lift"] = doc["poses"]["travel"]
    for m in doc["sequence"]:
        if m["phase"] in ("leave_drive", "approach", "lift", "clear"):
            m["via"] = []
    doc["about"] = "A pick-and-drop cycle aimed at one point on the floor (tools/moss_arm_pick_at.py): pick and travel/lift come from the floor-reach table, the rest is the delivered cycle. " + doc.get("about", "")
    out.write_text(json.dumps(doc, indent=1))
    return doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("--chassis", action="store_true", help="the point is in the chassis frame")
    ap.add_argument("--out", default=str(POSES_OUT))
    a = ap.parse_args(argv)
    if not R.OUT_PATH.is_file():
        print(f"no {R.OUT_PATH.name}: run tools/moss_arm_reach.py scan first", file=sys.stderr)
        return 2
    doc = json.load(open(R.OUT_PATH))
    if not doc.get("above"):
        print("the reach table has no poses above the floor: rerun tools/moss_arm_reach.py scan", file=sys.stderr)
        return 2
    x, y = (a.x - R.SCANNER_XY_CHASSIS[0], a.y - R.SCANNER_XY_CHASSIS[1]) if a.chassis else (a.x, a.y)
    f, ab, err = pick_poses(doc, x, y)
    print(f"target base_link ({x:+.3f}, {y:+.3f}): pick pose {err * 100:.1f} cm off at {f['xy_base_m']}, "
          f"jaws {f['jaw_floor_m'] * 1000:.0f} mm, tilt {f['tilt_deg']} deg, ticks {f['ticks']}")
    if err > 0.04:
        print(f"REFUSED: {err * 100:.1f} cm from the nearest floor pose: the point is out of reach; move the base by about "
              f"{x - f['xy_base_m'][0]:+.2f} m forward / {y - f['xy_base_m'][1]:+.2f} m left first", file=sys.stderr)
        return 1
    if ab is None:
        print("REFUSED: no pose above that pick (same pan, 5-12 cm higher, within 3 cm)", file=sys.stderr)
        return 1
    print(f"above: jaws {ab['jaw_floor_m'] * 1000:.0f} mm at {ab['xy_base_m']}, ticks {ab['ticks']}")
    write_cycle(f, ab, Path(a.out))
    print(f"cycle written -> {a.out}; check it: python3 tools/moss_arm_drop.py plan --poses {a.out} --mujoco auto")
    return 0


if __name__ == "__main__":
    sys.exit(main())
