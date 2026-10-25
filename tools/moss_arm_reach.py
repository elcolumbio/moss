#!/usr/bin/env python3
"""The floor the gripper reaches: every allowed pose of the keep-out table posed in
the CAD scene, kept when the gripper points down and its jaws sit just above the
floor. The result is a table of floor points the arm can pick at, in base_link
(x forward, y left, the lidar's floor projection) and in the chassis frame, with
the pose (ticks) for each; and a lookup: the pose nearest to a point on the floor.

    python3 tools/moss_arm_reach.py scan                       # writes config/arm_floor_reach.json, prints the envelope
    python3 tools/moss_arm_reach.py near 0.40 -0.11            # the nearest floor pose to base_link (x, y), and its error
    python3 tools/moss_arm_reach.py near 0.40 -0.11 --chassis  # the point given in the chassis frame instead

Needs mujoco and the CAD scene (MOSS_ARM_CAD, else ~/moss-*/cad/*/MOSS_R31.xml).
The jaws' lowest point is taken 20 mm below the tool point when the gripper is
vertical (pick: tool 82 mm, jaws 62 mm; pick_low: tool 41 mm, jaws 21 mm).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos.arm_guard import ArmGuard, TABLE_PATH  # noqa: E402

OUT_PATH = ROOT / "config" / "arm_floor_reach.json"
TOOL_BODY = "nc90_tool"
TCP_IN_TOOL_M = (0.0, 0.0, -0.035)     # as tools/moss_arm_drop.py
JAWS_BELOW_TCP_M = 0.020               # the jaws' lowest point under the tool point, gripper vertical
TICKS_PER_REV = 4096
SCANNER_XY_CHASSIS = (0.130, 0.084)    # the lidar's axis in the chassis frame (CAD R31): base_link = chassis - this
TILT_MAX_DEG = 25.0                    # the gripper within this of vertical
JAW_MIN_M, JAW_MAX_M = 0.008, 0.040    # the jaws' lowest point above the floor
ABOVE_MIN_M, ABOVE_MAX_M = 0.050, 0.140  # the jaws' lowest point for a pose ABOVE a floor point (the pre-pick / post-pick)


def find_cad() -> Path | None:
    env = os.environ.get("MOSS_ARM_CAD")
    if env and Path(env).is_file():
        return Path(env)
    hits = sorted(glob.glob(os.path.expanduser("~/moss-*/cad/*/MOSS_R31.xml")))
    return Path(hits[-1]) if hits else None


def rad_to_ticks(rad: float, jm: dict) -> int:
    return int(round(jm["model_zero_ticks"] + jm["sign"] * rad * TICKS_PER_REV / (2.0 * math.pi)))


def scan(out: Path = OUT_PATH, say=print) -> dict:
    import mujoco

    cad = find_cad()
    if cad is None:
        raise SystemExit("no CAD scene (MOSS_ARM_CAD, or ~/moss-*/cad/*/MOSS_R31.xml)")
    table = json.load(open(TABLE_PATH))
    guard = ArmGuard(table)
    joints = list(guard.joints)
    jmap = table["joint_map"]
    model = mujoco.MjModel.from_xml_path(str(cad))
    data = mujoco.MjData(model)
    jid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
    if min(jid) < 0:
        raise SystemExit(f"joint missing in {cad.name}: {dict(zip(joints, jid))}")
    adr = [int(model.jnt_qposadr[j]) for j in jid]
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)
    if tool < 0:
        raise SystemExit(f"body {TOOL_BODY} missing in {cad.name}")
    base_qpos = data.qpos.copy()
    grids = guard.grids
    free = np.argwhere(~guard.occ)
    say(f"{cad.name}: {len(free)} allowed cells of {guard.occ.size}, scanning")
    t0 = time.monotonic()
    rows, above = [], []
    tcp_off = np.array(TCP_IN_TOOL_M)
    for idx in free:
        q = [float(g[i]) for g, i in zip(grids, idx)]
        data.qpos[:] = base_qpos
        for a, v in zip(adr, q):
            data.qpos[a] = v
        mujoco.mj_kinematics(model, data)
        R = data.xmat[tool].reshape(3, 3)
        tcp = data.xpos[tool] + R @ tcp_off
        # the approach runs along -z of the tool frame: vertical when the tool's z axis points up
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        if tilt > TILT_MAX_DEG:
            continue
        jaw = float(tcp[2]) - JAWS_BELOW_TCP_M
        if JAW_MIN_M <= jaw <= JAW_MAX_M:
            dest = rows
        elif ABOVE_MIN_M <= jaw <= ABOVE_MAX_M:
            dest = above
        else:
            continue
        dest.append({"idx": [int(i) for i in idx],
                     "rad": [round(v, 6) for v in q],
                     "ticks": [rad_to_ticks(v, jmap[j]) for v, j in zip(q, joints)],
                     "tcp_chassis_m": [round(float(x), 4) for x in tcp],
                     "xy_base_m": [round(float(tcp[0]) - SCANNER_XY_CHASSIS[0], 4), round(float(tcp[1]) - SCANNER_XY_CHASSIS[1], 4)],
                     "jaw_floor_m": round(jaw, 4), "tilt_deg": round(tilt, 1)})
    dt = time.monotonic() - t0
    doc = {"format": "moss-arm-floor-reach/1", "model": {"path": cad.name}, "table": TABLE_PATH.name,
           "joints": joints, "scanner_xy_chassis_m": list(SCANNER_XY_CHASSIS),
           "rule": {"tilt_max_deg": TILT_MAX_DEG, "jaw_floor_m": [JAW_MIN_M, JAW_MAX_M], "jaws_below_tcp_m": JAWS_BELOW_TCP_M},
           "scanned_cells": int(len(free)), "poses": rows, "above": above, "seconds": round(dt, 1),
           "above_rule": {"jaw_floor_m": [ABOVE_MIN_M, ABOVE_MAX_M]}}
    out.write_text(json.dumps(doc, indent=0))
    say(f"{len(rows)} floor poses and {len(above)} poses above the floor in {dt:.1f} s -> {out}")
    envelope(doc, say)
    return doc


def envelope(doc: dict, say=print) -> None:
    pts = np.array([p["xy_base_m"] for p in doc["poses"]]) if doc["poses"] else np.zeros((0, 2))
    if not len(pts):
        say("no floor pose: the gripper never sits above the floor within the rule")
        return
    say(f"floor reach in base_link (x forward, y left, from the lidar's floor point): "
        f"x {pts[:, 0].min():+.3f}..{pts[:, 0].max():+.3f} m, y {pts[:, 1].min():+.3f}..{pts[:, 1].max():+.3f} m")
    for x0 in (0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50):
        band = pts[np.abs(pts[:, 0] - x0) < 0.025]
        if len(band):
            say(f"  at x {x0:.2f} m: {len(band):4d} poses, y {band[:, 1].min():+.3f}..{band[:, 1].max():+.3f}")
        else:
            say(f"  at x {x0:.2f} m: none")


PICK_LOW_TICKS = (2922, 2593, 1978, 2609, 1015)   # the family of poses the cycle uses: the arm forward over the nose
NEAR_BAND_M = 0.015                                 # among the poses this close to the nearest, the one nearest pick_low in joint space wins


def nearest(doc: dict, x: float, y: float, like=PICK_LOW_TICKS) -> tuple[dict, float]:
    """The floor pose nearest to (x, y) in base_link; ties within NEAR_BAND_M go to the
    pose closest to `like` in joint space (the same floor point can be reached from the
    other side of the pan, the arm bent backwards over the robot: legal, not wanted)."""
    pts = np.array([p["xy_base_m"] for p in doc["poses"]])
    d = np.hypot(pts[:, 0] - x, pts[:, 1] - y)
    best = float(d.min())
    cands = [k for k in range(len(d)) if d[k] <= best + NEAR_BAND_M]
    like = np.array(like, dtype=float)
    k = min(cands, key=lambda i: float(np.abs(np.array(doc["poses"][i]["ticks"], dtype=float) - like).max()))
    return doc["poses"][k], float(d[k])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    pn = sub.add_parser("near")
    pn.add_argument("x", type=float)
    pn.add_argument("y", type=float)
    pn.add_argument("--chassis", action="store_true", help="the point is in the chassis frame")
    a = ap.parse_args(argv)
    if a.cmd == "scan":
        scan()
        return 0
    if not OUT_PATH.is_file():
        print(f"no {OUT_PATH.name}: run `scan` first", file=sys.stderr)
        return 2
    doc = json.load(open(OUT_PATH))
    x, y = (a.x - SCANNER_XY_CHASSIS[0], a.y - SCANNER_XY_CHASSIS[1]) if a.chassis else (a.x, a.y)
    p, d = nearest(doc, x, y)
    print(f"target base_link ({x:+.3f}, {y:+.3f}) -> nearest floor pose {d * 100:.1f} cm away: "
          f"ticks {p['ticks']} (pan, lift, elbow, wrist_flex, roll), tool at base_link {p['xy_base_m']}, "
          f"jaws {p['jaw_floor_m'] * 1000:.0f} mm above the floor, tilt {p['tilt_deg']} deg")
    envelope(doc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
