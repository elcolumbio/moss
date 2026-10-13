"""Cold bench for tools/moss_arm_drop.py plan: the pick-and-drop sequence of
config/arm_drop_poses.json checked against the keep-out guard, with nothing
connected. numpy; mujoco for G (skipped without it, or without the CAD scene).

Known input -> known output, in ticks, degrees, millimetres and newton metres:

  A. the numbers: every pose's ticks are the table's joint_map on its angles
     (travel 1.357553 rad on the pan -> 2933 ticks; pick's pan 76.78 deg ->
     2922 ticks -> 76.77 deg, half a tick); the gripper: 90 mm -> 3674 ticks,
     53 mm -> 2780, 71.5 mm -> 3227; the quintic: 0 at 0, 0.5 at 0.5, 1 at 1,
     its peak speed 1.875 times the mean at the middle; a move cut into
     samples a quarter of a grid step apart at most, and the same move the
     other way is exactly its samples reversed
  B. the order: drive, travel, pick (the gripper closes), lift, clear, drop
     (the gripper opens), clear, drive; the pan from clear to drop turns
     through 0 (the angles are never wrapped): every sample of the transfer
     has its pan between clear's and drop's, in order
  C. plan on the sequence as configured (the proposal applied), clearance 1:
     exit 0, every move OK; checked again here on every sample, with the
     table's forbidden points grown by one grid step by this bench: each
     sample allowed and inside the calibrated ranges, each step between two
     samples through segment_allowed and a box check (every grid point
     between the two samples, each widened by 25 ticks, free)
  D. plan --as-delivered: refused (exit 1); at clearance 0 the delivered travel
     is refused on the arm's own base and the delivered drop on the bin, pick,
     lift and clear pass. The proposal reproduces: the smallest shift for the
     delivered drop at clearance 1 is the configured one (shoulder_lift -9,
     wrist_flex -10 deg)
  E. the way back is the way out reversed: the move from drop back to clear
     passes through exactly the samples of the move from clear to drop, in the
     other order
  F. a pose inside the rover (a forbidden grid point of the table whose
     neighbours are all forbidden) put in place of drop is refused by plan,
     and why() names the part; a speed over 400 ticks/s without clearance is
     refused; no calibration, no plan
  G. the CAD scene (MOSS_ARM_CAD, else ~/moss-*/cad/*/MOSS_R31.xml): every
     delivered pose's tool point lands on its tcp_m within 1 mm; plan --mujoco
     counts 0 contacts on the whole sequence; the poses to wait in (drive and
     clear) hold the tool point within 12 cm of the pan axis and load the
     shoulder with 0.2 N m or less, the pick (the arm reaching to the floor)
     more than 15 cm out and more than 0.5 N m; the process stays under 4 GB

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_arm_drop_cold.py
"""
import atexit
import copy
import io
import json
import math
import os
import resource
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import moss_arm_drop as D  # noqa: E402
import moss_arm_sweep as W  # noqa: E402
from moss_dimos.arm_guard import ArmGuard  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


_TMP = tempfile.TemporaryDirectory(prefix="arm_drop_")
atexit.register(_TMP.cleanup)
TMP = Path(_TMP.name)
TOL = 25
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# the drive pose of docs/arm.md, the travel of an SO-101 arm as its calibration reads (as the sweep's bench)
FIXTURE = {
    "shoulder_pan": {"id": 1, "range_min": 702, "range_max": 3394},
    "shoulder_lift": {"id": 2, "homing_offset": -1369, "range_min": 860, "range_max": 3236},
    "elbow_flex": {"id": 3, "range_min": 890, "range_max": 3206},
    "wrist_flex": {"id": 4, "range_min": 850, "range_max": 3202},
    "wrist_roll": {"id": 5, "range_min": 171, "range_max": 3925},
    "gripper": {"id": 6, "range_min": 1500, "range_max": 4095},
    "poses": {"drive": {"ticks": {"shoulder_pan": 2857, "shoulder_lift": 860, "elbow_flex": 3206, "wrist_flex": 868,
                                  "wrist_roll": 3221, "gripper": 3674}}},
}


def quiet(fn, *args, **kw):
    buf = io.StringIO()
    with redirect_stdout(buf):
        out = fn(*args, **kw)
    return out, buf.getvalue()


def plan(*extra):
    """tools/moss_arm_drop.py plan, its exit status and its text."""
    lines = []
    code = D.main(["plan", "--table", str(table_path), "--calib", str(calib_path), *extra], say=lines.append)
    return code, "\n".join(lines)


def grown(occ, steps):
    """The forbidden grid points grown by `steps` grid steps along each joint."""
    for _ in range(steps):
        g = occ.copy()
        for k in range(occ.ndim):
            lo = tuple(slice(0, -1) if i == k else slice(None) for i in range(occ.ndim))
            hi = tuple(slice(1, None) if i == k else slice(None) for i in range(occ.ndim))
            g[lo] |= occ[hi]
            g[hi] |= occ[lo]
        occ = g
    return occ


def touched(g, lo, hi):
    step = g[1] - g[0]
    flo, fhi = (lo - g[0]) / step, (hi - g[0]) / step
    if flo < -1e-6 or fhi > len(g) - 1 + 1e-6:
        return None
    a = int(round(flo)) if abs(flo - round(flo)) <= 1e-6 else int(math.floor(flo))
    b = int(round(fhi)) if abs(fhi - round(fhi)) <= 1e-6 else int(math.floor(fhi)) + 1
    return np.arange(a, b + 1)


def box_ok(guard, occ, a, b, drive):
    """Every grid point between two poses (ticks), each end widened by the
    tolerance unless it is the drive pose, is free in occ."""
    jm, idx = guard.joint_map, []
    pad = TOL * 2 * math.pi / 4096
    for j, g in zip(guard.joints, guard.grids):
        qa, qb = jm.ticks_to_rad(j, a[j]), jm.ticks_to_rad(j, b[j])
        pa = 0.0 if a == drive else pad
        pb = 0.0 if b == drive else pad
        t = touched(np.asarray(g), min(qa - pa, qb - pb), max(qa + pa, qb + pb))
        if t is None:
            return False
        idx.append(t)
    return not occ[np.ix_(*idx)].any()


# ── inputs ──

table_path = Path(os.environ.get("MOSS_ARM_TABLE", ROOT / "config" / "arm_keepout.json"))
poses_path = ROOT / "config" / "arm_drop_poses.json"
calib_env = os.environ.get("MOSS_ARM_CALIB")
calib_path = Path(calib_env) if calib_env else ROOT / "config" / "arm_calibration.json"
if calib_path.is_file():
    CAL = json.loads(calib_path.read_text())
    cal_src = str(calib_path)
else:
    CAL = FIXTURE
    calib_path = TMP / "calibration.json"
    calib_path.write_text(json.dumps(FIXTURE))
    cal_src = "the fixture (the drive pose of docs/arm.md)"
GUARD = ArmGuard.load(table_path, calib_path)
DOC = json.loads(poses_path.read_text())
DRIVE = {j: int(CAL["poses"]["drive"]["ticks"][j]) for j in JOINTS}
print(f"table {table_path.name}: {GUARD.summary()}")
print(f"calibration: {cal_src}")

# ── A. the numbers ──
print("A. the numbers")
jm = GUARD.joint_map
bad = [(n, j) for n, p in DOC["poses"].items() for j, r, t in zip(JOINTS, p["rad"], p["ticks"]) if jm.rad_to_ticks(j, r) != t]
check("every pose's ticks are the table's joint_map on its angles", not bad and len(DOC["poses"]) == 5, str(bad))
t = jm.rad_to_ticks("shoulder_pan", DOC["poses"]["travel"]["rad"][0])
check("travel's pan: 1.357553 rad -> 2933 ticks", abs(DOC["poses"]["travel"]["rad"][0] - 1.357553) < 1e-6 and t == 2933, str(t))
deg = math.degrees(DOC["poses"]["pick"]["rad"][0])
back = math.degrees(jm.ticks_to_rad("shoulder_pan", jm.rad_to_ticks("shoulder_pan", math.radians(deg))))
check("pick's pan: 76.78 deg -> 2922 ticks -> back within half a tick (0.044 deg)",
      jm.rad_to_ticks("shoulder_pan", math.radians(deg)) == 2922 and abs(back - deg) <= 360 / 4096 / 2,
      f"{deg:.2f} -> {back:.3f} deg")
G = DOC["gripper"]
gt = [D.gap_to_ticks(G, mm) for mm in (90, 53, 71.5)]
check("gripper: 90 mm -> 3674 ticks, 53 mm -> 2780, 71.5 mm -> 3227", gt == [3674, 2780, 3227], str(gt))
du = 1e-6
peak = (D.quintic(0.5 + du) - D.quintic(0.5 - du)) / (2 * du)
check("quintic: 0, 0.5, 1 at 0, 0.5, 1; its peak 1.875 times the mean speed at the middle",
      D.quintic(0) == 0 and abs(D.quintic(0.5) - 0.5) < 1e-12 and D.quintic(1) == 1 and abs(peak - 1.875) < 1e-6,
      f"{peak:.6f}")
STEP = {j: abs(g[1] - g[0]) * 4096 / (2 * math.pi) for j, g in zip(GUARD.joints, GUARD.grids)}
a, b = {j: 1000 for j in JOINTS}, {"shoulder_pan": 3000, "shoulder_lift": 1001, "elbow_flex": 1500, "wrist_flex": 999, "wrist_roll": 1000}
s = D.samples(a, b, JOINTS, STEP)
gaps = max(abs(q[j] - p[j]) / STEP[j] for p, q in zip(s, s[1:]) for j in JOINTS)
check("a move of 2000 ticks on the pan cut a quarter of a grid step apart at most, both ends kept",
      s[0] == a and s[-1] == b and gaps <= 0.25 + 1e-9 and len(s) == math.ceil(2000 / (STEP["shoulder_pan"] // 4)) + 1,
      f"{len(s)} samples, largest {gaps:.3f} grid step")
check("the same move the other way: exactly its samples reversed", D.samples(b, a, JOINTS, STEP) == s[::-1])

# ── B. the order ──
print("B. the order")
moves = D.build(DOC, jm, JOINTS, DRIVE)
order = [moves[0].a.name] + [m.b.name for m in moves]
named = [n for n in order if not n.startswith("via")]
dedup = [n for i, n in enumerate(named) if i == 0 or n != named[i - 1]]
check("drive, travel, pick, lift, clear, drop, clear, drive", dedup == ["drive", "travel", "pick", "lift", "clear", "drop", "clear", "drive"],
      " ".join(dedup))
grip = [(m.phase, m.gap_mm) for m in moves if m.gap_mm[0] != m.gap_mm[1]]
check("the gripper closes at pick (90 -> 53 mm) and opens at drop (53 -> 90 mm), nowhere else",
      grip == [("grasp", (90.0, 53.0)), ("release", (53.0, 90.0))]
      and [m.a.name for m in moves if m.gap_mm[0] != m.gap_mm[1]] == ["pick", "drop"], str(grip))
held = [m.phase for m in moves if m.holds_can]
check("the can is held from the lift to the release", sorted(set(held)) == ["clear", "lift", "release", "transfer"], str(sorted(set(held))))
tr = [m for m in moves if m.phase == "transfer"]
pans = [p["shoulder_pan"] for m in tr for p in D.samples(m.a.ticks, m.b.ticks, JOINTS, STEP)]
c_pan, d_pan = tr[0].a.ticks["shoulder_pan"], tr[-1].b.ticks["shoulder_pan"]
check("clear -> drop: the pan turns through 0 deg (2048 ticks), never wrapped: every sample between clear's and drop's, in order",
      all(x >= y for x, y in zip(pans, pans[1:])) and pans[0] == c_pan and pans[-1] == d_pan and c_pan > 2048 > d_pan,
      f"{math.degrees(jm.ticks_to_rad('shoulder_pan', c_pan)):+.1f} -> {math.degrees(jm.ticks_to_rad('shoulder_pan', d_pan)):+.1f} deg, {len(pans)} samples")

# ── C. the plan ──
print("C. plan, clearance 1")
code, text = plan()
check("plan: exit 0, every move OK", code == 0 and "every move passes" in text and "REFUSED" not in text,
      [ln for ln in text.splitlines() if ln.startswith("sequence:")][0][:120])
OCC1 = grown(GUARD.occ, 1)
G1 = copy.copy(GUARD)
G1.occ = OCC1
pts = []
for m in moves:
    pts.extend((m.phase, p) for p in D.samples(m.a.ticks, m.b.ticks, JOINTS, STEP))
check("every sample allowed with the forbidden points grown by one step (by this bench)",
      all(G1.allowed_ticks(p) for _, p in pts), f"{len(pts)} samples")
check("every sample inside the calibrated ranges",
      all(CAL[j]["range_min"] <= p[j] <= CAL[j]["range_max"] for _, p in pts for j in JOINTS))
steps = [(p, q) for m in moves for p, q in zip(*(lambda s: (s, s[1:]))(D.samples(m.a.ticks, m.b.ticks, JOINTS, STEP)))]
check("every step between two samples: segment_allowed", all(G1.segment_allowed(G1.ticks_pose_to_rad(p), G1.ticks_pose_to_rad(q)) for p, q in steps),
      f"{len(steps)} steps")
check("every step between two samples: the box, both ends widened by 25 ticks, holds free grid points only",
      all(box_ok(GUARD, OCC1, p, q, DRIVE) for p, q in steps))
check("no joint moves more than a quarter of a grid step between two samples",
      max(abs(q[j] - p[j]) / STEP[j] for p, q in steps for j in JOINTS) <= 0.25 + 1e-9)

# ── D. as delivered ──
print("D. the poses as delivered")
code, text = plan("--as-delivered")
check("--as-delivered, clearance 1: refused (exit 1)", code == 1 and "REFUSED" in text,
      [ln for ln in text.splitlines() if ln.startswith("sequence:")][0][:100])
delivered = {n: dict(zip(JOINTS, p["ticks"])) for n, p in DOC["poses"].items()}
verdict = {n: GUARD.allowed_ticks(t) for n, t in delivered.items()}
check("clearance 0: travel and drop refused, pick, lift and clear allowed",
      verdict == {"travel": False, "pick": True, "lift": True, "clear": True, "drop": False}, str(verdict))
check("travel touches the arm's own base, drop the bin",
      "SO101 base link" in GUARD.why_ticks(delivered["travel"]) and "bin" in GUARD.why_ticks(delivered["drop"]),
      GUARD.why_ticks(delivered["drop"])[:80])
C1 = D.Checker(GUARD, CAL, TOL, 1)
sh = D.suggest_pose(C1, delivered["drop"])
check("the smallest shift for the delivered drop at clearance 1 is the configured one",
      sh == {"shoulder_lift": -9, "wrist_flex": -10} == DOC["poses"]["drop"]["shift_deg"], str(sh))

# ── E. the way back ──
print("E. the way back")
out = [p for m in moves if m.phase == "transfer" for p in D.samples(m.a.ticks, m.b.ticks, JOINTS, STEP)]
back = [p for m in moves if m.phase == "back_to_clear" for p in D.samples(m.a.ticks, m.b.ticks, JOINTS, STEP)]
check("drop -> clear passes through exactly the samples of clear -> drop, in the other order",
      back == out[::-1] and len(out) > 20, f"{len(out)} samples")

# ── F. refusals ──
print("F. refusals")
occ = GUARD.occ
deep = None
for c in np.argwhere(occ):
    sl = tuple(slice(max(x - 1, 0), x + 2) for x in c)
    if occ[sl].all() and all(0 < x < n - 1 for x, n in zip(c, occ.shape)):
        q = {j: float(g[c[k]]) for k, (j, g) in enumerate(zip(GUARD.joints, GUARD.grids))}
        t = jm.pose_to_ticks(q)
        if all(CAL[j]["range_min"] <= t[j] <= CAL[j]["range_max"] for j in JOINTS):
            deep = q
            break
bad_doc = json.loads(json.dumps(DOC))
bad_doc["poses"]["drop"]["rad"] = [deep[j] for j in JOINTS]
bad_doc["poses"]["drop"].pop("shift_deg", None)
for e in bad_doc["sequence"]:
    if e["to"] == "drop" or e["from"] == "drop":
        e.pop("via", None)
bad_path = TMP / "inside.json"
bad_path.write_text(json.dumps(bad_doc))
code, text = plan("--poses", str(bad_path))
part = GUARD.why_ticks(jm.pose_to_ticks(deep))
name = part.split("touches ", 1)[-1].split(" (also")[0].split(" at the grid point")[0]
at_drop = [ln for ln in text.splitlines() if ln.startswith("release: at drop")]
check("a drop inside the rover: refused, and at the drop pose why() names the part",
      code == 1 and part.startswith("forbidden") and at_drop and "REFUSED" in at_drop[0] and name in at_drop[0],
      name[:90])
code, text = plan("--speed", "900", "--clearance", "0")
check("--speed 900 without clearance: refused", code == 1 and "needs --clearance" in text)
code, text = D.main(["plan", "--table", str(table_path), "--calib", str(TMP / "none.json")], say=lambda *_: None), ""
check("no calibration: no plan (exit 2)", code == 2)

# ── G. the CAD scene ──
print("G. the CAD scene")
try:
    import mujoco
except ImportError:
    mujoco = None
cad = D.find_cad("auto")
if mujoco is None:
    print("  -- skipped: mujoco not installed")
elif cad is None or not cad.is_file():
    print("  -- skipped, no CAD scene here (MOSS_ARM_CAD, or ~/moss-*/cad/*/MOSS_R31.xml)")
else:
    print(f"  model {cad.name}")
    import gc
    import moss_arm_keepout as K
    K._lean_malloc()            # a dropped model's memory goes back to the system
    m = mujoco.MjModel.from_xml_path(str(cad))
    d = mujoco.MjData(m)
    tool = m.body(D.TOOL_BODY).id
    err = {}
    for n, p in DOC["poses"].items():
        for j, r in zip(JOINTS, p["rad"]):
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = r
        mujoco.mj_kinematics(m, d)
        tcp = d.xpos[tool] + d.xmat[tool].reshape(3, 3) @ np.array(D.TCP_IN_TOOL_M)
        err[n] = float(np.linalg.norm(tcp - np.array(p["tcp_m"])))
    del m, d
    gc.collect()
    K._trim()
    check("every delivered pose's tool point lands on its tcp_m within 1 mm",
          max(err.values()) < 0.001, ", ".join(f"{n} {1000 * e:.2f}" for n, e in err.items()) + " mm")
    c = D.Checker(GUARD, CAL, TOL, 1)
    scene, _ = quiet(D.scene_check, c, moves, cad)
    total = sum(r["arm"] + r["shoulder"] + r["can"] + r["errors"] for r in scene.contacts)
    n_poses = sum(r["samples"] for r in scene.contacts)
    check("plan --mujoco: 0 contacts on the whole sequence (the arm, the shoulder, the can while held)",
          total == 0 and n_poses > 500, f"{n_poses} poses")
    rest = {n: scene.poses[n] for n in DOC["rest"]}
    check("the poses to wait in (drive, clear): the tool point within 12 cm of the pan axis",
          all(p["axis_m"] <= 0.12 for p in rest.values()), ", ".join(f"{n} {100 * p['axis_m']:.1f} cm" for n, p in rest.items()))
    check("and the shoulder's static load 0.2 N m or less",
          all(abs(p["shoulder_nm"]) <= 0.2 for p in rest.values()), ", ".join(f"{n} {p['shoulder_nm']:+.2f}" for n, p in rest.items()))
    pk = scene.poses["pick"]
    check("the pick (reaching to the floor): more than 15 cm out, more than 0.5 N m",
          pk["axis_m"] > 0.15 and pk["shoulder_nm"] > 0.5, f"{100 * pk['axis_m']:.1f} cm, {pk['shoulder_nm']:+.2f} N m")
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    check("one process, under 4 GB", rss < 4096, f"{rss:.0f} MB at the peak")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
_TMP.cleanup()
sys.exit(1 if KO else 0)
