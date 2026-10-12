"""Cold bench for tools/moss_arm_sweep.py: the arm's tour along its keep-out
zones, planned on the table and run on the fake bus. numpy only.

Known input -> known output, in ticks, radians and seconds:

  A. the speed model: 200 ticks at speed 200, acc 8 take 1.25 s (1.0 s at 200
     ticks/s and 0.25 s of ramp), 20 ticks 0.316 s (no cruise); the joint is
     within 25 ticks of its goal 0.25 s before the end; a fake servo told to
     move 200 ticks reads its goal at 1.215 s (within half a tick of it) and
     is within 25 ticks at 1.0 s; the
     drive pose's ticks -> model radians -> ticks round trip
     (pan 2857 -> +71.1 deg -> 2857)
  B. the plan on the table (config/arm_keepout.json, MOSS_ARM_TABLE to point
     elsewhere) with the calibration (config/arm_calibration.json when this
     checkout holds one, MOSS_ARM_CALIB, else a fixture: drive and look as
     docs/arm.md gives them, the travel of an SO-101). When the table forbids
     that drive pose, the refusal names the part and the tour runs from a
     stand-in: the free cell nearest the drive pose.
     - it starts and ends at the drive pose; every waypoint is allowed, in a
       cell whose 32 corners are free, inside the calibrated ranges by the
       tolerance; every move passes segment_allowed and a box check done here
       (every grid point between the two ends, each widened by the tolerance,
       free); no joint moves more than one cell per move
     - it hugs the boundary: most waypoints have a forbidden grid point within
       one grid step of their cell
     - it fits --duration within 20 %, never over (25 s and 40 s)
     - the same seed gives the same tour, another seed another
  C. refusals: a drive pose on a forbidden grid point (why() names the part),
     a drive pose outside the table, a tolerance wider than half a grid step,
     no calibration, a speed over the cap
  D. the dry run (--dry-run --fast): exit 0; t strictly increasing; rad equal
     to JointMap(ticks) on every line; every reading allowed or within the
     tolerance of the drive pose; the last within the tolerance of the drive
     pose; every waypoint in order; the gripper never written; the torque
     only switched on; the run's time within 20 % of the plan's and of
     --duration. --plan-only writes the same log and names the parts skirted.
  E. faults mid-tour: Ctrl-C, a joint blocked mid-move (the timeout), a
     waypoint the guard refuses slipped into the plan. The first goals after
     the fault are the arm's readings (it stops where it is), every later
     goal is reached by a move that passes segment_allowed and the box check,
     the arm ends within the tolerance of the drive pose (exit 2). A knock
     into a pose the guard refuses: no allowed move leaves it, the arm holds
     there (exit 3), nothing is written after the hold.
  F. an arm away from the drive pose is refused before anything is written.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_arm_sweep_cold.py
"""
import atexit
import io
import json
import math
import os
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

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


_TMP = tempfile.TemporaryDirectory(prefix="arm_sweep_")
atexit.register(_TMP.cleanup)
TMP = Path(_TMP.name)
TOL = 25
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# drive and look as docs/arm.md gives them, the travel of an SO-101 arm as its calibration reads
FIXTURE = {
    "shoulder_pan": {"id": 1, "range_min": 702, "range_max": 3394},
    "shoulder_lift": {"id": 2, "homing_offset": -1369, "range_min": 860, "range_max": 3236},
    "elbow_flex": {"id": 3, "range_min": 890, "range_max": 3206},
    "wrist_flex": {"id": 4, "range_min": 850, "range_max": 3202},
    "wrist_roll": {"id": 5, "range_min": 171, "range_max": 3925},
    "gripper": {"id": 6, "range_min": 1500, "range_max": 4095},
    "poses": {
        "drive": {"ticks": {"shoulder_pan": 2857, "shoulder_lift": 860, "elbow_flex": 3206, "wrist_flex": 868,
                            "wrist_roll": 3221, "gripper": 3674}},
        "look": {"ticks": {"shoulder_pan": 2869, "shoulder_lift": 1057, "elbow_flex": 3208, "wrist_flex": 2018,
                           "wrist_roll": 3220, "gripper": 3674}},
    },
}


def quiet(fn, *args, **kw):
    buf = io.StringIO()
    with redirect_stdout(buf):
        out = fn(*args, **kw)
    return out, buf.getvalue()


def write_json(name, doc) -> Path:
    path = TMP / name
    path.write_text(json.dumps(doc))
    return path


def touched(g, lo, hi):
    """Indices of the grid values [lo, hi] touches by the guard's rule (the
    value below lo and above hi unless they sit on one), None outside."""
    step = g[1] - g[0]
    flo, fhi = (lo - g[0]) / step, (hi - g[0]) / step
    if flo < -1e-6 or fhi > len(g) - 1 + 1e-6:
        return None
    a = int(round(flo)) if abs(flo - round(flo)) <= 1e-6 else int(math.floor(flo))
    b = int(round(fhi)) if abs(fhi - round(fhi)) <= 1e-6 else int(math.floor(fhi)) + 1
    return np.arange(a, b + 1)


def box_ok(guard, a, b, drive):
    """Every grid point between two poses (ticks), each end widened by the
    tolerance unless it is the drive pose, is free."""
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
    return not guard.occ[np.ix_(*idx)].any()


def cell_of(guard, ticks):
    q = guard.ticks_pose_to_rad(ticks)
    return tuple(min(int(math.floor((q[j] - g[0]) / (g[1] - g[0]))), len(g) - 2) for j, g in zip(guard.joints, guard.grids))


def corners_free(guard, c):
    return not guard.occ[tuple(slice(x, x + 2) for x in c)].any()


def near_forbidden(guard, c):
    return bool(guard.occ[tuple(slice(max(x - 1, 0), x + 3) for x in c)].any())


def stand_in_drive(guard, cal):
    """The centre of the free cell, inside the ranges, nearest the drive pose."""
    free = ~guard.occ
    cells = free
    for k in range(free.ndim):
        lo = tuple(slice(0, -1) if i == k else slice(None) for i in range(free.ndim))
        hi = tuple(slice(1, None) if i == k else slice(None) for i in range(free.ndim))
        cells = cells[lo] & cells[hi]
    q = guard.ticks_pose_to_rad(cal["poses"]["drive"]["ticks"])
    f = np.array([(q[j] - g[0]) / (g[1] - g[0]) - 0.5 for j, g in zip(guard.joints, guard.grids)])
    best, key = None, math.inf
    for c in np.argwhere(cells):
        ticks = {j: guard.joint_map.rad_to_ticks(j, (g[c[k]] + g[c[k] + 1]) / 2) for k, (j, g) in enumerate(zip(guard.joints, guard.grids))}
        if any(not (cal[j]["range_min"] + 2 * TOL <= ticks[j] <= cal[j]["range_max"] - 2 * TOL) for j in JOINTS):
            continue
        d = np.abs(c - f)
        k = d.max() * 1e3 + d.sum()
        if k < key:
            best, key = ticks, k
    return best


def read_log(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def run(argv, hook=None):
    """main() with the stdout caught: (exit status, the bus, the clock, the plan, the planner, the text)."""
    seen = {}

    def keep(bus, clk, plan, p):
        seen.update(bus=bus, clk=clk, plan=plan, p=p)
        if hook:
            hook(bus, clk, plan, p)

    code, text = quiet(W.main, argv, bus_hook=keep)
    return code, seen.get("bus"), seen.get("clk"), seen.get("plan"), seen.get("p"), text


def goal_writes(bus, after=-1.0):
    """(t, {servo: goal}) per complete write of the five goals, in order."""
    out, cur = [], {}
    for t, sid, addr, v in bus.writes:
        if addr != W.REG_GOAL or t <= after:
            continue
        cur[sid] = v
        if len(cur) == 5:
            out.append((t, dict(cur)))
            cur = {}
    return out


def by_joint(ids, goals):
    return {j: goals[s] for j, s in ids.items()}


def log_zero(bus):
    """The log's t = 0: the last torque write (the log opens right after it)."""
    return [t for t, _, a, _ in bus.writes if a == W.REG_TORQUE][-1]


def checked_move(a, b, p):
    """A move the bench checks itself: segment_allowed and the box (the last
    move from within the tolerance of the drive pose onto it excepted)."""
    if p.parked(a) and b == p.drive:
        return True
    qa, qb = GUARD.ticks_pose_to_rad(a), GUARD.ticks_pose_to_rad(b)
    return GUARD.segment_allowed(qa, qb) and box_ok(GUARD, a, b, p.drive)


# ── inputs ──

table_path = Path(os.environ.get("MOSS_ARM_TABLE", ROOT / "config" / "arm_keepout.json"))
calib_env = os.environ.get("MOSS_ARM_CALIB")
calib_path = Path(calib_env) if calib_env else ROOT / "config" / "arm_calibration.json"
if calib_path.is_file():
    CAL = json.loads(calib_path.read_text())
    cal_src = str(calib_path)
else:
    CAL = FIXTURE
    calib_path = write_json("calibration.json", FIXTURE)
    cal_src = "the fixture (drive and look of docs/arm.md)"
GUARD = ArmGuard.load(table_path, calib_path)
print(f"table {table_path.name}: {GUARD.summary()}")
print(f"calibration: {cal_src}")

# ── A. the speed model ──
print("A. the speed model")
M = W.SpeedModel(200, 8, TOL)
check("200 ticks at speed 200, acc 8: 1.25 s", abs(M.joint_time(200) - 1.25) < 1e-9, f"{M.joint_time(200):.4f} s")
check("20 ticks: 2 sqrt(20 / 800) = 0.316 s", abs(M.joint_time(20) - 2 * math.sqrt(20 / 800)) < 1e-9, f"{M.joint_time(20):.4f} s")
check("200 ticks: within 25 ticks 0.25 s before the end", abs(M.reach_time(200) - 1.0) < 1e-9, f"{M.reach_time(200):.4f} s")
check("a joint already within the tolerance takes no time", M.reach_time(20) == 0.0)
clk = W.VirtualClock()
bus = W.FakeBus({1: 1000}, clk)
bus.write1ByteTxRx(None, 1, W.REG_ACC, 8)
bus.write2ByteTxRx(None, 1, W.REG_SPEED, 200)
bus.write1ByteTxRx(None, 1, W.REG_TORQUE, 1)
t0 = clk.now()
bus.write2ByteTxRx(None, 1, W.REG_GOAL, 1200)
t_in, t_at = None, None
while clk.now() - t0 < 3.0:
    v = bus.read2ByteTxRx(None, 1, W.REG_PRESENT)[0]
    if t_in is None and abs(v - 1200) <= TOL:
        t_in = clk.now() - t0
    if v == 1200 and t_at is None:
        t_at = clk.now() - t0
    clk.sleep(0.005)
# it reads its goal once within half a tick of it: 1.25 - sqrt(2 * 0.5 / 800) = 1.215 s
check("a fake servo told to move 200 ticks reads its goal at 1.215 s (+-0.02)",
      t_at is not None and abs(t_at - (1.25 - math.sqrt(1.0 / 800))) < 0.02, f"{t_at:.3f} s")
check("and is within 25 ticks at 1.0 s (+-0.02)", t_in is not None and abs(t_in - 1.0) < 0.02, f"{t_in:.3f} s")
d = CAL["poses"]["drive"]["ticks"]
deg = math.degrees(GUARD.joint_map.ticks_to_rad("shoulder_pan", d["shoulder_pan"]))
back = GUARD.joint_map.rad_to_ticks("shoulder_pan", math.radians(deg))
check("drive's pan: ticks -> model degrees -> ticks", back == d["shoulder_pan"],
      f"{d['shoulder_pan']} -> {deg:+.1f} deg -> {back}")

# ── B. the plan ──
print("B. the plan on the table")
drive_q = GUARD.ticks_pose_to_rad(d)
if GUARD.allowed(drive_q):
    CAL_RUN, calib_run = CAL, calib_path
    print(f"  the table allows the calibration's drive pose: {GUARD.why(drive_q)}")
else:
    code, _, _, _, _, text = run(["--plan-only", "--table", str(table_path), "--calib", str(calib_path),
                                  "--log", str(TMP / "refused.jsonl")])
    part = GUARD.why(drive_q)
    check("the table forbids the calibration's drive pose: the plan is refused and why() names the part",
          code == 1 and "not allowed" in text and part in text, text.strip()[:160])
    stand = stand_in_drive(GUARD, CAL)
    CAL_RUN = json.loads(json.dumps(CAL))
    CAL_RUN["poses"]["drive"]["ticks"].update(stand)
    calib_run = write_json("calibration_stand_in.json", CAL_RUN)
    print(f"  the tour checks run from a stand-in drive pose: {stand}")
DRIVE = {j: int(CAL_RUN["poses"]["drive"]["ticks"][j]) for j in JOINTS}
P = W.Planner(GUARD, CAL_RUN, M, TOL)
plan = P.plan(25.0, seed=0)
wps = plan.waypoints
ticks = [w.ticks for w in wps]
check("it starts and ends at the drive pose", ticks[0] == DRIVE and ticks[-1] == DRIVE and len(wps) > 4,
      f"{len(wps)} waypoints")
inner = ticks[1:-1]
check("every waypoint is allowed by the guard", all(GUARD.allowed_ticks(t) for t in ticks))
check("every waypoint but the drive pose sits in a cell whose 32 corners are free",
      all(corners_free(GUARD, cell_of(GUARD, t)) for t in inner))
check("every waypoint is inside the calibrated ranges by the tolerance",
      all(CAL_RUN[j]["range_min"] + TOL <= t[j] <= CAL_RUN[j]["range_max"] - TOL for t in inner for j in JOINTS))
seg = [GUARD.segment_allowed(GUARD.ticks_pose_to_rad(a), GUARD.ticks_pose_to_rad(b)) for a, b in zip(ticks, ticks[1:])]
check("every move passes segment_allowed", all(seg), f"{len(seg)} moves")
boxes = [box_ok(GUARD, a, b, DRIVE) for a, b in zip(ticks, ticks[1:])]
check("every move's box (both ends widened by the tolerance) holds free grid points only", all(boxes))
steps = [np.abs(np.array(cell_of(GUARD, b)) - np.array(cell_of(GUARD, a))).max() for a, b in zip(ticks, ticks[1:])]
check("no joint moves more than one cell per move", max(steps) <= 1, f"largest {max(steps)}")
near = [near_forbidden(GUARD, cell_of(GUARD, t)) for t in inner]
check("it hugs the boundary: 60 % or more of the waypoints have a forbidden grid point within one step",
      np.mean(near) >= 0.6, f"{100 * np.mean(near):.0f} % ({sum(near)} of {len(near)})")
for dur in (25.0, 40.0):
    pl = plan if dur == 25.0 else P.plan(dur, seed=0)
    check(f"the plan fits --duration {dur:g} s within 20 %, never over",
          0.8 * dur <= pl.total_s <= dur + 1e-9, f"{pl.total_s:.2f} s, {len(pl.waypoints)} waypoints")
again = P.plan(25.0, seed=0)
other = P.plan(25.0, seed=7)
check("the same seed gives the same tour, another seed another",
      [w.ticks for w in again.waypoints] == ticks and [w.ticks for w in other.waypoints] != ticks)
skirted = [w.skirts for w in wps if w.skirts]
check("the frontier waypoints name the part they skirt, as why() does",
      skirted and all(w.skirts in (w.why or "") for w in wps if w.skirts), ", ".join(sorted(set(skirted)))[:160])

# ── C. refusals ──
print("C. refusals")
idx = np.argwhere(GUARD.occ)
mid = idx[len(idx) // 2]
q_bad = {j: float(g[mid[k]]) for k, (j, g) in enumerate(zip(GUARD.joints, GUARD.grids))}
bad = json.loads(json.dumps(CAL_RUN))
bad["poses"]["drive"]["ticks"].update(GUARD.joint_map.pose_to_ticks(q_bad))
code, _, _, _, _, text = run(["--plan-only", "--table", str(table_path), "--calib", str(write_json("bad.json", bad)),
                              "--log", str(TMP / "bad.jsonl")])
part = GUARD.why(GUARD.ticks_pose_to_rad(bad["poses"]["drive"]["ticks"]))
check("a drive pose on a forbidden grid point is refused, why() names the part",
      code == 1 and "not allowed" in text and "forbidden" in part and part in text and not (TMP / "bad.jsonl").exists(),
      text.strip()[:140])
out = json.loads(json.dumps(CAL_RUN))
out["poses"]["drive"]["ticks"]["wrist_roll"] = 4095
code, _, _, _, _, text = run(["--plan-only", "--table", str(table_path), "--calib", str(write_json("out.json", out))])
check("a drive pose outside the table is refused", code == 1 and "outside the table" in text, text.strip()[-90:])
try:
    W.Planner(GUARD, CAL_RUN, W.SpeedModel(200, 8, 120), 120)
    check("a tolerance wider than half a grid step is refused", False)
except W.Refused as e:
    check("a tolerance wider than half a grid step is refused", "half a grid step" in str(e), str(e)[:90])
code, _, _, _, _, text = run(["--plan-only", "--table", str(table_path), "--calib", str(TMP / "none.json")])
check("no calibration: refused", code == 1 and "no calibration" in text)
code, _, _, _, _, text = run(["--plan-only", "--speed", "900", "--table", str(table_path), "--calib", str(calib_run)])
check("a speed over the cap is refused", code == 1 and "--speed 900" in text)

# ── D. the dry run ──
print("D. the dry run")
BASE = ["--table", str(table_path), "--calib", str(calib_run)]
log = TMP / "dry.jsonl"
code, bus, clk, pl, p, text = run(BASE + ["--dry-run", "--fast", "--log", str(log)])
lines = read_log(log)
ts = [r["t"] for r in lines]
ids = W.servo_ids(p)
check("exit 0, back at the drive pose", code == 0 and "done" in text, text.strip().splitlines()[-1][:120])
check("t strictly increasing", all(b > a for a, b in zip(ts, ts[1:])) and ts[0] == 0.0, f"{len(lines)} readings")
err = max(abs(r["rad"][j] - GUARD.joint_map.ticks_to_rad(j, r["ticks"][j])) for r in lines for j in JOINTS)
check("rad = JointMap(ticks) on every line", err < 1e-12 and all(set(r["rad"]) == set(JOINTS) for r in lines), f"{err:.1e}")
check("every reading allowed by the guard or within the tolerance of the drive pose",
      all(GUARD.allowed_ticks(r["ticks"]) or P.parked(r["ticks"]) for r in lines))
check("the last reading within the tolerance of the drive pose", P.parked(lines[-1]["ticks"]), str(lines[-1]["ticks"]))
wseen = [r["waypoint"] for r in lines]
check("every waypoint in order", wseen == sorted(wseen) and set(wseen) == set(range(len(pl.waypoints))))
check("the gripper (servo 6) never written", not [w for w in bus.writes if w[1] == W.GRIPPER_ID])
torque = [(s, v) for _, s, a, v in bus.writes if a == W.REG_TORQUE]
check("the torque only switched on, servos 1-5", sorted(torque) == [(s, 1) for s in range(1, 6)], str(torque))
goals = [by_joint(ids, g) for _, g in goal_writes(bus)]
check("the goals written: the arm's own reading first (a hold), then the plan's waypoints",
      goals[0] == lines[0]["ticks"] and goals[1:] == [w.ticks for w in pl.waypoints[1:]])
check("the run's time within 20 % of the plan's and of --duration",
      abs(ts[-1] - pl.total_s) <= 0.2 * pl.total_s and abs(ts[-1] - 25.0) <= 5.0, f"run {ts[-1]:.2f} s, plan {pl.total_s:.2f} s")
plog = TMP / "plan.jsonl"
code, _, _, _, _, text = run(BASE + ["--plan-only", "--log", str(plog)])
plines = read_log(plog)
check("--plan-only: the tour printed with the parts skirted, the log written",
      code == 0 and "skirts" in text and "planned log" in text and len(plines) == len(lines)
      and all(b["t"] > a["t"] for a, b in zip(plines, plines[1:])), f"{len(plines)} readings")

# ── E. faults ──
print("E. faults mid-tour")


def after_fault(bus, ids, lines, k_fault, p):
    """The fault's checks: the hold, the way home, the end."""
    rets = [r for r in lines if r["phase"] == "return"]
    t_fault = rets[0]["t"] if rets else math.inf
    t0 = log_zero(bus)
    later = [(t - t0, by_joint(ids, g)) for t, g in goal_writes(bus) if t - t0 >= t_fault - 1e-3]
    hold_ok = bool(later) and later[0][1] == rets[0]["ticks"]
    moves = [g for _, g in later]
    seg_ok = all(checked_move(a, b, p) for a, b in zip(moves, moves[1:]))
    home = all(GUARD.allowed_ticks(r["ticks"]) or p.parked(r["ticks"]) for r in rets)
    after_fault.moves = moves
    return hold_ok, seg_ok, home, len(moves) - 1


def ctrl_c(bus, clk, plan, p):
    clk.alarm(0.45 * plan.total_s, lambda: setattr(clk.stop, "asked", True))


code, bus, clk, pl, p, text = run(BASE + ["--dry-run", "--fast", "--log", str(TMP / "c.jsonl")], ctrl_c)
lines = read_log(TMP / "c.jsonl")
hold_ok, seg_ok, home, n = after_fault(bus, ids, lines, None, p)
check("Ctrl-C: the arm stops at its reading", hold_ok and "Ctrl-C" in text)
check("Ctrl-C: home through moves that pass segment_allowed and the box check, every reading allowed",
      seg_ok and home, f"{n} moves home")
check("Ctrl-C: exit 2, the last reading within the tolerance of the drive pose",
      code == 2 and p.parked(lines[-1]["ticks"]), text.strip().splitlines()[-1][:100])

K = len(pl.waypoints) // 2


def block(bus, clk, plan, p):
    a, b = plan.waypoints[K - 1].ticks, plan.waypoints[K].ticks
    j = max(JOINTS, key=lambda j: abs(b[j] - a[j]))
    sid = W.servo_ids(p)[j]
    raw = bus.write2ByteTxRx
    n = {"goals": 0}

    def write(ph, s, addr, value):
        if addr == W.REG_GOAL:
            if n["goals"] // 5 == K and s == sid:      # goal set 0 is the hold, set i is waypoint i
                bus.servos[sid].block_at((a[j] + b[j]) / 2.0)
                clk.alarm(clk.now() + 5.5, lambda: setattr(bus.servos[sid], "block", None))
            n["goals"] += 1
        return raw(ph, s, addr, value)

    bus.write2ByteTxRx = write


code, bus, clk, pl, p, text = run(BASE + ["--dry-run", "--fast", "--timeout", "5", "--log", str(TMP / "b.jsonl")], block)
lines = read_log(TMP / "b.jsonl")
hold_ok, seg_ok, home, n = after_fault(bus, ids, lines, K, p)
check(f"a joint blocked on the way to waypoint {K}: a timeout fault, the arm stops at its reading",
      hold_ok and f"waypoint {K} not reached" in text, [s for s in text.splitlines() if "fault" in s][:1][0][:110] if "fault" in text else text[-100:])
check("blocked: back the way it came (to waypoint K - 1 first), home through checked moves, exit 2 at the drive pose",
      len(after_fault.moves) > 1 and after_fault.moves[1] == pl.waypoints[K - 1].ticks
      and seg_ok and home and code == 2 and p.parked(lines[-1]["ticks"]), f"{n} moves home")


def knock(bus, clk, plan, p):
    def hit():
        pose = {j: int(round(bus.servos[s].pos)) for j, s in W.servo_ids(p).items()}
        for j in JOINTS:
            g = GUARD.grids[GUARD.joints.index(j)]
            for v in g:
                t = GUARD.joint_map.rad_to_ticks(j, float(v))
                cand = dict(pose, **{j: t})
                if not GUARD.allowed_ticks(cand) and abs(t - pose[j]) < 700:
                    s = W.servo_ids(p)[j]
                    bus.servos[s].pos = float(t)
                    hit.where = cand
                    return
        hit.where = None
    hit.where = None
    clk.alarm(0.5 * plan.total_s, hit)
    knock.hit = hit


code, bus, clk, pl, p, text = run(BASE + ["--dry-run", "--fast", "--log", str(TMP / "k.jsonl")], knock)
lines = read_log(TMP / "k.jsonl")
rets = [r for r in lines if r["phase"] == "return"]
t0 = log_zero(bus)
after = [g for t, g in goal_writes(bus) if t - t0 >= rets[0]["t"] - 1e-3] if rets else []
check("a knock into a pose the guard refuses: a fault, the arm holds there, exit 3",
      knock.hit.where is not None and code == 3 and "refuses the arm's pose" in text and "holds there" in text,
      text.strip().splitlines()[-1][:110])
check("nothing written after the hold", len(after) == 1 and by_joint(ids, after[0]) == rets[0]["ticks"],
      f"{len(after)} goal writes after the fault")

# a waypoint the guard refuses, slipped into the plan
plan2 = P.plan(25.0, seed=0)
k = len(plan2.waypoints) // 2
forbidden = None
for c in np.argwhere(GUARD.occ):
    q = {j: float(g[c[i]]) for i, (j, g) in enumerate(zip(GUARD.joints, GUARD.grids))}
    t = GUARD.joint_map.pose_to_ticks(q)
    if all(CAL_RUN[j]["range_min"] <= t[j] <= CAL_RUN[j]["range_max"] for j in JOINTS):
        forbidden = t
        break
plan2.waypoints.insert(k, W.Waypoint(forbidden, "frontier"))
clk = W.VirtualClock()
bus = W.fake_bus(P, CAL_RUN, clk)
arm = W.Arm(bus, bus, bus, W.servo_ids(P))
code, text = quiet(W.execute, plan2, P, arm, clk, TMP / "t.jsonl", 12.0, W.Stop())
lines = read_log(TMP / "t.jsonl")
gs = [by_joint(ids, g) for _, g in goal_writes(bus)]
check(f"a forbidden waypoint slipped in at {k}: refused before it is written, exit 2 at the drive pose",
      forbidden not in gs and code == 2 and "refuses the move to waypoint" in text and P.parked(lines[-1]["ticks"]),
      text.strip().splitlines()[0][:110])

# ── F. the start ──
print("F. the start")
clk = W.VirtualClock()
start = {s: DRIVE[j] for j, s in W.servo_ids(P).items()}
start[1] += 4 * TOL
bus = W.FakeBus(start, clk)
arm = W.Arm(bus, bus, bus, W.servo_ids(P))
try:
    quiet(W.execute, plan, P, arm, clk, TMP / "f.jsonl", 12.0, W.Stop())
    check("an arm 100 ticks away from the drive pose is refused", False)
except W.Refused as e:
    check("an arm 100 ticks away from the drive pose is refused, nothing written",
          "not at the drive pose" in str(e) and not bus.writes and not (TMP / "f.jsonl").exists(), str(e)[:100])

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
_TMP.cleanup()
sys.exit(1 if KO else 0)
