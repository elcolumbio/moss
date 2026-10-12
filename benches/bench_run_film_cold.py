"""Cold bench for the run readers: tools/run_film.py (the film of a run and
its verdict), tools/decision_replay.py (a recorded decision through the real
explorer) and tools/gyro_sign_bench.py (the gyro's sign against the lidar).

Known input -> known output, in metres, minutes, cells and degrees. The
recording is written by dimOS's own SqliteStore, stream by stream as
MossRecorder writes it (odom with its pose, global_costmap poseless), so the
readers meet the real schema. Groups:

  A. the film      - a 2 min run that drives 2.0 m out and 1.0 m back over a
                     map with a 41-cell wall: the PNG is written next to the
                     recording, the verdict counts every event family (goals,
                     reached, abandons and their triggers, escapes, contacts, the
                     IMU and the stall guard) and names the back-offs; a cornered
                     back-off's own planner lines (its BUMP and its reverse escape)
                     are not a contact, a contact escape nor an abandon; the real
                     script does the same; a recording from another run is refused;
                     without matplotlib it says how to install it
  B. the inputs    - the newest exploration log (the log root and the run
                     registry; a mapping run is not an exploration) and the
                     newest recording under MOSS_STATE_DIR
  C. replay: log   - an excluded timeout is a failure, not a goal; the goal line
                     with "(beyond the cap ...)", the starvation escape and the
                     "exploration start" line that carries exp, cap and radius
  D. replay: state - note_failed gets the goal, the point the rover STOOD at and
                     the map (unknown signature in cells); a new "exploration
                     start" is a fresh memory; the tuning is the profile's
                     clearances plus the logged exp, cap and radius
  E. the decode    - a costmap blob decoded without dimOS equals dimOS's own
                     grid, a truncated one is refused; Recording reads the
                     synthetic recording back by time
  F. explain       - the radius and the goal cap give the reasons next_target
                     acts on, and the eligible top is next_target's goal
  G. gyro verdict  - the rungs -> the verdict (no turn the lidar saw = no verdict,
                     a rung the lidar lost = no verdict); the prior's state read
                     from the run's log (on = a refusal)
  H. gyro, recorded - recorded gyro and odometry messages through the bus
                     callbacks: a known sign gives its verdict; the gyro is
                     integrated on its own stamps, gaps over 0.1 s skipped
  I. gyro, a rung  - a rover that pivots at full power whatever rate is asked:
                     the rung ends on the lidar yaw, not on a clock; a lidar that
                     follows a tenth of the turn ends the rung on the gyro and
                     the verdict is INCONCLUSIVE, never "axes wrong"

Needs dimOS (the store and the messages), numpy, matplotlib and PIL: the
groups that need a missing one are SKIPPED with a line.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_run_film_cold.py
"""
import ast
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def missing(*mods):
    return [m for m in mods if importlib.util.find_spec(m) is None]


for _var in [k for k in os.environ if k.startswith("MOSS_")] + ["XDG_STATE_HOME"]:
    os.environ.pop(_var, None)

T0 = 1_800_000_000.0            # a known epoch: the run starts here
TMP = tempfile.TemporaryDirectory()
WORK = Path(TMP.name)
EXPLORER = "/home/x/moss/moss_dimos/explorer2.py"
PLANNER = "/home/x/moss/moss_dimos/recovering_planner.py"
ADAPTER = "/home/x/moss/moss_dimos/moss_adapter.py"
IMU = "/home/x/moss/moss_dimos/bump_imu.py"
GPLANNER = "/home/x/dimos/navigation/replanning_a_star/global_planner.py"


def iso(t):
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def line(t, logger, event, **fields):
    return json.dumps({"event": event, "level": "info", "logger": logger, "timestamp": iso(t), **fields})


# One run, 120 s: a known event of every family at a known time.
RUN_LINES = [
    (0.0, EXPLORER, "exploration start: distance exp 2, cap 3 m, radius 4 m (0 = no limit; the radius is centred "
                    "on the first pose)", {}),
    (1.0, EXPLORER, "goal 1: (1.00, 0.50) 1.1 m away, 12 frontier cells, 3 clusters, chosen in 40 ms", {}),
    (30.0, GPLANNER, "Arrived at goal.", {}),
    (31.0, EXPLORER, "goal 2: (2.50, 0.00) 3.4 m away (beyond the cap: nothing safe within it), 8 frontier cells, "
                     "2 clusters, chosen in 35 ms", {}),
    (58.0, IMU, "IMU contact: forward -12.5 m/s^2 (threshold 8.0) while commanded L=+50% R=+50%", {}),
    (58.1, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True}),
    (60.0, PLANNER, "Contact escape", {"trigger": "bump", "direction": "reverse", "travelled_m": 0.2}),
    (70.0, ADAPTER, "MOSS base: STALL #1: tracks commanded L=+50% R=+50% but the encoders moved L=+3 R=+2 ticks "
                    "in 1.0 s - stopped (the encoders are the bumper)", {}),
    (70.1, PLANNER, "Goal abandoned, no recovery motion", {"trigger": "stall"}),
    (90.0, EXPLORER, "born cornered: 2.10 m2 of reachable floor and no frontier - one 0.22 m back-off via the "
                     "bump reflex", {}),
    # the planner's answer to the bump the explorer published: the back-off itself, not a contact
    (90.05, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True}),
    (92.0, PLANNER, "Contact escape", {"trigger": "bump", "direction": "reverse", "travelled_m": 0.2}),
    (100.0, PLANNER, "Goal abandoned, no recovery motion", {"trigger": "stuck"}),
    (120.0, EXPLORER, "exploration complete: no reachable frontier left (2 targets, 20 ms)", {}),
]


def write_run_log(run_dir: Path, lines=RUN_LINES, t0=T0) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / "main.jsonl"
    with open(path, "w") as f:
        f.write("not json at all\n")
        for dt, logger, event, fields in lines:
            f.write(line(t0 + dt, logger, event, **fields) + "\n")
    return path


def x_at(dt):
    """The trajectory: 2.0 m out along x in 60 s, then 1.0 m back in 60 s (3.0 m driven)."""
    return 2.0 * dt / 60.0 if dt <= 60.0 else 2.0 - 1.0 * (dt - 60.0) / 60.0


WALL_COL, WALL_ROWS = 70, range(10, 51)      # 41 occupied cells, x = -1.0 + 70.5 * 0.05 = 2.525 m


def costmap_grid():
    g = np.zeros((60, 80), dtype=np.int8)
    g[:, 75:] = -1
    g[list(WALL_ROWS), WALL_COL] = 100
    return g


HAVE_DIMOS = not missing("dimos", "dimos_lcm")
HAVE_DRAW = not missing("matplotlib", "PIL")


def write_recording(db: Path, t0=T0, seconds=120, decision_grids=None):
    """A recording as MossRecorder writes it, through dimOS's own SqliteStore:
    odom every 1 s with its pose, global_costmap poseless (one at the end, or
    `decision_grids` [(t, grid, res, ox, oy)])."""
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    store = SqliteStore(path=str(db))
    store.start()
    odom = store.stream("odom", PoseStamped)
    for k in range(seconds + 1):
        t = t0 + k
        p = PoseStamped(ts=t, frame_id="world", position=[x_at(k), 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        odom.append(p, ts=t, pose=Pose(position=p.position, orientation=p.orientation))
    cm = store.stream("global_costmap", OccupancyGrid)
    for t, grid, res, ox, oy in (decision_grids or [(t0 + seconds - 1, costmap_grid(), 0.05, -1.0, -1.5)]):
        og = OccupancyGrid(grid=grid, resolution=res, frame_id="world", ts=t,
                           origin=Pose(position=[ox, oy, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]))
        cm.append(og, ts=t, pose=None)
    store.stop()
    return db


# ── A. the film ─────────────────────────────────────────────────────────
print("A. the film: a 2 min, 3.0 m run with an event of every family, drawn next to its recording")
RF = None
if not HAVE_DIMOS or not HAVE_DRAW:
    print(f"  -- SKIP A and B: missing {missing('dimos', 'dimos_lcm', 'matplotlib', 'PIL')}")
else:
    import run_film as RF
    STATE = WORK / "state" / "moss"
    REC_DIR = STATE / "recordings"
    REC_DIR.mkdir(parents=True)
    DB = write_recording(REC_DIR / "moss.db")
    RUN = WORK / "dimos" / "logs" / "run-a-moss-dimos-moss-explore"
    LOG = write_run_log(RUN)

    events, t0, t1 = RF.read_log(str(LOG))
    n = {fam: len(rows) for fam, rows in events.items()}
    check("the log reads 120 s of run, the junk line skipped", t0 == T0 and abs(t1 - t0 - 120.0) < 1e-6, f"{t1 - t0:.3f} s")
    check("every family counted: 2 goals, 1 reached, 2 abandons, 1 escape, 1 contact, 1 IMU, 1 stall, 1 cornered",
          n == {"goal": 2, "abandon": 2, "escape": 1, "bump": 1, "arrived": 1, "imu": 1, "stall": 1, "cornered": 1}, f"{n}")
    check("a goal is drawn where it was sent: (1.00, 0.50) m and (2.50, 0.00) m",
          [r[1] for r in events["goal"]] == [(1.0, 0.5), (2.5, 0.0)], f"{[r[1] for r in events['goal']]}")
    check("the escape keeps its direction, the abandons their triggers",
          [r[2] for r in events["escape"]] == ["reverse"] and [r[2] for r in events["abandon"]] == ["stall", "stuck"])
    check("the cornered back-off took its own BUMP (90.05 s) and reverse escape (92 s): 1 contact at 58.1 s, "
          "1 escape at 60 s, the cornered row 'reversed'",
          [round(r[0] - T0, 2) for r in events["bump"]] == [58.1] and [r[0] - T0 for r in events["escape"]] == [60.0]
          and [r[2] for r in events["cornered"]] == ["reversed"] and [r[2] for r in events["bump"]] == ["front"],
          f"bumps {[round(r[0] - T0, 2) for r in events['bump']]}, escapes {[r[0] - T0 for r in events['escape']]}, "
          f"cornered {[r[2] for r in events['cornered']]}")
    odom, occ = RF.read_db(str(DB), t0, t1)
    xs = [r[1] for r in odom]
    dist = float(np.hypot(np.diff(xs), np.zeros(len(xs) - 1)).sum())
    check("the recording gives back 121 poses and 3.0 m of path", len(odom) == 121 and abs(dist - 3.0) < 1e-9,
          f"{len(odom)} poses, {dist:.6f} m")
    check("and the map's wall: 41 obstacle centres at x = 2.525 m",
          occ.shape == (41, 2) and np.allclose(occ[:, 0], 2.525), f"{occ.shape}, x {sorted(set(np.round(occ[:, 0], 4)))}")

    code, lines = RF.film(str(LOG), str(DB))
    PNG = REC_DIR / "run-a-moss-dimos-moss-explore.film.png"
    check("film() returns 0 and writes <run>.film.png next to the recording",
          code == 0 and PNG.is_file() and lines[0] == f"THE FILM -> {PNG}", lines[0] if lines else "(nothing)")
    from PIL import Image
    with Image.open(PNG) as im:
        size = im.size
        colours = len(set(im.convert("RGB").getdata()))
        blue = sum(1 for r, g, b in im.convert("RGB").getdata() if b > 120 and b - r > 60)
    check("a real picture: 1430 x 1170 px (11 x 9 in at 130 dpi), not a blank page",
          size == (1430, 1170) and colours > 100 and blue > 500, f"{size}, {colours} colours, {blue} blue px of trajectory")
    want = ["  2.0 min, 3.0 m driven",
            "  goals: 2 chosen, 1 reached, 3 abandoned (stuck + contact: every escape cancels its goal), "
            "1 contact escapes, 1 contacts",
            "  back-offs: 2 (1 after a contact, 1 cornered); 0 forward contact escapes; contacts felt by the IMU 1, "
            "inferred from an encoder stall 1",
            "  abandon triggers: stall 1, stuck 1",
            "  verdict: 1.5 stuck/contact events per minute - LOOK AT THE MAP before any conclusion"]
    check("the verdict, line for line (minutes, metres, counts, back-offs, triggers, rate)", lines[1:] == want,
          " | ".join(lines[1:]))
    check("it names the back-offs: 2 (a 0.20 m reverse escape and a cornered back-off)",
          any(ln.startswith("  back-offs: 2 (") for ln in lines))
    fwd = [r if r[0] != 60.0 else (r[0], r[1], r[2], {**r[3], "direction": "forward", "trigger": "bump_rear"})
           for r in RUN_LINES]
    ev_f, f0, f1 = RF.read_log(str(write_run_log(WORK / "fwd" / "run-f", fwd)))
    vf = RF.verdict_lines(ev_f, f0, f1, 3.0, "x.png")
    check("a contact behind the rover escapes FORWARD: not a back-off (1 = the cornered one)",
          any(ln.startswith("  back-offs: 1 (0 after a contact, 1 cornered); 1 forward contact escapes") for ln in vf),
          vf[3])
    calm = [r for r in RUN_LINES if not r[2].startswith(("BUMP received", "Goal abandoned", "Contact escape"))]
    ev_c, c0, c1 = RF.read_log(str(write_run_log(WORK / "calm" / "run-c", calm)))
    vc = RF.verdict_lines(ev_c, c0, c1, 3.0, "x.png")
    check("a run with no stuck and no contact: 0.0 per minute and no alarm",
          vc[-1] == "  verdict: 0.0 stuck/contact events per minute", vc[-1])
    check("its cornered line had no planner answer: said, and not counted as a back-off",
          "  cornered with no back-off logged: 1 (no planner BUMP and reverse escape followed the cornered line)" in vc
          and any(ln.startswith("  back-offs: 0 (0 after a contact, 0 cornered)") for ln in vc), " | ".join(vc[3:5]))

    # one cornered back-off, logged the way the stack logs it, and nothing else
    ALONE = [(0.0, EXPLORER, "exploration start: distance exp 2, cap 3 m, radius 4 m", {}),
             (10.0, EXPLORER, "born cornered: 0.40 m2 of reachable floor and no frontier - one 0.20 m back-off via the "
                              "bump reflex", {}),
             (10.05, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True}),
             (12.0, PLANNER, "Contact escape", {"trigger": "bump", "direction": "reverse", "travelled_m": 0.2}),
             (60.0, EXPLORER, "exploration complete: no reachable frontier left", {})]
    ev_a, a0, a1 = RF.read_log(str(write_run_log(WORK / "alone" / "run-g", ALONE)))
    va = RF.verdict_lines(ev_a, a0, a1, 0.4, "x.png")
    check("one cornered back-off alone: 1 back-off (cornered), 0 contacts, 0 abandoned, 0.0 per minute",
          va[2] == "  goals: 0 chosen, 0 reached, 0 abandoned (stuck + contact: every escape cancels its goal), "
                   "0 contact escapes, 0 contacts"
          and va[3].startswith("  back-offs: 1 (0 after a contact, 1 cornered)")
          and va[-1] == "  verdict: 0.0 stuck/contact events per minute", " | ".join(va[2:]))
    LATE = ALONE[:2] + [(13.0, IMU, "IMU contact: forward -12.5 m/s^2 (threshold 8.0) while commanded L=+50% R=+50%", {}),
                        (13.05, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True}),
                        (15.0, PLANNER, "Contact escape", {"trigger": "bump", "direction": "reverse", "travelled_m": 0.2})]
    ev_l, _l0, _l1 = RF.read_log(str(write_run_log(WORK / "late" / "run-h", LATE)))
    check("a real contact 3 s after a cornered line (no BUMP within 1 s of it) stays a contact and its escape",
          len(ev_l["bump"]) == 1 and len(ev_l["escape"]) == 1 and [r[2] for r in ev_l["cornered"]] == [None],
          f"{len(ev_l['bump'])} contacts, {len(ev_l['escape'])} escapes, cornered {[r[2] for r in ev_l['cornered']]}")
    REAR = ALONE[:2] + [(10.05, PLANNER, "BUMP received: rear contact", {"reflex": "escape forward 0.20 m", "acted": True}),
                        (12.0, PLANNER, "Contact escape", {"trigger": "bump_rear", "direction": "forward"})]
    ev_r, _r0, _r1 = RF.read_log(str(write_run_log(WORK / "rear" / "run-i", REAR)))
    check("a REAR contact right after a cornered line is a contact (the back-off rides the front bump only)",
          [r[2] for r in ev_r["bump"]] == ["rear"] and [r[2] for r in ev_r["escape"]] == ["forward"]
          and [r[2] for r in ev_r["cornered"]] == [None], f"{ev_r['bump']} {ev_r['escape']}")
    BUSY = ALONE[:1] + [(8.0, IMU, "IMU contact: forward -12.5 m/s^2 (threshold 8.0) while commanded L=+50% R=+50%", {}),
                        (8.05, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True}),
                        (9.0,) + ALONE[1][1:],
                        (9.05, PLANNER, "BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": False}),
                        (9.2, PLANNER, "Contact escape", {"trigger": "bump", "direction": "reverse", "aborted": True})]
    ev_b, b0, b1 = RF.read_log(str(write_run_log(WORK / "busy" / "run-j", BUSY)))
    vb = RF.verdict_lines(ev_b, b0, b1, 0.2, "x.png")
    check("a cornered bump the planner did not act on (an escape was running): that escape stays the contact's",
          [round(r[0] - T0, 2) for r in ev_b["bump"]] == [8.05] and [round(r[0] - T0, 1) for r in ev_b["escape"]] == [9.2]
          and [r[2] for r in ev_b["cornered"]] == [None]
          and vb[3].startswith("  back-offs: 1 (1 after a contact, 0 cornered)"), " | ".join(vb[2:5]))

    OTHER = write_recording(WORK / "other.db", t0=T0 + 86400)
    code, lines = RF.film(str(LOG), str(OTHER), str(WORK / "x.png"))
    check("a recording of another run (no odom in this log's window) is refused, nothing drawn",
          code == 1 and "wrong db?" in lines[0] and not (WORK / "x.png").exists(), lines[0])
    saved_mpl = {k: v for k, v in sys.modules.items() if k == "matplotlib" or k.startswith("matplotlib.")}
    sys.modules["matplotlib"] = None          # import matplotlib now raises ImportError
    try:
        code, lines = RF.film(str(LOG), str(DB), str(WORK / "no_mpl.png"))
    finally:
        del sys.modules["matplotlib"]
        sys.modules.update(saved_mpl)
    check("no matplotlib: exit 1, one line saying how to install it, nothing drawn",
          code == 1 and lines == ["matplotlib is missing, nothing drawn: .venv/bin/pip install matplotlib"]
          and not (WORK / "no_mpl.png").exists(), " | ".join(lines))

    env = {k: v for k, v in os.environ.items() if not k.startswith("MOSS_")}
    env.update({"PYTHONPATH": str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""), "PYTHONDONTWRITEBYTECODE": "1"})
    PNG.unlink()
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "run_film.py"), "--log", str(LOG), "--db", str(DB)],
                       capture_output=True, text=True, env=env, timeout=300)
    check("the real script: exit 0, the film next to the recording, the same verdict",
          r.returncode == 0 and PNG.is_file() and r.stdout.splitlines()[1:] == want,
          f"rc {r.returncode}, {len(r.stdout.splitlines())} lines" + ("" if r.returncode == 0 else
                                                                      f": {r.stderr.strip()[-160:]}"))

    # ── B. the inputs ───────────────────────────────────────────────────
    print("B. the inputs: the newest exploration log and the newest recording")
    LOGROOT = WORK / "dimos" / "logs"
    RUNS = WORK / "dimos" / "runs"
    RUNS.mkdir(parents=True)
    os.utime(LOG, (T0, T0))
    newer = write_run_log(LOGROOT / "run-b-moss-dimos-moss-explore")
    os.utime(newer, (T0 + 100, T0 + 100))
    mapping = write_run_log(LOGROOT / "run-c-moss-dimos-moss-nav")
    os.utime(mapping, (T0 + 200, T0 + 200))
    lg, rg = str(LOGROOT / "*-moss-explore" / "main.jsonl"), str(RUNS / "*.json")
    check("the newest exploration run's log, not a newer mapping run's", RF.newest_log(lg, rg) == str(newer),
          f"{RF.newest_log(lg, rg)}")
    clone = write_run_log(WORK / "clone" / "logs" / "run-d")
    os.utime(clone, (T0 + 300, T0 + 300))
    (RUNS / "d.json").write_text(json.dumps({"blueprint": "moss_dimos.moss-explore", "log_dir": str(clone.parent)}))
    (RUNS / "e.json").write_text(json.dumps({"blueprint": "moss_dimos.moss-nav", "log_dir": str(mapping.parent)}))
    (RUNS / "f.json").write_text("{half")
    check("a run from a git clone is found through its registry entry", RF.newest_log(lg, rg) == str(clone),
          f"{RF.newest_log(lg, rg)}")
    rotated = REC_DIR / "moss.11111111111111.db"
    rotated.write_bytes(DB.read_bytes())
    os.utime(rotated, (T0 - 100, T0 - 100))
    os.utime(DB, (T0 + 100, T0 + 100))
    check("the newest recording under MOSS_STATE_DIR: moss.db over a rotated moss.<stamp>.db",
          RF.newest_db({"MOSS_STATE_DIR": str(STATE)}) == str(DB), f"{RF.newest_db({'MOSS_STATE_DIR': str(STATE)})}")
    os.utime(rotated, (T0 + 500, T0 + 500))
    check("... and the rotated one when it is the newest", RF.newest_db({"MOSS_STATE_DIR": str(STATE)}) == str(rotated))
    check("no recording at all -> None (the script then says what is missing)",
          RF.newest_db({"MOSS_STATE_DIR": str(WORK / "empty")}) is None)


# ── C. replay: the log ──────────────────────────────────────────────────
print("C. decision_replay.read_log: an excluded timeout is a failure, every explorer line routed")
RD = None
if missing("scipy"):
    print(f"  -- SKIP C to F: missing {missing('scipy')}")
else:
    import decision_replay as RD
    from moss_dimos.explorer2 import DEFAULT_TUNING, ExploreState, next_target
    from moss_dimos import robot_profile

    LINES = [   # (seconds into the run, the explorer's own words)
        (0.0, "exploration start: distance exp 2, cap 3 m, radius 4 m (0 = no limit; the radius is centred on "
              "the first pose)"),
        (1.0, "goal 1: (1.00, 2.00) 3.2 m away, 11 frontier cells, 5 clusters, chosen in 100 ms"),
        (45.0, "goal timeout after 45 s having closed 0.10 m of 3.20 m: excluded until the map or the viewpoint "
               "changes"),
        (46.0, "goal 2: (3.00, 1.00) 2.1 m away (beyond the cap: nothing safe within it), 7 frontier cells, "
               "4 clusters, chosen in 120 ms"),
        (91.0, "goal timeout after 45 s, 1.20 m closer, re-deciding"),
        (95.0, "planner gave up on that goal: excluded until the map or the viewpoint changes"),
        (96.0, "starvation escape: the world stayed silent for 30 asks - reopening the oldest failed goal at "
               "(1.00, 2.00)"),
        (97.0, "exploration complete: no reachable frontier left (2 targets, 26 ms)"),
        (120.0, "exploration start: distance exp 1, cap 0 m, radius 0 m (0 = no limit; the radius is centred on "
                "the first pose)"),
        (121.0, "goal 1: (0.50, 0.50) 0.9 m away, 9 frontier cells, 2 clusters, chosen in 30 ms"),
    ]
    T_EXCLUDED = T0 + LINES[2][0]
    T_GAVE_UP = T0 + LINES[5][0]
    STOOD_EARLY, STOOD_AT_TIMEOUT, STOOD_AT_REFUSAL = (0.10, 0.20), (0.75, 0.25), (2.50, 1.30)
    RLOG = WORK / "replay.jsonl"
    with open(RLOG, "w") as fh:
        fh.write("not json at all\n")
        fh.write(json.dumps({"logger": "/x/local_planner.py", "event": "goal 9: nonsense",
                             "timestamp": iso(T0 + LINES[1][0])}) + "\n")
        for dt, event in LINES:
            fh.write(json.dumps({"logger": EXPLORER, "event": event, "timestamp": iso(T0 + dt)}) + "\n")
    ev = RD.read_log(str(RLOG))
    kinds = [e.kind for e in ev]
    check("10 explorer lines in, 9 events out (the 're-deciding' timeout is not one)", len(ev) == 9, f"{len(ev)}")
    check("kinds in order: start, goal, failed, goal, failed, wait, none, start, goal",
          kinds == ["start", "goal", "failed", "goal", "failed", "wait", "none", "start", "goal"], f"{kinds}")
    check("the excluded timeout is the failure at its own time (45 s in)",
          ev[2].kind == "failed" and abs(ev[2].t - T_EXCLUDED) < 1e-6, f"{ev[2].t - T0:.3f} s")
    g = ev[1]
    check("goal 1: n=1, (1.00, 2.00) m, 3.2 m, 11 cells, 5 clusters, 100 ms",
          (g.goal_n, g.xy, g.path_cost_m, g.info_cells, g.n_clusters, g.elapsed_ms) == (1, (1.0, 2.0), 3.2, 11, 5, 100.0),
          f"{g.goal_n} {g.xy} {g.path_cost_m} {g.info_cells} {g.n_clusters} {g.elapsed_ms}")
    g = ev[3]
    check("the goal beyond the cap: (3.00, 1.00) m, 2.1 m, 7 cells, 4 clusters, 120 ms",
          (g.xy, g.path_cost_m, g.info_cells, g.n_clusters, g.elapsed_ms) == ((3.0, 1.0), 2.1, 7, 4, 120.0),
          f"{g.xy} {g.path_cost_m} {g.info_cells} {g.n_clusters} {g.elapsed_ms}")
    check("the start lines carry exp, cap, radius: (2, 3, 4) then (1, 0, 0)",
          ev[0].shape == (2.0, 3.0, 4.0) and ev[7].shape == (1.0, 0.0, 0.0), f"{ev[0].shape} {ev[7].shape}")

    # ── D. replay: the memory and the tuning ────────────────────────────
    print("D. replay: note_failed carries the goal, the standing point and the map; a new start is a fresh memory")
    tun = RD.run_tuning((2.0, 3.0, 4.0))
    lethal, pivot = robot_profile.derived()
    check("the run's tuning: the profile's clearances (0.22 / 0.25 m) and the logged exp 2, cap 3 m, radius 4 m",
          (tun.lethal_clearance_m, tun.pivot_clearance_m, tun.distance_exponent, tun.max_goal_path_m,
           tun.explore_radius_m) == (lethal, pivot, 2.0, 3.0, 4.0) and abs(lethal - 0.22) < 1e-9,
          f"{tun.lethal_clearance_m:.2f} {tun.pivot_clearance_m:.2f} {tun.distance_exponent} {tun.max_goal_path_m} "
          f"{tun.explore_radius_m}")
    check("everything else as the blueprint leaves it (DEFAULT_TUNING)",
          tun.min_frontier_perimeter_m == DEFAULT_TUNING.min_frontier_perimeter_m
          and tun.failed_goal_radius_m == DEFAULT_TUNING.failed_goal_radius_m)

    def build_grid():
        """10 m x 10 m free map, origin (0, 0), with two unknown patches of known size."""
        grid = np.full((200, 200), 0, dtype=np.int8)
        grid[40:45, 20:24] = -1        # 20 cells, inside 0.6 m of (1.00, 2.00)
        grid[18:21, 58:62] = -1        # 12 cells, inside 0.6 m of (3.00, 1.00)
        return grid

    class FakeRecording:
        def __init__(self):
            self.grid = build_grid()

        def costmap_at(self, t):
            return RD._Grid(self.grid, 0.05, 0.0, 0.0, t)

        def pose_at(self, t):
            if t >= T_GAVE_UP:
                x, y = STOOD_AT_REFUSAL
            elif t >= T_EXCLUDED:
                x, y = STOOD_AT_TIMEOUT
            else:
                x, y = STOOD_EARLY
            return RD._Pose(x, y, 0.0, 1.0, t)

    out = list(RD.replay(FakeRecording(), ev, DEFAULT_TUNING))
    states = [o[4] for o in out]
    tunings = [o[6] for o in out]
    check("5 decisions replayed (3 goals, the wait, the end of the first exploration)", len(out) == 5, f"{len(out)}")
    check("decision 1 saw no exclusion yet", states[0].failed == [], f"{states[0].failed}")
    check("decision 2 saw exactly the one exclusion the timeout wrote", len(states[1].failed) == 1, f"{len(states[1].failed)}")
    if states[1].failed:
        x, y, sx, sy, sig = states[1].failed[0]
        check("excluded goal (1.00, 2.00) m, stood at (0.75, 0.25) m - where the rover was at the TIMEOUT",
              (x, y) == (1.0, 2.0) and (sx, sy) == STOOD_AT_TIMEOUT, f"({x}, {y}) stood ({sx}, {sy})")
        check("unknown signature = 20 cells around that goal", sig == 20, f"{sig}")
    if len(states) > 2 and len(states[2].failed) == 2:
        x, y, sx, sy, sig = states[2].failed[1]
        check("the planner refusal excludes goal 2 (3.00, 1.00) m, stood at (2.50, 1.30) m, 12 cells",
              (x, y) == (3.0, 1.0) and (sx, sy) == STOOD_AT_REFUSAL and sig == 12, f"({x}, {y}) ({sx}, {sy}) {sig}")
    else:
        check("the wait decision carries both exclusions", False, f"{[s.failed for s in states]}")
    check("the second 'exploration start' is a fresh memory: no goal, no exclusion, no anchor",
          states[4].visited == [] and states[4].failed == [] and states[4].anchor is None,
          f"{states[4].visited} {states[4].failed} {states[4].anchor}")
    check("each exploration replays with its own logged shape: exp 2 / cap 3 / radius 4, then 1 / 0 / 0",
          (tunings[0].distance_exponent, tunings[0].max_goal_path_m, tunings[0].explore_radius_m) == (2.0, 3.0, 4.0)
          and (tunings[4].distance_exponent, tunings[4].max_goal_path_m, tunings[4].explore_radius_m) == (1.0, 0.0, 0.0))
    check("the anchor of the first exploration is its first pose (0.10, 0.20) m", states[1].anchor == STOOD_EARLY,
          f"{states[1].anchor}")

    # ── E. the decode and the recording ─────────────────────────────────
    print("E. the costmap blob decoded without dimOS; the recording read back by time")
    if not HAVE_DIMOS:
        print(f"  -- SKIP E: missing {missing('dimos', 'dimos_lcm')}")
    else:
        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
        grid = costmap_grid()
        og = OccupancyGrid(grid=grid, resolution=0.05, frame_id="world", ts=T0,
                           origin=Pose(position=[-1.0, -1.5, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]))
        blob = og.lcm_encode()
        g2, res, ox, oy, frame = RD.decode_occupancy_grid(blob)
        check("dimOS's own blob decodes to the same 60 x 80 grid, 0.05 m, origin (-1.0, -1.5), 'world'",
              np.array_equal(g2, grid) and abs(res - 0.05) < 1e-6 and (ox, oy) == (-1.0, -1.5) and frame == "world",
              f"{g2.shape} {res:.4f} ({ox}, {oy}) {frame}")
        try:
            RD.decode_occupancy_grid(blob[:-7])
            check("a truncated blob is refused", False, "no exception")
        except ValueError as exc:
            check("a truncated blob is refused (a format change cannot pass silently)", True, str(exc))
        if RF is not None:
            rec = RD.Recording(str(DB))
            cm = rec.costmap_at(T0 + 200)
            check("Recording: 121 poses and 1 costmap from the synthetic recording",
                  len(rec.odom_ts) == 121 and len(rec.costmap_ts) == 1)
            check("the costmap held at a time is the last one before it, its wall where it was written",
                  int((cm.grid == 100).sum()) == 41 and cm.ts == T0 + 119)
            p = rec.pose_at(T0 + 90.5)
            check("the pose at t + 90.5 s is the row before it: x = 1.50 m (on the way back)",
                  abs(p.position.x - 1.5) < 1e-9 and p.ts == T0 + 90, f"x {p.position.x:.4f} at {p.ts - T0:.1f} s")
            try:
                rec.pose_at(T0 - 1)
                check("no pose before the recording starts", False, "no exception")
            except LookupError:
                check("no pose before the recording starts: LookupError, not a guess", True)

    # ── F. explain mirrors next_target ──────────────────────────────────
    print("F. explain(): the radius and the cap give next_target's reasons; the eligible top is its goal")
    from dataclasses import replace as _replace
    from moss_dimos.explorer2 import PoseStamped as _PS

    def hall():
        """The explorer bench's hall: 6 x 2 m, its east end open onto 1 m of
        unknown (far frontier), a 0.50 m door into a closet on the north wall."""
        g = np.full((120, 220), -1, dtype=np.int8)
        g[40:80, 20:140] = 0
        g[39:81, 19] = 100
        g[39, 19:161] = 100
        g[80, 19:161] = 100
        g[39:81, 160] = 100
        g[80:93, 57] = 100
        g[80:93, 78] = 100
        g[93, 57:79] = 100
        g[80, 63:73] = -1
        return g

    def pose(x, y, yaw=0.0):
        p = _PS(ts=0.0, frame_id="world")
        p.position.x, p.position.y = x, y
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    hall_cm = RD._Grid(hall(), 0.05, 0.0, 0.0, 0.0)
    start = pose(50.5 * 0.05, 60.5 * 0.05)
    for label, tuning, far_reason in (
            ("radius 2 m: the far end is outside it", _replace(DEFAULT_TUNING, forward_bonus=0.0, distance_exponent=1.0,
                                                               max_goal_path_m=0.0, explore_radius_m=2.0), "outside-radius"),
            ("cap 3 m, a safe closet within it: the far end waits beyond the cap",
             _replace(DEFAULT_TUNING, forward_bonus=0.0, distance_exponent=1.0, max_goal_path_m=3.0,
                      explore_radius_m=0.0), "beyond-cap"),
            ("no cap, no radius, exponent 1: both compete and the big far end wins",
             _replace(DEFAULT_TUNING, forward_bonus=0.0, distance_exponent=1.0, max_goal_path_m=0.0,
                      explore_radius_m=0.0), "")):
        target = next_target(hall_cm, start, ExploreState(), now=1.0, tuning=tuning)
        scored, info = RD.explain(hall_cm, start, ExploreState(), 1.0, tuning)
        top = [s for s in scored if not s.reason]
        far = max(scored, key=lambda s: s.cluster.goal_xy[0])
        same = (target is not None and top and abs(top[0].cluster.goal_xy[0] - target.position.x) < 1e-9
                and abs(top[0].cluster.goal_xy[1] - target.position.y) < 1e-9 and abs(top[0].score - target.score) < 1e-12)
        check(f"{label}: far end '{far.reason or 'competes'}', the top is next_target's goal",
              len(scored) == 2 and far.reason == far_reason and bool(same),
              f"far ({far.cluster.goal_xy[0]:.2f}, {far.cluster.goal_xy[1]:.2f}) {far.reason!r}, "
              f"top {top[0].cluster.goal_xy if top else None}, next_target "
              f"{(round(target.position.x, 2), round(target.position.y, 2)) if target else None}")


# ── G. gyro verdict ─────────────────────────────────────────────────────
print("G. gyro_sign_bench.verdict(): the rungs -> the verdict")
import gyro_sign_bench as GB  # noqa: E402

STILL = [("+30 deg (left)", 29.5, 0.4, "time"), ("-30 deg (right)", -28.9, -0.3, "time")]
AGREE = [("+30 deg (left)", 29.5, 30.2, "lidar"), ("-30 deg (right)", -28.9, -29.7, "lidar")]
ONE_RUNG = [("+30 deg (left)", 29.5, 30.2, "lidar"), ("-30 deg (right)", -1.2, -0.4, "time")]
FLIPPED = [("+30 deg (left)", -29.5, 30.2, "lidar"), ("-30 deg (right)", 28.9, -29.7, "lidar")]
DEAD_GYRO = [("+30 deg (left)", 0.2, 30.2, "lidar"), ("-30 deg (right)", -0.1, -29.7, "lidar")]
TOO_BIG = [("+30 deg (left)", 91.0, 30.2, "lidar"), ("-30 deg (right)", -89.0, -29.7, "lidar")]
LOST = [("+30 deg (left)", 90.4, 9.0, "gyro"), ("-30 deg (right)", -29.1, -30.3, "lidar")]
SLOW = [("+30 deg (left)", 20.3, 19.6, "time"), ("-30 deg (right)", -29.1, -30.3, "lidar")]
msg, code = GB.verdict(STILL)
check("0.4 deg of lidar yaw is not a turn: INCONCLUSIVE, exit 3, never CORRECT",
      "INCONCLUSIVE" in msg and "CORRECT" not in msg and code == 3, msg)
msg, code = GB.verdict([])
check("no rung at all: INCONCLUSIVE, exit 3", "INCONCLUSIVE" in msg and code == 3, f"{code}")
check("(the all() over an empty set this guards against WOULD say CORRECT on the same rungs)",
      all(g * l > 0 and 0.5 < abs(g / l) < 2.0 for _, g, l, _e in STILL if abs(l) > 5) is True)
msg, code = GB.verdict(AGREE)
check("29.5 deg gyro vs 30.2 deg lidar, both rungs: CORRECT, exit 0, and the next step MOSS_GYRO=1",
      "CORRECT" in msg and code == 0 and "MOSS_GYRO=1" in msg, msg)
msg, code = GB.verdict(ONE_RUNG)
check("one real rung is enough, and it says how many were measured (1/2)",
      "CORRECT" in msg and code == 0 and "1/2 rungs measured" in msg, msg)
msg, code = GB.verdict(FLIPPED)
check("-29.5 deg gyro vs +30.2 deg lidar: SIGN FLIPPED -> 'y', exit 2", "SIGN FLIPPED" in msg and "'y'" in msg and code == 2, msg)
msg, code = GB.verdict(DEAD_GYRO)
check("0.2 deg gyro over a 30 deg turn: WRONG, exit 2", "WRONG" in msg and code == 2, msg)
msg, code = GB.verdict(TOO_BIG)
check("91 deg gyro over a 30 deg turn the lidar ended (ratio 3): WRONG, exit 2", "WRONG" in msg and code == 2, msg)
msg, code = GB.verdict(LOST)
check("a rung that ended on the GYRO (90 deg) with 9 deg of lidar: INCONCLUSIVE, exit 3, relaunch at "
      "MOSS_PIVOT_PCT=50, never 'WRONG' (the other rung agreeing changes nothing)",
      "INCONCLUSIVE" in msg and code == 3 and "MOSS_PIVOT_PCT=50" in msg and "WRONG" not in msg, msg)
msg, code = GB.verdict([(lb, g, l, "time") for lb, g, l, _e in LOST])
check("the same rung ended by the time limit: INCONCLUSIVE too", "INCONCLUSIVE" in msg and code == 3, msg)
msg, code = GB.verdict(SLOW)
check("a slow rung cut by the time limit where gyro and lidar AGREE (20.3 vs 19.6 deg) still measures: CORRECT",
      "CORRECT" in msg and code == 0, msg)

# the prior's state, from the run's own log
PRIOR_DIR = Path(TMP.name) / "prior"
PRIOR_DIR.mkdir()


def prior_log(name, *events):
    path = PRIOR_DIR / name
    with open(path, "w") as fh:
        for k, ev in enumerate(events):
            fh.write(json.dumps({"event": ev, "level": "info", "timestamp": iso(T0 + k)}) + "\n")
    return str(path)


OFF_LOG = prior_log("off.jsonl", "MOSS blueprint: lidar odometry prior: gyro=False (run tools/gyro_sign_bench.py "
                                 "once, then MOSS_GYRO=1)",
                    "lidar odometry up (kiss-icp, voxel 0.05 m, range 0.15-12.0 m, prior: gyro=False axis -y, "
                    "wheels NEVER) -> world")
ON_LOG = prior_log("on.jsonl", "MOSS blueprint: lidar odometry prior: gyro=True (MOSS_GYRO=1)",
                   "lidar odometry up (kiss-icp, voxel 0.05 m, range 0.15-12.0 m, prior: gyro=True axis -y, "
                   "wheels NEVER) -> world")
MUTE_LOG = prior_log("mute.jsonl", "lidar odom #1: x=+0.000 y=+0.000 yaw=+0.0deg")
msg, code = GB.prior_check(ON_LOG)
check("the run's log says 'prior: gyro=True': refused, exit 3, naming MOSS_GYRO", code == 3 and "MOSS_GYRO" in msg, msg)
msg, code = GB.prior_check(OFF_LOG)
check("'prior: gyro=False': the bench goes on", code is None and "OFF" in msg, msg)
msg, code = GB.prior_check(MUTE_LOG)
check("a log that says nothing of the prior: a warning naming MOSS_GYRO, the bench goes on",
      code is None and msg.startswith("WARNING") and "MOSS_GYRO" in msg, msg)
msg, code = GB.prior_check(None)
check("no run log at all: a warning, the bench goes on", code is None and msg.startswith("WARNING"), msg)
STATE_P = PRIOR_DIR / "state"
for name, mtime, src in (("run-old", T0, ON_LOG), ("run-new", T0 + 60, OFF_LOG)):
    (STATE_P / "dimos" / "logs" / name).mkdir(parents=True)
    dst = STATE_P / "dimos" / "logs" / name / "main.jsonl"
    dst.write_text(Path(src).read_text())
    os.utime(dst, (mtime, mtime))
check("the newest run's log is the one read (an older run with the prior on does not count)",
      GB.newest_run_log({"XDG_STATE_HOME": str(STATE_P)}) == str(STATE_P / "dimos" / "logs" / "run-new" / "main.jsonl"),
      f"{GB.newest_run_log({'XDG_STATE_HOME': str(STATE_P)})}")
_tree = ast.parse((ROOT / "moss_dimos" / "lidar_odometry.py").read_text())
_axis = None
for _node in ast.walk(_tree):
    if isinstance(_node, ast.FunctionDef) and _node.name == "__init__":
        _args = _node.args
        for _a, _d in zip(_args.kwonlyargs + _args.args[len(_args.args) - len(_args.defaults):],
                          _args.kw_defaults + list(_args.defaults)):
            if _a.arg == "gyro_axis" and isinstance(_d, ast.Constant):
                _axis = _d.value
check("the axis under test is the one the odometry's prior reads (LidarOdometry gyro_axis)",
      GB.GYRO_AXIS == _axis == "-y", f"bench {GB.GYRO_AXIS!r}, odometry {_axis!r}")


# ── H. gyro, a recorded sequence ────────────────────────────────────────
class FakeLC:
    """The LCM handle the bench subscribes on: the handlers are kept, nothing spins."""

    def __init__(self):
        self.handlers = {}

    def subscribe(self, channel, handler):
        self.handlers[channel] = handler

    def handle_timeout(self, ms):
        time.sleep(ms / 1000.0)


class Recorder:
    def __init__(self):
        self.sent = []

    def broadcast(self, _who, msg):
        self.sent.append(msg)


print("H. gyro, recorded: Imu and PoseStamped messages through the bus callbacks -> a known sign, a known verdict")
if not HAVE_DIMOS:
    print(f"  -- SKIP H and I: missing {missing('dimos', 'dimos_lcm')}")
else:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.Imu import Imu

    def recorded_rung(lc, deg, sign_y=-1.0, rate_deg_s=60.0, t0=T0, hz=200.0, gap_at=None):
        """The messages of one rung, through the subscribed handlers: the gyro
        reads `sign_y` * the yaw rate on its y axis (the upright mount: y points
        down, so a left turn reads NEGATIVE y), the odometry the yaw itself."""
        w = math.radians(rate_deg_s) * (1 if deg > 0 else -1)
        n = int(round(abs(deg) / rate_deg_s * hz))
        for k in range(n + 1):
            t = t0 + k / hz
            if gap_at is not None and gap_at[0] <= t - t0 < gap_at[1]:
                continue
            m = Imu(angular_velocity=Vector3(0.0, sign_y * w, 0.0), ts=t, frame_id="camera_imu_optical_frame")
            lc.handlers["/imu#sensor_msgs.Imu"]("/imu#sensor_msgs.Imu", m.lcm_encode())
            if k % 20 == 0 or k == n:
                yaw = w * (k / hz)
                p = PoseStamped(ts=t, frame_id="world", position=[0.0, 0.0, 0.0],
                                orientation=[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
                lc.handlers["/odom#geometry_msgs.PoseStamped"]("/odom#geometry_msgs.PoseStamped", p.lcm_encode())

    def measure(deg, **kw):
        lc = FakeLC()
        b = GB.Bench(lc=lc, twist=Recorder(), start=False)
        p = PoseStamped(ts=T0, frame_id="world", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
        lc.handlers["/odom#geometry_msgs.PoseStamped"]("/odom#geometry_msgs.PoseStamped", p.lcm_encode())
        b.reset()
        _, y0 = b.snapshot()
        recorded_rung(lc, deg, **kw)
        gyro, y1 = b.snapshot()
        return math.degrees(gyro), GB.wrap_deg(y1 - y0)

    gl, ll = measure(+30.0)
    gr, lr = measure(-30.0)
    check("upright mount, +30 deg left: gyro(-y) +30.0 deg, lidar +30.0 deg",
          abs(gl - 30.0) < 0.2 and abs(ll - 30.0) < 0.2, f"gyro {gl:+.2f}, lidar {ll:+.2f}")
    check("and -30 deg right: gyro -30.0 deg, lidar -30.0 deg", abs(gr + 30.0) < 0.2 and abs(lr + 30.0) < 0.2,
          f"gyro {gr:+.2f}, lidar {lr:+.2f}")
    msg, code = GB.verdict([("+30 deg (left)", gl, ll, "lidar"), ("-30 deg (right)", gr, lr, "lidar")])
    check("-> CORRECT, exit 0", "CORRECT" in msg and code == 0, msg)
    fl, fll = measure(+30.0, sign_y=+1.0)
    fr, flr = measure(-30.0, sign_y=+1.0)
    msg, code = GB.verdict([("+30 deg (left)", fl, fll, "lidar"), ("-30 deg (right)", fr, flr, "lidar")])
    check("a camera mounted upside down (y up): gyro -30 deg against lidar +30 deg -> SIGN FLIPPED 'y', exit 2",
          abs(fl + 30.0) < 0.2 and "SIGN FLIPPED" in msg and "'y'" in msg and code == 2, f"{fl:+.2f} vs {fll:+.2f}: {msg}")
    gg, _ = measure(+30.0, gap_at=(0.2025, 0.3525))
    check("a 0.155 s hole in the gyro stream adds nothing (the prior's 0.1 s gap rule): "
          "(0.500 - 0.155) s x 60 deg/s = 20.7 deg", abs(gg - 20.7) < 0.05, f"{gg:+.2f} deg")

    # ── I. a rung on a rover that pivots at full power ──────────────────
    print("I. gyro, a rung: the pivot ends on the lidar yaw (the tracks pivot at full power whatever rate is asked)")

    class FullPowerRover:
        """Every pivot command turns the body at 250 deg/s (MOSS_PIVOT_PCT 100),
        whatever wz says; zero stops it. Sim time moves with sleep()."""

        def __init__(self, bench, deg_s=250.0, sign_y=-1.0, moves=True, lidar_gain=1.0):
            self.b, self.deg_s, self.sign_y, self.moves = bench, deg_s, sign_y, moves
            self.lidar_gain = lidar_gain      # the share of the turn the lidar odometry follows
            self.t, self.yaw, self.w = T0, 0.0, 0.0
            self.commands = 0

        def broadcast(self, _who, msg):
            wz = float(msg.angular.z)
            self.commands += wz != 0.0
            self.w = (math.radians(self.deg_s) * (1 if wz > 0 else -1)) if (wz and self.moves) else 0.0

        def clock(self):
            return self.t

        def sleep(self, s):
            steps = max(1, int(round(s * 200)))
            for _ in range(steps):
                self.t += s / steps
                self.yaw += self.w * s / steps
                self.b.on_imu(-(self.sign_y * self.w), self.t)      # the gyro reads sign_y * w on y; mapped "-y"
                if int(round((self.t - T0) * 200)) % 20 == 0:
                    self.b.on_odom(self.lidar_gain * self.yaw)

    b = GB.Bench(lc=FakeLC(), twist=Recorder(), start=False)
    rover = FullPowerRover(b)
    b.twist = rover
    b.on_odom(0.0)
    took, ended = b.turn(+GB.TURN_RAD_S, clock=rover.clock, sleep=rover.sleep)
    turned = math.degrees(rover.yaw)
    timed = 250.0 * GB.TURN_DEG / math.degrees(GB.TURN_RAD_S)
    check(f"asked 0.3 rad/s for 30 deg, the body pivots at 250 deg/s: the rung stops at {turned:.0f} deg "
          f"(30 + at most one 0.1 s command), not the {timed:.0f} deg a rung timed at 0.3 rad/s would turn",
          30.0 <= turned <= 30.0 + 25.0 + 1.0 and took < 0.5 and ended == "lidar",
          f"{turned:.1f} deg in {took:.2f} s, ended on {ended}")
    check("the last command sent is a zero twist", isinstance(b.twist, FullPowerRover) and rover.w == 0.0)
    b.on_odom(rover.yaw)
    b2 = GB.Bench(lc=FakeLC(), twist=Recorder(), start=False)
    r2 = FullPowerRover(b2)
    b2.twist = r2
    b2.on_odom(0.0)
    results = GB.run_rungs(b2, settle_s=1.0, sleep=r2.sleep, clock=r2.clock)
    msg, code = GB.verdict(results)
    check("both rungs on that rover, the upright mount: CORRECT, exit 0",
          code == 0 and all(30.0 <= abs(l) <= 56.0 and abs(g / l - 1.0) < 0.05 and e == "lidar"
                            for _, g, l, e in results),
          " | ".join(f"{lb}: gyro {g:+.1f} lidar {l:+.1f} ({e})" for lb, g, l, e in results) + f" -> {msg[:40]}")
    b3 = GB.Bench(lc=FakeLC(), twist=Recorder(), start=False)
    r3 = FullPowerRover(b3, moves=False)
    b3.twist = r3
    b3.on_odom(0.0)
    results = GB.run_rungs(b3, settle_s=1.0, sleep=r3.sleep, clock=r3.clock)
    msg, code = GB.verdict(results)
    check(f"tracks off the ground (nothing turns): each rung gives up after {GB.MAX_RUNG_S:.0f} s, INCONCLUSIVE, exit 3",
          code == 3 and "INCONCLUSIVE" in msg and r3.commands >= 2 * int(GB.MAX_RUNG_S / 0.1) - 2,
          f"{r3.commands} commands, {msg}")
    b4 = GB.Bench(lc=FakeLC(), twist=Recorder(), start=False)
    r4 = FullPowerRover(b4, lidar_gain=0.10)
    b4.twist = r4
    b4.on_odom(0.0)
    results = GB.run_rungs(b4, settle_s=1.0, sleep=r4.sleep, clock=r4.clock)
    msg, code = GB.verdict(results)
    stop = GB.GYRO_STOP_RATIO * GB.TURN_DEG
    check(f"a lidar that follows 10 % of a full-power pivot: each rung ends on the GYRO at {stop:.0f} deg "
          f"(+ one 0.1 s command), not after {GB.MAX_RUNG_S:.0f} s of pivot",
          all(e == "gyro" and stop <= abs(g) <= stop + 25.0 + 1.0 for _, g, _l, e in results),
          " | ".join(f"{lb}: gyro {g:+.1f} lidar {l:+.1f} ({e})" for lb, g, l, e in results))
    check("and the verdict is INCONCLUSIVE (exit 3, relaunch at MOSS_PIVOT_PCT=50), never 'axis mapping WRONG'",
          code == 3 and "INCONCLUSIVE" in msg and "MOSS_PIVOT_PCT=50" in msg and "WRONG" not in msg, msg)
    check("the twist the bench builds is a pure pivot (dimOS Twist, wz only)",
          isinstance(Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.3]).angular.z, float))

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
