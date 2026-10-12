"""Draw the FILM of a run: trajectory + every event, on the map it built.

Reading a log as event counters hides the behaviour: "35 stuck, 22 bumps"
read three times over did not show a rover walking backwards across a flat in
20 cm steps into the rear wall; one picture showed it in five seconds.

So: after EVERY run, draw the run BEFORE talking about it. Presumption of
stupidity - the picture answers "what did the robot do that was dumb?".

The inputs are the run's recording (odom + global_costmap, written by
MossRecorder) and its dimOS log (main.jsonl: the goal, abandon, escape and
contact lines). The film is a PNG written next to the recording, and the
behavioural verdict is printed.

Run on the robot, or anywhere dimOS and matplotlib are installed (dimOS
decodes the costmap blobs; `.venv/bin/pip install matplotlib` when the venv
lacks it - the script says so instead of drawing):

    .venv/bin/python tools/run_film.py                  # newest exploration run, newest recording
    .venv/bin/python tools/run_film.py --log <dimos log dir>/<run>/main.jsonl \\
                                       --db ~/.local/state/moss/recordings/moss.db \\
                                       --out /tmp/film.png
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import os
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from moss_dimos import robot_profile  # noqa: E402

_DIMOS_STATE = os.path.join(os.environ.get("XDG_STATE_HOME") or "~/.local/state", "dimos")
# The exploration runs' logs: an installed dimOS writes <state>/dimos/logs/<stamp>-moss-dimos-moss-explore/,
# a dimOS from a git clone writes under the clone and only the run registry names that dir.
LOG_GLOB = os.path.join(_DIMOS_STATE, "logs", "*-moss-explore", "main.jsonl")
RUNS_GLOB = os.path.join(_DIMOS_STATE, "runs", "*.json")
# MossRecorder's recordings: moss.db (the last run) and the rotated moss.<stamp>.db
DB_NAME_GLOB = "moss*.db"

# one entry per event family: (regex on the log line, marker, colour, legend label)
EVENTS = {
    "goal":    (re.compile(r"^goal \d+: \(([\d.\-]+), ([\d.\-]+)\)"), "*", "#2e7d32", "goals chosen"),
    "abandon": (re.compile(r"^Goal abandoned"), "o", "#c62828", "goals abandoned (stuck)"),
    "escape":  (re.compile(r"^Contact escape"), "P", "#ef6c00", "contact escapes (0.20 m)"),
    "bump":    (re.compile(r"BUMP received"), "v", "#111111", "contacts received"),
    "arrived": (re.compile(r"^Arrived at goal"), "D", "#1b5e20", "goals reached"),
    # this robot's own contact sources and back-off, so the film says which
    # contact was felt (the IMU) and which was inferred (the encoder stall guard)
    "imu":     (re.compile(r"^IMU contact:"), "^", "#6a1b9a", "IMU contacts (felt)"),
    "stall":   (re.compile(r"^MOSS base: STALL #"), "x", "#ad1457", "encoder stalls (inferred)"),
    "cornered": (re.compile(r"^born cornered"), "s", "#8d6e63", "cornered back-offs"),
}

# A cornered back-off rides the bump reflex (explorer2: "born cornered", then a
# publish on bump): the planner answers with "BUMP received: front contact" at
# once and, once it has reversed, "Contact escape" (direction reverse). Those
# two lines are the back-off's, not a contact's.
CORNERED_BUMP_S = 1.0      # the planner's BUMP line follows the cornered line within this
CORNERED_ESCAPE_S = 6.0    # its escape line follows that BUMP within recovering_planner's ESCAPE_TIMEOUT_S (5 s) + 1 s


def parse_ts(s: str) -> float:
    return datetime.datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


def newest(pattern: str) -> str | None:
    hits = sorted(glob.glob(os.path.expanduser(pattern)), key=os.path.getmtime)
    return hits[-1] if hits else None


def newest_log(log_glob: str = LOG_GLOB, runs_glob: str = RUNS_GLOB) -> str | None:
    """The newest exploration run's main.jsonl: the log root's run dirs, plus the
    log_dir of every registry entry whose blueprint is moss-explore."""
    hits = [p for p in glob.glob(os.path.expanduser(log_glob)) if os.path.isfile(p)]
    for entry in glob.glob(os.path.expanduser(runs_glob)):
        try:
            with open(entry) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            continue
        if not isinstance(doc, dict) or not str(doc.get("blueprint", "")).endswith("moss-explore"):
            continue
        path = os.path.join(str(doc.get("log_dir") or ""), "main.jsonl")
        if os.path.isfile(path):
            hits.append(path)
    hits = sorted(set(hits), key=os.path.getmtime)
    return hits[-1] if hits else None


def newest_db(env=None) -> str | None:
    """The newest recording under <MOSS_STATE_DIR>/recordings (moss.db or a rotated moss.<stamp>.db)."""
    return newest(str(robot_profile.moss_state_dir(env) / "recordings" / DB_NAME_GLOB))


def read_log(path: str):
    """-> (events {family: [(t, xy or None, detail)]}, t0, t1). Goal lines carry
    their own xy; `detail` is the escape's direction ("reverse" / "forward"),
    the abandon's trigger (both from the line's own fields), the contact's side
    ("front" / "rear", from its words), "reversed" for a cornered back-off whose
    escape was found, else None.

    A cornered back-off's own planner lines are folded into its cornered row:
    the first front BUMP within CORNERED_BUMP_S of the cornered line is not a
    contact, and the first reverse escape within CORNERED_ESCAPE_S of that BUMP
    is not a contact escape (nor an abandoned goal: no goal was out). A BUMP
    the planner did not act on (acted=False: it only stopped an escape already
    running) starts no escape of its own."""
    events: dict[str, list] = {k: [] for k in EVENTS}
    t0 = t1 = None
    pending = None    # [index of the cornered row, t of its line, t of its BUMP or None]
    for line in open(path):
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(e, dict):
            continue
        ts = e.get("timestamp")
        if not ts:
            continue
        t = parse_ts(ts)
        t0 = t if t0 is None else t0
        t1 = t
        msg = e.get("event", "")
        for fam, (rx, *_rest) in EVENTS.items():
            m = rx.search(msg) if fam == "bump" else rx.match(msg)
            if not m:
                continue
            xy = (float(m.group(1)), float(m.group(2))) if fam == "goal" else None
            detail = (e.get("direction") if fam == "escape" else e.get("trigger") if fam == "abandon"
                      else ("rear" if "rear" in msg else "front") if fam == "bump" else None)
            if pending is not None:
                row, t_corner, t_bump = pending
                if (fam == "bump" and t_bump is None and detail == "front"
                        and 0.0 <= t - t_corner <= CORNERED_BUMP_S):
                    pending = None if e.get("acted") is False else [row, t_corner, t]
                    continue            # the back-off's own bump
                if (fam == "escape" and t_bump is not None and detail != "forward"
                        and 0.0 <= t - t_bump <= CORNERED_ESCAPE_S):
                    events["cornered"][row] = (events["cornered"][row][0], None, "reversed")
                    pending = None
                    continue            # the back-off's own reverse
            if fam == "cornered":
                pending = [len(events["cornered"]), t, None]
            events[fam].append((t, xy, detail))
    return events, t0, t1


def read_db(path: str, t0: float, t1: float):
    """-> (odom [(t,x,y)], occupied cell centres Nx2) for the run's time window."""
    import sqlite3

    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    odom = db.execute("SELECT ts, pose_x, pose_y FROM odom WHERE ts BETWEEN ? AND ? "
                      "AND pose_x IS NOT NULL ORDER BY ts", (t0 - 5, t1 + 5)).fetchall()
    occ = np.zeros((0, 2))
    row = db.execute("SELECT id FROM global_costmap WHERE ts <= ? ORDER BY ts DESC LIMIT 1",
                     (t1 + 5,)).fetchone()
    if row:
        blob = db.execute("SELECT data FROM global_costmap_blob WHERE id=?", (row[0],)).fetchone()[0]
        g = OccupancyGrid.lcm_decode(blob)
        ys, xs = np.nonzero(np.asarray(g.grid) == 100)
        res = float(g.resolution)
        occ = np.stack([g.origin.position.x + (xs + 0.5) * res,
                        g.origin.position.y + (ys + 0.5) * res], axis=1)
    db.close()
    return odom, occ


def verdict_lines(events: dict, t0: float, t1: float, dist: float, out: str) -> list[str]:
    """The behavioural verdict, one string per printed line."""
    mins = (t1 - t0) / 60.0
    n = {fam: len(rows) for fam, rows in events.items()}
    # what is left in "escape" are contact escapes (read_log folded the cornered
    # back-offs' own): a contact behind the rover escapes forward, the others reverse
    reverse = sum(1 for _t, _xy, direction in events["escape"] if direction != "forward")
    cornered = sum(1 for _t, _xy, done in events["cornered"] if done == "reversed")
    triggers: dict[str, int] = {}
    for _t, _xy, trigger in events["abandon"]:
        triggers[str(trigger or "?")] = triggers.get(str(trigger or "?"), 0) + 1
    lines = [f"THE FILM -> {out}",
             f"  {mins:.1f} min, {dist:.1f} m driven",
             f"  goals: {n['goal']} chosen, {n['arrived']} reached, "
             f"{n['abandon'] + n['escape']} abandoned (stuck + contact: every escape cancels its goal), "
             f"{n['escape']} contact escapes, {n['bump']} contacts",
             f"  back-offs: {reverse + cornered} ({reverse} after a contact, {cornered} cornered); "
             f"{n['escape'] - reverse} forward contact escapes; contacts felt by the IMU {n['imu']}, "
             f"inferred from an encoder stall {n['stall']}"]
    if n["cornered"] > cornered:
        lines.append(f"  cornered with no back-off logged: {n['cornered'] - cornered} "
                     "(no planner BUMP and reverse escape followed the cornered line)")
    if triggers:
        lines.append("  abandon triggers: " + ", ".join(f"{k} {v}" for k, v in sorted(triggers.items())))
    per_min = (n["abandon"] + n["bump"]) / max(mins, 0.1)
    lines.append(f"  verdict: {per_min:.1f} stuck/contact events per minute"
                 + (" - LOOK AT THE MAP before any conclusion" if per_min > 1 else ""))
    return lines


def draw(out: str, title: str, odom, occ, events: dict) -> None:
    """The film: the map's obstacles, the trajectory coloured by time (light ->
    dark), every event where it happened, the start and the end."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    ot = np.array([r[0] for r in odom]); ox = np.array([r[1] for r in odom]); oy = np.array([r[2] for r in odom])

    def pos_at(t):
        i = int(np.argmin(np.abs(ot - t)))
        return ox[i], oy[i]

    fig, ax = plt.subplots(figsize=(11, 9))
    fig.patch.set_facecolor("#fafaf8"); ax.set_facecolor("#fafaf8")
    if len(occ):
        ax.scatter(occ[:, 0], occ[:, 1], s=3, c="#555555", marker="s", linewidths=0)
    tnorm = (ot - ot[0]) / max(ot[-1] - ot[0], 1e-6)
    for i in range(len(ox) - 1):
        ax.plot(ox[i:i + 2], oy[i:i + 2], color=plt.cm.Blues(0.25 + 0.7 * tnorm[i]), lw=1.8, zorder=2)

    handles = [Line2D([], [], color="#7da7d9", lw=2, label="trajectory (light -> dark = time)")]
    if len(occ):
        handles.append(Line2D([], [], marker="s", color="#555555", ls="none", ms=5, label="obstacles (map)"))
    for fam, (_rx, mark, colour, label) in EVENTS.items():
        rows = events[fam]
        if not rows:
            continue
        for t, xy, _detail in rows:
            x, y = xy if xy else pos_at(t)
            ax.plot(x, y, marker=mark, ms=9, color=colour, mec="#fafaf8", mew=0.6, ls="none", zorder=6)
        handles.append(Line2D([], [], marker=mark, color=colour, ls="none", ms=9,
                              label=f"{label} ({len(rows)})"))

    ax.plot(ox[0], oy[0], "o", ms=11, color="#1d4f91", mec="white", mew=1.5, zorder=8)
    ax.annotate("START", (ox[0], oy[0]), textcoords="offset points", xytext=(8, -14),
                fontsize=9, color="#1d4f91", weight="bold")
    ax.plot(ox[-1], oy[-1], "s", ms=11, color="#0d2b52", mec="white", mew=1.5, zorder=8)
    ax.annotate("END", (ox[-1], oy[-1]), textcoords="offset points", xytext=(8, 8),
                fontsize=9, color="#0d2b52", weight="bold")
    ax.set_title(title, fontsize=11, color="#222222")
    ax.legend(handles=handles, loc="upper left", fontsize=9, framealpha=0.95)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, color="#e0e0dc", lw=0.5)
    for sp in ax.spines.values():
        sp.set_color("#cccccc")
    plt.tight_layout()
    plt.savefig(out, dpi=130, facecolor="#fafaf8")
    plt.close(fig)


def film(log: str, db: str, out: str | None = None) -> tuple[int, list[str]]:
    """(exit code, printed lines) for one run: the film written to `out`
    (default: <run dir name>.film.png next to the recording, one per run:
    the recording itself is renamed when the next run rotates it)."""
    try:
        import matplotlib  # noqa: F401  the film is drawn with it
    except ImportError:
        return 1, ["matplotlib is missing, nothing drawn: .venv/bin/pip install matplotlib"]
    run = os.path.basename(os.path.dirname(os.path.abspath(log)))
    out = out or os.path.join(os.path.dirname(os.path.abspath(db)), f"{run}.film.png")
    events, t0, t1 = read_log(log)
    if t0 is None:
        return 1, [f"no dated line in {log} - not a dimOS run log?"]
    odom, occ = read_db(db, t0, t1)
    if len(odom) < 2:
        return 1, [f"no odom rows in {db} for this run's window - wrong db?"]
    ox = np.array([r[1] for r in odom]); oy = np.array([r[2] for r in odom])
    dist = float(np.hypot(np.diff(ox), np.diff(oy)).sum())
    n = {fam: len(rows) for fam, rows in events.items()}
    title = (f"{run} - {(t1 - t0) / 60.0:.0f} min, "
             f"{dist:.1f} m - {n['goal']} goals, {n['arrived']} reached, {n['abandon']} stuck, "
             f"{n['bump']} contacts")
    draw(out, title, odom, occ, events)
    return 0, verdict_lines(events, t0, t1, dist, out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", default=None, help="main.jsonl of the run (default: the newest exploration run)")
    ap.add_argument("--db", default=None, help="recording db (default: the newest moss*.db in the recordings)")
    ap.add_argument("--out", default=None, help="output PNG (default: <run dir name>.film.png next to the db)")
    args = ap.parse_args()

    log = args.log or newest_log()
    db = args.db or newest_db()
    if not log or not db:
        print(f"missing input: log={log} db={db}")
        return 1
    code, lines = film(log, db, args.out)
    for line in lines:
        print(line)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
