#!/usr/bin/env python3
"""The arm's pick-and-drop sequence, checked against the keep-out guard before it moves.

The sequence (config/arm_drop_poses.json) is a list of joint-space moves
between named poses: from the drive pose to a travel pose, down to the pick,
the gripper closing, up through lift to clear (the arm folded high over the
base), the base turned to the drop pose over the bin, the gripper opening,
back to clear, and from clear to the drive pose. Each move is a straight line
in joint space, timed by a quintic smootherstep (zero speed and acceleration
at both ends); the pan is never wrapped: the route turns the way the angles
say, never the short way round.

    tools/moss_arm_drop.py plan                       # the guard on every move, one line per move
    tools/moss_arm_drop.py plan --suggest             # also the smallest change that would pass, for each refusal
    tools/moss_arm_drop.py plan --as-delivered        # the poses as delivered, without the shifts and via poses
    tools/moss_arm_drop.py plan --mujoco auto         # also every sample posed in the CAD scene: contacts, tool point, shoulder load

Options
  --poses       the sequence (config/arm_drop_poses.json)
  --table       the keep-out table (config/arm_keepout.json)
  --calib       the arm's calibration (config/arm_calibration.json): the drive pose,
                the servo IDs and their ranges
  --clearance   grid steps kept clear of any forbidden grid point (1): the table's
                forbidden points grown by that many steps along each joint
  --tolerance   ticks: the box rule's margin around every sample (25)
  --speed       ticks per second: the fastest any joint may go (400); a move the
                sequence times faster is stretched; above 400 needs --clearance 1
  --as-delivered  the poses without their shift_deg and the moves without their via poses
  --suggest     for each refused pose, the smallest shift (whole degrees, one or
                two joints) that passes; for each refused move between two poses
                that pass, the via pose (a point of the move, shifted the same way)
                that splits it into two moves that pass
  --mujoco      a MOSS_R31.xml scene (auto: MOSS_ARM_CAD, else ~/moss-*/cad/*/MOSS_R31.xml)

What plan checks
  The guard's table (moss_dimos/arm_guard.py) with its forbidden grid points
  grown by --clearance. Every move is cut into samples, servo ticks rounded as
  they would be sent, no joint moving more than a quarter of a grid step from
  one sample to the next. Each sample: inside the calibrated range of every
  joint, and allowed by the guard's rule. Each pair of samples: the straight
  move between them allowed (segment_allowed), and the box rule of
  tools/moss_arm_sweep.py: every grid point of the box the pair spans, each
  end widened by --tolerance (the drive pose's end by nothing: it is the pose
  itself), is free. The first sample that fails names the reason and the
  rover part (the nearest forbidden grid point of the table itself when the
  refusal comes from the clearance). Exit status 0 when every move passes,
  1 when one is refused, 2 when the inputs do not load.

  The timing: each move keeps the sequence's duration unless a joint would go
  faster than --speed at the quintic's peak (1.875 times the mean speed):
  then it is stretched to that speed. The table knows poses, not overshoot:
  above 400 ticks per second the plan needs --clearance 1, as the sweep does.

  --mujoco poses every sample again (finer: one degree per joint at most) in
  the CAD scene, as tools/moss_arm_keepout.py builds it (the rover's parts,
  the arm with its fingers swept over every opening, the shoulder on its own,
  and the can of the table while the gripper holds it), and counts the
  contacts closer than the table's margin: zero expected. It also gives, for
  each pose, the tool point (in the model's world frame), its horizontal
  distance from the pan axis, and the shoulder's static load: the gravity
  torque of the links below the shoulder joint, from the model's masses
  (an order of magnitude: the model's masses are not weighed parts). It
  needs mujoco and runs at nice 10, one process.
"""
from __future__ import annotations

import argparse
import copy
import glob
import itertools
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import moss_arm_sweep as W  # noqa: E402  (the planner's clearance and box rule, the calibration loader)

POSES = ROOT / "config" / "arm_drop_poses.json"
FORMAT = "moss-arm-drop/1"
TICKS_PER_REV = 4096
SAMPLES_PER_STEP = 4            # a move is cut so that no joint moves more than 1/4 of a grid step between samples
QUINTIC_PEAK = 1.875            # the quintic smootherstep's peak speed over its mean speed
MJ_STEP_DEG = 1.0               # --mujoco: at most this many degrees per joint between two posed samples
SHIFT_MAX_DEG = 30              # --suggest: the largest shift tried, whole degrees
VIA_FRACTIONS = [0.5, 0.45, 0.55, 0.4, 0.6, 0.35, 0.65, 0.3, 0.7, 0.25, 0.75, 0.2, 0.8, 0.15, 0.85, 0.1, 0.9]
TCP_IN_TOOL_M = (0.0, 0.0, -0.035)   # the tool point, in the gripper's tool frame (35 mm along the approach)
TOOL_BODY = "nc90_tool"
LOAD_JOINT = "shoulder_lift"
PAN_JOINT = "shoulder_pan"
GRAVITY = 9.81


class Refused(Exception):
    """The inputs do not make a plan."""


def quintic(u: float) -> float:
    u = min(max(float(u), 0.0), 1.0)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def cubic(u: float) -> float:
    u = min(max(float(u), 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


# ── the sequence ──

@dataclass
class Pose:
    name: str
    ticks: dict                 # as used: the shift applied unless --as-delivered
    delivered: dict             # ticks of the pose as delivered
    shift_deg: dict = field(default_factory=dict)
    added: bool = False         # a pose of ours (drive, a via pose), not of the delivered sequence


@dataclass
class Move:
    phase: str
    a: Pose
    b: Pose
    duration_s: float           # the sequence's own duration for this move
    gap_mm: tuple               # the jaw gap at the start and at the end
    holds_can: bool             # the gripper holds the can during this move
    added: bool = False


def gap_to_ticks(gripper: dict, gap_mm: float) -> int:
    """The gripper's goal for a jaw gap: the straight line through its open and grasp points."""
    o, g = gripper["open"], gripper["grasp"]
    k = (float(o["ticks"]) - float(g["ticks"])) / (float(o["gap_mm"]) - float(g["gap_mm"]))
    return int(round(float(g["ticks"]) + k * (float(gap_mm) - float(g["gap_mm"]))))


def load_doc(path) -> dict:
    try:
        doc = json.loads(Path(path).read_text())
    except (OSError, ValueError) as e:
        raise Refused(f"the sequence {path} does not load: {e}") from e
    if doc.get("format") != FORMAT:
        raise Refused(f"{path} is not a drop sequence (format {doc.get('format')!r}, want {FORMAT!r})")
    return doc


def build(doc: dict, jm, joints, drive: dict, as_delivered: bool = False) -> list:
    """The moves, in order, from the parsed sequence (angles in model radians,
    turned into ticks through jm, the table's JointMap): each via pose splits
    its move into two, the duration shared in proportion to each piece's
    largest joint move."""
    joints = list(joints)
    if list(doc.get("joints", [])) != joints:
        raise Refused(f"the sequence's joints {doc.get('joints')} are not the table's {joints}")

    def ticks(rad, shift):
        if len(rad) != len(joints):
            raise Refused(f"{len(rad)} angles for {len(joints)} joints")
        unknown = [j for j in shift if j not in joints]
        if unknown:
            raise Refused(f"shift_deg names {unknown}")
        return {j: jm.rad_to_ticks(j, float(v) + math.radians(float(shift.get(j, 0.0)))) for j, v in zip(joints, rad)}

    poses = {"drive": Pose("drive", dict(drive), dict(drive), added=True)}
    for name, p in doc["poses"].items():
        shift = {} if as_delivered else {j: float(v) for j, v in (p.get("shift_deg") or {}).items()}
        poses[name] = Pose(name, ticks(p["rad"], shift), ticks(p["rad"], {}), shift)
    moves = []
    for k, s in enumerate(doc["sequence"]):
        for key in ("from", "to"):
            if s[key] not in poses:
                raise Refused(f"move {k} ({s.get('phase')}): no pose {s[key]!r}")
        a, b = poses[s["from"]], poses[s["to"]]
        gap = tuple(float(v) for v in s.get("gap_mm", (None, None)))
        pts = [a]
        for i, v in enumerate([] if as_delivered else (s.get("via") or [])):
            t = ticks(v["rad"], {})
            pts.append(Pose(f"via {a.name}>{b.name}" + (f" {i + 1}" if i else ""), t, t, added=True))
        pts.append(b)
        spans = [max(abs(q.ticks[j] - p.ticks[j]) for j in joints) for p, q in zip(pts, pts[1:])]
        total = sum(spans)
        for p, q, d in zip(pts, pts[1:], spans):
            share = d / total if total else 1.0 / len(spans)
            moves.append(Move(s["phase"], p, q, float(s["duration_s"]) * share, gap,
                              bool(s.get("holds_can", False)), bool(s.get("added", False))))
    return moves


def samples(a: dict, b: dict, joints, step: dict, per_step: int = SAMPLES_PER_STEP) -> list:
    """The move a -> b cut so that no joint moves more than step[j] / per_step
    ticks between two samples, rounded to ticks (half up, in exact integers:
    samples(b, a) is samples(a, b) reversed); a and b included."""
    most = {j: max(1, int(step[j] // per_step)) for j in joints}     # ticks: the rounding never steps past it
    n = max(1, max(int(math.ceil(abs(b[j] - a[j]) / most[j])) for j in joints))
    return [{j: (2 * (int(a[j]) * n + (int(b[j]) - int(a[j])) * k) + n) // (2 * n) for j in joints} for k in range(n + 1)]


def peak_speed(m: Move, joints) -> float:
    """Ticks per second at the quintic's peak, on the joint that moves most."""
    d = max(abs(m.b.ticks[j] - m.a.ticks[j]) for j in joints)
    return QUINTIC_PEAK * d / m.duration_s if m.duration_s > 0 else (math.inf if d else 0.0)


def stretched(m: Move, joints, speed: float) -> float:
    d = max(abs(m.b.ticks[j] - m.a.ticks[j]) for j in joints)
    return max(m.duration_s, QUINTIC_PEAK * d / speed) if d else m.duration_s


# ── the checks ──

class Checker:
    """The guard's rule, segment_allowed and the sweep's box rule, on the
    table's forbidden grid points grown by `clearance` grid steps."""

    def __init__(self, guard, calibration: dict, tolerance: int = 25, clearance: int = 1):
        self.guard, self.tol, self.clearance = guard, int(tolerance), int(clearance)
        model = W.SpeedModel(W.SLOW_SPEED, 8, self.tol)
        grown = W.Planner(guard, calibration, model, self.tol, self.clearance)   # the sweep's clearance
        self.used = copy.copy(guard)
        self.used.occ = grown.occ_used
        self.used._targets = None
        self.p = W.Planner(self.used, calibration, model, self.tol, 0)           # its box rule on that table
        self.joints = list(guard.joints)
        self.drive = dict(self.p.drive)
        self.ranges = dict(self.p.ranges)
        self.step = {j: s for j, s in zip(self.joints, self.p.step_ticks)}

    def rad(self, t: dict) -> dict:
        return self.p.rad(t)

    def why(self, t: dict) -> str:
        """Why a pose is refused: the table's own why() where the table refuses
        it, else the forbidden grid point of the table the clearance grew from."""
        q = self.rad(t)
        if not self.guard.allowed(q):
            return self.guard.why(q)
        f = self.guard._fracs(q)
        if self.used._outside(f) is not None:
            return self.used.why(q)
        return self._grown_why(self.guard._axes(f), f)

    def _grown_why(self, axes, f) -> str:
        c = self.clearance
        idx = [np.arange(max(min(a) - c, 0), min(max(a) + c, n - 1) + 1) for a, n in zip(axes, self.guard.shape)]
        sub = self.guard.occ[np.ix_(*idx)]
        pts = np.argwhere(sub) + np.array([i[0] for i in idx])
        if not len(pts):
            return "refused (no forbidden grid point of the table nearby: check the clearance)"
        d = np.abs(pts - f).max(axis=1)
        p = tuple(int(x) for x in pts[int(np.argmin(d))])
        q = {j: float(g[p[k]]) for k, (j, g) in enumerate(zip(self.joints, self.guard.grids))}
        return (f"within {c} grid step(s) of a forbidden grid point (clearance {c}): "
                + self.guard.why(q).replace("forbidden: ", ""))

    def in_range(self, t: dict) -> str | None:
        out = [f"{j} {t[j]} outside {lo}..{hi}" for j, (lo, hi) in self.ranges.items() if not lo <= t[j] <= hi]
        return "; ".join(out) or None

    def robust(self, t: dict) -> bool:
        """The pose and its box widened by the tolerance pass (a pose to stop at)."""
        return self.in_range(t) is None and self.used.allowed(self.rad(t)) and self.p.box_free(t, t)

    def pair(self, a: dict, b: dict) -> str | None:
        """None when the step a -> b passes every check, else why not."""
        for t in (a, b):
            r = self.in_range(t)
            if r:
                return f"outside the calibrated range: {r}"
            if not self.used.allowed(self.rad(t)):
                return f"the pose is refused: {self.why(t)}"
        if not self.used.segment_allowed(self.rad(a), self.rad(b)):
            return f"segment_allowed refuses the step: {self.why(b)}"
        if not self.p.box_free(a, b):
            return f"the box rule refuses the step (a forbidden grid point within {self.tol} ticks): {self._box_why(a, b)}"
        return None

    def _box_why(self, a: dict, b: dict) -> str:
        qa, qb = self.rad(a), self.rad(b)
        pad = self.tol * 2.0 * math.pi / TICKS_PER_REV
        pa = 0.0 if a == self.drive else pad
        pb = 0.0 if b == self.drive else pad
        idx = []
        for j, g in zip(self.joints, self.guard.grids):
            lo, hi = min(qa[j] - pa, qb[j] - pb), max(qa[j] + pa, qb[j] + pb)
            tl, th = W._touched(W._frac(g, lo), len(g)), W._touched(W._frac(g, hi), len(g))
            if tl is None or th is None:
                return f"{j} leaves the table"
            idx.append(np.arange(tl[0], th[1] + 1))
        sub = self.used.occ[np.ix_(*idx)]
        pts = np.argwhere(sub) + np.array([i[0] for i in idx])
        if not len(pts):
            return "nothing found"
        f = np.array([W._frac(g, (qa[j] + qb[j]) / 2.0) for j, g in zip(self.joints, self.guard.grids)])
        p = tuple(int(x) for x in pts[int(np.argmin(np.abs(pts - f).max(axis=1)))])
        q = {j: float(g[p[k]]) for k, (j, g) in enumerate(zip(self.joints, self.guard.grids))}
        if self.guard.occ[p]:
            return self.guard.why(q).replace("forbidden: ", "")
        return self._grown_why([(i,) for i in p], np.array(p, dtype=float))

    def move(self, a: dict, b: dict) -> tuple[int, str | None, float | None]:
        """(samples, why the first failing step fails or None, where along the move 0..1)."""
        pts = samples(a, b, self.joints, self.step)
        n = len(pts) - 1
        for k in range(n):
            why = self.pair(pts[k], pts[k + 1])
            if why:
                return n + 1, why, (k + 1) / n if n else 1.0
        if n == 0:
            why = self.pair(pts[0], pts[0])
            if why:
                return 1, why, 0.0
        return n + 1, None, None


# ── the smallest change ──

def _shifts(max_deg: int):
    """(joint indices, degrees) for one or two joints, by the largest shift
    first, then the sum, whole degrees up to max_deg."""
    out = []
    vals = list(range(-max_deg, max_deg + 1))
    for k in (1, 2):
        for js in itertools.combinations(range(5), k):
            for combo in itertools.product(vals, repeat=k):
                if 0 in combo:
                    continue
                out.append((max(map(abs, combo)), sum(map(abs, combo)), k, js, combo))
    out.sort(key=lambda x: x[:3])
    return out


def _shifted(c: Checker, t: dict, js, combo) -> dict:
    """t with joints js turned by combo degrees (through the table's JointMap)."""
    out = dict(t)
    jm = c.guard.joint_map
    for i, d in zip(js, combo):
        j = c.joints[i]
        out[j] = jm.rad_to_ticks(j, jm.ticks_to_rad(j, t[j]) + math.radians(d))
    return out


def suggest_pose(c: Checker, t: dict, max_deg: int = SHIFT_MAX_DEG):
    """The smallest shift ({joint: degrees}) after which the pose passes with
    its tolerance box, or None."""
    for m, _, _, js, combo in _shifts(max_deg):
        if c.robust(_shifted(c, t, js, combo)):
            return {c.joints[i]: d for i, d in zip(js, combo)}
    return None


def suggest_via(c: Checker, a: dict, b: dict, max_deg: int = SHIFT_MAX_DEG):
    """A via pose (ticks) on the move a -> b, shifted by the smallest whole
    degrees on one or two joints, that splits it into two moves that pass;
    (via ticks, fraction, shift) or None."""
    shifts = [(0, 0, 0, (), ())] + _shifts(max_deg)
    for m in range(0, max_deg + 1):
        level = [s for s in shifts if s[0] == m]
        for f in VIA_FRACTIONS:
            base = {j: int(round(a[j] + (b[j] - a[j]) * f)) for j in c.joints}
            for _, _, _, js, combo in level:
                v = _shifted(c, base, js, combo)
                if not c.robust(v):
                    continue
                if not (c.used.segment_allowed(c.rad(a), c.rad(v)) and c.used.segment_allowed(c.rad(v), c.rad(b))):
                    continue
                if c.move(a, v)[1] is None and c.move(v, b)[1] is None:
                    return v, f, {c.joints[i]: d for i, d in zip(js, combo)}
    return None


# ── the plan ──

@dataclass
class Verdict:
    move: Move
    samples: int
    why: str | None
    at: float | None
    duration_s: float
    peak: float


def check_moves(c: Checker, moves: list, speed: float) -> list:
    out = []
    for m in moves:
        n, why, at = c.move(m.a.ticks, m.b.ticks)
        out.append(Verdict(m, n, why, at, stretched(m, c.joints, speed), peak_speed(m, c.joints)))
    return out


def _deg(t: dict, c: Checker) -> str:
    return " ".join(f"{math.degrees(v):+7.1f}" for v in c.rad(t).values())


def print_plan(c: Checker, verdicts: list, doc: dict, speed: float, say=print) -> None:
    say(f"table: {c.guard.summary()}")
    say(f"checks: clearance {c.clearance} grid step(s) (grid steps "
        + ", ".join(f"{j} {s:.0f}" for j, s in c.step.items()) + " ticks), "
        f"box rule tolerance {c.tol} ticks, samples at most a quarter of a grid step apart")
    say("poses (ticks; model degrees " + " ".join(j[:8] for j in c.joints) + "):")
    seen = {}
    for v in verdicts:
        for p in (v.move.a, v.move.b):
            seen.setdefault(p.name, p)
    for p in seen.values():
        sh = (" shifted " + ", ".join(f"{j} {d:+g} deg" for j, d in p.shift_deg.items())
              + f" (delivered {[p.delivered[j] for j in c.joints]})") if p.shift_deg else ""
        say(f"  {p.name:16s} {str([p.ticks[j] for j in c.joints]):29s} {_deg(p.ticks, c)}{sh}")
    g = doc.get("gripper") or {}
    if g:
        say(f"gripper: open {g['open']['gap_mm']:g} mm = {g['open']['ticks']} ticks, grasp {g['grasp']['gap_mm']:g} mm "
            f"= {g['grasp']['ticks']} ticks (force {g.get('force')} at registers {g.get('force_registers')})")
    say(f"{'move':40s} {'samples':>7s} {'seq s':>6s} {'peak t/s':>8s} {'run s':>6s}  verdict")
    t_seq = t_run = 0.0
    for v in verdicts:
        m = v.move
        name = f"{m.phase}: {m.a.name} -> {m.b.name}" if m.a is not m.b and m.a.name != m.b.name else f"{m.phase}: at {m.a.name}"
        if m.gap_mm[0] != m.gap_mm[1] and g:
            name += f" (gap {m.gap_mm[0]:g} -> {m.gap_mm[1]:g} mm)"
        verdict = "OK" if v.why is None else f"REFUSED at {v.at:.2f}: {v.why}"
        say(f"{name[:40]:40s} {v.samples:7d} {m.duration_s:6.2f} {v.peak:8.0f} {v.duration_s:6.2f}  {verdict}")
        t_seq += m.duration_s
        t_run += v.duration_s
    bad = [v for v in verdicts if v.why]
    say(f"sequence: {len(verdicts)} moves, {t_seq:.1f} s as timed, {t_run:.1f} s with no joint over {speed:g} ticks/s; "
        + ("every move passes" if not bad else f"{len(bad)} move(s) refused"))


def print_suggestions(c: Checker, verdicts: list, say=print) -> dict:
    """For each refused pose its smallest shift, then for each refused move
    between poses that pass (shifts applied) a via pose."""
    out = {"shift_deg": {}, "via": {}}
    names = {}
    for v in verdicts:
        for p in (v.move.a, v.move.b):
            names.setdefault(p.name, p)
    fixed = {}
    for name, p in names.items():
        if c.robust(p.ticks):
            fixed[name] = p.ticks
            continue
        s = suggest_pose(c, p.ticks)
        if s is None:
            say(f"suggest: pose {name}: no shift of one or two joints up to {SHIFT_MAX_DEG} deg passes ({c.why(p.ticks)})")
            continue
        fixed[name] = _shifted(c, p.ticks, [c.joints.index(j) for j in s], list(s.values()))
        total = {j: p.shift_deg.get(j, 0.0) + d for j, d in s.items()}
        out["shift_deg"][name] = total
        say(f"suggest: pose {name}: shift " + ", ".join(f"{j} {d:+d} deg" for j, d in s.items())
            + f" -> {[fixed[name][j] for j in c.joints]} ({c.why(p.ticks)})")
    for v in verdicts:
        a, b = v.move.a.name, v.move.b.name
        if a not in fixed or b not in fixed or a == b:
            continue
        ta, tb = fixed[a], fixed[b]
        if c.move(ta, tb)[1] is None:
            continue
        r = suggest_via(c, ta, tb)
        if r is None:
            say(f"suggest: move {a} -> {b}: no via pose found (shifts up to {SHIFT_MAX_DEG} deg)")
            continue
        via, f, s = r
        out["via"][f"{a}->{b}"] = {"ticks": [via[j] for j in c.joints], "at": f, "shift_deg": s}
        say(f"suggest: move {a} -> {b}: a via pose at {f:.2f} of the move"
            + (", shifted " + ", ".join(f"{j} {d:+d} deg" for j, d in s.items()) if s else "")
            + f": {[via[j] for j in c.joints]} ({_deg(via, c)} deg)")
    return out


# ── the CAD scene ──

def find_cad(arg: str) -> Path | None:
    if arg and arg != "auto":
        return Path(arg)
    env = os.environ.get("MOSS_ARM_CAD", "").strip()
    if env:
        return Path(env)
    found = sorted(glob.glob(os.path.expanduser("~/moss-*/cad/*/MOSS_R31.xml")))
    return Path(found[-1]) if found else None


@dataclass
class SceneReport:
    contacts: list              # per move: {"arm": n, "shoulder": n, "can": n, "parts": {name: n}, "samples": n, "errors": n}
    poses: dict                 # name: {"tcp_m", "axis_m", "height_m", "shoulder_nm", "lever_m"}
    margin_m: float
    model: str


def scene_check(c: Checker, moves: list, model_path: Path, say=print) -> SceneReport:
    """Every move posed in the CAD scene, one degree per joint at most between
    samples; contacts closer than the table's margin between the rover's parts
    and the arm (fingers swept), the shoulder (as the table counts it: every
    contact), and the table's can while the gripper holds it."""
    import gc

    import mujoco
    import moss_arm_keepout as K

    if os.nice(0) < 10:
        os.nice(10 - os.nice(0))
    K._lean_malloc()
    layout = K.layout_of(model_path)
    L = K.LAYOUTS[layout]
    margin = float(c.guard.meta.get("margin_m") or L["margin"])
    pay = (c.guard.meta.get("payload") or {})
    payload = K.parse_payload(pay.get("kind", "none")) if pay else None
    if payload and pay.get("diameter_m"):
        payload = dict(payload, diameter_m=float(pay["diameter_m"]), height_m=float(pay["height_m"]))
    manifest = model_path.parent / "parts_manifest.json"
    names = K.load_names(manifest) if manifest.is_file() else {}
    p = K.prepare(model_path, arm_root=L["arm_root"], rover_body=L["rover_body"],
                  rover_parts=tuple(s.strip() for s in L["rover_parts"].split(",") if s.strip()),
                  margin=margin, lidar_xyz=None, joints=c.joints, visual=L["visual"],
                  root_voxel=L["root_voxel"], names=names, payload=payload)
    model = p.model
    jid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in c.joints]
    adr = [int(model.jnt_qposadr[j]) for j in jid]
    is_arm, is_part = K._masks(model, p.arm_ids, p.part_ids)
    is_root, _ = K._masks(model, p.root_ids, p.part_ids)
    pay_ids = list(p.payload_ids.values()) if p.payload_model is not None else []
    is_pay, _ = K._masks(model, pay_ids, p.part_ids) if pay_ids else (None, None)
    part_name = {g: p.display[k] for g, k in zip(p.part_ids, p.part_label)}
    d_arm, d_root = mujoco.MjData(model), mujoco.MjData(p.root_model)
    d_pay = mujoco.MjData(p.payload_model) if pay_ids else None

    def pose(data, base, t):
        data.qpos[:] = base
        for a, j in zip(adr, c.joints):
            data.qpos[a] = c.guard.joint_map.ticks_to_rad(j, t[j])

    step = {j: MJ_STEP_DEG * TICKS_PER_REV / 360.0 for j in c.joints}
    report = []
    for m in moves:
        r = {"arm": 0, "shoulder": 0, "can": 0, "parts": {}, "samples": 0, "errors": 0}
        for t in samples(m.a.ticks, m.b.ticks, c.joints, step, 1):
            r["samples"] += 1
            pose(d_arm, p.base_qpos, t)
            hits = {"arm": K._touch(model, d_arm, is_arm, is_part, margin)}
            pose(d_root, p.base_qpos, t)
            hits["shoulder"] = K._touch(p.root_model, d_root, is_root, is_part, math.inf)
            if m.holds_can and d_pay is not None:
                pose(d_pay, p.payload_qpos, t)
                hits["can"] = K._touch(p.payload_model, d_pay, is_pay, is_part, margin)
            for k, h in hits.items():
                if h is None:
                    r["errors"] += 1
                    continue
                r[k] += len(h)
                for _, g, _ in h:
                    r["parts"][part_name.get(g, str(g))] = r["parts"].get(part_name.get(g, str(g)), 0) + 1
        report.append(r)
    # the poses: tool point, distance from the pan axis, the shoulder's static load
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)
    pan = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, PAN_JOINT)
    lift = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, LOAD_JOINT)
    below = [b for b in range(model.nbody) if _under(model, b, int(model.jnt_bodyid[lift]))]
    poses = {}
    for m in moves:
        for q in (m.a, m.b):
            if q.name in poses:
                continue
            pose(d_arm, p.base_qpos, q.ticks)
            mujoco.mj_kinematics(model, d_arm)
            tcp = d_arm.xpos[tool] + d_arm.xmat[tool].reshape(3, 3) @ np.array(TCP_IN_TOOL_M)
            ax = d_arm.xanchor[pan]
            g = np.array([0.0, 0.0, -GRAVITY])
            tau, mass, mom = 0.0, 0.0, np.zeros(3)
            for b in below:
                tau += float(d_arm.xaxis[lift] @ np.cross(d_arm.xipos[b] - d_arm.xanchor[lift], model.body_mass[b] * g))
                mass += float(model.body_mass[b])
                mom += model.body_mass[b] * d_arm.xipos[b]
            v = mom / mass - d_arm.xanchor[lift]
            v -= (v @ d_arm.xaxis[lift]) * d_arm.xaxis[lift]
            poses[q.name] = {"tcp_m": [round(float(x), 4) for x in tcp],
                             "axis_m": round(float(np.hypot(*(tcp - ax)[:2])), 4),
                             "height_m": round(float(tcp[2] - ax[2]), 4),
                             "shoulder_nm": round(tau, 3), "lever_m": round(float(np.hypot(v[0], v[1])), 4),
                             "mass_kg": round(mass, 3)}
    del d_arm, d_root, d_pay, model, p
    gc.collect()
    K._trim()
    return SceneReport(report, poses, margin, model_path.name)


def _under(model, b: int, top: int) -> bool:
    while b > 0:
        if b == top:
            return True
        b = int(model.body_parentid[b])
    return False


def print_scene(s: SceneReport, verdicts: list, say=print) -> None:
    say(f"CAD scene {s.model}: contacts closer than {s.margin_m * 1000:g} mm (the arm with its fingers swept, "
        "the shoulder, the can while held)")
    for v, r in zip(verdicts, s.contacts):
        m = v.move
        name = f"{m.phase}: {m.a.name} -> {m.b.name}" if m.a.name != m.b.name else f"{m.phase}: at {m.a.name}"
        parts = ", ".join(f"{k} x{n}" for k, n in sorted(r["parts"].items(), key=lambda kv: -kv[1])[:3])
        say(f"  {name[:40]:40s} {r['samples']:4d} poses: arm {r['arm']}, shoulder {r['shoulder']}, "
            f"can {r['can'] if m.holds_can else '-'}" + (f", errors {r['errors']}" if r["errors"] else "")
            + (f" ({parts})" if parts else ""))
    total = sum(r["arm"] + r["shoulder"] + r["can"] + r["errors"] for r in s.contacts)
    say(f"  total contacts: {total}")
    say("poses: the tool point in the model's world frame, its distance from the pan axis, its height over the "
        "pan axis's anchor, the shoulder's static load (links below the shoulder joint, the model's masses)")
    for name, p in s.poses.items():
        say(f"  {name:16s} tool {p['tcp_m']} m, {p['axis_m'] * 100:5.1f} cm from the axis, {p['height_m'] * 100:+5.1f} cm; "
            f"shoulder {p['shoulder_nm']:+.2f} N m ({p['mass_kg']:g} kg, centre of mass {p['lever_m'] * 100:.1f} cm out)")


# ── the command line ──

def plan_main(a, say=print) -> int:
    if a.speed <= 0 or a.speed > W.MAX_SPEED:
        say(f"refused: --speed {a.speed:g}: above 0, at most {W.MAX_SPEED}")
        return 2
    if a.clearance < 0:
        say("refused: --clearance below 0")
        return 2
    try:
        guard, calibration = W.load(a.table, a.calib)
        doc = load_doc(a.poses)
        c = Checker(guard, calibration, a.tolerance, a.clearance)
        moves = build(doc, guard.joint_map, c.joints, c.drive, a.as_delivered)
    except (W.Refused, Refused, KeyError, ValueError) as e:
        say(f"refused: {e}")
        return 2
    verdicts = check_moves(c, moves, a.speed)
    print_plan(c, verdicts, doc, a.speed, say)
    fast = max((v.peak for v in verdicts), default=0.0)
    code = 1 if any(v.why for v in verdicts) else 0
    if a.speed > W.SLOW_SPEED and a.clearance < 1:
        say(f"refused: --speed {a.speed:g} above {W.SLOW_SPEED} needs --clearance 1 or more (overshoot at speed is not in the table)")
        code = 1
    if fast > W.SLOW_SPEED:
        say(f"note: the sequence's own timing peaks at {fast:.0f} ticks/s; the run times above stretch it to {a.speed:g}")
    if a.suggest and code:
        print_suggestions(c, verdicts, say)
    if a.mujoco:
        path = find_cad(a.mujoco)
        if path is None or not path.is_file():
            say(f"refused: --mujoco: no CAD scene ({a.mujoco}; MOSS_ARM_CAD, or ~/moss-*/cad/*/MOSS_R31.xml)")
            return 2
        try:
            import mujoco  # noqa: F401
        except ImportError:
            say("refused: --mujoco needs mujoco (pip install mujoco)")
            return 2
        s = scene_check(c, moves, path, say)
        print_scene(s, verdicts, say)
        if any(r["arm"] + r["shoulder"] + r["can"] + r["errors"] for r in s.contacts):
            code = 1
    return code


def main(argv=None, say=print) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="command", required=True)
    pp = sub.add_parser("plan", help="check the sequence against the guard (nothing moves)")
    pp.add_argument("--poses", default=str(POSES))
    pp.add_argument("--table", default=str(W.TABLE))
    pp.add_argument("--calib", default=str(W.CALIBRATION))
    pp.add_argument("--clearance", type=int, default=1)
    pp.add_argument("--tolerance", type=int, default=25)
    pp.add_argument("--speed", type=float, default=float(W.SLOW_SPEED))
    pp.add_argument("--as-delivered", action="store_true")
    pp.add_argument("--suggest", action="store_true")
    pp.add_argument("--mujoco", default="", help="a MOSS_R31.xml, or auto")
    a = ap.parse_args(argv)
    if a.command == "plan":
        return plan_main(a, say)
    return 2


if __name__ == "__main__":
    sys.exit(main())
