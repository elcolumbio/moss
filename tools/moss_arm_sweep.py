#!/usr/bin/env python3
"""Tour the arm along the edge of its keep-out zones, slowly, and log every reading.

The tool drives the SO-101 arm (servos 1 to 5; the gripper, servo 6, is never
commanded) from the drive pose through poses the keep-out guard allows, each
one next to poses the guard forbids, and back to the drive pose. Every
reading of the five servos goes to a JSONL log, so a render of the
collision-zone map can follow the arm frame by frame next to the film of it.

    tools/moss_arm_sweep.py --plan-only --log plan.jsonl      # print the tour, write the log the speed model gives
    tools/moss_arm_sweep.py --dry-run --log dry.jsonl          # the whole run on a fake bus, on the wall clock
    tools/moss_arm_sweep.py --dry-run --fast --log dry.jsonl   # the same on a virtual clock
    tools/moss_arm_sweep.py --log sweep.jsonl                  # the arm

Options
  --port        a /dev/serial/by-id path, or auto (default): MOSS_ARM_PORT, else
                the one link /dev/serial/by-id/usb-1a86_USB_Single_Serial_*
  --table       the keep-out table (config/arm_keepout.json)
  --calib       the arm's calibration (config/arm_calibration.json): the drive
                pose under "poses", the servo IDs and their ranges
  --duration    seconds the whole tour takes, the way back included (25)
  --speed       Goal_Speed written once to the five servos (200, at most 400)
  --acc         Acc written once to the five servos (8)
  --tolerance   ticks: a waypoint is reached when all five are this close (25)
  --timeout     seconds per waypoint before it counts as a fault (12)
  --seed        the random spread's seed: the same inputs give the same tour (0)
  --spread      how much the random spread weighs against the nearest move (0.25)
  --log         the JSONL log (arm_sweep.jsonl)

The plan
  The table's grid cuts the joint space into cells. A cell is free when every
  grid point at its corners is free, and the guard allows any pose inside a
  free cell. A waypoint is the centre of a free cell, rounded to ticks and
  inside the servos' calibrated ranges by the tolerance. A frontier cell is a
  free cell one grid step, along one joint, from a forbidden grid point: the
  edge of a keep-out zone, as close as the table's grid can tell. The
  tour starts at the drive pose and steps from cell to neighbouring frontier
  cell (at most one cell on any joint per move), the nearest by the speed
  model with a small random spread and a penalty for turning a joint back;
  when no unvisited frontier cell is next to the arm, it takes the shortest
  way through free cells to the nearest one. It comes back to the drive pose
  by the shortest way through free cells, in time for --duration. Every move
  passes two checks: segment_allowed on the straight move, and every grid
  point of the box the move spans (both ends widened by the tolerance) must
  be free. The second one is there because the five servos run at the same
  speed: the arm's path is not straight in joint space, but it stays in that
  box. The plan is refused when the drive pose is not allowed (the guard
  says why), when the tolerance is wider than half a grid step, or when no
  frontier cell is in reach in the time given.

The speed model
  Feetech STS servos read Goal_Speed in steps per second (one step is one
  tick, 0.088 degree) and Acc in units of 100 steps per second squared. A
  joint moving d ticks takes d / v + v / a when it reaches full speed, and
  2 sqrt(d / a) when it does not. The run writes the next waypoint as soon
  as every joint is within the tolerance (the last ticks of a move overlap
  the next one), so a move counts until its slowest joint is within the
  tolerance, plus 0.05 s (one poll and the next write). --speed 200 is 17.6
  degrees per second, --acc 8 is 800 ticks per second squared. The fake bus
  moves its servos by the same law.

The run
  The arm must sit within twice the tolerance of the drive pose. Each
  servo's goal is set to its present reading, then the speed and the
  acceleration are written, then the torque comes on. For each waypoint the
  five goals are written and the positions are read every 40 ms until all
  five are within the tolerance, or the timeout passes. Every reading is one
  line of the log:

      {"t": seconds since the start, "ticks": {joint: reported ticks},
       "rad": {joint: model radians, through the table's JointMap},
       "waypoint": the index of the waypoint the arm is heading to,
       "phase": "tour" or "return"}

  A fault stops the tour: a reading the guard refuses (within the tolerance
  of the drive pose excepted: the arm settles there, and the drive pose can
  sit a few ticks from the table's edge), a waypoint not reached in time, a
  move the guard refuses, servos that stop answering, or Ctrl-C. The arm
  stops where it is (each goal set to its present reading), then goes back
  to the drive pose through allowed moves only: to the waypoint it came from
  (the one it was heading to when that move is refused), then the shortest
  way home. When no allowed move leaves the pose it
  stopped at, it holds there. The torque stays on in every case: at the end
  the arm holds the drive pose. Exit status: 0 the tour done, 1 refused
  (nothing moved), 2 a fault and the arm back at the drive pose, 3 a fault
  and the arm holding where it stopped.

  --plan-only prints the tour (each waypoint, the time the speed model gives,
  and the rover part it skirts, from why() on the nearest forbidden grid
  point) and writes the log of a run on the fake bus and a virtual clock, so
  the render can be made before the arm moves. --dry-run runs the whole tool
  on that fake bus, on the wall clock unless --fast.
"""
from __future__ import annotations

import argparse
import glob
import heapq
import itertools
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from moss_arm_calibrate import BAUD, read2  # noqa: E402  (the bus speed and the read retry the calibration uses)
from moss_dimos.arm_guard import ArmGuard  # noqa: E402

TABLE = ROOT / "config" / "arm_keepout.json"
CALIBRATION = ROOT / "config" / "arm_calibration.json"
SERVO_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5}
GRIPPER_ID = 6                  # never written by this tool
REG_TORQUE, REG_ACC, REG_GOAL, REG_SPEED, REG_PRESENT = 40, 41, 42, 46, 56
TICKS_PER_REV = 4096
TICKS_PER_SPEED_UNIT = 1.0      # Goal_Speed: steps per second
TICKS_PER_ACC_UNIT = 100.0      # Acc: 100 steps per second squared per unit
SETTLE_S = 0.05                 # per move: one poll and the next write
POLL_S = 0.04
MAX_SPEED = 400                 # the tour hugs the rover: slow or not at all
BUS_SILENT_S = 1.0              # no complete reading for this long is a fault
REVERSE_S = 0.4                 # the penalty, in seconds, for each joint a move turns back
SEARCH_POPS = 50000             # the way to the nearest frontier cell gives up after this many cells
PORT_GLOB = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_*"
ON_GRID = 1e-6


class Refused(Exception):
    """The plan or the run is refused before anything moves."""


class Fault(Exception):
    """The run stops: the arm holds, then goes home through allowed moves."""


# ── the speed model ──

@dataclass(frozen=True)
class SpeedModel:
    speed: int
    acc: int
    tol: int = 25

    @property
    def v(self) -> float:
        return self.speed * TICKS_PER_SPEED_UNIT

    @property
    def a(self) -> float:
        return self.acc * TICKS_PER_ACC_UNIT

    def joint_time(self, d_ticks: float) -> float:
        d = abs(float(d_ticks))
        if d <= 0.0:
            return 0.0
        if d >= self.v * self.v / self.a:
            return d / self.v + self.v / self.a
        return 2.0 * math.sqrt(d / self.a)

    def reach_time(self, d_ticks: float) -> float:
        """Seconds until a joint moving d ticks is within the tolerance of its goal."""
        d = abs(float(d_ticks))
        if d <= self.tol:
            return 0.0
        peak = min(self.v, math.sqrt(self.a * d))
        braking = peak * peak / (2.0 * self.a)
        if self.tol <= braking:
            return self.joint_time(d) - math.sqrt(2.0 * self.tol / self.a)
        if peak >= self.v:
            return self.joint_time(d) - peak / self.a - (self.tol - braking) / peak
        return math.sqrt(2.0 * (d - self.tol) / self.a)

    def move_time(self, a: dict, b: dict) -> float:
        """A move: until its slowest joint is within the tolerance, plus a poll and a write."""
        return max(self.reach_time(b[j] - a[j]) for j in a) + SETTLE_S


# ── the plan ──

@dataclass
class Waypoint:
    ticks: dict
    kind: str                   # "drive", "frontier" or "pass"
    cell: tuple | None = None
    move_s: float = 0.0         # the speed model's time from the waypoint before
    skirts: str | None = None   # the rover part at the nearest forbidden grid point
    why: str | None = None
    steps_away: float | None = None


@dataclass
class Plan:
    waypoints: list
    duration: float
    model: SpeedModel
    tolerance: int
    seed: int = 0
    notes: list = field(default_factory=list)

    @property
    def total_s(self) -> float:
        return sum(w.move_s for w in self.waypoints)

    @property
    def drive(self) -> dict:
        return self.waypoints[0].ticks


def _frac(grid: np.ndarray, v: float) -> float:
    """v as a fractional index into a uniform grid (past the ends too)."""
    return (float(v) - float(grid[0])) / (float(grid[1]) - float(grid[0]))


def _touched(f: float, n: int) -> tuple[int, int] | None:
    """The grid indices [lo, hi] a fractional index touches, None outside the grid."""
    if f < -ON_GRID or f > n - 1 + ON_GRID:
        return None
    r = round(f)
    if abs(f - r) <= ON_GRID:
        return int(r), int(r)
    return int(math.floor(f)), int(math.floor(f)) + 1


def _shifted(arr: np.ndarray, o) -> np.ndarray:
    """out[c] = arr[c + o], False where c + o leaves the array."""
    out = np.zeros_like(arr)
    src, dst = [], []
    for ok, n in zip(o, arr.shape):
        src.append(slice(max(ok, 0), n + min(ok, 0)))
        dst.append(slice(max(-ok, 0), n - max(ok, 0)))
    out[tuple(dst)] = arr[tuple(src)]
    return out


class Planner:
    """The keep-out table as a graph of free cells, and the tour on it."""

    def __init__(self, guard: ArmGuard, calibration: dict, model: SpeedModel, tolerance: int = 25):
        self.guard, self.model, self.tol = guard, model, int(tolerance)
        self.joints = tuple(guard.joints)
        unknown = [j for j in self.joints if j not in SERVO_IDS]
        if unknown:
            raise Refused(f"the table has joints this tool does not drive: {', '.join(unknown)}")
        self.jm = guard.joint_map
        drive = ((calibration.get("poses") or {}).get("drive") or {}).get("ticks") or {}
        missing = [j for j in self.joints if j not in drive]
        if missing:
            raise Refused(f"the calibration holds no drive pose for {', '.join(missing)} (poses.drive.ticks)")
        self.drive = {j: int(drive[j]) for j in self.joints}
        self.ranges = {}
        for j in self.joints:
            cal = calibration.get(j) if isinstance(calibration.get(j), dict) else {}
            self.ranges[j] = (int(cal.get("range_min", 0)), int(cal.get("range_max", TICKS_PER_REV - 1)))
        self.grids = [np.asarray(g, dtype=float) for g in guard.grids]
        self.step_ticks = [abs(g[1] - g[0]) * TICKS_PER_REV / (2.0 * math.pi) for g in self.grids]
        self.pad_rad = self.tol * 2.0 * math.pi / TICKS_PER_REV
        for j, s in zip(self.joints, self.step_ticks):
            if self.tol >= s / 2.0 - 1.0:
                raise Refused(f"the tolerance ({self.tol} ticks) is wider than half a grid step on {j} ({s / 2.0:.0f} ticks)")

        q_drive = self.rad(self.drive)
        if not guard.allowed(q_drive):
            raise Refused(f"the drive pose {self._fmt(self.drive)} is not allowed: {guard.why(q_drive)}")

        occ = guard.occ
        d = occ.ndim
        cells, near = ~occ, occ.copy()
        for k in range(d):     # near: forbidden, or one grid step from a forbidden point along one joint
            lo = tuple(slice(0, -1) if i == k else slice(None) for i in range(d))
            hi = tuple(slice(1, None) if i == k else slice(None) for i in range(d))
            near[lo] |= occ[hi]
            near[hi] |= occ[lo]
        for k in range(d):     # a cell: the grid points k and k + 1 on every joint
            lo = tuple(slice(0, -1) if i == k else slice(None) for i in range(d))
            hi = tuple(slice(1, None) if i == k else slice(None) for i in range(d))
            cells = cells[lo] & cells[hi]
            near = near[lo] | near[hi]
        self.cshape = cells.shape
        self.cells = cells
        self.centre = []
        in_range = []
        for j, g in zip(self.joints, self.grids):
            c = np.array([self.jm.rad_to_ticks(j, (g[i] + g[i + 1]) / 2.0) for i in range(len(g) - 1)])
            lo, hi = self.ranges[j]
            self.centre.append(c)
            in_range.append((c >= lo + self.tol) & (c <= hi - self.tol))
        span = np.ones(self.cshape, dtype=bool)
        for k, r in enumerate(in_range):
            span &= r.reshape([-1 if i == k else 1 for i in range(d)])
        self.cell_ok = cells & span
        self.frontier = self.cell_ok & near

        f = [_frac(g, q_drive[j]) for j, g in zip(self.joints, self.grids)]
        self.c0 = tuple(int(min(max(math.floor(x), 0), n - 1)) for x, n in zip(f, self.cshape))
        if not cells[self.c0]:
            raise Refused(f"the drive pose sits on the edge of a cell with a forbidden corner: {guard.why(q_drive)}")

        self.offsets = [o for o in itertools.product((-1, 0, 1), repeat=d) if any(o)]
        self.offsets.sort(key=lambda o: sum(map(abs, o)))
        box = {tuple([0] * d): cells}
        for o in self.offsets:
            k = max(i for i, v in enumerate(o) if v)
            prev = tuple(0 if i == k else v for i, v in enumerate(o))
            box[o] = box[prev] & _shifted(box[prev], tuple(o[i] if i == k else 0 for i in range(d)))
        self.N = int(np.prod(self.cshape))
        stride = [int(np.prod(self.cshape[k + 1:])) for k in range(d)]
        self.offs = np.array(self.offsets, dtype=np.int64)
        self.shift = self.offs @ np.array(stride, dtype=np.int64)
        self.edge = np.stack([(box[o] & self.cell_ok & _shifted(self.cell_ok, o)).ravel() for o in self.offsets])
        times = [self.model.reach_time(s + 1.0) for s in self.step_ticks]   # + 1: centres rounded to ticks
        self.cost = np.array([max(times[i] for i, v in enumerate(o) if v) + SETTLE_S for o in self.offsets])
        # from the drive pose (exact, inside c0) to a cell next to c0, or to c0 itself
        self.drive_next = {}
        for o in [tuple([0] * d)] + self.offsets:
            n = tuple(c + v for c, v in zip(self.c0, o))
            if all(0 <= x < s for x, s in zip(n, self.cshape)) and box[o][self.c0] and self.cell_ok[n]:
                self.drive_next[int(np.ravel_multi_index(n, self.cshape))] = o
        del box
        self.frontier_f = self.frontier.ravel()
        self.dist = self._dist_home()

    # ── ticks, radians, cells ──

    def rad(self, ticks: dict) -> dict:
        return {j: self.jm.ticks_to_rad(j, ticks[j]) for j in self.joints}

    def deg(self, ticks: dict) -> dict:
        return {j: math.degrees(v) for j, v in self.rad(ticks).items()}

    def _fmt(self, ticks: dict) -> str:
        return ", ".join(f"{j} {ticks[j]}" for j in self.joints)

    def centre_ticks(self, n: int) -> dict:
        c = np.unravel_index(n, self.cshape)
        return {j: int(self.centre[k][c[k]]) for k, j in enumerate(self.joints)}

    def to_drive_s(self, n: int) -> float:
        return self.model.move_time(self.centre_ticks(n), self.drive) if n in self.drive_next else math.inf

    def parked(self, ticks: dict) -> bool:
        """Within the tolerance of the drive pose: where the arm settles at the end."""
        return all(abs(int(ticks[j]) - self.drive[j]) <= self.tol for j in self.joints)

    def box_free(self, a: dict, b: dict) -> bool:
        """Every grid point of the box a move spans is free: each end widened by
        the tolerance (the drive pose's end by nothing: it is the pose itself)."""
        qa, qb = self.rad(a), self.rad(b)
        pa = 0.0 if a == self.drive else self.pad_rad
        pb = 0.0 if b == self.drive else self.pad_rad
        idx = []
        for j, g in zip(self.joints, self.grids):
            lo = min(qa[j] - pa, qb[j] - pb)
            hi = max(qa[j] + pa, qb[j] + pb)
            tl, th = _touched(_frac(g, lo), len(g)), _touched(_frac(g, hi), len(g))
            if tl is None or th is None:
                return False
            idx.append(np.arange(tl[0], th[1] + 1))
        return not bool(self.guard.occ[np.ix_(*idx)].any())

    def move_refused(self, a: dict, b: dict) -> str | None:
        """None when the move from a to b passes every check, else why not."""
        for name, p in (("from", a), ("to", b)):
            if not self.guard.allowed(self.rad(p)):
                return f"the pose it goes {name} is refused: {self.guard.why(self.rad(p))}"
        if not self.guard.segment_allowed(self.rad(a), self.rad(b)):
            return "segment_allowed refuses the straight move"
        if not self.box_free(a, b):
            return "the box the move spans holds a forbidden grid point"
        return None

    # ── the way home ──

    def _dist_home(self) -> np.ndarray:
        """Seconds from each cell's centre to the drive pose by the speed model,
        moving through free cells, one cell per joint per move (inf: no way)."""
        dist = np.full(self.N, np.inf)
        for n in self.drive_next:
            dist[n] = self.to_drive_s(n)
        N = self.N
        for _ in range(4 * sum(self.cshape)):
            changed = False
            for k in range(len(self.offsets)):
                s, ok, c = int(self.shift[k]), self.edge[k], float(self.cost[k])
                if s > 0:
                    cand, tgt, m = dist[s:] + c, dist[:N - s], ok[:N - s]
                else:
                    cand, tgt, m = dist[:N + s] + c, dist[-s:], ok[-s:]
                better = m & (cand < tgt - 1e-9)
                if better.any():
                    tgt[better] = cand[better]
                    changed = True
            if not changed:
                break
        return dist

    def home_cells(self, n: int) -> list:
        """The cells from n to the drive pose (n excluded), the drive pose as None."""
        out, guard_steps = [], 0
        while True:
            direct = self.to_drive_s(n)
            ok = self.edge[:, n]
            nbr = np.clip(n + self.shift, 0, self.N - 1)
            cand = np.where(ok, self.cost + self.dist[nbr], np.inf)
            k = int(np.argmin(cand))
            if direct <= cand[k] + 1e-9:
                out.append(None)
                return out
            n = int(nbr[k])
            out.append(n)
            guard_steps += 1
            if guard_steps > self.N or not math.isfinite(cand[k]):
                raise Refused("no way home through free cells")

    # ── the tour ──

    def _nearest_frontier(self, cur: int | None, t: float, budget: float, visited: np.ndarray) -> list:
        """The shortest way (cells after cur; cur None: the drive pose) to the
        nearest unvisited frontier cell that still leaves the time to come
        home; [] when none."""
        if cur is None:
            best = {n: self.to_drive_s(n) for n in self.drive_next}
            parent = {n: None for n in self.drive_next}
        else:
            best, parent = {cur: 0.0}, {cur: None}
        heap = [(d, n) for n, d in best.items()]
        heapq.heapify(heap)
        pops = 0
        while heap and pops < SEARCH_POPS:
            dn, n = heapq.heappop(heap)
            if dn > best.get(n, math.inf) + 1e-12:
                continue
            pops += 1
            if n != cur and self.frontier_f[n] and not visited[n] and t + dn + self.dist[n] <= budget + 1e-9:
                path = []
                while n is not None and n != cur:
                    path.append(n)
                    n = parent[n]
                return path[::-1]
            if t + dn > budget:
                continue
            ok = self.edge[:, n]
            for k in np.flatnonzero(ok):
                m = int(n + self.shift[k])
                dm = dn + float(self.cost[k])
                if dm < best.get(m, math.inf) - 1e-12:
                    best[m], parent[m] = dm, n
                    heapq.heappush(heap, (dm, m))
        return []

    def plan(self, duration: float, seed: int = 0, spread: float = 0.25) -> Plan:
        budget = float(duration)
        rng = np.random.default_rng(seed)
        visited = np.zeros(self.N, dtype=bool)
        cells = []          # the tour's cells, in order
        t, prev_o = 0.0, np.zeros(len(self.joints), dtype=np.int64)
        cur = None
        while True:
            if cur is None:     # leaving the drive pose
                nbr = np.array(sorted(self.drive_next), dtype=np.int64)
                if not len(nbr):
                    break
                costs = np.array([self.to_drive_s(int(n)) for n in nbr])
                offs = np.array([self.drive_next[int(n)] for n in nbr], dtype=np.int64)
                ok = np.ones(len(nbr), dtype=bool)
            else:
                nbr = np.clip(cur + self.shift, 0, self.N - 1)
                costs, offs, ok = self.cost, self.offs, self.edge[:, cur]
            m = ok & self.frontier_f[nbr] & ~visited[nbr] & (t + costs + self.dist[nbr] <= budget + 1e-9)
            if m.any():
                idx = np.flatnonzero(m)
                turns = ((offs[idx] * prev_o) < 0).sum(axis=1)
                score = costs[idx] * (1.0 + spread * rng.random(len(idx))) + REVERSE_S * turns
                k = int(idx[int(np.argmin(score))])
                step = [int(nbr[k])]
            else:
                step = self._nearest_frontier(cur, t, budget, visited)
                if not step:
                    break
            for n in step:
                if cur is None:
                    t += self.to_drive_s(n)
                    o = np.array(self.drive_next[n], dtype=np.int64)
                else:
                    o = np.array(np.unravel_index(n, self.cshape)) - np.array(np.unravel_index(cur, self.cshape))
                    t += float(self.cost[self.offsets.index(tuple(int(x) for x in o))])
                visited[n] = True
                cells.append(n)
                prev_o, cur = o, n
        if not any(self.frontier_f[n] for n in cells):
            raise Refused(f"no frontier cell is in reach of the drive pose in {duration:g} s "
                          f"at speed {self.model.speed}")
        cells += self.home_cells(cells[-1])
        wps = [Waypoint(dict(self.drive), "drive")]
        for n in cells:
            if n is None:
                w = Waypoint(dict(self.drive), "drive")
            else:
                w = Waypoint(self.centre_ticks(n), "frontier" if self.frontier_f[n] else "pass",
                             tuple(int(x) for x in np.unravel_index(n, self.cshape)))
            w.move_s = self.model.move_time(wps[-1].ticks, w.ticks)
            wps.append(w)
        for w in wps:
            if w.kind == "frontier":
                self._skirt(w)
        plan = Plan(wps, budget, self.model, self.tol, seed)
        for i in range(1, len(wps)):
            why = self.move_refused(wps[i - 1].ticks, wps[i].ticks)
            if why:
                raise Refused(f"the planned move to waypoint {i} fails its check ({why})")
        return plan

    def _skirt(self, w: Waypoint) -> None:
        """The forbidden grid point nearest the waypoint (grid steps, the most
        off joint first), why() there, and the part it names."""
        c = w.cell
        lo = [max(x - 1, 0) for x in c]
        hi = [min(x + 3, n) for x, n in zip(c, self.guard.shape)]
        sub = self.guard.occ[tuple(slice(a, b) for a, b in zip(lo, hi))]
        pts = np.argwhere(sub) + np.array(lo)
        if not len(pts):
            return
        f = np.array([_frac(g, v) for g, v in zip(self.grids, self.rad(w.ticks).values())])
        diff = np.abs(pts - f)
        key = diff.max(axis=1) * 1e3 + np.sqrt((diff ** 2).sum(axis=1))
        p = tuple(int(x) for x in pts[int(np.argmin(key))])
        q = {j: float(g[p[k]]) for k, (j, g) in enumerate(zip(self.joints, self.grids))}
        w.why = self.guard.why(q)
        w.steps_away = float(diff[int(np.argmin(key))].max())
        k = int(self.guard.cell_part[p])
        names = list(getattr(self.guard, "part_names", []) or [])
        w.skirts = names[k - 1] if 1 <= k <= len(names) else "the rover"


# ── clocks and the fake bus ──

class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, s: float) -> None:
        time.sleep(s)

    def bus(self, s: float) -> None:
        pass


class VirtualClock:
    """Time that moves only when the tool sleeps or talks on the bus. alarm(t, f)
    calls f once the clock passes t (a bench's Ctrl-C, a knock on the arm)."""

    def __init__(self):
        self.t = 0.0
        self.alarms = []

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s
        self._ring()

    def bus(self, s: float) -> None:
        self.t += s
        self._ring()

    def alarm(self, at: float, fn) -> None:
        self.alarms.append((at, fn))

    def _ring(self) -> None:
        due = [a for a in self.alarms if a[0] <= self.t]
        self.alarms = [a for a in self.alarms if a[0] > self.t]
        for _, fn in due:
            fn()


class _FakeServo:
    def __init__(self, pos: float, limits=(0, TICKS_PER_REV - 1)):
        self.pos, self.vel, self.goal = float(pos), 0.0, int(round(pos))
        self.speed, self.acc, self.torque = 0, 0, 0
        self.limits = limits
        self.block = None       # (tick, side): an obstacle the joint cannot pass, on that side of it

    def block_at(self, tick: float) -> None:
        self.block = (float(tick), 1.0 if tick >= self.pos else -1.0)

    def step(self, dt: float) -> None:
        if not self.torque:
            self.vel = 0.0
            return
        vmax = self.speed * TICKS_PER_SPEED_UNIT if self.speed else 3000.0
        amax = self.acc * TICKS_PER_ACC_UNIT if self.acc else 1e6
        e = self.goal - self.pos
        if abs(e) < 0.5 and abs(self.vel) <= amax * dt:
            self.pos, self.vel = float(self.goal), 0.0
            return
        want = math.copysign(min(vmax, math.sqrt(2.0 * amax * abs(e))), e)
        self.vel += max(-amax * dt, min(amax * dt, want - self.vel))
        new = self.pos + self.vel * dt
        if self.block is not None and (new - self.block[0]) * self.block[1] > 0:
            new, self.vel = self.block[0], 0.0
        self.pos = new


class FakeBus:
    """The calls this tool makes on scservo_sdk (PortHandler, PacketHandler and
    COMM_SUCCESS in one object), on servos that follow the speed model."""

    COMM_SUCCESS = 0

    def __init__(self, start: dict, clock, limits: dict | None = None, latency: float = 0.0012):
        self.clock, self.latency = clock, latency
        self.servos = {int(s): _FakeServo(p, (limits or {}).get(int(s), (0, TICKS_PER_REV - 1))) for s, p in start.items()}
        self.writes = []        # (t, id, register, value)
        self.last = clock.now()

    def openPort(self):  # noqa: N802 - the SDK's names
        return True

    def setBaudRate(self, baud):  # noqa: N802
        return True

    def closePort(self):  # noqa: N802
        pass

    def _tx(self) -> None:
        self.clock.bus(self.latency)
        now = self.clock.now()
        left = now - self.last
        while left > 1e-12:
            dt = min(0.002, left)
            for s in self.servos.values():
                s.step(dt)
            left -= dt
        self.last = now

    def ping(self, ph, sid):
        self._tx()
        return 0, (0 if sid in self.servos else -1001), 0

    def read2ByteTxRx(self, ph, sid, addr):  # noqa: N802
        self._tx()
        s = self.servos.get(sid)
        if s is None:
            return 0, -1001, 0
        return {REG_PRESENT: int(round(s.pos)), REG_GOAL: s.goal, REG_SPEED: s.speed}.get(addr, 0), 0, 0

    def write1ByteTxRx(self, ph, sid, addr, value):  # noqa: N802
        self._tx()
        self.writes.append((self.clock.now(), sid, addr, int(value)))
        s = self.servos.get(sid)
        if s is None:
            return -1001, 0
        if addr == REG_TORQUE:
            s.torque = int(value)
        elif addr == REG_ACC:
            s.acc = int(value)
        return 0, 0

    def write2ByteTxRx(self, ph, sid, addr, value):  # noqa: N802
        self._tx()
        self.writes.append((self.clock.now(), sid, addr, int(value)))
        s = self.servos.get(sid)
        if s is None:
            return -1001, 0
        if addr == REG_GOAL:
            s.goal = int(min(max(int(value), s.limits[0]), s.limits[1]))
        elif addr == REG_SPEED:
            s.speed = int(value)
        return 0, 0


# ── the run ──

class Stop:
    """Ctrl-C asks for a stop; the run checks between two bus calls."""

    def __init__(self):
        self.asked = False

    def __call__(self, *_):
        self.asked = True


class Arm:
    def __init__(self, scs, ph, pk, ids: dict):
        self.scs, self.ph, self.pk, self.ids = scs, ph, pk, ids

    def read(self) -> dict | None:
        out = {}
        for j, sid in self.ids.items():
            v = read2(self.scs, self.ph, self.pk, sid, REG_PRESENT)
            if v is None:
                return None
            out[j] = -(v & 0x7FFF) if v & 0x8000 else int(v)
        return out

    def _write(self, sid: int, addr: int, value: int, width: int) -> None:
        if sid == GRIPPER_ID or sid not in self.ids.values():
            raise Fault(f"servo {sid} is not one this tool drives")
        fn = self.pk.write1ByteTxRx if width == 1 else self.pk.write2ByteTxRx
        for _ in range(3):
            try:
                comm, _err = fn(self.ph, sid, addr, int(value))
                if comm == self.scs.COMM_SUCCESS:
                    return
            except Exception:  # noqa: BLE001 - a short packet inside the SDK
                pass
        raise Fault(f"servo {sid} did not take register {addr} = {value}")

    def goals(self, ticks: dict) -> None:
        for j, sid in self.ids.items():
            self._write(sid, REG_GOAL, int(ticks[j]), 2)

    def start(self, speed: int, acc: int) -> None:
        for sid in self.ids.values():
            self._write(sid, REG_ACC, acc, 1)
            self._write(sid, REG_SPEED, speed, 2)
        for sid in self.ids.values():
            self._write(sid, REG_TORQUE, 1, 1)


class Log:
    def __init__(self, path, planner: Planner, clock, t0: float):
        self.fh = open(path, "w")
        self.p, self.clock, self.t0 = planner, clock, t0
        self.lines = 0
        self.last = None

    def line(self, pose: dict, waypoint: int, phase: str) -> None:
        self.last = pose
        rad = {j: self.p.jm.ticks_to_rad(j, pose[j]) for j in self.p.joints}
        self.fh.write(json.dumps({"t": round(self.clock.now() - self.t0, 4), "ticks": {j: int(pose[j]) for j in self.p.joints},
                                  "rad": rad, "waypoint": waypoint, "phase": phase}) + "\n")
        self.fh.flush()
        self.lines += 1

    def close(self) -> None:
        self.fh.close()


def _reach(arm: Arm, p: Planner, log: Log, clock, stop: Stop, target: dict, i: int, phase: str, timeout: float) -> None:
    t_end = clock.now() + timeout
    heard = clock.now()
    while True:
        pose = arm.read()
        now = clock.now()
        if pose is not None:
            heard = now
            log.line(pose, i, phase)
            if not (p.parked(pose) or p.guard.allowed_ticks(pose)):
                raise Fault(f"the guard refuses the arm's pose {p._fmt(pose)}: {p.guard.why_ticks(pose)}")
            if all(abs(pose[j] - target[j]) <= p.tol for j in p.joints):
                return
        elif now - heard > BUS_SILENT_S:
            raise Fault(f"no complete reading of the five servos for {BUS_SILENT_S:g} s")
        if stop.asked:
            raise Fault("stopped by the operator (Ctrl-C)")
        if now > t_end:
            off = {j: pose[j] - target[j] for j in p.joints} if pose else {}
            raise Fault(f"waypoint {i} not reached in {timeout:g} s (off by {off} ticks)")
        clock.sleep(POLL_S)


def _hold(arm: Arm, log: Log) -> dict | None:
    """Stop where the arm is: each goal set to its present reading."""
    pose = None
    for _ in range(5):
        pose = arm.read()
        if pose is not None:
            break
    pose = pose or log.last
    if pose is not None:
        try:
            arm.goals(pose)
        except Fault:
            pass        # a servo that takes no goal keeps the last one, a checked waypoint
    return pose


def _route_home(p: Planner, plan: Plan, pose: dict, around: list) -> list | None:
    """Waypoints (ticks) from pose to the drive pose through allowed moves, or
    None: back to the waypoint the arm left (what stopped it may lie ahead),
    else on to the one it was heading to, then the shortest way home."""
    if p.parked(pose):
        return [dict(p.drive)]
    if not p.guard.allowed_ticks(pose):
        return None
    for i in around:
        w = plan.waypoints[i]
        if p.move_refused(pose, w.ticks):
            continue
        if w.kind == "drive":
            return [dict(p.drive)]
        n = int(np.ravel_multi_index(w.cell, p.cshape))
        return [w.ticks] + [dict(p.drive) if c is None else p.centre_ticks(c) for c in p.home_cells(n)]
    return None


def execute(plan: Plan, p: Planner, arm: Arm, clock, log_path, timeout: float = 12.0, stop: Stop | None = None,
            say=print) -> int:
    """Run the plan on the arm. Returns the exit status (see the docstring)."""
    stop = stop or Stop()
    present = arm.read()
    if present is None:
        raise Refused("the five servos do not answer")
    off = {j: present[j] - p.drive[j] for j in p.joints}
    if any(abs(v) > 2 * p.tol for v in off.values()):
        raise Refused(f"the arm is not at the drive pose: it reads {p._fmt(present)}, "
                      f"{', '.join(f'{j} {v:+d}' for j, v in off.items() if abs(v) > 2 * p.tol)} ticks away")
    arm.goals(present)            # hold where it is before the torque comes on
    arm.start(plan.model.speed, plan.model.acc)
    log = Log(log_path, p, clock, clock.now())
    try:
        return _run(plan, p, arm, clock, log, timeout, stop, say, present)
    finally:
        log.close()


def _run(plan: Plan, p: Planner, arm: Arm, clock, log: Log, timeout: float, stop: Stop, say, present: dict) -> int:
    wps = plan.waypoints
    at, i = 0, 0
    try:
        log.line(present, 0, "tour")
        for i in range(1, len(wps)):
            why = p.move_refused(wps[at].ticks, wps[i].ticks)
            if why:
                raise Fault(f"the guard refuses the move to waypoint {i}: {why}")
            arm.goals(wps[i].ticks)
            _reach(arm, p, log, clock, stop, wps[i].ticks, i, "tour", timeout)
            at = i
        say(f"done: back at the drive pose after {clock.now() - log.t0:.1f} s, {log.lines} readings in {log.fh.name}; "
            "torque on, the arm holds the drive pose")
        return 0
    except Fault as e:
        say(f"fault on the way to waypoint {i}: {e}")
    stop.asked = False
    pose = _hold(arm, log)
    if pose is None:
        say("the servos do not answer: nothing more is commanded")
        return 3
    log.line(pose, i, "return")
    route = _route_home(p, plan, pose, [at, i] if i != at else [at])
    if route is None:
        say(f"no allowed move leaves the pose {p._fmt(pose)} ({p.guard.why_ticks(pose)}): "
            "the arm holds there, torque on")
        return 3
    say(f"going home through {len(route)} allowed move(s)")
    here = pose
    try:
        for k, target in enumerate(route, start=1):
            if not (p.parked(here) and target == p.drive):
                why = p.move_refused(here, target)
                if why:
                    raise Fault(f"the guard refuses the way home at move {k}: {why}")
            arm.goals(target)
            _reach(arm, p, log, clock, stop, target, k, "return", timeout)
            here = target
    except Fault as e:
        pose = _hold(arm, log)
        say(f"fault on the way home: {e}; the arm holds at {p._fmt(pose) if pose else 'its last pose'}, torque on")
        return 3
    say(f"back at the drive pose after {clock.now() - log.t0:.1f} s, {log.lines} readings in {log.fh.name}; torque on")
    return 2


# ── the command line ──

def resolve_port(arg: str) -> str:
    if arg and arg != "auto":
        return arg
    env = os.environ.get("MOSS_ARM_PORT")
    if env:
        return env
    found = sorted(glob.glob(PORT_GLOB))
    if len(found) != 1:
        raise Refused(f"{len(found)} links match {PORT_GLOB} ({', '.join(found) or 'none'}): give --port or MOSS_ARM_PORT")
    return found[0]


def open_bus(port: str):
    import scservo_sdk as scs
    ph = scs.PortHandler(port)
    if not ph.openPort():
        raise Refused(f"cannot open {port}")
    ph.setBaudRate(BAUD)
    return scs, ph, scs.PacketHandler(0)


def servo_ids(p: Planner) -> dict:
    ids = {}
    for j in p.joints:
        sid = p.jm.joints[j].servo_id if j in p.jm.joints else None
        ids[j] = int(sid if sid is not None else SERVO_IDS[j])
    if GRIPPER_ID in ids.values():
        raise Refused(f"servo {GRIPPER_ID} is the gripper: the calibration gives it to {[j for j, s in ids.items() if s == GRIPPER_ID]}")
    return ids


def fake_bus(p: Planner, calibration: dict, clock) -> FakeBus:
    start = {sid: p.drive[j] for j, sid in servo_ids(p).items()}
    grip = (((calibration.get("poses") or {}).get("drive") or {}).get("ticks") or {}).get("gripper", 2048)
    start[GRIPPER_ID] = int(grip)
    limits = {sid: p.ranges[j] for j, sid in servo_ids(p).items()}
    return FakeBus(start, clock, limits)


def print_plan(plan: Plan, p: Planner, full: bool, say=print) -> None:
    m = p.guard.meta.get("model", {}) or {}
    say(f"table: {p.guard.summary()}; model {m.get('path')}")
    say(f"drive: {p._fmt(p.drive)} ticks; {p.guard.why_ticks(p.drive)}")
    say(f"speed model: {plan.model.v:.0f} ticks/s ({plan.model.v * 360.0 / TICKS_PER_REV:.1f} deg/s), "
        f"{plan.model.a:.0f} ticks/s2, {SETTLE_S} s per move; tolerance {plan.tolerance} ticks")
    t = 0.0
    if full:
        say(" #     t s  kind      " + " ".join(f"{j[:12]:>12s}" for j in p.joints) + "   (model degrees)")
    for i, w in enumerate(plan.waypoints):
        t += w.move_s
        if not full:
            continue
        deg = p.deg(w.ticks)
        say(f"{i:2d} {t:7.2f}  {w.kind:8s}  " + " ".join(f"{deg[j]:+12.1f}" for j in p.joints)
            + (f"   skirts {w.skirts} ({w.steps_away:.1f} grid steps)" if w.skirts else ""))
        if w.why:
            say(f"             {w.why}")
    parts = {}
    for w in plan.waypoints:
        if w.skirts:
            parts[w.skirts] = parts.get(w.skirts, 0) + 1
    kinds = [w.kind for w in plan.waypoints]
    say(f"tour: {len(kinds)} waypoints ({kinds.count('frontier')} on the frontier, {kinds.count('pass')} passing, "
        f"drive at both ends), {plan.total_s:.1f} s by the speed model for --duration {plan.duration:g} s")
    say("skirts: " + ("; ".join(f"{n} x{c}" for n, c in sorted(parts.items(), key=lambda kv: -kv[1])) or "nothing"))


def load(table, calib) -> tuple[ArmGuard, dict]:
    if not Path(calib).is_file():
        raise Refused(f"no calibration at {calib}: the drive pose and the servo ranges come from it "
                      "(tools/moss_arm_calibrate.py, docs/arm.md)")
    try:
        calibration = json.loads(Path(calib).read_text())
    except ValueError as e:
        raise Refused(f"the calibration {calib} does not parse: {e}") from e
    try:
        guard = ArmGuard.load(table, calib)
    except (OSError, ValueError, KeyError) as e:
        raise Refused(f"the table {table} does not load: {e}") from e
    return guard, calibration


def main(argv=None, bus_hook=None, say=print) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="auto")
    ap.add_argument("--table", default=str(TABLE))
    ap.add_argument("--calib", default=str(CALIBRATION))
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--speed", type=int, default=200)
    ap.add_argument("--acc", type=int, default=8)
    ap.add_argument("--tolerance", type=int, default=25)
    ap.add_argument("--timeout", type=float, default=12.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--spread", type=float, default=0.25)
    ap.add_argument("--log", default="arm_sweep.jsonl")
    ap.add_argument("--dry-run", action="store_true", help="a fake bus instead of the arm")
    ap.add_argument("--fast", action="store_true", help="with --dry-run: a virtual clock")
    ap.add_argument("--plan-only", action="store_true", help="print the tour, write the log the speed model gives")
    a = ap.parse_args(argv)
    if not 1 <= a.speed <= MAX_SPEED:
        say(f"refused: --speed {a.speed}: 1 to {MAX_SPEED}")
        return 1
    if not 1 <= a.acc <= 254:
        say(f"refused: --acc {a.acc}: 1 to 254")
        return 1
    try:
        guard, calibration = load(a.table, a.calib)
        p = Planner(guard, calibration, SpeedModel(a.speed, a.acc, a.tolerance), a.tolerance)
        plan = p.plan(a.duration, a.seed, a.spread)
    except Refused as e:
        say(f"refused: {e}")
        return 1
    print_plan(plan, p, full=a.plan_only, say=say)
    try:
        quiet = (lambda *_: None) if a.plan_only else say
        if a.plan_only or a.dry_run:
            clk = VirtualClock() if (a.plan_only or a.fast) else RealClock()
            bus = fake_bus(p, calibration, clk)
            if bus_hook:
                bus_hook(bus, clk, plan, p)
            scs = ph = pk = bus
        else:
            clk = RealClock()
            scs, ph, pk = open_bus(resolve_port(a.port))
        arm = Arm(scs, ph, pk, servo_ids(p))
        stop = Stop()
        old = signal.signal(signal.SIGINT, stop) if not a.plan_only else None
        if isinstance(clk, VirtualClock):
            clk.stop = stop
        try:
            code = execute(plan, p, arm, clk, a.log, a.timeout, stop, say=quiet)
        finally:
            if old is not None:
                signal.signal(signal.SIGINT, old)
            ph.closePort()
    except Refused as e:
        say(f"refused: {e}")
        return 1
    if a.plan_only:
        lines = Path(a.log).read_text().splitlines()
        end = json.loads(lines[-1])["t"] if lines else 0.0
        say(f"planned log: {a.log}, {len(lines)} readings over {end:.1f} s (a run on the fake bus, t from the speed model)")
    return code


if __name__ == "__main__":
    sys.exit(main())
