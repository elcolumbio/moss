"""The arm's keep-out guard: does an arm pose touch the rover?

The answer comes from the table that tools/moss_arm_keepout.py sweeps on the
robot's MuJoCo model (config/arm_keepout.json): a verdict per grid point of
the five arm joints, free or forbidden, and for a forbidden point the rover
part it touches, by its geom name in the model (the hull, a bin wall, a
track, the lidar, or a mesh such as the deck or the nose camera). Pure
Python + numpy: no mujoco on the robot.

    from moss_dimos.arm_guard import ArmGuard
    guard = ArmGuard.load()                        # the table + config/arm_calibration.json
    guard.allowed({"shoulder_pan": 0.1, "shoulder_lift": 0.0, ...})   # model radians
    guard.allowed_ticks({"shoulder_pan": 2100, ...})   # servo readings (homing offset applied)
    guard.why(q)              # "forbidden: the arm touches bin_x1 ..." / "free: ..."
    guard.nearest_allowed(q)  # the closest pose that stays allowed once rounded to ticks
    guard.nearest_allowed_ticks(pose_ticks)          # the same, in servo ticks

The rule is conservative. The table knows grid points only. A pose between
them is allowed when every grid point its multilinear interpolation touches is
free: along each joint, the grid value below and the one above (the value
itself when the pose sits on it), all their combinations. A pose outside the
swept range of a joint is refused: the table says nothing there.
segment_allowed(a, b) follows a straight joint-space move through every grid
cell it crosses and applies the same rule in each.

Ticks <-> radians (JointMap). The servos report positions with their homing
offset applied (reported = raw - offset, config/arm_calibration.json); the
model's angle is rad = sign * (ticks - model_zero_ticks) * 2 pi / 4096, with
model_zero_ticks and sign per joint from the table's "joint_map" (default
2048 and +1: the middle of each calibrated travel on the model's zero). The
defaults are a starting point; docs/arm.md aligns them with two measured
poses, POSE_A and POSE_B (align_joint_map does the arithmetic for all five
joints, align_from_two_poses for one).
"""
from __future__ import annotations

import json
import math
from collections import Counter
from dataclasses import dataclass
from itertools import product
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
TABLE_PATH = ROOT / "config" / "arm_keepout.json"
CALIBRATION_PATH = ROOT / "config" / "arm_calibration.json"
FORMAT = "moss-arm-keepout/1"
TICKS_PER_REV = 4096
MODEL_ZERO_TICKS = 2048
# servo IDs by convention (base 1 ... gripper 6), used when the calibration file is absent
SERVO_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
ON_GRID = 1e-6          # a fractional grid index this close to an integer sits ON that grid value
WHY_OTHERS = 3          # why() names the nearest part and up to three more
TICKS_TRIES = 64        # nearest_allowed_ticks: candidates tried, nearest first
COLLISION_ERROR = "a MuJoCo collision error"   # the part name the sweep gives a pose MuJoCo could not collide
MIN_ALIGN_RAD = 0.2     # two alignment poses closer than this (11 deg) say nothing about the scale
SCALE_TOL = 0.15        # measured ticks per radian within 15 % of 4096 / 2 pi, or the poses are wrong
# the two alignment poses, in model radians (docs/arm.md shows what they look like)
POSE_A = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
POSE_B = {"shoulder_pan": 1.571, "shoulder_lift": 1.571, "elbow_flex": -1.571, "wrist_flex": 1.571, "wrist_roll": 1.571}


@dataclass(frozen=True)
class JointCal:
    servo_id: int | None
    homing_offset: int          # reported = raw - homing_offset (the servo applies it)
    model_zero_ticks: float     # the reported ticks at the model's 0 rad
    sign: int                   # +1: ticks grow with the model's angle, -1: the other way


class JointMap:
    """Servo ticks <-> model radians, per joint."""

    def __init__(self, joints: dict[str, JointCal], ticks_per_rev: int = TICKS_PER_REV):
        self.joints = dict(joints)
        self.ticks_per_rev = int(ticks_per_rev)

    @classmethod
    def from_docs(cls, calibration: dict | None, table: dict | None, joints=None) -> "JointMap":
        """From the parsed config/arm_calibration.json (ids, homing offsets;
        None = absent: IDs by convention, offsets 0) and the parsed table
        (joint_map: model_zero_ticks, sign; default 2048 and +1)."""
        calibration = calibration if isinstance(calibration, dict) else {}
        table = table if isinstance(table, dict) else {}
        jmap = table.get("joint_map") or {}
        names = list(joints or table.get("joints") or jmap or [n for n in SERVO_IDS if n != "gripper"])
        out = {}
        for name in names:
            cal = calibration.get(name) if isinstance(calibration.get(name), dict) else {}
            m = jmap.get(name) if isinstance(jmap.get(name), dict) else {}
            sign = int(m.get("sign", 1))
            if sign not in (1, -1):
                raise ValueError(f"joint_map[{name}].sign must be +1 or -1, got {sign}")
            out[name] = JointCal(servo_id=cal.get("id", SERVO_IDS.get(name)),
                                 homing_offset=int(cal.get("homing_offset", 0)),
                                 model_zero_ticks=float(m.get("model_zero_ticks", MODEL_ZERO_TICKS)),
                                 sign=sign)
        return cls(out, int(table.get("ticks_per_rev", TICKS_PER_REV)))

    @classmethod
    def load(cls, calibration_path=CALIBRATION_PATH, table_path=TABLE_PATH) -> "JointMap":
        return cls.from_docs(_read_json(calibration_path, missing_ok=True), _read_json(table_path))

    def _cal(self, joint: str) -> JointCal:
        try:
            return self.joints[joint]
        except KeyError:
            raise ValueError(f"no joint {joint!r} in the joint map ({', '.join(self.joints)})") from None

    def raw_to_ticks(self, joint: str, raw: float) -> int:
        """A raw encoder reading -> the position the servo reports (offset applied)."""
        return int(round(raw - self._cal(joint).homing_offset)) % self.ticks_per_rev

    def ticks_to_raw(self, joint: str, ticks: float) -> int:
        return int(round(ticks + self._cal(joint).homing_offset)) % self.ticks_per_rev

    def ticks_to_rad(self, joint: str, ticks: float, raw: bool = False) -> float:
        c = self._cal(joint)
        if raw:
            ticks = self.raw_to_ticks(joint, ticks)
        return c.sign * (float(ticks) - c.model_zero_ticks) * 2.0 * math.pi / self.ticks_per_rev

    def rad_to_ticks(self, joint: str, rad: float, raw: bool = False) -> int:
        c = self._cal(joint)
        ticks = int(round(c.model_zero_ticks + c.sign * float(rad) * self.ticks_per_rev / (2.0 * math.pi)))
        return self.ticks_to_raw(joint, ticks) if raw else ticks

    def pose_to_rad(self, pose_ticks: dict, raw: bool = False) -> dict[str, float]:
        """The joints of this map found in pose_ticks, as model radians (other keys ignored)."""
        return {j: self.ticks_to_rad(j, pose_ticks[j], raw) for j in self.joints if j in pose_ticks}

    def pose_to_ticks(self, q: dict, raw: bool = False) -> dict[str, int]:
        return {j: self.rad_to_ticks(j, q[j], raw) for j in self.joints if j in q}


def align_from_two_poses(ticks_a: float, rad_a: float, ticks_b: float, rad_b: float,
                         ticks_per_rev: int = TICKS_PER_REV) -> dict:
    """One joint's {"model_zero_ticks", "sign"} from two physical poses whose
    model angle is known: the servo's reading (ticks, offset applied) and the
    model's angle (rad) at each. Refuses poses less than 0.2 rad apart, and a
    measured scale more than 15 % off 4096 / 2 pi ticks per radian (a wrong
    pose, a wrong joint, or a reading across the encoder's zero)."""
    if abs(rad_b - rad_a) < MIN_ALIGN_RAD:
        raise ValueError(f"the two poses are {abs(rad_b - rad_a):.3f} rad apart: {MIN_ALIGN_RAD} rad or more")
    nominal = ticks_per_rev / (2.0 * math.pi)
    slope = (ticks_b - ticks_a) / (rad_b - rad_a)
    if abs(abs(slope) / nominal - 1.0) > SCALE_TOL:
        raise ValueError(f"{abs(slope):.0f} ticks per rad measured, {nominal:.0f} expected: check the poses")
    sign = 1 if slope > 0 else -1
    zero = ((ticks_a - sign * rad_a * nominal) + (ticks_b - sign * rad_b * nominal)) / 2.0
    return {"model_zero_ticks": round(zero, 1), "sign": sign}


def align_joint_map(ticks_a: dict, ticks_b: dict, pose_a: dict | None = None, pose_b: dict | None = None,
                    ticks_per_rev: int = TICKS_PER_REV) -> dict:
    """The table's "joint_map" from two measured poses: ticks_a and ticks_b
    are the servo readings {joint: ticks} with the arm placed in pose_a and
    pose_b ({joint: model radians}, default POSE_A and POSE_B). Each joint
    gets the angle of its own pose, signs included (the elbow is negative
    in POSE_B)."""
    pose_a = POSE_A if pose_a is None else pose_a
    pose_b = POSE_B if pose_b is None else pose_b
    missing = [j for j in pose_a if j not in ticks_a or j not in ticks_b or j not in pose_b]
    if missing:
        raise ValueError(f"no reading or no model angle for {', '.join(missing)}")
    return {j: align_from_two_poses(ticks_a[j], pose_a[j], ticks_b[j], pose_b[j], ticks_per_rev) for j in pose_a}


def _read_json(path, missing_ok: bool = False):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        if missing_ok:
            return None
        raise


def _frac_index(grid: np.ndarray, v: float) -> float:
    """v as a fractional index into a strictly increasing grid, extrapolated
    past the ends with the end steps."""
    n = len(grid)
    if v < grid[0]:
        return (v - grid[0]) / (grid[1] - grid[0])
    if v > grid[-1]:
        return (n - 1) + (v - grid[-1]) / (grid[-1] - grid[-2])
    return float(np.interp(v, grid, np.arange(n)))


def _axis(f: float, n: int) -> tuple[int, ...]:
    """The grid indices a fractional index touches: itself on a grid value,
    else the one below and the one above."""
    r = round(f)
    if abs(f - r) <= ON_GRID:
        return (int(min(max(r, 0), n - 1)),)
    lo = int(math.floor(f))
    return (lo, lo + 1)


class ArmGuard:
    """The keep-out table, queried. See the module docstring."""

    def __init__(self, table: dict, joint_map: JointMap | None = None):
        if table.get("format") != FORMAT:
            raise ValueError(f"not a keep-out table (format {table.get('format')!r}, want {FORMAT!r})")
        self.joints = tuple(table["joints"])
        self.shape = tuple(int(n) for n in table["shape"])
        self.grids = [np.asarray(table["grid"][j], dtype=float) for j in self.joints]
        if len(self.shape) != len(self.joints) or any(len(g) != n for g, n in zip(self.grids, self.shape)):
            raise ValueError("grid and shape disagree")
        if any(len(g) < 2 or np.any(np.diff(g) <= 0) for g in self.grids):
            raise ValueError("every grid needs 2 values or more, strictly increasing")
        cells = int(np.prod(self.shape))
        bits = np.unpackbits(np.frombuffer(bytes.fromhex(table["occupancy_bits_hex"]), dtype=np.uint8))
        if len(bits) < cells:
            raise ValueError(f"occupancy holds {len(bits)} cells, the grid has {cells}")
        self.occ = bits[:cells].astype(bool).reshape(self.shape)
        part = table.get("cell_part")
        if part is None:
            self.cell_part = self.occ.astype(np.uint8)
            self.part_names = ["the rover"]
        else:
            arr = np.asarray(part, dtype=np.uint8)
            if arr.size != cells or not np.array_equal(arr.reshape(self.shape) > 0, self.occ):
                raise ValueError("cell_part and the occupancy bits disagree")
            self.cell_part = arr.reshape(self.shape)
            self.part_names = list(table.get("part_names", []))
        self.meta = {k: v for k, v in table.items() if k not in ("occupancy_bits_hex", "cell_part")}
        self.joint_map = joint_map or JointMap.from_docs(None, table)
        self._targets = None

    @classmethod
    def load(cls, path=TABLE_PATH, calibration=CALIBRATION_PATH) -> "ArmGuard":
        """The table at `path`; the joint map from it plus the calibration file
        (absent: servo IDs by convention, homing offsets 0)."""
        table = _read_json(path)
        return cls(table, JointMap.from_docs(_read_json(calibration, missing_ok=True), table))

    def summary(self) -> str:
        m = self.meta
        return (f"arm keep-out: {m.get('cells')} cells {'x'.join(map(str, self.shape))}, "
                f"{100.0 * float(m.get('forbidden_fraction', 0)):.1f} % forbidden, margin {m.get('margin_m')} m, "
                f"model {m.get('model', {}).get('path')}")

    # ── poses in model radians ──

    def _fracs(self, q: dict) -> np.ndarray:
        missing = [j for j in self.joints if j not in q]
        if missing:
            raise ValueError(f"the pose lacks {', '.join(missing)}")
        out = np.empty(len(self.joints))
        for k, (j, g) in enumerate(zip(self.joints, self.grids)):
            v = float(q[j])
            if not math.isfinite(v):
                raise ValueError(f"{j} = {v}")
            out[k] = _frac_index(g, v)
        return out

    def _outside(self, f: np.ndarray) -> int | None:
        for k, n in enumerate(self.shape):
            if f[k] < -ON_GRID or f[k] > n - 1 + ON_GRID:
                return k
        return None

    def _axes(self, f: np.ndarray) -> list[tuple[int, ...]]:
        return [_axis(fk, n) for fk, n in zip(f, self.shape)]

    def allowed(self, q: dict) -> bool:
        """True when q is inside the table and every grid point its
        interpolation touches is free. q: {joint: model radians}."""
        f = self._fracs(q)
        if self._outside(f) is not None:
            return False
        return not bool(self.occ[np.ix_(*self._axes(f))].any())

    def why(self, q: dict) -> str:
        """One line: why q is refused (the rover part, or the joint out of the
        table), or that it is free."""
        f = self._fracs(q)
        k = self._outside(f)
        if k is not None:
            j, g = self.joints[k], self.grids[k]
            return (f"refused: {j} = {float(q[j]):.3f} rad is outside the table "
                    f"({g[0]:.3f} .. {g[-1]:.3f} rad)")
        axes = self._axes(f)
        parts: Counter = Counter()
        nearest, nearest_d = None, math.inf
        for c in product(*axes):
            if self.occ[c]:
                name = self._part_name(int(self.cell_part[c]))
                parts[name] += 1
                d = float(np.max(np.abs(np.asarray(c) - f)))
                if d < nearest_d:
                    nearest, nearest_d = (name, c), d
        if nearest is None:
            return "free: no grid point around this pose touches the rover"
        name, c = nearest
        others = [p for p, _ in parts.most_common() if p != name]
        also = ", ".join(others[:WHY_OTHERS]) + (f" and {len(others) - WHY_OTHERS} more" if len(others) > WHY_OTHERS else "")
        at = ", ".join(f"{j} {self.grids[i][c[i]]:+.3f}" for i, j in enumerate(self.joints))
        what = "MuJoCo's collision failed there (counted as a contact)" if name == COLLISION_ERROR \
            else f"the arm touches {name}"
        return f"forbidden: {what}" + (f" (also {also})" if others else "") + f" at the grid point {at} rad"

    def _part_name(self, k: int) -> str:
        return self.part_names[k - 1] if 1 <= k <= len(self.part_names) else "the rover"

    def _robust_targets(self) -> np.ndarray:
        """Doubled fractional indices (int16) of the poses that stay allowed
        when nudged by less than half a grid step: the centre of every cell
        whose corners are all free, and every grid point inside the table
        whose neighbours (one step on every joint, diagonals included) are
        all free. A bare free grid point is not one: rounded to servo ticks
        it moves off the grid and the rule then also asks its neighbours."""
        if self._targets is None:
            free = ~self.occ
            d = free.ndim
            cells = free
            for k in range(d):
                lo = tuple(slice(0, -1) if i == k else slice(None) for i in range(d))
                hi = tuple(slice(1, None) if i == k else slice(None) for i in range(d))
                cells = cells[lo] & cells[hi]
            core = free
            for k in range(d):
                eroded = np.zeros_like(core)
                mid = tuple(slice(1, -1) if i == k else slice(None) for i in range(d))
                below = tuple(slice(0, -2) if i == k else slice(None) for i in range(d))
                above = tuple(slice(2, None) if i == k else slice(None) for i in range(d))
                eroded[mid] = core[below] & core[mid] & core[above]
                core = eroded
            self._targets = np.concatenate([np.argwhere(cells) * 2 + 1, np.argwhere(core) * 2]).astype(np.int16)
        return self._targets

    def _ranked(self, q: dict, k: int) -> list[np.ndarray]:
        """The k robust targets nearest q (L-inf in grid steps, then L2), as
        fractional indices."""
        targets = self._robust_targets()
        if not len(targets):
            raise ValueError("the table has no pose that stays allowed")
        diff = np.abs(targets / 2.0 - self._fracs(q))
        key = diff.max(axis=1) * 1e3 + np.sqrt((diff ** 2).sum(axis=1)) * 1e-3
        k = min(k, len(targets))
        pick = np.argpartition(key, k - 1)[:k] if k < len(targets) else np.arange(len(targets))
        pick = pick[np.argsort(key[pick], kind="stable")]
        return [targets[i] / 2.0 for i in pick]

    def _at(self, f: np.ndarray) -> dict[str, float]:
        return {j: float(np.interp(f[k], np.arange(len(g)), g)) for k, (j, g) in enumerate(zip(self.joints, self.grids))}

    def nearest_allowed(self, q: dict) -> dict[str, float]:
        """The allowed pose nearest q, in grid steps (L-inf; ties go to the
        smaller L2), as {joint: model radians}, among the poses that stay
        allowed when rounded to servo ticks (see _robust_targets). The way
        there is not checked: segment_allowed does that."""
        return self._at(self._ranked(q, 1)[0])

    def segment_allowed(self, q_from: dict, q_to: dict) -> bool:
        """A straight joint-space move from q_from to q_to: allowed when both
        ends are inside the table and every grid cell the move passes
        through (corners included: the move is cut where it crosses a grid
        value on any joint) has all its touched grid points free."""
        fa, fb = self._fracs(q_from), self._fracs(q_to)
        if self._outside(fa) is not None or self._outside(fb) is not None:
            return False
        cuts = {0.0, 1.0}
        for a, b in zip(fa, fb):
            if abs(b - a) > ON_GRID:
                lo, hi = sorted((a, b))
                for k in range(int(math.ceil(lo)), int(math.floor(hi)) + 1):
                    cuts.add(min(1.0, max(0.0, (k - a) / (b - a))))
        ts = sorted(cuts)
        for t0, t1 in zip(ts[:-1], ts[1:]):
            for t in (t0, (t0 + t1) / 2.0):
                f = fa + t * (fb - fa)
                if self.occ[np.ix_(*self._axes(f))].any():
                    return False
        return not self.occ[np.ix_(*self._axes(fb))].any()

    # ── poses in servo ticks ──

    def ticks_pose_to_rad(self, pose_ticks: dict, raw: bool = False) -> dict[str, float]:
        missing = [j for j in self.joints if j not in pose_ticks]
        if missing:
            raise ValueError(f"the pose lacks {', '.join(missing)}")
        return {j: self.joint_map.ticks_to_rad(j, pose_ticks[j], raw) for j in self.joints}

    def allowed_ticks(self, pose_ticks: dict, raw: bool = False) -> bool:
        """allowed() for servo readings {joint: ticks} (the gripper and other
        keys are ignored); raw=True for encoder readings without the offset."""
        return self.allowed(self.ticks_pose_to_rad(pose_ticks, raw))

    def why_ticks(self, pose_ticks: dict, raw: bool = False) -> str:
        return self.why(self.ticks_pose_to_rad(pose_ticks, raw))

    def nearest_allowed_ticks(self, pose_ticks: dict, raw: bool = False) -> dict[str, int]:
        """nearest_allowed() for servo readings, returned as ticks: the
        nearest robust pose whose tick rounding allowed_ticks() accepts."""
        q = self.ticks_pose_to_rad(pose_ticks, raw)
        for f in self._ranked(q, TICKS_TRIES):
            ticks = self.joint_map.pose_to_ticks(self._at(f), raw)
            if self.allowed_ticks(ticks, raw):
                return ticks
        raise ValueError(f"none of the {TICKS_TRIES} nearest robust poses survives the rounding to ticks")
