#!/usr/bin/env python3
"""The arm's keep-out table: every arm pose that touches the rover, swept on a
MuJoCo model of the robot.

The arm is every geom from --arm-root down (links, servo shells, palm, pads,
visual meshes included). It is placed at every point of a grid over its five
joints. At each point the model is posed (mj_kinematics, the position stage of
mj_forward) and mj_collision lists the contacts closer than --margin between an
arm geom and a rover part. A grid point with one such contact is forbidden.
Nothing is stepped: no dynamics, the arm is placed, never moved.

The root link (the body --arm-root, which only its own joints move: the pan)
is done apart. Its meshes are replaced by stand-ins that follow their surface
(below), because the convex hull of a bracket fills its inside and would sweep
through whatever sits next to the arm's base. It is placed every
--root-step-deg along its own joints, and a contact there forbids every grid
value within half a grid step (a pose between two grid values touches both:
the guard's rule), whatever the other joints do. It turns in place at a
clearance the build sets, so it counts contacts closer than --root-margin
(0: where it meets the rover), not --margin. The other links collide as they
are: a mesh as its convex hull, larger than the part, never smaller.

The rover is every geom of --rover-body and of the bodies under it, except the
arm, the bodies that carry it (its mount: base_link on the SO-101) and the
geoms --exclude names:
  * the geoms --rover-parts names (hull, bin_*, track_*, tread_*, lidar) and
    every other primitive (box, sphere, capsule, cylinder, ellipsoid) count as
    they are;
  * every other mesh counts through stand-ins: its convex hull would fill a
    concave part (the inside of the bin, the deck around the arm's base).
    Its surface is sampled every half voxel, the samples more than
    --cover-tol outside the parts above are kept, and boxes on a
    --stand-in-voxel grid cover them (whole voxels, merged). Every point of
    the surface lies inside a stand-in or within --cover-tol plus half a
    voxel of a part that counts as it is; a stand-in reaches past the surface
    by less than one and a half voxels along each axis. The root link's
    stand-ins use --root-voxel and cover all of its surface.
The JSON lists every rover mesh with the share of its surface the parts that
count as they are miss, where that share lies, and its number of boxes.

A rover part that the root link touches at every value of its joints is where
the arm sits: the root link's contacts with it are not counted (every other
link's are). Such parts are printed and listed in the JSON (mount_parts).

The lidar is not in the robot's MJCF. When the model has no geom named
"lidar", a cylinder (radius 0.038 m, height 0.04 m) is added on the rover body
at the profile's pose: x = y = 0 (base_link is the lidar's floor projection,
on the rotation centre) and z = lidar_height_m (moss_dimos/robot_profile.py,
MOSS_LIDAR_HEIGHT_M). --lidar-xyz moves it, --no-lidar leaves it out. The rover
parts it overlaps are printed and stored (lidar.overlaps_m).

The fingers (every arm joint that is not swept) take every position: each geom
they carry is replaced by its swept volume, the convex hull of the geom at both
ends of a slide (every 10 degrees of a hinge), on the body above the finger.
A closed, open or half-open gripper is inside it. --fix name=value pins a
joint to one value instead.

The margin is MuJoCo's geom margin on the rover parts only (MuJoCo adds the
margins of the two geoms of a pair). MuJoCo's own convex collider is switched
off for libccd's: the former (3.9) reports no contact for a geom held whole
inside a mesh's hull. A pose where MuJoCo's collision code fails
(mujoco.FatalError) is marked forbidden and counted (collision_errors).

Each joint is swept over its model range widened by --pad-deg on both sides:
the real arm's end stops (its calibrated travel) sit a few degrees past the
SO-101 model's limits, and the folded driving pose lies on them.

Checks before writing: --check grid points are done again with a full
mj_forward and must agree, and --check free grid points are done again with
the real fingers closed, half open, open and at a random opening: none may
touch the rover.

    python tools/moss_arm_keepout.py --model <robot.xml>          # -> config/arm_keepout.json
    python tools/moss_arm_keepout.py --model <robot.xml> --steps 8,8,8,6,4 --out /tmp/k.json

Options: --steps pan,lift,elbow,flex,roll (16,16,16,12,8), --margin metres
(0.01), --pad-deg (10), --joints (the five SO-101 joint names), --arm-root
(shoulder_link), --rover-body (rover), --rover-parts (hull,bin_*,track_*,
tread_*,lidar), --exclude (none), --stand-in-voxel (0.004 m, 0 = rover meshes
left out), --root-voxel (0.001 m), --root-margin (0 m), --root-step-deg (1),
--cover-tol (0.002 m), --lidar-xyz x,y,z, --no-lidar, --fix, --check N (64),
--jobs (one per core, up to 16), --quiet.

The JSON keeps the joint_map (ticks <-> model radians) of the file it
replaces: the alignment measured on the arm survives a new sweep.
Needs mujoco and numpy here (scipy, when present, lets a mesh named in
--rover-parts cover the stand-ins with its hull); moss_dimos/arm_guard.py
reads the table with numpy only. docs/arm.md has the whole story.
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import itertools
import json
import math
import os
import re
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

FORMAT = "moss-arm-keepout/1"
DEFAULT_OUT = ROOT / "config" / "arm_keepout.json"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
STEPS = (16, 16, 16, 12, 8)
ROVER_PARTS = ("hull", "bin_*", "track_*", "tread_*", "lidar")
MARGIN_M = 0.01
PAD_DEG = 10.0
VOXEL_M = 0.004
ROOT_VOXEL_M = 0.001
ROOT_MARGIN_M = 0.0
ROOT_STEP_DEG = 1.0
COVER_TOL_M = 0.002
LIDAR_NAME = "lidar"
LIDAR_RADIUS_M = 0.038
LIDAR_HEIGHT_M = 0.04
TICKS_PER_REV = 4096
MODEL_ZERO_TICKS = 2048
CHECK_N = 64
MAX_JOBS = 16
HINGE_SWEEP_DEG = 10.0
MAX_FINGER_POSES = 4096
COLLISION_ERROR = "a MuJoCo collision error"
STAND_IN_SEP = "~"
SWEPT_SUFFIX = "~swept"
# contype of an arm geom = conaffinity of a rover part: they meet each other and
# nothing else (arm-arm, rover-rover, the ground and loose objects never do)
ARM_BIT = 2


def profile_lidar_xyz() -> tuple[float, float, float]:
    """The lidar in the rover body frame, from the profile: base_link is the
    lidar's floor projection and sits on the rotation centre (x = y = 0)."""
    from moss_dimos import robot_profile
    return (0.0, 0.0, float(robot_profile.current().lidar_height_m))


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


GEOMETRY_ARRAYS = ("names", "geom_type", "geom_size", "geom_pos", "geom_quat", "geom_bodyid", "geom_dataid",
                   "body_parentid", "body_pos", "body_quat", "jnt_type", "jnt_bodyid", "jnt_pos", "jnt_axis",
                   "jnt_range", "jnt_limited", "mesh_vertadr", "mesh_vertnum", "mesh_vert", "mesh_faceadr",
                   "mesh_facenum", "mesh_face")


def geometry_sha256(model) -> str:
    """The compiled model's geometry: names, bodies, joints, geoms and every
    mesh's vertices and faces. It changes with an included file or a new mesh
    export, which the top file's own sha256 does not see."""
    h = hashlib.sha256()
    for name in GEOMETRY_ARRAYS:
        arr = np.ascontiguousarray(getattr(model, name))
        h.update(name.encode() + str(arr.dtype).encode() + str(arr.shape).encode())
        h.update(arr.tobytes())
    return h.hexdigest()


def model_label(path: Path) -> str:
    """The model's path as written in the JSON: relative to the working
    directory when it lies under it, else its file name (the hashes say which)."""
    path = Path(path)
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return path.name


def _matches(name: str, patterns) -> bool:
    return bool(name) and any(fnmatch.fnmatchcase(name, p) for p in patterns)


def _subtree(model, root: int) -> set[int]:
    out = {root}
    for b in range(root + 1, model.nbody):
        if int(model.body_parentid[b]) in out:
            out.add(b)
    return out


def _name(model, obj, i: int) -> str:
    import mujoco
    return mujoco.mj_id2name(model, obj, i) or ""


def geom_name(model, gid: int) -> str:
    import mujoco
    name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    if name:
        return name
    body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or f"body{model.geom_bodyid[gid]}"
    return f"{body}/geom{gid}"


def joint_name(model, jid: int) -> str:
    import mujoco
    return _name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or f"joint{jid}"


# ── geometry in numpy ──

def _mesh_arrays(model, gid: int):
    mid = int(model.geom_dataid[gid])
    va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
    return np.asarray(model.mesh_vert[va:va + vn], float), np.asarray(model.mesh_face[fa:fa + fn], int)


def hull_points_local(model, gid: int) -> np.ndarray:
    """Points whose convex hull holds the geom, in its own frame: a mesh's
    vertices and a box's corners (exact), the bounding box of the other
    primitives (larger)."""
    import mujoco

    t = int(model.geom_type[gid])
    s = np.asarray(model.geom_size[gid], float)
    if t == int(mujoco.mjtGeom.mjGEOM_MESH):
        return _mesh_arrays(model, gid)[0]
    if t == int(mujoco.mjtGeom.mjGEOM_BOX):
        half = s
    elif t == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        half = np.array([s[0]] * 3)
    elif t == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        half = np.array([s[0], s[0], s[0] + s[1]])
    elif t == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        half = np.array([s[0], s[0], s[1]])
    elif t == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        half = s
    else:
        raise SystemExit(f"geom {geom_name(model, gid)}: type {t} cannot be swept (box, sphere, capsule, "
                         "cylinder, ellipsoid or mesh)")
    return np.array(list(itertools.product(*[(-h, h) for h in half])), float)


def _triangle_points(v: np.ndarray, f: np.ndarray, spacing: float) -> np.ndarray:
    """Points on every triangle, along its two edges from its first corner in
    steps no longer than `spacing`: every point of the triangle lies within
    `spacing` of one of them."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    k1 = np.maximum(1, np.ceil(np.linalg.norm(b - a, axis=1) / spacing)).astype(int)
    k2 = np.maximum(1, np.ceil(np.linalg.norm(c - a, axis=1) / spacing)).astype(int)
    out = [v]
    for n1, n2 in set(zip(k1.tolist(), k2.tolist())):
        if n1 == 1 and n2 == 1:
            continue
        sel = (k1 == n1) & (k2 == n2)
        u, w = np.meshgrid(np.arange(n1 + 1) / n1, np.arange(n2 + 1) / n2, indexing="ij")
        keep = (u + w) <= 1.0 + 1e-12
        u, w = u[keep], w[keep]
        p = a[sel][:, None, :] + u[None, :, None] * (b[sel] - a[sel])[:, None, :] \
            + w[None, :, None] * (c[sel] - a[sel])[:, None, :]
        out.append(p.reshape(-1, 3))
    return np.concatenate(out)


def mesh_surface_points(model, gid: int, spacing: float) -> np.ndarray:
    """Points on a mesh geom's surface, every point of it within `spacing` of
    one of them, in the geom's frame."""
    return _triangle_points(*_mesh_arrays(model, gid), spacing)


def _to_world(data, gid: int, local: np.ndarray) -> np.ndarray:
    return local @ np.asarray(data.geom_xmat[gid]).reshape(3, 3).T + data.geom_xpos[gid]


def _to_body(data, bid: int, world: np.ndarray) -> np.ndarray:
    return (world - data.xpos[bid]) @ np.asarray(data.xmat[bid]).reshape(3, 3)


class _Solid:
    """Point-in-geom (inflated by tol) for the rover parts that count as they are."""

    def __init__(self, model, data, gid: int):
        import mujoco

        self.type = int(model.geom_type[gid])
        self.size = np.asarray(model.geom_size[gid], float)
        self.pos = np.array(data.geom_xpos[gid])
        self.rot = np.array(data.geom_xmat[gid]).reshape(3, 3)
        self.lo = self.hi = self.planes = None
        try:
            corners = _to_world(data, gid, hull_points_local(model, gid))
        except SystemExit:          # a plane or a height field covers nothing
            return
        self.lo, self.hi = corners.min(0), corners.max(0)
        if self.type == int(mujoco.mjtGeom.mjGEOM_MESH):
            try:
                from scipy.spatial import ConvexHull
                self.planes = ConvexHull(corners).equations
            except Exception:          # no scipy, or a flat mesh: this part covers nothing
                self.planes = None

    def inside(self, p: np.ndarray, tol: float) -> np.ndarray:
        import mujoco

        out = np.zeros(len(p), bool)
        if self.lo is None:
            return out
        near = np.all((p >= self.lo - tol) & (p <= self.hi + tol), axis=1)
        if not near.any():
            return out
        q = p[near]
        loc = (q - self.pos) @ self.rot
        s, t = self.size, self.type
        if t == int(mujoco.mjtGeom.mjGEOM_BOX):
            ok = np.all(np.abs(loc) <= s + tol, axis=1)
        elif t == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            ok = np.linalg.norm(loc, axis=1) <= s[0] + tol
        elif t == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
            axial = np.zeros_like(loc)
            axial[:, 2] = np.clip(loc[:, 2], -s[1], s[1])
            ok = np.linalg.norm(loc - axial, axis=1) <= s[0] + tol
        elif t == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            ok = (np.hypot(loc[:, 0], loc[:, 1]) <= s[0] + tol) & (np.abs(loc[:, 2]) <= s[1] + tol)
        elif t == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            ok = np.sum((loc / (s + tol)) ** 2, axis=1) <= 1.0
        elif t == int(mujoco.mjtGeom.mjGEOM_MESH) and self.planes is not None:
            ok = np.all(q @ self.planes[:, :3].T + self.planes[:, 3] <= tol, axis=1)
        else:
            ok = np.zeros(len(q), bool)
        out[near] = ok
        return out


def merge_voxels(grid: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Boxes (first voxel index, last voxel index) covering every True voxel
    and nothing else: runs along x, grown along y, then along z."""
    g = grid.copy()
    boxes = []
    for x, y, z in np.argwhere(g):
        if not g[x, y, z]:
            continue
        x2 = x
        while x2 + 1 < g.shape[0] and g[x2 + 1, y, z]:
            x2 += 1
        y2 = y
        while y2 + 1 < g.shape[1] and g[x:x2 + 1, y2 + 1, z].all():
            y2 += 1
        z2 = z
        while z2 + 1 < g.shape[2] and g[x:x2 + 1, y:y2 + 1, z2 + 1].all():
            z2 += 1
        g[x:x2 + 1, y:y2 + 1, z:z2 + 1] = False
        boxes.append((np.array([x, y, z]), np.array([x2, y2, z2])))
    return boxes


def voxel_boxes(points: np.ndarray, voxel: float) -> list[tuple[np.ndarray, np.ndarray]]:
    """Boxes [(centre, half size)] made of whole voxels that hold every point
    within half a voxel (per axis) of `points`: sampled every half voxel, a
    surface is covered whole."""
    if not len(points):
        return []
    h = voxel / 2.0
    corners = np.array(list(itertools.product((-h, h), repeat=3)))
    idx = np.unique(np.floor((points[:, None, :] + corners[None, :, :]).reshape(-1, 3) / voxel).astype(np.int64),
                    axis=0)
    base = idx.min(0)
    grid = np.zeros(idx.max(0) - base + 1, bool)
    grid[tuple((idx - base).T)] = True
    out = []
    for a, b in merge_voxels(grid):
        lo, hi = (a + base) * voxel, (b + base + 1) * voxel
        out.append(((lo + hi) / 2.0, (hi - lo) / 2.0))
    return out


def rover_mesh_stand_ins(model, data, gid: int, solids: list[_Solid], rover_bid: int, voxel: float, tol: float):
    """The boxes standing in for one rover mesh, in the rover body's frame,
    and its report: the share of its sampled surface the solids miss (by
    more than tol), the extent of that share, the number of boxes."""
    local = mesh_surface_points(model, gid, voxel / 2.0)
    pts = _to_world(data, gid, local)
    lo, hi = pts.min(0) - tol, pts.max(0) + tol
    covered = np.zeros(len(pts), bool)
    for s in solids:
        if s.lo is not None and np.all(s.hi >= lo) and np.all(s.lo <= hi):
            covered |= s.inside(pts, tol)
    report = {"outside_fraction": round(float(1.0 - covered.mean()), 4), "boxes": 0}
    out = _to_body(data, rover_bid, pts[~covered])
    if not len(out):
        return [], report
    report["outside_extent_m"] = [[round(float(v), 4) for v in out.min(0)], [round(float(v), 4) for v in out.max(0)]]
    boxes = voxel_boxes(out, voxel)
    report["boxes"] = len(boxes)
    return boxes, report


# ── the model, ready to sweep ──

class Prepared:
    """The compiled models with their contact masks, and what the sweep needs:
    model (every link but the root), root_model (the root link's stand-ins
    and primitives only), the arm and rover geom ids (the same in both)."""

    def __init__(self, **kw):
        self.__dict__.update(kw)


def _unique_name(taken: set, want: str) -> str:
    name, k = want, 1
    while name in taken:
        name, k = f"{want}_{k}", k + 1
    taken.add(name)
    return name


def prepare(model_path: Path, *, arm_root: str = "shoulder_link", rover_body: str = "rover",
            rover_parts=ROVER_PARTS, exclude=(), margin: float = MARGIN_M, lidar_xyz=None,
            joints=JOINTS, fixed=None, fingers: str = "swept", voxel: float = VOXEL_M,
            root_voxel: float = ROOT_VOXEL_M, root_margin: float = ROOT_MARGIN_M, cover_tol: float = COVER_TOL_M,
            stand_ins=None) -> Prepared:
    """Load the MJCF, add the lidar when it has none, the stand-ins of the
    rover's meshes and of the root link's, the fingers' swept volumes, and
    set the contact masks so that mj_collision reports arm <-> rover-part
    contacts only, within `margin`. fingers="real" keeps the finger geoms as
    they are (the finger check). stand_ins: from an earlier prepare of the
    same model (skips the sampling)."""
    import mujoco

    fixed = dict(fixed or {})
    spec = mujoco.MjSpec.from_file(str(model_path))
    geometry = geometry_sha256(spec.compile())
    if spec.body(arm_root) is None:
        raise SystemExit(f"no body named {arm_root!r} in {model_path} (--arm-root)")
    rover = spec.body(rover_body)
    if rover is None:
        raise SystemExit(f"no body named {rover_body!r} in {model_path} (--rover-body)")
    taken = {g.name for g in spec.geoms if g.name} | {m.name for m in spec.meshes if m.name}
    for k, g in enumerate(spec.geoms):
        if not g.name:
            g.name = _unique_name(taken, f"{g.parent.name or 'body'}_geom{k}")
    lidar = {"added": False}
    if lidar_xyz is not None and not any(g.name == LIDAR_NAME for g in spec.geoms):
        rover.add_geom(name=LIDAR_NAME, type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                       size=[LIDAR_RADIUS_M, LIDAR_HEIGHT_M / 2.0, 0.0], pos=[float(v) for v in lidar_xyz])
        taken.add(LIDAR_NAME)
        lidar = {"added": True, "pos_m": [round(float(v), 6) for v in lidar_xyz],
                 "radius_m": LIDAR_RADIUS_M, "height_m": LIDAR_HEIGHT_M}
    # the masks below decide every contact: the model's own exclusions and
    # explicit pairs would hide or add some, the parent filter would hide an
    # arm root hung straight on the rover body
    for elem in list(spec.excludes) + list(spec.pairs):
        spec.delete(elem)
    spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    spec.option.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    # MuJoCo's own convex collider (3.9) reports no contact for a geom held
    # whole inside a mesh's hull (a stand-in box inside an arm link, a small
    # part inside the gripper's swept volume); the libccd one does
    if hasattr(mujoco.mjtDisableBit, "mjDSBL_NATIVECCD"):
        spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    spec.option.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_OVERRIDE)

    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    for name, value in fixed.items():
        jid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise SystemExit(f"--fix: no joint named {name!r}")
        d0.qpos[m0.jnt_qposadr[jid]] = float(value)
    mujoco.mj_kinematics(m0, d0)
    root_bid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_BODY, arm_root)
    rover_bid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_BODY, rover_body)
    arm_bodies = _subtree(m0, root_bid)
    rover_bodies = _subtree(m0, rover_bid)
    mount_bodies, b = [], int(m0.body_parentid[root_bid])
    while root_bid in rover_bodies and b != rover_bid:
        mount_bodies.append(b)
        b = int(m0.body_parentid[b])
    gname = {g: geom_name(m0, g) for g in range(m0.ngeom)}
    mesh_t = int(mujoco.mjtGeom.mjGEOM_MESH)
    prim_t = {int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_SPHERE), int(mujoco.mjtGeom.mjGEOM_CAPSULE),
              int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)}

    # the rover: as they are, or through stand-ins
    as_is, meshes, unused = [], [], []
    for g in range(m0.ngeom):
        bid = int(m0.geom_bodyid[g])
        if bid not in rover_bodies or bid in arm_bodies or bid in mount_bodies:
            continue
        if _matches(gname[g], exclude):
            unused.append(gname[g])
        elif _matches(gname[g], rover_parts) or int(m0.geom_type[g]) in prim_t:
            as_is.append(g)
        elif int(m0.geom_type[g]) == mesh_t:
            meshes.append(g)
        else:
            unused.append(gname[g])
    root_meshes = [g for g in range(m0.ngeom) if int(m0.geom_bodyid[g]) == root_bid and int(m0.geom_type[g]) == mesh_t]
    if stand_ins is None:
        stand_ins = {"voxel_m": voxel, "root_voxel_m": root_voxel, "cover_tol_m": cover_tol,
                     "per_mesh": {}, "boxes": [], "root": {}, "root_boxes": []}
        solids = [_Solid(m0, d0, g) for g in as_is]
        for g in meshes:
            if voxel > 0:
                boxes, rep = rover_mesh_stand_ins(m0, d0, g, solids, rover_bid, voxel, cover_tol)
                stand_ins["per_mesh"][gname[g]] = rep
                stand_ins["boxes"].extend((gname[g], c, h) for c, h in boxes)
            else:
                stand_ins["per_mesh"][gname[g]] = {"left_out": "--stand-in-voxel 0"}
        for g in root_meshes:
            local = mesh_surface_points(m0, g, root_voxel / 2.0)
            boxes = voxel_boxes(_to_body(d0, root_bid, _to_world(d0, g, local)), root_voxel)
            stand_ins["root"][gname[g]] = len(boxes)
            stand_ins["root_boxes"].extend((gname[g], c, h) for c, h in boxes)
    if voxel <= 0:
        unused += [gname[g] for g in meshes]
    label = {gname[g]: gname[g] for g in as_is}
    for k, (src, c, h) in enumerate(stand_ins["boxes"]):
        name = _unique_name(taken, f"{src}{STAND_IN_SEP}{k}")
        rover.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(v) for v in h],
                       pos=[float(v) for v in c], group=3, rgba=[0, 0, 0, 0])
        label[name] = src
    part_names = list(label)
    if not part_names:
        raise SystemExit(f"no rover part on {rover_body!r}")
    root_spec = spec.body(arm_root)
    root_names = [gname[g] for g in range(m0.ngeom) if int(m0.geom_bodyid[g]) == root_bid
                  and int(m0.geom_type[g]) != mesh_t]
    arm_label = {n: n for n in root_names}
    for k, (src, c, h) in enumerate(stand_ins["root_boxes"]):
        name = _unique_name(taken, f"{src}{STAND_IN_SEP}{k}")
        root_spec.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(v) for v in h],
                           pos=[float(v) for v in c], group=3, rgba=[0, 0, 0, 0])
        root_names.append(name)
        arm_label[name] = src

    # the other links: as they are, the fingers' geoms as swept volumes
    swept_j = set()
    for j in joints:
        jid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jid < 0:
            raise SystemExit(f"no joint named {j!r} in the model")
        swept_j.add(jid)
    movable = (int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_HINGE))
    loose = []
    for j in range(m0.njnt):
        if int(m0.jnt_bodyid[j]) not in arm_bodies or j in swept_j or joint_name(m0, j) in fixed:
            continue
        if int(m0.jnt_type[j]) not in movable:
            raise SystemExit(f"arm joint {joint_name(m0, j)!r} is a ball or free joint: pin it with --fix")
        if int(m0.jnt_bodyid[j]) == root_bid:
            raise SystemExit(f"joint {joint_name(m0, j)!r} on the root link is not swept: sweep it or pin it with --fix")
        loose.append(j)
    body_joints: dict[int, list[int]] = {}
    for j in range(m0.njnt):
        body_joints.setdefault(int(m0.jnt_bodyid[j]), []).append(j)
    link_names, replaced = [], []
    for g in range(m0.ngeom):
        bid = int(m0.geom_bodyid[g])
        if bid not in arm_bodies or bid == root_bid:
            continue
        chain, b = [], bid           # bodies from the geom's up to the arm root
        while True:
            chain.append(b)
            if b == root_bid:
                break
            b = int(m0.body_parentid[b])
        on_chain = [(b, j) for b in chain for j in body_joints.get(b, []) if j in loose]
        if not on_chain or fingers == "real":
            link_names.append(gname[g])
            arm_label[gname[g]] = gname[g]
            continue
        top = max(chain.index(b) for b, _ in on_chain)      # the body of the loose joint nearest the root
        if any(j in swept_j for b in chain[:top + 1] for j in body_joints.get(b, [])):
            raise SystemExit(f"geom {gname[g]}: a swept joint lies below an unswept one; sweep it or pin it with --fix")
        frame = int(m0.body_parentid[chain[top]])
        values = []
        for _, j in on_chain:
            lo, hi = (float(v) for v in m0.jnt_range[j]) if m0.jnt_limited[j] else (None, None)
            if int(m0.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_SLIDE):
                if lo is None:
                    raise SystemExit(f"finger {joint_name(m0, j)!r} slides without limits: pin it with --fix")
                values.append((lo, hi))
            else:
                if lo is None:
                    lo, hi = -math.pi, math.pi
                n = int(max(2, math.ceil(math.degrees(hi - lo) / HINGE_SWEEP_DEG) + 1))
                values.append(tuple(np.linspace(lo, hi, n)))
        if math.prod(len(v) for v in values) > MAX_FINGER_POSES:
            raise SystemExit(f"geom {gname[g]}: too many finger poses to sweep; pin some with --fix")
        local = hull_points_local(m0, g)
        d = mujoco.MjData(m0)
        pts = []
        for combo in itertools.product(*values):
            d.qpos[:] = d0.qpos
            for (_, j), v in zip(on_chain, combo):
                d.qpos[m0.jnt_qposadr[j]] = v
            mujoco.mj_kinematics(m0, d)
            pts.append(_to_body(d, frame, _to_world(d, g, local)))
        mesh = _unique_name(taken, re.sub(r"[^A-Za-z0-9_.-]", "_", gname[g]) + SWEPT_SUFFIX)
        spec.add_mesh(name=mesh, uservert=np.concatenate(pts).ravel().tolist())
        spec.body(_name(m0, mujoco.mjtObj.mjOBJ_BODY, frame)).add_geom(
            name=mesh, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh, group=3, rgba=[0, 0, 0, 0])
        link_names.append(mesh)
        arm_label[mesh] = mesh
        replaced.append(gname[g])
    if not link_names and not root_names:
        raise SystemExit(f"no geom under {arm_root!r}: nothing to sweep")

    def compile_with(arm_on, part_margin):
        for g in spec.geoms:
            if g.name in arm_on:
                g.contype, g.conaffinity, g.margin, g.gap, g.condim = ARM_BIT, 0, 0.0, 0.0, 1
            elif g.name in label:
                g.contype, g.conaffinity, g.margin, g.gap, g.condim = 0, ARM_BIT, float(part_margin), 0.0, 1
            else:
                g.contype, g.conaffinity = 0, 0
        return spec.compile()

    # the root model's parts carry the root margin itself: MuJoCo's box-box
    # contacts inside the margin band report no reliable distance, so the
    # root pass counts every contact MuJoCo makes rather than filtering on it
    model = compile_with(set(link_names), margin)
    root_model = compile_with(set(root_names), root_margin)
    gid = {geom_name(model, g): g for g in range(model.ngeom)}
    if any(geom_name(root_model, g) != n for n, g in gid.items()):
        raise SystemExit("the two compiled models number their geoms differently")
    base_qpos = model.qpos0.copy()
    for n, v in fixed.items():
        base_qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
    part_ids = [gid[n] for n in part_names]
    if lidar.get("added"):
        lidar["overlaps_m"] = lidar_overlaps(model, base_qpos, gid[LIDAR_NAME], part_ids, label)
    return Prepared(model=model, root_model=root_model, root_bid=root_bid,
                    arm_ids=[gid[n] for n in link_names], root_ids=[gid[n] for n in root_names],
                    part_ids=part_ids, part_label=[label[n] for n in part_names], arm_label=arm_label,
                    lidar=lidar, stand_ins=stand_ins, base_qpos=base_qpos,
                    mount_bodies=[_name(m0, mujoco.mjtObj.mjOBJ_BODY, b) for b in mount_bodies],
                    unused=unused, loose={joint_name(m0, j): [float(v) for v in m0.jnt_range[j]] for j in loose},
                    fixed=fixed, replaced=replaced, geometry_sha256=geometry, fingers=fingers)


def lidar_overlaps(model, base_qpos, lidar_id: int, part_ids, label: dict) -> dict:
    """{rover part: signed distance (m)} for the parts the lidar cylinder goes
    into (distance < 0)."""
    import mujoco

    d = mujoco.MjData(model)
    d.qpos[:] = base_qpos
    mujoco.mj_kinematics(model, d)
    flags = int(model.opt.disableflags)
    native = int(getattr(mujoco.mjtDisableBit, "mjDSBL_NATIVECCD", 0))
    out: dict[str, float] = {}
    try:
        for g in part_ids:
            if g == lidar_id:
                continue
            # libccd sees a part held whole inside the cylinder, MuJoCo's own
            # collider gives the true depth: an overlap for either, the depth
            # of the native one when it sees it
            model.opt.disableflags = flags & ~native
            deep = float(mujoco.mj_geomDistance(model, d, lidar_id, g, 0.01, None))
            model.opt.disableflags = flags | native
            held = float(mujoco.mj_geomDistance(model, d, lidar_id, g, 0.01, None))
            if min(deep, held) < 0:
                dist = deep if deep < 0 else held
                name = label[geom_name(model, g)]
                out[name] = round(min(dist, out.get(name, dist)), 4)
    finally:
        model.opt.disableflags = flags
    return dict(sorted(out.items(), key=lambda kv: kv[1]))


def joint_grid(model, jid: int, steps: int, pad_deg: float) -> tuple[np.ndarray, list[float]]:
    import mujoco

    if model.jnt_limited[jid]:
        lo, hi = (float(v) for v in model.jnt_range[jid])
    else:
        lo, hi = -math.pi, math.pi
    pad = math.radians(pad_deg) if int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_HINGE) else 0.0
    if steps < 2:
        raise SystemExit("every joint needs 2 steps or more")
    return np.linspace(lo - pad, hi + pad, steps), [lo, hi]


def _contacts(model, data, is_arm: np.ndarray, is_part: np.ndarray, margin: float, ignore=frozenset()):
    """[(arm geom, rover geom, dist)] of the current contacts closer than
    margin, without the (arm geom, rover geom) pairs in `ignore`."""
    n = data.ncon
    if n == 0:
        return []
    out = []
    for (g1, g2), dd in zip(data.contact.geom[:n], data.contact.dist[:n]):
        if dd >= margin:
            continue
        if is_arm[g1] and is_part[g2]:
            pair = (int(g1), int(g2))
        elif is_arm[g2] and is_part[g1]:
            pair = (int(g2), int(g1))
        else:
            continue
        if pair not in ignore:
            out.append((pair[0], pair[1], float(dd)))
    return out


def _masks(model, arm_ids, part_ids):
    is_arm = np.zeros(model.ngeom, bool)
    is_arm[arm_ids] = True
    is_part = np.zeros(model.ngeom, bool)
    is_part[part_ids] = True
    return is_arm, is_part


def _touch(model, data, is_arm, is_part, margin, ignore=frozenset(), full=False):
    """The contacts at the pose in data (mj_kinematics + mj_collision, or a
    full mj_forward), or None when MuJoCo's collision code fails there."""
    import mujoco

    try:
        if full:
            mujoco.mj_forward(model, data)
        else:
            mujoco.mj_kinematics(model, data)
            mujoco.mj_collision(model, data)
    except mujoco.FatalError:
        return None
    return _contacts(model, data, is_arm, is_part, margin, ignore)


def _owners(grid: np.ndarray, v: float) -> list[int]:
    """The grid indices whose half-step neighbourhood holds v: a pose
    between two grid values touches both (the guard's rule), so a contact
    anywhere within half a step of a grid value must forbid that value."""
    eps = 1e-12
    out = []
    for i in range(len(grid)):
        lo = grid[0] if i == 0 else (grid[i - 1] + grid[i]) / 2.0
        hi = grid[-1] if i == len(grid) - 1 else (grid[i] + grid[i + 1]) / 2.0
        if lo - eps <= v <= hi + eps:
            out.append(i)
    return out


def root_pass(p: Prepared, joint_ids: list[int], grids: list[np.ndarray],
              step_rad: float = math.radians(ROOT_STEP_DEG), full: bool = False):
    """The root link along its own swept joints, sampled every step_rad
    (finer than the grid): (the grid axes of those joints, {their grid
    indices: the contacts found within half a grid step of that value, or
    None after a collision error}, the (root geom, rover part) pairs left
    out because the root link touches that part at every sample, the number
    of samples). Every contact MuJoCo makes counts: the root model's parts
    carry the root margin (prepare)."""
    import mujoco

    model = p.root_model
    axes = [k for k, j in enumerate(joint_ids) if int(model.jnt_bodyid[j]) == p.root_bid]
    if not p.root_ids:
        return axes, {}, frozenset(), 0
    fine = []
    for k in axes:
        g = grids[k]
        fine.append(np.unique(np.concatenate([g, np.linspace(g[0], g[-1], int(math.ceil((g[-1] - g[0]) / step_rad)) + 1)])))
    is_arm, is_part = _masks(model, p.root_ids, p.part_ids)
    raw = {}
    for combo in itertools.product(*fine):
        data = mujoco.MjData(model)
        data.qpos[:] = p.base_qpos
        for k, v in zip(axes, combo):
            data.qpos[model.jnt_qposadr[joint_ids[k]]] = v
        raw[combo] = _touch(model, data, is_arm, is_part, math.inf, full=full)
    always = None
    if all(h is not None for h in raw.values()):
        for h in raw.values():
            parts = {r for _, r, _ in h}
            always = parts if always is None else always & parts
    ignore = frozenset((a, r) for a in p.root_ids for r in (always or ()))
    merged: dict = {}
    for combo, h in raw.items():
        for idx in itertools.product(*[_owners(grids[k], v) for k, v in zip(axes, combo)]):
            if h is None or merged.get(idx, {}) is None:
                merged[idx] = None
                continue
            cell = merged.setdefault(idx, {})
            for a, r, dd in h:
                if (a, r) not in ignore:
                    cell[(a, r)] = min(dd, cell.get((a, r), dd))
    hits = {idx: (None if c is None else [(a, r, dd) for (a, r), dd in c.items()]) for idx, c in merged.items()}
    return axes, hits, ignore, len(raw)


def _sweep_range(model, joint_ids, grids, base_qpos, arm_ids, part_ids, margin, part_no, arm_no, root_axes,
                 root_hits, lo: int, hi: int):
    """The flat cells lo..hi-1: (lo, occupancy, part numbers, pair counts,
    part counts, collision errors)."""
    import mujoco

    data = mujoco.MjData(model)
    is_arm, is_part = _masks(model, arm_ids, part_ids)
    err_no = max(part_no.values()) + 1
    occ = np.zeros(hi - lo, np.uint8)
    cell_part = np.zeros(hi - lo, np.uint16)
    pair_cells: Counter = Counter()
    part_cells: Counter = Counter()
    errors = 0
    adr = [int(model.jnt_qposadr[j]) for j in joint_ids]
    data.qpos[:] = base_qpos
    indices = itertools.product(*[range(len(g)) for g in grids])
    for i, idx in enumerate(itertools.islice(indices, lo, hi)):
        for a, g, k in zip(adr, grids, idx):
            data.qpos[a] = g[k]
        hits = _touch(model, data, is_arm, is_part, margin) if arm_ids else []
        root = root_hits.get(tuple(idx[k] for k in root_axes), []) if root_hits else []
        if hits is None or root is None:
            errors += 1
            occ[i] = 1
            cell_part[i] = err_no
            part_cells[err_no] += 1
            data = mujoco.MjData(model)
            data.qpos[:] = base_qpos
            continue
        hits = hits + root
        if hits:
            occ[i] = 1
            per_part = Counter(part_no[r] for _, r, _ in hits)
            deepest = {}
            for _, r, dd in hits:
                deepest[part_no[r]] = min(dd, deepest.get(part_no[r], dd))
            cell_part[i] = max(per_part, key=lambda k: (per_part[k], -deepest[k], -k))
            for pair in {(arm_no.get(a, a), part_no[r]) for a, r, _ in hits}:
                pair_cells[pair] += 1
            for k in per_part:
                part_cells[k] += 1
    return lo, occ, cell_part, pair_cells, part_cells, errors


_JOB = None     # what a forked worker sweeps (set by sweep() before the pool starts)


def _sweep_job(bounds):
    return _sweep_range(*_JOB, *bounds)


def sweep(model, joint_ids: list[int], grids: list[np.ndarray], base_qpos: np.ndarray,
          arm_ids: list[int], part_ids: list[int], margin: float, part_no=None, root_axes=(), root_hits=None,
          arm_no=None, progress=None, jobs: int = 1):
    """The grid, row-major (the last joint fastest). part_no: {rover geom: its
    part number, 1..} (a mesh's stand-ins share one), default one per geom
    in part_ids order; arm_no likewise for the arm geoms (pair counts).
    root_hits: the root link's contacts per index of its own joints
    (root_pass), added to every cell with that index. jobs > 1 splits the
    grid over forked processes (same result). Returns occupancy (uint8,
    1 = forbidden), the most-hit part per cell (0 = free, the part number, or
    the largest part number + 1 for a collision error), the cell counts per
    (arm number, part number) and per part number, and the number of
    collision errors."""
    import multiprocessing

    global _JOB
    part_no = part_no or {g: k + 1 for k, g in enumerate(part_ids)}
    total = int(np.prod([len(g) for g in grids]))
    occ = np.zeros(total, np.uint8)
    cell_part = np.zeros(total, np.uint16)
    pair_cells: Counter = Counter()
    part_cells: Counter = Counter()
    errors = 0
    size = max(2000, total // (max(1, jobs) * 8))
    chunks = [(lo, min(total, lo + size)) for lo in range(0, total, size)]
    job = (model, joint_ids, grids, base_qpos, arm_ids, part_ids, margin, part_no, arm_no or {},
           tuple(root_axes), root_hits)
    t0, done, shown = time.monotonic(), 0, 0

    def merge(res):
        nonlocal errors, done, shown
        lo, o, c, pc, partc, e = res
        occ[lo:lo + len(o)] = o
        cell_part[lo:lo + len(o)] = c
        pair_cells.update(pc)
        part_cells.update(partc)
        errors += e
        done += len(o)
        if progress and done < total and done - shown >= total / 10:
            shown = done
            progress(done, total, time.monotonic() - t0)

    if jobs > 1 and len(chunks) > 1 and "fork" in multiprocessing.get_all_start_methods():
        _JOB = job
        try:
            with multiprocessing.get_context("fork").Pool(min(jobs, len(chunks))) as pool:
                for res in pool.imap_unordered(_sweep_job, chunks):
                    merge(res)
        finally:
            _JOB = None
    else:
        for lo, hi in chunks:
            merge(_sweep_range(*job, lo, hi))
    return occ, cell_part, pair_cells, part_cells, errors


def _pick(occ: np.ndarray, n: int, rng, free_only: bool = False) -> list[int]:
    pools = [np.flatnonzero(occ == 0)] if free_only else [np.flatnonzero(occ), np.flatnonzero(occ == 0)]
    pick = []
    for pool in pools:
        if len(pool):
            pick.extend(rng.choice(pool, size=min(len(pool), max(1, n // len(pools))), replace=False).tolist())
    return pick


def recheck(p: Prepared, joint_ids, grids, margin, occ, n: int, seed: int = 0,
            step_rad: float = math.radians(ROOT_STEP_DEG)) -> list[int]:
    """Grid points re-done with a full mj_forward on fresh data, the root
    link included: the flat indices where the verdict differs from the table
    (half the sample from the forbidden cells, half from the free ones)."""
    import mujoco

    if n <= 0:
        return []
    rng = np.random.default_rng(seed)
    shape = tuple(len(g) for g in grids)
    axes, root_hits, _, _ = root_pass(p, joint_ids, grids, step_rad, full=True)
    is_arm, is_part = _masks(p.model, p.arm_ids, p.part_ids)
    bad = []
    for flat in _pick(occ, n, rng):
        idx = np.unravel_index(flat, shape)
        data = mujoco.MjData(p.model)
        data.qpos[:] = p.base_qpos
        for j, g, i in zip(joint_ids, grids, idx):
            data.qpos[p.model.jnt_qposadr[j]] = g[i]
        hits = _touch(p.model, data, is_arm, is_part, margin, full=True) if p.arm_ids else []
        root = root_hits.get(tuple(int(idx[k]) for k in axes), []) if root_hits else []
        hit = hits is None or root is None or bool(hits) or bool(root)
        if hit != bool(occ[flat]):
            bad.append(int(flat))
    return bad


def finger_check(real: Prepared, joint_ids, grids, margin, occ, n: int, seed: int = 1) -> tuple[int, list]:
    """Free grid points of the table, done again with the real finger geoms
    closed, half open, open and at a random opening: the (flat index, finger
    values) where the arm touches the rover. The swept volumes hold every
    finger position, so the list must be empty."""
    import mujoco

    model = real.model
    if n <= 0 or not real.loose:
        return 0, []
    rng = np.random.default_rng(seed)
    loose = [(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j), lo, hi) for j, (lo, hi) in real.loose.items()]
    settings = [{j: lo for j, lo, _ in loose}, {j: (lo + hi) / 2 for j, lo, hi in loose}, {j: hi for j, _, hi in loose}]
    is_arm, is_part = _masks(model, real.arm_ids, real.part_ids)
    shape = tuple(len(g) for g in grids)
    bad, done = [], 0
    for flat in _pick(occ, n, rng, free_only=True):
        idx = np.unravel_index(flat, shape)
        for fing in settings + [{j: rng.uniform(lo, hi) for j, lo, hi in loose}]:
            data = mujoco.MjData(model)
            data.qpos[:] = real.base_qpos
            for j, g, i in zip(joint_ids, grids, idx):
                data.qpos[model.jnt_qposadr[j]] = g[i]
            for j, v in fing.items():
                data.qpos[model.jnt_qposadr[j]] = v
            hits = _touch(model, data, is_arm, is_part, margin)
            done += 1
            if hits:
                bad.append((int(flat), {joint_name(model, j): round(float(v), 6) for j, v in fing.items()}))
    return done, bad


def keep_joint_map(out: Path, joints) -> dict:
    """The joint_map of the table being replaced (the measured alignment),
    else model_zero_ticks 2048 and sign +1 for every joint."""
    try:
        old = json.loads(Path(out).read_text()).get("joint_map", {}) or {}
    except (OSError, ValueError, AttributeError):
        old = {}
    jm = {}
    for j in joints:
        e = old.get(j, {}) if isinstance(old, dict) else {}
        jm[j] = {"model_zero_ticks": e.get("model_zero_ticks", MODEL_ZERO_TICKS), "sign": e.get("sign", 1)}
    return jm


def write_table(out: Path, doc: dict, big: dict) -> int:
    """The JSON: the header indented, each big array compact on one line."""
    head = json.dumps(doc, indent=1)
    body = ",\n".join(f' {json.dumps(k)}: {json.dumps(v, separators=(",", ":"))}' for k, v in big.items())
    text = head[:-2] + ",\n" + body + "\n}\n"
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, out)
    return len(text)


def _parse_fixed(text: str) -> dict[str, float]:
    out = {}
    for item in filter(None, (s.strip() for s in (text or "").split(","))):
        name, _, value = item.partition("=")
        out[name.strip()] = float(value)
    return out


def run(args) -> int:
    import mujoco

    t_start = time.monotonic()
    model_path = Path(args.model)
    if not model_path.is_file():
        print(f"no model at {model_path}", file=sys.stderr)
        return 2
    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    steps = [int(s) for s in args.steps.split(",")]
    if len(steps) != len(joints):
        print(f"--steps gives {len(steps)} numbers for {len(joints)} joints", file=sys.stderr)
        return 2
    parts = tuple(p.strip() for p in args.rover_parts.split(",") if p.strip())
    exclude = tuple(p.strip() for p in args.exclude.split(",") if p.strip())
    if args.no_lidar:
        lidar_xyz = None
    elif args.lidar_xyz:
        lidar_xyz = tuple(float(v) for v in args.lidar_xyz.split(","))
        if len(lidar_xyz) != 3:
            print("--lidar-xyz takes x,y,z in metres", file=sys.stderr)
            return 2
    else:
        lidar_xyz = profile_lidar_xyz()
    fixed = _parse_fixed(args.fix)
    jobs = args.jobs if args.jobs > 0 else min(MAX_JOBS, os.cpu_count() or 1)
    root_step = math.radians(args.root_step_deg)
    say = (lambda *a: None) if args.quiet else (lambda *a: print(*a, flush=True))
    common = dict(arm_root=args.arm_root, rover_body=args.rover_body, rover_parts=parts, exclude=exclude,
                  margin=args.margin, lidar_xyz=lidar_xyz, joints=joints, fixed=fixed,
                  voxel=args.stand_in_voxel, root_voxel=args.root_voxel, root_margin=args.root_margin,
                  cover_tol=args.cover_tol)
    p = prepare(model_path, **common)
    model = p.model
    lidar = p.lidar
    if lidar_xyz is not None and not lidar["added"]:
        lidar = {"added": False, "note": "the model has its own geom named lidar"}
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
    grids, ranges = [], []
    for jid, n in zip(joint_ids, steps):
        g, r = joint_grid(model, jid, n, args.pad_deg)
        grids.append(g)
        ranges.append(r)
    labels = sorted(set(p.part_label))
    if len(labels) > 253:
        print(f"{len(labels)} rover parts: more than a uint8 per cell can name (--exclude some)", file=sys.stderr)
        return 2
    shape = tuple(steps)
    total = int(np.prod(shape))
    n_stand = len(p.stand_ins["boxes"])
    meshes_in = sum(1 for r in p.stand_ins["per_mesh"].values() if r.get("boxes"))
    say(f"model {model_label(model_path)}: {len(p.arm_ids)} link geoms ({len(p.replaced)} finger geoms as swept "
        f"volumes), root link {len(p.root_ids)} geoms ({len(p.stand_ins['root_boxes'])} stand-in boxes), "
        f"{len(p.part_ids) - n_stand} rover parts as they are, {n_stand} stand-in boxes for {meshes_in} rover meshes, "
        f"lidar {'added at ' + str(lidar.get('pos_m')) if lidar.get('added') else 'not added'}")
    if lidar.get("overlaps_m"):
        say("warning: the lidar goes into " + ", ".join(f"{k} ({1000 * v:.1f} mm)" for k, v in lidar["overlaps_m"].items())
            + ": give its measured centre with --lidar-xyz")
    say(f"grid {'x'.join(map(str, shape))} = {total} poses, margin {args.margin} m, pad {args.pad_deg} deg, "
        f"{jobs} job(s), fingers {'swept: ' + ', '.join(p.loose) if p.loose else 'none'}"
        f"{', fixed: ' + ', '.join(f'{k}={v:g}' for k, v in fixed.items()) if fixed else ''}")
    label_no = {lb: k + 1 for k, lb in enumerate(labels)}
    part_no = {g: label_no[lb] for g, lb in zip(p.part_ids, p.part_label)}
    label_name = dict(enumerate(labels, start=1))
    label_name[len(labels) + 1] = COLLISION_ERROR
    label_of = dict(zip(p.part_ids, p.part_label))
    arm_labels = sorted({p.arm_label[geom_name(model, g)] for g in p.arm_ids + p.root_ids})
    arm_no = {g: arm_labels.index(p.arm_label[geom_name(model, g)]) for g in p.arm_ids + p.root_ids}

    t0 = time.monotonic()
    root_axes, root_hits, ignore, root_samples = root_pass(p, joint_ids, grids, root_step)
    mount = sorted({label_of[r] for _, r in ignore})
    if mount:
        say("the root link touches these at every value of its joints (those contacts are not counted): "
            + ", ".join(mount))

    def progress(done, tot, dt):
        say(f"  {done}/{tot} poses, {dt:.0f} s, about {dt / done * (tot - done):.0f} s left")

    occ, cell_raw, pair_cells, part_cells, errors = sweep(model, joint_ids, grids, p.base_qpos, p.arm_ids, p.part_ids,
                                                          args.margin, part_no, root_axes, root_hits, arm_no,
                                                          progress=None if args.quiet else progress, jobs=jobs)
    sweep_s = time.monotonic() - t0
    bad = recheck(p, joint_ids, grids, args.margin, occ, args.check, step_rad=root_step)
    if bad:
        print(f"{len(bad)} grid point(s) differ between mj_kinematics+mj_collision and a full mj_forward "
              f"(first flat index {bad[0]}): table not written", file=sys.stderr)
        return 3
    real = prepare(model_path, **common, fingers="real", stand_ins=p.stand_ins) if p.loose else None
    checked, touched = finger_check(real, joint_ids, grids, args.margin, occ, args.check) if real else (0, [])
    if touched:
        print(f"{len(touched)} free grid point(s) touch the rover with the real fingers "
              f"(first: flat {touched[0][0]}, fingers {touched[0][1]}): table not written", file=sys.stderr)
        return 3

    forbidden = int(occ.sum())
    # the parts hit, numbered 1.. from the most frequent most-hit part down, so
    # the common ones take one digit in cell_part
    top = Counter(int(v) for v in cell_raw[occ.astype(bool)])
    order = sorted(set(top) | set(part_cells), key=lambda k: (-top.get(k, 0), -part_cells.get(k, 0), label_name[k]))
    lut = np.zeros(len(labels) + 2, np.uint8)
    for new_no, k in enumerate(order, start=1):
        lut[k] = new_no
    cell_part = lut[cell_raw]
    pairs = sorted(((arm_labels[a], label_name[k], c) for (a, k), c in pair_cells.items()),
                   key=lambda t: (-t[2], t[0], t[1]))
    run_s = time.monotonic() - t_start
    per_mesh = dict(sorted(p.stand_ins["per_mesh"].items(), key=lambda kv: -kv[1].get("outside_fraction", 0)))
    doc = {
        "format": FORMAT,
        "made_by": "tools/moss_arm_keepout.py",
        "model": {"path": model_label(model_path), "sha256": sha256_of(model_path), "geometry_sha256": p.geometry_sha256},
        "mujoco": mujoco.__version__,
        "joints": joints,
        "shape": list(shape),
        "order": "row-major: the last joint varies fastest; flat = ravel_multi_index(indices, shape)",
        "grid": {j: [round(float(v), 9) for v in g] for j, g in zip(joints, grids)},
        "model_range": {j: [round(r[0], 9), round(r[1], 9)] for j, r in zip(joints, ranges)},
        "pad_deg": args.pad_deg,
        "margin_m": args.margin,
        "margin_rule": "a pose is forbidden when an arm geom comes closer than margin_m to a rover part; "
                       "MuJoCo's geom margin is set on the rover parts only",
        "arm_root": args.arm_root,
        "rover_body": args.rover_body,
        "rover_parts": list(parts),
        "exclude": list(exclude),
        "mount_bodies": p.mount_bodies,
        "arm_geoms": arm_labels,
        "rover_geoms": sorted(set(label_of.values()) - set(per_mesh)),
        "rover_left_out": p.unused,
        "stand_ins": {"voxel_m": p.stand_ins["voxel_m"], "cover_tol_m": p.stand_ins["cover_tol_m"], "boxes": n_stand,
                      "rule": "per rover mesh: the share of its sampled surface lying more than cover_tol_m outside "
                              "the parts that count as they are, the extent of that share in the rover frame (m), "
                              "and the boxes standing in for it",
                      "per_mesh": per_mesh},
        "root_link": {"body": args.arm_root, "joints": [joints[k] for k in root_axes], "margin_m": args.root_margin,
                      "step_deg": args.root_step_deg, "samples": root_samples,
                      "voxel_m": p.stand_ins["root_voxel_m"], "stand_in_boxes": p.stand_ins["root"],
                      "rule": "its meshes as surface stand-ins, its own joints sampled every step_deg; a contact "
                              "closer than margin_m forbids every grid value within half a grid step"},
        "mount_parts": mount,
        "mount_rule": "the root link touches these at every value of its joints: those contacts are not counted",
        "lidar": lidar,
        "unswept_joints": {"swept_volume": p.loose, "fixed": p.fixed, "replaced_geoms": p.replaced},
        "collision_errors": errors,
        "cells": total,
        "forbidden": forbidden,
        "forbidden_fraction": round(forbidden / total, 6),
        "part_names": [label_name[k] for k in order],
        "part_counts": {label_name[k]: part_cells[k] for k in order},
        "part_counts_rule": "cells where the arm touches this part (a cell counts for every part it touches)",
        "part_top_counts": {label_name[k]: top.get(k, 0) for k in order},
        "pair_counts": {f"{a}|{r}": c for a, r, c in pairs},
        "run_time_s": round(run_s, 1),
        "sweep_time_s": round(sweep_s, 1),
        "rechecked_with_mj_forward": args.check,
        "finger_check": {"poses": checked, "touching": len(touched)},
        "ticks_per_rev": TICKS_PER_REV,
        "joint_map": keep_joint_map(Path(args.out), joints),
        "joint_map_rule": "rad = sign * (ticks - model_zero_ticks) * 2 pi / ticks_per_rev; "
                          "ticks as the servo reports them (homing offset applied); align with two poses (docs/arm.md)",
        "occupancy_rule": "occupancy: one uint8 per cell, 1 = forbidden, flat in row-major order, packed 8 cells "
                          "per byte (numpy packbits, first cell in the high bit), written in hex",
        "cell_part_rule": "0 = free, k = part_names[k - 1]: the rover part with the most contacts in that cell",
        "occupancy_bits_hex": np.packbits(occ).tobytes().hex(),
    }
    size = write_table(Path(args.out), doc, {"cell_part": cell_part.tolist()})

    say(f"{total} cells, {forbidden} forbidden ({100.0 * forbidden / total:.1f} %), "
        f"sweep {sweep_s:.1f} s, total {run_s:.1f} s, {size / 1e6:.2f} MB -> {args.out}")
    if errors:
        say(f"warning: {errors} pose(s) where MuJoCo's collision failed, marked forbidden")
    say("rover parts hit (cells): " + ", ".join(f"{label_name[k]} {part_cells[k]}" for k in order[:12]))
    say("worst pairs (cells): " + ", ".join(f"{a}|{r} {c}" for a, r, c in pairs[:8]))
    if checked:
        say(f"finger check: {checked} poses of free grid points with the real fingers, none touches")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True, help="the robot's MJCF")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="the table (default config/arm_keepout.json)")
    ap.add_argument("--steps", default=",".join(map(str, STEPS)), help="grid points per joint, in --joints order")
    ap.add_argument("--margin", type=float, default=MARGIN_M, help="metres: closer than this is a contact")
    ap.add_argument("--pad-deg", type=float, default=PAD_DEG, help="each hinge range widened by this on both sides")
    ap.add_argument("--joints", default=",".join(JOINTS))
    ap.add_argument("--arm-root", default="shoulder_link", help="the first moving body of the arm")
    ap.add_argument("--rover-body", default="rover")
    ap.add_argument("--rover-parts", default=",".join(ROVER_PARTS),
                    help="geom name patterns (fnmatch) that count as they are, meshes included")
    ap.add_argument("--exclude", default="", help="geom name patterns (fnmatch) that are not rover parts")
    ap.add_argument("--stand-in-voxel", type=float, default=VOXEL_M,
                    help="metres: the box grid of the rover meshes' stand-ins (0: those meshes are left out)")
    ap.add_argument("--root-voxel", type=float, default=ROOT_VOXEL_M,
                    help="metres: the box grid of the root link's stand-ins")
    ap.add_argument("--root-margin", type=float, default=ROOT_MARGIN_M,
                    help="metres: the root link's contacts count closer than this (it turns in place; at most --margin)")
    ap.add_argument("--root-step-deg", type=float, default=ROOT_STEP_DEG,
                    help="degrees between the root link's samples along its own joints")
    ap.add_argument("--cover-tol", type=float, default=COVER_TOL_M,
                    help="metres: a mesh surface this close to a part that counts as it is needs no stand-in")
    ap.add_argument("--lidar-xyz", default="", help="x,y,z of the lidar's centre on the rover body (m)")
    ap.add_argument("--no-lidar", action="store_true", help="do not add the lidar")
    ap.add_argument("--fix", default="", help="name=value,... pins unswept arm joints (else swept as volumes)")
    ap.add_argument("--check", type=int, default=CHECK_N,
                    help="grid points re-done with mj_forward, and free ones with the real fingers")
    ap.add_argument("--jobs", type=int, default=0, help="processes for the sweep (0: one per core, up to 16)")
    ap.add_argument("--quiet", action="store_true")
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("mujoco is not installed: pip install mujoco", file=sys.stderr)
        return 2
    args = ap.parse_args(argv)
    if args.root_voxel <= 0 or args.root_step_deg <= 0:
        print("--root-voxel and --root-step-deg must be positive", file=sys.stderr)
        return 2
    if not 0.0 <= args.root_margin <= args.margin:
        print("--root-margin goes from 0 to --margin", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
