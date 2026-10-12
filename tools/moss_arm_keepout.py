#!/usr/bin/env python3
"""The arm's keep-out table: every arm pose that touches the rover, swept on a
MuJoCo model of the robot.

Two model layouts, told apart by their body names (--layout auto):
  * cad: the CAD scene (a body "rover_exterior"). Every part of the CAD has
    its own collision geoms (one convex hull, or the convex pieces of a
    decomposition, named <part>_NNN) and its visual meshes (contype and
    conaffinity 0). Defaults: --rover-body rover_exterior,base (the rover's
    exterior and the arm's own base, which hangs from the world, not from
    the rover), --arm-root shoulder, --rover-parts "*" (every collision geom
    counts as it is), --visual skip (the visual meshes are left out, on the
    arm and on the rover: the collision geoms stand for them; a body with no
    collision geom keeps its visual meshes, nothing else stands for them:
    the gripper's drive gear and servo horn), --root-voxel
    0 (the root link's collision geoms count as they are), no lidar added
    (the scene has the lidar, its base and its carriage as parts), --margin
    0.002 (the hulls are the parts themselves, and the folded arm rests 3 to
    4 mm from the bin's front wall: 10 mm would forbid the driving pose).
  * sim: the simulation model (a body "rover", or anything else). Defaults:
    --rover-body rover, --arm-root shoulder_link, --rover-parts hull,bin_*,
    track_*,tread_*,lidar, --visual count (a visual mesh is a part too),
    --root-voxel 0.001, the lidar cylinder added at the profile's pose,
    --margin 0.01.
Any of these flags given on the command line wins over the layout.

The arm is every geom from --arm-root down (with --visual count the visual
meshes too). At every point of a grid over its five joints the model is posed
(mj_kinematics, the position stage of mj_forward) and mj_collision lists the
contacts closer than --margin between an arm geom and a rover part. A grid
point with one such contact is forbidden. Nothing is stepped: no dynamics,
the arm is placed, never moved. A mesh collides as its convex hull, the whole
hull: the tool lifts the model's cap on hull vertices (maxhullvert), which
would cut a hull to fewer vertices and leave the part's corners outside it.

The root link (the body --arm-root, which only its own joints move: the pan)
is done apart. It is placed every --root-step-deg along its own joints, and a
contact there forbids every grid value within half a grid step (a pose
between two grid values touches both: the guard's rule), whatever the other
joints do. It turns in place at a clearance the build sets, so it counts
contacts closer than --root-margin (0: where it meets the rover), not
--margin. With --root-voxel above 0 its meshes are replaced by stand-ins that
follow their surface (below): the convex hull of a bracket fills its inside
and would sweep through whatever sits next to the arm's base.

The rover is every geom of the bodies --rover-body lists (comma separated)
and of the bodies under them, except the arm, the bodies between the arm and
the nearest listed body above it (its mount: base_link in the simulation
model), the visual meshes with --visual skip, and the geoms --exclude names:
  * the geoms --rover-parts names and every other primitive (box, sphere,
    capsule, cylinder, ellipsoid) count as they are;
  * every other mesh counts through stand-ins: its convex hull would fill a
    concave part (the inside of the bin, the deck around the arm's base).
    Its surface is sampled every half voxel, the samples more than
    --cover-tol outside the parts above are kept, and boxes on a
    --stand-in-voxel grid cover them (whole voxels, merged). Every point of
    the surface lies inside a stand-in or within --cover-tol plus half a
    voxel of a part that counts as it is; a stand-in reaches past the surface
    by less than one and a half voxels along each axis. The root link's
    stand-ins use --root-voxel and cover all of its surface.
The JSON lists every rover mesh that has stand-ins with the share of its
surface the parts that count as they are miss, where that share lies, and
its number of boxes. A rover geom held whole inside another that counts as
it is (as MuJoCo collides it: a mesh's hull, a primitive) adds no forbidden
pose, since an arm geom near it is nearer the other: it is left out and
listed (rover_inside; --keep-inside keeps it). In the CAD scene that drops
the wheels inside the track envelopes and the hardware inside the hull's
envelope.

The rover's geoms are counted: those that count as they are, the meshes
with stand-ins, those held inside another, and those left out by name add
up to every collision geom of the rover bodies (rover_geoms_counted,
rover_inside, rover_left_out in the JSON).

Parts. The collision geoms of one CAD part count as one part: a geom named
<part>_NNN (three digits) or <part>_envelope, after a part name that ends in
digits, is a piece of <part> (col_part_0996_017 -> col_part_0996). --names
<parts_manifest.json> ({"parts": [{"id": "part_0996", "name": ...}]}) gives
each part its name from the CAD: the part col_part_0996 (or part_0996) shows
as that name in the counts, the summary and the guard's why(). A name the
CAD exporter made up ("Open CASCADE STEP translator ...") takes the name of
the component that holds it, a name several parts share gets the part's id
in brackets, and the CAD's working code for the gripper (the first word of
its parts' names, two capitals and two digits) reads "gripper". A visual
mesh that counts (above) is named after the parts of its body in its colour.
Without --names the part key is the name.

The model's own exclusions between an arm body and a rover body are kept
(the CAD scene excludes the base and the shoulder: one turns on the other);
its other exclusions and its explicit pairs are dropped, and the parent
filter is off: the masks below decide every other contact. A rover part that
the root link touches at every value of its joints is where the arm sits:
the root link's contacts with it are not counted (every other link's are).
Such parts are printed and listed in the JSON (mount_parts).

The lidar. In the simulation model it is not there: when no geom is named
"lidar", a cylinder (radius 0.038 m, height 0.04 m) is added on the first
rover body at the profile's pose: x = y = 0 (base_link is the lidar's floor
projection, on the rotation centre) and z = lidar_height_m
(moss_dimos/robot_profile.py, MOSS_LIDAR_HEIGHT_M). --lidar-xyz moves it,
--no-lidar leaves it out. The rover parts it overlaps are printed and stored
(lidar.overlaps_m). In the CAD scene the lidar is a part like the others and
nothing is added unless --lidar-xyz asks for it.

The fingers (every arm joint that is not swept) take every position: each
geom they carry is replaced by its swept volume, the convex hull of the geom
at both ends of a slide (every 10 degrees of a hinge), on the body above the
finger. A closed, open or half-open gripper is inside it. --fix name=value
pins a joint to one value instead. The base pose of the sweep holds the
fingers open: each finger joint at the end of its range that spreads the
gripper the most, and a joint that one of the model's joint equalities ties
to another finger joint follows it (the CAD scene's right jaw follows the
left one, its pinion follows the left jaw's rack: MuJoCo applies none of
that under mj_kinematics, so the tool sets them itself). The JSON says which
(fingers.open).

The margin is MuJoCo's geom margin on the rover parts only (MuJoCo adds the
margins of the two geoms of a pair). MuJoCo's own convex collider is switched
off for libccd's: the former (3.9) reports no contact for a geom held whole
inside a mesh's hull. libccd says whether two geoms come within the margin;
the distance it reports with a contact is approximate (the verdicts use
only the first). The arena is raised to 256 MB per MjData (reserved, used
only by a pose deep in the rover, which makes 10000 contacts and more). A
pose where MuJoCo's collision code still fails (mujoco.FatalError) is marked
forbidden and counted (collision_errors).

Each joint is swept over its model range widened by --pad-deg on both sides:
the real arm's end stops (its calibrated travel) sit a few degrees past the
SO-101 model's limits, and the folded driving pose lies on them. With
--calibration (the arm's config/arm_calibration.json) the grid also covers
each joint's calibrated travel, range_min to range_max through the
joint_map, plus 3 degrees past each end (travel in the JSON): the guard
refuses a pose outside the table, so the table must hold every pose the
arm can reach.

The payload (--payload; the CAD layout: can, the simulation layout: none).
A pose the table allows must stay clear with a can in the gripper. The can
is a cylinder (66 mm across, 120 mm tall; cyl:<d>x<h> gives another) added
to the fingers' body as an arm geom (it meets the rover parts only), the
fingers closed on it: their pads, the parts of the two fingers past the
hand's own geoms, read from the finger geoms at the slide values, are its
diameter apart. It is centred between the pads' inner faces and across
them, its centre at the pads' middle along the approach (the hand's axis,
perpendicular to the slide, that points from the hand to the fingers),
then pushed out along the approach until it clears the hand's other geoms:
a can cannot go into the palm. It is held two ways: upright, its axis along
the approach (a can standing on the floor, picked from above), and across,
its axis across both the slide and the approach (a can lying down, picked
from the side). The table is the union of the empty gripper (every jaw
position) and the can held either way: the can is swept alone, the arm at
the holding gap lying inside the jaws' swept volumes. The JSON records the
payload: its size, the finger values that hold it, the pads, its pose in
the fingers' body's frame for each orientation, and the forbidden cells of
each sweep (payload, occupancy_empty_bits_hex). --reuse-empty takes the
empty gripper's sweep from an earlier table of the same model (both
sha256), grid and settings, and sweeps only the payload.

Checks before writing: --check grid points are done again with a full
mj_forward and must agree (the empty gripper's own cells, then the table
with the payload), and --check free grid points are done again with
the real fingers closed, half open, open and at a random opening: none may
touch the rover.

    python tools/moss_arm_keepout.py --model MOSS_R31.xml --names parts_manifest.json \
        --calibration config/arm_calibration.json                                       # -> config/arm_keepout.json
    python tools/moss_arm_keepout.py --model <robot.xml> --steps 8,8,8,6,4 --out /tmp/k.json

Options: --layout (auto, cad, sim), --steps pan,lift,elbow,flex,roll
(16,16,16,12,8), --margin metres, --pad-deg (10), --calibration (none),
--joints (the five SO-101 joint names), --arm-root, --rover-body,
--rover-parts, --visual (count, skip), --names, --exclude (none),
--keep-inside, --stand-in-voxel (0.004 m, 0 = rover meshes left out),
--root-voxel (0 = the root link's geoms as they are), --root-margin (0 m),
--root-step-deg (1), --cover-tol (0.002 m), --lidar-xyz x,y,z, --no-lidar,
--fix, --payload (by layout), --reuse-empty, --check N (64), --jobs (one
per core, up to 16), --quiet.

The table lists only the parts the sweep hit (part_names); cell_part names
each forbidden cell's part by its number in that list, one byte per cell
while the list holds 255 parts or fewer, two bytes beyond (cell_part_dtype).
pair_counts names the most frequent (arm part, rover part) pairs, pair_cells
holds them all by number.
The JSON keeps the joint_map (ticks <-> model radians) of the file it
replaces, with its basis: the alignment measured on the arm survives a new
sweep. It names the model by its file name, never its path (the two sha256
say which file). Needs mujoco and numpy here (scipy, when present, lets a
mesh named in --rover-parts cover the stand-ins with its hull);
moss_dimos/arm_guard.py reads the table with numpy only. docs/arm.md has
the whole story.
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
CAD_MARGIN_M = 0.002
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
# the two model layouts (--layout): the defaults each one gives the flags left unset
LAYOUTS = {
    "sim": {"rover_body": "rover", "arm_root": "shoulder_link", "rover_parts": ",".join(ROVER_PARTS),
            "visual": "count", "root_voxel": ROOT_VOXEL_M, "lidar": True, "margin": MARGIN_M, "payload": "none"},
    # the CAD's collision hulls are the parts themselves, and the folded arm
    # rests 3 to 4 mm from the bin's front wall: a 10 mm margin would forbid
    # the robot's own driving pose
    # the robot's rule: a pose the table allows must stay clear with a can in the gripper
    "cad": {"rover_body": "rover_exterior,base", "arm_root": "shoulder", "rover_parts": "*",
            "visual": "skip", "root_voxel": 0.0, "lidar": False, "margin": CAD_MARGIN_M, "payload": "can"},
}
CAD_BODY = "rover_exterior"          # the body that tells the CAD scene apart
# a piece of a CAD part: <part>_NNN (a decomposition's hull) or <part>_envelope,
# after a part name that ends in digits
PIECE = re.compile(r"^(?P<part>.+_\d+)_(?:\d{3}|envelope)$")
EXPORTER_NAME = re.compile(r"^Open CASCADE STEP translator")    # a name the CAD exporter made up
MAX_PARTS_U8 = 255                   # cell_part in one byte while the table names this many parts or fewer
ARENA_BYTES = 256 << 20              # MuJoCo's arena per MjData: a pose deep in the rover makes 10000+ contacts
TOP_PAIRS = 100                      # pair_counts by name: the most frequent pairs (pair_cells holds them all)
TRAVEL_PAD_DEG = 3.0                 # --calibration: the grid reaches this far past each end of the calibrated travel
# --payload: what the gripper holds; a pose the table allows stays clear with it
PAYLOADS = {"can": (0.066, 0.120)}   # a soda can: diameter, height (m)
PAYLOAD_ORIENTATIONS = ("upright", "across")
PAD_BAND_M = 0.001                   # a finger's pad: its points within this of the finger's inner face
PUSH_STEP_M = 0.001                  # the payload leaves the hand in steps of this, then halved down to PUSH_TOL_M
PUSH_TOL_M = 0.00005
# the CAD names the gripper's parts after its working code, the first word
# (two capitals and two digits, sometimes after "MOSS "): the table says "gripper"
WORK_CODE = re.compile(r"^(?:MOSS )?[A-Z]{2}\d{2} +")
JOINT_MAP_BASIS = (
    "first alignment: model_zero_ticks 2048 and sign +1 on all five joints (the middle of each calibrated travel "
    "on the model's zero). Rendered on the SO-101 of the CAD scene with this map, the two poses recorded on the arm "
    "look right: drive (ticks pan 2857, lift 860, elbow 3206, flex 868, roll 3221 = pan +71.1, lift -104.4, "
    "elbow +101.8, flex -103.7, roll +103.1 deg) is the arm folded, the forearm back over the upper arm, the "
    "gripper straight up; look (ticks pan 2869, lift 1057, elbow 3208, flex 2018, roll 3220) is the forearm level, "
    "pointing ahead, the gripper forward and down (flex -2.6 deg). drive sits past the model's limits on the lift "
    "(4 deg), the elbow (5) and the wrist flex (9), look on the elbow (5): the real end stops lie past the model's "
    "(the reason for pad_deg). The pan zero is the least certain, about 15 deg either way. The final check is the "
    "operator's eyes on the robot: the same two poses, model against arm (docs/arm.md).")


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
    """The model as written in the JSON: its file name, never its path (a
    path can name a private checkout; the two sha256 say which file)."""
    return Path(path).name


def layout_of(model_path: Path) -> str:
    """"cad" for the CAD scene (it has a body named rover_exterior), else "sim"."""
    import mujoco

    spec = mujoco.MjSpec.from_file(str(model_path))       # held while its bodies are read: they live in it
    bodies = {b.name for b in spec.bodies}
    return "cad" if CAD_BODY in bodies else "sim"


def part_key(name: str) -> str:
    """The part a geom belongs to: a stand-in or a swept volume answers to its
    source geom, a CAD piece to its part (col_part_0996_017 -> col_part_0996)."""
    name = name.removesuffix(SWEPT_SUFFIX)
    src, sep, k = name.rpartition(STAND_IN_SEP)
    if sep and k.isdigit():
        name = src
    m = PIECE.match(name)
    return m.group("part") if m else name


def public_name(name: str) -> str:
    """A CAD name as the table shows it: the gripper's working code (WORK_CODE)
    reads "gripper"."""
    m = WORK_CODE.match(name)
    if not m:
        return name
    rest = name[m.end():]
    return rest if rest.lower().startswith("gripper") else "gripper " + rest


def _rgba_key(value) -> tuple:
    try:
        return tuple(round(float(v), 3) for v in value)
    except (TypeError, ValueError):
        return ()


def load_names(path) -> dict[str, str]:
    """{part id: the name to show} from a parts manifest: {"parts": [{"id",
    "name", "path", "rig", "rgba"}], "visuals": [{"id", "rig", "rgba"}]}. A
    name the CAD exporter made up takes the name of the component that holds
    it (the path's next-to-last step); the gripper's working code reads
    "gripper" (public_name); a name several parts share gets the part's id in
    brackets. A visual mesh is named after the parts of its rigid group in
    its colour, "(visual mesh)" after it."""
    doc = json.loads(Path(path).read_text())
    parts = doc.get("parts") if isinstance(doc, dict) else None
    if not isinstance(parts, list):
        raise SystemExit(f"{path}: no \"parts\" list (--names takes a parts manifest)")
    raw, groups = {}, {}
    for p in parts:
        if not isinstance(p, dict) or not p.get("id"):
            continue
        name = str(p.get("name") or p["id"]).strip()
        steps = [s.rsplit(":", 1)[0] for s in str(p.get("path", "")).split("+") if s]
        if EXPORTER_NAME.match(name) and len(steps) >= 2:
            name = steps[-2]
        raw[str(p["id"])] = public_name(name)
        groups.setdefault((p.get("rig"), _rgba_key(p.get("rgba"))), []).append(str(p["id"]))
    shared = Counter(raw.values())
    out = {i: (f"{n} ({i})" if shared[n] > 1 else n) for i, n in raw.items()}
    for v in doc.get("visuals") or []:
        if not isinstance(v, dict) or not v.get("id"):
            continue
        ids = sorted(groups.get((v.get("rig"), _rgba_key(v.get("rgba"))), []))
        if ids:
            more = f" and {len(ids) - 1} more" if len(ids) > 1 else ""
            out[str(v["id"])] = f"{raw[ids[0]]}{more} (visual mesh)"
    return out


def display_name(names: dict, key: str) -> str:
    """A part key as shown: its manifest name (the key, or the key without its
    "col_" prefix, as the manifest's id), else the key."""
    if key in names:
        return names[key]
    if key.startswith("col_") and key[4:] in names:
        return names[key[4:]]
    return key


def joint_couplings(model) -> dict[int, tuple[int, list[float]]]:
    """{joint1: (joint2, polycoef)} for the model's joint equalities that tie a
    joint to another: joint1 - its qpos0 = poly(joint2 - its qpos0)."""
    import mujoco

    out = {}
    for e in range(model.neq):
        if int(model.eq_type[e]) != int(mujoco.mjtEq.mjEQ_JOINT) or not bool(model.eq_active0[e]):
            continue
        j1, j2 = int(model.eq_obj1id[e]), int(model.eq_obj2id[e])
        if j2 >= 0:
            out[j1] = (j2, [float(v) for v in model.eq_data[e][:5]])
    return out


def apply_couplings(model, qpos: np.ndarray, couplings: dict, joints) -> None:
    """Sets every joint of `joints` that a coupling ties to another from that
    other's value (MuJoCo applies equalities only when it steps)."""
    joints = set(joints)
    for _ in range(len(couplings) + 1):
        for j1, (j2, c) in couplings.items():
            if j1 not in joints:
                continue
            a1, a2 = int(model.jnt_qposadr[j1]), int(model.jnt_qposadr[j2])
            x = float(qpos[a2] - model.qpos0[a2])
            qpos[a1] = model.qpos0[a1] + sum(ck * x ** k for k, ck in enumerate(c))


def open_fingers(model, qpos: np.ndarray, loose: list[int], couplings: dict) -> dict[int, float]:
    """{finger joint: its value with the gripper open}: each finger joint at
    the end of its range that spreads the gripper the most (the bounding box
    of every geom the finger joints move, the other fingers as in qpos), and a
    joint a coupling ties to another finger joint following it. Ties go to the
    lower end."""
    import mujoco

    if not loose:
        return {}
    data = mujoco.MjData(model)
    q = np.array(qpos, float)
    moved = set()
    for j in loose:
        moved |= _subtree(model, int(model.jnt_bodyid[j]))
    local = {}
    for g in range(model.ngeom):
        if int(model.geom_bodyid[g]) in moved:
            try:
                local[g] = hull_points_local(model, g)
            except SystemExit:
                continue

    def spread(qq):
        data.qpos[:] = qq
        mujoco.mj_kinematics(model, data)
        if not local:
            return 0.0
        pts = np.concatenate([_to_world(data, g, p) for g, p in local.items()])
        return float(np.linalg.norm(pts.max(0) - pts.min(0)))

    followers = {j for j in loose if j in couplings and couplings[j][0] in loose}
    for j in loose:
        if j in followers:
            continue
        lo, hi = (float(v) for v in model.jnt_range[j]) if model.jnt_limited[j] else (-math.pi, math.pi)
        best = None
        for v in (lo, hi):
            qq = q.copy()
            qq[model.jnt_qposadr[j]] = v
            apply_couplings(model, qq, couplings, followers)
            s = spread(qq)
            if best is None or s > best[0] + 1e-9:
                best = (s, v)
        q[model.jnt_qposadr[j]] = best[1]
    apply_couplings(model, q, couplings, followers)
    return {j: float(q[model.jnt_qposadr[j]]) for j in loose}


def parse_payload(text) -> dict | None:
    """--payload: "none", "can" (66 mm across, 120 mm tall) or
    "cyl:<diameter mm>x<height mm>", as {"kind", "diameter_m", "height_m"}
    (None for none)."""
    t = str(text or "").strip().lower()
    if t in ("", "none"):
        return None
    if t in PAYLOADS:
        d, h = PAYLOADS[t]
        return {"kind": t, "diameter_m": d, "height_m": h}
    m = re.fullmatch(r"cyl:(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)", t)
    if not m or float(m.group(1)) <= 0 or float(m.group(2)) <= 0:
        raise SystemExit(f"--payload takes none, can or cyl:<diameter mm>x<height mm>, not {text!r}")
    return {"kind": "cyl", "diameter_m": float(m.group(1)) / 1000.0, "height_m": float(m.group(2)) / 1000.0}


def payload_label(payload: dict, orientation: str) -> str:
    """The payload as an arm part of the table: "payload can, upright"."""
    what = payload["kind"] if payload["kind"] != "cyl" else (
        f"cylinder {1000 * payload['diameter_m']:g} x {1000 * payload['height_m']:g} mm")
    return f"payload {what}, {orientation}"


def _frame_quat(z_axis, x_axis) -> list[float]:
    """The quaternion of the frame whose z is z_axis and whose x is x_axis
    (unit and perpendicular), in the frame they are written in."""
    import mujoco

    z, x = np.asarray(z_axis, float), np.asarray(x_axis, float)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.column_stack([x, np.cross(z, x), z]).ravel())
    return [float(v) for v in q]


def grip_payload(model, qpos, loose, couplings, counts, payload: dict) -> dict:
    """Where a cylinder payload sits in the gripper, in the frame of the
    fingers' body (the hand). The gripper: two fingers on slide joints under
    one body (a finger a coupling ties to the other follows it). s, the
    slide axis, is the first finger's joint axis; a, the approach (where the
    fingers point), is the axis of the hand's frame perpendicular to s that
    points from the hand's own geoms to the fingers; c = s x a. A finger's
    pad is its geoms that reach the finger's inner face (its nearest point
    to the other finger along s, among its points past the hand's own geoms
    along a), within 1 mm, as far as they reach past the hand. The fingers close
    until the two faces are the payload's diameter apart (the slides at one
    value, a coupled joint through its coupling). The payload's centre lies
    midway between the faces, at the pads' middle across (c) and along the
    approach (a); from there it is pushed out along a (1 mm steps, then
    halved down to 0.05 mm) until it clears every other geom of the hand
    and of its fingers: a can cannot go into the palm. "upright" holds its
    axis along a (a can standing on the floor, picked from above), "across"
    along c (a can lying down, picked from the side). qpos: the open pose.
    counts(geom): an arm geom of the sweep. Returns the hand, the finger
    values that hold it, the gap, the axes, the pads and a pose per
    orientation (pos, axis, quat, how far it was pushed out and by what)."""
    import mujoco

    loose = list(loose)
    followers = {j for j in loose if j in couplings and couplings[j][0] in loose}
    slides = [j for j in loose if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_SLIDE)]
    fingers = sorted({int(model.jnt_bodyid[j]) for j in slides})
    parents = {int(model.body_parentid[b]) for b in fingers}
    drivers = [j for j in slides if j not in followers]
    if len(fingers) != 2 or len(parents) != 1 or not drivers:
        raise SystemExit("--payload: the gripper needs two fingers on slide joints under one body, swept (not "
                         "pinned with --fix); the arm's unswept joints here: "
                         + (", ".join(joint_name(model, j) for j in loose) or "none"))
    hand = parents.pop()
    for j in drivers:
        if not model.jnt_limited[j]:
            raise SystemExit(f"--payload: finger {joint_name(model, j)!r} slides without limits")
    lo = max(float(model.jnt_range[j][0]) for j in drivers)
    hi = min(float(model.jnt_range[j][1]) for j in drivers)
    sub = _subtree(model, hand)
    geoms = [g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in sub and counts(g)]
    local = {g: hull_points_local(model, g) for g in geoms}
    side = [_subtree(model, f) for f in fingers]
    on = [[g for g in geoms if int(model.geom_bodyid[g]) in side[k]] for k in (0, 1)]
    own = [g for g in geoms if int(model.geom_bodyid[g]) == hand]
    if not on[0] or not on[1]:
        raise SystemExit("--payload: a finger carries no geom")
    data = mujoco.MjData(model)

    def pose(v):
        q = np.array(qpos, float)
        for j in drivers:
            q[model.jnt_qposadr[j]] = v
        apply_couplings(model, q, couplings, followers)
        return q

    def points(q):
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        return {g: _to_body(data, hand, _to_world(data, g, local[g])) for g in geoms}

    pts = points(np.array(qpos, float))
    j0 = next(j for j in slides if int(model.jnt_bodyid[j]) in side[0])
    s = np.asarray(data.xmat[hand]).reshape(3, 3).T @ np.asarray(data.xaxis[j0], float)
    s /= np.linalg.norm(s)

    def cat(gs, pp):
        return np.concatenate([pp[g] for g in gs])

    if float((cat(on[0], pts) @ s).mean()) > float((cat(on[1], pts) @ s).mean()):
        s = -s                      # s points from the first finger to the second
    fp = cat(on[0] + on[1], pts)
    ref = (fp.min(0) + fp.max(0)) / 2.0
    if own:
        hp = cat(own, pts)
        ref = ref - (hp.min(0) + hp.max(0)) / 2.0
    ref = ref - (ref @ s) * s
    cands = [e for e in np.vstack([np.eye(3), -np.eye(3)]) if abs(float(e @ s)) < 1e-3]
    a = max(cands, key=lambda e: float(e @ ref)) if cands else ref / max(float(np.linalg.norm(ref)), 1e-12)
    a = np.asarray(a, float)
    if float(a @ ref) <= 0:
        raise SystemExit("--payload: the fingers do not reach past the hand's own geoms: no approach direction")
    s = s - (s @ a) * a
    s /= np.linalg.norm(s)
    c = np.cross(s, a)
    palm = max(float((pts[g] @ a).max()) for g in own) if own else -math.inf

    def faces(v):
        pp = points(pose(v))
        out = []
        for k in (0, 1):
            proj = []
            for g in on[k]:
                x = pp[g][pp[g] @ a > palm + 1e-9]
                if len(x):
                    proj.append((g, x, x @ s))
            if not proj:
                raise SystemExit("--payload: a finger has nothing past the hand's own geoms")
            f = max(float(d.max()) for _, _, d in proj) if k == 0 else min(float(d.min()) for _, _, d in proj)
            # the pad: every geom with a point on the face, as far as it reaches past the hand
            out.append((f, [(g, x) for g, x, d in proj
                            if np.any((d >= f - PAD_BAND_M) if k == 0 else (d <= f + PAD_BAND_M))]))
        return out

    def gap(v):
        (fa, _), (fb, _) = faces(v)
        return fb - fa

    dia, height = float(payload["diameter_m"]), float(payload["height_m"])
    g_lo, g_hi = gap(lo), gap(hi)
    if not min(g_lo, g_hi) - 1e-9 <= dia <= max(g_lo, g_hi) + 1e-9:
        raise SystemExit(f"--payload: {1000 * dia:g} mm across does not fit: the pads open from "
                         f"{1000 * min(g_lo, g_hi):.1f} to {1000 * max(g_lo, g_hi):.1f} mm")
    v = lo if g_hi == g_lo else lo + (dia - g_lo) * (hi - lo) / (g_hi - g_lo)
    if abs(gap(v) - dia) > 1e-7:          # the gap is not linear in the slide: halve the interval
        a_, b_, ga = lo, hi, g_lo
        for _ in range(60):
            m_ = (a_ + b_) / 2.0
            gm = gap(m_)
            if (gm - dia) * (ga - dia) <= 0:
                b_ = m_
            else:
                a_, ga = m_, gm
        v = (a_ + b_) / 2.0
    (fa, pad_a), (fb, pad_b) = faces(v)
    q_hold = pose(v)
    pad_ids = sorted({g for g, _ in pad_a + pad_b})

    def middle(pad, axis):
        x = np.concatenate([x for _, x in pad]) @ axis
        return (float(x.min()) + float(x.max())) / 2.0

    along_a = (middle(pad_a, a) + middle(pad_b, a)) / 2.0
    across_c = (middle(pad_a, c) + middle(pad_b, c)) / 2.0
    pad_pts = np.concatenate([x for _, x in pad_a + pad_b])
    pad_range = [float((pad_pts @ a).min()), float((pad_pts @ a).max())]
    centre0 = s * (fa + fb) / 2.0 + a * along_a + c * across_c

    # the hand and its fingers at the holding pose, the pads left out (the
    # payload touches them), in a small model of their own: MuJoCo says
    # where the payload goes into one of them
    data.qpos[:] = q_hold
    mujoco.mj_kinematics(model, data)
    rot = np.asarray(data.xmat[hand]).reshape(3, 3)
    probe = mujoco.MjSpec()
    if hasattr(mujoco.mjtDisableBit, "mjDSBL_NATIVECCD"):
        probe.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    body = probe.worldbody.add_body(name="hand")
    others = [g for g in geoms if g not in pad_ids]
    mesh_t = int(mujoco.mjtGeom.mjGEOM_MESH)
    for k, g in enumerate(others):
        pos = rot.T @ (np.asarray(data.geom_xpos[g]) - data.xpos[hand])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, (rot.T @ np.asarray(data.geom_xmat[g]).reshape(3, 3)).ravel())
        if int(model.geom_type[g]) == mesh_t:
            mesh = probe.add_mesh(name=f"m{k}", uservert=local[g].ravel().tolist())
            mesh.maxhullvert = -1
            body.add_geom(name=f"g{k}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"m{k}",
                          pos=pos.tolist(), quat=quat.tolist())
        else:
            body.add_geom(name=f"g{k}", type=mujoco.mjtGeom(int(model.geom_type[g])),
                          size=[float(x) for x in model.geom_size[g]], pos=pos.tolist(), quat=quat.tolist())
    body.add_geom(name="payload", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[dia / 2.0, height / 2.0, 0.0])
    pm = probe.compile()
    pdata = mujoco.MjData(pm)
    cid = mujoco.mj_name2id(pm, mujoco.mjtObj.mjOBJ_GEOM, "payload")
    pm.geom_sameframe[cid] = 0         # its pos and quat change below: kinematics must read them
    probe_ids = [mujoco.mj_name2id(pm, mujoco.mjtObj.mjOBJ_GEOM, f"g{k}") for k in range(len(others))]

    def blocker(t):
        pm.geom_pos[cid] = centre0 + t * a
        mujoco.mj_kinematics(pm, pdata)
        for k, gk in enumerate(probe_ids):
            if mujoco.mj_geomDistance(pm, pdata, cid, gk, 0.0, None) < 0:
                return others[k]
        return None

    poses = {}
    for o in PAYLOAD_ORIENTATIONS:
        axis = a if o == "upright" else c
        quat = _frame_quat(axis, s)
        pm.geom_quat[cid] = quat
        t, by = 0.0, blocker(0.0)
        last = by
        while by is not None:
            t += PUSH_STEP_M
            if t > height + dia:
                raise SystemExit(f"--payload: held {o}, it goes into {geom_name(model, last)} wherever it sits "
                                 "along the approach")
            by = blocker(t)
            last = by if by is not None else last
        if t > 0:
            t_in, t_out = t - PUSH_STEP_M, t
            while t_out - t_in > PUSH_TOL_M:
                mid = (t_in + t_out) / 2.0
                hit = blocker(mid)
                if hit is None:
                    t_out = mid
                else:
                    t_in, last = mid, hit
            t = t_out
        centre = centre0 + t * a
        ca = float(centre @ a)
        held = (ca - height / 2.0 < pad_range[1] and ca + height / 2.0 > pad_range[0]) if o == "upright" \
            else pad_range[0] <= ca <= pad_range[1]
        if not held:
            raise SystemExit(f"--payload: held {o}, {geom_name(model, last)} pushes it off the pads")
        poses[o] = {"pos": centre, "axis": axis, "quat": quat, "pushed_out_m": t,
                    "pushed_by": last if t > 0 else None}
    return {"hand": hand, "holding": {j: float(q_hold[model.jnt_qposadr[j]]) for j in loose}, "slide": v,
            "gap_m": fb - fa, "gap_range_m": [min(g_lo, g_hi), max(g_lo, g_hi)], "s": s, "a": a, "c": c,
            "pads": pad_ids, "pad_range_m": pad_range, "poses": poses}


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
    model (every link but the root), root_model (the root link's geoms or
    stand-ins only), the arm and rover geom ids (the same in both), the part
    key of each rover geom (part_label, in part_ids order) and of each arm
    geom (arm_label, by name), and the name each part key shows (display)."""

    def __init__(self, **kw):
        self.__dict__.update(kw)


def _unique_name(taken: set, want: str) -> str:
    name, k = want, 1
    while name in taken:
        name, k = f"{want}_{k}", k + 1
    taken.add(name)
    return name


def body_list(value) -> list[str]:
    """--rover-body as a list: "a,b" or a sequence of names."""
    if isinstance(value, str):
        return [b.strip() for b in value.split(",") if b.strip()]
    return [str(b) for b in value]


def prepare(model_path: Path, *, arm_root: str = "shoulder_link", rover_body="rover",
            rover_parts=ROVER_PARTS, exclude=(), margin: float = MARGIN_M, lidar_xyz=None,
            joints=JOINTS, fixed=None, fingers: str = "swept", voxel: float = VOXEL_M,
            root_voxel: float = ROOT_VOXEL_M, root_margin: float = ROOT_MARGIN_M, cover_tol: float = COVER_TOL_M,
            visual: str = "count", names=None, keep_inside: bool = False, stand_ins=None,
            payload=None) -> Prepared:
    """Load the MJCF, add the lidar when asked and the model has none, the
    stand-ins of the rover's meshes and of the root link's (root_voxel > 0),
    the fingers' swept volumes, and set the contact masks so that
    mj_collision reports arm <-> rover-part contacts only, within `margin`.
    rover_body: one body or several ("a,b"). visual="skip" leaves out the
    geoms the model makes visual only (contype and conaffinity 0), except on
    a body that has no other geom to stand for them (no collision geom):
    there they count. names:
    {part id: name} (load_names). A rover geom held whole inside another one
    that counts as it is adds no forbidden pose (an arm geom near it is
    nearer the other): it is left out unless keep_inside. fingers="real"
    keeps the finger geoms as they are (the finger check). stand_ins: from
    an earlier prepare of the same model (skips the sampling). payload
    (parse_payload): a cylinder held in the gripper (grip_payload), once
    per orientation, on the fingers' body; payload_model sweeps those
    cylinders alone, the fingers at the gap that holds it."""
    import mujoco

    if visual not in ("count", "skip"):
        raise SystemExit(f"--visual takes count or skip, not {visual!r}")
    fixed = dict(fixed or {})
    names = dict(names or {})
    rover_names = body_list(rover_body)
    if not rover_names:
        raise SystemExit("--rover-body names no body")
    spec = mujoco.MjSpec.from_file(str(model_path))
    geometry = geometry_sha256(spec.compile())
    if spec.body(arm_root) is None:
        raise SystemExit(f"no body named {arm_root!r} in {model_path} (--arm-root)")
    for rb in rover_names:
        if spec.body(rb) is None:
            raise SystemExit(f"no body named {rb!r} in {model_path} (--rover-body)")
    rover = spec.body(rover_names[0])
    # a mesh collides as its whole convex hull: a cap on its vertices would
    # cut corners off the part
    hull_cap_lifted = 0
    for mesh in spec.meshes:
        if mesh.maxhullvert != -1:
            mesh.maxhullvert = -1
            hull_cap_lifted += 1
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
    # the masks below decide every contact; the parent filter would hide an
    # arm root hung straight on a rover body
    spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    spec.option.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    # MuJoCo's own convex collider (3.9) reports no contact for a geom held
    # whole inside a mesh's hull (a stand-in box inside an arm link, a small
    # part inside the gripper's swept volume); the libccd one does
    if hasattr(mujoco.mjtDisableBit, "mjDSBL_NATIVECCD"):
        spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    spec.option.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_OVERRIDE)
    # a pose deep in the rover makes thousands of contacts: room for them
    # (the arena is reserved, not touched, until a pose needs it)
    if spec.memory < ARENA_BYTES:
        spec.memory = ARENA_BYTES

    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    for name, value in fixed.items():
        jid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise SystemExit(f"--fix: no joint named {name!r}")
        d0.qpos[m0.jnt_qposadr[jid]] = float(value)
    mujoco.mj_kinematics(m0, d0)
    BODY = mujoco.mjtObj.mjOBJ_BODY
    root_bid = mujoco.mj_name2id(m0, BODY, arm_root)
    rover_bids = [mujoco.mj_name2id(m0, BODY, rb) for rb in rover_names]
    arm_bodies = _subtree(m0, root_bid)
    # the mount: the bodies between the arm root and the nearest listed rover body above it
    chain, b = [], int(m0.body_parentid[root_bid])
    while b not in rover_bids and b != 0:
        chain.append(b)
        b = int(m0.body_parentid[b])
    mount_bodies = chain if b in rover_bids else []
    owner: dict[int, int] = {}      # rover body -> the listed body whose frame its stand-ins use
    for rb in rover_bids:
        for b in _subtree(m0, rb):
            owner.setdefault(b, rb)
    rover_bodies = set(owner) - arm_bodies - set(mount_bodies)
    # the model's own exclusions between an arm body and a rover body stay
    # (an interface the model declares, such as a base and the shoulder that
    # turns on it); the others and every explicit pair go
    kept_excludes = []
    for elem in list(spec.excludes):
        b1, b2 = mujoco.mj_name2id(m0, BODY, elem.bodyname1), mujoco.mj_name2id(m0, BODY, elem.bodyname2)
        if (b1 in arm_bodies and b2 in rover_bodies) or (b2 in arm_bodies and b1 in rover_bodies):
            kept_excludes.append([elem.bodyname1, elem.bodyname2])
        else:
            spec.delete(elem)
    for elem in list(spec.pairs):
        spec.delete(elem)
    gname = {g: geom_name(m0, g) for g in range(m0.ngeom)}
    mesh_t = int(mujoco.mjtGeom.mjGEOM_MESH)
    prim_t = {int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_SPHERE), int(mujoco.mjtGeom.mjGEOM_CAPSULE),
              int(mujoco.mjtGeom.mjGEOM_CYLINDER), int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)}

    def visual_only(g: int) -> bool:
        return int(m0.geom_contype[g]) == 0 and int(m0.geom_conaffinity[g]) == 0

    # a body with collision geoms: they stand for its visual meshes
    collides = {int(m0.geom_bodyid[g]) for g in range(m0.ngeom) if not visual_only(g)}

    def is_visual(g: int) -> bool:
        return visual == "skip" and visual_only(g) and int(m0.geom_bodyid[g]) in collides

    # the visual meshes that count with --visual skip: nothing else stands for them
    visual_kept = [gname[g] for g in range(m0.ngeom)
                   if visual == "skip" and visual_only(g) and not is_visual(g)
                   and int(m0.geom_bodyid[g]) in arm_bodies | rover_bodies]

    # the rover: as they are, or through stand-ins
    as_is, meshes, unused = [], [], []
    visual_out = {"arm": 0, "rover": 0}
    for g in range(m0.ngeom):
        bid = int(m0.geom_bodyid[g])
        if bid in arm_bodies and is_visual(g):
            visual_out["arm"] += 1
        if bid not in rover_bodies:
            continue
        if is_visual(g):
            visual_out["rover"] += 1
        elif _matches(gname[g], exclude):
            unused.append(gname[g])
        elif _matches(gname[g], rover_parts) or int(m0.geom_type[g]) in prim_t:
            as_is.append(g)
        elif int(m0.geom_type[g]) == mesh_t:
            meshes.append(g)
        else:
            unused.append(gname[g])
    inside = {} if keep_inside else held_inside(m0, d0, as_is)
    if inside:
        as_is = [g for g in as_is if g not in inside]
    root_geoms = [g for g in range(m0.ngeom) if int(m0.geom_bodyid[g]) == root_bid and not is_visual(g)]
    root_meshes = [g for g in root_geoms if int(m0.geom_type[g]) == mesh_t] if root_voxel > 0 else []
    if stand_ins is None:
        stand_ins = {"voxel_m": voxel, "root_voxel_m": root_voxel, "cover_tol_m": cover_tol,
                     "per_mesh": {}, "boxes": [], "root": {}, "root_boxes": []}
        solids = [_Solid(m0, d0, g) for g in as_is] if meshes and voxel > 0 else []
        for g in meshes:
            if voxel > 0:
                frame = owner[int(m0.geom_bodyid[g])]
                boxes, rep = rover_mesh_stand_ins(m0, d0, g, solids, frame, voxel, cover_tol)
                stand_ins["per_mesh"][gname[g]] = rep
                fname = _name(m0, BODY, frame)
                stand_ins["boxes"].extend((gname[g], fname, c, h) for c, h in boxes)
            else:
                stand_ins["per_mesh"][gname[g]] = {"left_out": "--stand-in-voxel 0"}
        for g in root_meshes:
            local = mesh_surface_points(m0, g, root_voxel / 2.0)
            boxes = voxel_boxes(_to_body(d0, root_bid, _to_world(d0, g, local)), root_voxel)
            stand_ins["root"][gname[g]] = len(boxes)
            stand_ins["root_boxes"].extend((gname[g], c, h) for c, h in boxes)
    if voxel <= 0:
        unused += [gname[g] for g in meshes]
    label = {gname[g]: part_key(gname[g]) for g in as_is}
    for k, (src, fname, c, h) in enumerate(stand_ins["boxes"]):
        name = _unique_name(taken, f"{src}{STAND_IN_SEP}{k}")
        spec.body(fname).add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(v) for v in h],
                                  pos=[float(v) for v in c], group=3, rgba=[0, 0, 0, 0])
        label[name] = part_key(src)
    part_names = list(label)
    if not part_names:
        raise SystemExit(f"no rover part on {', '.join(rover_names)!r}")
    root_spec = spec.body(arm_root)
    root_names = [gname[g] for g in root_geoms if g not in root_meshes]
    arm_label = {n: part_key(n) for n in root_names}
    for k, (src, c, h) in enumerate(stand_ins["root_boxes"]):
        name = _unique_name(taken, f"{src}{STAND_IN_SEP}{k}")
        root_spec.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[float(v) for v in h],
                           pos=[float(v) for v in c], group=3, rgba=[0, 0, 0, 0])
        root_names.append(name)
        arm_label[name] = part_key(src)

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
    couplings = joint_couplings(m0)
    open_pose = open_fingers(m0, d0.qpos, loose, couplings)
    grip = None
    if payload:
        q_open = np.array(d0.qpos)
        for j, v in open_pose.items():
            q_open[m0.jnt_qposadr[j]] = v
        grip = grip_payload(m0, q_open, loose, couplings,
                            lambda g: int(m0.geom_bodyid[g]) in arm_bodies and not is_visual(g), payload)
    body_joints: dict[int, list[int]] = {}
    for j in range(m0.njnt):
        body_joints.setdefault(int(m0.jnt_bodyid[j]), []).append(j)
    link_names, replaced = [], []
    for g in range(m0.ngeom):
        bid = int(m0.geom_bodyid[g])
        if bid not in arm_bodies or bid == root_bid or is_visual(g):
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
            arm_label[gname[g]] = part_key(gname[g])
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
        swept = spec.add_mesh(name=mesh, uservert=np.concatenate(pts).ravel().tolist())
        swept.maxhullvert = -1
        spec.body(_name(m0, BODY, frame)).add_geom(
            name=mesh, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh, group=3, rgba=[0, 0, 0, 0])
        link_names.append(mesh)
        arm_label[mesh] = part_key(gname[g])
        replaced.append(gname[g])
    if not link_names and not root_names:
        raise SystemExit(f"no geom under {arm_root!r}: nothing to sweep")
    payload_names, record = {}, None
    if grip:
        hand = _name(m0, BODY, grip["hand"])
        for o, pose in grip["poses"].items():
            name = _unique_name(taken, f"payload_{o}")
            spec.body(hand).add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                                     size=[payload["diameter_m"] / 2.0, payload["height_m"] / 2.0, 0.0],
                                     pos=[float(v) for v in pose["pos"]], quat=pose["quat"], group=3,
                                     rgba=[0, 0, 0, 0])
            payload_names[o] = name
            arm_label[name] = payload_label(payload, o)

        def vec(v, nd=6):
            return [round(float(x), nd) + 0.0 for x in v]

        def part_of(g):
            return display_name(names, part_key(gname[g]))

        record = {"kind": payload["kind"], "diameter_m": payload["diameter_m"], "height_m": payload["height_m"],
                  "orientations": list(grip["poses"]), "hand": hand,
                  "fingers": {joint_name(m0, j): round(v, 6) for j, v in grip["holding"].items()},
                  "gap_m": round(grip["gap_m"], 6), "gap_range_m": vec(grip["gap_range_m"]),
                  "slide_axis": vec(grip["s"]), "approach_axis": vec(grip["a"]), "across_axis": vec(grip["c"]),
                  "pads": sorted({part_of(g) for g in grip["pads"]}), "pad_range_m": vec(grip["pad_range_m"]),
                  "pose": {o: {"pos_m": vec(q["pos"]), "axis": vec(q["axis"]), "quat": vec(q["quat"], 9),
                               "pushed_out_m": round(q["pushed_out_m"], 6),
                               "pushed_out_by": part_of(q["pushed_by"]) if q["pushed_by"] is not None else None}
                           for o, q in grip["poses"].items()},
                  "frame": f"the frame of the body {hand} (the fingers' body)",
                  "rule": "a cylinder held between the finger pads, swept once per orientation as an arm geom "
                          "(it meets the rover parts only), the fingers at the gap that holds it: upright = its "
                          "axis along the approach (where the fingers point: a can standing on the floor, picked "
                          "from above), across = its axis across both the slide and the approach (a can lying "
                          "down, picked from the side). Centred between the pads' inner faces and across them, "
                          "its centre at the pads' middle along the approach, pushed out along the approach "
                          "until it clears the hand's other geoms (pushed_out_m, pushed_out_by). The table "
                          "forbids a cell when the empty gripper (every jaw position) or the payload in either "
                          "orientation comes within margin_m of a rover part"}

    def compile_with(arm_on, part_margin):
        for g in spec.geoms:
            if g.name in arm_on:
                g.contype, g.conaffinity, g.margin, g.gap, g.condim = ARM_BIT, 0, 0.0, 0.0, 1
            elif g.name in label:
                g.contype, g.conaffinity, g.margin, g.gap, g.condim = 0, ARM_BIT, float(part_margin), 0.0, 1
            else:
                g.contype, g.conaffinity = 0, 0
        return spec.compile()

    # what the result needs from the first compiled model, read before it
    # goes: one compiled CAD scene holds some 350 MB
    jn = {j: joint_name(m0, j) for j in range(m0.njnt)}
    mount_names = [_name(m0, BODY, b) for b in mount_bodies]
    loose_ranges = {jn[j]: [float(v) for v in m0.jnt_range[j]] for j in loose}
    inside_geoms = {gname[g]: gname[o] for g, o in inside.items()}
    inside_parts = {display_name(names, part_key(gname[g])): display_name(names, part_key(gname[o]))
                    for g, o in inside.items()}
    holding = dict(grip["holding"]) if grip else {}
    m0 = d0 = d = None
    # the root model's parts carry the root margin itself: MuJoCo's box-box
    # contacts inside the margin band report no reliable distance, so the
    # root pass counts every contact MuJoCo makes rather than filtering on it
    model = compile_with(set(link_names), margin)
    root_model = compile_with(set(root_names), root_margin)
    payload_model = compile_with(set(payload_names.values()), margin) if payload_names else None
    gid = {geom_name(model, g): g for g in range(model.ngeom)}
    if any(geom_name(m, g) != n for m in (root_model, payload_model) if m is not None for n, g in gid.items()):
        raise SystemExit("the compiled models number their geoms differently")
    base_qpos = model.qpos0.copy()
    for n, v in fixed.items():
        base_qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
    for j, v in open_pose.items():
        base_qpos[model.jnt_qposadr[j]] = v
    payload_qpos = base_qpos.copy()
    for j, v in holding.items():
        payload_qpos[model.jnt_qposadr[j]] = v
    part_ids = [gid[n] for n in part_names]
    keys = set(label.values()) | set(arm_label.values())
    display = {k: display_name(names, k) for k in keys}
    if lidar.get("added"):
        lidar["overlaps_m"] = lidar_overlaps(model, base_qpos, gid[LIDAR_NAME], part_ids,
                                             {gid[n]: display[label[n]] for n in part_names})
    return Prepared(model=model, root_model=root_model, root_bid=root_bid,
                    arm_ids=[gid[n] for n in link_names], root_ids=[gid[n] for n in root_names],
                    part_ids=part_ids, part_label=[label[n] for n in part_names], arm_label=arm_label,
                    display=display, lidar=lidar, stand_ins=stand_ins, base_qpos=base_qpos,
                    mount_bodies=mount_names,
                    unused=unused, loose=loose_ranges,
                    open_pose={jn[j]: round(v, 9) for j, v in open_pose.items()},
                    couplings={jn[j1]: {"follows": jn[j2], "polycoef": c} for j1, (j2, c) in couplings.items()
                               if j1 in loose},
                    coupling_ids=couplings, fixed=fixed, replaced=replaced, geometry_sha256=geometry,
                    fingers=fingers, kept_excludes=kept_excludes, visual=visual, visual_out=visual_out,
                    hull_cap_lifted=hull_cap_lifted, rover_bodies=rover_names, visual_kept=visual_kept,
                    rover_counted=len(as_is), rover_stand_in_meshes=len(meshes) if voxel > 0 else 0,
                    inside=inside_geoms, inside_parts=inside_parts,
                    payload=record, payload_model=payload_model, payload_qpos=payload_qpos,
                    payload_ids={o: gid[n] for o, n in payload_names.items()})


def held_inside(model, data, geoms) -> dict[int, int]:
    """{geom: the geom it lies in} for the geoms held whole inside another of
    `geoms` as MuJoCo collides it (a mesh as its convex hull, a primitive as
    it is). The larger geom of two is the container; a container is never
    left out itself."""
    solids = {g: _Solid(model, data, g) for g in geoms}
    order = sorted((g for g in geoms if solids[g].lo is not None),
                   key=lambda g: -float(np.prod(solids[g].hi - solids[g].lo)))
    kept, out = [], {}
    for g in order:
        s = solids[g]
        pts = _to_world(data, g, hull_points_local(model, g))
        for o in kept:
            so = solids[o]
            if np.all(s.lo >= so.lo) and np.all(s.hi <= so.hi) and so.inside(pts, 0.0).all():
                out[g] = o
                break
        else:
            kept.append(g)
    return out


def lidar_overlaps(model, base_qpos, lidar_id: int, part_ids, part_name: dict) -> dict:
    """{rover part: signed distance (m)} for the parts the lidar cylinder goes
    into (distance < 0). part_name: {rover geom id: its part's name}."""
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
                name = part_name[g]
                out[name] = round(min(dist, out.get(name, dist)), 4)
    finally:
        model.opt.disableflags = flags
    return dict(sorted(out.items(), key=lambda kv: kv[1]))


def joint_grid(model, jid: int, steps: int, pad_deg: float, travel=None) -> tuple[np.ndarray, list[float]]:
    """The joint's grid: its model range widened by pad_deg on both sides (a
    hinge), and at least `travel` ([lo, hi] radians: the calibrated travel
    and its pad, calibrated_travel); and its model range."""
    import mujoco

    if model.jnt_limited[jid]:
        lo, hi = (float(v) for v in model.jnt_range[jid])
    else:
        lo, hi = -math.pi, math.pi
    hinge = int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_HINGE)
    pad = math.radians(pad_deg) if hinge else 0.0
    if steps < 2:
        raise SystemExit("every joint needs 2 steps or more")
    a, b = lo - pad, hi + pad
    if travel is not None and hinge:
        a, b = min(a, float(travel[0])), max(b, float(travel[1]))
    return np.linspace(a, b, steps), [lo, hi]


def calibrated_travel(path, joints, joint_map: dict, pad_deg: float = TRAVEL_PAD_DEG) -> dict:
    """{joint: {"ticks": [range_min, range_max], "rad": [lo, hi], "grid_rad":
    [lo - pad, hi + pad]}} from the arm's calibration (config/arm_calibration.json:
    each joint's range in reported ticks) through the joint_map
    ({joint: {"model_zero_ticks", "sign"}})."""
    try:
        doc = json.loads(Path(path).read_text())
    except (OSError, ValueError) as exc:
        raise SystemExit(f"--calibration {path}: {exc}") from None
    out = {}
    pad = math.radians(pad_deg)
    for j in joints:
        e = doc.get(j) if isinstance(doc, dict) else None
        if not isinstance(e, dict) or "range_min" not in e or "range_max" not in e:
            raise SystemExit(f"--calibration {path}: no range_min and range_max for {j}")
        m = joint_map.get(j, {})
        z, sign = float(m.get("model_zero_ticks", MODEL_ZERO_TICKS)), int(m.get("sign", 1))
        rad = sorted(sign * (float(e[k]) - z) * 2.0 * math.pi / TICKS_PER_REV for k in ("range_min", "range_max"))
        out[j] = {"ticks": [int(e["range_min"]), int(e["range_max"])], "rad": [round(v, 9) for v in rad],
                  "grid_rad": [round(rad[0] - pad, 9), round(rad[1] + pad, 9)]}
    return out


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
                 root_hits, split, lo: int, hi: int):
    """The flat cells lo..hi-1: (lo, occupancy, part numbers, pair counts,
    part counts, collision errors, split bits: per cell the OR of split[arm
    geom] over the arm geoms in contact, every bit after a collision error)."""
    import mujoco

    data = mujoco.MjData(model)
    is_arm, is_part = _masks(model, arm_ids, part_ids)
    err_no = max(part_no.values()) + 1
    occ = np.zeros(hi - lo, np.uint8)
    cell_part = np.zeros(hi - lo, np.uint16)
    split = split or {}
    grp = np.zeros(hi - lo, np.uint8)
    every_bit = 0
    for bit in split.values():
        every_bit |= bit
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
            grp[i] = every_bit
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
            for a, _, _ in hits:
                grp[i] |= split.get(a, 0)
    return lo, occ, cell_part, pair_cells, part_cells, errors, grp


_JOB = None     # what a forked worker sweeps (set by sweep() before the pool starts)


def _sweep_job(bounds):
    return _sweep_range(*_JOB, *bounds)


def sweep(model, joint_ids: list[int], grids: list[np.ndarray], base_qpos: np.ndarray,
          arm_ids: list[int], part_ids: list[int], margin: float, part_no=None, root_axes=(), root_hits=None,
          arm_no=None, progress=None, jobs: int = 1, split=None):
    """The grid, row-major (the last joint fastest). part_no: {rover geom: its
    part number, 1..} (a mesh's stand-ins share one), default one per geom
    in part_ids order; arm_no likewise for the arm geoms (pair counts).
    root_hits: the root link's contacts per index of its own joints
    (root_pass), added to every cell with that index. jobs > 1 splits the
    grid over forked processes (same result). split: {arm geom: a bit} (one
    payload orientation per bit). Returns occupancy (uint8, 1 = forbidden),
    the most-hit part per cell (0 = free, the part number, or the largest
    part number + 1 for a collision error), the cell counts per (arm number,
    part number) and per part number, the number of collision errors, and
    per cell the OR of the bits of the arm geoms in contact (uint8)."""
    import multiprocessing

    global _JOB
    part_no = part_no or {g: k + 1 for k, g in enumerate(part_ids)}
    total = int(np.prod([len(g) for g in grids]))
    occ = np.zeros(total, np.uint8)
    cell_part = np.zeros(total, np.uint16)
    grp = np.zeros(total, np.uint8)
    pair_cells: Counter = Counter()
    part_cells: Counter = Counter()
    errors = 0
    size = max(200, total // (max(1, jobs) * 8))
    chunks = [(lo, min(total, lo + size)) for lo in range(0, total, size)]
    job = (model, joint_ids, grids, base_qpos, arm_ids, part_ids, margin, part_no, arm_no or {},
           tuple(root_axes), root_hits, split)
    t0, done, shown = time.monotonic(), 0, 0

    def merge(res):
        nonlocal errors, done, shown
        lo, o, c, pc, partc, e, gb = res
        occ[lo:lo + len(o)] = o
        cell_part[lo:lo + len(o)] = c
        grp[lo:lo + len(o)] = gb
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
    return occ, cell_part, pair_cells, part_cells, errors, grp


def _pick(occ: np.ndarray, n: int, rng, free_only: bool = False) -> list[int]:
    pools = [np.flatnonzero(occ == 0)] if free_only else [np.flatnonzero(occ), np.flatnonzero(occ == 0)]
    pick = []
    for pool in pools:
        if len(pool):
            pick.extend(rng.choice(pool, size=min(len(pool), max(1, n // len(pools))), replace=False).tolist())
    return pick


def recheck(p: Prepared, joint_ids, grids, margin, occ, n: int, seed: int = 0,
            step_rad: float = math.radians(ROOT_STEP_DEG), with_payload: bool = True) -> list[int]:
    """Grid points re-done with a full mj_forward on fresh data, the root
    link included, and the payload in every orientation when p has one (and
    with_payload): the flat indices where the verdict differs from the table
    (half the sample from the forbidden cells, half from the free ones)."""
    import mujoco

    if n <= 0:
        return []
    rng = np.random.default_rng(seed)
    shape = tuple(len(g) for g in grids)
    axes, root_hits, _, _ = root_pass(p, joint_ids, grids, step_rad, full=True)
    is_arm, is_part = _masks(p.model, p.arm_ids, p.part_ids)
    pay = getattr(p, "payload_model", None) if with_payload else None
    if pay is not None:
        pay_arm, pay_part = _masks(pay, list(p.payload_ids.values()), p.part_ids)
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
        if not hit and pay is not None:
            data = mujoco.MjData(pay)
            data.qpos[:] = p.payload_qpos
            for j, g, i in zip(joint_ids, grids, idx):
                data.qpos[pay.jnt_qposadr[j]] = g[i]
            held = _touch(pay, data, pay_arm, pay_part, margin, full=True)
            hit = held is None or bool(held)
        if hit != bool(occ[flat]):
            bad.append(int(flat))
    return bad


def finger_check(real: Prepared, joint_ids, grids, margin, occ, n: int, seed: int = 1) -> tuple[int, list]:
    """Free grid points of the table, done again with the real finger geoms
    closed, half open, open and at a random opening (a finger a coupling ties
    to another follows it): the (flat index, finger values) where the arm
    touches the rover. The swept volumes hold every finger position, so the
    list must be empty."""
    import mujoco

    model = real.model
    if n <= 0 or not real.loose:
        return 0, []
    rng = np.random.default_rng(seed)
    couplings = getattr(real, "coupling_ids", {})
    loose = [(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j), lo, hi) for j, (lo, hi) in real.loose.items()]
    followers = {j for j, _, _ in loose if j in couplings and couplings[j][0] in {k for k, _, _ in loose}}
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
            apply_couplings(model, data.qpos, couplings, followers)
            hits = _touch(model, data, is_arm, is_part, margin)
            done += 1
            if hits:
                bad.append((int(flat), {joint_name(model, j): round(float(data.qpos[model.jnt_qposadr[j]]), 6)
                                        for j, _, _ in loose}))
    return done, bad


def keep_joint_map(out: Path, joints) -> tuple[dict, str]:
    """The joint_map of the table being replaced (the measured alignment) and
    its basis, else model_zero_ticks 2048 and sign +1 for every joint with
    the first alignment's basis (JOINT_MAP_BASIS)."""
    try:
        doc = json.loads(Path(out).read_text())
        old = doc.get("joint_map", {}) or {}
        basis = doc.get("joint_map_basis")
    except (OSError, ValueError, AttributeError):
        old, basis = {}, None
    jm = {}
    for j in joints:
        e = old.get(j, {}) if isinstance(old, dict) else {}
        e = e if isinstance(e, dict) else {}
        jm[j] = {"model_zero_ticks": e.get("model_zero_ticks", MODEL_ZERO_TICKS), "sign": e.get("sign", 1)}
    default = all(v == {"model_zero_ticks": MODEL_ZERO_TICKS, "sign": 1} for v in jm.values())
    if not isinstance(basis, str) or not basis:
        basis = JOINT_MAP_BASIS if default else "kept from the table this one replaced (no basis written there)"
    return jm, basis


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



MMAP_THRESHOLD = 128 << 10    # glibc serves allocations this large and larger by mmap: freed, they go back


def _lean_malloc() -> None:
    """One compiled CAD scene holds some 350 MB and its compilation more: with
    glibc serving every allocation of 128 KB or more by mmap, what a dropped
    model held goes back to the system instead of staying in the heap."""
    try:
        import ctypes
        ctypes.CDLL("libc.so.6").mallopt(-3, MMAP_THRESHOLD)       # M_MMAP_THRESHOLD
    except (OSError, AttributeError):
        pass


def _trim() -> None:
    """Hands the memory a dropped model freed back to the system: MuJoCo's
    asset cache emptied (a scene loads as fast without it), glibc's free
    pages returned."""
    try:
        import mujoco
        mujoco.mj_clearCache(mujoco.mj_getCache())
    except (ImportError, AttributeError, TypeError):
        pass
    try:
        import ctypes
        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except (OSError, AttributeError):
        pass


REUSE_KEYS = ("mujoco", "layout", "shape", "grid", "margin_m", "pad_deg", "arm_root", "rover_body", "rover_parts",
              "visual", "exclude", "excludes_kept", "lidar", "mount_parts")


def _reuse_view(doc: dict, n_arm: int) -> dict:
    """What two tables share when one reuses the other's empty-gripper sweep:
    the model by both hashes, the names, the grid, the margin, the rover's
    and the arm's settings, the fingers and the arm's parts."""
    view = {k: doc.get(k) for k in REUSE_KEYS}
    model = doc.get("model") or {}
    view["model"] = [model.get("sha256"), model.get("geometry_sha256")]
    view["names"] = (doc.get("names") or {}).get("sha256")
    view["root_link"] = {k: (doc.get("root_link") or {}).get(k) for k in ("margin_m", "step_deg", "voxel_m")}
    view["stand_ins"] = {k: (doc.get("stand_ins") or {}).get(k) for k in ("voxel_m", "cover_tol_m")}
    view["fingers"] = {k: (doc.get("fingers") or {}).get(k) for k in ("open", "fixed")}
    view["rover_inside"] = (doc.get("rover_inside") or {}).get("geoms")
    view["arm_geoms"] = list(doc.get("arm_geoms") or [])[:n_arm]
    return json.loads(json.dumps(view))      # as a JSON file reads back: lists, string keys


def reuse_empty(path: Path, view: dict, n_arm: int, key_no: dict, err_no: int, cells: int):
    """The empty gripper's sweep from an earlier table (--reuse-empty) that
    shares view with this run (_reuse_view): its occupancy
    (occupancy_empty_bits_hex in a table with a payload), the part of each
    forbidden cell, the pairs of the arm's own parts, the cells per part and
    its collision errors, numbered for this run; and its sweep's record."""
    path = Path(path)
    try:
        old = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise SystemExit(f"--reuse-empty {path}: {exc}") from None
    if not isinstance(old, dict) or old.get("format") != FORMAT:
        raise SystemExit(f"--reuse-empty {path}: not a keep-out table")
    theirs = _reuse_view(old, n_arm)
    differ = sorted(k for k in view if view[k] != theirs.get(k))
    if differ:
        raise SystemExit(f"--reuse-empty {path.name}: another sweep ({', '.join(differ)} differ)")
    pay = old.get("payload") or {}
    bits = old["occupancy_empty_bits_hex"] if pay else old["occupancy_bits_hex"]
    occ = np.unpackbits(np.frombuffer(bytes.fromhex(bits), np.uint8))[:cells].astype(np.uint8)
    keys = list(old.get("part_keys") or [])
    lut = np.zeros(len(keys) + 1, np.uint16)
    for i, k in enumerate(keys, start=1):
        if k != COLLISION_ERROR and k not in key_no:
            raise SystemExit(f"--reuse-empty {path.name}: part {k} is not a rover part of this model")
        lut[i] = err_no if k == COLLISION_ERROR else key_no[k]
    cell = np.asarray(old["cell_part"], np.int64)
    if len(occ) != cells or len(cell) != cells:
        raise SystemExit(f"--reuse-empty {path.name}: {len(occ)} cells, this grid has {cells}")
    cell_raw = np.where(occ > 0, lut[cell], 0).astype(np.uint16)
    if np.any((occ > 0) & (cell_raw == 0)):
        raise SystemExit(f"--reuse-empty {path.name}: a forbidden cell names no part")
    pair_cells: Counter = Counter()
    for a, k, c in old.get("pair_cells") or []:
        if int(a) < n_arm:
            pair_cells[(int(a), int(lut[int(k) + 1]))] += int(c)
    number = {name: int(lut[i]) for i, name in enumerate(old.get("part_names") or [], start=1)}
    part_cells: Counter = Counter()
    for name, c in (old.get("part_counts") or {}).items():
        if c:
            part_cells[number[name]] += int(c)
    errors = int(old.get("collision_errors", 0)) - int(pay.get("collision_errors", 0))
    src = old.get("empty_sweep") or old
    record = {k: src.get(k) for k in ("sweep_time_s", "sweep_jobs", "sweep_ms_per_pose")}
    record["reused_from"] = {"file": path.name, "sha256": sha256_of(path)}
    return occ, cell_raw, pair_cells, part_cells, errors, record


def run(args) -> int:
    import mujoco

    t_start = time.monotonic()
    model_path = Path(args.model)
    if not model_path.is_file():
        print(f"no model at {model_path}", file=sys.stderr)
        return 2
    layout = args.layout if args.layout != "auto" else layout_of(model_path)
    L = LAYOUTS[layout]
    arm_root = args.arm_root or L["arm_root"]
    rover_body = args.rover_body or L["rover_body"]
    rover_parts = args.rover_parts if args.rover_parts is not None else L["rover_parts"]
    visual = args.visual or L["visual"]
    root_voxel = args.root_voxel if args.root_voxel is not None else L["root_voxel"]
    if args.margin is None:
        args.margin = L["margin"]
    if not 0.0 <= args.root_margin <= args.margin:
        print("--root-margin goes from 0 to --margin", file=sys.stderr)
        return 2
    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    steps = [int(s) for s in args.steps.split(",")]
    if len(steps) != len(joints):
        print(f"--steps gives {len(steps)} numbers for {len(joints)} joints", file=sys.stderr)
        return 2
    parts = tuple(p.strip() for p in rover_parts.split(",") if p.strip())
    exclude = tuple(p.strip() for p in args.exclude.split(",") if p.strip())
    if args.no_lidar:
        lidar_xyz = None
    elif args.lidar_xyz:
        lidar_xyz = tuple(float(v) for v in args.lidar_xyz.split(","))
        if len(lidar_xyz) != 3:
            print("--lidar-xyz takes x,y,z in metres", file=sys.stderr)
            return 2
    elif L["lidar"]:
        lidar_xyz = profile_lidar_xyz()
    else:
        lidar_xyz = None
    payload = parse_payload(args.payload if args.payload is not None else L["payload"])
    names = load_names(args.names) if args.names else {}
    jm, basis = keep_joint_map(Path(args.out), joints)
    travel = calibrated_travel(args.calibration, joints, jm) if args.calibration else {}
    fixed = _parse_fixed(args.fix)
    jobs = args.jobs if args.jobs > 0 else min(MAX_JOBS, os.cpu_count() or 1)
    root_step = math.radians(args.root_step_deg)
    say = (lambda *a: None) if args.quiet else (lambda *a: print(*a, flush=True))
    common = dict(arm_root=arm_root, rover_body=rover_body, rover_parts=parts, exclude=exclude,
                  margin=args.margin, lidar_xyz=lidar_xyz, joints=joints, fixed=fixed,
                  voxel=args.stand_in_voxel, root_voxel=root_voxel, root_margin=args.root_margin,
                  cover_tol=args.cover_tol, visual=visual, names=names, keep_inside=args.keep_inside)
    p = prepare(model_path, **common, payload=payload)
    model = p.model
    lidar = p.lidar
    if lidar_xyz is not None and not lidar["added"]:
        lidar = {"added": False, "note": "the model has its own geom named lidar"}
    elif not lidar["added"]:
        held = sorted({p.display[k] for k in p.part_label if "lidar" in p.display[k].lower()})
        lidar = {"added": False, "note": ("the model carries the lidar as parts: " + ", ".join(held)) if held
                 else "no lidar added (--no-lidar, or a layout whose model carries it)"}
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
    grids, ranges = [], []
    for j, jid, n in zip(joints, joint_ids, steps):
        g, r = joint_grid(model, jid, n, args.pad_deg, travel[j]["grid_rad"] if j in travel else None)
        grids.append(g)
        ranges.append(r)
    keys = sorted(set(p.part_label))
    if len(keys) > np.iinfo(np.uint16).max - 1:
        print(f"{len(keys)} rover parts: more than cell_part can number", file=sys.stderr)
        return 2
    shape = tuple(steps)
    total = int(np.prod(shape))
    n_stand = len(p.stand_ins["boxes"])
    n_as_is = len(p.part_ids) - n_stand
    meshes_in = sum(1 for r in p.stand_ins["per_mesh"].values() if r.get("boxes"))
    arm_keys = sorted({p.arm_label[geom_name(model, g)] for g in p.arm_ids + p.root_ids})
    orients = list(p.payload["orientations"]) if p.payload else []
    pay_ids = [p.payload_ids[o] for o in orients]
    arm_names = [p.display[k] for k in arm_keys] + [p.arm_label[geom_name(model, g)] for g in pay_ids]
    say(f"model {model_label(model_path)} (layout {layout}): {len(p.arm_ids)} link geoms "
        f"({len(p.replaced)} finger geoms as swept volumes), root link {len(p.root_ids)} geoms "
        f"({len(p.stand_ins['root_boxes'])} stand-in boxes), {len(arm_keys)} arm parts; "
        f"{n_as_is} rover geoms as they are, {n_stand} stand-in boxes for {meshes_in} rover meshes, "
        f"{len(keys)} rover parts on {', '.join(p.rover_bodies)}; visual geoms {p.visual}"
        + (f" ({p.visual_out['arm']} on the arm and {p.visual_out['rover']} on the rover left out"
           + (f"; {', '.join(p.display.get(part_key(n), n) for n in p.visual_kept)} kept: no collision geom on their body"
              if p.visual_kept else "") + ")" if p.visual == "skip" else "")
        + f"; lidar {'added at ' + str(lidar.get('pos_m')) if lidar.get('added') else 'not added'}")
    if p.kept_excludes or p.hull_cap_lifted or p.inside:
        say(f"model exclusions kept: {', '.join('|'.join(e) for e in p.kept_excludes) or 'none'}; "
            f"hull vertex cap lifted on {p.hull_cap_lifted} meshes; {len(p.inside)} rover geoms held whole inside "
            f"another left out ({len(p.inside_parts)} parts)")
    if lidar.get("overlaps_m"):
        say("warning: the lidar goes into " + ", ".join(f"{k} ({1000 * v:.1f} mm)" for k, v in lidar["overlaps_m"].items())
            + ": give its measured centre with --lidar-xyz")
    if p.loose:
        say("fingers: swept volumes over " + ", ".join(f"{j} {lo:g}..{hi:g}" for j, (lo, hi) in p.loose.items())
            + "; open (the widest, the sweep's base pose): " + ", ".join(f"{j} {v:g}" for j, v in p.open_pose.items())
            + (" (" + ", ".join(f"{j} follows {c['follows']}" for j, c in p.couplings.items()) + ")"
               if p.couplings else ""))
    if p.payload:
        r = p.payload
        say(f"payload {r['kind']} ({1000 * r['diameter_m']:g} mm across, {1000 * r['height_m']:g} mm long) in "
            f"{r['hand']}, held at a {1000 * r['gap_m']:.1f} mm gap ("
            + ", ".join(f"{j} {v:g}" for j, v in r["fingers"].items()) + f") between {', '.join(r['pads'])}; "
            + "; ".join(f"{o}: centre {[round(1000 * v, 2) for v in q['pos_m']]} mm, axis {q['axis']}"
                        + (f", pushed out {1000 * q['pushed_out_m']:.2f} mm by {q['pushed_out_by']}"
                           if q["pushed_out_by"] else "") for o, q in r["pose"].items()))
    say(f"grid {'x'.join(map(str, shape))} = {total} poses, margin {args.margin} m, pad {args.pad_deg} deg, "
        f"{jobs} job(s){', fixed: ' + ', '.join(f'{k}={v:g}' for k, v in fixed.items()) if fixed else ''}")
    if travel:
        say(f"calibrated travel (+{TRAVEL_PAD_DEG:g} deg) covered: " + ", ".join(
            f"{j} {math.degrees(t['rad'][0]):.1f}..{math.degrees(t['rad'][1]):.1f} deg in "
            f"{math.degrees(g[0]):.1f}..{math.degrees(g[-1]):.1f}" for (j, t), g in zip(travel.items(), grids)))
    key_no = {k: i + 1 for i, k in enumerate(keys)}
    part_no = {g: key_no[k] for g, k in zip(p.part_ids, p.part_label)}
    label_name = {i: p.display[k] for k, i in key_no.items()}
    label_key = {i: k for k, i in key_no.items()}
    label_name[len(keys) + 1] = COLLISION_ERROR
    label_key[len(keys) + 1] = COLLISION_ERROR
    key_of = dict(zip(p.part_ids, p.part_label))
    arm_no = {g: arm_keys.index(p.arm_label[geom_name(model, g)]) for g in p.arm_ids + p.root_ids}

    root_axes, root_hits, ignore, root_samples = root_pass(p, joint_ids, grids, root_step)
    mount = sorted({p.display[key_of[r]] for _, r in ignore})
    if mount:
        say("the root link touches these at every value of its joints (those contacts are not counted): "
            + ", ".join(mount))
    per_mesh = dict(sorted(p.stand_ins["per_mesh"].items(), key=lambda kv: -kv[1].get("outside_fraction", 0)))
    stand_keys = {part_key(m) for m in per_mesh}
    # what the table says before any sweep: the model, the grid and the settings
    head = {
        "format": FORMAT,
        "made_by": "tools/moss_arm_keepout.py",
        "model": {"path": model_label(model_path), "sha256": sha256_of(model_path), "geometry_sha256": p.geometry_sha256},
        "mujoco": mujoco.__version__,
        "layout": layout,
        "names": ({"file": Path(args.names).name, "sha256": sha256_of(Path(args.names)), "parts": len(names)}
                  if args.names else None),
        "joints": joints,
        "shape": list(shape),
        "order": "row-major: the last joint varies fastest; flat = ravel_multi_index(indices, shape)",
        "grid": {j: [round(float(v), 9) for v in g] for j, g in zip(joints, grids)},
        "model_range": {j: [round(r[0], 9), round(r[1], 9)] for j, r in zip(joints, ranges)},
        "pad_deg": args.pad_deg,
        "travel": ({"pad_deg": TRAVEL_PAD_DEG, "joints": travel,
                    "rule": "each joint's calibrated travel (the arm's range_min and range_max, reported ticks) "
                            "through joint_map, in model radians; the grid reaches pad_deg past each end"}
                   if travel else None),
        "margin_m": args.margin,
        "margin_rule": "a pose is forbidden when an arm geom comes closer than margin_m to a rover part; "
                       "MuJoCo's geom margin is set on the rover parts only",
        "arm_root": arm_root,
        "rover_body": ",".join(p.rover_bodies),
        "rover_parts": list(parts),
        "visual": visual,
        "visual_left_out": p.visual_out if visual == "skip" else {"arm": 0, "rover": 0},
        "visual_kept": {"geoms": p.visual_kept,
                        "rule": "with visual skip, a body with no collision geom keeps its visual meshes: nothing "
                                "else stands for them"},
        "exclude": list(exclude),
        "excludes_kept": p.kept_excludes,
        "hull_vertex_cap_lifted": p.hull_cap_lifted,
        "mount_bodies": p.mount_bodies,
        "arm_geoms": arm_names,
        "rover_geoms": sorted({p.display[k] for k in set(p.part_label) - stand_keys}),
        "rover_parts_total": len(keys),
        "rover_geoms_counted": p.rover_counted,
        "rover_stand_in_meshes": p.rover_stand_in_meshes,
        "rover_geoms_rule": "every collision geom of the rover bodies (not the arm's, not its mount's) is one of: "
                            "rover_geoms_counted (as it is), rover_stand_in_meshes (through stand-ins), "
                            "rover_inside.geoms (held inside another), rover_left_out (by name)",
        "rover_left_out": p.unused,
        "rover_inside": {"geoms": len(p.inside), "parts": p.inside_parts,
                         "rule": "a rover geom held whole inside another (as MuJoCo collides it) adds no forbidden "
                                 "pose: an arm geom near it is nearer the other; left out (--keep-inside keeps it)"},
        "stand_ins": {"voxel_m": p.stand_ins["voxel_m"], "cover_tol_m": p.stand_ins["cover_tol_m"], "boxes": n_stand,
                      "rule": "per rover mesh: the share of its sampled surface lying more than cover_tol_m outside "
                              "the parts that count as they are, the extent of that share in the rover frame (m), "
                              "and the boxes standing in for it",
                      "per_mesh": per_mesh},
        "root_link": {"body": arm_root, "joints": [joints[k] for k in root_axes], "margin_m": args.root_margin,
                      "step_deg": args.root_step_deg, "samples": root_samples,
                      "voxel_m": p.stand_ins["root_voxel_m"], "stand_in_boxes": p.stand_ins["root"],
                      "rule": "its own joints sampled every step_deg, its meshes as surface stand-ins when voxel_m "
                              "is above 0 (else its geoms as they are); a contact closer than margin_m forbids "
                              "every grid value within half a grid step"},
        "mount_parts": mount,
        "mount_rule": "the root link touches these at every value of its joints: those contacts are not counted",
        "lidar": lidar,
        "fingers": {"joints": p.loose, "open": p.open_pose, "coupled": p.couplings, "fixed": p.fixed,
                    "rule": "every finger position: each geom a finger joint moves is replaced by its swept volume "
                            "(the convex hull of the geom at both ends of a slide, every 10 degrees of a hinge); "
                            "the sweep's base pose holds the fingers open: each at the end of its range that "
                            "spreads the gripper the most, a joint a model equality ties to another following it"},
        "unswept_joints": {"swept_volume": p.loose, "fixed": p.fixed, "replaced_geoms": p.replaced},
    }

    def progress(done, tot, dt):
        say(f"  {done}/{tot} poses, {dt:.0f} s, about {dt / done * (tot - done):.0f} s left")

    # the empty gripper: swept here, or taken from an earlier table of the same sweep
    t0 = time.monotonic()
    if args.reuse_empty:
        occ0, cell0, pairs0, parts0, err0, empty_rec = reuse_empty(
            Path(args.reuse_empty), _reuse_view(head, len(arm_keys)), len(arm_keys), key_no, len(keys) + 1, total)
        empty_s = 0.0
        say(f"empty gripper: {int(occ0.sum())} forbidden cells taken from {Path(args.reuse_empty).name} "
            "(the same model, grid and settings)")
    else:
        occ0, cell0, pairs0, parts0, err0, _ = sweep(model, joint_ids, grids, p.base_qpos, p.arm_ids, p.part_ids,
                                                     args.margin, part_no, root_axes, root_hits, arm_no,
                                                     progress=None if args.quiet else progress, jobs=jobs)
        empty_s = time.monotonic() - t0
        empty_rec = {"sweep_time_s": round(empty_s, 1), "sweep_jobs": jobs,
                     "sweep_ms_per_pose": round(1000.0 * empty_s * jobs / total, 3), "reused_from": None}
    # the payload: its cylinders alone (the arm at the holding gap lies inside the jaws' swept volumes)
    occ, cell_raw, pair_cells, part_cells, errors = occ0, cell0, Counter(pairs0), parts0, err0
    pay_s, pay_parts, pay_err, grp = 0.0, Counter(), 0, np.zeros(total, np.uint8)
    if pay_ids:
        say(f"payload sweep: the {payload['kind']} alone, held {' and '.join(orients)}")
        t1 = time.monotonic()
        occ_p, cell_p, pairs_p, pay_parts, pay_err, grp = sweep(
            p.payload_model, joint_ids, grids, p.payload_qpos, pay_ids, p.part_ids, args.margin, part_no,
            arm_no={g: len(arm_keys) + i for i, g in enumerate(pay_ids)},
            progress=None if args.quiet else progress, jobs=jobs, split={g: 1 << i for i, g in enumerate(pay_ids)})
        pay_s = time.monotonic() - t1
        occ = occ0 | occ_p
        cell_raw = np.where(occ0 > 0, cell0, cell_p).astype(np.uint16)
        pair_cells = pair_cells + pairs_p
        errors = err0 + pay_err
    sweep_s = empty_s + pay_s
    bad = recheck(p, joint_ids, grids, args.margin, occ0, args.check, step_rad=root_step, with_payload=False)
    if not bad and pay_ids:
        bad = recheck(p, joint_ids, grids, args.margin, occ, args.check, seed=2, step_rad=root_step)
    if bad:
        print(f"{len(bad)} grid point(s) differ between mj_kinematics+mj_collision and a full mj_forward "
              f"(first flat index {bad[0]}): table not written", file=sys.stderr)
        return 3
    p.model = p.root_model = p.payload_model = model = None      # the finger check compiles its own
    _trim()
    real = prepare(model_path, **common, fingers="real", stand_ins=p.stand_ins) if p.loose else None
    checked, touched = finger_check(real, joint_ids, grids, args.margin, occ, args.check) if real else (0, [])
    real = None
    _trim()
    if touched:
        print(f"{len(touched)} free grid point(s) touch the rover with the real fingers "
              f"(first: flat {touched[0][0]}, fingers {touched[0][1]}): table not written", file=sys.stderr)
        return 3

    forbidden = int(occ.sum())
    # the parts hit, numbered 1.. from the most frequent most-hit part down, so
    # the common ones take one digit in cell_part; the parts never hit are not listed
    top = Counter(int(v) for v in cell_raw[occ.astype(bool)])
    order = sorted(set(top) | set(part_cells) | set(pay_parts),
                   key=lambda k: (-top.get(k, 0), -part_cells.get(k, 0), -pay_parts.get(k, 0), label_name[k]))
    lut = np.zeros(len(keys) + 2, np.uint16)
    for new_no, k in enumerate(order, start=1):
        lut[k] = new_no
    cell_part = lut[cell_raw]
    # one byte per cell while the table names 255 parts or fewer
    dtype = np.uint8 if len(order) <= MAX_PARTS_U8 else np.uint16
    cell_part = cell_part.astype(dtype)
    pairs = sorted(((arm_names[a], label_name[k], c) for (a, k), c in pair_cells.items()),
                   key=lambda t: (-t[2], t[0], t[1]))
    pair_rows = sorted(([a, int(lut[k]) - 1, c] for (a, k), c in pair_cells.items()), key=lambda t: (-t[2], t[0], t[1]))
    record = None
    if pay_ids:
        empty_b = occ0.astype(bool)
        bits = {o: ((grp >> i) & 1).astype(bool) for i, o in enumerate(orients)}
        record = dict(p.payload)
        record["forbidden"] = {"empty": int(empty_b.sum()), **{o: int(b.sum()) for o, b in bits.items()},
                               "union": forbidden}
        record["added"] = {**{o: int((b & ~empty_b).sum()) for o, b in bits.items()},
                           "total": forbidden - int(empty_b.sum())}
        record["part_counts"] = {label_name[k]: pay_parts[k] for k in order if pay_parts.get(k)}
        record["collision_errors"] = pay_err
        record["sweep_time_s"] = round(pay_s, 1)
        record["sweep_ms_per_pose"] = round(1000.0 * pay_s * jobs / total, 3)
        record["counts_rule"] = ("forbidden: the cells where the empty gripper, the payload held each way, and any "
                                 "of them (union, the table) come within margin_m of a rover part; added: those of "
                                 "them the empty gripper leaves free; part_counts: the cells where the payload, "
                                 "either way, touches that part")
    run_s = time.monotonic() - t_start
    doc = dict(head)
    doc.update({
        "payload": record,
        "empty_sweep": empty_rec,
        "collision_errors": errors,
        "cells": total,
        "forbidden": forbidden,
        "forbidden_fraction": round(forbidden / total, 6),
        "part_names": [label_name[k] for k in order],
        "part_keys": [label_key[k] for k in order],
        "part_counts": {label_name[k]: part_cells[k] for k in order if part_cells.get(k)},
        "part_counts_rule": "cells where the arm, the gripper empty, touches this part (a cell counts for every "
                            "part it touches); payload.part_counts the same for the payload",
        "part_top_counts": {label_name[k]: top.get(k, 0) for k in order},
        "pair_counts": {f"{a}|{r}": c for a, r, c in pairs[:TOP_PAIRS]},
        "pair_counts_rule": f"cells where this arm part and this rover part touch: the {TOP_PAIRS} most frequent "
                            "pairs; pair_cells holds every pair as [i, k, cells] for arm_geoms[i] and part_names[k]",
        "pairs": len(pairs),
        "run_time_s": round(run_s, 1),
        "sweep_time_s": round(sweep_s, 1),
        "sweep_jobs": jobs,
        "sweep_ms_per_pose": empty_rec.get("sweep_ms_per_pose"),
        "rechecked_with_mj_forward": args.check,
        "finger_check": {"poses": checked, "touching": len(touched)},
        "ticks_per_rev": TICKS_PER_REV,
        "joint_map": jm,
        "joint_map_basis": basis,
        "joint_map_rule": "rad = sign * (ticks - model_zero_ticks) * 2 pi / ticks_per_rev; "
                          "ticks as the servo reports them (homing offset applied); align with two poses (docs/arm.md)",
        "occupancy_rule": "occupancy: one uint8 per cell, 1 = forbidden, flat in row-major order, packed 8 cells "
                          "per byte (numpy packbits, first cell in the high bit), written in hex; with a payload, "
                          "occupancy_empty_bits_hex holds the empty gripper's alone, the same way",
        "cell_part_rule": "0 = free, k = part_names[k - 1]: the rover part with the most contacts in that cell "
                          "(the empty gripper's where it forbids the cell, else the payload's)",
        "cell_part_dtype": np.dtype(dtype).name,
        "occupancy_bits_hex": np.packbits(occ).tobytes().hex(),
    })
    if record is not None:
        doc["occupancy_empty_bits_hex"] = np.packbits(occ0).tobytes().hex()
    size = write_table(Path(args.out), doc, {"pair_cells": pair_rows, "cell_part": cell_part.tolist()})

    say(f"{total} cells, {forbidden} forbidden ({100.0 * forbidden / total:.1f} %), "
        + (f"empty gripper taken from {empty_rec['reused_from']['file']}" if empty_rec["reused_from"]
           else f"sweep {empty_s:.1f} s ({1000.0 * empty_s * jobs / total:.2f} ms per pose and job)")
        + (f", payload sweep {pay_s:.1f} s" if pay_ids else "")
        + f", total {run_s:.1f} s, {size / 1e6:.2f} MB -> {args.out}")
    if record is not None:
        f, a = record["forbidden"], record["added"]
        say(f"payload: empty gripper {f['empty']} cells, " + ", ".join(
            f"{o} {f[o]} ({a[o]} the empty gripper leaves free)" for o in orients) + f", union {f['union']}")
    if errors:
        say(f"warning: {errors} pose(s) where MuJoCo's collision failed, marked forbidden")
    n_hit = sum(1 for k in order if k <= len(keys))
    say(f"rover parts hit: {n_hit} of {len(keys)}{' (and the collision errors)' if errors else ''}, "
        f"cell_part {np.dtype(dtype).name}")
    say("rover parts hit (cells): " + ", ".join(f"{label_name[k]} {part_cells.get(k, 0) or pay_parts.get(k, 0)}"
                                               for k in order[:12]))
    say("worst pairs (cells): " + ", ".join(f"{a}|{r} {c}" for a, r, c in pairs[:8]))
    if checked:
        say(f"finger check: {checked} poses of free grid points with the real fingers, none touches")
    return 0


def main(argv=None) -> int:
    _lean_malloc()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--model", required=True, help="the robot's MJCF")
    ap.add_argument("--out", default=str(DEFAULT_OUT), help="the table (default config/arm_keepout.json)")
    ap.add_argument("--layout", default="auto", choices=("auto", "cad", "sim"),
                    help="the defaults below: cad for the CAD scene (a body rover_exterior), sim otherwise")
    ap.add_argument("--names", default="", help="a parts manifest: {\"parts\": [{\"id\", \"name\"}]} names the parts")
    ap.add_argument("--steps", default=",".join(map(str, STEPS)), help="grid points per joint, in --joints order")
    ap.add_argument("--margin", type=float, default=None,
                    help="metres: closer than this is a contact (layout: 0.01; 0.002)")
    ap.add_argument("--pad-deg", type=float, default=PAD_DEG, help="each hinge range widened by this on both sides")
    ap.add_argument("--calibration", default="",
                    help="the arm's calibration (config/arm_calibration.json): the grid covers each joint's "
                         f"calibrated travel through the joint_map, plus {TRAVEL_PAD_DEG:g} degrees")
    ap.add_argument("--joints", default=",".join(JOINTS))
    ap.add_argument("--arm-root", default=None, help="the first moving body of the arm (layout: shoulder_link, shoulder)")
    ap.add_argument("--rover-body", default=None,
                    help="the rover's bodies, comma separated (layout: rover; rover_exterior,base)")
    ap.add_argument("--rover-parts", default=None,
                    help="geom name patterns (fnmatch) that count as they are, meshes included "
                         "(layout: hull,bin_*,track_*,tread_*,lidar; *)")
    ap.add_argument("--visual", default=None, choices=("count", "skip"),
                    help="the geoms the model makes visual only (contype and conaffinity 0): parts, or left out "
                         "(layout: count; skip)")
    ap.add_argument("--exclude", default="", help="geom name patterns (fnmatch) that are not rover parts")
    ap.add_argument("--keep-inside", action="store_true",
                    help="keep the rover geoms held whole inside another one (they add no forbidden pose)")
    ap.add_argument("--stand-in-voxel", type=float, default=VOXEL_M,
                    help="metres: the box grid of the rover meshes' stand-ins (0: those meshes are left out)")
    ap.add_argument("--root-voxel", type=float, default=None,
                    help="metres: the box grid of the root link's stand-ins, 0: its geoms as they are "
                         "(layout: 0.001; 0)")
    ap.add_argument("--root-margin", type=float, default=ROOT_MARGIN_M,
                    help="metres: the root link's contacts count closer than this (it turns in place; at most --margin)")
    ap.add_argument("--root-step-deg", type=float, default=ROOT_STEP_DEG,
                    help="degrees between the root link's samples along its own joints")
    ap.add_argument("--cover-tol", type=float, default=COVER_TOL_M,
                    help="metres: a mesh surface this close to a part that counts as it is needs no stand-in")
    ap.add_argument("--lidar-xyz", default="", help="x,y,z of the lidar's centre on the first rover body (m)")
    ap.add_argument("--no-lidar", action="store_true", help="do not add the lidar")
    ap.add_argument("--fix", default="", help="name=value,... pins unswept arm joints (else swept as volumes)")
    ap.add_argument("--payload", default=None,
                    help="what the gripper holds, swept upright and across: none, can (66 x 120 mm) or "
                         "cyl:<diameter mm>x<height mm> (layout: none; can)")
    ap.add_argument("--reuse-empty", default="",
                    help="an earlier table of the same model, grid and settings: its empty-gripper sweep is taken "
                         "as it is and only the payload is swept")
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
    if (args.root_voxel is not None and args.root_voxel < 0) or args.root_step_deg <= 0:
        print("--root-voxel must be 0 or more and --root-step-deg positive", file=sys.stderr)
        return 2
    try:
        return run(args)
    finally:
        import gc
        gc.collect()
        _trim()                         # the run's models are gone: hand their memory back


if __name__ == "__main__":
    sys.exit(main())
