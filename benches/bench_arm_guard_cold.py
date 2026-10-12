"""Cold bench for the arm's keep-out: tools/moss_arm_keepout.py (the sweep, on
MuJoCo) and moss_dimos/arm_guard.py (the guard, numpy only).

Known input -> known output, in radians and metres:

  A. the sweep on toys
     - a base with a fixed box "wall" (0.29..0.33 m on +x) and a two-joint
       planar arm (links 0.20 + 0.10 m, a 0.01 m ball at the tip): the table
       matches the analytic verdict (tip within the 0.005 m margin) at every
       grid point; the band at j2 = 0 ends within one grid step of the
       analytic edge; the MuJoCo margin sits on the rover parts only
     - every rover geom counts: a box "decor" that --rover-parts does not
       name is a rover part unless --exclude names it
     - a rover mesh "cam" (an OBJ box) is covered by stand-ins: every grid
       point closer than the margin is forbidden, none further than the
       margin plus the stand-ins' reach; --stand-in-voxel 0 misses it; a
       mesh inside the wall needs no box; editing the mesh file changes the
       geometry hash and not the top file's
     - the lidar cylinder is added where asked and counted; a model with its
       own "lidar" keeps it; the lidar's overlap with a part is reported
     - the root link: a "bar" under its hub is touched at every j1 (mount:
       those contacts are left out, the tip's are counted); a "horn" on the
       root link hits a "stop" for a band of j1, which forbids those j1
       values whole and the values within half a grid step
     - a slide finger whose pad passes a post only between its two ends:
       the swept volume forbids it, the open-only and closed-only sweeps miss
       it, the real-finger check passes
     - a payload: a two-finger hand at the tip holds a cylinder 40 mm
       across and 100 mm long (--payload cyl:40x100), a wall 0.6 m wide
       ahead (the reach decides, not its ends): the fingers close to
       10 mm each (40 mm apart), the cylinder held upright is pushed out of
       the palm to 48 mm, held across it sits at the pads' middle (20 mm);
       the table matches the analytic union (the empty hand OR the cylinder
       either way within 5 mm of the wall) at every grid point, --payload
       none matches the empty hand alone, and the band at j2 = 0 is wider
       on both ends with the cylinder; a cylinder wider than the hand
       opens and a hand with no slide fingers are refused
     - a MuJoCo collision error (mujoco.FatalError) marks its cell forbidden
       and is counted, and why() says so
     - a re-sweep keeps the joint_map; the table records both hashes, the
       margin and the run time
  B. the guard on the wall table - inside the band refused, one grid step
     outside allowed, a pose whose interpolation touches the band refused,
     outside the table refused; nearest_allowed is the nearest pose that
     stays allowed once rounded to ticks (checked against a brute force),
     nearest_allowed_ticks too; segment_allowed refuses a move that cuts a
     forbidden cell's corner between quarter-step samples, and never allows
     a move that a fine sampling refuses
  C. ticks <-> radians - 3072 ticks = +90 deg (zero 2048, sign +1), sign -1,
     homing offsets on raw readings, round trips, two-pose alignment of one
     joint and of all five (POSE_A / POSE_B, the elbow negative)
  D. the real model (MOSS_ARM_MODEL, else ~/moss-*/live/model/moss.xml): a
     coarse sweep (8,8,8,6,4) finds forbidden cells, the hull or the bin
     among the parts hit, the lidar added at the profile's height; the arm
     straight up is free and allowed; the robot model (moss_robot.xml next
     to it) at --margin 0.03 finishes; --payload can on it holds the can
     between its two pads at 66 mm (29 mm of slide each)
  E. the CAD scene (MOSS_ARM_CAD, else ~/moss-*/cad/*/MOSS_R31.xml, with its
     parts_manifest.json): a CAD piece answers to its part
     (col_part_0996_017 -> col_part_0996), --names gives col_part_0996 its
     CAD name, the layout is told by the body names; a coarse sweep
     (8,8,8,6,4) finds forbidden cells with the lidar itself (RPLIDAR C1),
     its carriage, the bin, the upper cover, the arm's own base and the arm
     mounting plate among the parts hit, adds no lidar cylinder, leaves the
     visual meshes out but the gripper drive's (no collision geom there),
     keeps the base|shoulder exclusion, has margin 2 mm, sweeps the jaws
     between closed and open (open: both slides at 0, the pinion at 0),
     lists only the parts hit, carries the joint_map with its basis and both
     sha256, accounts for every rover collision geom of the model (counted
     apart from the tool), carries no CAD working code in its names, covers
     the arm's calibrated travel (when a calibration is here); the 2 mm
     margin forbids cells a 0 mm margin leaves free; every hull is whole
     (the scene's 128-vertex cap lifted); the geoms held inside another
     change no verdict (--keep-inside); drive and look at their exact angles
     stay clear of the rover by the margin, with the can held either way
     too. The payload (a can by default here): two cylinders 66 mm across
     and 120 mm long on nc90_tool, the jaws at 12 mm each, both TPU pads
     within 1 mm of the can's surface (checked on the scene's own pad
     meshes, apart from the tool), upright pushed out of the gripper's
     rails, across at the pads; --payload none gives the empty gripper's
     table, the same cells the can's table records as its empty sweep, and
     the can forbids more; a pose with the gripper over the bin's rim is
     clear of the rover empty and goes into the bin with the can upright;
     three postures of an independent clearance study (travel, pick,
     clear: the pan turned from +110 deg down) touch where it found: the
     left jaw or its pad against the bin near -46 deg, the can before the
     arm against the left road-wheel shaft near -45, nothing
  F. the committed table (config/arm_keepout.json) - it loads, 16x16x16x12x8,
     under 2 MB; from the CAD scene: drive and look are allowed, the parts
     carry their CAD names, margin 2 mm, the lidar itself among the parts
     hit, the model by its file name, no CAD working code; the arm's
     calibrated travel inside the table (when a calibration is here:
     config/arm_calibration.json, MOSS_ARM_CALIBRATION, or
     ~/moss-*/docs/arm_calibration_*.json); when the model it names is here
     (next to D's or E's): both hashes match, 200 cells re-done with
     mj_forward agree (the can held either way included, and the empty
     gripper's own cells alone), free cells stay free with the real fingers,
     the payload record (a can, both orientations, the empty gripper's cells
     inside the table's), the pose over the bin's rim free for the empty
     gripper and refused with the can (why() names the bin), the study's
     postures (clear allowed at every pan, travel and pick refused where it
     found contacts), every
     rover geom counts (the CAD scene: every collision geom of the model
     accounted for; the simulation model: every rover mesh the named parts
     miss has stand-ins, the camera poses a review found are refused),
     nearest_allowed survives the tick rounding and segment_allowed agrees
     with a fine sampling

Needs mujoco (the sweep). Without it: one line "skipped: mujoco not installed",
exit 0. Every sweep here runs in one process (--jobs 1).

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_arm_guard_cold.py
"""
import atexit
import glob
import itertools
import json
import math
import os
import re
import sys
import tempfile
from pathlib import Path

try:
    import mujoco
except ImportError:
    print("skipped: mujoco not installed")
    sys.exit(0)

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import moss_arm_keepout as K  # noqa: E402
K._lean_malloc()     # the CAD scene is loaded a dozen times here: what each load frees goes back to the system
from moss_dimos import robot_profile  # noqa: E402
from moss_dimos.arm_guard import POSE_A, POSE_B, ArmGuard, JointMap, align_from_two_poses, align_joint_map  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


_TMP = tempfile.TemporaryDirectory(prefix="arm_keepout_")
atexit.register(_TMP.cleanup)
TMP = Path(_TMP.name)
NOWHERE = TMP / "no_calibration.json"

# ── A. the sweep on toys ──
L1, L2, TIP_R = 0.20, 0.10, 0.01
WALL_C, WALL_H = np.array([0.31, 0.0, 0.0]), np.array([0.02, 0.03, 0.05])
DECOR_AT = (0.3 * math.cos(-1.0), 0.3 * math.sin(-1.0), 0.0)
CAM_C, CAM_H = np.array([0.3 * math.cos(0.6), 0.3 * math.sin(0.6), 0.0]), np.array([0.008, 0.02, 0.02])
MARGIN = 0.005
LIDAR_AT = (0.3 * math.cos(1.0), 0.3 * math.sin(1.0), 0.0)
J1 = (-1.5, 1.5, 61)    # step 0.05 rad
J2 = (-0.5, 0.5, 11)    # step 0.1 rad, 0 on the grid
BASE = ["--joints", "j1,j2", "--steps", f"{J1[2]},{J2[2]}", "--arm-root", "link1", "--rover-body", "base",
        "--margin", str(MARGIN), "--pad-deg", "0", "--quiet", "--jobs", "1"]


def box_obj(path: Path, half) -> None:
    """An OBJ box centred on its origin."""
    hx, hy, hz = half
    v = [(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    lines = [f"v {x} {y} {z}" for x, y, z in v]
    for a, b, c, d in quads:
        lines += [f"f {a + 1} {b + 1} {c + 1}", f"f {a + 1} {c + 1} {d + 1}"]
    path.write_text("\n".join(lines) + "\n")


def toy_xml(extra_base="", extra_link1="", assets=""):
    return f"""<mujoco model="toy">
  <compiler angle="radian" autolimits="true" meshdir="{TMP}"/>
  <asset>{assets}</asset>
  <worldbody>
    <body name="base">
      <geom name="wall" type="box" pos="{WALL_C[0]} {WALL_C[1]} {WALL_C[2]}" size="{WALL_H[0]} {WALL_H[1]} {WALL_H[2]}"/>
      <geom name="decor" type="box" pos="{DECOR_AT[0]} {DECOR_AT[1]} 0" size="0.02 0.02 0.02"/>
      {extra_base}
      <body name="link1">
        <joint name="j1" type="hinge" axis="0 0 1" range="{J1[0]} {J1[1]}"/>
        <inertial pos="0.1 0 0" mass="0.05" diaginertia="1e-4 1e-4 1e-4"/>
        {extra_link1}
        <body name="link2" pos="{L1} 0 0">
          <joint name="j2" type="hinge" axis="0 0 1" range="{J2[0]} {J2[1]}"/>
          <geom name="tip" type="sphere" pos="{L2} 0 0" size="{TIP_R}"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def tip(j1, j2):
    return np.array([L1 * math.cos(j1) + L2 * math.cos(j1 + j2), L1 * math.sin(j1) + L2 * math.sin(j1 + j2), 0.0])


def box_clearance(p, c, h, r):
    """Analytic: ball (centre p, radius r) to an axis-aligned box surface; <= 0 inside."""
    return float(np.linalg.norm(np.maximum(np.abs(np.asarray(p) - c) - h, 0.0))) - r


def wall_clearance(j1, j2):
    return box_clearance(tip(j1, j2), WALL_C, WALL_H, TIP_R)


def decor_clearance(j1, j2):
    return box_clearance(tip(j1, j2), np.array(DECOR_AT), np.array([0.02, 0.02, 0.02]), TIP_R)


def cam_clearance(j1, j2):
    return box_clearance(tip(j1, j2), CAM_C, CAM_H, TIP_R)


def lidar_clearance(j1, j2):
    p = tip(j1, j2)
    radial = max(math.hypot(p[0] - LIDAR_AT[0], p[1] - LIDAR_AT[1]) - K.LIDAR_RADIUS_M, 0.0)
    vertical = max(abs(p[2] - LIDAR_AT[2]) - K.LIDAR_HEIGHT_M / 2.0, 0.0)
    return math.hypot(radial, vertical) - TIP_R


def sweep_toy(name, args, xml=None, base=BASE):
    model = TMP / f"{name}.xml"
    model.write_text(xml or toy_xml())
    out = TMP / f"{name}.json"
    try:
        rc = K.main(["--model", str(model), "--out", str(out)] + base + args)
    except SystemExit as exc:      # the tool's refusals (a missing body, no rover part)
        rc = exc.code
    return rc, out, (json.loads(out.read_text()) if out.exists() else {})


def truth(table, clearances, slack=0.0):
    """{flat: analytic forbidden} and the flats on the boundary (clearance
    within 0.1 mm of the margin, or between the margin and margin + slack:
    either verdict is right there)."""
    g1, g2 = table["grid"]["j1"], table["grid"]["j2"]
    want, edge = {}, set()
    for a, v1 in enumerate(g1):
        for b, v2 in enumerate(g2):
            c = min(f(v1, v2) for f in clearances)
            flat = a * len(g2) + b
            want[flat] = c < MARGIN
            if abs(c - MARGIN) < 1e-4 or MARGIN <= c < MARGIN + slack:
                edge.add(flat)
    return want, edge


def occ_of(table):
    bits = np.unpackbits(np.frombuffer(bytes.fromhex(table["occupancy_bits_hex"]), np.uint8))
    return bits[:table["cells"]].astype(bool)


def differ(table, clearances, slack=0.0):
    occ = occ_of(table)
    want, edge = truth(table, clearances, slack)
    return [f for f, w in want.items() if f not in edge and bool(occ[f]) != w], occ, want, edge


print("A. the sweep on toys")
rc, out1, t1 = sweep_toy("toy", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor"])
check("the tool exits 0, its re-checks with a full mj_forward agree", rc == 0, f"rc {rc}")
check("shape 61 x 11, j1 grid -1.5 .. 1.5 rad in 0.05 steps, j2 holds 0",
      t1.get("shape") == [61, 11] and abs(t1["grid"]["j1"][0] + 1.5) < 1e-9 and abs(t1["grid"]["j1"][1] - t1["grid"]["j1"][0] - 0.05) < 1e-9
      and abs(t1["grid"]["j2"][5]) < 1e-12)
bad, occ1, want1, edge1 = differ(t1, [wall_clearance])
check("every grid point matches the analytic verdict (tip within 5 mm of the wall)",
      not bad and sum(want1.values()) > 0, f"{int(occ1.sum())} forbidden, {sum(want1.values())} expected, "
      f"{len(bad)} differ, {len(edge1)} on the boundary")
g1 = np.array(t1["grid"]["j1"])
row = occ1.reshape(61, 11)[:, 5]
ks = np.flatnonzero(row)
lo_k, hi_k = int(ks[0]), int(ks[-1])
a, b = 0.0, 1.0
for _ in range(60):            # the analytic band edge at j2 = 0, by bisection
    m = (a + b) / 2
    a, b = (m, b) if wall_clearance(m, 0.0) < MARGIN else (a, m)
edge = (a + b) / 2
check(f"at j2 = 0 the band is one run, its ends within one step of +-{edge:.4f} rad (analytic)",
      len(ks) == hi_k - lo_k + 1 and g1[hi_k] <= edge < g1[hi_k] + 0.05 + 1e-9 and g1[lo_k] >= -edge > g1[lo_k] - 0.05 - 1e-9,
      f"forbidden j1 {g1[lo_k]:+.2f} .. {g1[hi_k]:+.2f}")
check("--exclude decor: the decor box at j1 = -1 is free, parts = [wall], decor listed as left out",
      not occ1.reshape(61, 11)[g1 < -0.5].any() and t1.get("part_names") == ["wall"]
      and t1.get("part_counts") == {"wall": int(occ1.sum())} and t1.get("rover_left_out") == ["decor"])
check("cell_part names the wall wherever forbidden, 0 elsewhere",
      np.array_equal(np.array(t1["cell_part"]) > 0, occ1) and set(np.array(t1["cell_part"])[occ1]) == {1})
check("--no-lidar: no lidar added", t1.get("lidar", {}).get("added") is False)
pm = K.prepare(TMP / "toy.xml", arm_root="link1", rover_body="base", rover_parts=("wall",), margin=MARGIN,
               joints=("j1", "j2"))
gm = {K.geom_name(pm.model, g): g for g in range(pm.model.ngeom)}
dm = mujoco.MjData(pm.model)
dm.qpos[:] = [0.6, 0.0]
mujoco.mj_forward(pm.model, dm)
inc = {round(float(c.includemargin), 6) for c in dm.contact[:dm.ncon]}
check("the MuJoCo margin sits on the rover parts only: tip 0, wall and decor 0.005, a contact's includemargin 0.005",
      pm.model.geom_margin[gm["tip"]] == 0.0 and pm.model.geom_margin[gm["wall"]] == MARGIN
      and pm.model.geom_margin[gm["decor"]] == MARGIN and inc <= {MARGIN}, f"includemargin {sorted(inc)}")

rc, _, t_dec = sweep_toy("toy_decor", ["--no-lidar", "--rover-parts", "wall"])
bad_d, _, _, _ = differ(t_dec, [wall_clearance, decor_clearance])
check("without --exclude, decor counts though --rover-parts does not name it (wall OR decor, analytic)",
      rc == 0 and not bad_d and set(t_dec.get("part_names", [])) == {"wall", "decor"}, f"{len(bad_d)} differ")
u8_max = K.MAX_PARTS_U8
K.MAX_PARTS_U8 = 1          # two parts hit: more than one byte would number here
try:
    rc, out_u16, t_u16 = sweep_toy("toy_u16", ["--no-lidar", "--rover-parts", "wall"])
finally:
    K.MAX_PARTS_U8 = u8_max
g16 = ArmGuard.load(out_u16, calibration=NOWHERE) if t_u16 else None
check("more parts hit than one byte numbers: cell_part in two bytes, the same verdicts, the guard reads the dtype "
      "from the table and names the part; one byte up to 255 parts",
      rc == 0 and t_u16.get("cell_part_dtype") == "uint16" and t_dec.get("cell_part_dtype") == "uint8"
      and t_u16["occupancy_bits_hex"] == t_dec["occupancy_bits_hex"] and g16.cell_part.dtype == np.uint16
      and "decor" in g16.why({"j1": -1.0, "j2": 0.0}), g16.why({"j1": -1.0, "j2": 0.0}) if g16 else "")

box_obj(TMP / "cam.obj", CAM_H)
box_obj(TMP / "inner.obj", (0.005, 0.005, 0.005))
cam_xml = toy_xml(
    extra_base=f'<geom name="cam" type="mesh" mesh="cam" pos="{CAM_C[0]} {CAM_C[1]} 0"/>'
               f'<geom name="inner" type="mesh" mesh="inner" pos="{WALL_C[0]} 0 0"/>',
    assets='<mesh name="cam" file="cam.obj"/><mesh name="inner" file="inner.obj"/>')
VOX = 0.004
reach = 1.5 * VOX * math.sqrt(3)
rc, out_cam, t_cam = sweep_toy("toy_cam", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor",
                                           "--stand-in-voxel", str(VOX)], xml=cam_xml)
bad_c, occ_c, want_c, _ = differ(t_cam, [wall_clearance, cam_clearance], slack=reach)
missed = [f for f, w in want_c.items() if w and not occ_c[f]]
pm_c = t_cam.get("stand_ins", {}).get("per_mesh", {})
check("a rover mesh no pattern names (cam) is covered by stand-ins: no grid point within the margin is free, "
      f"none forbidden beyond the margin + {1000 * reach:.1f} mm",
      rc == 0 and not missed and not bad_c and pm_c.get("cam", {}).get("outside_fraction") == 1.0
      and pm_c["cam"].get("boxes", 0) >= 1 and "cam" in t_cam.get("part_names", []),
      f"{pm_c.get('cam')}, {len(bad_c)} differ outside the band")
check("a mesh inside the wall needs no stand-in (outside 0, 0 boxes)",
      pm_c.get("inner", {}).get("outside_fraction") == 0.0 and pm_c["inner"].get("boxes") == 0)
rc, _, t_nostand = sweep_toy("toy_cam0", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor",
                                          "--stand-in-voxel", "0"], xml=cam_xml)
occ_n = occ_of(t_nostand)
gc1, gc2 = t_cam["grid"]["j1"], t_cam["grid"]["j2"]
cam_only = [a * len(gc2) + b for a, v1 in enumerate(gc1) for b, v2 in enumerate(gc2)
            if cam_clearance(v1, v2) < MARGIN and wall_clearance(v1, v2) >= MARGIN]
check("--stand-in-voxel 0 leaves the mesh out: the grid points only cam touches come back free, cam listed as left out",
      rc == 0 and cam_only and not occ_n[cam_only].any() and "cam" in t_nostand.get("rover_left_out", []),
      f"{len(cam_only)} grid points")
h_before = (t_cam["model"]["sha256"], t_cam["model"]["geometry_sha256"])
box_obj(TMP / "cam.obj", CAM_H * 1.1)
st = (TMP / "cam.obj").stat()     # MuJoCo caches a mesh file by name and time (whole seconds): move it on
os.utime(TMP / "cam.obj", (st.st_atime + 5, st.st_mtime + 5))
rc, _, t_cam2 = sweep_toy("toy_cam", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor",
                                      "--stand-in-voxel", str(VOX)], xml=cam_xml)
check("a new mesh file under the same MJCF: same file sha256, new geometry_sha256",
      rc == 0 and t_cam2["model"]["sha256"] == h_before[0] and t_cam2["model"]["geometry_sha256"] != h_before[1])

rc, out2, t2 = sweep_toy("toy_lidar", ["--lidar-xyz", ",".join(f"{v:.6f}" for v in LIDAR_AT), "--rover-parts", "wall,lidar",
                                       "--exclude", "decor"])
bad2, occ2, _, _ = differ(t2, [wall_clearance, lidar_clearance]) if t2 else ([0], None, None, None)
lid = t2.get("lidar", {})
check("the lidar cylinder is added where asked: radius 0.038 m, height 0.04 m, it overlaps nothing",
      rc == 0 and lid.get("added") is True and lid.get("radius_m") == 0.038 and lid.get("height_m") == 0.04
      and np.allclose(lid.get("pos_m"), LIDAR_AT, atol=1e-6) and lid.get("overlaps_m") == {})
check("with the lidar: every grid point matches wall OR lidar (analytic), both parts named",
      not bad2 and set(t2.get("part_names", [])) == {"wall", "lidar"}, f"{len(bad2)} differ, parts {t2.get('part_names')}")
rc, _, t_ov = sweep_toy("toy_lidar_ov", ["--lidar-xyz", f"{WALL_C[0] - WALL_H[0]},0,0", "--rover-parts", "wall,lidar",
                                         "--exclude", "decor"])
ov = t_ov.get("lidar", {}).get("overlaps_m", {})
check("a lidar placed into the wall is reported: overlaps_m names the wall, 38 mm deep",
      rc == 0 and list(ov) == ["wall"] and abs(ov["wall"] + 0.038) < 0.002, str(ov))
rc, _, t3 = sweep_toy("toy_own", ["--lidar-xyz", "0,0,0", "--rover-parts", "wall,lidar", "--exclude", "decor"],
                      xml=toy_xml(extra_base='<geom name="lidar" type="box" pos="-0.5 0 0" size="0.01 0.01 0.01"/>'))
check("a model with its own geom named lidar keeps it (nothing added)", rc == 0 and t3.get("lidar", {}).get("added") is False)

# the root link (link1): a hub on it over a bar on the base (the mount), and a horn that hits a stop
BAR_C, BAR_H = np.array([0.0, 0.145, -0.016]), np.array([0.01, 0.175, 0.004])     # along +y, under the hub
HORN_R, STOP_A, BALL = 0.03, -1.2, 0.008
root_xml = toy_xml(
    extra_base=f'<geom name="bar" type="box" pos="{BAR_C[0]} {BAR_C[1]} {BAR_C[2]}" size="{BAR_H[0]} {BAR_H[1]} {BAR_H[2]}"/>'
               f'<geom name="stop" type="sphere" pos="{HORN_R * math.cos(STOP_A)} {HORN_R * math.sin(STOP_A)} 0" size="{BALL}"/>',
    extra_link1=f'<geom name="hub" type="sphere" size="0.02"/><geom name="horn" type="sphere" pos="{HORN_R} 0 0" size="{BALL}"/>')
rc, out_root, t_root = sweep_toy("toy_root", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor"], xml=root_xml)
occ_r = occ_of(t_root).reshape(61, 11) if t_root else np.zeros((61, 11), bool)
half = math.asin(BALL / HORN_R) * 2       # the horn and the stop overlap for |j1 - STOP_A| < half
inside = np.abs(g1 - STOP_A) < half - 1e-9
far = np.abs(g1 - STOP_A) > half + 0.025 + math.radians(K.ROOT_STEP_DEG) + 1e-9
want_rest = np.array([[min(wall_clearance(v1, v2), box_clearance(tip(v1, v2), BAR_C, BAR_H, TIP_R)) < MARGIN
                       for v2 in t_root["grid"]["j2"]] for v1 in g1]) if t_root else occ_r
bar_only = far[:, None] & want_rest & ~occ1.reshape(61, 11)
check("the root link: a bar its hub touches at every j1 is its mount (mount_parts); the tip still counts against it "
      "(wall OR bar, analytic, away from the horn's band)",
      rc == 0 and t_root.get("mount_parts") == ["bar"] and np.array_equal(occ_r[far], want_rest[far]) and bar_only.any(),
      f"mount {t_root.get('mount_parts')}, {int(bar_only.sum())} grid points only the bar forbids")
near_rows = (~inside) & (~far) & occ_r.all(axis=1)
check(f"the horn hits the stop for |j1 - ({STOP_A})| < {half:.3f} rad: those j1 values forbidden whole, "
      "and a value just outside, within half a grid step",
      rc == 0 and occ_r[inside].all() and near_rows.any() and "stop" in t_root.get("part_names", [])
      and t_root.get("root_link", {}).get("joints") == ["j1"], f"{int(inside.sum())} inside, {int(near_rows.sum())} near")

# a slide finger: its pad passes the post only between the two ends of its travel
slide_xml = """<mujoco model="slide">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="base">
      <geom name="post" type="sphere" pos="0.23 0 0" size="0.003"/>
      <body name="link1">
        <joint name="j1" type="hinge" axis="0 0 1" range="-1 1"/>
        <inertial pos="0.1 0 0" mass="0.05" diaginertia="1e-4 1e-4 1e-4"/>
        <body name="finger" pos="0.2 0 0">
          <joint name="finger" type="slide" axis="1 0 0" range="0 0.06"/>
          <geom name="pad" type="box" size="0.005 0.01 0.01"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
SL = ["--joints", "j1", "--steps", "41", "--arm-root", "link1", "--rover-body", "base", "--margin", str(MARGIN),
      "--pad-deg", "0", "--quiet", "--no-lidar", "--jobs", "1"]
rc, _, t_sw = sweep_toy("slide", [], xml=slide_xml, base=SL)
gs = np.array(t_sw.get("grid", {}).get("j1", [0.0]))


def post_clear(j1):
    """Analytic: post ball to the pad's swept box (x 0.195..0.265 in link1's frame)."""
    p = np.array([0.23 * math.cos(-j1), 0.23 * math.sin(-j1), 0.0])
    return box_clearance(p, np.array([0.23, 0, 0]), np.array([0.035, 0.01, 0.01]), 0.003)


occ_sw = occ_of(t_sw) if t_sw else np.zeros(0, bool)
want_sw = np.array([post_clear(v) < MARGIN for v in gs])
sure = np.array([abs(post_clear(v) - MARGIN) > 1e-4 for v in gs])
check("a slide finger's swept volume: the table matches the analytic pad sweep (0 at j1 = 0, the post inside it)",
      rc == 0 and len(occ_sw) == len(gs) and np.array_equal(occ_sw[sure], want_sw[sure]) and want_sw[20]
      and t_sw["unswept_joints"]["swept_volume"] == {"finger": [0.0, 0.06]}
      and t_sw["finger_check"]["poses"] > 0 and t_sw["finger_check"]["touching"] == 0,
      f"{int(occ_sw.sum())} of {len(occ_sw)} forbidden, finger check {t_sw.get('finger_check')}")
ends = []
for v in ("0.06", "0"):
    rc_e, _, t_e = sweep_toy(f"slide_{v}", ["--fix", f"finger={v}"], xml=slide_xml, base=SL)
    ends.append(rc_e == 0 and not occ_of(t_e)[20] and t_e["unswept_joints"]["fixed"] == {"finger": float(v)})
check("pinned open (--fix finger=0.06) or closed (finger=0), j1 = 0 comes back free: one end alone misses the post",
      all(ends))

# a payload: a hand at the tip of link2 (0.05 m out) with a palm and two slide
# fingers; everything is a prism over the plane of the arm within the wall's
# height, so the clearances are 2-D (convex polygons and a circle)
HAND_AT = 0.06
PWALL_C, PWALL_H = (0.31, 0.0, 0.0), (0.02, 0.3, 0.05)    # a wide wall: the reach decides, not the wall's ends
PALM_X, PALM_HY = (-0.012, -0.002), 0.04
PAD_X, PAD_T, PAD_FACE0, PAD_TRAVEL = (0.005, 0.035), 0.006, 0.030, 0.02
CYL_D, CYL_H = 0.040, 0.100
box_obj(TMP / "palm.obj", ((PALM_X[1] - PALM_X[0]) / 2, PALM_HY, 0.02))
box_obj(TMP / "pad.obj", ((PAD_X[1] - PAD_X[0]) / 2, PAD_T / 2, 0.015))
pay_xml = f"""<mujoco model="toy_payload">
  <compiler angle="radian" autolimits="true" meshdir="{TMP}"/>
  <asset><mesh name="palm" file="palm.obj"/><mesh name="pad" file="pad.obj"/></asset>
  <worldbody>
    <body name="base">
      <geom name="wall" type="box" pos="{PWALL_C[0]} {PWALL_C[1]} {PWALL_C[2]}" size="{PWALL_H[0]} {PWALL_H[1]} {PWALL_H[2]}"/>
      <body name="link1">
        <joint name="j1" type="hinge" axis="0 0 1" range="{J1[0]} {J1[1]}"/>
        <inertial pos="0.1 0 0" mass="0.05" diaginertia="1e-4 1e-4 1e-4"/>
        <body name="link2" pos="{L1} 0 0">
          <joint name="j2" type="hinge" axis="0 0 1" range="{J2[0]} {J2[1]}"/>
          <body name="hand" pos="{HAND_AT} 0 0">
            <geom name="palm" type="mesh" mesh="palm" pos="{sum(PALM_X) / 2} 0 0"/>
            <body name="finger_a">
              <joint name="finger_a" type="slide" axis="0 1 0" range="0 {PAD_TRAVEL}"/>
              <geom name="pad_a" type="mesh" mesh="pad" pos="{sum(PAD_X) / 2} {-(PAD_FACE0 + PAD_T / 2)} 0"/>
            </body>
            <body name="finger_b">
              <joint name="finger_b" type="slide" axis="0 -1 0" range="0 {PAD_TRAVEL}"/>
              <geom name="pad_b" type="mesh" mesh="pad" pos="{sum(PAD_X) / 2} {PAD_FACE0 + PAD_T / 2} 0"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
PAY = ["--no-lidar", "--rover-parts", "wall"]
WALL2 = np.array([[PWALL_C[0] - PWALL_H[0], -PWALL_H[1]], [PWALL_C[0] + PWALL_H[0], -PWALL_H[1]],
                  [PWALL_C[0] + PWALL_H[0], PWALL_H[1]], [PWALL_C[0] - PWALL_H[0], PWALL_H[1]]])


def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)


def _seg(p, a, b):
    ab = b - a
    t = min(1.0, max(0.0, float(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-18))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _apart(P, Q):
    """A separating axis between the convex polygons (or points) P and Q."""
    for poly in (P, Q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            if np.dot(n, n) > 0 and ((P @ n).max() < (Q @ n).min() or (Q @ n).max() < (P @ n).min()):
                return True
    return False


def poly_clear(P, Q):
    """Analytic: the distance between two convex polygons, -1 when they overlap."""
    if not _apart(P, Q):
        return -1.0
    return min(min(_seg(p, Q[i], Q[(i + 1) % len(Q)]) for p in P for i in range(len(Q))),
               min(_seg(q, P[i], P[(i + 1) % len(P)]) for q in Q for i in range(len(P))))


def circle_clear(c, r, Q):
    """Analytic: a disc to a convex polygon, -1 when they overlap."""
    if not _apart(np.array([c]), Q):
        return -1.0
    d = min(_seg(np.asarray(c), Q[i], Q[(i + 1) % len(Q)]) for i in range(len(Q))) - r
    return d if d > 0 else -1.0


def hand_world(P, j1, j2):
    """Hand-frame points (x, y) -> the base's frame."""
    th = j1 + j2
    rot = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return P @ rot.T + np.array([L1 * math.cos(j1) + HAND_AT * math.cos(th), L1 * math.sin(j1) + HAND_AT * math.sin(th)])


def empty_hand_clear(j1, j2):
    """The palm and the two pads' swept volumes (each pad over its whole slide) to the wall."""
    shapes = [rect(PALM_X[0], PALM_X[1], -PALM_HY, PALM_HY),
              rect(PAD_X[0], PAD_X[1], -(PAD_FACE0 + PAD_T), -(PAD_FACE0 - PAD_TRAVEL)),
              rect(PAD_X[0], PAD_X[1], PAD_FACE0 - PAD_TRAVEL, PAD_FACE0 + PAD_T)]
    return min(poly_clear(hand_world(s, j1, j2), WALL2) for s in shapes)


def can_clear(j1, j2, upright_x, across_x):
    """The cylinder held upright (its axis along the fingers: its outline in the plane is a rectangle) and
    held across (its axis vertical: a disc) to the wall."""
    up = poly_clear(hand_world(rect(upright_x - CYL_H / 2, upright_x + CYL_H / 2, -CYL_D / 2, CYL_D / 2), j1, j2), WALL2)
    across = circle_clear(hand_world(np.array([[across_x, 0.0]]), j1, j2)[0], CYL_D / 2, WALL2)
    return min(up, across)


rc_p, _, t_pay = sweep_toy("toy_pay", PAY + ["--payload", f"cyl:{1000 * CYL_D:g}x{1000 * CYL_H:g}"], xml=pay_xml)
rc_n, _, t_pn = sweep_toy("toy_pay_none", PAY + ["--payload", "none"], xml=pay_xml)
pp = t_pay.get("payload") or {}
pose_u = pp.get("pose", {}).get("upright", {})
pose_a = pp.get("pose", {}).get("across", {})
want_up = PALM_X[1] + CYL_H / 2           # pushed out of the palm: its end on the palm's face
want_across = sum(PAD_X) / 2              # at the pads' middle, 2 mm clear of the palm
check(f"a cylinder {1000 * CYL_D:g} x {1000 * CYL_H:g} mm in a two-finger hand: both fingers at 10 mm (40 mm apart), "
      f"upright along the fingers pushed out of the palm to {1000 * want_up:g} mm, across (vertical) at the pads' "
      f"middle ({1000 * want_across:g} mm), the palm named as what pushed it",
      rc_p == 0 and pp.get("kind") == "cyl" and pp.get("diameter_m") == CYL_D and pp.get("height_m") == CYL_H
      and pp.get("orientations") == ["upright", "across"] and pp.get("hand") == "hand"
      and pp.get("fingers") == {"finger_a": 0.01, "finger_b": 0.01} and abs(pp.get("gap_m", 0) - CYL_D) < 1e-9
      and np.allclose(pose_u.get("pos_m", [9] * 3), [want_up, 0, 0], atol=1e-4)
      and np.allclose(np.abs(pose_u.get("axis", [0] * 3)), [1, 0, 0]) and pose_u.get("pushed_out_by") == "palm"
      and np.allclose(pose_a.get("pos_m", [9] * 3), [want_across, 0, 0], atol=1e-4)
      and np.allclose(np.abs(pose_a.get("axis", [0] * 3)), [0, 0, 1]) and pose_a.get("pushed_out_m") == 0.0,
      f"upright {pose_u.get('pos_m')}, across {pose_a.get('pos_m')}, fingers {pp.get('fingers')}")
occ_pay = occ_of(t_pay) if t_pay else np.zeros(0, bool)
occ_pn = occ_of(t_pn) if t_pn else np.zeros(0, bool)
ux = pose_u.get("pos_m", [want_up])[0]
ax_ = pose_a.get("pos_m", [want_across])[0]
gp1, gp2 = (t_pay.get("grid", {}).get(j, []) for j in ("j1", "j2"))
bad_u, bad_n, band_only = 0, 0, []
for a, v1 in enumerate(gp1):
    for b, v2 in enumerate(gp2):
        flat = a * len(gp2) + b
        ce, cc = empty_hand_clear(v1, v2), can_clear(v1, v2, ux, ax_)
        cu = min(ce, cc)
        bad_u += abs(cu - MARGIN) > 1e-4 and bool(occ_pay[flat]) != (cu < MARGIN)
        bad_n += abs(ce - MARGIN) > 1e-4 and bool(occ_pn[flat]) != (ce < MARGIN)
        if cc < MARGIN - 1e-4 and ce > MARGIN + 1e-4:
            band_only.append(flat)
check("every grid point matches the analytic verdicts: with the cylinder, the empty hand OR the cylinder (upright "
      "OR across) within 5 mm of the wall; --payload none, the empty hand alone; the cylinder adds the cells only "
      "it reaches",
      rc_p == 0 and rc_n == 0 and len(occ_pay) == len(gp1) * len(gp2) and bad_u == 0 and bad_n == 0
      and band_only and occ_pay[band_only].all() and not occ_pn[band_only].any(),
      f"{int(occ_pn.sum())} cells empty, {int(occ_pay.sum())} with the cylinder, {len(band_only)} only it reaches, "
      f"{bad_u} + {bad_n} differ")
row_u, row_n = occ_pay.reshape(61, 11)[:, 5], occ_pn.reshape(61, 11)[:, 5]
ku, kn = np.flatnonzero(row_u), np.flatnonzero(row_n)
pf = pp.get("forbidden", {})
check("at j2 = 0 the cylinder hits the wall sooner on both sides: its band is wider than the empty hand's at both "
      "ends; the JSON counts add up (empty = --payload none, union = the table)",
      len(ku) and len(kn) and ku[0] < kn[0] and ku[-1] > kn[-1]
      and pf.get("empty") == int(occ_pn.sum()) == t_pn.get("forbidden") and pf.get("union") == t_pay.get("forbidden")
      and pp.get("added", {}).get("total") == t_pay.get("forbidden", 0) - int(occ_pn.sum())
      and t_pn.get("payload") is None,
      f"j1 {gp1[ku[0]]:+.2f}..{gp1[ku[-1]]:+.2f} with the cylinder, {gp1[kn[0]]:+.2f}..{gp1[kn[-1]]:+.2f} without; "
      f"{pf}" if len(ku) and len(kn) else str(pf))
CYL_ARG = ["--payload", f"cyl:{1000 * CYL_D:g}x{1000 * CYL_H:g}"]
rc_re, _, t_re = sweep_toy("toy_pay_reuse", PAY + CYL_ARG + ["--reuse-empty", str(TMP / "toy_pay_none.json")], xml=pay_xml)
rc_rx, _, _ = sweep_toy("toy_pay_reuse_x", PAY + CYL_ARG + ["--reuse-empty", str(TMP / "toy_pay_none.json"),
                                                            "--margin", "0.004"], xml=pay_xml)
same = [k for k in ("occupancy_bits_hex", "occupancy_empty_bits_hex", "cell_part", "pair_cells", "part_names",
                    "part_counts", "forbidden") if t_re.get(k) != t_pay.get(k)]
check("--reuse-empty: the empty hand taken from the --payload none table, only the cylinder swept, gives the same "
      "table; a table swept at another margin is refused",
      rc_re == 0 and not same and (t_re.get("empty_sweep") or {}).get("reused_from", {}).get("file") == "toy_pay_none.json"
      and rc_rx not in (0, None), f"differ: {same}, refused: {rc_rx!r}"[:160])
refusals = []
for extra, xml in ((["--payload", "cyl:70x100"], pay_xml), (["--payload", "can"], None)):
    rc_r, _, _ = sweep_toy("toy_refused", PAY + extra, xml=xml)
    refusals.append(rc_r not in (0, None))
check("refused: a cylinder 70 mm across in a hand that opens 60 mm, and a payload on an arm with no slide fingers",
      all(refusals), str(refusals))

# a MuJoCo collision error at one grid point
bad_j1 = float(g1[40])
real_collision, real_forward = mujoco.mj_collision, mujoco.mj_forward


def failing(fn):
    def call(m, d):
        if m.nq == 2 and abs(d.qpos[0] - bad_j1) < 1e-9:
            raise mujoco.FatalError("collision function returned too many contacts (bench)")
        return fn(m, d)
    return call


mujoco.mj_collision, mujoco.mj_forward = failing(real_collision), failing(real_forward)
try:
    rc, out_err, t_err = sweep_toy("toy_err", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor"])
finally:
    mujoco.mj_collision, mujoco.mj_forward = real_collision, real_forward
occ_e = occ_of(t_err).reshape(61, 11) if t_err else np.zeros((61, 11), bool)
ge = ArmGuard.load(out_err, calibration=NOWHERE) if t_err else None
check("a MuJoCo collision error: the cell is forbidden, counted, and why() says so",
      rc == 0 and t_err["collision_errors"] == 11 and occ_e[40].all() and "a MuJoCo collision error" in t_err["part_names"]
      and "collision failed" in ge.why({"j1": bad_j1, "j2": 0.0}), f"errors {t_err.get('collision_errors')}")

doc = json.loads(out1.read_text())
doc["joint_map"]["j1"] = {"model_zero_ticks": 1990, "sign": -1}
out1.write_text(json.dumps(doc))
rc, _, t1b = sweep_toy("toy", ["--no-lidar", "--rover-parts", "wall", "--exclude", "decor"])
check("a re-sweep keeps the joint_map of the table it replaces",
      rc == 0 and t1b["joint_map"]["j1"] == {"model_zero_ticks": 1990, "sign": -1}
      and t1b["joint_map"]["j2"] == {"model_zero_ticks": 2048, "sign": 1})
check("the table records the model's sha256 and geometry sha256, the margin and the run time",
      len(t1b["model"]["sha256"]) == 64 and len(t1b["model"]["geometry_sha256"]) == 64
      and t1b["margin_m"] == MARGIN and t1b["run_time_s"] >= 0)

print("B. the guard on the wall table")
guard = ArmGuard.load(out1, calibration=NOWHERE)
step = 0.05
inside_q = {"j1": 0.0, "j2": 0.0}
check("inside the band (0, 0) refused, why() names the wall",
      guard.allowed(inside_q) is False and "wall" in guard.why(inside_q), guard.why(inside_q))
one_out = {"j1": float(g1[hi_k + 1]), "j2": 0.0}
one_out_lo = {"j1": float(g1[lo_k - 1]), "j2": 0.0}
check(f"one grid step outside ({g1[hi_k + 1]:+.2f} and {g1[lo_k - 1]:+.2f} rad) allowed, why() says free",
      guard.allowed(one_out) and guard.allowed(one_out_lo) and guard.why(one_out).startswith("free"))
mid = {"j1": float((g1[hi_k] + g1[hi_k + 1]) / 2), "j2": 0.0}
check("between the band's last point and the free one: the interpolation touches the band, refused",
      guard.allowed(mid) is False and "wall" in guard.why(mid),
      f"analytic clearance there {1000 * (wall_clearance(mid['j1'], 0.0) - MARGIN):+.1f} mm past the margin")
off_grid = {"j1": float(g1[hi_k + 1] + 0.3 * step), "j2": 0.05}
check("off the grid with all four grid points around it free: allowed", guard.allowed(off_grid))
check("outside the table (j1 = 1.6 rad > 1.5) refused, why() says outside",
      guard.allowed({"j1": 1.6, "j2": 0.0}) is False and "outside" in guard.why({"j1": 1.6, "j2": 0.0}))
try:
    guard.allowed({"j1": 0.0})
    check("a pose without j2 raises ValueError", False)
except ValueError:
    check("a pose without j2 raises ValueError", True)
# nearest_allowed: the nearest pose that stays allowed when nudged (brute force on the 2-D table)
occ2d = guard.occ
free2d = ~occ2d
robust = [(a + 0.5, b + 0.5) for a in range(60) for b in range(10) if free2d[a:a + 2, b:b + 2].all()]
robust += [(a, b) for a in range(1, 60) for b in range(1, 10) if free2d[a - 1:a + 2, b - 1:b + 2].all()]
toy_map = JointMap.from_docs(None, {"joints": ["j1", "j2"], "joint_map": {"j1": {"model_zero_ticks": 2000, "sign": -1}}})
gt = ArmGuard(json.loads(out1.read_text()), toy_map)
worst_gap, nudged, ticks_ok = 0.0, 0, 0
rng = np.random.default_rng(11)
qs = [inside_q, mid] + [{"j1": float(rng.uniform(-1.5, 1.5)), "j2": float(rng.uniform(-0.5, 0.5))} for _ in range(300)]
for q in qs:
    near = gt.nearest_allowed(q)
    f = gt._fracs(q)
    best = min(max(abs(c[0] - f[0]), abs(c[1] - f[1])) for c in robust)
    got = gt._fracs(near)
    worst_gap = max(worst_gap, max(abs(got[0] - f[0]), abs(got[1] - f[1])) - best)
    for d1, d2 in itertools.product((-0.01, 0.0, 0.01), repeat=2):
        nudged += not gt.allowed({"j1": near["j1"] + d1 * step, "j2": near["j2"] + d2 * 0.1})
    t = toy_map.pose_to_ticks(near)
    ticks_ok += gt.allowed_ticks(t) and gt.allowed_ticks(gt.nearest_allowed_ticks(toy_map.pose_to_ticks(q)))
check("nearest_allowed: the nearest pose (in grid steps) among those that stay allowed when nudged, "
      "never refused after the tick rounding (zero 2000, sign -1), nor nearest_allowed_ticks",
      worst_gap < 1e-9 and nudged == 0 and ticks_ok == len(qs),
      f"{len(qs)} poses, {len(robust)} robust targets, gap {worst_gap:.2g} steps, {nudged} nudges refused")


def quarter(gd, a, b):
    """The old rule: samples every quarter grid step."""
    fa, fb = gd._fracs(a), gd._fracs(b)
    n = max(1, int(math.ceil(float(np.max(np.abs(fb - fa))) / 0.25)))
    return all(gd.allowed({j: (1 - i / n) * a[j] + i / n * b[j] for j in gd.joints}) for i in range(n + 1))


def fine(gd, a, b, per_step=256):
    fa, fb = gd._fracs(a), gd._fracs(b)
    n = max(1, int(math.ceil(float(np.max(np.abs(fb - fa))) * per_step)))
    return all(gd.allowed({j: (1 - i / n) * a[j] + i / n * b[j] for j in gd.joints}) for i in range(n + 1))


corner, loose_ok, seg_n = None, 0, 0
for _ in range(3000):
    a = {"j1": float(rng.uniform(-1.5, 1.5)), "j2": float(rng.uniform(-0.5, 0.5))}
    b = {"j1": float(np.clip(a["j1"] + rng.uniform(-0.2, 0.2), -1.5, 1.5)),
         "j2": float(np.clip(a["j2"] + rng.uniform(-0.3, 0.3), -0.5, 0.5))}
    if not (guard.allowed(a) and guard.allowed(b)):
        continue
    seg_n += 1
    exact, fi = guard.segment_allowed(a, b), fine(guard, a, b, 64)
    loose_ok += exact and not fi
    if corner is None and quarter(guard, a, b) and not fi:
        corner = (a, b)
check("segment_allowed: a move that cuts a forbidden cell's corner between quarter-step samples is refused; "
      "no move a 1/64-step sampling refuses is allowed",
      corner is not None and not guard.segment_allowed(*corner) and loose_ok == 0 and seg_n > 500,
      f"{seg_n} moves, corner case {corner}")
check("a move across the band (both ends allowed) refused; a move far from it allowed",
      not guard.segment_allowed(one_out_lo, one_out)
      and guard.segment_allowed({"j1": -1.4, "j2": -0.3}, {"j1": -1.0, "j2": 0.3}))
guard2 = ArmGuard.load(out2, calibration=NOWHERE)
at_lidar = {"j1": 1.0, "j2": 0.0}
check("why() at the lidar's angle (j1 = 1 rad) names the lidar",
      not guard2.allowed(at_lidar) and "lidar" in guard2.why(at_lidar), guard2.why(at_lidar))
tm = dict(t1b)
tm["occupancy_bits_hex"] = "00" * len(bytes.fromhex(t1b["occupancy_bits_hex"]))
try:
    ArmGuard(tm)
    check("a table whose bits and cell_part disagree is refused", False)
except ValueError:
    check("a table whose bits and cell_part disagree is refused", True)
bad_tables = 0
for change in ({"cell_part": [300 if v else 0 for v in t1b["cell_part"]]},                       # past one byte
               {"cell_part": [len(t1b["part_names"]) + 1 if v else 0 for v in t1b["cell_part"]]},   # past part_names
               {"cell_part_dtype": "int32"}):
    try:
        ArmGuard(dict(t1b, **change))
    except ValueError:
        bad_tables += 1
check("a table whose cell_part does not fit its dtype, names a part past part_names, or has another dtype is refused",
      bad_tables == 3, f"{bad_tables} of 3 refused")

print("C. ticks <-> radians")
cal = {"shoulder_pan": {"id": 1, "homing_offset": 2042}, "shoulder_lift": {"id": 2, "homing_offset": -1369}}
tab = {"joints": ["shoulder_pan", "shoulder_lift"],
       "joint_map": {"shoulder_pan": {"model_zero_ticks": 2048, "sign": 1},
                     "shoulder_lift": {"model_zero_ticks": 1990, "sign": -1}}}
jm = JointMap.from_docs(cal, tab)
check("pan: 3072 ticks = +90 deg, 1024 = -90 deg (zero 2048, sign +1)",
      abs(jm.ticks_to_rad("shoulder_pan", 3072) - math.pi / 2) < 1e-12
      and abs(jm.ticks_to_rad("shoulder_pan", 1024) + math.pi / 2) < 1e-12)
check("lift: 3014 ticks = -90 deg (zero 1990, sign -1), and back",
      abs(jm.ticks_to_rad("shoulder_lift", 3014) + math.pi / 2) < 1e-12
      and jm.rad_to_ticks("shoulder_lift", -math.pi / 2) == 3014)
check("raw readings: pan raw 1018 = reported 3072 (offset +2042), lift raw 1645 = reported 3014 (offset -1369)",
      abs(jm.ticks_to_rad("shoulder_pan", 1018, raw=True) - math.pi / 2) < 1e-12
      and jm.rad_to_ticks("shoulder_pan", math.pi / 2, raw=True) == 1018
      and abs(jm.ticks_to_rad("shoulder_lift", 1645, raw=True) + math.pi / 2) < 1e-12
      and jm.rad_to_ticks("shoulder_lift", -math.pi / 2, raw=True) == 1645)
ok_t = all(jm.rad_to_ticks(j, jm.ticks_to_rad(j, t)) == t for j in ("shoulder_pan", "shoulder_lift") for t in range(4096))
ok_r = all(jm.ticks_to_raw(j, jm.raw_to_ticks(j, r)) == r for j in ("shoulder_pan", "shoulder_lift") for r in range(4096))
half_tick = math.pi / 4096
worst = max(abs(jm.ticks_to_rad(j, jm.rad_to_ticks(j, q)) - q) for j in ("shoulder_pan", "shoulder_lift")
            for q in rng.uniform(-2.5, 2.5, 500))
check("round trips: ticks -> rad -> ticks exact (4096 x 2), raw -> ticks -> raw exact, rad -> ticks -> rad within half a tick",
      ok_t and ok_r and worst <= half_tick + 1e-12, f"worst {math.degrees(worst):.4f} deg")
check("ids and offsets from the calibration; no file: ids by convention, offsets 0",
      jm.joints["shoulder_lift"].servo_id == 2 and jm.joints["shoulder_lift"].homing_offset == -1369
      and JointMap.from_docs(None, {"joints": ["elbow_flex"]}).joints["elbow_flex"].servo_id == 3
      and JointMap.from_docs(None, {"joints": ["elbow_flex"]}).joints["elbow_flex"].homing_offset == 0)
nominal = 4096 / (2 * math.pi)
ta = 1990.0 - 0.3 * nominal
tb = 1990.0 - 1.4 * nominal
al = align_from_two_poses(round(ta), 0.3, round(tb), 1.4)
check("two-pose alignment: poses at 0.3 and 1.4 rad give back zero 1990 and sign -1",
      al["sign"] == -1 and abs(al["model_zero_ticks"] - 1990) <= 0.5, str(al))
refused = 0
for args in ((2048, 0.1, 2050, 0.15), (2048, 0.0, 2600, 1.57)):
    try:
        align_from_two_poses(*args)
    except ValueError:
        refused += 1
check("two-pose alignment refuses poses 0.05 rad apart and a scale 45 % off", refused == 2)
true_map = {"shoulder_pan": (2010.0, 1), "shoulder_lift": (2100.0, -1), "elbow_flex": (1980.0, -1),
            "wrist_flex": (2048.0, 1), "wrist_roll": (2070.0, -1)}
read_a = {j: round(z + s * POSE_A[j] * nominal) for j, (z, s) in true_map.items()}
read_b = {j: round(z + s * POSE_B[j] * nominal) for j, (z, s) in true_map.items()}
full = align_joint_map(read_a, read_b)
naive = align_from_two_poses(read_a["elbow_flex"], 0.0, read_b["elbow_flex"], 1.571)
check("align_joint_map with POSE_A / POSE_B (elbow -1.571 in B): every sign and zero back (within half a tick); "
      "the elbow aligned with +1.571 would get the wrong sign",
      all(full[j]["sign"] == s and abs(full[j]["model_zero_ticks"] - z) <= 0.5 for j, (z, s) in true_map.items())
      and naive["sign"] == -true_map["elbow_flex"][1], f"{full['elbow_flex']} vs {naive}")
check("allowed_ticks: 2000/2048 is (0, 0) in the band; j1 = 1348 ticks (about 1 rad, sign -1) is free; the gripper key is ignored",
      gt.allowed_ticks({"j1": 2000, "j2": 2048, "gripper": 3000}) is False
      and abs(toy_map.ticks_to_rad("j1", 1348) - 1.0) < 0.002 and gt.allowed_ticks({"j1": 1348, "j2": 2048}))

print("D. the real model")
model_path = os.environ.get("MOSS_ARM_MODEL", "").strip()
if not model_path:
    found = sorted(glob.glob(os.path.expanduser("~/moss-*/live/model/moss.xml")))
    model_path = found[0] if found else ""
if not model_path or not Path(model_path).is_file():
    print("  -- skipped, no robot model here (MOSS_ARM_MODEL, or ~/moss-*/live/model/moss.xml)")
else:
    print(f"  model {Path(model_path).name}")
    outd = TMP / "real.json"
    rc = K.main(["--model", model_path, "--out", str(outd), "--steps", "8,8,8,6,4", "--quiet", "--jobs", "1"])
    td = json.loads(outd.read_text()) if outd.exists() else {}
    check("the coarse sweep 8,8,8,6,4 exits 0 (its mj_forward and real-finger checks agree)", rc == 0, f"rc {rc}")
    check("at least one forbidden cell", td.get("forbidden", 0) >= 1,
          f"{td.get('forbidden')} of {td.get('cells')} ({100 * td.get('forbidden_fraction', 0):.1f} %)")
    hit = list(td.get("part_counts", {}))
    check("the hull or the bin is among the parts hit", any(p == "hull" or p.startswith("bin_") for p in hit),
          ", ".join(f"{p} {td['part_counts'][p]}" for p in hit[:6]))
    z = robot_profile.current().lidar_height_m
    check(f"the lidar is added at (0, 0, {z}) on the rover (profile lidar_height_m)",
          td.get("lidar", {}).get("added") is True and np.allclose(td["lidar"]["pos_m"], (0.0, 0.0, z)))
    print(f"  fraction forbidden: {100 * td.get('forbidden_fraction', 0):.1f} %")
    # the model's poses: all zeros, and straight up (the tool centre at its highest), read off the model
    pd = K.prepare(Path(model_path), margin=td["margin_m"], lidar_xyz=td["lidar"]["pos_m"])
    model = pd.model
    data = mujoco.MjData(model)
    adr = {j: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in K.JOINTS}
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    is_arm, is_part = K._masks(model, pd.arm_ids + pd.root_ids, pd.part_ids)

    def place(q):
        data.qpos[:] = pd.base_qpos
        for j, v in q.items():
            data.qpos[adr[j]] = v
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        return K._contacts(model, data, is_arm, is_part, td["margin_m"])

    def body_z(name):
        return float(data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)][2])

    zeros = {j: 0.0 for j in K.JOINTS}
    place(zeros)
    print(f"  all joints 0: upper arm from z {body_z('upper_arm_link'):.3f} to {body_z('lower_arm_link'):.3f} m (vertical), "
          f"forearm to z {body_z('wrist_link'):.3f} m (level), tool centre z {data.site_xpos[tcp][2]:.3f} m: an L, not straight up")
    best = (-1.0, None)
    for lift in np.arange(-1.7, 1.71, 0.1):
        for elbow in np.arange(-1.65, 1.66, 0.1):
            for flex in np.arange(-1.6, 1.61, 0.1):
                q = dict(zeros, shoulder_lift=lift, elbow_flex=elbow, wrist_flex=flex)
                data.qpos[:] = pd.base_qpos
                for j, v in q.items():
                    data.qpos[adr[j]] = v
                mujoco.mj_kinematics(model, data)
                if data.site_xpos[tcp][2] > best[0]:
                    best = (float(data.site_xpos[tcp][2]), q)
    up = best[1]
    contacts_up = place(up)
    gd = ArmGuard.load(outd, calibration=NOWHERE)
    print(f"  straight up: lift {up['shoulder_lift']:+.2f}, elbow {up['elbow_flex']:+.2f}, flex {up['wrist_flex']:+.2f} rad, "
          f"tool centre z {best[0]:.3f} m")
    check("the arm straight up touches nothing (model) and the guard allows it",
          not contacts_up and gd.allowed(up), gd.why(up))
    outdc = TMP / "real_can.json"
    rc = K.main(["--model", model_path, "--out", str(outdc), "--steps", "8,8,8,6,4", "--quiet", "--jobs", "1",
                 "--payload", "can"])
    tdc = json.loads(outdc.read_text()) if outdc.exists() else {}
    pdc = tdc.get("payload") or {}
    check("--payload can on the simulation model: the can between pad_left and pad_right at a 66 mm gap (both "
          "fingers at 29 mm), held both ways, the empty gripper's cells the same as without it, and more forbidden",
          rc == 0 and pdc.get("pads") == ["pad_left", "pad_right"] and abs(pdc.get("gap_m", 0) - 0.066) < 1e-6
          and all(abs(v - 0.029) < 1e-6 for v in pdc.get("fingers", {"x": 0}).values())
          and pdc.get("forbidden", {}).get("empty") == td.get("forbidden") and tdc.get("forbidden", 0) > td["forbidden"],
          f"{pdc.get('fingers')}, {pdc.get('forbidden')}")
    robot = Path(model_path).parent / "moss_robot.xml"
    if robot.is_file():
        outm = TMP / "margin.json"
        rc = K.main(["--model", str(robot), "--out", str(outm), "--steps", "8,8,8,6,4", "--margin", "0.03", "--quiet",
                     "--jobs", "1"])
        tm3 = json.loads(outm.read_text()) if outm.exists() else {}
        check("the robot model at --margin 0.03 finishes and writes its table (collision errors, if any, counted)",
              rc == 0 and tm3.get("margin_m") == 0.03, f"rc {rc}, {tm3.get('collision_errors')} collision errors, "
              f"{100 * tm3.get('forbidden_fraction', 0):.1f} % forbidden")

print("E. the CAD scene")
# the two poses the robot stands in every day, in servo ticks (homing offset applied)
DRIVE = {"shoulder_pan": 2857, "shoulder_lift": 860, "elbow_flex": 3206, "wrist_flex": 868, "wrist_roll": 3221}
LOOK = {"shoulder_pan": 2869, "shoulder_lift": 1057, "elbow_flex": 3208, "wrist_flex": 2018, "wrist_roll": 3220}
# three postures of an independent clearance study on the same scene (model radians; the pan turned from
# +110 deg down, the rest held), with a can upright in the gripper: travel first touches near -46 deg (the
# left jaw and its pad against the bin, contacts down to -94), pick near -45 deg (the can against the left
# road-wheel shaft and collar before the arm, contacts down to -110), clear never
STUDY = {"travel": {"shoulder_lift": -0.8097555478420315, "elbow_flex": 0.3832857982375162,
                    "wrist_flex": 1.5518884280412557, "wrist_roll": -0.6859999406231995},
         "pick": {"shoulder_lift": 0.5532629270078482, "elbow_flex": -0.08444815096969016,
                  "wrist_flex": 1.1019852841030555, "wrist_roll": -1.585330564271262},
         "clear": {"shoulder_lift": -1.2339528188850326, "elbow_flex": -0.3445092260113539,
                   "wrist_flex": 1.651086912169343, "wrist_roll": -1.5853578786239502}}
STUDY_CONTACTS = {"travel": (-46, -94), "pick": (-45, -110)}
CAD_MARGIN = 0.002          # the CAD layout's margin: the largest at which drive and look stay allowed
# a CAD working code (two capitals and two digits, as a word): never in the public table
WORK_CODE = re.compile(r"\b[A-Z]{2}\d{2}\b")
# the arm's calibration: in the robot's checkout, else MOSS_ARM_CALIBRATION, else a private copy
cal_path = ROOT / "config" / "arm_calibration.json"
if not cal_path.is_file():
    cal_path = Path(os.environ.get("MOSS_ARM_CALIBRATION", "").strip() or "/nonexistent")
if not cal_path.is_file():
    found = sorted(glob.glob(os.path.expanduser("~/moss-*/docs/arm_calibration_*.json")))
    cal_path = Path(found[-1]) if found else None
calibration = json.loads(cal_path.read_text()) if cal_path else None


def travel_outside(table, cal):
    """[(joint, degrees outside)] for each joint whose calibrated travel
    (range_min..range_max through the table's joint_map) leaves the table's grid."""
    jm_t = JointMap.from_docs(None, table)
    out = []
    for j in table["joints"]:
        g = table["grid"][j]
        lo, hi = sorted(jm_t.ticks_to_rad(j, cal[j][k]) for k in ("range_min", "range_max"))
        past = max(g[0] - lo, hi - g[-1])
        if past > 0:
            out.append((j, round(math.degrees(past), 2)))
    return out


def rover_collision_geoms(path, rover=("rover_exterior", "base"), arm="shoulder", also=()):
    """The rover's geoms straight from the model, apart from the tool: every
    geom that collides (contype or conaffinity), plus the geoms named in
    `also`, whose body chain meets a rover body before the arm's root."""
    m = mujoco.MjModel.from_xml_path(str(path))
    bname = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
    n = 0
    for g in range(m.ngeom):
        if not (m.geom_contype[g] or m.geom_conaffinity[g]
                or mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) in also):
            continue
        b = int(m.geom_bodyid[g])
        while b != 0 and bname[b] not in rover and bname[b] != arm:
            b = int(m.body_parentid[b])
        n += bname[b] in rover
    return n


def rover_accounted(t):
    return (t.get("rover_geoms_counted", 0) + t.get("rover_stand_in_meshes", 0) + t["rover_inside"]["geoms"]
            + len(t["rover_left_out"]))


cad_path = os.environ.get("MOSS_ARM_CAD", "").strip()
if not cad_path:
    found = sorted(glob.glob(os.path.expanduser("~/moss-*/cad/*/MOSS_R31.xml")))
    cad_path = found[-1] if found else ""
manifest = Path(cad_path).parent / "parts_manifest.json" if cad_path else None
if not cad_path or not Path(cad_path).is_file():
    print("  -- skipped, no CAD scene here (MOSS_ARM_CAD, or ~/moss-*/cad/*/MOSS_R31.xml)")
else:
    print(f"  model {Path(cad_path).name}")
    names = K.load_names(manifest) if manifest.is_file() else {}
    raw_names = {p["id"]: p["name"] for p in json.loads(manifest.read_text())["parts"]} if manifest.is_file() else {}
    cad_model = mujoco.MjModel.from_xml_path(cad_path)
    geoms = {mujoco.mj_id2name(cad_model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(cad_model.ngeom)}

    def hull_verts(m, g):
        adr = int(m.mesh_graphadr[m.geom_dataid[g]])
        return int(m.mesh_graph[adr]) if adr >= 0 else 0

    # what the checks below read from the scene as MuJoCo loads it, taken now: the model then goes (some 350 MB)
    drive_geoms = [mujoco.mj_id2name(cad_model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(cad_model.ngeom)
                   if cad_model.geom_bodyid[g] == mujoco.mj_name2id(cad_model, mujoco.mjtObj.mjOBJ_BODY, "nc90_drive")]
    cap_0280 = hull_verts(cad_model, mujoco.mj_name2id(cad_model, mujoco.mjtObj.mjOBJ_GEOM, "col_part_0280"))
    JAW_TRAVEL = 0.045

    def pad_points(slide):
        """The two TPU pads' mesh vertices in nc90_tool's frame, both jaws at `slide`."""
        dd = mujoco.MjData(cad_model)
        for j in ("nc90_jaw_left_slide", "nc90_jaw_right_slide"):
            dd.qpos[cad_model.jnt_qposadr[mujoco.mj_name2id(cad_model, mujoco.mjtObj.mjOBJ_JOINT, j)]] = slide
        mujoco.mj_kinematics(cad_model, dd)
        tb = mujoco.mj_name2id(cad_model, mujoco.mjtObj.mjOBJ_BODY, "nc90_tool")
        rot, at = np.asarray(dd.xmat[tb]).reshape(3, 3), np.asarray(dd.xpos[tb])
        out = []
        for pad in ("col_part_0316", "col_part_0317"):
            g = mujoco.mj_name2id(cad_model, mujoco.mjtObj.mjOBJ_GEOM, pad)
            mid = int(cad_model.geom_dataid[g])
            v = cad_model.mesh_vert[cad_model.mesh_vertadr[mid]:cad_model.mesh_vertadr[mid] + cad_model.mesh_vertnum[mid]]
            out.append(((v @ np.asarray(dd.geom_xmat[g]).reshape(3, 3).T + dd.geom_xpos[g]) - at) @ rot)
        return out

    pads_open, pads_closed = pad_points(0.0), pad_points(JAW_TRAVEL)   # a slide moves them along a line
    cad_model = None
    K._trim()
    piece = "col_part_0996_017" if "col_part_0996_017" in geoms else next(
        (n for n in sorted(geoms) if n and K.PIECE.match(n)), "")
    check("a CAD piece answers to its part: col_part_0996_017 -> col_part_0996, col_part_0277_envelope -> col_part_0277, "
          "col_part_0005 stays",
          piece in geoms and K.part_key(piece) == piece.rsplit("_", 1)[0]
          and K.part_key("col_part_0277_envelope") == "col_part_0277" and K.part_key("col_part_0005") == "col_part_0005"
          and K.part_key("tread_1_21") == "tread_1_21", f"{piece} -> {K.part_key(piece)}")
    carriage = K.display_name(names, "col_part_0996")
    check("--names: col_part_0996 shows as its CAD name; a name the exporter made up takes its component's; "
          "a shared name gets the part id",
          carriage == raw_names.get("part_0996") and "Lidar" in carriage
          and not K.EXPORTER_NAME.match(K.display_name(names, "col_part_0896"))
          and len(set(names.values())) == len(names) and names.get("part_0997", "").endswith("(part_0997)"),
          f"{carriage!r}, col_part_0896 -> {K.display_name(names, 'col_part_0896')!r}")
    check("the layout is told by the body names: cad here, sim for the simulation model",
          K.layout_of(Path(cad_path)) == "cad" and (not model_path or K.layout_of(Path(model_path)) == "sim"))
    oute = TMP / "cad.json"
    cal_args = ["--calibration", str(cal_path)] if calibration else []
    rc = K.main(["--model", cad_path, "--names", str(manifest), "--out", str(oute), "--steps", "8,8,8,6,4", "--quiet",
                 "--jobs", "1"] + cal_args)
    te = json.loads(oute.read_text()) if oute.exists() else {}
    check("the coarse sweep 8,8,8,6,4 exits 0 (its mj_forward and real-finger checks agree), no collision error",
          rc == 0 and te.get("collision_errors") == 0, f"rc {rc}, {te.get('collision_errors')} errors")
    hit = te.get("part_names", [])
    check("forbidden cells, and the lidar itself (RPLIDAR C1), its carriage, the bin and the upper cover among the "
          "parts hit",
          te.get("forbidden", 0) >= 1 and any(n.startswith("RPLIDAR C1") for n in hit)
          and any("Lidar dovetail carriage" in n for n in hit)
          and any("bin" in n for n in hit) and any("Upper cover" in n for n in hit),
          f"{te.get('forbidden')} of {te.get('cells')} ({100 * te.get('forbidden_fraction', 0):.1f} %); "
          + ", ".join(f"{n} {te['part_counts'][n]}" for n in hit[:5]))
    check("the arm's own base and the arm mounting plate are rover parts, and the arm hits them",
          any(n.startswith("SO101 base link") for n in hit) and any(n.startswith("Arm mounting plate") for n in hit))
    check("the CAD defaults: rover_exterior and base, arm from shoulder, every collision geom, the visual meshes left "
          "out, margin 2 mm, the base|shoulder exclusion kept",
          te.get("layout") == "cad" and te.get("rover_body") == "rover_exterior,base" and te.get("arm_root") == "shoulder"
          and te.get("rover_parts") == ["*"] and te.get("visual") == "skip" and te["visual_left_out"]["rover"] > 0
          and te["visual_left_out"]["arm"] > 0 and te.get("margin_m") == CAD_MARGIN
          and te.get("excludes_kept") == [["base", "shoulder"]], f"visual left out {te.get('visual_left_out')}")
    kept = te.get("visual_kept", {}).get("geoms", [])
    check("the gripper's drive (gear, servo horn: visual meshes only, no collision geom) counts: its visual meshes "
          "are arm geoms, swept over the pinion, named after their parts",
          drive_geoms and sorted(kept) == sorted(drive_geoms)
          and set(kept) <= set(te["unswept_joints"]["replaced_geoms"])
          and any(n.startswith("Gripper Gear") and n.endswith("(visual mesh)") for n in te.get("arm_geoms", [])),
          f"{kept}")
    n_rover = rover_collision_geoms(cad_path, also=kept)
    check("every rover collision geom of the model is accounted for (counted, held inside another, or left out by "
          "name), counted from the model apart from the tool",
          te.get("rover_geoms_counted", 0) > 0 and n_rover == rover_accounted(te),
          f"{n_rover} in the model = {te.get('rover_geoms_counted')} counted + {te['rover_inside']['geoms']} inside "
          f"+ {len(te['rover_left_out'])} left out + {te.get('rover_stand_in_meshes')} with stand-ins")
    text_e = oute.read_text() if oute.exists() else ""
    check("no CAD working code in the table (two capitals and two digits as a word): the gripper's parts read "
          "\"gripper\"", not WORK_CODE.search(text_e) and any(n.startswith("gripper ") for n in te.get("arm_geoms", [])),
          ", ".join(sorted(set(WORK_CODE.findall(text_e)))))
    out0 = TMP / "cad_margin0.json"
    rc0 = K.main(["--model", cad_path, "--out", str(out0), "--steps", "8,8,8,6,4", "--quiet", "--margin", "0",
                  "--check", "0", "--jobs", "1"] + cal_args)
    t0 = json.loads(out0.read_text()) if out0.exists() else {}
    occ_e = np.unpackbits(np.frombuffer(bytes.fromhex(te.get("occupancy_bits_hex", "")), np.uint8))
    occ_0 = np.unpackbits(np.frombuffer(bytes.fromhex(t0.get("occupancy_bits_hex", "")), np.uint8))
    gained = int((occ_e & ~occ_0 & 1).sum()) if occ_e.shape == occ_0.shape else -1
    check("the 2 mm margin forbids more than a 0 mm margin: cells where the arm comes within 2 mm of the rover "
          "without touching it are refused, every cell 0 mm forbids stays forbidden",
          rc0 == 0 and gained > 0 and occ_e.shape == occ_0.shape and not (occ_0 & ~occ_e & 1).any(),
          f"{gained} cells within 2 mm, not touching")
    if calibration:
        past = travel_outside(te, calibration)
        check("--calibration: the grid covers every joint's calibrated travel (through the joint_map) and its pad",
              te.get("travel") is not None and not past and all(
                  te["grid"][j][0] <= te["travel"]["joints"][j]["grid_rad"][0] + 1e-9
                  and te["grid"][j][-1] >= te["travel"]["joints"][j]["grid_rad"][1] - 1e-9 for j in te["joints"]),
              f"outside: {past}" if past else "every joint inside")
    else:
        print("  -- skipped, no arm calibration here (config/arm_calibration.json, MOSS_ARM_CALIBRATION)")
    lid = te.get("lidar", {})
    check("no lidar cylinder added: the scene carries the lidar as parts, and the note names them",
          lid.get("added") is False and "Lidar dovetail carriage R21" in lid.get("note", ""), lid.get("note", "")[:90])
    fing = te.get("fingers", {})
    check("fingers swept between closed and open; open = both slides at 0 (90 mm, the widest) and the pinion at 0, "
          "the right jaw and the pinion following the left jaw",
          fing.get("open") == {"nc90_jaw_left_slide": 0.0, "nc90_jaw_right_slide": 0.0, "nc90_pinion": 0.0}
          and fing.get("coupled", {}).get("nc90_jaw_right_slide", {}).get("follows") == "nc90_jaw_left_slide"
          and fing.get("coupled", {}).get("nc90_pinion", {}).get("follows") == "nc90_jaw_left_slide"
          and te["unswept_joints"]["replaced_geoms"] and te["finger_check"]["touching"] == 0, str(fing.get("open")))
    every = all(te["part_counts"].get(n, 0) > 0 or (te.get("payload") or {}).get("part_counts", {}).get(n, 0) > 0
                for n in hit) and len(hit) <= te.get("rover_parts_total", 0)
    check("the parts list is built from the hits (every listed part was hit, by the arm or by the can), one byte "
          "per cell while it is 255 parts or fewer, pairs named by part",
          every and te.get("cell_part_dtype") == ("uint8" if len(hit) <= 255 else "uint16")
          and all("|" in k for k in te.get("pair_counts", {})) and len(te.get("pair_cells", [])) == te.get("pairs"),
          f"{len(hit)} parts of {te.get('rover_parts_total')}, {te.get('pairs')} pairs")
    check("the JSON carries the joint_map (2048, +1 on all five joints) with its basis, the model's file name and "
          "sha256, and the manifest's",
          te.get("joint_map") == {j: {"model_zero_ticks": 2048, "sign": 1} for j in K.JOINTS}
          and "drive" in te.get("joint_map_basis", "") and te["model"]["path"] == Path(cad_path).name
          and te["model"]["sha256"] == K.sha256_of(Path(cad_path))
          and te.get("names", {}).get("sha256") == K.sha256_of(manifest))
    outk = TMP / "cad_keep.json"
    rc = K.main(["--model", cad_path, "--out", str(outk), "--steps", "8,8,8,6,4", "--quiet", "--keep-inside", "--check", "0",
                 "--jobs", "1"] + cal_args)
    tk = json.loads(outk.read_text()) if outk.exists() else {}
    check("the rover geoms held whole inside another add no forbidden pose: the same occupancy with --keep-inside",
          rc == 0 and te["rover_inside"]["geoms"] > 0 and tk.get("occupancy_bits_hex") == te["occupancy_bits_hex"],
          f"{te['rover_inside']['geoms']} geoms left out, {tk.get('forbidden')} vs {te.get('forbidden')} forbidden")
    # the sweeps first, then the models the checks below hold (one compiled scene is some 350 MB)
    outn = TMP / "cad_none.json"
    rcn = K.main(["--model", cad_path, "--names", str(manifest), "--out", str(outn), "--steps", "8,8,8,6,4", "--quiet",
                  "--jobs", "1", "--check", "0", "--payload", "none"] + cal_args)
    tn = json.loads(outn.read_text()) if outn.exists() else {}
    pce = K.prepare(Path(cad_path), arm_root="shoulder", rover_body="rover_exterior,base", rover_parts=("*",),
                    visual="skip", root_voxel=0.0, margin=te["margin_m"], root_margin=0.0, names=names,
                    payload=K.parse_payload("can"))
    gid = mujoco.mj_name2id(pce.model, mujoco.mjtObj.mjOBJ_GEOM, "col_part_0280")
    check("every collision mesh as its whole hull: the scene caps hulls at 128 vertices, the sweep lifts the cap",
          hull_verts(pce.model, gid) > 128 >= cap_0280 and te.get("hull_vertex_cap_lifted", 0) > 0,
          f"col_part_0280: {cap_0280} -> {hull_verts(pce.model, gid)} vertices")
    # the two poses in the model, at their exact angles (first alignment: 2048 and +1)
    jm0 = JointMap.from_docs(None, {"joints": list(K.JOINTS)})
    is_arm_e, is_part_e = K._masks(pce.model, pce.arm_ids, pce.part_ids)
    is_root_e, _ = K._masks(pce.root_model, pce.root_ids, pce.part_ids)
    is_pay_e, _ = K._masks(pce.payload_model, list(pce.payload_ids.values()), pce.part_ids)
    part_name_e = dict(zip(pce.part_ids, [pce.display[k] for k in pce.part_label]))

    def touching(q, models):
        """The (arm geom, rover part name) contacts closer than the margin at q, in each model."""
        out = []
        for mdl, isa, mg, base in models:
            dd = mujoco.MjData(mdl)
            dd.qpos[:] = base
            for j, v in q.items():
                dd.qpos[mdl.jnt_qposadr[mujoco.mj_name2id(mdl, mujoco.mjtObj.mjOBJ_JOINT, j)]] = v
            out += [(a, part_name_e[r]) for a, r, _ in (K._touch(mdl, dd, isa, is_part_e, mg) or [])]
        return out

    EMPTY_E = ((pce.model, is_arm_e, te["margin_m"], pce.base_qpos), (pce.root_model, is_root_e, 1e-12, pce.base_qpos))
    CAN_E = ((pce.payload_model, is_pay_e, te["margin_m"], pce.payload_qpos),)
    free_at = {name: (not touching(jm0.pose_to_rad(t), EMPTY_E), not touching(jm0.pose_to_rad(t), CAN_E))
               for name, t in (("drive", DRIVE), ("look", LOOK))}
    check("drive and look, at their exact angles, come no closer than the margin to the rover (the model), the can "
          "held either way too", all(all(v) for v in free_at.values()), f"(empty, can): {free_at}")
    pay = te.get("payload") or {}
    tool = mujoco.mj_name2id(pce.payload_model, mujoco.mjtObj.mjOBJ_BODY, "nc90_tool")
    can_geoms = [(o, g) for o, g in pce.payload_ids.items()]
    check("the can: two cylinders 66 mm across and 120 mm long on nc90_tool (upright, across), the jaws at 12 mm "
          "each (66 mm apart)",
          [o for o, _ in can_geoms] == ["upright", "across"]
          and all(int(pce.payload_model.geom_bodyid[g]) == tool
                  and int(pce.payload_model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER)
                  and np.allclose(pce.payload_model.geom_size[g][:2], [0.033, 0.060]) for _, g in can_geoms)
          and abs(pay.get("fingers", {}).get("nc90_jaw_left_slide", 0) - 0.012) < 1e-6
          and abs(pay.get("fingers", {}).get("nc90_jaw_right_slide", 0) - 0.012) < 1e-6
          and abs(pay.get("gap_m", 0) - 0.066) < 1e-6,
          f"fingers {pay.get('fingers')}, gap {pay.get('gap_m')}")

    def pad_contact(slide, pose, grow):
        """Does the can (radius grown by `grow`, at pose in nc90_tool's frame) reach each TPU pad, the jaws at
        `slide`? Built apart from the tool: the scene's own pad meshes and a cylinder, nothing else."""
        sp = mujoco.MjSpec()
        sp.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
        body = sp.worldbody.add_body(name="b")
        for k, (v0, v1) in enumerate(zip(pads_open, pads_closed)):
            mesh = sp.add_mesh(name=f"pad{k}", uservert=(v0 + (slide / JAW_TRAVEL) * (v1 - v0)).ravel().tolist())
            mesh.maxhullvert = -1
            body.add_geom(name=f"pad{k}", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"pad{k}")
        body.add_geom(name="can", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.033 + grow, 0.060, 0],
                      pos=pose["pos_m"], quat=pose["quat"])
        mm = sp.compile()
        md = mujoco.MjData(mm)
        mujoco.mj_kinematics(mm, md)
        can = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_GEOM, "can")
        return [mujoco.mj_geomDistance(mm, md, can, mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_GEOM, f"pad{k}"),
                                       0.0, None) < 0 for k in (0, 1)]

    hold = pay.get("fingers", {}).get("nc90_jaw_left_slide", 0.012)
    pads_ok = {o: (pad_contact(hold, q, 0.001), pad_contact(hold, q, -0.001)) for o, q in pay.get("pose", {}).items()}
    check("both TPU pads within 1 mm of the can's surface, either way: 1 mm wider it goes into both, 1 mm narrower "
          "it touches neither (the scene's pad meshes, apart from the tool)",
          set(pads_ok) == {"upright", "across"} and all(g == [True, True] and s == [False, False]
                                                       for g, s in pads_ok.values())
          and sorted(pay.get("pads", [])) == sorted(K.display_name(names, p) for p in ("col_part_0316", "col_part_0317")),
          f"(wider, narrower): {pads_ok}")
    pu, pa = pay.get("pose", {}).get("upright", {}), pay.get("pose", {}).get("across", {})
    check("upright: the can along the approach (-z of the tool), pushed out until it clears the gripper's rails "
          "(centre 67.5 mm out); across: along x, its centre among the pads (18 to 54 mm out)",
          pay.get("approach_axis") == [0.0, 0.0, -1.0] and np.allclose(np.abs(pay.get("slide_axis", [0] * 3)), [0, 1, 0])
          and np.allclose(pu.get("pos_m", [9] * 3), [0, 0, -0.0675], atol=5e-4)
          and np.allclose(np.abs(pu.get("axis", [0] * 3)), [0, 0, 1]) and "rails" in (pu.get("pushed_out_by") or "")
          and np.allclose(pa.get("pos_m", [9] * 3)[:2], [0, 0], atol=1e-6) and -0.054 < pa.get("pos_m", [0] * 3)[2] < -0.018
          and np.allclose(np.abs(pa.get("axis", [0] * 3)), [1, 0, 0]),
          f"upright {pu.get('pos_m')} by {pu.get('pushed_out_by')}, across {pa.get('pos_m')} by {pa.get('pushed_out_by')}")
    bits_e = np.unpackbits(np.frombuffer(bytes.fromhex(te.get("occupancy_empty_bits_hex", "")), np.uint8))
    bits_n = np.unpackbits(np.frombuffer(bytes.fromhex(tn.get("occupancy_bits_hex", "")), np.uint8))
    bits_c = np.unpackbits(np.frombuffer(bytes.fromhex(te.get("occupancy_bits_hex", "")), np.uint8))
    pf = pay.get("forbidden", {})
    check("--payload none: the empty gripper's table, the same cells the can's table records as its empty sweep; "
          "with the can the table forbids every one of them and more; the counts add up",
          rcn == 0 and tn.get("payload") is None and bits_n.shape == bits_e.shape == bits_c.shape
          and np.array_equal(bits_n, bits_e) and not (bits_n & ~bits_c & 1).any()
          and te.get("forbidden", 0) > tn.get("forbidden", 0) == pf.get("empty")
          and pf.get("union") == te.get("forbidden") and pay.get("added", {}).get("total") == pf["union"] - pf["empty"]
          and min(pf.get("upright", 0), pf.get("across", 0)) > 0,
          f"{tn.get('forbidden')} without the can, {te.get('forbidden')} with it; {pf}")
    # the gripper over the bin's rim (grid values of the default table, inside the arm's calibrated travel)
    BIN_RIM = {"shoulder_pan": -1.834818282, "shoulder_lift": -0.895935705, "elbow_flex": 1.118719755,
               "wrist_flex": 0.140475568, "wrist_roll": -2.082015953}
    rim_empty, rim_can = touching(BIN_RIM, EMPTY_E), touching(BIN_RIM, CAN_E)
    pm_ = pce.payload_model
    dd = mujoco.MjData(pm_)
    dd.qpos[:] = pce.payload_qpos
    for j, v in BIN_RIM.items():
        dd.qpos[pm_.jnt_qposadr[mujoco.mj_name2id(pm_, mujoco.mjtObj.mjOBJ_JOINT, j)]] = v
    mujoco.mj_kinematics(pm_, dd)
    flags = int(pm_.opt.disableflags)
    pm_.opt.disableflags = flags & ~int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)   # the native collider's depth
    try:
        depth = min(mujoco.mj_geomDistance(pm_, dd, pce.payload_ids["upright"], g, 0.01, None)
                    for g in pce.part_ids if "bin" in part_name_e[g])
    finally:
        pm_.opt.disableflags = flags
    check("the gripper over the bin's rim (pan -105.1, lift -51.3, elbow +64.1, flex +8.0, roll -119.3 deg): clear of "
          "the rover by the margin with the gripper empty, the can held upright 20 mm or more into the bin",
          not rim_empty and rim_can and all("bin" in r for _, r in rim_can) and depth < -0.02,
          f"empty: {len(rim_empty)} contacts, can: {sorted({r for _, r in rim_can})}, {1000 * depth:.1f} mm")

    def arm_part(a):
        return pce.display.get(K.part_key(K.geom_name(pce.model, a)), "")

    study = {}
    for posture, held in STUDY.items():
        study[posture] = None
        for deg in range(110, -111, -1):
            q = dict(held, shoulder_pan=math.radians(deg))
            arm = touching(q, EMPTY_E)
            can = [(a, r) for a, r in touching(q, CAN_E) if a == pce.payload_ids["upright"]]
            if arm or can:
                study[posture] = (deg, sorted({arm_part(a) for a, _ in arm}), sorted({r for _, r in arm}),
                                  sorted({r for _, r in can}))
                break
    tr, pk, cl = study["travel"], study["pick"], study["clear"]
    check("the independent study's postures, the pan turned from +110 deg down: travel first touches between -50 "
          "and -35 deg (the study: -46 with the jaws at the can's gap; here the jaws swept fully open and the 2 mm "
          "margin come sooner), the left jaw or its TPU pad against the bin; pick within 3 deg of -45, the can "
          "upright against the left road-wheel shaft or collar while the arm is clear; clear touches nothing",
          tr is not None and -50 <= tr[0] <= -35 and any("bin" in r for r in tr[2])
          and any("jaw left" in a or "TPU retaining pad" in a for a in tr[1])
          and pk is not None and abs(pk[0] + 45) <= 3 and not pk[1] and any("road wheel" in r for r in pk[3])
          and cl is None,
          f"travel {tr[0] if tr else None} ({', '.join(tr[1]) if tr else ''}), pick {pk[0] if pk else None} "
          f"({', '.join(pk[3]) if pk else ''}), clear {cl}")
    pce = pm_ = dd = EMPTY_E = CAN_E = None      # every reference to its models: F compiles its own
    K._trim()

print("F. the committed table")
table_path = ROOT / "config" / "arm_keepout.json"
if not table_path.is_file():
    check("config/arm_keepout.json exists", False)
else:
    size = table_path.stat().st_size
    tc = json.loads(table_path.read_text())
    gc = ArmGuard.load(table_path)
    check("it loads (format, bits and cell_part agree), 16x16x16x12x8, under 2 MB",
          gc.shape == (16, 16, 16, 12, 8) and size < 2_000_000,
          f"{size / 1e6:.2f} MB, {100 * tc['forbidden_fraction']:.1f} % forbidden, model {tc['model']['path']}")
    print(f"  {gc.summary()}")
    homes = ([Path(model_path).parent] if model_path else []) + ([Path(cad_path).parent] if cad_path else [])
    sibling = next((h / tc["model"]["path"] for h in homes if (h / tc["model"]["path"]).is_file()), None)
    layout = tc.get("layout", "sim")
    if layout == "cad":
        gt0 = ArmGuard.load(table_path, calibration=NOWHERE)
        refused = {posture: {deg for deg in range(110, -111, -1)
                             if not gt0.allowed(dict(held, shoulder_pan=math.radians(deg)))}
                   for posture, held in STUDY.items()}
        check("the independent study's postures in the table: clear allowed at every pan; travel and pick refused "
              "wherever the study found a contact (travel -46 to -94 deg, pick -45 to -110)",
              not refused["clear"] and all(set(range(lo, hi - 1, -1)) <= refused[p]
                                           for p, (lo, hi) in STUDY_CONTACTS.items()),
              ", ".join(f"{p} refused at {len(r)} of 221 pans" for p, r in refused.items()))
        check("the robot's two daily poses are allowed: drive and look (servo ticks, first alignment)",
              gt0.allowed_ticks(DRIVE) and gt0.allowed_ticks(LOOK), f"{gt0.why_ticks(DRIVE)[:40]} | {gt0.why_ticks(LOOK)[:40]}")
        check("the table names its parts by their CAD names, a list built from the hits, one byte per cell",
              tc.get("names") is not None and tc.get("cell_part_dtype") == "uint8"
              and all(tc["part_counts"].get(n, 0) > 0 or (tc.get("payload") or {}).get("part_counts", {}).get(n, 0) > 0
                      for n in tc["part_names"]) and "basis" not in tc["joint_map"]
              and "drive" in tc.get("joint_map_basis", ""), f"{len(tc['part_names'])} parts, {tc.get('pairs')} pairs")
        text_c = table_path.read_text()
        tp = tc.get("payload") or {}
        occ_ce = np.unpackbits(np.frombuffer(bytes.fromhex(tc.get("occupancy_empty_bits_hex", "")), np.uint8))
        occ_ce = occ_ce[:tc["cells"]].astype(bool).reshape(gc.occ.shape) if len(occ_ce) >= tc["cells"] else None
        rim = {"shoulder_pan": -1.834818282, "shoulder_lift": -0.895935705, "elbow_flex": 1.118719755,
               "wrist_flex": 0.140475568, "wrist_roll": -2.082015953}
        rim_idx = tuple(int(np.argmin(np.abs(g - rim[j]))) for j, g in zip(gc.joints, gc.grids))
        check("the payload: a can (66 x 120 mm) held upright and across, the empty gripper's cells all inside the "
              "table's; the gripper over the bin's rim is free for the empty gripper and refused with the can, why() "
              "naming the bin",
              tp.get("kind") == "can" and tp.get("orientations") == ["upright", "across"]
              and tp.get("diameter_m") == 0.066 and tp.get("height_m") == 0.12 and occ_ce is not None
              and not (occ_ce & ~gc.occ).any() and tp.get("forbidden", {}).get("union") == tc["forbidden"]
              and not occ_ce[rim_idx] and not gc.allowed(rim) and "bin" in gc.why(rim),
              f"{tp.get('forbidden')}; at the rim: {gc.why(rim)[:70]}")
        check("margin 2 mm; the lidar itself (RPLIDAR C1) among the parts hit; the model named by its file name only; "
              "no CAD working code anywhere in the table",
              tc.get("margin_m") == CAD_MARGIN and any(n.startswith("RPLIDAR C1") for n in tc["part_names"])
              and "/" not in tc["model"]["path"] and not WORK_CODE.search(text_c),
              f"margin {tc.get('margin_m')}, model {tc['model']['path']!r}"
              + (", codes " + ", ".join(sorted(set(WORK_CODE.findall(text_c)))) if WORK_CODE.search(text_c) else ""))
    if calibration:
        past = travel_outside(tc, calibration)
        check("every joint's calibrated travel (the arm's range_min..range_max, through the table's joint_map) lies "
              "inside the table: the guard refuses no pose the arm can reach for being outside it",
              not past, f"outside: {past}" if past else ", ".join(
                  f"{j} {math.degrees(tc['grid'][j][0]):.1f}..{math.degrees(tc['grid'][j][-1]):.1f}" for j in tc["joints"]))
    else:
        print("  -- skipped, no arm calibration here (config/arm_calibration.json, MOSS_ARM_CALIBRATION)")
    if sibling is None or not sibling.is_file():
        print("  -- skipped, the model the table names is not here")
    else:
        si = tc["stand_ins"]
        names_doc = sibling.parent / tc["names"]["file"] if tc.get("names") else None
        common = dict(arm_root=tc["arm_root"], rover_body=tc["rover_body"], rover_parts=tuple(tc["rover_parts"]),
                      exclude=tuple(tc["exclude"]), margin=tc["margin_m"],
                      lidar_xyz=tc["lidar"]["pos_m"] if tc["lidar"].get("added") else None, joints=tuple(tc["joints"]),
                      fixed=tc["unswept_joints"]["fixed"], voxel=si["voxel_m"], root_voxel=tc["root_link"]["voxel_m"],
                      root_margin=tc["root_link"]["margin_m"], cover_tol=si["cover_tol_m"],
                      visual=tc.get("visual", "count"),
                      names=K.load_names(names_doc) if names_doc and names_doc.is_file() else None)
        tpay = tc.get("payload")
        pc = K.prepare(sibling, **common,
                       payload={k: tpay[k] for k in ("kind", "diameter_m", "height_m")} if tpay else None)
        check("both hashes match the model it names, here (its file, and its compiled geometry: meshes included)",
              K.sha256_of(sibling) == tc["model"]["sha256"] and pc.geometry_sha256 == tc["model"]["geometry_sha256"])
        jids = [mujoco.mj_name2id(pc.model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in tc["joints"]]
        grids = [np.array(tc["grid"][j]) for j in tc["joints"]]
        occ_c = gc.occ.ravel().astype(np.uint8)
        bad = K.recheck(pc, jids, grids, tc["margin_m"], occ_c, 200, seed=7,
                        step_rad=math.radians(tc["root_link"]["step_deg"]))
        check("200 of its cells (half forbidden, half free) re-done on that model with mj_forward agree"
              + (" (the can held either way included)" if tpay else ""), not bad, f"{len(bad)} differ")
        if tpay:
            occ_e = np.unpackbits(np.frombuffer(bytes.fromhex(tc["occupancy_empty_bits_hex"]), np.uint8))[:tc["cells"]]
            bad_e = K.recheck(pc, jids, grids, tc["margin_m"], occ_e, 100, seed=8,
                              step_rad=math.radians(tc["root_link"]["step_deg"]), with_payload=False)
            check("100 of the empty gripper's own cells re-done with mj_forward agree", not bad_e, f"{len(bad_e)} differ")
        pc.model = pc.root_model = pc.payload_model = None
        K._trim()
        real = K.prepare(sibling, **common, fingers="real", stand_ins=pc.stand_ins)
        done, touched = K.finger_check(real, jids, grids, tc["margin_m"], occ_c, 100, seed=9)
        real = None
        K._trim()
        check("100 free cells stay free with the real fingers closed, half open, open and at a random opening",
              done == 400 and not touched, f"{done} poses, {len(touched)} touch")
        pm_t = si["per_mesh"]
        silent = [n for n, r in pm_t.items() if r.get("outside_fraction", 0) > 0 and not r.get("boxes")]
        if layout == "sim":
            check("every rover mesh the named parts miss has stand-ins (none dropped silently)",
                  not silent and len(pm_t) > 0, f"{len(pm_t)} meshes, {sum(r.get('boxes', 0) for r in pm_t.values())} boxes")
        else:
            n_rover_c = rover_collision_geoms(sibling, also=tc.get("visual_kept", {}).get("geoms", []))
            check("every rover collision geom counts as it is: none left out by name, no stand-ins needed, every one "
                  "of the model's accounted for (counted apart from the tool), the names on the table match the "
                  "model's parts",
                  not tc["rover_left_out"] and not pm_t and n_rover_c == rover_accounted(tc)
                  and set(tc["part_names"]) <= set(pc.display.values()) | {K.COLLISION_ERROR},
                  f"{n_rover_c} rover geoms in the model = {tc.get('rover_geoms_counted')} counted + "
                  f"{tc['rover_inside']['geoms']} held inside another; {tc['rover_parts_total']} parts")

        def snap(q):
            return {j: float(g[np.argmin(np.abs(g - q[j]))]) for j, g in zip(gc.joints, gc.grids)}

        if layout == "sim":
            poses = [{"shoulder_pan": 0.419, "shoulder_lift": 0.128, "elbow_flex": 1.865, "wrist_flex": -1.166, "wrist_roll": 1.32},
                     {"shoulder_pan": 0.419, "shoulder_lift": -0.384, "elbow_flex": 1.119, "wrist_flex": 0.833, "wrist_roll": -2.071}]
            why = [gc.why(snap(q)) for q in poses]
            check("the two grid poses a review found inside the nose camera (free in the earlier table) are refused",
                  all(not gc.allowed(snap(q)) for q in poses), " | ".join(w[:60] for w in why))
        lo = np.array([g[0] for g in gc.grids])
        hi = np.array([g[-1] for g in gc.grids])
        n_ref, n_bad = 0, 0
        for _ in range(3000):
            q = dict(zip(gc.joints, rng.uniform(lo, hi)))
            if gc.allowed(q):
                continue
            n_ref += 1
            near = gc.nearest_allowed(q)
            n_bad += not (gc.allowed(near) and gc.allowed_ticks(gc.joint_map.pose_to_ticks(near)))
            if n_ref >= 500:
                break
        check("nearest_allowed -> ticks -> allowed_ticks holds for 500 refused poses", n_ref == 500 and n_bad == 0,
              f"{n_bad} refused after the rounding")
        free_q = [q for q in (dict(zip(gc.joints, rng.uniform(lo, hi))) for _ in range(3000)) if gc.allowed(q)][:301]
        seg_bad = sum(1 for a, b in zip(free_q[:-1], free_q[1:]) if gc.segment_allowed(a, b) and not fine(gc, a, b, 64))
        check("segment_allowed never allows a move that a 1/64-step sampling refuses",
              seg_bad == 0 and len(free_q) > 100, f"{len(free_q) - 1} moves")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
_TMP.cleanup()
sys.exit(1 if KO else 0)
