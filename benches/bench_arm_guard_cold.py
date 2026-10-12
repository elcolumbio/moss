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
     to it) at --margin 0.03 finishes
  E. the committed table (config/arm_keepout.json) - it loads, 16x16x16x12x8,
     under 2 MB; when the model it names is next to D's: both hashes match,
     200 cells re-done with mj_forward agree, free cells stay free with the
     real fingers, every rover mesh the named parts miss has stand-ins, the
     camera poses a review found are refused, nearest_allowed survives the
     tick rounding and segment_allowed agrees with a fine sampling

Needs mujoco (the sweep). Without it: one line "skipped: mujoco not installed",
exit 0.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_arm_guard_cold.py
"""
import atexit
import glob
import itertools
import json
import math
import os
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
    rc = K.main(["--model", model_path, "--out", str(outd), "--steps", "8,8,8,6,4", "--quiet"])
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
    robot = Path(model_path).parent / "moss_robot.xml"
    if robot.is_file():
        outm = TMP / "margin.json"
        rc = K.main(["--model", str(robot), "--out", str(outm), "--steps", "8,8,8,6,4", "--margin", "0.03", "--quiet"])
        tm3 = json.loads(outm.read_text()) if outm.exists() else {}
        check("the robot model at --margin 0.03 finishes and writes its table (collision errors, if any, counted)",
              rc == 0 and tm3.get("margin_m") == 0.03, f"rc {rc}, {tm3.get('collision_errors')} collision errors, "
              f"{100 * tm3.get('forbidden_fraction', 0):.1f} % forbidden")

print("E. the committed table")
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
    sibling = Path(model_path).parent / tc["model"]["path"] if model_path else None
    if sibling is None or not sibling.is_file():
        print("  -- skipped, the model the table names is not here")
    else:
        si = tc["stand_ins"]
        common = dict(arm_root=tc["arm_root"], rover_body=tc["rover_body"], rover_parts=tuple(tc["rover_parts"]),
                      exclude=tuple(tc["exclude"]), margin=tc["margin_m"],
                      lidar_xyz=tc["lidar"]["pos_m"] if tc["lidar"].get("added") else None, joints=tuple(tc["joints"]),
                      fixed=tc["unswept_joints"]["fixed"], voxel=si["voxel_m"], root_voxel=tc["root_link"]["voxel_m"],
                      root_margin=tc["root_link"]["margin_m"], cover_tol=si["cover_tol_m"])
        pc = K.prepare(sibling, **common)
        check("both hashes match the model next to D's (its file, and its compiled geometry: meshes included)",
              K.sha256_of(sibling) == tc["model"]["sha256"] and pc.geometry_sha256 == tc["model"]["geometry_sha256"])
        jids = [mujoco.mj_name2id(pc.model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in tc["joints"]]
        grids = [np.array(tc["grid"][j]) for j in tc["joints"]]
        occ_c = gc.occ.ravel().astype(np.uint8)
        bad = K.recheck(pc, jids, grids, tc["margin_m"], occ_c, 200, seed=7,
                        step_rad=math.radians(tc["root_link"]["step_deg"]))
        check("200 of its cells (half forbidden, half free) re-done on that model with mj_forward agree",
              not bad, f"{len(bad)} differ")
        real = K.prepare(sibling, **common, fingers="real", stand_ins=pc.stand_ins)
        done, touched = K.finger_check(real, jids, grids, tc["margin_m"], occ_c, 100, seed=9)
        check("100 free cells stay free with the real fingers closed, half open, open and at a random opening",
              done == 400 and not touched, f"{done} poses, {len(touched)} touch")
        pm_t = si["per_mesh"]
        silent = [n for n, r in pm_t.items() if r.get("outside_fraction", 0) > 0 and not r.get("boxes")]
        check("every rover mesh the named parts miss has stand-ins (none dropped silently)",
              not silent and len(pm_t) > 0, f"{len(pm_t)} meshes, {sum(r.get('boxes', 0) for r in pm_t.values())} boxes")

        def snap(q):
            return {j: float(g[np.argmin(np.abs(g - q[j]))]) for j, g in zip(gc.joints, gc.grids)}

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
