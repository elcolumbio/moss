"""Cold bench for moss_dimos.arm: the arm driven by dimOS in the MuJoCo
simulation of the CAD scene, one process, no robot.

Known input -> known output in physical units (degrees, millimetres,
seconds). Groups:

  A. units      - ticks <-> model radians (2048 = 0, 2857 = +71.1 deg), the
                  simulator's gripper command and read-back (90 mm open ->
                  command 0 / slide 45 mm; 53 mm -> command 18.5 mm)
  B. scene      - the CAD scene is found; the simulation scene written from it
                  loads, its mesh directory is absolute, the arm joints' ranges
                  are the calibration's measured travel (shoulder_lift
                  +-104.4 deg where the delivered scene says +-100) and the
                  drive pose sits inside them, the three self-contact
                  exclusions are in, the twelve bodies are the CAD's; at the
                  drive pose the simulation scene has no contact where the
                  delivered scene has its folded-arm self-contacts
  C. cycle      - config/arm_drop_poses.json as delivered: ten moves, the
                  phases in order, as timed 36.9 s, stretched to 400 ticks/s
                  54.6 s (the same numbers tools/moss_arm_drop.py plan prints);
                  the trajectory: six joints, t = 0 first, strictly increasing
                  times, 50 points per second, the gap 90 -> 53 mm over the
                  grasp and back over the release, every value finite
  D. live       - the sim module (headless) and the coordinator with the
                  sim_mujoco adapter and the trajectory task in this process:
                  the jaws start closed and stay so until a command;
                  (1) the pan +30 deg over 2 s reads +30 deg, then back, and
                  that first trajectory opens the jaws to 90 mm;
                  (2) the gripper commanded to 53 mm measures 53 mm between the
                  jaws in the model, 90 mm again when opened;
                  (3) the whole cycle at the ceiling, from the drive pose: the
                  arm ends at the drive pose, no arm-rover contact at any
                  sample, the following error stays small, the run takes the
                  stretched duration
  E. resources  - resident memory under 4 GB, the simulator's step cost

Run (the dimOS main venv; MuJoCo and dimOS's sim module):
    nice -n 10 <dimos venv>/bin/python benches/bench_moss_arm_sim_cold.py
Env: MOSS_ARM_CAD (a MOSS_R31.xml or its directory) when the CAD is not at
~/moss-*/cad/*/MOSS_R31.xml; MOSS_ARM_CALIB for a calibration file elsewhere.
"""

from __future__ import annotations

import math
import os
import resource
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

if os.nice(0) < 10:
    os.nice(10 - os.nice(0))

from moss_dimos import arm as A  # noqa: E402

FAIL: list[str] = []
OK = 0
DEG = 180.0 / math.pi


def check(cond: bool, what: str) -> None:
    global OK
    if cond:
        OK += 1
        print(f"  ok   {what}")
    else:
        FAIL.append(what)
        print(f"  FAIL {what}")


def rss_gb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6


# ── A. units ──
print("A. units")
check(abs(A.ticks_to_rad(2048)) < 1e-12, "2048 ticks = 0 rad")
check(abs(A.ticks_to_rad(2857) * DEG - 71.1) < 0.05, f"2857 ticks = +71.1 deg ({A.ticks_to_rad(2857) * DEG:.2f})")
check(A.rad_to_ticks(A.ticks_to_rad(860)) == 860, "ticks -> rad -> ticks round trip at 860")
check(abs(A.sim_gripper_command(0.090) - 0.0) < 1e-12, "90 mm open -> sim command 0")
check(abs(A.sim_gripper_command(0.053) - 0.0185) < 1e-9, f"53 mm -> sim command 18.5 mm ({A.sim_gripper_command(0.053) * 1000:.2f})")
check(abs(A.sim_gripper_command(0.0) - 0.045) < 1e-12, "0 mm closed -> sim command 45 mm")
check(abs(A.sim_gripper_gap(0.0) - 0.090) < 1e-12 and abs(A.sim_gripper_gap(0.0185) - 0.053) < 1e-12,
      "read back 0 -> gap 90 mm, 18.5 mm -> 53 mm (the read-back is in the command's convention)")
check(all(abs(A.sim_gripper_gap(A.sim_gripper_command(g)) - g) < 1e-12 for g in (0.0, 0.03, 0.053, 0.09)),
      "gap -> command -> read-back -> gap round trip")
check(abs(A.velocity_limits()["arm/shoulder_pan"] - 400 / A.TICKS_PER_RAD) < 1e-12 and
      abs(A.velocity_limits()["arm/shoulder_pan"] * DEG - 35.16) < 0.01,
      "the task's ceiling on an arm joint = 400 ticks/s = 35.2 deg/s")

# ── B. scene ──
print("B. scene")
cad = A.find_cad()
check(cad is not None, f"CAD scene found: {cad}")
if cad is None:
    print("no CAD scene: B, D and E need it (MOSS_ARM_CAD)")
    sys.exit(2)
import mujoco  # noqa: E402

calib = os.environ.get("MOSS_ARM_CALIB") or None
t0 = time.time()
scene = A.sim_scene(cad, out_dir=Path(tempfile.mkdtemp(prefix="moss_arm_sim_")), calibration=calib)
check(scene.is_file() and scene.name == A.SIM_SCENE_NAME, f"simulation scene written: {scene}")
m_sim = mujoco.MjModel.from_xml_path(str(scene))
load_s = time.time() - t0
check(m_sim.nbody == 12, f"twelve bodies in the simulation scene (load + write {load_s:.1f} s)")
import xml.etree.ElementTree as ET  # noqa: E402

root = ET.parse(scene).getroot()
check(Path(root.find("compiler").get("meshdir")).is_absolute(), "mesh directory absolute")
excl = {(e.get("body1"), e.get("body2")) for e in root.find("contact").findall("exclude")}
check(all(p in excl for p in A.SIM_SELF_CONTACT_EXCLUDES), f"the three self-contact exclusions are in ({len(excl)} exclusions)")
names = [mujoco.mj_id2name(m_sim, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m_sim.nbody)]
check(all(b in names for b in (*A.ARM_BODIES, *A.ROVER_BODIES)), "arm and rover bodies named as expected")
m_cad = mujoco.MjModel.from_xml_path(str(cad))
drive = A.drive_pose_rad(calib)
ranges = A.calibrated_ranges_rad(calib)
check(bool(ranges), f"calibration found: {len(ranges)} joint travels" + ("" if ranges else " (set MOSS_ARM_CALIB)"))
j_lift = mujoco.mj_name2id(m_sim, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_lift")
lo, hi = m_sim.jnt_range[j_lift] * DEG
pad = A.SIM_RANGE_PAD_TICKS * 360.0 / A.TICKS_PER_REV
check(abs(lo + 104.4 + pad) < 0.02 and abs(hi - 104.4 - pad) < 0.02,
      f"simulation scene: shoulder_lift range {lo:+.2f}..{hi:+.2f} deg = the measured +-104.4 and a tick of slack (the delivered scene: +-100)")
a_lift = mujoco.mj_name2id(m_sim, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_lift")
check(abs(m_sim.actuator_ctrlrange[a_lift][0] - m_sim.jnt_range[j_lift][0]) < 1e-9, "its actuator's control range follows")
a_jaw = mujoco.mj_name2id(m_sim, mujoco.mjtObj.mjOBJ_ACTUATOR, A.SIM_GRIPPER_ACTUATOR)
check(abs(m_sim.actuator_gainprm[a_jaw][0] - A.SIM_GRIPPER_KP) < 1e-6 and abs(m_sim.actuator_forcerange[a_jaw][1] - A.SIM_GRIPPER_FORCE_N) < 1e-9,
      f"the jaw actuator stiffened: kp {m_sim.actuator_gainprm[a_jaw][0]:g}, force +-{m_sim.actuator_forcerange[a_jaw][1]:g} N (delivered: 50, +-10)")
check(all(m_sim.jnt_range[mujoco.mj_name2id(m_sim, mujoco.mjtObj.mjOBJ_JOINT, j)][0] <= q <= m_sim.jnt_range[mujoco.mj_name2id(m_sim, mujoco.mjtObj.mjOBJ_JOINT, j)][1]
          for j, q in zip(A.ARM_JOINTS, drive)), "the drive pose sits inside the simulation scene's ranges")


def contacts_at(model, q, slide=0.045):
    d = mujoco.MjData(model)
    d.qpos[:] = 0.0
    d.qpos[:5] = q
    d.qpos[5] = d.qpos[6] = slide
    mujoco.mj_forward(model, d)
    return [c for c in d.contact[: d.ncon] if c.dist < 0]


raw = contacts_at(m_cad, drive)
sim_c = contacts_at(m_sim, drive)
check(len(raw) > 0 and len(sim_c) == 0,
      f"drive pose: {len(raw)} penetrating self-contacts in the delivered scene, {len(sim_c)} in the simulation scene")

# ── C. cycle ──
print("C. cycle")
cyc = A.load_cycle(calibration=calib)
phases = [s.phase for s in cyc.steps]
check(len(cyc.steps) == 10, f"ten moves ({len(cyc.steps)})")
check(phases == ["leave_drive", "approach", "grasp", "lift", "clear", "transfer", "release", "back_to_clear", "hold", "to_drive"],
      f"phases in order: {' '.join(phases)}")
check(abs(cyc.timed_s - 36.9) < 0.05, f"as timed {cyc.timed_s:.1f} s")
check(abs(cyc.duration_s - 54.6) < 0.1, f"stretched to 400 ticks/s {cyc.duration_s:.1f} s")
check(cyc.steps[0].start == "drive" and cyc.steps[-1].end == "drive", "from drive, back to drive")
check(all(abs(a - b) < 1e-12 for a, b in zip(cyc.first_pose, drive)), "the first pose is the drive pose")
grasp = next(s for s in cyc.steps if s.phase == "grasp")
check(abs(grasp.gap_from_m - 0.090) < 1e-9 and abs(grasp.gap_to_m - 0.053) < 1e-9 and grasp.duration_s == 1.0,
      "grasp: 90 -> 53 mm in 1 s, the arm still")
fastest = max(cyc.steps, key=lambda s: s.peak_ticks_per_s)
check(abs(fastest.peak_ticks_per_s - 1810) < 5, f"the sequence's own peak {fastest.peak_ticks_per_s:.0f} ticks/s on {fastest.phase}")
traj = A.cycle_trajectory(cyc.steps, sim=True)
pts = traj.points
check(traj.joint_names == A.JOINT_NAMES and all(len(p.positions) == 6 for p in pts), "six joints per point")
check(pts[0].time_from_start == 0.0 and all(b.time_from_start > a.time_from_start for a, b in zip(pts[:-1], pts[1:])),
      f"t = 0 first, strictly increasing ({len(pts)} points)")
check(abs(traj.duration - cyc.duration_s) < 1e-6, f"trajectory duration {traj.duration:.2f} s")
check(len(pts) / traj.duration > 45, f"{len(pts) / traj.duration:.0f} points per second")
check(all(math.isfinite(v) for p in pts for v in (*p.positions, *p.velocities)), "every value finite")
t_grasp_end = sum(s.duration_s for s in cyc.steps[:3])
g_before = A.sim_gripper_gap(A.sample(traj, t_grasp_end - 1.0)[5])
g_after = A.sim_gripper_gap(A.sample(traj, t_grasp_end)[5])
check(abs(g_before - 0.090) < 1e-6 and abs(g_after - 0.053) < 1e-6,
      f"gap in the trajectory: {g_before * 1000:.0f} mm before the grasp, {g_after * 1000:.0f} mm after")
real = A.cycle_trajectory(cyc.steps, sim=False)
check(abs(real.points[0].positions[5] - 0.090) < 1e-12 and abs(A.sample(real, t_grasp_end)[5] - 0.053) < 1e-6,
      "on the robot the gripper rides as the gap in metres (0.090 then 0.053)")
goto = A.goto_step([0.0] * 5, 0.090, drive, 0.090)
check(goto.duration_s >= 1.0 and abs(goto.duration_s - A.QUINTIC_PEAK * max(abs(v) for v in drive) * A.TICKS_PER_RAD / 400) < 1e-9,
      f"a goto from the zero pose to drive takes {goto.duration_s:.2f} s at the ceiling")

# ── D. live ──
print("D. live")
from dimos.control.coordinator import ControlCoordinator  # noqa: E402
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory  # noqa: E402
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint  # noqa: E402
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule  # noqa: E402
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus  # noqa: E402

sim_bp = A.sim_parts(scene, start_rad=drive)[0]
sim_kwargs = dict(sim_bp.kwargs) if hasattr(sim_bp, "kwargs") else {}
module = MujocoSimModule(address=str(scene), headless=True, dof=A.ARM_DOF, enable_color=False,
                         enable_depth=False, enable_pointcloud=False,
                         reset_joint_positions=[*drive, A.GAP_CLOSED_M])
coord = None
try:
    t0 = time.time()
    module.start()
    engine = module._engine
    check(module._gripper_idx == 5 and module._gripper_joint_range == (0.0, 0.045),
          f"sim module up in {time.time() - t0:.1f} s: gripper = joint 5, range 0..45 mm")
    hw = A.arm_hardware(sim=True, address=scene)
    coord = ControlCoordinator(publish_joint_state=False, hardware=[hw], tasks=[A.arm_trajectory_task()])
    coord.start()
    time.sleep(1.5)
    pos = coord.get_joint_positions()
    check(set(pos) == set(A.JOINT_NAMES), f"coordinator joints: {sorted(pos)}")
    at_drive = all(abs(pos[f"arm/{j}"] - drive[i]) * DEG < 0.5 for i, j in enumerate(A.ARM_JOINTS))
    check(at_drive, "the simulated arm starts at the drive pose (within 0.5 deg)")
    gap0 = A.sim_gripper_gap(pos["arm/gripper"])
    check(abs(gap0 - 0.0) < 0.002, f"the jaws start closed and stay so until a command: {gap0 * 1000:.1f} mm (the adapter's read-back)")

    def run(trajectory, label: str, poll_hz: float = 20.0):
        """Execute and wait for the duration; return (max following error ticks,
        arm-rover contacts seen, elapsed s, final positions)."""
        res = coord.execute_trajectory(trajectory)
        check(res.status is TrajectoryExecutionStatus.ACCEPTED, f"{label}: accepted ({res.status.name} {res.message})")
        t_start = time.time()
        worst = 0.0
        contacts = 0
        arm_ids = {names.index(b) for b in A.ARM_BODIES}
        rover_ids = {names.index(b) for b in A.ROVER_BODIES}
        while time.time() - t_start < trajectory.duration + 0.3:
            t = time.time() - t_start
            cur = coord.get_joint_positions()
            err = A.following_error_ticks(trajectory, t, cur)
            worst = max(worst, max(err.values(), default=0.0))
            with engine._lock:
                d = engine._data
                for c in d.contact[: d.ncon]:
                    b1, b2 = m_sim.geom_bodyid[c.geom1], m_sim.geom_bodyid[c.geom2]
                    if c.dist < 0 and ((b1 in arm_ids and b2 in rover_ids) or (b2 in arm_ids and b1 in rover_ids)):
                        contacts += 1
            time.sleep(1.0 / poll_hz)
        return worst, contacts, time.time() - t_start, coord.get_joint_positions()

    def line_trajectory(q0, q1, g0, g1, seconds):
        steps = [A.Step("line", "a", "b", list(q0), list(q1), g0, g1, seconds, seconds)]
        return A.cycle_trajectory(steps, sim=True)

    def jaw_gap_mm() -> float:
        with engine._lock:
            d = engine._data
            left, right = names.index("nc90_jaw_left"), names.index("nc90_jaw_right")
            return float(((d.xpos[left] - d.xpos[right]) ** 2).sum() ** 0.5 * 1000.0)

    # (1) pan +30 deg and back
    q_pan = list(drive)
    q_pan[0] = drive[0] + math.radians(30.0)
    worst, contacts, el, fin = run(line_trajectory(drive, q_pan, 0.090, 0.090, 2.0), "pan +30 deg")
    got = (fin["arm/shoulder_pan"] - drive[0]) * DEG
    check(abs(got - 30.0) < 0.5, f"pan read back +{got:.2f} deg (commanded +30.00)")
    check(abs(jaw_gap_mm() - 90.0) < 1.0, f"the first trajectory opened the jaws: {jaw_gap_mm():.1f} mm")
    worst, contacts, el, fin = run(line_trajectory(q_pan, drive, 0.090, 0.090, 2.0), "pan back")
    got = (fin["arm/shoulder_pan"] - drive[0]) * DEG
    check(abs(got) < 0.5, f"pan back to {got:+.2f} deg")

    # (2) the gripper: 53 mm then 90 mm, measured between the jaws in the model
    run(line_trajectory(drive, drive, 0.090, 0.053, 1.0), "jaws to 53 mm")
    time.sleep(1.5)   # the jaws settle after the command (about half a second)
    g53 = jaw_gap_mm()
    check(abs(g53 - 53.0) < 1.0, f"jaw gap measured {g53:.1f} mm (commanded 53)")
    g53_read = A.sim_gripper_gap(coord.get_joint_positions()["arm/gripper"]) * 1000
    check(abs(g53_read - g53) < 1.0, f"the adapter's read-back says {g53_read:.1f} mm")
    run(line_trajectory(drive, drive, 0.053, 0.090, 1.0), "jaws to 90 mm")
    time.sleep(1.5)
    g90 = jaw_gap_mm()
    check(abs(g90 - 90.0) < 1.0, f"jaw gap measured {g90:.1f} mm (commanded 90)")

    # (3) the whole cycle
    step_ms = None
    worst, contacts, el, fin = run(traj, "the cycle")
    back = all(abs(fin[f"arm/{j}"] - drive[i]) * A.TICKS_PER_RAD < 20 for i, j in enumerate(A.ARM_JOINTS))
    check(back, "the arm ends at the drive pose (within 20 ticks)")
    check(contacts == 0, f"arm-rover contacts during the cycle: {contacts}")
    check(worst < 40, f"worst following error {worst:.0f} ticks ({worst / A.TICKS_PER_RAD * DEG:.1f} deg)")
    check(abs(el - (traj.duration + 0.3)) < 0.5, f"the cycle took {el:.1f} s (trajectory {traj.duration:.1f} s)")
    time.sleep(1.5)
    gap_end = jaw_gap_mm()
    with engine._lock:
        ctrl_end = float(engine._data.ctrl[5])
    check(abs(gap_end - 90.0) < 1.0, f"jaws open at the end: {gap_end:.1f} mm (jaw ctrl {ctrl_end:.4f}, last commanded {traj.points[-1].positions[5]:.4f})")
finally:
    if coord is not None:
        try:
            coord.stop()
        except Exception as exc:  # noqa: BLE001
            print(f"  (coordinator stop: {exc})")
    try:
        module.stop()
    except Exception as exc:  # noqa: BLE001
        print(f"  (sim module stop: {exc})")

# ── E. resources ──
print("E. resources")
d = mujoco.MjData(m_sim)
t0 = time.time()
for _ in range(200):
    mujoco.mj_step(m_sim, d)
step_ms = (time.time() - t0) / 200 * 1000
check(step_ms < 5.0, f"simulator step {step_ms:.2f} ms (timestep {m_sim.opt.timestep * 1000:.0f} ms)")
check(rss_gb() < 4.0, f"resident memory peak {rss_gb():.2f} GB")

print(f"\n{OK} OK, {len(FAIL)} FAIL")
for f in FAIL:
    print(f"  - {f}")
sys.exit(1 if FAIL else 0)
