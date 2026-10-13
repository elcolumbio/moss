"""The arm in dimOS: what the coordinator drives, what the simulator loads,
and the pick-and-drop cycle as one joint trajectory.

Three things live here, used by `moss_dimos.blueprints` (the `moss-arm`
blueprint), `tools/moss_arm_play.py` (the executor) and the cold bench:

* the hardware component: five joints and the gripper, under one adapter.
  In simulation the adapter is dimOS's `sim_mujoco`, fed by a
  `MujocoSimModule` loading the CAD scene (`sim_parts`); on the robot it is
  the Feetech adapter of `moss_dimos.arm_feetech`. The coordinator's one
  trajectory task runs the joints at the sweep's ceiling, no joint faster
  than SPEED_TICKS_PER_S;
* the simulation scene: the CAD scene with the joints' ranges set to the
  arm's measured travel (the calibration's end stops; the delivered scene
  carries the upstream model's nominal ranges, narrower than the drive pose)
  and the arm's self-contacts of the folded pose left out (`sim_scene`),
  written next to the other MOSS state;
* the cycle of `config/arm_drop_poses.json` sampled into a
  `JointTrajectory` (`cycle_trajectory`): each move a straight line in joint
  space timed by the quintic, the jaw gap by the cubic, as the JSON says,
  stretched where the sequence's own timing exceeds the ceiling.

Units. Arm joints: the model's radians (the keep-out table's model); the
gripper: the jaw gap in metres, 0 closed to GAP_OPEN_M. The simulator's
gripper joint is the left jaw's slide in metres, half the gap, and dimOS's
sim adapter inverts both the command and the read-back over the joint's
range (a command of 0 drives the slide to its top, and reads back as 0):
`sim_gripper_command` and `sim_gripper_gap` hold that, nothing else does. Ticks, for the messages people read, follow
the keep-out table's rule: 2048 at the model's zero, 4096 per turn, sign +1.
"""

from __future__ import annotations

import glob
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
POSES_PATH = ROOT / "config" / "arm_drop_poses.json"
CALIBRATION_PATH = ROOT / "config" / "arm_calibration.json"
CALIBRATION_ENV = "MOSS_ARM_CALIB"       # a calibration file elsewhere (a rig without the robot's config)

HW_ID = "arm"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER = "gripper"
JOINT_NAMES = [f"{HW_ID}/{j}" for j in (*ARM_JOINTS, GRIPPER)]   # the adapter's array order
ARM_DOF = len(ARM_JOINTS)

TICKS_PER_REV = 4096
MODEL_ZERO_TICKS = 2048
TICKS_PER_RAD = TICKS_PER_REV / (2.0 * math.pi)
SPEED_TICKS_PER_S = 400.0        # the sweep's ceiling (docs/arm.md): no joint faster than this
QUINTIC_PEAK = 1.875             # the quintic smootherstep's peak speed over its mean
SAMPLE_HZ = 50.0                 # trajectory points per second (the task interpolates between them)
GAP_OPEN_M = 0.090               # the jaws fully open
GAP_CLOSED_M = 0.0
GRIPPER_SPEED_M_PER_S = 0.2      # the task's ceiling on the gripper: the whole gap in half a second
START_TOLERANCE_RAD = 0.05       # the task refuses a trajectory starting further from the arm than this

ARM_COORDINATOR_NAME = "ArmCoordinator"   # the arm's coordinator serves its RPCs under this name: the base's keeps ControlCoordinator
SIM_ADAPTER = "sim_mujoco"                                    # dimOS's shared-memory MuJoCo adapter
REAL_ADAPTER = "feetech_so101"
REAL_ADAPTER_PATH = "moss_dimos.arm_feetech:FeetechSo101Adapter"
ARM_PORT_ENV = "MOSS_ARM_PORT"
ARM_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D041749-if00"

CAD_ENV = "MOSS_ARM_CAD"
CAD_GLOB = "~/moss-*/cad/*/MOSS_R31.xml"
SIM_SCENE_NAME = "MOSS_R31_sim.xml"
# The model's jaw slide (one jaw, metres) over the gap: both jaws move.
SIM_SLIDE_PER_GAP = 0.5
SIM_SLIDE_MAX_M = GAP_OPEN_M * SIM_SLIDE_PER_GAP
# Pairs of arm links whose convex colliders overlap in the model when the arm
# folds into the drive pose (the forearm back over the upper arm, 16 mm deep
# shoulder-forearm in the delivered scene), where the real arm rests. Left out
# of the simulation scene so the contact forces do not fight the folded pose;
# arm-rover contacts stay, they are what the scene is for.
SIM_SELF_CONTACT_EXCLUDES = (("shoulder", "lower_arm"), ("upper_arm", "wrist"), ("shoulder", "wrist"))
# The measured end stops are integer ticks: a tick of slack each side keeps the
# recorded poses on the stops (the drive pose) inside the simulated ranges.
SIM_RANGE_PAD_TICKS = 1
# The delivered scene's jaw actuator is soft (kp 50 N/m through the rack and
# pinion: the jaws crawl at about a millimetre per second); the servo is stiff.
# Set in the simulation scene so the jaws follow their command.
SIM_GRIPPER_ACTUATOR = "nc90_close"
SIM_GRIPPER_KP = 2000.0
SIM_GRIPPER_FORCE_N = 20.0
ARM_BODIES = ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper",
              "nc90_tool", "nc90_jaw_left", "nc90_jaw_right", "nc90_drive")
ROVER_BODIES = ("rover_exterior", "base")   # the arm's mount counts as rover

# The pose the arm rests in for driving, as recorded on the arm (docs/arm.md);
# config/arm_calibration.json carries it on the robot and wins when present.
DRIVE_TICKS = {"shoulder_pan": 2857, "shoulder_lift": 860, "elbow_flex": 3206,
               "wrist_flex": 868, "wrist_roll": 3221}


# ── units ──

def calibration_path(explicit: str | os.PathLike | None = None, env: dict | None = None) -> Path:
    """The calibration file: `explicit`, else MOSS_ARM_CALIB, else config/arm_calibration.json."""
    env = os.environ if env is None else env
    raw = str(explicit or "").strip() or env.get(CALIBRATION_ENV, "").strip()
    return Path(raw).expanduser() if raw else CALIBRATION_PATH


def ticks_to_rad(ticks: float) -> float:
    return (float(ticks) - MODEL_ZERO_TICKS) / TICKS_PER_RAD


def rad_to_ticks(rad: float) -> int:
    return int(round(MODEL_ZERO_TICKS + float(rad) * TICKS_PER_RAD))


def sim_gripper_command(gap_m: float) -> float:
    """The value to command on the simulator's gripper joint for a jaw gap:
    the slide is half the gap, and the sim adapter inverts commands over the
    joint's range."""
    gap = min(max(float(gap_m), GAP_CLOSED_M), GAP_OPEN_M)
    return SIM_SLIDE_MAX_M - gap * SIM_SLIDE_PER_GAP


def sim_gripper_gap(value: float) -> float:
    """The jaw gap for a value read back from the simulator's gripper joint:
    the read-back is in the command's convention (inverted over the range),
    so the slide is the top of the range less the value, and the gap twice
    the slide."""
    return (SIM_SLIDE_MAX_M - float(value)) / SIM_SLIDE_PER_GAP


def quintic(u: float) -> float:
    u = min(max(float(u), 0.0), 1.0)
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def cubic(u: float) -> float:
    u = min(max(float(u), 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


# ── the CAD scene and the simulation scene ──

def find_cad(spec: str | os.PathLike | None = None, env: dict | None = None) -> Path | None:
    """The CAD scene: `spec` (a file, or a directory holding MOSS_R31.xml), else
    MOSS_ARM_CAD, else the first match of CAD_GLOB; None when nothing exists."""
    env = os.environ if env is None else env
    raw = str(spec or "").strip() or env.get(CAD_ENV, "").strip()
    candidates = [Path(raw).expanduser()] if raw else [Path(p) for p in sorted(glob.glob(os.path.expanduser(CAD_GLOB)))]
    for c in candidates:
        if c.is_dir():
            c = c / "MOSS_R31.xml"
        if c.is_file():
            return c.resolve()
    return None


def state_dir(env: dict | None = None) -> Path:
    """MOSS_STATE_DIR, default ~/.local/state/moss (the same rule as robot_profile)."""
    env = os.environ if env is None else env
    raw = env.get("MOSS_STATE_DIR", "").strip()
    return Path(raw or "~/.local/state/moss").expanduser().absolute()


def calibrated_ranges_rad(calibration: str | os.PathLike | None = None) -> dict[str, tuple[float, float]]:
    """Each arm joint's measured travel in model radians, from the calibration's
    end stops (reported ticks); empty when there is no calibration file."""
    path = calibration_path(calibration)
    if not path.is_file():
        return {}
    doc = json.loads(path.read_text())
    out: dict[str, tuple[float, float]] = {}
    for j in ARM_JOINTS:
        rec = doc.get(j) or {}
        if "range_min" in rec and "range_max" in rec:
            out[j] = (ticks_to_rad(rec["range_min"] - SIM_RANGE_PAD_TICKS),
                      ticks_to_rad(rec["range_max"] + SIM_RANGE_PAD_TICKS))
    return out


def sim_scene(cad: str | os.PathLike, out_dir: str | os.PathLike | None = None,
              calibration: str | os.PathLike | None = None) -> Path:
    """The simulation scene written from the CAD scene: the mesh directory made
    absolute (the file lives elsewhere), the arm joints' ranges and their
    actuators' control ranges set to the measured travel when a calibration
    exists (the delivered ranges stop short of the drive pose), the folded
    pose's self-contacts excluded. Rewritten on every call; the path is the
    SHM key the sim adapter and the sim module share, so both must get this
    path."""
    cad = Path(cad).resolve()
    tree = ET.parse(cad)
    root = tree.getroot()
    ranges = calibrated_ranges_rad(calibration)
    if ranges:
        for el in root.iter("joint"):
            r = ranges.get(el.get("name", ""))
            if r:
                el.set("range", f"{r[0]:.9f} {r[1]:.9f}")
    act = root.find("actuator")
    for el in (list(act) if act is not None else []):
        r = ranges.get(el.get("name", "")) or ranges.get(el.get("joint", ""))
        if r:
            el.set("ctrlrange", f"{r[0]:.9f} {r[1]:.9f}")
        if el.get("name") == SIM_GRIPPER_ACTUATOR:
            el.set("kp", f"{SIM_GRIPPER_KP:g}")
            el.set("forcerange", f"{-SIM_GRIPPER_FORCE_N:g} {SIM_GRIPPER_FORCE_N:g}")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    meshdir = Path(compiler.get("meshdir", "")) if compiler.get("meshdir") else Path(".")
    if not meshdir.is_absolute():
        meshdir = cad.parent / meshdir
    compiler.set("meshdir", str(meshdir.resolve()))
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    have = {(e.get("body1"), e.get("body2")) for e in contact.findall("exclude")}
    for a, b in SIM_SELF_CONTACT_EXCLUDES:
        if (a, b) not in have and (b, a) not in have:
            ET.SubElement(contact, "exclude", body1=a, body2=b)
    out = Path(out_dir) if out_dir is not None else state_dir() / "arm_sim"
    out.mkdir(parents=True, exist_ok=True)
    path = out / SIM_SCENE_NAME
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path.resolve()


# ── the hardware component, the task, the simulator ──

def drive_pose_rad(calibration: str | os.PathLike | None = None) -> list[float]:
    """The drive pose in model radians, in ARM_JOINTS order: from the
    calibration file when it exists, else DRIVE_TICKS."""
    path = calibration_path(calibration)
    ticks = dict(DRIVE_TICKS)
    if path.is_file():
        doc = json.loads(path.read_text())
        rec = (doc.get("poses") or {}).get("drive", {}).get("ticks") or {}
        for j in ARM_JOINTS:
            if j in rec:
                ticks[j] = int(rec[j])
    return [ticks_to_rad(ticks[j]) for j in ARM_JOINTS]


def arm_port(env: dict | None = None) -> str:
    env = os.environ if env is None else env
    return env.get(ARM_PORT_ENV, "").strip() or ARM_PORT_DEFAULT


def arm_hardware(sim: bool, address: str | os.PathLike | None = None,
                 calibration: str | os.PathLike | None = None, **adapter_kwargs):
    """The one hardware component: in simulation the sim adapter on the
    simulation scene (address = its path), on the robot the Feetech adapter on
    the serial port."""
    from dimos.control.components import HardwareComponent, HardwareType

    if sim:
        addr = str(address) if address else None
        if addr is None:
            cad = find_cad()
            if cad is None:
                raise FileNotFoundError(f"no CAD scene: {CAD_ENV}, or {CAD_GLOB}")
            addr = str(sim_scene(cad, calibration=calibration))
        return HardwareComponent(hardware_id=HW_ID, hardware_type=HardwareType.MANIPULATOR,
                                 joints=list(JOINT_NAMES), adapter_type=SIM_ADAPTER, address=addr,
                                 auto_enable=True, adapter_kwargs=dict(adapter_kwargs))
    return HardwareComponent(hardware_id=HW_ID, hardware_type=HardwareType.MANIPULATOR,
                             joints=list(JOINT_NAMES), adapter_type=REAL_ADAPTER,
                             address=str(address) if address else arm_port(),
                             auto_enable=True, adapter_kwargs=dict(adapter_kwargs))


def velocity_limits() -> dict[str, float]:
    """The task's per-joint ceilings: the sweep's ticks per second in radians
    per second on the arm, the gripper's in metres of gap per second."""
    lim = {f"{HW_ID}/{j}": SPEED_TICKS_PER_S / TICKS_PER_RAD for j in ARM_JOINTS}
    lim[f"{HW_ID}/{GRIPPER}"] = GRIPPER_SPEED_M_PER_S
    return lim


def arm_trajectory_task():
    """The coordinator's trajectory task on the six joints. The per-joint
    ceiling goes in where the installed dimOS takes it (`velocity_limits`);
    an older dimOS caps at 1 rad/s itself, and the servos' own goal speed
    (the Feetech adapter sets it) holds the sweep's ceiling on the robot."""
    import inspect

    from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task

    kw = {"start_position_tolerance": START_TOLERANCE_RAD}
    if "velocity_limits" in inspect.signature(joint_trajectory_task).parameters:
        kw["velocity_limits"] = velocity_limits()
    return joint_trajectory_task(list(JOINT_NAMES), **kw)


def sim_parts(scene: str | os.PathLike, start_rad: list[float] | None = None, gap_m: float = GAP_CLOSED_M):
    """The simulator module for the blueprint: headless, no camera, the arm
    reset to `start_rad` (the drive pose by default). The jaws start at
    `gap_m`, closed by default: the reset sets the left jaw's slide alone, and
    the right jaw and the pinion (coupled to it) start at zero; the sim
    adapter's first command opens them."""
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

    start = list(start_rad if start_rad is not None else drive_pose_rad())
    reset = [*start, gap_m * SIM_SLIDE_PER_GAP]
    return (MujocoSimModule.blueprint(address=str(scene), headless=True, dof=ARM_DOF,
                                      enable_color=False, enable_depth=False, enable_pointcloud=False,
                                      reset_joint_positions=reset),)


# ── the cycle as a trajectory ──

@dataclass
class Step:
    phase: str
    start: str
    end: str
    q_from: list[float]          # ARM_JOINTS, model radians
    q_to: list[float]
    gap_from_m: float
    gap_to_m: float
    timed_s: float               # the sequence's own duration
    duration_s: float            # as run: stretched to the speed ceiling
    holds_can: bool = False

    @property
    def peak_ticks_per_s(self) -> float:
        d = max(abs(b - a) for a, b in zip(self.q_from, self.q_to)) * TICKS_PER_RAD
        return QUINTIC_PEAK * d / self.timed_s if self.timed_s > 0 else (math.inf if d else 0.0)


@dataclass
class Cycle:
    steps: list[Step]
    joints: list[str] = field(default_factory=lambda: list(ARM_JOINTS))

    @property
    def timed_s(self) -> float:
        return sum(s.timed_s for s in self.steps)

    @property
    def duration_s(self) -> float:
        return sum(s.duration_s for s in self.steps)

    @property
    def first_pose(self) -> list[float]:
        return list(self.steps[0].q_from)

    @property
    def first_gap_m(self) -> float:
        return self.steps[0].gap_from_m


def stretched(q_a: list[float], q_b: list[float], timed_s: float, speed: float = SPEED_TICKS_PER_S) -> float:
    d = max(abs(b - a) for a, b in zip(q_a, q_b)) * TICKS_PER_RAD
    return max(timed_s, QUINTIC_PEAK * d / speed) if d else timed_s


def load_cycle(path: str | os.PathLike = POSES_PATH, calibration: str | os.PathLike | None = None,
               speed: float = SPEED_TICKS_PER_S, stop_after: str | None = None,
               start: str | None = None) -> Cycle:
    """The sequence of the JSON as delivered: its poses as they are (no shift,
    no via), the drive pose from the calibration, each move stretched to
    `speed`. `stop_after` keeps the moves up to that phase (e.g. "release":
    the arm stays over the bin, the return is another run); `start` drops
    the moves before that phase (e.g. "back_to_clear": the return)."""
    doc = json.loads(Path(path).read_text())
    joints = list(doc.get("joints") or ARM_JOINTS)
    if tuple(joints) != ARM_JOINTS:
        raise ValueError(f"{path}: joints {joints} are not {list(ARM_JOINTS)}")
    poses = {name: [float(v) for v in rec["rad"]] for name, rec in doc["poses"].items()}
    poses.setdefault("drive", drive_pose_rad(calibration))
    steps: list[Step] = []
    skipping = start is not None
    for m in doc["sequence"]:
        if skipping:
            if m.get("phase") != start:
                continue
            skipping = False
        a, b = m["from"], m["to"]
        if a not in poses or b not in poses:
            raise KeyError(f"{path}: move {m.get('phase')} uses an unknown pose ({a} -> {b})")
        gap = m.get("gap_mm") or [GAP_OPEN_M * 1000, GAP_OPEN_M * 1000]
        timed = float(m.get("duration_s", 0.0))
        steps.append(Step(phase=str(m.get("phase", f"{a}_to_{b}")), start=a, end=b,
                          q_from=list(poses[a]), q_to=list(poses[b]),
                          gap_from_m=float(gap[0]) / 1000.0, gap_to_m=float(gap[1]) / 1000.0,
                          timed_s=timed, duration_s=stretched(poses[a], poses[b], timed, speed),
                          holds_can=bool(m.get("holds_can", False))))
        if stop_after and steps[-1].phase == stop_after:
            break
    if not steps:
        raise ValueError(f"{path}: empty sequence (start {start!r}, stop_after {stop_after!r})")
    return Cycle(steps=steps, joints=joints)


def goto_step(q_now: list[float], gap_now_m: float, q_to: list[float], gap_to_m: float,
              speed: float = SPEED_TICKS_PER_S, least_s: float = 1.0) -> Step:
    """A move from where the arm is to the cycle's first pose, at the ceiling."""
    d = stretched(q_now, q_to, least_s, speed)
    return Step(phase="goto", start="current", end="first", q_from=list(q_now), q_to=list(q_to),
                gap_from_m=gap_now_m, gap_to_m=gap_to_m, timed_s=d, duration_s=d)


def cycle_trajectory(steps: list[Step], sim: bool, sample_hz: float = SAMPLE_HZ):
    """The steps sampled into one JointTrajectory in JOINT_NAMES order: arm
    joints in radians, the gripper in the adapter's unit (the gap in metres on
    the robot, the simulator's command in simulation). Each step runs the
    quintic on the joints and the cubic on the gap over its duration; the
    first point is at t = 0."""
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
    from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

    g = sim_gripper_command if sim else (lambda gap: float(gap))
    points: list[TrajectoryPoint] = []
    t0 = 0.0
    for s in steps:
        n = max(1, int(math.ceil(s.duration_s * sample_hz)))
        for k in range(0 if not points else 1, n + 1):
            u = k / n
            w = quintic(u)
            q = [a + (b - a) * w for a, b in zip(s.q_from, s.q_to)]
            gap = s.gap_from_m + (s.gap_to_m - s.gap_from_m) * cubic(u)
            points.append(TrajectoryPoint(time_from_start=t0 + s.duration_s * u, positions=[*q, g(gap)]))
        t0 += s.duration_s
    # velocities: finite differences, for the record (the task interpolates positions)
    for i, p in enumerate(points):
        if i == 0 or i == len(points) - 1:
            p.velocities = [0.0] * len(p.positions)
            continue
        a, b = points[i - 1], points[i + 1]
        dt = b.time_from_start - a.time_from_start
        p.velocities = [(y - x) / dt if dt > 0 else 0.0 for x, y in zip(a.positions, b.positions)]
    return JointTrajectory(points=points, joint_names=list(JOINT_NAMES))


def sample(trajectory, t: float) -> list[float]:
    """The trajectory's positions at time t (linear between points, held at the ends)."""
    pts = trajectory.points
    if t <= pts[0].time_from_start:
        return list(pts[0].positions)
    for a, b in zip(pts[:-1], pts[1:]):
        if a.time_from_start <= t <= b.time_from_start:
            dt = b.time_from_start - a.time_from_start
            w = (t - a.time_from_start) / dt if dt > 0 else 1.0
            return [x + (y - x) * w for x, y in zip(a.positions, b.positions)]
    return list(pts[-1].positions)


def following_error_ticks(trajectory, t: float, measured: dict[str, float]) -> dict[str, float]:
    """Per arm joint, |commanded - measured| in ticks at time t: the number
    that says whether the arm follows (the shoulder short of its target under
    load is the case this watches)."""
    want = sample(trajectory, t)
    out: dict[str, float] = {}
    for name, w in zip(trajectory.joint_names, want):
        if name == f"{HW_ID}/{GRIPPER}":
            continue
        m = measured.get(name)
        if m is None:
            continue
        out[name] = abs(w - m) * TICKS_PER_RAD
    return out
