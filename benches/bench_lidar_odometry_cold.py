"""Cold bench for lidar_odometry: the pose and the clouds it publishes, in physical units.

kiss-icp is faked (the bench commands the pose its registration reports);
the dimOS messages are real. Known input -> known output, in metres and degrees:

  A. the module as built: streams, rpcs, defaults, and start() building
     kiss-icp with its configuration and logging "lidar odometry up".
  B. roundtrip 73 deg: a revolution registered at (1.20, -0.70, +73.0 deg)
     comes out as odom (1.20, -0.70, z = 0.00) heading 73.0 deg, a world cloud
     whose point 2.0 m ahead lands at (1.7847, 1.2126, 0.20), and a tf with
     base_link->lidar_link at z = 0.20 m. One cloud per revolution.
  C. the lidar height contract with costmap2d: MOSS_LIDAR_HEIGHT_M=0.215 moves
     the cloud and the tf to 0.215 m, and costmap2d.lidar_returns tags those
     points (float32 on the wire) exactly when both read the same height.
  D. the gyro prior: a yaw rate of 0.5 rad/s for 0.2 s (gyro y = -0.5 on the
     default "-y" axis) -> the next revolution turns 5.73 deg; with the prior
     off, kiss-icp's constant-velocity model is used instead. start() with the
     prior on (the MOSS default) says "prior: gyro=True" in its up line.
  E. the optional depth path: 1.000 m of depth at the principal point, rover
     at the origin -> a camera point at (1.156, 0, 0.075) m on the next lidar
     cloud, which costmap2d.lidar_returns does NOT tag as lidar.
  G. a contact (`bump`): at rest after 10 revolutions of 300 points, a
     revolution of 90 points turned 6 deg right after a bump is dropped (the
     yaw moves by the gyro's +0.40 deg only, the log says so); the next 5 full
     revolutions register with sigma x2 and the 6th with the plain sigma; with
     no bump, outside the 0.4 s window or MOSS_CONTACT_DROP_RATIO=0 the same
     revolution registers at +6.0 deg. MOSS_RELOC_SWEEPS=3
     MOSS_RELOC_SIGMA_GAIN=3 give 3 revolutions at x3; MOSS_CONTACT_WINDOW_S=0.05
     lets a thin revolution 0.1 s after the bump register.

Needs dimOS (kiss-icp is not needed: faked through sys.modules).

Run:  PYTHONPATH=. .venv/bin/python benches/bench_lidar_odometry_cold.py
"""

import asyncio
import gc
import math
import os
import sys
import threading
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

# MOSS's own numbers, whatever the shell exported
for _var in [v for v in os.environ if v.startswith("MOSS_")]:
    del os.environ[_var]

from dimos.core.coordination.blueprints import BlueprintAtom  # noqa: E402
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped  # noqa: E402
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo  # noqa: E402
from dimos.msgs.sensor_msgs.Image import Image  # noqa: E402
from dimos.msgs.sensor_msgs.Imu import Imu  # noqa: E402
from dimos.msgs.sensor_msgs.JointState import JointState  # noqa: E402
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: E402
from dimos.msgs.tf2_msgs.TFMessage import TFMessage  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402

from moss_dimos import costmap2d  # noqa: E402
from moss_dimos import lidar_odometry as LO  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- a fake kiss-icp: the pose it reports is the bench's to command ----------

class Threshold:
    def __init__(self, v=0.2):
        self.v = v
        self.deviations = 0

    def get_threshold(self):
        return self.v

    def update_model_deviation(self, corr):
        self.deviations += 1


class Preproc:
    def preprocess(self, pts, ts, prior):
        return pts


class LocalMap:
    def __init__(self):
        self.updates = 0

    def update(self, frame, pose):
        self.updates += 1


class Registration:
    """The registration "finds" exactly the prediction: the published pose is
    then the prior the bench commanded. `fail_first` makes the next N calls
    raise the board's thread-pool error, as the real one does once at start."""

    def __init__(self):
        self.calls = 0
        self.fail_first = 0

    def align_points_to_map(self, points, voxel_map, initial_guess, max_correspondance_distance, kernel):
        self.calls += 1
        if self.fail_first > 0:
            self.fail_first -= 1
            raise RuntimeError("pthread_create has failed: Invalid argument")
        return np.asarray(initial_guess)


class Kiss:
    """Starts at the identity; its constant-velocity model (last_delta) is
    the motion the bench commands for the next revolution."""

    def __init__(self, dx=0.0, dy=0.0, dyaw=0.0, cfg=None):
        self.cfg = cfg
        self.adaptive_threshold = Threshold()
        self.preprocessor = Preproc()
        self.local_map = LocalMap()
        self.registration = Registration()
        self.last_pose = np.eye(4)
        self.last_delta = LO._se2(dx, dy, dyaw)

    def voxelize(self, frame):
        return frame, frame


_cfg_mod = types.ModuleType("kiss_icp.config")
_cfg_mod.KISSConfig = lambda: types.SimpleNamespace(
    data=types.SimpleNamespace(), mapping=types.SimpleNamespace(),
    adaptive_threshold=types.SimpleNamespace())
_kiss_mod = types.ModuleType("kiss_icp.kiss_icp")
_kiss_mod.KissICP = lambda cfg: Kiss(cfg=cfg)
sys.modules.setdefault("kiss_icp", types.ModuleType("kiss_icp"))
sys.modules["kiss_icp.config"] = _cfg_mod
sys.modules["kiss_icp.kiss_icp"] = _kiss_mod


class Sink:
    """An Out stream that keeps what was published."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class LogSpy:
    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **kwargs):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = exception = _record


LOOP = asyncio.new_event_loop()
REAL = []   # modules built through dimOS's __init__, for the record


def odometry(kiss, use_gyro_prior=False):
    """The real __init__ (the fields handle_pointcloud needs come from it),
    the fake kiss-icp, and Sinks in place of the Outs."""
    o = LO.LidarOdometry(use_gyro_prior=use_gyro_prior)
    REAL.append(o)
    o._kiss, o._lock = kiss, threading.Lock()
    o.log_every_s = 1e9                 # this bench reads publications, not the log
    o.odom, o.lidar, o.tf, o.camera_floor = Sink(), Sink(), Sink(), Sink()
    return o


def drive(o, n_revs: int, pts):
    msg = PointCloud2.from_numpy(pts.astype(np.float32), frame_id="lidar_link", timestamp=time.time())
    for _ in range(n_revs):
        LOOP.run_until_complete(o.handle_pointcloud(msg))


def yaw_of(q) -> float:
    """Heading of a quaternion, in degrees."""
    return math.degrees(2.0 * math.atan2(q.z, q.w))


def cloud_xyz(msg) -> np.ndarray:
    out = msg.as_numpy()
    return np.asarray(out[0] if isinstance(out, tuple) else out)


# A flat revolution in lidar_link: 360 returns on a 2 m circle, plus one
# marker 2.0 m dead ahead (index 0: angle 0).
SCAN_FLAT = np.stack([np.cos(np.linspace(0, 2 * math.pi, 360, endpoint=False)) * 2.0,
                      np.sin(np.linspace(0, 2 * math.pi, 360, endpoint=False)) * 2.0,
                      np.zeros(360)], axis=1)


# --- A. the module as built ---------------------------------------------------
print("A. streams, rpcs, defaults, start()")
streams = {(s.name, s.type): s.direction for s in BlueprintAtom.create(LO.LidarOdometry, {}).streams}
want = {("pointcloud", PointCloud2): "in", ("imu", Imu): "in", ("coordinator_joint_state", JointState): "in",
        ("depth_image", Image): "in", ("camera_info", CameraInfo): "in", ("bump", Bool): "in",
        ("odom", PoseStamped): "out", ("lidar", PointCloud2): "out", ("camera_floor", PointCloud2): "out",
        ("tf", TFMessage): "out"}
check("the stream table: 6 Ins, 4 Outs, names and types as wired by the MOSS blueprints",
      streams == want, f"extra {set(streams) - set(want)}, missing {set(want) - set(streams)}")
check("bump is an In of dimos_lcm's Bool (the class BumpImu, the coordinator and the costmap use); no reloc_frame or bump_rear stream",
      streams.get(("bump", Bool)) == "in" and not any(n in ("reloc_frame", "bump_rear") for n, _ in streams))
check("LidarOdometry.rpcs carries start and stop", {"start", "stop"} <= set(LO.LidarOdometry.rpcs))
check("dimOS instance name 'lidarodometry'", LO.LidarOdometry.name == "lidarodometry")

o = LO.LidarOdometry(use_gyro_prior=False)
REAL.append(o)
check("defaults: range 0.35-12 m, voxel 0.05 m, initial threshold 0.3 m, gyro axis -y, frames world/base_link/lidar_link",
      (o.min_range_m, o.max_range_m, o.voxel_size_m, o.initial_threshold_m, o.gyro_axis,
       o.world_frame, o.base_frame, o.lidar_frame)
      == (0.35, 12.0, 0.05, 0.3, "-y", "world", "base_link", "lidar_link"))
check("nothing native before start(): no kiss-icp, no lock (dimOS pickles the module)",
      o._kiss is None and o._lock is None)

spy, real_logger = LogSpy(), LO.logger
LO.logger = spy
real_module_start = LO.Module.start
LO.Module.start = lambda self: None          # the dimOS runtime is not under test here
try:
    o.start()
finally:
    LO.Module.start = real_module_start
    LO.logger = real_logger
cfg = getattr(o._kiss, "cfg", None)
check("start() builds kiss-icp: max 12 m, min 0.35 m, deskew OFF by default, voxel 0.05 m, 30 points/voxel, threshold 0.3 m, min motion 0.02 m",
      cfg is not None and (cfg.data.max_range, cfg.data.min_range, cfg.data.deskew, cfg.mapping.voxel_size,
                           cfg.mapping.max_points_per_voxel, cfg.adaptive_threshold.initial_threshold,
                           cfg.adaptive_threshold.min_motion_th) == (12.0, 0.35, False, 0.05, 30, 0.3, 0.02))
check("deskew=True reaches the kiss-icp config (MOSS_DESKEW=1 in the blueprint)",
      LO.LidarOdometry(deskew=True).deskew is True and LO.LidarOdometry().deskew is False)
up = [line for line in spy.lines if line.startswith("lidar odometry up")]
check("start() logs 'lidar odometry up (kiss-icp, ...' once (the launcher greps this prefix)",
      len(up) == 1 and up[0].startswith("lidar odometry up (kiss-icp,") and up[0].endswith("-> world"),
      up[0] if up else "no line")


# --- B. roundtrip 73 deg --------------------------------------------------------
print("A2. sweep_times: when each point of a revolution was taken")
ang = np.radians(np.arange(0, 360, 2.0))
rev = np.stack([2.0 * np.cos(ang), 2.0 * np.sin(ang), np.zeros_like(ang)], 1)
st = LO.sweep_times(rev)
check("a full revolution in bearing order: 0 at the first point, 1 at the last, monotonic",
      abs(st[0]) < 1e-9 and abs(st[-1] - 1.0) < 1e-9 and np.all(np.diff(st) >= 0), f"{st[:3]} .. {st[-3:]}")
check("half way round the sweep -> about 0.5", abs(st[len(st) // 2] - 0.5) < 0.02, f"{st[len(st) // 2]:.3f}")
rev_cw = rev[::-1]
st_cw = LO.sweep_times(rev_cw)
check("the other way round (clockwise): still 0 first, 1 last", abs(st_cw[0]) < 1e-9 and abs(st_cw[-1] - 1.0) < 1e-9 and np.all(np.diff(st_cw) >= 0))
check("a revolution with a gap (points 0-170 deg only) keeps the fraction of the bearing covered",
      abs(LO.sweep_times(rev[:86])[-1] - 1.0) < 1e-9 and abs(LO.sweep_times(rev[:86])[43] - 0.5) < 0.02)
check("two points at the same bearing -> the point order", np.allclose(LO.sweep_times(np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])), [0.0, 1.0]))
check("one point -> 0", LO.sweep_times(np.array([[1.0, 0.0, 0.0]])).tolist() == [0.0])

print("B. roundtrip: a revolution registered at (1.20, -0.70, +73.0 deg)")
TRUE = (1.20, -0.70, math.radians(73.0))
o = odometry(Kiss(*TRUE))
drive(o, 1, SCAN_FLAT)
od = o.odom.msgs[-1]
check("odom position (1.20, -0.70) m, z = 0.00 (the floor plane, never the lidar height)",
      abs(od.position.x - 1.20) < 1e-9 and abs(od.position.y + 0.70) < 1e-9 and od.position.z == 0.0,
      f"({od.position.x:.4f}, {od.position.y:.4f}, {od.position.z:.4f})")
check("odom heading 73.0 deg", abs(yaw_of(od.orientation) - 73.0) < 0.1, f"{yaw_of(od.orientation):.3f} deg")
check("odom frame 'world'", od.frame_id == "world")
xyz = cloud_xyz(o.lidar.msgs[-1])
c, s = math.cos(TRUE[2]), math.sin(TRUE[2])
ahead = (1.20 + 2.0 * c, -0.70 + 2.0 * s)
check(f"the return 2.0 m dead ahead lands at ({ahead[0]:.4f}, {ahead[1]:.4f}, 0.20) m in world",
      abs(xyz[0, 0] - ahead[0]) < 1e-5 and abs(xyz[0, 1] - ahead[1]) < 1e-5 and abs(xyz[0, 2] - 0.20) < 1e-6,
      f"({xyz[0, 0]:.4f}, {xyz[0, 1]:.4f}, {xyz[0, 2]:.4f})")
check("every lidar point sits at z = 0.20 m (MOSS lidar_height_m), to the bit of a float32 on the wire",
      np.all(xyz[:, 2] == np.float64(np.float32(0.20))), f"z in [{xyz[:, 2].min()}, {xyz[:, 2].max()}]")
check("the cloud is in 'world', 360 points", o.lidar.msgs[-1].frame_id == "world" and len(xyz) == 360)
tf = o.tf.msgs[-1].transforms
check("tf world->base_link at (1.20, -0.70, 0.00), heading 73.0 deg",
      tf[0].frame_id == "world" and tf[0].child_frame_id == "base_link"
      and abs(tf[0].translation.x - 1.20) < 1e-9 and abs(tf[0].translation.y + 0.70) < 1e-9
      and tf[0].translation.z == 0.0 and abs(yaw_of(tf[0].rotation) - 73.0) < 0.1)
check("tf base_link->lidar_link at z = 0.20 m",
      tf[1].frame_id == "base_link" and tf[1].child_frame_id == "lidar_link"
      and abs(tf[1].translation.z - 0.20) < 1e-12 and tf[1].translation.x == 0.0)
check("kiss-icp integrated the scan: local map updated, adaptive threshold fed",
      o._kiss.local_map.updates == 1 and o._kiss.adaptive_threshold.deviations == 1)
p = odometry(Kiss())
drive(p, 25, SCAN_FLAT)
check("a parked rover: 25 revolutions -> 25 odom, 25 clouds, 25 tf, all at (0, 0)",
      len(p.odom.msgs) == len(p.lidar.msgs) == len(p.tf.msgs) == 25
      and all(m.position.x == 0.0 and m.position.y == 0.0 for m in p.odom.msgs))
short = odometry(Kiss())
drive(short, 3, SCAN_FLAT[:29])
check("a revolution under 30 points is skipped (nothing published)", not short.odom.msgs and not short.lidar.msgs)


# --- C. the lidar height contract with costmap2d --------------------------------
print("C. lidar_height_m: the lift here, the tag in costmap2d")
os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.215"
try:
    o = odometry(Kiss(0.5, 0.0, 0.0))
    drive(o, 1, SCAN_FLAT)
    xyz = cloud_xyz(o.lidar.msgs[-1])
    check("MOSS_LIDAR_HEIGHT_M=0.215 (read at call time): every point at z = 0.215 m",
          np.all(xyz[:, 2] == np.float64(np.float32(0.215))))
    check("... and the base_link->lidar_link tf at z = 0.215 m",
          abs(o.tf.msgs[-1].transforms[1].translation.z - 0.215) < 1e-12)
    check("... and odom stays on the floor, z = 0.00", o.odom.msgs[-1].position.z == 0.0)
    tagged = costmap2d.lidar_returns(xyz.astype(np.float64))
    check("costmap2d reads the same height: all 360 points tagged as lidar returns", bool(tagged.all()))
finally:
    del os.environ["MOSS_LIDAR_HEIGHT_M"]
check("a cloud lifted to 0.215 m is NOT tagged when the costmap reads 0.20 m (one value, both ends)",
      not costmap2d.lidar_returns(xyz.astype(np.float64)).any())


# --- D. the gyro prior ----------------------------------------------------------
print("D. the rotation prior: gyro when on and fed, constant velocity otherwise")


def imu(gy, ts):
    return types.SimpleNamespace(angular_velocity=types.SimpleNamespace(x=0.0, y=gy, z=0.0), ts=ts)


o = odometry(Kiss(0.10, 0.0, math.radians(2.0)), use_gyro_prior=True)
t0 = 1000.0
for i in range(11):                                   # 0.2 s at 50 Hz, yaw rate 0.5 rad/s
    LOOP.run_until_complete(o.handle_imu(imu(-0.5, t0 + 0.02 * i)))
drive(o, 1, SCAN_FLAT)
yaw = yaw_of(o.odom.msgs[-1].orientation)
check("0.5 rad/s for 0.2 s on the -y axis -> the revolution turns 5.73 deg (not the 2.0 deg the motion model says)",
      abs(yaw - math.degrees(0.1)) < 0.01, f"{yaw:.3f} deg, prior {o._prior_used}")
check("the translation still comes from the motion model: x = 0.10 m",
      abs(o.odom.msgs[-1].position.x - 0.10) < 1e-9)
check("the prior is consumed: the next revolution has no gyro sample", o._gyro_seen is False and o._gyro_acc == 0.0)
q = odometry(Kiss(0.10, 0.0, math.radians(2.0)), use_gyro_prior=False)
for i in range(11):
    LOOP.run_until_complete(q.handle_imu(imu(-0.5, t0 + 0.02 * i)))
drive(q, 1, SCAN_FLAT)
check("use_gyro_prior=False (MOSS_GYRO=0 in the blueprint): the same gyro is ignored, 2.0 deg from the motion model",
      abs(yaw_of(q.odom.msgs[-1].orientation) - 2.0) < 0.01 and q._prior_used == "cv")
js = types.SimpleNamespace(name=["base/vx", "base/vy", "base/wz"], position=[0.5, 0.0, 0.1])
LOOP.run_until_complete(q.handle_coordinator_joint_state(js))
drive(q, 1, SCAN_FLAT)
step = math.radians(2.0)                               # second revolution: (0.10, 0) then 0.10 m along 2 deg
check("wheel odometry is read for the log only: the pose follows the motion model to (0.1999, 0.0035) m, 4.0 deg",
      q._wheel == (0.5, 0.0, 0.1)
      and abs(q.odom.msgs[-1].position.x - (0.10 + 0.10 * math.cos(step))) < 1e-9
      and abs(q.odom.msgs[-1].position.y - 0.10 * math.sin(step)) < 1e-9
      and abs(yaw_of(q.odom.msgs[-1].orientation) - 4.0) < 0.01,
      f"({q.odom.msgs[-1].position.x:.4f}, {q.odom.msgs[-1].position.y:.4f})")
gyro_spy, gyro_on = LogSpy(), LO.LidarOdometry(use_gyro_prior=True)
REAL.append(gyro_on)
_real_logger, _real_start = LO.logger, LO.Module.start
LO.logger, LO.Module.start = gyro_spy, (lambda self: None)   # the dimOS runtime is not under test here
try:
    gyro_on.start()
finally:
    LO.logger, LO.Module.start = _real_logger, _real_start
up_on = [line for line in gyro_spy.lines if line.startswith("lidar odometry up")]
check("start() with the prior on (the MOSS default) logs 'prior: gyro=True' in its 'lidar odometry up' line "
      "(and section A's, prior off, said 'prior: gyro=False')",
      len(up_on) == 1 and "prior: gyro=True axis -y" in up_on[0] and len(up) == 1 and "prior: gyro=False" in up[0],
      up_on[0] if up_on else "no line")


# --- E. the optional depth path -------------------------------------------------
print("E. depth camera (live on MOSS): one pixel through the profile's mount")
o = odometry(Kiss())
o._K = (600.0, 600.0, 0.0, 0.0)
o._depth_n = LO.DEPTH_EVERY - 1                       # the next frame is the one processed
depth = types.SimpleNamespace(data=np.array([[1000]], dtype=np.uint16), ts=0.0)
LOOP.run_until_complete(o.handle_depth_image(depth))
cam_pt = None if o._pending_cam_pts is None else [float(v) for v in o._pending_cam_pts[0][0]]
check("1.000 m at the principal point, rover at the origin -> (1.156, 0, 0.075) m (MOSS camera 0.156/0/0.075, pitch 0)",
      cam_pt is not None and all(abs(a - b) < 1e-5 for a, b in zip(cam_pt, (1.156, 0.0, 0.075))), f"{cam_pt}")
drive(o, 1, SCAN_FLAT)
xyz = cloud_xyz(o.lidar.msgs[-1]).astype(np.float64)
tag = costmap2d.lidar_returns(xyz)
check("the next lidar cloud carries 360 lidar points + the camera point",
      len(xyz) == 361 and abs(xyz[-1, 2] - 0.075) < 1e-6)
check("costmap2d tags the 360 lidar points and not the camera point", int(tag.sum()) == 360 and not tag[-1])

# Open3D (behind PointCloud2) reports every cloud still alive at interpreter
# exit as a leak and turns the exit status to 1: drop the published clouds first.
for m in REAL:
    for name in ("odom", "lidar", "tf", "camera_floor"):
        sink = getattr(m, name)
        if isinstance(sink, Sink):
            sink.msgs.clear()
    m._pending_cam_pts = None
# the module-level views of section E (cloud_xyz output and its tag) keep Open3D buffers alive too
for _name in ("xyz", "tag", "cam_pt", "depth"):
    globals().pop(_name, None)
gc.collect()

print()
print("F. the first registration call fails once (thread pool start-up): absorbed, never a lost run")
reg_fail = Registration(); reg_fail.fail_first = 1
kiss6 = Kiss(); kiss6.registration = reg_fail
_kiss_mod.KissICP = lambda cfg: kiss6
o6 = LO.LidarOdometry(use_gyro_prior=False)
try:
    o6.log_every_s = 1e9
    o6.odom, o6.lidar, o6.tf, o6.camera_floor = Sink(), Sink(), Sink(), Sink()
    _ms = LO.Module.start
    LO.Module.start = lambda self: None          # the dimOS runtime is not under test here
    try:
        o6.start()
    finally:
        LO.Module.start = _ms
    check(o6._kiss is kiss6 and reg_fail.calls == 2, f"start(): the warm-up failed once and was retried ({reg_fail.calls} registration calls)")
    check(kiss6.local_map.updates == 0, "the warm-up touched no map")
    _a = np.linspace(0.0, 2.0 * math.pi, 64, endpoint=False)
    ring6 = np.stack([2.0 * np.cos(_a), 2.0 * np.sin(_a), np.zeros_like(_a)], 1)
    reg_fail.fail_first = 1
    drive(o6, 1, ring6)
    check(len(o6.odom.msgs) == 1 and reg_fail.calls == 4, f"a scan whose first registration fails is retried and published ({len(o6.odom.msgs)} odom, {reg_fail.calls} calls)")
    reg_fail.fail_first = 2
    drive(o6, 1, ring6)
    check(len(o6.odom.msgs) == 1 and o6._dropped == 1, f"two failures in a row: the revolution is dropped, nothing escapes (dropped {o6._dropped})")
    drive(o6, 1, ring6)
    check(len(o6.odom.msgs) == 2, "the next revolution publishes again")
finally:
    _kiss_mod.KissICP = lambda cfg: Kiss(cfg=cfg)

# --- G. a contact and the map ---------------------------------------------------
print()
print("G. a contact: the shaken revolution dropped, the relocation after it")


class MarkerRegistration(Registration):
    """The registration as a scan matcher on one feature: the revolution's
    marker (its farthest return) lies at bearing 0 in the world, so the
    heading the registration finds is minus the marker's bearing in the scan;
    the position stays the prediction's. A shaken sweep turned by 6 degrees
    registers 6 degrees off, as the real one did."""

    def align_points_to_map(self, points, voxel_map, initial_guess, max_correspondance_distance, kernel):
        guess = super().align_points_to_map(points, voxel_map, initial_guess, max_correspondance_distance, kernel)
        p = np.asarray(points)
        m = p[int(np.argmax(np.hypot(p[:, 0], p[:, 1])))]
        out = np.array(guess, dtype=float)
        out[:3, :3] = LO._se2(0.0, 0.0, -math.atan2(m[1], m[0]))[:3, :3]
        return out


def marker_ring(n, heading_deg):
    """One revolution of n returns on a 2 m circle, the first one a marker at
    3 m, as the lidar sees them from a rover heading heading_deg."""
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False) - math.radians(heading_deg)
    r = np.full(n, 2.0)
    r[0] = 3.0
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], 1)


_imu_clock = [5000.0]


def rev(o, pts, gyro_deg=0.0, n=1):
    """n revolutions, each after 0.1 s of gyro (6 samples at 50 Hz) turning the
    rover by gyro_deg degrees: the robot's 200 Hz IMU, thinned."""
    rate = math.radians(gyro_deg) / 0.1
    for _ in range(n):
        t = _imu_clock[0]
        _imu_clock[0] += 1.0                # a gap the integration skips: each revolution starts its own integral
        for i in range(6):
            LOOP.run_until_complete(o.handle_imu(imu(-rate, t + 0.02 * i)))
        drive(o, 1, pts)


def contact_rover(use_gyro_prior):
    """A rover at rest that has registered 10 revolutions of 300 points at
    heading 0, with a spy on the sigma each registration gets."""
    k = Kiss()
    k.registration = MarkerRegistration()
    o = odometry(k, use_gyro_prior=use_gyro_prior)
    sigmas = []
    real_register = LO.LidarOdometry._register

    def spy_register(kiss, source, guess, sigma, where="scan"):
        sigmas.append(sigma)
        return real_register(kiss, source, guess, sigma, where)

    o._register = spy_register
    rev(o, marker_ring(300, 0.0), n=10)
    return o, sigmas


def bump(o, data=True):
    LOOP.run_until_complete(o.handle_bump(Bool(data=data)))


def heading(o):
    return yaw_of(o.odom.msgs[-1].orientation)


def contact_section(spy):
    import importlib
    SHAKEN = marker_ring(90, 6.0)           # the lidar shaken: 90 returns, 6 degrees off
    check("the knobs at their defaults: window 0.4 s, drop ratio 0.5, 5 relocation revolutions, sigma x2.0",
          (LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN) == (0.4, 0.5, 5, 2.0))

    # (a) the contact, the gyro prior on and fed
    spy.lines.clear()
    o, sig = contact_rover(True)
    before = heading(o)
    bump(o)
    rev(o, SHAKEN, gyro_deg=0.4)
    check("(a) at rest, 10 revolutions of 300 points, a bump, a revolution of 90 points turned 6 deg: "
          "the yaw moves +0.40 deg (the gyro's), not 6",
          abs(before) < 1e-9 and abs(heading(o) - before - 0.4) < 0.01 and abs(heading(o) - before) <= 0.5,
          f"{before:+.3f} -> {heading(o):+.3f} deg")
    dropped = [line for line in spy.lines if "revolution dropped" in line]
    check("... one log line: 'contact: revolution dropped (90 pts, median 300), pose carried by the gyro'",
          len(dropped) == 1 and "contact: revolution dropped (90 pts, median 300), pose carried by the gyro" in dropped[0],
          dropped[0] if dropped else "no line")
    check("... counted (1), no registration for it, kiss-icp's map and threshold untouched",
          o._dropped_contact == 1 and len(sig) == 10 and o._kiss.local_map.updates == 10
          and o._kiss.adaptive_threshold.deviations == 10, f"{o._dropped_contact} dropped, {len(sig)} registrations")
    check("... the pose goes out (11 odom, 11 tf), the shaken cloud never reaches the map (10 clouds)",
          len(o.odom.msgs) == len(o.tf.msgs) == 11 and len(o.lidar.msgs) == 10)
    check("... the gyro integral consumed, kiss-icp's last pose at +0.40 deg (the next prior starts there)",
          o._gyro_acc == 0.0 and o._gyro_seen is False
          and abs(math.degrees(math.atan2(o._kiss.last_pose[1, 0], o._kiss.last_pose[0, 0])) - 0.4) < 0.01)

    # (c) the relocation: the rover truly heads +1.0 deg, the gyro said +0.4
    rev(o, marker_ring(300, 1.0), n=6)
    check("(c) after the bump, the next 5 full revolutions register with sigma x2 (0.40), the 6th with the plain 0.20",
          [round(s, 9) for s in sig[10:]] == [0.4] * 5 + [0.2], f"{sig[10:]}")
    reloc = [line for line in spy.lines if "relocat" in line]
    check("... one line at the first, 'contact: relocating over 5 revolutions (sigma x2.0)', one at the end: "
          "the registration corrected the yaw by +0.6 deg in total",
          len(reloc) == 2 and "contact: relocating over 5 revolutions (sigma x2.0)" in reloc[0]
          and "relocation done over 5 revolutions" in reloc[1] and "+0.6 deg in total" in reloc[1], f"{reloc}")
    check("... the heading the full revolutions show: +1.00 deg, the shaken +6 never published",
          abs(heading(o) - 1.0) < 0.01 and max(abs(yaw_of(m.orientation)) for m in o.odom.msgs) < 1.01, f"{heading(o):+.3f} deg")

    # (a) again, the gyro prior off: the pose is held
    spy.lines.clear()
    o, sig = contact_rover(False)
    bump(o)
    rev(o, SHAKEN, gyro_deg=0.4)
    check("(a) the prior off: dropped too, the pose held at 0.00 deg, the log says so",
          abs(heading(o)) < 1e-9 and o._dropped_contact == 1 and len(sig) == 10
          and any("revolution dropped (90 pts, median 300), pose held (no gyro prior)" in line for line in spy.lines),
          f"{heading(o):+.3f} deg")

    # (a) the prior on, but the imu stream quiet: no gyro sample since the last revolution
    spy.lines.clear()
    o, sig = contact_rover(True)
    bump(o)
    drive(o, 1, SHAKEN)                     # no imu message before this revolution
    dropped = [line for line in spy.lines if "revolution dropped" in line]
    check("(a) the prior on but no gyro sample since the last revolution: dropped, the pose held at 0.00 deg, the log says "
          "'pose held (no gyro sample since the last revolution)', not 'no gyro prior'",
          abs(heading(o)) < 1e-9 and o._dropped_contact == 1 and len(sig) == 10 and len(dropped) == 1
          and "pose held (no gyro sample since the last revolution)" in dropped[0] and "no gyro prior" not in dropped[0],
          dropped[0] if dropped else "no line")

    # (b) the control: no bump
    o, sig = contact_rover(True)
    bump(o, data=False)
    rev(o, SHAKEN, gyro_deg=0.4)
    check("(b) the same shaken revolution with no bump before it (a Bool False is none): registered, the yaw jumps to +6.0 deg",
          o._contact_t is None and abs(heading(o) - 6.0) < 0.01 and o._dropped_contact == 0
          and len(sig) == 11 and len(o.lidar.msgs) == 11, f"{heading(o):+.3f} deg")
    check("... with the plain sigma (no contact, no relocation)", abs(sig[-1] - 0.2) < 1e-12, f"{sig[-1]}")

    # (e) outside the window
    o, sig = contact_rover(True)
    bump(o)
    o._contact_t -= LO.CONTACT_WINDOW_S + 0.1         # the revolution comes 0.5 s after the contact
    rev(o, SHAKEN, gyro_deg=0.4)
    check("(e) the same revolution 0.5 s after the bump (window 0.4 s): registered as usual, +6.0 deg, nothing dropped",
          abs(heading(o) - 6.0) < 0.01 and o._dropped_contact == 0 and len(sig) == 11, f"{heading(o):+.3f} deg")

    # the running median follows the last 20 registered revolutions
    o, sig = contact_rover(True)
    rev(o, marker_ring(300, 0.0), n=15)
    rev(o, marker_ring(120, 0.0), n=20)
    bump(o)
    rev(o, SHAKEN, gyro_deg=0.4)
    check("the median is the last 20 revolutions': after 25 of 300 points then 20 of 120, 90 points is not shaken (>= 0.5 x 120)",
          o._dropped_contact == 0 and abs(heading(o) - 6.0) < 0.01, f"{o._dropped_contact} dropped")

    # start() says the knobs
    spy.lines.clear()
    up_mod = LO.LidarOdometry(use_gyro_prior=True)
    REAL.append(up_mod)
    real_start = LO.Module.start
    LO.Module.start = lambda self: None          # the dimOS runtime is not under test here
    try:
        up_mod.start()
    finally:
        LO.Module.start = real_start
    said = [line for line in spy.lines if line.startswith("lidar odometry contacts:")]
    check("start() says the contact knobs in a line of its own",
          len(said) == 1 and "within 0.4 s of a bump under 0.5 x the median" in said[0]
          and "then 5 revolutions register with sigma x2" in said[0], said[0] if said else "no line")

    # (d) the knobs, read at import: bad values fall back, a drop ratio of 0 never drops
    knobs = {"MOSS_CONTACT_DROP_RATIO": "0", "MOSS_CONTACT_WINDOW_S": "soon",
             "MOSS_RELOC_SWEEPS": "3", "MOSS_RELOC_SIGMA_GAIN": "0.5"}
    tuned = {"MOSS_RELOC_SWEEPS": "3", "MOSS_RELOC_SIGMA_GAIN": "3", "MOSS_CONTACT_WINDOW_S": "0.05"}
    os.environ.update(knobs)
    try:
        importlib.reload(LO)
        LO.logger = spy
        check("MOSS_CONTACT_DROP_RATIO=0, MOSS_RELOC_SWEEPS=3 read at import; 'soon' -> the 0.4 s default, gain 0.5 -> its floor 1.0",
              (LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN) == (0.4, 0.0, 3, 1.0),
              f"{(LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN)}")
        o, sig = contact_rover(True)
        bump(o)
        rev(o, SHAKEN, gyro_deg=0.4)
        check("(d) MOSS_CONTACT_DROP_RATIO=0 never drops: the shaken revolution registers, +6.0 deg",
              o._dropped_contact == 0 and abs(heading(o) - 6.0) < 0.01 and len(sig) == 11, f"{heading(o):+.3f} deg")

        # (d) once more, with values that change the behaviour
        for name in knobs:
            os.environ.pop(name, None)
        os.environ.update(tuned)
        importlib.reload(LO)
        LO.logger = spy
        check("MOSS_RELOC_SWEEPS=3 MOSS_RELOC_SIGMA_GAIN=3 MOSS_CONTACT_WINDOW_S=0.05 read at import (drop ratio 0.5)",
              (LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN) == (0.05, 0.5, 3, 3.0),
              f"{(LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN)}")
        o, sig = contact_rover(True)
        bump(o)
        rev(o, SHAKEN, gyro_deg=0.4)
        rev(o, marker_ring(300, 0.0), n=4)
        check("(d) after a bump and the shaken revolution (dropped), 4 full ones register with sigma 0.60, 0.60, 0.60, "
              "then the plain 0.20: 3 relocation revolutions at x3",
              o._dropped_contact == 1 and [round(s, 9) for s in sig[10:]] == [0.6] * 3 + [0.2], f"{sig[10:]}")
        o, sig = contact_rover(True)
        bump(o)
        o._contact_t -= 0.1                     # the thin revolution comes 0.1 s after the contact
        rev(o, SHAKEN, gyro_deg=0.4)
        check("(d) a thin revolution 0.1 s after the bump registers (the 0.05 s window has closed), +6.0 deg, nothing dropped",
              o._dropped_contact == 0 and abs(heading(o) - 6.0) < 0.01 and len(sig) == 11, f"{heading(o):+.3f} deg")
    finally:
        for name in {**knobs, **tuned}:
            os.environ.pop(name, None)
        importlib.reload(LO)
    check("... unset again: the defaults are back (0.4, 0.5, 5, 2.0)",
          (LO.CONTACT_WINDOW_S, LO.CONTACT_DROP_RATIO, LO.RELOC_SWEEPS, LO.RELOC_SIGMA_GAIN) == (0.4, 0.5, 5, 2.0))


contact_spy, contact_logger = LogSpy(), LO.logger
LO.logger = contact_spy
try:
    contact_section(contact_spy)
finally:
    LO.logger = contact_logger
gc.collect()

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
# section F built its own module after the clean-up above: drop its clouds too, or
# Open3D reports them as leaked at interpreter exit and the exit status turns to 1
for _m in list(REAL) + [v for k, v in list(globals().items()) if k.startswith("o") and hasattr(v, "camera_floor")]:
    for _name in ("odom", "lidar", "tf", "camera_floor"):
        _sink = getattr(_m, _name, None)
        if isinstance(_sink, Sink):
            _sink.msgs.clear()
    if hasattr(_m, "_pending_cam_pts"):
        _m._pending_cam_pts = None
for _name in ("ring6", "o6", "kiss6"):
    globals().pop(_name, None)
gc.collect()
sys.exit(1 if KO else 0)
