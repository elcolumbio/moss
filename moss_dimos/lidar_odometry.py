"""Lidar odometry for MOSS: KISS-ICP scan-to-map on the RPLIDAR C1.

dimOS ships no odometry for a 2D lidar, so this module supplies the pose. Each
revolution from RPLidarC1 (a flat cloud in `lidar_link`) is registered against
kiss-icp's local map, and the result is projected on SE(2) and published.

A planar scan leaves z/roll/pitch unobservable for a 3D ICP: the cloud is
thickened (copies at +-PLANE_THICKNESS_M) and the pose is projected on SE(2).

The prior of each registration is kiss-icp's own constant-velocity model, or a
gyro's yaw rate when `use_gyro_prior` is on and `imu` messages arrive. The
wheel (track) odometry never feeds the prior: tracks slip in every turn, and a
scan stamped at a slipping pose doubles the walls. It is only logged, as a
sanity signal. Rotation without a gyro is the known weak spot: on C1 scans of
~340 points, kiss-icp alone was measured under-estimating turns by 20-35 %.
Bench the turn tracking before trusting long runs.

A contact shakes the lidar: the revolution swept during it comes in with a
fraction of its points and registers at a wrong heading (measured: +6.4 deg
in one revolution of 97 points where the gyro moved +0.5 deg), and every
later revolution keeps that heading. So a contact on `bump` (the IMU's
BumpImu, the base's stall guard, the explorer's back-off) is remembered: a
revolution within MOSS_CONTACT_WINDOW_S of it that carries fewer than
MOSS_CONTACT_DROP_RATIO times the running median point count is not
registered (the pose is carried by the gyro, or held without it, and its
cloud never reaches the map), and the next MOSS_RELOC_SWEEPS registered
revolutions search with their sigma times MOSS_RELOC_SIGMA_GAIN.

Frames and heights:
  * `world` is kiss-icp's own frame: every run starts at the origin, facing +x.
  * `odom` is published on the floor plane (z = 0) - see handle_pointcloud.
  * the lidar returns are lifted to exactly the profile's lidar_height_m, in
    the `lidar` cloud and in the base_link->lidar_link tf. costmap2d recognises
    lidar returns by that exact height, so both read the same profile value.

Optional depth camera: when `depth_image` and `camera_info` are fed (a
RealSense built with depth on), obstacles seen by the camera are added to the
next `lidar` clouds at their true height, and the bare floor it sees goes out
on `camera_floor` (z = 0). The camera mount comes from the profile
(camera_xyz_base_m, camera_pitch_deg). The MOSS blueprints build the RealSense
with depth on (aligned to colour, CAMERA_FPS), so this path is live there and
costmap2d maps the camera obstacles next to the lidar returns.

Streams
  pointcloud              : In[PointCloud2]  one revolution, lidar_link (rplidar_c1.py)
  imu                     : In[Imu]          gyro (angular_velocity) for the optional rotation prior
  bump                    : In[Bool]         a contact: the shaken revolution after it is dropped
  coordinator_joint_state : In[JointState]   base/vx, base/vy, base/wz positions = wheel odometry, logged only
  depth_image             : In[Image]        optional depth camera, DEPTH16 mm, aligned to colour
  camera_info             : In[CameraInfo]   its colour intrinsics
  odom   : Out[PoseStamped]  base pose in `world`, z = 0
  lidar  : Out[PointCloud2]  the revolution re-expressed in `world` (costmap2d input)
  camera_floor : Out[PointCloud2]  world-frame floor samples from the depth camera, z = 0
  tf     : Out[TFMessage]    world->base_link, base_link->lidar_link
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool   # the SAME Bool class as BumpImu / the coordinator / the costmap: one 'bump' stream type

from moss_dimos.robot_profile import RobotProfile, current as _current_profile

logger = setup_logger()

PLANE_THICKNESS_M = 0.05
DEPTH_STRIDE = 8               # 640x480 -> 80x60 samples per frame: what the map needs, not more
DEPTH_EVERY = 1                # every depth frame: the camera runs at 5 fps on MOSS, nothing to skip
CAMERA_MIN_RANGE_M = 0.12      # the D455's close-range depth (Min-Z update): nearer than this is noise or the rover's own nose
DEPTH_MAX_M = 3.0              # beyond that the floor noise (1-2 % of range) leaks into the obstacle band
OBSTACLE_MAX_M = 3.0           # camera obstacles are trusted up to this range; past it, depth reflections
                               # on a glossy floor arm phantom obstacles. Floor samples keep the full
                               # DEPTH_MAX_M range: erasing stays cheap.
OBSTACLE_Z_M = (0.12, 1.30)    # world z band the camera turns into floor obstacles. Upper bound 1.30, not 0.70: a table top is a BLOCK - the rover goes around tables, never between the legs


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    """A number from the environment, read once at import. Unset or empty ->
    `default`; not a number -> `default`, said in the log; outside [lo, hi]
    -> the nearest bound, said in the log."""
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError:
        logger.warning(f"lidar odometry: {name}={raw!r} is not a number - using {default}")
        return default
    if not lo <= value <= hi:
        bounded = hi if value > hi else lo      # below lo, or nan
        logger.warning(f"lidar odometry: {name}={raw!r} is outside [{lo}, {hi}] - using {bounded}")
        return bounded
    return value


# A contact (`bump`) shakes the lidar: the revolution swept during it comes in thin and
# registers at a wrong heading, and the map jumps. These four decide what happens next.
CONTACT_WINDOW_S = _env_float("MOSS_CONTACT_WINDOW_S", 0.4, 0.0, 5.0)        # a revolution this soon after a contact may be the shaken one
CONTACT_DROP_RATIO = _env_float("MOSS_CONTACT_DROP_RATIO", 0.5, 0.0, 1.0)    # ... and is dropped under this share of the median point count (0 = never)
RELOC_SWEEPS = int(round(_env_float("MOSS_RELOC_SWEEPS", 5, 0, 50)))        # registered revolutions after a contact that search wider (0 = none)
RELOC_SIGMA_GAIN = _env_float("MOSS_RELOC_SIGMA_GAIN", 2.0, 1.0, 10.0)      # their sigma (kernel, and 3 sigma of correspondence distance) times this
COUNT_HISTORY = 20             # registered revolutions in the running median of the point count


def _profile() -> RobotProfile:
    """The geometry the tf, the lidar lift and the depth projection use, read
    at call time so a MOSS_* override takes effect without a restart."""
    return _current_profile()


def _yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _se2(dx: float, dy: float, dyaw: float) -> np.ndarray:
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([[c, -s, 0.0, dx], [s, c, 0.0, dy], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


def sweep_times(pts: np.ndarray) -> np.ndarray:
    """When, within one revolution, each point was taken: 0 for the first, 1 for
    the last, from the unwrapped bearing of the points in scan order (the C1
    publishes a revolution in sweep order; its bearing runs one way round). A
    revolution whose bearings do not progress (fewer than a few degrees of span)
    falls back to the point order. kiss-icp's deskew moves each point by that
    fraction of the motion prior, so a scan bent by a fast pivot comes straight."""
    n = len(pts)
    if n < 2:
        return np.zeros(n)
    ang = np.unwrap(np.arctan2(pts[:, 1], pts[:, 0]))
    span = ang[-1] - ang[0]
    if abs(span) < math.radians(5.0):
        return np.linspace(0.0, 1.0, n)
    t = (ang - ang[0]) / span
    return np.clip(t, 0.0, 1.0)


def split_floor_and_obstacles(bx: np.ndarray, by: np.ndarray, bz: np.ndarray,
                              floor_z: np.ndarray, cell_m: float = 0.05) -> tuple[np.ndarray, np.ndarray]:
    """Masks (obstacle, floor) over base-frame depth points.

    Obstacle = above the floor threshold and below OBSTACLE_Z_M[1] (1.30 m:
    table tops included, so a table is a block the rover drives around).
    Floor = at floor level. Under a table the camera sees BOTH the top and
    the floor between the legs on the same ground cell; the obstacle wins,
    so floor samples on a cell that also holds an obstacle are dropped -
    otherwise hit and miss cancel and the table vanishes from the costmap.
    """
    obst = (bz > floor_z) & (bz < OBSTACLE_Z_M[1])
    floor = (bz <= floor_z) & (bz > -0.10)
    if obst.any() and floor.any():
        keys_o = set(map(tuple, np.floor(np.stack([bx[obst], by[obst]], 1) / cell_m).astype(np.int64)))
        fk = np.floor(np.stack([bx, by], 1) / cell_m).astype(np.int64)
        under = np.array([tuple(k) in keys_o for k in fk])
        floor = floor & ~under
    return obst, floor


class LidarOdometry(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]
    bump: In[Bool]                  # a contact (BumpImu, the stall guard, the explorer's back-off): the shaken revolution is dropped
    coordinator_joint_state: In[JointState]
    depth_image: In[Image]          # depth camera (aligned to colour), DEPTH16 mm
    camera_info: In[CameraInfo]     # colour intrinsics (= aligned depth intrinsics)
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    camera_floor: Out[PointCloud2]   # world-frame floor samples (z = 0) the depth camera saw bare
    tf: Out[TFMessage]

    def __init__(self, max_range_m: float = 12.0, min_range_m: float = 0.35,
                 voxel_size_m: float = 0.05, initial_threshold_m: float = 0.3,
                 gyro_axis: str = "-y", use_gyro_prior: bool = True, deskew: bool = False,
                 world_frame: str = "world",
                 base_frame: str = "base_link", lidar_frame: str = "lidar_link",
                 log_every_s: float = 2.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.max_range_m, self.min_range_m, self.voxel_size_m = max_range_m, min_range_m, voxel_size_m
        self.initial_threshold_m = initial_threshold_m
        # which IMU axis carries the robot's yaw rate, with its sign. A D455F
        # motion module reports in the camera's optical frame (x right, y
        # down, z forward): a level camera looking forward gives
        # yaw_rate = -gyro.y. A camera mounted upside down flips the sign.
        self.gyro_axis = gyro_axis
        self.use_gyro_prior = use_gyro_prior
        self.deskew = deskew          # kiss-icp straightens each revolution by the motion prior (sweep_times): a full-power pivot bends a 10 Hz
                                      # scan by 25 degrees. OFF by default: on the rover kiss-icp's deskew dies at its first thread
                                      # (pthread_create: Invalid argument) and the odometry with it; MOSS_DESKEW=1 is the experiment
        self.world_frame, self.base_frame, self.lidar_frame = world_frame, base_frame, lidar_frame
        self.log_every_s = log_every_s
        # dimOS pickles module instances into their worker: no lock, no native
        # object before start().
        self._kiss: Any = None
        self._dropped = 0                  # revolutions lost to a registration that failed twice
        self._lock: Any = None
        self._n = 0
        self._last_log = 0.0
        self._last_ms = 0.0
        self.pose2d = (0.0, 0.0, 0.0)
        # gyro integration since the last scan (+ totals per axis for calibration)
        self._gyro_last_ts: float | None = None
        self._gyro_acc = 0.0
        self._gyro_seen = False
        self._gyro_totals = np.zeros(3)
        self._wheel: tuple[float, float, float] | None = None   # latest wheel odometry: logged only
        self._prior_used = "cv"
        self._K: tuple[float, float, float, float] | None = None
        self._pose_hist: list[tuple[float, float, float, float]] = []   # (wall ts, x, y, yaw), last ~3 s
        self._depth_n = 0
        self._depth_pts_last = 0
        self._pending_cam_pts = None
        # contacts: the last one's monotonic stamp, the point counts of the last
        # registered revolutions, and the relocation that follows a contact
        self._contact_t: float | None = None
        self._dropped_contact = 0          # revolutions dropped as shaken by a contact
        self._counts: list[int] = []
        self._reloc_armed_t: float | None = None   # the contact stamp that armed the current relocation
        self._reloc_left = 0               # registered revolutions still to search wider
        self._reloc_n = 0                  # ... done so far in this relocation
        self._reloc_yaw = 0.0              # the yaw the registration corrected over them, rad
        self._reloc_announce = False

    # ── lifecycle ──────────────────────────────────────────────────────
    @rpc
    def start(self) -> None:
        super().start()
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        cfg = KISSConfig()
        cfg.data.max_range = self.max_range_m
        cfg.data.min_range = self.min_range_m
        cfg.data.deskew = self.deskew
        cfg.mapping.voxel_size = self.voxel_size_m
        cfg.mapping.max_points_per_voxel = 30
        # kiss-icp defaults are tuned for cars (2 m initial threshold)
        cfg.adaptive_threshold.initial_threshold = self.initial_threshold_m
        cfg.adaptive_threshold.min_motion_th = 0.02
        self._kiss = KissICP(cfg)
        self._lock = threading.Lock()
        self._warm_up()
        logger.info(f"lidar odometry up (kiss-icp, voxel {self.voxel_size_m} m, range "
                    f"{self.min_range_m}-{self.max_range_m} m, prior: gyro={self.use_gyro_prior} "
                    f"axis {self.gyro_axis}, wheels NEVER) -> {self.world_frame}")
        logger.info(f"lidar odometry contacts: a revolution within {CONTACT_WINDOW_S:g} s of a bump under "
                    f"{CONTACT_DROP_RATIO:g} x the median point count is dropped; then {RELOC_SWEEPS} "
                    f"revolutions register with sigma x{RELOC_SIGMA_GAIN:g}")

    @rpc
    def stop(self) -> None:
        super().stop()

    # ── registration, with the first-call failure absorbed ──────────────
    def _warm_up(self) -> None:
        """One registration on a synthetic ring before the first scan. On this
        board the first call into kiss-icp's registration can fail once with
        'pthread_create has failed: Invalid argument' (its thread pool starting
        up); the next call works. Taking that failure here keeps the first real
        revolution and keeps the error out of the run log. The map is not
        touched (registration only)."""
        k = self._kiss
        ang = np.linspace(0.0, 2.0 * math.pi, 64, endpoint=False)
        ring = np.stack([2.0 * np.cos(ang), 2.0 * np.sin(ang), np.zeros_like(ang)], 1)
        try:
            source, _ = k.voxelize(ring)
            sigma = k.adaptive_threshold.get_threshold()
            self._register(k, source, np.eye(4), sigma, where="warm-up")
        except Exception as exc:  # noqa: BLE001 - a warm-up never stops the module
            logger.warning(f"lidar odometry: registration warm-up skipped ({type(exc).__name__}: {exc})")

    @staticmethod
    def _register(k, source, initial_guess, sigma, where: str = "scan"):
        """align_points_to_map with ONE retry: a RuntimeError from the first
        attempt (thread pool start-up) is logged and the call repeated; a second
        failure propagates to the caller, which drops that revolution."""
        try:
            return k.registration.align_points_to_map(
                points=source, voxel_map=k.local_map, initial_guess=initial_guess,
                max_correspondance_distance=3 * sigma, kernel=sigma)
        except RuntimeError as exc:
            logger.warning(f"lidar odometry: registration failed once at {where} ({exc}) - retrying")
            return k.registration.align_points_to_map(
                points=source, voxel_map=k.local_map, initial_guess=initial_guess,
                max_correspondance_distance=3 * sigma, kernel=sigma)

    # ── priors ─────────────────────────────────────────────────────────
    async def handle_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        ts = float(getattr(msg, "ts", 0.0) or time.time())
        if self._gyro_last_ts is not None:
            dt = ts - self._gyro_last_ts
            if 0.0 < dt < 0.1:
                v = np.array([w.x, w.y, w.z], dtype=float)
                self._gyro_totals += v * dt
                axis = {"x": 0, "y": 1, "z": 2}[self.gyro_axis[-1]]
                sign = -1.0 if self.gyro_axis.startswith("-") else 1.0
                self._gyro_acc += sign * v[axis] * dt
                self._gyro_seen = True
        self._gyro_last_ts = ts

    async def handle_bump(self, msg: Bool) -> None:
        """A contact: stamped, nothing else here. The next revolutions read the
        stamp (handle_pointcloud): the shaken one is dropped, the relocation
        is armed."""
        if bool(getattr(msg, "data", False)):
            self._contact_t = time.monotonic()

    async def handle_coordinator_joint_state(self, msg: JointState) -> None:
        names = list(msg.name); pos = list(msg.position)
        try:
            ix, iy, ith = (names.index(n) for n in ("base/vx", "base/vy", "base/wz"))
        except ValueError:
            return
        self._wheel = (float(pos[ix]), float(pos[iy]), float(pos[ith]))

    async def handle_camera_info(self, msg: CameraInfo) -> None:
        K = list(msg.K or [])
        if len(K) == 9 and K[0] > 0:
            self._K = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))

    async def handle_depth_image(self, msg: Image) -> None:
        """Sparse obstacle cloud from the depth image, in `world`, onto `lidar`.

        The 2D lidar sees one plane: it misses objects lower than its scan and
        thin dark legs. The depth camera sees them: every DEPTH_STRIDE-th pixel
        is back-projected, placed in the world with the pose at capture time,
        cropped to the rover's height band, voxel-deduplicated and attached to
        the next `lidar` clouds."""
        self._depth_n += 1
        if self._K is None or self._depth_n % DEPTH_EVERY:
            return
        d = np.asarray(msg.data)
        if d.ndim == 3:
            d = d[..., 0]
        fx, fy, cx, cy = self._K
        h, w = d.shape
        vs, us = np.mgrid[0:h:DEPTH_STRIDE, 0:w:DEPTH_STRIDE]
        z = d[vs, us].astype(np.float64) / 1000.0
        ok = (z > CAMERA_MIN_RANGE_M) & (z < DEPTH_MAX_M)
        if not ok.any():
            return
        z, us, vs = z[ok], us[ok], vs[ok]
        xo = (us - cx) * z / fx          # optical x (right)
        yo = (vs - cy) * z / fy          # optical y (down)
        # optical (x right, y down, z forward) -> base (x forward, y left, z up),
        # camera pitched down by the profile's camera_pitch_deg: forward axis =
        # (cos, 0, -sin), down axis = (-sin, 0, -cos); then the mount offset
        prof = _profile()
        pitch = math.radians(prof.camera_pitch_deg)
        cam = prof.camera_xyz_base_m
        cp, sp = math.cos(pitch), math.sin(pitch)
        bx = z * cp - yo * sp + cam[0]
        by = -xo + cam[1]
        bz = -z * sp - yo * cp + cam[2]
        # floor threshold grows with range (depth noise ~1-2 % of range): a 5 cm
        # chair base is an obstacle at 1 m, floor noise is not an obstacle at 3 m
        floor_z = 0.03 + 0.03 * np.clip(bx - 1.0, 0.0, None)
        obst, floor = split_floor_and_obstacles(bx, by, bz, floor_z)
        if not obst.any() and not floor.any():
            return
        fx, fy = bx[floor], by[floor]          # bare floor: published separately, z = 0
        near = np.hypot(bx, by) < OBSTACLE_MAX_M
        bx, by, bz = bx[obst & near], by[obst & near], bz[obst & near]
        # pose at the frame's capture time: the depth handler runs 50-200 ms
        # late, and at 17 deg/s the latest pose would smear the camera points
        # by several degrees per frame
        fts = float(getattr(msg, "ts", 0.0) or 0.0)
        pose = self._pose_at(fts) if fts > 0 else self.pose2d
        x, y, yaw = pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        if len(fx):
            fwx, fwy = c * fx - s_ * fy + x, s_ * fx + c * fy + y
            fpts = np.stack([fwx, fwy, np.zeros_like(fwx)], axis=1)
            fkeys = np.floor(fpts / 0.05).astype(np.int64)
            _, fidx = np.unique(fkeys, axis=0, return_index=True)
            self.camera_floor.publish(PointCloud2.from_numpy(fpts[fidx].astype(np.float32), frame_id=self.world_frame, timestamp=time.time()))
        if len(bx) == 0:
            return
        wx, wy = c * bx - s_ * by + x, s_ * bx + c * by + y
        pts = np.stack([wx, wy, bz], axis=1)
        # voxel-deduplicate at the map resolution
        keys = np.floor(pts / 0.05).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts = pts[idx].astype(np.float32)
        self._depth_pts_last = len(pts)
        # attached to EVERY lidar revolution until replaced (0.5 s cap): the
        # 10 Hz lidar against 7.5 Hz depth would otherwise leave one revolution
        # in four without camera points, and the camera points would blink.
        self._pending_cam_pts = (pts, time.monotonic())

    def _pose_at(self, ts: float) -> tuple[float, float, float]:
        """Pose interpolated at wall time ts from the recent history (else the latest)."""
        h = self._pose_hist
        if len(h) < 2 or ts <= h[0][0]:
            return self.pose2d if not h else (h[0][1], h[0][2], h[0][3])
        if ts >= h[-1][0]:
            return (h[-1][1], h[-1][2], h[-1][3])
        for a, b in zip(h, h[1:]):
            if a[0] <= ts <= b[0]:
                f = (ts - a[0]) / max(b[0] - a[0], 1e-6)
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw)
        return self.pose2d

    def _prior_delta(self) -> np.ndarray:
        """Motion since the last scan, in the previous lidar frame (SE(2) as 4x4)."""
        last_delta = np.asarray(self._kiss.last_delta)
        dx, dy = float(last_delta[0, 3]), float(last_delta[1, 3])
        dyaw = math.atan2(last_delta[1, 0], last_delta[0, 0])
        used = []
        # the wheels are NEVER consulted here - see the module docstring
        if self.use_gyro_prior and self._gyro_seen:
            dyaw = self._gyro_acc
            used.append("gyro")
        self._prior_used = "+".join(used) or "cv"
        return _se2(dx, dy, dyaw)

    # ── contacts ───────────────────────────────────────────────────────
    def _arm_relocation(self) -> None:
        """A contact not seen yet arms the relocation: the next RELOC_SWEEPS
        registered revolutions search wider. A contact during a relocation
        starts its count again; the yaw correction keeps adding up."""
        stamp = self._contact_t
        if stamp is None or stamp == self._reloc_armed_t:
            return
        self._reloc_armed_t = stamp
        if RELOC_SWEEPS > 0:
            self._reloc_left = RELOC_SWEEPS
            self._reloc_announce = True

    def _shaken(self, n_pts: int, t0: float) -> float | None:
        """The running median point count when this revolution is a shaken
        sweep to drop (within CONTACT_WINDOW_S of a contact, under
        CONTACT_DROP_RATIO times the median of the last registered ones),
        else None."""
        stamp = self._contact_t
        if stamp is None or t0 - stamp > CONTACT_WINDOW_S or not self._counts:
            return None
        median = float(np.median(self._counts))
        return median if n_pts < CONTACT_DROP_RATIO * median else None

    def _carry(self, k, n_pts: int, median: float) -> np.ndarray:
        """The pose for a dropped revolution: the last one turned by the gyro
        integral when the prior is on and fed, else the last one unchanged.
        kiss-icp's last pose follows it (the next prior starts from there);
        its map and its threshold are not touched."""
        last = np.asarray(k.last_pose)
        turned = self.use_gyro_prior and self._gyro_seen
        pose = last @ _se2(0.0, 0.0, self._gyro_acc) if turned else last
        k.last_pose = pose
        self._gyro_acc, self._gyro_seen = 0.0, False
        self._dropped_contact += 1
        if turned:
            how = "pose carried by the gyro"
        elif self.use_gyro_prior:      # the prior is on but the imu stream went quiet
            how = "pose held (no gyro sample since the last revolution)"
        else:
            how = "pose held (no gyro prior)"
        logger.warning(f"lidar odom contact: revolution dropped ({n_pts} pts, median {median:.0f}), {how}; "
                       f"dropped by contacts so far: {self._dropped_contact}")
        return pose

    # ── the work ───────────────────────────────────────────────────────
    async def handle_pointcloud(self, msg: PointCloud2) -> None:
        out = msg.as_numpy()
        pts = np.asarray(out[0] if isinstance(out, tuple) else out, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 30:
            return
        t0 = time.monotonic()
        thick = np.concatenate([pts, pts + [0, 0, PLANE_THICKNESS_M], pts - [0, 0, PLANE_THICKNESS_M]])
        with self._lock:
            k = self._kiss
            self._arm_relocation()
            median = self._shaken(len(pts), t0)
            shaken = median is not None
            if shaken:
                new_pose = self._carry(k, len(pts), median)
            else:
                new_pose = self._register_revolution(k, pts, thick)
                if new_pose is None:
                    return
        pose = np.asarray(new_pose)
        R, t = pose[:3, :3], pose[:3, 3]
        x, y, yaw = float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])
        now_wall = time.time()
        self._pose_hist.append((now_wall, x, y, yaw))
        self._pose_hist = [h for h in self._pose_hist if now_wall - h[0] < 3.0]
        self.pose2d = (x, y, yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        R2 = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        lidar_h = _profile().lidar_height_m    # costmap2d tags lidar returns by this exact z
        world_pts = (pts @ R2.T) + np.array([x, y, lidar_h])
        ts = time.time(); q = _yaw_quat(yaw)
        # odom on the FLOOR PLANE (z=0): dimOS's planner measures 3D distances
        # (goal_tolerance 0.2 m, path checks down to 0.01 m) - an odom published
        # at lidar height keeps the robot lidar_height_m away from every z=0
        # goal FOREVER. The lidar height lives in the tf child and in the
        # world-cloud transform, never in the pose.
        self.odom.publish(PoseStamped(ts, self.world_frame, position=Vector3(x, y, 0.0), orientation=q))
        if not shaken:      # a shaken sweep: its pose goes out, its cloud never reaches the map
            cam = self._pending_cam_pts
            if cam is not None and time.monotonic() - cam[1] < 0.5:
                world_pts = np.vstack([world_pts, cam[0]])
            self.lidar.publish(PointCloud2.from_numpy(world_pts.astype(np.float32), frame_id=self.world_frame, timestamp=ts))
        self.tf.publish(TFMessage(
            Transform(translation=Vector3(x, y, 0.0), rotation=q, frame_id=self.world_frame, child_frame_id=self.base_frame, ts=ts),
            Transform(translation=Vector3(0.0, 0.0, lidar_h), rotation=Quaternion(0, 0, 0, 1),
                      frame_id=self.base_frame, child_frame_id=self.lidar_frame, ts=ts)))
        if shaken:
            return
        self._n += 1
        self._last_ms = (time.monotonic() - t0) * 1000.0
        if t0 - self._last_log >= self.log_every_s:
            self._last_log = t0
            g = np.degrees(self._gyro_totals); w = self._wheel
            logger.info(f"lidar odom #{self._n}: x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}deg "
                        f"({len(pts)} pts, {self._last_ms:.1f} ms, prior {self._prior_used}, depth cloud "
                        f"{self._depth_pts_last} pts) | gyro integral "
                        f"x={g[0]:+.1f} y={g[1]:+.1f} z={g[2]:+.1f}deg | wheels "
                        + (f"x={w[0]:+.3f} y={w[1]:+.3f} th={math.degrees(w[2]):+.1f}deg" if w else "none"))

    def _register_revolution(self, k, pts: np.ndarray, thick: np.ndarray) -> np.ndarray | None:
        """Register one revolution into kiss-icp and update its state; the new
        pose, or None when the registration failed twice (revolution dropped).
        Called under the lock. During a relocation the sigma is RELOC_SIGMA_GAIN
        times the adaptive one, from the same prior (the gyro's heading when
        the prior is on)."""
        prior = self._prior_delta()
        frame = k.preprocessor.preprocess(thick, np.tile(sweep_times(pts), 3) if self.deskew else np.zeros(len(thick)), prior)
        source, frame_down = k.voxelize(frame)
        sigma = k.adaptive_threshold.get_threshold()
        relocating = self._reloc_left > 0
        if relocating:
            sigma *= RELOC_SIGMA_GAIN       # after a contact: search wider, from the prior's heading
            if self._reloc_announce:
                self._reloc_announce = False
                logger.info(f"lidar odom contact: relocating over {self._reloc_left} revolutions "
                            f"(sigma x{RELOC_SIGMA_GAIN:.1f})")
        initial_guess = np.asarray(k.last_pose) @ prior
        try:
            new_pose = self._register(k, source, initial_guess, sigma)
        except RuntimeError as exc:
            self._dropped += 1
            logger.warning(f"lidar odometry: registration failed twice, revolution dropped ({exc}); "
                           f"dropped so far: {self._dropped}")
            return None
        corr = np.linalg.inv(initial_guess) @ np.asarray(new_pose)
        k.adaptive_threshold.update_model_deviation(corr)
        k.local_map.update(frame_down, new_pose)
        k.last_delta = np.linalg.inv(np.asarray(k.last_pose)) @ new_pose
        k.last_pose = new_pose
        # consume the priors
        self._gyro_acc, self._gyro_seen = 0.0, False
        self._counts.append(len(pts))
        del self._counts[:-COUNT_HISTORY]
        if relocating:
            self._reloc_yaw += math.atan2(corr[1, 0], corr[0, 0])
            self._reloc_n += 1
            self._reloc_left -= 1
            if self._reloc_left == 0:
                logger.info(f"lidar odom contact: relocation done over {self._reloc_n} revolutions, "
                            f"the registration corrected the yaw by {math.degrees(self._reloc_yaw):+.1f} deg in total")
                self._reloc_n, self._reloc_yaw = 0, 0.0
        return new_pose
