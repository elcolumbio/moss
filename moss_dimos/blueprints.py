"""dimOS blueprints for MOSS, and the coordinator they all share.

Entry points (pyproject [project.entry-points."dimos.blueprints"]):

    dimos run moss-dimos.moss-base      coordinator + the tracked base, twist on cmd_vel
    dimos run moss-dimos.moss-gamepad   base + gamepad, direct drive
    dimos run moss-dimos.moss-nav       mapping + pad teleop, recording
    dimos run moss-dimos.moss-explore   mapping + planner + frontier
                                        exploration + pad teleop, recording

The chain on the mapping blueprints: RPLidarC1 -> LidarOdometry ->
MossCostMap, the coordinator, and on moss-explore RecoveringPlanner,
MovementManager and Explorer2. All four blueprints are built on first
access (module __getattr__ below), never at import: importing this module
costs the coordinator class and the two side effects below, nothing more.

Nothing here moves the robot by itself:
  * moss-gamepad and moss-nav have no planner and no explorer: the pad drives
    the coordinator directly (tele_cmd_vel -> cmd_vel), and only while its
    deadman is held.
  * moss-explore's explorer starts only on Bool(True) on `explore_cmd`
    (tools/explore_ctl.py start, the cockpit), and the planner moves only
    toward a goal on `goal_request` / `clicked_point`. MovementManager
    arbitrates: any pad input cancels navigation.
  * the adapter's 0.5 s deadman and the firmware's 300 ms deadman are
    untouched.

Devices (moss_ports): the QT Py and the C1 by their /dev/serial/by-id path,
MOSS_PORT / MOSS_LIDAR_PORT win. A missing device resolves to a sentinel
path that cannot open - logged as an error at build time and again by the
module that fails to open it - never to another device.

Switches (env, read when moss-nav / moss-explore is built; robot_profile.env_on
parses them: 0, false, no or off = off, any other value = on, unset = the default):
    MOSS_CAMERA=0     no RealSense (default on: colour 640x480 @ 5 fps, JPEG on the bus, plus depth aligned
                      to colour at the same rate, turned into map obstacles by lidar_odometry)
    MOSS_WRIST=0      no wrist camera (default on: the UVC camera on the arm, 640x480 @ 5 fps,
                      JPEG on the bus as wrist_image; absent device = warning, module skipped)
    MOSS_WRIST_DEV    wrist camera: a /dev/videoN path or an index, instead of the by-id lookup
    MOSS_RS_SERIAL    pick the RealSense by serial number
    MOSS_RECORD=0     no recording (default on: MossRecorder, <MOSS_STATE_DIR>/recordings/moss.db)
    MOSS_RERUN=1      add the stock Rerun/websocket viewers (default off: no 7779/9877/3030 on MOSS)

Why the coordinator is a subclass defined HERE. An out-of-tree drive train
needs two things, and the second one is not obvious:

1. The 'moss' adapter name must be registered. dimOS's drive_trains manifest
   scan only walks its own package, so external packages use the public
   registry escape hatch (register_path, below).

2. That registration must also happen in the process that BUILDS the
   adapter. dimOS deploys every module into a `forkserver` worker
   (dimos/core/coordination/python_worker.py), and a forkserver worker is a
   fresh interpreter: it re-imports dimos, gets a brand-new
   twist_base_adapter_registry, and never imports this module on its own.
   Registering in the CLI parent alone is not enough -
   ControlCoordinator._setup_hardware runs in the worker and dies with:

       KeyError: "Unknown twist base adapter: moss.
                  Available: ['flowbase', 'mock_twist_base', ...]"

   The worker does import whatever it needs to UNPICKLE the module class it
   was told to deploy, and classes pickle by reference. So the coordinator
   handed to dimOS is our own subclass, defined here: unpickling
   `moss_dimos.blueprints:MossControlCoordinator` in the worker imports this
   module, which runs the registration above the class. That mechanism holds
   for fork, forkserver and spawn alike. The same import installs the LCM
   queue clamp (lcm_latest) in the worker. Keep the registration, the clamp
   and the class in this one module.

   Subclassing the coordinator is dimOS's own documented pattern for this
   (see dimos/robot/manipulators/common/blueprints.py:coordinator(cls=...));
   it requires instance_name="ControlCoordinator" so the shipped RPC clients
   still find the coordinator by the name they expect, and ControlCoordinator
   .start() warns if a subclass omits it.

The wrist camera module (WristCamera) is a class built on first use and
served by the same module __getattr__, so dimOS pickles it by reference too.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import time
from typing import Any, Callable

from dimos.control.components import (HardwareComponent, HardwareType,
                                      make_twist_base_joints)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.hardware.drive_trains.registry import twist_base_adapter_registry
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger

from moss_dimos import robot_profile
from dimos_lcm.std_msgs import Bool   # the SAME Bool class as Explorer2 / RecoveringPlanner: one 'bump' stream type on the bus
from moss_dimos.moss_adapter import telemetry_payload
from moss_dimos.robot_profile import (MOSS_LIDAR_MISSING, MOSS_QTPY_MISSING,
                                      RobotProfile)

_logger = setup_logger()

# Register the MOSS base adapter under its name, in every process that imports
# this module (see the docstring). Registering the same path twice is a no-op;
# dimOS raises only when "moss" already points somewhere else - then the first
# registration stays in force, and that must be said, not swallowed.
MOSS_ADAPTER_NAME = "moss"
MOSS_ADAPTER_PATH = "moss_dimos.moss_adapter:MossBaseAdapter"
try:
    twist_base_adapter_registry.register_path(MOSS_ADAPTER_NAME, MOSS_ADAPTER_PATH)
except ValueError as _exc:
    _existing = getattr(twist_base_adapter_registry, "_factory_paths", {}).get(MOSS_ADAPTER_NAME)
    _logger.warning(f"twist base adapter '{MOSS_ADAPTER_NAME}' is already registered as "
                    f"{_existing or 'another path'!r}; {MOSS_ADAPTER_PATH!r} is NOT used "
                    f"in this process ({_exc})")

# Velocity commands must never queue up (stale twists drive the rover past
# the point asked): clamp the LCM queue of command channels to one message.
from moss_dimos.lcm_latest import install as _install_latest_only  # noqa: E402
_install_latest_only()

# That clamp is LCM-only, and zenoh's subscriber has no queue bound to set, so
# the coordinator drops its own stale twists below (moss_dimos.latest_only).
from moss_dimos.latest_only import LatestOnly  # noqa: E402

LATEST_ONLY_STREAM = "twist_command"

# The shipped dimOS RPC clients address the coordinator by this name; keep it
# even though the class below is a subclass.
COORDINATOR_INSTANCE_NAME = "ControlCoordinator"

_base_joints = make_twist_base_joints("base")


class MossControlCoordinator(ControlCoordinator):
    """Stock ControlCoordinator, deployed under this package's import path.

    One job of plumbing: make the forkserver worker import
    moss_dimos.blueprints (see this module's docstring), which is what
    registers the 'moss' twist base adapter in the process that actually
    instantiates it;

    plus one guard: twist_command is consumed latest-only (see below),

    and one window: `base_telemetry` republishes, every TELEMETRY_PERIOD_S,
    the firmware telemetry of any adapter that keeps one (the MOSS adapter:
    battery, mode, track outputs - see moss_adapter.telemetry_payload). The
    cockpit reads the battery there, so nothing but this process ever opens
    the motor board's port. An adapter without telemetry publishes nothing.
    """

    base_telemetry: Out[String]
    bump: Out[Bool]                 # a stall of the base (moss_adapter stall guard) = a contact: the planner's escape backs off

    TELEMETRY_PERIOD_S = 0.5

    # Run the control loop in its own process: sharing a worker (and its GIL)
    # with a busy module - a camera, a point cloud - delays the execution of
    # twists even with the command queue clamped to one.
    dedicated_worker = True

    @rpc
    def start(self) -> None:
        super().start()
        if getattr(self, "_telemetry_running", False):
            return
        self._telemetry_running = True
        self.spawn(self._telemetry_loop())

    @rpc
    def stop(self) -> None:
        self._telemetry_running = False
        super().stop()

    async def _telemetry_loop(self) -> None:
        """2 Hz: one String JSON per adapter that has telemetry. Never raises:
        a bad line or a publish error is logged once per cause and the loop
        goes on (it must not take the control process down with it)."""
        last_error = None
        while getattr(self, "_telemetry_running", False):
            try:
                with self._hardware_lock:
                    connected = list(self._hardware.values())
                now = time.time()
                for hw in connected:
                    payload = telemetry_payload(getattr(hw, "adapter", None), now)
                    if payload is not None:
                        self.base_telemetry.publish(String(data=json.dumps(payload)))
                        stalls = int(payload.get("stall_count") or 0)
                        if stalls > getattr(self, "_stalls_seen", 0):
                            self._stalls_seen = stalls
                            _logger.warning(f"base stall #{stalls}: publishing bump (contact) for the planner's escape")
                            if getattr(self.bump, "transport", None) is not None:
                                self.bump.publish(Bool(data=True))
                last_error = None
            except Exception as exc:  # noqa: BLE001
                cause = f"{type(exc).__name__}: {exc}"
                if cause != last_error:
                    _logger.warning(f"base_telemetry: {cause}")
                    last_error = cause
            await asyncio.sleep(self.TELEMETRY_PERIOD_S)

    def _make_stream_cb(self, stream: str) -> Callable[[Any], None]:
        """Consume twist_command latest-only, whatever the transport.

        lcm_latest clamps the LCM queue to one message; zenoh's subscriber has
        no such bound and dimOS exposes no capacity for it, so the drop has to
        happen here. dimOS builds a fresh callback per subscription (hardware
        add/remove re-syncs them) - keep one drain thread and re-point it.
        """
        callback = super()._make_stream_cb(stream)
        if stream != LATEST_ONLY_STREAM:
            return callback
        latest = getattr(self, "_latest_twist", None)
        if latest is None:
            latest = self._latest_twist = LatestOnly(callback, name=stream)
        else:
            latest.rebind(callback)
        return latest


def _moss_base(hw_id: str = "base",
               address: str | None = None) -> HardwareComponent:
    """MOSS tracked base (differential; vy ignored) - QT Py ESP32-S3 over USB CDC."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type=MOSS_ADAPTER_NAME,
        address=address,   # serial port; adapter default: MOSS_PORT, the QT Py by-id, or a sentinel that fails loudly
    )


def _coordinator_blueprint(base: HardwareComponent):
    """Coordinator driving one base at velocity."""
    return MossControlCoordinator.blueprint(
        hardware=[base],
        tasks=[
            TaskConfig(name="vel_base", type="velocity",
                       joint_names=_base_joints, priority=10),
        ],
        instance_name=COORDINATOR_INSTANCE_NAME,
    )


# ── devices, cameras, switches ──────────────────────────────────────────────

LIDAR_BAUDRATE = 460800
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS = 640, 480, 5
EXPLORER_MIN_FRONTIER_PERIMETER_M = 0.3   # doorway-sized frontiers stay visible
EXPLORER_GOAL_TIMEOUT_S = 45.0
# Where the explorer sends the rover, set for the room (docs/cockpit.md):
# env -> Explorer2 config field, and the largest value accepted. Unset leaves
# the explorer's own default. The exponent stops at 6: the score is
# gain / (1 + path cost) ** exponent, and a large float power raises
# OverflowError instead of giving inf (31.0 ** 250 does), which would end the
# exploration loop.
EXPLORER_ENV = (("MOSS_EXPLORE_DISTANCE_EXP", "distance_exponent", 6.0),  # 2: distance squared against gain
                ("MOSS_GOAL_MAX_M", "max_goal_path_m", math.inf),         # 3 m of path cost at most, 0 = no limit
                ("MOSS_EXPLORE_RADIUS_M", "explore_radius_m", math.inf))  # 0 = no limit, 4 for a room


def explorer_env(env: dict | None = None) -> dict:
    """The explorer knobs set in the environment, as Explorer2 config
    fields. A value that is not a finite number from 0 to its largest
    (EXPLORER_ENV) is refused with a warning and the explorer keeps its
    default."""
    env = os.environ if env is None else env
    out = {}
    for name, field_name, largest in EXPLORER_ENV:
        raw = str(env.get(name, "")).strip()
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError:
            value = math.nan
        if not math.isfinite(value) or not 0 <= value <= largest:
            bound = f"from 0 to {largest:g}" if math.isfinite(largest) else ">= 0"
            _logger.warning(f"{name}={raw!r} is not a number {bound}: the explorer keeps its default")
            continue
        out[field_name] = value
    return out


def moss_ports(env: dict | None = None) -> dict:
    """{"base": QT Py port, "lidar": C1 port}: env first, then the by-id glob
    under robot_profile.BYID_ROOT, then the sentinel.

    Raises ValueError when both resolve to the same device (realpath): the
    base adapter would open the lidar's tty at 115200, retune the line under
    RPLidarC1 (460800) and write its 'status' probe into the C1. The usual
    cause is a stale MOSS_PORT pointing at the CP2102N."""
    ports = {"base": robot_profile.moss_qtpy_port(env),
             "lidar": robot_profile.moss_lidar_port(env)}
    if (ports["base"] != MOSS_QTPY_MISSING and ports["lidar"] != MOSS_LIDAR_MISSING
            and os.path.realpath(ports["base"]) == os.path.realpath(ports["lidar"])):
        raise ValueError(
            f"MOSS: the base port and the lidar port are the same device "
            f"({ports['base']} -> {os.path.realpath(ports['base'])}); the motor board is the "
            "QT Py (*QT_Py*), the lidar the CP2102N - check MOSS_PORT / MOSS_LIDAR_PORT")
    return ports


WRIST_WIDTH, WRIST_HEIGHT, WRIST_FPS = 640, 480, 5
WRIST_BY_ID_GLOBS = ("*KD-USB*-video-index0", "*Sonix*-video-index0")   # known arm cameras, tried first (as moss_preflight)
WRIST_ANY_GLOB = "*-video-index0"                # then any other UVC camera on the robot...
WRIST_NOT = ("realsense",)                       # ...except the nose camera (lower-cased by-id name)
V4L_BY_ID = "/dev/v4l/by-id"


def wrist_camera_index(env: dict | None = None, by_id_dir: str | None = None) -> int | None:
    """/dev/videoN index of the wrist camera, or None when it is not there.
    MOSS_WRIST_DEV wins (an index or a /dev/videoN path); otherwise the first
    by-id match of the known names; otherwise the first other UVC camera under
    by-id that is not the RealSense (MOSS carries two cameras: the nose and the
    arm). The kernel numbers video devices in plug order, so the by-id name is
    the stable handle and the index is resolved when the blueprint is built."""
    import glob
    import re
    env = os.environ if env is None else env
    by_id_dir = V4L_BY_ID if by_id_dir is None else by_id_dir
    dev = str(env.get("MOSS_WRIST_DEV", "")).strip()
    if dev:
        if dev.isdigit():
            return int(dev)
        target = os.path.realpath(dev)
    else:
        target = None
        for pat in WRIST_BY_ID_GLOBS:
            hits = sorted(glob.glob(os.path.join(by_id_dir, pat)))
            if hits:
                target = os.path.realpath(hits[0])
                break
        if target is None:
            others = [h for h in sorted(glob.glob(os.path.join(by_id_dir, WRIST_ANY_GLOB)))
                      if not any(x in os.path.basename(h).lower() for x in WRIST_NOT)]
            if others:
                target = os.path.realpath(others[0])
        if target is None:
            return None
    m = re.fullmatch(r"/dev/video(\d+)", target)
    return int(m.group(1)) if m else None


_WRIST_CLASS = None


def _wrist_module_class():
    """A CameraModule of its own class, so the blueprint remaps its outputs
    (color_image -> wrist_image, ...) without touching the RealSense. Built
    on first use (the camera stack is imported only then, so a coordinator
    worker never loads it) and exposed as `moss_dimos.blueprints.WristCamera`
    through the module __getattr__, so dimOS can pickle it by reference into
    its worker processes."""
    global _WRIST_CLASS
    if _WRIST_CLASS is None:
        from dimos.hardware.sensors.camera.module import CameraModule

        cls = type("WristCamera", (CameraModule,),
                   {"__doc__": "The arm's UVC camera (dimOS Webcam hardware, cv2 capture).",
                    "__module__": __name__, "__qualname__": "WristCamera"})
        _WRIST_CLASS = cls
    return _WRIST_CLASS


def _wrist_part(index: int):
    from functools import partial
    from dimos.hardware.sensors.camera.webcam import Webcam

    WristCamera = _wrist_module_class()
    # transform=None: no base_link -> camera_link TF from this module (that frame
    # is the RealSense's); the frames are "wrist/..." through the hardware prefix
    return WristCamera.blueprint(
        hardware=partial(Webcam, camera_index=index, width=WRIST_WIDTH, height=WRIST_HEIGHT,
                         fps=float(WRIST_FPS), frame_id_prefix="wrist"),
        frame_id="wrist_camera_link", transform=None, frequency=float(WRIST_FPS))


def camera_mount_xyzq(profile: RobotProfile) -> tuple[float, ...]:
    """base_link -> camera_link for the RealSense module, as
    (x, y, z, qx, qy, qz, qw): the mechanical pose only (dimOS adds the
    optical rotation itself), a pure pitch about +y, positive = nose down."""
    x, y, z = profile.camera_xyz_base_m
    half = math.radians(profile.camera_pitch_deg) / 2.0
    return (x, y, z, 0.0, math.sin(half), 0.0, math.cos(half))


def _announce(ports: dict, profile: RobotProfile) -> None:
    """The build-time lines: which devices, which body - and loudly when a
    device is missing."""
    if ports["base"] == MOSS_QTPY_MISSING:
        _logger.error("MOSS: no QT Py under /dev/serial/by-id (*QT_Py*) and MOSS_PORT unset - "
                      f"the base gets {MOSS_QTPY_MISSING} and will not connect")
    if ports["lidar"] == MOSS_LIDAR_MISSING:
        _logger.error("MOSS: no CP2102N under /dev/serial/by-id (*CP2102N*) and MOSS_LIDAR_PORT unset - "
                      f"the lidar gets {MOSS_LIDAR_MISSING} and will publish nothing")
    _logger.info(f"MOSS blueprint: base {ports['base']}, lidar {ports['lidar']}, profile {profile}")


def _gamepad_part():
    """The pad, its stick scale and clamp sized to the base's envelope
    (MOSS_MPS_AT_LIMIT / MOSS_TRACK_M / MOSS_TURN_GAIN / MOSS_MAX_PCT, read
    now, in the process that builds the blueprint): see teleop_speeds and
    teleop_envelope."""
    from moss_dimos.gamepad import GamepadTeleop
    from moss_dimos.moss_adapter import MossGeometry, teleop_envelope, teleop_speeds

    g = MossGeometry.from_env()
    linear, angular = teleop_speeds(g)
    return GamepadTeleop.blueprint(linear_speed=linear, angular_speed=angular,
                                   envelope=teleop_envelope(g))


def _common_parts(ports: dict, profile: RobotProfile) -> list:
    """Base, pad, lidar, odometry, costmap, camera, recorder, viewer: the
    modules moss-nav and moss-explore share."""
    from moss_dimos.costmap2d import MossCostMap
    from moss_dimos.lidar_odometry import LidarOdometry
    from moss_dimos.rplidar_c1 import RPLidarC1

    parts = [
        _coordinator_blueprint(_moss_base(address=ports["base"])),
        _gamepad_part(),
        RPLidarC1.blueprint(port=ports["lidar"], baudrate=LIDAR_BAUDRATE, frame_id="lidar_link"),
        # no IMU wired: kiss-icp on its constant-velocity prior
        LidarOdometry.blueprint(use_gyro_prior=False),
        MossCostMap.blueprint(),
    ]
    if robot_profile.env_on("MOSS_CAMERA", True):
        from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        x, y, z, qx, qy, qz, qw = camera_mount_xyzq(profile)
        cam_kwargs = dict(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS,
                          # depth on, aligned to colour: lidar_odometry turns it into the obstacles the
                          # 2D lidar cannot see (below its scan plane); the point cloud stays off (not used)
                          enable_depth=True, enable_pointcloud=False, enable_imu=False,
                          base_transform=Transform(translation=Vector3(x, y, z),
                                                   rotation=Quaternion(qx, qy, qz, qw)))
        serial = os.environ.get("MOSS_RS_SERIAL", "").strip()
        if serial:
            cam_kwargs["serial_number"] = serial
        parts.append(RealSenseCamera.blueprint(**cam_kwargs))
    if robot_profile.env_on("MOSS_WRIST", True):
        idx = wrist_camera_index()
        if idx is None:
            _logger.warning("MOSS blueprint: no wrist camera (%s under %s, or MOSS_WRIST_DEV): "
                            "wrist_image will not be published", " / ".join(WRIST_BY_ID_GLOBS + (WRIST_ANY_GLOB,)), V4L_BY_ID)
        else:
            _logger.info(f"MOSS blueprint: wrist camera /dev/video{idx}, {WRIST_WIDTH}x{WRIST_HEIGHT} @ {WRIST_FPS} fps")
            parts.append(_wrist_part(idx))
    if robot_profile.env_on("MOSS_RECORD", True):
        from moss_dimos.memory import MossRecorder
        parts.append(MossRecorder.blueprint(db_path=str(robot_profile.moss_recording_path())))
    if robot_profile.env_on("MOSS_RERUN", False):
        from dimos.core.global_config import global_config
        from dimos.visualization.vis_module import vis_module
        parts.append(vis_module(viewer_backend=global_config.viewer))
    return parts


def wrist_instance(bp) -> str | None:
    """The wrist camera's instance name in a blueprint (dimOS lowercases module
    names unless one is given), or None when the module is not in it."""
    for atom in bp.blueprints:
        if getattr(atom.module, "__name__", "") == "WristCamera":
            return atom.name
    return None


def _jpeg_camera(bp):
    """Colour frames cross the bus as JPEG (encoded once in the camera's
    process): the cockpit passes them through, the recorder stores JPEG.
    The wrist camera's outputs are remapped away from the RealSense's names
    (wrist_image, wrist_camera_info, wrist_tf) and wrist_image is JPEG too."""
    from dimos.core.transport import JpegLcmTransport
    from dimos.msgs.sensor_msgs.Image import Image
    transports = {}
    if robot_profile.env_on("MOSS_CAMERA", True):
        transports[("color_image", Image)] = JpegLcmTransport.spec("/color_image", Image)
    wrist = wrist_instance(bp)
    if robot_profile.env_on("MOSS_WRIST", True) and wrist is not None:
        bp = bp.remappings([(wrist, "color_image", "wrist_image"),
                            (wrist, "camera_info", "wrist_camera_info"),
                            (wrist, "tf", "wrist_tf")])
        transports[("wrist_image", Image)] = JpegLcmTransport.spec("/wrist_image", Image)
    return bp.transports(transports) if transports else bp


# ── the four blueprints, built on first access ──────────────────────────────

def build_moss_base():
    """The coordinator and the tracked base, twist_command on cmd_vel: any
    publisher of cmd_vel drives the base (the adapter's deadman still holds).
    The port is the adapter's default: MOSS_PORT, the QT Py by-id, or the
    sentinel."""
    return _coordinator_blueprint(_moss_base()).remappings(
        [(MossControlCoordinator, "twist_command", "cmd_vel")])


def build_moss_gamepad():
    """Base + pad, direct drive: the pad IS the commander (tele_cmd_vel ->
    cmd_vel), no nav, no MovementManager. tools/moss-teleop.sh runs it."""
    from dimos.core.coordination.blueprints import autoconnect
    from moss_dimos.gamepad import GamepadTeleop

    return autoconnect(
        _coordinator_blueprint(_moss_base()),
        _gamepad_part(),
    ).remappings([
        (MossControlCoordinator, "twist_command", "cmd_vel"),
        (GamepadTeleop, "tele_cmd_vel", "cmd_vel"),
    ])


def build_moss_nav():
    """Mapping + pad teleop. No planner, no explorer, no MovementManager: the
    pad drives the coordinator directly, like moss-gamepad."""
    from dimos.core.coordination.blueprints import autoconnect
    from moss_dimos.gamepad import GamepadTeleop

    profile = robot_profile.current()
    ports = moss_ports()
    _announce(ports, profile)
    return _jpeg_camera(autoconnect(*_common_parts(ports, profile))).remappings([
        (MossControlCoordinator, "twist_command", "cmd_vel"),
        (GamepadTeleop, "tele_cmd_vel", "cmd_vel"),
    ])


def build_moss_explore():
    """moss-nav's modules plus the planner, the arbiter and the explorer. The
    pad stays on tele_cmd_vel: MovementManager lets it win and cancels any
    navigation. Exploration waits for `explore_cmd`."""
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from moss_dimos.explorer2 import Explorer2
    from moss_dimos.recovering_planner import RecoveringPlanner

    profile = robot_profile.current()
    ports = moss_ports()
    _announce(ports, profile)
    lethal, pivot = robot_profile.derived(profile)
    parts = _common_parts(ports, profile) + [
        RecoveringPlanner.blueprint(robot_width=profile.robot_width_m,
                                    robot_rotation_diameter=profile.pivot_diameter_m),
        # pass-through scaling: the pad's own envelope is the guard
        MovementManager.blueprint(
            tele_cmd_vel_scaling=Twist(Vector3(1.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))),
        Explorer2.blueprint(min_frontier_perimeter=EXPLORER_MIN_FRONTIER_PERIMETER_M,
                            goal_timeout=EXPLORER_GOAL_TIMEOUT_S,
                            lethal_clearance_m=lethal, pivot_clearance_m=pivot,
                            **explorer_env()),
    ]
    return _jpeg_camera(autoconnect(*parts)).remappings([
        (MossControlCoordinator, "twist_command", "cmd_vel"),
    ])


# pyproject entry point target -> builder. Keys must equal the targets.
_BUILDERS = {"moss_base_blueprint": build_moss_base,
             "moss_gamepad_blueprint": build_moss_gamepad,
             "moss_nav_blueprint": build_moss_nav,
             "moss_explore_blueprint": build_moss_explore}


def __getattr__(name: str):
    """Entry points load `moss_<x>_blueprint` as attributes: build on first
    access, then cache. `WristCamera` is the dynamic camera class (built
    once, so pickling by reference finds the same object)."""
    if name == "WristCamera":
        return _wrist_module_class()
    builder = _BUILDERS.get(name)
    if builder is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    bp = builder()
    globals()[name] = bp
    return bp
