"""SLAMTEC RPLIDAR C1 -> flat PointCloud2 for dimOS.

dimOS has no 2D lidar path (no LaserScan message at all), so this module
publishes each 360-degree scan as a FLAT PointCloud2 (z = 0) in `lidar_link`.
LidarOdometry turns that cloud into the pose and the world-frame `lidar`
cloud the costmap is built from.

The geometry (mounting yaw, minimum range, masked sectors, the body box) comes
from the robot profile, read once per revolution, so a MOSS_* override takes
effect without a restart.

The C1 is hot-pluggable here: a missing port, a permission error or a mid-scan
unplug is logged once and retried every RETRY_PERIOD_S on the same path. The
module survives the sensor being pulled out and plugged back (its by-id path
stays the same), and with a known port (MOSS_LIDAR_PORT) it can start before
the sensor is plugged in. A lidar absent when the port was resolved gets the
sentinel path (DEFAULT_PORT) and stays silent until the stack is relaunched
with the lidar plugged in.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

from moss_dimos import robot_profile
from moss_dimos.robot_profile import RobotProfile, current as _current_profile

logger = setup_logger()

# The port when nothing better is known: the sentinel path that cannot exist,
# so the open fails with this name in the log. The blueprint passes the port
# robot_profile.moss_lidar_port() resolves (MOSS_LIDAR_PORT, else the
# CP2102N by-id link); a module built without one resolves it the same way.
DEFAULT_PORT = robot_profile.MOSS_LIDAR_MISSING
# 460800 is the C1's line rate. The reader is plain pyserial (c1_serial.py).
DEFAULT_BAUDRATE = 460800
DEFAULT_FRAME = "lidar_link"
RETRY_PERIOD_S = 5.0
# Logged once per connection, on the first published cloud. The launcher
# greps "RPLIDAR C1 scanning": keep the words.
SCANNING_LINE = "RPLIDAR C1 scanning: %d points in the first turn"


def polar_to_xy(angle_deg: float, distance_mm: float,
                yaw_offset_deg: float | None = None) -> tuple[float, float]:
    """One lidar measure -> (x, y) in metres in the sensor frame.

    The reader yields the heading in degrees over [0, 360) and the distance
    in millimetres. SLAMTEC's heading grows CLOCKWISE seen from above (C1
    datasheet, fig. 2-4, "left hand"), while the robot frame is right-handed
    (x forward, y left), so y is negated: 0 deg -> +X, 90 deg -> -Y.

    ``yaw_offset_deg`` is the lidar's mounting yaw (its 0-deg mark against the
    body's +x). None = the active profile's value; scan_to_points resolves it
    once per revolution and passes it in.
    """
    if yaw_offset_deg is None:
        yaw_offset_deg = _current_profile().lidar_yaw_offset_deg
    theta = math.radians(angle_deg + yaw_offset_deg)
    distance_m = distance_mm / 1000.0
    return distance_m * math.cos(theta), -distance_m * math.sin(theta)


def _in_mask(angle_deg: float, distance_m: float,
             mask: tuple[tuple[float, float], ...], mask_range_m: float) -> bool:
    if distance_m >= mask_range_m:
        return False
    a = angle_deg % 360.0
    for start, end in mask:
        if (start <= a <= end) if start <= end else (a >= start or a <= end):
            return True
    return False


def scan_to_points(scan: list[tuple[float, float, float]],
                   min_quality: int,
                   mask: tuple[tuple[float, float], ...] | None = None,
                   mask_range_m: float | None = None,
                   profile: RobotProfile | None = None) -> list[tuple[float, float, float]]:
    """A 360-degree scan -> flat (x, y, 0.0) points in metres.

    ``scan`` is what ``iter_scans`` yields: (quality, angle_deg, distance_mm)
    measures. Weak returns (quality below min_quality), invalid ones
    (distance 0), returns closer than the profile's minimum range, and parts
    of the robot standing in the scan plane (inside ``mask`` and closer than
    ``mask_range_m``) are dropped.

    Geometry (yaw offset, minimum range, mask, body box) comes from
    ``profile``, default the active robot_profile, resolved ONCE per scan;
    an explicit ``mask`` / ``mask_range_m`` still wins over the profile's.
    """
    p = profile or _current_profile()
    if mask is None:
        mask = p.scan_mask_sectors_deg
    if mask_range_m is None:
        mask_range_m = p.scan_mask_range_m
    yaw = p.lidar_yaw_offset_deg
    min_mm = p.scan_min_range_m * 1000.0
    pts = [(*polar_to_xy(angle, distance, yaw), 0.0)
           for (quality, angle, distance) in scan
           if quality >= min_quality and distance > min_mm
           and not _in_mask(angle, distance / 1000.0, mask, mask_range_m)]
    # Anything landing INSIDE the body rectangle is the robot itself:
    # geometric and angle-free, the profile's body box (half extents around
    # the lidar centre, margin included).
    bx, by = p.body_box_m
    return [q for q in pts if abs(q[0]) > bx or abs(q[1]) > by]


class RPLidarC1(Module):
    """RPLIDAR C1 driver module. Publishes flat pointclouds."""

    dedicated_worker = True

    pointcloud: Out[PointCloud2]

    def __init__(self, port: str | None = None,
                 baudrate: int = DEFAULT_BAUDRATE,
                 frame_id: str = DEFAULT_FRAME,
                 min_quality: int = 0,   # at 10, ~14 % of valid returns vanished - whole thin or dark objects (table legs) with no strong return
                 retry_period_s: float = RETRY_PERIOD_S,
                 **kwargs: Any) -> None:
        # frame_id is a dimOS ModuleConfig field and Module exposes it as a
        # read-only property, so it goes through the config, never onto self.
        super().__init__(frame_id=frame_id, **kwargs)
        # No port given: resolved now, the way the blueprint resolves it.
        self.port = port or robot_profile.moss_lidar_port()
        self.baudrate = baudrate
        self.min_quality = min_quality
        self.retry_period_s = retry_period_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # @rpc is not decoration for its own sake: Module.start/stop carry it,
    # and an override that drops it falls out of the class's rpcs table.
    # dimOS then proxies the call by pickling the module across the worker
    # pipe, which dies on our threading.Event ('cannot pickle _thread.lock').
    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker,
                                        name="rplidar-c1", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        super().stop()

    # Named _worker, NOT _loop: dimOS's Module keeps its asyncio event
    # loop in self._loop, which would shadow the method and make the
    # thread target the event loop object.
    # Injection point for the cold bench (a fake device class); None = C1Lidar.
    lidar_class: Any = None

    def _worker(self) -> None:
        from moss_dimos.c1_serial import C1Lidar
        RPLidar = self.lidar_class or C1Lidar

        last_error: str | None = None
        while not self._stop_event.is_set():
            lidar = None
            try:
                # The reader opens the serial port inside __init__: a missing
                # device or a permission error raises right here.
                lidar = RPLidar(self.port, baudrate=self.baudrate)
                logger.info("RPLIDAR C1 up on %s @ %d baud: %s",
                            self.port, self.baudrate, _describe(lidar))
                last_error = None
                self._scan(lidar)
            except Exception as exc:  # noqa: BLE001 - absent/unplugged sensor
                error = f"{type(exc).__name__}: {exc}"
                if error != last_error:  # log once per distinct cause
                    logger.warning(
                        "RPLIDAR C1 unavailable on %s (%s) - retrying every "
                        "%.0f s", self.port, error, self.retry_period_s)
                    last_error = error
            finally:
                _shutdown(lidar)
            self._stop_event.wait(self.retry_period_s)

    def _scan(self, lidar: Any) -> None:
        """Publish one flat cloud per revolution until stop or a reader error.

        The first published cloud of each connection is logged once
        (SCANNING_LINE): "RPLIDAR C1 up" only proves the sensor answers on
        its port, this line proves it spins and sends scans. The launcher
        (tools/moss-fly.sh) waits for it before calling the lidar proven."""
        first = True
        for scan in lidar.iter_scans():
            if self._stop_event.is_set():
                return
            points = scan_to_points(scan, self.min_quality)
            if points:
                self.pointcloud.publish(PointCloud2.from_numpy(
                    np.asarray(points, dtype=np.float32),
                    frame_id=self.frame_id, timestamp=time.time()))
                if first:
                    first = False
                    logger.info(SCANNING_LINE, len(points))


def _describe(lidar: Any) -> str:
    """Device info for the log line - never worth failing a session over."""
    try:
        return str(lidar.get_info())
    except Exception as exc:  # noqa: BLE001
        return f"info unavailable ({type(exc).__name__})"


def _shutdown(lidar: Any) -> None:
    """Stop the scan, stop the motor, close the port. Each step best-effort.

    On an unplug every step raises (the port is gone); that is expected and
    must not mask the original error or block the retry.
    """
    if lidar is None:
        return
    for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
        try:
            step()
        except Exception:  # noqa: BLE001
            logger.debug("rplidar shutdown step %s failed",
                         getattr(step, "__name__", step), exc_info=True)
