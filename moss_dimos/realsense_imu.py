"""MossRealSense: dimOS's RealSense module with its IMU opened in the camera's own process.

dimOS's RealSenseCamera streams colour and depth on one pipeline, and with
enable_imu=True it starts a SECOND pipeline for the motion module
(_start_imu). On MOSS's D455F that second pipeline fails ("No device
connected") and takes colour and depth down with it, while the same motion
module streams fine next to colour and depth when it is opened from the
process that already holds the device. A second process cannot touch the
device at all while the stack holds it ("failed to set power state").

So this subclass changes ONE thing: _start_imu opens the motion module
through the sensor API of the device the running pipeline already holds
(device.query_sensors(), the sensor named "Motion Module"), never through
another pipeline. Accel at the highest rate the sensor offers, gyro at
config.imu_hz, both handed to the parent's own frame handler
(_on_motion_frame), which pairs them and publishes `imu` exactly as before.

What the parent publishes on `imu` (docs/imu.md): one Imu per gyro sample,
the accel interpolated to that instant, in librealsense's motion axes, which
librealsense already aligns with the depth imager's optical frame: x right,
y down, z forward out of the lens. Nothing is rotated on the way; frame_id is
"<camera_name>_accel_optical_frame". At rest, level, the accel reads the
reaction to gravity: about (0, -9.81, 0) m/s^2.

A failure of the motion module never stops the camera: no motion module, a
gyro rate the sensor does not offer, or an error while opening it is logged
as a warning and colour and depth go on without `imu`.

Same module, same Outs, same config: the blueprint swaps the class only
when MOSS_IMU is on (moss_dimos/blueprints.py).
"""
from __future__ import annotations

from typing import Any, Callable

from dimos.core.core import rpc
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MOTION_SENSOR_MARK = "Motion"     # the D4xx motion sensor is named "Motion Module"


def find_motion_sensor(device: Any, rs: Any) -> Any | None:
    """The device's motion sensor (its name contains "Motion"), or None."""
    for sensor in device.query_sensors():
        try:
            name = str(sensor.get_info(rs.camera_info.name))
        except Exception:  # noqa: BLE001 - a sensor without a name is not the motion module
            continue
        if MOTION_SENSOR_MARK in name:
            return sensor
    return None


def motion_profiles(sensor: Any, rs: Any, imu_hz: int) -> tuple[Any, Any]:
    """(accel profile at the highest offered rate, gyro profile at imu_hz),
    float xyz format, as motion stream profiles. Raises ValueError naming the
    offered rates when the accel is missing or the gyro does not offer
    imu_hz."""
    accel: list[Any] = []
    gyro: list[Any] = []
    for profile in sensor.get_stream_profiles():
        kind = profile.stream_type()
        if kind not in (rs.stream.accel, rs.stream.gyro):
            continue
        if profile.format() != rs.format.motion_xyz32f:
            continue
        if hasattr(profile, "is_motion_stream_profile") and not profile.is_motion_stream_profile():
            continue
        motion = profile.as_motion_stream_profile() if hasattr(profile, "as_motion_stream_profile") else profile
        (accel if kind == rs.stream.accel else gyro).append(motion)
    if not accel:
        raise ValueError("the motion module offers no accel stream (motion_xyz32f)")
    gyro_rates = sorted({int(p.fps()) for p in gyro})
    at_rate = [p for p in gyro if int(p.fps()) == int(imu_hz)]
    if not at_rate:
        raise ValueError(f"imu_hz={imu_hz} is not offered by the gyro; it offers {gyro_rates}")
    return max(accel, key=lambda p: int(p.fps())), at_rate[0]


def open_motion_sensor(device: Any, rs: Any, imu_hz: int,
                       callback: Callable[[Any], None]) -> tuple[Any, Any, Any] | None:
    """Open and start the motion sensor of a device the caller already
    streams from: accel + gyro profiles (motion_profiles), sensor.open, then
    sensor.start(callback). Returns (sensor, accel profile, gyro profile), or
    None when the device has no motion sensor. Raises on any other failure,
    after closing a sensor it had opened."""
    sensor = find_motion_sensor(device, rs)
    if sensor is None:
        return None
    accel, gyro = motion_profiles(sensor, rs, imu_hz)
    sensor.open([accel, gyro])
    try:
        sensor.start(callback)
    except Exception:
        try:
            sensor.close()
        except Exception:  # noqa: BLE001 - the start error is the one to report
            pass
        raise
    return sensor, accel, gyro


class MossRealSense(RealSenseCamera):
    """RealSenseCamera whose IMU rides the camera's own device handle (see the
    module docstring). Everything but _start_imu and stop is the parent's."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._imu_sensor: Any | None = None
        self._imu_warned = False

    def _start_imu(self) -> None:
        """Open the motion module on the device the colour/depth pipeline holds.
        Never raises: the camera keeps streaming without `imu` on a failure."""
        import pyrealsense2 as rs

        if self._profile is None:
            return
        try:
            opened = open_motion_sensor(self._profile.get_device(), rs, int(self.config.imu_hz),
                                        self._on_motion_frame)
        except Exception as exc:  # noqa: BLE001 - the IMU never takes colour and depth down
            self._imu_sensor = None
            logger.warning(f"MOSS IMU: the motion module did not start ({type(exc).__name__}: {exc}); "
                           "colour and depth go on, no imu")
            return
        if opened is None:
            if not self._imu_warned:
                self._imu_warned = True
                logger.warning("MOSS IMU: this camera has no motion module; colour and depth go on, no imu")
            return
        sensor, accel, gyro = opened
        self._imu_sensor = sensor
        logger.info(f"MOSS IMU: motion module open on the camera's own device (no second pipeline): "
                    f"accel {int(accel.fps())} Hz, gyro {int(gyro.fps())} Hz -> imu in {self._imu_optical_frame} "
                    "(x right, y down, z forward)")

    @rpc
    def stop(self) -> None:
        sensor, self._imu_sensor = getattr(self, "_imu_sensor", None), None
        if sensor is not None:
            for step in (sensor.stop, sensor.close):
                try:
                    step()
                except Exception:  # noqa: BLE001 - already stopped or the device is gone
                    pass
        super().stop()


# dimOS main (7a83fe5b1): the RealSense is a native (Rust) module that publishes only its own
# subtree from camera_link and takes no base_transform; the mount is a static edge of its own.
from dimos.msgs.geometry_msgs.Quaternion import Quaternion  # noqa: E402
from dimos.msgs.geometry_msgs.Transform import Transform  # noqa: E402
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # noqa: E402
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig  # noqa: E402


class MossCameraMountConfig(StaticTfPublisherConfig):
    xyzq: tuple[float, float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


class MossCameraMount(StaticTfPublisher):
    """base_link -> camera_link (blueprints.camera_mount_xyzq), for a camera that no longer publishes it."""
    config: MossCameraMountConfig

    def transforms(self) -> list[Transform]:
        x, y, z, qx, qy, qz, qw = self.config.xyzq
        return [Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw),
                          frame_id="base_link", child_frame_id="camera_link")]
