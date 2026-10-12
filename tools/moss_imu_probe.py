#!/usr/bin/env python3
"""Check, on the robot, that the RealSense streams colour, depth AND its IMU
from ONE process: the way MOSS_IMU=1 opens it in the stack
(moss_dimos/realsense_imu.py).

One pipeline with depth 640x480 and colour 640x480 at 30 fps, read by
wait_for_frames in a thread; then the motion module of THAT pipeline's
device, opened through the sensor API (accel at its highest rate, gyro at
--imu-hz), never a second pipeline. From the moment the motion module is
open it counts --seconds (default 4) of frames, prints the count and rate of
each stream, the last accel sample and the mean accel (where gravity sits:
about -9.81 m/s^2 on y when the camera is level), then stops everything.

Exit 0 when depth, colour, accel and gyro all counted frames; 1 otherwise,
with the exception text of the step that failed.

Run it with the stack DOWN: while the stack holds the camera, another process
cannot open it ("failed to set power state"), so the probe refuses to start
when it finds a running stack.

    tools/moss_imu_probe.py                    # 4 s, gyro 200 Hz
    tools/moss_imu_probe.py --seconds 10 --imu-hz 400 --serial <serial number>

Nothing moves; nothing is written.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

STACK_RE = r"[b]in/dimos( [^ ]+)* run( |$)"
WIDTH, HEIGHT, FPS = 640, 480, 30


def stack_running() -> bool:
    try:
        out = subprocess.run(["pgrep", "-f", STACK_RE], capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return False
    return bool(out.strip())


def gravity_axis(mean: tuple[float, float, float]) -> str:
    """The axis that carries gravity, signed: '-y' for (0, -9.8, 0)."""
    i = max(range(3), key=lambda k: abs(mean[k]))
    return ("-" if mean[i] < 0 else "+") + "xyz"[i]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="colour + depth + IMU from one process, on the robot, stack down")
    ap.add_argument("--seconds", type=float, default=4.0, help="how long to stream (default 4)")
    ap.add_argument("--imu-hz", type=int, default=200, help="gyro rate (default 200, as MOSS_IMU)")
    ap.add_argument("--serial", default=os.environ.get("MOSS_RS_SERIAL", "").strip() or None,
                    help="the RealSense serial number (default MOSS_RS_SERIAL, else the first camera)")
    args = ap.parse_args(argv)

    if stack_running():
        print("a dimOS stack is running: it holds the camera, and this probe could not open it. "
              "Stop the stack first (tools/moss-fly.sh stop).")
        return 1
    try:
        import pyrealsense2 as rs
        from moss_dimos.realsense_imu import open_motion_sensor
    except Exception as exc:  # noqa: BLE001
        print(f"import failed: {type(exc).__name__}: {exc}")
        print("(run it with the robot's venv; on a Jetson export GLIBC_TUNABLES as the launchers do, docs/base.md)")
        return 1

    counts = {"depth": 0, "color": 0, "accel": 0, "gyro": 0}
    last_accel: list[float] | None = None
    accel_sum = [0.0, 0.0, 0.0]
    lock = threading.Lock()
    errors: list[str] = []
    stop = threading.Event()

    def on_motion(frame) -> None:   # noqa: ANN001 - a pyrealsense2 frame
        nonlocal last_accel
        motion = frame.as_motion_frame()
        if not motion:
            return
        kind = motion.get_profile().stream_type()
        data = motion.get_motion_data()
        with lock:
            if kind == rs.stream.accel:
                counts["accel"] += 1
                last_accel = [data.x, data.y, data.z]
                accel_sum[0] += data.x
                accel_sum[1] += data.y
                accel_sum[2] += data.z
            elif kind == rs.stream.gyro:
                counts["gyro"] += 1

    def video_loop(pipeline) -> None:   # noqa: ANN001
        while not stop.is_set():
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
            except Exception as exc:  # noqa: BLE001
                if not stop.is_set():
                    errors.append(f"wait_for_frames: {type(exc).__name__}: {exc}")
                return
            with lock:
                if frames.get_depth_frame():
                    counts["depth"] += 1
                if frames.get_color_frame():
                    counts["color"] += 1

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    sensor = None
    thread = None
    t_start = time.monotonic()
    try:
        try:
            profile = pipeline.start(config)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"pipeline start (depth + colour): {type(exc).__name__}: {exc}")
            profile = None
        if profile is not None:
            device = profile.get_device()
            name = device.get_info(rs.camera_info.name) if device.supports(rs.camera_info.name) else "?"
            print(f"camera: {name}, depth + colour {WIDTH}x{HEIGHT} @ {FPS} fps on one pipeline")
            thread = threading.Thread(target=video_loop, args=(pipeline,), daemon=True)
            thread.start()
            try:
                opened = open_motion_sensor(device, rs, args.imu_hz, on_motion)
                if opened is None:
                    errors.append("this camera has no motion module")
                else:
                    sensor, accel_p, gyro_p = opened
                    print(f"motion module open on the same device: accel {accel_p.fps()} Hz, gyro {gyro_p.fps()} Hz")
            except Exception as exc:  # noqa: BLE001
                errors.append(f"motion module: {type(exc).__name__}: {exc}")
            with lock:   # count from here: the video must keep flowing once the motion module is open
                counts.update(depth=0, color=0, accel=0, gyro=0)
                accel_sum[:] = [0.0, 0.0, 0.0]
            t_start = time.monotonic()
            time.sleep(max(0.5, args.seconds))
    finally:
        stop.set()
        if sensor is not None:
            for step in (sensor.stop, sensor.close):
                try:
                    step()
                except Exception:  # noqa: BLE001
                    pass
        try:
            pipeline.stop()
        except Exception:  # noqa: BLE001
            pass
        if thread is not None:
            thread.join(timeout=3)
    span = max(1e-6, time.monotonic() - t_start)

    with lock:
        c = dict(counts)
        last = list(last_accel) if last_accel else None
        mean = tuple(s / c["accel"] for s in accel_sum) if c["accel"] else None
    for key in ("depth", "color", "accel", "gyro"):
        print(f"  {key:6s} {c[key]:6d} frames  ({c[key] / span:6.1f} per s)")
    if last:
        print(f"  last accel  ({last[0]:+.3f}, {last[1]:+.3f}, {last[2]:+.3f}) m/s^2")
    if mean:
        print(f"  mean accel  ({mean[0]:+.3f}, {mean[1]:+.3f}, {mean[2]:+.3f}) m/s^2 - gravity on {gravity_axis(mean)} "
              "(level camera, librealsense axes: -y)")
    for err in errors:
        print(f"  error: {err}")
    ok = all(c[k] > 0 for k in ("depth", "color", "accel", "gyro"))
    print("video and motion together: YES" if ok else "video and motion together: NO")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
