# Contacts from the IMU

MOSS knows it touched something in two ways. The base adapter watches the
encoders: tracks commanded but not turning for a second is a stall, and the
coordinator publishes it on `bump`. That misses the contacts that matter
most: a track that slips keeps turning against the wall, an oblique contact
leaves one track free, and the verdict comes a second or more after the
impact. The chassis says it at once: hitting something while driving is a
short, hard deceleration along the direction of travel. The RealSense on the
nose carries an IMU, and `MOSS_IMU=1` turns its decelerations into contacts on
the same `bump` stream.

It is opt-in (default off) until a stationary launch on the robot shows
colour, depth and the IMU alive together (below).

## The IMU

The D455F has a motion module inside the camera: a 3-axis accelerometer
(`accel`) and a 3-axis gyroscope (`gyro`). Measured on the robot: accel at
100, 200 or 400 Hz, gyro at 200 or 400 Hz, streaming alone and in ONE
pipeline next to depth and colour without an error.

Two facts decide how it must be opened:

* A second process cannot touch the camera while the stack holds it
  ("failed to set power state").
* dimOS's `RealSenseCamera` with `enable_imu=True` opens a SECOND pipeline for
  the motion module (`_start_imu`). On the rover that pipeline fails ("No
  device connected") and takes colour and depth down with it. That is why
  `MOSS_GYRO=1` alone stays an experiment.

So `moss_dimos/realsense_imu.py` defines `MossRealSense`, the same module with
the same outputs, which changes one method: `_start_imu` opens the motion
module through the sensor API of the device the colour/depth pipeline already
holds (`device.query_sensors()`, the sensor named "Motion Module"), accel at
its highest rate and gyro at `imu_hz` (200 with `MOSS_IMU`), and hands the
samples to dimOS's own handler, which publishes `imu`. No second pipeline, no
second process. If the camera has no motion module, or the gyro does not offer
the rate, or opening fails, the camera logs a warning and goes on with colour
and depth only. `stop()` stops and closes the motion module before the
pipeline.

## The axes

dimOS publishes the motion data as librealsense delivers it, without
rotating it: one `Imu` per gyro sample, the accel interpolated linearly
between the two accel samples around it, `frame_id`
`camera_accel_optical_frame`, timestamps on the host clock (dimOS turns on the
camera's global time). librealsense already aligns the motion module with the
depth imager's optical frame:

| Axis | Points | At rest, camera level |
|---|---|---|
| x | right | 0 |
| y | down | -9.81 m/s^2 (measured on the robot) |
| z | forward, out of the lens | 0 |

An accelerometer reads the push that holds it up, so gravity shows as -9.81 on
the axis that points down. The gyro uses the same axes: the rover's yaw rate
is `-gyro.y` (lidar_odometry's `gyro_axis="-y"`).

Forward is `+z`. The y reading is measured; the direction of z comes from
librealsense's convention and is the one thing the probe checks on the robot
before the default is trusted: run the probe (below) with the nose tilted down by hand,
and the mean accel on z goes negative (about -1.7 m/s^2 at 10 degrees). If it
goes positive, the camera is mounted the other way round: set
`MOSS_BUMP_AXIS=-z`.

## The rule

`moss_dimos/bump_imu.py`, class `ImuContactDetector` (pure, benched with known
traces), inside the `BumpImu` module:

1. **Forward component.** The accel sample on the forward axis
   (`MOSS_BUMP_AXIS`, default `+z`).
2. **High-pass.** That value minus the mean of the samples of the last 0.5 s
   before it. Gravity leaking onto z (a pitched camera, a slope) and a slow
   tilt cancel; a hit does not. Nothing is judged before the window holds
   0.25 s of history.
3. **Hard enough.** |high-pass| >= `MOSS_BUMP_ACCEL` (default 4.0 m/s^2).
4. **While driving.** The tracks are commanded: the firmware's `target_pct`
   from `base_telemetry`, not (0, 0), younger than 1 s (time since it arrived
   plus the firmware's own age of that line). A rover kicked or lifted while
   idle is not a contact.
5. **Against the command.** Driving forward (no track backward): only a
   deceleration, high-pass <= -threshold. Reversing (no track forward): only a
   forward kick, high-pass >= +threshold. Pivoting (tracks of opposite signs):
   either sign. Taking off is never a contact.
6. **Once.** After a contact, nothing for `MOSS_BUMP_DEBOUNCE_S` (default
   0.7 s). The debounce also covers the planner's escape taking off in
   reverse while the last telemetry line still says forward.

On a contact `BumpImu` publishes `Bool(True)` on `bump` and logs one line:

```
IMU contact: forward -4.8 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%
```

`bump` is the stream the coordinator's stall guard and the explorer's back-off
already publish on, so the same three listeners react: the planner's escape
(stop, 0.20 m in reverse, the goal abandoned; a second contact during an
escape stops dead), the costmap's contact memory (the band ahead becomes a
wall) and the sounds (`bump_surprise`, at the impact rather than a second
later). The stall guard stays on: a contact with the tracks frozen can be
reported twice, by the IMU at the impact and by the encoders a second later.

The cockpit shows each contact in its decision feed
([cockpit.md](cockpit.md#what-it-decides)). The module's other lines start
with `MOSS IMU:` and so appear in `tools/moss-fly.sh log`: the knobs at
start, the first sample (where gravity sits), then once a minute the sample
rate, the mean accel and the contacts so far. The contact lines themselves
are in `.venv/bin/dimos log` and in the cockpit.

Why the threshold should hold against the rover's own stops: the launcher sets
the firmware ramp to 5 ms per % (`MOSS_RAMP_MS="5 5"`), so a commanded stop
from 70 % takes about 0.35 s, roughly 1 m/s^2 of deceleration at the rover's
speeds. That figure comes from the ramp setting, not from a recording. With `MOSS_PULSE_MS` the ramp
is 1 ms per % and every 100 ms slice starts and stops the tracks: record a
signature before using the two together.

Where it could fire wrongly: pivots, which accept either sign. The camera sits
0.156 m ahead of the centre, so a pivot pulls it toward the back with
yaw rate squared times 0.156 m: about 3 m/s^2 at the 254 deg/s measured for a
full-power pivot. Held steady, the high-pass removes it; its onset and its end
are what the signature must look at, with the gyro columns of the CSV next to
`hp` (a figure from the geometry, not from a recording).

## Knobs

| Variable | Default | What |
|---|---|---|
| `MOSS_IMU` | on | MossRealSense with the IMU at 200 Hz and the BumpImu module; `0` goes back to dimOS's camera without IMU. Needs the camera (`MOSS_CAMERA=0` refuses it with a warning). |
| `MOSS_BUMP_ACCEL` | 4.0 | threshold in m/s^2 on the high-passed forward accel, above 0 and up to 100 |
| `MOSS_BUMP_AXIS` | `+z` | the forward axis of the camera's frame, x, y or z with a sign |
| `MOSS_BUMP_DEBOUNCE_S` | 0.7 | seconds of silence after a contact, 0 to 10 |
| `MOSS_IMU_LOG` | unset | a CSV path: every sample is appended (below) |
| `MOSS_GYRO` | off | the gyro prior of the lidar odometry; with `MOSS_IMU=1` it reads the in-process IMU |

They are read when the blueprint is built (the build log says
`MOSS blueprint: IMU contacts on bump, forward axis +z, threshold 4 m/s^2, ...`);
a value that does not parse is refused with a warning and the default stays.

## Bringing it up

1. **The probe, stack down.** `tools/moss_imu_probe.py` opens ONE pipeline with
   depth and colour at 640x480 30 fps, then the motion module of that same
   device through the sensor API, exactly as `MossRealSense` does, and counts
   frames for 4 s:

   ```
   $ tools/moss-fly.sh stop
   $ export GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304   # on the Jetson, as the launchers do
   $ .venv/bin/python tools/moss_imu_probe.py
   camera: Intel RealSense D455F, depth + colour 640x480 @ 30 fps on one pipeline
   motion module open on the same device: accel 400 Hz, gyro 200 Hz
     depth     120 frames  (  30.0 per s)
     color     120 frames  (  30.0 per s)
     accel    1600 frames  ( 400.0 per s)
     gyro      800 frames  ( 200.0 per s)
     last accel  (+0.031, -9.794, +0.112) m/s^2
     mean accel  (+0.028, -9.801, +0.104) m/s^2 - gravity on -y (level camera, librealsense axes: -y)
   video and motion together: YES
   ```

   (illustrative numbers). Exit 0 when depth, colour, accel and gyro all
   counted frames, 1 otherwise with the error of the step that failed. It
   refuses to start while a stack is running. `--seconds`, `--imu-hz` and
   `--serial` (default `MOSS_RS_SERIAL`) change the run. Tilt the nose down
   by hand during a run to check the sign of z (above).

2. **A stationary launch.** `tools/moss-fly.sh map`, the rover still.
   `tools/moss-fly.sh log` must show `MOSS IMU: motion module open on the
   camera's own device` and `MOSS IMU: first sample`, and the cockpit must show
   colour and the depth obstacles as before. This launch was seen on the robot
   (colour and depth at their usual rate, the IMU at 199 Hz, gravity on -y),
   which is why the IMU is on by default; `MOSS_IMU=0` turns it off.

3. **A signature before tightening the threshold.** Record what real contacts,
   real stops and the real floor look like:

   ```
   $ MOSS_IMU=1 MOSS_IMU_LOG=~/imu_signature.csv tools/moss-fly.sh map
   ```

   The CSV gets one row per sample, flushed every 2 s:
   `ts,ax,ay,az,gx,gy,gz,hp,cmd_l,cmd_r` (`hp` empty while the window warms
   up, the command empty when there is none or it is stale). Drive into a
   soft obstacle a few times, straight and oblique, stop hard a few times,
   drive over the floor's joints. Then list the hardest moments:

   ```
   $ python3 - ~/imu_signature.csv <<'EOF'
   import csv, sys
   rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["hp"]]
   for r in sorted(rows, key=lambda r: abs(float(r["hp"])), reverse=True)[:30]:
       print(r["ts"], r["hp"], r["cmd_l"], r["cmd_r"])
   EOF
   ```

   The threshold belongs between the largest |hp| of the stops and the floor
   and the smallest |hp| of the contacts. A write error is logged once and
   never stops the stack.

## Benches

`benches/bench_bump_imu_cold.py`: the rule on synthetic 200 Hz traces with
gravity on y (a 6 m/s^2 pulse of 60 ms while commanded is exactly one contact
of about -6.0 m/s^2; no command, a wobble, the wrong sign, a tilt, a pitched
camera are none; the debounce; the knobs), the module (telemetry then samples
-> one `bump` and the log line), the CSV, and `MossRealSense` on a fake
pyrealsense2 (the motion sensor found by name on the pipeline's own device,
the profiles it opens, stop, every failure leaving the camera running).
`benches/bench_moss_blueprints_cold.py` section J: the blueprint with and
without `MOSS_IMU`, and where `bump`, `imu` and `base_telemetry` are wired.
