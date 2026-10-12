# Contacts from the IMU

MOSS knows it touched something in two ways. The base adapter watches the
encoders: tracks commanded but not turning for a second is a stall, and the
coordinator publishes it on `stall`. A stall is inferred, not felt (a pivot
that fails to turn reads the same as a wall), so the planner answers it by
stopping and abandoning the goal, with no motion. The encoders also miss the
contacts that matter most: a track that slips keeps turning against the
wall, an oblique contact leaves one track free, and the verdict comes a
second or more after the impact. The chassis says it at once: hitting
something while driving is a short, hard deceleration along the direction of
travel. The RealSense on the nose carries an IMU, and `MOSS_IMU=1` turns its
decelerations into contacts: on `bump` for a contact in front, on
`bump_rear` for a contact behind. A felt contact is the one thing that earns
the planner's scripted move, 0.20 m away from it.

It is on by default since a stationary launch on the robot showed colour,
depth and the IMU alive together (below); `MOSS_IMU=0` turns it off.

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
  the gyro prior never takes that path unless forced (below).

So `moss_dimos/realsense_imu.py` defines `MossRealSense`, the same module with
the same outputs, which changes one method: `_start_imu` opens the motion
module through the sensor API of the device the colour/depth pipeline already
holds (`device.query_sensors()`, the sensor named "Motion Module"), accel at
its highest rate and gyro at `imu_hz` (200 with `MOSS_IMU`), and hands the
samples to dimOS's own handler, which publishes `imu`. No second pipeline, no
second process. If the camera has no motion module, or the gyro does not offer
the rate, or opening fails, the camera logs a warning and goes on with colour
and depth only. `stop()` stops and closes the motion module before the
pipeline. The camera module runs in a worker process of its own
(`dedicated_worker`, the blueprint's `DedicatedMossRealSense`): a camera
sharing its worker with the odometry, the costmap or the recorder is starved
under load, and its depth and IMU samples arrive late or not at all while
the sensor drops no frame.

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

## The gyro and the odometry

The same IMU gives the lidar odometry its rotation prior. On the robot the
`imu` messages from the camera's own process reach `LidarOdometry` at about
200 Hz; it integrates the yaw rate (`-gyro.y`) from one lidar revolution to
the next, the next registration starts from that yaw (the translation stays
kiss-icp's constant-velocity model), and the integral starts again from
zero. A full-power pivot (254 deg/s measured on the floor) turns the rover 25
degrees between two scans, more than the constant-velocity prior follows; the
gyro says where the rover turned. A revolution with no gyro sample since the
last one falls back to the constant-velocity model. The prior is off by
default until its sign is measured on this robot, because a wrong-sign prior
is worse than none (the build log says `prior: gyro=False (run
tools/gyro_sign_bench.py once, then MOSS_GYRO=1)`): run that bench once, then
`MOSS_GYRO=1` turns it on with `MOSS_IMU` (the odometry's start line then
says `prior: gyro=True`), while with `MOSS_IMU=0`, `MOSS_GYRO=1` is refused
with a warning, because the gyro would then come from dimOS's second IMU
pipeline (above), and `MOSS_GYRO_FORCE=1` builds it anyway, the experiment
path.

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
   deceleration, high-pass <= -threshold, a contact in front. Reversing (no
   track forward): only a forward kick, high-pass >= +threshold, a contact
   behind. Pivoting (tracks of opposite signs): either sign, and the sign
   gives the side: a deceleration is in front, a forward kick behind. Taking
   off is never a contact.
6. **Once.** After a contact, nothing more on the same side for
   `MOSS_BUMP_DEBOUNCE_S` (default 0.7 s). The debounce also covers the
   planner's escape taking off in reverse while the last telemetry line still
   says forward. One debounce per side: a front contact and a rear one are
   opposite reflexes, not repeats of each other, so the other side counts at
   once (a rover wedging into a corner during one pivot is two contacts, and
   so is the escape that backs off a front contact and hits something
   behind). The one exception is the impact's own rebound: a kick of the
   other sign within 0.1 s of a contact, whatever the command says by then
   (the telemetry can already carry the escape's reverse). The contact
   measured below lasted 20 ms, so 0.1 s covers it five times over; the
   figure comes from that impact length, and a pivot contact recorded with
   `MOSS_IMU_LOG` is the recording that sets it.

On a contact `BumpImu` publishes `Bool(True)` on `bump` (in front) or
`bump_rear` (behind) and logs one line first:

```
IMU contact: forward -4.8 m/s^2 (threshold 4.0) while commanded L=+70% R=+60%, front contact -> bump
IMU contact: forward +5.2 m/s^2 (threshold 4.0) while commanded L=-70% R=-60%, rear contact -> bump_rear
```

A rebound is published nowhere, and logged once, so that no felt kick is
silent:

```
MOSS IMU: forward +6.8 m/s^2 while commanded L=+70% R=-70%, 0.06 s after the front contact: taken as its rebound, not published on bump_rear
```

`bump` is the stream the explorer's back-off also publishes on. Its
listeners: the planner's escape (stop, 0.20 m in reverse, the goal
abandoned; a second contact during an escape stops dead), the costmap (which
logs the contact and, with `MOSS_CONTACT_WALL=1` and the rover driving
straight forward, marks the band ahead as a wall), the sounds
(`bump_surprise`, at the impact rather than a second later) and the lidar
odometry, which drops the shaken revolution that follows
([below](#a-contact-and-the-map)). `bump_rear` has one listener, the
planner: stop, 0.20 m FORWARD, the goal abandoned. A rear contact is not a
front wall. The stall guard stays on, on its own stream: a stall that comes
while an escape runs stops the escape dead.

The cockpit shows each contact in its decision feed
([cockpit.md](cockpit.md#what-it-decides)). The module's other lines start
with `MOSS IMU:` and so appear in `tools/moss-fly.sh log`: the knobs at
start, the first sample (where gravity sits), the watchdog's lines (below),
and every 10 s a status line with the state, the sample rate, the mean
accel, the contacts and the quiet spells so far. The contact lines
themselves are in `.venv/bin/dimos log` and in the cockpit.

## When the IMU goes quiet

The IMU is this robot's main bumper, and the rotation prior when
`MOSS_GYRO=1`. A stream that stops must be said, not discovered in the map.
A watchdog thread in `BumpImu` (`ImuWatchdog`) looks every 0.1 s at the time
since the last sample arrived:

```
MOSS IMU: no sample for 1 s - IMU contacts and gyro prior OFF
MOSS IMU: samples back (199 Hz) after 3.2 s quiet
MOSS IMU: status alive, 1990 samples in 10 s (199 Hz), mean accel (+0.03, -9.80, +0.10) m/s^2, 0 contact(s) so far, 1 quiet spell(s)
MOSS IMU: status QUIET, no sample for 12 s - IMU contacts and gyro prior OFF, 0 contact(s) so far, 1 quiet spell(s)
MOSS IMU: status waiting, no sample 10 s after start - IMU contacts and gyro prior OFF
```

The first line comes once, 1 s after the last sample (the samples had been
flowing); the second once the resumed stream has run for 1 s and still
flows, with its rate over that second (a burst that stops again within that
second says nothing, and the quiet spell goes on). A stream that keeps
dropping out says "no sample" at most once every 5 s, and the next such
line counts the spells it did not say.
The status line comes every 10 s, quiet or not; before the first sample it
says `waiting` (the camera takes a few seconds to start, so nothing louder
comes earlier). All of it goes through the dimOS logger, at warning level for
quiet, into the run's log.

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

## A contact and the map

A contact shakes the lidar as well as the chassis. Measured on the robot,
during the drive into the box turned 45 degrees: in the revolution swept
during the contact the lidar odometry's yaw jumped +6.4 degrees while the
gyro integral moved +0.5 degrees, and that revolution carried 97 points where
its neighbours carried about 300. The revolutions after it registered at the
wrong heading and kept it: one shaken sweep, one jump of the whole map.

So `LidarOdometry` listens to `bump` too (the same stream, whoever publishes
on it: the IMU's front contacts and the explorer's back-off) and stamps the
time of each contact. Then:

1. **The shaken sweep is dropped.** A revolution that arrives within
   `MOSS_CONTACT_WINDOW_S` (0.4 s) after a contact and carries fewer than
   `MOSS_CONTACT_DROP_RATIO` (0.5) times the median point count of the last
   20 registered revolutions is not registered. The pose published for it is
   the last one turned by the gyro integral since then (prior on), or the
   last one unchanged (prior off, or no gyro sample since the last
   revolution: the log then says `pose held (no gyro prior)` or `pose held
   (no gyro sample since the last revolution)`); kiss-icp's map and adaptive
   threshold do not see it, and its cloud never reaches the costmap. One line
   in the log:

   ```
   lidar odom contact: revolution dropped (97 pts, median 300), pose carried by the gyro; dropped by contacts so far: 1
   ```

   A full revolution inside the window is registered as usual: only a thin
   one counts as shaken.

2. **The relocation.** The next `MOSS_RELOC_SWEEPS` (5) registered
   revolutions search wider: their sigma (the kernel, and the correspondence
   distance of 3 sigma) is the adaptive one times `MOSS_RELOC_SIGMA_GAIN`
   (2.0). The prior stays what it is: the gyro's rotation with the prior on,
   the constant-velocity translation. The registration starts from the
   gyro's heading and has room to pull the map back into place. Two lines
   frame it:

   ```
   lidar odom contact: relocating over 5 revolutions (sigma x2.0)
   lidar odom contact: relocation done over 5 revolutions, the registration corrected the yaw by +0.6 deg in total
   ```

   A second contact during a relocation starts its count again.

The knobs are read when the module is imported (a value that does not parse
keeps the default, a value outside its range takes the nearest bound, both
said in the log), and the odometry says them at start, on the line after
`lidar odometry up`:

```
lidar odometry contacts: a revolution within 0.4 s of a bump under 0.5 x the median point count is dropped; then 5 revolutions register with sigma x2
```

| Variable | Default | What |
|---|---|---|
| `MOSS_CONTACT_WINDOW_S` | 0.4 | seconds after a contact during which a thin revolution counts as shaken, 0 to 5 |
| `MOSS_CONTACT_DROP_RATIO` | 0.5 | a revolution under this share of the median point count is dropped, 0 to 1; `0` never drops |
| `MOSS_RELOC_SWEEPS` | 5 | registered revolutions after a contact that search wider, 0 to 50; `0` turns the relocation off |
| `MOSS_RELOC_SIGMA_GAIN` | 2.0 | their sigma times this, 1 to 10 |

`MOSS_CONTACT_DROP_RATIO=0 MOSS_RELOC_SWEEPS=0`: the odometry ignores `bump`, as before.

The contact lines start with `lidar odom` and so appear in
`tools/moss-fly.sh log`. `benches/bench_lidar_odometry_cold.py` section G
holds the mechanism on known numbers: at rest after 10 revolutions of 300
points, a bump and a revolution of 90 points turned 6 degrees leave the yaw
at the gyro's +0.40 degrees and log the drop; the next 5 full revolutions
register with sigma 0.40 m and the 6th with 0.20 m; with no bump, 0.5 s after
it, or with `MOSS_CONTACT_DROP_RATIO=0`, the same revolution registers at
+6.0 degrees. Reloaded with `MOSS_RELOC_SWEEPS=3 MOSS_RELOC_SIGMA_GAIN=3
MOSS_CONTACT_WINDOW_S=0.05`, 3 revolutions register at 0.60 m, and a thin
revolution 0.1 s after the bump registers. `benches/bench_moss_blueprints_cold.py`
checks that `bump` reaches the odometry in moss-nav and moss-explore, on the
costmap's stream.

What it does not cover: a shaken sweep that keeps its point count (the drop
reads the count only); a shaken sweep whose revolution is handled before
`handle_bump` stamps the contact (the window only looks forward from the
stamp, so that sweep is registered at the wrong heading, the measured +6.4
degree case, and the relocation the late stamp arms starts from that
heading); a contact behind, which goes out on `bump_rear`, a stream the
odometry does not listen to; and a contact nothing publishes on `bump`. The
robot replay will show how often a sweep beats its bump.

## Knobs

| Variable | Default | What |
|---|---|---|
| `MOSS_IMU` | on | MossRealSense with the IMU at 200 Hz and the BumpImu module; `0` goes back to dimOS's camera without IMU. Needs the camera (`MOSS_CAMERA=0` refuses it with a warning). |
| `MOSS_BUMP_ACCEL` | 4.0 | threshold in m/s^2 on the high-passed forward accel, above 0 and up to 100 |
| `MOSS_BUMP_AXIS` | `+z` | the forward axis of the camera's frame, x, y or z with a sign |
| `MOSS_BUMP_DEBOUNCE_S` | 0.7 | seconds of silence after a contact, 0 to 10 |
| `MOSS_IMU_LOG` | unset | a CSV path: every sample is appended (below) |
| `MOSS_GYRO` | off | `1` turns on the gyro prior of the lidar odometry from the in-process IMU once `tools/gyro_sign_bench.py` has measured its sign on this robot (above); with `MOSS_IMU=0`, `1` is refused with a warning |
| `MOSS_GYRO_FORCE` | off | `1` with `MOSS_IMU=0 MOSS_GYRO=1`: the prior on dimOS's second IMU pipeline, an experiment |

They are read when the blueprint is built (the build log says
`MOSS blueprint: IMU contacts on bump (front) and bump_rear (rear), forward axis +z, threshold 4 m/s^2, ...`);
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
   camera's own device`, `MOSS IMU: first sample`, then `MOSS IMU: status
   alive` every 10 s, and the cockpit must show
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

   Measured on the robot, one commanded drive at 0.15 m/s (tracks at 54 %) into
   a heavy box 30 cm ahead, every IMU sample logged:

   | what | peak of the high-passed forward acceleration |
   |---|---|
   | at rest | 0.06 m/s^2 |
   | the start of the drive | 1.0 m/s^2 |
   | rolling on the workshop floor | 0.4 to 1.0 m/s^2 |
   | the contact with the box, 2.0 s after the start | **-12.5 m/s^2** (four samples past -4, 20 ms) |
   | the back-off in reverse that followed, a push at 75 % included | 1.1 to 2.0 m/s^2 |
   | the same drive into the box turned 45 degrees (the oblique contact the encoders miss) | **-28 m/s^2** forward, with 19 m/s^2 sideways |

   Both times the detector fired on the first sample past the threshold, and the
   bump sound, the wall on the map and the back-off followed within 60 ms; the
   encoder bumper, which needs a second of frozen tracks, never had to fire.
   The 4.0 m/s^2 default sits six times under this contact and twice above the
   worst motion without one.

## Benches

`benches/bench_bump_imu_cold.py`: the rule on synthetic 200 Hz traces with
gravity on y (a 6 m/s^2 pulse of 60 ms while commanded is exactly one contact
of about -6.0 m/s^2, in front; no command, a wobble, the wrong sign, a tilt, a
pitched camera are none; the debounce; the knobs), the sides (a kick while
reversing is behind; a front contact then a contact behind 0.18 s or 0.4 s
later, during one pivot or once the command reversed, is two contacts; a
kick of the other sign 0.06 s after a contact, the ringing included when the
telemetry already says reverse, is its rebound: one contact, the rebound
counted once), the module (telemetry then samples -> one `bump`, or one
`bump_rear` while reversing, and the log line; a contact and its rebound ->
one `bump` and one rebound line), the CSV, `MossRealSense` on a fake
pyrealsense2 (the motion sensor found by name on the pipeline's own device,
the profiles it opens, stop, every failure leaving the camera running) and
the watchdog on a fake clock and on its own thread (quiet at 1 s, back with
its rate, a burst that stops again says nothing, the rate limit, the status
line, and `start()` starting the thread).
`benches/bench_moss_blueprints_cold.py` sections J, K and L: the blueprint with
and without `MOSS_IMU`, where `bump`, `bump_rear`, `stall`, `imu` and
`base_telemetry` are wired, the gyro prior off by default, and every camera
in a worker of its own. `benches/bench_recovering_planner_cold.py`: `bump`
backs off in reverse, `bump_rear` moves forward, `stall` abandons the goal
with no motion and stops a running escape dead.
