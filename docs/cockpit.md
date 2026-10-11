# Running MOSS: flight check, mapping, exploration, recording, cockpit

MOSS is a small tracked rover (two geared DC motors behind a QT Py ESP32-S3
motor board, an RPLidar C1, a RealSense D455F on the nose, a UVC camera on the
arm). This is the operator's manual for running it on dimOS with this
repository's `moss_dimos` package: lidar odometry, a 2D costmap, a planner and
a frontier explorer under the tracked base, with a flight check and a web
cockpit in front of them.

Everything below runs ON THE ROBOT, on the computer where the board, the lidar
and the cameras are plugged in. The cockpit is a web page served by the robot;
open it from any machine on the same network.

## The four pieces

| Piece | File | Runs where |
|---|---|---|
| Flight check | `tools/moss_preflight.py` | robot, read-only, ten seconds |
| Blueprints | `moss_dimos/blueprints.py` → `moss-dimos.moss-nav`, `moss-dimos.moss-explore` | robot, inside `dimos run` |
| Launcher | `tools/moss-fly.sh` | robot, gates then stack |
| Cockpit | `tools/moss_cockpit.py` + `tools/moss_cockpit/` | robot, port 8904 |

The pad-only launcher, `tools/moss-teleop.sh` ([base.md](base.md)), is the
smallest thing that moves the robot.

## Run it

```
$ tools/moss-fly.sh check              # flight check only, nothing launched
$ tools/moss-fly.sh map                # gates -> moss-nav: lidar map + pad teleop, recording
$ tools/moss-fly.sh explore            # gates -> moss-explore: map + navigation + explorer, NOT armed
$ EXPLORE=1 tools/moss-fly.sh explore  # ... and arm exploration once the stack is proven
$ tools/moss-fly.sh stop               # stop exploration, then the stack
$ tools/moss-fly.sh log                # what the base, the lidar, the odometry and the map said
$ tools/moss-fly.sh cockpit            # start the cockpit server if it is not up, print the URL
$ MOCK=1 tools/moss-fly.sh map         # fake motor board, no motor; the lidar must still be plugged (mapping needs it)
```

Every gate refuses rather than guesses, with the fix on the same line: a stack
already up, a missing board, a port held by another program, a mute firmware, a
battery at the BMS floor, a lidar that does not spin, a stack that does not
prove itself in the log within two minutes.

The map opens when the rover has moved 5 cm from its boot pose (the costmap's
mission gate) or when an `explore_cmd` arrives, and it stays open until the
stack stops: the cockpit's STOP, a goal reached or the end of exploration leave
map writing on. A new launch starts a fresh map.

Nothing moves by itself. `moss-nav` has no planner: the pad drives, the map
builds. `moss-explore` adds the planner, the movement manager and the frontier
explorer, but the explorer only starts on an explicit `explore_cmd` (the
launcher with `EXPLORE=1`, the cockpit's button, or
`.venv/bin/python tools/explore_ctl.py start` from the checkout root), and the
planner only on a goal (a tap on the cockpit map, or `goal_request` on the
bus). Any pad input cancels navigation (dimOS MovementManager), and two deadmen
stay underneath: the adapter stops the tracks 0.5 s after the last twist, the
firmware 300 ms after the last drive.

## What the blueprints contain

`moss-dimos.moss-nav` (mapping):

- the control coordinator (`MossControlCoordinator`) with the MOSS base adapter
  (`moss_dimos/moss_adapter.py`, see [base.md](base.md)), publishing the
  firmware telemetry on `/base_telemetry` (JSON: firmware, mode, battery mV,
  ticks, applied PWM) so nothing else ever has to open the serial port;
- the gamepad module (`moss_dimos/gamepad.py`) with the MOSS envelope, wired
  straight to `cmd_vel`;
- the RPLidar C1 reader (`moss_dimos/rplidar_c1.py`) on its own USB serial adapter;
- lidar odometry (kiss-icp, `moss_dimos/lidar_odometry.py`), the pose source:
  the encoders give metres only once `MOSS_TICKS_PER_M` is set, and no IMU
  feeds the odometry;
- the 2D scored costmap (`moss_dimos/costmap2d.py`), where the lidar writes obstacles
  and the depth camera adds the ones below the scan plane. An obstacle becomes a
  wall once seen from two places 10 cm apart, never from one spot. The lidar's
  rays clear the lidar layer within 4 m of the rover all around, and the camera
  layer only inside the camera's own view (87 degrees ahead, up to 3 m): somebody
  who walked past fades once gone (about 0.2 s of rays through a fresh wall, 1.8 s
  through one seen from many places), what the camera keeps seeing stays, and a
  table top the rover has turned away from keeps its wall until the camera looks
  at that spot again and sees nothing. The knobs are `MOSS_HIT_CAP`,
  `MOSS_FREE_FLOOR` and `MOSS_RAY_MAX_M` (below);
- an experiment, off by default: the lidar odometry's rotation prior from the RealSense's
  gyro (`MOSS_GYRO=1`): a full-power pivot turns the rover 25 degrees between two scans,
  more than the constant-velocity prior follows, and the map drifts; the gyro would say
  where the rover turned, but the IMU pipeline fails to open on the rover today and takes
  the camera down with it;
- the sound module (`moss_dimos/sounds.py`): a pack of short synthesized cues in
  `assets/sounds` played through the robot user's PulseAudio session (a Bluetooth
  speaker once connected, the default sink otherwise) on confirmed events only: the
  stack up, a contact, the tracks moving again after a stall, the order to explore,
  the end of an exploration, the battery crossing low and critical (with hysteresis).
  Each cue has a cooldown and the player keeps a gap between cues, so the rover
  never chatters. `MOSS_SOUNDS=0` leaves it out, `MOSS_SOUND_VOLUME` (0 to 1) and
  `MOSS_SOUND_SINK` (a PulseAudio sink name) tune it; a stack launched as a system
  service has no audio session and stays silent;
- the RealSense colour stream at 640x480, 5 fps, carried as JPEG on the bus, and its
  depth aligned to colour at the same rate, which `lidar_odometry` turns into obstacle
  points from 12 cm (the camera's close-range depth) to 3 m, drawn in amber on the map
  (`MOSS_CAMERA=0` leaves it out; `MOSS_RS_SERIAL=<serial>` picks one camera
  when several are plugged in, and the flight check looks for the same one);
- the wrist camera (the UVC camera on the arm, found by its `/dev/v4l/by-id`
  name, `MOSS_WRIST_DEV` to point at another `/dev/videoN`), 640x480, 5 fps,
  JPEG on the bus as `wrist_image` (`MOSS_WRIST=0` leaves it out; an absent
  camera is a warning and the module is skipped);
- the recorder (below), `MOSS_RECORD=0` leaves it out.

`moss-dimos.moss-explore` adds the recovering A* planner
(`moss_dimos/recovering_planner.py`) with the MOSS footprint, the movement
manager (pad input wins over navigation for one second) and Explorer2
(`moss_dimos/explorer2.py`) with the MOSS clearances. The pad is still there,
through the manager.

Three variables decide how far the explorer sends the rover. They are read when
`moss-explore` is built; unset keeps the default, and a value that is not a
number of 0 or more (0 to 6 for the exponent) is refused with a warning (the default stays):

| Env | Default | What it does |
|---|---|---|
| `MOSS_EXPLORE_DISTANCE_EXP` | 2 | how hard distance counts: a frontier scores its information over (1 + its path cost in metres) to this power; a metre of unmapped floor counts as 4, a metre hugging a wall up to 2. At 1 a frontier twice as far needs only twice the information to win, so a big frontier across the room beats a small one beside the rover; at 2 it needs four times as much |
| `MOSS_GOAL_MAX_M` | 3 m | the longest path cost a goal is sent along (0 = no limit), in the same metres as above; a frontier beyond it only waits while a safe one lies within it. A farther frontier waits until the rover is nearer; when nothing nearer is left it goes out anyway, so the cap never ends a run |
| `MOSS_EXPLORE_RADIUS_M` | 0 in the code; `tools/moss-fly.sh` exports 4 (one room) unless set | the run stays within this distance of where exploration started: a frontier whose goal lies farther is never a candidate, and once nothing is left inside, exploration is complete. Set 4 to keep the rover in one room. A doorway is a frontier like any other: a door the rover must not take has to lie outside the radius Each start of exploration (the start button, after STOP or after a tapped goal) re-centres the radius on where the rover stands then. |

The explorer logs the three values in one line when exploration starts
(`exploration start: distance exp 2, cap 3 m, radius 0 m`); each goal line
gives its path cost.

`moss-dimos.moss-gamepad` and `moss-dimos.moss-base` are the base alone, with
or without the pad: [base.md](base.md).

None of them starts Rerun or dimOS's own web modules: the cockpit is the
display. `MOSS_RERUN=1` adds the Rerun bridge for the desktop viewer.

The switches `MOSS_CAMERA`, `MOSS_WRIST`, `MOSS_RECORD` and `MOSS_RERUN` read
`0`, `false`, `no` and `off` (any case) as off, anything else as on, and unset
or empty as their default. The flight check reads `MOSS_CAMERA` the same way.

## The robot profile

`moss_dimos/robot_profile.py` carries MOSS's geometry: lidar height and masks,
body box, footprint, planner widths, camera mount. Every module reads it when
it needs a value, so an environment variable set before the launch applies
everywhere. The values are PLACEHOLDERS until measured on the robot:

| Value | Placeholder | Env |
|---|---|---|
| lidar scan plane above the floor | 0.20 m | `MOSS_LIDAR_HEIGHT_M` |
| lidar yaw offset (its 0 deg mark vs the nose) | 0 deg | `MOSS_LIDAR_YAW_DEG` |
| lidar returns ignored closer than | 0.15 m | `MOSS_SCAN_MIN_RANGE_M` |
| masked lidar sectors, and the range they apply within | none, 0.30 m | `MOSS_SCAN_MASK_DEG="a b, c d"`, `MOSS_SCAN_MASK_RANGE_M` |
| returns that are the robot itself (half extents) | 0.22 x 0.18 m | `MOSS_BODY_BOX_M="0.22 0.18"` |
| area cleared under the body (half extents) | 0.20 x 0.16 m | `MOSS_BODY_CLEAR_M="0.20 0.16"` |
| robot width for the planner | 0.34 m | `MOSS_ROBOT_WIDTH_M` |
| pivot diameter | 0.50 m | `MOSS_PIVOT_DIAMETER_M` |
| control margin | 0.05 m | `MOSS_CONTROL_MARGIN_M` |
| track centre distance | 0.266 m (CAD) | `MOSS_TRACK_M` |
| nose camera position on the base | 0.156, 0, 0.075 m (CAD) | `MOSS_CAMERA_XYZ_M="0.156 0 0.075"` |
| nose camera pitch | 0 deg | `MOSS_CAMERA_PITCH_DEG` |
| full-duty pulse period of the base adapter (ms, 0 = off) | 0 | `MOSS_PULSE_MS` |
| static-friction floor of the base adapter when pulsing is off (percent) | 50 | `MOSS_MIN_PCT` |
| percent every pivot is sent at, both tracks (0 = as asked) | 100 | `MOSS_PIVOT_PCT` |
| bus voltage the floor was measured at, for the battery compensation (mV, 0 = off) | 12400 | `MOSS_BUS_REF_MV` |
| a pivot with one frozen track turns on the other track instead of counting as a contact | on | `MOSS_ONE_TRACK_PIVOT` |

The speed knobs are `MOSS_MAX_PCT` (the PWM cap of the base adapter, 100 by
default) and `MOSS_NERF_SPEED` (the launcher's `--nerf-speed` for the planner,
0.7 by default).

The base runs on continuous power: the tracks turn from 40 % on a full pack,
so `MOSS_MIN_PCT` (50 by default, 10 of margin for the sag under load) raises
every non-zero command to the floor, an arc scaled up whole so its curvature
survives, and every pivot goes out at `MOSS_PIVOT_PCT` (100) on both tracks.
A percent is a duty of the bus voltage, so a pack at 10.5 V under load puts
15 % less on the motors than the 12.4 V the floor was measured at: the adapter
scales every command by `MOSS_BUS_REF_MV` / bus (never below 1, at most 1.5,
capped at 100). A pivot whose one track freezes while the other turns is that
track not turning in that direction under the pivot's load, not a contact:
the adapter brakes it and turns on the other track for the rest of the run
(`MOSS_ONE_TRACK_PIVOT`, said once in the log). `tools/moss_floor_probe.py
--go` measures the floor (raw percents: it turns the compensation and the
rewrite off), `tools/moss-pulse-test.sh` and `MOSS_PULSE_MS` keep the
full-duty pulse mode for a base whose tracks only turn at full power
([base.md](base.md)).

The lidar's floor projection is `base_link`, the rover's rotation centre: mount
the C1 there, or the footprint and the pivot clearance are off by its offset.

Other variables:

| Env | What it does |
|---|---|
| `MOSS_PORT`, `MOSS_LIDAR_PORT` | the motor board's and the lidar's serial ports; by default the QT Py's and the CP2102N's `/dev/serial/by-id` paths. Only a `dimos run` by hand reads them from the shell: `tools/moss-fly.sh` and `tools/moss-teleop.sh` always take the by-id paths |
| `MOSS_RS_SERIAL` | the RealSense to use, by serial number |
| `MOSS_REQUIRE_PAD=1` | a missing pad fails the flight check instead of warning |
| `MOSS_COCKPIT_PORT` | the cockpit's port, 8904 by default (also `--port`; `tools/moss-cockpit.service` sets it) |
| `PROOF_S` | how long `tools/moss-fly.sh` waits for the stack to prove itself in its log, 120 s by default |
| `MOSS_MOCK_BUS=1` | the fake firmware, no motors (`MOCK=1` on the launchers sets it) |
| `MOSS_STATE_DIR` | the state directory, `~/.local/state/moss` by default: recordings, the saved flight check |
| `MOSS_BRAKE_S` | how long the pad sends zeros after the deadman is released, 0.5 s by default, 0 to 1.0 s |
| `MOSS_TURN_GAIN` | turn authority against forward speed on the pad, 2 by default, 0.5 to 4 ([base.md](base.md)) |
| `MOSS_HIT_CAP` | how certain a costmap cell can get, 10 by default, 2 to 127. A cell at the cap needs 2 x (cap - 1) lidar revolutions with rays through it to be forgotten (1.8 s at 10, 0.6 s at 4): lower it to forget passers-by faster, at the price of walls that fade sooner once nothing sees them |
| `MOSS_FREE_FLOOR` | how free a costmap cell can get, -3 by default, -128 to 0. Nearer 0, something that appears on floor seen free for long becomes a wall after fewer viewpoints (for the lidar: 3 at -1, 5 at -3) |
| `MOSS_RAY_MAX_M` | how far from the rover the lidar's rays clear the costmap, 4 m by default (0 = never). Anything farther than that from every place the rover stood is never forgotten: 6 forgets somebody who stood 4.5 m away, and longer rays cost more CPU per revolution |

## Recording

`MossRecorder` (`moss_dimos/memory.py`: a dimOS `Recorder` with a WAL-safe
rotation) writes the whole run into one SQLite database,
`~/.local/state/moss/recordings/moss.db` (`MOSS_STATE_DIR` moves the state
directory; a symlink to a larger disk works). Streams: `odom`,
`global_costmap`, `lidar` (world-frame cloud), `cmd_vel`, `base_telemetry`,
`color_image` (JPEG), `wrist_image` (JPEG) and `episode_marker`. The previous
run is rotated aside with its `-wal` and `-shm` files (`moss.<stamp>.db`, the
last ten kept).

The cockpit's "Record episode" button does not start or stop the recorder: it
writes a marker (`{"action": "start"|"stop", "name": ..., "ts": ...}`) on the
`episode_marker` stream. dimOS's `dataprep` cuts episodes on its own `status`
stream or on hand-entered ranges, not on these markers; cutting on them takes a
small script of your own. Reading a recording without the robot:

```
$ .venv/bin/dimos mem summary ~/.local/state/moss/recordings/moss.db
$ .venv/bin/dimos dataprep build --source <db> --format lerobot --output <dir> --config <json>
```

`--config` is required: a JSON `DataPrepConfig` naming the observation and action
streams (for MOSS: `color_image`, `odom`, `cmd_vel`) and the episode extractor.

## The cockpit

`tools/moss_cockpit.py` is a stdlib HTTP server on port 8904 (`MOSS_COCKPIT_PORT`
or `--port`) that taps the
dimOS bus (explicit LCM channels, decoded on request, never a serial port) and
serves one page, `tools/moss_cockpit/index.html`, in the MOSS brand (Space
Grotesk, studio blue, mint accent, salmon counterpoint; fonts and marks served
locally, no CDN, the field has no internet).

Panels: flight check (the last `moss_preflight.py --json`), base (battery on a
3S scale, mode, applied PWM per track, ticks, firmware), system (CPU, RAM,
temperatures, disk), the live map (occupancy grid + lidar scan + pose + path +
goal; drag to pan, wheel or pinch to zoom, tap to send a goal), the two cameras
under the map (nose = RealSense colour, wrist = the arm's camera; MJPEG, 5 fps
each, one "Pause video" for both), controls (launch map / explore, stop stack,
start / stop exploration, record episode, STOP) and the log tail.

STOP on the page is a soft stop: it publishes `stop_explore_cmd`,
`stop_movement` and a zero twist. The hard stops are the firmware deadman
(300 ms) and `tools/moss-fly.sh stop`.

A goal tapped on the map goes out at once when nothing is exploring. While
exploring, the page stops the explorer first and sends the goal three seconds
later, after the explorer's own last goal, which would override it otherwise.
The cockpit refuses goals, exploration and launches when the running stack is
not a MOSS one (its blueprint name must start with `moss`), so another dimOS
stack sharing the computer and the bus never receives its commands; STOP is
always allowed. Commands that publish on the bus need the stack on LCM
(`DIMOS_TRANSPORT`, else `TRANSPORT`, set to `lcm`; the launchers export both,
`lcm` by default) and are refused with the reason otherwise. Every `POST /cmd/*` must carry the header
`X-Requested-With: moss-cockpit` (the page does, a cross-site form cannot).

Endpoints, for anything else that wants the data: `/state.json`, `/map.png`,
`/map_meta`, `/scan.json`, `/path.json`, `/log.json`, `/preflight.json`,
`/launch.json`, `/camera.mjpg` (`?cam=nose`, the default, or `?cam=wrist`), and

`/scan.json` splits the latest `lidar` cloud by height: `points` (and `n`) are the lidar returns, at
the profile's lidar height; `camera` (and `camera_n`) are the depth camera's obstacle points, at
their true height, which the page draws in amber; `pose` is the rover's pose at the
scan's time (x, y, yaw, age_s), so the page moves the rover marker at the scan's
rate (3 Hz) rather than the state's (1 Hz). The raw `/pointcloud` fallback has no
`camera` key. The map panel's **Clear map** button (`/cmd/map` with `action: clear`,
`tools/explore_ctl.py clear` from a shell) makes the costmap start a fresh grid centred on
the rover, without stopping the stack or the run: an open exploration keeps writing from
the next revolution. The base panel shows a STALLED badge for at least 4 s whenever the
base adapter's stall guard fires (a commanded track whose encoder did not move for
1 s: the tracks stop and the planner backs off), and a `stalls` row counts them
over the run.
`POST /cmd/{explore,goal,stop,episode,stack}`.
`tools/moss_cockpit.py --demo` serves synthetic data for working on the page
without a robot.

## The cockpit as a service

By hand, `tools/moss-fly.sh cockpit` starts the server and prints its address;
the launcher also restarts a server older than `tools/moss_cockpit.py`, and
refuses one that runs from another checkout, naming its directory. To have it
start with the robot, install `tools/moss-cockpit.service`. The unit is a
template: `MOSS_USER` is the account that runs the stack, `MOSS_DIR` the
absolute path of this checkout (with its `.venv`). From the root of the
checkout:

```
$ sed -e "s|MOSS_USER|$USER|g" -e "s|MOSS_DIR|$PWD|g" -e "s|MOSS_UID|$(id -u)|g" tools/moss-cockpit.service \
    | sudo tee /etc/systemd/system/moss-cockpit.service >/dev/null
$ sudo systemctl daemon-reload && sudo systemctl enable moss-cockpit && sudo systemctl restart moss-cockpit
```

`restart`, not `enable --now`: `enable --now` leaves a cockpit that is already
running as it is, still serving from its old checkout. `KillMode=process` keeps
a stack launched from the page alive when the cockpit restarts; stop a flight
with `tools/moss-fly.sh stop`. The installed unit holds absolute paths: after
moving the checkout, stop any flight (`tools/moss-fly.sh stop`), run the two
lines above again from the new place, and check that the running cockpit is
this checkout: `readlink /proc/$(systemctl show -p MainPID --value moss-cockpit)/cwd`
prints its path.

A stack launched from the page needs the same LCM settings as one launched from
a shell ([Software.md, First run](../Software.md#first-run-system-settings)): either
passwordless sudo for the stack's user, or `tools/lcm-loopback.service`, which
turns on loopback multicast, adds the `224.0.0.0/4` route on `lo` and raises the
socket buffers once at boot:

```
$ sudo cp tools/lcm-loopback.service /etc/systemd/system/
$ sudo systemctl daemon-reload && sudo systemctl enable --now lcm-loopback
```

## Limits

- The geometry and the base numbers are placeholders until measured: ground
  speed, ticks per metre, track width, lidar pose, the footprint and the
  static-friction floor. [base.md](base.md) says how to measure the base's.
- The lidar odometry runs without a gyro prior (`use_gyro_prior=False`); the
  D455F IMU could feed it once the camera mount is measured, and there is no
  switch for it.
- Every launch starts a fresh map: no map is saved between runs and there are
  no keep-out zones. The recording keeps the run.
- A recording is not replayed through the stack; the SQLite file is readable
  with the dimOS tools above.
- The planner's slowest pivot (0.2 rad/s, about 6 % PWM at the defaults) sits
  inside the motors' dead band at the current gearing: planned turns in place
  stall without a static-friction floor (`MOSS_MIN_PCT`, above).

## Deploying from another machine

The robot can run a git clone or an rsync copy of this repository. With rsync,
leave the venv and the package metadata out of the sync and install the package
on the robot itself:

```
$ rsync -a --exclude .venv --exclude .git --exclude 'moss_dimos.egg-info' ./ <user>@<robot>:moss/
$ ssh <user>@<robot> 'cd moss && uv pip install --python .venv/bin/python -e . --no-deps'
```

uv's installer puts it in `~/.local/bin`, which a command run over ssh may not
have on its `PATH`: give the full path there if `uv` is not found. In a venv made
with `python3.12 -m venv`, `.venv/bin/pip install -e . --no-deps` does the same.

New blueprint names only become runnable after that install on the robot. An
rsync that carries `moss_dimos.egg-info/` from a development machine overwrites
the robot's copy: the flight check then reports the MOSS blueprints missing
while the venv knows them. Run the install again on the robot to regenerate it.

## Cold benches

`benches/bench_moss_preflight_cold.py`, `benches/bench_moss_fly_gates_cold.py`,
`benches/bench_moss_cockpit_cold.py` and `benches/bench_explore_ctl_cold.py`
cover these tools, without the robot. Run one with
`PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/<name>.py`, or
every bench with `benches/run_all.sh` ([Software.md](../Software.md#tests-and-benches)).
`bench_explore_ctl_cold.py` needs `lcm` and `dimos_lcm`, which come with dimOS;
the other three run on any Linux machine and skip, with a line that says so,
the sections that need dimOS.
