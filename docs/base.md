# The tracked base adapter

`moss_dimos/moss_adapter.py` lets dimOS drive MOSS: the dimOS control
coordinator and a gamepad module on top, this adapter underneath. MOSS has two
tracks on geared DC motors, driven by a QT Py ESP32-S3 running the firmware in
`firmware/MOSS_QTPY_MDD3A/`, over the board's native USB CDC at 115200 baud.

## Run it (on the robot, MOSS wired)

The gated way, with the pad receiver and the QT Py on the robot's USB:

```
$ tools/moss-teleop.sh            # gates: no stack up, js0, QT Py port free, firmware disarmed, battery >= 10.8 V -> stack
$ tools/moss-teleop.sh log        # what the base and the pad said
$ tools/moss-teleop.sh stop       # stack down (the firmware deadman stopped the tracks 300 ms after the last drive)
$ MOCK=1 tools/moss-teleop.sh     # same stack on the fake firmware, no motor
```

Pad (a shanwan receiver in Android mode): **R1 held = deadman** (SDL button 7),
left stick up/down = forward/back, right stick left/right = turn, R2 = boost
(capped at `MOSS_MAX_PCT`). The stick scale comes from the base's envelope
(`teleop_speeds`: full stick = `MOSS_MPS_AT_LIMIT`, full turn = a pivot at that
limit), so the stick stays proportional across its travel. With the default
`MOSS_TURN_GAIN` of 2, the inner track stops at half turn stick and reverses
beyond it; a gain of 1 only ever stops it. Releasing R1 sends 0.5 s of zeros,
then silence (`MOSS_BRAKE_S`, 0 to 1.0 s). Unloaded, the motors start between
5 % and 15 % duty; on the floor they only turn at full duty, so the base
adapter delivers every non-zero command as full-duty pulses (`MOSS_PULSE_MS`,
see [cockpit.md](cockpit.md)).

This is the dimOS path. The bench tool in `moss_pi/` drives the same firmware
without dimOS and reads raw SDL button 4 as its deadman: L1 on an Xbox-layout
pad, not on this one ([Software.md](../Software.md)).

By hand, from the root of the checkout:

```
$ ls /dev/serial/by-id/*QT_Py*     # the motor board (the CP2102N next to it is the RPLidar C1)
$ export MOSS_PORT=/dev/serial/by-id/usb-Adafruit_QT_Py_ESP32-S3_..._-if00
$ export GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304   # as the launchers do: some aarch64 (Jetson) imports fail without it; harmless elsewhere
$ .venv/bin/dimos run moss-dimos.moss-gamepad          # gamepad -> tracks
$ .venv/bin/dimos run moss-dimos.moss-base             # base only, cmd_vel from elsewhere
```

Without hardware: `MOSS_MOCK_BUS=1 .venv/bin/dimos run moss-dimos.moss-gamepad`
(same `GLIBC_TUNABLES` export on a Jetson) drives the fake firmware
(`moss_dimos/moss_mock.py`) end to end; the `.venv/bin/dimos log` lines
`MOSS base MOCK: twist ... -> tracks L=..% R=..%` are the proof.

Cold bench (no hardware, no dimOS): `PYTHONPATH=. python3 benches/bench_moss_adapter_cold.py`.

## What the firmware imposes, and what the adapter does about it

| Firmware (Control.h) | Adapter |
|---|---|
| `arm` puts it READY for 10 s, then it disarms itself | a drive refused `ERR not_armed` re-arms and retries once; nothing is guessed from telemetry |
| `drive L R` in percent, `-100..+100`, refused beyond | `MossGeometry.max_pct` <= 100, clamped; the twist is scaled to it |
| a drive not refreshed within 300 ms stops the tracks (`timeout`) | keepalive re-sends the last drive every 100 ms while the last twist is < 0.5 s old; beyond that one `stop`, then silence |
| any invalid line stops and disarms | only `arm` / `drive` / `stop` / `status` ever go on the wire, plus one `ramp UP DOWN` at connect when `MOSS_RAMP_MS` is set |
| telemetry JSON every 200 ms (`mode`, `ticks`, ...) | mode and encoder ticks are read from it; ticks feed odometry once calibrated |

Two deadmen, both on the robot: the adapter's (0.5 s, in the dimOS process)
and the firmware's (0.3 s, on the ESP32). A cockpit on a laptop that loses the
link stops the tracks without anyone's help.

A third guard watches the body itself, for unattended runs: the speed watchdog,
`tools/speed_guard.py`. It reads the run's dimOS log and measures the speed
between consecutive `lidar odom #N` lines (one every 2 s), dated by the
timestamps the log carries. One interval faster than the profile's
`max_speed_mps` (0.45 m/s, `MOSS_MAX_SPEED_MPS`) stops the run: the exploration
stop on the bus (`tools/explore_ctl.py stop`), then `tools/moss-fly.sh stop`,
and the firmware deadman stops the tracks 300 ms later. It guards one run and
stands down by itself when a newer run starts or the log stays silent for 30 s.
The ceiling is the planner's cruise plus a margin: dimOS's follower drives at
0.55 m/s times the launcher's speed scale (`MOSS_NERF_SPEED`, 0.7), 0.385 m/s,
and 0.45 m/s leaves 0.065 m/s above it. A ceiling at or under that cruise would
stop every normal run, so the watchdog refuses to start with one and says which
number to change. `tools/moss-fly.sh` does not arm it on its own: once the stack
is proven, run `tools/arm_speed_guard.sh <run log dir>` by hand (without the
argument it takes the newest run). The script kills the previous run's watchdog
by the pid in `/tmp/moss_speed_guard.pid`, starts a fresh one and exits 1 unless
that file then names a process born after the launch; the watchdog writes to
`/tmp/moss_speed_guard.log`. Like the speeds below, the ceiling is a
placeholder: once the floor speed is measured (`MOSS_MPS_AT_LIMIT`), check that
what an exploration run really drives stays under it.

The coordinator publishes the firmware telemetry on `/base_telemetry` every
0.5 s (JSON: `ts`, `fw`, `driver`, `mode`, `reason`, `bus_mV`, `ina_ok`,
`ticks`, `target_pct`, `output_pct`, `ramp_ms`, `firmware_age_s`, `mock`), so
nothing else ever has to open the serial port.

## robot.env: what this robot is, declared once

`~/.local/state/moss/robot.env` (or `$MOSS_STATE_DIR/robot.env`), `KEY=VALUE` lines. The launchers
(`tools/moss-fly.sh`, `tools/moss-teleop.sh`) read it before anything else, the flight check reads the
same file, so a verdict there is a verdict on what will actually run. The shell's own exports win.

```
MOSS_DRIVER=mdd10a          # the motor driver on the board: the flight check refuses a firmware built for another one
MOSS_PAD_LAYOUT=xinput      # android = the pad on its 2.4 GHz dongle; xinput = the same pad on its USB cable (an Xbox 360 pad to the kernel)
MOSS_MIN_PCT=20             # measured floor: both tracks turn from 20 %
```

The flight check then refuses, in red, what used to pass as "OK": a firmware that does not match
`MOSS_DRIVER`, a pad the receiver has never heard (every axis at its never-reported value), a pad
plugged in a layout other than the declared one. And `tools/moss-teleop.sh` prints "YOU CAN DRIVE"
only once the stack itself has written that it saw the sticks at neutral and trusts the commands.

## Calibrate before trusting a number

The firmware is open loop (PWM percent). Three numbers turn a twist into
tracks and ticks into metres; all three ship as PLACEHOLDERS and each has an
env override, so a run can be re-scaled without a code change:

| Env | Meaning | How to measure |
|---|---|---|
| `MOSS_TRACK_M` | distance between the two track centrelines (0.266 m from the CAD) | tape measure, sprocket centre to sprocket centre |
| `MOSS_MPS_AT_LIMIT` | floor speed at `MOSS_MAX_PCT` (0.45 m/s placeholder) | full stick (or `drive 100 100`) for 3 s on the floor, tape measure, divide |
| `MOSS_TICKS_PER_M` | encoder ticks per metre of track travel (unset) | `zero`, push or drive the rover exactly 1 m, read `ticks` in a `status` line, average the two sides |
| `MOSS_MIN_PCT` | static-friction floor: the lowest percent that turns both tracks (50 on MOSS: 40 measured on a full pack, 10 of margin; 0 = off). An arc whose inner track sits under the floor is scaled up whole, so the curvature the planner asked for survives; raising only the inner track would straighten it | `tools/moss_floor_probe.py --go` with the stack down, the rover on blocks or with a clear metre ahead: it steps the percent up, reads the encoders around each step and prints the value to export |

| `MOSS_BUS_REF_MV` | the bus voltage the floor and the speeds were measured at (12400 = a full 3S pack; 0 = off). A percent is a duty of the bus, so a pack at 10.5 V under load puts 15 % less voltage on the motors for the same command and the tracks creep or freeze: every command is scaled by reference/bus (never below 1, at most 1.5, capped at the limit) | read `bus_mV` in the telemetry while the probe measures the floor |
| `MOSS_ONE_TRACK_PIVOT` | a pivot whose one track freezes while the other turns is not a contact: that track did not turn in that direction under the pivot's load. It is braked (0) and the pivot turns on the other track for the rest of the run, said once in the log; both tracks frozen in that turn is a contact. On by default, 0 = such a pivot is a stall | the stack log: `pivot ... did not turn` |

Until `MOSS_TICKS_PER_M` is set, `read_odometry()` returns `None` and
`read_velocities()` returns the commanded twist: an honest "no measurement"
rather than a pose made of guesses. Set it and both come from the encoders.

The other base knobs: `MOSS_MAX_PCT` (percent ceiling, 100 by default, never
above the firmware's 100), `MOSS_PULSE_MS` (full-duty pulse period, 0 = off, the default: continuous power), `MOSS_MIN_PCT` (static-friction floor when pulsing is off, 50 by default), `MOSS_PIVOT_PCT` (every pivot sent at this percent on both tracks, 100 by default),
`MOSS_RAMP_MS` (`"UP DOWN"`, sent as `ramp UP DOWN` at connect), `MOSS_PORT`
(the serial port; by default the QT Py's `/dev/serial/by-id` path; only a
`dimos run` by hand reads it from the shell, `tools/moss-teleop.sh` and
`tools/moss-fly.sh` always take the by-id path) and `MOSS_MOCK_BUS=1` (the fake
firmware, no motors). The other variables are listed in
[cockpit.md](cockpit.md#the-robot-profile).

`vy` (strafing) is accepted and ignored: a tracked base cannot do it, and the
stock dimOS twist joints still send it. `dof` is reported as 3 for that reason.
