# Litter: see it, go to it, pick it

`moss-litter` is `moss-explore` plus two modules: the litter detection on
the nose camera and the mission that drives from a sighting to the pick.
The arm runs in its own `dimos run` (`moss-arm`, docs/arm.md); the mission
calls its cycle over RPC.

```sh
.venv/bin/dimos run moss-dimos.moss-arm        # one terminal: the arm's coordinator
EXPLORE=1 tools/moss-fly.sh litter             # another: the gates, then explore, detect, approach, pick, explore
```

`tools/moss-fly.sh litter` runs the same gates and proofs as `explore` and
keeps the arming: the mission does not send the first `explore_cmd` itself
under the launcher (`MOSS_MISSION_AUTOSTART=0`). By hand,
`.venv/bin/dimos run moss-dimos.moss-litter` starts exploring on its own.

## Detection

`moss_dimos/litter_detect.py` is dimOS's object scene registration
(`ObjectSceneRegistrationModule`) with one change: it runs a YOLO trained
on litter from a weights file of ours instead of the open-vocabulary model
dimOS downloads. The module aligns colour and depth frames, lifts every box
to a 3D object through the depth image and the camera's transform to the
world frame of the odometry, and publishes on `objects` the ones confirmed
by repeated sightings (`MOSS_LITTER_MIN_DETECTIONS`, 6). Two sightings
closer than 0.2 m are one object.

The weights are not in this repository. The ones the mission is tuned to
are a small YOLO trained on waste images with five material classes
(glass, metal, paper, plastic, waste), published under the MIT licence at
github.com/gianlucasposito/YOLO-Waste-Detection (`best_model.pt`, 6 MB):

```sh
mkdir -p ~/.local/state/moss/models
curl -L -o ~/.local/state/moss/models/litter.pt \
  https://raw.githubusercontent.com/gianlucasposito/YOLO-Waste-Detection/main/best_model.pt
```

`MOSS_LITTER_WEIGHTS` points elsewhere; `MOSS_LITTER_CLASSES` names the
classes the mission picks up (the five by default); `MOSS_LITTER_DEVICE`
forces `cpu` or `cuda`. The detector needs `torch` and `ultralytics` in the
robot's venv, which the base stack does not: on an arm64 host without a
CUDA torch, the CPU wheels do (about 200 ms a frame for this model on the
rover's CPU is the expectation; the camera runs at 5 fps):

```sh
uv pip install --python .venv/bin/python --index-url https://download.pytorch.org/whl/cpu torch torchvision
uv pip install --python .venv/bin/python ultralytics lap    # lap: the tracker ultralytics would otherwise fetch on first use
```

## The mission

`moss_dimos/mission.py`. The decision (`MissionLogic`) is pure and the
bench drives it on a fake robot; the module around it reads `objects`,
`odom` and `goal_reached`, writes `goal_request`, `tele_cmd_vel`,
`stop_explore_cmd`, `explore_cmd` and a `mission_state` line, ticks at
10 Hz.

| State | What happens |
|---|---|
| EXPLORING | the explorer drives; a confirmed object of a litter class within 4 m, not picked or skipped, becomes the target |
| STOPPING | `stop_explore_cmd`, then 3 s for the explorer's own "stay here" goal to land first |
| APPROACHING | `goal_request`: 0.6 m before the object on the line from the robot, facing it; `goal_reached` True → SERVOING, False → the object is skipped; 90 s → SERVOING from wherever the rover is |
| SERVOING | `tele_cmd_vel` at every tick (the pad's channel: the movement manager lets it win): one law drives the robot's pick point onto the object (the pick point is the controlled point: `w = vy / px`, `v = vx + w py`), within 2 cm; at most 0.08 m/s and 0.4 rad/s. The base raises any non-zero command to its power floor and pivots at full power, so within `MOSS_SERVO_FAR_M` (0.25) the twist comes in pulses: one tick on (`MOSS_SERVO_PULSE_S`, 0.1), `MOSS_SERVO_REST_S` (0.4) of zeros, a step at a time; farther it is continuous. 60 s → skipped |
| PICKING | the pick command (`MOSS_PICK_CMD`, default `tools/moss_arm_play.py`; `MOSS_PICK_SIM=1` adds `--sim`; `none` marks the object picked without moving the arm); exit 0 → picked, else skipped |
| RESUMING | `explore_cmd`, then EXPLORING; a picked object, and anything within 0.15 m of it, is never a target again; a skipped one is tried again after 10 min |

**The pick point** is where the cycle's `pick` pose puts the tool, in
base_link (the lidar's floor projection): `MOSS_PICK_XY_M`, default
`0.215 -0.094` (the cycle's tool point `0.345 -0.010` less the scanner's
axis `0.130 0.084`, both in the CAD scene's chassis coordinates). The
camera's mount must use the same origin: with base_link at the scanner,
`MOSS_CAMERA_XYZ_M="0.026 -0.084 0.075"` (the CAD nose position less the
scanner's axis; the profile's default is the CAD nose position itself). The
whole final approach rests on these; a tape measure from the lidar's axis
to the arm's pan axis on the rover is the number to set before a run. `MOSS_STANDOFF_M` (0.6) and `MOSS_PICK_TOL_M` (0.02) are the
other two knobs. `MOSS_MISSION_AUTOSTART=0` leaves the first `explore_cmd`
to the cockpit.

```sh
python benches/bench_moss_mission_cold.py      # the logic on a fake robot; the module under dimOS
```

## What this does not do yet

The approach is open-loop on odometry once the object is confirmed: the
object's world position is refreshed while the camera still sees it, and
the last 20 cm run on the odometry alone. The pick pose is the cycle's
one pose: the rover brings the object to it, the arm does not reach for
the object. The bin is the rover's own. No wrist-camera check before the
grasp.
