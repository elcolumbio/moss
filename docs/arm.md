# The arm

MOSS carries an SO-101 arm: five joints and a gripper, six Feetech servos
on one bus (protocol 0, 1 Mbps, the port in `MOSS_ARM_PORT`).

| Joint | Servo ID | What it does |
|---|---|---|
| `shoulder_pan` | 1 | the base: turns the whole arm about the vertical |
| `shoulder_lift` | 2 | the shoulder: tips the upper arm |
| `elbow_flex` | 3 | the elbow |
| `wrist_flex` | 4 | the wrist, up and down |
| `wrist_roll` | 5 | the wrist, turning the gripper about its axis |
| `gripper` | 6 | a CF35-12 force-controlled gripper, with its own register map |

## Calibration

`tools/moss_arm_calibrate.py --sweep`, with the torque off: move every joint
by hand from one end stop to the other, then stop the tool. For each joint
the middle of its travel becomes 2048 (the servo's homing offset, register
31) and the end stops become its position limits (registers 9 and 11). The
gripper gets its limits only. `--show` prints what the servos hold.

The servos' EEPROM holds the calibration; `config/arm_calibration.json`
mirrors it, one entry per joint:

```json
"shoulder_lift": {"id": 2, "homing_offset": -1369, "range_min": 860, "range_max": 3236, "travel_ticks": 2376}
```

A servo reports `raw - homing_offset`: every position in this document and
in the code is that reported value, in ticks (4096 per turn, 0.088 degrees
each). The file describes one arm, so it lives in that robot's checkout.

## Named poses

The same file keeps named poses under `"poses"`, each a tick per servo and a
note. They are recorded by reading the six servos with the arm placed by
hand (torque off), and replayed by commanding the same ticks.

| Name | Pose |
|---|---|
| `drive` | driving: the arm folded, a few millimetres past the nose, the wrist camera looking up. The pose for driving, and the one to return to whenever a collision is possible. |
| `look` | observation: from `drive`, the shoulder a little higher and the wrist tilted, so the wrist camera, higher than the RealSense, looks ahead for litter. |

## Keep-out zones

The arm can hit its own robot: every element of the rover that the model
holds, the hull, the bin, the tracks and treads, the deck, the nose camera
and its mount, the parts beside the arm's base, and the lidar. The keep-out
table says, for every pose of the five joints on a grid, whether the arm
comes closer than the margin to one of them, and which.
`moss_dimos/arm_guard.py` reads it and refuses the poses it forbids.

### How the table is made

`tools/moss_arm_keepout.py` sweeps it on the robot's MuJoCo model:

```sh
pip install mujoco
python tools/moss_arm_keepout.py --model <robot.xml>       # writes config/arm_keepout.json
```

* **The rover** is every geom of the `rover` body and of the bodies under
  it, except the arm, the bodies that carry the arm (`base_link`) and the
  geoms `--exclude` names. The geoms named `hull`, `bin_*`, `track_*`,
  `tread_*` and `lidar`, and every other box, sphere, capsule, cylinder or
  ellipsoid, count as they are. Every other mesh counts through stand-ins:
  MuJoCo collides a mesh as its convex hull, which would fill the inside of
  the bin or the deck around the arm's base. The tool samples the mesh's
  surface, keeps the samples that lie more than `--cover-tol` (2 mm) outside
  the parts above, and covers them with boxes on a `--stand-in-voxel` grid
  (4 mm). Every point of the surface then lies inside a stand-in or within
  `--cover-tol` plus half a voxel of a named part, and a stand-in reaches
  past the surface by less than one and a half voxels along each axis. The
  JSON keeps, for every mesh, the share
  of its surface the named parts miss, where that share lies in the rover
  frame, and its number of boxes.
* **The arm** is every geom from `shoulder_link` down: the links, the servo
  shells, the palm and the pads, the visual meshes included. A mesh collides
  as its convex hull, a little larger than the part, never smaller.
* **The root link** (`shoulder_link`, which only the pan moves) is done
  apart. Its meshes become surface stand-ins (`--root-voxel`, 1 mm), since
  the hull of its bracket would sweep through the deck and the parts beside
  the base. It is placed every degree of pan (`--root-step-deg`), and a
  contact there forbids every pan value of the grid within half a step,
  whatever the other joints do. It turns in place at a clearance the build
  sets, so it counts the places where it meets the rover (`--root-margin`,
  0 m), not the margin. A part it touches at every pan value is where the arm
  sits: those contacts are not counted (`mount_parts` in the JSON).
* **The lidar** is not in the model. When no geom is named `lidar`, the tool
  adds a cylinder (radius 0.038 m, height 0.04 m) on the rover body at the
  profile's pose: on the rotation centre (x = y = 0, since `base_link` is the
  lidar's floor projection) and `lidar_height_m` up (0.20 m,
  `MOSS_LIDAR_HEIGHT_M`). `--lidar-xyz` sets its centre. The tool prints the
  rover parts the cylinder goes into and stores them with their depth in
  `lidar.overlaps_m`.
* **The fingers** take every position: each geom on a finger is replaced by
  its swept volume, the convex hull of the geom at both ends of the slide, on
  the gripper body. A closed, open or half-open gripper is inside it.
  `--fix finger_left=0.02,...` pins a finger instead.
* **The grid** spans each joint's model range widened by 10 degrees on both
  sides: the arm's calibrated travel ends a few degrees past the model's
  limits, and the `drive` pose sits on its end stops. The default grid is
  16 x 16 x 16 x 12 x 8 points (pan, lift, elbow, flex, roll): one step is
  14 to 19 degrees on the first four joints, 49 on the roll.
* **Each grid point** is placed on the model (`mj_kinematics`, the position
  stage of `mj_forward`), then `mj_collision` lists every contact closer than
  the margin (0.01 m) between an arm geom and a rover part. One contact
  makes the point forbidden. Nothing is simulated: no dynamics, no gravity.
  The margin is MuJoCo's geom margin set on the rover parts only (MuJoCo adds
  the margins of the two geoms of a pair). The collider is libccd's: MuJoCo's
  own one reports no contact for a geom held whole inside a mesh's hull. A
  pose where MuJoCo's collision code fails is forbidden and counted
  (`collision_errors`).
* **Before writing**, 64 grid points are done again with a full `mj_forward`
  and must agree, and 64 free grid points are done again with the real
  fingers closed, half open, open and at a random opening: none may touch.
  The sweep runs on one process per core (`--jobs`).

The JSON holds the joint order, each joint's grid (radians), the model's
ranges, the occupancy (one bit per pose, 1 = forbidden, row-major with the
last joint fastest, packed 8 to a byte and written in hex), `cell_part` (one
number per pose: 0 = free, k = the k-th name of `part_names`, the part with
the most contacts there), the poses per rover part and per pair of arm geom
and rover part, the stand-ins per mesh, the root link's settings, the mount
parts, the lidar and its overlaps, the model's file name with two hashes
(the file, and the compiled geometry with every mesh), the margin, the run
time, and the `joint_map` (below). The numbers of the sweep (forbidden
share, parts hit) live there. It stays near 1 MB at the default grid.

### The knobs

| Option | Default | What it changes |
|---|---|---|
| `--model` | (required) | the MJCF to sweep |
| `--out` | `config/arm_keepout.json` | where the table goes |
| `--steps` | `16,16,16,12,8` | grid points per joint: more is finer and slower (the time grows with the product) |
| `--margin` | `0.01` | metres: an arm link closer than this to a rover part counts as a contact |
| `--pad-deg` | `10` | degrees added past each end of every joint's model range |
| `--rover-parts` | `hull,bin_*,track_*,tread_*,lidar` | geom names (patterns) that count as they are, meshes included |
| `--exclude` | none | geom names (patterns) that are not rover parts |
| `--stand-in-voxel` | `0.004` | metres: the box grid of the rover meshes' stand-ins (0 leaves those meshes out) |
| `--cover-tol` | `0.002` | metres: mesh surface this close to a named part needs no stand-in |
| `--root-voxel` | `0.001` | metres: the box grid of the root link's stand-ins |
| `--root-margin` | `0` | metres: the root link's contacts count closer than this (at most `--margin`) |
| `--root-step-deg` | `1` | degrees between the root link's samples along the pan |
| `--lidar-xyz` / `--no-lidar` | the profile's pose | where the lidar cylinder goes, or none |
| `--arm-root`, `--rover-body`, `--joints` | `shoulder_link`, `rover`, the five joints | for a model with other names |
| `--fix` | the fingers swept | `name=value,...` pins the finger joints |
| `--check` | `64` | grid points done again with `mj_forward`, and free ones with the real fingers |
| `--jobs` | one per core, up to 16 | processes for the sweep |

### What the guard refuses

```python
from moss_dimos.arm_guard import ArmGuard
guard = ArmGuard.load()                       # the table + config/arm_calibration.json
guard.allowed_ticks(pose["ticks"])            # servo positions; the gripper is ignored
guard.allowed(q)                              # q = {joint: model radians}
guard.why(q)                                  # "forbidden: the arm touches bin_x1 ..."
guard.nearest_allowed(q)                      # the nearest pose that stays allowed in ticks
guard.nearest_allowed_ticks(pose["ticks"])    # the same, read and returned in ticks
guard.segment_allowed(q_from, q_to)           # a straight move, through every cell it crosses
```

* A pose any of whose surrounding grid points is forbidden. Between grid
  points the table knows nothing, so a pose is allowed only when every grid
  point its interpolation touches is free: along each joint the grid value
  below it and the one above (the value itself when the pose sits on it).
  A pose on the free grid point next to a forbidden one passes; a pose
  halfway between the two does not.
* A pose outside the table on any joint.
* A pose missing a joint is an error, not a verdict.

`why` names the rover part (a mesh's stand-ins answer to the mesh's name).
`nearest_allowed` gives the nearest pose, in grid steps, among those that
stay allowed when rounded to servo ticks: the centre of a cell whose corners
are all free, or a grid point whose neighbours are all free. A bare free
grid point is not enough, since a tick of rounding moves it off the grid
and the rule then asks its neighbours too. The way there is not checked;
`segment_allowed` checks a move: it cuts the move where it crosses a grid
value on any joint and applies the rule in every piece, so a move that only
clips the corner of a forbidden cell is refused. The guard needs numpy only:
no MuJoCo on the robot.

What the table does not see: the arm hitting itself or its own base, the
ground, anything the model lacks (cables, a cover, a load in the gripper),
and, for every link but the root, an obstacle thinner than the arm's travel
over one grid step that sits between two free grid points: at the default
grid, one step of the shoulder moves the gripper up to 12 cm, and the bin's
walls are 6 mm thick. A finer `--steps` narrows that gap; the margin covers
part of it. What it sees larger than it is: a link's convex hull and a
stand-in's reach past its mesh.

### Ticks and the model's angles

The table is in the model's radians; the servos speak ticks. Per joint:

    rad = sign * (ticks - model_zero_ticks) * 2 pi / 4096

`model_zero_ticks` and `sign` live in the table's `"joint_map"`. The default,
2048 and +1, takes the middle of each calibrated travel as the model's zero
and both turning the same way; the two-pose alignment below replaces it with
the arm's own numbers. The guard's verdicts on servo readings hold for the
map the table carries.

Align it with two poses whose model angles are known. Open the model in
MuJoCo's viewer (`python -m mujoco.viewer --mjcf=<robot.xml>`), pause it,
set the joints in its Joint panel, and place the real arm (torque off) the
same way:

| Pose | pan | lift | elbow | flex | roll | What it looks like |
|---|---|---|---|---|---|---|
| A | 0 | 0 | 0 | 0 | 0 | upper arm up, forearm level, the gripper in line with it, the arm facing 75 degrees left of the nose |
| B | +1.571 | +1.571 | -1.571 | +1.571 | +1.571 | the arm stretched out level over the nose (15 degrees right of it), the gripper pointing down, turned a quarter turn |

Positive angles, in the model: pan turns the arm clockwise seen from above,
lift tips the upper arm toward the way the arm faces, elbow and flex fold
the next link down, and roll turns the gripper counter-clockwise seen from
behind the wrist. With the gripper level, roll +0.785 opens the jaws one
above the other and -0.785 side by side.

Read the five positions in both poses (`tools/moss_arm_calibrate.py
--show`), then:

```python
from moss_dimos.arm_guard import align_joint_map
joint_map = align_joint_map(ticks_a, ticks_b)   # {joint: ticks} read in pose A and in pose B
```

`align_joint_map` takes each joint's angle from `POSE_A` and `POSE_B` (the
table above, the elbow negative in B), so no sign is typed by hand.
`align_from_two_poses(ticks_a, rad_a, ticks_b, rad_b)` does one joint. Both
refuse two poses less than 0.2 rad apart, and a slope more than 15 % off 652
ticks per radian (a wrong pose or a wrong joint). Write the result into
`"joint_map"` of `config/arm_keepout.json`; a new sweep keeps it. Then check
`guard.why_ticks(...)` on `drive` and `look`: the arm in each pose should
look like the model in the viewer.

### When a new model arrives

1. Export the robot from the CAD: one mesh per part (STL, metres), the arm's
   links with their joint frames.
2. Build the MJCF: a `rover` body with the collision geoms named `hull`,
   `bin_*`, `track_*`, `tread_*` (boxes are fine and keep the bin's opening
   open), a geom named `lidar` where the lidar sits, the arm from
   `shoulder_link` down with the five joints and their ranges, and the
   meshes as geoms. Every other mesh on the rover counts through its
   stand-ins. Load it once in the viewer and look.
3. Sweep: `python tools/moss_arm_keepout.py --model <robot.xml>`. Read what it
   prints: the parts the lidar goes into, the mount parts, the parts hit.
4. Bench: `python benches/bench_arm_guard_cold.py` (with
   `MOSS_ARM_MODEL=<robot.xml>` for its real-model part).
5. Commit `config/arm_keepout.json`. Its `model.sha256` and
   `model.geometry_sha256` say which model it came from (the second one
   changes with any mesh); its `joint_map` is the one it replaced.
