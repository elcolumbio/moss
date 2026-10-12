# The arm's boundary tour

`tools/moss_arm_sweep.py` drives the arm slowly through poses the keep-out
guard allows, each one next to a pose it forbids, and logs every reading of
the five servos. It is made for film: the arm tours its reach for a set
time, and the log drives a picture-in-picture render of the collision-zone
map, frame for frame with the footage.

## Filming

1. Plan with nothing moving:

   ```sh
   python tools/moss_arm_sweep.py --plan-only --duration 25 --log plan.jsonl
   ```

   It prints the tour (every waypoint in model degrees, the time the speed
   model gives, the rover part each one skirts) and writes `plan.jsonl`, the
   log of a run on a fake bus, so the render can be made before the arm
   moves. `--dry-run` runs the whole tool on that fake bus, on the wall
   clock.
2. Place the arm in the drive pose, start the camera, then the tour:

   ```sh
   python tools/moss_arm_sweep.py --duration 25 --log sweep.jsonl
   ```

   The arm leaves the drive pose, follows the edge of the keep-out zones for
   the 25 seconds, and comes back to the drive pose. The same inputs give
   the same tour; `--seed` picks another, `--speed` (200 by default, 17.6
   degrees per second) sets the pace.
3. Render `sweep.jsonl` into the picture-in-picture clip with the render
   script that the NOTES file of the video folder names. Each line holds `t`
   (seconds from the start), the ticks and the model radians of the five
   joints, the waypoint the arm is heading to, and the phase (`tour`, or
   `return` after a fault).

Ctrl-C stops the tour: the arm stops where it is, then goes back to the
drive pose through moves the guard allows.

## What it refuses

* A drive pose the guard forbids: no plan, and why() names the part.
* An arm more than twice the tolerance (50 ticks) from the drive pose at
  the start.
* A speed over 400 (35 degrees per second), a tolerance wider than half a
  grid step, a calibration without a drive pose.
* During the run, a move or a reading the guard refuses, a waypoint not
  reached in time and servos that stop answering are faults: the arm stops
  and goes home through allowed moves, or holds where it is when no allowed
  move leaves that pose.

## What it never does

* It never moves the gripper: servo 6 is never written. The keep-out table
  counts the jaws at every opening.
* It never turns the torque off: at the end the arm holds the drive pose.

## Before the first tour

The tour passes within one or two grid steps of the poses the table forbids,
and the table knows its grid points only ([arm.md](arm.md), "What the table
does not see"). The joint map behind it is the first alignment: check `drive`
and `look` against the model first ([arm.md](arm.md), "The operator's
check"), the pan most of all, whose zero is good to about 15 degrees, one
grid step. Keep a hand on the E-stop.
