#!/usr/bin/env python3
"""Play the pick-and-drop cycle on the arm through the running `moss-arm`
coordinator (simulation or robot), watching the arm follow.

    dimos --simulation mujoco run moss-dimos.moss-arm        # one terminal: the simulator and the coordinator
    tools/moss_arm_play.py --sim                             # another: the cycle, from wherever the arm is
    tools/moss_arm_play.py --sim --stop-after release        # up to the release: the arm waits over the bin
    tools/moss_arm_play.py --sim --dry-run                   # the trajectory's numbers, nothing sent

On the robot the same, without --sim, after `dimos run moss-dimos.moss-arm`
(the Feetech adapter on the arm's port). Nothing here checks the scene:
the cycle is the one tools/moss_arm_drop.py plan --mujoco verified in the CAD
scene, run as delivered. What this tool adds is the watch: every 50 ms it
reads the joints back and compares them with the trajectory; a joint more
than --max-lag ticks behind for longer than --lag-seconds cancels the
trajectory (the coordinator holds the last commanded position) and the run
ends with ARM LAG and the joint's numbers. That is the case of the shoulder
short of its target under load; the number is printed on every run, lag or
not, so the gap between the simulation and the arm is a measurement.

Options:
  --sim                 the gripper in the simulator's unit (see moss_dimos.arm)
  --poses PATH          the cycle (config/arm_drop_poses.json)
  --calib PATH          the calibration (the drive pose); MOSS_ARM_CALIB, else config/arm_calibration.json
  --speed TICKS_PER_S   the ceiling (400)
  --stop-after PHASE    end the cycle after this phase (e.g. release)
  --from PHASE          start the cycle at this phase (e.g. back_to_clear: the return from the bin)
  --pick-pose NAME      the pose played as "pick": pick (the video's, an object standing) or pick_low (lying on
                        the floor); MOSS_ARM_PICK_POSE
  --max-lag TICKS       the following error that counts as a lag (150 = 13 deg)
  --lag-seconds S       how long a lag must last before the stop (1.0)
  --no-goto             refuse to start unless the arm is at the cycle's first pose
  --dry-run             print the steps and the trajectory, send nothing
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos import arm as A  # noqa: E402

POLL_S = 0.05
DEG = 180.0 / math.pi


def say(*a):
    print(*a, flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--poses", default=str(A.POSES_PATH))
    ap.add_argument("--calib", default=None)
    ap.add_argument("--speed", type=float, default=A.SPEED_TICKS_PER_S)
    ap.add_argument("--stop-after", default=None)
    ap.add_argument("--from", dest="start", default=None)
    ap.add_argument("--pick-pose", default=None)
    ap.add_argument("--max-lag", type=float, default=150.0)
    ap.add_argument("--lag-seconds", type=float, default=1.0)
    ap.add_argument("--no-goto", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    cycle = A.load_cycle(a.poses, calibration=a.calib, speed=a.speed, stop_after=a.stop_after, start=a.start,
                         pick_pose=a.pick_pose)
    say(f"cycle: {len(cycle.steps)} moves, {cycle.timed_s:.1f} s as timed, {cycle.duration_s:.1f} s at {a.speed:g} ticks/s, "
        f"pick pose {a.pick_pose or A.pick_pose_name()}")
    for s in cycle.steps:
        say(f"  {s.phase:14s} {s.start:>7s} -> {s.end:<7s} {s.duration_s:5.2f} s  gap {s.gap_from_m * 1000:.0f} -> {s.gap_to_m * 1000:.0f} mm"
            f"{'  (holds the can)' if s.holds_can else ''}")
    if a.dry_run:
        traj = A.cycle_trajectory(cycle.steps, sim=a.sim)
        say(f"trajectory: {len(traj.points)} points, {traj.duration:.2f} s, joints {traj.joint_names}")
        for p in traj.points[:: max(1, len(traj.points) // 12)]:
            say(f"  t {p.time_from_start:6.2f}  " + " ".join(f"{A.rad_to_ticks(v):5d}" for v in p.positions[:5]) + f"  gripper {p.positions[5]:.4f}")
        return 0

    from dimos.control.coordinator import ControlCoordinator
    from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
    from dimos.core.rpc_client import RPCClient

    client = RPCClient(None, ControlCoordinator, remote_name=A.ARM_COORDINATOR_NAME)
    try:
        now = client.get_joint_positions() or {}
        missing = [j for j in A.JOINT_NAMES if j not in now]
        if missing:
            say(f"refused: the coordinator does not report {missing} (is moss-arm running? joints: {sorted(now)})")
            return 2
        q_now = [now[f"{A.HW_ID}/{j}"] for j in A.ARM_JOINTS]
        g_raw = now[f"{A.HW_ID}/{A.GRIPPER}"]
        gap_now = A.sim_gripper_gap(g_raw) if a.sim else float(g_raw)
        say("arm now: " + " ".join(f"{j} {A.rad_to_ticks(v)}" for j, v in zip(A.ARM_JOINTS, q_now)) + f"  gap {gap_now * 1000:.0f} mm")
        steps = list(cycle.steps)
        off = max(abs(x - y) for x, y in zip(q_now, cycle.first_pose)) * A.TICKS_PER_RAD
        if off > A.START_TOLERANCE_RAD * A.TICKS_PER_RAD:
            if a.no_goto:
                say(f"refused: the arm is {off:.0f} ticks from the cycle's first pose ({steps[0].start}) and --no-goto is set")
                return 2
            goto = A.goto_step(q_now, gap_now, cycle.first_pose, cycle.first_gap_m, speed=a.speed)
            say(f"goto: {off:.0f} ticks from {steps[0].start}, {goto.duration_s:.1f} s first")
            steps.insert(0, goto)
        traj = A.cycle_trajectory(steps, sim=a.sim)
        res = client.execute_trajectory(traj)
        if res is None or res.status is not TrajectoryExecutionStatus.ACCEPTED:
            say(f"refused by the coordinator: {getattr(res, 'status', None)} {getattr(res, 'message', '')}")
            return 3
        say(f"running: {len(traj.points)} points, {traj.duration:.1f} s")
        t0 = time.time()
        worst: dict[str, float] = {}
        lag_since: dict[str, float] = {}
        last_line = 0.0
        while True:
            t = time.time() - t0
            cur = client.get_joint_positions() or {}
            err = A.following_error_ticks(traj, t, cur)
            for j, e in err.items():
                worst[j] = max(worst.get(j, 0.0), e)
                if e > a.max_lag:
                    lag_since.setdefault(j, t)
                    if t - lag_since[j] >= a.lag_seconds:
                        client.cancel_trajectory()
                        say(f"ARM LAG: {j} {e:.0f} ticks behind for {t - lag_since[j]:.1f} s at t = {t:.1f} s; "
                            f"trajectory cancelled, the coordinator holds the last commanded position")
                        _report(worst)
                        return 4
                else:
                    lag_since.pop(j, None)
            if t - last_line >= 2.0:
                last_line = t
                say(f"  t {t:5.1f} s  " + " ".join(f"{j.split('/')[-1]} {e:4.0f}" for j, e in err.items()))
            if t >= traj.duration + 0.3:
                break
            time.sleep(POLL_S)
        fin = client.get_joint_positions() or {}
        say("done: " + " ".join(f"{j} {A.rad_to_ticks(fin.get(f'{A.HW_ID}/{j}', float('nan')))}" for j in A.ARM_JOINTS))
        _report(worst)
        return 0
    finally:
        try:
            client.stop_rpc_client()
        except Exception:  # noqa: BLE001
            pass


def _report(worst: dict[str, float]) -> None:
    say("worst following error: " + ", ".join(f"{j.split('/')[-1]} {e:.0f} ticks ({e / A.TICKS_PER_RAD * DEG:.1f} deg)" for j, e in worst.items()))


if __name__ == "__main__":
    sys.exit(main())
