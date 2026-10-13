"""Cold bench for moss_dimos.arm_feetech: the SO-101's servos as a dimOS
adapter, on a fake bus, no robot.

Known input -> known output in physical units. Groups:

  A. units     - rad <-> ticks through the joint map (2048 = 0, +30 deg =
                 2389 ticks, the drive pose's 860 = -104.4 deg), the gripper's
                 line (90 mm = 3674, 53 mm = 2780, 24.2 ticks/mm, clamped)
  B. connect   - the adapter opens the fake bus, reads the six servos, sets
                 speed 400 and acceleration 50 on the five arm servos, the
                 force 400 on the gripper's registers 44 and 48, writes no
                 torque; activate turns torque on and holds the present
                 position; limits = the calibration's end stops in radians
  C. follow    - a goal of +30 deg on the pan writes 2389 to register 42 of
                 servo 1 and the fake servo reaches it at 400 ticks/s: read
                 back +30.0 deg; the gripper to 53 mm writes 2780 and reads
                 53 mm; a repeated goal writes nothing; velocities come from
                 the reads
  D. watch     - a blocked shoulder (the fake servo stops 338 ticks short)
                 raises FOLLOWING_ERROR after lag_s once it stands still,
                 naming the joint and the ticks, not while it is on its way
                 to a far goal; the error clears when the servo arrives; a bus
                 that stops answering leaves the last positions and turns
                 has_live_state False after stale_s
  E. stop      - write_stop sets every goal to the present position;
                 deactivate turns torque off; disconnect leaves a bus it was
                 handed open (the owner closes it)
  F. coordinator - the adapter under dimOS's ControlCoordinator in this
                 process (registered as feetech_so101 with the fake bus
                 through adapter_kwargs): a 2 s pan trajectory reads back
                 +30 deg within 1 deg; the trajectory task's start check
                 refuses a trajectory 10 deg off the arm

Run (the dimOS main venv for F; A-E need only the repo):
    nice -n 10 <dimos venv>/bin/python benches/bench_moss_arm_feetech_cold.py
Env: MOSS_ARM_CALIB (a calibration file elsewhere than config/arm_calibration.json).
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

if os.nice(0) < 10:
    os.nice(10 - os.nice(0))

from moss_dimos import arm as A  # noqa: E402
from moss_dimos import arm_feetech as F  # noqa: E402

FAIL: list[str] = []
OK = 0
DEG = 180.0 / math.pi
CALIB = os.environ.get("MOSS_ARM_CALIB") or str(A.CALIBRATION_PATH)


def check(cond: bool, what: str) -> None:
    global OK
    if cond:
        OK += 1
        print(f"  ok   {what}")
    else:
        FAIL.append(what)
        print(f"  FAIL {what}")


class FakeServo:
    def __init__(self, pos: int) -> None:
        self.pos = float(pos)
        self.goal = int(pos)
        self.speed = 0
        self.acc = 0
        self.torque = 0
        self.regs: dict[int, int] = {}
        self.block: float | None = None   # the servo cannot pass this tick

    def step(self, dt: float) -> None:
        if not self.torque:
            return
        vmax = float(self.speed or 3000)
        e = self.goal - self.pos
        if abs(e) < 0.5:
            self.pos = float(self.goal)
            return
        new = self.pos + math.copysign(min(vmax * dt, abs(e)), e)
        if self.block is not None and (self.pos - self.block) * (new - self.block) <= 0:
            new = self.block
        self.pos = new


class FakeBus:
    """The four calls of FeetechBus on servos that move at their speed, in real time."""

    def __init__(self, start: dict[int, int], dead: bool = False) -> None:
        self.servos = {sid: FakeServo(p) for sid, p in start.items()}
        self.writes: list[tuple[int, int, int]] = []
        self.dead = dead
        self.last = time.monotonic()
        self.opened = False
        self.closed = False

    def _tick(self) -> None:
        now = time.monotonic()
        for s in self.servos.values():
            s.step(now - self.last)
        self.last = now

    def open(self) -> bool:
        self.opened = True
        return True

    def close(self) -> None:
        self.closed = True

    def read2(self, sid: int, addr: int) -> int | None:
        self._tick()
        if self.dead or sid not in self.servos:
            return None
        s = self.servos[sid]
        if addr == F.REG_PRESENT:
            p = int(round(s.pos))
            return (abs(p) | 0x8000) if p < 0 else p
        return s.regs.get(addr, 0)

    def write1(self, sid: int, addr: int, value: int) -> bool:
        self._tick()
        if self.dead or sid not in self.servos:
            return False
        self.writes.append((sid, addr, int(value)))
        s = self.servos[sid]
        if addr == F.REG_TORQUE:
            s.torque = int(value)
        elif addr == F.REG_ACC:
            s.acc = int(value)
        s.regs[addr] = int(value)
        return True

    def write2(self, sid: int, addr: int, value: int) -> bool:
        self._tick()
        if self.dead or sid not in self.servos:
            return False
        self.writes.append((sid, addr, int(value)))
        s = self.servos[sid]
        if addr == F.REG_GOAL:
            s.goal = int(value)
        elif addr == F.REG_SPEED:
            s.speed = int(value)
        s.regs[addr] = int(value)
        return True


DRIVE = {1: 2857, 2: 860, 3: 3206, 4: 868, 5: 3221, 6: 3674}


def make(dead: bool = False, **kw):
    bus = FakeBus(DRIVE, dead=dead)
    ad = F.FeetechSo101Adapter(dof=6, address="/fake", hardware_id="arm", calibration=CALIB, bus=bus, **kw)
    return bus, ad


# ── A. units ──
print("A. units")
bus, ad = make()
check(ad.connect(), "connect on the fake bus")
check(ad.rad_to_ticks("shoulder_pan", math.radians(30.0)) == 2389, f"+30 deg -> {ad.rad_to_ticks('shoulder_pan', math.radians(30.0))} ticks (2389)")
check(abs(ad.ticks_to_rad("shoulder_lift", 860) * DEG + 104.4) < 0.05, f"860 ticks -> {ad.ticks_to_rad('shoulder_lift', 860) * DEG:.1f} deg (-104.4)")
check(ad.gap_to_ticks(0.090) == 3674 and ad.gap_to_ticks(0.053) == 2780, "90 mm -> 3674, 53 mm -> 2780")
check(abs(ad.ticks_to_gap(2780) - 0.053) < 1e-9 and abs(ad.ticks_to_gap(3674) - 0.090) < 1e-9, "and back")
check(ad.gap_to_ticks(0.2) == 3674 and ad.gap_to_ticks(-1) == ad.gap_to_ticks(0.0), "the gap is clamped to 0..90 mm")
check(ad.rad_to_ticks("shoulder_lift", -3.0) == 860 and ad.rad_to_ticks("shoulder_lift", 3.0) == 3236,
      "arm goals are clamped to the calibration's end stops")

# ── B. connect ──
print("B. connect")
w = bus.writes
check(all((sid, F.REG_SPEED, 400) in w and (sid, F.REG_ACC, 50) in w for sid in range(1, 6)), "speed 400 and acceleration 50 on the five arm servos")
check(all((6, r, 400) in w for r in F.GRIPPER_FORCE_REGS), "force 400 on the gripper's registers 44 and 48")
check(not any(addr == F.REG_TORQUE for _, addr, _ in w), "no torque written at connect")
check(ad.present_ticks() == {"shoulder_pan": 2857, "shoulder_lift": 860, "elbow_flex": 3206, "wrist_flex": 868, "wrist_roll": 3221, "gripper": 3674},
      "the six present positions read")
q = ad.read_joint_positions()
check(abs(q[0] * DEG - 71.1) < 0.05 and abs(q[5] - 0.090) < 1e-9, f"positions: pan {q[0] * DEG:.1f} deg, gap {q[5] * 1000:.0f} mm")
check(not ad.read_enabled() and ad.write_joint_positions(q) is False, "no command before activate")
check(ad.activate() and ad.read_enabled() and all(s.torque == 1 for s in bus.servos.values()), "activate: torque on, six servos")
check(ad.goal_ticks() == ad.present_ticks(), "torque on holds the present position")
lim = ad.get_limits()
check(abs(lim.position_lower[1] * DEG + 104.4) < 0.05 and abs(lim.position_upper[0] * DEG - 118.3) < 0.05 and lim.position_upper[5] == 0.090,
      f"limits: lift from {lim.position_lower[1] * DEG:.1f} deg, pan to {lim.position_upper[0] * DEG:.1f} deg, gap to 90 mm")
check(ad.has_live_state(), "live state after a read")

# ── C. follow ──
print("C. follow")
target = list(q)
target[0] = math.radians(30.0)
n0 = len(bus.writes)
check(ad.write_joint_positions(target), "a goal of +30 deg on the pan is written")
check((1, F.REG_GOAL, 2389) in bus.writes[n0:] and len(bus.writes) - n0 == 1, "one write: servo 1, register 42, 2389")
check(ad.write_joint_positions(target) and len(bus.writes) == n0 + 1, "the same goal again writes nothing")
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    ad.read_joint_positions()
    time.sleep(0.02)
q = ad.read_joint_positions()
check(abs(q[0] * DEG - 30.0) < 0.1, f"pan reads back {q[0] * DEG:+.2f} deg (2857 -> 2389 at 400 ticks/s took {abs(2857 - 2389) / 400:.1f} s)")
target[5] = 0.053
ad.write_joint_positions(target)
check((6, F.REG_GOAL, 2780) in bus.writes, "the gripper to 53 mm writes 2780")
time.sleep(0.5)
q = ad.read_joint_positions()
check(abs(q[5] - 0.053) < 0.0005, f"gap reads back {q[5] * 1000:.1f} mm")
target[0] = math.radians(0.0)
ad.write_joint_positions(target)
time.sleep(0.3)
ad.read_joint_positions()
time.sleep(0.05)
ad.read_joint_positions()
v = ad.read_joint_velocities()
check(v[0] < -math.radians(20), f"velocity from the reads: pan {v[0] * DEG:.0f} deg/s while moving (400 ticks/s = -35)")

# ── D. watch ──
print("D. watch")
bus, ad = make(lag_ticks=150, lag_s=0.5, stale_s=0.3)
ad.connect()
ad.activate()
bus.servos[2].block = 860 + 400          # the shoulder stops 338 ticks short of a goal 738 ticks up
target = ad.read_joint_positions()
target[1] = ad.ticks_to_rad("shoulder_lift", 860 + 738)
ad.write_joint_positions(target)
t0 = time.monotonic()
code = 0
while time.monotonic() - t0 < 3.0:
    ad.read_joint_positions()
    code, msg = ad.read_error()
    if code:
        break
    time.sleep(0.02)
el = time.monotonic() - t0
check(code == F.FOLLOWING_ERROR and "shoulder_lift" in msg and "338" in msg,
      f"FOLLOWING_ERROR after {el:.1f} s: {msg!r}")
check(el > 1.0 + 0.5 - 0.2, f"not before the servo stood still (400 ticks at 400 ticks/s = 1.0 s, then lag_s 0.5 s: {el:.1f} s)")
bus.servos[2].block = None
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0 and ad.read_error()[0]:
    ad.read_joint_positions()
    time.sleep(0.02)
check(ad.read_error()[0] == 0, "the error clears when the servo arrives")
bus.dead = True
q_last = ad.read_joint_positions()
time.sleep(0.4)
q_dead = ad.read_joint_positions()
check(q_dead == q_last and not ad.has_live_state(), "a dead bus keeps the last positions and the state goes stale")

# ── E. stop ──
print("E. stop")
bus, ad = make()
ad.connect()
ad.activate()
t = ad.read_joint_positions()
t[0] = math.radians(30.0)
ad.write_joint_positions(t)
time.sleep(0.3)
ad.read_joint_positions()
n0 = len(bus.writes)
check(ad.write_stop(), "write_stop")
goals = {sid: val for sid, addr, val in bus.writes[n0:] if addr == F.REG_GOAL}
present = ad.present_ticks()
check(goals[1] == present["shoulder_pan"] and len(goals) == 6, f"every goal set to the present position (pan {goals[1]}, mid-move)")
check(ad.deactivate() and all(s.torque == 0 for s in bus.servos.values()) and not ad.read_enabled(), "deactivate: torque off")
ad.disconnect()
check(not bus.closed and not ad.is_connected(), "disconnect leaves the handed-in bus open, the adapter disconnected")

# ── F. coordinator ──
print("F. coordinator")
try:
    from dimos.control.coordinator import ControlCoordinator
    from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
    from dimos.hardware.manipulators.registry import adapter_registry
except Exception as exc:  # noqa: BLE001
    print(f"  (dimOS not importable here: {exc}; F skipped)")
else:
    try:
        adapter_registry.register_path(A.REAL_ADAPTER, A.REAL_ADAPTER_PATH)
    except ValueError:
        pass
    bus = FakeBus(DRIVE)
    hw = A.arm_hardware(sim=False, address="/fake", calibration=CALIB, bus=bus)
    coord = ControlCoordinator(publish_joint_state=False, tick_rate=50.0, hardware=[hw], tasks=[A.arm_trajectory_task()])
    try:
        coord.start()
        time.sleep(0.3)
        pos = coord.get_joint_positions()
        check(abs(pos["arm/shoulder_pan"] * DEG - 71.1) < 0.1 and abs(pos["arm/gripper"] - 0.090) < 1e-9,
              f"the coordinator reads the arm through the adapter: pan {pos['arm/shoulder_pan'] * DEG:.1f} deg, gap 90 mm")
        check(all(s.torque == 1 for s in bus.servos.values()), "auto_enable: torque on at start")
        drive = A.drive_pose_rad(CALIB)
        q_pan = list(drive)
        q_pan[0] = drive[0] + math.radians(30.0)
        steps = [A.Step("line", "a", "b", list(drive), q_pan, 0.090, 0.090, 2.0, 2.0)]
        traj = A.cycle_trajectory(steps, sim=False)
        res = coord.execute_trajectory(traj)
        check(res.status is TrajectoryExecutionStatus.ACCEPTED, f"trajectory accepted ({res.status.name})")
        time.sleep(2.5)
        pos = coord.get_joint_positions()
        got = (pos["arm/shoulder_pan"] - drive[0]) * DEG
        check(abs(got - 30.0) < 1.0, f"pan read back +{got:.2f} deg after a 2 s trajectory (goal writes on servo 1: {sum(1 for s, a, _ in bus.writes if s == 1 and a == F.REG_GOAL)})")
        off = list(drive)
        off[0] = drive[0] + math.radians(10.0)
        bad = A.cycle_trajectory([A.Step("line", "a", "b", off, drive, 0.090, 0.090, 1.0, 1.0)], sim=False)
        res = coord.execute_trajectory(bad)
        check(res.status is TrajectoryExecutionStatus.START_STATE_MISMATCH, f"a trajectory 10 deg off the arm is refused ({res.status.name})")
    finally:
        coord.stop()
    check(all(s.torque == 0 for s in bus.servos.values()), "stop: torque off")

print(f"\n{OK} OK, {len(FAIL)} FAIL")
for f in FAIL:
    print(f"  - {f}")
sys.exit(1 if FAIL else 0)
