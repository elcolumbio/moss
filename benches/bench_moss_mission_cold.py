"""Cold bench for moss_dimos.mission: the litter mission on a fake robot,
no hardware, no dimOS stack.

Known input -> known output in metres, degrees and seconds. Groups:

  A. geometry   - a world point in base_link for a robot facing +x, +y and
                  -x; the standoff goal 0.6 m before an object, facing it;
                  the angle wrap
  B. servo      - the servo on a fake unicycle (v, w integrated at 10 Hz)
                  brings an object 1 m ahead and 0.3 m left onto the pick
                  point within 2 cm in under 60 s, turning first, then
                  creeping; an object behind the robot too; the twists
                  never exceed the ceilings
  C. the mission - the state machine on the fake robot, tick by tick:
                  nothing while no litter is seen; a confirmed "Plastic"
                  2 m away stops the explorer, then (3 s later) sends the
                  standoff goal; goal_reached True starts the servo; the
                  servo ends at the pick point and the pick command fires;
                  pick_done True resumes exploring and the same object is
                  never targeted again; a "Chair" is ignored; goal_reached
                  False skips the object (explore resumes) and it stays
                  skipped; a servo timeout skips too; an object already
                  within reach goes straight from STOPPING to SERVOING
  D. the module - under dimOS: the module instantiates, its status RPC
                  answers, a goal command becomes a PoseStamped in the world
                  frame with the right yaw, a twist command a Twist, objects
                  and odometry messages reach the logic, the "none" pick
                  command marks the object picked

Run (A-C need only the repository; D the dimOS of the robot or main):
    nice -n 10 <dimos venv>/bin/python benches/bench_moss_mission_cold.py
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

if os.nice(0) < 10:
    os.nice(10 - os.nice(0))

from moss_dimos import mission as M  # noqa: E402

FAIL: list[str] = []
OK = 0
DEG = 180.0 / math.pi


def check(cond: bool, what: str) -> None:
    global OK
    if cond:
        OK += 1
        print(f"  ok   {what}")
    else:
        FAIL.append(what)
        print(f"  FAIL {what}")


class FakeRobot:
    """A unicycle: v along its heading, w about z, integrated per tick."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0) -> None:
        self.x, self.y, self.yaw = x, y, yaw

    def step(self, v: float, w: float, dt: float) -> None:
        self.yaw = M.wrap(self.yaw + w * dt)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


PICK = (0.209, -0.098)

# ── A. geometry ──
print("A. geometry")
bx, by = M.in_base(1.0, 0.0, 0.0, 0.0, 0.0)
check(abs(bx - 1.0) < 1e-9 and abs(by) < 1e-9, "object 1 m ahead, robot facing +x: (1, 0) in base_link")
bx, by = M.in_base(0.0, 1.0, 0.0, 0.0, math.pi / 2)
check(abs(bx - 1.0) < 1e-9 and abs(by) < 1e-9, "object at +y, robot facing +y: (1, 0)")
bx, by = M.in_base(0.0, 1.0, 0.0, 0.0, 0.0)
check(abs(bx) < 1e-9 and abs(by - 1.0) < 1e-9, "object at +y, robot facing +x: (0, 1) = on the left")
bx, by = M.in_base(1.0, 0.0, 2.0, 0.0, math.pi)
check(abs(bx - 1.0) < 1e-9 and abs(by) < 1e-9, "robot at x 2 facing -x, object at x 1: 1 m ahead")
gx, gy, gyaw = M.standoff_goal(2.0, 0.0, 0.0, 0.0, 0.6)
check(abs(gx - 1.4) < 1e-9 and abs(gy) < 1e-9 and abs(gyaw) < 1e-9, "standoff goal (1.4, 0, 0 deg) for an object at (2, 0)")
gx, gy, gyaw = M.standoff_goal(0.0, 2.0, 0.0, 0.0, 0.6)
check(abs(gx) < 1e-9 and abs(gy - 1.4) < 1e-9 and abs(gyaw - math.pi / 2) < 1e-9, "standoff goal (0, 1.4, 90 deg) for an object at (0, 2)")
check(abs(M.wrap(3 * math.pi) - math.pi) < 1e-9 or abs(M.wrap(3 * math.pi) + math.pi) < 1e-9, "wrap(3 pi) = +-pi")
check(abs(M.wrap(math.radians(-370)) - math.radians(-10)) < 1e-9, "wrap(-370 deg) = -10 deg")

# ── B. servo ──
print("B. servo")


def run_servo(obj, start, limit_s=60.0):
    clk = Clock()
    lg = M.MissionLogic(pick=PICK, clock=clk)
    lg.target = M.Target(1, "Plastic", *obj)
    lg._enter(M.SERVOING, clk.t)
    rob = FakeRobot(*start)
    lg.on_odom(rob.x, rob.y, rob.yaw)
    vmax = wmax = 0.0
    t = 0.0
    turned_first = None
    while t < limit_s:
        cmds = lg.tick(clk.t)
        tw = [c for c in cmds if c[0] == "twist"]
        if any(c[0] == "pick" for c in cmds):
            break
        if tw:
            _, v, w = tw[-1]
            vmax, wmax = max(vmax, abs(v)), max(wmax, abs(w))
            if turned_first is None and (abs(v) > 0 or abs(w) > 0):
                turned_first = abs(v) == 0.0 and abs(w) > 0.0
            rob.step(v, w, 0.1)
        clk.t += 0.1
        t += 0.1
        lg.on_odom(rob.x, rob.y, rob.yaw)
    bx, by = M.in_base(obj[0], obj[1], rob.x, rob.y, rob.yaw)
    err = math.hypot(bx - PICK[0], by - PICK[1])
    return lg, t, err, vmax, wmax, turned_first, (bx, by)


lg, t, err, vmax, wmax, turned_first, b = run_servo((1.0, 0.3), (0.0, 0.0, 0.0))
check(lg.state == M.PICKING, f"object 1 m ahead, 0.3 m left: PICKING after {t:.1f} s")
check(err <= 0.02, f"object in base_link ({b[0]:.3f}, {b[1]:.3f}) = the pick point within {err * 100:.1f} cm")
check(turned_first is True, "it turned first (v = 0 while aligning)")
check(vmax <= M.SERVO_V_MAX + 1e-9 and wmax <= M.SERVO_W_MAX + 1e-9, f"twists within the ceilings (v {vmax:.2f} m/s, w {wmax:.2f} rad/s)")
lg, t, err, *_ = run_servo((-0.8, 0.4), (0.0, 0.0, 0.0))
check(lg.state == M.PICKING and err <= 0.02, f"object behind the robot: PICKING after {t:.1f} s, {err * 100:.1f} cm")
lg, t, err, *_ = run_servo((0.0, 1.5), (0.0, 0.0, 0.0), limit_s=80)
check(lg.state == M.PICKING and err <= 0.02, f"object 1.5 m to the left: PICKING after {t:.1f} s, {err * 100:.1f} cm")

# ── C. the mission ──
print("C. the mission")
clk = Clock()
lg = M.MissionLogic(pick=PICK, clock=clk)
rob = FakeRobot()
lg.on_odom(rob.x, rob.y, rob.yaw)
check(lg.tick(clk.t) == [] and lg.state == M.EXPLORING, "nothing seen: EXPLORING, no command")
lg.on_objects([M.Seen(7, "Chair", 1.0, 0.0, 9)])
check(lg.tick(clk.t) == [] and lg.state == M.EXPLORING, "a Chair is not litter")
lg.on_objects([M.Seen(7, "Chair", 1.0, 0.0, 9), M.Seen(3, "Plastic", 2.0, 0.5, 6)])
cmds = lg.tick(clk.t)
check(cmds and cmds[0] == ("stop_explore",) and lg.state == M.STOPPING and lg.target.track_id == 3,
      "a confirmed Plastic 2 m away: stop_explore, STOPPING, target #3")
clk.t += 1.0
check(lg.tick(clk.t) == [] and lg.state == M.STOPPING, "1 s later: still settling")
clk.t += 2.5
cmds = lg.tick(clk.t)
goal = [c for c in cmds if c[0] == "goal"]
gx, gy, gyaw = M.standoff_goal(2.0, 0.5, 0.0, 0.0, M.STANDOFF_DEFAULT)
check(lg.state == M.APPROACHING and goal and abs(goal[0][1] - gx) < 1e-9 and abs(goal[0][2] - gy) < 1e-9 and abs(goal[0][3] - gyaw) < 1e-9,
      f"3.5 s after the stop: goal ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f} deg), APPROACHING")
clk.t += 5.0
check(lg.tick(clk.t) == [] and lg.state == M.APPROACHING, "waiting for the planner")
rob.x, rob.y, rob.yaw = gx, gy, gyaw
lg.on_odom(rob.x, rob.y, rob.yaw)
lg.on_goal_reached(True)
cmds = lg.tick(clk.t)
check(lg.state == M.SERVOING, "goal reached: SERVOING")
picked = False
t = 0.0
while t < 60.0 and not picked:
    cmds = lg.tick(clk.t)
    for c in cmds:
        if c[0] == "twist":
            rob.step(c[1], c[2], 0.1)
        if c[0] == "pick":
            picked = True
    clk.t += 0.1
    t += 0.1
    lg.on_odom(rob.x, rob.y, rob.yaw)
bx, by = M.in_base(2.0, 0.5, rob.x, rob.y, rob.yaw)
check(picked and lg.state == M.PICKING, f"the servo ends at the pick point in {t:.1f} s and the pick command fires")
check(math.hypot(bx - PICK[0], by - PICK[1]) <= 0.02, f"object in base_link ({bx:.3f}, {by:.3f})")
check(cmds and cmds[0] == ("twist", 0.0, 0.0), "a zero twist first")
clk.t += 2.0
check(lg.tick(clk.t) == [] and lg.state == M.PICKING, "PICKING waits for the pick command")
lg.pick_done(True)
cmds = lg.tick(clk.t)
check(("explore",) in cmds and lg.state == M.RESUMING and lg.done == [(2.0, 0.5)], "pick done: explore_cmd, RESUMING, the object remembered")
clk.t += 1.5
lg.tick(clk.t)
check(lg.state == M.EXPLORING, "then EXPLORING")
lg.on_objects([M.Seen(3, "Plastic", 2.02, 0.49, 20), M.Seen(9, "Plastic", 2.05, 0.52, 6)])
check(lg.tick(clk.t) == [] and lg.state == M.EXPLORING, "the picked object, and a new track at the same spot, are not targeted again")
# a refused goal skips the object
lg.on_objects([M.Seen(11, "Metal", rob.x + 4.5, rob.y, 6)])
lg.tick(clk.t)
check(lg.state == M.EXPLORING, "an object 4.5 m away is out of range")
lg.on_objects([M.Seen(11, "Metal", rob.x + 1.5, rob.y, 6)])
lg.tick(clk.t)
clk.t += 3.5
lg.tick(clk.t)
check(lg.state == M.APPROACHING and lg.target.track_id == 11, "a Metal 1.5 m away: APPROACHING #11")
lg.on_goal_reached(False)
cmds = lg.tick(clk.t)
check(lg.state == M.RESUMING and ("explore",) in cmds and 11 in lg.skipped, "the planner gave up: skipped, explore again")
clk.t += 1.5
lg.tick(clk.t)
lg.tick(clk.t)
check(lg.state == M.EXPLORING and lg.target is None, "the skipped object is not targeted again")
# servo timeout
lg.on_objects([M.Seen(12, "Paper", 2.3, 0.0, 6)])
lg.tick(clk.t)
clk.t += 3.5
lg.tick(clk.t)
lg.on_goal_reached(True)
lg.tick(clk.t)
check(lg.state == M.SERVOING and lg.target.track_id == 12, "a Paper: SERVOING #12")
clk.t += M.SERVO_TIMEOUT_S + 0.5
cmds = lg.tick(clk.t)
check(lg.state == M.RESUMING and 12 in lg.skipped and cmds[0] == ("twist", 0.0, 0.0), "the robot never got there (no odometry progress): servo timeout, zero twist, skipped")
clk.t += 1.5
lg.tick(clk.t)
# already within reach
lg.on_objects([M.Seen(13, "Glass", rob.x + 0.5, rob.y, 6)])
lg.on_odom(rob.x, rob.y, 0.0)
lg.tick(clk.t)
clk.t += 3.5
cmds = lg.tick(clk.t)
check(lg.state == M.SERVOING and not any(c[0] == "goal" for c in cmds), "a Glass 0.5 m ahead: no goal, straight to SERVOING")

# ── D. the module ──
print("D. the module")
try:
    cls = M.LitterMission
except Exception as exc:  # noqa: BLE001
    print(f"  (dimOS not importable here: {exc}; D skipped)")
else:
    mod = cls(pick_cmd="none", autostart=False)
    st = mod.mission_status()
    check(st["state"] == M.EXPLORING and st["picked"] == 0, f"status: {st}")
    sent: dict = {}

    class Rec:
        def __init__(self, k):
            self.k = k

        def publish(self, msg):
            sent.setdefault(self.k, []).append(msg)

    for k in ("goal_request", "tele_cmd_vel", "stop_explore_cmd", "explore_cmd", "mission_state"):
        setattr(mod, k, Rec(k))
    mod._do(("goal", 1.0, 2.0, math.pi / 2))
    g = sent["goal_request"][0]
    yaw = g.orientation.to_euler().z
    check(g.frame_id == "world" and abs(g.position.x - 1.0) < 1e-9 and abs(g.position.y - 2.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-6,
          f"a goal command: PoseStamped in world at (1, 2), yaw {math.degrees(yaw):.0f} deg")
    mod._do(("twist", 0.05, -0.2))
    tw = sent["tele_cmd_vel"][0]
    check(abs(tw.linear.x - 0.05) < 1e-9 and abs(tw.angular.z + 0.2) < 1e-9, "a twist command: Twist(0.05, -0.2)")
    mod._do(("stop_explore",))
    mod._do(("explore",))
    check(sent["stop_explore_cmd"][0].data is True and sent["explore_cmd"][0].data is True, "stop_explore and explore: Bool(True)")
    mod._do(("state", "SERVOING: test"))
    check(sent["mission_state"][0].data == "SERVOING: test", "a state line on mission_state")
    objs = [SimpleNamespace(track_id=5, name="Plastic", center=SimpleNamespace(x=1.5, y=0.2, z=0.0), detections_count=8)]
    mod._on_objects(objs)
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    mod._on_odom(PoseStamped(time.time(), "world", position=Vector3(0.0, 0.0, 0.0),
                             orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(30)))))
    lg2 = mod.logic
    check(lg2.objects[0].track_id == 5 and abs(lg2.objects[0].x - 1.5) < 1e-9 and abs(lg2.pose[2] - math.radians(30)) < 1e-6,
          "objects and odometry reach the logic (yaw 30 deg)")
    lg2.state = M.PICKING
    lg2.target = M.Target(5, "Plastic", 1.5, 0.2)
    mod._start_pick()
    check(lg2._pick_result is True, "the 'none' pick command marks the object picked without moving anything")

print(f"\n{OK} OK, {len(FAIL)} FAIL")
for f in FAIL:
    print(f"  - {f}")
sys.exit(1 if FAIL else 0)
