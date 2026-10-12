"""Cold bench for RecoveringGlobalPlanner and RecoveringPlanner's inputs.

Under test: stuck = goal abandoned, NO recovery motion of any kind (a blind
back-off on every stuck verdict walks the robot into whatever is behind it).
A stall of the base (the `stall` stream) is inferred, not felt: the same,
goal abandoned and no motion, and a stall during an escape stops it dead.
The only scripted move left is the contact escape: `bump` (in front: the
IMU, the explorer's back-off) backs off 0.20 m in reverse, `bump_rear`
(behind: the IMU while reversing) moves 0.20 m FORWARD, and a second contact
aborts a running escape dead instead of being dropped.

No robot. A fake local planner integrates the escape twists it receives into
the odometry the planner reads. Needs dimOS and reactivex.

Run from the repository root:  python benches/bench_recovering_planner_cold.py
"""

import asyncio
import sys
import threading
import time
import types
from pathlib import Path as _FsPath

sys.path.insert(0, str(_FsPath(__file__).resolve().parent.parent))

from reactivex import Subject  # noqa: E402

from dimos.core.global_config import GlobalConfig  # noqa: E402
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped  # noqa: E402
from dimos.msgs.geometry_msgs.Twist import Twist  # noqa: E402
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # noqa: E402
from dimos.msgs.nav_msgs.Path import Path  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402
import moss_dimos.recovering_planner as rp_mod  # noqa: E402
from moss_dimos.recovering_planner import RecoveringGlobalPlanner, RecoveringPlanner  # noqa: E402

FOLLOWER_SPEED_MPS = 0.30   # what the fake follower puts on cmd_vel once started


class FakeLocalPlanner:
    def __init__(self) -> None:
        self.cmd_vel: Subject = Subject()
        self.stopped = 0
        self.starts = 0
        self.events: list[str] = []
        self.sent: list[Twist] = []
        self._lock = threading.Lock()
        self._state = "path_following"

    def stop_planning(self) -> None:
        self.stopped += 1
        self.events.append("stop")

    def start_planning(self, path) -> None:
        # the real follower drives cmd_vel from here on; one forward twist is
        # enough to show up as a second writer on the escape's Subject
        self.starts += 1
        self.events.append("start")
        self.cmd_vel.on_next(Twist(linear=Vector3(FOLLOWER_SPEED_MPS, 0.0, 0.0)))

    def get_state(self):
        from dimos.navigation.base import NavigationState
        return NavigationState.IDLE


def make_planner(moves: bool):
    planner = RecoveringGlobalPlanner(GlobalConfig())
    fake = FakeLocalPlanner()
    planner._local_planner = fake
    planner._current_odom = PoseStamped(position=Vector3(2.0, 1.0, 0.0))
    last = [time.perf_counter()]

    def integrate(tw: Twist) -> None:
        fake.sent.append(tw)
        now = time.perf_counter()
        dt, last[0] = now - last[0], now
        if moves and tw.linear.x != 0.0:
            with planner._lock:
                p = planner._current_odom.position
                planner._current_odom = PoseStamped(
                    position=Vector3(p.x + tw.linear.x * dt, p.y, 0.0)
                )

    fake.cmd_vel.subscribe(integrate)
    return planner, fake


def straight_path(start: Vector3, goal: Vector3) -> Path:
    return Path(poses=[
        PoseStamped(position=Vector3(start.x + (goal.x - start.x) * f,
                                     start.y + (goal.y - start.y) * f, 0.0))
        for f in (0.0, 0.5, 1.0)
    ])


def plannable(planner) -> None:
    """No costmap on a cold bench: every goal gets a straight path."""
    planner._find_safe_goal = lambda goal: goal
    planner._find_wide_path = lambda goal, robot: straight_path(robot, goal)


def wait_until(predicate, timeout: float = 4.0) -> bool:
    t0 = time.perf_counter()
    while not predicate() and time.perf_counter() - t0 < timeout:
        time.sleep(0.02)
    return predicate()


def wait_escape_done(planner, timeout: float = 8.0) -> None:
    t0 = time.perf_counter()
    while planner._escaping and time.perf_counter() - t0 < timeout:
        time.sleep(0.02)
    assert not planner._escaping, "escape never finished"


def test_stuck_abandons_goal_without_any_motion() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0   # has been driving a path
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    planner._replan_path()
    assert fake.sent == [], "stuck must not command ANY motion"
    assert planner._current_goal is None, "the goal must be abandoned"
    assert failed == [False], "goal_reached=False is what makes the explorer exclude the spot"
    print("  stuck -> zero twists, goal abandoned, goal_reached=False published")


def test_stuck_needs_a_driven_path_and_ignores_rotation() -> None:
    calls = []
    planner, fake = make_planner(moves=True)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    base = RecoveringGlobalPlanner.__mro__[1]
    base_replan = base._replan_path
    base._replan_path = lambda self: calls.append("replan")  # type: ignore[method-assign]
    try:
        planner._path_started_at = float("inf")             # fresh goal, no path driven yet
        planner._replan_path()
    finally:
        base._replan_path = base_replan  # type: ignore[method-assign]
    assert calls == ["replan"] and planner._current_goal is not None, calls
    print("  stuck but not driving a path -> normal replan, goal kept")

    calls.clear()
    planner._path_started_at = time.perf_counter() - 30.0
    fake._state = "initial_rotation"                         # turning in place: position still, not stuck
    resets = []
    planner._position_tracker.reset_data = lambda: resets.append(1)  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == [] and planner._current_goal is not None, calls
    assert resets, "rotation must RESTART the stuck tracker window (else a 1 Hz replan wiggle)"
    planner._in_stop_message = True                          # but a follower stop message is handled
    planner._replan_path()
    planner._in_stop_message = False
    assert planner._current_goal is None, "stuck while driving (stop message path) -> abandoned"
    fake._state = "path_following"
    print("  rotating in place -> ignored; follower stop message -> handled")


def test_escape_moves_away_then_abandons() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    t0 = time.perf_counter()
    assert planner.escape(direction=-1.0, trigger="bump") is True
    wait_escape_done(planner)
    dt = time.perf_counter() - t0
    assert fake.stopped >= 1, "local follower must be stopped before the escape"
    assert all(tw.linear.x <= 0.0 for tw in fake.sent), "a bump escapes in REVERSE"
    assert fake.sent[-1].linear.x == 0.0, "must end with a zero twist"
    assert planner._current_odom.position.x < 1.85, "must have moved ~0.20 m away"
    assert 1.5 < dt < 4.0, dt
    assert planner._current_goal is None and failed == [False], "escape ends by abandoning the goal"
    print(f"  bump -> escaped {2.0 - planner._current_odom.position.x:.2f} m in {dt:.1f} s, goal abandoned")


def test_escape_times_out_when_not_moving() -> None:
    planner, fake = make_planner(moves=False)
    planner.escape_timeout_s = 1.0
    assert planner.escape(direction=+1.0, trigger="bump_rear") is True
    wait_escape_done(planner)
    assert all(tw.linear.x >= 0.0 for tw in fake.sent), "a rear bump escapes FORWARD"
    assert fake.sent[-1].linear.x == 0.0
    print("  blocked escape times out and ends with a zero twist")


def test_second_contact_aborts_the_escape() -> None:
    planner, fake = make_planner(moves=False)     # never reaches 0.20 m on its own
    planner.escape_timeout_s = 5.0
    t0 = time.perf_counter()
    assert planner.escape(direction=-1.0, trigger="bump") is True
    time.sleep(0.15)
    assert planner.escape(direction=+1.0, trigger="bump_rear") is False, \
        "a second contact must ABORT, not start a new scripted move"
    wait_escape_done(planner)
    dt = time.perf_counter() - t0
    assert dt < 2.0, f"abort must stop the escape early, took {dt:.1f} s"
    assert fake.sent[-1].linear.x == 0.0
    print(f"  second contact aborted the escape after {dt:.2f} s, wheels zeroed")


def test_stuck_is_silent_during_an_escape() -> None:
    planner, fake = make_planner(moves=False)
    planner.escape_timeout_s = 1.0
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    assert planner.escape(direction=-1.0, trigger="bump") is True
    planner._replan_path()          # the monitor fires mid-escape
    assert planner._escaping, "the escape must keep the wheels"
    wait_escape_done(planner)
    print("  monitor stuck check during an escape -> ignored")


def test_goal_during_an_escape_is_held_then_planned() -> None:
    """The explorer wakes 3 s into a 5 s escape and sends a goal. Without the
    hold there are two writers on cmd_vel, then the escape cancels the fresh
    goal and publishes goal_reached=False - the explorer excludes a frontier
    it never drove to. Same timeline here, 1 s escape."""
    planner, fake = make_planner(moves=False)      # wedged: the escape runs its full timeout
    plannable(planner)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))   # the goal being driven
    planner.escape_timeout_s = 1.0

    assert planner.escape(direction=-1.0, trigger="bump") is True
    time.sleep(0.3)                                # mid-escape, as the explorer wakes up
    frontier = PoseStamped(position=Vector3(6.0, 1.0, 0.0))
    planner.handle_goal_request(frontier)

    assert fake.starts == 0, "the follower must NOT be started while the escape owns the wheels"
    assert planner._held_goal is frontier, "the goal is held, not dropped"
    time.sleep(0.2)                                # let the escape keep writing
    assert [tw for tw in fake.sent if tw.linear.x > 0.0] == [], \
        "no forward follower twist may interleave with the reverse escape"

    wait_escape_done(planner)
    assert wait_until(lambda: fake.starts == 1), "the held goal must be planned once the escape ends"
    assert planner._current_goal is frontier, "the held goal becomes the current goal"
    assert planner._held_goal is None
    assert failed == [], "no goal_reached=False for a goal the rover never drove"
    assert fake.events.count("start") == 1 and fake.events[-1] == "start"
    print("  goal mid-escape -> zero follower twists during the escape, planned after, no false failure")


def test_goal_outside_an_escape_is_planned_at_once() -> None:
    planner, fake = make_planner(moves=False)
    plannable(planner)
    goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner.handle_goal_request(goal)
    assert fake.starts == 1 and planner._current_goal is goal, "unchanged outside an escape"
    assert getattr(planner, "_held_goal", None) is None   # getattr: this case also runs pre-fix
    assert [round(tw.linear.x, 2) for tw in fake.sent] == [FOLLOWER_SPEED_MPS], \
        "the follower is alone on cmd_vel"
    print("  goal outside an escape -> planned immediately, follower alone on cmd_vel")


def test_a_plan_racing_the_escape_start_never_leaves_the_follower_running() -> None:
    planner, fake = make_planner(moves=False)
    plannable(planner)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))

    planner._escaping = True                        # the escape took the wheels first
    planner._plan_path()
    assert fake.starts == 0, "a replan during an escape must not start the follower"
    assert planner._held_goal is planner._current_goal

    planner._escaping = False                       # ... and the other way round:
    planner._held_goal = None
    def contact_while_astar_runs(goal, robot):      # a contact arrives while A* runs
        planner._escaping = True
        return straight_path(robot, goal)
    planner._find_wide_path = contact_while_astar_runs
    planner._plan_path()
    assert fake.events[-1] == "stop", "a follower started under a contact must be stopped again"
    assert planner._held_goal is planner._current_goal
    planner._escaping = False
    print("  plan racing an escape -> follower never left running, goal held")


class LogSpy:
    """Stands in for the planner module's logger: (level, event, fields)."""

    def __init__(self):
        self.lines = []

    def _at(level):
        def record(self, event, *args, **kwargs):
            self.lines.append((level, str(event), kwargs))
        return record

    info, warning, error, debug, exception = _at("info"), _at("warning"), _at("error"), _at("debug"), _at("exception")
    del _at

    def events(self, prefix):
        return [(event, fields) for _, event, fields in self.lines if event.startswith(prefix)]


def with_log(fn):
    spy, real = LogSpy(), rp_mod.logger
    rp_mod.logger = spy
    try:
        return fn(spy)
    finally:
        rp_mod.logger = real


def module_on(planner):
    """The RecoveringPlanner handlers on this planner, without the dimOS module
    machinery: the handlers only use self._planner."""
    return types.SimpleNamespace(_planner=planner)


def test_the_module_has_the_three_inputs() -> None:
    ann = RecoveringPlanner.__annotations__
    assert {"bump", "bump_rear", "stall"} <= set(ann), sorted(ann)
    for name in ("bump", "bump_rear", "stall"):
        assert getattr(ann[name], "__args__", (None,))[0] is Bool, (name, ann[name])
    assert all(callable(getattr(RecoveringPlanner, f"handle_{n}", None)) for n in ("bump", "bump_rear", "stall"))
    print("  RecoveringPlanner: In bump, In bump_rear, In stall (dimos_lcm Bool), one handler each")


def test_front_contact_backs_off_in_reverse() -> None:
    planner, fake = make_planner(moves=True)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))

    def go(spy):
        asyncio.run(RecoveringPlanner.handle_bump(module_on(planner), Bool(data=True)))
        wait_escape_done(planner)
        return spy.events("BUMP received")
    got = with_log(go)
    assert got == [("BUMP received: front contact", {"reflex": "escape reverse 0.20 m", "acted": True})], got
    assert fake.sent and all(tw.linear.x <= 0.0 for tw in fake.sent), "a front contact escapes in REVERSE"
    moved = 2.0 - planner._current_odom.position.x
    assert 0.19 <= moved <= 0.30, moved
    assert planner._current_goal is None
    print(f"  bump -> 'BUMP received: front contact', {moved:.2f} m in reverse, goal abandoned")


def test_rear_contact_escapes_forward() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))

    def go(spy):
        asyncio.run(RecoveringPlanner.handle_bump_rear(module_on(planner), Bool(data=True)))
        wait_escape_done(planner)
        return spy.events("BUMP received"), spy.events("Contact escape")
    bumps, escapes = with_log(go)
    assert bumps == [("BUMP received: rear contact", {"reflex": "escape forward 0.20 m", "acted": True})], bumps
    assert fake.sent and all(tw.linear.x >= 0.0 for tw in fake.sent), "a rear contact escapes FORWARD, never in reverse"
    moved = planner._current_odom.position.x - 2.0
    assert 0.19 <= moved <= 0.30, moved
    assert len(escapes) == 1 and escapes[0][1]["trigger"] == "bump_rear" and escapes[0][1]["direction"] == "forward", escapes
    assert planner._current_goal is None and failed == [False], "a rear contact abandons the goal too"
    print(f"  bump_rear -> 'BUMP received: rear contact', {moved:.2f} m FORWARD, goal abandoned")


def test_a_false_message_does_nothing() -> None:
    planner, fake = make_planner(moves=True)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    for handler in (RecoveringPlanner.handle_bump, RecoveringPlanner.handle_bump_rear, RecoveringPlanner.handle_stall):
        asyncio.run(handler(module_on(planner), Bool(data=False)))
    assert fake.sent == [] and not planner._escaping and planner._current_goal is not None
    print("  Bool(False) on bump / bump_rear / stall -> nothing")


def test_stall_abandons_the_goal_without_any_motion() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0

    def go(spy):
        asyncio.run(RecoveringPlanner.handle_stall(module_on(planner), Bool(data=True)))
        time.sleep(0.3)                          # an escape would have written twists by now
        return spy.events("Goal abandoned"), spy.events("STALL received")
    abandoned, stalls = with_log(go)
    assert fake.sent == [] and not planner._escaping, "a stall must not command ANY motion"
    assert planner._current_goal is None and failed == [False], "the goal is abandoned (the explorer excludes the spot)"
    assert abandoned == [("Goal abandoned, no recovery motion", {"trigger": "stall"})], abandoned
    assert stalls == [("STALL received", {"reflex": "stop, goal abandoned, no motion", "acted": True})], stalls
    assert planner._path_started_at == float("inf")
    print("  stall -> zero twists, 'Goal abandoned, no recovery motion' (trigger stall), goal_reached=False")


def test_stall_without_a_goal_gives_nothing_up() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))

    def go(spy):
        acted = planner.stall(trigger="stall")
        return acted, spy.events("Goal abandoned")
    acted, abandoned = with_log(go)
    assert acted is False and abandoned == [] and failed == [] and fake.sent == []
    print("  stall with no goal (the pad was driving) -> nothing cancelled, no line")


def test_stall_during_an_escape_stops_it_dead_and_drops_the_held_goal() -> None:
    planner, fake = make_planner(moves=False)       # wedged: the escape would run its full timeout
    plannable(planner)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner.escape_timeout_s = 5.0
    assert planner.escape(direction=-1.0, trigger="bump") is True
    time.sleep(0.2)
    planner.handle_goal_request(PoseStamped(position=Vector3(6.0, 1.0, 0.0)))   # held by the escape
    assert planner._held_goal is not None
    t0 = time.perf_counter()

    def go(spy):      # through the module's handler, the path the coordinator's `stall` takes
        asyncio.run(RecoveringPlanner.handle_stall(module_on(planner), Bool(data=True)))
        wait_escape_done(planner)
        return spy.events("STALL received"), spy.events("Stall during an escape"), spy.events("Contact escape")
    stalls, dead, escapes = with_log(go)
    dt = time.perf_counter() - t0
    assert stalls == [("STALL received", {"reflex": "stop, goal abandoned, no motion", "acted": True})], stalls
    assert dead == [("Stall during an escape: stopping dead", {"trigger": "stall"})], dead
    assert dt < 1.0, f"the stall must stop the escape at once, took {dt:.1f} s"
    assert fake.sent[-1].linear.x == 0.0, "the wheels end at zero"
    assert len(escapes) == 1 and escapes[0][1]["aborted"] is True, escapes
    time.sleep(0.2)
    assert fake.starts == 0 and planner._held_goal is None, "the held goal is dropped, never planned"
    assert planner._current_goal is None and False in failed
    print(f"  stall (handle_stall) during an escape -> stopped dead after {dt:.2f} s, held goal dropped, nothing planned")


def test_a_rear_contact_during_a_front_escape_stops_it_dead() -> None:
    planner, fake = make_planner(moves=False)
    planner.escape_timeout_s = 5.0
    asyncio.run(RecoveringPlanner.handle_bump(module_on(planner), Bool(data=True)))
    time.sleep(0.15)
    t0 = time.perf_counter()

    def go(spy):
        asyncio.run(RecoveringPlanner.handle_bump_rear(module_on(planner), Bool(data=True)))
        wait_escape_done(planner)
        return spy.events("BUMP received"), spy.events("Contact during an escape")
    bumps, dead = with_log(go)
    assert bumps == [("BUMP received: rear contact", {"reflex": "escape forward 0.20 m", "acted": False})], bumps
    assert dead == [("Contact during an escape: stopping dead", {"trigger": "bump_rear"})], dead
    assert time.perf_counter() - t0 < 1.0 and fake.sent[-1].linear.x == 0.0
    assert all(tw.linear.x <= 0.0 for tw in fake.sent), "nothing moves forward: the rear contact only stops the reverse"
    print("  bump then bump_rear during the reverse -> stopped dead, logged acted=False, never a forward move")


if __name__ == "__main__":
    for t in (test_stuck_abandons_goal_without_any_motion,
              test_stuck_needs_a_driven_path_and_ignores_rotation,
              test_escape_moves_away_then_abandons,
              test_escape_times_out_when_not_moving,
              test_second_contact_aborts_the_escape,
              test_stuck_is_silent_during_an_escape,
              test_goal_during_an_escape_is_held_then_planned,
              test_goal_outside_an_escape_is_planned_at_once,
              test_a_plan_racing_the_escape_start_never_leaves_the_follower_running,
              test_the_module_has_the_three_inputs,
              test_front_contact_backs_off_in_reverse,
              test_rear_contact_escapes_forward,
              test_a_false_message_does_nothing,
              test_stall_abandons_the_goal_without_any_motion,
              test_stall_without_a_goal_gives_nothing_up,
              test_stall_during_an_escape_stops_it_dead_and_drops_the_held_goal,
              test_a_rear_contact_during_a_front_escape_stops_it_dead):
        print(t.__name__); t()
    print("TEST PASSED")
