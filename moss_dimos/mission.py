"""The litter mission: explore, see a piece of litter, go to it, put it at
the arm's pick point, run the pick-and-drop cycle, explore again.

`MissionLogic` is the whole decision, pure: it takes what the robot knows
(the confirmed objects in the world frame from `moss_dimos.litter_detect`,
the odometry pose, the planner's verdict) and returns commands; the bench
drives it with a fake robot and a fake clock. `LitterMission` is the thin
dimOS module around it: streams in, streams out, a 10 Hz tick, the arm's
cycle as a subprocess (tools/moss_arm_play.py over RPC to the arm's
coordinator).

States and what they publish:

  EXPLORING   nothing; the explorer drives. A confirmed object whose class
              is litter, not yet picked or skipped, becomes the target.
  STOPPING    stop_explore_cmd once; wait STOP_SETTLE_S so the explorer's
              own "stay here" goal lands first (the cockpit's rule).
  APPROACHING goal_request once: the point STANDOFF_M before the object on
              the line from the robot, facing it (or straight to SERVOING
              when the robot is already that close). goal_reached True ->
              SERVOING; False -> the object is skipped; a timeout -> SERVOING
              from wherever the rover stopped.
  SERVOING    tele_cmd_vel at every tick (the pad's channel: the movement
              manager lets it win and cancels any navigation): turn until
              the robot's pick point reaches the object (one law for the
              turn and the creep, the pick point as the controlled point),
              then a zero twist. The base has
              a power floor (a non-zero track is raised to about half
              power) and pivots at full power, so a slow twist does not
              exist: near the object (SERVO_FAR_M) and while turning, the
              commands come as pulses, one tick of twist then SERVO_REST_S
              of zeros, and the rover moves a step at a time. Far from it
              the twist is continuous. A timeout skips the object.
  PICKING     the pick command runs (blocking, in its own thread); done ->
              the object is marked picked; a failure skips it.
  RESUMING    explore_cmd once, then EXPLORING.

The pick point is where the cycle's `pick` pose puts the tool, in
base_link (the lidar's floor projection): PICK_XY_M, from the CAD scene by
default (the tool point less the scanner's axis) and set from a tape
measure on the robot with MOSS_PICK_XY_M. The whole final approach depends
on it, and the camera's mount must share the origin (MOSS_CAMERA_XYZ_M).
"""

from __future__ import annotations

import math
import os
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

PICK_XY_ENV = "MOSS_PICK_XY_M"
PICK_XY_DEFAULT = (0.215, -0.094)    # the cycle's pick tool point (0.345, -0.010) less the scanner's axis (0.130, 0.084), CAD scene; to measure
STANDOFF_ENV = "MOSS_STANDOFF_M"
STANDOFF_DEFAULT = 0.6
TOL_ENV = "MOSS_PICK_TOL_M"
TOL_DEFAULT = 0.02
PICK_CMD_ENV = "MOSS_PICK_CMD"       # the command that runs the cycle; "none" = mark picked without moving the arm
PICK_SIM_ENV = "MOSS_PICK_SIM"       # 1: the cycle on the simulated arm (--sim)
AUTOSTART_ENV = "MOSS_MISSION_AUTOSTART"   # 0: start in EXPLORING without sending explore_cmd (the cockpit started it)

STOP_SETTLE_S = 3.0
APPROACH_TIMEOUT_S = 90.0
SERVO_TIMEOUT_S = 60.0
PICK_TIMEOUT_S = 180.0
RESUME_SETTLE_S = 1.0
SKIP_FOR_S = 600.0                   # a skipped object is tried again after this
NEAR_DONE_M = 0.15                   # an object this close to a picked one is the same object
MAX_TARGET_RANGE_M = 4.0
SERVO_V_MAX = 0.08                   # m/s (the base raises any non-zero command to its floor anyway)
SERVO_V_MIN = 0.03
SERVO_W_MAX = 0.4                    # rad/s
SERVO_W_MIN = 0.12
ALIGN_TOL_ENV = "MOSS_SERVO_ALIGN_DEG"
ALIGN_TOL_DEG = 4.0                  # kept as a knob for the report; the law below needs no bearing tolerance
K_V = 0.8
K_W = 1.5
TICK_HZ = 10.0
PULSE_S_ENV = "MOSS_SERVO_PULSE_S"
PULSE_S = 0.1                        # one tick of twist...
REST_S_ENV = "MOSS_SERVO_REST_S"
REST_S = 0.4                         # ...then this long of zeros, near the object and while turning
FAR_M_ENV = "MOSS_SERVO_FAR_M"
FAR_M = 0.25                         # beyond this error the twist is continuous

EXPLORING, STOPPING, APPROACHING, SERVOING, PICKING, RESUMING = (
    "EXPLORING", "STOPPING", "APPROACHING", "SERVOING", "PICKING", "RESUMING")


def pick_xy(env: dict | None = None) -> tuple[float, float]:
    env = os.environ if env is None else env
    raw = env.get(PICK_XY_ENV, "").split()
    if len(raw) == 2:
        try:
            return (float(raw[0]), float(raw[1]))
        except ValueError:
            pass
    return PICK_XY_DEFAULT


def _env_float(name: str, default: float, env: dict | None = None) -> float:
    env = os.environ if env is None else env
    try:
        return float(env.get(name, "").strip() or default)
    except ValueError:
        return default


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def in_base(ox: float, oy: float, rx: float, ry: float, yaw: float) -> tuple[float, float]:
    """A world point in base_link (x ahead, y left) for a robot at (rx, ry, yaw)."""
    dx, dy = ox - rx, oy - ry
    c, s = math.cos(yaw), math.sin(yaw)
    return (dx * c + dy * s, -dx * s + dy * c)


def standoff_goal(ox: float, oy: float, rx: float, ry: float, d: float) -> tuple[float, float, float]:
    """The pose d before the object on the line from the robot, facing it."""
    dx, dy = ox - rx, oy - ry
    r = math.hypot(dx, dy)
    if r < 1e-6:
        return (rx, ry, 0.0)
    ux, uy = dx / r, dy / r
    gx, gy = ox - d * ux, oy - d * uy
    return (gx, gy, math.atan2(oy - gy, ox - gx))


@dataclass
class Target:
    track_id: int
    name: str
    x: float
    y: float


@dataclass
class Seen:
    track_id: int
    name: str
    x: float
    y: float
    count: int = 1


@dataclass
class MissionLogic:
    pick: tuple[float, float] = PICK_XY_DEFAULT
    standoff_m: float = STANDOFF_DEFAULT
    tol_m: float = TOL_DEFAULT
    classes: set = field(default_factory=lambda: {"Glass", "Metal", "Paper", "Plastic", "Waste"})
    clock: object = time.monotonic
    align_tol_rad: float = math.radians(ALIGN_TOL_DEG)
    pulse_s: float = PULSE_S
    rest_s: float = REST_S
    far_m: float = FAR_M
    _pulse_until: float = -1.0         # the pulse in progress ends here
    _rest_until: float = -1.0          # zeros until here
    _last_db: float = 0.0              # the bearing error of the last servo_want
    state: str = EXPLORING
    target: Target | None = None
    pose: tuple[float, float, float] | None = None
    objects: list = field(default_factory=list)
    done: list = field(default_factory=list)          # (x, y) of picked objects
    skipped: dict = field(default_factory=dict)       # track_id -> time skipped
    picked_ids: set = field(default_factory=set)
    _since: float = 0.0
    _goal_verdict: bool | None = None
    _pick_result: bool | None = None
    log: list = field(default_factory=list)

    # ── inputs ──

    def on_objects(self, seen: list) -> None:
        self.objects = list(seen)

    def on_odom(self, x: float, y: float, yaw: float) -> None:
        self.pose = (x, y, yaw)

    def on_goal_reached(self, ok: bool) -> None:
        if self.state == APPROACHING:
            self._goal_verdict = bool(ok)

    def pick_done(self, ok: bool) -> None:
        if self.state == PICKING:
            self._pick_result = bool(ok)

    # ── the decision ──

    def _enter(self, state: str, now: float, why: str = "") -> None:
        self.log.append((now, state, why))
        self.state = state
        self._since = now

    def _is_done(self, x: float, y: float) -> bool:
        return any(math.hypot(x - dx, y - dy) < NEAR_DONE_M for dx, dy in self.done)

    def choose_target(self, now: float) -> Target | None:
        if self.pose is None:
            return None
        rx, ry, _ = self.pose
        best = None
        for o in self.objects:
            if o.name not in self.classes or o.track_id in self.picked_ids or self._is_done(o.x, o.y):
                continue
            t_skip = self.skipped.get(o.track_id)
            if t_skip is not None and now - t_skip < SKIP_FOR_S:
                continue
            r = math.hypot(o.x - rx, o.y - ry)
            if r > MAX_TARGET_RANGE_M:
                continue
            if best is None or r < best[0]:
                best = (r, o)
        return Target(best[1].track_id, best[1].name, best[1].x, best[1].y) if best else None

    def _refresh_target(self) -> None:
        """The object's latest position, when the detector still sees it."""
        if self.target is None:
            return
        for o in self.objects:
            if o.track_id == self.target.track_id:
                self.target.x, self.target.y = o.x, o.y
                return

    def servo_want(self) -> tuple[float, float, bool, float]:
        """(v, w, arrived, error m): the twist that drives the pick point onto
        the object. The pick point P sits at (px, py) in base_link; its
        velocity under a twist (v, w) is (v - w py, w px), so the twist for a
        wanted velocity of P is w = vy / px, v = vx + w py: one law for the
        turn and the creep together, no turn-then-creep alternation."""
        rx, ry, yaw = self.pose
        bx, by = in_base(self.target.x, self.target.y, rx, ry, yaw)
        px, py = self.pick
        ex, ey = bx - px, by - py
        err = math.hypot(ex, ey)
        bearing = math.atan2(by, bx)
        self._last_db = wrap(bearing - math.atan2(py, px))
        if err <= self.tol_m:
            return (0.0, 0.0, True, err)
        vx, vy = K_V * ex, K_V * ey
        speed = math.hypot(vx, vy)
        if speed > SERVO_V_MAX:
            vx, vy = vx * SERVO_V_MAX / speed, vy * SERVO_V_MAX / speed
        w = vy / px
        if abs(w) > SERVO_W_MAX:
            k = SERVO_W_MAX / abs(w)
            w, vx = w * k, vx * k
        v = max(-SERVO_V_MAX, min(SERVO_V_MAX, vx + w * py))
        if abs(v) < SERVO_V_MIN and abs(v) > 1e-6:
            v = math.copysign(SERVO_V_MIN, v)
        if abs(w) < SERVO_W_MIN and abs(w) > 1e-6:
            w = math.copysign(SERVO_W_MIN, w)
        return (v, w, False, err)

    def servo(self, now: float | None = None) -> tuple[float, float, bool]:
        """(v, w, arrived) toward the pick point for the current target, pulsed
        near it and while turning: a twist for pulse_s, zeros for rest_s."""
        now = self.clock() if now is None else now
        v, w, arrived, err = self.servo_want()
        if arrived:
            self._pulse_until = self._rest_until = -1.0
            return (0.0, 0.0, True)
        if err > self.far_m:
            return (v, w, False)
        if now < self._pulse_until:
            return (v, w, False)
        if now < self._rest_until:
            return (0.0, 0.0, False)
        self._pulse_until = now + self.pulse_s
        self._rest_until = self._pulse_until + self.rest_s
        return (v, w, False)

    def _skip(self, now: float, why: str) -> list:
        if self.target is not None:
            self.skipped[self.target.track_id] = now
        self._enter(RESUMING, now, why)
        return [("twist", 0.0, 0.0), ("explore",), ("state", f"{RESUMING}: {why}")]

    def tick(self, now: float | None = None) -> list:
        """One step; the commands to publish, in order."""
        now = self.clock() if now is None else now
        out: list = []
        if self.state == EXPLORING:
            t = self.choose_target(now)
            if t is not None:
                self.target = t
                self._enter(STOPPING, now, f"{t.name} #{t.track_id} at ({t.x:.2f}, {t.y:.2f})")
                out += [("stop_explore",), ("state", f"{STOPPING}: {t.name} #{t.track_id} at ({t.x:.2f}, {t.y:.2f})")]
        elif self.state == STOPPING:
            if now - self._since >= STOP_SETTLE_S:
                self._refresh_target()
                rx, ry, _ = self.pose
                r = math.hypot(self.target.x - rx, self.target.y - ry)
                if r <= self.standoff_m + 0.2:
                    self._enter(SERVOING, now, f"already {r:.2f} m away")
                    out.append(("state", f"{SERVOING}: already {r:.2f} m away"))
                else:
                    gx, gy, gyaw = standoff_goal(self.target.x, self.target.y, rx, ry, self.standoff_m)
                    self._goal_verdict = None
                    self._enter(APPROACHING, now, f"goal ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f} deg)")
                    out += [("goal", gx, gy, gyaw), ("state", f"{APPROACHING}: goal ({gx:.2f}, {gy:.2f})")]
        elif self.state == APPROACHING:
            if self._goal_verdict is True:
                self._enter(SERVOING, now, "goal reached")
                out.append(("state", f"{SERVOING}: goal reached"))
            elif self._goal_verdict is False:
                out += self._skip(now, "the planner gave up the goal")
            elif now - self._since >= APPROACH_TIMEOUT_S:
                self._enter(SERVOING, now, "approach timeout, servoing from here")
                out.append(("state", f"{SERVOING}: approach timeout"))
        elif self.state == SERVOING:
            self._refresh_target()
            if self.pose is None:
                return out
            v, w, arrived = self.servo(now)
            if arrived:
                out.append(("twist", 0.0, 0.0))
                self._pick_result = None
                self._enter(PICKING, now, "at the pick point")
                out += [("pick",), ("state", f"{PICKING}: {self.target.name} #{self.target.track_id}")]
            elif now - self._since >= SERVO_TIMEOUT_S:
                out += self._skip(now, "servo timeout")
            else:
                out.append(("twist", v, w))
        elif self.state == PICKING:
            if self._pick_result is True:
                self.done.append((self.target.x, self.target.y))
                self.picked_ids.add(self.target.track_id)
                self._enter(RESUMING, now, "picked")
                out += [("explore",), ("state", f"{RESUMING}: picked {self.target.name} #{self.target.track_id}")]
            elif self._pick_result is False:
                out += self._skip(now, "the pick failed")
            elif now - self._since >= PICK_TIMEOUT_S:
                out += self._skip(now, "pick timeout")
        elif self.state == RESUMING:
            if now - self._since >= RESUME_SETTLE_S:
                self.target = None
                self._enter(EXPLORING, now, "")
                out.append(("state", EXPLORING))
        return out


# ── the dimOS module ──

def _module_class():
    """Built on first use: the dimOS imports stay out of a plain import of this file."""
    from dimos.core.core import rpc
    from dimos.core.module import Module
    from dimos.core.stream import In, Out
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.std_msgs.String import String
    from dimos.perception.experimental.object import Object as DetObject
    from dimos_lcm.std_msgs import Bool
    from dimos.utils.logging_config import setup_logger

    logger = setup_logger()
    # dimOS resolves the stream annotations in this module's namespace: the names must be there
    globals().update(In=In, Out=Out, PoseStamped=PoseStamped, Twist=Twist, Bool=Bool, String=String,
                     DetObject=DetObject)

    class LitterMission(Module):
        objects: In[list[DetObject]]
        odom: In[PoseStamped]
        goal_reached: In[Bool]

        goal_request: Out[PoseStamped]
        tele_cmd_vel: Out[Twist]
        stop_explore_cmd: Out[Bool]
        explore_cmd: Out[Bool]
        mission_state: Out[String]

        def __init__(self, pick_x: float | None = None, pick_y: float | None = None,
                     standoff_m: float | None = None, tol_m: float | None = None,
                     classes: str | None = None, pick_cmd: str | None = None, pick_sim: bool | None = None,
                     autostart: bool | None = None, **kwargs):
            super().__init__(**kwargs)
            px, py = pick_xy()
            from moss_dimos.litter_detect import litter_classes

            cls = {c.strip() for c in classes.split(",")} if classes else litter_classes()
            self._logic = MissionLogic(pick=(pick_x if pick_x is not None else px, pick_y if pick_y is not None else py),
                                       standoff_m=standoff_m if standoff_m is not None else _env_float(STANDOFF_ENV, STANDOFF_DEFAULT),
                                       tol_m=tol_m if tol_m is not None else _env_float(TOL_ENV, TOL_DEFAULT),
                                       classes=cls,
                                       align_tol_rad=math.radians(_env_float(ALIGN_TOL_ENV, ALIGN_TOL_DEG)),
                                       pulse_s=_env_float(PULSE_S_ENV, PULSE_S), rest_s=_env_float(REST_S_ENV, REST_S),
                                       far_m=_env_float(FAR_M_ENV, FAR_M))
            self._pick_cmd = pick_cmd if pick_cmd is not None else os.environ.get(PICK_CMD_ENV, "").strip()
            self._pick_sim = pick_sim if pick_sim is not None else os.environ.get(PICK_SIM_ENV, "").strip() in ("1", "true", "yes", "on")
            self._autostart = autostart if autostart is not None else os.environ.get(AUTOSTART_ENV, "1").strip() not in ("0", "false", "no", "off")
            self._lock = threading.Lock()
            self._stop = threading.Event()
            self._thread: threading.Thread | None = None
            self._pick_thread: threading.Thread | None = None
            self._last_state_text = ""

        @property
        def logic(self) -> MissionLogic:
            return self._logic

        @rpc
        def start(self) -> None:
            super().start()
            self.objects.subscribe(self._on_objects)
            self.odom.subscribe(self._on_odom)
            self.goal_reached.subscribe(self._on_goal_reached)
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="litter-mission", daemon=True)
            self._thread.start()
            lg = self._logic
            logger.info(f"litter mission started: pick point {lg.pick} m in base_link, standoff "
                        f"{lg.standoff_m} m, tolerance {lg.tol_m} m, align {math.degrees(lg.align_tol_rad):.0f} deg, "
                        f"pulses {lg.pulse_s} s on / {lg.rest_s} s off within {lg.far_m} m, classes {sorted(lg.classes)}, "
                        f"pick command {self._pick_cmd or 'tools/moss_arm_play.py'}{' --sim' if self._pick_sim else ''}")
            if self._autostart:
                self.explore_cmd.publish(Bool(data=True))

        @rpc
        def stop(self) -> None:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            try:
                self.tele_cmd_vel.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
            except Exception:  # noqa: BLE001
                pass
            super().stop()

        @rpc
        def mission_status(self) -> dict:
            lg = self._logic
            return {"state": lg.state, "target": None if lg.target is None else
                    {"track_id": lg.target.track_id, "name": lg.target.name, "x": lg.target.x, "y": lg.target.y},
                    "picked": len(lg.done), "skipped": len(lg.skipped)}

        def _on_objects(self, msg) -> None:
            seen = []
            for o in msg or []:
                try:
                    seen.append(Seen(int(o.track_id), str(o.name), float(o.center.x), float(o.center.y),
                                     int(getattr(o, "detections_count", 1))))
                except Exception:  # noqa: BLE001 - an object without a centre
                    continue
            with self._lock:
                self._logic.on_objects(seen)

        def _on_odom(self, msg) -> None:
            try:
                yaw = float(msg.orientation.to_euler().z)
                x, y = float(msg.position.x), float(msg.position.y)
            except Exception:  # noqa: BLE001
                return
            with self._lock:
                self._logic.on_odom(x, y, yaw)

        def _on_goal_reached(self, msg) -> None:
            with self._lock:
                self._logic.on_goal_reached(bool(msg.data))

        def _run(self) -> None:
            period = 1.0 / TICK_HZ
            while not self._stop.is_set():
                t0 = time.monotonic()
                with self._lock:
                    cmds = self._logic.tick(t0)
                for c in cmds:
                    try:
                        self._do(c)
                    except Exception as exc:  # noqa: BLE001
                        logger.warning(f"litter mission: command {c[0]} failed: {exc}")
                time.sleep(max(0.0, period - (time.monotonic() - t0)))

        def _do(self, c: tuple) -> None:
            kind = c[0]
            if kind == "twist":
                self.tele_cmd_vel.publish(Twist(Vector3(float(c[1]), 0.0, 0.0), Vector3(0.0, 0.0, float(c[2]))))
            elif kind == "goal":
                _, x, y, yaw = c
                q = Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw)))
                self.goal_request.publish(PoseStamped(time.time(), "world",
                                                      position=Vector3(float(x), float(y), 0.0), orientation=q))
            elif kind == "stop_explore":
                self.stop_explore_cmd.publish(Bool(data=True))
            elif kind == "explore":
                self.explore_cmd.publish(Bool(data=True))
            elif kind == "pick":
                self._start_pick()
            elif kind == "state":
                text = str(c[1])
                if text != self._last_state_text:
                    self._last_state_text = text
                    logger.info(f"litter mission: {text}")
                self.mission_state.publish(String(data=text))

        def _start_pick(self) -> None:
            if self._pick_cmd.lower() == "none":
                with self._lock:
                    self._logic.pick_done(True)
                return
            argv = shlex.split(self._pick_cmd) if self._pick_cmd else [sys.executable, str(ROOT / "tools" / "moss_arm_play.py")]
            if self._pick_sim and "--sim" not in argv:
                argv.append("--sim")

            def run() -> None:
                try:
                    rc = subprocess.run(argv, timeout=PICK_TIMEOUT_S, check=False).returncode
                except Exception as exc:  # noqa: BLE001
                    logger.warning(f"litter mission: the pick command failed to run: {exc}")
                    rc = 1
                logger.info(f"litter mission: pick command ended with {rc}")
                with self._lock:
                    self._logic.pick_done(rc == 0)

            self._pick_thread = threading.Thread(target=run, name="litter-pick", daemon=True)
            self._pick_thread.start()

    return LitterMission


def __getattr__(name: str):
    if name == "LitterMission":
        cls = _module_class()
        globals()[name] = cls
        return cls
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
