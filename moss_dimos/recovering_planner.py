"""ReplanningAStarPlanner that abandons a goal instead of fighting for it.

The detectors tell the truth; what goes wrong is a scripted reflex on top of
them. A blind back-off on every "stuck" verdict walks the robot backwards,
step by step, into whatever is behind it (a rover running this planner did
it in one run: 38 blind reverses and 14 rear hits, all chained by the
reflex itself): a recovery manoeuvre that creates the crash is worse than
none. What is left, and it is all of it:

  * stuck, or a stall of the base (``stall``: the base's stall guard, tracks
    commanded and encoders frozen for a second) -> STOP and abandon the goal
    (``cancel_goal`` publishes goal_reached=False; the explorer excludes that
    spot and picks another frontier - the pivot toward it is the only motion
    that follows). No scripted motion of any kind. A stall is inferred, not
    felt: a pivot that fails to turn reads the same as a wall, and answering
    it with a blind move is the loop above.
  * a CONTACT is the one thing still allowed a scripted move: away from the
    contact, 0.20 m, once. ``bump`` is a contact in front (the IMU's, or the
    explorer's born-cornered back-off) and backs off in reverse;
    ``bump_rear`` is a contact behind (the IMU's, felt while reversing or as a
    forward kick during a pivot) and moves FORWARD. A second contact during that escape aborts it dead - a
    contact is never dropped or queued behind a running move.

Paths are planned on clearance_cost_map(), a cost field built from the
active robot profile (moss_dimos.robot_profile) at call time.
"""

import threading
import time

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.gradient import voronoi_gradient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.core.stream import In
from dimos.navigation.base import NavigationState
try:  # dimOS main moved the planner under navigation/go2 (e5679069d)
    from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner
    from dimos.navigation.go2.replanning_a_star.min_cost_astar import min_cost_astar
    from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
except ModuleNotFoundError as _e:  # the pinned dimOS (pyproject.toml) keeps it at navigation/replanning_a_star
    if not (_e.name or "").startswith("dimos.navigation.go2"):
        raise
    from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
    from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
    from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos_lcm.std_msgs import Bool
from dimos.utils.logging_config import setup_logger

from moss_dimos.robot_profile import RobotProfile, current as _current_profile, derived as _derived

logger = setup_logger()

ESCAPE_DISTANCE_M = 0.20   # just enough to turn
ESCAPE_SPEED_MPS = 0.10
ESCAPE_TIMEOUT_S = 5.0
ESCAPE_PERIOD_S = 0.05  # base watchdog is 0.2 s; 10 Hz left gaps that stopped the wheels

# --- obstacle cost gradient -------------------------------------------------
#
# dimOS does NOT plan on a flat costmap: global_planner.py builds a gradient
# (navigation_map.py -> path_map.py) and its A* really does pay per-cell cost
# rather than only thresholding it (min_cost_astar.py; the C++ extension
# behaves identically). On a recorded indoor map that field already keeps a
# path within 3 cm of the widest bottleneck any route could possibly have.
# Two things it still gets wrong:
#
#   1. It cannot tell a wide door from a narrow one. voronoi_gradient puts cost
#      0 on the medial axis of EVERY corridor, so a 0.65 m doorway and a 0.90 m
#      doorway both offer a zero-cost route and the tie falls to path length.
#      Measured: with dimOS's field alone the planner takes the 0.65 m corridor
#      when a 0.90 m one of the same length is right there - the path then
#      hugs the walls of the narrow door.
#   2. simple_inflate rounds its radius up to whole cells (inflation.py) and
#      dilates with a disc, so the nominal 0.275 m becomes "block everything
#      within 6 cells INCLUSIVE" = 0.30 m inclusive. The centre cell of a 0.55 m
#      doorway sits at exactly 0.30 m and dies by one cell. Measured with a
#      0.50 m wide reference body: every gap up to 0.62 m plans "no path", and
#      0.65 m is the first that plans, although the body fits through all of
#      them. That comes from dimOS's own default inflation.
#
# So: keep dimOS's ridge cost (it is already near-optimal and its zero-cost
# medial axis is what stops A*, which compares summed cell cost before length,
# from wandering off on infinite detours), take the inflation from a distance
# transform instead of a ceil'd dilation, and add one term dimOS has no way to
# know about - the turning circle.

# The two radii come from the robot profile (clearances() below):
#
#   lethal = robot width / 2 + control margin. Lethal is STRICTLY below it, so
#            a cell sitting exactly ON the radius stays passable. That cell is
#            the centre of a doorway the body fits through, and rounding it
#            away - which is what a ceil'd dilation does - walls off doorways
#            the robot can drive through.
#   pivot  = pivot diameter / 2. Below this a turn in place sweeps into the
#            obstacle. It is also where the extra cost stops: everything the
#            robot can pivot in must stay at the cost dimOS gave it, because A*
#            minimises the SUM of cell costs, so a penalty that never reaches
#            zero is just a penalty on length. An influence radius past the
#            turning circle made both clearance and path length worse.
#
# Cost added at the lethal edge, on top of dimOS's 0-99 ridge cost, and how
# sharply it falls off across the band. Both were swept on recorded maps and
# on synthetic corridors (0.50 m reference body). The exponent matters more
# than the size: A* minimises the SUM of cell costs, so a penalty that is
# still noticeable in the middle of a pinch just makes the path shorter - it
# cuts the corner and LOSES clearance. The 4th power keeps the middle of the
# band almost free and puts the whole cost against the wall. Measured over
# five recorded start/goal pairs:
#
#     penalty  ramp   mean     worst-min  length   corridor chosen
#     0        -      0.507    0.316      31.28    narrow
#     100      ^1     0.506    0.292      31.34    wide
#     100      ^2     0.508    0.300      31.34    wide
#     100      ^4     0.508    0.316      31.28    wide     <- chosen
#     200      ^4     0.509    0.316      32.63    wide     (starts detouring)
#
# So 100 with a 4th-power ramp is the one setting that takes the wide door and
# gives up nothing: same worst case and same path length as dimOS's own field.
PIVOT_PENALTY = 100
PIVOT_RAMP_EXPONENT = 4
# Matches path_map.py so the ridge cost keeps the shape dimOS tuned.
GRADIENT_DISTANCE_M = 1.5


def clearances(profile: RobotProfile | None = None) -> tuple[float, float]:
    """(lethal, pivot) clearance radii in metres for the active robot profile,
    read at call time: lethal = width/2 + control margin, pivot = pivot
    diameter/2 (robot_profile.derived)."""
    d = _derived(profile or _current_profile())
    return d.lethal_clearance_m, d.pivot_clearance_m


def clearance_cost_map(
    binary: OccupancyGrid,
    lethal_m: float | None = None,
    pivot_m: float | None = None,
    penalty: int = PIVOT_PENALTY,
) -> OccupancyGrid:
    """Binary costmap -> the cost field A* searches.

    Three layers: lethal only where the body cannot fit, dimOS's Voronoi ridge
    cost outside it, and a penalty across the band the body can drive through
    but not pivot in. The penalty ramps as the 4th power of how far into that
    band a cell is, so it stays near zero at the turning circle and only bites
    hard against the wall - see PIVOT_RAMP_EXPONENT for why the shape matters.

    lethal_m / pivot_m None = clearances() of the active robot profile.
    """
    if lethal_m is None or pivot_m is None:
        prof_lethal, prof_pivot = clearances()
        lethal_m = prof_lethal if lethal_m is None else lethal_m
        pivot_m = prof_pivot if pivot_m is None else pivot_m
    grid = binary.grid
    resolution = binary.resolution

    # metres from every cell to the nearest observed obstacle cell
    distance_m = ndimage.distance_transform_edt(grid < CostValues.OCCUPIED) * resolution
    # strictly inside the radius, with a tolerance so a cell landing exactly on
    # it survives float noise (6 cells x 0.05 is not exactly 0.30)
    too_close = distance_m + 1e-6 < lethal_m

    lethal = OccupancyGrid(
        grid=np.where(too_close, np.int8(CostValues.OCCUPIED), grid).astype(np.int8),
        resolution=resolution,
        origin=binary.origin,
        frame_id=binary.frame_id,
        ts=binary.ts,
    )
    base = voronoi_gradient(lethal, max_distance=GRADIENT_DISTANCE_M).grid.astype(np.int16)

    # unknown keeps its -1 (A* prices it itself, see min_cost_astar.py) and
    # lethal keeps its 100; only genuinely passable cells take the penalty.
    passable = (base >= CostValues.FREE) & (base < CostValues.OCCUPIED)
    ramp = np.clip((pivot_m - distance_m) / (pivot_m - lethal_m), 0.0, 1.0)
    extra = (penalty * ramp**PIVOT_RAMP_EXPONENT).astype(np.int16)

    out = base.copy()
    out[passable] = np.clip(
        base[passable] + extra[passable], CostValues.FREE, CostValues.OCCUPIED - 1
    )

    return OccupancyGrid(
        grid=out.astype(np.int8),
        resolution=resolution,
        origin=binary.origin,
        frame_id=binary.frame_id,
        ts=binary.ts,
    )


class RecoveringGlobalPlanner(GlobalPlanner):
    """Stuck or stall -> stop, abandon the goal. Contact -> one escape move, then abandon."""

    escape_distance_m: float = ESCAPE_DISTANCE_M
    escape_speed_mps: float = ESCAPE_SPEED_MPS
    escape_timeout_s: float = ESCAPE_TIMEOUT_S

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._escape_abort = threading.Event()

    def _find_wide_path(self, goal: Vector3, robot_pos: Vector3) -> Path | None:
        """Plan on the clearance cost field instead of dimOS's fixed 1.1x
        inflation (global_planner.py, _find_wide_path).

        Same three steps as upstream - build a costmap, keep the cells under the
        robot passable, run A* - with our field in place of
        make_gradient_costmap(1.1). Upstream's single 1.1x try walls off
        doorways the body fits through; the lethal radius here is strictly
        smaller, so this only ever adds routes.
        """
        binary = self._navigation_map.binary_costmap
        costmap = clearance_cost_map(binary)
        self._clear_robot_footprint(costmap, binary, robot_pos)
        path = min_cost_astar(costmap, goal, robot_pos)
        if path and path.poses:
            return path
        return None

    def _replan_path(self) -> None:
        # Runs in the planner's monitoring thread. Upstream asserts a goal
        # exists; the goal can vanish (explorer cancels on "no path") while an
        # escape runs, and an assert there kills the thread for the rest of
        # the run (no "Arrived"/"stuck" is handled after it).
        try:
            if self._escaping:
                return                     # a contact escape owns the wheels right now
            # The stuck detector reads position only: while the follower turns in
            # place (initial/final rotation) the position does not change and a
            # 2.5 s window reads "stuck" -> replan -> new rotation -> "stuck"...
            # Rotating is not stuck.
            if not self._in_stop_message and self._local_planner_state() in ("initial_rotation", "final_rotation", "idle"):
                # "idle": while the explorer WAITS out an exclusion the follower
                # commands nothing - stillness without intent is not "stuck".
                # And the tracker's window must RESTART here: a turn in place
                # can take seconds, and if the tracker counted that stillness
                # the first blink into path_following would fire a replan - new
                # path, sometimes the other way round an obstacle, new
                # rotation, replan again: a 1 Hz bang-bang wiggle. Stillness
                # while rotating never feeds the stuck verdict.
                self._position_tracker.reset_data()
                return
            with self._lock:
                has_goal = self._current_goal is not None and self._current_odom is not None
            if not has_goal:
                logger.info("Replan requested without a goal; ignoring.")
                return
            # Stuck counts only if we were actually driving a path for a while:
            # a fresh goal with "no path" also reads as "stuck" (the tracker's
            # window is already still).
            driving_for = time.perf_counter() - self._path_started_at
            if self._position_tracker.is_stuck() and driving_for >= self._stuck_time_window:
                self.abandon_goal(trigger="stuck_detector")
                return
            super()._replan_path()
        except Exception:  # noqa: BLE001 - never let the monitor thread die
            logger.exception("Replan failed; planner monitor keeps running")

    _path_started_at: float = float("inf")
    _escaping: bool = False
    _in_stop_message: bool = False
    _held_goal: PoseStamped | None = None

    def _local_planner_state(self) -> str:
        with self._local_planner._lock:
            return str(self._local_planner._state)

    def _handle_stop_message(self, stop_message) -> None:  # type: ignore[override]
        # so _replan_path can tell "the follower stopped (obstacle/arrived)" from "the monitor thinks we are stuck"
        self._in_stop_message = True
        try:
            super()._handle_stop_message(stop_message)
        finally:
            self._in_stop_message = False

    # --- one writer on cmd_vel ------------------------------------------------
    #
    # The escape thread writes cmd_vel at 20 Hz. Upstream plans a goal the moment
    # it arrives, and start_planning() puts the path follower on that same
    # Subject at 10 Hz: the wheels then get interleaved forward-path and
    # reverse-escape twists until the escape ends and its cancel_goal() kills a
    # goal accepted seconds earlier - goal_reached=False, and the explorer
    # excludes a frontier it never drove to. So while an escape runs the
    # goal is HELD instead: latest-only, the last one wins, planned when the
    # escape gives the wheels back.

    def handle_goal_request(self, goal: PoseStamped) -> None:
        with self._lock:
            if self._escaping:
                self._held_goal = goal
                logger.info("Goal held: a contact escape owns the wheels")
                return
        super().handle_goal_request(goal)

    def _hold_current_goal(self) -> None:
        """An escape started under a planning attempt: keep the goal for later."""
        with self._lock:
            self._held_goal = self._current_goal
        logger.warning("Planning refused: a contact escape owns the wheels")

    def _plan_held_goal(self) -> bool:
        """Plan the goal an escape held, if any. True when there was one.

        Planned, not cancelled: it never had a follower, and cancel_goal would
        publish goal_reached=False for a goal the rover never drove."""
        with self._lock:
            goal, self._held_goal = self._held_goal, None
        if goal is None:
            return False
        logger.warning("Planning the goal held during the escape")
        self.handle_goal_request(goal)
        return True

    def abandon_goal(self, trigger: str) -> None:
        """Stop, give the goal up, no recovery motion of any kind.

        ``cancel_goal`` publishes goal_reached=False; the explorer notes the
        failed spot (an exclusion) and picks another frontier. There is no
        blind reverse here on purpose: a scripted back-off on every stuck
        verdict walks the robot into whatever is behind it."""
        logger.warning("Goal abandoned, no recovery motion", trigger=trigger)
        self._path_started_at = float("inf")
        self.cancel_goal()

    def stall(self, trigger: str = "stall") -> bool:
        """A stall of the base (the stall guard: tracks commanded, encoders
        frozen): an INFERRED contact, so stop and abandon the goal, no motion
        of any kind, exactly like a stuck verdict.

        A stall during an escape means the escape itself is pushing into
        something: it stops dead, and the goal it held is dropped with it.
        Without a goal or an escape there is nothing to give up (the pad was
        driving): nothing is cancelled. Returns True when a goal or an escape
        was given up."""
        with self._lock:
            self._held_goal = None
            escaping = self._escaping
            if escaping:
                self._escape_abort.set()
            has_goal = self._current_goal is not None
        if escaping:
            logger.warning("Stall during an escape: stopping dead", trigger=trigger)
        if not (has_goal or escaping):
            return False
        self.abandon_goal(trigger=trigger)
        return True

    def escape(self, direction: float, trigger: str) -> bool:
        """Contact escape: the ONE scripted move left. Away from the contact,
        escape_distance_m, once, then the goal is abandoned.

        A second contact while an escape runs aborts the motion: stop dead
        rather than script another move. A contact is never dropped.

        Returns False if this contact aborted a running escape instead."""
        with self._lock:
            # under the lock: a goal request arriving right here is held, never planned
            if self._escaping:
                self._escape_abort.set()
                logger.warning("Contact during an escape: stopping dead", trigger=trigger)
                return False
            self._escaping = True
            self._escape_id += 1
            escape_id = self._escape_id
            # cleared under the lock: a halt() landing right after this
            # escape took the wheels must not be wiped by a late clear()
            self._escape_abort.clear()
        threading.Thread(target=self._escape_run, args=(direction, trigger, escape_id),
                         daemon=True).start()
        return True

    _escape_abort: threading.Event
    _escape_id: int = 0

    def halt(self, trigger: str) -> bool:
        """Operator stop (stop_movement): abort a running escape, drop the
        goal it held, cancel the current goal.

        Without this a soft STOP only cancelled the goal: the escape thread
        kept writing its reverse twist, and once MovementManager's teleop
        cooldown (1 s) ran out the back-off resumed until its 0.20 m - then
        planned the goal it had held. A stop is a stop: no scripted motion
        and no held goal survive it. Contacts keep their own semantics
        (escape()). Returns True when an escape was running."""
        with self._lock:
            self._held_goal = None
            escaping = self._escaping
            if escaping:
                self._escape_abort.set()
        if escaping:
            logger.warning("Stop during an escape: stopping dead", trigger=trigger)
        self.cancel_goal()
        return escaping

    def _end_escape(self, escape_id: int) -> None:
        """Give the wheels back - unless a newer contact has already taken them."""
        with self._lock:
            if self._escape_id == escape_id:
                self._escaping = False

    def _escape_run(self, direction: float, trigger: str, escape_id: int) -> None:
        try:
            self._local_planner.stop_planning()
            with self._lock:
                start = self._current_odom
            if start is None:
                return
            twist = Twist(linear=Vector3(direction * self.escape_speed_mps, 0.0, 0.0))
            t0 = time.perf_counter()
            travelled = 0.0
            while time.perf_counter() - t0 < self.escape_timeout_s:
                if self._escape_abort.is_set():
                    break
                with self._lock:
                    odom = self._current_odom
                travelled = odom.position.distance(start.position)
                if travelled >= self.escape_distance_m:
                    break
                self._local_planner.cmd_vel.on_next(twist)
                time.sleep(ESCAPE_PERIOD_S)
            self._local_planner.cmd_vel.on_next(Twist())
            self._position_tracker.reset_data()
            self._path_started_at = float("inf")
            logger.warning(
                "Contact escape",
                trigger=trigger,
                direction="forward" if direction > 0 else "reverse",
                travelled_m=round(travelled, 3),
                seconds=round(time.perf_counter() - t0, 1),
                aborted=self._escape_abort.is_set(),
            )
            with self._lock:
                held = self._held_goal is not None
            if not held:
                self.cancel_goal()          # unchanged: a contact abandons the goal
                return
            # The wheels are free again: the flag drops BEFORE the follower can
            # be started, so cmd_vel is never written by two threads at once.
            self._end_escape(escape_id)
            self._plan_held_goal()
        except Exception:  # noqa: BLE001
            logger.exception("Contact escape failed")
        finally:
            self._end_escape(escape_id)
            try:
                self._plan_held_goal()      # an early return must not strand it
            except Exception:  # noqa: BLE001
                logger.exception("Held goal could not be planned")

    # dimOS's default is 8 s / 0.4 m. At exploration speed (around 0.3 m/s)
    # eight seconds without progress is far too long to notice a stall:
    # 2.5 s / 0.12 m keeps about the same 5 cm/s floor.
    _stuck_time_window: float = 2.5
    _stuck_threshold: float = 0.12

    def _plan_path(self) -> None:
        if self._escaping:
            self._hold_current_goal()   # a replan racing the start of an escape
            return
        super()._plan_path()
        if self._escaping:
            # a contact fired while A* ran: the escape owns the wheels, so the
            # follower that just started goes, and the goal waits for it
            self._local_planner.stop_planning()
            self._hold_current_goal()
            self._path_started_at = float("inf")
            return
        if self._local_planner.get_state() != NavigationState.IDLE:
            self._path_started_at = time.perf_counter()
        else:
            self._path_started_at = float("inf")


class RecoveringPlanner(ReplanningAStarPlanner):
    """Drop-in for ReplanningAStarPlanner where stuck = goal abandoned (no
    motion), a stall of the base (``stall`` In) = goal abandoned (no motion),
    and a contact (``bump`` / ``bump_rear`` In) triggers the one scripted
    escape move away from the contact: in reverse for ``bump`` (in front: the
    IMU's contacts and the explorer's born-cornered back-off), FORWARD for
    ``bump_rear`` (behind: the IMU's contacts felt while reversing, or a
    forward kick during a pivot)."""

    bump: In[Bool]
    bump_rear: In[Bool]
    stall: In[Bool]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._planner = RecoveringGlobalPlanner(self._planner._global_config)

    def _on_stop_movement(self, msg: Bool) -> None:  # type: ignore[override]
        """stop_movement (the operator's soft STOP): upstream only cancels the
        goal; here it also aborts a running contact escape and its held goal."""
        if getattr(msg, "data", False):
            self._planner.halt(trigger="stop_movement")

    async def handle_bump(self, msg: Bool) -> None:
        """Front contact: stop, escape 0.20 m in reverse, abandon the goal."""
        if getattr(msg, "data", False):
            acted = self._planner.escape(direction=-1.0, trigger="bump")
            logger.warning("BUMP received: front contact", reflex="escape reverse 0.20 m",
                           acted=acted)

    async def handle_bump_rear(self, msg: Bool) -> None:
        """Rear contact: stop, escape 0.20 m FORWARD, abandon the goal."""
        if getattr(msg, "data", False):
            acted = self._planner.escape(direction=+1.0, trigger="bump_rear")
            logger.warning("BUMP received: rear contact", reflex="escape forward 0.20 m",
                           acted=acted)

    async def handle_stall(self, msg: Bool) -> None:
        """A stall of the base: stop, abandon the goal, no motion."""
        if getattr(msg, "data", False):
            acted = self._planner.stall(trigger="stall")
            logger.warning("STALL received", reflex="stop, goal abandoned, no motion", acted=acted)
