"""Cold bench for explorer2: known maps in, known decisions out.

Every check is a known input giving a known output in physical units
(metres, seconds, a named directive) - not "it returned something". Groups:

  A. frontier + goal placement  - where the target lands on a plain room
  B. prefer-forward             - two mirror-image clusters, the one ahead wins
  C. failed-target memory       - an exclusion WAITS, never ends the run, and
                                  reopens on TRIGGERS (map changed / new
                                  viewpoint), never on a clock
  D. born cornered              - the pocket signature gives ONE back-off
  E. blocked region             - a block published as occupied is never targeted
  F. the end of a run           - None exactly when there is nothing left
  H. revisits                   - fresh ground beats ground already worked
  I. shut in                    - pinched is not finished
  K. how far                    - the distance exponent (a big frontier across
                                  the room does not beat a small one beside
                                  the rover), the goal cap (a farther frontier
                                  waits while a safe one lies within it, never
                                  behind a probe, and never ends a run) and the
                                  exploration radius around the first pose
                                  (nothing left inside it is the end of a run)
  G. purity                     - the costmap, the pose and the rest of the
                                  state come back untouched, and the function
                                  is deterministic
  J. the loop around it         - which goal a bare goal_reached is credited to,
                                  and the one line that states exp, cap, radius
  L. the blueprint's knobs      - MOSS_EXPLORE_DISTANCE_EXP, MOSS_GOAL_MAX_M,
                                  MOSS_EXPLORE_RADIUS_M -> the Explorer2 config

No robot and no LCM. A to I, K and G need numpy and scipy only (the geometry is
built on the default Tuning, a 0.50 m wide reference body). Only J and L touch
the module class and the blueprint, and they skip themselves where dimOS is
not installed.

Run from the repository root:  python benches/bench_explorer2_cold.py
"""
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# moss_dimos/__init__.py imports nothing, so this works with or without dimOS
import moss_dimos.explorer2 as _explorer2_module  # noqa: E402
from moss_dimos.explorer2 import (  # noqa: E402
    DIRECTIVE_BACK_OFF, DIRECTIVE_FRONTIER, DIRECTIVE_WAIT, DEFAULT_TUNING,
    ExploreState, PoseStamped, Tuning, WAIT_REOPEN_POLLS, next_target,
)

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
RES = 0.05

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- the smallest thing that looks like a costmap --------------------------

class Grid:
    """Everything next_target reads off an OccupancyGrid, and nothing else."""

    def __init__(self, grid, res=RES, ox=0.0, oy=0.0, ts=0.0):
        self.grid = grid
        self.resolution = res
        self.frame_id = "world"
        self.ts = ts
        self.origin = type("O", (), {
            "position": type("P", (), {"x": ox, "y": oy, "z": 0.0})()})()


def pose(x, y, yaw=0.0):
    p = PoseStamped(ts=0.0, frame_id="world")
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def cell(x, y, ox=0.0, oy=0.0, res=RES):
    return int((y - oy) / res), int((x - ox) / res)


def room(n=140, x0=30, x1=110, y0=30, y1=110, walls=True):
    """A square of free floor in the middle of unknown, walled all round.

    n x n cells at 5 cm; with the defaults that is 7 x 7 m of grid holding a
    4 x 4 m room. Openings are punched into the wall afterwards.
    """
    g = np.full((n, n), UNKNOWN, dtype=np.int8)
    g[y0:y1, x0:x1] = FREE
    if walls:
        g[y0 - 1:y1 + 1, x0 - 1] = OCCUPIED
        g[y0 - 1:y1 + 1, x1] = OCCUPIED
        g[y0 - 1, x0 - 1:x1 + 1] = OCCUPIED
        g[y1, x0 - 1:x1 + 1] = OCCUPIED
    return g


def door(g, side, centre, width_cells, x0=30, x1=110, y0=30, y1=110):
    """Punch an opening of `width_cells` in one wall, centred on `centre`."""
    half = width_cells // 2
    if side == "east":
        g[centre - half:centre + half, x1] = UNKNOWN
    elif side == "west":
        g[centre - half:centre + half, x0 - 1] = UNKNOWN
    elif side == "north":
        g[y1, centre - half:centre + half] = UNKNOWN
    elif side == "south":
        g[y0 - 1, centre - half:centre + half] = UNKNOWN
    return g


def w_of(cell_index):
    """Cell index -> the world coordinate of its centre, origin at 0."""
    return (cell_index + 0.5) * RES


# ===========================================================================
print("A. a room with one door: the target stands on free floor, at the door")

g = door(room(), "east", 70, 24)
cm = Grid(g)
here = pose(w_of(70), w_of(70), 0.0)          # middle of the room, facing +x
st = ExploreState()
target = next_target(cm, here, st, now=100.0)

check("a target comes back", target is not None)
assert target is not None
check("directive is 'frontier'", target.directive == DIRECTIVE_FRONTIER, target.directive)
gy, gx = cell(target.position.x, target.position.y)
check("the target stands on FREE floor, not in the unknown",
      g[gy, gx] == FREE and not target.on_frontier, f"grid value {g[gy, gx]}")
check("the target is at the door, not at the centroid of the room",
      target.position.x > w_of(100), f"x = {target.position.x:.2f} m")
check("the look-at point is in the unknown beyond the door",
      g[cell(*target.look_at_xy)] == UNKNOWN)
nearest_wall = min(math.hypot(target.position.x - w_of(cx), target.position.y - w_of(cy))
                   for cy, cx in zip(*np.nonzero(g == OCCUPIED)))
check("it is at least a body radius from any wall",
      nearest_wall >= DEFAULT_TUNING.lethal_clearance_m,
      f"{nearest_wall:.2f} m >= {DEFAULT_TUNING.lethal_clearance_m} m")
check("path cost is the real walking distance, not a straight line",
      target.path_cost_m >= abs(target.position.x - here.position.x) - 1e-6,
      f"{target.path_cost_m:.2f} m")
check("state: one target issued, one visited, one observed pose",
      (st.targets_issued, len(st.visited), len(st.observed)) == (1, 1, 1),
      f"{st.targets_issued}, {len(st.visited)}, {len(st.observed)}")


# ===========================================================================
print("B. prefer-forward: two mirror-image doors, the one ahead wins")

g = door(door(room(), "east", 70, 24), "west", 70, 24)
cm = Grid(g)
centre = w_of(70)

east = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
west = next_target(cm, pose(centre, centre, math.pi), ExploreState(), now=100.0)
check("facing +x picks the east door", east is not None and east.position.x > centre,
      f"x = {east.position.x:.2f}" if east else "None")
check("facing -x picks the west door", west is not None and west.position.x < centre,
      f"x = {west.position.x:.2f}" if west else "None")
check("the two doors really are the same cluster size and the same distance",
      east is not None and west is not None
      and east.info_cells == west.info_cells
      and abs(east.path_cost_m - west.path_cost_m) < 0.11,
      f"{east.info_cells} vs {west.info_cells} cells, "
      f"{east.path_cost_m:.2f} vs {west.path_cost_m:.2f} m" if east and west else "")
sideways = next_target(cm, pose(centre, centre, math.pi / 2), ExploreState(), now=100.0)
check("facing sideways, neither bonus applies and one is still chosen",
      sideways is not None and sideways.directive == DIRECTIVE_FRONTIER)


# ===========================================================================
print("C. failed-target memory: an exclusion WAITS, reopens on triggers, never a clock")

g = door(room(), "east", 70, 24)
cm = Grid(g)
here = pose(centre, centre, 0.0)

probe = next_target(cm, here, ExploreState(), now=0.0)
assert probe is not None
gx, gy = probe.position.x, probe.position.y

st = ExploreState()
st.note_failed(gx, gy, (centre, centre), cm)
waited = next_target(cm, here, st, now=10.0)
check("the only cluster is excluded -> NOT None", waited is not None)
assert waited is not None
check("directive is 'wait'", waited.directive == DIRECTIVE_WAIT, waited.directive)
check("the rover is told to stay where it is",
      (waited.position.x, waited.position.y) == (here.position.x, here.position.y))
check("the cluster is still counted as a frontier, not as absence",
      waited.n_clusters == 1 and waited.n_excluded == 1,
      f"{waited.n_clusters} clusters, {waited.n_excluded} excluded")

much_later = next_target(cm, here, st, now=1e9)
check("NO clock reopens it: same map, same pose, a billion seconds later -> still wait",
      much_later is not None and much_later.directive == DIRECTIVE_WAIT,
      much_later.directive if much_later else "None")

# starvation escape: after WAIT_REOPEN_POLLS silent asks the oldest entry reopens
starved = ExploreState()
starved.note_failed(gx, gy, (centre, centre), cm)
out = None
for _ in range(WAIT_REOPEN_POLLS + 2):
    out = next_target(cm, here, starved, now=10.0)
    if out is not None and out.directive == DIRECTIVE_FRONTIER:
        break
check(f"starved {WAIT_REOPEN_POLLS} asks -> the oldest failed goal reopens and is targeted",
      out is not None and out.directive == DIRECTIVE_FRONTIER and starved.failed == [],
      out.directive if out else "None")

# trigger 1: the rover stands a viewpoint away from where it failed
moved_st = ExploreState()
moved_st.note_failed(gx, gy, (centre, centre), cm)
moved = next_target(cm, pose(centre - 1.2, centre, 0.0), moved_st, now=10.0)
check("the rover 1.2 m from where it failed -> the exclusion reopens (viewpoint trigger)",
      moved is not None and moved.directive == DIRECTIVE_FRONTIER,
      moved.directive if moved else "None")
check("and the reopened entry is pruned from the state", moved_st.failed == [],
      f"{moved_st.failed}")

# trigger 2: the map around the goal changed (new cells observed)
changed_st = ExploreState()
changed_st.note_failed(gx, gy, (centre, centre), cm)
g_changed = g.copy()
cgx, cgy = int((gx - cm.origin.position.x) / cm.resolution), int((gy - cm.origin.position.y) / cm.resolution)
patch = g_changed[cgy - 4:cgy + 5, cgx - 4:cgx + 5]
patch[patch == UNKNOWN] = FREE                     # the world got observed there
changed = next_target(Grid(g_changed), here, changed_st, now=10.0)
check("new observations around the failed goal -> the exclusion reopens (map trigger)",
      changed is not None and changed.directive == DIRECTIVE_FRONTIER,
      changed.directive if changed else "None")

# two exclusions: both held -> wait; one held -> the other is simply used
g2 = door(door(room(), "east", 70, 24), "west", 70, 24)
cm2 = Grid(g2)
e = next_target(cm2, pose(centre, centre, 0.0), ExploreState(), now=0.0)
wst = next_target(cm2, pose(centre, centre, math.pi), ExploreState(), now=0.0)
assert e is not None and wst is not None
both = ExploreState()
both.note_failed(e.position.x, e.position.y, (centre, centre), cm2)
both.note_failed(wst.position.x, wst.position.y, (centre, centre), cm2)
waited2 = next_target(cm2, pose(centre, centre, 0.0), both, now=45.0)
check("with two exclusions held -> wait, both counted",
      waited2 is not None and waited2.directive == DIRECTIVE_WAIT
      and waited2.n_excluded == 2,
      f"{waited2.n_excluded if waited2 else None} excluded")

one = ExploreState()
one.note_failed(e.position.x, e.position.y, (centre, centre), cm2)
other = next_target(cm2, pose(centre, centre, 0.0), one, now=45.0)
check("only one of two excluded -> the other is targeted, no wait",
      other is not None and other.directive == DIRECTIVE_FRONTIER
      and other.position.x < centre,
      f"{other.directive} x={other.position.x:.2f}" if other else "None")


# ===========================================================================
print("D. born cornered: the pocket signature gives ONE back-off")

# 3 x 3 free cells (0.0225 m2) sealed by a ring of obstacle, unknown beyond.
g = np.full((40, 40), UNKNOWN, dtype=np.int8)
g[19:22, 19:22] = FREE
g[18:23, 18] = OCCUPIED
g[18:23, 22] = OCCUPIED
g[18, 18:23] = OCCUPIED
g[22, 18:23] = OCCUPIED
cm = Grid(g)
here = pose(w_of(20), w_of(20), 0.0)
st = ExploreState()

first = next_target(cm, here, st, now=100.0)
check("a cornered rover gets a directive, not None", first is not None)
assert first is not None
check("directive is 'back_off'", first.directive == DIRECTIVE_BACK_OFF, first.directive)
check("the back-off is 0.22 m straight behind the heading",
      abs(first.position.x - (here.position.x - 0.22)) < 1e-9
      and abs(first.position.y - here.position.y) < 1e-9,
      f"({first.position.x:.3f}, {first.position.y:.3f})")
check("it reports the pocket it measured: 9 cells = 0.0225 m2",
      abs(first.reachable_free_m2 - 0.0225) < 1e-9, f"{first.reachable_free_m2}")
check("state remembers the back-off went out", st.back_off_issued is True)

second = next_target(cm, here, st, now=101.0)
check("the SAME pocket does not give a second back-off: it gives None",
      second is None, str(second))

# facing another way, the back-off follows the heading
st3 = ExploreState()
turned = next_target(cm, pose(w_of(20), w_of(20), math.pi / 2), st3, now=100.0)
check("heading +y -> back-off is 0.22 m in -y",
      turned is not None and abs(turned.position.y - (w_of(20) - 0.22)) < 1e-9
      and abs(turned.position.x - w_of(20)) < 1e-9,
      f"({turned.position.x:.3f}, {turned.position.y:.3f})" if turned else "None")


# ===========================================================================
print("E. blocked region: a block published as occupied is never targeted")

# The room's only door leads into a corridor the map publishes as occupied (100),
# which is all the explorer ever sees of an obstacle.
g = door(room(), "east", 70, 24)
keep = np.zeros_like(g, dtype=bool)
keep[58:82, 110:118] = True          # the corridor behind the door
g[keep] = OCCUPIED
cm = Grid(g)
st = ExploreState()
blocked = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)

check("with the only way out forbidden, exploration ends: None",
      blocked is None, str(blocked))

# now give the room a second, legal door and check the target never lands in
# the forbidden block, nor within a body radius of it
g2 = door(g.copy(), "north", 70, 24)
cm2 = Grid(g2)
seen_in_zone = 0
closest = math.inf
st2 = ExploreState()
for i in range(12):
    t = next_target(cm2, pose(centre, centre, 0.0), st2, now=100.0 + i)
    if t is None or t.directive != DIRECTIVE_FRONTIER:
        break
    ty, tx = cell(t.position.x, t.position.y)
    if keep[ty, tx]:
        seen_in_zone += 1
    for ky, kx in zip(*np.nonzero(keep)):
        closest = min(closest, math.hypot(t.position.x - w_of(kx), t.position.y - w_of(ky)))
check("no target ever lands inside the forbidden block", seen_in_zone == 0,
      f"{seen_in_zone} did")
check("and none comes within the lethal clearance of it",
      closest >= DEFAULT_TUNING.lethal_clearance_m,
      f"closest {closest:.2f} m >= {DEFAULT_TUNING.lethal_clearance_m} m")


# ===========================================================================
print("F. the end of a run: None exactly when there is nothing left")

# a completely known map: free room, wall, and solid obstacle everywhere else
g = np.full((140, 140), OCCUPIED, dtype=np.int8)
g[30:110, 30:110] = FREE
cm = Grid(g)
st = ExploreState()
done = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a map with no unknown cell at all -> None", done is None, str(done))
check("and it is not the cornered branch: the floor is 16 m2, not a pocket",
      st.back_off_issued is False and st.last_directive == "")

# one unknown cell short of complete, but too small to be a frontier cluster
g2 = g.copy()
g2[60:62, 60:62] = UNKNOWN            # 4 unknown cells, min cluster is 6
check("an unknown speck below min_frontier_perimeter is not a frontier -> None",
      next_target(Grid(g2), pose(centre, centre, 0.0), ExploreState(), now=100.0) is None)

g3 = g.copy()
g3[60:70, 60:70] = UNKNOWN            # 100 unknown cells in the middle of the room
kept = next_target(Grid(g3), pose(w_of(100), w_of(100), 0.0), ExploreState(), now=100.0)
check("a real unknown pocket inside the room IS a frontier -> a target",
      kept is not None and kept.directive == DIRECTIVE_FRONTIER,
      kept.directive if kept else "None")


# ===========================================================================
print("H. fresh ground beats ground already worked")

# Two mirror-image doors. Test B has already shown that, facing east, the east
# door wins. Now the rover has already been sent to the east door once - and
# has been somewhere else since - so the east door is a REVISIT.
g = door(door(room(), "east", 70, 24), "west", 70, 24)
cm = Grid(g)
centre = w_of(70)

plain = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
assert plain is not None
east_goal = (plain.position.x, plain.position.y)
check("(setup) facing east with no memory, the east door is the pick",
      east_goal[0] > centre, f"x = {east_goal[0]:.2f}")

faded = next_target(cm, pose(centre, centre, 0.0),
                    ExploreState(visited=[east_goal, (-9.0, -9.0)]), now=100.0)
check("a door already published, with a goal since, loses to the fresh one",
      faded is not None and faded.position.x < centre,
      f"x = {faded.position.x:.2f}" if faded else "None")

# ... but the goal it was driving to when it last stopped is not a revisit:
# that one is unfinished business and must not be dropped.
current = next_target(cm, pose(centre, centre, 0.0),
                      ExploreState(visited=[(-9.0, -9.0), east_goal]), now=100.0)
check("the goal just attempted keeps its score: no fade on unfinished business",
      current is not None and current.position.x > centre,
      f"x = {current.position.x:.2f}" if current else "None")

# The fade is the SQUARE of the distance: at half the revisit radius a cluster
# keeps a quarter of its score, not a half. One door, so the same cluster comes
# back either way and only the fade differs.
one_door = Grid(door(room(), "east", 70, 24))
clean = next_target(one_door, pose(centre, centre, 0.0), ExploreState(), now=100.0)
assert clean is not None
half = (clean.position.x - DEFAULT_TUNING.revisit_radius_m / 2, clean.position.y)
faded_half = next_target(one_door, pose(centre, centre, 0.0),
                         ExploreState(visited=[half, (-9.0, -9.0)]), now=100.0)
ratio = (faded_half.score / clean.score) if faded_half is not None else None
check("a cluster half a revisit radius from an older goal keeps a QUARTER of its score",
      ratio is not None and abs(ratio - 0.25) < 1e-9, f"ratio {ratio}")


# ===========================================================================
print("H2. the gain is what the viewpoint can SEE, not unknown behind the wall")

# Same room, two doors. Behind the east door, a cupboard: 0.35 m of unknown
# closed by walls. Behind the west door, open unknown. The two frontiers have
# the same cell count and the same distance, and the rover faces EAST, so a
# gain that counts unknown through a wall picks the cupboard.
g = door(door(room(), "east", 70, 24), "west", 70, 24)
g[57:83, 117] = OCCUPIED           # cupboard back wall, 0.35 m beyond the door
g[57, 110:118] = OCCUPIED          # and its two sides
g[82, 110:118] = OCCUPIED
cm = Grid(g)
seen = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
check("facing the cupboard, the rover still takes the door with a room behind it",
      seen is not None and seen.position.x < centre,
      f"x = {seen.position.x:.2f}" if seen else "None")
check("and the gain it reports is a fraction of a disc, not a cell count",
      seen is not None and 0.0 <= seen.info_gain <= 1.0, f"{seen.info_gain}" if seen else "")


# ===========================================================================
print("I. shut in is not finished")

# A room the body cannot leave: its only opening is 0.50 m wide, and the default
# (reference body) clearance needs 0.60 m to pass. Beyond it, a corridor and
# real unknown.
def pinched_map():
    g = np.full((160, 160), OCCUPIED, dtype=np.int8)
    g[30:110, 30:110] = FREE                  # the room the rover is in
    g[65:75, 110:140] = FREE                  # a 0.50 m x 1.50 m throat, floor seen
    g[30:110, 140:155] = UNKNOWN              # the room beyond, never seen
    return g

g = pinched_map()
cm = Grid(g)
st = ExploreState()
shut_in = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a rover shut in with frontiers left on the map does NOT report completion",
      shut_in is not None, str(shut_in))
assert shut_in is not None
check("it gets one back-off: reverse once and look again",
      shut_in.directive == DIRECTIVE_BACK_OFF, shut_in.directive)
check("and it says how many clusters the map still holds",
      getattr(shut_in, "n_on_the_map", 0) >= 1, f"{getattr(shut_in, 'n_on_the_map', 0)}")
# NOT the born-cornered case: the 4 x 4 m room, minus the 0.30 m the body
# cannot use along each wall, is a 3.50 x 3.50 m square = 12.25 m2, plus the
# six cells at the mouth of the throat where the wall opens out and the
# clearance rises back over 0.30 m: 12.265 m2, twenty-four times the 0.5 m2
# that means "born cornered".
check("this is NOT the born-cornered case: 12.265 m2 of floor under it",
      abs(getattr(shut_in, "reachable_free_m2", 0.0) - 12.265) < 1e-9,
      f"{getattr(shut_in, 'reachable_free_m2', float('nan')):.4f} m2")
again = next_target(cm, pose(centre, centre, 0.0), st, now=101.0)
check("one back-off per pocket and no more: then the run ends", again is None, str(again))


# ===========================================================================
print("I2. a frontier the body cannot walk to is looked at from where it can stand")

# The same throat, but the unknown starts right at its mouth: the frontier is
# 0.55 m from floor the body can stand on. Dropping a frontier like this would
# let the run declare itself finished with it still on the map.
g = np.full((160, 160), OCCUPIED, dtype=np.int8)
g[30:110, 30:110] = FREE
g[65:75, 110:116] = FREE                      # 0.50 m throat, 0.30 m of seen floor
g[62:78, 116:150] = UNKNOWN                   # the unmapped room, opening out
cm = Grid(g)
st = ExploreState()
peek = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a target comes back rather than None", peek is not None, str(peek))
assert peek is not None
check("it is a frontier goal", peek.directive == DIRECTIVE_FRONTIER, peek.directive)
gy, gx = cell(peek.position.x, peek.position.y)
check("the rover is told to stand on floor it can reach, not in the unknown",
      g[gy, gx] == FREE and not peek.on_frontier, f"grid {g[gy, gx]}")
nearest_wall = min(math.hypot(peek.position.x - w_of(cx), peek.position.y - w_of(cy))
                   for cy, cx in zip(*np.nonzero(g == OCCUPIED)))
check("and it stands a body radius clear of the walls",
      nearest_wall >= DEFAULT_TUNING.lethal_clearance_m,
      f"{nearest_wall:.2f} m")
check("the look-at point is the unknown behind the throat",
      peek.look_at_xy[0] > w_of(110), f"{peek.look_at_xy}")


# ===========================================================================
print("I3. a frontier is retired by being SEEN, not by a viewpoint being used")

# One door, one cluster. The rover drives to the target and decides from there;
# the map does not change (the shadow the 2D scan plane never enters). Standing
# in plain view of it and learning nothing is the end of that frontier - this
# is the 300-goals-and-184-m protection, and it must still hold.
g = door(room(), "east", 70, 24)
cm = Grid(g)
st = ExploreState()
first = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
assert first is not None
arrived = next_target(cm, pose(first.position.x, first.position.y, 0.0), st, now=101.0)
check("looked straight at it from close up and nothing changed: that cluster is done",
      arrived is None, str(arrived))

# Same distance, same everything, one wall of difference. A closed room with
# one unknown pocket in the middle of its floor, and a spot the rover has
# already decided from, 0.70 m from that pocket.
def pocket_room(partition: bool):
    g = np.full((140, 140), OCCUPIED, dtype=np.int8)
    g[30:110, 30:110] = FREE
    g[66:74, 66:74] = UNKNOWN            # 0.40 x 0.40 m of unknown floor
    if partition:
        g[60:80, 80] = OCCUPIED          # a screen between the spot and the pocket
    return g

stood_at = (w_of(84), w_of(70))          # 0.50 m east of the pocket's edge
in_view = next_target(Grid(pocket_room(False)), pose(*stood_at, 0.0),
                      ExploreState(observed=[stood_at]), now=100.0)
check("a pocket in plain view of a spot already decided from is retired",
      in_view is None, str(in_view))
hidden = next_target(Grid(pocket_room(True)), pose(*stood_at, 0.0),
                     ExploreState(observed=[stood_at]), now=100.0)
check("the same pocket behind a screen is NOT retired: it was never seen",
      hidden is not None and hidden.directive == DIRECTIVE_FRONTIER,
      (hidden.directive if hidden else "None"))
if hidden is not None:
    check("and the viewpoint it is given is on the pocket's side of the screen",
          hidden.position.x < w_of(80), f"x = {hidden.position.x:.2f} m")


# ===========================================================================
print("I4. and it always terminates: no map, however awkward, spins for ever")

for label, grid_in in (("one door", door(room(), "east", 70, 24)),
                       ("three doors", door(door(door(room(), "east", 70, 24),
                                                 "west", 70, 24), "north", 70, 24)),
                       ("shut in", pinched_map())):
    st = ExploreState()
    here = pose(centre, centre, 0.0)
    calls = 0
    for i in range(60):
        calls += 1
        t = next_target(Grid(grid_in), here, st, now=100.0 + i)
        if t is None:
            break
        if t.directive == DIRECTIVE_FRONTIER:
            here = pose(t.position.x, t.position.y, 0.0)   # the rover arrives
    check(f"{label}: the map runs out and the function says so ({calls} calls)",
          t is None and calls < 60, f"{calls} calls, last {t}")


# ===========================================================================
print("K. how far: the distance exponent, the goal cap, the exploration radius")

# A hall, 6 x 2 m, walled on three sides. Its east end opens onto 1 m of
# unknown (the far frontier: 38 cells, the whole width of the hall). Halfway
# along the north wall, a 0.50 m door into a 1.0 x 0.6 m closet (the near
# frontier: 8 cells). Heading is taken out of the picture (forward_bonus 0) so
# every score below is exactly gain / (1 + path) ** exponent.
#
# With `bay`, a 1.0 m opening in the south wall (x 2.0 .. 3.0 m) gives onto a
# 0.10 m strip of unknown, then 0.25 m of seen floor, then a 0.70 m unknown
# pocket closed by a wall. The pocket's frontier is looked at from that seen
# floor, which the rover can only reach across the unknown strip: a PROBE.
def hall(closet=True, bay=False):
    g = np.full((120, 220), UNKNOWN, dtype=np.int8)
    g[40:80, 20:140] = FREE                   # x 1.0 .. 7.0 m, y 2.0 .. 4.0 m
    g[39:81, 19] = OCCUPIED                   # west wall
    g[39, 19:161] = OCCUPIED                  # south wall, on past the open end
    g[80, 19:161] = OCCUPIED                  # north wall, likewise
    g[39:81, 160] = OCCUPIED                  # the far pocket's back wall, 1 m out
    if closet:
        g[80:93, 57] = OCCUPIED               # closet sides, x 2.85 and 3.90 m
        g[80:93, 78] = OCCUPIED
        g[93, 57:79] = OCCUPIED               # closet back, 0.6 m behind the door
        g[80, 63:73] = UNKNOWN                # the 0.50 m door, x 3.15 .. 3.65 m
    if bay:
        g[38:40, 40:60] = UNKNOWN             # the opening and the unknown strip, y 1.90 .. 2.00 m
        g[33:38, 40:60] = FREE                # seen floor beyond it, y 1.65 .. 1.90 m
        g[18:39, 39] = OCCUPIED               # bay sides, x 1.95 and 3.00 m
        g[18:39, 60] = OCCUPIED
        g[18, 39:61] = OCCUPIED               # its back wall, y 0.90 m
    return g


def flat(**kw):
    return Tuning(forward_bonus=0.0, **kw)


hall_cm = Grid(hall())
start = pose(w_of(50), w_of(60), 0.0)          # (2.53, 3.03): 1.2 m from the closet's
                                               # viewpoint, 4.5 m from the far end's

linear = next_target(hall_cm, start, ExploreState(), now=100.0,
                     tuning=flat(distance_exponent=1.0, max_goal_path_m=0.0))
squared = next_target(hall_cm, start, ExploreState(), now=100.0,
                      tuning=flat(distance_exponent=2.0, max_goal_path_m=0.0))
assert linear is not None and squared is not None
far_end = (linear.info_gain, linear.path_cost_m)
closet = (squared.info_gain, squared.path_cost_m)
for exp in (1.0, 2.0):
    print(f"     exponent {exp:g}: far end gain {far_end[0]:.4f} / (1 + {far_end[1]:.2f} m)^{exp:g} "
          f"= {far_end[0] / (1 + far_end[1]) ** exp:.5f}, "
          f"closet gain {closet[0]:.4f} / (1 + {closet[1]:.2f} m)^{exp:g} "
          f"= {closet[0] / (1 + closet[1]) ** exp:.5f}")
check("(setup) two clusters, both on seen floor, the far one the bigger",
      linear.n_clusters == 2 and not linear.probe and not squared.probe
      and linear.info_cells > squared.info_cells and far_end[0] > closet[0],
      f"{linear.info_cells} vs {squared.info_cells} cells, gain {far_end[0]:.4f} vs {closet[0]:.4f}")
check("(setup) the far end is 4.5 m of path away, the closet 1.2 m",
      abs(far_end[1] - 4.5) < 0.1 and abs(closet[1] - 1.2) < 0.1,
      f"{far_end[1]:.2f} m, {closet[1]:.2f} m")
check("exponent 1 (PR #2830's shape): the big far end wins",
      linear.position.x > w_of(130), f"x = {linear.position.x:.2f} m")
check("exponent 2: the small closet beside the rover wins",
      w_of(57) < squared.position.x < w_of(78) and squared.position.y > w_of(70),
      f"({squared.position.x:.2f}, {squared.position.y:.2f})")
check("the score handed back is gain / (1 + path) ** exponent, exactly",
      abs(linear.score - far_end[0] / (1 + far_end[1])) < 1e-12
      and abs(squared.score - closet[0] / (1 + closet[1]) ** 2) < 1e-12,
      f"{linear.score:.6f}, {squared.score:.6f}")
default = next_target(hall_cm, start, ExploreState(), now=100.0)
check("the defaults (exponent 2, cap 3 m) send the rover to the closet",
      default is not None and abs(default.path_cost_m - closet[1]) < 1e-9
      and DEFAULT_TUNING.distance_exponent == 2.0 and DEFAULT_TUNING.max_goal_path_m == 3.0,
      f"{default.path_cost_m:.2f} m" if default else "None")

capped = next_target(hall_cm, start, ExploreState(), now=100.0,
                     tuning=flat(distance_exponent=1.0, max_goal_path_m=3.0))
check("cap 3 m, exponent 1: the 4.5 m far end is not a candidate, the closet goes out",
      capped is not None and abs(capped.path_cost_m - closet[1]) < 1e-9
      and capped.n_beyond_cap == 1 and capped.beyond_cap is False,
      f"{capped.path_cost_m:.2f} m, {capped.n_beyond_cap} beyond" if capped else "None")
roomy = next_target(hall_cm, start, ExploreState(), now=100.0,
                    tuning=flat(distance_exponent=1.0, max_goal_path_m=5.0))
check("cap 5 m: the 4.5 m far end is a candidate again and wins",
      roomy is not None and abs(roomy.path_cost_m - far_end[1]) < 1e-9 and roomy.n_beyond_cap == 0)

# The closet mapped, the far end the only frontier left: the cap must not end
# the run (only None does, and only when nothing is left).
bare_hall = Grid(hall(closet=False))
only_far = next_target(bare_hall, start, ExploreState(), now=100.0)
check("nothing within the cap: the far end still goes out, flagged, the run goes on",
      only_far is not None and only_far.directive == DIRECTIVE_FRONTIER
      and only_far.beyond_cap is True and only_far.path_cost_m > 3.0,
      f"{only_far.path_cost_m:.2f} m, beyond_cap {only_far.beyond_cap}" if only_far else "None")
nearer = next_target(bare_hall, pose(w_of(90), w_of(60), 0.0), ExploreState(), now=100.0)
check("and once the rover is within 3 m of it, it is a plain candidate again",
      nearer is not None and nearer.path_cost_m <= 3.0 and nearer.beyond_cap is False,
      f"{nearer.path_cost_m:.2f} m" if nearer else "None")

# The cap against the probe penalty. In the hall with the bay and no closet,
# only the bay's probe lies within 3 m; the far end, safe, lies beyond. The
# rover faces the bay, so the heading bonus goes to the probe. The defaults
# throughout, except where probe_penalty is named.
bay_hall = Grid(hall(closet=False, bay=True))
facing_bay = pose(w_of(50), w_of(60), -math.pi / 2)
safe_far = next_target(bay_hall, facing_bay, ExploreState(), now=100.0)
no_penalty = next_target(bay_hall, facing_bay, ExploreState(), now=100.0,
                         tuning=Tuning(probe_penalty=1.0))
assert safe_far is not None and no_penalty is not None
penalised = no_penalty.score * DEFAULT_TUNING.probe_penalty
print(f"     probe {no_penalty.path_cost_m:.2f} m away: {no_penalty.score:.5f}, "
      f"x {DEFAULT_TUNING.probe_penalty:g} = {penalised:.5f}; "
      f"safe far end {safe_far.path_cost_m:.2f} m away: {safe_far.score:.5f}")
check("(setup) the bay's frontier is a probe within the cap, the far end safe beyond it",
      no_penalty.probe and no_penalty.path_cost_m < 3.0
      and not safe_far.probe and safe_far.path_cost_m > 3.0,
      f"probe {no_penalty.probe} at {no_penalty.path_cost_m:.2f} m, "
      f"far end probe {safe_far.probe} at {safe_far.path_cost_m:.2f} m")
check("a probe within 3 m, a safe frontier at 4.5 m: under the defaults the safe one goes out",
      safe_far.position.x > w_of(130) and safe_far.beyond_cap is True
      and safe_far.n_beyond_cap == 1,
      f"x = {safe_far.position.x:.2f} m, beyond_cap {safe_far.beyond_cap}, "
      f"{safe_far.n_beyond_cap} beyond")
check("the probe penalty decides it, not the cap: at probe_penalty 1 the probe wins",
      w_of(40) < no_penalty.position.x < w_of(60) and no_penalty.position.y < w_of(38)
      and penalised < safe_far.score < no_penalty.score,
      f"({no_penalty.position.x:.2f}, {no_penalty.position.y:.2f})")

# The closet back in: a safe frontier within the cap (1.2 m), the probe within
# it (1.7 m), and the far end, safe, beyond it (4.5 m).
both_hall = Grid(hall(closet=True, bay=True))
near_safe = next_target(both_hall, start, ExploreState(), now=100.0)
check("a safe frontier within 3 m, a safe one at 4.5 m: under the defaults the near one goes out",
      near_safe is not None and not near_safe.probe
      and w_of(57) < near_safe.position.x < w_of(78) and near_safe.position.y > w_of(70)
      and near_safe.path_cost_m < 3.0 and near_safe.beyond_cap is False
      and near_safe.n_beyond_cap == 1,
      f"({near_safe.position.x:.2f}, {near_safe.position.y:.2f}) {near_safe.path_cost_m:.2f} m"
      if near_safe else "None")
open_far = next_target(both_hall, start, ExploreState(), now=100.0,
                       tuning=flat(distance_exponent=1.0, max_goal_path_m=0.0))
held = next_target(both_hall, start, ExploreState(), now=100.0,
                   tuning=flat(distance_exponent=1.0, max_goal_path_m=3.0))
check("exponent 1: the far end wins with no cap; cap 3 m holds it back for the near safe one",
      open_far is not None and open_far.position.x > w_of(130)
      and held is not None and not held.probe and held.beyond_cap is False
      and w_of(57) < held.position.x < w_of(78) and held.position.y > w_of(70),
      f"no cap x = {open_far.position.x:.2f} m, cap 3 m "
      f"({held.position.x:.2f}, {held.position.y:.2f})" if open_far and held else "None")

# The radius. The run starts at (4.03, 2.48): the closet's viewpoint is 1.5 m
# from there, the far end's 3.0 m.
anchor_pose = pose(w_of(80), w_of(49), 0.0)
ax, ay = anchor_pose.position.x, anchor_pose.position.y
st = ExploreState()
wide = next_target(hall_cm, anchor_pose, ExploreState(), now=100.0,
                   tuning=flat(max_goal_path_m=0.0))
assert wide is not None
d_far = math.hypot(wide.position.x - ax, wide.position.y - ay)
inside = next_target(hall_cm, anchor_pose, st, now=100.0,
                     tuning=flat(max_goal_path_m=0.0, explore_radius_m=2.0))
d_near = (math.hypot(inside.position.x - ax, inside.position.y - ay)
          if inside is not None else float("nan"))
print(f"     from the anchor: far end's target {d_far:.2f} m, closet's {d_near:.2f} m")
check("(setup) with no radius the far end wins from there, its target 3.0 m from the start",
      wide.position.x > w_of(130) and abs(d_far - 3.0) < 0.05, f"{d_far:.2f} m")
check("radius 2 m: the target 3.0 m from the anchor is dropped, the one 1.5 m away kept",
      inside is not None and abs(d_near - 1.5) < 0.05 and inside.n_outside_radius == 1,
      f"{d_near:.2f} m, {getattr(inside, 'n_outside_radius', None)} outside")
check("the anchor is the first pose the run was given",
      st.anchor == (ax, ay), f"{st.anchor}")

# Later in the same run: the closet is mapped and the rover has driven up the
# hall, 1.5 m from the far end's target. The radius is still centred where the
# run began, so the far end stays out, and nothing is left inside it.
up_hall = pose(w_of(110), w_of(60), 0.0)
fresh = next_target(bare_hall, up_hall, ExploreState(), now=100.0,
                    tuning=flat(max_goal_path_m=0.0, explore_radius_m=2.0))
check("(setup) a run STARTING up the hall has the far end inside its 2 m",
      fresh is not None and fresh.position.x > w_of(130),
      f"x = {fresh.position.x:.2f} m" if fresh else "None")
ended = next_target(bare_hall, up_hall, st, now=101.0,
                    tuning=flat(max_goal_path_m=0.0, explore_radius_m=2.0))
check("only out-of-radius frontiers left -> None: exploration complete",
      ended is None, str(ended))
check("an end, not a back-off or a wait, and the anchor did not move",
      st.last_directive == "" and st.back_off_issued is False and st.anchor == (ax, ay),
      f"{st.last_directive!r}, {st.back_off_issued}, {st.anchor}")
unlimited = next_target(bare_hall, up_hall, st.copy(), now=102.0, tuning=flat(max_goal_path_m=0.0))
check("radius 0 is no limit: the same run, same anchor, goes on to the far end",
      unlimited is not None and unlimited.position.x > w_of(130),
      f"x = {unlimited.position.x:.2f} m" if unlimited else "None")


# ===========================================================================
print("G. purity: nothing but the state's own fields changes")

g = door(room(), "east", 70, 24)
grid_before = g.copy()
cm = Grid(g)
here = pose(centre, centre, 0.3)
pose_before = (here.position.x, here.position.y, here.orientation.z, here.orientation.w)

st = ExploreState(back_off_issued=False)
st.note_failed(1.0, 2.0, (here.position.x, here.position.y), cm)   # robot has not moved since
before = st.copy()
out = next_target(cm, here, st, now=60.0)

check("the costmap is not written to", np.array_equal(g, grid_before))
check("the pose is not written to",
      (here.position.x, here.position.y, here.orientation.z, here.orientation.w) == pose_before)
check("failed entries whose triggers have not fired are kept as they were",
      st.failed == before.failed, f"{st.failed}")
check("back_off_issued untouched when nothing was cornered",
      st.back_off_issued == before.back_off_issued)
check("heading was refreshed from the pose (0.3 rad)", abs(st.heading - 0.3) < 1e-6,
      f"{st.heading}")
check("visited / observed / targets_issued / last_directive / anchor are the only other changes",
      (len(st.visited), len(st.observed), st.targets_issued, st.last_directive, st.anchor)
      == (1, 1, 1, DIRECTIVE_FRONTIER, (here.position.x, here.position.y)))

reopened = ExploreState()
reopened.note_failed(1.0, 2.0, (here.position.x - 5.0, here.position.y), cm)   # failed 5 m from here
next_target(cm, here, reopened, now=61.0)
check("a failed entry whose rover has since moved a viewpoint away is pruned",
      reopened.failed == [], f"{reopened.failed}")

a = next_target(cm, here, ExploreState(), now=100.0)
b = next_target(cm, here, ExploreState(), now=100.0)
check("same map + same pose + same state -> same answer",
      a is not None and b is not None
      and (a.position.x, a.position.y) == (b.position.x, b.position.y)
      and a.score == b.score)

tuned = next_target(cm, here, ExploreState(), now=100.0,
                    tuning=Tuning(forward_bonus=0.0, revisit_radius_m=0.1))
check("tuning is honoured and is not global state",
      tuned is not None
      and next_target(cm, here, ExploreState(), now=100.0).score == a.score)

noclock = ExploreState()
noclock.note_failed(gx, gy, (here.position.x, here.position.y), cm)
check("no clock is read when `now` is given: a 1970 timestamp behaves",
      next_target(cm, here, noclock, now=-1e9) is not None)


# ===========================================================================
print("J. the loop: a verdict owed to the PREVIOUS goal never excludes the new one")
# ===========================================================================
#
# The only group that needs dimOS: it drives Explorer2._run_exploration_loop
# itself, with next_target stubbed (A-I own the decisions, this owns the
# plumbing) and a scripted planner. goal_reached is a bare Bool with no goal
# identity in it, so the cancel of a goal that timed out unanswered could be
# credited to the goal published a millisecond earlier - an exclusion on ground
# the rover never drove to, killing every cluster within 0.6 m of it.

Explorer2 = getattr(_explorer2_module, "Explorer2", None)

if Explorer2 is None:
    print("  -- skipped: no dimOS here, the module class does not exist")
else:
    GOAL_TIMEOUT_S = 0.4
    STUB_GOAL = (2.0, 3.0)      # metres, the only target the stub hands out
    START = (1.0, 3.0)          # 1.00 m from it
    STEP_M = 0.4                # what the rover covers per goal: > GOAL_PROGRESS_M

    FRONTIER_LEFT = [True]      # flipped by `explored()`: next_target then returns None
    STUB_BEYOND_CAP = [False]   # what the stub's target says about the goal cap

    def stub_target(costmap, odom, state, **kwargs):
        if not FRONTIER_LEFT[0]:
            return None         # "exploration complete: no reachable frontier left"
        t = PoseStamped(ts=0.0, frame_id="world")
        t.position.x, t.position.y = STUB_GOAL
        t.beyond_cap = STUB_BEYOND_CAP[0]
        return t

    class DoneOut:
        """The `explore_done` Out, recording what the module publishes on it.

        `transport` None reproduces a module built outside a blueprint - the
        same guard `bump` uses."""

        def __init__(self, wired=True):
            self.transport = object() if wired else None
            self.sent = []

        def publish(self, msg):
            self.sent.append(bool(getattr(msg, "data", False)))

    class Planner:
        """The goal_request Out plus the planner behind it: `script` says what
        the planner does the instant a goal is published - the worst case for
        the race."""

        transport = object()

        def __init__(self, script):
            self.script = script
            self.goals = []
            self.published_at = []
            self.explorer = None

        def publish(self, goal):
            self.goals.append((round(float(goal.position.x), 2), round(float(goal.position.y), 2)))
            self.published_at.append(time.perf_counter())
            act = self.script.get(len(self.goals))
            if act is not None:
                act(self.explorer)

    def gave_up(ex):
        """The planner publishes goal_reached=False (no path / goal abandoned)."""
        ex._on_goal_reached(type("M", (), {"data": False})())

    def drove(ex):
        """The rover covers STEP_M toward the goal; the planner says nothing."""
        o = ex.latest_odometry
        ex.latest_odometry = pose(float(o.position.x) + STEP_M, float(o.position.y))

    def last(ex):
        ex.exploration_active = False

    def explored(ex):
        """The map is fully explored: the NEXT next_target returns None."""
        FRONTIER_LEFT[0] = False

    def acts(*fns):
        return lambda ex: [f(ex) for f in fns]

    def run_loop(script, wired=True, tuning=DEFAULT_TUNING):
        FRONTIER_LEFT[0] = True
        ex = object.__new__(Explorer2)
        ex._state = ExploreState()
        ex._last_goal = None
        ex._last_goal_ok = None
        ex._late_answer_owed = False
        ex._goal_published_t = 0.0
        ex._tuning = tuning
        ex.goal_reached_event = threading.Event()
        ex.stop_event = threading.Event()
        ex.exploration_active = True
        ex.latest_costmap = Grid(door(room(), "east", 70, 24))
        ex.latest_odometry = pose(*START)
        ex.config = type("Cfg", (), {"goal_timeout": GOAL_TIMEOUT_S})()
        ex.bump = type("Bump", (), {"transport": None})()
        ex.explore_done = DoneOut(wired)
        ex.goal_request = Planner(script)
        ex.goal_request.explorer = ex
        real_next_target = _explorer2_module.next_target
        _explorer2_module.next_target = stub_target
        try:
            ex._run_exploration_loop()
        finally:
            _explorer2_module.next_target = real_next_target
        return ex, ex.goal_request, time.perf_counter()

    # Goal 1 times out while the rover is still closing on it (0.40 m of 1.00 m),
    # so it is re-decided, not excluded - and the planner still owes it a verdict.
    # That verdict lands the instant goal 2 goes out, while the rover drives to
    # goal 2 exactly as the planner plans for it.
    ex, planner, ended = run_loop({1: drove, 2: acts(gave_up, drove, last)})
    check("two goals went out", planner.goals == [STUB_GOAL, STUB_GOAL], f"{planner.goals}")
    check("the late verdict excludes nothing: the rover never failed at (2.00, 3.00)",
          ex._state.failed == [], f"{ex._state.failed}")
    check("the goal in flight keeps its own 0.4 s timeout instead of an instant verdict",
          ended - planner.published_at[1] >= GOAL_TIMEOUT_S,
          f"{ended - planner.published_at[1]:.2f} s")
    check("and the timeout then judges on the gap closed: 0.40 m of 1.00 m, so no exclusion",
          ex._state.failed == [] and abs(float(ex.latest_odometry.position.x)
                                         - (START[0] + 2 * STEP_M)) < 1e-6)

    # The verdict owed to goal 1 is only swallowed inside LATE_VERDICT_S of goal 2's
    # publish. A verdict landing after that window is goal 2's own (its "no path"
    # after a replanning): credited, and the loop wakes at once instead of
    # sleeping the whole goal_timeout (a 45 s pause on the floor otherwise).
    _explorer2_module.LATE_VERDICT_S = 0.05
    def gave_up_later(ex):
        threading.Timer(0.15, lambda: gave_up(ex)).start()
    ex, planner, ended = run_loop({1: drove, 2: acts(gave_up_later, last)})
    _explorer2_module.LATE_VERDICT_S = 1.0
    check("a verdict past the late window is credited to the goal in flight",
          [(round(f[0], 2), round(f[1], 2)) for f in ex._state.failed] == [STUB_GOAL],
          f"{ex._state.failed}")
    check("and wakes the loop at once (0.15 s, not the 0.4 s timeout)",
          ended - planner.published_at[1] < GOAL_TIMEOUT_S - 0.05,
          f"{ended - planner.published_at[1]:.2f} s")

    # Nothing is owed on the first goal of a run: a no-path verdict on it is the
    # planner talking about THAT goal, and it must still bite at once.
    ex, planner, ended = run_loop({1: acts(gave_up, last)})
    check("a no-path verdict on the first goal is still credited",
          [(round(f[0], 2), round(f[1], 2)) for f in ex._state.failed] == [STUB_GOAL],
          f"{ex._state.failed}")
    check("and it still wakes the loop at once (no 0.4 s slept for nothing)",
          ended - planner.published_at[0] < GOAL_TIMEOUT_S,
          f"{ended - planner.published_at[0]:.3f} s")

    # An ANSWERED goal owes nothing either: two verdicts in a row are both real.
    ex, planner, ended = run_loop({1: gave_up, 2: acts(gave_up, last)})
    check("two answered goals in a row are both credited (the swallow is one deep)",
          len(ex._state.failed) == 2, f"{ex._state.failed}")

    # -- the end of the mission, SAID on the bus ------------------------------
    # The base loop only sets exploration_active False, logs one line and
    # breaks: nothing crosses the process boundary. explore_done is what tells
    # the other processes (the cockpit) that the run is over.
    ex, planner, ended = run_loop({1: acts(explored, gave_up)})
    check("the loop ends on 'no reachable frontier left'", ex.exploration_active is False)
    check("and explore_done=True goes out exactly once",
          ex.explore_done.sent == [True], f"{ex.explore_done.sent}")
    ex, planner, ended = run_loop({1: acts(gave_up, last)})
    check("a run stopped from outside publishes NO explore_done "
          "(that end already crosses on stop_explore_cmd)",
          ex.explore_done.sent == [], f"{ex.explore_done.sent}")
    ex, planner, ended = run_loop({1: acts(explored, gave_up)}, wired=False)
    check("an unwired Out is never published to, and does not crash the loop",
          ex.explore_done.sent == [] and ex.exploration_active is False)

    # -- exp, cap and radius: said once, when exploration starts ----------
    class Said:
        """Stands in for the module's logger and keeps every line."""

        def __init__(self):
            self.lines = []

        def info(self, msg, *args, **kwargs):
            self.lines.append(str(msg))

        warning = error = debug = info

    real_logger = _explorer2_module.logger
    said = Said()
    _explorer2_module.logger = said
    try:
        ex, planner, ended = run_loop({1: drove, 2: acts(drove, last)})
        said_default = list(said.lines)
        said.lines.clear()
        ex, planner_r, ended = run_loop({1: acts(explored, gave_up)},
                                        tuning=Tuning(explore_radius_m=4.0))
        said_radius = list(said.lines)
        said.lines.clear()
        STUB_BEYOND_CAP[0] = True
        ex, planner_c, ended = run_loop({1: acts(explored, gave_up)})
        said_cap = list(said.lines)
    finally:
        STUB_BEYOND_CAP[0] = False
        _explorer2_module.logger = real_logger
    starts = [line for line in said_default if "exp 2, cap 3 m, radius 0 m" in line]
    check("one line at the start says 'exp 2, cap 3 m, radius 0 m', and no goal line repeats it",
          len(planner.goals) == 2 and len(starts) == 1 and said_default[0] == starts[0]
          and sum("exp " in line for line in said_default) == 1,
          f"{len(planner.goals)} goals; {said_default[:1]}")
    check("a radius run says its radius, and its end says 'within 4 m of the start'",
          any("radius 4 m" in line for line in said_radius[:1])
          and any("no reachable frontier left within 4 m of the start" in line for line in said_radius),
          f"{said_radius}")
    cap_lines = [line for line in said_cap if line.startswith("goal ")]
    check("a goal beyond the cap says so on its line, and a goal within it does not",
          len(cap_lines) == 1 and "(beyond the cap: nothing safe within it)" in cap_lines[0]
          and not any("beyond the cap" in line for line in said_default),
          f"{cap_lines}")


# ===========================================================================
print("L. the blueprint: three env knobs -> the Explorer2 config")
# ===========================================================================

if Explorer2 is None:
    print("  -- skipped: no dimOS here, the blueprint does not import")
else:
    import os

    import moss_dimos.blueprints as mb
    from moss_dimos.explorer2 import Explorer2Config

    check("unset: nothing passed, the explorer keeps its defaults", mb.explorer_env({}) == {})
    check("'1', '0', ' 4 ' -> exponent 1.0, cap 0.0 m (no limit), radius 4.0 m",
          mb.explorer_env({"MOSS_EXPLORE_DISTANCE_EXP": "1", "MOSS_GOAL_MAX_M": "0",
                           "MOSS_EXPLORE_RADIUS_M": " 4 "})
          == {"distance_exponent": 1.0, "max_goal_path_m": 0.0, "explore_radius_m": 4.0})
    refused = mb.explorer_env({"MOSS_EXPLORE_DISTANCE_EXP": "two", "MOSS_GOAL_MAX_M": "-1",
                               "MOSS_EXPLORE_RADIUS_M": "inf"})
    check("'two', '-1', 'inf' are refused (warned): the defaults stay", refused == {}, f"{refused}")
    steep = [mb.explorer_env({"MOSS_EXPLORE_DISTANCE_EXP": raw}) for raw in ("250", "7", "6")]
    check("an exponent of 250 or 7 is refused (warned), 6 is the largest taken",
          steep == [{}, {}, {"distance_exponent": 6.0}], f"{steep}")
    try:
        overflow = repr(31.0 ** 250)
    except OverflowError as exc:
        overflow = f"OverflowError {exc}"
    check("(why) a float power of 250 raises instead of giving inf, at 6 it is finite",
          overflow.startswith("OverflowError") and math.isfinite(31.0 ** 6),
          f"31.0 ** 250: {overflow}")
    check("the config's defaults are the Tuning's: exponent 2, cap 3 m, radius 0",
          (Explorer2Config.model_fields["distance_exponent"].default,
           Explorer2Config.model_fields["max_goal_path_m"].default,
           Explorer2Config.model_fields["explore_radius_m"].default)
          == (DEFAULT_TUNING.distance_exponent, DEFAULT_TUNING.max_goal_path_m,
              DEFAULT_TUNING.explore_radius_m) == (2.0, 3.0, 0.0))

    knobs = {"MOSS_CAMERA": "0", "MOSS_RECORD": "0", "MOSS_WRIST": "0",
             "MOSS_EXPLORE_DISTANCE_EXP": "1.5", "MOSS_GOAL_MAX_M": "2.5", "MOSS_EXPLORE_RADIUS_M": "4"}
    saved = {k: os.environ.get(k) for k in knobs}
    os.environ.update(knobs)
    try:
        built = mb.build_moss_explore()
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    kw = next(b for b in built.active_blueprints if b.module.__name__ == "Explorer2").kwargs
    check("moss-explore built with the three set: Explorer2 gets 1.5, 2.5 m, 4 m",
          (kw.get("distance_exponent"), kw.get("max_goal_path_m"), kw.get("explore_radius_m"))
          == (1.5, 2.5, 4.0), f"{kw}")
    cfg = Explorer2Config(**kw)
    check("and its config holds them", (cfg.distance_exponent, cfg.max_goal_path_m,
                                        cfg.explore_radius_m) == (1.5, 2.5, 4.0))


print(f"{OK} OK, {KO} KO")
if not KO:
    print("TEST PASSED")
sys.exit(1 if KO else 0)
