"""Cold bench for costmap2d: the scored grid and MossCostMap, written as numbers.

Known input -> known output, in metres and cells:
  1. a table leg at (1.0, 0) seen from two places -> occupied; the floor along
     the ray (0.3..0.9 m) -> free; nothing else touched
  2. a ramp: 50 hits from the SAME spot -> never occupied (one viewpoint)
  3. a chair: occupied, then lidar revolutions see through it -> free
  4. a ceiling of reinforcement: 30 viewpoints -> score 10, then bounded
     unlearning (never a wall for a day)
  5. a parked rover: a wall raised to HIT_CAP by 12 viewpoints, then
     revolutions from the spot it is parked on and 0.5 m of creep -> still
     HIT_CAP, still occupancy 100 (the viewpoint rule caps the increase, it
     never lowers a score)
  6. body_clear: the MOSS footprint (0.20 x 0.16 m half extents) clears 63
     cells at the origin; MOSS_BODY_CLEAR_M moves it at call time
  7. the lidar tag: a float32 point at exactly the profile's lidar height
     (0.20 m) is a lidar return; 0.37 m, 0.2005 m and the floor are not;
     MOSS_LIDAR_HEIGHT_M moves the tag at call time; a camera obstacle maps
     after two viewpoints (even when the second lands on a carving
     revolution), survives the rays while the camera sees it, and is freed
     by the camera's floor samples once taken away
  8. the mission gate: 20 revolutions and a passer-by on a PARKED rover ->
     0 cells; 6 cm of displacement -> the map opens and the room's wall at
     2.5 m is mapped; "exploration complete" and "exploration stopped" ->
     the map keeps writing (lidar is MOSS's only obstacle source)
  9. the explore order: parked revolutions write nothing until explore_cmd,
     then the same parked revolutions carve free floor up to the wall
 10. what goes out: one OccupancyGrid every 5 revolutions on global_costmap,
     0.05 m cells, frame `world`, the wall at 2.5 m = 100 in the grid
 11. the mission signals land on the topics the rest of the stack uses: one
     Bool class for explore_cmd / explore_done / stop_explore_cmd across
     costmap2d, explorer2 and tools/explore_ctl.py
 12. the cost of one revolution (400 rays, 0.5-12 m) stays under 40 ms
 13. a camera obstacle seen from ONE parked spot for 20 frames, with the
     lidar rays crossing its cell, stays below OCCUPIED_AT (never a wall); a
     camera cell at FREE_FLOOR hit 5 frames from one spot stays below 0
 14. a passer-by's legs: a camera wall (two viewpoints) in the camera's view,
     no longer seen, under lidar rays through its cell -> freed within 10
     revolutions (2 at score 2, 10 at score 6)
 15. a camera wall the camera KEEPS seeing from the same spot, rays carving
     its cell every other revolution -> a wall for all 20 revolutions
 16. the knobs, read at import: MOSS_HIT_CAP=4 unlearns a leg seen from 6
     viewpoints in fewer revolutions than 10; MOSS_FREE_FLOOR=-1 lets long-seen
     floor become a wall in fewer hits; MOSS_RAY_MAX_M=6 carves a leg at 4.5 m
     that the default 4 m never reaches; bad values fall back or clamp
 17. the rays carve the camera layer only inside the camera's view (87 deg,
     3 m): a table top (320 cells, a wall from 3 viewpoints) stays a wall
     while the rover turns its back on it and for 20 revolutions facing away,
     rays through every cell; legs at 35 deg off the heading are freed while
     in view, kept while behind the rover; MossCostMap passes the odom heading
 18. body_clear resets camera_wall: a camera wall the body drives over is
     FREE_FLOOR and unflagged, and same-spot camera hits never make it a wall
     again (10 frames right away, and 10 after new viewpoints lift it to
     OCCUPIED_AT - 1)

Needs dimOS (the messages are real).

Run:  PYTHONPATH=. .venv/bin/python benches/bench_costmap2d_cold.py
"""

import asyncio
import copy
import importlib
import importlib.util
import math
import os
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

# MOSS's own numbers, whatever the shell exported
for _var in [v for v in os.environ if v.startswith("MOSS_")]:
    del os.environ[_var]
# the checks below pin the cautious rules (a wall from two places, viewpoints 10 cm apart, one grid per
# five revolutions); the module defaults are the fast ones (first sighting counts, 5 cm, every two),
# checked in a subprocess by test_the_fast_defaults
os.environ["MOSS_OCCUPIED_AT"] = "2"
os.environ["MOSS_NEW_VIEWPOINT_M"] = "0.10"
os.environ["MOSS_PUBLISH_EVERY"] = "5"

from dimos_lcm.std_msgs import Bool  # noqa: E402

from moss_dimos import costmap2d as cmod  # noqa: E402
from moss_dimos.robot_profile import current as _current_profile
from moss_dimos.costmap2d import (  # noqa: E402
    FREE_FLOOR, HIT_CAP, MISSION_START_M, OCCUPIED_AT, PUBLISH_EVERY, RAY_EVERY, RESOLUTION_M,
    CONTACT_BAND_M, MossCostMap, ScoredGrid, lidar_returns,
)
from moss_dimos.robot_profile import MOSS  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
LEG = np.array([[1.0, 0.0]])
LIDAR_Z = MOSS.lidar_height_m          # 0.20 m: where lidar_odometry lifts every return


def fresh() -> ScoredGrid:
    return ScoredGrid(span_m=6.0)


def _cloud(pts: np.ndarray):
    """A world cloud on the wire, float32 like the real one."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    return PointCloud2.from_numpy(pts.astype(np.float32), frame_id="world", timestamp=time.time())


def _lidar_cloud(xy: np.ndarray, z: float = LIDAR_Z) -> np.ndarray:
    """(x, y) hits lifted to the lidar's own tag height (see lidar_returns)."""
    return np.column_stack([xy, np.full(len(xy), np.float32(z))])


def _odom(x: float, y: float, yaw: float = 0.0):
    """A pose on the `odom` stream."""
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    p = PoseStamped(frame_id="world", ts=time.time())
    p.position.x, p.position.y, p.position.z = x, y, 0.0
    p.orientation.w, p.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
    return p


class Sink:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class LogSpy:
    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **kwargs):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = exception = _record


def costmap() -> MossCostMap:
    """A real MossCostMap; its Out is a Sink so the bench reads what it publishes."""
    m = MossCostMap()
    m.global_costmap = Sink()
    MODULES.append(m)
    return m


MODULES: list = []
ANG = np.radians(np.arange(0, 360, 2.0))
ROOM = np.stack([2.5 * np.cos(ANG), 2.5 * np.sin(ANG)], 1)    # a 5 m round room


def test_leg_needs_two_viewpoints_and_ray_is_free() -> None:
    """An obstacle is REAL when seen from two viewpoints: a passer-by hammered
    from one parked spot must never become a wall."""
    g = fresh()
    for _ in range(10):
        g.lidar_revolution(LEG, (0.0, 0.0))
    assert g.value_at(1.0, 0.0) == 0, "any number of hits from ONE spot: not a wall"
    g.lidar_revolution(LEG, (0.0, 0.15))          # second viewpoint (>= NEW_VIEWPOINT_M)
    assert g.value_at(1.0, 0.0) == 100, "one hit from a SECOND viewpoint: obstacle"
    for x in (0.3, 0.5, 0.7, 0.9):
        assert g.value_at(x, 0.0) == 0, f"floor along the ray at {x} m must be free"
    assert g.value_at(1.3, 0.0) == -1, "behind the leg: never seen"
    print("  10 hits one spot -> free; +1 hit second viewpoint -> occupied; ray -> free")


def test_ramp_hit_from_one_spot_never_becomes_a_wall() -> None:
    g = fresh()
    for _ in range(50):
        g.lidar_revolution(LEG, (0.0, 0.0))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.value_at(1.0, 0.0) == 0 and g.lidar[gy[0], gx[0]] == 1, "same spot forever: capped BELOW occupied"
    g.lidar_revolution(LEG, (0.0, 0.15))
    assert g.value_at(1.0, 0.0) == 100, "the second viewpoint makes it a wall"
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
    assert n <= 4, n
    print(f"  50 hits from the same spot -> score 2, forgotten after {n} clear revolutions (never a wall)")


def test_chair_moved_is_forgotten_by_lidar_rays() -> None:
    g = fresh()
    for i in range(3):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    assert g.value_at(1.0, 0.0) == 100
    far = np.array([[3.0, 0.0]])                      # the chair is gone: the ray now reaches the wall at 3 m
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(far, (0.0, 0.0)); n += 1
        assert n < 20
    assert g.value_at(1.0, 0.0) == 0
    print(f"  chair (score 3) -> free after {n} revolutions seeing through it")


def test_reinforcement_is_capped_and_unlearning_bounded() -> None:
    g = fresh()
    for i in range(30):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.lidar[gy[0], gx[0]] == HIT_CAP
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
    assert n <= 2 * HIT_CAP and g.lidar[gy[0], gx[0]] >= FREE_FLOOR
    print(f"  30 viewpoints -> score capped at {HIT_CAP}; forgotten after {n} clear revolutions (bounded)")


def test_parked_rover_never_erases_an_established_wall() -> None:
    """The viewpoint rule caps the INCREASE, not the value: a hit from a spot
    the cell already knows adds nothing and takes nothing away. Capping the
    value would collapse a wall built by 12 approach viewpoints to 1 on the
    next revolution taken from a parked rover."""
    g = fresh()
    for i in range(12):                                # approach: 12 viewpoints, 0.15 m apart
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.lidar[gy[0], gx[0]] == HIT_CAP and g.value_at(1.0, 0.0) == 100, "12 viewpoints -> a wall"
    parked = (0.0, 0.15 * 11)                          # the rover stops where it last looked from
    g.lidar_revolution(LEG, parked)
    assert g.lidar[gy[0], gx[0]] == HIT_CAP, "a same-viewpoint hit adds nothing - and takes nothing"
    assert g.value_at(1.0, 0.0) == 100, "the wall in front of a parked rover stays a wall"
    # driving slowly is the same case: 0.2 m/s at 10 Hz = 0.02 m per
    # revolution, so 4 revolutions out of 5 come from a viewpoint the cell knows
    for k in range(1, 26):                             # 0.5 m of creep
        g.lidar_revolution(LEG, (0.02 * k, parked[1]))
        assert g.value_at(1.0, 0.0) == 100, f"wall published FREE while creeping, revolution {k}"
    assert g.lidar[gy[0], gx[0]] == HIT_CAP
    print(f"  wall at 1.0 m, 12 viewpoints -> score {HIT_CAP}; parked + 1 revolution and 0.5 m of "
          f"creep -> still {HIT_CAP}, occupancy 100")


def test_body_clear_is_the_moss_footprint() -> None:
    """body_clear reads the profile at call time: MOSS 0.20 x 0.16 m half
    extents -> 9 x 7 cells of 5 cm at the origin (63), every one free and seen."""
    g = fresh()
    gx, gy = g.cell(np.array([0.10]), np.array([0.05]))
    g.lidar[gy[0], gx[0]] = HIT_CAP                     # a ghost under the body
    g.body_clear((0.0, 0.0, 0.0))
    ys, xs = np.nonzero(g.seen)
    x0, x1 = g.ox + xs.min() * g.res, g.ox + (xs.max() + 1) * g.res
    y0, y1 = g.oy + ys.min() * g.res, g.oy + (ys.max() + 1) * g.res
    assert int(g.seen.sum()) == 63, int(g.seen.sum())
    assert abs(x0 + 0.25) < 1e-9 and abs(x1 - 0.20) < 1e-9 and abs(y0 + 0.20) < 1e-9 and abs(y1 - 0.15) < 1e-9, (x0, x1, y0, y1)
    assert g.lidar[gy[0], gx[0]] == FREE_FLOOR and g.value_at(0.10, 0.05) == 0, "the ghost under the body is cleared"
    os.environ["MOSS_BODY_CLEAR_M"] = "0.30 0.20"
    try:
        h = fresh()
        h.body_clear((0.0, 0.0, 0.0))
        wide = int(h.seen.sum())
    finally:
        del os.environ["MOSS_BODY_CLEAR_M"]
    assert wide == 117, wide
    r = fresh()
    r.body_clear((1.0, 0.0, math.pi / 2))               # turned 90 deg: the footprint turns too
    ys, xs = np.nonzero(r.seen)
    assert int(r.seen.sum()) == 63 and xs.max() - xs.min() + 1 == 7 and ys.max() - ys.min() + 1 == 9, \
        (int(r.seen.sum()), xs.max() - xs.min() + 1, ys.max() - ys.min() + 1)
    assert abs(r.ox + xs.min() * r.res - 0.85) < 1e-9 and abs(r.oy + ys.min() * r.res + 0.25) < 1e-9
    print(f"  MOSS footprint: 63 cells over [-0.25, 0.20) x [-0.20, 0.15) m; MOSS_BODY_CLEAR_M='0.30 0.20' "
          f"-> {wide} cells; turned 90 deg at (1, 0) -> 7 x 9 cells over [0.85, 1.20) x [-0.25, 0.20) m")


def test_the_lidar_tag_is_one_exact_height() -> None:
    """lidar_odometry lifts every return to exactly lidar_height_m; anything
    else on the cloud (a depth camera's points, at their true height) is not
    a lidar return, even millimetres away from the scan plane - it is a camera
    obstacle, hit without a ray."""
    cloud = np.array([[1.0, 0.0, np.float32(0.20)], [2.0, 0.0, 0.2005], [3.0, 0.0, 0.37],
                      [4.0, 0.0, 0.0]], dtype=np.float32).astype(np.float64)
    assert list(lidar_returns(cloud)) == [True, False, False, False], list(lidar_returns(cloud))
    os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.37"
    try:
        moved = list(lidar_returns(cloud))
    finally:
        del os.environ["MOSS_LIDAR_HEIGHT_M"]
    assert moved == [False, False, True, False], moved
    # through the module: one cloud with 180 wall returns at 0.20 m and a
    # point 2 mm above the scan plane - only the wall is mapped
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    stray = np.array([[1.0, 0.5, LIDAR_Z + 0.002]])
    for y in (0.0, 0.15):
        asyncio.run(m.handle_odom(_odom(0.0, y)))
        asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), stray]))))
    assert m._grid.value_at(2.5, 0.0) == 100, "the wall is mapped"
    assert m._grid.value_at(1.0, 0.5) == 100, "the off-plane point is a camera obstacle: mapped after two viewpoints"
    n = m._revolutions
    asyncio.run(m.handle_lidar(_cloud(stray)))
    assert m._revolutions == n, "a cloud with no lidar return is not a revolution"
    # a camera obstacle from ONE viewpoint stays below OCCUPIED_AT, like a lidar hit
    m2 = costmap()
    asyncio.run(m2.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m2.handle_explore_cmd(Bool(data=True)))
    low = np.array([[1.5, -0.4, 0.05]])          # a shoe: 5 cm high, invisible to the scan plane
    for _ in range(3):
        asyncio.run(m2.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), low]))))
    assert m2._grid.value_at(1.5, -0.4) != 100, "one parked viewpoint never makes a camera obstacle a wall"
    asyncio.run(m2.handle_odom(_odom(0.0, 0.15)))
    asyncio.run(m2.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), low]))))
    assert m2._grid.value_at(1.5, -0.4) == 100, "the second viewpoint makes the shoe a wall"
    # the lidar sees the wall behind the shoe: its rays cross the shoe's cell every other
    # revolution, and the shoe the camera still sees survives them
    for _ in range(6):
        asyncio.run(m2.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), low]))))
        assert m2._grid.value_at(1.5, -0.4) == 100, "the shoe the camera still sees survives the rays"
    # the same rays without the shoe in the cloud do cross its cell: two revolutions free it
    probe = copy.deepcopy(m2._grid)
    for _ in range(2):
        probe.lidar_revolution(ROOM, (0.0, 0.15), yaw=0.0)
    assert probe.value_at(1.5, -0.4) != 100, "the rays cross the shoe's cell (so the survival above is the camera's)"
    # the shoe is taken away: the camera sees bare floor there -> freed after two floor samples
    floor = np.array([[1.5, -0.4, 0.0], [1.52, -0.42, 0.0]])
    for _ in range(2):
        asyncio.run(m2.handle_camera_floor(_cloud(floor)))
    assert m2._grid.value_at(1.5, -0.4) != 100, "bare floor seen by the camera frees the shoe's cell"
    # the camera point carves no ray: the floor between the rover and the shoe is not marked free by it
    print("  float32 0.20 m -> lidar; 0.2005 / 0.37 / 0.0 -> camera; MOSS_LIDAR_HEIGHT_M=0.37 moves the tag; "
          "a camera obstacle maps after two viewpoints (a 5 cm shoe), never from one parked spot; "
          "it survives the rays while seen, and the camera's floor frees it")


def test_the_map_writes_only_during_the_mission() -> None:
    """A rover parked with a spinning lidar must not write: it thickens the
    obstacles around its one viewpoint and records whoever walks past.

    Known input -> known output, in metres and in cells: revolutions injected
    while parked -> 0 cells; 6 cm of displacement -> cells written; the end of
    the exploration -> the map keeps writing."""
    spy, real = LogSpy(), cmod.logger
    cmod.logger = spy
    try:
        m = costmap()
        assert m._mission_frozen and m._write_frozen(), "the pre-mission gate is closed by default"
        passer_by = np.array([[0.8, 0.0]])                             # somebody standing 0.8 m away
        for _ in range(20):                                            # parked at the first pose
            asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
            asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(np.vstack([ROOM, passer_by])))))
        assert m._grid is not None, "the grid is built on the first odom"
        assert (m._grid.ox, m._grid.oy) == (-12.0, -12.0), "24 m grid centred on the first pose"
        parked_cells = int(m._grid.seen.sum())
        assert parked_cells == 0, f"a parked rover wrote {parked_cells} cells before departing"
        assert m._grid.value_at(0.8, 0.0) == -1, "the passer-by is not in the map"
        assert not m.global_costmap.msgs, "nothing is published while the gate is closed"

        asyncio.run(m.handle_odom(_odom(0.04, 0.0)))
        assert m._mission_frozen, "4 cm (< 5 cm) does not open the map"
        asyncio.run(m.handle_odom(_odom(0.06, 0.0)))
        assert not m._mission_frozen, "6 cm from the first pose (>= 5 cm) opens the map"
        assert any(line.startswith("costmap: the rover left its boot pose (6 cm)") for line in spy.lines), spy.lines
        for k in range(6):                                             # driving on, two viewpoints
            asyncio.run(m.handle_odom(_odom(0.06 + 0.04 * k, 0.0)))
            asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
        driving_cells = int(m._grid.seen.sum())
        assert driving_cells > 0, "on the mission, the revolutions land in the map"
        assert m._grid.value_at(2.5, 0.0) == 100, "the wall of the room is mapped while driving"

        asyncio.run(m.handle_explore_done(Bool(data=True)))
        asyncio.run(m.handle_stop_explore_cmd(Bool(data=True)))
        assert not m._write_frozen(), "the end of the exploration does not close the map"
        assert "costmap: exploration complete - map writing continues" in spy.lines
        assert "costmap: exploration stopped - map writing continues" in spy.lines
        for k in range(6):
            asyncio.run(m.handle_odom(_odom(1.0, 0.3 * k)))
            asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM + [1.0, 0.0]))))
        after = int(m._grid.seen.sum())
        assert after > driving_cells, "the map keeps writing after the explorer stopped"
        assert m._grid.value_at(3.5, 0.0) == 100, "a wall seen after the stop is mapped"

        n = costmap()
        asyncio.run(n.handle_odom(_odom(0.0, 0.0)))
        asyncio.run(n.handle_explore_done(Bool(data=False)))
        asyncio.run(n.handle_stop_explore_cmd(Bool(data=False)))
        asyncio.run(n.handle_explore_cmd(Bool(data=False)))
        assert n._mission_frozen, "only data=True counts: Bool(False) opens nothing"
        asyncio.run(n.handle_stop_explore_cmd(Bool(data=True)))
        asyncio.run(n.handle_odom(_odom(1.0, 0.0)))
        assert not n._write_frozen() and int(n._grid.seen.sum()) > 0, (
            "a stop BEFORE the map opened: the 5 cm gate still opens it, and body_clear writes")
    finally:
        cmod.logger = real
    print(f"  parked: 20 revolutions + a passer-by -> {parked_cells} cells; after 6 cm -> {driving_cells} cells "
          f"and the wall at 2.5 m mapped; after 'exploration complete' + 'stopped' -> {after} cells, still writing")


def test_explore_order_opens_the_map_before_any_motion() -> None:
    """Displacement-only opening deadlocks a fresh map (no cell -> no frontier
    -> no goal -> no motion -> never 5 cm). The explore order opens it.

    Known input -> known output: parked revolutions -> 0 cells; explore_cmd ->
    the SAME parked revolutions carve free floor up to the wall at 2.5 m."""
    spy, real = LogSpy(), cmod.logger
    cmod.logger = spy
    try:
        m = costmap()
        for _ in range(5):
            asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
            asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
        assert int(m._grid.seen.sum()) == 0, "still closed before the order"
        asyncio.run(m.handle_explore_cmd(Bool(data=True)))
        assert not m._mission_frozen, "the explore ORDER opens the map, no motion needed"
        assert spy.lines.count("costmap: explore_cmd received - map writing OPEN for the mission") == 1, spy.lines
        for _ in range(3):
            asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
            asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
        assert int(m._grid.seen.sum()) > 0, "the parked revolutions now land in the map"
        # One viewpoint never makes a wall - what the explorer NEEDS is free
        # floor along the rays: its first frontier before one wheel turn.
        assert m._grid.value_at(1.25, 0.0) == 0, "the ray to the wall carved free floor from the start pose"
        assert m._grid.value_at(2.5, 0.0) == 0, "one viewpoint: the wall cell is seen, not yet an obstacle"
        asyncio.run(m.handle_explore_done(Bool(data=True)))
        asyncio.run(m.handle_explore_cmd(Bool(data=True)))
        assert not m._mission_frozen, "a new order after the end changes nothing: still open"
        assert spy.lines.count("costmap: explore_cmd received - map writing OPEN for the mission") == 1, \
            "the open line is said once"
    finally:
        cmod.logger = real
    print("  parked: 0 cells; explore_cmd: free floor carved to the wall before one wheel turn; "
          "'costmap: explore_cmd received' logged once")


def test_clear_map_starts_a_fresh_grid_and_keeps_the_gate() -> None:
    """The cockpit's Clear map: a fresh grid centred on the current pose; an open
    mission stays open (the next revolution writes), a mission not started stays
    shut; a clear before any pose does nothing."""
    m = costmap()
    asyncio.run(m.handle_clear_map_cmd(Bool(data=True)))       # no pose yet: nothing to clear
    assert m._grid is None
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    for y in (0.0, 0.15):
        asyncio.run(m.handle_odom(_odom(0.0, y)))
        asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
    assert m._grid.value_at(2.5, 0.0) == 100, "the wall is mapped before the clear"
    asyncio.run(m.handle_clear_map_cmd(Bool(data=True)))
    assert m._grid.value_at(2.5, 0.0) == -1, "after the clear the wall's cell is unknown again"
    assert m._write_frozen() is False, "an open mission stays open"
    assert m._revolutions == 0
    asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
    assert m._grid.value_at(2.5, 0.0) != -1, "the next revolution writes into the fresh grid"
    n = costmap()
    asyncio.run(n.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(n.handle_clear_map_cmd(Bool(data=True)))
    assert n._write_frozen() is True, "a mission not started stays shut after a clear"
    asyncio.run(n.handle_clear_map_cmd(Bool(data=False)))
    print("  clear_map_cmd: fresh grid centred on the pose, open mission stays open, unstarted one stays shut, False ignored")


def test_a_contact_marks_the_band_ahead_as_a_wall() -> None:
    """The stall guard fired: the band just beyond the body's front becomes a wall
    at once, in the heading direction, and the planner's occupancy shows it;
    the body itself and the floor behind stay clear; a bump before any pose or
    with the mission shut does nothing; the camera's bare floor frees it."""
    m = costmap()
    asyncio.run(m.handle_bump(Bool(data=True)))              # no pose yet
    assert m._grid is None
    asyncio.run(m.handle_odom(_odom(0.0, 0.0, yaw=0.0)))
    asyncio.run(m.handle_bump(Bool(data=True)))              # mission not started: nothing written
    assert m._grid.value_at(0.35, 0.0) != 100
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    asyncio.run(m.handle_bump(Bool(data=True)))
    hx, _ = _current_profile().body_clear_m
    assert m._grid.value_at(hx + 0.10, 0.0) == 100, "the band ahead is a wall"
    assert m._grid.value_at(hx + 0.10, 0.12) == 100 and m._grid.value_at(hx + 0.10, -0.12) == 100, "across the body's width"
    assert m._grid.value_at(0.0, 0.0) != 100 and m._grid.value_at(-0.3, 0.0) != 100, "the body and the floor behind are not"
    assert m._grid.value_at(hx + 0.40, 0.0) != 100, "the wall is a short band, not a ray"
    # heading east, the band lies east; turned north, a new contact marks north
    asyncio.run(m.handle_odom(_odom(0.0, 0.0, yaw=math.pi / 2)))
    asyncio.run(m.handle_bump(Bool(data=True)))
    assert m._grid.value_at(0.0, hx + 0.10) == 100, "a contact while heading north marks the band north"
    floor = np.array([[hx + 0.10, 0.0, 0.0], [hx + 0.12, 0.02, 0.0]])
    for _ in range(3):
        asyncio.run(m.handle_camera_floor(_cloud(floor)))
    assert m._grid.value_at(hx + 0.10, 0.0) != 100, "bare floor seen by the camera frees the contact mark"
    asyncio.run(m.handle_bump(Bool(data=False)))
    print(f"  contact: the band {CONTACT_BAND_M[0]:.2f}-{CONTACT_BAND_M[1]:.2f} m beyond the body is a wall at once, in the heading, freed by seen floor")


def test_what_goes_out_on_global_costmap() -> None:
    """One cropped OccupancyGrid every PUBLISH_EVERY revolutions."""
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    for k in range(2 * PUBLISH_EVERY):
        asyncio.run(m.handle_odom(_odom(0.0, 0.15 * (k % 2))))
        asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
    out = m.global_costmap.msgs
    assert len(out) == 2, f"{2 * PUBLISH_EVERY} revolutions -> {len(out)} grids (one every {PUBLISH_EVERY})"
    og = out[-1]
    assert isinstance(og, OccupancyGrid) and og.frame_id == "world" and abs(og.resolution - RESOLUTION_M) < 1e-9
    grid = np.asarray(og.grid)
    ox, oy = og.origin.position.x, og.origin.position.y
    i, j = int(math.floor((0.0 - oy) / RESOLUTION_M)), int(math.floor((2.5 - ox) / RESOLUTION_M))
    assert grid[i, j] == 100, f"the wall at (2.5, 0) is 100 in the published grid ({grid[i, j]})"
    i0, j0 = int(math.floor((0.0 - oy) / RESOLUTION_M)), int(math.floor((1.0 - ox) / RESOLUTION_M))
    assert grid[i0, j0] == 0, "the floor at (1.0, 0) is free"
    assert ox <= -2.5 - 20 * RESOLUTION_M + 1e-9 and grid.shape[0] < 480, \
        f"cropped to the seen area + 20 cells of margin ({grid.shape}, origin ({ox:+.2f}, {oy:+.2f}))"
    print(f"  {2 * PUBLISH_EVERY} revolutions -> 2 grids; {grid.shape[1]}x{grid.shape[0]} cells at "
          f"({ox:+.2f}, {oy:+.2f}), wall at 2.5 m = 100, floor at 1.0 m = 0")


def test_the_mission_signals_cross_the_process_boundary() -> None:
    """The mission streams must land on the topics the rest of the stack uses.

    dimOS gives a stream the canonical `/<name>` topic only while (name, type)
    is unique across the blueprint; the same name with a DIFFERENT message
    class gives every module on it a random topic instead. So: one Bool class
    (dimos_lcm.std_msgs.Bool) in costmap2d, explorer2 and explore_ctl.py."""
    from dimos.core.coordination.blueprints import BlueprintAtom
    from dimos.core.transport_factory import transport_topic
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic

    cm = {(s.name, s.type): s.direction for s in BlueprintAtom.create(MossCostMap, {}).streams}
    want = {("lidar", PointCloud2): "in", ("camera_floor", PointCloud2): "in", ("odom", PoseStamped): "in", ("explore_cmd", Bool): "in",
            ("clear_map_cmd", Bool): "in", ("bump", Bool): "in",
            ("explore_done", Bool): "in", ("stop_explore_cmd", Bool): "in",
            ("global_costmap", OccupancyGrid): "out"}
    assert cm == want, f"stream table: extra {set(cm) - set(want)}, missing {set(want) - set(cm)}"
    assert MossCostMap.name == "mosscostmap", MossCostMap.name
    assert {"start", "stop"} <= set(MossCostMap.rpcs)

    from moss_dimos import explorer2 as e2
    ex = {(s.name, s.type): s.direction for s in BlueprintAtom.create(e2.Explorer2, {}).streams}
    assert ex[("explore_done", Bool)] == "out", "explorer2 is the producer of the completion"
    assert ex[("stop_explore_cmd", Bool)] == "in", "and a consumer of the stop, like us"
    for name in ("explore_cmd", "explore_done", "stop_explore_cmd"):
        types_ = {t for (n, t) in list(cm) + list(ex) if n == name}
        assert types_ == {Bool}, f"{name} declared with {len(types_)} classes -> random topics: {types_}"

    want_topic = str(LCMTopic("/stop_explore_cmd", Bool))
    assert transport_topic("/stop_explore_cmd") == "/stop_explore_cmd"
    assert want_topic == "/stop_explore_cmd#std_msgs.Bool", want_topic
    spec = importlib.util.spec_from_file_location("explore_ctl", ROOT / "tools" / "explore_ctl.py")
    ctl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ctl)
    saved = {v: os.environ.pop(v, None) for v in ("DIMOS_TRANSPORT", "TRANSPORT")}   # the default: LCM
    try:
        assert ctl.topic("stop") == want_topic, f"explore_ctl stop -> {ctl.topic('stop')} != {want_topic}"
        assert ctl.topic("start") == str(LCMTopic("/explore_cmd", Bool)), ctl.topic("start")
    finally:
        for v, val in saved.items():
            if val is not None:
                os.environ[v] = val
    print(f"  costmap: 5 Ins + global_costmap, one Bool class; explorer2 agrees; "
          f"stop lands on {want_topic} (explore_ctl.py publishes there)")


def test_revolution_cost() -> None:
    ang = np.radians(np.arange(0, 360, 0.9)); r = np.random.default_rng(0).uniform(0.5, 12.0, len(ang))
    hits = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
    g = ScoredGrid(span_m=24.0)
    t0 = time.perf_counter()
    for i in range(10):
        g.lidar_revolution(hits, (0.01 * i, 0.0), yaw=0.0)
    ms = (time.perf_counter() - t0) / 10 * 1000
    assert ms < 40, ms
    print(f"  one revolution (400 rays, 0.5-12 m, carving every other one, camera layer in view) = {ms:.0f} ms")



# --- the camera layer forgets: lidar rays carve it, the camera holds what it sees ---
SHOE = np.array([[1.525, 0.025]])                  # a camera obstacle at a cell centre, 1.5 m ahead
WALL = np.column_stack([np.full(101, 3.0), np.linspace(-0.5, 0.5, 101)])   # lidar returns behind it, 3 m away
SPOT_B = (0.0, 0.025)                              # the last viewpoint: it looks straight through the shoe
HEADING = 0.0                                      # the rover faces +x: the shoe is straight ahead, in the camera's view


def _camera_score(g: ScoredGrid, xy: np.ndarray) -> int:
    gx, gy = g.cell(xy[:, 0], xy[:, 1])
    return int(g.camera[gy[0], gx[0]])


def _in_view(xy: np.ndarray, rover_xy, yaw: float, margin_deg: float = 1.0) -> np.ndarray:
    """What a D455 on MOSS's nose sees of these points, written here apart
    from costmap2d: within 87 / 2 degrees (+ margin_deg) of the heading, seen
    from the camera (the profile's camera_xyz_base_m ahead of base_link),
    and within 3 m of the rover. The default margin makes this camera see a
    degree wider than the carve, so every cell the carve reaches inside the
    view is one the camera reports."""
    cx, cy = MOSS.camera_xyz_base_m[0], MOSS.camera_xyz_base_m[1]
    cam = (rover_xy[0] + math.cos(yaw) * cx - math.sin(yaw) * cy, rover_xy[1] + math.sin(yaw) * cx + math.cos(yaw) * cy)
    off = np.degrees(np.arctan2(xy[:, 1] - cam[1], xy[:, 0] - cam[0])) - math.degrees(yaw)
    off = (off + 180.0) % 360.0 - 180.0
    return (np.abs(off) <= 87.0 / 2 + margin_deg) & (np.hypot(xy[:, 0] - rover_xy[0], xy[:, 1] - rover_xy[1]) <= 3.0)


def _revolution(g: ScoredGrid, from_xy, camera_xy=None, spare=True) -> None:
    """One revolution as MossCostMap.handle_lidar runs it: lidar rays and hits
    (sparing the cells the camera sees in it), then the camera's hits.
    spare=False lets the carve land on the camera's cells first, as it does
    when the camera's points fall in a cell only every other revolution."""
    g.lidar_revolution(WALL, from_xy, camera_xy=camera_xy if spare else None, yaw=HEADING)
    if camera_xy is not None:
        g.camera_hits(camera_xy, from_xy)


def _camera_wall(viewpoints: int = 2) -> ScoredGrid:
    """The shoe made a wall by the camera from `viewpoints` spots 0.15 m apart, ending at SPOT_B."""
    g = fresh()
    for k in range(viewpoints - 1, -1, -1):
        g.camera_hits(SHOE, (0.0, SPOT_B[1] + 0.15 * k))
    assert g.value_at(*SHOE[0]) == 100 and _camera_score(g, SHOE) == min(viewpoints, HIT_CAP)
    return g


def test_camera_one_viewpoint_never_becomes_a_wall() -> None:
    """A camera hit from the same viewpoint adds 1 while the cell is at 0 or
    above and below its ceiling (the camera must hold what it sees against
    the rays), but one spot never makes a wall: 20 frames from one parked
    spot, the rays crossing the cell, stay below OCCUPIED_AT. Below 0 it adds
    nothing: a camera cell held free at FREE_FLOOR, hit 5 frames from one
    spot, stays below 0 and needs OCCUPIED_AT - FREE_FLOOR viewpoints."""
    traces = {}
    for spare in (True, False):
        g = fresh()
        trace = []
        for _ in range(20):
            _revolution(g, SPOT_B, SHOE, spare=spare)
            trace.append(_camera_score(g, SHOE))
            assert g.value_at(*SHOE[0]) != 100, f"a wall from one spot after {len(trace)} frames (spare={spare})"
        assert max(trace) == OCCUPIED_AT - 1, trace
        traces[spare] = trace
    # long-seen free floor on the camera layer: the camera's floor samples took it to FREE_FLOOR
    g = fresh()
    for _ in range(-FREE_FLOOR + 2):
        g.camera_floor(SHOE)
    assert _camera_score(g, SHOE) == FREE_FLOOR, _camera_score(g, SHOE)
    floor_trace = []
    for _ in range(5):                                 # a phantom seen 5 frames from one parked spot
        g.camera_hits(SHOE, SPOT_B)
        floor_trace.append(_camera_score(g, SHOE))
    assert max(floor_trace) < 0, f"one spot lifted floor held free at {FREE_FLOOR} to {floor_trace}"
    assert floor_trace == [FREE_FLOOR + 1] * 5, floor_trace
    k = 1                                              # SPOT_B was the first viewpoint
    while g.value_at(*SHOE[0]) != 100:
        g.camera_hits(SHOE, (0.0, SPOT_B[1] + 0.15 * k)); k += 1
        assert k <= 20
    assert k == OCCUPIED_AT - FREE_FLOOR, k
    print(f"  20 frames from one spot: camera score {sorted(set(traces[True]))} (OCCUPIED_AT {OCCUPIED_AT}); "
          f"rays landing first every other frame: {sorted(set(traces[False]))}; never a wall")
    print(f"  camera cell at FREE_FLOOR {FREE_FLOOR}, 5 frames from one spot: scores {floor_trace}; "
          f"a wall after {k} viewpoints (as on the lidar layer)")


def test_camera_wall_no_longer_seen_is_carved_free() -> None:
    """The passer-by's legs: a camera wall in the camera's view that the
    camera no longer sees, under lidar rays that pass through its cell, is
    freed (1 per RAY_EVERY revolutions)."""
    assert _in_view(SHOE, SPOT_B, HEADING, margin_deg=-1.0).all(), "the shoe is well inside the camera's view"
    counts = {}
    for viewpoints in (2, 6):
        g = _camera_wall(viewpoints)
        n = 0
        while g.value_at(*SHOE[0]) == 100:
            _revolution(g, SPOT_B); n += 1
            assert n <= 10, f"a camera wall from {viewpoints} viewpoints still stands after 10 revolutions"
        m = n
        while _camera_score(g, SHOE) > 0:
            _revolution(g, SPOT_B); m += 1
            assert m <= 40
        gx, gy = g.cell(SHOE[:, 0], SHOE[:, 1])
        assert not g.camera_wall[gy[0], gx[0]], "carved back to 0: one spot no longer holds it"
        counts[viewpoints] = (n, m)
    assert counts[2][0] <= 2 and counts[6][0] <= 10, counts
    # once forgotten, the same spot cannot raise it back to a wall: it needs two viewpoints again
    g = _camera_wall(2)
    while _camera_score(g, SHOE) > 0:
        _revolution(g, SPOT_B)
    for _ in range(10):
        _revolution(g, SPOT_B, SHOE)
    assert g.value_at(*SHOE[0]) != 100, "forgotten, then seen again from the same spot: not a wall"
    print(f"  camera wall no longer seen: 2 viewpoints -> free after {counts[2][0]} revolutions (score 0 after "
          f"{counts[2][1]}); 6 viewpoints -> free after {counts[6][0]} (score 0 after {counts[6][1]}); "
          f"seen again from the same spot afterwards -> not a wall")


def test_camera_wall_still_seen_survives_the_rays() -> None:
    """The shoe the camera keeps seeing: the rays take 1 every RAY_EVERY
    revolutions, the camera's same-viewpoint hit puts it back every
    revolution, the wall holds. Both orders are checked: the module's (the
    camera's cells spared by the carve) and the carve landing first."""
    carves = 0
    for spare in (True, False):
        g = _camera_wall(2)
        dips = 0
        for k in range(20):
            before = _camera_score(g, SHOE)
            g.lidar_revolution(WALL, SPOT_B, camera_xy=SHOE if spare else None, yaw=HEADING)
            dips += _camera_score(g, SHOE) < before
            g.camera_hits(SHOE, SPOT_B)
            assert g.value_at(*SHOE[0]) == 100, f"the wall the camera still sees fell at revolution {k + 1} (spare={spare})"
        if spare:
            assert dips == 0, f"the carve touched a cell the camera saw in the same revolution ({dips} times)"
        else:
            assert dips == 20 // RAY_EVERY, f"the rays reached the cell {dips} times, expected {20 // RAY_EVERY}"
            carves = dips
    print(f"  camera wall kept in view from one spot: a wall for 20/20 revolutions, spared by the carve; "
          f"with the carve landing first ({carves} carves through it) the same-viewpoint hit holds it at "
          f"{OCCUPIED_AT}")


def _reload(**env):
    """costmap2d re-imported with these knobs (and none of the other two)."""
    for var in ("MOSS_HIT_CAP", "MOSS_FREE_FLOOR", "MOSS_RAY_MAX_M"):
        os.environ.pop(var, None)
    os.environ.update(env)
    return importlib.reload(cmod)


def test_the_knobs_are_read_at_import() -> None:
    """MOSS_HIT_CAP, MOSS_FREE_FLOOR and MOSS_RAY_MAX_M, known input -> known
    output in revolutions and hits."""
    try:
        unlearn = {}
        for cap in ("10", "4"):
            mod = _reload(MOSS_HIT_CAP=cap)
            assert mod.HIT_CAP == int(cap), mod.HIT_CAP
            for viewpoints in (6, 20):
                g = mod.ScoredGrid(span_m=6.0)
                for i in range(viewpoints):
                    g.lidar_revolution(LEG, (0.0, 0.15 * i))
                assert g.value_at(1.0, 0.0) == 100
                n = 0
                while g.value_at(1.0, 0.0) == 100:
                    g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
                    assert n <= 40
                unlearn[(cap, viewpoints)] = n
        assert unlearn[("4", 6)] < unlearn[("10", 6)] and unlearn[("4", 20)] < unlearn[("10", 20)], unlearn

        hits_to_wall = {}
        for floor in ("-3", "-1"):
            mod = _reload(MOSS_FREE_FLOOR=floor)
            assert mod.FREE_FLOOR == int(floor), mod.FREE_FLOOR
            g = mod.ScoredGrid(span_m=6.0)
            for _ in range(20):                                  # long-seen floor at 1.0 m
                g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0))
            gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
            assert g.lidar[gy[0], gx[0]] == int(floor), g.lidar[gy[0], gx[0]]
            k = 0
            while g.value_at(1.0, 0.0) != 100:                   # somebody steps there; the rover drives on
                k += 1
                g.lidar_revolution(LEG, (0.0, 0.15 * k))
                assert k <= 20
            hits_to_wall[floor] = k
        assert hits_to_wall["-1"] < hits_to_wall["-3"], hits_to_wall

        far_leg = {}
        for ray_max in (None, "6"):
            mod = _reload(**({"MOSS_RAY_MAX_M": ray_max} if ray_max else {}))
            assert mod.RAY_MAX_M == (float(ray_max) if ray_max else 4.0), mod.RAY_MAX_M
            g = mod.ScoredGrid(span_m=16.0)
            leg = np.array([[4.5, 0.0]])                         # somebody standing 4.5 m away
            g.lidar_revolution(leg, (0.0, 0.0)); g.lidar_revolution(leg, (0.0, 0.15))
            assert g.value_at(4.5, 0.0) == 100
            n = 0
            while g.value_at(4.5, 0.0) == 100 and n < 20:        # gone: the rays now reach the wall at 6 m
                g.lidar_revolution(np.array([[6.0, 0.0]]), (0.0, 0.0)); n += 1
            far_leg[ray_max or "4"] = n if g.value_at(4.5, 0.0) != 100 else None
        assert far_leg["4"] is None and far_leg["6"] is not None and far_leg["6"] <= 2, far_leg

        odd = {}
        for var, raw in (("MOSS_HIT_CAP", "ten"), ("MOSS_HIT_CAP", "500"), ("MOSS_HIT_CAP", "1"),
                         ("MOSS_HIT_CAP", "4.4"), ("MOSS_FREE_FLOOR", "2"), ("MOSS_RAY_MAX_M", "-1")):
            mod = _reload(**{var: raw})
            odd[(var, raw)] = {"MOSS_HIT_CAP": mod.HIT_CAP, "MOSS_FREE_FLOOR": mod.FREE_FLOOR,
                               "MOSS_RAY_MAX_M": mod.RAY_MAX_M}[var]
        assert list(odd.values()) == [10, 127, OCCUPIED_AT, 4, 0, 0.0], odd
    finally:
        mod = _reload()
    assert (mod.HIT_CAP, mod.FREE_FLOOR, mod.RAY_MAX_M) == (10, -3, 4.0), "defaults restored"
    print(f"  leg from 6 viewpoints: HIT_CAP 10 -> a wall for {unlearn[('10', 6)]} clear revolutions, "
          f"MOSS_HIT_CAP=4 -> {unlearn[('4', 6)]}; from 20 viewpoints: {unlearn[('10', 20)]} vs {unlearn[('4', 20)]}")
    print(f"  long-seen floor becomes a wall after {hits_to_wall['-3']} viewpoints at FREE_FLOOR -3, "
          f"{hits_to_wall['-1']} at MOSS_FREE_FLOOR=-1")
    print(f"  a leg at 4.5 m: never carved with rays to 4 m (20 revolutions), freed after {far_leg['6']} with "
          f"MOSS_RAY_MAX_M=6; MOSS_HIT_CAP 'ten' / 500 / 1 / 4.4 -> {odd[('MOSS_HIT_CAP', 'ten')]} / "
          f"{odd[('MOSS_HIT_CAP', '500')]} / {odd[('MOSS_HIT_CAP', '1')]} / {odd[('MOSS_HIT_CAP', '4.4')]}, "
          f"MOSS_FREE_FLOOR 2 -> {odd[('MOSS_FREE_FLOOR', '2')]}, MOSS_RAY_MAX_M -1 -> {odd[('MOSS_RAY_MAX_M', '-1')]}")


# --- the carve stays in the camera's view; body_clear forgets the camera_wall flag ---
_tx, _ty = np.meshgrid(np.arange(1.525, 2.30, 0.05), np.arange(-0.475, 0.50, 0.05))
TABLE = np.column_stack([_tx.ravel(), _ty.ravel()])            # a table top: 16 x 20 cell centres, 1.5-2.3 m ahead
RING = np.column_stack([2.9 * np.cos(np.radians(np.arange(0, 360, 0.5))),
                        2.9 * np.sin(np.radians(np.arange(0, 360, 0.5)))])   # lidar returns 2.9 m all around


def _cells_scores(g: ScoredGrid, xy: np.ndarray) -> np.ndarray:
    gx, gy = g.cell(xy[:, 0], xy[:, 1])
    return g.camera[gy, gx].astype(int)


def _turn(g: ScoredGrid, rover_xy, yaw: float, obstacle: np.ndarray | None) -> int:
    """One revolution as MossCostMap.handle_lidar runs it, with the camera
    seeing the part of `obstacle` inside its view. Returns how many points it saw."""
    seen = obstacle[_in_view(obstacle, rover_xy, yaw)] if obstacle is not None else np.zeros((0, 2))
    g.lidar_revolution(RING + rover_xy, rover_xy, camera_xy=seen if len(seen) else None, yaw=yaw)
    if len(seen):
        g.camera_hits(seen, rover_xy)
    return len(seen)


def test_the_carve_stays_in_the_cameras_view() -> None:
    """The camera layer holds what the scan plane misses (a table top, a
    chair seat, a shoe). A lidar ray passing under such a cell is no evidence
    against it unless the camera looks at the cell too, so the rays carve the
    camera layer only inside the camera's horizontal field of view.

    Known input -> known output: a table top seen from 3 viewpoints (score 3
    on 320 cells); the rover pivots 180 degrees on the spot in 10 revolutions,
    then faces away for 20, the rays crossing every table cell -> all 320
    cells still score 3. Facing it again with the table gone -> free in
    RAY_EVERY * 2 revolutions."""
    g = fresh()
    for x in (-0.30, -0.15, 0.0):                                  # the approach, facing the table
        assert _turn(g, (x, 0.0), 0.0, TABLE) == len(TABLE)
    assert (_cells_scores(g, TABLE) == 3).all(), _cells_scores(g, TABLE)
    seen_per_rev = []
    for k in range(1, 11):                                         # pivot on the spot, 18 deg per revolution
        seen_per_rev.append(_turn(g, (0.0, 0.0), math.radians(18.0 * k), TABLE))
        assert (_cells_scores(g, TABLE) == 3).all(), f"table cells carved at yaw {18 * k} deg"
    assert seen_per_rev[0] == len(TABLE) and seen_per_rev[-1] == 0, seen_per_rev
    for _ in range(20):                                            # facing away: the table is behind the rover
        _turn(g, (0.0, 0.0), math.pi, TABLE)
    walls = int((_cells_scores(g, TABLE) >= OCCUPIED_AT).sum())
    assert walls == len(TABLE) and (_cells_scores(g, TABLE) == 3).all(), walls
    # the same rays DO cross every table cell: facing it with the table taken away frees it
    probe = copy.deepcopy(g)
    gone = 0
    while (_cells_scores(probe, TABLE) >= OCCUPIED_AT).any():
        _turn(probe, (0.0, 0.0), 0.0, None); gone += 1
        assert gone <= 10, int((_cells_scores(probe, TABLE) >= OCCUPIED_AT).sum())
    assert gone <= RAY_EVERY * 2, gone

    # a passer-by's legs 35 deg off the heading, 1.2 m away: in the camera's view
    legs = np.array([[1.025, 0.625]])
    assert _in_view(legs, (0.0, 0.0), 0.0, margin_deg=-5.0).all() and not _in_view(legs, (0.0, 0.0), math.pi).any()
    kept = None
    freed = {}
    for yaw, label in ((0.0, "facing"), (math.pi, "away")):
        h = fresh()
        h.camera_hits(legs, (0.0, -0.15)); h.camera_hits(legs, (0.0, 0.0))
        assert h.value_at(*legs[0]) == 100
        n = 0
        while h.value_at(*legs[0]) == 100 and n < 20:              # gone: the camera sees nothing there any more
            _turn(h, (0.0, 0.0), yaw, None); n += 1
        freed[label] = n if h.value_at(*legs[0]) != 100 else None
        if label == "away":
            kept = h
    assert freed["facing"] is not None and freed["facing"] <= 10 and freed["away"] is None, freed
    m = 0
    while kept.value_at(*legs[0]) == 100:                          # the rover turns back toward where they stood
        _turn(kept, (0.0, 0.0), 0.0, None); m += 1
        assert m <= 10
    # through the module: handle_lidar carves with the odom heading
    module = {}
    for yaw in (math.pi, 0.0):
        cm = costmap()
        asyncio.run(cm.handle_odom(_odom(0.0, 0.0)))
        asyncio.run(cm.handle_explore_cmd(Bool(data=True)))
        top = np.array([[1.5, 0.3, 0.72]])                         # a table top corner: above the scan plane
        for y in (0.0, 0.15):
            asyncio.run(cm.handle_odom(_odom(0.0, y)))
            asyncio.run(cm.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), top]))))
        assert cm._grid.value_at(1.5, 0.3) == 100, "two viewpoints make the table top a wall"
        asyncio.run(cm.handle_odom(_odom(0.0, 0.15, yaw)))
        n = 0
        while cm._grid.value_at(1.5, 0.3) == 100 and n < 10:      # the camera no longer reports it
            asyncio.run(cm.handle_lidar(_cloud(_lidar_cloud(ROOM)))); n += 1
        module[round(math.degrees(yaw))] = n if cm._grid.value_at(1.5, 0.3) != 100 else None
    assert module[180] is None and module[0] is not None and module[0] <= RAY_EVERY, module
    print(f"  table top, 320 cells at score 3: points in view while turning {seen_per_rev}; after the turn and "
          f"20 revolutions facing away -> {walls}/320 still a wall; facing it with the table gone -> free after {gone} "
          f"revolutions")
    print(f"  legs 35 deg off the heading: freed after {freed['facing']} revolutions in view; behind the rover a wall "
          f"for 20; turning back -> freed after {m}; MossCostMap with odom yaw 180 deg -> kept 10, yaw 0 -> freed after "
          f"{module[0]}")


def test_body_clear_resets_the_camera_wall_flag() -> None:
    """What the body drove over is free, and a camera wall there loses its
    `camera_wall` flag: with the flag left on, same-spot camera hits would
    raise the cell back to OCCUPIED_AT, a wall from one spot."""
    g = _camera_wall(2)
    gx, gy = g.cell(SHOE[:, 0], SHOE[:, 1])
    assert g.camera_wall[gy[0], gx[0]], "two viewpoints flag the camera wall"
    g.body_clear((SHOE[0, 0], SHOE[0, 1], 0.0))                    # the rover drove over the shoe's cell
    assert not g.camera_wall[gy[0], gx[0]], "body_clear resets the flag"
    assert _camera_score(g, SHOE) == FREE_FLOOR and g.value_at(*SHOE[0]) == 0
    right_away = []
    for _ in range(10):                                            # the last viewpoint, 10 frames
        g.camera_hits(SHOE, SPOT_B)
        right_away.append(_camera_score(g, SHOE))
    assert max(right_away) < OCCUPIED_AT, right_away
    # new viewpoints lift it back to OCCUPIED_AT - 1, then 10 frames from the last of them
    k = 0
    while _camera_score(g, SHOE) < OCCUPIED_AT - 1:
        k += 1
        g.camera_hits(SHOE, (0.0, SPOT_B[1] - 0.15 * k))
        assert k <= 10
    last = (0.0, SPOT_B[1] - 0.15 * k)
    flagged = copy.deepcopy(g)
    flagged.camera_wall[gy[0], gx[0]] = True                       # what a body_clear that kept the flag leaves
    after = []
    for _ in range(10):
        g.camera_hits(SHOE, last)
        flagged.camera_hits(SHOE, last)
        after.append(_camera_score(g, SHOE))
        assert g.value_at(*SHOE[0]) != 100, "a wall from one spot after body_clear"
    assert max(after) == OCCUPIED_AT - 1, after
    assert flagged.value_at(*SHOE[0]) == 100, "the flag is what this check guards: left on, one spot makes a wall"
    print(f"  camera wall under the body -> score {FREE_FLOOR}, flag reset; 10 same-spot frames -> {sorted(set(right_away))}; "
          f"{k} new viewpoints to {OCCUPIED_AT - 1}, then 10 same-spot frames -> {sorted(set(after))} (with the flag "
          f"left on: a wall)")


def test_the_fast_defaults() -> None:
    """Without the environment the module counts an obstacle at its first sighting,
    takes a new viewpoint every 5 cm and publishes every two revolutions (the
    obstacle-latency levers)."""
    import subprocess, sys as _sys
    env = {k: v for k, v in os.environ.items() if not k.startswith("MOSS_")}
    env["PYTHONPATH"] = os.environ.get("PYTHONPATH", "")
    out = subprocess.run([_sys.executable, "-c", "import moss_dimos.costmap2d as c; print(c.OCCUPIED_AT, c.NEW_VIEWPOINT_M, c.PUBLISH_EVERY)"],
                         capture_output=True, text=True, env=env, timeout=120).stdout.split()
    assert out == ["1", "0.05", "2"], out
    print("  defaults: OCCUPIED_AT 1, NEW_VIEWPOINT_M 0.05, PUBLISH_EVERY 2 (this bench pins 2 / 0.10 / 5 for its own checks)")


TESTS = (test_the_fast_defaults, test_leg_needs_two_viewpoints_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
         test_chair_moved_is_forgotten_by_lidar_rays, test_reinforcement_is_capped_and_unlearning_bounded,
         test_parked_rover_never_erases_an_established_wall, test_body_clear_is_the_moss_footprint,
         test_the_lidar_tag_is_one_exact_height, test_the_map_writes_only_during_the_mission,
         test_explore_order_opens_the_map_before_any_motion, test_clear_map_starts_a_fresh_grid_and_keeps_the_gate,
         test_a_contact_marks_the_band_ahead_as_a_wall,
         test_what_goes_out_on_global_costmap,
         test_the_mission_signals_cross_the_process_boundary, test_revolution_cost,
         test_camera_one_viewpoint_never_becomes_a_wall, test_camera_wall_no_longer_seen_is_carved_free,
         test_camera_wall_still_seen_survives_the_rays, test_the_knobs_are_read_at_import,
         test_the_carve_stays_in_the_cameras_view, test_body_clear_resets_the_camera_wall_flag)

if __name__ == "__main__":
    OK = KO = 0
    for t in TESTS:
        print(t.__name__)
        try:
            t()
            OK += 1
        except Exception:  # noqa: BLE001 - one KO line per test, the bench goes on
            KO += 1
            print("  KO " + traceback.format_exc().strip().splitlines()[-1])
            traceback.print_exc()
    # Open3D (behind PointCloud2) reports every cloud still alive at
    # interpreter exit as a leak and turns the exit status to 1: drop them first.
    for m in MODULES:
        m.global_costmap.msgs.clear()
    MODULES.clear()
    import gc
    gc.collect()
    print()
    print(f"{OK} OK, {KO} KO")
    print("TEST PASSED" if KO == 0 else "TEST FAILED")
    sys.exit(1 if KO else 0)
