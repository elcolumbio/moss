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
     MOSS_LIDAR_HEIGHT_M moves the tag at call time
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

Needs dimOS (the messages are real).

Run:  PYTHONPATH=. .venv/bin/python benches/bench_costmap2d_cold.py
"""

import asyncio
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

from dimos_lcm.std_msgs import Bool  # noqa: E402

from moss_dimos import costmap2d as cmod  # noqa: E402
from moss_dimos.costmap2d import (  # noqa: E402
    FREE_FLOOR, HIT_CAP, MISSION_START_M, OCCUPIED_AT, PUBLISH_EVERY, RESOLUTION_M,
    MossCostMap, ScoredGrid, lidar_returns,
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
    a lidar return, even millimetres away from the scan plane."""
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
    assert m._grid.value_at(1.0, 0.5) != 100, "the stray point is not"
    n = m._revolutions
    asyncio.run(m.handle_lidar(_cloud(stray)))
    assert m._revolutions == n, "a cloud with no lidar return is not a revolution"
    print("  float32 0.20 m -> lidar; 0.2005 / 0.37 / 0.0 -> not; MOSS_LIDAR_HEIGHT_M=0.37 moves the tag; "
          "a point 2 mm off the plane never reaches the map")


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
    want = {("lidar", PointCloud2): "in", ("odom", PoseStamped): "in", ("explore_cmd", Bool): "in",
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
        g.lidar_revolution(hits, (0.01 * i, 0.0))
    ms = (time.perf_counter() - t0) / 10 * 1000
    assert ms < 40, ms
    print(f"  one revolution (400 rays, 0.5-12 m, carving every other one) = {ms:.0f} ms")


TESTS = (test_leg_needs_two_viewpoints_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
         test_chair_moved_is_forgotten_by_lidar_rays, test_reinforcement_is_capped_and_unlearning_bounded,
         test_parked_rover_never_erases_an_established_wall, test_body_clear_is_the_moss_footprint,
         test_the_lidar_tag_is_one_exact_height, test_the_map_writes_only_during_the_mission,
         test_explore_order_opens_the_map_before_any_motion, test_what_goes_out_on_global_costmap,
         test_the_mission_signals_cross_the_process_boundary, test_revolution_cost)

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
