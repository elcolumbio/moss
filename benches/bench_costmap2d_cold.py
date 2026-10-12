"""Cold bench for costmap2d: the scored grid and MossCostMap, written as numbers.

Known input -> known output, in metres, cells and seconds:
  1. the doctrine defaults: without the environment a wall needs two places
     10 cm apart (OCCUPIED_AT 2, NEW_VIEWPOINT_M 0.10), a grid goes out every
     2 revolutions, the contact wall is off, a camera hit protects its cell
     for 3 s and the camera rays carve between 0.013 and 0.060 m up;
     MOSS_OCCUPIED_AT, MOSS_NEW_VIEWPOINT_M and MOSS_CONTACT_WALL still move them
  2. a table leg at (1.0, 0) seen from two places -> occupied; the floor along
     the ray (0.3..0.9 m) -> free; nothing else touched
  3. a ramp: 50 hits from the SAME spot -> never occupied (one viewpoint)
  4. a chair: occupied, then lidar revolutions see through it -> free
  5. a ceiling of reinforcement: 30 viewpoints -> score 10, then bounded
     unlearning (never a wall for a day)
  6. a parked rover: a wall raised to HIT_CAP by 12 viewpoints, then
     revolutions from the spot it is parked on and 0.5 m of creep -> still
     HIT_CAP, still occupancy 100, on the lidar layer and on the camera layer
     (the viewpoint rule caps the increase, it never lowers a score)
  7. body_clear: the MOSS footprint (0.20 x 0.16 m half extents) clears 63
     cells at the origin; MOSS_BODY_CLEAR_M moves it at call time
  8. the lidar tag: a float32 point at exactly the profile's lidar height
     (0.20 m) is a lidar return; 0.37 m, 0.2005 m and the floor are not;
     MOSS_LIDAR_HEIGHT_M moves the tag at call time; through MossCostMap a
     camera obstacle maps after the gate's frame and two viewpoints, never
     from one parked spot, the lidar's rays through its cell take nothing
     from it, and the camera's floor frees it once the 3 s window is over
  9. the mission gate: 20 revolutions and a passer-by on a PARKED rover ->
     0 cells; 6 cm of displacement -> the map opens and the room's wall at
     2.5 m is mapped; "exploration complete" and "exploration stopped" ->
     the map keeps writing
 10. the explore order: parked revolutions write nothing until explore_cmd,
     then the same parked revolutions carve free floor up to the wall
 11. Clear map: a fresh grid on the pose, the mission gate as it was
 12. the contact wall (MOSS_CONTACT_WALL): off, a contact on `bump` writes
     nothing and says so; on, only a contact felt while driving straight
     forward marks the band 0.02-0.20 m beyond the body's front as a wall (a
     pivot, a reverse, a contact while standing still, 10 cm driven while
     turning 20 deg: nothing; 10 cm with 6 deg of turn: written); the camera's
     floor frees the band
 13. what goes out: one OccupancyGrid every 5 revolutions on global_costmap,
     0.05 m cells, frame `world`, the wall at 2.5 m = 100 in the grid
 14. the mission signals land on the topics the rest of the stack uses: one
     Bool class for explore_cmd / explore_done / stop_explore_cmd across
     costmap2d, explorer2 and tools/explore_ctl.py
 15. the cost: one lidar revolution (400 rays, 0.5-12 m) and one camera floor
     frame (its floor samples and rays) each stay under 40 ms
 16. a camera obstacle seen from ONE parked spot for 20 frames, with the
     lidar's rays and the camera's rays crossing its cell, stays below
     OCCUPIED_AT (never a wall); a camera cell at FREE_FLOOR seen 5 frames
     from one spot stays below 0
 17. a passer-by's legs: a camera wall no longer seen is untouched by the
     camera's rays for 3 s, then freed by them (1 look at score 2, 5 at 6)
 18. a camera wall the camera KEEPS seeing from one spot, its own floor rays
     and the lidar's crossing its cell, stays a wall for 20 frames (4 s, past
     the window: every frame stamps it again)
 19. the knobs, read at import: MOSS_HIT_CAP=4 unlearns a leg seen from 6
     viewpoints in fewer revolutions than 10; MOSS_FREE_FLOOR=-1 lets long-seen
     floor become a wall in fewer hits; MOSS_RAY_MAX_M=6 carves a leg at 4.5 m
     that the default 4 m never reaches; bad values fall back or clamp
 20. the camera layer is carved only where the camera looks: a table top
     (320 cells, a wall from 3 viewpoints) stays a wall while the rover turns
     its back on it and 20 revolutions of lidar rays cross every cell; facing
     it with the table gone, the camera's floor frees it; legs 35 deg off the
     heading are freed while in view, kept while behind the rover, through
     MossCostMap and the odom heading
 21. body_clear frees the camera layer under the body, and same-spot camera
     frames never make the cell a wall again
 22. the thin leg: armed at 2, 25 floor samples on its own cell inside 3 s ->
     still 2 and a wall, the lidar layer untouched; past the window ->
     forgotten; through MossCostMap a parked rover seeing the leg and the
     floor on its cell for 10 frames keeps the wall (without the window the
     first floor frame frees it)
 23. moving things: a vacuum rolling past one cell per frame leaves no wall and
     no 3 s stamp (only the cells the gate keeps are stamped); a standing box
     maps; ONE depth frame carried by 5 revolutions while the rover drives
     12 cm takes no hit (the same 5 revolutions taken as 5 frames would hit);
     the same bytes on a cloud stamped 0.4 s later are that frame again, 1.5 s
     later a new look (by the clouds' own stamps); a revolution without camera
     points clears the gate
 24. the camera's rays: a floor point 3 m ahead carves cells 0.75 to 2.50 m
     (36 cells: from 20 % to 82 % of the ray), not its endpoint; a ghost
     crossed low dies, a ray climbing over it (22 cm up) leaves it, a cell hit
     2 s ago is protected; camera rays never touch the lidar layer and lidar
     rays never touch the camera layer; through MossCostMap, a 5 cm threshold
     beyond a ghost at score 5: one depth frame carried by 3 revolutions takes
     one miss, a new frame a second; in a frame, hits go before rays, so a
     thin leg the frame confirms is not carved by a low ray of that frame
 25. the rays start at the camera, 0.156 m ahead of base_link and turned by
     the heading: a floor sample 3 m ahead carves from 0.75 m, not from the
     0.60 m a base-cast ray would, and a ghost at 0.675 m survives the look
     that a base-cast ray would erase; through MossCostMap, a stale ghost at
     1.5 m dies to a floor sample 3 m ahead, facing +x and facing +y; a 4 cm
     cable 0.95 m ahead leaves a ghost at 0.475 m that a base-cast ray would
     carve; under MOSS_CAMERA_XYZ_M="0.156 0.05 0.15" the origin moves 5 cm
     sideways and the band and height move together (the floor ray still
     carves 0.75 to 2.50 m)
 26. the carve band follows the camera height: 0.013-0.060 m at 0.075 m,
     the original 0.10-0.45 m at 0.56 m
 27. the premap needs both inputs: a loaded_map alone, or a `world` -> `map`
     fix alone, writes nothing and says it arrived (the fix once); a
     `map` -> `world` transform is not a fix; both (in either order, or
     before the first pose) write 20 walls from 100 points with the fix
     (x 1 m, yaw 90 deg): map (mx, -1.025) lands at world (2.025, mx), on the
     lidar layer at OCCUPIED_AT; the floor layer (z 0.025) and points above
     1.30 m are left out; a cell the lidar has seen free stays free; the
     mission gate does not hold the write and nothing is published before
     it; a different premap under the same fix replaces the old one; a
     5 300-point premap is written in one go
 28. a lidar ray through a premap wall carves it like any wall: a premap cell
     and a live wall at the same score both turn free on the same revolution
 29. a fix 5 cm away clears the premap (each cell loses what the premap
     added: a live hit since stays, a ray's miss since stays, an unseen cell
     goes back to unknown) and writes it 5 cm over; a fix 1 cm away, the same
     fix again or the same premap again change nothing; a fix creeping by
     1.5 cm steps is measured against the written one (re-written at 2.5 cm);
     1.5 deg of turn alone re-writes; +179.6 to -179.6 deg (0.8 deg apart)
     does not, -178.4 (2 deg) does; a live wall from two viewpoints under the
     premap is still a wall after the re-write; a cell at HIT_CAP stays at
     HIT_CAP through the write and the clear; Clear map takes the premap out
     and the same fix does not bring it back
 30. premap off (the default, MOSS_PREMAP=0): loaded_map and the fix write
     nothing, publish nothing, and "costmap: premap ignored (MOSS_PREMAP=0)"
     is said once for three loaded_maps
 31. the camera layer and the seen mask are untouched by a premap write, a
     re-write and a clear
 32. nothing is published before the mission, premap or not; the first
     grid of the mission carries the premap walls as 100, the floor the
     lidar saw as 0 and the floor beside the walls as -1; once a grid has
     gone out a re-write publishes at once, the old cells -1 again; after
     Clear map a re-write waits for the map's next grid
 33. a premap wall a lidar ray carved stays carved through a re-write, even
     over a one-viewpoint hit from before the premap (1 + 1 - 1 = 1 would
     be a wall again): free, not flagged; a lidar hit from a new viewpoint
     lifts the mark and the next write claims the cell again

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
# the checks below pin the doctrine (a wall from two places 10 cm apart: the module defaults too,
# checked in a subprocess by test_the_doctrine_defaults) and one grid per five revolutions (the
# module publishes every two; test_what_goes_out counts five)
os.environ["MOSS_OCCUPIED_AT"] = "2"
os.environ["MOSS_NEW_VIEWPOINT_M"] = "0.10"
os.environ["MOSS_PUBLISH_EVERY"] = "5"

os.environ.setdefault("DIMOS_TRANSPORT", "lcm")  # MOSS runs on LCM (tools/moss-fly.sh sets it); dimOS main defaults to zenoh (4333a8fa6)
from dimos_lcm.std_msgs import Bool  # noqa: E402

from moss_dimos import costmap2d as cmod  # noqa: E402
from moss_dimos.robot_profile import current as _current_profile  # noqa: E402
from moss_dimos.costmap2d import (  # noqa: E402
    CONTACT_BAND_M, FREE_FLOOR, HIT_CAP, LOW_HIT_PROTECT_S, MISSION_START_M, OCCUPIED_AT, PUBLISH_EVERY,
    RAY_EVERY, RESOLUTION_M, MossCostMap, ScoredGrid, camera_xy, carve_z_band, lidar_returns,
)
from moss_dimos.robot_profile import MOSS  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
LEG = np.array([[1.0, 0.0]])
LIDAR_Z = MOSS.lidar_height_m          # 0.20 m: where lidar_odometry lifts every return
CAM_X, CAM_Y, CAM_Z = MOSS.camera_xyz_base_m   # 0.156, 0, 0.075 m: the depth camera on the nose


def fresh() -> ScoredGrid:
    return ScoredGrid(span_m=6.0)


def _cloud(pts: np.ndarray, ts: float | None = None):
    """A world cloud on the wire, float32 like the real one, stamped now (or `ts`, wall seconds)."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    return PointCloud2.from_numpy(pts.astype(np.float32), frame_id="world",
                                  timestamp=time.time() if ts is None else ts)


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


def _frame(pts: np.ndarray, k: int) -> np.ndarray:
    """Depth frame number k of the same scene: the points moved by k x 0.1 mm
    in x, as depth noise moves them, so every frame's bytes differ from the
    last one's (MossCostMap takes a cloud carrying the previous frame's
    points, byte for byte, as that frame again). Points sit at cell centres,
    so 2 mm of noise never changes their cell."""
    return np.asarray(pts, dtype=np.float64) + np.array([1e-4 * k] + [0.0] * (np.shape(pts)[1] - 1))


def _armed(g: ScoredGrid, xy: np.ndarray, t: float, viewpoints=((0.0, 0.0), (0.0, 0.15))) -> None:
    """A camera wall: the gate's first frame, then one frame per viewpoint, all at time t."""
    g.camera_obstacles(_frame(xy, 0), viewpoints[0], now=t)
    for k, vp in enumerate(viewpoints, start=1):
        g.camera_obstacles(_frame(xy, k), vp, now=t)


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
    next revolution taken from a parked rover. Same rule on the camera layer."""
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
    # the camera layer holds an obstacle the same way
    c = fresh()
    box = np.array([[1.025, 0.025]])
    t0 = 500.0
    c.camera_obstacles(_frame(box, 0), (0.0, 0.0), now=t0)          # the moving-object gate's first frame
    for i in range(12):
        c.camera_obstacles(_frame(box, i + 1), (0.0, 0.15 * i), now=t0 + 0.2 * (i + 1))
    bx, by = c.cell(box[:, 0], box[:, 1])
    assert c.camera[by[0], bx[0]] == HIT_CAP and c.value_at(*box[0]) == 100, "12 viewpoints -> a camera wall"
    c.camera_obstacles(_frame(box, 13), (0.0, 0.15 * 11), now=t0 + 2.8)
    assert c.camera[by[0], bx[0]] == HIT_CAP, "the camera layer is capped the same way"
    assert c.value_at(*box[0]) == 100, "a camera obstacle survives a parked rover too"
    print(f"  wall at 1.0 m, 12 viewpoints -> score {HIT_CAP}; parked + 1 revolution and 0.5 m of "
          f"creep -> still {HIT_CAP}, occupancy 100 (lidar and camera layers)")


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
    a lidar return, even millimetres away from the scan plane: it is a camera
    obstacle, on the camera layer, behind the moving-object gate, and only
    the camera can take it back."""
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
    # point 2 mm above the scan plane - the wall on the lidar layer, the point
    # on the camera layer
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    stray = np.array([[1.025, 0.525, LIDAR_Z + 0.002]])
    for k, y in enumerate((0.0, 0.0, 0.15)):                      # the gate's first frame, then two viewpoints
        asyncio.run(m.handle_odom(_odom(0.0, y)))
        asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(stray, k)]))))
    assert m._grid.value_at(2.5, 0.0) == 100, "the wall is mapped"
    sx, sy = m._grid.cell(stray[:, 0], stray[:, 1])
    assert m._grid.value_at(1.025, 0.525) == 100 and m._grid.camera[sy[0], sx[0]] == 2, \
        "the off-plane point is a camera obstacle: mapped after the gate's frame and two viewpoints"
    assert m._grid.lidar[sy[0], sx[0]] <= 0, "the lidar layer never claimed a cell the lidar never returned"
    n = m._revolutions
    asyncio.run(m.handle_lidar(_cloud(stray)))
    assert m._revolutions == n, "a cloud with no lidar return is not a revolution"
    # a shoe: from ONE parked viewpoint it stays below OCCUPIED_AT, like a lidar hit
    m2 = costmap()
    asyncio.run(m2.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m2.handle_explore_cmd(Bool(data=True)))
    low = np.array([[1.525, -0.375, 0.05]])           # 5 cm high, invisible to the scan plane
    for k in range(3):                                # the gate's first frame, then two frames from one spot
        asyncio.run(m2.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(low, k)]))))
    assert m2._grid.value_at(1.525, -0.375) != 100, "one parked viewpoint never makes a camera obstacle a wall"
    asyncio.run(m2.handle_odom(_odom(0.0, 0.15)))
    asyncio.run(m2.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(low, 3)]))))
    assert m2._grid.value_at(1.525, -0.375) == 100, "the second viewpoint makes the shoe a wall"
    # the lidar sees the room's wall behind the shoe: its rays cross the shoe's
    # cell (its own layer goes below 0 there) and take nothing from the camera layer
    probe = copy.deepcopy(m2._grid)
    lx, ly = probe.cell(low[:, 0], low[:, 1])
    before = int(probe.camera[ly[0], lx[0]])
    for _ in range(4):
        probe.lidar_revolution(ROOM, (0.0, 0.15))
    assert probe.lidar[ly[0], lx[0]] < 0, "the lidar's rays do cross the shoe's cell"
    assert int(probe.camera[ly[0], lx[0]]) == before == 2, "and leave the shoe on the camera layer"
    # the shoe is taken away: the camera sees bare floor on its cell. Inside
    # the 3 s window that floor is the thin leg's and changes nothing; past it,
    # two floor samples free the cell
    floor = np.array([[1.525, -0.375, 0.0], [1.535, -0.385, 0.0]])
    asyncio.run(m2.handle_camera_floor(_cloud(floor)))
    assert m2._grid.value_at(1.525, -0.375) == 100, "a floor sample inside the 3 s window changes nothing"
    m2._grid._last_low_hit[:] = time.monotonic() - LOW_HIT_PROTECT_S - 1.0   # nothing confirmed the shoe since
    for _ in range(2):
        asyncio.run(m2.handle_camera_floor(_cloud(floor)))
    assert m2._grid.value_at(1.525, -0.375) != 100, "bare floor seen by the camera frees the shoe's cell"
    print("  float32 0.20 m -> lidar; 0.2005 / 0.37 / 0.0 -> camera; MOSS_LIDAR_HEIGHT_M=0.37 moves the tag; "
          "a camera obstacle maps after the gate's frame and two viewpoints (a 5 cm shoe), never from one parked "
          "spot; the lidar's rays through it take nothing; the camera's floor frees it after the 3 s window")


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


# the cockpit's decision feed reads a written contact by this line (tools/moss_cockpit.py)
_N = r"([-+]?\d+(?:\.\d+)?)"
CONTACT_LINE = r"costmap: contact at \(" + _N + r", " + _N + r"\) heading " + _N + r" deg: (\d+) cells ahead marked as a wall"


def _drive(m: MossCostMap, poses) -> None:
    for x, y, yaw in poses:
        asyncio.run(m.handle_odom(_odom(x, y, yaw)))


def test_a_contact_marks_the_band_ahead_as_a_wall() -> None:
    """Contacts are reflexes: with MOSS_CONTACT_WALL off (the default) a
    contact on `bump` writes nothing, and says so. On, a contact felt while
    driving straight forward makes the band just beyond the body's front a
    wall at once, in the heading direction; a contact while standing still
    (an explorer's back-off), pivoting or reversing writes nothing; the body
    itself and the floor behind stay clear; a bump before any pose or with the
    mission shut does nothing; the camera's bare floor frees the band."""
    import re
    spy, real = LogSpy(), cmod.logger
    cmod.logger = spy
    hx, _ = _current_profile().body_clear_m
    try:
        assert cmod.CONTACT_WALL is False, "the contact wall is off by default"
        off = costmap()
        _drive(off, [(0.0, 0.0, 0.0)])
        asyncio.run(off.handle_explore_cmd(Bool(data=True)))
        _drive(off, [(0.05, 0.0, 0.0), (0.10, 0.0, 0.0)])
        asyncio.run(off.handle_bump(Bool(data=True)))
        assert off._grid.value_at(0.10 + hx + 0.125, 0.025) != 100, "knob off: a forward contact writes no wall"
        assert spy.lines[-1].endswith("not written: contacts are reflexes, only the lidar and the camera write the "
                                      "map (MOSS_CONTACT_WALL=1 writes a forward contact)"), spy.lines[-1]
        assert not re.search(CONTACT_LINE, spy.lines[-1]), "a contact not written never reads as a wall in the feed"

        cmod.CONTACT_WALL = True
        m = costmap()
        asyncio.run(m.handle_bump(Bool(data=True)))              # no pose yet
        assert m._grid is None
        _drive(m, [(0.0, 0.0, 0.0)])
        asyncio.run(m.handle_bump(Bool(data=True)))              # mission not started: nothing written
        assert m._grid.value_at(hx + 0.125, 0.025) != 100
        asyncio.run(m.handle_explore_cmd(Bool(data=True)))
        asyncio.run(m.handle_bump(Bool(data=True)))              # standing still: an explorer's back-off
        assert m._grid.value_at(hx + 0.125, 0.025) != 100, "a contact while standing still writes nothing"
        assert "not driving straight forward (+0 cm along the heading, +0 deg in the last 0.5 s)" in spy.lines[-1], \
            spy.lines[-1]
        _drive(m, [(0.05, 0.0, 0.0), (0.10, 0.0, 0.0)])          # 10 cm straight ahead
        asyncio.run(m.handle_bump(Bool(data=True)))
        x0 = 0.10
        assert m._grid.value_at(x0 + hx + 0.125, 0.025) == 100, "a forward contact: the band ahead is a wall"
        assert m._grid.value_at(x0 + hx + 0.125, 0.125) == 100 and m._grid.value_at(x0 + hx + 0.125, -0.125) == 100, \
            "across the body's width"
        assert m._grid.value_at(x0, 0.025) != 100 and m._grid.value_at(x0 - 0.3, 0.025) != 100, \
            "the body and the floor behind are not"
        assert m._grid.value_at(x0 + hx + 0.425, 0.025) != 100, "the wall is a short band, not a ray"
        written = re.search(CONTACT_LINE, spy.lines[-1])
        assert written and written.group(1, 2, 3) == ("+0.10", "+0.00", "+0"), spy.lines[-1]
        cells = int(written.group(4))

        verdicts = {}
        for label, poses in (("pivot", [(0.0, 0.0, 0.0), (0.0, 0.0, math.radians(30.0))]),
                             ("reverse", [(0.10, 0.0, 0.0), (0.05, 0.0, 0.0), (0.0, 0.0, 0.0)]),
                             ("arc", [(0.0, 0.0, 0.0), (0.05, 0.0, math.radians(3.0)), (0.10, 0.005, math.radians(6.0))]),
                             # 10 cm along the heading (more than the 3 cm) while turning 20 deg: the turn refuses it
                             ("sharp arc", [(0.0, 0.0, 0.0), (0.05, 0.004, math.radians(10.0)),
                                            (0.10, 0.017, math.radians(20.0))])):
            c = costmap()
            _drive(c, poses[:1])
            asyncio.run(c.handle_explore_cmd(Bool(data=True)))
            _drive(c, poses[1:])
            asyncio.run(c.handle_bump(Bool(data=True)))
            x, y, yaw = poses[-1]
            ahead = (x + math.cos(yaw) * (hx + 0.11), y + math.sin(yaw) * (hx + 0.11))
            verdicts[label] = c._grid.value_at(*ahead) == 100
        assert verdicts == {"pivot": False, "reverse": False, "arc": True, "sharp arc": False}, verdicts
        assert "not driving straight forward (+10 cm along the heading, +20 deg in the last 0.5 s)" in spy.lines[-1], \
            spy.lines[-1]
        # heading north, driving north: the band lies north
        n = costmap()
        _drive(n, [(0.0, 0.0, math.pi / 2)])
        asyncio.run(n.handle_explore_cmd(Bool(data=True)))
        _drive(n, [(0.0, 0.05, math.pi / 2), (0.0, 0.10, math.pi / 2)])
        asyncio.run(n.handle_bump(Bool(data=True)))
        assert n._grid.value_at(0.025, 0.10 + hx + 0.125) == 100, "a contact while driving north marks the band north"
        # the camera's bare floor frees the band (a contact mark is no camera hit: no 3 s window)
        floor = np.array([[x0 + hx + 0.125, 0.025, 0.0], [x0 + hx + 0.13, 0.03, 0.0]])
        for _ in range(3):
            asyncio.run(m.handle_camera_floor(_cloud(floor)))
        assert m._grid.value_at(x0 + hx + 0.125, 0.025) != 100, "bare floor seen by the camera frees the contact mark"
        asyncio.run(m.handle_bump(Bool(data=False)))
    finally:
        cmod.CONTACT_WALL = False
        cmod.logger = real
    print(f"  MOSS_CONTACT_WALL off: a forward contact writes nothing, said in the log; on: driving 10 cm straight "
          f"ahead -> the band {CONTACT_BAND_M[0]:.2f}-{CONTACT_BAND_M[1]:.2f} m beyond the body is a wall ({cells} "
          f"cells); standing still, pivoting 30 deg, reversing or 10 cm while turning 20 deg -> nothing; a 6 deg arc -> "
          "written; freed by seen floor")


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
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic

    cm = {(s.name, s.type): s.direction for s in BlueprintAtom.create(MossCostMap, {}).streams}
    want = {("lidar", PointCloud2): "in", ("camera_floor", PointCloud2): "in", ("odom", PoseStamped): "in", ("explore_cmd", Bool): "in",
            ("clear_map_cmd", Bool): "in", ("bump", Bool): "in",
            ("explore_done", Bool): "in", ("stop_explore_cmd", Bool): "in",
            ("loaded_map", PointCloud2): "in", ("tf", TFMessage): "in",
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
    print(f"  costmap: {len(want) - 1} Ins + global_costmap, one Bool class; explorer2 agrees; "
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
    # one depth frame's bare floor: every cell centre the camera sees within 3 m, its samples and its rays
    floor = _floor_view((0.0, 0.0), 0.0)
    t0 = time.perf_counter()
    for i in range(10):
        g.camera_floor(floor[:, :2], now=1000.0 + i)
        g.camera_rays(floor, camera_xy((0.01 * i, 0.0), 0.0), now=1000.0 + i)
    floor_ms = (time.perf_counter() - t0) / 10 * 1000
    assert floor_ms < 40, floor_ms
    print(f"  one revolution (400 rays, 0.5-12 m, carving every other one) = {ms:.1f} ms; one camera floor frame "
          f"({len(floor)} samples and their rays) = {floor_ms:.1f} ms")


# --- the camera layer: only the camera writes it, only the camera clears it ---
SHOE = np.array([[1.525, 0.025]])                  # a camera obstacle at a cell centre, 1.5 m ahead
WALL = np.column_stack([np.full(101, 3.0), np.linspace(-0.5, 0.5, 101)])   # lidar returns behind it, 3 m away
SPOT_B = (0.0, 0.025)                              # the last viewpoint: it looks straight through the shoe
HEADING = 0.0                                      # the rover faces +x: the shoe is straight ahead, in the camera's view
BEYOND = np.array([[3.025, 0.025, 0.0]])           # bare floor 3 m ahead: from the camera at (0.156, 0.025), 7.5 cm up,
                                                   # its ray crosses the shoe's cell 4 cm up (inside the carve band)


def _camera_score(g: ScoredGrid, xy: np.ndarray) -> int:
    gx, gy = g.cell(xy[:, 0], xy[:, 1])
    return int(g.camera[gy[0], gx[0]])


def _in_view(xy: np.ndarray, rover_xy, yaw: float, margin_deg: float = 1.0) -> np.ndarray:
    """What a D455 on MOSS's nose sees of these points, written here apart
    from costmap2d: within 87 / 2 degrees (+ margin_deg) of the heading, seen
    from the camera (the profile's camera_xyz_base_m ahead of base_link),
    and within 3 m of the rover."""
    cx, cy = MOSS.camera_xyz_base_m[0], MOSS.camera_xyz_base_m[1]
    cam = (rover_xy[0] + math.cos(yaw) * cx - math.sin(yaw) * cy, rover_xy[1] + math.sin(yaw) * cx + math.cos(yaw) * cy)
    off = np.degrees(np.arctan2(xy[:, 1] - cam[1], xy[:, 0] - cam[0])) - math.degrees(yaw)
    off = (off + 180.0) % 360.0 - 180.0
    return (np.abs(off) <= 87.0 / 2 + margin_deg) & (np.hypot(xy[:, 0] - rover_xy[0], xy[:, 1] - rover_xy[1]) <= 3.0)


_FX, _FY = np.meshgrid(np.arange(-5.975, 6.0, 0.05), np.arange(-5.975, 6.0, 0.05))
_CENTRES = np.column_stack([_FX.ravel(), _FY.ravel()])   # cell centres around the origin, 0.05 m apart


def _floor_view(rover_xy, yaw: float) -> np.ndarray:
    """One depth frame's bare floor, written here apart from lidar_odometry:
    every cell centre the camera sees from this pose (87 degrees, 3 m), z = 0."""
    pts = _CENTRES[_in_view(_CENTRES, rover_xy, yaw, margin_deg=0.0)]
    return np.column_stack([pts, np.zeros(len(pts))])


def _camera_wall(viewpoints: int = 2, t: float = 100.0) -> ScoredGrid:
    """The shoe made a wall by the camera from `viewpoints` spots 0.15 m apart, ending at SPOT_B, stamped at t."""
    g = fresh()
    _armed(g, SHOE, t, viewpoints=[(0.0, SPOT_B[1] + 0.15 * k) for k in range(viewpoints - 1, -1, -1)])
    assert g.value_at(*SHOE[0]) == 100 and _camera_score(g, SHOE) == min(viewpoints, HIT_CAP)
    return g


def test_camera_one_viewpoint_never_becomes_a_wall() -> None:
    """One spot never makes a wall on the camera layer either: 20 depth
    frames from one parked spot stay below OCCUPIED_AT, with or without the
    lidar's rays and the camera's own floor rays crossing the cell. Below 0
    nothing changes that: a camera cell held free at FREE_FLOOR, seen 5 frames
    from one spot, stays below 0 and needs OCCUPIED_AT - FREE_FLOOR viewpoints."""
    cam = camera_xy(SPOT_B, HEADING)
    traces = {}
    for rays in (False, True):
        g = fresh()
        trace = []
        for k in range(20):
            t = 100.0 + 0.2 * k
            g.camera_obstacles(_frame(SHOE, k), SPOT_B, now=t)
            if rays:
                g.lidar_revolution(WALL, SPOT_B)
                g.camera_rays(BEYOND, cam, now=t)
            trace.append(_camera_score(g, SHOE))
            assert g.value_at(*SHOE[0]) != 100, f"a wall from one spot after {len(trace)} frames (rays={rays})"
        traces[rays] = trace
    assert max(traces[False]) == OCCUPIED_AT - 1 and max(traces[True]) <= OCCUPIED_AT - 1, traces
    # long-seen free floor on the camera layer: the camera's floor samples took it to FREE_FLOOR
    g = fresh()
    for _ in range(-FREE_FLOOR + 2):
        g.camera_floor(SHOE, now=50.0)
    assert _camera_score(g, SHOE) == FREE_FLOOR, _camera_score(g, SHOE)
    g.camera_obstacles(_frame(SHOE, 0), SPOT_B, now=100.0)          # the gate's first frame
    floor_trace = []
    for k in range(1, 6):                                            # a phantom seen 5 frames from one parked spot
        g.camera_obstacles(_frame(SHOE, k), SPOT_B, now=100.0 + 0.2 * k)
        floor_trace.append(_camera_score(g, SHOE))
    assert floor_trace == [FREE_FLOOR + 1] * 5, floor_trace
    k = 1                                                            # SPOT_B was the first viewpoint
    while g.value_at(*SHOE[0]) != 100:
        g.camera_obstacles(_frame(SHOE, 10 + k), (0.0, SPOT_B[1] + 0.15 * k), now=102.0 + 0.2 * k); k += 1
        assert k <= 20
    assert k == OCCUPIED_AT - FREE_FLOOR, k
    print(f"  20 frames from one spot: camera score {sorted(set(traces[False]))} (OCCUPIED_AT {OCCUPIED_AT}); "
          f"with the lidar's and the camera's rays through it: {sorted(set(traces[True]))}; never a wall")
    print(f"  camera cell at FREE_FLOOR {FREE_FLOOR}, 5 frames from one spot: scores {floor_trace}; "
          f"a wall after {k} viewpoints (as on the lidar layer)")


def test_camera_wall_no_longer_seen_is_carved_free() -> None:
    """The passer-by's legs: a camera wall the camera no longer sees keeps
    its score for LOW_HIT_PROTECT_S however many rays cross it, then the
    camera's own rays through its cell at low height free it, one miss per
    look; once forgotten, the same spot cannot raise it back to a wall."""
    cam = camera_xy(SPOT_B, HEADING)
    assert _in_view(SHOE, SPOT_B, HEADING, margin_deg=-1.0).all(), "the shoe is well inside the camera's view"
    counts = {}
    for viewpoints in (2, 6):
        g = _camera_wall(viewpoints, t=100.0)
        for k in range(1, 11):                                   # 2 s of looks through it, inside the window
            g.camera_rays(BEYOND, cam, now=100.0 + 0.2 * k)
        assert _camera_score(g, SHOE) == viewpoints, "the 3 s window: a cell hit 2 s ago is not carved"
        n = 0
        while g.value_at(*SHOE[0]) == 100:
            n += 1
            g.camera_rays(BEYOND, cam, now=100.0 + LOW_HIT_PROTECT_S + 0.2 * n)
            assert n <= 10, f"a camera wall from {viewpoints} viewpoints still stands after 10 looks"
        m = n
        while _camera_score(g, SHOE) > 0:
            m += 1
            g.camera_rays(BEYOND, cam, now=100.0 + LOW_HIT_PROTECT_S + 0.2 * m)
            assert m <= 20
        counts[viewpoints] = (n, m)
    assert counts == {2: (1, 2), 6: (5, 6)}, counts
    g = _camera_wall(2, t=100.0)
    t = 100.0 + LOW_HIT_PROTECT_S
    while _camera_score(g, SHOE) > 0:
        t += 0.2
        g.camera_rays(BEYOND, cam, now=t)
    for k in range(10):
        t += 0.2
        g.camera_obstacles(_frame(SHOE, 20 + k), SPOT_B, now=t)
    assert g.value_at(*SHOE[0]) != 100, "forgotten, then seen again from the same spot: not a wall"
    print(f"  camera wall no longer seen: 10 looks through it inside {LOW_HIT_PROTECT_S:.0f} s -> untouched; then "
          f"2 viewpoints -> free after {counts[2][0]} look (score 0 after {counts[2][1]}); 6 viewpoints -> free after "
          f"{counts[6][0]} (score 0 after {counts[6][1]}); seen again from the same spot afterwards -> not a wall")


def test_camera_wall_still_seen_survives_the_rays() -> None:
    """The shoe the camera keeps seeing from one spot, 20 frames (4 s, past
    the 3 s window): every frame stamps its cell again, so the camera's own
    floor rays through it, the floor sampled beside it on its own cell and
    the lidar's rays take nothing; it stays a wall at the score two
    viewpoints gave it. Stop seeing it, and the same looks free it once the
    window is over."""
    g = _camera_wall(2, t=100.0)
    cam = camera_xy(SPOT_B, HEADING)
    dips = 0
    for k in range(20):
        t = 100.0 + 0.2 * (k + 1)
        before = _camera_score(g, SHOE)
        g.camera_obstacles(_frame(SHOE, 10 + k), SPOT_B, now=t)
        g.camera_rays(BEYOND, cam, now=t)
        g.camera_floor(SHOE + [0.02, 0.0], now=t)                  # the floor beside it, on its own cell
        g.lidar_revolution(WALL, SPOT_B)
        dips += _camera_score(g, SHOE) < before
        assert g.value_at(*SHOE[0]) == 100, f"the wall the camera still sees fell at frame {k + 1}"
    assert dips == 0 and _camera_score(g, SHOE) == 2, (dips, _camera_score(g, SHOE))
    t_last = 100.0 + 0.2 * 20                                      # the last frame's stamp
    looks = 0
    while g.value_at(*SHOE[0]) == 100:                             # unseen now: a look every 0.25 s
        looks += 1
        g.camera_rays(BEYOND, cam, now=t_last + 0.25 * looks)
        assert looks <= 30
    assert looks == round(LOW_HIT_PROTECT_S / 0.25), f"freed at look {looks}: the first one 3 s after the last frame"
    print(f"  camera wall kept in view from one spot: a wall for 20/20 frames (4 s) under its own floor rays, "
          f"the floor beside it and the lidar's rays; unseen afterwards -> free at look {looks}, "
          f"{0.25 * looks:.2f} s after the last frame")


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


# --- the camera layer is carved only where the camera looks; body_clear frees it ---
_tx, _ty = np.meshgrid(np.arange(1.525, 2.30, 0.05), np.arange(-0.475, 0.50, 0.05))
TABLE = np.column_stack([_tx.ravel(), _ty.ravel()])            # a table top: 16 x 20 cell centres, 1.5-2.3 m ahead
RING = np.column_stack([2.9 * np.cos(np.radians(np.arange(0, 360, 0.5))),
                        2.9 * np.sin(np.radians(np.arange(0, 360, 0.5)))])   # lidar returns 2.9 m all around


def _cells_scores(g: ScoredGrid, xy: np.ndarray, layer: str = "camera") -> np.ndarray:
    gx, gy = g.cell(xy[:, 0], xy[:, 1])
    return getattr(g, layer)[gy, gx].astype(int)


def _turn(g: ScoredGrid, rover_xy, yaw: float, obstacle: np.ndarray | None, t: float, k: int) -> int:
    """One revolution as MossCostMap.handle_lidar runs it, carrying a new depth
    frame of the part of `obstacle` inside the camera's view (none: a frame
    with no obstacle). Returns how many points the camera saw."""
    seen = obstacle[_in_view(obstacle, rover_xy, yaw)] if obstacle is not None else np.zeros((0, 2))
    g.camera_obstacles(_frame(seen, k) if len(seen) else seen, rover_xy, now=t)
    g.lidar_revolution(RING + rover_xy, rover_xy)
    return len(seen)


def _look(g: ScoredGrid, rover_xy, yaw: float, t: float) -> None:
    """One depth frame of bare floor from this pose: its samples and its rays."""
    floor = _floor_view(rover_xy, yaw)
    g.camera_floor(floor[:, :2], now=t)
    g.camera_rays(floor, camera_xy(rover_xy, yaw), now=t)


def test_the_carve_stays_in_the_cameras_view() -> None:
    """The camera layer holds what the scan plane misses (a table top, a
    chair seat, a shoe), and only the camera clears it, so it is carved only
    where the camera looks.

    Known input -> known output: a table top seen from 3 viewpoints (score 3
    on 320 cells); the rover pivots 180 degrees on the spot in 10 frames, then
    faces away for 20 (4 s, past the 3 s window), the lidar's rays crossing
    every table cell -> all 320 cells still score 3. Facing it again with the
    table gone, the camera's floor frees it."""
    g = fresh()
    t, k = 100.0, 0
    g.camera_obstacles(_frame(TABLE, k), (-0.30, 0.0), now=t)        # the gate's first frame
    for x in (-0.30, -0.15, 0.0):                                  # the approach, facing the table
        t, k = t + 0.2, k + 1
        assert _turn(g, (x, 0.0), 0.0, TABLE, t, k) == len(TABLE)
    assert (_cells_scores(g, TABLE) == 3).all(), _cells_scores(g, TABLE)
    seen_per_rev = []
    for j in range(1, 11):                                         # pivot on the spot, 18 deg per frame
        t, k = t + 0.2, k + 1
        seen_per_rev.append(_turn(g, (0.0, 0.0), math.radians(18.0 * j), TABLE, t, k))
        assert (_cells_scores(g, TABLE) == 3).all(), f"table cells carved at yaw {18 * j} deg"
    assert seen_per_rev[0] == len(TABLE) and seen_per_rev[-1] == 0, seen_per_rev
    for _ in range(20):                                            # facing away: the table is behind the rover
        t, k = t + 0.2, k + 1
        _turn(g, (0.0, 0.0), math.pi, TABLE, t, k)
    walls = int((_cells_scores(g, TABLE) >= OCCUPIED_AT).sum())
    assert walls == len(TABLE) and (_cells_scores(g, TABLE) == 3).all(), walls
    assert (_cells_scores(g, TABLE, "lidar") < 0).all(), "the lidar's rays did cross every table cell"
    # facing it again with the table taken away: the camera's floor frees it
    probe = copy.deepcopy(g)
    gone = 0
    while (_cells_scores(probe, TABLE) >= OCCUPIED_AT).any():
        gone += 1
        _look(probe, (0.0, 0.0), 0.0, t + 0.2 * gone)
        assert gone <= 5, int((_cells_scores(probe, TABLE) >= OCCUPIED_AT).sum())
    assert gone <= 2, gone

    # a passer-by's legs 35 deg off the heading, 1.2 m away: in the camera's view
    legs = np.array([[1.025, 0.625]])
    assert _in_view(legs, (0.0, 0.0), 0.0, margin_deg=-5.0).all() and not _in_view(legs, (0.0, 0.0), math.pi).any()
    kept = None
    freed = {}
    for yaw, label in ((0.0, "facing"), (math.pi, "away")):
        h = fresh()
        _armed(h, legs, 100.0, viewpoints=((0.0, -0.15), (0.0, 0.0)))
        assert h.value_at(*legs[0]) == 100
        n = 0
        while h.value_at(*legs[0]) == 100 and n < 20:              # gone: the camera sees the floor in its view
            n += 1
            _look(h, (0.0, 0.0), yaw, 100.0 + LOW_HIT_PROTECT_S + 0.2 * n)
            h.lidar_revolution(RING, (0.0, 0.0))
        freed[label] = n if h.value_at(*legs[0]) != 100 else None
        if label == "away":
            kept = h
    assert freed["facing"] is not None and freed["facing"] <= 2 and freed["away"] is None, freed
    m = 0
    while kept.value_at(*legs[0]) == 100:                          # the rover turns back toward where they stood
        m += 1
        _look(kept, (0.0, 0.0), 0.0, 120.0 + 0.2 * m)
        assert m <= 5
    # through the module: handle_camera_floor casts from the camera turned by the odom heading
    module = {}
    for yaw in (math.pi, 0.0):
        cm = costmap()
        asyncio.run(cm.handle_odom(_odom(0.0, 0.0)))
        asyncio.run(cm.handle_explore_cmd(Bool(data=True)))
        top = np.array([[1.525, 0.325, 0.72]])                     # a table top corner: above the scan plane
        for f, y in enumerate((0.0, 0.0, 0.15)):                   # the gate's frame, then two viewpoints
            asyncio.run(cm.handle_odom(_odom(0.0, y)))
            asyncio.run(cm.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(top, f)]))))
        assert cm._grid.value_at(1.525, 0.325) == 100, "two viewpoints make the table top a wall"
        asyncio.run(cm.handle_odom(_odom(0.0, 0.15, yaw)))
        cm._grid._last_low_hit[:] = time.monotonic() - LOW_HIT_PROTECT_S - 1.0   # the camera stopped reporting it
        n = 0
        while cm._grid.value_at(1.525, 0.325) == 100 and n < 10:
            n += 1
            asyncio.run(cm.handle_camera_floor(_cloud(_floor_view((0.0, 0.15), yaw))))
            asyncio.run(cm.handle_lidar(_cloud(_lidar_cloud(ROOM))))
        module[round(math.degrees(yaw))] = n if cm._grid.value_at(1.525, 0.325) != 100 else None
    assert module[180] is None and module[0] is not None and module[0] <= 2, module
    print(f"  table top, 320 cells at score 3: points in view while turning {seen_per_rev}; after the turn and "
          f"20 frames facing away under the lidar's rays -> {walls}/320 still a wall; facing it with the table gone "
          f"-> free after {gone} floor frame(s)")
    print(f"  legs 35 deg off the heading: freed after {freed['facing']} floor frame(s) in view; behind the rover a "
          f"wall for 20; turning back -> freed after {m}; MossCostMap with odom yaw 180 deg -> kept 10, yaw 0 -> "
          f"freed after {module[0]}")


def test_body_clear_frees_the_camera_layer() -> None:
    """What the body drove over is free on both layers, and a camera wall
    there does not come back from one spot: same-spot frames add nothing,
    new viewpoints lift it to OCCUPIED_AT - 1, and the same spot holds it
    there."""
    g = _camera_wall(2, t=100.0)
    gx, gy = g.cell(SHOE[:, 0], SHOE[:, 1])
    g.lidar[gy[0], gx[0]] = HIT_CAP                                 # a lidar ghost on the same cell
    g.body_clear((SHOE[0, 0], SHOE[0, 1], 0.0))                    # the rover drove over the shoe's cell
    assert _camera_score(g, SHOE) == FREE_FLOOR and g.lidar[gy[0], gx[0]] == FREE_FLOOR and g.value_at(*SHOE[0]) == 0
    right_away = []
    for k in range(10):                                            # the last viewpoint, 10 frames
        g.camera_obstacles(_frame(SHOE, 10 + k), SPOT_B, now=101.0 + 0.2 * k)
        right_away.append(_camera_score(g, SHOE))
    assert right_away == [FREE_FLOOR] * 10, right_away
    k = 0
    while _camera_score(g, SHOE) < OCCUPIED_AT - 1:                # new viewpoints lift it back
        k += 1
        g.camera_obstacles(_frame(SHOE, 30 + k), (0.0, SPOT_B[1] - 0.15 * k), now=104.0 + 0.2 * k)
        assert k <= 10
    assert k == OCCUPIED_AT - 1 - FREE_FLOOR, k
    last = (0.0, SPOT_B[1] - 0.15 * k)
    after = []
    for j in range(10):
        g.camera_obstacles(_frame(SHOE, 50 + j), last, now=106.0 + 0.2 * j)
        after.append(_camera_score(g, SHOE))
        assert g.value_at(*SHOE[0]) != 100, "a wall from one spot after body_clear"
    assert after == [OCCUPIED_AT - 1] * 10, after
    print(f"  camera wall under the body -> score {FREE_FLOOR} on both layers; 10 same-spot frames -> "
          f"{sorted(set(right_away))}; {k} new viewpoints to {OCCUPIED_AT - 1}, then 10 same-spot frames -> "
          f"{sorted(set(after))}")


# --- the three camera guards: thin legs, moving things, the camera's own rays ---
def test_thin_leg_survives_the_floor_sampled_beside_it() -> None:
    """A table leg thinner than a 5 cm cell, seen ten times, sat at -3: the
    camera calls the cell an obstacle in one frame and bare floor in the next
    (5 cm of quantisation), and the two cancelled forever. A cell the camera
    hit is deaf to floor samples for LOW_HIT_PROTECT_S; past that window it is
    forgotten exactly as before."""
    g = fresh()
    leg = np.array([[1.025, 0.025]])
    t = 1000.0                                      # an injected clock, in seconds
    _armed(g, leg, t)
    gx, gy = g.cell(leg[:, 0], leg[:, 1])
    assert g.camera[gy[0], gx[0]] == 2 and g.value_at(*leg[0]) == 100, "two hits, two viewpoints"
    for i in range(25):                             # 2.5 s of floor samples on its own cell
        g.camera_floor(leg, now=t + 0.1 + 0.1 * i)
    assert g.camera[gy[0], gx[0]] == 2, "a leg seen 0.1 s ago is not erased by the floor beside it"
    assert g.value_at(*leg[0]) == 100
    assert g.lidar[gy[0], gx[0]] == 0, "the camera's floor never touches the lidar layer"
    n = 0                                           # nothing hits it any more
    while g.value_at(*leg[0]) == 100:
        n += 1
        g.camera_floor(leg, now=t + LOW_HIT_PROTECT_S + n)
        assert n < 10
    assert g.camera[gy[0], gx[0]] < OCCUPIED_AT and n == 1, n
    # through MossCostMap: a parked rover sees the leg and, in every frame, the
    # floor on the leg's own cell; without the window the first floor frame frees it
    traces = {}
    for protect in (LOW_HIT_PROTECT_S, 0.0):
        cmod.LOW_HIT_PROTECT_S = protect
        try:
            m = costmap()
            asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
            asyncio.run(m.handle_explore_cmd(Bool(data=True)))
            leg3 = np.array([[1.025, 0.025, 0.40]])
            for k, y in enumerate((0.0, 0.0, 0.15)):            # the gate's frame, then two viewpoints
                asyncio.run(m.handle_odom(_odom(0.0, y)))
                asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(leg3, k)]))))
            assert m._grid.value_at(1.025, 0.025) == 100
            trace = []
            for k in range(10):                                  # 10 frames, parked
                asyncio.run(m.handle_camera_floor(_cloud(np.array([[1.04, 0.04, 0.0]]))))
                asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(leg3, 3 + k)]))))
                trace.append(m._grid.value_at(1.025, 0.025))
            traces[protect] = trace
        finally:
            cmod.LOW_HIT_PROTECT_S = LOW_HIT_PROTECT_S
    assert traces[LOW_HIT_PROTECT_S] == [100] * 10, traces
    assert 100 not in traces[0.0], traces
    print(f"  thin leg: 25 floor samples inside the {LOW_HIT_PROTECT_S:.0f} s window -> still a wall, the lidar "
          f"layer untouched; {n} past it -> forgotten; MossCostMap, leg and floor on its cell for 10 frames -> "
          f"a wall for all 10 (without the window: free from the first frame)")


def test_moving_object_never_writes_the_map() -> None:
    """A cell takes a camera hit only if the previous depth frame also saw an
    obstacle within one cell of it: a rolling vacuum or a walking person moves
    on before the second look, a standing box repeats, a thin leg one cell off
    still counts. The gate counts depth frames, not lidar revolutions:
    lidar_odometry carries one frame on every revolution for 0.5 s."""
    g = fresh()
    path = (1.025, 1.225, 1.425, 1.625, 1.825)
    for k, x in enumerate(path):                    # rolls by 20 cm per frame, two viewpoints anyway
        g.camera_obstacles(np.array([[x, 0.025]]), (0.0, 0.15 * k), now=100.0 + 0.2 * k)
    for x in path:
        assert g.value_at(x, 0.025) != 100 and _camera_score(g, np.array([[x, 0.025]])) == 0, \
            f"a moving object left a hit at {x} m"
    assert np.isinf(g._last_low_hit).all(), \
        "cells the gate held back are not stamped: a walker's trail stays open to the camera's floor and rays"
    box = fresh()                                   # a box STANDS there: three frames, two viewpoints
    _armed(box, np.array([[1.025, 0.025]]), 100.0)
    assert box.value_at(1.025, 0.025) == 100, "a standing box must still map"
    thin = fresh()                                  # a thin leg re-projected one cell off, diagonally
    thin.camera_obstacles(np.array([[1.025, 0.025]]), (0.0, 0.0), now=100.0)
    thin.camera_obstacles(np.array([[1.075, 0.075], [1.525, 0.525]]), (0.0, 0.0), now=100.2)   # + a walker, new
    assert _camera_score(thin, np.array([[1.075, 0.075]])) == 1, "one cell of tolerance"
    assert _camera_score(thin, np.array([[1.525, 0.525]])) == 0, "the walker in the same frame is held back"
    tx, ty = thin.cell(np.array([1.025, 1.075, 1.525]), np.array([0.025, 0.075, 0.525]))
    assert list(thin._last_low_hit[ty, tx]) == [-np.inf, 100.2, -np.inf], \
        "only the cell the gate kept is stamped, not the walker held back in the same frame"
    # through MossCostMap: ONE depth frame (somebody 1.1 m away) carried by 5
    # revolutions while the rover drives 12 cm, after a frame that saw
    # something elsewhere
    walker = np.array([[1.025, 0.525, 0.50]])
    elsewhere = np.array([[2.025, -1.025, 0.50]])
    hits = {}
    spy, real = LogSpy(), cmod.logger
    cmod.logger = spy
    try:
        for distinct in (False, True):
            m = costmap()
            asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
            asyncio.run(m.handle_explore_cmd(Bool(data=True)))
            asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), elsewhere]))))
            for k in range(5):                      # 10 Hz, 3 cm per revolution
                asyncio.run(m.handle_odom(_odom(0.03 * k, 0.0)))
                pts = _frame(walker, k + 1) if distinct else walker
                asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), pts]))))
            hits[distinct] = _camera_score(m._grid, walker)
            if not distinct:
                line = [s for s in spy.lines if s.startswith("costmap: camera obstacles:")][-1]
    finally:
        cmod.logger = real
    assert hits == {False: 0, True: 1}, hits
    assert line == ("costmap: camera obstacles: 2 points, 0 cell hits over the last 5 revolutions (2 depth frames, "
                    "3 repeats skipped, 2 cells held back by the moving-object gate)"), line
    # a revolution without camera points: no frame of the last 0.5 s saw an obstacle, the gate forgets
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    standing = np.array([[1.525, 0.025, 0.30]])
    asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(standing, 0)]))))
    asyncio.run(m.handle_lidar(_cloud(_lidar_cloud(ROOM))))
    asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(standing, 1)]))))
    assert _camera_score(m._grid, standing) == 0, "the frame before saw nothing: no hit"
    asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(standing, 2)]))))
    assert _camera_score(m._grid, standing) == 1, "two frames in a row: a hit"
    # the repeat window is read on the clouds' own stamps: the same bytes 0.4 s
    # later are the frame lidar_odometry re-attached; 1.5 s later (it never
    # re-attaches a frame past 0.5 s) they are a new look, and the gate passes
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    box = np.vstack([_lidar_cloud(ROOM), standing])
    t0 = time.time()
    window = []
    for dt in (0.0, 0.4, 1.5):
        asyncio.run(m.handle_lidar(_cloud(box, ts=t0 + dt)))
        window.append((_camera_score(m._grid, standing), m._camera_frames, m._camera_repeats))
    assert window == [(0, 1, 0), (0, 1, 1), (1, 2, 1)], window
    print("  vacuum rolling 20 cm per frame -> no hit on its path, no stamp; standing box -> mapped; a leg one cell "
          "off -> kept, only its kept cell stamped (not a walker held back in that frame); one depth frame on 5 revolutions over 12 cm -> 0 hits (as 5 "
          f"frames it would hit: {hits[True]}); the same bytes 0.4 s later -> a repeat, 1.5 s later -> a new frame "
          "(a hit); an empty revolution resets the gate")


def test_the_cameras_rays_carve_the_camera_layer_only() -> None:
    """Every cell a camera ray crosses at low height held nothing standing:
    a miss on the camera layer, from the camera (0.156, 0.025), 7.5 cm up.
    A floor point at 3.025 m: the ray is inside the band (0.013-0.060 m)
    from 20 % to 82 % of its 2.869 m, i.e. the cells from 0.75 to 2.50 m."""
    cam = camera_xy((0.0, 0.025), 0.0)
    assert cam == (CAM_X, 0.025), cam
    g = fresh()
    g.camera_rays(BEYOND, cam, now=1000.0)
    ys, xs = np.nonzero(g.camera < 0)
    edges = np.round(g.ox + xs * g.res, 3)
    assert set(np.round(g.oy + ys * g.res, 3)) == {0.0} and edges.min() == 0.75 and edges.max() == 2.50 \
        and len(xs) == 36, (sorted(set(edges)), len(xs))
    assert g.value_at(3.025, 0.025) == -1 and g.value_at(0.725, 0.025) == -1 and g.value_at(2.575, 0.025) == -1
    assert (g.lidar == 0).all(), "camera rays never touch the lidar layer"
    # a ghost at 2.0 m, armed by the camera: one look decides
    ghost = np.array([[2.025, 0.025]])
    out = {}
    for label, end, after in (("floor 3 m ahead", 0.0, LOW_HIT_PROTECT_S + 1.0),
                              ("a 4 cm cable 3 m ahead", 0.04, LOW_HIT_PROTECT_S + 1.0),
                              ("a 0.30 m box 3 m ahead", 0.30, LOW_HIT_PROTECT_S + 1.0),
                              ("floor, 2 s after the hit", 0.0, 2.0)):
        h = fresh()
        _armed(h, ghost, 100.0)
        h.camera_rays(np.array([[3.025, 0.025, end]]), cam, now=100.0 + after)
        out[label] = h.value_at(*ghost[0])
    # heights at the ghost's cell (1.85 m out of 2.869): floor 2.7 cm, cable 5.2 cm, box 22 cm
    assert out == {"floor 3 m ahead": 0, "a 4 cm cable 3 m ahead": 0, "a 0.30 m box 3 m ahead": 100,
                   "floor, 2 s after the hit": 100}, out
    # a lidar wall: the camera's rays and floor leave it; a camera wall: the lidar's rays leave it
    w = fresh()
    w.lidar_revolution(np.array([[2.025, 0.025]]), (0.01, 0.025))
    w.lidar_revolution(np.array([[2.025, 0.025]]), (0.01, 0.175))
    assert w.value_at(2.025, 0.025) == 100
    w.camera_rays(BEYOND, cam, now=200.0)
    w.camera_floor(np.array([[2.025, 0.025]]), now=200.0)
    assert w.value_at(2.025, 0.025) == 100, "only a lidar ray may retract a lidar claim"
    b = fresh()
    _armed(b, ghost, 100.0)
    for _ in range(20):
        b.lidar_revolution(np.array([[3.025, 0.025]]), (0.01, 0.025))
    bx, by = b.cell(ghost[:, 0], ghost[:, 1])
    assert b.lidar[by[0], bx[0]] < 0 and b.value_at(*ghost[0]) == 100, \
        "20 lidar rays through a camera wall: a ray 0.20 m up does not prove a low box gone"
    # through MossCostMap, the frame path: a 5 cm door threshold 1.5 m ahead (an
    # obstacle for lidar_odometry: its floor band is 4.6 cm there), a ghost at
    # 1.225 m the rover saw from many places (score 5); the threshold's ray
    # crosses it 5.5 cm up, inside the band. ONE depth frame carried by 3
    # revolutions is one look: one miss; a new frame is a second look
    sill = np.array([[1.525, 0.025, 0.05]])
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    sx, sy = m._grid.cell(np.array([1.225]), np.array([0.025]))
    m._grid.camera[sy[0], sx[0]] = 5
    trail = [int(m._grid.camera[sy[0], sx[0]])]
    for k in (0, 0, 0, 1):
        asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(sill, k)]))))
        trail.append(int(m._grid.camera[sy[0], sx[0]]))
    assert trail == [5, 4, 4, 4, 3], trail
    # hits before rays inside a frame: a thin leg at 1.225 m, then a frame with
    # the leg and the threshold behind it. The gate passes the leg, its hit
    # stamps it, and the threshold's low ray through the leg's cell skips it:
    # +1. Rays first would carve it before the hit (-1 + 1 = 0)
    m = costmap()
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    leg = np.array([[1.225, 0.025, 0.40]])
    asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(leg, 0)]))))
    first = _camera_score(m._grid, leg)
    asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), _frame(leg, 1), sill]))))
    assert (first, _camera_score(m._grid, leg)) == (0, 1), (first, _camera_score(m._grid, leg))
    print(f"  floor 3 m ahead -> cells {edges.min():.2f}-{edges.max():.2f} m carved ({len(xs)}), not its endpoint; "
          f"ghost at 2 m: floor ray / cable ray -> free, ray over it at 22 cm -> kept, 2 s after its hit -> kept; "
          f"lidar wall untouched by the camera, camera wall untouched by 20 lidar revolutions; MossCostMap: a ghost "
          f"at score 5 behind a threshold, one frame on 3 revolutions then a new one -> {trail}; a leg the frame "
          "confirms is not carved by that frame's ray (+1)")


def test_camera_rays_start_at_the_camera_not_the_base() -> None:
    """The rays start where the camera is: the profile's camera_xyz_base_m,
    turned by the heading (0.156 m ahead of base_link on MOSS). Cast from the
    base, a floor sample's ray opens the band 0.15 m too close and carves
    cells the camera's ray crosses above it."""
    assert camera_xy((0.0, 0.0), 0.0) == (CAM_X, CAM_Y), "facing +x: the camera is 0.156 m ahead"
    x, y = camera_xy((1.0, 2.0), math.pi / 2)
    assert abs(x - 1.0) < 1e-9 and abs(y - (2.0 + CAM_X)) < 1e-9, "turned 90 deg: the offset turns with the rover"
    os.environ["MOSS_CAMERA_XYZ_M"] = "-0.20 0 0.56"
    try:
        moved = camera_xy((0.0, 0.0), 0.0)
    finally:
        del os.environ["MOSS_CAMERA_XYZ_M"]
    assert moved == (-0.20, 0.0), "read from the profile at call time"

    def first_carved(origin) -> float:
        g = fresh()
        g.camera_rays(BEYOND, origin, now=1000.0)
        return round(g.ox + np.nonzero(g.camera < 0)[1].min() * g.res, 3)

    pose = (0.02, 0.025)
    from_camera, from_base = first_carved(camera_xy(pose, 0.0)), first_carved(pose)
    assert (from_camera, from_base) == (0.75, 0.60), (from_camera, from_base)
    # a ghost at 0.675 m: the camera's own ray crosses it 6.2 cm up, above the band
    ghost = np.array([[0.675, 0.025]])
    old = time.monotonic() - 10.0
    m = costmap()
    asyncio.run(m.handle_odom(_odom(*pose)))
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    m._grid = fresh()
    _armed(m._grid, ghost, old)
    asyncio.run(m.handle_camera_floor(_cloud(BEYOND)))
    assert m._grid.value_at(*ghost[0]) == 100, "the camera's ray crosses the ghost above the band: it stays"
    g = fresh()
    _armed(g, ghost, old)
    g.camera_rays(BEYOND, pose, now=time.monotonic())
    assert g.value_at(*ghost[0]) != 100, "cast from the base, the same look erases it"

    # through MossCostMap, the floor path: a ghost 1.5 m ahead, hit 10 s ago
    # (outside its window), dies to ONE floor sample 3 m ahead whose ray from
    # the camera crosses it 3.9 cm up. Facing +y, the camera sits at (0, 0.156)
    # and the same look along +y kills the same ghost turned by 90 deg; an
    # offset not turned with the heading, (0.156, 0), would pass a cell beside it
    stale = {}
    for yaw, ghost_xy, floor_xy in ((0.0, (1.525, 0.025), (3.025, 0.025)),
                                    (math.pi / 2, (0.025, 1.525), (0.025, 3.025))):
        m = costmap()
        asyncio.run(m.handle_odom(_odom(0.0, 0.0, yaw)))
        asyncio.run(m.handle_explore_cmd(Bool(data=True)))
        _armed(m._grid, np.array([ghost_xy]), old)
        before = (_camera_score(m._grid, np.array([ghost_xy])), m._grid.value_at(*ghost_xy))
        asyncio.run(m.handle_camera_floor(_cloud(np.array([[*floor_xy, 0.0]]))))
        stale[round(math.degrees(yaw))] = (before, (_camera_score(m._grid, np.array([ghost_xy])),
                                                    m._grid.value_at(*ghost_xy)))
    assert stale == {0: ((2, 100), (1, 0)), 90: ((2, 100), (1, 0))}, stale
    turned = m._camera_xy()
    assert abs(turned[0]) < 1e-9 and abs(turned[1] - CAM_X) < 1e-9, turned

    # through MossCostMap, the frame path: a 4 cm cable 0.95 m ahead of a rover
    # at x = 0.02 (an obstacle for lidar_odometry, whose floor band is 3 cm
    # there). The camera's own ray crosses the cell at 0.475 m 6.19 cm up, above
    # the band, and leaves a ghost there; a base-cast ray crosses it 5.85 cm up
    # and would carve it
    cable = np.array([[0.975, 0.025, 0.04]])
    frame_path = {}
    for label in ("camera", "base"):
        m = costmap()
        asyncio.run(m.handle_odom(_odom(*pose)))
        asyncio.run(m.handle_explore_cmd(Bool(data=True)))
        m._grid = fresh()
        cx, cy = m._grid.cell(np.array([0.475]), np.array([0.025]))
        m._grid.camera[cy[0], cx[0]] = 5
        if label == "base":
            m._camera_xy = lambda m=m: m._pose_xy
        asyncio.run(m.handle_lidar(_cloud(np.vstack([_lidar_cloud(ROOM), cable]))))
        frame_path[label] = int(m._grid.camera[cy[0], cx[0]])
    assert frame_path == {"camera": 5, "base": 4}, frame_path

    # the mount is read at call time, all three numbers: 5 cm to the left moves
    # the origin sideways (turned with the heading), and a camera 0.15 m up
    # moves the band with it, so a floor ray carves the same stretch of its
    # length, in the row of the moved origin
    os.environ["MOSS_CAMERA_XYZ_M"] = "0.156 0.05 0.15"
    try:
        left, left_north = camera_xy((0.0, 0.0), 0.0), camera_xy((0.0, 0.0), math.pi / 2)
        h = fresh()
        h.camera_rays(np.array([[3.025, 0.075, 0.0]]), camera_xy((0.0, 0.025), 0.0), now=1000.0)
    finally:
        del os.environ["MOSS_CAMERA_XYZ_M"]
    assert left == (CAM_X, 0.05), left
    assert abs(left_north[0] + 0.05) < 1e-9 and abs(left_north[1] - CAM_X) < 1e-9, left_north
    hy, hx = np.nonzero(h.camera < 0)
    rows, cols = set(np.round(h.oy + hy * h.res, 3)), np.round(h.ox + hx * h.res, 3)
    assert rows == {0.05} and (cols.min(), cols.max(), len(hx)) == (0.75, 2.50, 36), (rows, sorted(set(cols)))
    print(f"  camera at {camera_xy((0.0, 0.0), 0.0)} facing +x; floor at 3 m: carving starts at {from_camera:.2f} m "
          f"from the camera vs {from_base:.2f} m from the base; ghost at 0.675 m kept by the camera-cast ray, "
          "erased by a base-cast one; MossCostMap: a stale ghost at 1.5 m, floor 3 m ahead -> score 2 -> 1, free, "
          f"facing +x and +y (camera at {turned[0]:.2f}, {turned[1]:.3f}); a cable 0.95 m ahead -> ghost at 0.475 m "
          f"{frame_path['camera']} from the camera, {frame_path['base']} from the base; camera 5 cm left and 0.15 m "
          "up -> origin y 0.05 (-0.05 m in x facing +y), the floor ray still carves 0.75-2.50 m")


def test_the_carve_band_follows_the_camera_height() -> None:
    """The original band, 0.10-0.45 m for a camera 0.56 m up, as fractions of
    the camera height: on a floor sample's ray, the stretch from 20 % to 82 %
    of its range at any camera height. 0.013-0.060 m on MOSS's 0.075 m; the
    profile is read at call time."""
    lo, hi = carve_z_band()
    assert (round(lo, 4), round(hi, 4)) == (0.0134, 0.0603), (lo, hi)
    assert abs(lo - 0.075 * 0.10 / 0.56) < 1e-12 and abs(hi - 0.075 * 0.45 / 0.56) < 1e-12
    o_lo, o_hi = carve_z_band(0.56)
    assert abs(o_lo - 0.10) < 1e-12 and abs(o_hi - 0.45) < 1e-12, "the original band at the original height"
    for h in (0.075, 0.56):
        b_lo, b_hi = carve_z_band(h)
        assert abs((1 - b_hi / h) - 0.196) < 1e-3 and abs((1 - b_lo / h) - 0.821) < 1e-3
    os.environ["MOSS_CAMERA_XYZ_M"] = "0.156 0 0.15"
    try:
        r_lo, r_hi = carve_z_band()
    finally:
        del os.environ["MOSS_CAMERA_XYZ_M"]
    assert (round(r_lo, 4), round(r_hi, 4)) == (0.0268, 0.1205), (r_lo, r_hi)
    print(f"  band {lo:.4f}-{hi:.4f} m at 0.075 m; {o_lo:.2f}-{o_hi:.2f} m at 0.56 m; a floor ray carves 20-82 % "
          f"of its range at both; MOSS_CAMERA_XYZ_M z=0.15 -> {r_lo:.4f}-{r_hi:.4f} m")


def test_the_doctrine_defaults() -> None:
    """Without the environment the module takes a wall from two places 10 cm
    apart (a passer-by painted from one parked spot never walls the rover
    in), publishes every two revolutions and writes no contact into the map;
    the knobs still move them."""
    import subprocess
    code = ("import moss_dimos.costmap2d as c; lo, hi = c.carve_z_band(); "
            "print(c.OCCUPIED_AT, c.NEW_VIEWPOINT_M, c.PUBLISH_EVERY, c.CONTACT_WALL, c.LOW_HIT_PROTECT_S, "
            "f'{lo:.4f}', f'{hi:.4f}')")

    def run(**extra) -> list:
        env = {k: v for k, v in os.environ.items() if not k.startswith("MOSS_")}
        env["PYTHONPATH"] = os.environ.get("PYTHONPATH", "")
        env.update(extra)
        return subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env,
                              timeout=120).stdout.split()

    out = run()
    assert out == ["2", "0.1", "2", "False", "3.0", "0.0134", "0.0603"], out
    knobs = run(MOSS_OCCUPIED_AT="1", MOSS_NEW_VIEWPOINT_M="0.05", MOSS_CONTACT_WALL="1")
    assert knobs[:4] == ["1", "0.05", "2", "True"], knobs
    print("  defaults: OCCUPIED_AT 2, NEW_VIEWPOINT_M 0.1, PUBLISH_EVERY 2, contact wall off, thin-leg window 3 s, "
          "carve band 0.0134-0.0603 m; MOSS_OCCUPIED_AT=1 MOSS_NEW_VIEWPOINT_M=0.05 MOSS_CONTACT_WALL=1 -> "
          f"{' '.join(knobs[:4])} (this bench pins 2 / 0.10 / 5)")



# --- the premap: a relocalization's saved map, placed by its fix, as lidar walls ---
# The scene in `map`: a wall of 20 cells along x at y = -1.025, each cell at three heights
# inside the band (60 points), the floor layer of a 5 cm voxel premap (z 0.025) at y = -0.525
# and points above the band (z 1.325 and 1.5) at y = -1.525: 100 points, 20 wall cells.
# The fix (x 1 m, yaw 90 deg) puts map (mx, my) at world (1 - my, mx): the wall at x = 2.025.
PREMAP_XS = 0.025 + 0.05 * np.arange(20)
PREMAP_WALL = np.array([[mx, -1.025, z] for mx in PREMAP_XS for z in (0.075, 0.375, 1.275)])
PREMAP_FLOOR = np.array([[mx, -0.525, 0.025] for mx in PREMAP_XS])
PREMAP_HIGH = np.array([[mx, -1.525, 1.325 if k % 2 else 1.5] for k, mx in enumerate(PREMAP_XS)])
PREMAP_SCENE = np.vstack([PREMAP_WALL, PREMAP_FLOOR, PREMAP_HIGH])
FIX = (1.0, 0.0, 90.0)


def _premap_cloud(pts: np.ndarray):
    """A premap as it arrives off the bus: float32, frame `map`, through an LCM round trip."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    pc = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id="map", timestamp=time.time())
    return PointCloud2.lcm_decode(pc.lcm_encode())


def _tf(x: float, y: float, yaw_deg: float, parent: str = "world", child: str = "map"):
    """One transform on the tf stream, as dimOS's relocalization sends its fix, through an LCM round trip."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    h = math.radians(yaw_deg) / 2
    msg = TFMessage(Transform(translation=Vector3(x, y, 0.0), rotation=Quaternion(0.0, 0.0, math.sin(h), math.cos(h)),
                              frame_id=parent, child_frame_id=child))
    return TFMessage.lcm_decode(msg.lcm_encode())


def _premap_module(on: bool = True, pose=(0.0, 0.0)) -> MossCostMap:
    m = MossCostMap(premap=on)
    m.global_costmap = Sink()
    MODULES.append(m)
    if pose is not None:
        asyncio.run(m.handle_odom(_odom(*pose)))
    return m


def _cells(g: ScoredGrid, xy) -> set:
    gx, gy = g.cell(np.asarray(xy)[:, 0], np.asarray(xy)[:, 1])
    return set(zip(gy.tolist(), gx.tolist()))


def _nonzero(layer: np.ndarray) -> set:
    return set(map(tuple, np.argwhere(layer != 0).tolist()))


def _wall_world(dx: float = 0.0) -> np.ndarray:
    """The scene's wall in `world` under FIX, moved dx along world x."""
    return np.column_stack([np.full(20, 2.025 + dx), PREMAP_XS])


def _logged(fn):
    """Run fn with costmap2d's logger replaced; returns the lines it said."""
    spy, real = LogSpy(), cmod.logger
    cmod.logger = spy
    try:
        fn()
    finally:
        cmod.logger = real
    return spy.lines


def test_the_premap_needs_its_map_and_its_fix() -> None:
    """Nothing changes until BOTH a loaded_map and a `world` -> `map` fix
    have arrived; then the premap's points inside the band become walls on
    the lidar layer, at OCCUPIED_AT, at the cells the fix places them on.
    The mission gate stays shut through all of it: the premap is not a
    reading from where the rover stands."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic

    # the premap alone: kept and said, nothing written; the odometry's own tf and an inverse
    # map -> world transform change nothing
    m = _premap_module()
    lines = _logged(lambda: (asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE))),
                             asyncio.run(m.handle_tf(_tf(0.3, 0.1, 10.0, child="base_link"))),
                             asyncio.run(m.handle_tf(_tf(*FIX, parent="map", child="world")))))
    assert not _nonzero(m._grid.lidar) and not m._grid.premap.any() and not m.global_costmap.msgs, \
        "a loaded_map without a fix writes nothing (world -> base_link and map -> world are not fixes)"
    assert lines == ["costmap: premap received: 100 points, 60 in the 0.05-1.3 m band; "
                     "written when a world -> map fix arrives on tf"], lines
    assert m._premap_fix is None
    # the fix alone: said once, nothing written
    t = _premap_module()
    lines = _logged(lambda: (asyncio.run(t.handle_tf(_tf(*FIX))), asyncio.run(t.handle_tf(_tf(*FIX)))))
    assert not _nonzero(t._grid.lidar) and not t._grid.premap.any() and not t.global_costmap.msgs, \
        "a fix without a loaded_map writes nothing"
    assert lines == ["costmap: premap fix received on tf: (+1.00, +0.00, +90.0 deg); "
                     "written when a loaded_map arrives"], lines
    # both: 20 walls at the placed cells, nothing else on the lidar layer
    lines = _logged(lambda: asyncio.run(m.handle_tf(_tf(*FIX))))
    g = m._grid
    want = _cells(g, _wall_world())
    assert len(want) == 20 and _nonzero(g.lidar) == want, sorted(_nonzero(g.lidar) ^ want)[:5]
    assert set(map(tuple, np.argwhere(g.premap).tolist())) == want, "every wall cell is flagged premap"
    gy, gx = zip(*sorted(want))
    assert set(g.lidar[list(gy), list(gx)].tolist()) == {OCCUPIED_AT}, "written at OCCUPIED_AT, the lowest wall score"
    assert all(g.value_at(2.025, y) == 100 for y in PREMAP_XS), "the wall at world x = 2.025"
    assert not (_cells(g, PREMAP_WALL[:, :2]) & _nonzero(g.lidar)), "the map-frame positions themselves hold nothing"
    for name, pts in (("the floor layer (z 0.025)", PREMAP_FLOOR), ("above the band (z 1.325, 1.5)", PREMAP_HIGH)):
        placed = np.column_stack([1.0 - pts[:, 1], pts[:, 0]])
        assert not (_cells(g, placed) & _nonzero(g.lidar)), f"{name} is left out"
    assert m._mission_frozen, "the mission gate is still shut: the premap does not need it"
    assert not m.global_costmap.msgs, "nothing goes out before the mission (test 32)"
    head = "costmap: premap written: 20 walls from 100 points, tf (+1.00, +0.00, +90.0 deg)"
    assert len(lines) == 1 and lines[0].startswith(head), lines
    # the fix first, then the premap: the same 20 cells
    asyncio.run(t.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    assert np.array_equal(t._grid.lidar, g.lidar) and np.array_equal(t._grid.premap, g.premap), "order does not matter"
    # both before the first pose: written when the grid is born
    early = _premap_module(pose=None)
    asyncio.run(early.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(early.handle_tf(_tf(*FIX)))
    assert early._grid is None
    asyncio.run(early.handle_odom(_odom(0.0, 0.0)))
    assert np.array_equal(early._grid.lidar, g.lidar), "a premap and a fix that came before the first pose"
    # a different premap under the same fix replaces the old one: the wall 5 cm further along -y in
    # `map` is 5 cm further along +x in `world`, and the old cells are empty again
    other = PREMAP_SCENE.copy()
    other[:60, 1] -= 0.05
    lines = _logged(lambda: asyncio.run(t.handle_loaded_map(_premap_cloud(other))))
    moved = _cells(t._grid, _wall_world(0.05))
    assert set(map(tuple, np.argwhere(t._grid.premap).tolist())) == moved and _nonzero(t._grid.lidar) == moved, \
        "a different premap replaces the old one"
    assert len(lines) == 3 and lines[0] == "costmap: premap received: 100 points, 60 in the 0.05-1.3 m band" \
        and lines[1] == "costmap: premap cleared: 20 cells" and lines[2].startswith("costmap: premap written: 20 walls"), lines
    # a cell the lidar has seen free stays free
    f = _premap_module()
    fy, fx = sorted(want)[3]
    f._grid.lidar[fy, fx] = -2
    f._grid.seen[fy, fx] = True
    lines = _logged(lambda: (asyncio.run(f.handle_loaded_map(_premap_cloud(PREMAP_SCENE))),
                             asyncio.run(f.handle_tf(_tf(*FIX)))))
    assert f._grid.lidar[fy, fx] == -2 and not f._grid.premap[fy, fx] and int(f._grid.premap.sum()) == 19, \
        "the live map wins over the prior: a cell seen free is not overwritten"
    assert lines and lines[-1].startswith("costmap: premap written: 19 walls from 100 points") \
        and "1 cells the lidar has seen free left free" in lines[-1], lines
    # where they land on the bus: the canonical topics dimOS's relocalization publishes on
    assert str(LCMTopic("/loaded_map", PointCloud2)) == "/loaded_map#sensor_msgs.PointCloud2"
    assert str(LCMTopic("/tf", TFMessage)) == "/tf#tf2_msgs.TFMessage"
    # the cost of a room-sized premap (5 300 points over 15 x 8 m), written and published once
    rng = np.random.default_rng(1)
    big = np.column_stack([rng.uniform(-7.5, 7.5, 5300), rng.uniform(-1.5, 6.5, 5300), rng.uniform(0.07, 1.27, 5300)])
    b = _premap_module()
    asyncio.run(b.handle_loaded_map(_premap_cloud(big)))
    t0 = time.perf_counter()
    asyncio.run(b.handle_tf(_tf(0.3, -0.2, 30.0)))
    ms = (time.perf_counter() - t0) * 1000
    n_big = int(b._grid.premap.sum())
    assert n_big > 4000 and not b.global_costmap.msgs and ms < 100, (n_big, len(b.global_costmap.msgs), ms)
    print(f"  a premap alone, a fix alone, a map -> world tf: nothing written, each input said; both: 20 walls at "
          f"world x = 2.025 (score {OCCUPIED_AT}), floor layer and > 1.30 m left out, any order, before the first pose "
          f"too, gate still shut, nothing published; a cell seen free stays free (19 walls); a different premap "
          f"replaces the old one; /loaded_map and /tf; 5 300 points -> {n_big} walls written in {ms:.1f} ms")


def test_a_lidar_ray_carves_a_premap_wall_like_any_wall() -> None:
    """A premap wall is a lidar wall at OCCUPIED_AT: the room may have
    changed since the premap was made, and one lidar ray through the cell
    takes it out, exactly as it takes out a live wall at the same score."""
    m = _premap_module()
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(m.handle_tf(_tf(*FIX)))
    g = m._grid
    p, q = (2.025, 0.025), (2.025, -0.025)          # a premap cell, and a live wall right below it
    g._hit(g.lidar, np.array([q[0]]), np.array([q[1]]), (0.0, -1.0))
    g._hit(g.lidar, np.array([q[0]]), np.array([q[1]]), (0.0, -0.8))
    (py, px), = _cells(g, [p])
    (qy, qx), = _cells(g, [q])
    assert g.premap[py, px] and not g.premap[qy, qx]
    assert g.lidar[py, px] == g.lidar[qy, qx] == OCCUPIED_AT and g.value_at(*p) == g.value_at(*q) == 100
    behind = _lidar_cloud(np.array([[3.0, 0.025], [3.0, -0.025]]))   # returns behind both: each ray crosses one of them
    trace = []
    for _ in range(2 * RAY_EVERY):
        asyncio.run(m.handle_lidar(_cloud(behind)))
        trace.append((g.value_at(*p), g.value_at(*q)))
    assert trace[0] == (100, 100), trace
    assert trace[RAY_EVERY - 1] == (0, 0), f"both freed by the first carving revolution: {trace}"
    assert g.lidar[py, px] == g.lidar[qy, qx] == OCCUPIED_AT - 2 and g.seen[py, px], (g.lidar[py, px], g.lidar[qy, qx])
    assert g.value_at(2.025, 0.525) == 100, "a premap cell no ray crossed is still a wall"
    print(f"  premap cell and live wall at score {OCCUPIED_AT}: {trace} over {2 * RAY_EVERY} revolutions "
          f"(rays every {RAY_EVERY}): both free on the same one, the uncrossed premap cells stay walls")


def test_a_moved_fix_clears_and_rewrites_the_premap() -> None:
    """The premap is written again only when a fix differs from the one it
    was written with by more than 2 cm or 1 deg; its old cells lose exactly
    what the premap added first, so a corrected fix leaves no ghost wall."""
    m = _premap_module()
    g = m._grid
    # before the premap: a live wall from two viewpoints (OCCUPIED_AT, the premap adds nothing to it)
    # and a cell at HIT_CAP, both under premap points
    wy, cy_ = PREMAP_XS[2], PREMAP_XS[3]
    g._hit(g.lidar, np.array([2.025]), np.array([wy]), (0.0, 0.0))
    g._hit(g.lidar, np.array([2.025]), np.array([wy]), (0.0, 0.3))
    (wgy, wgx), = _cells(g, [(2.025, wy)])
    (cgy, cgx), = _cells(g, [(2.025, cy_)])
    g.lidar[cgy, cgx] = HIT_CAP
    g.seen[cgy, cgx] = True
    asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(m.handle_tf(_tf(*FIX)))
    old = _cells(g, _wall_world())
    assert g.lidar[wgy, wgx] == OCCUPIED_AT and g.lidar[cgy, cgx] == HIT_CAP and g.premap[wgy, wgx] and g.premap[cgy, cgx], \
        (g.lidar[wgy, wgx], g.lidar[cgy, cgx])
    (ay, ax), (by_, bx) = sorted(old)[0], sorted(old)[1]
    g._hit(g.lidar, np.array([2.025]), np.array([g.oy + (ay + 0.5) * g.res]), (0.0, 0.0))   # a live hit since
    g._miss(g.lidar, np.array([bx]), np.array([by_]))                                       # a ray through since
    assert g.lidar[ay, ax] == OCCUPIED_AT + 1 and g.lidar[by_, bx] == OCCUPIED_AT - 1
    unseen = [c for c in sorted(old) if not g.seen[c]]
    lines = _logged(lambda: asyncio.run(m.handle_tf(_tf(FIX[0] + 0.05, FIX[1], FIX[2]))))
    new = _cells(g, _wall_world(0.05))
    assert len(lines) == 3 and lines[0] == "costmap: premap re-written: tf moved by (5.0 cm, 0.0 deg)" \
        and lines[1] == "costmap: premap cleared: 20 cells" \
        and lines[2].startswith("costmap: premap written: 20 walls from 100 points, tf (+1.05, +0.00, +90.0 deg)"), lines
    assert set(map(tuple, np.argwhere(g.premap).tolist())) == new and not (new & old), "the wall moved 5 cm, one cell"
    assert g.lidar[ay, ax] == 1 and g.lidar[by_, bx] == -1, \
        f"what the sensors did stays: the live hit {g.lidar[ay, ax]} (1), the ray's miss {g.lidar[by_, bx]} (-1)"
    assert all(g.lidar[c] == 0 and g.value_at(g.ox + (c[1] + 0.5) * g.res, g.oy + (c[0] + 0.5) * g.res) == -1
               for c in unseen), "a cell no sensor saw is unknown again: no ghost wall, no ghost floor"
    assert len(unseen) == 16
    assert g.lidar[wgy, wgx] == OCCUPIED_AT and g.value_at(2.025, wy) == 100, \
        f"the live wall seen from two viewpoints is still a wall: {g.lidar[wgy, wgx]}"
    assert g.lidar[cgy, cgx] == HIT_CAP, f"HIT_CAP through the write and the clear: {g.lidar[cgy, cgx]}"
    snap = g.lidar.copy()
    quiet = _logged(lambda: (asyncio.run(m.handle_tf(_tf(FIX[0] + 0.06, FIX[1], FIX[2]))),     # 1 cm from the written fix
                             asyncio.run(m.handle_tf(_tf(FIX[0] + 0.05, FIX[1], FIX[2]))),     # the same fix again
                             asyncio.run(m.handle_tf(_tf(FIX[0] + 0.065, FIX[1], FIX[2] + 0.5))),  # 1.5 cm, 0.5 deg
                             asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))))   # the same premap again
    assert not quiet and np.array_equal(g.lidar, snap) and set(map(tuple, np.argwhere(g.premap).tolist())) == new, quiet
    creep = _logged(lambda: asyncio.run(m.handle_tf(_tf(FIX[0] + 0.075, FIX[1], FIX[2]))))   # 1 cm from the last one received
    assert creep and creep[0] == "costmap: premap re-written: tf moved by (2.5 cm, 0.0 deg)", \
        f"measured against the WRITTEN fix: {creep}"
    turn = _logged(lambda: asyncio.run(m.handle_tf(_tf(FIX[0] + 0.075, FIX[1], FIX[2] + 1.5))))
    assert turn and turn[0] == "costmap: premap re-written: tf moved by (0.0 cm, 1.5 deg)", turn
    # Clear map: the premap goes with the rest, and the same fix does not bring it back
    cleared = _logged(lambda: asyncio.run(m.handle_clear_map_cmd(Bool(data=True))))
    assert any(ln.startswith("costmap: premap cleared with the map") for ln in cleared), cleared
    assert not m._grid.premap.any() and not _nonzero(m._grid.lidar)
    again = _logged(lambda: asyncio.run(m.handle_tf(_tf(FIX[0] + 0.075, FIX[1], FIX[2] + 1.5))))
    assert not again and not m._grid.premap.any(), again
    back = _logged(lambda: asyncio.run(m.handle_tf(_tf(FIX[0] + 0.105, FIX[1], FIX[2] + 1.5))))
    assert back[0] == "costmap: premap re-written: tf moved by (3.0 cm, 0.0 deg)" and int(m._grid.premap.sum()) == 20, back
    # across +-180 deg: 0.8 deg apart is no move, 2 deg is
    w = _premap_module()
    asyncio.run(w.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(w.handle_tf(_tf(0.0, 0.0, 179.6)))
    wrap = _logged(lambda: asyncio.run(w.handle_tf(_tf(0.0, 0.0, -179.6))))
    turned = _logged(lambda: asyncio.run(w.handle_tf(_tf(0.0, 0.0, -178.4))))
    assert not wrap and turned and turned[0] == "costmap: premap re-written: tf moved by (0.0 cm, 2.0 deg)", (wrap, turned)
    print("  5 cm: re-written, 20 cells cleared (live hit 3 -> 1, ray's miss 1 -> -1, a two-viewpoint wall still "
          "100, HIT_CAP kept, 16 unseen cells unknown again), 20 written one cell over; 1 cm, the same fix, "
          "1.5 cm + 0.5 deg, the same premap: nothing; creeping to 2.5 cm from the written fix: re-written; 1.5 deg "
          "alone: re-written; +179.6 -> -179.6 deg: nothing, -178.4: re-written; Clear map: gone until a fix moves 3 cm")


def test_the_premap_off_ignores_and_says_so_once() -> None:
    """premap=False (MOSS_PREMAP=0, the blueprints' default and the
    module's): the In exists, every loaded_map is ignored, and the log says
    so once."""
    m = _premap_module(on=False)
    assert MossCostMap().premap_on is False, "the module's own default is off"
    lines = _logged(lambda: [asyncio.run(f) for f in (m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)),
                                                       m.handle_tf(_tf(*FIX)),
                                                       m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)),
                                                       m.handle_tf(_tf(FIX[0] + 0.5, 0.0, 0.0)),
                                                       m.handle_loaded_map(_premap_cloud(PREMAP_SCENE[:10])))])
    assert lines == ["costmap: premap ignored (MOSS_PREMAP=0)"], lines
    assert not _nonzero(m._grid.lidar) and not m._grid.premap.any() and not m.global_costmap.msgs
    assert m._premap_xy is None and m._premap_fix is None
    print("  premap off: 3 loaded_maps and 2 fixes -> nothing written, nothing published, one line: "
          "'costmap: premap ignored (MOSS_PREMAP=0)'")


def test_the_premap_never_touches_the_camera_layer() -> None:
    """The premap writes the lidar layer only, and leaves `seen` alone: a
    write, a re-write and a clear leave the camera layer and the seen mask
    byte for byte as they were."""
    m = _premap_module()
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    g = m._grid
    shoe = np.array([[2.025, 0.125]])                   # a camera wall on a premap cell, and one beside the wall
    _armed(g, np.vstack([shoe, [[1.525, 0.325]]]), t=100.0, viewpoints=((0.0, 0.0), (0.0, 0.15)))
    g.body_clear((0.0, 0.0, 0.0))
    cam0, seen0 = g.camera.copy(), g.seen.copy()
    assert _cells(g, shoe) <= _nonzero(cam0) and int((cam0 > 0).sum()) == 2
    asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(m.handle_tf(_tf(*FIX)))
    after_write = (np.array_equal(g.camera, cam0), np.array_equal(g.seen, seen0))
    asyncio.run(m.handle_tf(_tf(FIX[0] + 0.05, FIX[1], FIX[2])))
    after_rewrite = (np.array_equal(g.camera, cam0), np.array_equal(g.seen, seen0))
    n = g.clear_premap()
    after_clear = (np.array_equal(g.camera, cam0), np.array_equal(g.seen, seen0))
    assert after_write == after_rewrite == after_clear == (True, True) and n == 20, (after_write, after_rewrite, after_clear)
    print("  a camera wall on a premap cell and one beside: camera layer and seen mask identical after the write, "
          "the re-write and the clear (20 cells)")


def test_the_published_grid_carries_the_premap() -> None:
    """Nothing goes out before the mission, premap or not: a grid of premap
    walls with no free floor reads to explorer2 as a walled-in start (a
    back-off, then "no reachable frontier left"). The premap goes out with
    the map's first grid of the mission: walls 100, the floor the lidar saw
    0, the floor beside the walls -1 (no sensor saw it). Once a grid has gone
    out, a re-write publishes at once, the old cells -1 again; after Clear
    map, a re-write waits for the map's next grid."""
    m = _premap_module()
    asyncio.run(m.handle_loaded_map(_premap_cloud(PREMAP_SCENE)))
    asyncio.run(m.handle_tf(_tf(*FIX)))
    for _ in range(3):
        asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    assert int(m._grid.premap.sum()) == 20 and not m.global_costmap.msgs, "written, not published, before the mission"
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    # the mission is open and the body's footprint is seen free, but the map has put no grid out:
    # a re-write still waits (a grid of walls and one footprint of floor is a walled-in start too)
    asyncio.run(m.handle_odom(_odom(0.0, 0.0)))
    asyncio.run(m.handle_tf(_tf(FIX[0] - 0.05, FIX[1], FIX[2])))
    assert m._grid.seen.any() and int(m._grid.premap.sum()) == 20 and not m.global_costmap.msgs, \
        "no grid before the map's own first one, even with the footprint seen"
    asyncio.run(m.handle_tf(_tf(*FIX)))
    # lidar returns behind the rover (x = -1 m): their rays never cross the premap wall at x = 2.025
    behind = _lidar_cloud(np.array([[-1.0, y] for y in np.arange(-0.5, 0.55, 0.1)]))
    for k in range(PUBLISH_EVERY):
        assert not m.global_costmap.msgs, f"nothing before the map's first grid (revolution {k})"
        asyncio.run(m.handle_lidar(_cloud(behind)))
    out = m.global_costmap.msgs
    assert len(out) == 1, len(out)

    def at(og, x, y):
        grid = np.asarray(og.grid)
        ox, oy = og.origin.position.x, og.origin.position.y
        return int(grid[int(math.floor((y - oy) / RESOLUTION_M)), int(math.floor((x - ox) / RESOLUTION_M))])

    og = out[0]
    grid = np.asarray(og.grid)
    assert og.frame_id == "world" and abs(og.resolution - RESOLUTION_M) < 1e-9
    assert [at(og, 2.025, y) for y in PREMAP_XS] == [100] * 20, "the premap walls are 100"
    assert at(og, 1.975, 0.475) == -1 and at(og, 2.075, 0.475) == -1, "the floor on both sides of the wall is unknown"
    assert at(og, -0.475, 0.025) == 0, "the floor the lidar's rays crossed is free"
    n_wall, n_free = int((grid == 100).sum()), int((grid == 0).sum())
    assert n_wall == 20 and n_free > 0, (n_wall, n_free)
    asyncio.run(m.handle_tf(_tf(FIX[0] + 0.05, FIX[1], FIX[2])))
    assert len(m.global_costmap.msgs) == 2, "once a grid has gone out, a re-write publishes at once"
    og2 = m.global_costmap.msgs[-1]
    assert [at(og2, 2.075, y) for y in PREMAP_XS] == [100] * 20 and [at(og2, 2.025, y) for y in PREMAP_XS] == [-1] * 20
    # Clear map: a re-write waits for the map's next grid, which carries it
    asyncio.run(m.handle_clear_map_cmd(Bool(data=True)))
    asyncio.run(m.handle_tf(_tf(FIX[0] + 0.10, FIX[1], FIX[2])))
    assert int(m._grid.premap.sum()) == 20 and len(m.global_costmap.msgs) == 2, "after Clear map: written, not published"
    for _ in range(PUBLISH_EVERY):
        asyncio.run(m.handle_lidar(_cloud(behind)))
    og3 = m.global_costmap.msgs[-1]
    assert len(m.global_costmap.msgs) == 3 and [at(og3, 2.125, y) for y in PREMAP_XS] == [100] * 20
    print(f"  before the mission: written, 0 grids; mission open, footprint seen, a re-write: 0 grids; first grid "
          f"of the mission (revolution {PUBLISH_EVERY}): 20 cells at "
          f"100 along x = 2.025, {n_free} free cells, -1 beside the wall; a 5 cm fix: published at once, 100 along "
          "x = 2.075, -1 where the wall was; after Clear map a re-write goes out with the next grid")


def test_a_carved_premap_wall_stays_carved_through_a_rewrite() -> None:
    """A premap wall a lidar ray took below OCCUPIED_AT was seen free: a
    re-write never paints it back. The case that needs the mark: the cell
    held a one-viewpoint hit (1) before the premap landed, the premap made it
    a wall (2), a ray carved it (1), and the clear takes the premap's share
    back (0): written again, it would be a wall the lidar removed. A lidar
    hit from a new viewpoint lifts the mark, and the next write claims the
    cell again."""
    m = _premap_module()
    asyncio.run(m.handle_explore_cmd(Bool(data=True)))
    g = m._grid
    p = (2.01, 0.01)
    (cy, cx), = _cells(g, [p])
    g._hit(g.lidar, np.array([p[0]]), np.array([p[1]]), (0.0, 0.0))     # one sighting from one place: 1
    asyncio.run(m.handle_loaded_map(_premap_cloud(np.array([[p[0], p[1], 0.4]]))))
    asyncio.run(m.handle_tf(_tf(0.0, 0.0, 0.0)))
    assert g.lidar[cy, cx] == OCCUPIED_AT and g.premap[cy, cx] and g.value_at(*p) == 100
    behind = _lidar_cloud(np.array([[3.0, 0.01]]))                     # a return behind it: the ray crosses the cell
    for _ in range(RAY_EVERY):
        asyncio.run(m.handle_lidar(_cloud(behind)))
    assert g.lidar[cy, cx] == OCCUPIED_AT - 1 and g.premap_carved[cy, cx] and g.value_at(*p) == 0, g.lidar[cy, cx]
    assert _cells(g, [(p[0], p[1] + 0.025)]) == {(cy, cx)}, "the moved fix keeps the point in its cell"
    lines = _logged(lambda: asyncio.run(m.handle_tf(_tf(0.0, 0.025, 0.0))))
    assert lines[0] == "costmap: premap re-written: tf moved by (2.5 cm, 0.0 deg)" \
        and lines[-1].startswith("costmap: premap written: 0 walls from 1 points") \
        and "1 cells the lidar has seen free left free" in lines[-1], lines
    assert g.value_at(*p) == 0 and not g.premap[cy, cx] and g.premap_carved[cy, cx], (g.lidar[cy, cx], g.value_at(*p))
    g._hit(g.lidar, np.array([p[0]]), np.array([p[1]]), (0.0, 0.5))     # seen again, from a new viewpoint
    assert not g.premap_carved[cy, cx]
    asyncio.run(m.handle_tf(_tf(0.0, 0.0, 0.0)))
    assert g.premap[cy, cx] and g.value_at(*p) == 100, (g.lidar[cy, cx], g.value_at(*p))
    print(f"  one hit (1) + premap -> {OCCUPIED_AT} -> a ray -> {OCCUPIED_AT - 1}, carved -> a 2.5 cm fix in the same "
          "cell: 0 walls, 1 left free, the cell free; a hit from a new viewpoint lifts the mark: the next write "
          "makes it a wall again")


TESTS = (test_the_doctrine_defaults, test_leg_needs_two_viewpoints_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
         test_chair_moved_is_forgotten_by_lidar_rays, test_reinforcement_is_capped_and_unlearning_bounded,
         test_parked_rover_never_erases_an_established_wall, test_body_clear_is_the_moss_footprint,
         test_the_lidar_tag_is_one_exact_height, test_the_map_writes_only_during_the_mission,
         test_explore_order_opens_the_map_before_any_motion, test_clear_map_starts_a_fresh_grid_and_keeps_the_gate,
         test_a_contact_marks_the_band_ahead_as_a_wall,
         test_what_goes_out_on_global_costmap,
         test_the_mission_signals_cross_the_process_boundary, test_revolution_cost,
         test_camera_one_viewpoint_never_becomes_a_wall, test_camera_wall_no_longer_seen_is_carved_free,
         test_camera_wall_still_seen_survives_the_rays, test_the_knobs_are_read_at_import,
         test_the_carve_stays_in_the_cameras_view, test_body_clear_frees_the_camera_layer,
         test_thin_leg_survives_the_floor_sampled_beside_it, test_moving_object_never_writes_the_map,
         test_the_cameras_rays_carve_the_camera_layer_only, test_camera_rays_start_at_the_camera_not_the_base,
         test_the_carve_band_follows_the_camera_height,
         test_the_premap_needs_its_map_and_its_fix, test_a_lidar_ray_carves_a_premap_wall_like_any_wall,
         test_a_moved_fix_clears_and_rewrites_the_premap, test_the_premap_off_ignores_and_says_so_once,
         test_the_premap_never_touches_the_camera_layer, test_the_published_grid_carries_the_premap,
         test_a_carved_premap_wall_stays_carved_through_a_rewrite)

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
