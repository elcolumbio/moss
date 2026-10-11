"""A 2D costmap for MOSS that learns AND unlearns, from the lidar.

dimOS's CostMapper turns a 3D voxel map (a set that never forgets) into a
terrain-slope map meant for a walking robot, and its erosion drops any cell
whose four neighbours are unobserved, which erases thin obstacles such as
table legs. This module replaces it for a rover with a single-plane lidar:

  * the map must REINFORCE, but with a ceiling (HIT_CAP), or a ramp the
    rover struggled on becomes a wall after a day;
  * it must UNLEARN what it sees absent again: every lidar ray that passes
    through a cell is a miss, down to FREE_FLOOR, so a chair that was moved
    away fades in a few revolutions;
  * reinforcement needs new viewpoints: from one spot a cell gets no more
    certain than OCCUPIED_AT - 1, so a passer-by hammered from one parked spot
    never becomes a wall; a second viewpoint NEW_VIEWPOINT_M away makes it an
    obstacle, and a leg seen from twenty positions is solid (up to HIT_CAP);
  * what the body drove over (body_clear) is cleared: that one is a physical
    certainty, not a sensor reading.

Two sensors write here, through one cloud. lidar_odometry publishes each
revolution in `world` with every lidar return at exactly the profile's
lidar_height_m, plus the depth camera's obstacle points at their true height
(the things below the scan plane: a shoe, a step, a chair base). `lidar_returns`
tells them apart. Lidar returns carve their rays free and hit their cell; the
camera points only hit their cell (a camera obstacle proves nothing about the
floor in front of it) under the same two-viewpoint rule, so a camera obstacle
becomes a wall on approach like a leg does.

Mission gate: the map writes only once the mission has started. A rover parked
with a spinning lidar thickens the obstacles around its one viewpoint and
records whoever walks past while it waits. The gate opens on the first of two
events: the rover moving MISSION_START_M away from its first pose, or an
`explore_cmd` (the order to explore, which comes just before the first
motion). It then stays open for the rest of the run: the end of an
exploration (`explore_done`, `stop_explore_cmd`) stops the explorer, not the
map, because the lidar map is MOSS's only obstacle source and a cockpit goal
or STOP sends `stop_explore_cmd` mid-run.

Numbers (HIT_CAP, FREE_FLOOR, OCCUPIED_AT) are a starting point to be tested.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

from moss_dimos.robot_profile import current as _current_profile

logger = setup_logger()

RESOLUTION_M = 0.05
GRID_SPAN_M = 24.0           # world-fixed square, centred on the first pose
HIT_CAP = 10                 # ceiling: no cell gets more certain than this
FREE_FLOOR = -3              # floor: no cell gets more "free" than this
OCCUPIED_AT = 2              # two hits from two places = an obstacle
NEW_VIEWPOINT_M = 0.10       # a hit counts only if the rover moved this much since the cell's last hit
LIDAR_Z_TOL_M = 1e-6         # lidar returns sit at EXACTLY the profile's lidar_height_m: the tag survives the float32 round trip to 5 nm (see lidar_returns)
PUBLISH_EVERY = 5            # one grid every 5 revolutions (~2 Hz at 10 Hz). A 24 m grid is ~230 kB and every
                             # subscriber (planner, explorer, recorder, cockpit) decodes each one; the map
                             # QUALITY does not depend on this rate (the odometry gives it).
RAY_MAX_M = 4.0              # rays are carved (misses) up to this range only: 12 m rays in 2.5 cm steps cost ~230 ms per revolution
RAY_EVERY = 2                # carve on every other revolution; hits are taken on all
MISSION_START_M = 0.05       # the map opens when the rover has moved this far from its first pose (or on explore_cmd).
                             # Measuring the displacement needs no clock: the wait accumulates nothing however
                             # long it lasts, and the first useful revolution leaves with the rover.


class ScoredGrid:
    """One int8 score layer over a fixed world grid, plus where each cell was last hit from."""

    def __init__(self, resolution: float = RESOLUTION_M, span_m: float = GRID_SPAN_M,
                 centre: tuple[float, float] = (0.0, 0.0)) -> None:
        self.res = resolution
        self.n = int(round(span_m / resolution))
        self.ox = centre[0] - span_m / 2.0
        self.oy = centre[1] - span_m / 2.0
        self.lidar = np.zeros((self.n, self.n), dtype=np.int8)     # what the lidar saw in its scan plane
        self.camera = np.zeros((self.n, self.n), dtype=np.int8)    # what the depth camera saw below that plane: hit by it, freed by its floor samples, never carved by a lidar ray
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self._last_hit_xy = np.full((self.n, self.n, 2), np.nan, dtype=np.float32)

    # ---- coordinates -------------------------------------------------------
    def cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gx = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        gy = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        ok = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        return gx[ok], gy[ok]

    # ---- learning ----------------------------------------------------------
    def _hit(self, layer: np.ndarray, xs: np.ndarray, ys: np.ndarray, from_xy: tuple[float, float]) -> int:
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return 0
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        last = self._last_hit_xy[gy, gx]
        moved = np.isnan(last[:, 0]) | (np.hypot(last[:, 0] - from_xy[0], last[:, 1] - from_xy[1]) >= NEW_VIEWPOINT_M)
        cur = layer[gy, gx].astype(np.int16)
        # An obstacle is REAL when it was seen from two viewpoints: a
        # passer-by hammered from one parked spot must never become a wall.
        # From one viewpoint a cell rises to OCCUPIED_AT - 1 and no
        # further; the second viewpoint (0.10 m of motion) makes it a wall.
        # Driving toward anything real crosses viewpoints within a metre, so
        # legs and furniture still map on approach.
        # The viewpoint rule bounds the INCREASE, never the value: a
        # same-viewpoint hit adds nothing and takes nothing away. Clamping the
        # value instead would let one revolution from a parked rover push a
        # wall built from twenty viewpoints back below OCCUPIED_AT, and
        # occupancy() would publish free floor where the wall stands.
        new = np.where(moved, np.minimum(cur + 1, HIT_CAP), cur).astype(np.int8)
        layer[gy, gx] = new
        self._last_hit_xy[gy[moved], gx[moved]] = from_xy
        self.seen[gy, gx] = True
        return int(moved.sum())

    def _miss(self, layer: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> None:
        if len(gx) == 0:
            return
        cur = layer[gy, gx].astype(np.int16)
        new = np.maximum(cur - 1, FREE_FLOOR).astype(np.int8)
        layer[gy, gx] = new
        self.seen[gy, gx] = True

    _revs: int = 0

    def lidar_revolution(self, hits_xy: np.ndarray, from_xy: tuple[float, float]) -> None:
        """World-frame lidar hits of one revolution, seen from `from_xy`:
        the cells along each ray (up to just short of the hit, RAY_MAX_M at
        most, every RAY_EVERY-th revolution) take a miss, the hit cells a hit."""
        if len(hits_xy) == 0:
            return
        self._revs += 1
        if self._revs % RAY_EVERY == 0:
            cells = self._ray_cells(from_xy, hits_xy)
            if cells is not None:
                self._miss(self.lidar, cells[0], cells[1])
        self._hit(self.lidar, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def camera_hits(self, hits_xy: np.ndarray, from_xy: tuple[float, float]) -> int:
        """World-frame obstacle points from the depth camera, seen from `from_xy`:
        a hit on their cells (two-viewpoint rule as for lidar hits), no ray carved.
        Returns the number of cells that took a hit."""
        if len(hits_xy) == 0:
            return 0
        return self._hit(self.camera, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def camera_floor(self, floor_xy: np.ndarray) -> None:
        """World-frame bare-floor samples from the depth camera: a miss on the
        camera layer (a shoe that was taken away stops being an obstacle once
        the camera sees the floor where it stood). The lidar layer is untouched."""
        if len(floor_xy) == 0:
            return
        gx, gy = self.cell(floor_xy[:, 0], floor_xy[:, 1])
        if len(gx):
            flat = np.unique(gy * self.n + gx)
            self._miss(self.camera, flat % self.n, flat // self.n)

    def _ray_cells(self, from_xy: tuple[float, float], hits_xy: np.ndarray):
        """Cells crossed by the rays from `from_xy` to each hit (capped at
        RAY_MAX_M), stopping one cell short of the hit; one sample per cell."""
        dx, dy = hits_xy[:, 0] - from_xy[0], hits_xy[:, 1] - from_xy[1]
        r = np.hypot(dx, dy)
        keep = r > self.res
        if not keep.any():
            return None
        dx, dy, r = dx[keep], dy[keep], r[keep]
        end = np.minimum(r - self.res, RAY_MAX_M)
        n_steps = np.floor(end / self.res).astype(int)
        nmax = int(n_steps.max())
        if nmax < 1:
            return None
        # (rays x steps) sample grid in one shot, masked past each ray's end
        d = (np.arange(1, nmax + 1) * self.res)[None, :]
        ux, uy = (dx / r)[:, None], (dy / r)[:, None]
        valid = d <= end[:, None]
        xs = (from_xy[0] + ux * d)[valid]
        ys = (from_xy[1] + uy * d)[valid]
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return None
        flat = np.unique(gy * self.n + gx)
        return flat % self.n, flat // self.n

    def body_clear(self, pose: tuple) -> None:
        """The body IS here: every cell under the profile's body_clear_m
        footprint (half extents around base_link) is certainly free, and seen.
        Without it, cells written before the rover drove over them stay and
        can wall it in. Keep the footprint exact, never wider, so a wall
        right against the body survives."""
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        hx, hy = _current_profile().body_clear_m
        lx = np.arange(-hx, hx + 1e-9, self.res / 2)
        ly = np.arange(-hy, hy + 1e-9, self.res / 2)
        bx, by = np.meshgrid(lx, ly)
        gx, gy = self.cell((x + c * bx - s * by).ravel(), (y + s * bx + c * by).ravel())
        if len(gx):
            self.lidar[gy, gx] = FREE_FLOOR
            self.camera[gy, gx] = FREE_FLOOR
            self.seen[gy, gx] = True

    # ---- output ------------------------------------------------------------
    def occupancy(self) -> np.ndarray:
        """int8 grid: 100 occupied, 0 free (observed), -1 unknown."""
        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[self.seen] = 0
        out[self.lidar >= OCCUPIED_AT] = 100
        out[self.camera >= OCCUPIED_AT] = 100
        return out

    def value_at(self, x: float, y: float) -> int:
        gx, gy = self.cell(np.array([x]), np.array([y]))
        return int(self.occupancy()[gy[0], gx[0]]) if len(gx) else -1

    def cropped(self, margin_cells: int = 20) -> tuple[np.ndarray, float, float] | None:
        """The occupancy grid cropped to the observed area (+ margin): (grid, origin_x, origin_y)."""
        if not self.seen.any():
            return None
        ys, xs = np.nonzero(self.seen)
        y0, y1 = max(0, ys.min() - margin_cells), min(self.n, ys.max() + margin_cells + 1)
        x0, x1 = max(0, xs.min() - margin_cells), min(self.n, xs.max() + margin_cells + 1)
        return self.occupancy()[y0:y1, x0:x1], self.ox + x0 * self.res, self.oy + y0 * self.res


def lidar_returns(pts: np.ndarray) -> np.ndarray:
    """Which points of the `lidar` cloud are lidar returns.

    lidar_odometry builds each revolution flat (z = 0) and lifts it by the
    profile's lidar_height_m, so every lidar return carries that height to
    the last bit of a float32. Any other point the cloud carries (a depth
    camera's, at its true height) can sit within millimetres of the scan
    plane, so the tag is read as one exact value (LIDAR_Z_TOL_M), never as a
    band.
    """
    return np.abs(pts[:, 2] - _current_profile().lidar_height_m) < LIDAR_Z_TOL_M


class MossCostMap(Module):
    """Replaces dimOS's CostMapper on MOSS.

    Ins: `lidar` (the world cloud from lidar_odometry, lidar returns tagged
    with z = profile.lidar_height_m exactly), `odom` (base pose in world),
    `explore_cmd` (the order that opens the map), `explore_done` and
    `stop_explore_cmd` (logged; the map keeps writing).
    Out: `global_costmap` (the stream name the planner and the explorer
    listen to)."""

    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]   # bare floor the depth camera saw (lidar_odometry): frees camera obstacles
    odom: In[PoseStamped]
    explore_done: In[Bool]          # explorer2 found no reachable frontier left: logged, the map keeps writing
    stop_explore_cmd: In[Bool]      # exploration stopped on request (explore_ctl.py stop, the cockpit): logged, the map keeps writing
    explore_cmd: In[Bool]           # the order to explore: opens the map before the first motion
    global_costmap: Out[OccupancyGrid]

    def __init__(self, world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.world_frame = world_frame
        self._grid: ScoredGrid | None = None
        self._pose_xy: tuple[float, float] | None = None
        self._pose_yaw = 0.0
        self._revolutions = 0
        self._camera_pts = 0           # camera obstacle points since the last publish (log only)
        self._camera_cells = 0         # ...and the cells they hit
        self._last_clear: tuple | None = None   # (x, y, yaw) of the last body_clear
        # The pre-mission gate, closed at construction: a rover parked with a
        # spinning lidar would write the room it waits in, from one spot. It
        # opens on MISSION_START_M of displacement or on explore_cmd, and stays
        # open for the rest of the run.
        self._mission_frozen = True
        self._boot_xy: tuple[float, float] | None = None   # the departure reference: the first pose

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(f"costmap up: {RESOLUTION_M} m cells, hit cap {HIT_CAP}, free floor {FREE_FLOOR}, occupied at {OCCUPIED_AT}")

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._pose_xy = (float(msg.position.x), float(msg.position.y))
        q = msg.orientation
        self._pose_yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self._grid is None:
            self._grid = ScoredGrid(centre=self._pose_xy)
            logger.info(f"costmap: grid centred on ({self._pose_xy[0]:+.2f}, {self._pose_xy[1]:+.2f})")
        self._check_departure()
        if self._write_frozen():
            return                  # before the mission: not even body_clear
        yaw = self._pose_yaw
        lc = self._last_clear
        if (lc is None
                or (self._pose_xy[0] - lc[0]) ** 2 + (self._pose_xy[1] - lc[1]) ** 2 > 0.03 ** 2
                or abs((yaw - lc[2] + np.pi) % (2 * np.pi) - np.pi) > np.radians(10.0)):
            self._last_clear = (self._pose_xy[0], self._pose_xy[1], yaw)
            self._grid.body_clear(self._last_clear)

    def _write_frozen(self) -> bool:
        """The ONE write gate: a hit, a miss or a body_clear needs the mission
        started (MISSION_START_M of displacement, or explore_cmd)."""
        return self._mission_frozen

    def _check_departure(self) -> None:
        """Open the gate once the rover has left its first pose.

        Cheap by design: two subtractions on a pose handle_odom already has,
        and only until it fires once.
        """
        if not self._mission_frozen or self._pose_xy is None:
            return
        if self._boot_xy is None:
            self._boot_xy = self._pose_xy
            return
        dx = self._pose_xy[0] - self._boot_xy[0]
        dy = self._pose_xy[1] - self._boot_xy[1]
        if dx * dx + dy * dy < MISSION_START_M * MISSION_START_M:
            return
        self._mission_frozen = False
        logger.info(f"costmap: the rover left its boot pose ({math.hypot(dx, dy) * 100:.0f} cm) - "
                    "map writing OPEN for the mission")

    async def handle_explore_done(self, msg: Bool) -> None:
        """explorer2 ran out of reachable frontier. The explorer stops; the map
        keeps writing (the lidar map is the only obstacle source)."""
        if bool(getattr(msg, "data", False)):
            logger.info("costmap: exploration complete - map writing continues")

    async def handle_stop_explore_cmd(self, msg: Bool) -> None:
        """Exploration stopped on request (explore_ctl.py stop, the cockpit's
        STOP or a goal). The explorer stops; the map keeps writing. A map not
        opened yet still opens later, on displacement or a new explore_cmd."""
        if bool(getattr(msg, "data", False)):
            logger.info("costmap: exploration stopped - map writing continues")

    async def handle_explore_cmd(self, msg: Bool) -> None:
        """The order to explore opens the map, without waiting for motion.

        Displacement alone deadlocks a fresh map: no written cell -> no
        frontier -> no goal -> no motion -> the 5 cm never happen. The order
        comes just before the rover departs, which is the moment the gate
        waits for; the displacement check stays as the opener for runs
        driven without an explore order (gamepad, cockpit goals)."""
        if not bool(getattr(msg, "data", False)):
            return
        if self._mission_frozen:
            self._mission_frozen = False
            logger.info("costmap: explore_cmd received - map writing OPEN for the mission")

    async def handle_camera_floor(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None or self._write_frozen():
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts):
            self._grid.camera_floor(pts[:, :2])

    async def handle_lidar(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None or self._write_frozen():
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts) == 0:
            return
        tag = lidar_returns(pts)
        hits = pts[tag]
        if len(hits) == 0:
            return                  # no lidar return on this cloud: not a revolution
        self._grid.lidar_revolution(hits[:, :2], self._pose_xy)
        cam = pts[~tag]
        if len(cam):
            self._camera_cells += self._grid.camera_hits(cam[:, :2], self._pose_xy)
            self._camera_pts += len(cam)
        self._revolutions += 1
        if self._revolutions % PUBLISH_EVERY == 0:
            if self._camera_pts:
                logger.info(f"costmap: camera obstacles: {self._camera_pts} points, {self._camera_cells} cell hits "
                            f"over the last {PUBLISH_EVERY} revolutions")
                self._camera_pts = self._camera_cells = 0
            self._publish()

    def _publish(self) -> None:
        assert self._grid is not None
        crop = self._grid.cropped()
        if crop is None:
            return
        grid, ox, oy = crop
        origin = Pose()
        origin.position.x, origin.position.y, origin.position.z = ox, oy, 0.0
        origin.orientation.w = 1.0
        self.global_costmap.publish(OccupancyGrid(grid=grid, resolution=RESOLUTION_M, origin=origin,
                                                  frame_id=self.world_frame, ts=time.time()))
