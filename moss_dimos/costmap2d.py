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
(the things below the scan plane: a shoe, a step, a chair base; and, like the
lidar, the legs of whoever walks past). `lidar_returns` tells them apart, and
each sensor has its own layer. Lidar returns hit their cell on the lidar
layer; camera points hit their cell on the camera layer under the same
two-viewpoint rule, so a camera obstacle becomes a wall on approach like a leg
does, and carve nothing (a camera obstacle proves nothing about the floor in
front of it). The lidar rays carve the lidar layer all around, and the camera
layer only inside the camera's view.

The camera layer forgets through the lidar rays and holds what the camera
keeps seeing. Without the carve, legs the camera saw while somebody walked
past would stay walls for the rest of the run. The balance, at MOSS's rates:

  * where: the camera layer holds what the scan plane misses (a shoe, a
    chair seat, a table top up to 1.30 m, thin dark legs), so a lidar ray
    passing under or beside such a cell is no evidence against it unless the
    camera looks at the cell too. The rays carve the camera layer only in the
    cells inside the depth camera's horizontal field of view from the current
    pose: a cone of CAMERA_HFOV_DEG from the camera on the nose, centred on
    the rover's heading, out to CAMERA_CARVE_MAX_M from the rover. Outside it
    the camera layer is untouched: a table the rover has turned away from
    stays a wall, and a passer-by's legs are forgotten once the rover looks
    back at where they stood;
  * the carve: a ray that crosses a cell takes 1 from it every RAY_EVERY
    revolutions (5 per second with the lidar at 10 Hz), within RAY_MAX_M of
    the rover;
  * the camera: it runs at 5 fps and lidar_odometry attaches its latest
    obstacle points to every revolution for up to 0.5 s, so a cell in view
    takes a camera hit on every revolution (10 per second);
  * a cell the camera hits in a revolution is spared by that revolution's
    carve, and a camera hit from the SAME viewpoint adds 1 while the cell is
    at 0 or above and below OCCUPIED_AT, so what a carve took in a revolution
    the camera missed comes back on the next one. That hit raises a cell to
    OCCUPIED_AT only if the cell became a wall from two viewpoints and was
    not carved back to 0 since (`camera_wall`); any other cell stops at
    OCCUPIED_AT - 1, so one spot still never makes a wall. Below 0 it adds
    nothing: floor the camera layer holds as free (down to FREE_FLOOR) needs
    as many viewpoints to become a wall as on the lidar layer. Above
    OCCUPIED_AT the parked-rover rule holds: a same-viewpoint hit adds nothing.

So a shoe the camera keeps seeing stays a wall, and a camera wall of score s
that the camera no longer sees while rays cross it inside the camera's view is
free after s - 1 carves, RAY_EVERY * (s - 1) revolutions: 0.2 s at 2 (two
viewpoints), 1.8 s at a HIT_CAP of 10. The price: an obstacle inside that view
that the camera does not see while the rays cross its cell (hidden behind
something nearer, or a table top nearer than the camera's vertical field of
view reaches) is forgotten as fast, and becomes a wall again once the camera
sees it from two viewpoints (NEW_VIEWPOINT_M of motion apart; turning on the
spot is one viewpoint).

Mission gate: the map writes only once the mission has started. A rover parked
with a spinning lidar thickens the obstacles around its one viewpoint and
records whoever walks past while it waits. The gate opens on the first of two
events: the rover moving MISSION_START_M away from its first pose, or an
`explore_cmd` (the order to explore, which comes just before the first
motion). It then stays open for the rest of the run: the end of an
exploration (`explore_done`, `stop_explore_cmd`) stops the explorer, not the
map, because the lidar map is MOSS's only obstacle source and a cockpit goal
or STOP sends `stop_explore_cmd` mid-run.

Numbers (HIT_CAP, FREE_FLOOR, OCCUPIED_AT, RAY_MAX_M) are a starting point to
be tested. HIT_CAP, FREE_FLOOR and RAY_MAX_M are read once, at import, from
MOSS_HIT_CAP, MOSS_FREE_FLOOR and MOSS_RAY_MAX_M: a flight tries other values
without a code change, and the "costmap up" log line says which ones it runs.
"""

from __future__ import annotations

import math
import os
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


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    """A number from the environment, read once at import. Unset or empty ->
    `default`; not a number -> `default`, said in the log; outside [lo, hi]
    -> the nearest bound, said in the log."""
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError:
        logger.warning(f"costmap: {name}={raw!r} is not a number - using {default}")
        return default
    if not lo <= value <= hi:
        bounded = hi if value > hi else lo      # below lo, or nan
        logger.warning(f"costmap: {name}={raw!r} is outside [{lo}, {hi}] - using {bounded}")
        return bounded
    return value


RESOLUTION_M = 0.05
GRID_SPAN_M = 24.0           # world-fixed square, centred on the first pose
OCCUPIED_AT = int(round(_env_float("MOSS_OCCUPIED_AT", 1, 1, 10)))   # hits before a cell is an obstacle. 1: the first sighting
                             # counts (the planner reacts within one revolution; a passer-by is carved back within a second).
                             # 2 = the cautious rule, a wall only from two places 10 cm apart (MOSS_OCCUPIED_AT=2)
# The layers are int8: the ceiling stays within 127, and at OCCUPIED_AT or above
# (below it nothing could ever become a wall); the floor stays within [-128, 0].
HIT_CAP = int(round(_env_float("MOSS_HIT_CAP", 10, OCCUPIED_AT, 127)))       # ceiling: no cell gets more certain than this.
                             # A lower cap unlearns a passer-by faster (a cell at the cap needs HIT_CAP - 1 carves,
                             # RAY_EVERY revolutions each, to stop being a wall) and keeps a wall less long once
                             # nothing sees it any more.
FREE_FLOOR = int(round(_env_float("MOSS_FREE_FLOOR", -3, -128, 0)))          # floor: no cell gets more "free" than this.
                             # A floor nearer 0 lets a new obstacle on long-seen floor become a wall in fewer hits.
NEW_VIEWPOINT_M = _env_float("MOSS_NEW_VIEWPOINT_M", 0.05, 0.0, 1.0)   # a hit counts only if the rover moved this much since the cell's last hit
LIDAR_Z_TOL_M = 1e-6         # lidar returns sit at EXACTLY the profile's lidar_height_m: the tag survives the float32 round trip to 5 nm (see lidar_returns)
PUBLISH_EVERY = int(round(_env_float("MOSS_PUBLISH_EVERY", 2, 1, 20)))   # one grid every N revolutions (2 = 5 Hz at 10 Hz). A 24 m grid is ~230 kB and every
                             # subscriber (planner, explorer, recorder, cockpit) decodes each one; the map
                             # QUALITY does not depend on this rate (the odometry gives it).
RAY_MAX_M = _env_float("MOSS_RAY_MAX_M", 4.0, 0.0, GRID_SPAN_M)   # rays are carved (misses) up to this range only:
                             # a cell farther than RAY_MAX_M from every viewpoint is never unlearned, so a passer-by
                             # 4.5 m away stays until the rover comes within RAY_MAX_M of where they stood. 0 = no carving.
RAY_EVERY = 2                # carve on every other revolution; hits are taken on all
CAMERA_HFOV_DEG = 87.0       # the D455's horizontal field of view: the rays carve the camera layer only inside it
CONTACT_BAND_M = (0.02, 0.20)   # a contact marks the band this far beyond the body's front edge, across its width
CAMERA_CARVE_MAX_M = 3.0     # ...and only this close to the rover. Coupled to lidar_odometry's DEPTH_MAX_M (depth
                             # range along the camera's axis) and OBSTACLE_MAX_M (camera obstacles, measured from
                             # base_link), both 3.0 m: measured from base_link, a cell within this range ahead of the
                             # camera is inside both. Past them the camera reports nothing, so it could not hold a cell.
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
        self.camera = np.zeros((self.n, self.n), dtype=np.int8)    # what the depth camera saw: hit by it, freed by its floor samples AND carved by the lidar rays inside its view
        self.camera_wall = np.zeros((self.n, self.n), dtype=bool)  # camera cells that became a wall from two viewpoints and were not carved back to 0 since
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
        new = np.where(moved, np.minimum(cur + 1, HIT_CAP), cur)
        took = moved
        if layer is self.camera:
            # The camera layer is carved by the lidar rays, so the camera must
            # be able to hold what it keeps seeing: a same-viewpoint hit adds 1
            # while the cell is at 0 or above and below its ceiling. The
            # ceiling is OCCUPIED_AT for a cell that became a wall from two
            # viewpoints (the hit puts back what the carve took), OCCUPIED_AT
            # - 1 for any other (one spot never makes a wall). Below 0 the hit
            # adds nothing, as on the lidar layer: the hold only undoes a
            # carve, and floor seen free for long still needs new viewpoints.
            # Above OCCUPIED_AT the parked-rover rule holds: a same-viewpoint
            # hit adds nothing and takes nothing away.
            wall = self.camera_wall[gy, gx]
            held = ~moved & (cur >= 0) & (cur < np.where(wall, OCCUPIED_AT, OCCUPIED_AT - 1))
            new = np.where(held, cur + 1, new)
            took = moved | held
            self.camera_wall[gy, gx] = wall | (new >= OCCUPIED_AT)
        layer[gy, gx] = new.astype(np.int8)
        self._last_hit_xy[gy[moved], gx[moved]] = from_xy
        self.seen[gy, gx] = True
        return int(took.sum())

    def _miss(self, layer: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> None:
        if len(gx) == 0:
            return
        cur = layer[gy, gx].astype(np.int16)
        new = np.maximum(cur - 1, FREE_FLOOR).astype(np.int8)
        layer[gy, gx] = new
        self.seen[gy, gx] = True
        if layer is self.camera:
            self.camera_wall[gy, gx] &= new > 0     # carved back to 0: forgotten, one spot no longer holds it

    _revs: int = 0

    def lidar_revolution(self, hits_xy: np.ndarray, from_xy: tuple[float, float],
                         camera_xy: np.ndarray | None = None, yaw: float | None = None) -> None:
        """World-frame lidar hits of one revolution, seen from `from_xy` with
        the rover heading `yaw`: the cells along each ray (up to just short of
        the hit, RAY_MAX_M at most, every RAY_EVERY-th revolution) take a miss
        on the lidar layer, and those of them inside the camera's view
        (`camera_view`) a miss on the camera layer; the hit cells take a hit
        on the lidar layer. A ray that crosses a cell where the camera saw an
        obstacle, inside the camera's view, carves it like any other cell, so
        what the camera stops seeing is forgotten (the module docstring has
        the balance). Without `yaw` the camera's view is unknown and the
        camera layer is not carved.

        `camera_xy` are the camera obstacle points of the same revolution:
        their cells are spared on the camera layer. A ray passing over a cell
        does not erase what the camera sees in it during that revolution;
        without the spare, a second viewpoint that lands on a carving
        revolution loses its hit to the carve and the obstacle never becomes
        a wall on approach."""
        if len(hits_xy) == 0:
            return
        self._revs += 1
        if self._revs % RAY_EVERY == 0:
            cells = self._ray_cells(from_xy, hits_xy)
            if cells is not None:
                gx, gy = cells
                self._miss(self.lidar, gx, gy)
                if yaw is not None:
                    view = self.camera_view(gx, gy, from_xy, yaw)
                    gx, gy = gx[view], gy[view]
                    if camera_xy is not None and len(camera_xy):
                        cx, cy = self.cell(camera_xy[:, 0], camera_xy[:, 1])
                        keep = ~np.isin(gy * self.n + gx, cy * self.n + cx)
                        gx, gy = gx[keep], gy[keep]
                    self._miss(self.camera, gx, gy)
        self._hit(self.lidar, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def camera_view(self, gx: np.ndarray, gy: np.ndarray, from_xy: tuple[float, float], yaw: float) -> np.ndarray:
        """Which cells (gx, gy) lie inside the depth camera's horizontal field
        of view from the pose (from_xy, yaw): the cell centre within
        CAMERA_HFOV_DEG / 2 of the rover's heading, seen from the camera's
        position (the profile's camera_xyz_base_m, read at call time; the
        camera looks along the heading), and within CAMERA_CARVE_MAX_M of the
        rover."""
        cam = _current_profile().camera_xyz_base_m
        c, s = math.cos(yaw), math.sin(yaw)
        cam_x = from_xy[0] + c * cam[0] - s * cam[1]
        cam_y = from_xy[1] + s * cam[0] + c * cam[1]
        xs = self.ox + (gx + 0.5) * self.res
        ys = self.oy + (gy + 0.5) * self.res
        bearing = np.arctan2(ys - cam_y, xs - cam_x) - yaw
        bearing = (bearing + np.pi) % (2 * np.pi) - np.pi
        near = np.hypot(xs - from_xy[0], ys - from_xy[1]) <= CAMERA_CARVE_MAX_M
        return near & (np.abs(bearing) <= math.radians(CAMERA_HFOV_DEG) / 2)

    def camera_hits(self, hits_xy: np.ndarray, from_xy: tuple[float, float]) -> int:
        """World-frame obstacle points from the depth camera, seen from `from_xy`:
        a hit on their cells (two-viewpoint rule as for lidar hits, plus the
        same-viewpoint hit that holds a wall the camera keeps seeing at
        OCCUPIED_AT), no ray carved. Returns the number of cells whose score
        took the hit."""
        if len(hits_xy) == 0:
            return 0
        return self._hit(self.camera, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def contact(self, pose: tuple) -> int:
        """The rover pushed against something its sensors did not map (the stall
        guard fired): the cells of a short band right ahead of the body become a
        wall on the camera layer at once (no viewpoint rule: a contact is a
        physical certainty, like body_clear). The planner then routes around the
        spot instead of driving back into it. Freed like any camera wall: by the
        camera seeing bare floor there, or by the body driving over it."""
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        hx, hy = _current_profile().body_clear_m
        ax = np.arange(hx + CONTACT_BAND_M[0], hx + CONTACT_BAND_M[1] + 1e-9, self.res / 2)
        ay = np.arange(-hy, hy + 1e-9, self.res / 2)
        bx, by = np.meshgrid(ax, ay)
        gx, gy = self.cell((x + c * bx - s * by).ravel(), (y + s * bx + c * by).ravel())
        if len(gx) == 0:
            return 0
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        self.camera[gy, gx] = np.maximum(self.camera[gy, gx], np.int8(OCCUPIED_AT))
        self.camera_wall[gy, gx] = True
        self.seen[gy, gx] = True
        return int(len(gx))

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
            self.camera_wall[gy, gx] = False
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
    clear_map_cmd: In[Bool]         # the cockpit's Clear map (explore_ctl.py clear): a fresh grid, the mission gate as it is
    bump: In[Bool]                  # a contact (the base's stall guard, the explorer's back-off): the band ahead becomes a wall
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
        logger.info(f"costmap up: {RESOLUTION_M} m cells, hit cap {HIT_CAP}, free floor {FREE_FLOOR}, "
                    f"occupied at {OCCUPIED_AT}, rays carved to {RAY_MAX_M:g} m")

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

    async def handle_bump(self, msg: Bool) -> None:
        """A contact: the cells just ahead of the body become a wall at once, so the
        planner routes around what the sensors did not see (a cable, a step, a
        bin edge below the camera's view) instead of driving into it again."""
        if not bool(getattr(msg, "data", False)) or self._grid is None or self._pose_xy is None:
            return
        if self._write_frozen():
            return
        n = self._grid.contact((self._pose_xy[0], self._pose_xy[1], self._pose_yaw))
        logger.info(f"costmap: contact at ({self._pose_xy[0]:+.2f}, {self._pose_xy[1]:+.2f}) heading "
                    f"{np.degrees(self._pose_yaw):+.0f} deg: {n} cells ahead marked as a wall")
        self._publish()

    async def handle_clear_map_cmd(self, msg: Bool) -> None:
        """A fresh grid on request (the cockpit's Clear map): everything the map
        learned goes, centred again on the current pose; the mission gate stays
        as it is, so an open mission keeps writing from the next revolution and
        a run that has not started still waits for its order or its 5 cm."""
        if not bool(getattr(msg, "data", False)) or self._pose_xy is None:
            return
        self._grid = ScoredGrid(centre=self._pose_xy)
        self._revolutions = 0
        self._camera_pts = self._camera_cells = 0
        logger.info(f"costmap: map CLEARED on request - fresh grid centred on ({self._pose_xy[0]:+.2f}, "
                    f"{self._pose_xy[1]:+.2f}), mission {'open' if not self._mission_frozen else 'not started'}")

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
        cam = pts[~tag][:, :2]
        self._grid.lidar_revolution(hits[:, :2], self._pose_xy, camera_xy=cam, yaw=self._pose_yaw)
        if len(cam):
            self._camera_cells += self._grid.camera_hits(cam, self._pose_xy)
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
