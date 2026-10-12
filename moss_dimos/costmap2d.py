"""A 2D costmap for MOSS that learns AND unlearns, from the lidar and the depth camera.

dimOS's CostMapper turns a 3D voxel map (a set that never forgets) into a
terrain-slope map meant for a walking robot, and its erosion drops any cell
whose four neighbours are unobserved, which erases thin obstacles such as
table legs. This module replaces it with the scored grid of the rover this
stack comes from, adapted to MOSS's two sensors: a single-plane lidar 0.20 m
above the floor and a depth camera on the nose, 0.075 m above it.

  * the map must REINFORCE, but with a ceiling (HIT_CAP), or a ramp the
    rover struggled on becomes a wall after a day;
  * it must UNLEARN what it sees absent again, down to FREE_FLOOR, so a
    chair that was moved away fades;
  * reinforcement needs new viewpoints: from one spot a cell gets no more
    certain than OCCUPIED_AT - 1, so a passer-by hammered from one parked spot
    never becomes a wall; a second viewpoint NEW_VIEWPOINT_M away makes it an
    obstacle, and a leg seen from twenty positions is solid (up to HIT_CAP);
  * two layers, because the sensors do not see the same things. What the
    lidar put down, a lidar ray may clear (`lidar_revolution`). What the
    camera put down (a shoe, a step, a chair base under the scan plane, a
    table top above it) only the camera may clear: by seeing the floor there
    (`camera_floor`), or by seeing THROUGH it at low height (`camera_rays`). A
    lidar ray passing over a shoe says nothing about the shoe;
  * what the body drove over (body_clear) is cleared on both layers: that one
    is a physical certainty, not a sensor reading.

The sensors arrive on two clouds. lidar_odometry publishes each revolution in
`world` with every lidar return at exactly the profile's lidar_height_m, plus
the depth camera's obstacle points at their true height (`lidar_returns`
tells them apart), and the bare floor the camera sees on `camera_floor`
(z = 0), once per depth frame.

Three guards on the camera layer, each one born from a floor run of the
original rover:

  * thin legs. A cell the camera JUST called an obstacle is deaf to floor
    samples and to camera rays for LOW_HIT_PROTECT_S. A table leg thinner than
    a 5 cm cell is seen in one frame and the floor on its own cell in the next
    (5 cm of quantisation): hit and miss cancelled forever, and a leg was
    found at -3 after ten real contacts. lidar_odometry's
    split_floor_and_obstacles only drops floor samples on an obstacle cell of
    the SAME frame. This takes nothing from the unlearning: an object that is
    really gone stops being hit, and fades as before once the window is over.
  * moving objects. A cell takes a camera hit only if the previous depth frame
    also saw an obstacle within one cell of it. A rolling vacuum or a walking
    person moves more than a cell between two depth frames and is rarely
    written; a foot planted for two frames can still mark a cell, and the
    camera's floor and rays clear it once it is gone. Furniture repeats, and
    the one cell of tolerance keeps a thin leg that re-projects one cell off.
    The price is one frame of latency on a new static obstacle. The gate
    counts DISTINCT depth frames: lidar_odometry attaches the latest frame's
    obstacle points to every lidar revolution for up to 0.5 s, so a gate run
    per revolution would compare a frame with itself and always pass, and the
    rover moving on during those 0.5 s would turn one look into two
    viewpoints. A cloud whose camera points are the previous cloud's, byte for
    byte, stamped less than CAMERA_REPEAT_S after it, is that frame again: it
    writes nothing on the camera layer. The clouds' own stamps (set when
    lidar_odometry publishes them) measure that, so a costmap handler running
    late never takes a re-attached frame for a new one.
  * camera rays. Every cell a camera ray crosses at low height held nothing
    standing, or the ray would have ended on it: a miss on the camera layer.
    Floor samples and obstacle points alike: a floor point 3 m ahead proves
    the low corridor in front of it. The rays start where the camera is (the
    profile's camera_xyz_base_m, turned by the heading) and stop one cell short
    of their point.

The carve band (`carve_z_band`). The band of the original was 0.10 to 0.45 m
for a camera 0.56 m above the floor: a ray crossing a cell inside it proves
nothing low stands there; a higher ray flies OVER boxes (0.9 m up says nothing
about a 0.3 m box) and must erase nothing; a lower one grazes the floor it
ends on. A floor sample's ray drops in a straight line from the camera height
H to 0 at its range R, so that band is the stretch from
1 - 0.45 / 0.56 = 20 % to 1 - 0.10 / 0.56 = 82 % of R: not the first fifth,
where the ray still rides near the lens height over whatever is low, not the
last 18 %, where it grazes the floor. MOSS's camera sits at
H = 0.075 m, under that whole band: with 0.10 to 0.45 m no floor ray would
ever carve, and a ray climbing toward a table top would erase a shoe from
0.10 m up. So the band is the same two fractions of the camera height
(CARVE_Z_BAND_OF_CAMERA), read against the profile at call time: 0.013 to
0.060 m at 0.075 m. A floor ray carves the same stretch of its length as on
the original; a ray climbing toward anything higher than 6 cm carves nothing.

The camera's floor samples miss the camera layer only. On the original, a
camera above the scan plane, they missed the lidar layer too; here the camera
looks UNDER the scan plane, and the floor it sees beneath a sofa's edge or a
bed frame says nothing about the edge the lidar sees 0.20 m up.

Sensor doctrine: only the lidar and the camera write the map. A contact is a
reflex (the planner's escape), not a mapper. MOSS_CONTACT_WALL=1 turns MOSS's
contact memory back on, off by default: a contact on `bump` (a front contact;
a contact felt while reversing goes out on `bump_rear`, which this module does
not read) marks a short band just ahead of the body as a wall, and only when
the odometry says the rover was driving straight forward when it came: at
least CONTACT_FORWARD_M along its heading over the last CONTACT_MOTION_S,
turning less than CONTACT_TURN_DEG. A pivot, a reverse, or a back-off asked by
an explorer standing still writes nothing; every contact on an open map says
in the log whether it was written.

Mission gate: the map writes only once the mission has started. A rover parked
with a spinning lidar thickens the obstacles around its one viewpoint and
records whoever walks past while it waits. The gate opens on the first of two
events: the rover moving MISSION_START_M away from its first pose, or an
`explore_cmd` (the order to explore, which comes just before the first
motion). It then stays open for the rest of the run: the end of an
exploration (`explore_done`, `stop_explore_cmd`) stops the explorer, not the
map, because a cockpit goal or STOP sends `stop_explore_cmd` mid-run.

The premap (`premap=True`, MOSS_PREMAP=1 on the blueprints; off by default).
dimOS's relocalization module (run apart from this stack, or inside it with
MOSS_RELOC_MAP on a dimOS build that carries it) places this run's
`world` inside a map saved earlier and publishes two things: the fix, a
`world` -> `map` transform on `tf` (sent again every few seconds), and the
saved map's points on `loaded_map`, in `map`. Until BOTH have arrived nothing
changes: the map runs as it does without a premap, and nothing waits for one;
the first of the two to arrive says so in the log, so a run where an input
never comes shows it. Then the premap's points inside PREMAP_Z_BAND_M are
placed in `world` by the fix's x, y and yaw and written once into the LIDAR
layer at OCCUPIED_AT, the lowest wall score, flagged premap; a cell the
lidar has seen free stays free (a score below 0, or a premap wall a lidar
ray took below OCCUPIED_AT, `premap_carved`), and the camera layer is never
touched. From there a premap wall is a lidar wall like any other: one lidar
ray through its cell takes it below OCCUPIED_AT (the room may have changed
since the premap was made), a lidar hit from a new viewpoint confirms it,
the body clears it. Rays carve up to RAY_MAX_M only: a premap wall farther
than that from every place the rover stood stays until the rover comes
closer. A table top or a shoe in the premap sits above or below the scan
plane, so a lidar ray passing under or over it carves it too; the camera
marks it again on its own layer when it sees it. The premap is written again
only when a fix differs from the one it was written with by more than
PREMAP_MOVE_M or PREMAP_TURN_DEG (the same fix sent again changes nothing);
the old premap cells are cleared first, each losing exactly what the premap
added (what the sensors added since stays), so a corrected fix leaves no
ghost wall, and a premap wall a lidar ray carved is never written back: it
stays carved until a lidar hit from a new viewpoint lands on it, or Clear
map. The write does not wait for the mission gate (the premap is not a
reading taken from where the rover stands), but the publish does: nothing
goes out before the mission, as without a premap, and the premap goes out
with the first grid of the mission. A grid of premap walls with no free
floor reads to the explorer as a rover walled in at its start: a back-off,
then an exploration over before it began. Once a grid has gone out (since
the mission opened, or since Clear map), a write or a re-write publishes at
once. Clear map takes the premap out with everything else; it comes back
with a fix that moved, or a different premap.

The rover this stack comes from kept a saved grid, and it went wrong three
ways, each avoided here. The grid was loaded into both layers at full
strength, so furniture moved since could not be unlearned by the lidar and
walled the rover in: here the premap is the lidar's to unlearn, at the
weakest wall score. The run's own map was saved back as the next run's grid,
so a run walled in by ghosts handed its prison on: this module never writes
a premap, so what a run maps never becomes a prior. The map froze while the
rover looked for itself, and the drive with it: here nothing freezes or
waits for the premap.

The band is read on the premap's own heights (its floor is z = 0, as in
`world`), and the fix's z, roll and pitch are dropped: read after a full 3D
placement, a 1 deg pitch error would lift a floor point 7 m away by 12 cm,
into the band, as a wall. The band's floor (5 cm) leaves out the bottom layer
of a 5 cm voxel premap, where a premap made by a 3D lidar holds the floor
itself, and a floor written as walls is a prison. Its top (1.30 m) is the top
of the band lidar_odometry turns depth into obstacles (a table top is a
block).

The explorer sees a premap cell as a wall: 100 in the grid, although no
sensor of this run looked at it (it is not marked seen). The floor between
the walls stays unknown, since a point cloud holds where things are, never
where nothing is, so the rover still has to look at the room to explore it.
dimOS's frontier is an unknown cell with a free neighbour and no occupied
one: an unknown cell against a premap wall is never a frontier, and the
planner routes around premap walls from the first grid of the mission. That
is the choice made here; left unknown instead, a premap cell would only be a
hint the explorer drives into. The price: a premap wall that is no longer
there (a door closed when the map was made) also hides the frontier beside
it, and beyond RAY_MAX_M of every place the rover stood no ray carves it, so
it holds that frontier back until the rover comes closer. A premap cell
carved by a lidar ray turns free (the ray saw it); one cleared by a
corrected fix turns unknown again unless a sensor saw it.

Logs: "costmap: premap received: M points, K in the band" (each new premap;
"written when a world -> map fix arrives on tf" while no fix is here),
"costmap: premap fix received on tf: (x, y, yaw); written when a loaded_map
arrives" (once, when the fix comes first), "costmap: premap written: N walls
from M points, tf (x, y, yaw)", "costmap: premap re-written: tf moved by (d
cm, d deg)", "costmap: premap cleared", and with the premap off, "costmap:
premap ignored (MOSS_PREMAP=0)", once, at the first loaded_map.

Numbers (HIT_CAP, FREE_FLOOR, OCCUPIED_AT, RAY_MAX_M) are a starting point to
be tested. HIT_CAP, FREE_FLOOR, OCCUPIED_AT, NEW_VIEWPOINT_M, PUBLISH_EVERY,
RAY_MAX_M and the contact wall are read once, at import, from MOSS_HIT_CAP,
MOSS_FREE_FLOOR, MOSS_OCCUPIED_AT, MOSS_NEW_VIEWPOINT_M, MOSS_PUBLISH_EVERY,
MOSS_RAY_MAX_M and MOSS_CONTACT_WALL: a flight tries other values without a
code change, and the "costmap up" log line says which ones it runs.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

from moss_dimos.robot_profile import current as _current_profile
from moss_dimos.robot_profile import env_on

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
OCCUPIED_AT = int(round(_env_float("MOSS_OCCUPIED_AT", 2, 1, 10)))   # two hits from two places = an obstacle: whole flights were once
                             # spent walled in by cells painted from one parked spot. 1 makes the first sighting a wall (a faster
                             # block, tried as a flag on the original and never judged in flight; MOSS_OCCUPIED_AT=1)
# The layers are int8: the ceiling stays within 127, and at OCCUPIED_AT or above
# (below it nothing could ever become a wall); the floor stays within [-128, 0].
HIT_CAP = int(round(_env_float("MOSS_HIT_CAP", 10, OCCUPIED_AT, 127)))       # ceiling: no cell gets more certain than this.
                             # A lower cap unlearns a passer-by faster (a cell at the cap needs HIT_CAP - OCCUPIED_AT + 1
                             # misses to stop being a wall) and keeps a wall less long once nothing sees it any more.
FREE_FLOOR = int(round(_env_float("MOSS_FREE_FLOOR", -3, -128, 0)))          # floor: no cell gets more "free" than this.
                             # A floor nearer 0 lets a new obstacle on long-seen floor become a wall in fewer hits.
NEW_VIEWPOINT_M = _env_float("MOSS_NEW_VIEWPOINT_M", 0.10, 0.0, 1.0)   # a hit counts only if the rover moved this much since the
                             # cell's last hit (MOSS_NEW_VIEWPOINT_M=0.05 with MOSS_OCCUPIED_AT=1 is the faster-block trial)
LOW_HIT_PROTECT_S = 3.0      # after a camera obstacle hit, that cell's camera layer ignores floor samples and camera rays this long
LIDAR_Z_TOL_M = 1e-6         # lidar returns sit at EXACTLY the profile's lidar_height_m: the tag survives the float32 round trip to 5 nm (see lidar_returns)
PUBLISH_EVERY = int(round(_env_float("MOSS_PUBLISH_EVERY", 2, 1, 20)))   # one grid every N revolutions (2 = 5 Hz at 10 Hz). A 24 m grid is ~230 kB and every
                             # subscriber (planner, explorer, recorder, cockpit) decodes each one; the map
                             # QUALITY does not depend on this rate (the odometry gives it).
RAY_MAX_M = _env_float("MOSS_RAY_MAX_M", 4.0, 0.0, GRID_SPAN_M)   # rays are carved (misses) up to this range only:
                             # a cell farther than RAY_MAX_M from every viewpoint is never unlearned, so a passer-by
                             # 4.5 m away stays until the rover comes within RAY_MAX_M of where they stood. 0 = no carving.
RAY_EVERY = 2                # lidar rays carve on every other revolution; hits are taken on all
CARVE_Z_BAND_OF_CAMERA = (0.10 / 0.56, 0.45 / 0.56)   # a camera ray carves where it crosses a cell between these two fractions
                             # of the camera's height: the original's 0.10-0.45 m band over its 0.56 m camera, 0.013-0.060 m
                             # on MOSS's 0.075 m (module docstring, carve_z_band)
CAMERA_REPEAT_S = 1.0        # camera points equal to the previous cloud's, byte for byte, on a cloud stamped this soon after it:
                             # the same depth frame again (lidar_odometry re-attaches one frame for 0.5 s; twice that for jitter)
CONTACT_WALL = env_on("MOSS_CONTACT_WALL", False)   # a contact on `bump` marks the band ahead as a wall (off: contacts are reflexes)
CONTACT_BAND_M = (0.02, 0.20)   # a contact marks the band this far beyond the body's front edge, across its width
CONTACT_MOTION_S = 0.5       # with the contact wall on, a contact writes only if the odometry of the last 0.5 s shows the base
CONTACT_FORWARD_M = 0.03     # ...advancing at least this far along its heading
CONTACT_TURN_DEG = 10.0      # ...and turning less than this (3 cm and 10 deg: the motion body_clear waits for)
MISSION_START_M = 0.05       # the map opens when the rover has moved this far from its first pose (or on explore_cmd).
                             # Measuring the displacement needs no clock: the wait accumulates nothing however
                             # long it lasts, and the first useful revolution leaves with the rover.
PREMAP_Z_BAND_M = (0.05, 1.30)   # premap points strictly between these heights (its own z) become walls: above the
                             # bottom layer of a 5 cm voxel premap (a 3D lidar's floor), up to the top of the band
                             # lidar_odometry turns depth into obstacles (module docstring, the premap)
PREMAP_MOVE_M = 0.02         # a fix this far (or less) from the one the premap was written with changes nothing...
PREMAP_TURN_DEG = 1.0        # ...nor one turned this much or less; beyond either, the premap is cleared and written again


def carve_z_band(camera_z_m: float | None = None) -> tuple[float, float]:
    """The heights (m) between which a camera ray crossing a cell carves it:
    CARVE_Z_BAND_OF_CAMERA times the camera's height above the floor, the
    profile's camera_xyz_base_m[2] unless given (0.013-0.060 m at 0.075 m).
    The module docstring has the derivation."""
    h = _current_profile().camera_xyz_base_m[2] if camera_z_m is None else camera_z_m
    return CARVE_Z_BAND_OF_CAMERA[0] * h, CARVE_Z_BAND_OF_CAMERA[1] * h


def camera_xy(pose_xy: tuple[float, float], yaw: float) -> tuple[float, float]:
    """Where the camera itself is, from the base pose: the profile's
    camera_xyz_base_m (read at call time, the same numbers lidar_odometry
    projects the depth with) turned by the heading.

    Rays cast from the base would open the carve band at the wrong range and
    sweep beside the cells the camera really looked through: on MOSS the
    camera is 0.156 m AHEAD of base_link, so a floor sample's ray cast from
    the base carves three cells right in front of the nose that the camera's
    own ray crosses above the band. The original rover's costmap kept its
    camera offset as a constant beside the odometry's, and when the camera
    moved, the two disagreed: here both read the profile."""
    cx, cy = _current_profile().camera_xyz_base_m[:2]
    c, s = math.cos(yaw), math.sin(yaw)
    return (pose_xy[0] + c * cx - s * cy, pose_xy[1] + s * cx + c * cy)


def planar_fix(transform: Any) -> tuple[float, float, float]:
    """(x, y, yaw) of a `world` -> `map` transform: where the premap's origin
    sits in `world` and how it is turned. A point (mx, my) of the premap lands
    at (x + cos(yaw) mx - sin(yaw) my, y + sin(yaw) mx + cos(yaw) my), the
    convention dimOS's own ray tracer places a loaded map by. The transform's
    z, roll and pitch are dropped (module docstring, the premap)."""
    q = transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return float(transform.translation.x), float(transform.translation.y), yaw


def fix_moved(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float]:
    """How far fix `b` is from fix `a`: (metres in the plane, degrees of yaw, wrapped)."""
    turn = (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi
    return math.hypot(b[0] - a[0], b[1] - a[1]), abs(math.degrees(turn))


class ScoredGrid:
    """Two int8 score layers over a fixed world grid, plus where each cell was last hit from."""

    def __init__(self, resolution: float = RESOLUTION_M, span_m: float = GRID_SPAN_M,
                 centre: tuple[float, float] = (0.0, 0.0)) -> None:
        self.res = resolution
        self.n = int(round(span_m / resolution))
        self.ox = centre[0] - span_m / 2.0
        self.oy = centre[1] - span_m / 2.0
        self.lidar = np.zeros((self.n, self.n), dtype=np.int8)     # what the lidar saw in its scan plane
        self.camera = np.zeros((self.n, self.n), dtype=np.int8)    # what the depth camera saw: hit by it, freed only by its floor samples, its rays and the body
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self._last_hit_xy = np.full((self.n, self.n, 2), np.nan, dtype=np.float32)
        # when the camera last called each cell an obstacle (monotonic seconds):
        # it protects a thin leg for LOW_HIT_PROTECT_S. Runtime only: a
        # monotonic clock means nothing in another process.
        self._last_low_hit = np.full((self.n, self.n), -np.inf, dtype=np.float64)
        self._cam_prev = np.zeros((self.n, self.n), dtype=bool)   # camera obstacle cells of the PREVIOUS depth frame (moving-object gate)
        self.premap = np.zeros((self.n, self.n), dtype=bool)          # cells the premap made (or found) a lidar wall
        self._premap_added = np.zeros((self.n, self.n), dtype=np.int8)  # ...and what it added to their lidar score
        # premap walls a lidar ray took below OCCUPIED_AT: seen free, so a
        # re-write never paints them back; a lidar hit from a new viewpoint
        # lifts the mark (the sensor sees something there again)
        self.premap_carved = np.zeros((self.n, self.n), dtype=bool)

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
        # passer-by hammered from one parked spot must never become a wall
        # (whole flights were once spent walled in by exactly such cells).
        # From one viewpoint a cell rises to OCCUPIED_AT - 1 and no
        # further; the second viewpoint (NEW_VIEWPOINT_M of motion, 0.10 m)
        # makes it a wall. Driving toward anything real crosses viewpoints
        # within a metre, so legs and furniture still map on approach.
        # The viewpoint rule bounds the INCREASE, never the value: a
        # same-viewpoint hit adds nothing and takes nothing away. Clamping the
        # value instead would let one revolution from a parked rover push a
        # wall built from twenty viewpoints back below OCCUPIED_AT, and
        # occupancy() would publish free floor where the wall stands.
        new = np.where(moved, np.minimum(cur + 1, HIT_CAP), cur).astype(np.int8)
        layer[gy, gx] = new
        self._last_hit_xy[gy[moved], gx[moved]] = from_xy
        self.seen[gy, gx] = True
        if layer is self.lidar:
            self.premap_carved[gy[moved], gx[moved]] = False
        return int(moved.sum())

    def _miss(self, layer: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> None:
        if len(gx) == 0:
            return
        cur = layer[gy, gx].astype(np.int16)
        new = np.maximum(cur - 1, FREE_FLOOR).astype(np.int8)
        layer[gy, gx] = new
        self.seen[gy, gx] = True
        if layer is self.lidar:
            self.premap_carved[gy, gx] |= self.premap[gy, gx] & (new < OCCUPIED_AT)

    _revs: int = 0

    def lidar_revolution(self, hits_xy: np.ndarray, from_xy: tuple[float, float]) -> None:
        """World-frame lidar hits of one revolution, seen from `from_xy`:
        the cells along each ray (up to just short of the hit, RAY_MAX_M at
        most, every RAY_EVERY-th revolution) take a miss, the hit cells a hit.
        Lidar layer only: a ray 0.20 m up passes over what the camera holds
        (a shoe, a chair base), so it is no evidence against it."""
        if len(hits_xy) == 0:
            return
        self._revs += 1
        if self._revs % RAY_EVERY == 0:
            cells = self._ray_cells(from_xy, hits_xy)
            if cells is not None:
                self._miss(self.lidar, cells[0], cells[1])
        self._hit(self.lidar, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def camera_obstacles(self, pts_xy: np.ndarray, from_xy: tuple[float, float],
                         now: float | None = None) -> tuple[int, int]:
        """One depth frame's obstacle points, seen from `from_xy`: a hit on
        the camera layer (the two-viewpoint rule as for lidar hits), and the
        cell is stamped: for the next LOW_HIT_PROTECT_S the floor beside it
        may not erase it (the thin leg, module docstring).

        MOVING things are never written into the map: a cell only takes a hit
        if the camera ALSO saw an obstacle within one cell of it in the
        previous frame. One call per DISTINCT depth frame (MossCostMap keys
        the calls on the frames); an empty frame clears the memory.

        Returns (cells that took a hit, cells held back by the gate)."""
        gx, gy = self.cell(pts_xy[:, 0], pts_xy[:, 1])
        if len(gx) == 0:
            self._cam_prev[:] = False
            return 0, 0
        # NEIGHBOURHOOD repeat, not exact-cell repeat: a THIN standing leg
        # re-projects a cell off between frames (5 cm of quantisation from a
        # moving rover), and an exact-cell repeat would filter it out like a
        # mover. One cell of tolerance keeps every immobile thing; a walker or
        # a vacuum still jumps further than 10 cm per frame.
        prev_near = self._cam_prev.copy()
        prev_near[:-1, :] |= self._cam_prev[1:, :]
        prev_near[1:, :] |= self._cam_prev[:-1, :]
        prev_near[:, :-1] |= prev_near[:, 1:]
        prev_near[:, 1:] |= prev_near[:, :-1]
        keep = prev_near[gy, gx]
        self._cam_prev[:] = False
        self._cam_prev[gy, gx] = True
        held = int(len(np.unique(gy[~keep] * self.n + gx[~keep])))
        if not keep.any():
            return 0, held
        rx = self.ox + (gx[keep] + 0.5) * self.res
        ry = self.oy + (gy[keep] + 0.5) * self.res
        took = self._hit(self.camera, rx, ry, from_xy)
        self._last_low_hit[gy[keep], gx[keep]] = time.monotonic() if now is None else now
        return took, held

    def camera_floor(self, pts_xy: np.ndarray, now: float | None = None) -> None:
        """The camera saw bare floor here: a miss on the camera layer, the
        sensor's way to forget a shoe that was taken away.

        Except on a cell the camera called an obstacle less than
        LOW_HIT_PROTECT_S ago: there the floor sample is dropped. A thing
        thinner than a 5 cm cell is seen in one frame and its floor in the
        next, and the two cancelled forever.

        The lidar layer is untouched: the camera looks under the scan plane,
        and floor seen beneath a sofa's edge says nothing about the edge the
        lidar sees 0.20 m up."""
        now = time.monotonic() if now is None else now
        gx, gy = self.cell(pts_xy[:, 0], pts_xy[:, 1])
        if len(gx) == 0:
            return
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        protected = self._last_low_hit[gy, gx] > now - LOW_HIT_PROTECT_S
        self._miss(self.camera, gx[~protected], gy[~protected])

    def camera_rays(self, pts_xyz: np.ndarray, from_xy: tuple[float, float],
                    now: float | None = None) -> None:
        """The camera's rays carve the camera layer, symmetric to the lidar's:
        the map corrects itself every time the camera passes over a spot,
        instead of a ghost living until somebody clears the map.

        `from_xy` is where the camera is (`camera_xy`), at the profile's
        camera height. Every cell a ray crosses inside the carve band
        (`carve_z_band`) held nothing standing, or the ray would have ended
        on it: a miss. Obstacle endpoints and floor samples alike; a ray stops
        one cell short of its point and at RAY_MAX_M. Cells the camera called
        an obstacle less than LOW_HIT_PROTECT_S ago are skipped: a real thing
        is re-hit every frame, so only what the camera has STOPPED confirming
        fades. The lidar layer is never touched: only a lidar ray may retract
        a lidar claim."""
        if len(pts_xyz) == 0:
            return
        now = time.monotonic() if now is None else now
        h = _current_profile().camera_xyz_base_m[2]
        lo, hi = carve_z_band(h)
        dx = pts_xyz[:, 0] - from_xy[0]
        dy = pts_xyz[:, 1] - from_xy[1]
        r = np.hypot(dx, dy)
        keep = r > self.res
        if not keep.any():
            return
        dx, dy, r = dx[keep], dy[keep], r[keep]
        z_end = pts_xyz[keep, 2]
        end = np.minimum(r - self.res, RAY_MAX_M)   # stop one cell short of the point
        nmax = int(np.floor(end.max() / self.res))
        if nmax < 1:
            return
        d = (np.arange(1, nmax + 1) * self.res)[None, :]
        f = d / r[:, None]
        z = h + (z_end[:, None] - h) * f
        valid = (d <= end[:, None]) & (z > lo) & (z < hi)
        if not valid.any():
            return
        ux, uy = (dx / r)[:, None], (dy / r)[:, None]
        xs = (from_xy[0] + ux * d)[valid]
        ys = (from_xy[1] + uy * d)[valid]
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        unprotected = self._last_low_hit[gy, gx] <= now - LOW_HIT_PROTECT_S
        self._miss(self.camera, gx[unprotected], gy[unprotected])

    def contact(self, pose: tuple) -> int:
        """The rover pushed against something (MossCostMap.handle_bump, with
        MOSS_CONTACT_WALL=1 and the rover driving straight forward): the cells
        of a short band right ahead of the body become a wall on the camera
        layer at once (no viewpoint rule: a contact is a physical certainty,
        like body_clear). The planner then routes around the spot instead of
        driving back into it. Freed like any camera wall: by the camera seeing
        bare floor there or through it at low height, or by the body driving
        over it."""
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
        self.seen[gy, gx] = True
        return int(len(gx))

    def write_premap(self, xy: np.ndarray) -> tuple[int, int]:
        """The premap's walls, already placed in `world` (x, y): each cell
        becomes a lidar wall at OCCUPIED_AT (a cell already above keeps its
        score), flagged premap, and remembers what the premap added to it.
        A cell the lidar has seen free (score below 0, or a premap wall a
        lidar ray carved, `premap_carved`) stays as it is: the live map wins
        over the prior. Not marked seen, and the camera layer is never touched
        (module docstring, the premap). A premap already in the grid is
        cleared first. Returns (cells flagged, cells left free)."""
        self.clear_premap()
        gx, gy = self.cell(xy[:, 0], xy[:, 1])
        if len(gx) == 0:
            return 0, 0
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        cur = self.lidar[gy, gx].astype(np.int16)
        free = (cur < 0) | self.premap_carved[gy, gx]
        gx, gy, cur = gx[~free], gy[~free], cur[~free]
        added = np.maximum(OCCUPIED_AT - cur, 0)
        self.lidar[gy, gx] = (cur + added).astype(np.int8)
        self._premap_added[gy, gx] = added.astype(np.int8)
        self.premap[gy, gx] = True
        return int(len(gx)), int(free.sum())

    def clear_premap(self) -> int:
        """Take the premap out: every flagged cell loses exactly what the
        premap added to its lidar score, so what the sensors did since stays
        (a live hit from a new viewpoint, a ray's miss, the body) and a
        corrected fix leaves no ghost wall. A cell no sensor saw goes back to
        unknown. The carved marks stay: what a lidar ray saw free is still
        free for the next write. Returns the cells cleared."""
        gy, gx = np.nonzero(self.premap)
        if len(gx) == 0:
            return 0
        back = self.lidar[gy, gx].astype(np.int16) - self._premap_added[gy, gx]
        self.lidar[gy, gx] = np.clip(back, FREE_FLOOR, HIT_CAP).astype(np.int8)
        self._premap_added[gy, gx] = 0
        self.premap[gy, gx] = False
        return int(len(gx))

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
        footprint (half extents around base_link) is certainly free, on both
        layers, and seen. Without it, cells written before the rover drove
        over them stay and can wall it in. Keep the footprint exact, never
        wider, so a wall right against the body survives."""
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
        """The occupancy grid cropped to the observed area and the premap's
        walls (+ margin): (grid, origin_x, origin_y)."""
        known = self.seen | self.premap
        if not known.any():
            return None
        ys, xs = np.nonzero(known)
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
    band: read as a 5 mm band it once stole every camera point at the scan
    plane's height, past the moving-object gate and out of the camera rays'
    reach, and a person's side became a permanent wall.
    """
    return np.abs(pts[:, 2] - _current_profile().lidar_height_m) < LIDAR_Z_TOL_M


class MossCostMap(Module):
    """Replaces dimOS's CostMapper on MOSS.

    Ins: `lidar` (the world cloud from lidar_odometry, lidar returns tagged
    with z = profile.lidar_height_m exactly, the depth camera's obstacle
    points at their true height), `camera_floor` (the camera's bare floor,
    z = 0), `odom` (base pose in world), `explore_cmd` (the order that opens
    the map), `explore_done` and `stop_explore_cmd` (logged; the map keeps
    writing), `clear_map_cmd` (a fresh grid), `bump` (a front contact: a wall
    ahead only with MOSS_CONTACT_WALL=1, module docstring), `loaded_map` and
    `tf` (a relocalization's premap and its `world` -> `map` fix: lidar walls
    with `premap=True`, ignored otherwise; module docstring, the premap).
    Out: `global_costmap` (the stream name the planner and the explorer
    listen to)."""

    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]   # bare floor the depth camera saw (lidar_odometry), once per depth frame: frees camera obstacles
    odom: In[PoseStamped]
    explore_done: In[Bool]          # explorer2 found no reachable frontier left: logged, the map keeps writing
    stop_explore_cmd: In[Bool]      # exploration stopped on request (explore_ctl.py stop, the cockpit): logged, the map keeps writing
    explore_cmd: In[Bool]           # the order to explore: opens the map before the first motion
    clear_map_cmd: In[Bool]         # the cockpit's Clear map (explore_ctl.py clear): a fresh grid, the mission gate as it is
    bump: In[Bool]                  # a front contact: logged; with MOSS_CONTACT_WALL=1 and the rover driving straight forward, the band ahead becomes a wall
    loaded_map: In[PointCloud2]     # a relocalization's premap, in `map`: lidar walls once a fix places it (premap=True)
    tf: In[TFMessage]               # the tf stream: only a `world` -> `map` transform (the relocalization's fix) is read
    global_costmap: Out[OccupancyGrid]

    def __init__(self, world_frame: str = "world", premap: bool = False, map_frame: str = "map",
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.world_frame = world_frame
        # the premap (module docstring): off, every loaded_map is ignored
        # (said once); on, it becomes lidar walls once a fix places it
        self.premap_on = bool(premap)
        self.map_frame = map_frame
        self._premap_xy: np.ndarray | None = None      # the premap's points inside PREMAP_Z_BAND_M, (x, y) in `map`
        self._premap_points = 0                        # ...out of this many in the loaded cloud
        self._premap_key: bytes | None = None          # the loaded cloud's points as bytes: the same premap sent again is no news
        self._premap_fix: tuple[float, float, float] | None = None       # the latest `world` -> `map` fix (x, y, yaw)
        self._premap_written: tuple[float, float, float] | None = None   # the fix the premap was last written with
        self._premap_ignored_said = False
        self._premap_fix_said = False                  # "fix received" is said once, when the fix comes first
        self._grid: ScoredGrid | None = None
        self._pose_xy: tuple[float, float] | None = None
        self._pose_yaw = 0.0                   # the camera rides on the nose: its rays need the heading
        self._poses: deque = deque()           # (monotonic s, x, y, yaw) of the last CONTACT_MOTION_S, for the contact wall
        self._revolutions = 0
        # the depth frame the camera layer last took (moving-object gate): its
        # points as bytes and the stamp of the cloud that first carried them
        self._cam_key: bytes | None = None
        self._cam_key_t = 0.0
        self._camera_pts = 0           # camera obstacle points of the frames taken since the last publish (log only)
        self._camera_cells = 0         # ...the cells they hit
        self._camera_held = 0          # ...the cells the moving-object gate held back
        self._camera_frames = 0        # ...how many distinct depth frames
        self._camera_repeats = 0       # ...and how many clouds carried a frame already taken
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
        lo, hi = carve_z_band()
        logger.info(f"costmap up: {RESOLUTION_M} m cells, hit cap {HIT_CAP}, free floor {FREE_FLOOR}, "
                    f"occupied at {OCCUPIED_AT} (new viewpoint {NEW_VIEWPOINT_M:g} m), rays carved to {RAY_MAX_M:g} m, "
                    f"camera rays carve {lo:.3f}-{hi:.3f} m up, contact wall {'on' if CONTACT_WALL else 'off'}, "
                    f"premap {'on' if self.premap_on else 'off'}")

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._pose_xy = (float(msg.position.x), float(msg.position.y))
        q = msg.orientation
        self._pose_yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        now = time.monotonic()
        self._poses.append((now, self._pose_xy[0], self._pose_xy[1], self._pose_yaw))
        while self._poses and now - self._poses[0][0] > CONTACT_MOTION_S:
            self._poses.popleft()
        if self._grid is None:
            self._grid = ScoredGrid(centre=self._pose_xy)
            logger.info(f"costmap: grid centred on ({self._pose_xy[0]:+.2f}, {self._pose_xy[1]:+.2f})")
            self._premap_update()       # a premap and its fix that came before the first pose
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
        keeps writing."""
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

    def _recent_motion(self, now: float | None = None) -> tuple[float, float]:
        """How the base moved over the last CONTACT_MOTION_S of odometry
        before `now`: (metres along the heading it had at the window's start,
        degrees turned). Less than two poses in the window (the odometry
        quiet) -> (0, 0)."""
        now = time.monotonic() if now is None else now
        poses = [p for p in self._poses if now - p[0] <= CONTACT_MOTION_S]
        if len(poses) < 2:
            return 0.0, 0.0
        _, x0, y0, yaw0 = poses[0]
        _, x1, y1, yaw1 = poses[-1]
        forward = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        turn = math.degrees((yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi)
        return forward, turn

    async def handle_bump(self, msg: Bool) -> None:
        """A front contact. Contacts are reflexes (the planner's escape), so by
        default nothing is written. With MOSS_CONTACT_WALL=1, a contact felt
        while driving straight forward (`_recent_motion`) turns the cells
        just ahead of the body into a wall at once, so the planner routes
        around what the sensors did not see (a cable, a bin edge below the
        camera's view) instead of driving into it again. Each contact says in
        the log whether it was written."""
        if not bool(getattr(msg, "data", False)) or self._grid is None or self._pose_xy is None:
            return
        if self._write_frozen():
            return
        where = (f"costmap: contact at ({self._pose_xy[0]:+.2f}, {self._pose_xy[1]:+.2f}) heading "
                 f"{np.degrees(self._pose_yaw):+.0f} deg")
        if not CONTACT_WALL:
            logger.info(f"{where} - not written: contacts are reflexes, only the lidar and the camera write the map "
                        "(MOSS_CONTACT_WALL=1 writes a forward contact)")
            return
        forward, turn = self._recent_motion()
        if forward < CONTACT_FORWARD_M or abs(turn) >= CONTACT_TURN_DEG:
            logger.info(f"{where} - not written: not driving straight forward ({forward * 100:+.0f} cm along the "
                        f"heading, {turn:+.0f} deg in the last {CONTACT_MOTION_S:g} s)")
            return
        n = self._grid.contact((self._pose_xy[0], self._pose_xy[1], self._pose_yaw))
        logger.info(f"{where}: {n} cells ahead marked as a wall")
        self._publish()

    async def handle_clear_map_cmd(self, msg: Bool) -> None:
        """A fresh grid on request (the cockpit's Clear map): everything the map
        learned goes, centred again on the current pose; the mission gate stays
        as it is, so an open mission keeps writing from the next revolution and
        a run that has not started still waits for its order or its 5 cm."""
        if not bool(getattr(msg, "data", False)) or self._pose_xy is None:
            return
        had_premap = self._grid is not None and bool(self._grid.premap.any())
        self._grid = ScoredGrid(centre=self._pose_xy)
        self._revolutions = 0
        self._cam_key = None
        self._camera_pts = self._camera_cells = self._camera_held = self._camera_frames = self._camera_repeats = 0
        logger.info(f"costmap: map CLEARED on request - fresh grid centred on ({self._pose_xy[0]:+.2f}, "
                    f"{self._pose_xy[1]:+.2f}), mission {'open' if not self._mission_frozen else 'not started'}")
        if had_premap:
            # the premap goes with the rest: a premap that walls the rover in
            # must have a way out. It comes back with a fix that moved.
            logger.info(f"costmap: premap cleared with the map - written again on a fix that moves more than "
                        f"{PREMAP_MOVE_M * 100:g} cm or {PREMAP_TURN_DEG:g} deg")

    async def handle_loaded_map(self, msg: PointCloud2) -> None:
        """A relocalization's premap, in `map`. Off: ignored, said once. On:
        its points inside PREMAP_Z_BAND_M are kept, and written once a fix
        places them (_premap_update). The same premap sent again changes
        nothing; another one replaces it (the old walls cleared first)."""
        if not self.premap_on:
            if not self._premap_ignored_said:
                self._premap_ignored_said = True
                logger.info("costmap: premap ignored (MOSS_PREMAP=0)")
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64).reshape(-1, 3)
        key = pts.tobytes()
        if key == self._premap_key:
            return
        lo, hi = PREMAP_Z_BAND_M
        self._premap_key = key
        self._premap_points = len(pts)
        self._premap_xy = pts[(pts[:, 2] > lo) & (pts[:, 2] < hi), :2]
        # said for each new premap: a run on MOSS_PREMAP=1 without this line
        # received none (the relocalization on another transport, or its one
        # publish lost)
        logger.info(f"costmap: premap received: {len(pts)} points, {len(self._premap_xy)} in the {lo:g}-{hi:g} m band"
                    + ("" if self._premap_fix is not None else "; written when a world -> map fix arrives on tf"))
        if self._premap_written is not None:
            self._premap_clear()
            self._premap_written = None
        self._premap_update()

    async def handle_tf(self, msg: TFMessage) -> None:
        """The tf stream. Only a `world` -> `map` transform is read (the
        relocalization's fix, sent again every few seconds); the odometry's
        and the cameras' transforms pass by. Off: nothing is read."""
        if not self.premap_on:
            return
        fix = None
        for t in getattr(msg, "transforms", ()):
            if t.frame_id == self.world_frame and t.child_frame_id == self.map_frame:
                fix = planar_fix(t)
        if fix is None:
            return
        if self._premap_xy is None and not self._premap_fix_said:
            self._premap_fix_said = True
            logger.info(f"costmap: premap fix received on tf: ({fix[0]:+.2f}, {fix[1]:+.2f}, "
                        f"{math.degrees(fix[2]):+.1f} deg); written when a loaded_map arrives")
        self._premap_fix = fix
        self._premap_update()

    def _premap_update(self) -> None:
        """Write the premap once the grid, the premap and a fix are all here;
        write it again only when the fix moved more than PREMAP_MOVE_M or
        turned more than PREMAP_TURN_DEG from the one it was written with
        (the same fix sent again changes nothing, and a fix creeping by small
        steps is measured against the written one, so it cannot drift away
        unnoticed). Published at once only when the map has already put a
        grid out since the mission opened (or since Clear map); before that,
        the premap goes out with the map's first grid (module docstring)."""
        if self._grid is None or self._premap_xy is None or self._premap_fix is None:
            return
        fix = self._premap_fix
        if self._premap_written is not None:
            moved_m, turned_deg = fix_moved(self._premap_written, fix)
            if moved_m <= PREMAP_MOVE_M and turned_deg <= PREMAP_TURN_DEG:
                return
            logger.info(f"costmap: premap re-written: tf moved by ({moved_m * 100:.1f} cm, {turned_deg:.1f} deg)")
            self._premap_clear()
        self._premap_written = fix
        x, y, yaw = fix
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = self._premap_xy[:, 0], self._premap_xy[:, 1]
        walls, kept_free = self._grid.write_premap(np.column_stack([x + c * mx - s * my, y + s * mx + c * my]))
        lo, hi = PREMAP_Z_BAND_M
        logger.info(f"costmap: premap written: {walls} walls from {self._premap_points} points, "
                    f"tf ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):+.1f} deg); {len(self._premap_xy)} points in the "
                    f"{lo:g}-{hi:g} m band, {kept_free} cells the lidar has seen free left free")
        # _revolutions counts the revolutions written since the mission opened
        # (or since Clear map), and the first grid leaves at PUBLISH_EVERY: a
        # grid of premap walls alone would read to explorer2 as a walled-in start
        if self._revolutions >= PUBLISH_EVERY:
            self._publish()

    def _premap_clear(self) -> None:
        """Take the premap's walls out of the grid (ScoredGrid.clear_premap), said in the log."""
        n = self._grid.clear_premap() if self._grid is not None else 0
        if n:
            logger.info(f"costmap: premap cleared: {n} cells")

    def _camera_xy(self) -> tuple[float, float]:
        """The ray origin for everything the camera saw (see camera_xy)."""
        assert self._pose_xy is not None
        return camera_xy(self._pose_xy, self._pose_yaw)

    async def handle_camera_floor(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None or self._write_frozen():
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts):
            now = time.monotonic()
            self._grid.camera_floor(pts[:, :2], now=now)
            self._grid.camera_rays(pts, self._camera_xy(), now=now)

    def _camera_frame(self, cam: np.ndarray, stamp: float) -> None:
        """The camera points of one revolution's cloud (stamped `stamp`, wall
        seconds), taken once per depth frame: hits through the moving-object
        gate, then the frame's rays.

        A cloud whose points equal the last frame's, byte for byte, stamped
        less than CAMERA_REPEAT_S after the cloud that first carried them,
        carries that frame again (lidar_odometry re-attaches it to every
        revolution for 0.5 s) and writes nothing. A cloud without camera
        points means no frame of the last 0.5 s saw an obstacle: the gate
        forgets, like an empty frame."""
        if len(cam) == 0:
            if self._cam_key is not None:
                self._grid.camera_obstacles(cam[:, :2], self._pose_xy)
                self._cam_key = None
            return
        key = cam.tobytes()
        if key == self._cam_key and stamp - self._cam_key_t < CAMERA_REPEAT_S:
            self._camera_repeats += 1
            return
        self._cam_key, self._cam_key_t = key, stamp
        now = time.monotonic()
        # hits first: they stamp the 3 s protection, so a frame never carves
        # the very cells it is confirming
        took, held = self._grid.camera_obstacles(cam[:, :2], self._pose_xy, now=now)
        self._grid.camera_rays(cam, self._camera_xy(), now=now)
        self._camera_frames += 1
        self._camera_pts += len(cam)
        self._camera_cells += took
        self._camera_held += held

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
        # the cloud's own stamp (lidar_odometry's publish time), else the wall clock
        self._camera_frame(pts[~tag], float(getattr(msg, "ts", 0.0) or 0.0) or time.time())
        self._grid.lidar_revolution(hits[:, :2], self._pose_xy)
        self._revolutions += 1
        if self._revolutions % PUBLISH_EVERY == 0:
            if self._camera_pts:
                logger.info(f"costmap: camera obstacles: {self._camera_pts} points, {self._camera_cells} cell hits "
                            f"over the last {PUBLISH_EVERY} revolutions ({self._camera_frames} depth frames, "
                            f"{self._camera_repeats} repeats skipped, {self._camera_held} cells held back by the "
                            "moving-object gate)")
            self._camera_pts = self._camera_cells = self._camera_held = 0
            self._camera_frames = self._camera_repeats = 0
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
