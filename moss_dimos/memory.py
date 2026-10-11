"""MOSS's run recording: what the cockpit shows, written to SQLite.

dimOS's Recorder (its "memory") stores the In ports below with a pose per
observation; afterwards the run can be replayed without the robot: draw the
map, try a costmap function or a planner on it, compare. The recording also
survives a crash, as long as the rotation of the previous recording keeps
the whole SQLite WAL trio together - see rotate_recording() below.

Two layers:

    RotatingRecorder   odom + global_costmap, the odom pose setter, and the
                       WAL-safe rotation of the previous recording in start()
    MossRecorder       MOSS's streams on top of it

MossRecorder's streams:

    odom             PoseStamped    the pose source of every anchored stream
    global_costmap   OccupancyGrid  world frame, poseless
    lidar            PointCloud2    world-frame cloud from lidar_odometry, at the last odom pose
    color_image      Image          D455F colour (JPEG on the bus and in the db), at the last odom pose
    wrist_image      Image          the arm's UVC camera (JPEG), at the last odom pose
    cmd_vel          Twist          what the base was told, poseless
    base_telemetry   String         firmware JSON (battery, mode, track outputs), poseless
    episode_marker   String         {action, name, ts} from the cockpit, poseless

The db path is absolute (robot_profile.moss_recording_path, default
~/.local/state/moss/recordings/moss.db, MOSS_STATE_DIR moves the whole
tree): a relative path would land in dimOS's site-packages.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger

from moss_dimos.robot_profile import moss_recording_path

logger = setup_logger()

# a WAL recording is three files: the .db + these two
WAL_SIDECARS = ("-wal", "-shm")


def _sidecars(db: Path) -> list[Path]:
    """The -wal/-shm of *db* that exist right now."""
    return [p for p in (db.with_name(db.name + s) for s in WAL_SIDECARS) if p.exists()]


def _rotated(db: Path, stamp: str) -> Path:
    return db.with_name(f"{db.stem}.{stamp}{db.suffix}")


def _free_stamp(db: Path) -> str:
    """A stamp no rotated file carries yet: a rotation never overwrites."""
    now = stamp = datetime.now().strftime("%Y%m%d%H%M%S")
    n = 0
    while True:
        taken = _rotated(db, stamp)
        if not any(taken.with_name(taken.name + s).exists() for s in ("", *WAL_SIDECARS)):
            return stamp
        n += 1
        stamp = f"{now}-{n}"


def quarantine_orphans(db: Path) -> Path | None:
    """Park a -wal/-shm left with no db beside it, dated, never deleted.

    Whatever the killed run committed can live in that -wal alone, and it takes
    its own db to read it back: moving it out of the way is all we may do. The
    next run then opens a fresh db with nothing stale beside it - opening one on
    top of an orphaned -wal rewrites the -wal, which is the loss itself.
    """
    orphans = _sidecars(db)
    if db.exists() or not orphans:
        return None
    stamp = datetime.now().strftime("%Y%m%d%H%M%S")
    dest = db.parent / "quarantine" / stamp
    n = 0
    while dest.exists():
        n += 1
        dest = db.parent / "quarantine" / f"{stamp}-{n}"
    dest.mkdir(parents=True)
    for p in orphans:
        p.rename(dest / p.name)
    logger.warning("Orphaned %s with no %s -> quarantined in %s",
                   [p.name for p in orphans], db.name, dest)
    return dest


def _prune(db: Path, keep_last: int) -> None:
    """backup_keep_last, applied to whole trios - dimOS's prune unlinks the .db
    and leaves its -wal behind to shadow a later recording. keep_last=0 means
    'delete the previous recording' upstream; here it prunes nothing, a
    recording is never destroyed to save room."""
    if keep_last <= 0:
        return
    pattern = re.compile(rf"^{re.escape(db.stem)}\.(\d{{14}})(?:-(\d+))?{re.escape(db.suffix)}$")
    found = [(m, p) for m, p in
             ((pattern.match(p.name), p) for p in db.parent.glob(f"{db.stem}.*{db.suffix}"))
             if m]
    # oldest first, by stamp then by same-second rank: sorting the names would
    # put "<stamp>-2.db" before "<stamp>.db" and retire the newest recording
    olds = [p for _, p in sorted(found, key=lambda f: (f[0].group(1), int(f[0].group(2) or 0)))]
    for old in olds[:-keep_last]:
        for p in (old, *_sidecars(old)):
            p.unlink()


def rotate_recording(db_path: str | Path, keep_last: int = 10) -> Path | None:
    """Move the previous recording aside - the WAL trio, not the db alone.

    A recording is moss.db + moss.db-wal + moss.db-shm, and a run killed with
    SIGKILL (`dimos stop` escalates to it) leaves its last commits in the
    -wal. dimOS's backup_file() renames the .db alone: the backup then opens
    with an empty schema and the next run's fresh db writes over the -wal, so
    the crashed run - the one worth reading back - is exactly the one
    destroyed. Rename the three together instead.

    Returns the rotated .db, or None when there was no recording to rotate.
    """
    db = Path(db_path)
    db.parent.mkdir(parents=True, exist_ok=True)
    if not db.exists():
        quarantine_orphans(db)
        return None

    rotated = _rotated(db, _free_stamp(db))
    # the db moves first: interrupted mid-rotation, the sidecars are quarantined
    # by the next run instead of being adopted by a fresh recording
    for src in (db, *_sidecars(db)):
        src.rename(rotated.with_name(rotated.name + src.name[len(db.name):]))
    logger.info("Rotated recording %s -> %s (WAL trio)", db, rotated)
    quarantine_orphans(db)
    _prune(db, keep_last)
    return rotated


class RotatingRecorderConfig(RecorderConfig):
    # a factory, not a value: MOSS_STATE_DIR is read when the module is built
    db_path: str | Path = Field(default_factory=moss_recording_path)
    # world-frame streams need no pose anchoring; the odom stream sets the pose
    poseless_streams: list[str] = Field(default_factory=lambda: ["global_costmap"])
    # tf at ~23 Hz is recorded by default as the POSE FALLBACK for streams
    # without a setter - every stream here has a setter or is poseless, so
    # recording tf would buy ~23 sqlite commits/s for nothing (the recorder
    # then eats a full core, one commit per observation).
    record_tf: bool = False


class RotatingRecorder(Recorder):
    """Recording base: trajectory + decision map, with the odom pose setter
    (subclasses anchor their own streams on `_last_odom_pose`) and the
    WAL-safe rotation of the previous recording in start()."""
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    config: RotatingRecorderConfig

    _last_odom_pose: Pose | None = None

    # dimOS rotates in Recorder.start(), with backup_file(): the .db alone.
    # Rotate here first, trio and all - Recorder then finds no db and rotates
    # nothing. Only BACKUP is ours; APPEND/OVERWRITE/ERROR stay upstream's.
    @rpc
    def start(self) -> None:
        if not self.config.g.replay and self.config.on_existing is OnExisting.BACKUP:
            rotate_recording(self.config.db_path, keep_last=self.config.backup_keep_last)
        super().start()

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        self._last_odom_pose = Pose(position=msg.position, orientation=msg.orientation)
        return self._last_odom_pose


MOSS_POSELESS_STREAMS = ["global_costmap", "cmd_vel", "base_telemetry", "episode_marker"]


class MossMemoryConfig(RotatingRecorderConfig):
    # a factory, not a value: MOSS_STATE_DIR is read when the module is built
    db_path: str | Path = Field(default_factory=moss_recording_path)
    poseless_streams: list[str] = Field(default_factory=lambda: list(MOSS_POSELESS_STREAMS))
    record_tf: bool = False


class MossRecorder(RotatingRecorder):
    """odom + global_costmap (inherited, with the odom pose setter and the
    WAL-safe rotation in start()), plus the MOSS streams."""

    lidar: In[PointCloud2]
    color_image: In[Image]
    wrist_image: In[Image]
    cmd_vel: In[Twist]
    base_telemetry: In[String]
    episode_marker: In[String]
    config: MossMemoryConfig

    @pose_setter_for("lidar", "color_image", "wrist_image")
    async def _anchor_on_odom(self, msg) -> Pose | None:
        # the cloud is already in the world frame and the images have no tf
        # chain on MOSS: anchor both at the rover's last pose, so memory
        # queries ("what did it see from here") still work
        return self._last_odom_pose
