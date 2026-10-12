#!/usr/bin/env python3
"""Speed watchdog: tails the run's dimOS log, computes the body's speed from
the lidar odometry lines, and stops the run when the body moves faster than
the profile's ceiling (robot_profile max_speed_mps, 0.45 m/s, overridden by
MOSS_MAX_SPEED_MPS). It must be running whenever the rover explores
unattended: tools/arm_speed_guard.sh arms it for one run.

The ceiling is the planner's cruise plus a margin, never below it: a ceiling
at or under the speed the stack itself commands would stop every normal run.
Cruise = dimOS LocalPlanner._speed (PLANNER_SPEED_MPS, 0.55 m/s) x the
launcher's speed scale (MOSS_NERF_SPEED, 0.7 by default in tools/moss-fly.sh)
= 0.385 m/s. The watchdog refuses to start (exit 1, before its pid file
exists, so the arm script reports it did not come up) when the ceiling is not
above that cruise.

Speed is measured between the TIMESTAMPS THE LOG CARRIES, never between the
moments the lines reach this process: LidarOdometry rate-limits `lidar odom #N`
(log_every_s = 2.0) and the interval jitters around it, so a wall-clock window
(0.02 < dt < 2.0) throws away most samples - 183 of the 238 intervals of a
recorded field log - and can never strike in time. One interval above the
ceiling is a kill; no cadence is assumed anywhere.

The kill is two steps. First the exploration stop through tools/explore_ctl.py,
on the run's own bus. Then the stack itself goes down (SPEED_GUARD_STACK_STOP,
default `tools/moss-fly.sh stop`): the motor board is open loop (PWM percent,
no speed loop), so a body faster than the ceiling is not something a stopped
explorer is sure to end; with the stack down the firmware deadman stops the
tracks 300 ms after the last command.

It guards ONE run: the run dir is frozen at startup (SPEED_GUARD_RUN_DIR, which the
arm script passes, else the newest run dir) and the pid written to PID_FILE
(the launcher kills THAT pid, not whatever `pgrep speed_guard` finds). A
watchdog whose run is over stands down by itself - either a newer run dir
appeared, or the log it tails stopped growing for IDLE_S. Without this, an
orphan of the previous run keeps tailing a dead log while everything that
reads its mere presence as 'armed' lets the live run explore unguarded.

Where dimOS logs a run: an installed dimOS under <state>/dimos/logs/<run>,
a dimOS from a git clone under the clone; the run registry
(<state>/dimos/runs/*.json) names the log_dir in both cases, so both are read.
"""
import atexit, glob, json, math, os, re, shlex, signal, subprocess, sys, threading, time
from datetime import datetime

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, ROOT)
import explore_ctl  # noqa: E402  the stop rides the run's bus, same module the launcher uses
from moss_dimos import robot_profile  # noqa: E402


def limit_ms(env=None) -> float:
    """The ceiling in m/s: the profile's max_speed_mps (MOSS_MAX_SPEED_MPS overrides it)."""
    return robot_profile.current(env).max_speed_mps


PLANNER_SPEED_MPS = 0.55   # dimOS LocalPlanner._speed (replanning_a_star/local_planner.py): the follower's cruise
NERF_SPEED_DEFAULT = 0.7   # tools/moss-fly.sh: NERF_SPEED="${MOSS_NERF_SPEED:-0.7}", passed as --nerf-speed


def cruise_mps(env=None) -> float:
    """The speed the stack commands when the follower is aligned: dimOS's
    PLANNER_SPEED_MPS times the run's speed scale (MOSS_NERF_SPEED, the
    launcher's default when unset; dimOS applies it only below 1)."""
    env = os.environ if env is None else env
    try:
        nerf = float(str(env.get("MOSS_NERF_SPEED", "")).strip() or NERF_SPEED_DEFAULT)
    except ValueError:
        nerf = NERF_SPEED_DEFAULT
    return PLANNER_SPEED_MPS * (nerf if 0.0 < nerf < 1.0 else 1.0)


def ceiling_refusal(limit: float, cruise: float):
    """Why this ceiling cannot guard a run (it would stop every normal one),
    or None when it sits above the commanded cruise."""
    if limit > cruise:
        return None
    return (f"ceiling {limit:g} m/s is not above the planner's cruise {cruise:.3f} m/s "
            f"({PLANNER_SPEED_MPS:g} m/s x MOSS_NERF_SPEED): every normal run would be stopped - "
            f"raise MOSS_MAX_SPEED_MPS above {cruise:.3f} or lower MOSS_NERF_SPEED; not armed")


LIMIT_MS = limit_ms()   # hard envelope, read once per watchdog: beyond = runaway
DIMOS_STATE = os.path.join(os.environ.get("XDG_STATE_HOME") or "~/.local/state", "dimos")
LOG_GLOB = os.path.join(DIMOS_STATE, "logs", "*")          # an installed dimOS: one dir per run
RUNS_GLOB = os.path.join(DIMOS_STATE, "runs", "*.json")    # the run registry: its log_dir, wherever dimOS logs
LAUNCH_LOG = "/tmp/moss_fly_launch.log"                    # tools/moss-fly.sh's launch output, tailed too when present
IDLE_S = float(os.environ.get("SPEED_GUARD_IDLE_S", "30"))   # log silent this long -> the run is over
POLL_S = float(os.environ.get("SPEED_GUARD_POLL_S", "2"))    # how often the stand-down is re-checked
PID_FILE = os.environ.get("SPEED_GUARD_PID_FILE", "/tmp/moss_speed_guard.pid")
STACK_STOP = shlex.split(os.environ.get("SPEED_GUARD_STACK_STOP", "tools/moss-fly.sh stop"))   # run from the checkout root
STACK_STOP_TIMEOUT_S = 60.0
ODOM_RE = re.compile(r"lidar odom #(\d+): x=([+-][\d.]+) y=([+-][\d.]+)")
TS_RE = re.compile(r'"timestamp":\s*"([^"]+)"')


def run_dirs(glob_pat: str = LOG_GLOB, runs_glob: str = RUNS_GLOB) -> list:
    """The dimos run dirs, oldest mtime first: every dir under the log root,
    plus every log_dir the run registry names (a dimOS from a git clone logs
    under the clone, and only its registry entry says where)."""
    dirs = {os.path.realpath(d) for d in glob.glob(os.path.expanduser(glob_pat)) if os.path.isdir(d)}
    for entry in glob.glob(os.path.expanduser(runs_glob)):
        try:
            with open(entry) as f:
                log_dir = json.load(f).get("log_dir")
        except (OSError, ValueError, AttributeError):
            continue          # half-written or foreign file
        if isinstance(log_dir, str) and os.path.isdir(log_dir):
            dirs.add(os.path.realpath(log_dir))
    return sorted(dirs, key=_mtime)


def _mtime(path: str) -> float:
    """mtime, or -1 for a path that vanished since it was listed."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return -1.0


def newest_run(glob_pat: str = LOG_GLOB, runs_glob: str = RUNS_GLOB) -> str:
    runs = run_dirs(glob_pat, runs_glob)
    if not runs:
        sys.exit("no dimos run logs found")
    return runs[-1]


def log_of(run_dir: str) -> str:
    return os.path.join(run_dir, "main.jsonl")


def write_pid_file(path: str = PID_FILE) -> None:
    """Stamp our pid so the launcher can kill THIS watchdog by pid."""
    with open(path, "w") as f:
        f.write(f"{os.getpid()}\n")
    atexit.register(clear_pid_file, path)


def clear_pid_file(path: str = PID_FILE) -> None:
    """Drop the pid file, but only while it still names us: a newer watchdog
    that overwrote it must keep its own."""
    try:
        if int(open(path).read().strip()) == os.getpid():
            os.remove(path)
    except (OSError, ValueError):
        pass


class RunWatch:
    """Stand-down decision for the ONE run this watchdog guards: a newer run dir
    means the live run has its own watchdog and we are its orphan; a log that
    stopped growing for idle_s means the run we guard is over (or dead)."""

    def __init__(self, run_dir: str, idle_s: float = IDLE_S, glob_pat: str = LOG_GLOB,
                 clock=time.time, runs_glob: str = RUNS_GLOB):
        self.run_dir = os.path.realpath(run_dir)
        self.log_path = log_of(run_dir)
        self.idle_s = idle_s
        self.glob_pat = glob_pat
        self.runs_glob = runs_glob
        self.clock = clock
        self.size = self._size()
        self.last_growth = clock()

    def _size(self) -> int:
        try:
            return os.path.getsize(self.log_path)
        except OSError:
            return -1      # no log to grow: the idle limit will retire us

    def reason(self):
        """Why this watchdog must stand down, or None while it still guards the
        live run."""
        # our own dir always competes: a run dir the listing misses (a registry
        # entry already swept) must not make an OLDER dir look like the newest
        runs = sorted(set(run_dirs(self.glob_pat, self.runs_glob)) | {self.run_dir}, key=_mtime)
        if runs[-1] != self.run_dir:
            return (f"{os.path.basename(runs[-1])} is now the newest run, this process only tails "
                    f"{os.path.basename(self.run_dir)} - the live run is guarded by its own watchdog")
        size, now = self._size(), self.clock()
        if size != self.size:
            self.size, self.last_growth = size, now
            return None
        idle = now - self.last_growth
        if idle >= self.idle_s:
            return (f"{self.log_path} has not grown for {idle:.0f} s (limit {self.idle_s:g} s) "
                    "- the run it guards is over")
        return None


def supervise(watch: "RunWatch", tail, poll_s: float = POLL_S) -> None:
    """Kill the tail - and with it the main loop - as soon as this watchdog is
    no longer the guard of a live run."""
    while tail.poll() is None:
        why = watch.reason()
        if why:
            print(f"[speed guard] standing down: {why}", flush=True)
            tail.terminate()
            return
        time.sleep(poll_s)


def parse_odom(line: str):
    """(t seconds, x metres, y metres) from an odom log line, or None when the
    line carries no odom sample or no timestamp to date it with."""
    m = ODOM_RE.search(line)
    ts = TS_RE.search(line)
    if not m or not ts:
        return None
    try:
        t = datetime.fromisoformat(ts.group(1).replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None
    return t, float(m.group(2)), float(m.group(3))


class SpeedGuard:
    """Body speed over consecutive odom lines, dated by the log itself."""

    def __init__(self, limit: float = None):
        self.limit = LIMIT_MS if limit is None else limit
        self.prev = None  # (t, x, y) of the last dated odom line

    def feed(self, line: str):
        """Speed in m/s over the interval this line closes, or None when the
        line closes no interval (not odom, undated, or not after the previous)."""
        sample = parse_odom(line)
        if sample is None:
            return None
        t, x, y = sample
        prev, self.prev = self.prev, sample
        if prev is None or t <= prev[0]:
            return None
        return math.hypot(x - prev[1], y - prev[2]) / (t - prev[0])

    def breach(self, line: str):
        """That speed when it exceeds the envelope, else None."""
        v = self.feed(line)
        return v if v is not None and v > self.limit else None


def stop_all() -> None:
    """Exploration stop on the run's bus, then the stack down. Neither failure
    takes the watchdog down with it."""
    try:
        print(f"[speed guard] STOP EXPLORE published on {explore_ctl.publish('stop')}", flush=True)
    except Exception as e:  # a sick bus must not take the watchdog down with it
        print(f"[speed guard] STOP EXPLORE FAILED: {e}", flush=True)
    try:
        r = subprocess.run(STACK_STOP, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, timeout=STACK_STOP_TIMEOUT_S)
        last = (r.stdout or "").strip().splitlines()[-1:] or ["(no output)"]
        print(f"[speed guard] STACK STOP ({' '.join(STACK_STOP)}) exit {r.returncode}: {last[0]}", flush=True)
    except Exception as e:  # a launcher that hangs or is missing must not either
        print(f"[speed guard] STACK STOP FAILED ({' '.join(STACK_STOP)}): {e}", flush=True)


def main() -> None:
    run_dir = os.environ.get("SPEED_GUARD_RUN_DIR", "").strip() or newest_run()   # frozen: this watchdog guards THIS run
    if not os.path.isdir(run_dir):
        sys.exit(f"SPEED_GUARD_RUN_DIR={run_dir} is not a directory")
    cruise = cruise_mps()
    why = ceiling_refusal(LIMIT_MS, cruise)
    if why:
        sys.exit(f"[speed guard] {why}")   # before the pid file: the arm script sees no watchdog come up
    path = log_of(run_dir)
    write_pid_file()
    watch = RunWatch(run_dir)
    print(f"[speed guard] watching {path} (limit {LIMIT_MS} m/s, cruise {cruise:.3f} m/s commanded, "
          f"idle limit {IDLE_S:g} s, pid {os.getpid()} -> {PID_FILE})", flush=True)
    guard = SpeedGuard()
    paths = [path] + ([LAUNCH_LOG] if os.path.exists(LAUNCH_LOG) else [])
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # killed by pid: leave through atexit, pid file and all
    with subprocess.Popen(["tail", "-Fn0", *paths], stdout=subprocess.PIPE, text=True) as tail:
        threading.Thread(target=supervise, args=(watch, tail), daemon=True).start()
        try:
            for line in tail.stdout:
                if "BUMP" in line or "bump" in line:
                    print(f"[speed guard] {line.rstrip()}", flush=True)
                v = guard.breach(line)
                if v is not None:
                    print(f"[speed guard] {v:.2f} m/s over the {LIMIT_MS} m/s envelope", flush=True)
                    stop_all()
        finally:
            tail.terminate()   # or the context manager waits forever on a tail nobody killed
    print(f"[speed guard] stopped watching {path}", flush=True)


if __name__ == "__main__":
    main()
