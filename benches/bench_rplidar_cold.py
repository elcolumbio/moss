"""Cold bench: the C1 reader, the lidar polar->xyz maths, and the retry loop with no sensor.

Known input -> known output, in degrees, millimetres and metres:
  A.  polar_to_xy / scan_to_points with MOSS's profile (mounting yaw 0):
      73.0 deg at 1500 mm -> (0.43856, -1.43446) m; a canned revolution ->
      its known points.
  A2. what is dropped: the 0.15 m minimum range, the body box
      (0.22 x 0.18 m half extents), a masked sector set by MOSS_SCAN_MASK_DEG.
  B.  the module's retry loop driven by a FAKE device class: sensor absent
      (the sentinel port) -> retried and logged once; on the by-id port: not
      answering -> answers -> yanked mid-scan -> back; a sensor that answers
      but sends no scan (motor dead). The "RPLIDAR C1 scanning" proof line:
      once per connection, on the first published cloud, never without one.
      Plus the rpc and stream tables.
  C.  the real C1 reader (pyserial) against a port that does not exist.
  D.  the C1 sample decoder on canned bytes: a sample worth 73.0 deg at
      1.5 m, quality 15, decoded from its 5 bytes; one revolution through
      C1Lidar.iter_scans in millimetres.

Run with the robot's dimOS stack STOPPED: a live stack on the same LCM bus
starves the bench module's pub/sub and section B reads zero clouds.

Run:  PYTHONPATH=. .venv/bin/python benches/bench_rplidar_cold.py
"""
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# MOSS's own numbers, whatever the shell exported
for _var in [v for v in os.environ if v.startswith("MOSS_")]:
    del os.environ[_var]

from moss_dimos import robot_profile  # noqa: E402
from moss_dimos import rplidar_c1 as rp  # noqa: E402
from moss_dimos.rplidar_c1 import (DEFAULT_BAUDRATE, DEFAULT_PORT, RPLidarC1,  # noqa: E402
                                   polar_to_xy, scan_to_points)

OK = 0
KO = 0


def check(cond, label):
    global OK, KO
    print(("  OK  " if cond else "  KO  ") + label)
    if cond:
        OK += 1
    else:
        KO += 1


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def wait_for(predicate, timeout=6.0, tick=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(tick)
    return bool(predicate())


class LogSpy:
    """Records what the module logs (dimOS's logger is structlog, not stdlib)."""

    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **kwargs):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = _record

    def count(self, needle):
        return sum(needle in line for line in self.lines)


# --- A. polar -> xyz, known values ----------------------------------------
# The reader yields (quality, angle_deg in [0,360), distance_mm). Convention:
# theta = angle + the profile's mounting yaw (0 on MOSS), x = d*cos(theta),
# y = -d*sin(theta) (SLAMTEC heading grows clockwise, the robot frame is CCW).
print("A. polar_to_xy / scan_to_points (MOSS profile, no device)")

x, y = polar_to_xy(73.0, 1500.0)
check(close(x, 0.43856, 1e-5) and close(y, -1.43446, 1e-5),
      f"73.0 deg, 1500 mm -> (0.43856, -1.43446) m  (got ({x:.5f}, {y:.5f}))")
x, y = polar_to_xy(90.0, 2000.0)
check(close(x, 0.0) and close(y, -2.0), f"90 deg, 2000 mm -> (0.0, -2.0) m  (got ({x:.9f}, {y:.9f}))")
x, y = polar_to_xy(0.0, 1500.0)
check(close(x, 1.5) and close(y, 0.0), "0 deg, 1500 mm -> (+1.5, 0.0) m: no mounting term on MOSS")
x, y = polar_to_xy(180.0, 1000.0)
check(close(x, -1.0) and close(y, 0.0), "180 deg, 1000 mm -> (-1.0, 0.0) m")
x, y = polar_to_xy(270.0, 500.0)
check(close(x, 0.0) and close(y, 0.5), "270 deg, 500 mm -> (0.0, +0.5) m")
x, y = polar_to_xy(45.0, 1414.213562)
check(close(x, 1.0) and close(y, -1.0), "45 deg, 1414.2136 mm -> (+1.0, -1.0) m")
x, y = polar_to_xy(30.0, 4000.0)
check(close(x, 3.464101615) and close(y, -2.0), "30 deg, 4000 mm -> (+3.4641016, -2.0) m")
x, y = polar_to_xy(87.25, 2000.0, yaw_offset_deg=2.75)
check(close(x, 0.0) and close(y, -2.0), "an explicit 2.75 deg mounting yaw: 87.25 deg raw -> theta 90 -> (0.0, -2.0) m")
os.environ["MOSS_LIDAR_YAW_DEG"] = "2.75"
x, y = polar_to_xy(87.25, 2000.0)
check(close(x, 0.0) and close(y, -2.0), "MOSS_LIDAR_YAW_DEG=2.75 is read at call time: 87.25 deg raw -> (0.0, -2.0) m")
del os.environ["MOSS_LIDAR_YAW_DEG"]
x, y = polar_to_xy(90.0, 2000.0)
check(close(x, 0.0) and close(y, -2.0), "override removed -> back to yaw 0")

# One canned revolution: 3 keepers, one weak return, one invalid (distance 0).
SCAN = [(15, 0.0, 1000.0),      # kept  -> (+1.0,  0.0)
        (15, 90.0, 2000.0),     # kept  -> ( 0.0, -2.0)
        (5, 180.0, 3000.0),     # dropped at min_quality 10: quality 5
        (20, 270.0, 500.0),     # kept  -> ( 0.0, +0.5)
        (18, 45.0, 0.0)]        # dropped: distance 0 = invalid measure
EXPECTED = [(1.0, 0.0, 0.0), (0.0, -2.0, 0.0), (0.0, 0.5, 0.0)]
EXPECTED_ALL = [(1.0, 0.0, 0.0), (0.0, -2.0, 0.0), (-3.0, 0.0, 0.0), (0.0, 0.5, 0.0)]   # + the weak (quality 5) return, kept by the module default

points = scan_to_points(SCAN, min_quality=10)
check(len(points) == 3, f"min_quality=10 keeps 3 of 5 measures (got {len(points)})")
check(all(close(g, e) for gp_, ep in zip(points, EXPECTED)
          for g, e in zip(gp_, ep)),
      f"points = {[tuple(round(v, 3) for v in p) for p in points]} m")
check(all(p[2] == 0.0 for p in points), "every point is flat: z = 0.0")
# At min_quality=16 only the quality-20 measure survives: the quality-18 one
# is the invalid (distance 0) return, dropped on distance regardless.
check(len(scan_to_points(SCAN, min_quality=16)) == 1,
      "raising min_quality to 16 keeps only the quality-20 return")
check(scan_to_points([], 10) == [], "an empty scan yields no points")


# --- A2. what is dropped: minimum range, body box, masked sectors ------------
print("\nA2. minimum range, body box, masked sectors")
M = robot_profile.MOSS
check(M.scan_min_range_m == 0.15 and M.body_box_m == (0.22, 0.18) and M.scan_mask_sectors_deg == (),
      "MOSS profile: 0.15 m minimum range, 0.22 x 0.18 m body box, no masked sector")
scan = [(40, 37.0, 120.0),                       # 0.12 m: under the 0.15 m minimum range: dropped
        (40, 0.0, 200.0),                        # (+0.20, 0): inside the body box: dropped
        (40, 90.0, 170.0),                       # (0, -0.17): inside the body box: dropped
        (40, 0.0, 250.0),                        # (+0.25, 0): outside the box: kept
        (40, 90.0, 190.0),                       # (0, -0.19): just outside |y| 0.18: kept
        (40, 20.0, 450.0),                       # 0.45 m: kept
        (40, 5.0, 800.0),                        # 0.80 m: kept
        (40, 180.0, 600.0),                      # (-0.60, 0): kept
        (3, 60.0, 500.0), (40, 60.0, 0.0)]       # weak / invalid: dropped
pts = scan_to_points(scan, min_quality=10)
check(len(pts) == 5, f"minimum range and body box drop 3, weak/invalid drop 2, 5 real points kept ({len(pts)})")
check(any(close(p[0], 0.25) and close(p[1], 0.0) for p in pts), "the 0.25 m return dead ahead survives at (+0.25, 0)")
check(any(close(p[0], 0.0) and close(p[1], -0.19) for p in pts), "the 0.19 m return at 90 deg survives at (0, -0.19)")
os.environ["MOSS_SCAN_MASK_DEG"] = "348 12"
pts_m = scan_to_points(scan, min_quality=10)
check(len(pts_m) == 4 and not any(close(p[0], 0.25) and close(p[1], 0.0) for p in pts_m),
      f"MOSS_SCAN_MASK_DEG='348 12' (under 0.30 m) drops the 0.25 m return ahead, keeps the 0.80 m one at 5 deg ({len(pts_m)} kept)")
del os.environ["MOSS_SCAN_MASK_DEG"]
os.environ["MOSS_BODY_BOX_M"] = "0.30 0.18"
check(len(scan_to_points(scan, min_quality=10)) == 4, "MOSS_BODY_BOX_M='0.30 0.18' swallows the 0.25 m return too (4 kept)")
del os.environ["MOSS_BODY_BOX_M"]
check(len(scan_to_points(scan, 10, mask=((348.0, 12.0),), mask_range_m=0.30)) == 4,
      "an explicit mask argument wins over the profile's (none)")
wide = robot_profile.moss_profile({"MOSS_BODY_BOX_M": "0.30 0.30"})
check(len(scan_to_points(scan, 10, profile=wide)) == 3,
      "an explicit profile argument wins over the environment (0.30 x 0.30 box: 3 kept)")


# --- B. retry loop on a fake device class ---------------------------------
print("\nB. retry loop (fake device, no sensor)")
check({"start", "stop"} <= set(RPLidarC1.rpcs), "RPLidarC1.rpcs carries start and stop (@rpc kept on the overrides)")
from dimos.core.coordination.blueprints import BlueprintAtom  # noqa: E402
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: E402

streams = {(s.name, s.type): s.direction for s in BlueprintAtom.create(RPLidarC1, {}).streams}
check(streams == {("pointcloud", PointCloud2): "out"}, f"one stream: pointcloud Out[PointCloud2] ({streams})")
check(RPLidarC1.name == "rplidarc1", f"dimOS instance name {RPLidarC1.name!r}")


class FakeLidarError(Exception):
    """Stand-in for a reader error."""


class Bench:
    """What the test flips to move the sensor in and out."""

    def __init__(self):
        self.present = False        # is the port openable?
        self.unplugged = False      # yanked mid-scan?
        self.mute = False           # answers on its port, but no scan ever comes (motor dead)
        self.opens = []             # (port, baudrate) per attempt
        self.shutdowns = 0


bench = Bench()


class FakeLidar:
    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        bench.opens.append((port, baudrate))
        if not bench.present or not os.path.exists(port):
            raise FakeLidarError(
                "Failed to connect to the sensor due to: [Errno 2] "
                "could not open port %s" % port)

    def get_info(self):
        return {"model": 65, "firmware": (1, 1)}

    def iter_scans(self):
        if bench.mute:
            raise FakeLidarError("scan stream timed out")
        while True:
            if bench.unplugged:
                raise FakeLidarError("device reports readiness to read but "
                                     "returned no data (port closed?)")
            yield list(SCAN)
            time.sleep(0.05)

    def _dead_port(self):
        bench.shutdowns += 1
        if bench.unplugged:  # a yanked USB dongle fails every close step
            raise FakeLidarError("port is gone")

    stop = stop_motor = disconnect = _dead_port


byid = tempfile.mkdtemp(prefix="byid_cold_")      # an empty /dev/serial/by-id
saved_root = robot_profile.BYID_ROOT
robot_profile.BYID_ROOT = byid
rp.RPLidarC1.lidar_class = FakeLidar   # the bench drives a fake device class
spy = LogSpy()
real_logger, rp.logger = rp.logger, spy

lidar_module = RPLidarC1(retry_period_s=0.5)
clouds = []
lidar_module.pointcloud.subscribe(clouds.append)
check(DEFAULT_PORT == robot_profile.MOSS_LIDAR_MISSING == "/dev/serial/by-id/MISSING-C1",
      f"DEFAULT_PORT is the sentinel {DEFAULT_PORT}")
check(lidar_module.port == DEFAULT_PORT and lidar_module.baudrate == 460800,
      f"no port given, nothing plugged -> {lidar_module.port} at 460800 baud")
check(DEFAULT_BAUDRATE == 460800, "DEFAULT_BAUDRATE is the C1 line rate")
os.environ["MOSS_LIDAR_PORT"] = "/dev/ttyUSB7"
check(RPLidarC1(retry_period_s=0.5).port == "/dev/ttyUSB7", "MOSS_LIDAR_PORT is read when the module is built")
del os.environ["MOSS_LIDAR_PORT"]
check(RPLidarC1(port="/dev/ttyUSB3").port == "/dev/ttyUSB3", "an explicit port wins")
lidar_module.start()

# 1. sensor absent at startup: retries, logs once, never dies.
check(wait_for(lambda: len(bench.opens) >= 3, 4.0),
      f"absent sensor -> keeps retrying ({len(bench.opens)} attempts)")
check(spy.count("RPLIDAR C1 unavailable") == 1,
      "the failure is logged ONCE, not once per retry")
check(spy.count(f"RPLIDAR C1 unavailable on {robot_profile.MOSS_LIDAR_MISSING}") == 1,
      "the log names the sentinel port")
check(lidar_module._thread.is_alive() and not clouds,
      "loop alive, nothing published while the sensor is missing")
check(lidar_module._thread.name == "rplidar-c1", f"thread name {lidar_module._thread.name!r}")
check(all(open_args == (DEFAULT_PORT, 460800) for open_args in bench.opens),
      "every attempt opens the sentinel at 460800 baud")

lidar_module.stop()
check(not lidar_module._thread.is_alive(), "a module on the sentinel stops cleanly")

# 2. a module built with the by-id port; the sensor appears later on it.
plugged = os.path.join(byid, "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0000-if00-port0")
open(plugged, "w").close()
check(robot_profile.moss_lidar_port() == plugged, "the CP2102N by-id link resolves as the lidar port")
bench.opens.clear()
lidar_module = RPLidarC1(retry_period_s=0.5)
lidar_module.pointcloud.subscribe(clouds.append)
check(lidar_module.port == plugged, f"no port given -> the by-id link ({os.path.basename(lidar_module.port)})")
spy.lines.clear()
lidar_module.start()
check(wait_for(lambda: len(bench.opens) >= 2, 3.0) and spy.count("RPLIDAR C1 unavailable") == 1,
      "the sensor not answering yet -> retried, logged once")
bench.present = True
check(wait_for(lambda: len(clouds) >= 2, 4.0),
      f"sensor answers -> publishes clouds ({len(clouds)} so far)")
check(spy.count(f"RPLIDAR C1 up on {plugged} @ 460800 baud") == 1,
      "logs the sensor coming up, with its port and baud rate")
check(wait_for(lambda: len(clouds) >= 4, 4.0)
      and spy.count("RPLIDAR C1 scanning: 4 points in the first turn") == 1
      and spy.count("RPLIDAR C1 scanning") == 1,
      f"'RPLIDAR C1 scanning: 4 points in the first turn' logged ONCE, not once per cloud ({len(clouds)} clouds)")
_up_at = next(i for i, line in enumerate(spy.lines) if line.startswith("RPLIDAR C1 up"))
_scan_at = next(i for i, line in enumerate(spy.lines) if line.startswith("RPLIDAR C1 scanning"))
check(_up_at < _scan_at, "the scanning line comes after the 'up' line (answers first, then spins)")
check(rp.SCANNING_LINE == "RPLIDAR C1 scanning: %d points in the first turn",
      "the proof line the launcher greps keeps its words")
xyz = clouds[0].as_numpy()[0]
# the module's default is min_quality=0: every measure with a distance is
# kept (the weak return included), only the distance-0 one is dropped
check(xyz.shape == (4, 3), f"cloud carries the 4 measures that have a distance {xyz.shape}")
check(all(close(g, e) for row, exp in zip(xyz, EXPECTED_ALL)
          for g, e in zip(row, exp)),
      f"cloud points = {[tuple(round(float(v), 3) for v in r) for r in xyz]} m")
check(clouds[0].frame_id == "lidar_link", "frame_id survives ModuleConfig")

# 3. yanked mid-scan: logged, closing steps all fail, loop keeps going.
published_before = len(clouds)
shutdowns_before = bench.shutdowns
opens_before = len(bench.opens)
bench.unplugged = True
check(wait_for(lambda: spy.count("returned no data") >= 1, 3.0),
      "unplugged mid-scan -> the reader error is logged")
check(wait_for(lambda: bench.shutdowns > shutdowns_before, 2.0),
      "the close sequence runs even though the port is gone")
check(lidar_module._thread.is_alive(), "loop survives the unplug")
check(wait_for(lambda: len(bench.opens) > opens_before, 3.0)
      and len(clouds) == published_before,
      f"re-opens the port ({opens_before} -> {len(bench.opens)}), no cloud published while unplugged")
check(all(p == plugged for p, _ in bench.opens), "every attempt opens the by-id port")

# 4. plugged back in: publishing resumes.
resumed_at = len(clouds)
bench.unplugged = False
check(wait_for(lambda: len(clouds) > resumed_at, 4.0),
      "sensor back -> publishing resumes")
check(wait_for(lambda: spy.count("RPLIDAR C1 scanning") == 2, 2.0),
      f"a new connection logs its first scan again: 2 scanning lines for 2 connections "
      f"({spy.count('RPLIDAR C1 scanning')})")

t0 = time.monotonic()
lidar_module.stop()
stop_s = time.monotonic() - t0
check(not lidar_module._thread.is_alive() and stop_s < 3.0,
      f"stop() joins the loop in {stop_s:.2f} s (retry wait is interruptible)")

# 5. a C1 that answers on its port but never sends a scan (motor dead): "up",
#    no cloud, and never the scanning line - the launcher must not prove it.
bench.mute = True
spy.lines.clear()
clouds_before = len(clouds)
bench.opens.clear()
lidar_module = RPLidarC1(retry_period_s=0.2)
lidar_module.pointcloud.subscribe(clouds.append)
lidar_module.start()
check(wait_for(lambda: len(bench.opens) >= 3, 3.0) and spy.count("RPLIDAR C1 up") >= 2,
      f"motor dead: the C1 answers ('up' x{spy.count('RPLIDAR C1 up')}), its scan stream times out, retried")
check(spy.count("RPLIDAR C1 unavailable") >= 1 and spy.count("scan stream timed out") >= 1,
      "the log names the cause (scan stream timed out)")
check(spy.count("RPLIDAR C1 scanning") == 0 and len(clouds) == clouds_before,
      "no cloud and NO scanning line: 'up' alone never proves a lidar that does not spin")
lidar_module.stop()
check(not lidar_module._thread.is_alive(), "stops cleanly")
bench.mute = False

rp.logger = real_logger
rp.RPLidarC1.lidar_class = None
robot_profile.BYID_ROOT = saved_root
shutil.rmtree(byid, ignore_errors=True)


# --- C. the real reader, against a port that does not exist ---------------
print("\nC. real C1 reader (pyserial) on a missing port")
spy = LogSpy()
real_logger, rp.logger = rp.logger, spy
absent = RPLidarC1(port="/dev/ttyUSB_moss_cold_test", retry_period_s=0.5)
absent.start()
check(wait_for(lambda: spy.count("RPLIDAR C1 unavailable") == 1, 4.0),
      "missing device -> the open error is caught and logged once")
check(any("No such file or directory" in line for line in spy.lines),
      "the log names the real cause")
check(absent._thread.is_alive(), "module still running with no sensor")
t0 = time.monotonic()
absent.stop()
check(not absent._thread.is_alive() and time.monotonic() - t0 < 3.0,
      "clean stop with the sensor never present")
rp.logger = real_logger


# --- D. the C1 sample decoder on canned bytes -----------------------------
print("\nD. c1_serial: 5-byte samples -> degrees and metres")
from moss_dimos import c1_serial  # noqa: E402


class FakeSerial:
    """Serves canned bytes; records what was written."""

    def __init__(self, data: bytes):
        self.data = data
        self.written = b""

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, n):
        out, self.data = self.data[:n], self.data[n:]
        return out

    def write(self, b):
        self.written += b

    def reset_input_buffer(self):
        pass


def sample(start: bool, quality: int, angle_deg: float, dist_m: float) -> bytes:
    q6 = int(round(angle_deg * 64))
    d4 = int(round(dist_m * 1000 * 4))
    b0 = (quality << 2) | (0b01 if start else 0b10)
    return bytes([b0, ((q6 & 0x7F) << 1) | 1, q6 >> 7, d4 & 0xFF, d4 >> 8])


check(sample(True, 15, 73.0, 1.5) == bytes([0x3D, 0x81, 0x24, 0x70, 0x17]),
      "73.0 deg, 1.5 m, quality 15, new turn = 3D 81 24 70 17")
sc = c1_serial.C1Scanner("/dev/null")
sc._ser = FakeSerial(c1_serial.SCAN_DESCRIPTOR + b"\x00" + bytes([0x3D, 0x81, 0x24, 0x70, 0x17])
                     + sample(False, 20, 180.5, 0.0) + sample(False, 40, 359.0, 11.25))
gen = sc.samples()
s1, s2, s3 = next(gen), next(gen), next(gen)
check(sc._ser.written == c1_serial.SCAN, "the scan is requested with A5 20")
check(s1 == (True, 15, 73.0, 1.5), f"one lost-sync byte skipped, then (new turn, q 15, 73.0 deg, 1.5 m) ({s1})")
check(s2 == (False, 20, 180.5, 0.0), f"(same turn, q 20, 180.5 deg, 0.0 m = no return) ({s2})")
check(s3 == (False, 40, 359.0, 11.25), f"(same turn, q 40, 359.0 deg, 11.25 m) ({s3})")

rev = [sample(True, 15, 0.0, 1.0), sample(False, 15, 120.0, 2.0), sample(False, 15, 240.0, 3.0),
       sample(True, 15, 0.5, 1.0)]
lid = c1_serial.C1Lidar.__new__(c1_serial.C1Lidar)
lid._s = c1_serial.C1Scanner("/dev/null")
lid._s._ser = FakeSerial(c1_serial.SCAN_DESCRIPTOR + b"".join(rev))
first = next(lid.iter_scans(min_len=3))
check(first == [(15, 0.0, 1000.0), (15, 120.0, 2000.0), (15, 240.0, 3000.0)],
      f"C1Lidar.iter_scans: one revolution of 3 samples, distances in mm ({first})")
check(scan_to_points(first, 0) and close(scan_to_points(first, 0)[0][0], 1.0),
      "and that revolution feeds scan_to_points: (1.0, 0, 0) m first")
bad = c1_serial.C1Scanner("/dev/null")
bad._ser = FakeSerial(b"\xa5\x5a\x05\x00\x00\x40\x82")
try:
    next(bad.samples())
    check(False, "a wrong scan descriptor is refused")
except RuntimeError as exc:
    check("unexpected scan descriptor" in str(exc), f"a wrong scan descriptor is refused ({exc})")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
raise SystemExit(1 if KO else 0)
