"""Cold bench for the relocalizer spectator in moss-explore (dimOS main only).

Known input -> known output in physical units. Groups:

  A. knob off    - MOSS_RELOC_MAP unset: moss-explore is the blueprint it was,
                   no relocalizer atom, and importing/building it never imports
                   dimos.mapping.relocalization (the pin a7d19f76 lacks it)
  B. knob on     - MOSS_RELOC_MAP=<premap>: one LidarWindowRelocalization atom
                   with map_file = that path; autoconnect puts LidarOdometry's
                   `lidar` Out and the relocalizer's `lidar` In on one stream,
                   the relocalizer's `tf` Out on the same `tf` stream as the
                   odometry's; its `loaded_map` and `tf` reach the costmap
                   only (MossCostMap(premap=False): ignored unless
                   MOSS_PREMAP=1), the odometry reads nothing back (open
                   loop); dimOS's stream type check accepts the build, with
                   MOSS_PREMAP=1 too; dimOS's `--map-file` and
                   `--republish-loaded-map` CLI options reach the relocalizer
  C. the handler - the module built for real (map_file = the courseB premap),
                   its tf / loaded_map Outs captured; recorded `lidar` clouds of
                   the same run windowed exactly as the module's window() does
                   (deque of max_frames, one window every reloc_interval s of
                   recording time, min_frames, raw sum), fed to the module's own
                   _relocalize(); the first accepted fix is published as ONE
                   world -> map transform on tf and the premap on loaded_map
                   (frame map), then the module stops (relocalize_once). The
                   premap is built from this same run, so the truth is the
                   identity: the fix must sit within 0.5 m and 10 degrees of it.

B and C need dimOS main (dimos.mapping.relocalization) and open3d: on the
pinned dimOS they are skipped, said in the log. C needs a premap and the
recording it was built from, both under MOSS_RELOC_BENCH_DATA=<dir>:
courseB_explorer2.pc2.lcm and rec/courseB_explorer2.db; without them C is
skipped, said in the log. No hardware, no network.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. <venv>/bin/python benches/bench_moss_reloc_cold.py
"""
from collections import defaultdict, deque
from functools import reduce
import math
import operator
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

_data = os.environ.get("MOSS_RELOC_BENCH_DATA", "").strip()
DATA = Path(_data) if _data else None
PREMAP = (DATA or Path("/maps")) / "courseB_explorer2.pc2.lcm"   # dimos map global --export of courseB, 5,294 points
RECORDING = DATA / "rec" / "courseB_explorer2.db" if DATA else None   # the same run: lidar (world frame) + odom
HAVE_DATA = PREMAP.is_file() and RECORDING is not None and RECORDING.is_file()
MAX_WINDOWS = 250                                     # courseB is 504 s: about 250 windows at 2 s

OK = KO = 0


def check(label, ok, detail=""):
    global OK, KO
    if ok:
        OK += 1
    else:
        KO += 1
    print(f"  {'OK' if ok else 'KO'}  {label}" + (f" - {detail}" if detail else ""), flush=True)


def finish():
    print(f"{OK} OK, {KO} KO")
    print("TEST PASSED" if KO == 0 else "TEST FAILED")
    sys.exit(0 if KO == 0 else 1)


os.environ.pop("MOSS_RELOC_MAP", None)
import moss_dimos.blueprints as mb  # noqa: E402
import moss_dimos.memory as mb_memory  # noqa: E402
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: E402
from dimos.msgs.tf2_msgs.TFMessage import TFMessage  # noqa: E402


def names(bp):
    return sorted(b.module.__name__ for b in bp.active_blueprints)


def wiring(bp):
    """{(stream name after remaps, type): {(module class name, direction)}}, as the blueprint bench."""
    out = defaultdict(set)
    for b in bp.active_blueprints:
        for conn in b.streams:
            name = bp.remapping_map.get((b.name, conn.name), conn.name)
            out[(name, conn.type)].add((b.module.__name__, conn.direction))
    return out


print("A. knob off")
proc = subprocess.run([sys.executable, "-c",
                       "import sys, moss_dimos.blueprints as mb; mb.build_moss_explore(); "
                       "print([m for m in sys.modules if m.startswith('dimos.mapping.relocalization')])"],
                      cwd=ROOT, capture_output=True, text=True, timeout=300,
                      env={**{k: v for k, v in os.environ.items() if k != "MOSS_RELOC_MAP"},
                           "PYTHONPATH": os.pathsep.join(filter(None, (str(ROOT), os.environ.get("PYTHONPATH")))), "PYTHONDONTWRITEBYTECODE": "1"})
last = (proc.stdout.strip().splitlines() or [""])[-1]
check("building moss-explore without MOSS_RELOC_MAP imports no dimos.mapping.relocalization module",
      proc.returncode == 0 and last == "[]", f"rc {proc.returncode}, {last or proc.stderr[-300:]}")
off = mb.build_moss_explore()
check("    ... and has no relocalizer atom", "LidarWindowRelocalization" not in names(off), f"{names(off)}")

import importlib.util  # noqa: E402
if importlib.util.find_spec("dimos.mapping.relocalization.lidar") is None:
    print("B., C. SKIPPED: this dimOS has no dimos.mapping.relocalization.lidar (the pin a7d19f76); MOSS_RELOC_MAP needs dimOS main")
    finish()
print("B. knob on")
os.environ["MOSS_RELOC_MAP"] = str(PREMAP)
on = mb.build_moss_explore()
os.environ.pop("MOSS_RELOC_MAP")
added = sorted(set(names(on)) - set(names(off)))
check("MOSS_RELOC_MAP=<premap>: exactly one atom more, LidarWindowRelocalization",
      added == ["LidarWindowRelocalization"] and len(names(on)) == len(names(off)) + 1, f"{added}")
reloc = next(b for b in on.active_blueprints if b.module.__name__ == "LidarWindowRelocalization")
check("    ... map_file = the premap path, every other knob dimOS's default (world_frame world, map_frame map, MID360)",
      reloc.kwargs == {"map_file": str(PREMAP)}, f"{reloc.kwargs}")
w = wiring(on)
lidar = w[("lidar", PointCloud2)]
check("autoconnect: LidarOdometry's `lidar` Out and the relocalizer's `lidar` In are one stream",
      ("LidarOdometry", "out") in lidar and ("LidarWindowRelocalization", "in") in lidar, f"{sorted(lidar)}")
tf = w[("tf", TFMessage)]
check("    ... the relocalizer's `tf` Out joins the odometry's `tf` stream (world -> map beside world -> base_link)",
      ("LidarWindowRelocalization", "out") in tf and ("LidarOdometry", "out") in tf, f"{sorted(tf)}")
lm = w[("loaded_map", PointCloud2)]
cm_kw = next(b for b in on.active_blueprints if b.module.__name__ == "MossCostMap").kwargs
check("    ... `loaded_map` is published by the relocalizer and read by the costmap only, which ignores it "
      "(MossCostMap(premap=False), MOSS_PREMAP unset)",
      lm == {("LidarWindowRelocalization", "out"), ("MossCostMap", "in")} and cm_kw == {"premap": False},
      f"{sorted(lm)} {cm_kw}")
ins = sorted(c.name for c in reloc.streams if c.direction == "in")
check("    ... the relocalizer reads `lidar` only", ins == ["lidar"], f"{ins}")
readers_tf = sorted(m for (m, d) in tf if d == "in")
rec = next(b for b in on.active_blueprints if b.module.__name__ == "MossRecorder")
check("open loop: the `tf` readers are the costmap (a world -> map fix only, with MOSS_PREMAP=1) and dimOS's "
      "Recorder base under MossRecorder, which records no tf (record_tf False) and anchors `lidar` on odom; "
      "the odometry reads nothing back",
      readers_tf == ["MossCostMap", "MossRecorder"]
      and rec.kwargs.get("record_tf", mb_memory.MossMemoryConfig.model_fields["record_tf"].default) is False
      and not any(c.direction == "in" and c.name in ("tf", "loaded_map")
                  for b in on.active_blueprints if b.module.__name__ == "LidarOdometry" for c in b.streams),
      f"{readers_tf}")
from dimos.core.coordination.module_coordinator import _verify_no_name_conflicts  # noqa: E402
os.environ.update(MOSS_RELOC_MAP=str(PREMAP), MOSS_PREMAP="1")
both = mb.build_moss_explore()
for v in ("MOSS_RELOC_MAP", "MOSS_PREMAP"):
    os.environ.pop(v)
lm_both = wiring(both)[("loaded_map", PointCloud2)]
check("MOSS_RELOC_MAP and MOSS_PREMAP=1: MossCostMap(premap=True) reads the relocalizer's `loaded_map`, one stream",
      next(b for b in both.active_blueprints if b.module.__name__ == "MossCostMap").kwargs == {"premap": True}
      and lm_both == {("LidarWindowRelocalization", "out"), ("MossCostMap", "in")}, f"{sorted(lm_both)}")
try:
    _verify_no_name_conflicts(on)
    _verify_no_name_conflicts(both)
    check("dimOS's stream type check accepts both builds (MOSS_PREMAP unset and 1)", True)
except ValueError as exc:
    check("dimOS's stream type check accepts both builds (MOSS_PREMAP unset and 1)", False, str(exc)[:300])
try:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    parsed = BlueprintConfigParser(on).parse(["--map-file", "/elsewhere/other.pc2.lcm", "--republish-loaded-map", "10"],
                                             environ={})
    got = parsed.module_kwargs(reloc.name)
    check("dimOS's CLI: `dimos run moss-dimos.moss-explore --map-file X --republish-loaded-map 10` sets the "
          "relocalizer's map_file and its loaded_map resend period (s)",
          got.get("map_file") == "/elsewhere/other.pc2.lcm" and got.get("republish_loaded_map") == 10.0, f"{got}")
except Exception as e:  # noqa: BLE001
    check("dimOS's CLI: `--map-file X --republish-loaded-map 10` reach the relocalizer", False,
          f"{type(e).__name__}: {str(e)[:300]}")

if not HAVE_DATA:
    print("C. SKIPPED: no premap and recording here (MOSS_RELOC_BENCH_DATA=<dir> with courseB_explorer2.pc2.lcm "
          "and rec/courseB_explorer2.db)")
    finish()
print("C. the module's handler on the recording")
import numpy as np  # noqa: E402
from reactivex import Subject  # noqa: E402
from dimos.mapping.relocalization.lidar.module import LidarWindowRelocalization  # noqa: E402
from dimos.mapping.relocalization.module import yaw_deg  # noqa: E402
from dimos.memory.store.sqlite import SqliteStore  # noqa: E402

m = LidarWindowRelocalization(map_file=str(PREMAP))
tf_out, map_out = [], []
m.tf = SimpleNamespace(publish=tf_out.append)            # the two Outs, captured
m.loaded_map = SimpleNamespace(publish=map_out.append)
m.lidar = SimpleNamespace(observable=lambda: Subject())   # the In: windows go to the handler below
m.start()
cfg, rc = m.config, m.config.relocalize
check("the module loads the premap in frame `map` and builds its relocalizer",
      m.premap is not None and m.premap.frame_id == "map" and m._relocalizer is not None,
      f"{len(m.premap) if m.premap is not None else None} points")
print(f"      window {rc.min_frames}..{rc.max_frames} scans, one per {cfg.reloc_interval} s, "
      f"min_local_points {cfg.min_local_points}, fitness_threshold {rc.fitness_threshold}")

store = SqliteStore(path=str(RECORDING), must_exist=True)
store.start()
held = deque(maxlen=rc.max_frames)
last_emit = t_first = None
attempts = skipped = 0
for obs in store.stream("lidar", PointCloud2).order_by("ts"):
    t_first = obs.ts if t_first is None else t_first
    held.append(obs.data)
    if last_emit is not None and obs.ts - last_emit < cfg.reloc_interval:
        continue
    last_emit = obs.ts
    if len(held) < rc.min_frames or not m.keep_relocalizing():
        if not m.keep_relocalizing():
            break
        continue
    msg = reduce(operator.add, held)
    if len(msg) < cfg.min_local_points:
        skipped += 1
        continue
    attempts += 1
    m._relocalize(msg)                                    # the module's own handler
    if tf_out or attempts >= MAX_WINDOWS:
        t_fix = obs.ts - t_first
        break
store.stop()
check(f"at least one window accepted ({attempts} attempts, {skipped} skipped under min_local_points)",
      bool(tf_out), f"first fix at {t_fix:.1f} s of recording" if tf_out else "no fix")
if tf_out:
    msg = tf_out[0]
    check("    ... published as a TFMessage with ONE transform world -> map",
          isinstance(msg, TFMessage) and len(msg.transforms) == 1
          and (msg.transforms[0].frame_id, msg.transforms[0].child_frame_id) == ("world", "map"),
          f"{[(t.frame_id, t.child_frame_id) for t in getattr(msg, 'transforms', [])]}")
    fix = msg.transforms[0]
    x, y, z, yaw = fix.translation.x, fix.translation.y, fix.translation.z, yaw_deg(fix)
    dist = math.hypot(x, y)
    print(f"      fix: translation ({x:+.3f}, {y:+.3f}, {z:+.3f}) m, yaw {yaw:+.2f} deg")
    check("    ... the premap is this run's own map: the fix is the identity within 0.5 m and 10 deg",
          dist < 0.5 and abs(yaw) < 10.0 and abs(z) < 0.5, f"{dist * 100:.1f} cm, {yaw:+.2f} deg")
    check("    ... the premap goes out once on loaded_map, frame map",
          len(map_out) == 1 and map_out[0].frame_id == "map", f"{len(map_out)}")
    check("    ... and the module stops attempting (relocalize_once)", m.placed and not m.keep_relocalizing())
m.dispose()
finish()
