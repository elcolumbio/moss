"""Cold bench: tools/moss_floor_probe.py, the static-friction floor finder.

Known input -> known output: a table of tick deltas gives a known floor; a
fake firmware whose encoders only turn from 35 % makes the probe report 35,
stop at that step, and leave the base stopped. No hardware, no dimOS.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_moss_floor_probe_cold.py
"""
import importlib.util
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos import moss_mock  # noqa: E402
from moss_dimos.moss_adapter import MossBaseAdapter, MossGeometry  # noqa: E402

spec = importlib.util.spec_from_file_location("moss_floor_probe", ROOT / "tools" / "moss_floor_probe.py")
fp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fp)

OK = KO = 0


def check(cond, label, detail=""):
    global OK, KO
    if cond:
        OK += 1
    else:
        KO += 1
    print(("  OK  " if cond else "  KO  ") + label + (f" - {detail}" if detail and not cond else ""))


print("A. plan(start, stop, step)")
check(fp.plan(20, 60, 5) == [20, 25, 30, 35, 40, 45, 50, 55, 60], "20..60 by 5 -> nine steps")
check(fp.plan(0, 200, 50) == [1, 51], "clamped to 1..100: 0..200 by 50 -> 1, 51")
check(fp.plan(60, 20, 5) == [], "start above stop -> no step")
try:
    fp.plan(20, 60, 0)
    check(False, "step 0 -> ValueError")
except ValueError:
    check(True, "step 0 -> ValueError")

print("B. judge(rows, min_ticks)")
rows = [{"pct": 20, "left": 0, "right": 0}, {"pct": 25, "left": 48, "right": 3},
        {"pct": 30, "left": 90, "right": 12}, {"pct": 35, "left": 120, "right": 95}, {"pct": 40, "left": 150, "right": 140}]
check(fp.judge(rows, 20) == 35, "left turns from 25 %, right from 35 % -> floor 35 (both sides)")
check(fp.judge(rows, 100) == 40, "a stricter motion threshold (100 ticks) -> 40")
check(fp.judge(rows[:3], 20) is None, "no row with both sides moving -> None")
check(fp.judge([{"pct": 30, "left": -60, "right": -70}], 20) == 30, "reverse ticks count too (absolute)")

print("C. the probe on a fake firmware whose encoders turn from 35 % (left from 25 %)")


class Moving(moss_mock.FakeMossFirmware):
    """The stock fake never turns its encoders; this one does, per telemetry line,
    once the applied percent is high enough: left from 25 %, right from 35 %."""

    def _telemetry_line(self):
        if self.mode == moss_mock.DRIVE:
            if abs(self.left) >= 25:
                self.ticks[0] += 60
            if abs(self.right) >= 35:
                self.ticks[1] += 60
        return super()._telemetry_line()


fake = Moving(auto_time=True)
factory = lambda: fake  # noqa: E731
factory.is_mock = True
adapter = MossBaseAdapter(port_factory=factory, geometry=MossGeometry(mps_at_limit=1.0, max_pct=100, min_pct=0))
check(adapter.connect(), "the probe's adapter connects to the fake")
said = []
rows = fp.run(adapter, [20, 25, 30, 35, 40], hold=0.6, settle=0.4, min_ticks=20, stop_at_first=True, say=said.append)
adapter.disconnect()
check([r["pct"] for r in rows] == [20, 25, 30, 35], f"stops at the first step where both move: steps driven {[r['pct'] for r in rows]}")
check(rows[-1]["left"] >= 20 and rows[-1]["right"] >= 20, f"the 35 % row shows both sides moving: {rows[-1]}")
check(rows[1]["left"] >= 20 and abs(rows[1]["right"]) < 20, f"the 25 % row shows the left side only: {rows[1]}")
check(fp.judge(rows, 20) == 35, "judge -> 35")
check(fake.mode == moss_mock.OFF, f"the base is stopped after the run (fake mode {fake.mode})")
check(any("BOTH" in s for s in said) and any("left" in s for s in said), "the table names the sides that moved")

print("D. main(): dry run, then the failure path through the stock fake")
rc = fp.main([])
check(rc == 0, "no --go -> plan only, exit 0")
os.environ["MOSS_MOCK_BUS"] = "1"
rc = fp.main(["--go", "--mock", "--from", "30", "--to", "40", "--step", "5", "--hold", "0.3", "--settle", "0.2"])
check(rc == 1, "the stock fake never turns its encoders -> 'no step turned both tracks', exit 1")
moss_mock.FakeMossFirmware, stock = Moving, moss_mock.FakeMossFirmware
try:
    rc = fp.main(["--go", "--mock", "--from", "30", "--to", "40", "--step", "5", "--hold", "0.4", "--settle", "0.3"])
    check(rc == 0, "the moving fake -> floor found, exit 0")
finally:
    moss_mock.FakeMossFirmware = stock
    os.environ.pop("MOSS_MOCK_BUS", None)
check(re.search(fp.STACK_RE, "/home/x/moss/.venv/bin/dimos --rerun-open none run moss-dimos.moss-nav --daemon") is not None,
      "the running-stack pattern matches a real launch line")
check(re.search(fp.STACK_RE, "pgrep -f [b]in/dimos( [^ ]+)* run( |$)") is None, "...and not its own pgrep")

print(f"\n{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
