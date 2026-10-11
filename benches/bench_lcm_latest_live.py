"""Live bench for moss_dimos.lcm_latest (needs dimOS and LCM multicast on this
machine; no robot, no motor): a slow subscriber on a command channel must
never accumulate a backlog once lcm_latest is installed.

Known in: 100 twists published in 1 s to a handler that takes 100 ms each.
Known out: with a 1-deep queue the handler sees ~10 messages and at most ONE
arrives after the publisher stopped; with dimOS's 10000-deep queue it grinds
through the backlog for seconds after the publisher has stopped.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_lcm_latest_live.py            # patched (default)
      PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_lcm_latest_live.py --stock    # dimOS default, for comparison
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
stock = "--stock" in sys.argv

OK = KO = 0


def check(cond, label):
    global OK, KO
    print(("  OK  " if cond else "  KO  ") + label)
    if cond:
        OK += 1
    else:
        KO += 1


if not stock:
    from moss_dimos.lcm_latest import capacity_for, install
    check(install() is True, "lcm_latest.install() found dimOS's LCM and patched it")
    check(capacity_for("/cmd_vel") == 1 and capacity_for("/tele_cmd_vel") == 1
          and capacity_for("/twist_command") == 1 and capacity_for("/odom") is None,
          "command channels (cmd_vel, tele_cmd_vel, twist_command) -> capacity 1; /odom untouched")

from dimos.core.transport_factory import make_transport  # noqa: E402
from dimos.msgs.geometry_msgs.Twist import Twist  # noqa: E402

TOPIC = "cmd_vel_latest_bench"           # contains "cmd_vel" -> clamped when patched
handled = []


def slow_handler(msg):
    handled.append(time.monotonic())
    time.sleep(0.1)


sub = make_transport(TOPIC, Twist)
sub.subscribe(slow_handler)
pub = make_transport(TOPIC, Twist)
time.sleep(0.5)
t0 = time.monotonic()
for i in range(100):
    pub.broadcast(None, Twist(linear=[0.1, 0, 0], angular=[0, 0, 0]))
    time.sleep(0.01)
t_stop = time.monotonic()
time.sleep(3.0)
late = [t for t in handled if t > t_stop + 0.15]
mode = "STOCK" if stock else "PATCHED"
if handled:
    print(f"  ..  {mode}: published 100 in {t_stop - t0:.2f} s; handled {len(handled)}; handled after the "
          f"publisher stopped (+150 ms): {len(late)}; last handled {max(handled) - t_stop:+.2f} s after stop")
check(len(handled) > 0, "the subscriber received twists (LCM multicast works on this machine)")
if stock:
    print(f"  ..  stock mode: {len(late)} twists executed after the publisher stopped (no pass/fail on this number)")
else:
    check(len(late) <= 1, f"at most one twist handled after the publisher stopped ({len(late)})")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
raise SystemExit(1 if KO else 0)
