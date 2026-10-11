"""Cold bench: velocity commands are consumed latest-only, on any transport.

lcm_latest clamps the LCM queue of command channels to one message, but under
TRANSPORT=zenoh the commands ride ZenohPubSub, which has no queue bound and no
capacity knob to set (section D checks that this is still true of the
installed dimOS). A coordinator that falls behind then executes the whole
backlog of stale twists, and the rover drives far past what it was asked.
The fix is consumer side: moss_dimos.latest_only.

Sections:
  A. the wrapper alone - 100 twists burst in while the consumer is busy, ONE is
     executed (the newest), 99 dropped. Known values in m/s.
  B. the production path - the callback MossControlCoordinator subscribes for
     twist_command, fed the same burst: the base joint velocities actually
     commanded are the newest twist's, once. Other streams stay unwrapped.
  C. without the wrapper - the same burst through dimOS's raw callback: all
     101 execute, the newest one last, behind 100 stale commands.
  D. dimOS's zenoh subscriber has no queue capacity to set (why the fix is a
     consumer-side wrapper and not a transport setting).

Needs dimOS. starlette/uvicorn are stubbed only when they are not installed
(dimOS's viewer bridge imports them; nothing here serves HTTP).

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_latest_only_cold.py
"""
import importlib.util
import inspect
import sys
import threading
import time
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

for _name in ("starlette", "starlette.applications", "starlette.responses",
              "starlette.routing", "uvicorn"):
    if importlib.util.find_spec(_name.split(".")[0]) is not None:
        continue
    _stub = types.ModuleType(_name)
    _stub.__getattr__ = lambda attr, _n=_name: type(attr, (), {})  # type: ignore[method-assign]
    sys.modules.setdefault(_name, _stub)

from dimos.control.components import HardwareType  # noqa: E402
from dimos.control.coordinator import ControlCoordinator  # noqa: E402
from dimos.control.routing import Routing  # noqa: E402
from dimos.msgs.geometry_msgs.Twist import Twist  # noqa: E402

from moss_dimos.blueprints import MossControlCoordinator  # noqa: E402
from moss_dimos.latest_only import LatestOnly  # noqa: E402

OK = KO = 0


def check(cond, label):
    global OK, KO
    print(("  OK  " if cond else "  KO  ") + label)
    if cond:
        OK += 1
    else:
        KO += 1


def wait_for(predicate, timeout=3.0, tick=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(tick)
    return bool(predicate())


# The rover's own numbers: a stop, then a burst that accelerates to 1.00 m/s
# while turning at 0.50 rad/s. Only the last of the burst may ever be executed.
HOLD = 0.0
BURST = [round(0.01 * i, 4) for i in range(1, 101)]   # 0.01 .. 1.00 m/s
LAST = BURST[-1]                                      # 1.00 m/s


class Gate:
    """Parks the consumer inside its FIRST message until released, so the burst
    lands on a consumer that is provably busy - no sleeps, no thresholds."""

    def __init__(self):
        self.seen = []
        self.entered = threading.Event()
        self.release = threading.Event()

    def consume(self, value):
        self.seen.append(value)
        if len(self.seen) == 1:
            self.entered.set()
            self.release.wait(5.0)


print("A. the wrapper alone: 100 twists in, the newest one out")
gate = Gate()
latest = LatestOnly(gate.consume, name="bench")
latest(HOLD)
check(gate.entered.wait(2.0), "the consumer is parked inside the first command")
for v in BURST:
    latest(v)
check(len(gate.seen) == 1, f"nothing executed while the consumer is busy (executed {len(gate.seen)})")
check(latest.received == 101, f"101 commands received ({latest.received})")
check(latest.dropped == 99, f"99 of the 100 burst commands dropped ({latest.dropped})")
gate.release.set()
check(wait_for(lambda: len(gate.seen) == 2), "the consumer wakes on the pending command")
check(gate.seen == [HOLD, LAST],
      f"executed {gate.seen} m/s - the stop, then only the last of the burst")
check(latest.delivered == 2 and latest.dropped == 99,
      f"mux log: received {latest.received}, delivered {latest.delivered}, dropped {latest.dropped}")
latest.stop()
check(not latest._thread.is_alive(), "the drain thread stops on request")


def bench_coordinator(handler, cls):
    """A coordinator instance with one BASE, one route, and no transport: just
    enough for the real _make_stream_cb / _map_twist_to_base_joints path."""
    inst = object.__new__(cls)
    inst._hardware = {"base": types.SimpleNamespace(
        component=types.SimpleNamespace(hardware_type=HardwareType.BASE),
        joint_names=["base/vx", "base/vy", "base/wz"])}
    inst._hardware_lock = threading.Lock()
    inst._task_lock = threading.Lock()
    task = types.SimpleNamespace(name="vel_base",
                                 claim=lambda: types.SimpleNamespace(joints=set()))
    inst._routes = {"joint_command": [(task, handler, Routing.BROADCAST)]}
    inst._stream_pre_hooks = {"twist_command": inst._map_twist_to_base_joints}
    return inst


def twist(v):
    """Forward v m/s, turning at v/2 rad/s."""
    return Twist(linear=[v, 0.0, 0.0], angular=[0.0, 0.0, v / 2])


print("\nB. the production path: what the coordinator subscribes for twist_command")
gate_b = Gate()


def base_handler(msg, t_now):
    gate_b.consume(dict(zip(msg.name, msg.velocity)))


def section_b():
    coord = bench_coordinator(base_handler, MossControlCoordinator)
    cb = coord._make_stream_cb("twist_command")
    check(isinstance(cb, LatestOnly), f"twist_command is wrapped latest-only ({type(cb).__name__})")
    cb(twist(HOLD))
    check(gate_b.entered.wait(2.0), "the base is parked inside the first command")
    for v in BURST:
        cb(twist(v))
    check(len(gate_b.seen) == 1, f"no stale twist reaches the base ({len(gate_b.seen)} executed)")
    gate_b.release.set()
    check(wait_for(lambda: len(gate_b.seen) == 2), "the base wakes on the pending command")
    commanded = gate_b.seen[-1] if len(gate_b.seen) > 1 else {}
    check(commanded.get("base/vx") == LAST,
          f"base/vx commanded = {commanded.get('base/vx')} m/s (the newest twist, {LAST})")
    check(commanded.get("base/wz") == LAST / 2,
          f"base/wz commanded = {commanded.get('base/wz')} rad/s (the newest twist, {LAST / 2})")
    dropped_b = getattr(cb, "dropped", 0)   # 0 when the callback is not wrapped at all
    check(len(gate_b.seen) == 2 and dropped_b == 99,
          f"2 of 101 twists executed, {dropped_b} dropped")

    before = len(gate_b.seen)
    joint_cb = coord._make_stream_cb("joint_command")
    check(not isinstance(joint_cb, LatestOnly), "other streams stay unwrapped")
    for _ in range(3):
        joint_cb(types.SimpleNamespace(name=["base/vx"], velocity=[0.0]))
    check(len(gate_b.seen) == before + 3,
          f"joint_command still delivers every message, in the caller's thread ({len(gate_b.seen) - before}/3)")
    check(coord._make_stream_cb("twist_command") is cb,
          "a re-subscription reuses the same wrapper (one drain thread per coordinator)")
    if isinstance(cb, LatestOnly):
        cb.stop()


section_b()

print("\nC. without the wrapper: the same burst through dimOS's raw callback")
gate_c = Gate()


def base_handler_c(msg, t_now):
    gate_c.consume(dict(zip(msg.name, msg.velocity)))


stock = bench_coordinator(base_handler_c, ControlCoordinator)
raw = ControlCoordinator._make_stream_cb(stock, "twist_command")
check(not isinstance(raw, LatestOnly), "dimOS's own callback is unwrapped")
msgs = [twist(HOLD)] + [twist(v) for v in BURST]
# One FIFO delivery thread = what both transports do; it blocks on the parked
# consumer, which is exactly how the backlog builds up.
feeder = threading.Thread(target=lambda: [raw(m) for m in msgs], daemon=True)
feeder.start()
check(gate_c.entered.wait(2.0), "the base is parked inside the first command")
gate_c.release.set()
feeder.join(10.0)
check(wait_for(lambda: len(gate_c.seen) == 101),
      f"the whole backlog executes: {len(gate_c.seen)}/101 twists reach the base")
check(len(gate_c.seen) == 101 and gate_c.seen[-1].get("base/vx") == LAST,
      f"the newest twist runs LAST, behind {len(gate_c.seen) - 1} stale ones")
print(f"  ..  unwrapped {len(gate_c.seen)} executed vs {len(gate_b.seen) - 3} with the wrapper")

print("\nD. dimOS's zenoh subscriber offers no queue capacity to clamp")
from dimos.protocol.pubsub.impl import zenohpubsub  # noqa: E402

params = list(inspect.signature(zenohpubsub.ZenohPubSubBase.subscribe).parameters)
check(params == ["self", "topic", "callback"],
      f"ZenohPubSubBase.subscribe{tuple(params)} takes no capacity/handler argument")
src = inspect.getsource(zenohpubsub)
check("RingChannel" not in src and "FifoChannel" not in src,
      "no zenoh RingChannel/FifoChannel handler anywhere in the module")
qos_fields = set(zenohpubsub.ZenohQoS.__dataclass_fields__)
check(qos_fields == {"reliability", "congestion_control"},
      f"Topic.qos is publisher-side only: {sorted(qos_fields)}")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
raise SystemExit(1 if KO else 0)
