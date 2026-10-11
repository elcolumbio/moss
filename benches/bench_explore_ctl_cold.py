"""Cold bench for tools/explore_ctl.py: exploration start/stop must land on the run's LCM bus.

Known input -> known output, here in bytes on a named channel (what a bus
actually carries). Groups:

  A. channels  - the exact LCM channel for start and stop; the bus read in
                 dimOS's order: DIMOS_TRANSPORT, then TRANSPORT (neither = lcm)
  B. payload   - Bool(True) is 9 known bytes (8-byte LCM fingerprint of
                 std_msgs.Bool + 0x01), and decodes back to True
  C. the stack - those channels are the ones dimOS's own factory derives for the
                 streams the explorer subscribes to, and the bus explore_ctl reads
                 is the one dimOS's GlobalConfig picks for every pair of
                 DIMOS_TRANSPORT / TRANSPORT (needs dimOS, skipped without)
  D. lcm       - a real publish lands on /explore_cmd#std_msgs.Bool byte for byte
  E. refusal   - any other bus: publish raises naming the variable it read and
                 the fix, nothing reaches the LCM bus, and the CLI exits non-zero
                 (DIMOS_TRANSPORT=zenoh wins over TRANSPORT=lcm, as in dimOS)

End to end into a live explorer is a check on the robot.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python benches/bench_explore_ctl_cold.py
"""

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import lcm  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402

import explore_ctl  # noqa: E402

try:
    from dimos.core.global_config import GlobalConfig, global_config
    from dimos.core.transport_factory import transport_topic
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic
    HAVE_DIMOS = True
except Exception:  # noqa: BLE001 - no dimOS on a laptop
    HAVE_DIMOS = False

TRUE_BYTES = b"\x1e\xbe\xf0jP\x95\x0e>\x01"   # std_msgs.Bool fingerprint + data=True
LCM_START = "/explore_cmd#std_msgs.Bool"
LCM_STOP = "/stop_explore_cmd#std_msgs.Bool"
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def set_bus(dimos_transport=None, transport=None):
    """Both variables dimOS reads, set or unset together: every check names the pair."""
    for name, value in (("DIMOS_TRANSPORT", dimos_transport), ("TRANSPORT", transport)):
        if value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = value


def listen(bus, heard, wait_s):
    deadline = time.monotonic() + wait_s
    while not heard and time.monotonic() < deadline:
        bus.handle_timeout(100)


saved_bus = (os.environ.get("DIMOS_TRANSPORT"), os.environ.get("TRANSPORT"))

# (DIMOS_TRANSPORT, TRANSPORT) -> the bus dimOS runs on: the first one set wins
PAIRS = [((None, None), "lcm"), ((None, "lcm"), "lcm"), ((None, "zenoh"), "zenoh"),
         (("zenoh", None), "zenoh"), (("lcm", None), "lcm"), (("zenoh", "lcm"), "zenoh"),
         (("lcm", "zenoh"), "lcm")]

try:
    print("A. channels")
    set_bus(None, None)
    check("neither DIMOS_TRANSPORT nor TRANSPORT set -> lcm", explore_ctl.transport() == "lcm",
          explore_ctl.transport())
    check(f"unset: start -> {LCM_START}", explore_ctl.topic("start") == LCM_START, explore_ctl.topic("start"))
    check(f"unset: stop  -> {LCM_STOP}", explore_ctl.topic("stop") == LCM_STOP, explore_ctl.topic("stop"))
    set_bus(None, "lcm")
    check(f"lcm: start -> {LCM_START}", explore_ctl.topic("start") == LCM_START, explore_ctl.topic("start"))
    check(f"lcm: stop  -> {LCM_STOP}", explore_ctl.topic("stop") == LCM_STOP, explore_ctl.topic("stop"))
    set_bus(None, "LCM")
    check("TRANSPORT=LCM -> lcm (read case-insensitively)", explore_ctl.transport() == "lcm",
          explore_ctl.transport())
    got = {}
    for (dimos_tr, tr), want in PAIRS:
        set_bus(dimos_tr, tr)
        got[(dimos_tr, tr)] = (explore_ctl.transport(), want)
    check("this process's environment, dimOS's order: DIMOS_TRANSPORT first, then TRANSPORT (7 pairs)",
          all(a == b for a, b in got.values()), str({k: v[0] for k, v in got.items()}))
    env = {"DIMOS_TRANSPORT": "zenoh", "TRANSPORT": "lcm"}
    set_bus(None, "lcm")
    check("an explicit environment (the cockpit passes a stack's): DIMOS_TRANSPORT=zenoh TRANSPORT=lcm -> "
          "('DIMOS_TRANSPORT', 'zenoh'), whatever this process has",
          explore_ctl.transport_setting(env) == ("DIMOS_TRANSPORT", "zenoh") and explore_ctl.transport(env) == "zenoh"
          and explore_ctl.transport() == "lcm", str(explore_ctl.transport_setting(env)))
    check("an explicit empty environment -> ('', 'lcm')", explore_ctl.transport_setting({}) == ("", "lcm"))
    check("generic channel: goal_request + geometry_msgs.PoseStamped -> /goal_request#geometry_msgs.PoseStamped",
          explore_ctl.channel("goal_request", "geometry_msgs.PoseStamped")
          == "/goal_request#geometry_msgs.PoseStamped")

    print("B. payload")
    payload = Bool(data=True).lcm_encode()
    check(f"Bool(True) -> {TRUE_BYTES!r}", payload == TRUE_BYTES, repr(payload))
    check("decodes back to data=True", Bool.lcm_decode(payload).data is True)

    print("C. the stack's own channels" if HAVE_DIMOS else "C. the stack's own channels - SKIPPED (no dimOS)")
    if HAVE_DIMOS:
        global_config.update(transport="lcm")
        stack_lcm = {c: str(LCMTopic(transport_topic(f"/{n}"), Bool))
                     for c, n in explore_ctl.CHANNELS.items()}
        check("lcm channels match dimOS's factory",
              stack_lcm == {"start": LCM_START, "stop": LCM_STOP}, str(stack_lcm))
        both = {}
        for (dimos_tr, tr), want in PAIRS:
            set_bus(dimos_tr, tr)
            both[(dimos_tr, tr)] = (GlobalConfig().transport, explore_ctl.transport(), want)
        check("for every pair, explore_ctl's bus == the bus dimOS's GlobalConfig picks (7 pairs, "
              "DIMOS_TRANSPORT=zenoh TRANSPORT=lcm -> zenoh in both)",
              all(d == e == w for d, e, w in both.values()), str(both))

    print("D. lcm delivery")
    set_bus(None, "lcm")
    bus = lcm.LCM()
    heard = []
    bus.subscribe(LCM_START, lambda ch, data: heard.append((ch, data)))
    used = explore_ctl.publish("start")
    listen(bus, heard, 2.0)
    check(f"publish('start') says {LCM_START}", used == LCM_START, used)
    check("a subscriber on that channel got exactly the known bytes",
          heard == [(LCM_START, TRUE_BYTES)], str(heard))

    print("E. refusal on any other transport")
    heard_stop = []
    bus.subscribe(LCM_STOP, lambda ch, data: heard_stop.append((ch, data)))
    errors = {}
    for dimos_tr, tr in ((None, "zenoh"), ("zenoh", "lcm")):
        set_bus(dimos_tr, tr)
        try:
            explore_ctl.publish("stop")
            errors[(dimos_tr, tr)] = None
        except explore_ctl.TransportError as exc:
            errors[(dimos_tr, tr)] = str(exc)
    e1, e2 = errors[(None, "zenoh")], errors[("zenoh", "lcm")]
    check("TRANSPORT=zenoh: publish('stop') raises TransportError naming TRANSPORT=zenoh and the fix",
          e1 is not None and e1.startswith("TRANSPORT=zenoh:") and "lcm" in e1, str(e1))
    check("DIMOS_TRANSPORT=zenoh TRANSPORT=lcm: publish('stop') raises, naming DIMOS_TRANSPORT=zenoh "
          "(the variable dimOS obeys)",
          e2 is not None and e2.startswith("DIMOS_TRANSPORT=zenoh:") and "lcm" in e2, str(e2))
    listen(bus, heard_stop, 0.5)
    check("and nothing reached /stop_explore_cmd on the LCM bus", heard_stop == [], str(heard_stop))
    clean = {k: v for k, v in os.environ.items() if k not in ("DIMOS_TRANSPORT", "TRANSPORT")}
    for extra, named in (({"TRANSPORT": "zenoh"}, "TRANSPORT=zenoh"),
                         ({"DIMOS_TRANSPORT": "zenoh", "TRANSPORT": "lcm"}, "DIMOS_TRANSPORT=zenoh")):
        env = dict(clean, PYTHONDONTWRITEBYTECODE="1", **extra)
        proc = subprocess.run([sys.executable, str(ROOT / "tools" / "explore_ctl.py"), "stop"],
                              capture_output=True, text=True, env=env, timeout=30)
        check(f"CLI 'explore_ctl.py stop' under {' '.join(f'{k}={v}' for k, v in extra.items())} -> exit 1, "
              f"the reason on stderr naming {named}, nothing claimed",
              proc.returncode == 1 and "LCM only" in proc.stderr and f"explore_ctl: {named}:" in proc.stderr
              and "published" not in proc.stdout, f"rc {proc.returncode}, stderr {proc.stderr.strip()[:120]!r}")
    listen(bus, heard_stop, 0.5)
    check("and still nothing on /stop_explore_cmd", heard_stop == [], str(heard_stop))
    env = dict(clean, PYTHONDONTWRITEBYTECODE="1", TRANSPORT="lcm")
    proc = subprocess.run([sys.executable, str(ROOT / "tools" / "explore_ctl.py"), "stop"],
                          capture_output=True, text=True, env=env, timeout=30)
    listen(bus, heard_stop, 2.0)
    check(f"CLI 'explore_ctl.py stop' under TRANSPORT=lcm -> exit 0, 'published {LCM_STOP} on lcm', known bytes",
          proc.returncode == 0 and proc.stdout.strip() == f"published {LCM_STOP} on lcm"
          and heard_stop == [(LCM_STOP, TRUE_BYTES)], f"rc {proc.returncode} {proc.stdout.strip()!r} {heard_stop}")
    proc = subprocess.run([sys.executable, str(ROOT / "tools" / "explore_ctl.py"), "dance"],
                          capture_output=True, text=True, env=env, timeout=30)
    check("CLI with an unknown command -> exit 1 and the usage line",
          proc.returncode == 1 and "usage: explore_ctl.py start|stop" in proc.stderr, proc.stderr.strip())
finally:
    set_bus(*saved_bus)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
