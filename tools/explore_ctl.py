#!/usr/bin/env python3
"""Start or stop autonomous exploration from outside the stack (usage: explore_ctl.py start|stop).

The explorer listens on two Bool streams, `explore_cmd` and `stop_explore_cmd`.
This script publishes Bool(True) on the LCM channel dimOS derives for each of
them (the stream name, then the message type after a '#'):

    start  /explore_cmd#std_msgs.Bool
    stop   /stop_explore_cmd#std_msgs.Bool

MOSS runs its stack on LCM, the launchers' default. dimOS picks the bus from
DIMOS_TRANSPORT first, then TRANSPORT (neither set: lcm); this script reads
them in the same order. Under any other transport nobody would hear an LCM
publish, so the script refuses to publish and says why instead of reporting a
success that reached no one.

Importable: tools/moss_cockpit.py publishes goals, the soft stop and episode
markers through publish_msg(name, msg), which lands any dimos_lcm message on
the same bus with the same naming rule.
"""
import os
import sys

from dimos_lcm.std_msgs import Bool

CHANNELS = {"start": "explore_cmd", "stop": "stop_explore_cmd", "clear": "clear_map_cmd"}   # clear: the costmap starts a fresh grid
SUPPORTED_TRANSPORT = "lcm"
TRANSPORT_VARS = ("DIMOS_TRANSPORT", "TRANSPORT")   # dimOS's order: the first one set wins


class TransportError(RuntimeError):
    """The run is not on LCM: a publish from here would reach nobody."""


def transport_setting(env=None) -> tuple:
    """(the variable that chose the bus, the bus) from `env` (default: this process's
    environment), in dimOS's order: DIMOS_TRANSPORT, then TRANSPORT, case-insensitive.
    Neither set: ("", "lcm"), dimOS's default on Linux."""
    env = os.environ if env is None else env
    for var in TRANSPORT_VARS:
        value = (env.get(var) or "").strip().lower()
        if value:
            return var, value
    return "", SUPPORTED_TRANSPORT


def transport(env=None) -> str:
    """The run's bus, read in dimOS's order (see transport_setting)."""
    return transport_setting(env)[1]


def require_lcm(env=None) -> None:
    """Raise TransportError, naming the variable it read and the fix, unless the run is on LCM."""
    var, bus = transport_setting(env)
    if bus != SUPPORTED_TRANSPORT:
        raise TransportError(f"{var}={bus}: explore_ctl publishes on LCM only, the bus MOSS runs on "
                             f"(its launchers default DIMOS_TRANSPORT and TRANSPORT to lcm). Set both to "
                             f"lcm, or unset both.")


def channel(name: str, msg_name: str) -> str:
    """The LCM channel of stream `name` carrying `msg_name`."""
    return f"/{name}#{msg_name}"


def topic(cmd: str) -> str:
    """The LCM channel this command lands on."""
    return channel(CHANNELS[cmd], Bool.msg_name)


def publish_msg(name: str, msg, msg_name: str = None, bus=None, env=None) -> str:
    """Publish any dimos_lcm message on stream `name`; returns the channel used.

    `msg_name` is the type suffix dimOS puts in the channel. It defaults to the
    message's own msg_name, but pass it for types whose dimos_lcm class carries a
    bare name: dimos_lcm's PoseStamped says "PoseStamped" while dimOS's wrapper
    (and so every stack channel) says "geometry_msgs.PoseStamped". `bus`: an
    already open lcm.LCM to publish through (a long-running caller keeps one);
    None opens one per call. `env`: the environment that chose the run's bus (the
    cockpit passes the running stack's); None reads this process's. Raises
    TransportError when the run is not on LCM.
    """
    require_lcm(env)
    where = channel(name, msg_name or msg.msg_name)
    payload = msg.lcm_encode()
    if bus is None:
        import lcm  # imported here: the cockpit runs without it in demo mode

        bus = lcm.LCM()
    bus.publish(where, payload)
    return where


def publish(cmd: str) -> str:
    """Publish Bool(True) for 'start'|'stop'; returns the channel used."""
    return publish_msg(CHANNELS[cmd], Bool(data=True))


def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "start"
    if cmd not in CHANNELS:
        sys.exit("usage: explore_ctl.py start|stop|clear")
    try:
        used = publish(cmd)
    except TransportError as exc:
        sys.exit(f"explore_ctl: {exc}")
    print(f"published {used} on {transport()}")


if __name__ == "__main__":
    main()
