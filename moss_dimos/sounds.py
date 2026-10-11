"""MossSounds: the rover's nonverbal voice, a sound pack played on confirmed events.

Sixty short synthesized cues (assets/sounds, twenty event families, three
variants each) and a non-blocking player (sound_pack/player.py: one subprocess
per cue through PulseAudio's `paplay` or PipeWire's `pw-play`, cooldown per
event, priority queue, a global gap so the rover never chatters). This module
maps the stack's events onto the families that EXIST on the robot today;
every other family stays silent until its event does.

  In  bump            : Bool     a contact (the base's stall guard, the explorer's back-off) -> bump_surprise
  In  base_telemetry  : String   JSON from the coordinator (2 Hz): stall_count, bus_mV ->
                                 recovery_success once the tracks move again after a stall,
                                 battery_low / battery_critical on the bus voltage (hysteresis)
  In  explore_cmd     : Bool     the order to explore -> resume
  In  explore_done    : Bool     the explorer found nothing left -> mission_complete

boot_ready plays once when the module starts (the stack is up). Nothing plays
twice in a row within an event's cooldown, and nothing plays while muted.

Environment: MOSS_SOUNDS (on by default; 0 = the blueprint leaves the module
out), MOSS_SOUND_VOLUME (0..1, default from assets/sounds/config.json),
MOSS_SOUND_SINK (a PulseAudio sink name or index; default = the session's
default sink, which is the Bluetooth speaker once it is connected),
MOSS_SOUND_BACKEND (paplay or pw-play; default: paplay when present).

The player needs the user's audio session: a stack launched from a login
shell or over ssh as the robot's user finds it (XDG_RUNTIME_DIR); a system
service would not. Without a backend the module logs once and stays silent.
"""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.std_msgs.String import String
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

from moss_dimos import robot_profile

logger = setup_logger()

ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "sounds"
BACKENDS = ("paplay", "pw-play")    # PulseAudio first: on the Jetson the Bluetooth speaker is a PulseAudio sink (module-bluez5),
                                    # PipeWire there has no Bluetooth; MOSS_SOUND_BACKEND forces one of the two
BATTERY_LOW_MV = 11000          # 3S pack: low below this at rest...
BATTERY_LOW_CLEAR_MV = 11300    # ...and back to normal above this (hysteresis: a sag under load does not toggle it)
BATTERY_CRITICAL_MV = 10600     # the flight check refuses below 10.8 V; critical is lower still
BATTERY_SAMPLES = 6             # consecutive telemetry lines (3 s at 2 Hz) below a threshold before it counts
RECOVERY_WINDOW_S = 20.0        # a stall followed by motion inside this window = a recovery
RECOVERY_MIN_TICKS = 100        # encoder ticks that count as "moving again" after a stall


def sound_volume(default: float, env: dict | None = None) -> float:
    """MOSS_SOUND_VOLUME clamped to 0..1; unset or unreadable -> default."""
    raw = (env if env is not None else os.environ).get("MOSS_SOUND_VOLUME", "")
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return default
    if v != v:
        return default
    return max(0.0, min(1.0, v))


def playback_backend(env: dict | None = None) -> str | None:
    """The playback command: MOSS_SOUND_BACKEND when set and on the PATH, else the
    first of BACKENDS on the PATH, None when there is none."""
    forced = (env if env is not None else os.environ).get("MOSS_SOUND_BACKEND", "").strip()
    if forced:
        return forced if forced in BACKENDS and shutil.which(forced) else None
    for name in BACKENDS:
        if shutil.which(name):
            return name
    return None


class EventMapper:
    """The stack's events -> sound families, with the state the decisions need.
    Pure: call `on_*` and read the list of family names it returns; the module
    hands them to the player. Benched without audio."""

    def __init__(self, now: float | None = None) -> None:
        self._stalls = 0
        self._stall_t: float | None = None
        self._stall_ticks: tuple[int, int] | None = None
        self._battery_state = "ok"           # ok | low | critical
        self._below_low = 0
        self._below_critical = 0

    def on_bump(self, data: bool) -> list[str]:
        return ["bump_surprise"] if data else []

    def on_explore_cmd(self, data: bool) -> list[str]:
        return ["resume"] if data else []

    def on_explore_done(self, data: bool) -> list[str]:
        return ["mission_complete"] if data else []

    def on_telemetry(self, doc: dict, now: float) -> list[str]:
        out: list[str] = []
        out += self._stall_events(doc, now)
        out += self._battery_events(doc)
        return out

    def _stall_events(self, doc: dict, now: float) -> list[str]:
        """A new stall remembers the ticks; motion of RECOVERY_MIN_TICKS on
        either track within RECOVERY_WINDOW_S after it = recovery_success.
        (The contact cue itself comes from `bump`, published by the coordinator.)"""
        out: list[str] = []
        stalls = int(doc.get("stall_count") or 0)
        ticks = doc.get("ticks")
        ticks = (int(ticks[0]), int(ticks[1])) if isinstance(ticks, (list, tuple)) and len(ticks) == 2 else None
        if stalls > self._stalls:
            self._stalls = stalls
            self._stall_t = now
            self._stall_ticks = ticks
            return out
        if self._stall_t is not None and ticks is not None and self._stall_ticks is not None:
            if now - self._stall_t > RECOVERY_WINDOW_S:
                self._stall_t = None
            elif max(abs(ticks[0] - self._stall_ticks[0]), abs(ticks[1] - self._stall_ticks[1])) >= RECOVERY_MIN_TICKS:
                self._stall_t = None
                out.append("recovery_success")
        return out

    def _battery_events(self, doc: dict) -> list[str]:
        """ok -> low below BATTERY_LOW_MV for BATTERY_SAMPLES lines, low -> ok above
        BATTERY_LOW_CLEAR_MV; low -> critical below BATTERY_CRITICAL_MV for the
        same count. Each entry into low or critical plays once."""
        mv = doc.get("bus_mV")
        try:
            mv = int(mv)
        except (TypeError, ValueError):
            return []
        if mv <= 0:
            return []
        out: list[str] = []
        self._below_low = self._below_low + 1 if mv < BATTERY_LOW_MV else 0
        self._below_critical = self._below_critical + 1 if mv < BATTERY_CRITICAL_MV else 0
        if self._battery_state == "ok" and self._below_low >= BATTERY_SAMPLES:
            self._battery_state = "low"
            out.append("battery_low")
        if self._battery_state == "low" and self._below_critical >= BATTERY_SAMPLES:
            self._battery_state = "critical"
            out.append("battery_critical")
        if self._battery_state != "ok" and mv > BATTERY_LOW_CLEAR_MV:
            self._battery_state = "ok"
        return out


class MossSounds(Module):
    """The sound pack on the bus. Ins only; see the module docstring."""

    bump: In[Bool]
    base_telemetry: In[String]
    explore_cmd: In[Bool]
    explore_done: In[Bool]

    def __init__(self, assets_dir: str | None = None, volume: float | None = None,
                 sink: str | None = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._assets_dir = Path(assets_dir) if assets_dir else ASSETS_DIR
        self._volume = volume
        self._sink = sink
        self._player = None
        self._mapper = EventMapper()
        self._silent_reason: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        backend = playback_backend()
        if backend is None:
            self._silent_reason = f"no playback backend on the PATH ({' / '.join(BACKENDS)})"
            logger.warning(f"MOSS sounds: {self._silent_reason} - the rover stays silent")
            return
        try:
            from moss_dimos.sound_pack.player import MossSoundPlayer
            config = {"backend": backend}
            if self._sink:
                config["target"] = self._sink
            self._player = MossSoundPlayer(root=self._assets_dir, config=config)
            vol = sound_volume(self._player.volume) if self._volume is None else self._volume
            self._player.set_volume(vol)
        except Exception as exc:  # noqa: BLE001 - a broken pack never stops the stack
            self._silent_reason = f"{type(exc).__name__}: {exc}"
            logger.warning(f"MOSS sounds: player not started ({self._silent_reason}) - the rover stays silent")
            self._player = None
            return
        logger.info(f"MOSS sounds: up, {len(self._player.rules)} event families, backend {backend}, "
                    f"sink {self._sink or 'default'}, volume {self._player.volume:.2f}, assets {self._assets_dir}")
        self._play(["boot_ready"])

    @rpc
    def stop(self) -> None:
        if self._player is not None:
            try:
                self._player.close()
            except Exception:  # noqa: BLE001
                pass
            self._player = None
        super().stop()

    @rpc
    def set_mute(self, muted: bool = True) -> bool:
        if self._player is None:
            return False
        self._player.set_mute(bool(muted))
        return True

    @rpc
    def set_volume(self, volume: float) -> bool:
        if self._player is None:
            return False
        self._player.set_volume(max(0.0, min(1.0, float(volume))))
        return True

    def _play(self, families: list[str]) -> None:
        if self._player is None:
            return
        for name in families:
            queued = self._player.emit(name)
            logger.info(f"MOSS sounds: {name} {'queued' if queued else 'suppressed (cooldown, mute or queue)'}")

    async def handle_bump(self, msg: Bool) -> None:
        self._play(self._mapper.on_bump(bool(getattr(msg, "data", False))))

    async def handle_explore_cmd(self, msg: Bool) -> None:
        self._play(self._mapper.on_explore_cmd(bool(getattr(msg, "data", False))))

    async def handle_explore_done(self, msg: Bool) -> None:
        self._play(self._mapper.on_explore_done(bool(getattr(msg, "data", False))))

    async def handle_base_telemetry(self, msg: String) -> None:
        try:
            doc = json.loads(getattr(msg, "data", "") or "{}")
        except (TypeError, ValueError):
            return
        if isinstance(doc, dict):
            self._play(self._mapper.on_telemetry(doc, time.monotonic()))


def sounds_enabled(env: dict | None = None) -> bool:
    """MOSS_SOUNDS: on unless set to 0 / false / no / off."""
    return robot_profile.env_on("MOSS_SOUNDS", True, env)
