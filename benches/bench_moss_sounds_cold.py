"""Cold bench: moss_dimos/sounds.py, the sound pack on the stack's events.

Known input -> known output: a bump plays bump_surprise; a stall followed by
motion plays recovery_success once; the battery crossing the low threshold for
long enough plays battery_low once, with hysteresis; the vendored pack has
every file its manifest names; the player in dry-run mode queues what the
module hands it. No audio device, no subprocess.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 benches/bench_moss_sounds_cold.py
"""
import asyncio
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OK = KO = 0


def check(cond, label, detail=""):
    global OK, KO
    if cond:
        OK += 1
    else:
        KO += 1
    print(("  OK  " if cond else "  KO  ") + label + (f" - {detail}" if detail and not cond else ""))


print("A. the vendored pack: manifest, files, player in dry run")
from moss_dimos.sound_pack.player import MossSoundPlayer, playback_command  # noqa: E402

ASSETS = ROOT / "assets" / "sounds"
manifest = json.loads((ASSETS / "manifest.json").read_text())
missing = [k for k, a in manifest["assets"].items() if not (ASSETS / a["path"]).exists()]
check(len(manifest["assets"]) == 60 and not missing, f"60 assets named and present ({len(manifest['assets'])} named, missing {missing[:3]})")
check(len(manifest["events"]) == 20 and all(len(e["variants"]) == 3 for e in manifest["events"].values()),
      "20 event families, three variants each")
check("date" not in manifest and "hardware_context" not in manifest, "the manifest carries no date and no workshop context")
cmd = playback_command(ASSETS / "boot_ready_01.wav", backend="paplay", target="bt_sink", volume=0.5) if __import__("shutil").which("paplay") else None
check(cmd is None or (cmd[0] == "paplay" and "--device=bt_sink" in cmd and cmd[-1].endswith("boot_ready_01.wav")),
      f"paplay command names the sink and the file: {cmd}")
player = MossSoundPlayer(root=ASSETS, dry_run=True)
check(set(player.rules) == set(manifest["events"]), "the player loads every family from the manifest")
check(player.emit("bump_surprise") is True, "emit(bump_surprise) is queued")
check(player.emit("bump_surprise") is False, "the same family again inside its cooldown is suppressed")
check(player.emit("no_such_family") is False, "an unknown family is refused")
player.wait_idle(timeout=3)
status = player.drain_status()
check(any(s.get("event") == "bump_surprise" for s in status), f"dry run reports the cue: {[s.get('state') for s in status][:4]}")
player.set_mute(True)
check(player.emit("resume") is False, "muted: nothing queued")
player.set_mute(False)
player.close()

print("B. EventMapper: the stack's events -> families")
from moss_dimos.sounds import (BATTERY_CRITICAL_MV, BATTERY_LOW_CLEAR_MV, BATTERY_LOW_MV, BATTERY_SAMPLES,  # noqa: E402
                               RECOVERY_MIN_TICKS, RECOVERY_WINDOW_S, EventMapper, MossSounds, sound_volume, sounds_enabled)

m = EventMapper()
check(m.on_bump(True) == ["bump_surprise"] and m.on_bump(False) == [], "bump True -> bump_surprise, False -> nothing")
check(m.on_explore_cmd(True) == ["resume"] and m.on_explore_done(True) == ["mission_complete"], "explore_cmd -> resume, explore_done -> mission_complete")
t0 = 1000.0
tel = lambda stalls, ticks, mv=11800: {"stall_count": stalls, "ticks": list(ticks), "bus_mV": mv}  # noqa: E731
check(m.on_telemetry(tel(0, (100, 100)), t0) == [], "no stall, healthy battery -> nothing")
check(m.on_telemetry(tel(1, (100, 100)), t0 + 1) == [], "a new stall is remembered, nothing plays (the bump cue comes from the bus)")
check(m.on_telemetry(tel(1, (120, 110)), t0 + 2) == [], "a few ticks after the stall are not a recovery")
check(m.on_telemetry(tel(1, (100 + RECOVERY_MIN_TICKS, 100)), t0 + 3) == ["recovery_success"], "the tracks move again -> recovery_success")
check(m.on_telemetry(tel(1, (100 + 3 * RECOVERY_MIN_TICKS, 100)), t0 + 4) == [], "...once")
check(m.on_telemetry(tel(2, (500, 500)), t0 + 10) == [] and m.on_telemetry(tel(2, (500 + RECOVERY_MIN_TICKS, 500)), t0 + 10 + RECOVERY_WINDOW_S + 1) == [],
      "motion after the window is not a recovery")
b = EventMapper()
low = BATTERY_LOW_MV - 100
res = []
for i in range(BATTERY_SAMPLES):
    res += b.on_telemetry({"bus_mV": low}, t0 + i)
check(res == ["battery_low"], f"battery_low after {BATTERY_SAMPLES} lines below {BATTERY_LOW_MV} mV, once: {res}")
check(b.on_telemetry({"bus_mV": low}, t0 + 9) == [], "...and not again while it stays low")
check(b.on_telemetry({"bus_mV": BATTERY_LOW_MV + 100}, t0 + 10) == [], "a sag recovery between low and clear changes nothing")
check(b._battery_state == "low", "still low below the clear level")
check(b.on_telemetry({"bus_mV": BATTERY_LOW_CLEAR_MV + 50}, t0 + 11) == [] and b._battery_state == "ok", "above the clear level -> ok again (hysteresis)")
c = EventMapper()
seq = []
for i in range(BATTERY_SAMPLES):
    seq += c.on_telemetry({"bus_mV": BATTERY_CRITICAL_MV - 50}, t0 + i)
check(seq == ["battery_low", "battery_critical"], f"straight to critical: low then critical, each once: {seq}")
check(c.on_telemetry({"bus_mV": 0}, t0 + 20) == [] and c.on_telemetry({"bus_mV": None}, t0 + 21) == [], "no reading -> nothing")
check(sound_volume(0.65, {"MOSS_SOUND_VOLUME": "0.3"}) == 0.3 and sound_volume(0.65, {}) == 0.65
      and sound_volume(0.65, {"MOSS_SOUND_VOLUME": "2"}) == 1.0 and sound_volume(0.65, {"MOSS_SOUND_VOLUME": "loud"}) == 0.65,
      "MOSS_SOUND_VOLUME: 0.3 -> 0.3, unset -> default, 2 -> 1.0, text -> default")
check(sounds_enabled({}) is True and sounds_enabled({"MOSS_SOUNDS": "0"}) is False and sounds_enabled({"MOSS_SOUNDS": "off"}) is False,
      "MOSS_SOUNDS: unset on, 0 / off -> off")

print("C. the module: handlers hand the families to the player")
from dimos.msgs.std_msgs.String import String  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402

mod = MossSounds()
asyncio.run(mod.handle_bump(Bool(data=True)))              # no player yet: nothing happens, nothing raises
mod._player = MossSoundPlayer(root=ASSETS, config={"global_gap_s": 0.0}, dry_run=True)   # no gap here: the bench wants every cue, the floor wants the gap
# events seconds apart on the floor: a short pause between them here, because the player drops a
# cue that arrives while its worker holds the lock (by design: never block the caller)
for call in (lambda: mod.handle_bump(Bool(data=True)),
             lambda: mod.handle_explore_cmd(Bool(data=True)),
             lambda: mod.handle_explore_done(Bool(data=True)),
             lambda: mod.handle_base_telemetry(String(data="not json")),
             lambda: mod.handle_base_telemetry(String(data=json.dumps({"stall_count": 1, "ticks": [0, 0], "bus_mV": 11800}))),
             lambda: mod.handle_base_telemetry(String(data=json.dumps({"stall_count": 1, "ticks": [RECOVERY_MIN_TICKS, 0], "bus_mV": 11800})))):
    asyncio.run(call())
    time.sleep(0.15)
mod._player.wait_idle(timeout=8)
played = [s.get("event") for s in mod._player.drain_status() if s.get("event")]
check({"bump_surprise", "resume", "mission_complete", "recovery_success"} <= set(played),
      f"bump, explore_cmd, explore_done and a recovery reached the player: {sorted(set(played))}")
check(mod.set_volume(0.2) is True and abs(mod._player.volume - 0.2) < 1e-9 and mod.set_mute(True) is True and mod._player.muted,
      "set_volume / set_mute reach the player")
mod._player.close()
check(MossSounds().set_mute(True) is False, "set_mute without a player answers False, never raises")
check(set(MossSounds.__annotations__) == {"bump", "base_telemetry", "explore_cmd", "explore_done"},
      f"Ins only: bump, base_telemetry, explore_cmd, explore_done ({sorted(MossSounds.__annotations__)})")

print("D. hygiene: the vendored player and the pack carry no workshop context")
import re  # noqa: E402
texts = [(ROOT / "moss_dimos" / "sound_pack" / "player.py").read_text(), (ASSETS / "manifest.json").read_text(), (ASSETS / "config.json").read_text()]
check(not any(re.search(r"\b(20\d\d-\d\d-\d\d|\d\d/\d\d/20\d\d)\b", t) for t in texts), "no date in the player, the manifest or the config")
check(all(ord(ch) < 128 for t in texts for ch in t), "ASCII only (English)")

print(f"\n{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
