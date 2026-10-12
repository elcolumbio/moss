"""Cold bench: moss_dimos/sounds.py, the sound pack on the stack's events.

Known input -> known output: a bump plays bump_surprise; a stall followed by
motion plays recovery_success once; the battery crossing the low threshold for
long enough plays battery_low once, with hysteresis; the vendored pack has
every file its manifest names; the player in dry-run mode queues what the
module hands it. E: the keep-awake period and the pre-roll are read from the
environment; the pre-rolled copy of a cue is N + 0.3 s * rate frames, silence
then the cue; the backend plays that copy, or the original with the pre-roll
off or when the copy cannot be written; the speaker script connects, sets the
default sink and gives up after its tries. No audio device and no Bluetooth:
the playback backend and the Bluetooth / PulseAudio tools are shell stubs that
record their arguments (the script runs with a PATH of stubs only).

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
import wave as _wave  # noqa: E402
with _wave.open(str(ASSETS / "keep_awake.wav"), "rb") as _w:
    check(_w.getnchannels() == 1 and _w.getframerate() == 44100 and 0.4 < _w.getnframes() / 44100 < 1.0
          and max(abs(x) for x in __import__("struct").unpack("<" + "h" * _w.getnframes(), _w.readframes(_w.getnframes()))) == 0,
          "keep_awake.wav: mono 44.1 kHz, about half a second, all zeros (silent)")
from moss_dimos.sounds import KEEP_AWAKE_S  # noqa: E402
check(KEEP_AWAKE_S == 15.0, "the keep-awake default is every 15 s")
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


print("E. the speaker that stays awake: keep-awake period, pre-roll, reconnection at boot")
import shutil  # noqa: E402
import subprocess  # noqa: E402
import tempfile  # noqa: E402
from dimos.core.module import Module  # noqa: E402
import moss_dimos.sounds as sounds_mod  # noqa: E402
from moss_dimos.sound_pack.player import PREROLL_S, preroll_wav  # noqa: E402

kap = sounds_mod.keep_awake_period
check(kap({}) == 15.0 and kap({"MOSS_SOUND_KEEP_AWAKE_S": "0"}) == 0.0 and kap({"MOSS_SOUND_KEEP_AWAKE_S": "often"}) == 15.0
      and kap({"MOSS_SOUND_KEEP_AWAKE_S": "30"}) == 30.0 and kap({"MOSS_SOUND_KEEP_AWAKE_S": "-5"}) == 0.0,
      "MOSS_SOUND_KEEP_AWAKE_S: unset -> 15 s, 0 -> off, text -> 15 s, 30 -> 30 s, negative -> off")
work = Path(tempfile.mkdtemp(prefix="moss_sounds_bench_"))
saved_env = {k: os.environ.get(k) for k in ("PATH", "MOSS_SOUND_CACHE", "MOSS_SOUND_PREROLL_S", "MOSS_SOUND_KEEP_AWAKE_S")}


def frames_of(path):
    with _wave.open(str(path), "rb") as w:
        return (w.getnchannels(), w.getsampwidth(), w.getframerate()), w.getnframes(), w.readframes(w.getnframes())


src = ASSETS / "bump_surprise_01.wav"
params, n, raw = frames_of(src)
pre = round(0.3 * params[2])
copy = preroll_wav(src, 0.3, work / "cache_unit")
cparams, cn, craw = frames_of(copy)
lead = pre * params[0] * params[1]
check(cparams == params and cn == n + pre, f"pre-rolled copy: {n} + 0.3 s * {params[2]} = {n + pre} frames, same params ({cn}, {cparams})")
check(craw[:lead] == bytes(lead) and craw[lead:] == raw, "its first 0.3 s are all zeros, the rest is the cue byte for byte")
mtime = copy.stat().st_mtime_ns
check(preroll_wav(src, 0.3, work / "cache_unit") == copy and copy.stat().st_mtime_ns == mtime
      and not list((work / "cache_unit").glob("*.tmp")), "a second call reuses the copy (written once), no temporary left")
os.environ.pop("MOSS_SOUND_PREROLL_S", None)
_p = MossSoundPlayer(root=ASSETS, dry_run=True)
check(PREROLL_S == 0.3 and _p.preroll_s == 0.3, "MOSS_SOUND_PREROLL_S unset -> 0.3 s")
_p.close()
for raw_env, want in (("0", 0.0), ("loud", 0.3), ("0.5", 0.5)):
    os.environ["MOSS_SOUND_PREROLL_S"] = raw_env
    _p = MossSoundPlayer(root=ASSETS, dry_run=True)
    check(_p.preroll_s == want, f"MOSS_SOUND_PREROLL_S={raw_env} -> {want} s ({_p.preroll_s})")
    _p.close()

# the backend: a paplay stub on the PATH that records its arguments, one call per line
stub_bin = work / "bin"
stub_bin.mkdir()
calls = work / "paplay_calls"
(stub_bin / "paplay").write_text(f'#!/bin/sh\necho "$*" >> "{calls}"\nexit 0\n')
(stub_bin / "paplay").chmod(0o755)
os.environ["PATH"] = f"{stub_bin}{os.pathsep}{saved_env['PATH']}"


def played_by(config_extra=None, log=None, dry_run=False):
    """One bump_surprise through a real (not dry-run) player on the stub backend; the file it played."""
    calls.write_text("")
    pl = MossSoundPlayer(root=ASSETS, config={"backend": "paplay", "global_gap_s": 0.0, **(config_extra or {})},
                         dry_run=dry_run, log=log)
    queued = pl.emit("bump_surprise")
    pl.wait_idle(timeout=8)
    rows = pl.drain_status()
    pl.close()
    lines = [ln for ln in calls.read_text().splitlines() if ln.strip()]
    return queued, (Path(lines[-1].split()[-1]) if lines else None), rows


os.environ.pop("MOSS_SOUND_PREROLL_S", None)
os.environ["MOSS_SOUND_CACHE"] = str(work / "cache_on")
queued, played, rows = played_by()
ok = played is not None and played.parent == work / "cache_on" and played.name.startswith("bump_surprise_0")
if ok:
    orig = ASSETS / (played.name.split("-")[0] + ".wav")
    oparams, on, oraw = frames_of(orig)
    pparams, pn, praw = frames_of(played)
    ok = pparams == oparams and pn == on + round(0.3 * oparams[2]) and praw[round(0.3 * oparams[2]) * 2:] == oraw
check(queued and ok and any(r["state"] == "finished" for r in rows),
      f"pre-roll on: the backend plays the cached copy (silence + the cue) from MOSS_SOUND_CACHE: {played}")
check(len(list((work / "cache_on").glob("*.wav"))) == len(manifest["assets"]), "one copy per asset, written before the first cue")
os.environ["MOSS_SOUND_PREROLL_S"] = "0"
os.environ["MOSS_SOUND_CACHE"] = str(work / "cache_off")
queued, played, rows = played_by()
check(queued and played is not None and played.parent == ASSETS.resolve() and played.name.startswith("bump_surprise_0")
      and not (work / "cache_off").exists(), f"MOSS_SOUND_PREROLL_S=0: the backend plays the original asset, no cache written: {played}")
os.environ.pop("MOSS_SOUND_PREROLL_S", None)
(work / "not_a_dir").write_text("a file where the cache dir should be")
os.environ["MOSS_SOUND_CACHE"] = str(work / "not_a_dir" / "cache")
said = []
queued, played, rows = played_by(log=said.append)
check(queued and played is not None and played.parent == ASSETS.resolve() and len(said) == 1
      and sum(r["state"] == "preroll_failed" for r in rows) == 1,
      f"the cache cannot be written: the original plays, said once ({len(said)} log line(s): {said[:1]})")
os.environ["MOSS_SOUND_CACHE"] = str(work / "cache_dry")
dry = MossSoundPlayer(root=ASSETS, config={"global_gap_s": 0.0}, dry_run=True)
check(dry.emit("bump_surprise") is True, "dry run with the pre-roll on: emit is queued")
dry.wait_idle(timeout=3)
drows = dry.drain_status()
dry.close()
check(any(r.get("state") == "simulated" and r.get("event") == "bump_surprise" for r in drows) and not (work / "cache_dry").exists(),
      f"dry run: the emit still reports (simulated), nothing written: {[r.get('state') for r in drows]}")

# the module's start: the period read there and said in its first line (dimOS's own start/stop left out: no bus here)
lines_said = []


class _Log:
    def info(self, text): lines_said.append(text)
    def warning(self, text): lines_said.append(text)


real_logger, real_start, real_stop = sounds_mod.logger, Module.start, Module.stop
sounds_mod.logger = _Log()
Module.start = lambda self: None
Module.stop = lambda self: None
try:
    os.environ["MOSS_SOUND_BACKEND"] = "paplay"
    os.environ["MOSS_SOUND_CACHE"] = str(work / "cache_mod")
    os.environ["MOSS_SOUND_KEEP_AWAKE_S"] = "0.3"
    calls.write_text("")
    mod2 = MossSounds()
    mod2.start()
    time.sleep(1.5)
    mod2.stop()
    breaths = sum("keep_awake.wav" in ln for ln in calls.read_text().splitlines())
    up = next((ln for ln in lines_said if "MOSS sounds: up" in ln), "")
    check(mod2._keep_awake_s == 0.3 and "keep-awake every 0.3 s" in up and "pre-roll 0.30 s" in up,
          f"start reads MOSS_SOUND_KEEP_AWAKE_S and its line says the period and the pre-roll: {up[:160]}")
    check(breaths >= 2, f"the keep-awake clip plays at that period ({breaths} in 1.5 s at 0.3 s)")
    os.environ["MOSS_SOUND_KEEP_AWAKE_S"] = "0"
    lines_said.clear()
    mod3 = MossSounds()
    mod3.start()
    up = next((ln for ln in lines_said if "MOSS sounds: up" in ln), "")
    check(getattr(mod3, "_awake_stop", None) is None and "keep-awake off" in up, f"MOSS_SOUND_KEEP_AWAKE_S=0: no keep-awake thread, the line says off: {up[:160]}")
    mod3.stop()
finally:
    sounds_mod.logger, Module.start, Module.stop = real_logger, real_start, real_stop
    os.environ.pop("MOSS_SOUND_BACKEND", None)
    for k, v in saved_env.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v

# tools/moss-jbl-connect.sh: syntax, refusal, and a run against stubs (bluetoothctl, pactl, sleep) on a PATH of stubs only
SCRIPT = ROOT / "tools" / "moss-jbl-connect.sh"
BASH = shutil.which("bash")
check(subprocess.run([BASH, "-n", str(SCRIPT)], capture_output=True).returncode == 0, "moss-jbl-connect.sh: bash -n")
jbin = work / "jbin"
jbin.mkdir()
for tool in ("timeout", "seq", "sed", "grep", "awk", "tr", "tail"):
    (jbin / tool).symlink_to(shutil.which(tool))
count, bt_log, pa_log, sl_log = work / "bt_count", work / "bt_args", work / "pa_args", work / "sleep_args"


def jbl_stubs(succeed_on, sink=True):
    """bluetoothctl answers "Connection successful" from its `succeed_on`-th call (0 = never);
    pactl lists the speaker's bluez sink unless `sink` is False (a link that dropped)."""
    count.write_text("0\n")
    for f in (bt_log, pa_log, sl_log):
        f.write_text("")
    (jbin / "bluetoothctl").write_text(
        "#!/bin/sh\n"
        f'read n < "{count}"; n=$((n + 1)); echo "$n" > "{count}"; echo "$*" >> "{bt_log}"\n'
        f'if [ {succeed_on} -gt 0 ] && [ "$n" -ge {succeed_on} ]; then echo "Attempting to connect to $2"; echo "Connection successful"; exit 0; fi\n'
        'echo "Failed to connect: org.bluez.Error.Failed br-connection-page-timeout"; exit 1\n')
    (jbin / "pactl").write_text(
        "#!/bin/sh\n"
        f'echo "$*" >> "{pa_log}"\n'
        'if [ "$1 $2 $3" = "list short sinks" ]; then\n'
        '  printf "0\\talsa_output.platform-sound.analog-stereo\\tmodule-alsa-card.c\\ts16le 2ch 44100Hz\\tSUSPENDED\\n"\n'
        + ('  printf "1\\tbluez_sink.00_00_00_00_00_00.a2dp_sink\\tmodule-bluez5-device.c\\ts16le 2ch 44100Hz\\tIDLE\\n"\n'
           if sink else "")
        + "fi\nexit 0\n")
    (jbin / "sleep").write_text(f'#!/bin/sh\necho "$*" >> "{sl_log}"\nexit 0\n')
    for t in ("bluetoothctl", "pactl", "sleep"):
        (jbin / t).chmod(0o755)


def jbl_run(env_extra):
    env = {"PATH": str(jbin), "HOME": str(work), "MOSS_STATE_DIR": str(work / "state"), **env_extra}
    return subprocess.run([BASH, str(SCRIPT)], env=env, capture_output=True, text=True, timeout=60)


jbl_stubs(2)
r = jbl_run({})
check(r.returncode == 2 and "no speaker address" in r.stderr and bt_log.read_text() == "",
      f"no MOSS_JBL_MAC and no jbl.env: exit 2 with a message, bluetoothctl never called ({r.returncode}: {r.stderr.strip()[:90]})")
r = jbl_run({"MOSS_JBL_MAC": "not-an-address"})
check(r.returncode == 2 and "not a Bluetooth address" in r.stderr, f"a malformed address: exit 2 ({r.returncode})")
jbl_stubs(2)
r = jbl_run({"MOSS_JBL_MAC": "00:00:00:00:00:00"})
pa = pa_log.read_text().splitlines()
check(r.returncode == 0 and bt_log.read_text().splitlines() == ["connect 00:00:00:00:00:00"] * 2 and sl_log.read_text().split() == ["5"],
      f"bluetoothctl fails then connects: exit 0 after two tries, one 5 s wait between ({r.returncode}, {bt_log.read_text().splitlines()}, sleep {sl_log.read_text().split()})")
check("set-default-sink bluez_sink.00_00_00_00_00_00.a2dp_sink" in pa and not any("volume" in ln for ln in pa),
      f"the speaker's bluez sink becomes the default sink, the volume is never touched: {pa}")
(work / "state").mkdir()
(work / "state" / "jbl.env").write_text('MOSS_JBL_MAC="00:00:00:00:00:00"\n')
jbl_stubs(1)
r = jbl_run({})
check(r.returncode == 0 and bt_log.read_text().splitlines() == ["connect 00:00:00:00:00:00"] and sl_log.read_text() == "",
      f"the address read from $MOSS_STATE_DIR/jbl.env (quoted): connected on the first try ({r.returncode}: {r.stderr.strip()[:90]})")
jbl_stubs(0)
r = jbl_run({"MOSS_JBL_MAC": "00:00:00:00:00:00", "MOSS_JBL_TRIES": "3"})
check(r.returncode == 1 and "gave up after 3 tries" in r.stderr and "page-timeout" in r.stderr
      and len(bt_log.read_text().splitlines()) == 3 and sl_log.read_text().split() == ["5", "5"] and pa_log.read_text() == "",
      f"a speaker that never answers: exit 1 after MOSS_JBL_TRIES=3 with the reason ({r.returncode}: {r.stderr.strip()[:120]})")
jbl_stubs(1, sink=False)
r = jbl_run({"MOSS_JBL_MAC": "00:00:00:00:00:00", "MOSS_JBL_TRIES": "3"})
pa = pa_log.read_text().splitlines()
check(r.returncode == 1 and "gave up after 3 tries: connected, but no bluez sink" in r.stderr
      and bt_log.read_text().splitlines() == ["connect 00:00:00:00:00:00"] * 3 and sl_log.read_text().split() == ["5", "5"]
      and pa == ["list short sinks"] * 3,
      f"connected but the sink never appears (the link dropped): every try connects again, exit 1 after 3 "
      f"({r.returncode}: {r.stderr.strip()[:90]}; bluetoothctl x{len(bt_log.read_text().splitlines())}, pactl {pa})")
shutil.rmtree(work, ignore_errors=True)

print(f"\n{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
