#!/usr/bin/env bash
# MOSS at the gamepad: gates -> stack -> "you can drive". Runs ON THE ROVER
# (the onboard computer the pad receiver and the QT Py are plugged into), from
# this checkout, with its .venv:
#
#     tools/moss-teleop.sh            gates, then the stack (pad -> tracks)
#     MOCK=1 tools/moss-teleop.sh     same stack on the fake firmware: no motor
#     tools/moss-teleop.sh stop       stack down (the firmware's own 300 ms
#                                     deadman has already stopped the tracks)
#     tools/moss-teleop.sh log        what the base and the pad said
#
# From another machine: ssh <user>@<rover> 'cd ~/moss && tools/moss-teleop.sh'
#
# Every gate REFUSES rather than guesses: a stack already up,
# no pad, no board, a port held by a bench, a mute firmware, a battery at the
# BMS floor - each is a refusal with the fix on the same line. Nothing moves
# until a stick leaves the deadzone (no deadman by default; MOSS_PAD_DEADMAN=r1
# in robot.env brings the held button back); the stack can stay up with the pad
# on the table.
set -u
cd "$(dirname "$0")/.." || exit 1
# robot.env: what THIS robot is (MOSS_DRIVER, MOSS_PAD_LAYOUT, measured numbers), next to the
# recordings; the flight check reads the same file, so its verdicts match what runs here.
# The shell's own exports win over the file.
ROBOT_ENV="${MOSS_STATE_DIR:-$HOME/.local/state/moss}/robot.env"
if [ -f "$ROBOT_ENV" ]; then
  while IFS= read -r _l || [ -n "$_l" ]; do
    _l="${_l#export }"; case "$_l" in ''|'#'*) continue;; esac
    _k="${_l%%=*}"; _v="${_l#*=}"; _v="${_v%\"}"; _v="${_v#\"}"
    case "$_k" in [A-Z]*) [ -z "${!_k:-}" ] && export "$_k=$_v";; esac
  done < "$ROBOT_ENV"
fi
DIMOS=.venv/bin/dimos
PY=.venv/bin/python
LAUNCH_LOG=/tmp/moss_teleop_launch.log
export GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304   # same as tools/moss-fly.sh (static TLS reserve)
# same bus as tools/moss-fly.sh, under both names (dimOS reads DIMOS_TRANSPORT first)
BUS="${DIMOS_TRANSPORT:-${TRANSPORT:-lcm}}"
export DIMOS_TRANSPORT="$BUS" TRANSPORT="$BUS"

case "${1:-}" in
  stop)
    $DIMOS stop 2>&1 | tail -2
    sleep 1
    pgrep -f "[b]in/dimos( [^ ]+)* run( |$)" >/dev/null && { echo "dimos still up - killing"; pkill -f "[b]in/dimos( [^ ]+)* run( |$)"; }
    echo "stack down - the tracks stopped on the firmware deadman (300 ms) before this line"
    exit 0 ;;
  log)
    $DIMOS log 2>/dev/null | grep -a "MOSS\|[Gg]amepad\|Traceback\|Error" | tail -${2:-30}
    exit 0 ;;
  "") ;;
  *) echo "usage: tools/moss-teleop.sh [stop|log]"; exit 1 ;;
esac

echo "== 0/5 no dimOS stack already up =="
# the real launch line is 'bin/dimos --rerun-open none run ...': root options sit between
# 'dimos' and 'run', so the pattern must allow them (a plain 'bin/dimos run' never matches)
if pgrep -f "[b]in/dimos( [^ ]+)* run( |$)" >/dev/null; then
  echo "A DIMOS STACK IS ALREADY RUNNING - tools/moss-teleop.sh stop first"; exit 1
fi
echo "  none"

echo "== 1/5 gamepad =="
# any /dev/input/js*: the pad on its cable (an Xbox 360 pad to the kernel) keeps the number after the
# dongle's, and SDL numbers the pads it can see from 0 whatever the js number (MOSS_PAD_INDEX picks)
if ! ls /dev/input/js* >/dev/null 2>&1; then
  echo "NO PAD (no /dev/input/js*) - receiver or cable plugged into the rover? if plugged but invisible: replug it, else sudo reboot"; exit 1
fi
echo "  pad(s): $(for j in /dev/input/js*; do n=$(basename $j); printf '%s=%s ' "$n" "$(cat /sys/class/input/$n/device/name 2>/dev/null | tr ' ' '_')"; done)(dongle mute: button UNDER the pad, then move a stick; cable: MOSS_PAD_LAYOUT=xinput)"

echo "== 2/5 MOSS board =="
PORT=$(ls /dev/serial/by-id/*QT_Py* 2>/dev/null | head -1)
if [ "${MOCK:-0}" = "1" ]; then
  echo "  MOCK=1: fake firmware, the real board is not touched"
else
  [ -n "$PORT" ] || { echo "NO QT PY on USB (/dev/serial/by-id/*QT_Py*) - cable, power, or the board is in bootloader"; exit 1; }
  echo "  $PORT"
  HOLDER=$(fuser "$PORT" 2>/dev/null)
  [ -z "$HOLDER" ] || { echo "PORT HELD by pid $HOLDER (a bench or watcher) - stop it first"; exit 1; }
fi

echo "== 3/5 firmware answers =="
if [ "${MOCK:-0}" != "1" ]; then
  $PY - "$PORT" <<'EOF' || exit 1
import json, sys, time, serial
port = sys.argv[1]
try:
    s = serial.Serial(port, 115200, timeout=0.05)
except Exception as exc:
    print(f"CANNOT OPEN {port}: {exc!r} - dialout group? another process?"); sys.exit(1)
time.sleep(0.3); s.reset_input_buffer()
t0 = time.time(); last = None; buf = b""
while time.time() - t0 < 1.5:
    buf += s.read(4096)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        try:
            last = json.loads(line)
        except Exception:
            pass
s.close()
if not last:
    print("FIRMWARE MUTE (no telemetry in 1.5 s) - power-cycle the board, or reflash it: firmware/MOSS_QTPY_MDD3A/bench/flash_qtpy.sh"
          " (esptool from the 'firmware' extra, and a compiled build: see that folder's README.md)"); sys.exit(1)
mode, reason, mv = last.get("mode"), last.get("reason"), last.get("bus_mV")
print(f"  firmware {last.get('fw')} mode={mode} ({reason}) battery={mv} mV ina_ok={last.get('ina_ok')}")
if mode not in (0,):
    print(f"FIRMWARE NOT DISARMED (mode {mode}) - something else is driving it"); sys.exit(1)
if last.get("ina_ok") and mv is not None and int(mv) < 10800:
    print(f"BATTERY {mv} mV - under 10.8 V the BMS cuts at 10.2 V mid-drive: recharge first"); sys.exit(1)
EOF
else
  echo "  (mock)"
fi

echo "== 4/5 stack =="
: > "$LAUNCH_LOG"
if [ "${MOCK:-0}" = "1" ]; then export MOSS_MOCK_BUS=1; else export MOSS_PORT="$PORT"; fi
export MOSS_MAX_PCT="${MOSS_MAX_PCT:-100}"   # firmware limit 100; lower it here for a cautious lap
export MOSS_RAMP_MS="${MOSS_RAMP_MS:-5 5}"       # firmware ramp, ms per % up/down (the pad slew still takes 0.75 s to full)
nohup $DIMOS --rerun-open none run moss-dimos.moss-gamepad --daemon > "$LAUNCH_LOG" 2>&1 < /dev/null &
# "YOU CAN DRIVE" is a commitment: the base connected, the pad connected AND seen at neutral by the
# stack itself ("commands now trusted"). A mute pad (dongle without its pad) connects but is never
# trusted: that is a refusal here, not a banner.
OK_BASE=0; OK_PAD=0; OK_TRUST=0
for i in $(seq 1 40); do
  sleep 1
  L=$($DIMOS log 2>/dev/null | grep -a "MOSS base: connected\|MOSS base: MOCK BUS\|Gamepad connected\|commands now trusted\|MOSS base: cannot open\|MOSS base: no telemetry\|Traceback")
  echo "$L" | grep -q "MOSS base: connected\|MOSS base: MOCK BUS" && OK_BASE=1
  echo "$L" | grep -q "Gamepad connected" && OK_PAD=1
  echo "$L" | grep -q "commands now trusted" && OK_TRUST=1
  echo "$L" | grep -q "cannot open\|no telemetry\|Traceback" && break
  [ $OK_BASE = 1 ] && [ $OK_PAD = 1 ] && [ $OK_TRUST = 1 ] && break
done
$DIMOS log 2>/dev/null | grep -a "MOSS\|[Gg]amepad\|Traceback\|Error" | tail -8
if [ $OK_BASE != 1 ] || [ $OK_PAD != 1 ] || [ $OK_TRUST != 1 ]; then
  echo "STACK NOT READY (base=$OK_BASE pad=$OK_PAD trusted=$OK_TRUST) - see above and $LAUNCH_LOG; stopping"
  [ $OK_PAD = 1 ] && [ $OK_TRUST != 1 ] && echo "  the pad connected but the stack never saw its sticks at neutral: a MUTE pad (dongle without its pad, or off), or the wrong MOSS_PAD_LAYOUT (cable = xinput, dongle = android) in robot.env"
  tail -5 "$LAUNCH_LOG"; $DIMOS stop >/dev/null 2>&1; exit 1
fi

echo "== 5/5 YOU CAN DRIVE (base connected, pad seen at neutral by the stack, layout ${MOSS_PAD_LAYOUT:-android}) =="
if [ -n "${MOSS_PAD_DEADMAN:-}" ] && [ "${MOSS_PAD_DEADMAN}" != "none" ]; then
  echo "  deadman = MOSS_PAD_DEADMAN (${MOSS_PAD_DEADMAN}) held the whole time; release = 0.5 s of zeros then silence"
else
  echo "  no deadman: a stick out of its deadzone drives, centred sticks = 0.5 s of zeros then silence"
fi
echo "  left stick up/down = forward/back   right stick left/right = turn   R2 = boost (capped at ${MOSS_MAX_PCT} %)"
echo "  the firmware stops 300 ms after the last drive"
echo "  stop:  tools/moss-teleop.sh stop      watch:  tools/moss-teleop.sh log"
