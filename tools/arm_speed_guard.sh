#!/usr/bin/env bash
# Arm the speed watchdog (tools/speed_guard.py) for THIS run, on the rover:
#
#     tools/arm_speed_guard.sh [RUN_DIR]       RUN_DIR: the run's dimOS log dir (tools/moss-fly.sh knows it);
#                                              without it the watchdog takes the newest run dir
#
# NEVER "start one only if pgrep finds none": the previous run's watchdog
# survives its run (`dimos stop` only sweeps the processes of its own run) and
# its mere presence would satisfy that check while it tails a log that no
# longer grows - "speed watchdog armed" on screen, the envelope unenforced on
# the floor. The handle is the pid file speed_guard.py stamps at startup:
# kill THAT pid, start a fresh watchdog, then prove the pid file names a
# process born AFTER that launch. No proof -> non-zero, and the caller must not
# explore. A ceiling (MOSS_MAX_SPEED_MPS) that is not above the planner's
# cruise makes the watchdog refuse to start, so arming fails the same way.
set -u
cd "$(dirname "$0")/.." || exit 1

PID_FILE="${SPEED_GUARD_PID_FILE:-/tmp/moss_speed_guard.pid}"
GUARD_PY="${SPEED_GUARD_PY:-tools/speed_guard.py}"
PYTHON="${SPEED_GUARD_PYTHON:-.venv/bin/python}"
LOG="${SPEED_GUARD_LOG:-/tmp/moss_speed_guard.log}"
DEADLINE_S="${SPEED_GUARD_ARM_DEADLINE_S:-15}"   # the fresh watchdog must show up within this
TICK=$(getconf CLK_TCK)
# the watchdog's STOP rides the run's bus (explore_ctl reads DIMOS_TRANSPORT,
# then TRANSPORT): the run's setting must reach the python we launch
BUS="${DIMOS_TRANSPORT:-${TRANSPORT:-lcm}}"
export DIMOS_TRANSPORT="$BUS" TRANSPORT="$BUS"
if [ "$#" -gt 0 ] && [ -n "$1" ]; then
  [ -d "$1" ] || { echo "RUN DIR $1 IS NOT A DIRECTORY - not arming"; exit 1; }
  export SPEED_GUARD_RUN_DIR="$1"
fi

alive() {   # a zombie still answers kill -0 and still tails nothing: it is not alive
  local rest state
  rest=$(cut -d')' -f2- "/proc/$1/stat" 2>/dev/null) || return 1
  read -r state _ <<<"$rest"
  [ "$state" != "Z" ]
}

is_guard() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q "speed_guard\.py"; }

born_s() {   # seconds since boot at which pid $1 was born (/proc/<pid>/stat field 22)
  local rest
  local -a f
  rest=$(cut -d')' -f2- "/proc/$1/stat" 2>/dev/null) || return 1
  read -r -a f <<<"$rest"        # f[0] is the state, so field 22 overall is f[19]
  [ "${#f[@]}" -ge 20 ] || return 1
  echo $(( f[19] / TICK ))
}

# 1. retire the previous run's watchdog, by the pid IT left behind
OLD=$(cat "$PID_FILE" 2>/dev/null || true)
case "$OLD" in
  '') ;;
  *[!0-9]*) echo "  $PID_FILE holds no pid ('$OLD') - ignored" ;;
  *)
    if ! alive "$OLD"; then
      echo "  previous watchdog (pid $OLD) already gone - stale pid file"
    elif ! is_guard "$OLD"; then
      echo "  pid $OLD is not a speed guard (pid reused) - NOT killed"
    else
      kill "$OLD" 2>/dev/null
      for _ in $(seq 1 50); do alive "$OLD" || break; sleep 0.1; done
      if alive "$OLD"; then
        kill -9 "$OLD" 2>/dev/null
        for _ in $(seq 1 20); do alive "$OLD" || break; sleep 0.1; done
      fi
      if alive "$OLD"; then
        echo "PREVIOUS WATCHDOG (pid $OLD) WILL NOT DIE - not arming over it"
        exit 1
      fi
      echo "  previous watchdog (pid $OLD) retired"
    fi ;;
esac
rm -f "$PID_FILE"      # only the NEW watchdog may satisfy the check below

# 2. launch the fresh one, keeping the mark it must postdate
read -r UP _ < /proc/uptime
LAUNCH_S="${UP%%.*}"                       # seconds since boot, taken BEFORE the launch
nohup "$PYTHON" "$GUARD_PY" > "$LOG" 2>&1 < /dev/null &

# 3. the pid file must now name a live process born after that mark
NEW=""
END=$((SECONDS + DEADLINE_S))
while [ "$SECONDS" -lt "$END" ]; do
  P=$(cat "$PID_FILE" 2>/dev/null || true)
  case "$P" in
    ''|*[!0-9]*) sleep 0.2; continue ;;
  esac
  if alive "$P"; then
    B=$(born_s "$P" || true)
    if [ -n "$B" ] && [ "$B" -ge "$LAUNCH_S" ]; then NEW="$P"; break; fi
  fi
  sleep 0.2
done
if [ -z "$NEW" ] || ! alive "$NEW"; then
  echo "SPEED WATCHDOG DID NOT COME UP: $PID_FILE names no live process born after the launch (${DEADLINE_S} s) - see $LOG"
  tail -3 "$LOG" 2>/dev/null | sed 's/^/  /'
  exit 1
fi
# the ceiling the watchdog enforces, said the way it reads it (the profile, MOSS_MAX_SPEED_MPS)
LIMIT=$("$PYTHON" -c 'from moss_dimos import robot_profile; print(robot_profile.current().max_speed_mps)' 2>/dev/null || echo "?")
echo "speed watchdog armed (stops the run beyond ${LIMIT} m/s) - pid $NEW, born after this launch"
