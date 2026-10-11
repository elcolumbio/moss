#!/bin/bash
# Start the servo watcher in the background (kill any previous one by pidfile, never by pattern).
# servo_watch.py sits next to this script; its log, pid and error files go to WATCH_DIR
# (default: <MOSS state dir>/servo_watch, outside the checkout). Arguments are the servo IDs.
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${PY:-$(cd "$HERE/../../.." && pwd)/.venv/bin/python}"   # this checkout's venv (needs pyserial); PY=... picks another
[ -x "$PY" ] || { echo "watch_start: no python at $PY - make the checkout's .venv (Software.md), or PY=<python with pyserial>" >&2; exit 1; }
WATCH_DIR="${WATCH_DIR:-${MOSS_STATE_DIR:-$HOME/.local/state/moss}/servo_watch}"
mkdir -p "$WATCH_DIR" && cd "$WATCH_DIR" || { echo "watch_start: cannot use $WATCH_DIR" >&2; exit 1; }
[ -f watch.pid ] && kill "$(cat watch.pid)" 2>/dev/null
rm -f watch.log
nohup "$PY" "$HERE/servo_watch.py" watch.log "$@" >watch.err 2>&1 &
echo $! > watch.pid
sleep 2; cat watch.log 2>/dev/null
echo "pid $(cat watch.pid) alive: $(kill -0 "$(cat watch.pid)" 2>/dev/null && echo yes || echo no) - log $WATCH_DIR/watch.log, errors $WATCH_DIR/watch.err"
