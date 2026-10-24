#!/usr/bin/env bash
# MOSS drives from the pad as soon as the rover is powered: tools/moss-teleop.service runs
# this at boot. It runs tools/moss-teleop.sh and retries until its gates pass - at boot the
# receiver may enumerate after the first try, the pad may be switched on later, the motor
# board may still be starting. A dimOS stack already up (the cockpit's, or a previous try)
# counts as done. Once a stack is up this exits: a stack stopped by hand (tools/moss-teleop.sh
# stop) stays stopped until the next boot, so benches and the cockpit's own launches keep
# the rover to themselves.
#
#     tools/moss-teleop-boot.sh                  retry every 15 s (MOSS_TELEOP_BOOT_RETRY_S)
#     journalctl -u moss-teleop -b               what each try said
#     sudo systemctl disable --now moss-teleop   no more teleop at boot
set -u
cd "$(dirname "$0")/.." || exit 1
STACK_RE='[b]in/dimos( [^ ]+)* run( |$)'   # the same pattern as tools/moss-fly.sh and the cockpit
EVERY="${MOSS_TELEOP_BOOT_RETRY_S:-15}"
n=0
while :; do
  n=$((n + 1))
  if pgrep -f "$STACK_RE" >/dev/null; then
    echo "try $n: a dimOS stack is already up - done"
    exit 0
  fi
  echo "=== try $n  $(date '+%H:%M:%S') ==="
  if tools/moss-teleop.sh; then
    echo "try $n: YOU CAN DRIVE"
    exit 0
  fi
  sleep "$EVERY"
done
