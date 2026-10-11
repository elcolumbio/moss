#!/usr/bin/env bash
# Three straight bursts (40, 70, 100 % duty, 1.5 s each, 1 s stop between) with a
# pulse preset, to SEE the motion profile on the floor. Stack down (the tool opens
# the motor board itself), two metres clear ahead.
#
#     tools/moss-pulse-test.sh sec      # 100 ms slices at full duty, hard brake between (period 500 ms)
#     tools/moss-pulse-test.sh moyen    # same period, release over 300 ms instead of a hard brake
#     tools/moss-pulse-test.sh doux     # one cycle per second, release over 500 ms
#     tools/moss-pulse-test.sh doux --from 20 --to 100 --step 20   # any floor-probe option after the preset
#
# The preset is two variables; the same two on the launcher give the stack the
# same profile:  MOSS_PULSE_MS=<ms> MOSS_RAMP_MS="<rise> <fall>" EXPLORE=1 tools/moss-fly.sh explore
set -u
cd "$(dirname "$0")/.." || exit 1
case "${1:-}" in
  sec)   export MOSS_PULSE_MS=500  MOSS_RAMP_MS="1 1" ;;
  moyen) export MOSS_PULSE_MS=500  MOSS_RAMP_MS="1 3" ;;
  doux)  export MOSS_PULSE_MS=1000 MOSS_RAMP_MS="1 5" ;;
  *) echo "usage: tools/moss-pulse-test.sh sec|moyen|doux [floor-probe options]"; exit 2 ;;
esac
shift
echo "preset $MOSS_PULSE_MS ms period, ramp $MOSS_RAMP_MS ms per % (rise fall) -> for the stack: MOSS_PULSE_MS=$MOSS_PULSE_MS MOSS_RAMP_MS=\"$MOSS_RAMP_MS\" EXPLORE=1 tools/moss-fly.sh explore"
exec .venv/bin/python tools/moss_floor_probe.py --go --all --from 40 --to 100 --step 30 --hold 1.5 --settle 1.0 --min-ticks 20 "$@"
