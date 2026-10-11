#!/usr/bin/env bash
# Run every bench (benches/bench_*.py, plus the public-repo gate's self-test
# benches/hygiene_selftest.sh) and print one line per bench.
#
#     benches/run_all.sh                     with .venv/bin/python
#     benches/run_all.sh /path/to/python     with another interpreter
#     LIVE=1 benches/run_all.sh ...          also the live benches (bench_*_live.py)
#
# A live bench talks on the host's real LCM bus (it needs loopback multicast on
# this host, and two runs at once hear each other): it is SKIPPED with a line
# unless LIVE=1. Each bench runs from the repository root with PYTHONPATH=. (a
# PYTHONPATH already set is kept after it), no bytecode written, and never
# under PYTHONOPTIMIZE (some benches check with assert, which -O removes). A
# bench passes when it exits 0; on a failure its last lines are printed under
# its line. Exit 1 if any bench failed (or none was found), 0 otherwise.
set -u
unset PYTHONOPTIMIZE
cd "$(dirname "$0")/.." || exit 2
PY="${1:-.venv/bin/python}"
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "no python at $PY - pass one: benches/run_all.sh /path/to/python"; exit 2
fi
LOGS=$(mktemp -d) || exit 2
trap 'rm -rf "$LOGS"' EXIT
FAILED=0; N=0; SKIPPED=0
for bench in benches/bench_*.py benches/*_selftest.sh; do
  [ -e "$bench" ] || continue
  name=$(basename "$bench"); name=${name%.*}
  case "$name" in
    *_live)
      if [ "${LIVE:-0}" != "1" ]; then
        printf 'SKIPPED %-40s live bench (real LCM bus): LIVE=1 runs it\n' "$name"
        SKIPPED=$((SKIPPED + 1)); continue
      fi ;;
  esac
  N=$((N + 1))
  t0=$(date +%s)
  case "$bench" in
    *.sh) bash "$bench" > "$LOGS/$name.log" 2>&1 ;;
    *) PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=".${PYTHONPATH:+:$PYTHONPATH}" "$PY" "$bench" > "$LOGS/$name.log" 2>&1 ;;
  esac
  rc=$?
  dt=$(( $(date +%s) - t0 ))
  counts=$(grep -E '^[0-9]+ OK, [0-9]+ KO$' "$LOGS/$name.log" | tail -1)
  if [ "$rc" -eq 0 ]; then
    printf 'PASSED  %-40s %s (%d s)\n' "$name" "${counts:-exit 0}" "$dt"
  else
    FAILED=$((FAILED + 1))
    printf 'FAILED  %-40s %s (exit %d, %d s)\n' "$name" "${counts:-no count line}" "$rc" "$dt"
    tail -n 8 "$LOGS/$name.log" | sed 's/^/        | /'
  fi
done
if [ "$N" -eq 0 ]; then
  echo "no bench found under benches/"; exit 1
fi
summary="$((N - FAILED))/$N benches passed"
[ "$SKIPPED" -gt 0 ] && summary="$summary, $SKIPPED live skipped (LIVE=1 runs them)"
echo "$summary"
[ "$FAILED" -eq 0 ]
