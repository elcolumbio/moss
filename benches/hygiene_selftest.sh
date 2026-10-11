#!/usr/bin/env bash
# Self-test of the public-repo gate (benches/hygiene.sh): it plants one defect
# per rule in scratch files, runs the gate on them and expects each one caught,
# under its own rule. It also expects the allowed tokens to pass, and the gate
# to refuse (exit 2) a path that does not exist or a private pattern that does
# not compile, instead of printing "clean".
#
#     benches/hygiene_selftest.sh
#
# One line per check, then "N OK, M KO" and TEST PASSED / TEST FAILED; exit 1
# on any KO. benches/run_all.sh runs it with the benches. Every planted string
# is built from pieces at run time, so this file itself passes the gate.
set -u
export LC_ALL=C.UTF-8
cd "$(dirname "$0")/.." || exit 2
GATE=benches/hygiene.sh
T=$(mktemp -d) || exit 2
trap 'rm -rf "$T"' EXIT
OK=0; KO=0
check() {   # $1 label, $2 1 = pass, $3 detail shown on a failure
  if [ "$2" = "1" ]; then OK=$((OK + 1)); echo "  OK  $1"
  else KO=$((KO + 1)); echo "  KO  $1${3:+ - $3}"; fi
}

# A private list: one made-up name and the allowance of a copyright line.
NAME="Zor""blax"
printf '%s\n' "# a test list" "$NAME" "!^Copyright 2099 $NAME" > "$T/private.txt"
gate() { MOSS_HYGIENE_PRIVATE="$T/private.txt" bash "$GATE" "$@"; }

# rule|file|text: one defect per file, on line 2
V="vec""tor"
S16="0123456789ABCDEF"
s16="0123456789abcdef"
PLANTS=(
  "$V|other_robot|the other robot: ${V^^}"
  "$V|other_package|import ${V}_dimos"
  "date|date|seen on 1""0/1""0"
  "clock|clock|at 16""h04"
  "rule|rule|see Rule"" #2"
  "launcher|launcher|run ./fl""y.sh map"
  "lan|lan_192|the rover at 192"".168.1.20"
  "lan|lan_10|over the tunnel at 10"".8.0.3"
  "home|home|cd /home""/someone/x"
  "device|mac_colon|mac a2:3c:9e"":11:47:05"
  "device|mac_dash|mac A2-3C-9E""-11-47-05"
  "device|serial_lower|usb-Bridge_${s16}${s16}-if00"
  "device|serial_upper|usb-Bridge_${S16}${S16}-if00"
  "accent|accent|caf"$'\xc3\xa9'
  "private|private|hello $NAME"
)
mkdir "$T/plant"
for p in "${PLANTS[@]}"; do
  IFS='|' read -r _rule file text <<< "$p"
  printf 'line one\n%s\n' "$text" > "$T/plant/$file.txt"
done
out=$(gate "$T/plant"); rc=$?
check "planted tree: exit 1" "$([ "$rc" = 1 ] && echo 1)" "rc $rc"
for p in "${PLANTS[@]}"; do
  IFS='|' read -r rule file text <<< "$p"
  n=$(printf '%s\n' "$out" | grep -c -F "$T/plant/$file.txt:2: [$rule]")
  check "caught under [$rule]: $file" "$([ "$n" = 1 ] && echo 1)" "$n hit(s) for it"
done
total=$(printf '%s\n' "$out" | grep -c -F "$T/plant/")
check "exactly one hit per planted file (${#PLANTS[@]})" "$([ "$total" = "${#PLANTS[@]}" ] && echo 1)" "$total hits"
check "the summary counts them" "$(printf '%s\n' "$out" | grep -q -F "hygiene: ${#PLANTS[@]} hit(s)" && echo 1)" \
      "$(printf '%s\n' "$out" | tail -1)"

# Allowed tokens: none of these is a hit.
ZERO_MAC="00:00:00:00"":00:00"
ZERO_DASH="00-00-00-00""-00-00"
Z16="0000000000000000"
cat > "$T/allowed.txt" <<ALLOWED
Vector3 and LcmVector3 are dimOS types; a vectorised loop
/home/x/moss is the placeholder home; tools/moss-fly.sh is the launcher
usb-Adafruit_QT_Py_${ZERO_MAC}-if00 and ${ZERO_DASH}
usb-Silicon_Labs_CP2102N_${Z16}${Z16}-if00-port0
a sha: ${s16}${s16}${s16}${s16}
36/36 sectors, 127.0.0.1, 8.8.8.8
Copyright 2099 $NAME
ALLOWED
out=$(gate "$T/allowed.txt"); rc=$?
check "allowed tokens: clean, exit 0" "$([ "$rc" = 0 ] && echo 1)" "rc $rc: $(printf '%s' "$out" | head -3 | tr '\n' ' ')"

# The gate fails closed.
gate "$T/no_such_dir" > /dev/null 2>&1; rc=$?
check "a path that does not exist -> exit 2, never clean" "$([ "$rc" = 2 ] && echo 1)" "rc $rc"
printf '%s\n' "$NAME(" > "$T/broken.txt"
MOSS_HYGIENE_PRIVATE="$T/broken.txt" bash "$GATE" "$T/plant/private.txt" > /dev/null 2>&1; rc=$?
check "a private pattern that does not compile -> exit 2" "$([ "$rc" = 2 ] && echo 1)" "rc $rc"
MOSS_HYGIENE_PRIVATE="$T/missing.txt" bash "$GATE" "$T/allowed.txt" > /dev/null 2>&1; rc=$?
check "a private list that cannot be read -> exit 2" "$([ "$rc" = 2 ] && echo 1)" "rc $rc"

echo "$OK OK, $KO KO"
if [ "$KO" -eq 0 ]; then echo "TEST PASSED"; exit 0; fi
echo "TEST FAILED"; exit 1
