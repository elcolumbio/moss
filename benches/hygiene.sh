#!/usr/bin/env bash
# Public-repo gate: refuses text that does not belong in this repository.
#
#     benches/hygiene.sh                 scan the default paths (below)
#     benches/hygiene.sh <path> ...      scan only these files or directories
#     benches/hygiene_selftest.sh        plants one defect per rule and checks each is caught
#
# Prints every hit as "file:line: [rule] text" and exits 1 on any hit, 0 when
# clean, 2 when it cannot do its job (a path that does not exist, a private
# list it cannot read, or a private pattern that does not compile): a gate that
# cannot look never says clean. Runs from anywhere: paths are taken relative
# to the repository root. This file holds the patterns, so it is the one file
# never scanned.
#
# Rules (case-sensitive unless said otherwise):
#   vector     the word "vector" in any case and spelling, inside identifiers
#              too; the dimOS types Vector3 / LcmVector3 and "vectorised" are allowed
#   date       a dd/mm date
#   clock      a time written hours-h-minutes (two digits after the h)
#   rule       an internal rule number ("Rule #")
#   launcher   a launcher named fly.sh (tools/moss-fly.sh is fine)
#   lan        a private IPv4 address (loopback is fine)
#   home       a path under /home/<name>/ (the placeholder /home/x is fine)
#   device     a MAC-style id (colon or dash, any case) or a 32-hex serial (any
#              case) that is not all zeros (benches use zeros for device identifiers)
#   accent     an accented letter (comments and messages are plain English)
#   private    one extended regex per line from a local file that is never
#              committed: $MOSS_HYGIENE_PRIVATE when set, else
#              .git/info/hygiene-private when it exists. Names, places and
#              device identifiers belong there, not in this script. A line
#              starting with "!" is an allowance instead: a hit whose text
#              matches it is not reported (e.g. the copyright line in NOTICE).
set -u
export LC_ALL=C.UTF-8   # the accent rule reads UTF-8 letters, not bytes
cd "$(dirname "$0")/.." || exit 2

if [ "$#" -gt 0 ]; then
  PATHS=("$@")
  for p in "${PATHS[@]}"; do
    [ -e "$p" ] || { echo "hygiene: no such file or directory: $p (nothing scanned)" >&2; exit 2; }
  done
else
  PATHS=()
  for p in moss_dimos tools benches docs Software.md README.md pyproject.toml NOTICE; do
    [ -e "$p" ] && PATHS+=("$p")
  done
fi

GREP=(grep -rnHI --exclude=hygiene.sh --exclude-dir=__pycache__ --exclude-dir=.git)
HITS=0

report() {   # $1 = rule; stdin = grep lines "file:line:text"
  local line
  while IFS= read -r line; do
    [ -n "$line" ] || continue
    local file=${line%%:*} rest=${line#*:}
    local num=${rest%%:*} text=${rest#*:}
    printf '%s:%s: [%s] %s\n' "$file" "$num" "$1" "$(printf '%s' "$text" | sed -E 's/^[[:space:]]+//' | cut -c1-200)"
    HITS=$((HITS + 1))
  done
}

# Lines of the grep for $2 whose text still matches $2 once $3 (allowed tokens) is removed.
filtered() {   # $1 = grep flag (-i or -E), $2 = pattern, $3 = sed ERE of allowed tokens
  local line text
  "${GREP[@]}" "$1" -e "$2" -- "${PATHS[@]}" 2>/dev/null | while IFS= read -r line; do
    text=${line#*:}; text=${text#*:}
    if printf '%s' "$text" | sed -E "s#$3##g" | grep -q "$1" -e "$2"; then
      printf '%s\n' "$line"
    fi
  done
}

rule() {   # $1 = rule, $2 = ERE
  report "$1" < <("${GREP[@]}" -E -e "$2" -- "${PATHS[@]}" 2>/dev/null)
}

report vector < <(filtered -i 'vector' 'LcmVector3|Vector3|[Vv]ectoris(ed|ing|ation)')
rule date '\b[0-3][0-9]/[01][0-9]\b'
rule clock '\b[0-9]{1,2}h[0-5][0-9]\b'
rule rule 'Rule #'
rule launcher '(^|[^[:alnum:]_-])fly\.sh\b'
rule lan '\b(192\.168|10\.[0-9]{1,3}|172\.(1[6-9]|2[0-9]|3[01]))\.[0-9]{1,3}\.[0-9]{1,3}\b'
report home < <(filtered -E '/home/[A-Za-z0-9_.-]+' '/home/x\b')
# by-id names glue the serial to "_": no \b before it
report device < <(filtered -E '([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}|(^|[^0-9A-Fa-f])[0-9A-Fa-f]{32}([^0-9A-Fa-f]|$)' \
                           '(00[:-]){5}00|(^|[^0-9A-Fa-f])0{32}([^0-9A-Fa-f]|$)')
rule accent '[àâäçéèêëîïôöûùüÀÂÄÇÉÈÊËÎÏÔÖÛÙÜœŒ]'

PRIVATE="${MOSS_HYGIENE_PRIVATE:-}"
if [ -z "$PRIVATE" ]; then
  gitdir=$(git rev-parse --git-dir 2>/dev/null) && [ -f "$gitdir/info/hygiene-private" ] && PRIVATE="$gitdir/info/hygiene-private"
fi
if [ -n "$PRIVATE" ]; then
  if [ ! -r "$PRIVATE" ]; then
    echo "hygiene: cannot read the private list $PRIVATE" >&2; exit 2
  fi
  NPRIV=0
  PATS=()
  ALLOW='$^'   # matches no text line
  while IFS= read -r pat || [ -n "$pat" ]; do
    case "$pat" in
      ''|'#'*) ;;
      '!'*) ALLOW="$ALLOW|${pat#!}" ;;
      *) PATS+=("$pat") ;;
    esac
  done < "$PRIVATE"
  # A pattern grep cannot compile would match nothing, silently: refuse instead.
  for pat in ${PATS[@]+"${PATS[@]}"} "$ALLOW"; do
    printf '' | grep -E -e "$pat" >/dev/null 2>&1
    if [ "$?" -eq 2 ]; then
      echo "hygiene: a pattern of the private list $PRIVATE does not compile: $pat" >&2; exit 2
    fi
  done
  for pat in ${PATS[@]+"${PATS[@]}"}; do
    NPRIV=$((NPRIV + 1))
    report private < <("${GREP[@]}" -E -e "$pat" -- "${PATHS[@]}" 2>/dev/null | while IFS= read -r line; do
                         text=${line#*:}; text=${text#*:}
                         printf '%s' "$text" | grep -qE -e "$ALLOW" || printf '%s\n' "$line"
                       done)
  done
  PRIV_NOTE="private list: $NPRIV pattern(s)"
else
  PRIV_NOTE="private list: none (set MOSS_HYGIENE_PRIVATE or create .git/info/hygiene-private)"
fi

if [ "$HITS" -gt 0 ]; then
  echo "hygiene: $HITS hit(s) in ${PATHS[*]} ($PRIV_NOTE)"
  exit 1
fi
echo "hygiene: clean - ${PATHS[*]} ($PRIV_NOTE)"
exit 0
