#!/usr/bin/env bash
# Connects the Bluetooth speaker at boot and makes it the default output of the
# robot user's audio session, so the sound pack plays on it after a reboot.
# Run by tools/moss-jbl.service (a systemd user unit); by hand it does the same:
#
#     tools/moss-jbl-connect.sh
#
# The speaker must be paired and trusted once (bluetoothctl pair / trust). Its
# address comes from MOSS_JBL_MAC in the environment, else from the line
# MOSS_JBL_MAC=<address> in $MOSS_STATE_DIR/jbl.env (default
# ~/.local/state/moss/jbl.env). This file carries no address.
#
# Up to MOSS_JBL_TRIES tries (default 120), 5 s between tries: `bluetoothctl connect`
# (cut at 20 s), until it answers "Connection successful"; then the speaker's
# PulseAudio sink (bluez_sink.<address> or bluez_output.<address>, from `pactl list
# short sinks`) becomes the default sink. A try that connects but finds no sink
# connects again on the next one (the link can drop before its sink appears). The
# sink's volume is left as PulseAudio remembers it. Exit 0 once the default sink is
# set, 1 after the last try (with the reason), 2 when it cannot run (no address, a
# malformed one, a missing tool). The waits alone add up to about ten minutes; with
# the speaker off each connect also waits out its page timeout, about twenty minutes.
set -u

STATE_DIR="${MOSS_STATE_DIR:-$HOME/.local/state/moss}"
MAC="${MOSS_JBL_MAC:-}"
if [ -z "$MAC" ] && [ -r "$STATE_DIR/jbl.env" ]; then
  MAC=$(sed -n 's/^[[:space:]]*MOSS_JBL_MAC=//p' "$STATE_DIR/jbl.env" | tail -n 1 | tr -d "\"' \t\r")
fi
if [ -z "$MAC" ]; then
  echo "moss-jbl: no speaker address: set MOSS_JBL_MAC or write MOSS_JBL_MAC=<address> into $STATE_DIR/jbl.env" >&2
  exit 2
fi
if ! printf '%s' "$MAC" | grep -qE '^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$'; then
  echo "moss-jbl: MOSS_JBL_MAC is not a Bluetooth address (six hex pairs separated by colons): $MAC" >&2
  exit 2
fi
for tool in bluetoothctl pactl timeout; do
  command -v "$tool" >/dev/null 2>&1 || { echo "moss-jbl: $tool is not on the PATH" >&2; exit 2; }
done

TRIES="${MOSS_JBL_TRIES:-120}"
case "$TRIES" in ''|*[!0-9]*) TRIES=120 ;; esac
[ "$TRIES" -ge 1 ] || TRIES=1
WAIT_S=5
SINK_KEY="${MAC//:/_}"
SINK_KEY="${SINK_KEY^^}"

connected=0
reason="no answer"
for try in $(seq 1 "$TRIES"); do
  if [ "$connected" -eq 0 ]; then
    # a hung bluetoothctl ends here: the loop always moves on
    out=$(timeout 20 bluetoothctl connect "$MAC" 2>&1)
    if printf '%s\n' "$out" | grep -qE 'Connection successful|AlreadyConnected'; then
      connected=1
      echo "moss-jbl: $MAC connected (try $try of $TRIES)"
    else
      reason=$(printf '%s\n' "$out" | grep -v '^[[:space:]]*$' | tail -n 1)
      reason="bluetoothctl: ${reason:-no output}"
    fi
  fi
  if [ "$connected" -eq 1 ]; then
    # the sink appears a moment after the link
    sink=$(timeout 10 pactl list short sinks 2>/dev/null | awk -v key="$SINK_KEY" \
             'toupper($2) ~ ("^BLUEZ_(SINK|OUTPUT)\\." key "([.]|$)") { print $2; exit }')
    if [ -n "$sink" ]; then
      if timeout 10 pactl set-default-sink "$sink"; then
        echo "moss-jbl: default sink $sink"
        exit 0
      fi
      reason="pactl set-default-sink $sink failed"
    else
      reason="connected, but no bluez sink for $MAC in pactl list short sinks"
      # connect again on the next try: the link may have dropped (PulseAudio restarted and
      # unregistered its A2DP endpoints); a speaker still connected answers success again
      connected=0
    fi
  fi
  [ "$try" -lt "$TRIES" ] && sleep "$WAIT_S"
done
echo "moss-jbl: gave up after $TRIES tries: $reason" >&2
exit 1
