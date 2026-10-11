#!/usr/bin/env python3
"""Find the static-friction floor of the tracks: the lowest PWM percent that
turns BOTH tracks. The planner's slowest commands map to a few percent, below
what the geared motors need to start, so the base adapter raises any non-zero
command to MOSS_MIN_PCT; this tool measures that number.

It steps the percent up (both tracks forward), holds each step, reads the
encoder ticks before and after, and reports the first step where both sides
moved. It prints the value to export; it writes nothing.

Run it with the stack DOWN (it opens the motor board itself), the rover on
blocks or with a clear metre ahead: every step rolls it forward a little.

    tools/moss_floor_probe.py              # the plan only, nothing moves
    tools/moss_floor_probe.py --go         # drive the steps
    tools/moss_floor_probe.py --go --all   # keep stepping past the first success

Options: --from 20 --to 60 --step 5 (percent), --hold 1.0 (s of drive per
step), --settle 1.0 (s of stop before reading the ticks), --min-ticks 20
(encoder ticks that count as motion), --port <by-id path> (default: the QT Py
by-id link or MOSS_PORT), --mock (the fake firmware, no motor).
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos.moss_adapter import KEEPALIVE_S, MossBaseAdapter, MossGeometry  # noqa: E402

STACK_RE = r"[b]in/dimos( [^ ]+)* run( |$)"
TELEMETRY_WAIT_S = 3.0


def plan(start: int, stop: int, step: int) -> list[int]:
    """The percents to try, start..stop inclusive, clamped to 1..100."""
    if step <= 0:
        raise ValueError("step must be positive")
    start, stop = max(1, min(100, start)), max(1, min(100, stop))
    return list(range(start, stop + 1, step)) if start <= stop else []


def judge(rows: list[dict], min_ticks: int) -> int | None:
    """rows: [{"pct": int, "left": ticks moved, "right": ticks moved}, ...] in
    ascending percent. The floor is the first percent where BOTH sides moved by
    at least min_ticks; None when no row qualifies."""
    for r in rows:
        if abs(int(r["left"])) >= min_ticks and abs(int(r["right"])) >= min_ticks:
            return int(r["pct"])
    return None


def stack_running() -> bool:
    try:
        out = subprocess.run(["pgrep", "-f", STACK_RE], capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return False
    return bool(out.strip())


def _ticks(adapter: MossBaseAdapter) -> list[int] | None:
    t = adapter.telemetry
    if not t or "ticks" not in t:
        return None
    return [int(t["ticks"][0]), int(t["ticks"][1])]


def _fresh_ticks(adapter: MossBaseAdapter, timeout: float) -> list[int] | None:
    """Ticks from a telemetry line younger than the call."""
    deadline = time.monotonic() + timeout
    t0 = time.monotonic()
    while time.monotonic() < deadline:
        age = adapter.telemetry_age_s
        if age is not None and age < (time.monotonic() - t0) + 0.05:
            return _ticks(adapter)
        time.sleep(0.05)
    return _ticks(adapter)


def run(adapter: MossBaseAdapter, pcts: list[int], hold: float, settle: float,
        min_ticks: int, stop_at_first: bool, say=print, pivot: bool = False) -> list[dict]:
    """Drive each step and read the ticks around it. Returns the rows; the
    adapter is stopped before returning, whatever happens."""
    rows: list[dict] = []
    try:
        for pct in pcts:
            before = _fresh_ticks(adapter, TELEMETRY_WAIT_S)
            if before is None:
                say("no telemetry from the firmware: is it the QT Py port, and is the firmware flashed?")
                break
            # the adapter refuses a non-zero drive until the firmware is armed through it
            if not adapter.read_enabled() and not adapter.write_enable(True):
                say("the firmware refused 'arm' (see the lines above)")
                break
            t_end = time.monotonic() + hold
            ok = True
            while time.monotonic() < t_end:
                # geometry: mps_at_limit 1.0 and max_pct 100, so vx 0.35 m/s = 35 %; a pivot
                # at p % is wz = p/100 / (track/2) (left -p, right +p)
                twist = [0.0, 0.0, pct / 100.0 / (adapter._geometry.track_m / 2.0)] if pivot else [pct / 100.0, 0.0, 0.0]
                ok = adapter.write_velocities(twist) and ok
                time.sleep(0.2)
            adapter.write_stop()
            time.sleep(settle)
            after = _fresh_ticks(adapter, TELEMETRY_WAIT_S) or before
            row = {"pct": pct, "left": after[0] - before[0], "right": after[1] - before[1], "sent": ok}
            rows.append(row)
            mark = "BOTH" if abs(row["left"]) >= min_ticks and abs(row["right"]) >= min_ticks else (
                "left" if abs(row["left"]) >= min_ticks else ("right" if abs(row["right"]) >= min_ticks else "-"))
            say(f"  {pct:3d} %  left {row['left']:+6d}  right {row['right']:+6d}  ticks  -> {mark}"
                + ("" if ok else "  (a drive was refused)"))
            if stop_at_first and mark == "BOTH":
                break
    finally:
        try:
            adapter.write_stop()
            adapter.write_enable(False)      # 'stop': disarmed, nothing moves after this tool
        except Exception:  # noqa: BLE001
            pass
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--go", action="store_true", help="drive the steps (without it: print the plan)")
    ap.add_argument("--all", action="store_true", help="keep stepping past the first success")
    ap.add_argument("--from", dest="start", type=int, default=20)
    ap.add_argument("--to", dest="stop", type=int, default=60)
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--min-ticks", type=int, default=20)
    ap.add_argument("--port", default=None)
    ap.add_argument("--mock", action="store_true", help="the fake firmware, no motor")
    ap.add_argument("--pivot", action="store_true", help="turn on the spot (left back, right forward) instead of driving straight")
    a = ap.parse_args(argv)

    pcts = plan(a.start, a.stop, a.step)
    print(f"floor probe ({'pivot on the spot' if a.pivot else 'straight'}): {len(pcts)} steps {pcts[0] if pcts else '-'}..{pcts[-1] if pcts else '-'} % "
          f"(hold {a.hold:.1f} s, settle {a.settle:.1f} s, motion = {a.min_ticks} ticks)")
    if not a.go:
        print("nothing moves without --go. Put the rover on blocks or give it a clear metre ahead.")
        return 0
    if not a.mock and stack_running():
        print("a dimOS stack is running: stop it first (tools/moss-fly.sh stop), this tool opens the motor board itself")
        return 2
    os.environ["MOSS_PIVOT_PCT"] = "0"        # the percent printed is the percent sent, pivots included
    kw = {"geometry": MossGeometry(mps_at_limit=1.0, max_pct=100, min_pct=0)}
    if a.mock:
        os.environ["MOSS_MOCK_BUS"] = "1"
    elif a.port:
        kw["address"] = a.port
    adapter = MossBaseAdapter(**kw)
    if not adapter.connect():
        print("cannot connect to the firmware (see the lines above)")
        return 2
    try:
        rows = run(adapter, pcts, a.hold, a.settle, a.min_ticks, stop_at_first=not a.all, pivot=a.pivot)
    finally:
        adapter.disconnect()
    if getattr(adapter, "_pulse_n", 0):
        print(f"pulse mode ({adapter._pulse_n} slices of {KEEPALIVE_S * 1000:.0f} ms, ramp {adapter._ramp or 'firmware default'}): "
              "the percent is a duty of full-power slices, the table above is the ticks per step; "
              "MOSS_MIN_PCT does not apply (MOSS_PULSE_MS=0 to measure a floor instead)")
        return 0 if rows else 1
    floor = judge(rows, a.min_ticks)
    if floor is None:
        print(f"no step up to {pcts[-1] if pcts else '-'} % turned both tracks: raise --to, or check the drive line")
        return 1
    print(f"both tracks turn from {floor} %: export MOSS_MIN_PCT={floor} (or launch with MOSS_MIN_PCT={floor} ...)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
