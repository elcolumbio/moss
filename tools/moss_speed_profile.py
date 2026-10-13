#!/usr/bin/env python3
"""Speed profile of one straight run (or pivot): drive at a percent for N
seconds, read the encoders all along, print the ticks per half second on
each side, the top rate, the time to reach 90 % of it, and the totals.
Stack down: this opens the motor board itself. Nothing moves without --go.

    tools/moss_speed_profile.py --pct 100 --seconds 5 --go
    tools/moss_speed_profile.py --pct 100 --seconds 3 --pivot --go
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from moss_dimos.moss_adapter import KEEPALIVE_S, MossBaseAdapter, MossGeometry  # noqa: E402

BIN_S = 0.5


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pct", type=int, default=100)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--pivot", action="store_true")
    ap.add_argument("--port", default=None)
    ap.add_argument("--go", action="store_true")
    a = ap.parse_args()
    if not a.go:
        print(f"plan: {'pivot' if a.pivot else 'straight'} at {a.pct} % for {a.seconds:.1f} s; nothing moves without --go")
        return 0
    os.environ["MOSS_PIVOT_PCT"] = "0"
    os.environ["MOSS_BUS_REF_MV"] = "0"
    os.environ["MOSS_ONE_TRACK_PIVOT"] = "0"
    kw = {"geometry": MossGeometry(mps_at_limit=1.0, max_pct=100, min_pct=0)}
    if a.port:
        kw["address"] = a.port
    adapter = MossBaseAdapter(**kw)
    if not adapter.connect():
        print("cannot connect to the firmware")
        return 2
    samples: list[tuple[float, int, int, int | None]] = []
    try:
        t_wait = time.monotonic()
        while (adapter.telemetry is None or "ticks" not in adapter.telemetry) and time.monotonic() - t_wait < 3.0:
            time.sleep(0.05)
        if not adapter.read_enabled() and not adapter.write_enable(True):
            print("cannot arm")
            return 2
        twist = [0.0, 0.0, a.pct / 100.0 / (adapter._geometry.track_m / 2.0)] if a.pivot else [a.pct / 100.0, 0.0, 0.0]
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < a.seconds:
            adapter.write_velocities(twist)
            time.sleep(KEEPALIVE_S)
            tel = adapter.telemetry
            if tel and "ticks" in tel:
                row = (time.monotonic() - t0, int(tel["ticks"][0]), int(tel["ticks"][1]), tel.get("bus"))
                if last is None or row[1:3] != last[1:3]:
                    samples.append(row)
                    last = row
        adapter.write_stop()
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 1.5:
            time.sleep(0.1)
            tel = adapter.telemetry
            if tel and "ticks" in tel:
                row = (time.monotonic() - t0, int(tel["ticks"][0]), int(tel["ticks"][1]), tel.get("bus"))
                if row[1:3] != samples[-1][1:3] if samples else True:
                    samples.append(row)
    finally:
        try:
            adapter.write_stop()
            adapter.write_enable(False)
        finally:
            adapter.disconnect()
    if len(samples) < 2:
        print("no encoder telemetry during the run")
        return 1
    l0, r0 = samples[0][1], samples[0][2]
    print(f"{'t (s)':>6s} {'left':>7s} {'right':>7s} {'L t/s':>7s} {'R t/s':>7s} {'bus mV':>7s}   (ticks since start; rate over the bin)")
    rates = []
    edge = BIN_S
    prev = samples[0]
    for s in samples[1:]:
        if s[0] >= edge or s is samples[-1]:
            dt = s[0] - prev[0]
            lr = (s[1] - prev[1]) / dt if dt > 0 else 0.0
            rr = (s[2] - prev[2]) / dt if dt > 0 else 0.0
            rates.append((s[0], lr, rr))
            print(f"{s[0]:6.2f} {s[1] - l0:7d} {s[2] - r0:7d} {lr:7.0f} {rr:7.0f} {str(s[3] or ''):>7s}")
            prev = s
            edge += BIN_S
    top = max((max(lr, rr) for _, lr, rr in rates), default=0.0)
    t90 = next((t for t, lr, rr in rates if max(lr, rr) >= 0.9 * top), None)
    tot_l, tot_r = samples[-1][1] - l0, samples[-1][2] - r0
    print(f"total {tot_l} / {tot_r} ticks in {a.seconds:.1f} s at {a.pct} %; top rate {top:.0f} ticks/s"
          + (f", reached 90 % of it at {t90:.1f} s" if t90 is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
