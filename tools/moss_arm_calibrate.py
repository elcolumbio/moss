#!/usr/bin/env python3
"""Calibrate the arm's Feetech servos (STS family, protocol 0, 1 Mbps), the SO-101 way.

One step, on the operator's word (the arm is moved by hand, torque off):
  --sweep       the operator moves every joint from one end stop to the other, then
                stops the sweep (SIGINT / SIGTERM, or --watch seconds). Positions are
                unwrapped on the 0..4095 circle, so a travel that crosses the encoder's
                zero is measured right. For every joint seen: the middle of its travel
                becomes 2048 (Homing_Offset, its sign checked against the servo), the
                end stops become Min/Max_Position_Limit, and all of it is saved.
  --middle      older flow: the arm is held in the middle of every travel and the
                offsets are written so that this pose reads 2048 (no limits).
  --range       older flow: write the raw min/max seen during --watch as the limits.
  --watch N     read positions for N seconds (used by --sweep and --range).
  --show        read what the servos hold now (offset, limits, position).

Registers (STS/SMS control table): Homing_Offset 31 (2 bytes, sign-magnitude, bit 11 =
sign), Min_Position_Limit 9, Max_Position_Limit 11, Lock 55 (0 unlocks the EEPROM),
Present_Position 56, Torque_Enable 40. The calibration is saved as JSON in
config/arm_calibration.json: {joint: {id, homing_offset, range_min, range_max}}.

    tools/moss_arm_calibrate.py --show
    tools/moss_arm_calibrate.py --middle           # arm held in the middle pose
    tools/moss_arm_calibrate.py --range --watch 30 # then sweep every joint for 30 s
"""
import argparse, json, os, sys, time

PORT = os.environ.get("MOSS_ARM_PORT", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D041749-if00")
BAUD = 1000000
JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
CENTRE = 2048
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "arm_calibration.json")


def encode_sign_magnitude(value: int, sign_bit: int = 11) -> int:
    """Feetech stores the homing offset as sign-magnitude: bit `sign_bit` set = negative."""
    mag = abs(int(value))
    if mag >= (1 << sign_bit):
        raise ValueError(f"offset {value} does not fit in {sign_bit} bits")
    return mag | (1 << sign_bit) if value < 0 else mag


def decode_sign_magnitude(raw: int, sign_bit: int = 11) -> int:
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


def open_bus():
    import scservo_sdk as scs
    ph = scs.PortHandler(PORT)
    if not ph.openPort():
        sys.exit(f"cannot open {PORT}")
    ph.setBaudRate(BAUD)
    return scs, ph, scs.PacketHandler(0)


def read2(scs, ph, pk, sid: int, addr: int, tries: int = 6):
    """A two-byte read with retries: the SDK raises on a truncated answer (seen right
    after an EEPROM write), and a bus collision gives a failed result. None when it never came."""
    for _ in range(tries):
        try:
            v, comm, _err = pk.read2ByteTxRx(ph, sid, addr)
            if comm == scs.COMM_SUCCESS:
                return v
        except Exception:  # noqa: BLE001 - a short packet inside the SDK
            pass
        time.sleep(0.03)
    return None


def present(scs, ph, pk):
    out = {}
    for name, sid in JOINTS.items():
        if pk.ping(ph, sid)[1] == scs.COMM_SUCCESS:
            v = read2(scs, ph, pk, sid, 56)
            if v is not None:
                out[name] = v
    return out


def write_eeprom(scs, ph, pk, sid, addr, value, width):
    pk.write1ByteTxRx(ph, sid, 55, 0)
    time.sleep(0.02)
    if width == 1:
        comm, err = pk.write1ByteTxRx(ph, sid, addr, value)
    else:
        comm, err = pk.write2ByteTxRx(ph, sid, addr, value)
    time.sleep(0.02)
    pk.write1ByteTxRx(ph, sid, 55, 1)
    return comm == scs.COMM_SUCCESS and err == 0


def load():
    try:
        with open(OUT) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save(doc):
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    print(f"saved {OUT}")


def unwrap_step(prev_raw: int, raw: int) -> int:
    """The signed step between two raw readings on the 0..4095 circle (the short way round)."""
    d = raw - prev_raw
    if d > 2048:
        d -= 4096
    elif d < -2048:
        d += 4096
    return d


def plan_from_sweep(lo_u: float, hi_u: float, raw_now: int, unwrapped_now: float, old_offset: int) -> dict:
    """From an unwrapped travel [lo_u, hi_u] (in the frame the servo reports now, offset `old_offset`
    included) the homing offset that puts the middle of the travel at 2048, and the limits in
    that new frame. Pure, benched."""
    centre_u = (lo_u + hi_u) / 2.0
    # the reported reading of the centre, on the circle
    centre_reported = (raw_now + (centre_u - unwrapped_now)) % 4096
    # reported = raw - offset  ->  raw at the centre = reported + old
    raw_centre = (centre_reported + old_offset) % 4096
    off = int(round(raw_centre - CENTRE))
    if off > 2047:
        off -= 4096
    if off < -2047:
        off += 4096
    half = (hi_u - lo_u) / 2.0
    lo = int(round(max(0, CENTRE - half)))
    hi = int(round(min(4095, CENTRE + half)))
    return {"homing_offset": off, "range_min": lo, "range_max": hi, "travel": int(round(hi_u - lo_u))}


def sweep(scs, ph, pk, seconds: float, doc: dict) -> int:
    """Read every joint while the operator sweeps them by hand; on their word (SIGINT) or after
    `seconds`, centre each travel on 2048 and write offsets and limits."""
    import signal
    pos0 = present(scs, ph, pk)
    if not pos0:
        print("no servo answers"); return 1
    unwrapped = {n: float(p) for n, p in pos0.items()}
    last_raw = dict(pos0)
    lo = dict(unwrapped); hi = dict(unwrapped)
    stop = {"now": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__("now", True))
    print("sweep: move every joint from one end stop to the other, then stop this tool (SIGINT)", flush=True)
    t0 = time.monotonic(); last_print = 0.0
    while not stop["now"] and time.monotonic() - t0 < seconds:
        for name, raw in present(scs, ph, pk).items():
            if name not in last_raw:
                continue
            unwrapped[name] += unwrap_step(last_raw[name], raw)
            last_raw[name] = raw
            lo[name] = min(lo[name], unwrapped[name]); hi[name] = max(hi[name], unwrapped[name])
        if time.monotonic() - last_print > 2.0:
            last_print = time.monotonic()
            print("travel so far:", {n: int(hi[n] - lo[n]) for n in lo}, flush=True)
        time.sleep(0.05)
    print("travel seen:", {n: (int(lo[n]), int(hi[n]), int(hi[n] - lo[n])) for n in lo}, flush=True)
    for name in lo:
        sid = JOINTS[name]
        if hi[name] - lo[name] < 50:
            print(f"  {name:14s} id {sid}: travel {int(hi[name] - lo[name])} ticks, not moved: skipped"); continue
        if name == "gripper":
            # the CF35-12's register map is its own: limits only, in the frame it reports
            glo, ghi = int(max(0, lo[name])), int(min(4095, hi[name]))
            ok1 = write_eeprom(scs, ph, pk, sid, 9, glo, 2)
            ok2 = write_eeprom(scs, ph, pk, sid, 11, ghi, 2)
            doc.setdefault(name, {})
            doc[name].update({"id": sid, "range_min": glo, "range_max": ghi, "travel_ticks": int(hi[name] - lo[name])})
            print(f"  {name:14s} id {sid}: travel {int(hi[name] - lo[name])} ticks -> limits {glo}..{ghi} "
                  f"({'OK' if ok1 and ok2 else 'FAILED'}), no offset on this servo", flush=True)
            continue
        old = decode_sign_magnitude(read2(scs, ph, pk, sid, 31) or 0)
        p_before = read2(scs, ph, pk, sid, 56)
        plan = plan_from_sweep(lo[name], hi[name], last_raw[name], unwrapped[name], old)
        off = plan["homing_offset"]
        ok = write_eeprom(scs, ph, pk, sid, 31, encode_sign_magnitude(off), 2)
        time.sleep(0.05)
        p_after = read2(scs, ph, pk, sid, 56)
        if p_before is None or p_after is None:
            print(f"  {name:14s} id {sid}: offset {off:+d} written but the servo did not answer the read-back: check with --show", flush=True)
            p_before = p_after = 0
        expect = (p_before - (off - old)) % 4096
        if abs(unwrap_step(expect, p_after)) > 8:
            # the servo applies the offset the other way round: negate and check again
            ok = write_eeprom(scs, ph, pk, sid, 31, encode_sign_magnitude(-off), 2)
            time.sleep(0.05)
            p_after = read2(scs, ph, pk, sid, 56) or 0
            if abs(unwrap_step(expect, p_after)) <= 8:
                off = -off
            else:
                print(f"  {name:14s} id {sid}: offset {off:+d} written but the reading moved from {p_before} to {p_after} "
                      f"(expected {expect}): check by hand", flush=True)
        ok1 = write_eeprom(scs, ph, pk, sid, 9, plan["range_min"], 2)
        ok2 = write_eeprom(scs, ph, pk, sid, 11, plan["range_max"], 2)
        doc.setdefault(name, {})
        doc[name].update({"id": sid, "homing_offset": off, "range_min": plan["range_min"], "range_max": plan["range_max"],
                          "travel_ticks": plan["travel"]})
        print(f"  {name:14s} id {sid}: travel {plan['travel']} ticks -> offset {off:+d} ({'OK' if ok else 'FAILED'}), "
              f"limits {plan['range_min']}..{plan['range_max']} ({'OK' if ok1 and ok2 else 'FAILED'}), reads {p_after} now", flush=True)
    save(doc)
    ph.closePort()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--middle", action="store_true", help="write homing offsets so the current pose reads 2048")
    ap.add_argument("--range", action="store_true", help="write the min/max seen during --watch as position limits")
    ap.add_argument("--watch", type=float, default=0.0, help="seconds to read positions and track min/max")
    ap.add_argument("--sweep", action="store_true", help="measure the end stops of every joint (unwrapped), centre them on 2048, write offsets and limits")
    a = ap.parse_args(argv)
    scs, ph, pk = open_bus()
    pos = present(scs, ph, pk)
    if not pos:
        print("no servo answers on", PORT); return 1
    print("answering:", {n: JOINTS[n] for n in pos})
    if a.show or not (a.middle or a.range or a.watch):
        for name in pos:
            sid = JOINTS[name]
            off = decode_sign_magnitude(pk.read2ByteTxRx(ph, sid, 31)[0])
            lo = pk.read2ByteTxRx(ph, sid, 9)[0]; hi = pk.read2ByteTxRx(ph, sid, 11)[0]
            print(f"  {name:14s} id {sid}  position {pos[name]:4d}  homing_offset {off:+5d}  limits {lo}..{hi}")
        return 0
    doc = load()
    if a.middle:
        for name, p in pos.items():
            sid = JOINTS[name]
            if name == "gripper":
                continue
            raw_now = pk.read2ByteTxRx(ph, sid, 31)[0]
            old = decode_sign_magnitude(raw_now)
            # the position the servo reports already includes its current offset: undo it
            raw_pos = (p + old) % 4096
            # offset such that raw_pos - offset == CENTRE, kept in (-2048, 2048)
            off = raw_pos - CENTRE
            if off > 2047: off -= 4096
            if off < -2047: off += 4096
            okw = write_eeprom(scs, ph, pk, sid, 31, encode_sign_magnitude(off), 2)
            time.sleep(0.05)
            after = pk.read2ByteTxRx(ph, sid, 56)[0]
            doc.setdefault(name, {})
            doc[name].update({"id": sid, "homing_offset": off})
            print(f"  {name:14s} id {sid}: was {p} (offset {old:+d}) -> offset {off:+d} written {'OK' if okw else 'FAILED'}, now reads {after}")
        save(doc)
    if a.sweep:
        return sweep(scs, ph, pk, a.watch or 900.0, doc)
    if a.watch > 0:
        lo = dict(present(scs, ph, pk)); hi = dict(lo)
        t0 = time.monotonic()
        # the operator sweeps the joints by hand: no fixed window is imposed on them, the watch
        # ends on their word (SIGINT / SIGTERM) or after --watch seconds, whichever comes first
        import signal
        stop = {"now": False}
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.__setitem__("now", True))
        last_print = 0.0
        while not stop["now"] and time.monotonic() - t0 < a.watch:
            for name, p in present(scs, ph, pk).items():
                lo[name] = min(lo[name], p); hi[name] = max(hi[name], p)
            if time.monotonic() - last_print > 2.0:
                last_print = time.monotonic()
                print("range so far:", {n: (lo[n], hi[n]) for n in lo}, flush=True)
            time.sleep(0.05)
        print("range seen:", {n: (lo[n], hi[n]) for n in lo}, flush=True)
        if a.range:
            for name in lo:
                sid = JOINTS[name]
                ok1 = write_eeprom(scs, ph, pk, sid, 9, lo[name], 2)
                ok2 = write_eeprom(scs, ph, pk, sid, 11, hi[name], 2)
                doc.setdefault(name, {})
                doc[name].update({"id": sid, "range_min": lo[name], "range_max": hi[name]})
                print(f"  {name:14s} id {sid}: limits {lo[name]}..{hi[name]} written {'OK' if ok1 and ok2 else 'FAILED'}")
            save(doc)
    ph.closePort()
    return 0


if __name__ == "__main__":
    sys.exit(main())
