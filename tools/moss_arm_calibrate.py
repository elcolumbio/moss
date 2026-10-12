#!/usr/bin/env python3
"""Calibrate the arm's Feetech servos (STS family, protocol 0, 1 Mbps), the SO-101 way.

Two steps, each on the operator's word (the arm is moved by hand, torque off):
  1. --middle   the arm is held in the middle of every joint's travel: the servo's
                Homing_Offset is written so that this pose reads 2048 (the centre of
                the 0..4095 turn), which puts the encoder's zero crossing OUT of the
                useful range.
  2. --range    every joint has just been moved through its whole travel: the lowest
                and highest positions seen during --watch are written as the servo's
                Min/Max_Position_Limit and saved with the offsets.
  --watch N     read positions for N seconds and print the min/max per joint (used by
                --range; run it while the operator sweeps the joints).
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


def present(scs, ph, pk):
    out = {}
    for name, sid in JOINTS.items():
        if pk.ping(ph, sid)[1] == scs.COMM_SUCCESS:
            out[name] = pk.read2ByteTxRx(ph, sid, 56)[0]
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--middle", action="store_true", help="write homing offsets so the current pose reads 2048")
    ap.add_argument("--range", action="store_true", help="write the min/max seen during --watch as position limits")
    ap.add_argument("--watch", type=float, default=0.0, help="seconds to read positions and track min/max")
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
