#!/usr/bin/env python3
"""Persist a LeRobot calibration file into the follower servos' EEPROM.

Dry run by default: prints the planned writes and changes nothing.
--apply writes them, per servo: check torque off -> unlock -> write + read back -> lock.
"""
import argparse
import json
import time

import serial

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081270-if00"
BAUD = 1_000_000
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_MIN_POSITION_LIMIT = 9   # 2 bytes, unsigned
ADDR_MAX_POSITION_LIMIT = 11  # 2 bytes, unsigned
ADDR_HOMING_OFFSET = 31       # 2 bytes, sign-magnitude, sign bit 11
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
HOMING_SIGN_BIT = 11
WRITE_SETTLE_S = 0.05         # time for the servo to commit each write


def packet(sid, inst, params):
    body = [sid, len(params) + 2, inst, *params]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def read_reg(ser, sid, addr, length):
    ser.reset_input_buffer()
    ser.write(packet(sid, INST_READ, [addr, length]))
    reply = ser.read(6 + length)
    if len(reply) != 6 + length or reply[:2] != b"\xff\xff" or reply[2] != sid:
        raise IOError(f"ID {sid}: bad reply {reply.hex(' ')}")
    if (~sum(reply[2:-1])) & 0xFF != reply[-1]:
        raise IOError(f"ID {sid}: bad checksum {reply.hex(' ')}")
    return reply[5:5 + length]


def write_verified(ser, sid, addr, data):
    ser.reset_input_buffer()
    ser.write(packet(sid, INST_WRITE, [addr, *data]))
    time.sleep(WRITE_SETTLE_S)  # any status reply arrives now; read_reg flushes it
    got = list(read_reg(ser, sid, addr, len(data)))
    if got != list(data):
        raise IOError(f"ID {sid} addr {addr}: wrote {list(data)}, read back {got}")


def u16(value):
    return [value & 0xFF, (value >> 8) & 0xFF]


def encode_sign_magnitude(value, sign_bit):
    magnitude = abs(value)
    if magnitude >= (1 << sign_bit):
        raise ValueError(f"{value} does not fit in {sign_bit} magnitude bits")
    return magnitude | (1 << sign_bit) if value < 0 else magnitude


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_json")
    parser.add_argument("--ids", type=int, nargs="*", help="only these servo IDs")
    parser.add_argument("--apply", action="store_true", help="actually write (default: dry run)")
    args = parser.parse_args()

    with open(args.calibration_json) as f:
        calibration = json.load(f)

    plan = [(joint, c["id"], c["homing_offset"], c["range_min"], c["range_max"])
            for joint, c in calibration.items()
            if not args.ids or c["id"] in args.ids]

    print(f"{'joint':14s} id  homing   min   max")
    for joint, sid, homing, lo, hi in plan:
        print(f"{joint:14s} {sid:2d}  {homing:6d}  {lo:4d}  {hi:4d}")
    if not args.apply:
        print("\nDry run: nothing written. Re-run with --apply to write.")
        return

    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        for joint, sid, homing, lo, hi in plan:
            if read_reg(ser, sid, ADDR_TORQUE_ENABLE, 1)[0] != 0:
                raise SystemExit(f"{joint} (ID {sid}): torque is ON, refusing to write")
            write_verified(ser, sid, ADDR_LOCK, [0])
            try:
                write_verified(ser, sid, ADDR_HOMING_OFFSET,
                               u16(encode_sign_magnitude(homing, HOMING_SIGN_BIT)))
                write_verified(ser, sid, ADDR_MIN_POSITION_LIMIT, u16(lo))
                write_verified(ser, sid, ADDR_MAX_POSITION_LIMIT, u16(hi))
            finally:
                write_verified(ser, sid, ADDR_LOCK, [1])
            print(f"{joint} (ID {sid}): written, read back OK, locked")
    print("\nPower-cycle the arm, then run the read-only dump to confirm the values persisted.")


if __name__ == "__main__":
    main()
