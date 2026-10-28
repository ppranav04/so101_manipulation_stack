#!/usr/bin/env python3
"""wrist_roll wrap check: torque off (verified), then stream Present_Position of ID 5."""

import csv
import sys
import time

import serial


PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081270-if00'
BAUD = 1_000_000
SERVO_ID = 5
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
INST_READ = 0x02
INST_WRITE = 0x03
URDF_MIN_COUNTS, URDF_MAX_COUNTS = 259, 3900  # wrist_roll URDF limits at centre 2048

LOG_PATH = sys.argv[1] if len(sys.argv) > 1 else 'wrist_roll_travel.csv'


def packet(servo_id, inst, params):
    body = [servo_id, len(params) + 2, inst, *params]
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def read_reg(ser, addr, length):
    ser.reset_input_buffer()
    ser.write(packet(SERVO_ID, INST_READ, [addr, length]))
    reply = ser.read(6 + length)
    if len(reply) != 6 + length or reply[:2] != b'\xff\xff' or reply[2] != SERVO_ID:
        raise IOError(f"bad reply: {reply.hex(' ')}")
    if (~sum(reply[2:-1])) & 0xFF != reply[-1]:
        raise IOError(f"bad checksum: {reply.hex(' ')}")
    if reply[4] != 0:
        print(f'\nservo error flags 0x{reply[4]:02X}')
    return reply[5:5 + length]


def sign_magnitude(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
    ser.write(packet(SERVO_ID, INST_WRITE, [ADDR_TORQUE_ENABLE, 0]))
    time.sleep(0.02)  # let any status reply arrive; read_reg flushes it
    if read_reg(ser, ADDR_TORQUE_ENABLE, 1)[0] != 0:
        raise SystemExit('torque did NOT turn off; stopping')
    print('wrist_roll torque OFF. Roll slowly to each end of travel; Ctrl+C to stop.')

with serial.Serial(PORT, BAUD, timeout=0.05) as ser, open(LOG_PATH, 'w', newline='') as f:
    log = csv.writer(f)
    log.writerow(['t_s', 'raw', 'unwrapped', 'deg_from_centre'])
    t0 = time.monotonic()

    lo = hi = prev = unwrapped = None
    try:
        while True:
            d = read_reg(ser, ADDR_PRESENT_POSITION, 2)
            pos = sign_magnitude(d[0] | (d[1] << 8), 15)
            if prev is None:
                unwrapped = pos
            else:
                step = pos - prev
                if step > 2048:
                    step -= 4096
                elif step < -2048:
                    step += 4096
                unwrapped += step
            deg = (unwrapped - 2048) * 360.0 / 4096
            log.writerow([f'{time.monotonic() - t0:.3f}', pos, unwrapped, f'{deg:.1f}'])
            lo = unwrapped if lo is None else min(lo, unwrapped)
            hi = unwrapped if hi is None else max(hi, unwrapped)
            print(f'\rraw {pos:5d}  unwrapped {unwrapped:6d} ({deg:+7.1f} deg)  '
                  f'min {lo:6d}  max {hi:6d}', end='', flush=True)
            prev = pos
            time.sleep(0.02)
    except KeyboardInterrupt:
        print(f'\n{LOG_PATH}: unwrapped {lo}..{hi}, span {(hi - lo) * 360 / 4096:.0f} deg')
