#!/usr/bin/env python3
"""Read-only dump of calibration registers for IDs 1-6, independent of the ROS driver."""
import serial

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081270-if00"
BAUD = 1_000_000
INST_READ = 0x02


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


def sign_magnitude(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
    print("id  homing   min   max")
    for sid in range(1, 7):
        homing = sign_magnitude(int.from_bytes(read_reg(ser, sid, 31, 2), "little"), 11)
        lo = int.from_bytes(read_reg(ser, sid, 9, 2), "little")
        hi = int.from_bytes(read_reg(ser, sid, 11, 2), "little")
        print(f"{sid:2d}  {homing:6d}  {lo:4d}  {hi:4d}")
        lock = read_reg(ser, sid, 55, 1)[0]
        torque = read_reg(ser, sid, 40, 1)[0]
        print(f"     lock {lock}  torque {torque}")
