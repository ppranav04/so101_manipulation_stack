# SYNC READ latency (2026-11-08)

## Summary

One SYNC READ of Present_Position, Present_Velocity and Present_Load (registers 56..61,
6 bytes) from all six servos takes **p50 1147-1149 us, p99 1265-1294 us** (3 runs x 2000
reads, 0 failures). This is the bus time SO101System::read() will spend per control cycle.

## Setup

- Test: so101_hardware/test/test_sts_bus.cpp, StsBusHardware.SyncReadAllServosLatency
  (read-only, no register written). 2000 back-to-back reads per run, 10 ms timeout.
- Port: QinHeng USB serial adapter (cdc_acm driver, by-id ...5AE6081270-if00),
  1 000 000 baud, IDs 1-6.
- Host: Ubuntu 24.04 laptop, no RT kernel, nothing else on the servo bus.
- Other USB devices on the same controller (Bus 001, each on its own root port): webcam,
  USB-C EarPods (idle), keyboard/mouse HID, Bluetooth.
- Torque: <off/on, from read_only_calibration_registers.py>.
- Elapsed time is measured inside StsBus with steady_clock, from before the request is
  encoded to after the last reply is parsed.

## Results

| Run | OK / failed | p50 (us) | p99 (us) | max (us) |
|---|---|---|---|---|
| 1 | 2000 / 0 | 1149 | 1294 | 2824 |
| 2 | 2000 / 0 | 1148 | 1271 | 1774 |
| 3 | 2000 / 0 | 1147 | 1265 | 1432 |
| All | 6000 / 0 | 1147-1149 | 1265-1294 | 2824 |

The p50 varies by 2 us across runs and the p99 by 29 us; only the max moves noticeably.

## Interpretation

- Wire time: request 8 + 6 = 14 bytes, replies 6 x (6 + 6) = 72 bytes, 86 bytes at
  10 bits/byte and 1 Mbaud = 860 us. The rest (~290 us at p50) is USB adapter and servo
  turnaround.
- At a 100 Hz loop (10 ms period), p50 uses ~11% of the cycle; the worst sample over all
  6000 reads (2824 us, run 1) ~28%.
- The max varies between runs (1432-2824 us) while p50 and p99 do not, so the tail is
- At a 100 Hz loop (10 ms period), p50 uses ~11% of the cycle; the worst sample over all
  6000 reads (2824 us, run 1) ~28%.
- The max varies between runs (1432-2824 us) while p50 and p99 do not, so the tail is
  occasional single samples. Its cause (USB scheduling, no RT kernel, other USB devices)
  is not yet verified.
