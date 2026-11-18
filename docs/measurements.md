# Measurements

Measured on the real SO-101 arm unless stated. Full write-ups and raw data: commit `8928dc5`,
`docs/measurements/`.

## Driver (M2)

| Measurement | Result |
|---|---|
| Control loop | 100 Hz, p99 period 10.255 ms, 0 of 11 996 cycles over 15 ms |
| SYNC READ, six servos | p50 1.15 ms, p99 1.27-1.29 ms, 0 failures in 6000 |
| Write reliability | 24 000 writes, 0 timeouts; 40/40 activations |
| Command-to-motion latency | p50 90-100 ms |
| Trajectory tracking, full speed | max error 0.097 rad, final error <= 0.020 rad |

## MoveIt 2 (M3)

`home` <-> `zero`, 10 runs per backend, RRTConnect, scaling 0.1.

| | mock | Gazebo | real |
|---|---|---|---|
| Success | 10/10 | 10/10 | 10/10 |
| Planning time, median (max) | 13 (32) ms | 22 (29) ms | 22 (34) ms |
| Final error | <= 0.001 rad | <= 0.001 rad | 0.042-0.103 rad (gravity sag) |

- Real-arm final error is gravity sag, worst at the elbow (0.087-0.100 rad at `zero`).
  The elbow goal tolerance was raised from 0.1 to 0.15 rad after three aborts at 0.103-0.105 rad.
- A held arm does not drift: 0 counts over 30 s after each of 6 moves.

## Servo undervoltage flag (M3)

The gripper's undervoltage flag dropped the arm mid-motion. With torque off, the last three
servos returned single-read glitches (min 3.9 V, 5.3 V on the reads around them), so the flag
is a measurement glitch, not a supply sag. The driver now faults only after 10 consecutive
flagged reads: 31 glitches tolerated, 0 faults in 5 sessions.

## Hardware

| Measurement | Result |
|---|---|
| Calibration | persists across power cycles after unlock -> write -> verify -> lock |
| Present_Velocity unit | counts/s (fit slope 0.996-0.997) |
| Joint limits | 35 counts inside the measured stops |
| wrist_roll travel | ~338 deg, encoder wrap inside the travel |
