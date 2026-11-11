# Control loop on the real arm (2026-11-11)

## Summary

The ros2_control loop runs at 100 Hz on the real arm with p99 period 10.255 ms and no cycle
over 15 ms in 2 minutes (11 996 cycles). read() takes 1.25 ms per cycle on average (one SYNC
READ of six servos). 24 000 single-servo WRITEs got a reply every time, and 40 activations
in a row succeeded. Command-to-motion latency, beyond an ideal plant, is 90-100 ms (p50).
The 30-minute soak has not been run yet.

## Setup

- hw:=real, rviz:=false, laptop without a real-time kernel (FIFO scheduling not permitted).
- Arm resting on a support with every joint inside its limits; torque on, holding.
- Supply 5 V 6 A; servos STS3215 (voltage limits 4.0-8.0 V).

## Loop timing (120 s, joint_state_broadcaster only)

Tool: so101_hardware/scripts/soak_monitor.py (periods from /joint_states header stamps,
plus the controller manager's /diagnostics).

| Measure | Value |
|---|---|
| Period p50 / p99 | 10.000 / 10.255 ms |
| Period min / max | 8.334 / 11.911 ms |
| Cycles over 15 ms | 0 of 11 996 |
| Controller manager rate | avg 100.001 Hz, std 0.37 Hz |
| read() execution time | avg 1254 us, range 1116-3110 us, std 80 us |
| write() execution time | avg 24 us, range 4-246 us |

write() time covers handing the 26-byte SYNC WRITE to the kernel only. Servos do not reply
to SYNC WRITE; at 1 Mbaud the packet takes about 260 us on the wire after write() returns.

## Write reliability

Tool: StsBusHardware.WriteReplyRate (so101_hardware/test/test_sts_bus.cpp). Each round is
one SYNC READ followed by a Goal_Position WRITE to each servo, writing back its own present
position, with a 20 ms timeout, as on_activate does.

| Condition | Writes | Timeouts | Reply p50 / p99 / max |
|---|---|---|---|
| Arm at rest, 5 runs | 15 000 | 0 | 322-363 / 522-565 / 669-2214 us |
| Arm held off its stop (loaded), 3 runs | 9 000 | 0 | 337-353 / 543-559 / 680-2020 us |

Writing Goal_Position switches the servo's torque on (torque 0 on all six before a run,
1 after). The test switches torque off again at the end.

## Activation

40 activate/deactivate cycles (20 with RViz, 20 without), arm resting on a support: 40
succeeded, 0 failed. In earlier sessions with the arm held by hand at the start of a
session, 4 of about 9 activations failed with a WRITE timeout inside on_activate (handled:
torque off, component stays inactive). That failure has not been reproduced; cause unknown.

## Command-to-motion latency

Tool: so101_hardware/scripts/latency_test.py. Linear ramp of 0.1 rad over 0.25 s on one
joint, 20 trials alternating direction. Latency = first controller_state stamp where the
feedback has moved more than 0.005 rad, minus the first where the reference has. On the mock
(ideal plant) this reads 0 ms, so the values below are the delay the hardware adds compared
with an ideal plant, in 10 ms steps.

| Joint | p50 | min | max |
|---|---|---|---|
| shoulder_pan | 99.8 ms | 70.0 ms | 100.1 ms |
| wrist_flex | 89.8 ms | 80.0 ms | 90.2 ms |

The value depends on direction by about one cycle. It includes the servo's lag following a
ramp and its deadband (about 4-5 counts at shoulder_pan); the two have not been separated.

## Not yet measured

- 30-minute soak with bus-error count.
- Supply voltage under load (the gripper reported a voltage error twice while holding).
