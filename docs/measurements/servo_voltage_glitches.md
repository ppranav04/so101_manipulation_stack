# Servo undervoltage flag: single-read glitches, not a supply sag (2026-11-17)

## Summary

From M2 to M3, the gripper servo (ID 6) repeatedly reported its undervoltage flag (status
error bit 0x01) at 3.8-3.9 V. The driver treated any error bit as a fault and switched torque
off on every servo, so the arm dropped: once while holding after a move and once in the middle
of a MoveIt execution. The suspected cause was voltage drop along the servo daisy chain.

Measurements show something else. With torque off on all six servos (no load current), over
941 s at 65 reads per second per servo, the last three servos in the chain returned isolated
low voltage readings: 90, 174 and 282 reads below 5.1 V (minimum 3.9 V), every one a single
read, with 5.3 V on the reads before and after. The gripper set its 0x01 flag exactly on its
3.9 V reads. Temperature readings show the same kind of single-read outliers (up to +14 C),
which cannot be physical. The first three servos returned no outliers. A program that holds
the arm without checking the flag ran 262 s with the gripper at a steady 5.3 V.

The driver now tolerates the voltage flag unless it is present on 10 consecutive reads
(100 ms at 100 Hz); any other error bit still faults at once. Across four real-arm sessions
since then (hold, RViz moves, 3 baseline attempts, 10-run baseline), 18 flags were tolerated,
every logged one a single read, and no voltage fault occurred.

## Setup

- Six Feetech STS3215 servos on one half-duplex bus, daisy-chained in ID order
  1 (shoulder_pan) to 6 (gripper). Supply 5 V, 6 A. Servo Min_Voltage_Limit 4.0 V.
- Error byte: bit 0 = voltage (`ERRBIT_VOLTAGE = 1` in the Feetech SCServo SDK,
  `protocol_packet_handler.py`). Present_Voltage (register 62) in 0.1 V,
  Present_Temperature (63) in C.
- ROS driver (`so101_hardware`): one SYNC READ of registers 56-63 for all six servos per
  cycle, 100 Hz. Before the change, `read()` returned ERROR on any nonzero error byte, and
  `on_error` switched torque off on every servo.
- Standalone monitor (Python, SCServo SDK, not part of this repository): reads registers
  56-63 and Torque_Enable of each servo in turn, logs every read to CSV, and counts every
  reply with a nonzero error byte. It can drive the arm from a game controller with the same
  motion scheme as an earlier teleop program, or run with torque never enabled.

## Before the change: what the driver saw

| Date | Situation | Fault |
|---|---|---|
| 2026-11-10 | Activation acceptance test, holding | gripper 0x01 |
| 2026-11-15 | Holding after the first real MoveIt execution | gripper 0x01 at 3.9 V, 4.7 s after the move |
| 2026-11-16 | Activation from rest, holding | gripper 0x01, 10-50 s after activation |
| 2026-11-16 | MoveIt execution | gripper 0x01 at 3.8 V, 93 s after activation, mid-trajectory |

The driver's per-servo statistics at each fault read like a sagging chain: lowest voltage
5.3 / 5.3 / 5.2 / 4.8 / 4.3 / 4.0 V from ID 1 to ID 6, gripper up to 48 C. Replacing the
gripper cable (2026-11-11) changed nothing. Those statistics are the minimum and maximum of
single reads at 100 Hz; the measurements below show they are glitch values.

## A/B: same pose, two programs

Rest pose, hands off, gripper cool at the start of each run.

| | A: monitor holding the arm | B: ROS driver holding the arm |
|---|---|---|
| Torque on, no motion | 262 s | fault after 25 s |
| Gripper voltage | 5.3 V on every 1 Hz read | 0x01 at 3.9 V at the fault |
| Gripper temperature | 30-32 C | 33 C at the fault |
| Gripper load (Present_Load) | 0 | not read by the driver |
| Replies with an error byte | 0 | the fault |
| Single low reads | gripper 4.9 V once; wrist_roll 5.0 V once; gripper 4.0 V once with torque off before enabling | - |

After B's fault, the component was configured again but not activated. The configure log
shows torque off on all six servos. `read()` kept polling, and about 270 s later the gripper
reported 0x01 at 3.9 V again, with no torque on any servo.

An earlier monitor run with motion (196 s, joystick moves, 1 Hz sampling) counted 15 gripper
replies with 0x01 during the motion commands. Their voltage was not captured; the lowest
sampled reading was 4.6 V. No servo lost torque and no transaction failed.

## Torque off, high rate

Question: with no load current at all, does the flag still appear, and how long does it last?

Method: torque never enabled; every servo read in turn as fast as the bus allows, 941 s,
61 279 reads per servo (65.1 per second per servo). A read is "low" below 5.1 V. A
temperature read is an outlier when it is more than 3 C above that servo's most common value.

| ID | Joint | Usual V | Reads < 5.1 V | Lowest | Longest low run | 0x01 reads | Temperature outliers (longest run) | Max T |
|---|---|---|---|---|---|---|---|---|
| 1 | shoulder_pan | 5.4 | 0 | 5.3 | - | 0 | 0 | 33 C |
| 2 | shoulder_lift | 5.3 | 0 | 5.3 | - | 0 | 0 | 33 C |
| 3 | elbow_flex | 5.4 | 0 | 5.2 | - | 0 | 0 | 32 C |
| 4 | wrist_flex | 5.3 | 90 | 4.8 | 1 read | 0 | 25 (1) | 36 C |
| 5 | wrist_roll | 5.3 | 174 | 4.4 | 1 read | 0 | 184 (2) | 40 C |
| 6 | gripper | 5.3 | 282 | 3.9 | 1 read | 4 | 313 (2) | 47 C |

- Every low voltage read is a single read. The four gripper 0x01 reads are exactly its four
  3.9 V reads, each with 5.3 V on the read before and after (about 15 ms apart). For example
  t = 268.076 s 5.3 V, 268.091 s 3.9 V 0x01, 268.106 s 5.3 V. Four further replies to the
  Torque_Enable reads in the same loop also carried 0x01 (voltage not captured in those).
- No read had both a voltage and a temperature outlier.
- The low-read rate was steady over the run (25-45 gripper reads per 2 min, 8 intervals).

## Interpretation

- **The flag is not caused by load current.** It appears with torque off on every servo,
  and a program holding the arm saw a steady 5.3 V on the gripper.
- **The readings are glitches inside the servo's measurement, not a real supply state.**
  A temperature reading cannot rise 14 C and fall back within 15 ms, and those outliers come
  from the same three servos. The undervoltage flag follows the servo's own voltage
  reading, so it glitches with it.
- **The rate grows toward the end of the chain** (ID 4 < 5 < 6, none on IDs 1-3). That
  points at electrical noise reaching the last servos rather than a fault in one servo.
  The source (supply, wiring, ground) was not identified.
- **Why the driver faulted and the earlier teleop program did not:** the driver reads every
  servo 100 times per second and acted on one read. The teleop program never read the error
  byte after enabling torque.
- Whether a real voltage dip of microseconds occurs (rather than a measurement artifact)
  cannot be told from these reads. Either way it is shorter than one read period, and no
  servo reset or lost torque in any run.

## Change in the driver

`conversion::check_servo_error(error, run, max_run)` (joint_conversion, unit-tested):

- error byte 0: OK, and the servo's run counter resets;
- error byte exactly 0x01: the counter increments; the read is tolerated until the counter
  reaches 10, then it is a fault;
- any other byte, including 0x01 combined with another bit (for example 0x21 = overload
  plus voltage): fault on the first read.

10 consecutive reads is 100 ms at 100 Hz. The longest run measured was 1 read. A tolerated
read logs a throttled warning; the deactivate log reports how many reads were tolerated per
servo. The per-servo minimum voltage and maximum temperature in that log are single-read
extremes and are labelled as such.

## After the change

| Session (2026-11-17) | Activity | Tolerated 0x01 reads | Voltage faults |
|---|---|---|---|
| Hold test (as B above) | 303 s holding at rest, controllers active | 1 | 0 |
| RViz moves | 4 executions, one flag during an execution | 5 | 0 |
| Baseline attempts | 3 attempts, 15 recorded runs | 6 | 0 |
| Baseline | 10/10 runs | 6 | 0 |

All 18 were on the gripper. Every logged one said "1 in a row" (the warning is throttled, so
not every tolerated read is logged; none reached 10). In one session the component was left
inactive after deactivation and `read()`, still polling, logged one more flag with torque off.

## Limitations

- One arm, one 5 V supply, one wiring layout. The cause of the glitches is not identified;
  a different supply or a shorter chain was not tried.
- The monitor reads servos one at a time (about 15 ms between reads of the same servo), so
  glitch length is bounded only as shorter than that.
- The torque-off run did not include the ROS driver's SYNC READ timing; the driver's own
  runs give run lengths only through its warnings.
- The monitor script is not in this repository.

## Files

In `data/servo_voltage_glitches/`:

- `torque_off_summary.csv`: the per-servo table above.
- `torque_off_outliers.csv`: every read from the torque-off run with a low voltage, a
  temperature outlier or a nonzero error byte (1068 rows), with the voltage of the read before
  and after on the same servo. The full run (61 279 reads per servo, 13 MB) is not committed.
