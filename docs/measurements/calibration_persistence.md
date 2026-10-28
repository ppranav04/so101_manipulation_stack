# Follower servo calibration persistence (2026-10-28)

## Summary

The follower arm's EEPROM held a *leader* calibration (`teleoperators/so_leader/None.json`).
LeRobot's "use existing calibration file" restore wrote the correct follower calibration
(`robots/so_follower/BT101_follower.json`, 2026-08-05) only until the next power-off; every
power-up reverted the servos to the leader values. BT-101 worked anyway because LeRobot
rewrites the calibration on every connect.

Fixed by writing BT101_follower.json with an explicit unlock -> write -> read back -> lock
sequence (`so101_hardware/scripts/persist_calibration.py`), verified by power cycles.

## How it was found

`so101_hardware`'s `on_configure` compares each servo's EEPROM Homing_Offset and position
limits against the `eeprom_homing_offset` params and URDF limits, and refuses to configure
on any mismatch. Over the morning it saw four different calibrations on the same arm:

| Set | When observed | ID 1 | ID 2 | ID 3 | ID 4 | ID 5 | ID 6 | Min/Max limits |
|---|---|---|---|---|---|---|---|---|
| A | first configure, ~09:33 | 1992 | -1038 | -1850 | -1989 | 1143 | 1971 | all 0..4095 |
| B | dump + configure, ~10:04 | 2041 | -1110 | -1810 | -2010 | -1990 | 1943 | swept ranges |
| C | after LeRobot restore | -2006 | -1061 | -1895 | -1967 | -2013 | 1922 | BT101 ranges |
| D | after every 12 V power cycle | 1657 | -983 | 1944 | 1248 | -1877 | 1683 | leader ranges |

(Homing_Offset in counts, sign-magnitude with sign bit 11.)

## Investigation

1. **Driver decoding ruled out.** A read-only Python dump
   (`scripts/read_only_calibration_registers.py`), which reads each register with its own
   READ, matched the driver's EEPROM table exactly for set B.
2. **Source of D identified.** `grep` found D's values only in
   `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/None.json`: a leader
   calibration run without `--robot.id`, most likely while the follower was on the port.
   Only one adapter (the follower's) was connected during all tests.
3. **Persistence tested directly.**
   - LeRobot restore (`lerobot-calibrate --robot.id=BT101_follower`, ENTER):
     dump shows C, Lock = 0.
   - 12 V off and on: dump shows D, Lock = 1.
   So the restore changes the live register values but not what the servo loads at power-up.
4. **Hypothesis (not verified against LeRobot's source):** LeRobot writes the calibration
   while the servo is still locked from power-up, and only unlocks (with torque off) at
   disconnect, which would explain Lock = 0 after the restore. The fix does not depend on this.
5. Origin of sets A and B is unknown; they were most likely intermediate live states from
   earlier LeRobot sessions.

## Fix

`scripts/persist_calibration.py <calibration.json> [--ids ...] [--apply]`, per servo:
check torque is off -> Lock (55) = 0 -> write Homing_Offset (31), Min_Position_Limit (9),
Max_Position_Limit (11), reading each back -> Lock = 1 (in `finally`). Dry run by default.

## Verification

| Step | Result |
|---|---|
| Dry run against BT101_follower.json | planned values match the file |
| `--ids 6 --apply`, power cycle, dump | ID 6 = 1922 / 2046 / 3472, Lock 1; IDs 1-5 still D |
| `--ids 1 2 3 4 5 --apply`, power cycle, dump | all six = C, Lock 1 |
| `on_configure` after power cycle | SUCCESS, table matches the dump |

## 2d acceptance (so101_hardware on_configure / on_cleanup), real arm

| Case | Result |
|---|---|
| 1. All powered | SUCCESS, including after a power cycle; ~7-9 ms per configure |
| 2. USB unplugged | FAILURE: port named, "No such file or directory" |
| 3. Gripper (ID 6) unplugged | FAILURE: only `gripper (ID 6): timeout` |
| 4. Bus power off | FAILURE: all six IDs time out (~20 ms each) |
| 5. configure -> cleanup -> configure | second configure SUCCESS |

## Rules from this

- After **any** LeRobot calibration, run `persist_calibration.py` with that file, power-cycle,
  and confirm with the read-only dump. Then update `eeprom_homing_offset` in the xacro.
- One program on the bus at a time (LeRobot, scripts, or the ROS driver).
- A mismatch on all six joints at once means the wrong calibration in the servos (or the
  wrong arm/port), not a single joint's drift.
