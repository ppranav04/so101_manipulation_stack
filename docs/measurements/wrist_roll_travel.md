# wrist_roll travel (2026-10-28)

## Summary

wrist_roll's hand-rotatable travel spans ~338 deg, limited by a stop that is not the camera
cable (re-routing it changed nothing). Under the persisted BT101_follower calibration the
travel runs from about -344 deg to -7 deg relative to the calibration centre (count 2048):
the calibration centre (URDF zero) is just outside the travel, and the encoder wrap
(raw 0 / 4095, -180 deg) lies inside it.

Decision B: keep the EEPROM calibration, and limit wrist_roll to the part of the travel on the
rest-pose side of the wrap: **-2.96 .. -0.32 rad** (encoder direction, provisional until the
joint sign is verified in read()).

## Setup

- Calibration: BT101_follower.json persisted to EEPROM (wrist_roll Homing_Offset -2013,
  Min/Max 0..4095). See calibration_persistence.md.
- Script: so101_hardware/scripts/wrist_roll_travel.py. Torque off on ID 5, then
  Present_Position (sign bit 15) read at 50 Hz while rolling the wrist by hand to each end
  of travel; readings unwrapped across the 0/4095 encoder wrap (a step > 2048 counts between
  consecutive samples is treated as a wrap, +-4096).
- Angles below are relative to the calibration centre (count 2048), in encoder direction:
  deg = (unwrapped - 2048) * 360 / 4096.

## Results

### Before re-routing the camera cable

| Run | Unwrapped min | Unwrapped max | Span |
|---|---|---|---|
| 1 | -1883 (-345.5 deg) | 1960 (-7.7 deg) | 337.8 deg |
| 2 | -1862 (-343.7 deg) | 1972 (-6.7 deg) | 337.0 deg |
| 3 | -1864 (-343.8 deg) | 1980 (-6.0 deg) | 337.9 deg |

### After re-routing the camera cable

| Run | Unwrapped min | Unwrapped max | Span |
|---|---|---|---|
| 1 | -1866 (-344.0 deg) | 1977 (-6.2 deg) | 337.8 deg |
| 2 | -1879 (-345.1 deg) | 1965 (-7.3 deg) | 337.9 deg |
| 3 | -1900 (-347.0 deg) | 1972 (-6.7 deg) | 340.3 deg |

Both ends repeat within ~2-3 deg across all six runs. Re-routing the camera cable did not
move either end.

### Cross-check with the earlier run (calibration set A)

An earlier run the same morning, while the servos held calibration set A (wrist_roll
homing 1143), measured -930 .. 2899. Converting to the current frame
(present_C = present_A - 940 counts) predicts -1870 .. 1959; measured -1862..-1900 ..
1960..1980. Same physical travel, different reference frame. The measurement method gives
consistent results across calibrations.

## Findings

1. Physical travel is ~338 deg. The unreachable gap is ~20 deg (raw ~1960 .. ~2234).
2. The calibration centre (count 2048 = URDF wrist_roll 0) lies inside that gap, ~7 deg
   past the upper end of travel. The wrist cannot reach its URDF zero pose.
3. The encoder wrap (-180 deg, raw 0) lies inside the travel. Readings from raw ~2234 to
   4095 correspond to the wrist rolled past the wrap (physically -344 to -180 deg) and would
   naively convert to +16 to +180 deg.
4. The travel limit is not the camera cable. Its physical cause is unidentified.

## Decision (option B)

Keep the EEPROM calibration (no EEPROM changes, BT-101 unaffected). Restrict wrist_roll to the
travel segment on the rest-pose side of the wrap, using the innermost ends observed across
all six runs (lower -1862, upper 1960), with ~10 deg margins:

| | Encoder direction | Raw counts | URDF (rad) |
|---|---|---|---|
| Lower | -170 deg (10 deg from the wrap) | ~118 | -2.96 |
| Upper | -18 deg (~10 deg inside the end) | ~1839 | -0.32 |

Usable roll: ~152 deg. Rest pose (raw ~1000, ~-90 deg) lies inside.

Rejected: re-centring the homing offset (EEPROM writes, changes the BT-101 calibration) and
re-routing the camera cable (tried; no effect).

## Consequences

- The wrap is inside the physical travel, so on_activate's limit check is the guard: if the
  wrist rests past the wrap, its reading falls outside -2.96 .. -0.32 rad and activation is
  refused. That check must cover every joint, every time.
- URDF zero is outside the limits. The mock/gz backends need a wrist_roll initial_value inside
  them (-1.57 rad), or MoveIt reports the start state as out of bounds.
- 152 deg of roll is less than the 180 deg a parallel gripper needs to reach every grasp
  angle. Some top-down grasp angles will need shoulder_pan to compensate or will be
  unreachable. Known constraint for M6.
- BT-101: a centred leader wrist commands the follower to count 2048, a pose it cannot
  reach. Noted for BT-101's README.

## Open items

- Joint sign: limits are in encoder direction. If read() shows wrist_roll's sign is -1, they
  become 0.32 .. 2.96 rad.
- Physical cause of the travel limit: unidentified. The margins keep commanded motion away
  from it.

## Data files

- wrist_roll_travel_2026-10-28_setA.csv: earlier run under calibration set A (superseded).
- wrist_roll_travel_run1.csv, wrist_roll_travel_run2.csv: before re-route, runs 1-2.
- wrist_roll_travel_run3.csv: after re-route, run 3. The script's fixed output name made
  the three post-re-route runs overwrite this file, including pre-re-route run 3. The
  summary lines above are the record for the overwritten runs. The script now takes the
  output name as an argument.
