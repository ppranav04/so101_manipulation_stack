# Joint signs, travel and limits (2026-11-09)

## Summary

All six joint signs match the URDF (sign +1 everywhere). Hand-swept travel shows the
EEPROM position limits sit at the mechanical stops (within about 2 deg), except the
shoulder_pan upper limit, which is about 6 deg beyond the stop. URDF limits are set
35 counts (~3.1 deg) inside the nearer of the EEPROM limit and the measured stop. The
gripper zero is set so the closed gripper matches the closed jaws in the URDF. The zeros
of the other five joints are not verified.

## Setup

- Driver: hw:=real, SO101System active, joint_state_broadcaster only, torque off on all
  six servos (checked before and after).
- Recorder: so101_hardware/scripts/joint_range_recorder.py (latest/min/max of
  /joint_states per joint).
- Counts = 2048 + offset + rad x 4096 / (2 pi). Travel was recorded with every offset at 0.

## Signs

Each joint was moved by hand in one direction, and the sign of the position change was
compared with the direction the RViz model moved (joint_signs.csv).

| Joint | Motion | Position change | RViz agrees |
|---|---|---|---|
| shoulder_pan | left, seen from the front | + | yes |
| shoulder_lift | forward / down | + | yes |
| elbow_flex | unfolding | - | yes |
| wrist_flex | gripper up | - | yes |
| wrist_roll | clockwise, seen from behind | - | yes |
| gripper | opening | + | yes |

## Travel

Three runs, each joint pushed gently to both stops (joint_travel_run{1,2,3}.csv).
The ranges below span the extremes reached across the three runs, in counts.

| Joint | Low stop | High stop | EEPROM | Rest |
|---|---|---|---|---|
| shoulder_pan | 730..762 | 3324..3347 | 761..3410 | not on a stop |
| shoulder_lift | 722..770 | 3297..3313 | 686..3308 | 822..859 |
| elbow_flex | 797..810 | 3109..3115 | 807..3106 | on the high stop, 3097..3109 |
| wrist_flex | 843..853 | 3197..3215 | 858..3187 | 2888..2901 |
| gripper | 2034..2041 | 3497..3514 | 2046..3472 | closed, 2053..2054 |

The extremes moved 0.4-4.2 deg from run to run, depending on how hard each joint was
pushed. Hand force can move a joint past its EEPROM limit, because the servo applies that
limit to goal positions only.

wrist_roll reached counts 0 and 4095 in all three runs: moving the other joints by hand
turned it through the encoder wrap. Its travel is in wrist_roll_travel.md.

## Limits

limit = min(EEPROM limit, innermost measured stop), moved 35 counts inward, then
rounded inward to 4 decimals in rad, so on_configure's check against the EEPROM range
passes.

| Joint | Counts | rad |
|---|---|---|
| shoulder_pan | 797..3289 | -1.9190..1.9036 |
| shoulder_lift | 805..3262 | -1.9067..1.8622 |
| elbow_flex | 845..3071 | -1.8453..1.5692 |
| wrist_flex | 893..3152 | -1.7717..1.6935 |
| wrist_roll | 118..1839 | -2.96..-0.32 (unchanged, see wrist_roll_travel.md) |
| gripper | 2048..3437 | -0.2070..1.9236 (lower limit = fully closed) |

Gripper zero: in the URDF the jaws touch at about -0.2 rad (checked visually in RViz),
and the closed gripper reads 2053 counts, so offset = 2053 - 2048 + 0.2 x 651.9 = 135
counts. The gripper limits keep the same counts and shift in rad by -135 / 651.9. After
the change, the closed gripper shows closed jaws in RViz (visual check).

Velocity limit: 5.0 rad/s on every joint. write() uses it as the per-cycle step limit
(0.05 rad at 100 Hz).

## Open

- At rest, elbow_flex sits on its stop, outside its limit. The activation check refuses
  to enable torque there; hold the forearm off the stop before activating.
- The zeros of shoulder_pan, shoulder_lift, elbow_flex, wrist_flex and wrist_roll are
  not verified.
