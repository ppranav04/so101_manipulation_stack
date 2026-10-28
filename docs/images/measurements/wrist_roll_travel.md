# wrist_roll travel (2026-10-28)

Method: torque off on ID 5; Present_Position read at 50 Hz while rolling by hand
to each end of travel (camera cable snug); unwrapped across the 0/4095 encoder wrap.
Script: so101_hardware/scripts/wrist_roll_travel.py. Data: wrist_roll_travel.csv.

| Run | Negative end | Positive end |
|---|---|---|
| 1 | -922 | 2870 |
| 2 | -926 | 2870 |
| 3 | -930 | 2899 |

Travel relative to calibration centre (2048), in encoder direction: about -262 deg
to +75 deg; span 337 deg; ends repeat within ~3 deg.

Finding: the encoder wrap (-180 deg) lies inside the travel. Readings from about +98
to +180 deg are physically -262 to -180 deg. The upstream URDF upper limit (+2.84 rad)
is unreachable and would accept those readings as valid.

Decision (option B): keep the EEPROM calibration; narrow the wrist_roll URDF limits to
about -170 deg to +65 deg (encoder direction) so commands never cross the wrap and
past-wrap readings fail the on_activate limit check. Final limits are set after the
wrist_roll sign is verified in read(). Rejected: re-centring the homing offset
(needs EEPROM writes, and changes the LeRobot calibration used by BT-101).
