# Measurements

Every number in the top-level README comes from one of these write-ups. Each one states the
question, the method, how many runs, the results and the limitations. Raw data (CSV) is in
`data/<topic>/`, next to the write-up that uses it.

| Write-up | Milestone | Date | Headline | Data |
|---|---|---|---|---|
| [calibration_persistence.md](calibration_persistence.md) | M2 | 2026-10-28 | EEPROM calibration survives power cycles | - |
| [wrist_roll_travel.md](wrist_roll_travel.md) | M2 | 2026-10-28 | ~338 deg travel, encoder wrap inside the travel | [data/wrist_roll_travel/](data/wrist_roll_travel/) |
| [velocity_unit.md](velocity_unit.md) | M2 | 2026-11-08 | Present_Velocity is counts/s (fit 0.996-0.997, R^2 >= 0.996) | - |
| [sync_read_latency.md](sync_read_latency.md) | M2 | 2026-11-08 | SYNC READ p50 1147-1149 us, p99 1265-1294 us, 0 failures in 6000 | - |
| [joint_limits.md](joint_limits.md) | M2 | 2026-11-09 | All joint signs +1; limits 35 counts inside the measured stops | [data/joint_limits/](data/joint_limits/) |
| [first_motion.md](first_motion.md) | M2 | 2026-11-10 | 5-joint trajectory: max tracking error 0.097 rad (elbow), final <= 0.02 rad | - |
| [control_loop.md](control_loop.md) | M2 | 2026-11-11 | 100 Hz loop, period p99 10.255 ms; 24 000 writes, 0 timeouts | - |
| [moveit_baseline.md](moveit_baseline.md) | M3 | 2026-11-15, 2026-11-17 | 10/10 plan + execute on mock, gz and the real arm; planning median 13-22 ms | [data/moveit_baseline/](data/moveit_baseline/) |
| [servo_voltage_glitches.md](servo_voltage_glitches.md) | M3 | 2026-11-17 | Undervoltage flag = single-read glitches on IDs 4-6 (also with torque off); debounced, 0 faults in 4 sessions | [data/servo_voltage_glitches/](data/servo_voltage_glitches/) |

## Conventions

- Angles in rad unless a column says counts (4096 counts per revolution).
- Dates are when the data was taken. A write-up updated later keeps its original data
  and says what changed.
- CSV files are kept as recorded; a write-up that excludes a run says which and why.
