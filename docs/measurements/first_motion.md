# First powered motion (2026-11-10)

## Summary

The driver moves the arm through the joint trajectory controller. Single-joint moves
(8 legs) and a 5-joint, 3-waypoint trajectory (4 runs) all reached their goals. At full
scale the largest tracking error during motion was 0.097 rad (elbow_flex); final errors were
at most 0.020 rad. Loaded joints settle short of their goal (gravity sag), and the held
elbow creeps further over time.

## Setup

- hw:=real, motion_controllers:=true; joint_trajectory_controller at 100 Hz.
- Driver limits: URDF joint limits, velocity limit 5.0 rad/s (0.05 rad per cycle).
- JTC tolerances: path (trajectory) 0.2 rad on every joint; goal 0.02 pan and wrist_roll,
  0.05 wrist_flex, 0.1 shoulder_lift and elbow_flex.
- Scripts: so101_hardware/scripts/jtc_step.py (one joint out and back) and
  jtc_trajectory.py (waypoints at 3, 6, 9 s, offsets up to 0.15 rad, zero velocity at each
  waypoint, so cubic segments).
- Error = measured - commanded. Tracking error = max |reference - feedback| during the goal,
  from /joint_trajectory_controller/controller_state.

## Single-joint moves

All 8 legs reached their goals (4 moves out and back).

| Joint | Steady-state error (rad) |
|---|---|
| shoulder_pan | -0.0067 approaching from below, +0.0077 from above |
| shoulder_lift | +0.0123 (held) |
| wrist_flex | +0.0138 or -0.0015, depending on the approach direction |
| elbow_flex | 0.015 after its own move, growing to 0.081 while held |

## Trajectory, scale 1.0, 3 runs

| Joint | Max tracking error (rad) | Final error (rad) |
|---|---|---|
| shoulder_pan | 0.0241-0.0278 | -0.0092 to -0.0107 |
| shoulder_lift | 0.0343-0.0352 | +0.0184 to +0.0199 |
| elbow_flex | 0.0938-0.0966 | +0.0077 to +0.0153 |
| wrist_flex | 0.0381-0.0404 | -0.0031 to 0.0000 |
| wrist_roll | 0.0322-0.0360 | -0.0138 to -0.0199 |

## Observations

- Gravity-loaded joints settle short of the goal (position servos, P control). The
  elbow lifting its forearm was the worst case: 0.077 rad.
- The held elbow keeps drifting while its command stays constant: 0.031 -> 0.081 rad over
  several moves of other joints. Cause not identified.
- pan and wrist_roll stop 4-13 counts short, on the side they approach from.
- Activation sometimes fails with a write timeout inside on_activate (4 of about 9
  attempts; torque was off each time and the failure was handled). A test with a fake bus
  showed that the controller manager's read() does not run during activation, so the
  cause is on the bus/servo side; not yet identified.
- The supply is 5 V; the servos raise a voltage error below 4.0 V. The gripper reported
  one twice while holding (torque on); the supply voltage under load is not yet measured.
