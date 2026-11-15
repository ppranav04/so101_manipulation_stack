# MoveIt baseline on mock and Gazebo (2026-11-15)

## Summary

MoveIt 2 plans and executes joint-space goals on the mock and Gazebo backends through the
same joint trajectory controller used on the real arm. Over 10 runs per backend between two
fixed poses, all 20 runs planned and executed successfully. Median planning time was 13 ms
(mock) and 22 ms (gz), with a maximum of 32 ms. Every path was within 0.005 rad of the
straight joint-space line, and every final error was within the 0.001 rad goal tolerance.
In Gazebo, a held joint moved by at most 6.6e-9 rad over 30 s, so the start-state check
before execution is not at risk from simulator drift. One operating issue was found and
explained: RViz's initial start state is not the robot's current state (see Known issue).

## Setup

- MoveIt 2.12.4 (ROS 2 Jazzy, apt), `move_group` started by `so101.launch.py moveit:=true`.
- Planning: OMPL only, RRTConnect, 1 planning attempt, 5 s allowed planning time.
  Kinematics: KDL. Time parameterisation: TOTG with joint_limits.yaml
  (1.0 rad/s, 1.0 rad/s^2 per joint).
- Scaling: velocity 0.1, acceleration 0.1, sent explicitly in every request.
- Goal: one `JointConstraint` per arm joint, tolerance +-0.001 rad.
- Poses (rad, pan / lift / elbow / wrist_flex / wrist_roll):
  - `home` (SRDF): 0, -0.8125, 0.4402, 0.7713, -1.6613
  - `zero` (mock/gz start pose): 0, 0, 0, 0, -1.57
- Script: `so101_bringup/scripts/moveit_baseline.py`. Each run plans with
  `/move_action` (`plan_only`), then executes the returned trajectory with
  `/execute_trajectory`, then waits 0.5 s and reads `/joint_states` once. Runs alternate
  home -> zero -> home. A first positioning move to `home` is not recorded.
- Timing: `planning_time` is the value reported by move_group. Wall times use the
  script's monotonic clock. Trajectory duration is the last waypoint's `time_from_start`,
  which is in simulation time on gz.

```
ros2 launch so101_bringup so101.launch.py hw:=mock moveit:=true rviz:=false
ros2 run so101_bringup moveit_baseline.py --runs 10 --out moveit_baseline_mock.csv
```

## Results, 10 runs per backend

| | mock | gz |
|---|---|---|
| Planned and executed | 10/10 | 10/10 |
| `planning_time`, median (max) | 12.9 (32.1) ms | 21.5 (28.9) ms |
| Plan wall time incl. TOTG and action round trip, median (max) | 36.2 (51.4) ms | 41.2 (52.2) ms |
| Trajectory duration, min-max | 9.108-9.142 s | 9.115-9.130 s |
| Execution wall time, min-max | 9.184-9.193 s | 10.418-10.763 s |
| Waypoints | 93 | 93 |
| Joint-space path length, min-max | 2.1138-2.1170 rad | 2.1112-2.1161 rad |
| Final error (max over joints), median (max) | 0.0009 (0.0010) rad | 0.0009 (0.0010) rad |

Path length is the sum of |dq| over consecutive waypoints and all five joints. The straight
joint-space line between the two poses is 2.1153 rad.

## Interpretation

- **Paths are straight.** Every path is within 0.005 rad of the straight line. Paths
  slightly shorter than the line are possible because both ends may lie anywhere inside
  the +-0.001 rad goal box on each of the five joints. On this pose pair, RRTConnect plus
  path simplification produced no detours in 20 runs.
- **Final error measures planner slack, not tracking.** Final errors stop at the
  0.001 rad goal tolerance on both backends. The planner places the last waypoint anywhere
  inside the goal box, and both backends end exactly on that waypoint. On the real arm, the
  same column measures tracking error.
- **Planning does not depend on the backend.** Planning time, path and duration agree
  between mock and gz within run-to-run spread, as expected: the planner only sees joint
  states.
- **Gazebo executes slower than real time.** On mock, execution wall time exceeds
  trajectory duration by 0.05-0.08 s. On gz it exceeds it by 1.3-1.6 s. Trajectory duration
  is in simulation time, so the ratio of duration to wall time estimates the real-time
  factor (RTF): about 0.86 for the gz baseline. In a later session with the RTF recorded,
  three moves gave ratios of 0.885-0.907. The gz stats topic over the same ~5 min gave a
  per-sample median RTF of 0.97 (range 0.19-1.10; 31% of 2958 samples below 0.9; idle
  periods included). The ratio is consistent with the RTF dropping while the arm moves. This
  is not measured directly: RTF was not recorded during the baseline itself.

## Gazebo hold drift

Question: after a move ends, does a held joint drift enough to fail MoveIt's start-state
check (`allowed_start_tolerance` 0.01 rad) on the next execution?

Method: `joint_range_recorder.py` (min/max/latest per joint from `/joint_states`), 30 s.
- After move (x3): started immediately after a home -> zero run (chained with `&&`).
- Idle (x3): the arm untouched for at least 60 s, then three recordings back to back.

| Condition | Runs | Largest range over 30 s, any arm joint |
|---|---|---|
| After move | 3 | 6.6e-9 rad (shoulder_lift) |
| Idle | 3 | 8.7e-19 rad (wrist_flex) |

Final positions were within 0.00091 rad of `zero` in all six files (the goal-tolerance
slack described above).

Gazebo does not drift: its position interface holds the commanded position, and gravity
does not act against a torque-limited motor as it does on the STS3215. The real arm's elbow
creeps by 0.03-0.08 rad while held (first_motion.md), so this result does not transfer to
the real arm. The same recording has to be made there before choosing the start tolerance.

## Known issue: RViz start state

In RViz's MotionPlanning panel, the first execution after a fresh launch is refused with
"start point deviates from current robot state more than 0.01 at joint 'wrist_roll'". Six
fresh gz launches, one Plan then Execute each, 2 s between them:

| Start State before the first Plan | Result |
|---|---|
| Left as RViz set it (3 launches) | 3/3 refused, all at `wrist_roll` |
| Re-selected `<current>` (3 launches) | 3/3 executed |

In one of the `<current>` launches the start state was first left on another named state
by mistake. That attempt was refused at `shoulder_pan`, and it executed after `<current>`
was selected. So the check refuses any stale start state, not only wrist_roll.

The plan-then-execute gap is not the cause: gaps of 4 to 10 s executed normally once the
start state was current. The script never hits this, because it sends no start state and
move_group then plans from the current state.

Likely mechanism (not verified): RViz starts from the robot model's default state. For a
revolute joint the default is 0 when 0 is inside its limits, otherwise the midpoint.
wrist_roll is the only arm joint whose limits (-2.96..-0.32 rad) exclude 0, so its default
(-1.64) differs from the gz start pose (-1.57) by about 0.07 rad.

Operating rule: before the first Plan after every launch, set Start State to `<current>`.
Keep `allowed_start_tolerance` at 0.01 rad; on the real arm this check is what turns a stale
start state into a refusal instead of a jump.

## Limitations

- One pose pair (home <-> zero), 10 runs per backend. This pair gives near-straight
  paths. Harder pairs (long wrist_roll travel, near joint limits) may show planner
  randomness that these runs do not.
- Final error is limited by the 0.001 rad goal tolerance and does not measure tracking on
  mock or gz.
- RTF was not recorded during the baseline runs. The execution-time explanation is
  inferred from the duration/wall-time ratio.
- Real arm not included (next step).

## Files

- `moveit_baseline_mock.csv`, `moveit_baseline_gz.csv`: one row per run (plan and execute
  codes, planning time, wall times, trajectory duration, waypoints, path length, final
  error).
- `gz_hold_after_move_{1,2,3}.csv`, `gz_hold_idle_{1,2,3}.csv`: latest/min/max per joint
  over 30 s.
