# so101_manipulation_stack

ROS 2 software stack for the **SO-101** 5-DOF arm: see an object on a table, pick it up, and place it.

> **Status:** M2 complete. Latest release: `v0.2-real-hw`. \
  **Next:** M3 (MoveIt in sim and on the real arm).

**Demo:** the real arm, Gazebo and RViz running the same trajectory.

https://github.com/user-attachments/assets/3eec670b-1465-4dd4-aecc-fa23d8b8ae2a


| Component | Status |
|---|---|
| Package skeletons, CI, Docker | ✅ `v0.0-skeleton` |
| Robot description (URDF/xacro, `hw:=mock\|gz\|real`) | ✅ joint signs verified on the arm, limits from measured travel |
| Mock backend (`ros2_control`) + launch test in CI | ✅ `v0.1-sim` |
| Real hardware driver (C++, STS3215 bus) | ✅ `v0.2-real-hw` |
| Gazebo Harmonic backend | ✅ `v0.2-real-hw` |
| MoveIt 2 | ⏳ M3 |


## M2 results (real arm)

| Measurement | Result | Details |
|---|---|---|
| Control loop | 100 Hz, period p50 10.000 ms, p99 10.255 ms, 0 of 11 996 cycles over 15 ms | [control_loop.md](docs/measurements/control_loop.md) |
| `read()` per cycle (one SYNC READ, 6 servos) | avg 1.25 ms (1.12-3.11 ms) | [control_loop.md](docs/measurements/control_loop.md) |
| SYNC READ latency | p50 1147-1149 us, p99 1265-1294 us, 0 failures in 6000 | [sync_read_latency.md](docs/measurements/sync_read_latency.md) |
| WRITE reliability | 24 000 writes, 0 timeouts; 40/40 activations | [control_loop.md](docs/measurements/control_loop.md) |
| Command-to-motion latency (beyond an ideal plant) | p50 90-100 ms | [control_loop.md](docs/measurements/control_loop.md) |
| Trajectory tracking, 5-joint trajectory | max error 0.097 rad (elbow), final error <= 0.02 rad | [first_motion.md](docs/measurements/first_motion.md) |
| Present_Velocity unit | counts/s, fit 0.996-0.997, R^2 >= 0.996 | [velocity_unit.md](docs/measurements/velocity_unit.md) |
| Joint signs, travel, limits | all signs +1; limits 35 counts inside the measured stops | [joint_limits.md](docs/measurements/joint_limits.md) |
| Calibration persistence | EEPROM calibration survives power cycles | [calibration_persistence.md](docs/measurements/calibration_persistence.md) |
| wrist_roll travel | ~338 deg, encoder wrap inside the travel | [wrist_roll_travel.md](docs/measurements/wrist_roll_travel.md) |
| Long-run stability (30 min) | not measured | |


## Goals

1. **Hardware interface:** a C++ `ros2_control` driver for the STS3215 servo bus, with measured loop rate and latency.
2. **Mock, sim and real:** the same controllers run on mock hardware, Gazebo Harmonic and the real arm.
3. **Inverse kinematics:** a closed-form 5-DOF IK solver as a MoveIt 2 plugin, benchmarked against KDL.
4. **Camera calibration:** intrinsics and hand-eye calibration, with error reported in mm.
5. **Pick and place:** RGB detection to a top-down grasp planned with MoveIt, evaluated over 50+ trials.
6. **Engineering workflow:** gtest and pytest, Docker, and GitHub Actions CI.

## Hardware and environment

| Item | Details |
|---|---|
| Arm | SO-101 follower (5-DOF + gripper), Feetech STS3215 servos |
| Cameras | Wrist camera + external RGB webcam |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy |
| Simulation | Gazebo Harmonic (`gz_ros2_control`) |
| Planning | MoveIt 2 |

## Packages

| Package | Build type | Purpose |
|---|---|---|
| `so101_description` | ament_cmake | URDF / xacro and meshes |
| `so101_hardware` | ament_cmake | `ros2_control` hardware interface (C++) |
| `so101_ik` | ament_cmake | Closed-form IK as a MoveIt kinematics plugin (C++) |
| `so101_bringup` | ament_cmake | Launch files and shared configuration |
| `so101_calibration` | ament_python | Camera intrinsics and hand-eye calibration tooling |
| `so101_perception` | ament_python | Object detection and back-projection |

`so101_moveit_config` will be generated with the MoveIt Setup Assistant in M3.

## Acknowledgements

The robot description and meshes are derived from TheRobotStudio's
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101`, commit
[`5f6d2b8`](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101)),
used under the
[Apache-2.0 license](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/LICENSE).

## License

Apache-2.0. See [LICENSE](LICENSE).
