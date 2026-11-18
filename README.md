# so101_manipulation_stack

ROS 2 software stack for the **SO-101** 5-DOF arm: see an object on a table, pick it up, and place it.

> **Status:** M3 complete. Latest release: `v0.3-moveit`. \
  **Next:** M4 (closed-form IK).

**Demo:** the real arm, Gazebo and RViz running the same trajectory.

https://github.com/user-attachments/assets/3eec670b-1465-4dd4-aecc-fa23d8b8ae2a


| Component | Status |
|---|---|
| Package skeletons, CI, Docker | ✅ `v0.0-skeleton` |
| Robot description (URDF/xacro, `hw:=mock\|gz\|real`) | ✅ joint signs verified on the arm, limits from measured travel |
| Mock backend (`ros2_control`) + launch test in CI | ✅ `v0.1-sim` |
| Real hardware driver (C++, STS3215 bus) | ✅ `v0.2-real-hw` |
| Gazebo Harmonic backend | ✅ `v0.2-real-hw` |
| MoveIt 2 | ✅ `v0.3-moveit` |
| Closed-form IK | ⏳ In Progress |


## Results

**MoveIt 2 (M3):** `home` <-> `zero`, 10 runs per backend, RRTConnect, scaling 0.1.

| | mock | Gazebo | real arm |
|---|---|---|---|
| Plan + execute | 10/10 | 10/10 | 10/10 |
| Planning time, median (max) | 13 (32) ms | 22 (29) ms | 22 (34) ms |
| Final joint error | <= 0.001 rad | <= 0.001 rad | 0.042-0.103 rad (gravity sag) |

**Driver (M2):**

| Measurement | Result |
|---|---|
| Control loop | 100 Hz, p99 period 10.255 ms, 0 of 11 996 cycles over 15 ms |
| SYNC READ, six servos | p50 1.15 ms, 0 failures in 6000 |
| Write reliability | 24 000 writes, 0 timeouts |
| Command-to-motion latency | p50 90-100 ms |

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
| `so101_moveit_config` | ament_cmake | Moveit config files |

### Known limitations

- **Gravity sag:** the servos hold position with a proportional loop, so loaded joints settle
  short of their goal (elbow up to 0.10 rad with the arm upright). The elbow goal tolerance
  is 0.15 rad.
- **Servo voltage flag:** the last servos in the chain report single-read undervoltage
  glitches; the driver faults only after 10 consecutive flagged reads.
- **RViz:** re-select Start State `<current>` before every Plan.
- **Not measured:** a 30 min soak test.


## Acknowledgements

The robot description and meshes are derived from TheRobotStudio's
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101`, commit
[`5f6d2b8`](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101)),
used under the
[Apache-2.0 license](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/LICENSE).

## License

Apache-2.0. See [LICENSE](LICENSE).
