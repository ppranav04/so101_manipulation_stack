#!/usr/bin/env python3
"""Plan and execute fixed joint-space goals with MoveIt and log one CSV row per run."""


import argparse
import csv
import math
import statistics
import sys
import time

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GROUP = 'arm'
# Named arm poses (rad, ARM_JOINTS order). 'home' must match so101.srdf; 'zero' is the
# mock/gz start pose (initial_value in so101.ros2_control.xacro).
POSES = {
    'home': [0.0, -0.8125, 0.4402, 0.7713, -1.6613],
    'zero': [0.0, 0.0, 0.0, 0.0, -1.57],
}

GOAL_TOLERANCE = 0.001
SETTLE_S = 0.5
FIELDS = ['run', 'start', 'goal', 'plan_code', 'exec_code', 'planning_time_s',
          'plan_wall_s', 'exec_wall_s', 'traj_duration_s', 'waypoints',
          'path_length_rad', 'final_error_rad']


class MoveItBaseline(Node):
    """Plan with move_group, execute the plan, and keep the latest arm positions."""

    def __init__(self):
        super().__init__('moveit_baseline')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.plan_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def on_joint_states(self, msg):
        """Keep the arm joints' positions, only when all five are present and finite."""
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions and math.isfinite(positions[j]) for j in ARM_JOINTS):
            self.latest = [positions[j] for j in ARM_JOINTS]

    def fresh_positions(self, timeout_s):
        """Return arm positions from a /joint_states message received after this call."""
        self.latest = None
        end = time.monotonic() + timeout_s
        while self.latest is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.latest

    def call(self, client, goal):
        """Send one action goal and wait for its result. Return the result or None."""
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        return done.result().result

    def plan(self, target, args):
        """Ask move_group for a joint-space plan to target. Return (result, wall s)."""
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = GROUP
        request.num_planning_attempts = 1
        request.allowed_planning_time = args.planning_time
        request.max_velocity_scaling_factor = args.velocity_scaling
        request.max_acceleration_scaling_factor = args.accel_scaling
        constraints = Constraints()
        for name, position in zip(ARM_JOINTS, target):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=position,
                tolerance_above=GOAL_TOLERANCE, tolerance_below=GOAL_TOLERANCE,
                weight=1.0))
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True

        start = time.monotonic()
        result = self.call(self.plan_client, goal)
        return result, time.monotonic() - start

    def execute(self, trajectory):
        """Execute a planned RobotTrajectory. Return (result, wall s)."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        start = time.monotonic()
        result = self.call(self.exec_client, goal)
        return result, time.monotonic() - start

    def settle(self, seconds):
        """Keep spinning for a while without blocking callbacks."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def path_length(joint_trajectory):
    """Sum of |delta q| over consecutive waypoints and all joints (rad)."""
    points = joint_trajectory.points
    return sum(
        abs(b - a)
        for p, q in zip(points, points[1:])
        for a, b in zip(p.positions, q.positions))


def duration_s(joint_trajectory):
    """Return time_from_start of the last waypoint in seconds."""
    if not joint_trajectory.points:
        return 0.0
    t = joint_trajectory.points[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


def code_of(result):
    """Return the MoveIt error code value, or 0 if the goal was rejected."""
    return 0 if result is None else result.error_code.val


def move(node, target, args):
    """Plan to target and execute. Return a dict with the measured fields."""
    row = dict.fromkeys(FIELDS, '')
    planned, row['plan_wall_s'] = node.plan(target, args)
    row['plan_code'] = code_of(planned)
    if row['plan_code'] != MoveItErrorCodes.SUCCESS:
        return row
    trajectory = planned.planned_trajectory.joint_trajectory
    row['planning_time_s'] = planned.planning_time
    row['traj_duration_s'] = duration_s(trajectory)
    row['waypoints'] = len(trajectory.points)
    row['path_length_rad'] = path_length(trajectory)

    executed, row['exec_wall_s'] = node.execute(planned.planned_trajectory)
    row['exec_code'] = code_of(executed)
    node.settle(SETTLE_S)
    measured = node.fresh_positions(1.0)
    if measured is None:
        row['exec_code'] = 0
    else:
        row['final_error_rad'] = max(abs(m - t) for m, t in zip(measured, target))
    return row


def summarise(rows):
    """Print median and max of the timing and path columns over successful runs."""
    ok = [r for r in rows if r['exec_code'] == MoveItErrorCodes.SUCCESS]
    print(f'{len(ok)}/{len(rows)} runs succeeded (plan and execute)')
    for field in ('planning_time_s', 'plan_wall_s', 'traj_duration_s',
                  'path_length_rad', 'final_error_rad'):
        values = [r[field] for r in ok if r[field] != '']
        if values:
            print(f'  {field:16s} median {statistics.median(values):.4f}  '
                  f'max {max(values):.4f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--poses', nargs='+', default=['home', 'zero'], choices=POSES)
    parser.add_argument('--velocity-scaling', type=float, default=0.1)
    parser.add_argument('--accel-scaling', type=float, default=0.1)
    parser.add_argument('--planning-time', type=float, default=5.0, help='seconds')
    parser.add_argument('--out', default='moveit_baseline.csv')
    args = parser.parse_args()
    if len(args.poses) < 2 or args.runs < 1:
        print('need at least two poses and one run')
        return 1
    for scale in (args.velocity_scaling, args.accel_scaling):
        if not 0.0 < scale <= 1.0:
            print('scaling factors must be in (0, 1]')
            return 1

    rclpy.init()
    node = MoveItBaseline()
    try:
        for client, name in ((node.plan_client, '/move_action'),
                             (node.exec_client, '/execute_trajectory')):
            if not client.wait_for_server(timeout_sec=10.0):
                print(f'{name} not available; is move_group running (moveit:=true)?')
                return 1
        if node.fresh_positions(2.0) is None:
            print(f'/joint_states does not contain all of {ARM_JOINTS}')
            return 1

        first = args.poses[0]
        print(f'positioning at {first} (not recorded)')
        row = move(node, POSES[first], args)
        if row['exec_code'] != MoveItErrorCodes.SUCCESS:
            print(f'positioning failed: plan {row["plan_code"]} exec {row["exec_code"]}')
            return 1

        rows = []
        with open(args.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            for run in range(args.runs):
                start = args.poses[run % len(args.poses)]
                goal = args.poses[(run + 1) % len(args.poses)]
                row = move(node, POSES[goal], args)
                row.update(run=run, start=start, goal=goal)
                writer.writerow(row)
                f.flush()
                rows.append(row)
                print(f'run {run}: {start} -> {goal} plan {row["plan_code"]} '
                      f'exec {row["exec_code"]} duration {row["traj_duration_s"]}')
                if row['exec_code'] != MoveItErrorCodes.SUCCESS:
                    print('stopping: the next run would not start from a known pose')
                    break
        summarise(rows)
        print(f'wrote {args.out}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
