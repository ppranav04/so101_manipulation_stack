#!/usr/bin/env python3
"""
Move one arm joint by a small amount with the trajectory controller, then back.

Starts from the trajectory controller's own reference (not the measured positions, which
sag under gravity), sends one goal that moves ONE joint by delta (rad) while every other
joint holds its reference, measures from /joint_states, then sends a second goal back to
the start and measures again. Prints the controller's result and the final error per leg.
Refuses |delta| above 0.3 rad and legs shorter than 1 s.
"""

import argparse
import math
import sys

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
STATE = '/joint_trajectory_controller/controller_state'
MAX_DELTA = 0.3    # rad
MIN_DURATION = 1.0  # s
SETTLE_S = 0.5     # wait after the result before measuring


class JtcStep(Node):
    """Hold the latest arm positions and talk to the trajectory controller."""

    def __init__(self):
        super().__init__('jtc_step')
        self.latest = None
        self.reference = None
        self.create_subscription(
            JointTrajectoryControllerState, STATE, self.on_controller_state, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def on_joint_states(self, msg):
        """Keep the arm joints' positions, only when all five are present and finite."""
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions and math.isfinite(positions[j]) for j in ARM_JOINTS):
            self.latest = [positions[j] for j in ARM_JOINTS]

    def on_controller_state(self, msg):
        """Keep the controller's reference positions for the arm joints."""
        reference = dict(zip(msg.joint_names, msg.reference.positions))
        if all(j in reference and math.isfinite(reference[j]) for j in ARM_JOINTS):
            self.reference = [reference[j] for j in ARM_JOINTS]

    def fresh(self, field, timeout_s):
        """Return self.<field> from a message received after this call, or None."""
        setattr(self, field, None)
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while getattr(self, field) is None and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return getattr(self, field)

    def settle(self, seconds):
        """Keep spinning for a while without blocking callbacks."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_to(self, target, duration_s):
        """Send one single-point goal and wait for its result. Return (code, text)."""
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = Duration(seconds=duration_s).to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None, 'goal rejected'
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result().result
        return result.error_code, result.error_string


def report(label, target, measured):
    """Print target, measured and error for every arm joint."""
    print(f'{label}:')
    for name, t, m in zip(ARM_JOINTS, target, measured):
        print(f'  {name:14s} target {t:+.4f}  measured {m:+.4f}  error {m - t:+.4f} rad')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('joint', choices=ARM_JOINTS)
    parser.add_argument('delta', type=float, help=f'rad, |delta| <= {MAX_DELTA}')
    parser.add_argument('--duration', type=float, default=3.0, help='seconds per leg')
    args = parser.parse_args()
    if not abs(args.delta) <= MAX_DELTA or args.duration < MIN_DURATION:
        print(f'refused: need |delta| <= {MAX_DELTA} rad and duration >= {MIN_DURATION} s')
        return 1

    rclpy.init()
    node = JtcStep()
    try:
        if not node.client.wait_for_server(timeout_sec=2.0):
            print(f'{ACTION} not available; is joint_trajectory_controller active?')
            return 1
        start = node.fresh('reference', 2.0)
        if start is None:
            print(f'no {STATE} with all arm joints')
            return 1

        target = list(start)
        target[ARM_JOINTS.index(args.joint)] += args.delta

        for label, goal in (('out', target), ('back', start)):
            code, text = node.move_to(goal, args.duration)
            print(f'{label}: result {code} {text}')
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                return 1
            node.settle(SETTLE_S)
            measured = node.fresh('latest', 1.0)
            if measured is None:
                print('no /joint_states after the move')
                return 1
            report(label, goal, measured)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
