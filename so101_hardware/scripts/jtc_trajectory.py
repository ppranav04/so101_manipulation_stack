#!/usr/bin/env python3
"""
Run one multi-joint, multi-point trajectory and report how well the arm tracked it.

Starts from the trajectory controller's own reference, moves all five arm joints through
small offsets (stopping at every waypoint) and returns to the start. While the goal runs it
records the largest |reference - feedback| per joint from controller_state. At the end it
prints the result code, that maximum tracking error and the final error per joint.
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
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
STATE = '/joint_trajectory_controller/controller_state'

# (seconds from start, offset in rad for pan, lift, elbow, wrist_flex, roll)
WAYPOINTS = [
    (3.0, [+0.15, +0.10, -0.10, -0.10, +0.15]),
    (6.0, [-0.15, +0.10, -0.10, -0.10, -0.15]),
    (9.0, [0.0, 0.0, 0.0, 0.0, 0.0]),
]
MAX_OFFSET = 0.2   # rad, refuse any waypoint further than this from the start
SETTLE_S = 0.5


class JtcTrajectory(Node):
    """Track the controller's reference and the worst tracking error during a goal."""

    def __init__(self):
        super().__init__('jtc_trajectory')
        self.reference = None
        self.feedback = None
        self.recording = False
        self.max_error = [0.0] * len(ARM_JOINTS)
        self.create_subscription(
            JointTrajectoryControllerState, STATE, self.on_controller_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def on_controller_state(self, msg):
        """Keep reference and feedback; while recording, keep the worst |error| per joint."""
        ref = dict(zip(msg.joint_names, msg.reference.positions))
        fb = dict(zip(msg.joint_names, msg.feedback.positions))
        if not all(j in ref and j in fb for j in ARM_JOINTS):
            return
        self.reference = [ref[j] for j in ARM_JOINTS]
        self.feedback = [fb[j] for j in ARM_JOINTS]
        if self.recording:
            for k, j in enumerate(ARM_JOINTS):
                self.max_error[k] = max(self.max_error[k], abs(ref[j] - fb[j]))

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

    def run(self, goal):
        """Send the goal, record tracking error until the result arrives, return (code, text)."""
        self.max_error = [0.0] * len(ARM_JOINTS)
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None, 'goal rejected'
        self.recording = True
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        self.recording = False
        result = done.result().result
        return result.error_code, result.error_string


def build_goal(start, scale):
    """Turn WAYPOINTS into a goal: start + scale * offset, zero velocity at every point."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM_JOINTS
    for t, offsets in WAYPOINTS:
        point = JointTrajectoryPoint()
        point.positions = [s + scale * o for s, o in zip(start, offsets)]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(seconds=t).to_msg()
        goal.trajectory.points.append(point)
    return goal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=float, default=1.0, help='shrink offsets, 0 < scale <= 1')
    args = parser.parse_args()
    if not 0.0 < args.scale <= 1.0:
        print('refused: need 0 < scale <= 1')
        return 1
    if any(abs(o) > MAX_OFFSET for _, offsets in WAYPOINTS for o in offsets):
        print(f'refused: a waypoint offset exceeds {MAX_OFFSET} rad')
        return 1

    rclpy.init()
    node = JtcTrajectory()
    try:
        if not node.client.wait_for_server(timeout_sec=5.0):
            print(f'{ACTION} not available; is joint_trajectory_controller active?')
            return 1
        start = node.fresh('reference', 2.0)
        if start is None or not all(math.isfinite(s) for s in start):
            print(f'no valid {STATE}')
            return 1

        code, text = node.run(build_goal(start, args.scale))
        print(f'result {code} {text}')
        node.settle(SETTLE_S)
        final = node.fresh('feedback', 1.0)
        print(f'{"joint":14s} {"max track err":>14s} {"final err":>10s}  (rad)')
        for k, name in enumerate(ARM_JOINTS):
            final_err = final[k] - start[k] if final is not None else float('nan')
            print(f'{name:14s} {node.max_error[k]:14.4f} {final_err:+10.4f}')
        return 0 if code == FollowJointTrajectory.Result.SUCCESSFUL else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
