#!/usr/bin/env python3
"""
Move the arm from its rest pose to a ready pose that MoveIt accepts as a start state.

At rest, elbow_flex sits on its stop, outside its URDF limit. The driver's activation band
accepts that, but MoveIt refuses to plan from a start state outside the limits. This sends one
goal to the trajectory controller: every arm joint closer than MARGIN to a URDF limit (or past
it) is moved to MARGIN inside that limit; every other joint holds its controller reference.
Limits come from /robot_description. Refuses moves larger than MAX_MOVE. Afterwards it reads
/joint_states and reports whether every arm joint is inside its limits.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
STATE = '/joint_trajectory_controller/controller_state'
MARGIN = 0.18      # rad inside the limit; > measured elbow sag (0.034 rad) with room to spare
MAX_MOVE = 0.3     # rad; the activation band is 0.077 rad, so more means something is wrong
SPEED = 0.1        # rad/s for the largest move
MIN_DURATION = 2.0  # s
SETTLE_S = 1.0     # wait after the result before measuring


def parse_limits(urdf, names):
    """Return {joint: (lower, upper)} for names from a URDF string, or None if any is missing."""
    limits = {}
    for joint in ET.fromstring(urdf).iter('joint'):
        limit = joint.find('limit')
        if joint.get('name') in names and limit is not None:
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits if all(n in limits for n in names) else None


def ready_pose(start, limits):
    """Move each joint that is within MARGIN of a limit (or past it) to MARGIN inside it."""
    target = []
    for name, q in zip(ARM_JOINTS, start):
        lower, upper = limits[name]
        target.append(min(max(q, lower + MARGIN), upper - MARGIN))
    return target


class Unpark(Node):
    """Hold the URDF, the controller reference and the measured positions; send one goal."""

    def __init__(self):
        super().__init__('unpark')
        self.urdf = None
        self.reference = None
        self.latest = None
        # robot_state_publisher latches /robot_description: transient local, keep the last one.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_urdf, latched)
        self.create_subscription(
            JointTrajectoryControllerState, STATE, self.on_controller_state, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def on_urdf(self, msg):
        """Keep the robot description."""
        self.urdf = msg.data

    def on_controller_state(self, msg):
        """Keep the controller's reference positions for the arm joints."""
        reference = dict(zip(msg.joint_names, msg.reference.positions))
        if all(j in reference and math.isfinite(reference[j]) for j in ARM_JOINTS):
            self.reference = [reference[j] for j in ARM_JOINTS]

    def on_joint_states(self, msg):
        """Keep the arm joints' positions, only when all five are present and finite."""
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions and math.isfinite(positions[j]) for j in ARM_JOINTS):
            self.latest = [positions[j] for j in ARM_JOINTS]

    def wait_for(self, field, timeout_s, fresh=False):
        """Spin until self.<field> is set (cleared first if fresh). Return it or None."""
        if fresh:
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
        point.velocities = [0.0] * len(ARM_JOINTS)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='print the target, do not move')
    args = parser.parse_args()

    rclpy.init()
    node = Unpark()
    try:
        urdf = node.wait_for('urdf', 5.0)
        limits = parse_limits(urdf, ARM_JOINTS) if urdf is not None else None
        if limits is None:
            print('no /robot_description with limits for every arm joint')
            return 1
        if not node.client.wait_for_server(timeout_sec=5.0):
            print(f'{ACTION} not available; is joint_trajectory_controller active?')
            return 1
        start = node.wait_for('reference', 2.0, fresh=True)
        if start is None:
            print(f'no {STATE} with all arm joints')
            return 1

        target = ready_pose(start, limits)
        moves = [t - s for s, t in zip(start, target)]
        for name, s, t in zip(ARM_JOINTS, start, target):
            print(f'  {name:14s} {s:+.4f} -> {t:+.4f} rad')
        largest = max(abs(m) for m in moves)
        if largest < 1e-6:
            print('already ready: every joint is at least MARGIN inside its limits')
            return 0
        if largest > MAX_MOVE:
            print(f'refused: a joint would move {largest:.3f} rad (> {MAX_MOVE})')
            return 1
        if args.dry_run:
            return 0

        code, text = node.move_to(target, max(MIN_DURATION, largest / SPEED))
        print(f'result {code} {text}')
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            return 1
        node.settle(SETTLE_S)
        measured = node.wait_for('latest', 1.0, fresh=True)
        if measured is None:
            print('no /joint_states after the move')
            return 1
        inside = True
        for name, m in zip(ARM_JOINTS, measured):
            lower, upper = limits[name]
            ok = lower <= m <= upper
            inside = inside and ok
            print(f'  {name:14s} measured {m:+.4f} rad  {"inside" if ok else "OUTSIDE"}')
        print('ready for MoveIt' if inside else 'still outside the limits')
        return 0 if inside else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
