#!/usr/bin/env python3
"""
Run the same smooth trajectory on the real arm and in Gazebo at the same time.

The two stacks run on different ROS domains (real on --real-domain, Gazebo on --sim-domain),
so their identically named nodes never meet. This script joins both domains, goes to a home
pose, flows through a set of waypoints around it without stopping, and returns home. At the
end it prints each arm's result and its largest tracking error per joint.
"""

import argparse
import os
import sys

from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from control_msgs.msg import JointTrajectoryControllerState
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
STATE = '/joint_trajectory_controller/controller_state'
GRIPPER_ACTION = '/parallel_gripper_controller/gripper_cmd'

# URDF limits (rad), same order as ARM_JOINTS. Every waypoint must stay inside them.
LIMITS = [(-1.9190, 1.9036), (-1.9067, 1.8622), (-1.8453, 1.5692), (-1.7717, 1.6935),
          (-2.96, -0.32)]

# A pose the real arm held during the first-motion runs (2026-11-10).
HOME = [0.0, -1.05, 0.79, 1.08, -1.47]
HOME_S = 4.0  # time to reach home from wherever the arm is

# Offsets from HOME (rad), visited in order and then back to HOME, with the time per segment.
# 'large' was checked along the whole spline: inside the limits, gripper >= 120 mm above the
# base, peak speed 0.99 rad/s (wrist_roll). It loads shoulder_lift and the elbow more.
MOTIONS = {
    'small': (2.0, [
        [+0.50, +0.00, -0.20, -0.20, +0.40],
        [+0.50, +0.15, -0.30, +0.10, -0.40],
        [+0.00, +0.20, -0.35, +0.20, +0.00],
        [-0.50, +0.15, -0.30, +0.10, +0.40],
        [-0.50, +0.00, -0.20, -0.20, -0.40],
        [+0.00, -0.10, +0.00, -0.25, +0.00],
    ]),
    'large': (2.5, [
        [+1.00, +0.00, -0.30, -0.50, +0.90],
        [+1.00, +0.25, -0.50, +0.30, -0.90],
        [+0.00, +0.35, -0.70, +0.50, +0.00],
        [-1.00, +0.25, -0.50, +0.30, +0.90],
        [-1.00, +0.00, -0.30, -0.50, -0.90],
        [+0.00, -0.20, +0.00, -0.40, +0.00],
    ]),
}


def waypoints(motion):
    """Return [(time, positions)] from home, through the motion's offsets, back to home."""
    segment_s, offsets = MOTIONS[motion]
    poses = [HOME] + [[h + o for h, o in zip(HOME, off)] for off in offsets] + [HOME]
    return [(HOME_S + i * segment_s, pose) for i, pose in enumerate(poses)]


def build_goal(points):
    """Cubic spline through every point: zero velocity at the ends, flowing in between."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM_JOINTS
    for i, (t, pose) in enumerate(points):
        if i == 0 or i == len(points) - 1:
            vel = [0.0] * len(pose)
        else:
            (t0, p0), (t1, p1) = points[i - 1], points[i + 1]
            vel = [(b - a) / (t1 - t0) for a, b in zip(p0, p1)]
        point = JointTrajectoryPoint()
        point.positions = list(pose)
        point.velocities = vel
        point.time_from_start = Duration(seconds=t).to_msg()
        goal.trajectory.points.append(point)
    return goal


class Arm:
    """One stack (real or sim): its own ROS context, node, executor and action client."""

    def __init__(self, label, domain_id):
        self.label = label
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = Node(f'demo_twin_{label}', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, FollowJointTrajectory, ACTION)
        self.gripper_client = ActionClient(self.node, ParallelGripperCommand, GRIPPER_ACTION)
        self.gripper = None
        self.node.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.max_error = [0.0] * len(ARM_JOINTS)
        self.recording = False
        self.node.create_subscription(
            JointTrajectoryControllerState, STATE, self.on_state, 10)
        self.send_future = None
        self.result_future = None

    def on_joint_states(self, msg):
        """Keep this arm's gripper position."""
        if 'gripper' in msg.name:
            self.gripper = msg.position[msg.name.index('gripper')]

    def on_state(self, msg):
        """While the goal runs, keep the worst |reference - feedback| per joint."""
        if not self.recording:
            return
        ref = dict(zip(msg.joint_names, msg.reference.positions))
        fb = dict(zip(msg.joint_names, msg.feedback.positions))
        for k, j in enumerate(ARM_JOINTS):
            if j in ref and j in fb:
                self.max_error[k] = max(self.max_error[k], abs(ref[j] - fb[j]))

    def spin_once(self):
        """Process this context's pending callbacks without blocking."""
        self.executor.spin_once(timeout_sec=0.0)

    def close(self):
        """Tear down this context."""
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)


def wait_servers(arms, timeout_s):
    """Return True once every arm's action server is visible."""
    for arm in arms:
        if not arm.client.wait_for_server(timeout_sec=timeout_s):
            print(f'{arm.label}: {ACTION} not available')
            return False
    return True


def sync_gripper(real, sim):
    """Send the real gripper's position to the sim gripper controller (no wait for the result)."""
    for _ in range(100):
        if real.gripper is not None:
            break
        real.executor.spin_once(timeout_sec=0.02)
    if real.gripper is None or not sim.gripper_client.wait_for_server(timeout_sec=5.0):
        print('gripper not synced (no real /joint_states or no sim gripper controller)')
        return
    goal = ParallelGripperCommand.Goal()
    goal.command.name = ['gripper']
    goal.command.position = [real.gripper]
    sim.gripper_client.send_goal_async(goal)
    print(f'sim gripper -> {real.gripper:.3f} rad (real)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real-domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', 0)))
    parser.add_argument('--sim-domain', type=int, default=1)
    parser.add_argument('--no-sim', action='store_true', help='real arm only')
    parser.add_argument('--no-real', action='store_true', help='Gazebo only')
    parser.add_argument('--motion', choices=sorted(MOTIONS), default='small')
    args = parser.parse_args()

    points = waypoints(args.motion)
    for _, pose in points:
        for (lo, hi), q, name in zip(LIMITS, pose, ARM_JOINTS):
            if not lo <= q <= hi:
                print(f'refused: {name} waypoint {q:.3f} outside [{lo}, {hi}]')
                return 1

    arms = []
    try:
        if not args.no_real:
            arms.append(Arm('real', args.real_domain))
        if not args.no_sim:
            arms.append(Arm('sim', args.sim_domain))
        if not arms or not wait_servers(arms, 5.0):
            return 1

        # The demo moves only the arm joints: first copy the real gripper position to the sim.
        real = next((a for a in arms if a.label == 'real'), None)
        sim = next((a for a in arms if a.label == 'sim'), None)
        if real is not None and sim is not None:
            sync_gripper(real, sim)

        goal = build_goal(points)
        for arm in arms:
            arm.send_future = arm.client.send_goal_async(goal)
        while not all(a.send_future.done() for a in arms):
            for arm in arms:
                arm.spin_once()
        for arm in arms:
            handle = arm.send_future.result()
            if handle is None or not handle.accepted:
                print(f'{arm.label}: goal rejected')
                return 1
            arm.recording = True
            arm.result_future = handle.get_result_async()
        print(f'moving: {points[-1][0]:.0f} s')

        while not all(a.result_future.done() for a in arms):
            for arm in arms:
                arm.spin_once()
                if arm.result_future.done():
                    arm.recording = False

        print(f'{"joint":14s}' + ''.join(f'{a.label + " max err":>16s}' for a in arms) + '  (rad)')
        for k, name in enumerate(ARM_JOINTS):
            print(f'{name:14s}' + ''.join(f'{a.max_error[k]:16.4f}' for a in arms))
        ok = True
        for arm in arms:
            result = arm.result_future.result().result
            print(f'{arm.label}: result {result.error_code} {result.error_string}')
            ok = ok and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return 0 if ok else 1
    finally:
        for arm in arms:
            arm.close()


if __name__ == '__main__':
    sys.exit(main())
