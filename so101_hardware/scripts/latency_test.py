#!/usr/bin/env python3
"""
Measure command-to-motion latency of one joint through the trajectory controller.

Each trial sends a short linear ramp (delta rad over --ramp s) to one joint while the others
hold, and records /joint_trajectory_controller/controller_state. Latency is the time from
the reference first moving more than --threshold rad from its start to the feedback first
doing the same, both read from the controller_state stamps (one clock, 10 ms cycles).
Trials alternate direction, so the joint ends where it started.
"""

import argparse
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
MAX_DELTA = 0.2  # rad


class LatencyTest(Node):
    """Record (stamp, reference, feedback) of every arm joint while a goal runs."""

    def __init__(self):
        super().__init__('latency_test')
        self.reference = None
        self.feedback = None
        self.recording = False
        self.samples = []
        self.create_subscription(
            JointTrajectoryControllerState, STATE, self.on_controller_state, 100)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def on_controller_state(self, msg):
        """Keep the latest state and, while recording, every sample with its stamp."""
        ref = dict(zip(msg.joint_names, msg.reference.positions))
        fb = dict(zip(msg.joint_names, msg.feedback.positions))
        if not all(j in ref and j in fb for j in ARM_JOINTS):
            return
        self.reference = [ref[j] for j in ARM_JOINTS]
        self.feedback = [fb[j] for j in ARM_JOINTS]
        if self.recording:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append((stamp, list(self.reference), list(self.feedback)))

    def spin_for(self, seconds):
        """Keep spinning for a while without blocking callbacks."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def fresh(self, timeout_s):
        """Return (reference, feedback) from a message received after this call."""
        self.reference = None
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while self.reference is None and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        return self.reference, self.feedback

    def ramp(self, target, ramp_s):
        """Send a single-point goal (positions only, so linear) and record until done."""
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start = Duration(seconds=ramp_s).to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        self.samples = []
        self.recording = True
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.recording = False
            return None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        self.recording = False
        return done.result().result.error_code


def first_crossing(samples, k, field, start, threshold):
    """Stamp of the first sample whose field (1 = reference, 2 = feedback) leaves start."""
    for s in samples:
        if abs(s[field][k] - start) > threshold:
            return s[0]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('joint', choices=ARM_JOINTS)
    parser.add_argument('--delta', type=float, default=0.1, help=f'rad, <= {MAX_DELTA}')
    parser.add_argument('--ramp', type=float, default=0.25, help='seconds per ramp')
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--threshold', type=float, default=0.005, help='rad')
    args = parser.parse_args()
    if not (0.0 < args.delta <= MAX_DELTA and args.ramp >= 0.1 and args.trials >= 1):
        print(f'refused: need 0 < delta <= {MAX_DELTA}, ramp >= 0.1 s, trials >= 1')
        return 1

    rclpy.init()
    node = LatencyTest()
    k = ARM_JOINTS.index(args.joint)
    latencies = []
    try:
        if not node.client.wait_for_server(timeout_sec=2.0):
            print(f'{ACTION} not available; is joint_trajectory_controller active?')
            return 1
        for trial in range(args.trials):
            ref0, fb0 = node.fresh(2.0)
            if ref0 is None:
                print(f'no {STATE}')
                return 1
            target = list(ref0)
            target[k] += args.delta if trial % 2 == 0 else -args.delta
            code = node.ramp(target, args.ramp)
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                print(f'trial {trial}: result {code}, stopping')
                return 1
            t_ref = first_crossing(node.samples, k, 1, ref0[k], args.threshold)
            t_fb = first_crossing(node.samples, k, 2, fb0[k], args.threshold)
            if t_ref is None or t_fb is None:
                print(f'trial {trial}: no crossing (ref {t_ref}, feedback {t_fb})')
            else:
                latencies.append(1000.0 * (t_fb - t_ref))
                print(f'trial {trial}: latency {latencies[-1]:.1f} ms')
            node.spin_for(0.5)
    finally:
        if latencies:
            s = sorted(latencies)
            print(f'{args.joint}: {len(s)} trials, latency ms p50 {s[len(s) // 2]:.1f}, '
                  f'min {s[0]:.1f}, max {s[-1]:.1f} (10 ms resolution)')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
