#!/usr/bin/env python3
"""
Watch the control loop for a fixed time and summarise its timing and health.

Read-only: it only subscribes to /joint_states and /diagnostics. Loop periods come from
the /joint_states header stamps (set by the controller manager each cycle). If the stream
stops for longer than --stall seconds (for example after a bus error sends the hardware to
on_error), the run ends early and says when. At the end it prints the period percentiles,
the number of late cycles and the controller manager's own diagnostics.
"""

import argparse
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

KEEP = ('Controller Manager Activity', 'Hardware Components Activity')


class SoakMonitor(Node):
    """Collect /joint_states periods and the latest controller manager diagnostics."""

    def __init__(self):
        super().__init__('soak_monitor')
        self.last_stamp = None
        self.last_wall = None
        self.periods = []
        self.diagnostics = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 100)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)

    def on_joint_states(self, msg):
        """Record the time between consecutive stamps, in seconds."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None:
            self.periods.append(stamp - self.last_stamp)
        self.last_stamp = stamp
        self.last_wall = time.monotonic()

    def on_diagnostics(self, msg):
        """Keep the latest key/value pairs of the controller manager's statuses."""
        for status in msg.status:
            if any(k in status.name for k in KEEP):
                self.diagnostics[status.name] = [(v.key, v.value) for v in status.values]


def percentile(sorted_values, p):
    """Nearest-rank percentile of an already sorted list."""
    return sorted_values[int(p * (len(sorted_values) - 1))]


def report(node, elapsed_s, stalled):
    """Print the summary."""
    print(f'ran {elapsed_s:.0f} s, {len(node.periods) + 1} /joint_states messages')
    if stalled:
        print(f'STREAM STOPPED at {elapsed_s:.1f} s (check the launch log for the cause)')
    if node.periods:
        p = sorted(node.periods)
        ms = [1000.0 * x for x in (percentile(p, 0.5), percentile(p, 0.99), p[0], p[-1])]
        late = sum(1 for x in p if x > 0.015)
        print(f'period ms: p50 {ms[0]:.3f}, p99 {ms[1]:.3f}, min {ms[2]:.3f}, max {ms[3]:.3f}')
        print(f'rate {len(p) / sum(p):.2f} Hz, cycles over 15 ms: {late}')
    for name, values in node.diagnostics.items():
        print(name)
        for key, value in values:
            print(f'  {key}: {value}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1800.0, help='seconds (default 30 min)')
    parser.add_argument('--stall', type=float, default=1.0, help='seconds without /joint_states')
    args = parser.parse_args()

    rclpy.init()
    node = SoakMonitor()
    start = time.monotonic()
    stalled = False
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.last_wall is not None and time.monotonic() - node.last_wall > args.stall:
                stalled = True
                break
    except KeyboardInterrupt:
        pass
    finally:
        report(node, time.monotonic() - start, stalled)
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if stalled else 0


if __name__ == '__main__':
    sys.exit(main())
