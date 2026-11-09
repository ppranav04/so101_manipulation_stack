#!/usr/bin/env python3
"""
Record each joint's latest, minimum and maximum position from /joint_states.

Read-only: it only subscribes to /joint_states and never touches the servo bus.
Run it while the driver publishes (hw:=real, joint_state_broadcaster active).
Ctrl+C prints the final table and, with --csv, writes it to a file.
"""

import argparse
import csv
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointRangeRecorder(Node):
    """Track the latest, minimum and maximum position of every joint."""

    def __init__(self):
        super().__init__('joint_range_recorder')
        self.latest = {}
        self.lowest = {}
        self.highest = {}
        self.count = 0
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_timer(1.0, self.print_table)

    def on_joint_states(self, msg):
        """Fold one /joint_states message into the running min and max."""
        self.count += 1
        for name, position in zip(msg.name, msg.position):
            if not math.isfinite(position):
                continue
            self.latest[name] = position
            self.lowest[name] = min(position, self.lowest.get(name, position))
            self.highest[name] = max(position, self.highest.get(name, position))

    def rows(self):
        """Return (name, latest, min, max) for every joint seen, sorted by name."""
        return [(n, self.latest[n], self.lowest[n], self.highest[n]) for n in sorted(self.latest)]

    def print_table(self):
        """Print one table: radians, with degrees for the latest value."""
        lines = [f'--- {self.count} messages']
        for name, now, low, high in self.rows():
            lines.append(
                f'{name:14s} now {now:+8.4f} rad ({math.degrees(now):+7.1f} deg)'
                f'  min {low:+8.4f}  max {high:+8.4f}')
        print('\n'.join(lines), flush=True)


def main():
    parser = argparse.ArgumentParser(description='Record joint ranges from /joint_states.')
    parser.add_argument('--csv', help='write the final table to this CSV file')
    args = parser.parse_args()

    rclpy.init()
    node = JointRangeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_table()
        if args.csv:
            with open(args.csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['joint', 'latest_rad', 'min_rad', 'max_rad'])
                writer.writerows(node.rows())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
