#!/usr/bin/env python3
"""Print every arm joint's origin and axis in base_link at zero, from an expanded URDF."""

import argparse
from dataclasses import dataclass
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE = 'base_link'
TIP = 'gripper_frame_link'
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


@dataclass
class Joint:
    """One URDF <joint>: its frame offset from the parent and its rotation axis."""

    name: str
    kind: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray


def _floats(element, attribute, default):
    """Read a space-separated attribute as a float vector, or the default if absent."""
    if element is None or element.get(attribute) is None:
        return np.array(default, dtype=float)
    array = np.array([])
    for v in element.get(attribute).split():
        array = np.append(array, [float(v)])
    return array


def load_joints(urdf_path):
    """Return {child link name: Joint} for every joint in the URDF."""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall('joint'):
        origin = j.find('origin')
        joint = Joint(
            name=j.get('name'),
            kind=j.get('type'),
            parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=_floats(origin, 'xyz', [0.0, 0.0, 0.0]),
            rpy=_floats(origin, 'rpy', [0.0, 0.0, 0.0]),
            axis=_floats(j.find('axis'), 'xyz', [1.0, 0.0, 0.0])
        )
        joints[joint.child] = joint
    return joints


def rpy_matrix(roll, pitch, yaw):
    """URDF rpy: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_matrix(axis, angle):
    """Rodrigues: rotates by angle (rad) about a unit axis."""
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def origin_transform(joint):
    """4x4 transform from the parent link frame to the joint frame (before q)."""
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*joint.rpy)
    t[:3, 3] = joint.xyz
    return t


def chain(joints, tip):
    """Joints from BASE down to tip, in order (walks child -> parent, then reverses)."""
    out = []
    link = tip
    while link != BASE:
        if link not in joints:
            raise ValueError(f'{tip} is not below {BASE}')
        out.append(joints[link])
        link = joints[link].parent
    return out[::-1]


def joint_frames(joints, tip, q):
    """Return {joint name: (T_base_jointframe before q, T_base_child after q)}."""
    t = np.eye(4)
    frames = {}
    for joint in chain(joints, tip):
        t_joint = t @ origin_transform(joint)
        r = np.eye(4)
        if joint.kind in ('revolute', 'continuous'):
            r[:3, :3] = axis_angle_matrix(joint.axis, q.get(joint.name, 0.0))
        t = t_joint @ r
        frames[joint.name] = (t_joint, t)
    return frames


def print_table_a(joints):
    """Table A: origin (mm) and axis (unit vector) of each arm joint in base_link at q = 0."""
    frames = joint_frames(joints, TIP, {})
    print(f'{"joint":<14} {"origin in base_link [mm]":>30}   {"axis in base_link":>26}')
    for name in ARM_JOINTS:
        t_joint, _ = frames[name]
        joint = next(j for j in joints.values() if j.name == name)
        origin_mm = t_joint[:3, 3] * 1000.0
        axis = t_joint[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis))
        print(f'{name:<14} {np.array2string(origin_mm, precision=3, suppress_small=True):>30}   '
              f'{np.array2string(axis, precision=6, suppress_small=True):>26}')
    t_tip = frames['gripper_frame_joint'][1]
    print(f'{"TCP (" + TIP + ")"} origin [mm]: '
          f'{np.array2string(t_tip[:3, 3] * 1000.0, precision=3, suppress_small=True)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', help='expanded URDF file (xacro output)')
    args = parser.parse_args()
    print_table_a(load_joints(args.urdf))
    return 0


if __name__ == '__main__':
    sys.exit(main())
