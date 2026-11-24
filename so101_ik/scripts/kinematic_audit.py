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
PITCH_JOINTS = ['shoulder_lift', 'elbow_flex', 'wrist_flex']

# One error budget for every check: 0.01 mm at the tip, over ~0.4 m of reach
TIP_BUDGET_M = 1e-5
REACH_M = 0.4
ANGLE_TOL = TIP_BUDGET_M / REACH_M


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
    lower: float = 0.0
    upper: float = 0.0


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
            axis=_floats(j.find('axis'), 'xyz', [1.0, 0.0, 0.0]),
        )
        limit = j.find('limit')
        if limit is not None:
            joint.lower = float(limit.get('lower', 0.0))
            joint.upper = float(limit.get('upper', 0.0))
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


def angle_between(a, b):
    """Angle (rad) between two vectors, accurate near 0 and pi."""
    return np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))


def angle_from_perpendicular(a, b):
    """How far (rad) two vectors are from being perpendicular."""
    return np.arctan2(abs(np.dot(a, b)), np.linalg.norm(np.cross(a, b)))


def skew_line_distance(p1, a1, p2, a2):
    """Shortest distance between two non-parallel lines (point p, direction a)."""
    n = np.cross(a1, a2)
    return abs(np.dot(p2 - p1, n)) / np.linalg.norm(n)


def axes_in_base(joints, q):
    """Return {joint name: (origin, unit axis)} in base_link, both from the frame before q."""
    by_name = {j.name: j for j in joints.values()}
    out = {}
    for name, (t_joint, _) in joint_frames(joints, TIP, q).items():
        if name in ARM_JOINTS:
            axis = by_name[name].axis / np.linalg.norm(by_name[name].axis)
            out[name] = (t_joint[:3, 3], t_joint[:3, :3] @ axis)
    return out


def random_q(joints, rng):
    """One random joint vector, uniform inside every arm joint's URDF limits."""
    by_name = {j.name: j for j in joints.values()}
    return {n: rng.uniform(by_name[n].lower, by_name[n].upper) for n in ARM_JOINTS}


def check_b(joints, q):
    """Check B: the ideal-structure assumptions, as [(label, value, tolerance, unit)]."""
    ax = axes_in_base(joints, q)
    pan_p, pan_a = ax['shoulder_pan']
    lift_p, lift_a = ax['shoulder_lift']
    wf_p, wf_a = ax['wrist_flex']
    roll_p, roll_a = ax['wrist_roll']
    rows = [
        ('B1 lift || elbow', angle_between(lift_a, ax['elbow_flex'][1]), ANGLE_TOL, 'rad'),
        ('B1 lift || wrist_flex', angle_between(lift_a, wf_a), ANGLE_TOL, 'rad'),
        ('B2 pan || -Z', angle_between(pan_a, np.array([0.0, 0.0, -1.0])), ANGLE_TOL, 'rad'),
        ('B3 pan _|_ lift', angle_from_perpendicular(pan_a, lift_a), ANGLE_TOL, 'rad'),
        ('B4 roll _|_ wrist_flex', angle_from_perpendicular(roll_a, wf_a), ANGLE_TOL, 'rad'),
        ('B4 roll meets wrist_flex', skew_line_distance(wf_p, wf_a, roll_p, roll_a),
         TIP_BUDGET_M, 'm'),
    ]
    offsets = [np.dot(ax[n][0] - pan_p, lift_a) for n in PITCH_JOINTS]
    rows.append(('B5 pitch-plane spread', max(offsets) - min(offsets), TIP_BUDGET_M, 'm'))
    return rows


def print_check_b(joints, poses):
    """Check B at every pose; report the worst value of each row."""
    worst = {}
    for q in poses:
        for label, value, tol, unit in check_b(joints, q):
            if label not in worst or value > worst[label][0]:
                worst[label] = (value, tol, unit)
    print(f'\nCheck B, worst of {len(poses)} poses (q = 0 + random), '
          f'budget {TIP_BUDGET_M * 1000.0} mm / angle {ANGLE_TOL:.1e} rad')
    for label, (value, tol, unit) in worst.items():
        verdict = 'PASS' if value <= tol else 'FAIL'
        print(f'{label:<26} {value:10.2e} {unit:<3} (tol {tol:.1e}) {verdict}')


def plane_coords(joints, q):
    """In-plane (u, v) of each arm joint origin: u from the pan axis, v = height in base_link."""
    ax = axes_in_base(joints, q)
    pan_p, pan_a = ax['shoulder_pan']
    up = -pan_a
    pitch = ax['shoulder_lift'][1]
    forward = np.cross(pitch, up)
    return {n: np.array([np.dot(p - pan_p, forward), np.dot(p, up)]) for n, (p, _) in ax.items()}


def planar_params(joints, q):
    """Check C: link lengths (m) and in-plane link angles (rad) at q."""
    c = plane_coords(joints, q)
    links = [('L1', 'shoulder_lift', 'elbow_flex'), ('L2', 'elbow_flex', 'wrist_flex'),
             ('L3', 'wrist_flex', 'wrist_roll')]

    out = {}
    for label, a, b in links:
        d = c[b] - c[a]
        out[label] = (np.linalg.norm(d), np.arctan2(d[1], d[0]))
    return out


def wrap(angle):
    """Wrap an angle into (-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def print_check_c(joints, poses):
    """Check C: planar constants at zero, then rigidity and the angle model at random poses."""
    zero = planar_params(joints, {})
    s = plane_coords(joints, {})['shoulder_lift']
    print(f'\nC0 shoulder_lift points S (u, v) [mm]: {np.round(s * 1000.0, 3)}')
    for label, (length, alpha) in zero.items():
        print(f'C1 {label} = {length * 1000.0:9.3f} mm  alpha = {alpha:+.6f} rad')
    worst_len = 0.0
    worst_ang = 0.0
    for q in poses:
        now = planar_params(joints, q)
        cum = 0.0
        for label, pitch_joint in zip(['L1', 'L2', 'L3'], PITCH_JOINTS):
            cum += q[pitch_joint]
            worst_len = max(worst_len, abs(now[label][0] - zero[label][0]))
            model = zero[label][1] - cum
            worst_ang = max(worst_ang, abs(wrap(now[label][1] - model)))
    print(f'C2 max length change over {len(poses)} poses: {worst_len:.2e} m (rigid links)')
    print(f'C3 max error of theta_k = alpha_k - (q_lift + ... + q_k): {worst_ang:.2e} rad')


def print_check_e(joints, grid):
    """Check E: singular configurations reachable inside the URDF limits."""
    by_name = {j.name: j for j in joints.values()}
    zero = planar_params(joints, {})
    a1, a2 = zero['L1'][1], zero['L2'][1]
    elbow = by_name['elbow_flex']
    print()
    for label, q_e in (('straight', a2 - a1), ('folded', a2 - a1 + np.pi)):
        q_e = wrap(q_e)
        inside = elbow.lower <= q_e <= elbow.upper
        print(f'E1 elbow {label:<8} at elbow_flex = {q_e:+.4f} rad: '
              f'{"REACHABLE" if inside else "outside limits"}'
              f' [{elbow.lower:+.4f}, {elbow.upper:+.4f}]')
    best = (np.inf, None)
    best_vertical = (np.inf, None)
    lifts = np.linspace(by_name['shoulder_lift'].lower, by_name['shoulder_lift'].upper, grid)
    elbows = np.linspace(elbow.lower, elbow.upper, grid)
    flexes = np.linspace(by_name['wrist_flex'].lower, by_name['wrist_flex'].upper, grid)
    for q_l in lifts:
        for q_e in elbows:
            for q_w in flexes:
                q = {'shoulder_lift': q_l, 'elbow_flex': q_e, 'wrist_flex': q_w}
                ax = axes_in_base(joints, q)
                pan_p, pan_a = ax['shoulder_pan']
                g = ax['wrist_roll'][0] - pan_p
                r = np.linalg.norm(g - np.dot(g, pan_a) * pan_a)
                if r < best[0]:
                    best = (r, q)
                tilt = angle_between(ax['wrist_roll'][1], pan_a)
                tilt = min(tilt, np.pi - tilt)
                if tilt < best_vertical[0]:
                    best_vertical = (tilt, q)
    pose = {k: round(v, 3) for k, v in best[1].items()}
    print(f'E2 min distance of gripper_link from the pan axis: {best[0] * 1000.0:.3f} mm at '
          f'{pose} ({grid}^3 grid)')
    pose = {k: round(v, 3) for k, v in best_vertical[1].items()}
    print(f'E3 min angle between roll axis and pan axis: {best_vertical[0]:.4f} rad at {pose}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', help='expanded URDF file (xacro output)')
    parser.add_argument('--poses', type=int, default=20, help='random poses for B and C')
    parser.add_argument('--seed', type=int, default=0, help='random seed (repeatable runs)')
    parser.add_argument('--grid', type=int, default=31, help='grid points per joint for E')
    args = parser.parse_args()
    joints = load_joints(args.urdf)
    rng = np.random.default_rng(args.seed)
    poses = [{}] + [random_q(joints, rng) for _ in range(args.poses)]
    print_table_a(joints)
    print_check_b(joints, poses)
    print_check_c(joints, poses[1:])
    print_check_e(joints, args.grid)
    return 0


if __name__ == '__main__':
    sys.exit(main())
