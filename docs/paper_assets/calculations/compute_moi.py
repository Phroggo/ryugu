#!/usr/bin/env python3
"""
Body-z moment of inertia of the hopper, computed from the deployed model.sdf
via the parallel-axis theorem.

Backs the paper's claim (Sec. 3.2): "I_bot = 0.012-0.020 kg.m^2 about the body
z-axis, posture-dependent (legs retracted vs. splayed), computed from the
model's per-link inertias via the parallel-axis theorem."

Advisor review comment #17 ("keep the calculations ready").

Method
------
1. Parse every <link> from models/spacehopper/model.sdf: mass, 3x3 inertia
   tensor (about that link's own CoM, expressed in the link frame), and the
   link's default pose, resolved through SDF 1.8 `relative_to` frame chains.
2. Rotate each leg link by its commanded joint angle about the joint axis
   (0 1 0 in the child link frame, per model.sdf), propagating down the
   hip -> thigh -> knee -> calf chain.
3. For each posture: compute the whole-body CoM, then
       I_zz = sum_i [ (R_i I_i R_i^T)_zz + m_i * (dx_i^2 + dy_i^2) ]
   where d is the link CoM offset from the body CoM in the model frame.
   The second term is the parallel-axis contribution.

Run:  python3 compute_moi.py
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

SDF = os.path.join(os.path.dirname(__file__),
                   '..', '..', '..', 'models', 'spacehopper', 'model.sdf')

# Joint angles as commanded by the deployed nodes.
#   retracted : flight neutral, legs stowed        (hopper_locomotion.py, FLIGHT retract)
#   splayed   : crouch / leg-tuck stance           (CROUCH_HIP, CROUCH_KNEE)
#   extended  : launch release pose                (EXTEND_HIP, EXTEND_KNEE)
POSTURES = {
    'retracted (flight neutral)': dict(hip=0.00, knee=0.00),
    'splayed (crouch stance)':    dict(hip=0.33, knee=-2.60),
    'extended (launch release)':  dict(hip=-0.42, knee=-1.10),
}


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def parse_pose(text):
    v = [float(x) for x in text.split()]
    return np.array(v[:3]), rpy(*v[3:6])


def axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def load_links():
    root = ET.parse(SDF).getroot()
    links = {}
    for link in root.iter('link'):
        pose_el = link.find('pose')
        rel = pose_el.get('relative_to') if pose_el is not None else None
        t, R = parse_pose(pose_el.text) if pose_el is not None else (np.zeros(3), np.eye(3))
        inertial = link.find('inertial')
        mass = float(inertial.find('mass').text)
        i = inertial.find('inertia')
        g = lambda k: float(i.find(k).text)
        I = np.array([[g('ixx'), g('ixy'), g('ixz')],
                      [g('ixy'), g('iyy'), g('iyz')],
                      [g('ixz'), g('iyz'), g('izz')]])
        links[link.get('name')] = dict(t=t, R=R, rel=rel, m=mass, I=I)
    return links


def world_frames(links, hip, knee):
    """Resolve every link's model-frame pose, applying the leg joint angles."""
    # joint rotation applied in the child link's own frame, axis 0 1 0
    Rj = {}
    for leg in range(3):
        Rj[f'thigh_{leg}'] = axis_angle([0, 1, 0], hip)
        Rj[f'calf_{leg}'] = axis_angle([0, 1, 0], knee)

    out = {}

    def resolve(name):
        if name in out:
            return out[name]
        L = links[name]
        if L['rel'] is None:
            T, R = L['t'].copy(), L['R'].copy()
        else:
            pt, pR = resolve(L['rel'])
            T = pt + pR @ L['t']
            R = pR @ L['R']
        R = R @ Rj.get(name, np.eye(3))
        out[name] = (T, R)
        return out[name]

    for n in links:
        resolve(n)
    return out


def izz(links, frames):
    m_tot = sum(L['m'] for L in links.values())
    com = sum(L['m'] * frames[n][0] for n, L in links.items()) / m_tot
    total = 0.0
    rows = []
    for n, L in links.items():
        T, R = frames[n]
        I_model = R @ L['I'] @ R.T
        d = T - com
        par = L['m'] * (d[0] ** 2 + d[1] ** 2)
        rows.append((n, L['m'], I_model[2, 2], par, I_model[2, 2] + par))
        total += I_model[2, 2] + par
    return m_tot, com, total, rows


def main():
    links = load_links()
    print(f'model: {os.path.normpath(SDF)}')
    results = {}
    for label, q in POSTURES.items():
        frames = world_frames(links, q['hip'], q['knee'])
        m_tot, com, I, rows = izz(links, frames)
        results[label] = I
        print(f'\n=== {label}  (hip={q["hip"]:+.2f} rad, knee={q["knee"]:+.2f} rad) ===')
        print(f'total mass            {m_tot:.3f} kg')
        print(f'body CoM (model frame) [{com[0]:+.4f} {com[1]:+.4f} {com[2]:+.4f}] m')
        print(f'{"link":<14}{"m (kg)":>9}{"I_zz,own":>12}{"parallel":>12}{"total":>12}')
        for n, m, own, par, tot in sorted(rows, key=lambda r: -r[4]):
            print(f'{n:<14}{m:>9.3f}{own:>12.3e}{par:>12.3e}{tot:>12.3e}')
        print(f'{"I_zz TOTAL":<14}{"":>9}{"":>12}{"":>12}{I:>12.4e} kg.m^2')

    lo, hi = min(results.values()), max(results.values())
    print(f'\nposture-dependent range: {lo:.4f} - {hi:.4f} kg.m^2')
    print(f'paper (Sec. 3.2) states: 0.012 - 0.020 kg.m^2')


if __name__ == '__main__':
    main()