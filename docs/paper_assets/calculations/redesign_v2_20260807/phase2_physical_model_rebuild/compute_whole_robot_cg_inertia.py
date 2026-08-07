#!/usr/bin/env python3
"""Phase 2 checkpoint: whole-robot total mass, full (x,y,z) CG, and full 3x3
inertia tensor, for BOTH the old (pre-Phase-2) and new model.sdf, computed
by the identical method for a direct comparison.

Extends ../../compute_moi.py (which only computed I_zz about the body
z-axis) to: (a) the full symmetric 3x3 tensor, not just izz, and (b) proper
handling of a link's <inertial><pose> CoM offset, which the original script
never needed (no link had one) but base_link now does post-Phase-2.

Run against the OLD reference copy (model_OLD_pre_phase2_reference.sdf,
committed alongside this script) and the NEW deployed model.sdf, same 3
postures compute_moi.py already uses, for apples-to-apples comparison.
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

REPO_ROOT = "/home/melvin/ryugu_v2_ws/src/ryugu_sim"
NEW_SDF = os.path.join(REPO_ROOT, "models/spacehopper/model.sdf")
OLD_SDF = os.path.join(os.path.dirname(__file__), "model_OLD_pre_phase2_reference.sdf")
# Added for the battery/antenna correction pass (2026-08-07): the first
# Phase 2 rebuild (pre-correction), for a 3-way comparison.
V1_SDF = os.path.join(os.path.dirname(__file__), "model_PHASE2_v1_reference.sdf")

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


def load_links(sdf_path):
    root = ET.parse(sdf_path).getroot()
    links = {}
    for link in root.iter('link'):
        pose_el = link.find('pose')
        rel = pose_el.get('relative_to') if pose_el is not None else None
        t, R = parse_pose(pose_el.text) if pose_el is not None else (np.zeros(3), np.eye(3))
        inertial = link.find('inertial')
        mass = float(inertial.find('mass').text)
        # NEW: inertial CoM offset from the link frame (base_link has one
        # post-Phase-2; every other link defaults to zero, same as before).
        inertial_pose_el = inertial.find('pose')
        if inertial_pose_el is not None:
            com_offset, com_R = parse_pose(inertial_pose_el.text)
        else:
            com_offset, com_R = np.zeros(3), np.eye(3)
        i = inertial.find('inertia')
        g = lambda k: float(i.find(k).text)
        I = np.array([[g('ixx'), g('ixy'), g('ixz')],
                      [g('ixy'), g('iyy'), g('iyz')],
                      [g('ixz'), g('iyz'), g('izz')]])
        links[link.get('name')] = dict(t=t, R=R, rel=rel, m=mass, I=I,
                                        com_offset=com_offset, com_R=com_R)
    return links


def world_frames(links, hip, knee):
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


def full_tensor(links, frames):
    """Total mass, full-body CoM (x,y,z), and the full 3x3 inertia tensor
    about that CoM, in the model/base_link frame."""
    m_tot = sum(L['m'] for L in links.values())

    # True per-link CoM location = link origin + link_R @ inertial_offset
    # (inertial_offset is expressed in the link's own frame).
    link_com = {}
    for n, L in links.items():
        T, R = frames[n]
        link_com[n] = T + R @ L['com_offset']

    com = sum(L['m'] * link_com[n] for n, L in links.items()) / m_tot

    I_total = np.zeros((3, 3))
    for n, L in links.items():
        T, R = frames[n]
        # Inertia tensor is expressed about the link's CoM in the inertial
        # frame's axes; rotate link_R (inertial com_R is identity for every
        # link in this model, so R_full = R @ com_R = R).
        I_model = (R @ L['com_R']) @ L['I'] @ (R @ L['com_R']).T
        d = link_com[n] - com
        I_total += I_model + L['m'] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m_tot, com, I_total


def main():
    for label_file, path in [("OLD (pre-Phase-2, original)", OLD_SDF),
                              ("PHASE2-v1 (first rebuild, battery/antenna bugs)", V1_SDF),
                              ("PHASE2-CORRECTED (battery+S-Band-antenna fix)", NEW_SDF)]:
        print(f"\n############ {label_file}: {path} ############")
        links = load_links(path)
        # base_link is the root link (no relative_to) -- its own <pose> IS
        # its absolute model-frame position, and every other link's pose in
        # this file is likewise given in that same absolute/model frame
        # (none use relative_to=base_link explicitly). "CG relative to
        # chassis base," per the phase instructions, means relative to
        # base_link's own origin -- so subtract base_link's absolute
        # position from every computed absolute CoM below.
        base_link_origin = links['base_link']['t'].copy()
        for label, q in POSTURES.items():
            frames = world_frames(links, q['hip'], q['knee'])
            m_tot, com_absolute, I = full_tensor(links, frames)
            com = com_absolute - base_link_origin
            print(f"\n=== {label} (hip={q['hip']:+.2f}, knee={q['knee']:+.2f}) ===")
            print(f"total mass: {m_tot:.4f} kg")
            print(f"CG (x,y,z) relative to base_link origin: "
                  f"({com[0]:+.5f}, {com[1]:+.5f}, {com[2]:+.5f}) m")
            print("full inertia tensor about CG (kg*m^2):")
            for row in I:
                print("  [" + "  ".join(f"{v: .6e}" for v in row) + "]")
            print(f"I_zz (for direct comparison to compute_moi.py's existing "
                  f"figure): {I[2,2]:.6e}")


if __name__ == '__main__':
    main()
