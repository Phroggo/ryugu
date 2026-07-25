#!/usr/bin/env python3
"""I_bot (robot moment of inertia about body z-axis) worked calculation,
via the parallel-axis theorem, from the actual per-link mass/inertia/pose
data in the deployed model.sdf. Properly resolves SDF's relative_to pose
chaining (calf links are posed relative to their parent thigh, not the
body frame -- caught and fixed after an initial pass got this wrong)."""
import re
import numpy as np

SDF = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models/spacehopper/model.sdf'
content = open(SDF).read()

def euler_to_R(r, p, y):
    Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx

def to_T(pose_str):
    x, y, z, r, p, yaw = [float(v) for v in pose_str.split()]
    T = np.eye(4)
    T[:3, :3] = euler_to_R(r, p, yaw)
    T[:3, 3] = [x, y, z]
    return T

links = {}  # name -> {mass, ixx, iyy, izz, pose_str, relative_to}
for m in re.finditer(r'<link name="([^"]*)"[^>]*>(.*?)</link>', content, re.S):
    name, body = m.group(1), m.group(2)
    pose_m = re.search(r'<pose(?:\s+relative_to="([^"]*)")?>([^<]*)</pose>', body)
    inertial_m = re.search(r'<inertial>(.*?)</inertial>', body, re.S)
    if not pose_m or not inertial_m:
        continue
    inertial = inertial_m.group(1)
    links[name] = dict(
        mass=float(re.search(r'<mass>([^<]*)</mass>', inertial).group(1)),
        ixx=float(re.search(r'<ixx>([^<]*)</ixx>', inertial).group(1)),
        iyy=float(re.search(r'<iyy>([^<]*)</iyy>', inertial).group(1)),
        izz=float(re.search(r'<izz>([^<]*)</izz>', inertial).group(1)),
        pose_str=pose_m.group(2),
        relative_to=pose_m.group(1),
    )

_resolved = {}
def resolve_T(name):
    """World(model)-frame transform for link `name`, resolving relative_to chains."""
    if name in _resolved:
        return _resolved[name]
    L = links[name]
    T_local = to_T(L['pose_str'])
    if L['relative_to']:
        T = resolve_T(L['relative_to']) @ T_local
    else:
        T = T_local
    _resolved[name] = T
    return T

rows = []
total_izz_about_origin = 0.0
total_mass = 0.0
for name, L in links.items():
    T = resolve_T(name)
    R, pos = T[:3, :3], T[:3, 3]
    I_local = np.diag([L['ixx'], L['iyy'], L['izz']])
    I_body_frame = R @ I_local @ R.T
    izz_rotated = I_body_frame[2, 2]
    d_perp2 = pos[0]**2 + pos[1]**2
    izz_about_origin = izz_rotated + L['mass'] * d_perp2
    rows.append((name, L['mass'], pos[0], pos[1], pos[2], izz_rotated, d_perp2, izz_about_origin))
    total_izz_about_origin += izz_about_origin
    total_mass += L['mass']

print(f"{'link':<12} {'mass(kg)':>9} {'x':>8} {'y':>8} {'Izz_own(rot)':>14} {'d_perp^2':>10} {'Izz_about_origin':>18}")
for r in rows:
    print(f"{r[0]:<12} {r[1]:>9.3f} {r[2]:>8.4f} {r[3]:>8.4f} {r[5]:>14.6e} {r[6]:>10.6f} {r[7]:>18.6e}")

print()
print(f"Total mass: {total_mass:.3f} kg")
print(f"Total I_bot (about body z-axis through model origin, this SDF default/rest posture): {total_izz_about_origin:.6f} kg*m^2")
