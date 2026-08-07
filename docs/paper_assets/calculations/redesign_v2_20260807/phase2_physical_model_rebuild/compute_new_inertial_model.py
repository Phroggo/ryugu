#!/usr/bin/env python3
"""Phase 2: rebuild base_link's mass, CoM offset, and full inertia tensor by
summing the Phase 1-audited components at real physical positions, via the
parallel-axis theorem -- the same method already used for the platform-level
I_bot in compute_moi.py, generalized from a single (zz) axis to the full 3x3
tensor plus a CoM offset.

Method
------
1. Every base_link-lumped component (i.e. everything Phase 1 found had no
   real link of its own: chassis structure, RW motors, leg motor+gearhead
   assemblies, avionics, power, antenna, cameras, MLI) is given a mass (from
   AUDIT_TABLE.md) and a physical position relative to base_link's own
   origin frame -- the same frame convention compute_moi.py already uses
   (link poses read relative to the body frame, matching how rw_x/thigh_0/
   etc. are actually placed in model.sdf today).
2. Each component gets its own-frame inertia tensor from real geometry
   where that's meaningful (the chassis shell is built from its own 6
   panels + 12 edges, each a thin plate/rod with a real parallel-axis
   contribution to the chassis's own center); small compact items (motors,
   BMS, cameras, antenna) are treated as point masses (own-inertia ~0) --
   flagged explicitly, valid because their own dimensions are small
   compared to their distance from the eventual combined CoM.
3. Overall CoM = mass-weighted mean position of every component.
4. Every component's own tensor, plus m*(|d|^2*I_3 - d(x)d) for its offset
   d from the *overall CoM* (not from base_link's origin), summed to get
   the full combined tensor about the overall CoM.
5. Reported both about the overall CoM (what goes in model.sdf's
   <inertial><pose> + <inertia>) and, separately, translated back to
   base_link's origin frame for direct comparison against the old lumped
   figures.

Every position not taken directly from an existing model.sdf/generator
visual pose is an explicit ASSUMPTION, flagged in the position table below.
"""
import numpy as np

# ---------------------------------------------------------------------------
# Material/geometry constants (Phase 1, AUDIT_TABLE.md)
AL_7075_DENSITY = 2810.0
CFRP_DENSITY = 1600.0
STEEL_DENSITY = 8000.0

def box_tensor(m, sx, sy, sz):
    """Solid-box inertia about its own center, axis-aligned."""
    return np.diag([m/12*(sy**2+sz**2), m/12*(sx**2+sz**2), m/12*(sx**2+sy**2)])

def rod_tensor(m, length, axis):
    """Thin rod (negligible cross-section) of given length along `axis`
    ('x','y','z'), about its own center."""
    i_perp = m * length**2 / 12
    diag = {'x': [0, i_perp, i_perp], 'y': [i_perp, 0, i_perp], 'z': [i_perp, i_perp, 0]}[axis]
    return np.diag(diag)

def point_tensor():
    return np.zeros((3, 3))

def parallel_axis(I_own, m, d):
    """General parallel-axis theorem: shift I_own (about the piece's own
    CoM) to a common reference point offset by vector d = own_CoM - ref."""
    d = np.array(d, dtype=float)
    return I_own + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

# ---------------------------------------------------------------------------
# 1. Chassis structure, built from its own sub-pieces (CFRP skin panels +
#    7075-T6 edge frame), matching AUDIT_TABLE.md row 5's geometry exactly.
chassis_side = 0.2
panel_thickness = 0.001
frame_edge = 0.004
panel_area = chassis_side**2
panel_mass = panel_area * panel_thickness * CFRP_DENSITY   # per face, 6 faces
frame_edge_mass = chassis_side * frame_edge**2 * AL_7075_DENSITY  # per edge, 12 edges

chassis_pieces = []  # list of (mass, position (relative to base_link origin), own_tensor)
h = chassis_side / 2
# 6 panels, each a thin plate at a cube face, axis-aligned box_tensor with
# thickness in the face-normal direction.
face_specs = [
    ('x+', (h, 0, 0), (panel_thickness, chassis_side, chassis_side)),
    ('x-', (-h, 0, 0), (panel_thickness, chassis_side, chassis_side)),
    ('y+', (0, h, 0), (chassis_side, panel_thickness, chassis_side)),
    ('y-', (0, -h, 0), (chassis_side, panel_thickness, chassis_side)),
    ('z+', (0, 0, h), (chassis_side, chassis_side, panel_thickness)),
    ('z-', (0, 0, -h), (chassis_side, chassis_side, panel_thickness)),
]
for name, pos, dims in face_specs:
    chassis_pieces.append((panel_mass, pos, box_tensor(panel_mass, *dims)))

# 12 edges, each a thin rod of length `chassis_side` along one axis,
# positioned at the midpoint of that cube edge.
edge_specs = []
for sy in (-h, h):
    for sz in (-h, h):
        edge_specs.append(((0, sy, sz), 'x'))
for sx in (-h, h):
    for sz in (-h, h):
        edge_specs.append(((sx, 0, sz), 'y'))
for sx in (-h, h):
    for sy in (-h, h):
        edge_specs.append(((sx, sy, 0), 'z'))
for pos, axis in edge_specs:
    chassis_pieces.append((frame_edge_mass, pos, rod_tensor(frame_edge_mass, chassis_side, axis)))

chassis_mass = sum(m for m, _, _ in chassis_pieces)
chassis_com = np.sum([np.array(p) * m for m, p, _ in chassis_pieces], axis=0) / chassis_mass
chassis_I_about_own_com = sum(
    parallel_axis(I, m, np.array(p) - chassis_com) for m, p, I in chassis_pieces
)
print(f"[chassis sub-assembly] mass={chassis_mass:.4f} kg, "
      f"own CoM={chassis_com.round(6).tolist()} (should be ~0,0,0 by symmetry)")

# ---------------------------------------------------------------------------
# 2. Every base_link-lumped component: (name, mass, position, own-inertia).
# Positions are relative to base_link's own origin frame -- the same
# convention model.sdf/compute_moi.py already use for rw_x/thigh_0/etc.
#
# POSITION SOURCING (flagged per component):
#   chassis, RW motors x3, MLI  -> centered at origin (0,0,0). Chassis by
#     construction (computed above); RW motors ASSUMED coincident with
#     their flywheels, matching how the flywheels themselves are already
#     modeled (rw_x/y/z sit at (0,0,0) relative to base_link -- confirmed
#     by reading model.sdf); MLI ASSUMED symmetric shell wrap, same
#     rationale.
#   leg motor+gearhead x6 -> at each thigh's own link origin (real model.sdf
#     pose, not assumed): thigh_0=(0.07,0,-0.10), thigh_1=(-0.035,0.0606,
#     -0.10), thigh_2=(-0.035,-0.0606,-0.10). Knee motor ASSUMED co-located
#     with its leg's hip motor (proximal/chassis-mounted actuation with a
#     remote drive to the knee, a common design choice for compact hoppers
#     to keep leg-swing inertia low -- not a claim this platform has that
#     linkage, an explicit simplification).
#   battery+BMS -> (0,0,0.05), the centroid of generate_detailed_spacehopper
#     .py's own battery_visuals() 3x4 cell grid (real existing visual
#     layout, reused for consistency rather than inventing a new position).
#   avionics (computer+sensing+comms) -> (0,0,0), ASSUMED centered; no
#     existing visual hook for this one.
#   antenna -> (0.06,0.06,0.11), the whip-antenna mast position from
#     antenna_mast() (real existing visual layout).
#   cameras x3 -> navcam (0.105,0,0.02), hazcam_L (0.100,0.035,0.02),
#     hazcam_R (0.100,-0.035,0.02), all real existing visual positions from
#     camera_housing(). NOTE: the visual model also has a 4th ("sci_camera")
#     not counted in Phase 1's 3-camera audit -- flagged in the change
#     report, not added here (staying faithful to what was actually costed).
components = [
    ("chassis", chassis_mass, (0, 0, 0), chassis_I_about_own_com),
    ("RW motor x3", 3 * 0.022, (0, 0, 0), point_tensor()),
    ("leg motor+gearhead (leg0: hip+knee)", 2 * 0.040, (0.07, 0.0, -0.10), point_tensor()),
    ("leg motor+gearhead (leg1: hip+knee)", 2 * 0.040, (-0.035, 0.0606, -0.10), point_tensor()),
    ("leg motor+gearhead (leg2: hip+knee)", 2 * 0.040, (-0.035, -0.0606, -0.10), point_tensor()),
    ("battery+BMS", 4 * 0.0475 + 0.020, (0, 0, 0.05), point_tensor()),
    ("avionics (computer+sensing+comms)", 0.094 + 0.030 + 0.094, (0, 0, 0), point_tensor()),
    ("antenna", 0.015, (0.06, 0.06, 0.11), point_tensor()),
    ("cameras x3", 3 * 0.010, (0.102, 0.0, 0.02), point_tensor()),  # mean of the 3 real positions
    ("MLI", 0.098, (0, 0, 0), point_tensor()),
]

total_mass = sum(m for _, m, _, _ in components)
com = sum(np.array(p) * m for _, m, p, _ in components) / total_mass

I_about_com = np.zeros((3, 3))
for name, m, p, I_own in components:
    d = np.array(p) - com
    I_about_com += parallel_axis(I_own, m, d)

print()
print("=== New base_link (lumped, computed from Phase 1 components) ===")
print(f"Total mass: {total_mass:.4f} kg")
print(f"CoM relative to base_link origin: "
      f"x={com[0]:.5f}  y={com[1]:.5f}  z={com[2]:.5f}  (m)")
print("Full inertia tensor about the CoM (kg*m^2):")
for row in I_about_com:
    print("  [" + "  ".join(f"{v: .6e}" for v in row) + "]")

# Also express the same tensor about base_link's ORIGIN (not the CoM) for
# direct comparison against the old lumped inertia, which was given about
# the link origin (implicitly assumed coincident with CoM at (0,0,0)).
I_about_origin = parallel_axis(I_about_com, total_mass, com)
print()
print("Same tensor translated to base_link's ORIGIN (for old-vs-new diagonal comparison):")
for row in I_about_origin:
    print("  [" + "  ".join(f"{v: .6e}" for v in row) + "]")

print()
print("=== OLD lumped base_link (for comparison) ===")
print("mass=1.350 kg, CoM=(0,0,0) [assumed], I=diag(9.000e-03, 9.000e-03, 9.000e-03)")
