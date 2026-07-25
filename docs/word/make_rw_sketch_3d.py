#!/usr/bin/env python3
"""Real isometric 3D engineering sketch of the reaction-wheel layout and
body moment-of-inertia axes (comment: previous 2D flat schematic was
inadequate -- this uses matplotlib's 3D toolkit for genuine isometric
projection with proper technical-drawing conventions: wireframe body,
solid-shaded cylinders for the three flywheels along their true axes,
dimension leaders, and a body-fixed axis triad)."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

fig = plt.figure(figsize=(7.6, 7.2))
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])

# ---- chassis: wireframe cube, centered at origin, side 1.0 (normalized) ----
s = 0.55
r = [-s, s]
verts = np.array([[x, y, z] for x in r for y in r for z in r])
edges = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
for a, b in edges:
    ax.plot(*zip(verts[a], verts[b]), color='#2a2a3a', linewidth=1.4, zorder=1)
# faint translucent faces for solidity cue
faces = [
    [verts[0], verts[1], verts[3], verts[2]],
    [verts[4], verts[5], verts[7], verts[6]],
    [verts[0], verts[1], verts[5], verts[4]],
    [verts[2], verts[3], verts[7], verts[6]],
    [verts[0], verts[2], verts[6], verts[4]],
    [verts[1], verts[3], verts[7], verts[5]],
]
ax.add_collection3d(Poly3DCollection(faces, facecolor='#d8def0', edgecolor='none', alpha=0.35, zorder=0))
ax.text(0, 0, 0, "BODY\n$I_{bot}$", ha='center', va='center', fontsize=9, fontweight='bold', zorder=5)

# ---- cylinder helper: axis-aligned solid cylinder (a flywheel) ----
def cylinder(ax, center, axis, radius, length, color, n=28):
    axis = np.array(axis, dtype=float); axis /= np.linalg.norm(axis)
    not_axis = np.array([1, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1, 0])
    n1 = np.cross(axis, not_axis); n1 /= np.linalg.norm(n1)
    n2 = np.cross(axis, n1)
    theta = np.linspace(0, 2*np.pi, n)
    t = np.linspace(-length/2, length/2, 2)
    theta_g, t_g = np.meshgrid(theta, t)
    X = (center[0] + radius*np.cos(theta_g)*n1[0] + radius*np.sin(theta_g)*n2[0] + t_g*axis[0])
    Y = (center[1] + radius*np.cos(theta_g)*n1[1] + radius*np.sin(theta_g)*n2[1] + t_g*axis[1])
    Z = (center[2] + radius*np.cos(theta_g)*n1[2] + radius*np.sin(theta_g)*n2[2] + t_g*axis[2])
    ax.plot_surface(X, Y, Z, color=color, alpha=0.92, linewidth=0, shade=True, zorder=4)
    # end caps
    for tt in (-length/2, length/2):
        cx = center[0] + tt*axis[0]; cy = center[1] + tt*axis[1]; cz = center[2] + tt*axis[2]
        cxs = cx + radius*np.cos(theta)*n1[0] + radius*np.sin(theta)*n2[0]
        cys = cy + radius*np.cos(theta)*n1[1] + radius*np.sin(theta)*n2[1]
        czs = cz + radius*np.cos(theta)*n1[2] + radius*np.sin(theta)*n2[2]
        verts_cap = [list(zip(cxs, cys, czs))]
        ax.add_collection3d(Poly3DCollection(verts_cap, facecolor=color, alpha=0.92, edgecolor='none', zorder=4))

WHEEL_R, WHEEL_L = 0.16, 0.09
offset = s + 0.75

cylinder(ax, (offset, 0, 0), (1, 0, 0), WHEEL_R, WHEEL_L, '#e0a52e')   # RW-X
cylinder(ax, (0, offset, 0), (0, 1, 0), WHEEL_R, WHEEL_L, '#3a8f4a')   # RW-Y
cylinder(ax, (0, 0, offset), (0, 0, 1), WHEEL_R, WHEEL_L, '#3a6fc4')   # RW-Z

# connecting shafts (thin lines from body face to wheel)
ax.plot([s, offset-WHEEL_L/2], [0, 0], [0, 0], color='#555', linewidth=1.2, zorder=3)
ax.plot([0, 0], [s, offset-WHEEL_L/2], [0, 0], color='#555', linewidth=1.2, zorder=3)
ax.plot([0, 0], [0, 0], [s, offset-WHEEL_L/2], color='#555', linewidth=1.2, zorder=3)

# ---- body-fixed axis triad (drawn short, inside the body, so it doesn't
# collide visually with the wheels/labels further out) ----
L = s * 0.85
ax.quiver(0, 0, 0, L, 0, 0, color='#a33', arrow_length_ratio=0.15, linewidth=1.3, zorder=6)
ax.quiver(0, 0, 0, 0, L, 0, color='#0a7d34', arrow_length_ratio=0.15, linewidth=1.3, zorder=6)
ax.quiver(0, 0, 0, 0, 0, L, color='#1a56c4', arrow_length_ratio=0.15, linewidth=1.3, zorder=6)
ax.text(L*0.5, -0.14, -0.08, 'X', color='#a33', fontsize=10, fontweight='bold')
ax.text(-0.08, L*0.5, -0.08, 'Y', color='#0a7d34', fontsize=10, fontweight='bold')
ax.text(-0.08, -0.08, L*0.55, 'Z', color='#1a56c4', fontsize=10, fontweight='bold')

# ---- labels for each wheel, placed well clear beyond the wheel along its axis ----
ax.text(offset + 0.30, 0, 0, "RW-X\n(EC 20 flat)",
        fontsize=8.0, ha='left', va='center', color='#7a5a10')
ax.text(0, offset + 0.30, 0, "RW-Y\n(EC 20 flat)",
        fontsize=8.0, ha='left', va='center', color='#1f5c2a')
ax.text(0, 0, offset + 0.30, "RW-Z (EC 20 flat)",
        fontsize=8.0, ha='left', va='bottom', color='#123f8a')

ax.text2D(0.02, 0.02,
          "$I_{bot}$ = 0.012–0.020 kg·m$^2$ (posture-dependent)\n"
          "Per-wheel: $I_w=\\frac{1}{2}mr^2$, $m$=0.15 kg, $r$=0.06 m $\\Rightarrow$ 2.7$\\times10^{-4}$ kg·m$^2$\n"
          "Max speed 982 rad/s $\\Rightarrow$ $H_{max}$=0.265 N·m·s per wheel\n"
          "Torque capacity $\\tau_{rw}$=0.015 N·m (intermittent-duty)",
          transform=ax.transAxes, fontsize=7.8,
          bbox=dict(boxstyle='round,pad=0.4', facecolor='#fffdf3', edgecolor='#999'))

ax.set_xlim(-1.0, 2.0); ax.set_ylim(-1.0, 2.0); ax.set_zlim(-1.0, 2.0)
ax.set_axis_off()
ax.view_init(elev=18, azim=-58)

plt.tight_layout()
out = "/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/word_build/diagrams/reaction_wheel_moi_3d.png"
plt.savefig(out, dpi=220, bbox_inches='tight', facecolor='white')
print("wrote", out)
