#!/usr/bin/env python3
"""Schematic explaining directional hopping: the forward-lean crouch
tilts the launch thrust vector off vertical, splitting delta-v into a
vertical (height) component and a horizontal (range) component."""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, Arc

fig, ax = plt.subplots(figsize=(7.2, 5.6))
ax.set_xlim(-1, 9)
ax.set_ylim(-1.3, 6.5)
ax.set_aspect('equal')
ax.axis('off')

GROUND_Y = 0
ax.plot([-1, 9], [GROUND_Y, GROUND_Y], color='#555', linewidth=1.3)
ax.text(-0.8, -0.35, 'regolith surface', fontsize=8, color='#555')

# --- symmetric (non-leaned) reference stance, faint, on the left ---
bx0, by0 = 1.4, 1.0
body = plt.Rectangle((bx0-0.32, by0-0.28), 0.64, 0.56, facecolor='#cfcfcf',
                      edgecolor='#888', linewidth=1.0, zorder=2, alpha=0.6)
ax.add_patch(body)
ax.annotate('', xy=(bx0, by0+1.55), xytext=(bx0, by0+0.05),
            arrowprops=dict(arrowstyle='-|>', lw=1.6, color='#999'))
ax.text(bx0+0.18, by0+1.55, 'purely vertical stroke:\nno ground range',
        fontsize=8, color='#777', va='top')
ax.text(bx0, by0-0.55, 'symmetric crouch\n(all legs equal)', fontsize=7.6,
        ha='center', color='#777')

# --- leaned (directional) stance, on the right, the actual deployed design ---
bx, by = 5.6, 1.15
LEAN_DEG = 17  # ~SIN2TH=0.56 -> elevation ~73deg off horizontal -> ~17deg thrust tilt off vertical
th = np.radians(LEAN_DEG)

# tilted body box
body2 = mpatches.FancyBboxPatch((-0.34, -0.30), 0.68, 0.60, boxstyle='round,pad=0.02',
                                 facecolor='#dfe3f5', edgecolor='#3a3a5a', linewidth=1.4)
tform = plt.matplotlib.transforms.Affine2D().rotate_deg(-LEAN_DEG).translate(bx, by) + ax.transData
body2.set_transform(tform)
ax.add_patch(body2)
ax.text(bx, by, 'BODY', fontsize=7.6, ha='center', va='center', fontweight='bold',
        rotation=-LEAN_DEG, zorder=5)

# thrust vector, tilted off vertical by LEAN_DEG toward +x
thrust_len = 2.7
tx = bx + thrust_len * np.sin(th)
ty = by + thrust_len * np.cos(th)
ax.annotate('', xy=(tx, ty), xytext=(bx, by),
            arrowprops=dict(arrowstyle='-|>', lw=2.2, color='#a33'))
ax.text(tx+0.15, ty-0.35, 'thrust vector\n(off vertical by\nlean angle)', fontsize=8, color='#a33')

# vertical dashed reference + arc showing the lean angle
ax.plot([bx, bx], [by, by+thrust_len], linestyle='--', color='#888', linewidth=1)
arc = Arc((bx, by), 1.4, 1.4, angle=0, theta1=90-LEAN_DEG, theta2=90, color='#333')
ax.add_patch(arc)
ax.text(bx+0.35, by+0.85, 'lean\nangle', fontsize=7.6, ha='center', color='#333')

# decompose thrust into vertical + horizontal component arrows at the tip
vx, vy = bx, by + thrust_len * np.cos(th)  # vertical component endpoint
ax.annotate('', xy=(vx, vy), xytext=(bx, by),
            arrowprops=dict(arrowstyle='-|>', lw=1.3, color='#1a56c4', linestyle=(0, (4, 2))))
ax.annotate('', xy=(tx, vy), xytext=(vx, vy),
            arrowprops=dict(arrowstyle='-|>', lw=1.3, color='#0a7d34', linestyle=(0, (4, 2))))
ax.text(bx-0.75, by+thrust_len*0.55, 'vertical\ncomponent\n(height)', fontsize=7.4,
        color='#1a56c4', ha='center')
ax.text((vx+tx)/2, vy+0.2, 'horizontal component\n(ground range)', fontsize=7.4,
        color='#0a7d34', ha='center')

ax.text(bx, by-0.95,
        'leaned crouch: leading leg flexed +LEAN,\ntrailing pair flexed -LEAN/2\n(stance height unchanged)',
        fontsize=7.6, ha='center', color='#555')

# landing/ground-track sketch below
ax.annotate('', xy=(bx+2.2, GROUND_Y), xytext=(bx, GROUND_Y),
            arrowprops=dict(arrowstyle='-|>', lw=1.2, color='#888',
                             connectionstyle='arc3,rad=-0.35'))
ax.text(bx+1.1, -0.75, 'ground displacement\n(measured: 4.3 m at 1° heading error)',
        fontsize=7.6, ha='center', color='#555')

ax.text(4.0, 6.0,
        'Directional hopping: tilting the launch thrust vector via an asymmetric\n'
        '("leaned") crouch trades some vertical delta-v for horizontal range,\n'
        'while yaw-hold keeps the tilt aimed at the commanded heading.',
        fontsize=9, ha='center', style='italic')

plt.tight_layout()
out = "/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/word_build/diagrams/directional_hop_sketch.png"
plt.savefig(out, dpi=210, bbox_inches='tight', facecolor='white')
print("wrote", out)
