#!/usr/bin/env python3
"""Hand-positioned vertical redraw of the system architecture diagram
(Figure 3) -- mermaid's automatic dagre layout kept resolving the
Coordination/Per-agent/Simulation tiers side-by-side because of the
feedback edges (bridge -> swarm manager/dashboard), no matter how the
graph was restructured. Manual layout guarantees a genuinely vertical
result."""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

fig, ax = plt.subplots(figsize=(6.8, 7.9))
ax.set_xlim(0, 10)
ax.set_ylim(3.9, 15)
ax.axis('off')

BOX = dict(boxstyle='round,pad=0.35', facecolor='#e8e4f8', edgecolor='#4a4a6a', linewidth=1.3)
GROUP = dict(boxstyle='round,pad=0.5', facecolor='#fdfbd4', edgecolor='#9a9440', linewidth=1.1)

def box(x, y, w, h, text, **kw):
    style = dict(BOX)
    style.update(kw)
    p = FancyBboxPatch((x - w / 2, y - h / 2), w, h, **style, zorder=3)
    ax.add_patch(p)
    ax.text(x, y, text, ha='center', va='center', fontsize=8.6, zorder=4, linespacing=1.35)
    return (x, y, w, h)

def arrow(p1, p2, label=None, color='#333333', curve=0.0, lx=None, ly=None, fs=7.6, style='-|>'):
    a = FancyArrowPatch(p1, p2, arrowstyle=style, mutation_scale=11,
                         connectionstyle=f'arc3,rad={curve}', color=color,
                         linewidth=1.1, zorder=2)
    ax.add_patch(a)
    if label:
        mx = lx if lx is not None else (p1[0] + p2[0]) / 2
        my = ly if ly is not None else (p1[1] + p2[1]) / 2
        ax.text(mx, my, label, ha='center', va='center', fontsize=fs,
                zorder=5, bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                                     edgecolor='none', alpha=0.9))

# --- Tier 1: Coordination ---
sm = box(5.0, 13.7, 3.6, 1.0, "swarm manager\nauction / roles / tasking")
gui = box(8.7, 13.7, 2.1, 1.0, "telemetry\ndashboard")

# --- Tier 2: Per-agent controllers (group) ---
grp2 = FancyBboxPatch((0.6, 8.7), 8.8, 3.3, **GROUP, zorder=1)
ax.add_patch(grp2)
ax.text(1.0, 11.65, "Per-agent controllers", fontsize=8.2, style='italic', color='#6a6420')
hop = box(2.15, 10.55, 2.7, 1.15, "hopper locomotion\ncrouch–launch\nstate machine")
land = box(7.85, 10.55, 2.7, 1.15, "landing controller\ntouchdown /\nrighting")
att = box(5.0, 9.05, 2.6, 1.0, "attitude controller\ntorque-based\nRW control")

# --- Tier 3: Simulation (group) ---
grp3 = FancyBboxPatch((0.6, 4.6), 8.8, 2.6, **GROUP, zorder=1)
ax.add_patch(grp3)
ax.text(1.0, 6.95, "Simulation", fontsize=8.2, style='italic', color='#6a6420')
br = box(3.3, 5.7, 3.4, 1.1, "ROS–Gazebo bridge\nper-agent, YAML-configured")
gz = box(7.7, 5.7, 2.6, 1.1, "Gazebo Harmonic\nDART physics")

# --- downward command edges ---
arrow((sm[0] - 0.6, sm[1] - 0.5), (hop[0] + 0.3, hop[1] + 0.7), "jump target,\nheading",
      curve=-0.15, lx=3.55, ly=12.35)
arrow((sm[0] - 0.2, sm[1] - 0.5), (att[0] - 0.4, att[1] + 1.55), curve=-0.25)

arrow((hop[0] + 1.5, hop[1] + 0.15), (land[0] - 1.5, land[1] + 0.15), "jump initiated",
      curve=-0.25, ly=10.45)
arrow((land[0] - 1.5, land[1] - 0.15), (hop[0] + 1.5, hop[1] - 0.15), "landed,\nrighting active",
      curve=-0.25, ly=9.72)
arrow((hop[0] + 0.2, hop[1] - 0.6), (att[0] - 0.9, att[1] + 0.45), curve=0.1)
arrow((land[0] - 0.2, land[1] - 0.6), (att[0] + 0.9, att[1] + 0.45), curve=-0.1)

arrow((hop[0] - 0.3, hop[1] - 0.6), (br[0] - 0.6, br[1] + 1.9), "leg position cmds",
      curve=0.12, lx=1.5, ly=8.3)
arrow((att[0], att[1] - 0.5), (br[0] + 0.4, br[1] + 1.65), "wheel velocity cmds",
      curve=0.05, lx=4.3, ly=7.7)
arrow((land[0] + 0.3, land[1] - 0.6), (br[0] + 1.3, br[1] + 1.9), "wheel cmds (righting)",
      curve=-0.15, lx=6.9, ly=8.3)

arrow((br[0] + 1.75, br[1]), (gz[0] - 1.35, gz[1]), style='<|-|>')

# --- feedback path: bridge -> up to SM, GUI, ATT, LAND (onboard sensors, odometry) ---
arrow((br[0] - 1.5, br[1] + 0.4), (0.4, 9.0), curve=0.0, color='#7a3b8a')
arrow((0.4, 9.0), (0.4, 13.4), curve=0.0, color='#7a3b8a')
arrow((0.4, 13.4), (sm[0] - 1.9, sm[1] - 0.15), "onboard sensors, odometry\n(to swarm manager, dashboard,\nattitude & landing controllers)",
      curve=0.0, color='#7a3b8a', lx=0.35, ly=11.3, fs=7.2)
arrow((0.4, 13.4), (gui[0] - 1.1, gui[1] - 0.15), curve=-0.15, color='#7a3b8a')
arrow((0.4, 10.1), (hop[0] - 1.5, hop[1]), curve=0.0, color='#7a3b8a')

plt.tight_layout()
out = "/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/word_build/diagrams/diagram1_vertical.png"
plt.savefig(out, dpi=220, bbox_inches='tight', facecolor='white')
print("wrote", out)
