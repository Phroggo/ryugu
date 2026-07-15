#!/usr/bin/env python3
"""
Swarm Dashboard GUI -- quick-glance status monitor for up to 3 scouts.

Shows, per bot: current role + activity (published by swarm_manager.py),
battery, landed/flight state, a square artificial-horizon "gyro" indicator
(from IMU orientation), leg joint commands, drill state, and reaction wheel
speeds. A bot that has never published anything (not yet spawned, e.g.
scout_2/scout_3 in the current single-bot launch) shows as OFFLINE rather
than blank or stale data.

Runs rclpy.spin() on a background thread and Tkinter's mainloop on the main
thread, sharing state through plain AgentState objects -- safe here because
every field is a single primitive assignment (GIL-atomic), not a compound
update that needs a lock.
"""
import math
import random
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import Imu

AGENTS = ["scout_1", "scout_2", "scout_3"]
STALE_AFTER = 5.0  # seconds with zero messages of any kind -> show OFFLINE

ROLE_COLORS = {
    "SCOUT": "#2196F3",
    "SAMPLER": "#FF9800",
    "RELAY": "#B388FF",
    "RECHARGE": "#FF5252",
    "Unassigned": "#5C6B7A",
}

# Deep-space / mission-control palette -- near-black indigo with a cyan HUD
# accent and a live starfield header (restyled 2026-07-16 per user request:
# "cleaner and aesthetic, a cool starry theme").
BG = "#04060f"
PANEL_BG = "#0a101d"
PANEL_BORDER = "#234f61"
SEPARATOR = "#152233"
FG = "#d7ecf5"
DIM_FG = "#5f7d8e"
ACCENT = "#53dcec"
ACCENT_DIM = "#2a8a9a"
STAR_COLORS = ["#ffffff", "#cfe8ff", "#9fd8ff", "#ffd9a0", "#e8e8ff"]

# Ubuntu Mono is confirmed installed on this machine and gives the dashboard
# a "terminal/HUD" feel appropriate for a mission-control-style readout;
# falls back to Tk's default if unavailable elsewhere.
FONT_MONO = "Ubuntu Mono"


class AgentState:
    def __init__(self):
        self.role = "Unassigned"
        self.activity = "-"
        self.battery = None
        self.power_rate = 0.0  # %/tick, +charging / -discharging
        self.landed = None
        self.roll = 0.0
        self.pitch = 0.0
        self.drill = 0.0
        self.rw = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.legs = {f"hip_{i}": 0.0 for i in range(3)}
        self.legs.update({f"knee_{i}": 0.0 for i in range(3)})
        self.last_seen = 0.0

    def touch(self):
        self.last_seen = time.time()

    @property
    def online(self):
        return (time.time() - self.last_seen) < STALE_AFTER


class SwarmGuiNode(Node):
    def __init__(self):
        super().__init__('swarm_gui')
        self.states = {agent: AgentState() for agent in AGENTS}

        for agent in AGENTS:
            st = self.states[agent]
            self.create_subscription(
                String, f'/{agent}/status_role',
                lambda msg, s=st: self._role(s, msg), 10)
            self.create_subscription(
                String, f'/{agent}/status_activity',
                lambda msg, s=st: self._activity(s, msg), 10)
            self.create_subscription(
                Float64, f'/{agent}/status_battery',
                lambda msg, s=st: self._battery(s, msg), 10)
            self.create_subscription(
                Float64, f'/{agent}/status_power_rate',
                lambda msg, s=st: self._power_rate(s, msg), 10)
            self.create_subscription(
                Bool, f'/{agent}/landed',
                lambda msg, s=st: self._landed(s, msg), 10)
            self.create_subscription(
                Float64, f'/{agent}/cmd_drill',
                lambda msg, s=st: self._drill(s, msg), 10)
            self.create_subscription(
                Imu, f'/{agent}/imu',
                lambda msg, s=st: self._imu(s, msg), 10)
            for axis in ['x', 'y', 'z']:
                self.create_subscription(
                    Float64, f'/{agent}/rw_{axis}_joint_cmd_vel',
                    lambda msg, s=st, a=axis: self._rw(s, a, msg), 10)
            for i in range(3):
                self.create_subscription(
                    Float64, f'/{agent}/joint_hip_joint_{i}_cmd_pos',
                    lambda msg, s=st, i=i: self._leg(s, f'hip_{i}', msg), 10)
                self.create_subscription(
                    Float64, f'/{agent}/joint_knee_joint_{i}_cmd_pos',
                    lambda msg, s=st, i=i: self._leg(s, f'knee_{i}', msg), 10)

    def _role(self, s, msg):
        s.role = msg.data
        s.touch()

    def _activity(self, s, msg):
        s.activity = msg.data
        s.touch()

    def _battery(self, s, msg):
        s.battery = msg.data
        s.touch()

    def _power_rate(self, s, msg):
        s.power_rate = msg.data
        s.touch()

    def _landed(self, s, msg):
        s.landed = msg.data
        s.touch()

    def _drill(self, s, msg):
        s.drill = msg.data
        s.touch()

    def _rw(self, s, axis, msg):
        s.rw[axis] = msg.data
        s.touch()

    def _leg(self, s, key, msg):
        s.legs[key] = msg.data
        s.touch()

    def _imu(self, s, msg):
        q = msg.orientation
        # Small-angle-friendly Euler extraction -- this is a display gauge,
        # not a control loop, so it doesn't need the large-angle robustness
        # attitude_controller.py's quaternion cross-product approach has.
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        s.roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        s.pitch = math.asin(sinp)
        s.touch()


class BotPanel(tk.Frame):
    GYRO_SIZE = 74

    def __init__(self, master, agent_name):
        super().__init__(master, bg=PANEL_BG, padx=10, pady=8,
                          highlightbackground=PANEL_BORDER, highlightthickness=1)
        self.agent_name = agent_name

        top = tk.Frame(self, bg=PANEL_BG)
        top.pack(fill="x")
        tk.Label(top, text="◆", font=(FONT_MONO, 10), bg=PANEL_BG,
                 fg=ACCENT_DIM).pack(side="left", padx=(0, 4))
        self.name_label = tk.Label(top, text=agent_name.upper(),
                                    font=(FONT_MONO, 12, "bold"),
                                    bg=PANEL_BG, fg=FG)
        self.name_label.pack(side="left")
        self.role_badge = tk.Label(top, text="OFFLINE", font=(FONT_MONO, 9, "bold"),
                                    bg="#2a3a45", fg="white", padx=8, pady=2)
        self.role_badge.pack(side="right")

        self.activity_label = tk.Label(self, text="-", font=(FONT_MONO, 9),
                                        bg=PANEL_BG, fg=DIM_FG, anchor="w",
                                        wraplength=200, justify="left")
        self.activity_label.pack(fill="x", pady=(3, 5))

        tk.Frame(self, height=1, bg=SEPARATOR).pack(fill="x", pady=(0, 6))

        mid = tk.Frame(self, bg=PANEL_BG)
        mid.pack(fill="x")

        # --- Gyro (square artificial-horizon indicator) ---
        self.gyro = tk.Canvas(mid, width=self.GYRO_SIZE, height=self.GYRO_SIZE,
                               bg="#000", highlightthickness=1,
                               highlightbackground=PANEL_BORDER)
        self.gyro.pack(side="left", padx=(0, 8))

        # --- Right column: battery, landed state, drill ---
        right = tk.Frame(mid, bg=PANEL_BG)
        right.pack(side="left", fill="both", expand=True)

        tk.Label(right, text="BATTERY", font=(FONT_MONO, 8), bg=PANEL_BG,
                 fg=ACCENT_DIM, anchor="w").pack(fill="x")
        self.battery_canvas = tk.Canvas(right, width=140, height=12, bg="#111",
                                         highlightthickness=0)
        self.battery_canvas.pack(fill="x")

        self.power_rate_label = tk.Label(right, text="-", font=(FONT_MONO, 8),
                                          bg=PANEL_BG, fg=DIM_FG, anchor="w")
        self.power_rate_label.pack(fill="x", pady=(0, 4))

        self.landed_label = tk.Label(right, text="-", font=(FONT_MONO, 9, "bold"),
                                      bg=PANEL_BG, fg=DIM_FG, anchor="w")
        self.landed_label.pack(fill="x")

        self.drill_label = tk.Label(right, text="Drill: -", font=(FONT_MONO, 9),
                                     bg=PANEL_BG, fg=DIM_FG, anchor="w")
        self.drill_label.pack(fill="x", pady=(2, 0))

        # --- Legs ---
        tk.Frame(self, height=1, bg=SEPARATOR).pack(fill="x", pady=(7, 0))
        tk.Label(self, text="LEGS · HIP / KNEE (rad)", font=(FONT_MONO, 8),
                 bg=PANEL_BG, fg=ACCENT_DIM, anchor="w").pack(fill="x", pady=(4, 1))
        legs_frame = tk.Frame(self, bg=PANEL_BG)
        legs_frame.pack(fill="x")
        self.leg_labels = []
        for i in range(3):
            lbl = tk.Label(legs_frame, text=f"L{i}: - / -", font=(FONT_MONO, 9),
                            bg=PANEL_BG, fg=FG, anchor="w")
            lbl.pack(fill="x")
            self.leg_labels.append(lbl)

        # --- Reaction wheels ---
        tk.Frame(self, height=1, bg=SEPARATOR).pack(fill="x", pady=(7, 0))
        tk.Label(self, text="REACTION WHEELS (rad/s)", font=(FONT_MONO, 8),
                  bg=PANEL_BG, fg=ACCENT_DIM, anchor="w").pack(fill="x", pady=(4, 1))
        rw_frame = tk.Frame(self, bg=PANEL_BG)
        rw_frame.pack(fill="x")
        self.rw_labels = {}
        for axis in ['x', 'y', 'z']:
            row = tk.Frame(rw_frame, bg=PANEL_BG)
            row.pack(fill="x")
            tk.Label(row, text=f"{axis.upper()}:", font=(FONT_MONO, 9), bg=PANEL_BG,
                     fg=DIM_FG, width=2, anchor="w").pack(side="left")
            bar = tk.Canvas(row, width=140, height=8, bg="#111", highlightthickness=0)
            bar.pack(side="left", padx=(2, 0))
            self.rw_labels[axis] = bar

    def set_offline(self):
        self.role_badge.config(text="OFFLINE", bg="#2a3a45")
        self.activity_label.config(text="No data received -- not spawned or not connected")
        self.landed_label.config(text="-", fg=DIM_FG)
        self.drill_label.config(text="Drill: -")
        for lbl in self.leg_labels:
            lbl.config(text="L-: - / -", fg=DIM_FG)
        self.battery_canvas.delete("all")
        self.power_rate_label.config(text="-", fg=DIM_FG)
        for axis in ['x', 'y', 'z']:
            self.rw_labels[axis].delete("all")
        self._draw_gyro(0.0, 0.0, dim=True)

    def update_from(self, st: AgentState):
        if not st.online:
            self.set_offline()
            return

        color = ROLE_COLORS.get(st.role, "#757575")
        self.role_badge.config(text=st.role, bg=color)
        self.activity_label.config(text=st.activity, fg=FG)

        if st.landed is True:
            self.landed_label.config(text="● LANDED", fg="#4CAF50")
        elif st.landed is False:
            self.landed_label.config(text="● IN FLIGHT", fg="#FF9800")
        else:
            self.landed_label.config(text="-", fg=DIM_FG)

        extended = st.drill < -0.02
        self.drill_label.config(
            text=f"Drill: {'EXTENDED' if extended else 'retracted'} ({st.drill:.2f})",
            fg="#FF9800" if extended else DIM_FG)

        for i, lbl in enumerate(self.leg_labels):
            hip = st.legs[f"hip_{i}"]
            knee = st.legs[f"knee_{i}"]
            lbl.config(text=f"L{i}: {hip:+.2f} / {knee:+.2f}", fg=FG)

        # Battery bar -- the canvas is packed with fill="x" so it stretches to
        # whatever width the panel actually ends up (was previously hardcoded
        # to the initial 140px construction width, so the fill fraction was
        # correct relative to that but tiny relative to the real, stretched
        # canvas -- e.g. 85% looked like a sliver). Query the live width.
        self.battery_canvas.delete("all")
        if st.battery is not None:
            cw = self.battery_canvas.winfo_width()
            if cw <= 1:
                cw = 140  # not mapped/rendered yet on the very first tick
            pct = max(0.0, min(100.0, st.battery))
            w = cw * pct / 100.0
            color = "#4CAF50" if pct > 40 else ("#FF9800" if pct > 15 else "#F44336")
            self.battery_canvas.create_rectangle(0, 0, w, 12, fill=color, outline="")
            self.battery_canvas.create_text(cw / 2, 6, text=f"{pct:.0f}%",
                                             fill="white", font=(FONT_MONO, 8))

        # Solar charge / discharge rate (swarm_manager.py's actual per-tick
        # rate for whatever the bot is currently doing, not just a static
        # spec number -- positive while RECHARGE-role sun-facing charging,
        # negative otherwise).
        if st.power_rate > 0:
            self.power_rate_label.config(text=f"☀ solar +{st.power_rate:.2f}%/tick",
                                          fg="#4CAF50")
        elif st.power_rate < 0:
            self.power_rate_label.config(text=f"⤓ draw {st.power_rate:.2f}%/tick",
                                          fg="#FF9800")
        else:
            self.power_rate_label.config(text="- %/tick", fg=DIM_FG)

        # RW speed bars (scale against the 1396 rad/s saturation limit)
        max_rw = 1396.0
        for axis in ['x', 'y', 'z']:
            canvas = self.rw_labels[axis]
            canvas.delete("all")
            val = st.rw[axis]
            frac = max(-1.0, min(1.0, val / max_rw))
            mid_x = 70
            w = frac * 70
            color = "#03A9F4" if abs(frac) < 0.85 else "#F44336"
            if w >= 0:
                canvas.create_rectangle(mid_x, 0, mid_x + w, 8, fill=color, outline="")
            else:
                canvas.create_rectangle(mid_x + w, 0, mid_x, 8, fill=color, outline="")
            canvas.create_line(mid_x, 0, mid_x, 8, fill="#666")

        self._draw_gyro(st.roll, st.pitch)

    def _draw_gyro(self, roll, pitch, dim=False):
        c = self.gyro
        c.delete("all")
        size = self.GYRO_SIZE
        cx, cy = size / 2, size / 2

        if dim:
            c.create_rectangle(0, 0, size, size, fill="#111", outline="")
            c.create_text(cx, cy, text="--", fill="#444", font=(FONT_MONO, 10))
            return

        # Pixels-per-radian for the pitch offset; clamps naturally since the
        # fill polygon is oversized and simply runs off-canvas at extremes.
        px_per_rad = size / math.pi
        direction = (math.cos(roll), math.sin(roll))
        perp = (-math.sin(roll), math.cos(roll))
        offset = pitch * px_per_rad
        line_cx = cx + perp[0] * offset
        line_cy = cy + perp[1] * offset

        span = size * 1.5
        p1 = (line_cx + direction[0] * span, line_cy + direction[1] * span)
        p2 = (line_cx - direction[0] * span, line_cy - direction[1] * span)
        p3 = (p2[0] - perp[0] * span, p2[1] - perp[1] * span)
        p4 = (p1[0] - perp[0] * span, p1[1] - perp[1] * span)

        # Sky fill (whole canvas), then ground polygon drawn on top -- the
        # canvas bounds themselves are the "gauge frame", no circular
        # clipping needed since this is a square indicator.
        c.create_rectangle(0, 0, size, size, fill="#3a6ea5", outline="")
        c.create_polygon([p1, p2, p3, p4], fill="#6b4a2b", outline="")
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill="white", width=2)

        # Fixed "aircraft" reference marker (always horizontal/centered) --
        # the classic artificial-horizon reference symbol.
        c.create_line(cx - 14, cy, cx - 4, cy, fill="#FFEB3B", width=2)
        c.create_line(cx + 4, cy, cx + 14, cy, fill="#FFEB3B", width=2)
        c.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill="#FFEB3B", outline="")

        # Corner tick marks -- subtle HUD framing.
        t = 7
        for (x0, y0, dx, dy) in ((1, 1, 1, 1), (size - 2, 1, -1, 1),
                                 (1, size - 2, 1, -1), (size - 2, size - 2, -1, -1)):
            c.create_line(x0, y0, x0 + dx * t, y0, fill=ACCENT_DIM)
            c.create_line(x0, y0, x0, y0 + dy * t, fill=ACCENT_DIM)


class StarfieldHeader(tk.Canvas):
    """Starfield banner with the mission title over it. Stars are seeded
    once (stable positions across redraws); a handful twinkle on a slow
    timer. Redraws on resize so the field always fills the width."""

    HEIGHT = 78
    N_STARS = 110

    def __init__(self, master):
        super().__init__(master, height=self.HEIGHT, bg=BG,
                         highlightthickness=0)
        rng = random.Random(162173)  # asteroid number -- stable star map
        # (u, v) in [0,1] so stars re-scatter correctly on resize
        self.stars = [(rng.random(), rng.random(),
                       rng.choice([1, 1, 1, 2]),  # mostly 1px, some 2px
                       rng.choice(STAR_COLORS))
                      for _ in range(self.N_STARS)]
        self.twinkle_set = set()
        self.bind("<Configure>", lambda e: self.redraw())
        self._twinkle()

    def _twinkle(self):
        rng = random.Random()
        self.twinkle_set = {rng.randrange(self.N_STARS) for _ in range(10)}
        self.redraw()
        self.after(700, self._twinkle)

    def redraw(self):
        self.delete("all")
        w = max(self.winfo_width(), 2)
        h = self.HEIGHT
        for i, (u, v, r, color) in enumerate(self.stars):
            x, y = u * w, v * h
            c = "#3a4a5a" if i in self.twinkle_set else color
            if r == 1:
                self.create_rectangle(x, y, x + 1, y + 1, fill=c, outline="")
            else:
                self.create_oval(x - 1, y - 1, x + 1, y + 1, fill=c, outline="")
        # a few slightly brighter "cross" stars for depth
        for u, v in [(0.12, 0.3), (0.87, 0.62), (0.55, 0.15), (0.33, 0.75)]:
            x, y = u * w, v * h
            self.create_line(x - 3, y, x + 3, y, fill="#bfe8ff")
            self.create_line(x, y - 3, x, y + 3, fill="#bfe8ff")
        self.create_text(w / 2, h / 2 - 8,
                         text="✹ 162173 RYUGU — SWARM TELEMETRY ✹",
                         fill=ACCENT, font=(FONT_MONO, 13, "bold"))
        self.create_text(w / 2, h / 2 + 12, text="· mission control uplink ·",
                         fill=DIM_FG, font=(FONT_MONO, 8))
        self.create_line(0, h - 1, w, h - 1, fill=PANEL_BORDER)


class DashboardApp:
    def __init__(self, root, node: SwarmGuiNode):
        self.root = root
        self.node = node
        root.title("Ryugu Swarm Dashboard")
        root.configure(bg=BG)

        StarfieldHeader(root).pack(fill="x")

        self.panels = {}
        container = tk.Frame(root, bg=BG)
        container.pack(fill="both", expand=True, padx=8, pady=6)
        for agent in AGENTS:
            panel = BotPanel(container, agent)
            panel.pack(fill="x", pady=5)
            self.panels[agent] = panel

        self._tick()

    def _tick(self):
        for agent in AGENTS:
            self.panels[agent].update_from(self.node.states[agent])
        self.root.after(200, self._tick)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmGuiNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.geometry("480x1000+1440+0")
    DashboardApp(root, node)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
