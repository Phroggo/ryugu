#!/usr/bin/env python3
"""Severe-tilt-only rerun (C15/C16), v2: fixes two issues found in v1 --
(1) a fresh rclpy context was created/destroyed every trial, which is not
how any other harness this week does it and is a plausible source of the
odometry-never-arrives failure seen in v1 (unconfirmed root cause, but
removed as a variable); (2) animation step count cut roughly in half.
One persistent monitor node for the whole batch; per-trial state is just
reset between trials, not the node/subscriptions themselves.
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/severe_tilt_rerun/results_v3.json"
LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/severe_tilt_rerun"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/install/ryugu_sim/share/ryugu_sim/worlds/ryugu.sdf'
SPAWN_XY = (0.0, 0.5)
SPAWN_Z = 6.0
SUCCESS_UZ = 0.9
LANDED_WAIT_TIMEOUT = 200.0
RIGHTING_WAIT_TIMEOUT = 120.0

SEVERE_TRIALS = [
    (121.61964553933414, 53.990614576521736),
    (143.31551374969447, 224.95001197124643),
    (168.5600704165762, 7.114322723335178),
    (142.5620423228658, 19.53345092603202),
    (159.0401403987306, 159.08094213262615),
    (160.69123558056054, 75.82424928489667),
    (138.84584792673297, 0.3982422324222501),
    (172.21419291491986, 8.319041944248857),
]


def slerp(q0, q1, t):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    dot = x0*x1 + y0*y1 + z0*z1 + w0*w1
    if dot < 0.0:
        x1, y1, z1, w1, dot = -x1, -y1, -z1, -w1, -dot
    if dot > 0.9995:
        x = x0 + t*(x1-x0); y = y0 + t*(y1-y0)
        z = z0 + t*(z1-z0); w = w0 + t*(w1-w0)
    else:
        theta_0 = math.acos(dot)
        theta = theta_0 * t
        s0 = math.cos(theta) - dot * math.sin(theta) / math.sin(theta_0)
        s1 = math.sin(theta) / math.sin(theta_0)
        x = s0*x0 + s1*x1; y = s0*y0 + s1*y1
        z = s0*z0 + s1*z1; w = s0*w0 + s1*w1
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/n, y/n, z/n, w/n)


def tilt_quaternion(tilt_deg, azimuth_deg):
    half = math.radians(tilt_deg) / 2.0
    az = math.radians(azimuth_deg)
    s = math.sin(half)
    return (s * math.cos(az), s * math.sin(az), 0.0, math.cos(half))


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def animate_to(quat, steps=15, step_dt=0.15, start=IDENTITY, holds=5):
    x, y = SPAWN_XY
    for i in range(1, steps + 1):
        t = i / steps
        q = slerp(start, quat, t)
        subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/set_pose',
                         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '1000', '--req',
                         f"name: 'scout_1', position: {{x: {x}, y: {y}, z: {SPAWN_Z}}}, "
                         f"orientation: {{x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}}}"],
                        capture_output=True)
        time.sleep(step_dt)
    for _ in range(holds):
        subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/set_pose',
                         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '1000', '--req',
                         f"name: 'scout_1', position: {{x: {x}, y: {y}, z: {SPAWN_Z}}}, "
                         f"orientation: {{x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}}}"],
                        capture_output=True)
        time.sleep(0.15)


def kill_landing_node():
    subprocess.run(['pkill', '-9', '-f', 'landing_scout_1'], capture_output=True)
    time.sleep(1.0)


def launch_landing_node(trial_idx):
    logf = open(f"{LOG_DIR}/landing_scout_1_severe_v2_trial{trial_idx}.log", 'w')
    return subprocess.Popen(['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
                              '--ros-args', '-r', '__node:=landing_scout_1'],
                             stdout=logf, stderr=subprocess.STDOUT)


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('severe_tilt_monitor_v2')
        self.uz = None
        self.landed = None
        self.ever_righted = False
        self.odom_count = 0
        self.current_quat = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/righting_active', self.righting_cb, 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.current_quat = (q.x, q.y, q.z, q.w)
        self.odom_count += 1

    def landed_cb(self, msg):
        self.landed = msg.data

    def righting_cb(self, msg):
        if msg.data:
            self.ever_righted = True

    def reset_for_trial(self):
        self.uz = None
        self.landed = None
        self.ever_righted = False

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    results = []

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    # "gz sim <args>" re-execs into child processes literally named "gz sim
    # server" / "gz sim gui" -- the launch-command-line pattern above does
    # NOT match those (found live 2026-08-05: two full leftover instances
    # from an earlier diagnostic survived exactly this pkill pattern and
    # ran concurrently with this harness's own instance, colliding on the
    # same default topic namespace with a duplicate "scout_1" entity --
    # this was the real cause of the intermittent odometry failures, not
    # anything about set_pose volume or rclpy context reuse). Match on
    # "gz sim" alone, which covers both the launcher and its children.
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)
    remaining = subprocess.run(['pgrep', '-af', 'gz sim|bridge_scout_1|landing_scout_1'],
                                capture_output=True, text=True).stdout
    remaining = '\n'.join(l for l in remaining.splitlines() if 'eval' not in l and 'bash -c' not in l)
    if remaining.strip():
        print(f"WARNING: processes survived cleanup:\n{remaining}", flush=True)

    log("Starting gz sim...")
    gz_log = open(f"{LOG_DIR}/gz_sim_v2.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)

    log("Spawning scout_1 UPRIGHT (once for the whole batch)...")
    x, y = SPAWN_XY
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     f"pose {{ position {{ x: {x} y: {y} z: {SPAWN_Z} }} }}"],
                    capture_output=True)
    time.sleep(2)

    bridge_log = open(f"{LOG_DIR}/bridge_scout_1_v2.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                       '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
                       '-p', f'config_file:={BRIDGE_YAML}'],
                      stdout=bridge_log, stderr=subprocess.STDOUT)
    time.sleep(3)

    # ONE persistent rclpy context + node for the whole batch, matching
    # every other harness this session, unlike v1 which re-init'd rclpy
    # every trial.
    rclpy.init()
    node = TrialMonitor()
    t0 = time.time()
    while node.odom_count == 0 and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    log(f"Sanity check: monitor node created, odom_count after 10s = {node.odom_count}")
    if node.odom_count == 0:
        log("FATAL: odometry never arrived even before any trial started. Aborting.")
        with open(OUT, 'w') as f:
            json.dump({"error": "odometry never arrived before first trial"}, f, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        return

    current_quat = IDENTITY
    for i, (tilt_deg, az) in enumerate(SEVERE_TRIALS):
        quat = tilt_quaternion(tilt_deg, az)
        log(f"--- severe trial {i+1}/{len(SEVERE_TRIALS)}: commanded_tilt={tilt_deg:.1f} deg az={az:.0f} ---")

        kill_landing_node()
        node.reset_for_trial()

        # Read the robot's REAL current orientation from odometry rather
        # than trusting our own bookkeeping variable -- after a trial where
        # righting never triggers, the robot can drift substantially during
        # the 200s landed-wait (observed live: trial 6 drifted to u_z=0.96,
        # near upright, by the end of its wait, while the script still
        # thought it was at the commanded 160.7 deg), so animating from an
        # assumed start silently produced the wrong final tilt for trials
        # 6-8 in the previous run. Get a few fresh reads first.
        for _ in range(10):
            node.spin_for(0.1)
        real_current = node.current_quat if node.current_quat is not None else current_quat
        log(f"Real current orientation before this trial's animation: {real_current}")

        log("Animating back to upright...")
        animate_to(IDENTITY, start=real_current)
        node.spin_for(0.5)
        time.sleep(1.0)
        log(f"Animating to target tilt {tilt_deg:.1f} deg...")
        animate_to(quat, start=IDENTITY)
        node.spin_for(0.5)
        current_quat = quat

        launch_landing_node(i + 1)
        time.sleep(3)

        t0 = time.time()
        while node.uz is None and time.time() - t0 < 10.0:
            node.spin_for(0.2)
        start_uz = node.uz
        log(f"start uz={start_uz} (expected {math.cos(math.radians(tilt_deg)):.4f}), "
            f"odom_count={node.odom_count}")

        land_t0 = time.time()
        while time.time() - land_t0 < LANDED_WAIT_TIMEOUT and node.landed is not True:
            node.spin_for(0.3)
        log(f"landed={node.landed} after {time.time()-land_t0:.1f}s uz={node.uz}")

        outcome = "no_landing"
        final_uz = node.uz
        if node.landed is True:
            right_t0 = time.time()
            recovered = False
            while time.time() - right_t0 < RIGHTING_WAIT_TIMEOUT:
                node.spin_for(0.2)
                if node.uz is not None and node.uz > SUCCESS_UZ:
                    recovered = True
                    break
            final_uz = node.uz
            outcome = "recovered" if recovered else "failed"
            log(f"outcome={outcome} final_uz={final_uz}")

        results.append({
            "trial": i + 1, "commanded_tilt_deg": tilt_deg, "azimuth_deg": az,
            "start_uz": start_uz, "landed": node.landed, "final_uz": final_uz,
            "outcome": outcome, "righting_ever_triggered": node.ever_righted,
        })
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    n_recovered = sum(1 for r in results if r["outcome"] == "recovered")
    n_triggered = sum(1 for r in results if r.get("righting_ever_triggered"))
    log(f"=== severe-tilt batch complete: {n_recovered}/{len(SEVERE_TRIALS)} recovered, "
        f"{n_triggered}/{len(SEVERE_TRIALS)} triggered righting ===")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
