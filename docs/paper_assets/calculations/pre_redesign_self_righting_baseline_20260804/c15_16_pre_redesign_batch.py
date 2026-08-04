#!/usr/bin/env python3
"""C15/C16 retest: reproduce the pre-redesign self-righting baseline
("5 of 21 attempts, 24%") by running the OLD landing_controller.py
(commit 5c9e278, swapped in and rebuilt for this test) through 21 trials
at randomized tilts, using the spawn-height and fresh-node-per-trial fixes
already confirmed working today. Original baseline was described as
measured "over a long run" (organic tilts from real operation), not a
controlled bucketed experiment, so tilts here are drawn uniformly random
rather than split into fixed categories."""
import json, math, subprocess, time, random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/c15_16_results.json"
NODE_LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1.yaml"
N_TRIALS = 21
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 200.0
RIGHTING_WAIT_TIMEOUT = 120.0  # long enough for an initial 5-attempt failure
                                # PLUS a follow-up "tilted while LANDED"
                                # re-trigger to resolve, per the isolated
                                # diagnostic (fail-then-succeed took ~46s)


def kill_scout1_nodes():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    time.sleep(1.5)


def launch_scout1_nodes(trial_idx):
    # Only bridge + landing_controller -- confirmed via isolated diagnostic
    # that running hopper_locomotion/attitude_controller alongside the
    # swapped-in pre-redesign landing_controller prevents landing detection
    # from ever completing (root cause not fully diagnosed; landing_controller
    # alone works reliably and is all this test actually needs).
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{NODE_LOG_DIR}/{name}_c1516_trial{trial_idx}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)


def tilt_quaternion(tilt_deg, azimuth_deg):
    half = math.radians(tilt_deg) / 2.0
    az = math.radians(azimuth_deg)
    s = math.sin(half)
    return (s * math.cos(az), s * math.sin(az), 0.0, math.cos(half))


def gz_respawn(x, y, z, quat):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    qx, qy, qz, qw = quat
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
           f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('c1516_monitor')
        self.uz = None
        self.landed = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)

    def landed_cb(self, msg):
        self.landed = msg.data

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    results = []

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    for i in range(N_TRIALS):
        tilt_deg = random.uniform(20, 180)
        az = random.uniform(0, 360)
        quat = tilt_quaternion(tilt_deg, az)
        log(f"--- trial {i+1}/{N_TRIALS}: commanded_tilt={tilt_deg:.0f} deg az={az:.0f} ---")

        kill_scout1_nodes()
        gz_respawn(0.0, 0.5, SPAWN_Z, quat)
        launch_scout1_nodes(i + 1)

        rclpy.init()
        node = TrialMonitor()
        t0 = time.time()
        while node.uz is None and time.time() - t0 < 10.0:
            node.spin_for(0.2)
        start_uz = node.uz
        log(f"start uz={start_uz}")

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
            "outcome": outcome,
        })
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

        node.destroy_node()
        rclpy.shutdown()

    n_recovered = sum(1 for r in results if r["outcome"] == "recovered")
    log(f"=== batch complete: {n_recovered}/{N_TRIALS} recovered ===")


if __name__ == '__main__':
    main()
