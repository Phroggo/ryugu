#!/usr/bin/env python3
"""Phase 19: damping Pareto sweep, bounce-energy leg.

Third Pareto axis: bounce energy retention (first-bounce apex-height
ratio, same "effective restitution" definition as Phase 8/10's terrain
restitution sweep), across the same 6 leg-joint damping values as
damping_launch_sweep.py. Bridge-only, no controllers -- isolates the
passive leg-joint damping's effect on bounce dynamics, same methodology
as restitution_sensitivity_sweep_postfix.py but varying the ROBOT's leg
damping instead of the TERRAIN's restitution coefficient.

TRACE_WINDOW shortened to 150s (vs. Phase 10's 900s) -- only the first
bounce's apex is needed for this metric, which occurs early; a 150s
window is ample based on the ~1s-scale bounce periods seen in prior
restitution work at this drop height.

n=2 reps per damping value (bounce dynamics are far less stochastic than
active-control maneuvers -- passive drop physics, same pattern Phase 10
found: e.g. restitution repeated to 3 decimal places across drops).

Run: python3 damping_bounce_sweep.py
"""
import json, math, os, subprocess, time

PHASE19_DIR = os.path.dirname(__file__)
os.environ['GZ_SIM_RESOURCE_PATH'] = (
    '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models:' + f'{PHASE19_DIR}/variant_models')

LOG_DIR = PHASE19_DIR
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p19bounce.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'

N_DROPS = 2
DROP_HEIGHT_ABOVE_REST = 1.15
FIXED_SETTLE_WAIT = 250.0
# BUG FIX (2026-08-14): was 150.0, on the wrong assumption that the first
# bounce apex occurs "early." Under Ryugu's gravity (g=1.14e-4 m/s^2),
# free-fall from 1.15m alone takes ~142s (t=sqrt(2h/g)) -- 150s left
# almost no time to see impact at all, let alone a bounce, so every
# single drop across all 6 damping values showed zero detected apexes.
# Reverted to Phase 10's proven value (restitution_sensitivity_sweep_postfix.py).
TRACE_WINDOW = 900.0
OUT = f"{LOG_DIR}/damping_bounce_sweep_results.json"

CONFIGS = [
    ("c0.005", "model://spacehopper_damp0p005"),
    ("c0.02", "model://spacehopper_damp0p02"),
    ("c0.05_current", "model://spacehopper"),
    ("c0.08", "model://spacehopper_damp0p08"),
    ("c0.12", "model://spacehopper_damp0p12"),
    ("c0.15", "model://spacehopper_damp0p15"),
]

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def make_bridge_yaml():
    entries = [
        ('/scout_1/odometry', '/model/scout_1/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
    ]
    with open(BRIDGE_YAML, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')


def kill_all():
    subprocess.run(['pkill', '-9', '-f', 'bridge_scout_1'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world(log):
    log("  (re)starting gz sim daemon...")
    gz_log = open(f"{LOG_DIR}/gz_damping_bounce.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def gz_respawn(model_uri, x, y, z):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    req = (f"sdf_filename: '{model_uri}', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


class Monitor(Node):
    def __init__(self):
        super().__init__('p19_damping_bounce_monitor')
        self.z = None
        self.vz = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def find_rest_z(model_uri, label, log):
    kill_all()
    start_world(log)
    gz_respawn(model_uri, 0.0, 0.5, 5.2)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_bounce_{label}_ref.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                       '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
                       '-p', f'config_file:={BRIDGE_YAML}'],
                      stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.z is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    log(f"  [{label}] reference drop from z=5.2, fixed {FIXED_SETTLE_WAIT:.0f}s wait...")
    t0 = time.time()
    while time.time() - t0 < FIXED_SETTLE_WAIT:
        node.spin_for(0.3)
    rest_z = node.z
    log(f"  [{label}] rest_z={rest_z}")
    node.destroy_node()
    rclpy.shutdown()
    return rest_z


def run_one_drop(model_uri, label, drop_idx, rest_z, log):
    kill_all()
    start_world(log)
    gz_respawn(model_uri, 0.0, 0.5, rest_z + DROP_HEIGHT_ABOVE_REST)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_bounce_{label}_drop{drop_idx}.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                       '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
                       '-p', f'config_file:={BRIDGE_YAML}'],
                      stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.z is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)

    trace = []
    t0 = time.time()
    last_sample = -1.0
    while time.time() - t0 < TRACE_WINDOW:
        node.spin_for(0.3)
        elapsed = time.time() - t0
        if elapsed - last_sample >= 0.3:
            trace.append({"t": round(elapsed, 2), "z": node.z, "vz": node.vz})
            last_sample = elapsed

    node.destroy_node()
    rclpy.shutdown()

    apexes = []
    for i in range(1, len(trace) - 1):
        a, b, c = trace[i - 1], trace[i], trace[i + 1]
        if b["z"] is not None and a["z"] is not None and c["z"] is not None:
            if b["z"] >= a["z"] and b["z"] >= c["z"] and (b["z"] - rest_z) > 0.001:
                apexes.append(b["z"] - rest_z)

    ratios = []
    for i in range(len(apexes) - 1):
        if apexes[i] > 1e-6:
            ratios.append(math.sqrt(max(apexes[i + 1], 0.0) / apexes[i]))

    log(f"  [{label} drop{drop_idx}] apex heights above rest (m): {[round(a,4) for a in apexes[:5]]}"
        f"{'...' if len(apexes) > 5 else ''}")
    first_bounce_e = ratios[0] if ratios else None
    log(f"  [{label} drop{drop_idx}] FIRST-BOUNCE e: {first_bounce_e}")

    return {"label": label, "drop": drop_idx, "rest_z": rest_z,
            "drop_height": DROP_HEIGHT_ABOVE_REST, "apex_heights_above_rest": apexes,
            "e_per_bounce": ratios, "first_bounce_e": first_bounce_e}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    all_results = []
    for label, model_uri in CONFIGS:
        log(f"=== {label} ({model_uri}) ===")
        rest_z = find_rest_z(model_uri, label, log)
        for i in range(1, N_DROPS + 1):
            log(f"  --- drop {i}/{N_DROPS} ---")
            r = run_one_drop(model_uri, label, i, rest_z, log)
            all_results.append(r)
            with open(f"{LOG_DIR}/damping_bounce_sweep_results.json", 'w') as f:
                json.dump(all_results, f, indent=2)

    log("=== sweep complete ===")
    for label, _ in CONFIGS:
        vals = [r["first_bounce_e"] for r in all_results
                if r["label"] == label and r["first_bounce_e"] is not None]
        log(f"{label}: first-bounce e per drop = {[round(v,4) for v in vals]}")


if __name__ == '__main__':
    main()
