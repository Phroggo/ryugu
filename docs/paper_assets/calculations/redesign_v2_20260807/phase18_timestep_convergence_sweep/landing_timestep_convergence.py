#!/usr/bin/env python3
"""Phase 18: landing timestep convergence sweep, 0.5/1/2/4/8 ms, n=5 each.

Reviewer backlog item: no landing-specific timestep check exists at all
currently (only launch, and only 1ms/4ms at that -- and a self-righting/
yaw-slew spot check, which the reviewer explicitly said isn't sufficient
on its own). This measures the passive landing/settle physics --
drop-and-rest-z convergence -- across a real 5-point timestep sweep,
reusing Phase 10's proven bridge-only drop methodology
(restitution_sensitivity_sweep_postfix.py's find_rest_z: drop from
z=5.2, fixed 250s settle wait, read final z). Bridge-only, no
controllers -- isolates the passive contact/settle physics from any
controller-timing interaction, matching how "landing" physics is
measured elsewhere in this codebase (Phase 8/10's restitution work).

1ms reuses the live worlds/ryugu.sdf; 4ms reuses Phase 4's
ryugu_4ms.sdf; 0.5ms/2ms/8ms use this phase's generated variants.

Run: python3 landing_timestep_convergence.py
"""
import json, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p18landing.yaml"

N_REPEATS = 5
SPAWN_Z = 5.2
FIXED_SETTLE_WAIT = 250.0
OUT = f"{LOG_DIR}/landing_timestep_convergence_results.json"

WORLDS = [
    ("0p5ms", f"{LOG_DIR}/ryugu_0p5ms.sdf"),
    ("1ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"),
    ("2ms", f"{LOG_DIR}/ryugu_2ms.sdf"),
    ("4ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/"
             "redesign_v2_20260807/phase4_attitude_revalidation/ryugu_4ms.sdf"),
    ("8ms", f"{LOG_DIR}/ryugu_8ms.sdf"),
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


def start_world(world_file, label):
    gz_log = open(f"{LOG_DIR}/gz_landing_{label}.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', world_file],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def gz_respawn(x, y, z):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


class Monitor(Node):
    def __init__(self):
        super().__init__('p18_landing_timestep_conv')
        self.z = None
        self.vz = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def run_one_drop(world_file, label, rep, log):
    kill_all()
    start_world(world_file, label)
    gz_respawn(0.0, 0.5, SPAWN_Z)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_landing_{label}_rep{rep}.log", 'w')
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
    log(f"  [{label} rep{rep}] drop from z={SPAWN_Z}, fixed {FIXED_SETTLE_WAIT:.0f}s wait...")
    t0 = time.time()
    while time.time() - t0 < FIXED_SETTLE_WAIT:
        node.spin_for(0.3)
    rest_z = node.z
    rest_vz = node.vz
    log(f"  [{label} rep{rep}] rest_z={rest_z} rest_vz={rest_vz}")
    node.destroy_node()
    rclpy.shutdown()

    return {"label": label, "rep": rep, "rest_z": rest_z, "rest_vz": rest_vz}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for label, world_file in WORLDS:
        log(f"=== starting {label} world, {N_REPEATS} repeats ===")
        for rep in range(1, N_REPEATS + 1):
            r = run_one_drop(world_file, label, rep, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== all repeats complete ===")
    for label, _ in WORLDS:
        vals = [r["rest_z"] for r in results if r["label"] == label and r["rest_z"] is not None]
        if vals:
            mean = sum(vals) / len(vals)
            spread = max(vals) - min(vals)
            log(f"{label}: n={len(vals)} rest_z={[round(v,6) for v in vals]} "
                f"mean={mean:.6f} range={spread:.6f}")
        else:
            log(f"{label}: no valid rest_z samples")


if __name__ == '__main__':
    main()
