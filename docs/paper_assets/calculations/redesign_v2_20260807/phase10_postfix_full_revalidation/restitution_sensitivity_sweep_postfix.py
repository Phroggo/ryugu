#!/usr/bin/env python3
"""Phase 10, item 3: re-run of Phase 8's Priority 3b (restitution e sweep),
against the Phase 9-fixed hopper_locomotion.py/landing_controller.py.

NOTE: this test launches NO controllers at all (bridge only, passive
physics) -- hopper_locomotion.py and landing_controller.py, the two files
Phase 9 touched, never load in this scenario. It is structurally
impossible for the Phase 9 fix to change this test's outcome. Rerunning
anyway per explicit instruction (cheap confirmatory evidence); expect an
exact repeat of Phase 8's e=0.113 for all three sweep points, not a
different number.

Reuses the SAME terrain variants generated for Phase 8
(phase8_overnight_batch/variant_models/regolith_plane_eXXX,
variant_worlds/ryugu_eXXX.sdf) -- static SDF files, no need to regenerate.

Run: python3 restitution_sensitivity_sweep_postfix.py
"""
import json, math, os, subprocess, time

LOG_DIR = os.path.dirname(__file__)
PHASE8_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch"
os.environ['GZ_SIM_RESOURCE_PATH'] = (
    '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models:' + f'{PHASE8_DIR}/variant_models')
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p10e.yaml"
N_DROPS = 3
DROP_HEIGHT_ABOVE_REST = 1.15
FIXED_SETTLE_WAIT = 250.0
TRACE_WINDOW = 900.0

E_VALUES = [
    (0.1, f"{PHASE8_DIR}/variant_worlds/ryugu_e010.sdf"),
    (0.2, f"{PHASE8_DIR}/variant_worlds/ryugu_e020.sdf"),
    (0.4, f"{PHASE8_DIR}/variant_worlds/ryugu_e040.sdf"),
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
    gz_log = open(f"{LOG_DIR}/gz_e{label}_postfix.log", 'a')
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
        super().__init__('p10_restitution_monitor')
        self.z = None
        self.vz = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def find_rest_z(world_file, label, log):
    kill_all()
    start_world(world_file, label)
    gz_respawn(0.0, 0.5, 5.2)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_e{label}pf_ref.log", 'w')
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
    log(f"  reference drop from z=5.2, fixed {FIXED_SETTLE_WAIT:.0f}s wait...")
    t0 = time.time()
    while time.time() - t0 < FIXED_SETTLE_WAIT:
        node.spin_for(0.3)
    rest_z = node.z
    log(f"  rest_z={rest_z}")
    node.destroy_node()
    rclpy.shutdown()
    return rest_z


def run_one_drop(world_file, label, drop_idx, rest_z, log):
    kill_all()
    start_world(world_file, label)
    gz_respawn(0.0, 0.5, rest_z + DROP_HEIGHT_ABOVE_REST)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_e{label}pf_drop{drop_idx}.log", 'w')
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
        if elapsed - last_sample >= 0.5:
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

    log(f"  [e={label} drop{drop_idx}] apex heights above rest (m): {[round(a,4) for a in apexes[:5]]}"
        f"{'...' if len(apexes) > 5 else ''}")
    first_bounce_e = ratios[0] if ratios else None
    log(f"  [e={label} drop{drop_idx}] FIRST-BOUNCE e: {first_bounce_e}")

    return {"e_target": label, "drop": drop_idx, "rest_z": rest_z,
            "drop_height": DROP_HEIGHT_ABOVE_REST, "apex_heights_above_rest": apexes,
            "e_per_bounce": ratios, "first_bounce_e": first_bounce_e}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    all_results = []
    for e_target, world_file in E_VALUES:
        label = f"{int(round(e_target*100)):03d}"
        log(f"=== e_target={e_target} ({world_file}) ===")
        rest_z = find_rest_z(world_file, label, log)
        for i in range(1, N_DROPS + 1):
            log(f"  --- drop {i}/{N_DROPS} ---")
            r = run_one_drop(world_file, label, i, rest_z, log)
            all_results.append(r)
            with open(f"{LOG_DIR}/restitution_sweep_postfix_results.json", 'w') as f:
                json.dump(all_results, f, indent=2)

    log("=== sweep complete ===")
    for e_target, _ in E_VALUES:
        label = f"{int(round(e_target*100)):03d}"
        vals = [r["first_bounce_e"] for r in all_results
                if r["e_target"] == label and r["first_bounce_e"] is not None]
        log(f"e_target={e_target}: first-bounce e per drop = {[round(v,3) for v in vals]}")
        log(f"  COMPARISON vs Phase 8 pre-fix: 0.113, 0.113, 0.113 (expect identical -- see docstring)")


if __name__ == '__main__':
    main()
