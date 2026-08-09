#!/usr/bin/env python3
"""Phase 7: restitution spot-check. The paper's e~=0.2 figure is an
ANALYTICAL derivation (zeta from effective leg stiffness/damping and body
mass, e = exp(-pi*zeta/sqrt(1-zeta^2))), not a measured simulation result
-- see scripts/generate_detailed_spacehopper.py's hip_joint comment
(~line 885-909). This script instead DROPS the robot (no active
controllers -- bridge/telemetry only, so no control input can add or
remove energy) from a known height and measures the peak-to-peak apex
height ratio across successive bounces directly from odometry, giving a
real empirical e = sqrt(h_next / h_prev) per bounce pair, independent of
the analytical derivation's assumptions.

NOTE, found while preparing this check: the hip_joint comment's own
derivation claims "restitution ~0.2 at c_joint = 0.15 N m s/rad -- ...
p=1.0 + 0.15 is the verified-stable operating point," but the actual
<dynamics><damping> value in both the generator and the generated
models/spacehopper/model.sdf is 0.05, not 0.15, for every leg joint
(hip and knee alike). This is a live discrepancy between the design
comment and the shipped model, not something this script changes --
flagged here and in the Phase 7 change report; this measurement reports
what the CURRENTLY SHIPPED value (0.05) actually produces, not a
re-derivation of the comment's own claimed 0.2 figure.

Drop height matches the original ad-hoc reference (~1.15 m, per the same
comment: "measured contact restitution was ~0.96 (bounces from a 1.15 m
drop did not decay)" -- that measurement was against the PRE-damping-fix
p=1.0-only configuration and was never retained as a logged run; this is
that missing logged run, against the CURRENT configuration).

N_DROPS repeats (small -- this is a spot-check, not a full battery).

Run: python3 restitution_spot_check.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p7rest.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
N_DROPS = 3
DROP_HEIGHT_ABOVE_REST = 1.15   # m, matches the original ad-hoc reference
SETTLE_TIMEOUT = 30.0
TRACE_WINDOW = 900.0            # generous: bounce period at this gravity is
                                 # large (~285s for the first bounce alone)
OUT = f"{LOG_DIR}/restitution_spot_check_results.json"

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


def start_world():
    gz_log = open(f"{LOG_DIR}/gz_restitution.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
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
        super().__init__('p7_restitution_monitor')
        self.z = None
        self.vz = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def find_rest_z(log):
    """Spawn high, no controllers, let it free-fall and settle passively;
    return the resting z (ground reference for this spawn x,y)."""
    kill_all()
    start_world()
    gz_respawn(0.0, 0.5, 5.2)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_restitution_ref.log", 'w')
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
    # BUG FIX (found live, Phase 7): the original version declared "rest"
    # once z stopped changing by >0.0005m between two 0.3s polls, 30 times
    # in a row. At this world's gravity (g=1.14e-4 m/s^2) that condition is
    # ALSO satisfied almost immediately after spawn -- early in a genuine
    # free-fall from 5.2m, z changes by only sub-mm amounts per 0.3s poll,
    # so the check false-triggered at t=1.6s (rest_z reported ~5.198, i.e.
    # still at the spawn height) in the first run, and both drops that
    # respawned from that bogus reference captured zero bounce apexes over
    # their full 900s trace windows. A tighter position-delta or velocity
    # threshold doesn't fix this either: velocity during early free-fall
    # here is ALSO sub-threshold for a long time (v=g*t stays under
    # 0.005 m/s until t~44s), so no purely-instantaneous or short-window
    # check can distinguish "still falling, very slowly" from "at rest" in
    # this gravity regime -- this is the same reason landing_controller.py
    # needs a genuinely long (60-120s) sustained-rest window, not a quick
    # check. Simplest reliable fix: don't auto-detect at all. This exact
    # SPAWN_Z=5.2 world has produced consistent contact-time data across
    # dozens of trials this session (Phase 7's self-righting/launch
    # batches) -- contact reliably occurs 150-210s after spawn. Wait a
    # fixed, comfortably-longer duration instead.
    FIXED_SETTLE_WAIT = 250.0
    log(f"  reference drop from z=5.2, waiting {FIXED_SETTLE_WAIT:.0f}s "
        f"(fixed, not auto-detected -- see BUG FIX comment) for passive settle...")
    t0 = time.time()
    while time.time() - t0 < FIXED_SETTLE_WAIT:
        node.spin_for(0.3)
    rest_z = node.z
    log(f"  rest_z={rest_z} after {time.time()-t0:.1f}s fixed wait")
    node.destroy_node()
    rclpy.shutdown()
    return rest_z


def run_one_drop(drop_idx, rest_z, log):
    kill_all()
    start_world()
    gz_respawn(0.0, 0.5, rest_z + DROP_HEIGHT_ABOVE_REST)
    make_bridge_yaml()
    logf = open(f"{LOG_DIR}/bridge_scout_1_restitution_drop{drop_idx}.log", 'w')
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

    # Find local apex heights: points where vz crosses from + to - (or a
    # local z maximum), above rest_z, after the drop.
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

    log(f"  [drop{drop_idx}] apex heights above rest (m): {[round(a,4) for a in apexes]}")
    log(f"  [drop{drop_idx}] empirical e per bounce pair: {[round(r,3) for r in ratios]}")

    return {"drop": drop_idx, "rest_z": rest_z, "drop_height": DROP_HEIGHT_ABOVE_REST,
            "apex_heights_above_rest": apexes, "e_per_bounce": ratios, "trace": trace}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    rest_z = find_rest_z(log)

    results = {"rest_z": rest_z, "drops": []}
    for i in range(1, N_DROPS + 1):
        log(f"=== drop {i}/{N_DROPS} ===")
        r = run_one_drop(i, rest_z, log)
        results["drops"].append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    all_e = [e for d in results["drops"] for e in d["e_per_bounce"]]
    log("=== spot-check complete ===")
    if all_e:
        mean_e = sum(all_e) / len(all_e)
        log(f"all empirical e values: {[round(e,3) for e in all_e]}")
        log(f"mean e = {mean_e:.3f} (analytical design target: ~0.2; "
            f"Biele et al. MASCOT median ~0.4, max ~0.6)")
    else:
        log("no bounce pairs captured -- see trace data for manual inspection")


if __name__ == '__main__':
    main()
