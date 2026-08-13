#!/usr/bin/env python3
"""Phase 14: torque-vs-geometry investigation into full_inversion's
righting failure mode, part 1 -- diagnostic instrumentation.

Question: when full_inversion's active RW righting gives up after 5
attempts (100% give-up rate per Phase 12/13), is it because the roll
never gets the body close to upright (a geometry/torque problem), or
because it gets close but can't complete the HOLD-CONFIRM stabilization
(u_z>0.9 sustained ~2s at low angular rate) before running out of time
or drifting back down (a damping/timing problem)?

landing_controller.py's _run_righting_sequence already has a per-tick
u_z<0.9 gate that switches into a HOLD-CONFIRM branch the instant u_z
crosses 0.9 -- so near-misses ARE being detected internally, just not
previously logged distinctly from ordinary active correction. Added two
purely-observational log lines (no control-law change, verified via diff
that nothing else changed):
  - HOLD-START: first tick of a fresh hold attempt (u_z just crossed 0.9,
    angular rate already low enough to begin holding)
  - HOLD-LOST: a hold that had genuinely started (>0 ticks held) got
    released before completing RIGHTING_HOLD_TICKS (~2s)
Existing "Self-righting successful" log marks a hold that completed.

n=15 full_inversion trials (bounded for a diagnostic pass, not a paper
number) with the baseline (unperturbed) model, using Phase 13's proven
harness structure and Phase 11's /righting_active-keyed recovery timer.

Run: python3 righting_hold_diagnostic.py
"""
import json, math, os, subprocess, time, random

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = os.path.dirname(__file__)
OUT = f"{LOG_DIR}/righting_hold_diagnostic_results.json"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p14.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
N_TRIALS = 15
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 350.0
RIGHTING_WAIT_TIMEOUT = 120.0
BUCKET_LABEL = "full_inversion"
BUCKET_LO, BUCKET_HI = 170.0, 180.0


def kill_scout1_nodes():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    time.sleep(1.5)


def make_bridge_yaml():
    imu_gz_topic = '/world/ryugu_world/model/scout_1/link/base_link/sensor/imu_sensor/imu'
    entries = [
        ('/scout_1/imu', imu_gz_topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
        ('/scout_1/odometry', '/model/scout_1/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
    ]
    for axis in ['x', 'y', 'z']:
        entries.append((f'/scout_1/rw_{axis}_joint_cmd_vel',
                         f'/model/scout_1/joint/rw_{axis}_joint/cmd_vel',
                         'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
    for j in range(3):
        for jt in ['hip', 'knee']:
            entries.append((f'/scout_1/joint_{jt}_joint_{j}_cmd_pos',
                             f'/model/scout_1/joint/{jt}_joint_{j}/0/cmd_pos',
                             'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
    entries.append(('/scout_1/cmd_drill', '/model/scout_1/joint/drill_joint/0/cmd_pos',
                     'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
    with open(BRIDGE_YAML, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')


def launch_scout1_nodes(label, idx):
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{LOG_DIR}/{name}_{label}_trial{idx}.log", 'w')
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


def start_world(log):
    log("  (re)starting gz sim daemon...")
    gz_log = open(f"{LOG_DIR}/gz_p14_batch.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def kill_all():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('p14_hold_diag_monitor')
        self.uz = None
        self.landed = None
        self.righting_started_at = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/righting_active', self.righting_cb, 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)

    def landed_cb(self, msg):
        self.landed = msg.data

    def righting_cb(self, msg):
        if msg.data and self.righting_started_at is None:
            self.righting_started_at = time.time()

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def run_one_trial(trial_idx, log):
    tilt_deg = random.uniform(BUCKET_LO, BUCKET_HI)
    az = random.uniform(0, 360)

    kill_scout1_nodes()
    quat = tilt_quaternion(tilt_deg, az)
    gz_respawn(0.0, 0.5, SPAWN_Z, quat)
    launch_scout1_nodes(BUCKET_LABEL, trial_idx)

    rclpy.init()
    node = TrialMonitor()
    t0 = time.time()
    while node.uz is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    start_uz = node.uz
    log(f"  [trial{trial_idx}/{N_TRIALS}] commanded_tilt={tilt_deg:.1f} "
        f"az={az:.0f} start_uz={start_uz}")

    land_t0 = time.time()
    while time.time() - land_t0 < LANDED_WAIT_TIMEOUT and node.landed is not True:
        node.spin_for(0.3)
    log(f"    landed={node.landed} after {time.time()-land_t0:.1f}s uz={node.uz}")

    outcome = "no_landing"
    final_uz = node.uz
    recover_time_s = None
    landed_flag = node.landed
    if node.landed is True:
        wait_t0 = time.time()
        recovered = False
        while time.time() - wait_t0 < RIGHTING_WAIT_TIMEOUT:
            node.spin_for(0.2)
            if node.uz is not None and node.uz > SUCCESS_UZ:
                recovered = True
                if node.righting_started_at is not None:
                    recover_time_s = time.time() - node.righting_started_at
                break
        final_uz = node.uz
        outcome = "recovered" if recovered else "failed"
        log(f"    outcome={outcome} final_uz={final_uz} recover_time_s={recover_time_s}")

    node.destroy_node()
    rclpy.shutdown()

    return {
        "trial": trial_idx, "commanded_tilt_deg": tilt_deg, "azimuth_deg": az,
        "start_uz": start_uz, "landed": landed_flag, "final_uz": final_uz,
        "outcome": outcome, "righting_started_at": node.righting_started_at,
        "recover_time_s": recover_time_s,
    }


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world(log)
    make_bridge_yaml()

    results = []
    log(f"=== bucket={BUCKET_LABEL} ({BUCKET_LO}-{BUCKET_HI}deg), n={N_TRIALS}, "
        f"HOLD-START/HOLD-LOST diagnostic instrumentation active ===")
    for i in range(N_TRIALS):
        r = run_one_trial(i + 1, log)
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== diagnostic batch complete ===")
    n_rec = sum(1 for r in results if r["outcome"] == "recovered")
    log(f"{BUCKET_LABEL}: {n_rec}/{len(results)} recovered")


if __name__ == '__main__':
    main()
