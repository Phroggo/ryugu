#!/usr/bin/env python3
"""Phase 17: re-run Phase 13's side_rest/full_inversion retime in a SINGLE
CONTINUOUS gz-sim session, matching Phase 7's original structure exactly.

User caught a real methodology gap in Phase 13, verified here before
writing a line of new harness code: Phase 7's self_righting_batch_3bucket.py
starts gz-sim ONCE, before its bucket loop, and never restarts it --
all 60 trials (3 buckets) ran on one continuous physics process. Phase
13's baseline_recovery_timer_retime.py added a kill_all()+start_world()
call BETWEEN buckets (inside the bucket loop, after each bucket's n=20
trials complete) -- so Phase 13's full_inversion trials ran on a freshly
restarted daemon, not the same continuously-running one side_rest (and,
in Phase 7, moderate before that) had already warmed up. That's a real,
unacknowledged difference in HOW the simulation was run, not just
random seed variation -- a long-running gz-sim/DART process vs. a cold
start can differ in solver warm-up state, floating-point accumulation,
timing-adjacent races. Phase 13's report claimed "harness parameters
identical" after diffing SPAWN_Z/timeouts/etc., but did not check the
daemon-restart structure itself -- confirmed as the gap.

This script is byte-identical to Phase 13's except: the kill_all()+
start_world() call between buckets is REMOVED, matching Phase 7's
single-continuous-session structure. If full_inversion's rate comes
back down near Phase 7's original 15% under this structure, that
confirms the daemon restart (not a genuine physics-nondeterminism
finding) was the cause of Phase 13's 3/20->13/20 shift. If it stays
near 65% even in a continuous session, the shift is real and unexplained
by this hypothesis, and needs further investigation before anything
about it goes into the paper.

recover_time_s is re-measured here too (not assumed from Phase 13),
since this is also a legitimate independent check of that number under
matched conditions -- though Phase 13's recover_time_s was independently
user-verified against raw data already and is not itself in question.

Run: python3 continuous_session_retime.py
"""
import json, math, os, subprocess, time, random

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = os.path.dirname(__file__)
OUT = f"{LOG_DIR}/continuous_session_retime_results.json"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p17.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
N_PER_BUCKET = 50  # Phase 26: external review round item 3, n=50 for tighter Wilson CIs, corrected I_wheel (Phase 24)
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 350.0
RIGHTING_WAIT_TIMEOUT = 120.0

BUCKETS = [
    ("side_rest", 85.0, 95.0),
    ("full_inversion", 170.0, 180.0),
]


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
    log("  starting gz sim daemon (ONCE, no inter-bucket restart)...")
    gz_log = open(f"{LOG_DIR}/gz_p17_batch.log", 'a')
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
        super().__init__('p17_continuous_retime_monitor')
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


def run_one_trial(bucket_label, lo, hi, trial_idx, log):
    tilt_deg = random.uniform(lo, hi)
    az = random.uniform(0, 360)

    kill_scout1_nodes()
    quat = tilt_quaternion(tilt_deg, az)
    gz_respawn(0.0, 0.5, SPAWN_Z, quat)
    launch_scout1_nodes(bucket_label, trial_idx)

    rclpy.init()
    node = TrialMonitor()
    t0 = time.time()
    while node.uz is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    start_uz = node.uz
    log(f"  [{bucket_label} trial{trial_idx}/{N_PER_BUCKET}] commanded_tilt={tilt_deg:.1f} "
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
        log(f"    outcome={outcome} final_uz={final_uz} "
            f"righting_started_at={node.righting_started_at} recover_time_s={recover_time_s}")

    node.destroy_node()
    rclpy.shutdown()

    return {
        "bucket": bucket_label, "trial": trial_idx, "commanded_tilt_deg": tilt_deg,
        "azimuth_deg": az, "start_uz": start_uz, "landed": landed_flag,
        "final_uz": final_uz, "outcome": outcome,
        "righting_started_at": node.righting_started_at, "recover_time_s": recover_time_s,
    }


def wilson_ci(k, n, z=1.959963985):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world(log)
    make_bridge_yaml()

    results = []
    for bucket_label, lo, hi in BUCKETS:
        log(f"=== bucket={bucket_label} ({lo}-{hi}deg), n={N_PER_BUCKET}, "
            f"baseline (unperturbed), SINGLE CONTINUOUS SESSION ===")
        for i in range(N_PER_BUCKET):
            r = run_one_trial(bucket_label, lo, hi, i + 1, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)
        # NO kill_all()/start_world() here -- this is the whole point of
        # Phase 17: matching Phase 7's single continuous gz-sim session
        # across both buckets, unlike Phase 13's inter-bucket restart.

    log("=== continuous-session retime batch complete ===")
    for bucket_label, _, _ in BUCKETS:
        bucket_results = [r for r in results if r["bucket"] == bucket_label]
        n = len(bucket_results)
        n_rec = sum(1 for r in bucket_results if r["outcome"] == "recovered")
        lo_ci, hi_ci = wilson_ci(n_rec, n)
        times = [r["recover_time_s"] for r in bucket_results if r["recover_time_s"] is not None]
        n_no_signal = sum(1 for r in bucket_results
                           if r["outcome"] == "recovered" and r["righting_started_at"] is None)
        log(f"{bucket_label}: {n_rec}/{n} recovered ({n_rec/n:.1%}), 95% CI [{lo_ci:.1%}, {hi_ci:.1%}]")
        if times:
            mean_t = sum(times) / len(times)
            std_t = math.sqrt(sum((t - mean_t) ** 2 for t in times) / len(times)) if len(times) > 1 else 0.0
            times_sorted = sorted(times)
            med_t = times_sorted[len(times_sorted)//2]
            log(f"  recover_time_s (RE-TIMED): n={len(times)} mean={mean_t:.2f} std={std_t:.2f} "
                f"median={med_t:.2f} min={min(times):.2f} max={max(times):.2f}")
        if n_no_signal:
            log(f"  WARNING: {n_no_signal} recovered trial(s) never saw righting_active=True")


if __name__ == '__main__':
    main()
