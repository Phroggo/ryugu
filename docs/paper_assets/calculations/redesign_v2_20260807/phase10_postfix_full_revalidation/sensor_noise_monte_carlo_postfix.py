#!/usr/bin/env python3
"""Phase 10, item 5: Priority 1 sensor-noise Monte Carlo, post Phase 9 fix.

Gaussian orientation noise injected on odometry (landing_controller.py's
odom_callback, ODOM_ORIENTATION_NOISE_STD env var -- see Phase 10's
landing_controller.py change), std=0.01 rad, applied to landing_scout_1's
process only. Noise math (_quat_mult / _random_small_rotation_quat)
sanity-checked standalone before this run: unit-norm preserved exactly,
mean angular deviation 0.00802 rad vs theoretical std*sqrt(2/pi)=0.00798 rad.

Same core methodology and bucket definitions as Phase 7's
self_righting_batch_3bucket.py (side_rest 85-95deg, moderate 45-60deg,
full_inversion 170-180deg), N_PER_BUCKET raised from 20 to 50 per the
current instruction ("n>=50 per tilt bucket"). Compared against the
Phase 7 post-orientation-fix, no-noise baseline:
  side_rest 10/20 (50%), moderate 20/20 (100%), full_inversion 3/20 (15%)
(confirmed by re-reading phase7_full_revalidation/self_righting_3bucket_results.json
directly, not from memory, before writing this harness).

Run: python3 sensor_noise_monte_carlo_postfix.py
"""
import json, math, os, subprocess, time, random

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = os.path.dirname(__file__)
OUT = f"{LOG_DIR}/sensor_noise_monte_carlo_postfix_results.json"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p10noise.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
N_PER_BUCKET = 50
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 350.0
RIGHTING_WAIT_TIMEOUT = 120.0
ODOM_NOISE_STD = 0.01

BASELINE = {  # Phase 7 post-orientation-fix, no-noise
    "side_rest": (10, 20),
    "moderate": (20, 20),
    "full_inversion": (3, 20),
}

BUCKETS = [
    ("side_rest", 85.0, 95.0),
    ("moderate", 45.0, 60.0),
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
    noisy_env = dict(os.environ)
    noisy_env['ODOM_ORIENTATION_NOISE_STD'] = str(ODOM_NOISE_STD)
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}'], os.environ),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1'], noisy_env),
    ]
    for name, cmd, env in specs:
        logf = open(f"{LOG_DIR}/{name}_{label}_trial{idx}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, env=env)
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
        super().__init__('p10_noise_sr_monitor')
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


def wilson_ci(k, n, z=1.959963985):
    """95% Wilson score interval for a binomial proportion."""
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

    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)

    log(f"Starting gz sim... (ODOM_ORIENTATION_NOISE_STD={ODOM_NOISE_STD} on landing_scout_1 only)")
    gz_log = open(f"{LOG_DIR}/gz_noise_batch.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)

    make_bridge_yaml()

    results = []
    for label, lo, hi in BUCKETS:
        log(f"=== bucket {label} ({lo}-{hi} deg), n={N_PER_BUCKET} ===")
        for i in range(N_PER_BUCKET):
            tilt_deg = random.uniform(lo, hi)
            az = random.uniform(0, 360)

            kill_scout1_nodes()
            quat = tilt_quaternion(tilt_deg, az)
            gz_respawn(0.0, 0.5, SPAWN_Z, quat)
            launch_scout1_nodes(label, i + 1)

            rclpy.init()
            node = TrialMonitor()
            t0 = time.time()
            while node.uz is None and time.time() - t0 < 10.0:
                node.spin_for(0.2)
            start_uz = node.uz
            log(f"  [{label} trial{i+1}/{N_PER_BUCKET}] commanded_tilt={tilt_deg:.1f} "
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
                right_t0 = time.time()
                recovered = False
                while time.time() - right_t0 < RIGHTING_WAIT_TIMEOUT:
                    node.spin_for(0.2)
                    if node.uz is not None and node.uz > SUCCESS_UZ:
                        recovered = True
                        recover_time_s = time.time() - right_t0
                        break
                final_uz = node.uz
                outcome = "recovered" if recovered else "failed"
                log(f"    outcome={outcome} final_uz={final_uz} recover_time_s={recover_time_s}")

            node.destroy_node()
            rclpy.shutdown()

            results.append({
                "bucket": label, "trial": i + 1, "commanded_tilt_deg": tilt_deg,
                "azimuth_deg": az, "start_uz": start_uz, "landed": landed_flag,
                "final_uz": final_uz, "outcome": outcome, "recover_time_s": recover_time_s,
                "odom_orientation_noise_std": ODOM_NOISE_STD,
            })
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== batch complete ===")
    for label, _, _ in BUCKETS:
        bucket_results = [r for r in results if r["bucket"] == label]
        n = len(bucket_results)
        n_rec = sum(1 for r in bucket_results if r["outcome"] == "recovered")
        n_failed = sum(1 for r in bucket_results if r["outcome"] == "failed")
        n_noland = sum(1 for r in bucket_results if r["outcome"] == "no_landing")
        lo_ci, hi_ci = wilson_ci(n_rec, n)
        base_k, base_n = BASELINE[label]
        base_p = base_k / base_n
        log(f"{label}: {n_rec}/{n} recovered ({n_rec/n:.1%}), 95% CI [{lo_ci:.1%}, {hi_ci:.1%}]; "
            f"{n_failed} failed, {n_noland} no_landing")
        log(f"  BASELINE (no noise, Phase 7 post-fix): {base_k}/{base_n} ({base_p:.1%}) -- "
            f"{'within CI (no significant shift)' if lo_ci <= base_p <= hi_ci else 'OUTSIDE CI -- FLAG'}")
        times = [r["recover_time_s"] for r in bucket_results if r["recover_time_s"] is not None]
        if times:
            times_sorted = sorted(times)
            mean_t = sum(times) / len(times)
            med_t = times_sorted[len(times_sorted)//2]
            log(f"  recover_time_s: n={len(times)} mean={mean_t:.1f} median={med_t:.1f} "
                f"min={min(times):.1f} max={max(times):.1f}")


if __name__ == '__main__':
    main()
