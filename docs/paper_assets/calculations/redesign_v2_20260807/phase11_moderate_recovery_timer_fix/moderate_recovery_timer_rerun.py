#!/usr/bin/env python3
"""Phase 11: fix the self-righting recovery-time harness bug and rerun the
moderate-tilt bucket only.

Bug (caught by paper reviewer, confirmed against phase7_full_revalidation/
self_righting_batch_3bucket.py before this fix): the original harness
started its recovery timer (`right_t0 = time.time()`) only after its own
landed-detection poll loop (0.3s granularity, `node.spin_for(0.3)` inside
`while ... node.landed is not True`) confirmed `landed=True`. But
landing_controller.py's own righting maneuver can start -- and for fast
moderate-tilt cases, finish -- before that poll loop catches up, so the
recorded "recovery time" was measuring polling latency, not maneuver
duration, specifically for trials where recovery is fast (moderate
bucket: real maneuver duration is often sub-second, so the ~0.3s harness
latency IS the whole signal). Side_rest and full_inversion are not
affected -- real recovery there takes seconds, so the same latency is a
negligible fraction of the true duration -- and are NOT rerun here.

Fix: landing_controller.py already publishes `/{robot_name}/righting_active`
(std_msgs/Bool, updated every tick, going True the instant
`self.state = self.RIGHTING` fires -- see landing_controller.py lines
856/928/981). Subscribe to it directly and record the timestamp of its
first True transition (righting_started_at) as the recovery-timer origin,
instead of re-deriving a start time from the harness's own separate
landed-poll loop. The subscription is live from node creation, well
before landing occurs, so there is no risk of missing the transition.

Same core methodology, world, spawn parameters, and bucket definition as
phase7_full_revalidation/self_righting_batch_3bucket.py (moderate:
tilt 45-60deg, u_z ~0.5-0.7), n=20 to match the original Table IX entry
being corrected (not the expanded/randomized n>=100 study, which is a
separate, deliberately-held item).

Run: python3 moderate_recovery_timer_rerun.py
"""
import json, math, os, subprocess, time, random

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = os.path.dirname(__file__)
OUT = f"{LOG_DIR}/moderate_recovery_timer_rerun_results.json"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p11.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
N_TRIALS = 20
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 350.0
RIGHTING_WAIT_TIMEOUT = 120.0
BUCKET_LABEL = "moderate"
BUCKET_LO, BUCKET_HI = 45.0, 60.0


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
    # righting_active is not a gz-native topic -- it's published directly by
    # landing_controller.py on the ROS side, no bridge entry needed.
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


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('p11_sr_timer_fix_monitor')
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
        # Record the FIRST True transition only -- that's the real start
        # of the maneuver. Subsequent True messages (published every tick
        # while righting) must not reset the clock.
        if msg.data and self.righting_started_at is None:
            self.righting_started_at = time.time()

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)

    log("Starting gz sim...")
    gz_log = open(f"{LOG_DIR}/gz_p11_batch.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)

    make_bridge_yaml()

    results = []
    log(f"=== bucket {BUCKET_LABEL} ({BUCKET_LO}-{BUCKET_HI} deg), n={N_TRIALS}, "
        f"FIXED recovery timer (keyed off /righting_active) ===")
    for i in range(N_TRIALS):
        tilt_deg = random.uniform(BUCKET_LO, BUCKET_HI)
        az = random.uniform(0, 360)

        kill_scout1_nodes()
        quat = tilt_quaternion(tilt_deg, az)
        gz_respawn(0.0, 0.5, SPAWN_Z, quat)
        launch_scout1_nodes(BUCKET_LABEL, i + 1)

        rclpy.init()
        node = TrialMonitor()
        t0 = time.time()
        while node.uz is None and time.time() - t0 < 10.0:
            node.spin_for(0.2)
        start_uz = node.uz
        log(f"  [trial{i+1}/{N_TRIALS}] commanded_tilt={tilt_deg:.1f} "
            f"az={az:.0f} start_uz={start_uz}")

        # Keep polling for landed (still needed for outcome bookkeeping /
        # the no_landing case), but the recovery-timer origin now comes
        # from node.righting_started_at, set independently by righting_cb
        # the instant the controller itself enters RIGHTING -- not from
        # this poll loop's own cadence.
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
                f"righting_started_at={node.righting_started_at} "
                f"recover_time_s={recover_time_s}")

        node.destroy_node()
        rclpy.shutdown()

        results.append({
            "bucket": BUCKET_LABEL, "trial": i + 1, "commanded_tilt_deg": tilt_deg,
            "azimuth_deg": az, "start_uz": start_uz, "landed": landed_flag,
            "final_uz": final_uz, "outcome": outcome,
            "righting_started_at": node.righting_started_at,
            "recover_time_s": recover_time_s,
        })
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== batch complete ===")
    n_rec = sum(1 for r in results if r["outcome"] == "recovered")
    n_failed = sum(1 for r in results if r["outcome"] == "failed")
    n_noland = sum(1 for r in results if r["outcome"] == "no_landing")
    times = [r["recover_time_s"] for r in results if r["recover_time_s"] is not None]
    log(f"{BUCKET_LABEL}: {n_rec}/{len(results)} recovered, {n_failed} failed, "
        f"{n_noland} no_landing")
    if times:
        times_sorted = sorted(times)
        mean_t = sum(times) / len(times)
        med_t = times_sorted[len(times_sorted)//2]
        log(f"  recover_time_s (FIXED timer): n={len(times)} mean={mean_t:.3f} "
            f"median={med_t:.3f} min={min(times):.3f} max={max(times):.3f}")
    n_no_signal = sum(1 for r in results if r["outcome"] == "recovered"
                       and r["righting_started_at"] is None)
    if n_no_signal:
        log(f"  WARNING: {n_no_signal} recovered trial(s) never saw righting_active=True "
            f"-- recover_time_s left as None for those, flagged not silently dropped")


if __name__ == '__main__':
    main()
