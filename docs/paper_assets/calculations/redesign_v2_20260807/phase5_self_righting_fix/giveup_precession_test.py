#!/usr/bin/env python3
"""Phase 5 targeted test: does a self-righting give-up leave residual
z-axis (yaw) rotation unarrested, drifting into deeper inversion?

Spawns scout_1 at a severe/near-full-inversion tilt (low historical
success rate, per Phase 1's C15-C18 batches -- good odds of a genuine
give-up within the trial), runs bridge + landing_controller only (matches
the C15/C16 methodology -- confirmed that running the full node stack
alongside a swapped controller can prevent landing detection from
completing; landing_controller alone is sufficient and reliable), and
watches u_z, angular velocity (all 3 axes), and state (landed/
righting_active) for a long window covering: righting attempts (up to
5 x 15s = 75s worst case) + give-up + a further post-give-up observation
period, to see whether the body settles/damps or precesses into deeper
inversion.

Run with LABEL=BEFORE against the pre-Phase-5 code (git stash the Phase 5
diff first) and LABEL=AFTER against the fixed code, same tilt/azimuth
pairs both times, for a direct comparison.

Usage: python3 giveup_precession_test.py <label>
"""
import json, math, os, subprocess, sys, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_phase5.yaml"
SPAWN_Z = 5.2
OBSERVE_WINDOW = 200.0  # s: covers up to 75s of righting attempts + ~125s post-give-up

# Same 3 (tilt, azimuth) pairs for both BEFORE and AFTER -- severe/near-
# full inversion, historically low success rate (Phase 1 C15-C18: ~25%
# for full inversion), chosen to maximize odds of a genuine give-up.
TRIALS = [
    (165.0, 45.0),
    (150.0, 200.0),
    (172.0, 310.0),
]

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


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
    with open(BRIDGE_YAML, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')


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


def kill_scout1_nodes():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|landing_scout_1'], capture_output=True)
    time.sleep(1.5)


def launch_scout1_nodes(trial_idx, label):
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{LOG_DIR}/{name}_{label}_trial{trial_idx}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)


class TrialMonitor(Node):
    def __init__(self):
        super().__init__(f'phase5_giveup_monitor_{int(time.time())}')
        self.uz = None
        self.wz = None
        self.wx = None
        self.wy = None
        self.landed = None
        self.righting_active = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/righting_active', self.righting_cb, 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)
        av = msg.twist.twist.angular
        self.wx, self.wy, self.wz = av.x, av.y, av.z

    def landed_cb(self, msg):
        self.landed = msg.data

    def righting_cb(self, msg):
        self.righting_active = msg.data

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def run_trial(i, tilt_deg, az_deg, label, log):
    quat = tilt_quaternion(tilt_deg, az_deg)
    log(f"--- trial {i+1}/{len(TRIALS)} [{label}]: tilt={tilt_deg}deg az={az_deg}deg ---")

    kill_scout1_nodes()
    gz_respawn(0.0, 0.5, SPAWN_Z, quat)
    make_bridge_yaml()
    launch_scout1_nodes(i, label)

    rclpy.init()
    node = TrialMonitor()

    trace = []
    t0 = time.time()
    gave_up_t = None
    last_righting_active = None
    last_sample_t = -10
    while time.time() - t0 < OBSERVE_WINDOW:
        node.spin_for(0.2)
        elapsed = time.time() - t0
        if node.righting_active is False and last_righting_active is True and gave_up_t is None:
            # transition RIGHTING -> not-RIGHTING; only call it "gave up"
            # if u_z is still bad (a genuine recovery also does this)
            if node.uz is not None and node.uz < 0.85:
                gave_up_t = elapsed
                log(f"  [{label} trial{i+1}] give-up/settle-not-upright detected "
                    f"at t={elapsed:.1f}s, u_z={node.uz:.3f}")
        last_righting_active = node.righting_active
        if elapsed - last_sample_t >= 2.0:
            trace.append({"t": round(elapsed, 1), "uz": node.uz, "wx": node.wx,
                          "wy": node.wy, "wz": node.wz, "landed": node.landed,
                          "righting_active": node.righting_active})
            last_sample_t = elapsed

    final_uz = node.uz
    node.destroy_node()
    rclpy.shutdown()

    # Trend of u_z over the back half of the window (post-give-up-ish
    # region), to quantify "still precessing" vs "settled/stable".
    back_half = [p for p in trace if p['t'] > OBSERVE_WINDOW / 2 and p['uz'] is not None]
    uz_trend = None
    if len(back_half) >= 2:
        uz_trend = back_half[-1]['uz'] - back_half[0]['uz']

    log(f"  [{label} trial{i+1}] DONE: final_uz={final_uz}, gave_up_t={gave_up_t}, "
        f"back-half u_z trend={uz_trend}")

    return {"trial": i + 1, "label": label, "tilt_deg": tilt_deg, "az_deg": az_deg,
            "gave_up_t": gave_up_t, "final_uz": final_uz, "uz_trend_back_half": uz_trend,
            "trace": trace}


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "UNLABELED"

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for i, (tilt, az) in enumerate(TRIALS):
        r = run_trial(i, tilt, az, label, log)
        results.append(r)
        with open(f"{LOG_DIR}/giveup_precession_results_{label}.json", 'w') as f:
            json.dump(results, f, indent=2)

    log(f"\n=== SUMMARY [{label}] ===")
    for r in results:
        log(f"trial {r['trial']} (tilt={r['tilt_deg']}, az={r['az_deg']}): "
            f"gave_up_t={r['gave_up_t']}, final_uz={r['final_uz']}, "
            f"back-half u_z trend={r['uz_trend_back_half']}")


if __name__ == '__main__':
    main()
