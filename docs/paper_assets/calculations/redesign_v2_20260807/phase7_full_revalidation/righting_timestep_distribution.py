#!/usr/bin/env python3
"""Phase 7: self-righting timestep-sensitivity DISTRIBUTION (not the old
single-run-per-timestep spot check at
phase0_baseline_lockin/self_righting_timestep_check/righting_timestep_check.py).
5 repeats per timestep (1ms shipped world vs 4ms, reusing the validated
phase4_attitude_revalidation/ryugu_4ms.sdf, confirmed identical to the
current worlds/ryugu.sdf except max_step_size), same fixed moderate tilt
(60 deg) as the old spot check for continuity, fresh respawn + fresh nodes
per repeat, against the CURRENT shipped landing_controller.py (final
model, post Phase 5/6).

Run: python3 righting_timestep_distribution.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p7rtd.yaml"
TILT_DEG = 60.0
SPAWN_Z = 5.2
N_REPEATS = 5
LANDED_WAIT_TIMEOUT = 350.0    # widened from 200s -- same fix as
                                # self_righting_batch_3bucket.py: a full
                                # 5-attempt give-up cycle plus fall/settle
                                # time can exceed 200s
RIGHTING_WAIT_TIMEOUT = 120.0
SUCCESS_UZ = 0.9
OUT = f"{LOG_DIR}/righting_timestep_distribution_results.json"

WORLDS = [
    ("1ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"),
    ("4ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/"
             "redesign_v2_20260807/phase4_attitude_revalidation/ryugu_4ms.sdf"),
]


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


def tilt_quaternion(tilt_deg, azimuth_deg):
    half = math.radians(tilt_deg) / 2.0
    az = math.radians(azimuth_deg)
    s = math.sin(half)
    return (s * math.cos(az), s * math.sin(az), 0.0, math.cos(half))


def kill_all():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world(label, world_file):
    gz_log = open(f"{LOG_DIR}/gz_rtd_{label}.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', world_file],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


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


def launch_nodes(label, rep):
    subprocess.run(['pkill', '-9', '-f', 'loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    time.sleep(1)
    make_bridge_yaml()
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{LOG_DIR}/{name}_rtd_{label}_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)


class Monitor(Node):
    def __init__(self):
        super().__init__('p7_rtd_monitor')
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


def run_one_repeat(label, rep, log):
    az = (rep * 61.0) % 360.0  # deterministic-but-varied azimuth per repeat
    quat = tilt_quaternion(TILT_DEG, az)

    kill_all()
    subprocess.run(['pkill', '-9', '-f', 'loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    world_file = dict(WORLDS)[label]
    start_world(label, world_file)
    gz_respawn(0.0, 0.5, SPAWN_Z, quat)
    launch_nodes(label, rep)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.uz is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    start_uz = node.uz
    log(f"  [{label} rep{rep}] az={az:.0f} start_uz={start_uz}")

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

    return {"timestep": label, "rep": rep, "tilt_deg": TILT_DEG, "az_deg": az,
            "start_uz": start_uz, "landed": landed_flag, "final_uz": final_uz,
            "outcome": outcome, "recover_time_s": recover_time_s}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for label, world_file in WORLDS:
        log(f"=== timestep {label}, {N_REPEATS} repeats ===")
        for rep in range(1, N_REPEATS + 1):
            r = run_one_repeat(label, rep, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== all repeats complete ===")
    for label, _ in WORLDS:
        bucket = [r for r in results if r["timestep"] == label]
        n_rec = sum(1 for r in bucket if r["outcome"] == "recovered")
        times = [r["recover_time_s"] for r in bucket if r["recover_time_s"] is not None]
        log(f"{label}: {n_rec}/{len(bucket)} recovered "
            f"recover_times={[round(t,1) for t in times]}")


if __name__ == '__main__':
    main()
