#!/usr/bin/env python3
"""Timestep-sensitivity check for the self-righting maneuver (contact +
friction + fast reaction-wheel/leg motion), as a counterpart to the
already-answered C13 yaw-slew spot check (../attitude_rerun_20260803),
which only exercised pure RW torque control with no ground contact during
the maneuver. This uses the CURRENT (as-shipped, uncommitted) landing_controller
state -- whatever it does, both timesteps see the identical code, so this
isolates timestep sensitivity, not correctness of the maneuver itself.

Single moderate tilt (60 deg, picked to reliably trigger a righting
attempt without landing in the harder near-full-inversion regime), one
run per timestep, u_z traced at 1s resolution for up to 180s.
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/timestep_check_righting"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_rtc.yaml"
TILT_DEG = 60.0
AZ_DEG = 30.0
SPAWN_Z = 5.2
TRACE_WINDOW = 180.0
SUCCESS_UZ = 0.9


def make_bridge_yaml():
    imu_gz_topic = '/world/ryugu_world/model/scout_1/link/base_link/sensor/imu_sensor/imu'
    entries = [
        ('/scout_1/imu', imu_gz_topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
        ('/scout_1/odometry', '/model/scout_1/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        ('/scout_1/landed', '/scout_1/landed', 'std_msgs/msg/Bool', 'gz.msgs.Boolean', 'NONE'),
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
            if dr == 'NONE':
                continue
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


def gz_spawn(world_file, x, y, z, quat):
    gz_log = open(f"{LOG_DIR}/gz_{'1ms' if '4ms' not in world_file else '4ms'}.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', world_file],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)
    qx, qy, qz, qw = quat
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
           f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)
    time.sleep(2)


class Monitor(Node):
    def __init__(self):
        super().__init__('righting_timestep_monitor')
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


def run_one(label, world_file):
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] [{label}] {msg}", flush=True)

    log(f"=== starting {world_file} ===")
    kill_all()
    quat = tilt_quaternion(TILT_DEG, AZ_DEG)
    gz_spawn(world_file, 0.0, 0.5, SPAWN_Z, quat)

    make_bridge_yaml()
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{LOG_DIR}/{name}_{label}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.uz is None and time.time() - t0 < 15.0:
        node.spin_for(0.2)
    log(f"attached: uz={node.uz}")

    trace = []
    t0 = time.time()
    last_sample = -1.0
    success_t = None
    while time.time() - t0 < TRACE_WINDOW:
        node.spin_for(0.2)
        elapsed = time.time() - t0
        if elapsed - last_sample >= 1.0:
            trace.append({"t": round(elapsed, 1), "uz": node.uz, "landed": node.landed})
            last_sample = elapsed
        if node.uz is not None and node.uz > SUCCESS_UZ and success_t is None:
            success_t = elapsed
            log(f"*** u_z crossed {SUCCESS_UZ} at t={elapsed:.1f}s ***")

    log(f"done. final uz={node.uz} landed={node.landed} success_t={success_t}")
    node.destroy_node()
    rclpy.shutdown()
    return {"label": label, "world_file": world_file, "tilt_deg": TILT_DEG,
            "az_deg": AZ_DEG, "success_t": success_t, "final_uz": node.uz,
            "final_landed": node.landed, "trace": trace}


def main():
    results = []
    results.append(run_one("1ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"))
    results.append(run_one("4ms",
        "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/timestep_check_contact/ryugu_4ms.sdf"))

    with open(f"{LOG_DIR}/righting_timestep_results.json", 'w') as f:
        json.dump(results, f, indent=2)

    print("\n=== SUMMARY ===")
    for r in results:
        print(f"{r['label']}: success_t={r['success_t']} final_uz={r['final_uz']} "
              f"final_landed={r['final_landed']}")


if __name__ == '__main__':
    main()
