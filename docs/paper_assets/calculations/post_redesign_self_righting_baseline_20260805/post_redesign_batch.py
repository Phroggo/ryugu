#!/usr/bin/env python3
"""Post-redesign self-righting batch: same exact methodology as the C15/C16
pre-redesign rerun (21 trials, uniform-random tilt 20-180 deg, fresh nodes
per trial, SPAWN_Z=5.2 terrain-clearance fix), but against the CURRENT,
shipped landing_controller.py (no swap -- this is the actual production
code), for a clean apples-to-apples comparison against the pre-redesign
1/21 (4.8%) result."""
import json, math, os, subprocess, time, random

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/post_redesign_batch/results.json"
NODE_LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/post_redesign_batch"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_pr.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/install/ryugu_sim/share/ryugu_sim/worlds/ryugu.sdf'
N_TRIALS = 21
SUCCESS_UZ = 0.9
SPAWN_Z = 5.2
LANDED_WAIT_TIMEOUT = 200.0
RIGHTING_WAIT_TIMEOUT = 120.0  # long enough for an initial 5-attempt failure
                                # PLUS a follow-up "tilted while LANDED"
                                # re-trigger to resolve, per the isolated
                                # diagnostic (fail-then-succeed took ~46s)


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


def launch_scout1_nodes(trial_idx):
    # Only bridge + landing_controller -- confirmed via isolated diagnostic
    # that running hopper_locomotion/attitude_controller alongside the
    # swapped-in pre-redesign landing_controller prevents landing detection
    # from ever completing (root cause not fully diagnosed; landing_controller
    # alone works reliably and is all this test actually needs). Applies
    # equally here against current shipped code for the same apples-to-
    # apples reason.
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{NODE_LOG_DIR}/{name}_pr_trial{trial_idx}.log", 'w')
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
        super().__init__('c1516_monitor')
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


def main():
    results = []

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    # "gz sim <args>" re-execs into children literally named "gz sim server"
    # / "gz sim gui" -- a launch-command-line-specific pkill pattern misses
    # those (found live 2026-08-05 during the severe-tilt rerun: leftover
    # instances survived and ran concurrently, colliding on the default
    # topic namespace). Match on "gz sim" alone.
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)

    log("Starting gz sim...")
    gz_log = open(f"{NODE_LOG_DIR}/gz_sim.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)

    make_bridge_yaml()

    for i in range(N_TRIALS):
        tilt_deg = random.uniform(20, 180)
        az = random.uniform(0, 360)
        quat = tilt_quaternion(tilt_deg, az)
        log(f"--- trial {i+1}/{N_TRIALS}: commanded_tilt={tilt_deg:.0f} deg az={az:.0f} ---")

        kill_scout1_nodes()
        gz_respawn(0.0, 0.5, SPAWN_Z, quat)
        launch_scout1_nodes(i + 1)

        rclpy.init()
        node = TrialMonitor()
        t0 = time.time()
        while node.uz is None and time.time() - t0 < 10.0:
            node.spin_for(0.2)
        start_uz = node.uz
        log(f"start uz={start_uz}")

        land_t0 = time.time()
        while time.time() - land_t0 < LANDED_WAIT_TIMEOUT and node.landed is not True:
            node.spin_for(0.3)
        log(f"landed={node.landed} after {time.time()-land_t0:.1f}s uz={node.uz}")

        outcome = "no_landing"
        final_uz = node.uz
        if node.landed is True:
            right_t0 = time.time()
            recovered = False
            while time.time() - right_t0 < RIGHTING_WAIT_TIMEOUT:
                node.spin_for(0.2)
                if node.uz is not None and node.uz > SUCCESS_UZ:
                    recovered = True
                    break
            final_uz = node.uz
            outcome = "recovered" if recovered else "failed"
            log(f"outcome={outcome} final_uz={final_uz}")

        results.append({
            "trial": i + 1, "commanded_tilt_deg": tilt_deg, "azimuth_deg": az,
            "start_uz": start_uz, "landed": node.landed, "final_uz": final_uz,
            "outcome": outcome,
        })
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

        node.destroy_node()
        rclpy.shutdown()

    n_recovered = sum(1 for r in results if r["outcome"] == "recovered")
    log(f"=== batch complete: {n_recovered}/{N_TRIALS} recovered ===")


if __name__ == '__main__':
    main()
