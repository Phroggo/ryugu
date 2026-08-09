#!/usr/bin/env python3
"""Phase 7: single-trial verification (not a batch) that the odometry-
orientation fix to landing_controller.py actually makes the badly-tilted
trigger fire for a full-inversion spawn (175 deg), the exact scenario
that showed a 20/20 miss rate before the fix.

Run: python3 verify_orientation_fix_single_trial.py
"""
import math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_verifyfix.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
TILT_DEG = 175.0
AZ_DEG = 40.0
SPAWN_Z = 5.2
WAIT_WINDOW = 180.0

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
    subprocess.run(['pkill', '-9', '-f', 'bridge_scout_1|landing_scout_1'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world():
    gz_log = open(f"{LOG_DIR}/gz_verifyfix.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def gz_spawn(x, y, z, quat):
    qx, qy, qz, qw = quat
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
           f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


class Monitor(Node):
    def __init__(self):
        super().__init__('verify_fix_monitor')
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
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world()
    make_bridge_yaml()
    quat = tilt_quaternion(TILT_DEG, AZ_DEG)
    log(f"spawning at tilt={TILT_DEG} az={AZ_DEG}")
    gz_spawn(0.0, 0.5, SPAWN_Z, quat)

    for name, cmd in [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]:
        logf = open(f"{LOG_DIR}/{name}_verifyfix.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.uz is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)
    log(f"start uz={node.uz}")

    t0 = time.time()
    triggered = False
    while time.time() - t0 < WAIT_WINDOW:
        node.spin_for(0.2)
        elapsed = time.time() - t0
        # Cheap way to know righting triggered: watch for state to have
        # left IDLE/CONTACT into an actively-correcting regime -- easiest
        # externally-observable proxy is landed flipping True WITHOUT a
        # righting attempt would mean the bug is still present; check the
        # console log directly at the end instead, this loop just watches
        # uz evolve.
        if int(elapsed) % 10 == 0:
            pass

    log(f"done. final uz={node.uz} landed={node.landed}")
    node.destroy_node()
    rclpy.shutdown()

    # Check the console log directly for the trigger message.
    with open(f"{LOG_DIR}/landing_scout_1_verifyfix.log") as f:
        content = f.read()
    fired = "Settled badly tilted/inverted" in content
    log(f"=== RESULT: badly-tilted trigger fired = {fired} ===")
    if fired:
        for line in content.splitlines():
            if "badly tilted" in line or "RW righting attempt" in line or "RIGHTTRACE" in line:
                log(f"  {line}")


if __name__ == '__main__':
    main()
