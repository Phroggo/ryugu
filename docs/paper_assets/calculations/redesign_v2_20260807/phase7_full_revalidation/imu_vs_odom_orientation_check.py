#!/usr/bin/env python3
"""Phase 7 targeted diagnostic (not a batch): spawn the robot at a known,
static, severe tilt (175 deg, matching the full_inversion bucket) and
compare u_z computed from /scout_1/imu's orientation field (what
landing_controller._is_badly_tilted / _run_righting_sequence actually
use) against u_z computed from /scout_1/odometry's orientation field
(what hopper_locomotion.py, attitude_controller.py, and every Phase 6/7
test harness use) -- side by side, from spawn through settle.

Hypothesis under test: gz-sim's IMU orientation output may be relative to
the sensor's pose at spawn/initialization rather than true world-frame
orientation, so a robot spawned already inverted with little further
rotation would read IMU-u_z near +1 (looks upright) while odometry-u_z
correctly reads near -1 (genuinely inverted) -- explaining why
_is_badly_tilted (IMU-based) never fires for the full_inversion bucket.

No controllers launched (bridge only) -- this isolates the sensor
comparison from any control-law behavior.

Run: python3 imu_vs_odom_orientation_check.py
"""
import math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_imuodom.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'
TILT_DEG = 175.0
AZ_DEG = 40.0
SPAWN_Z = 5.2
TRACE_WINDOW = 90.0

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


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
    subprocess.run(['pkill', '-9', '-f', 'bridge_scout_1'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world():
    gz_log = open(f"{LOG_DIR}/gz_imu_odom_check.log", 'w')
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
        super().__init__('imu_vs_odom_check')
        self.imu_uz = None
        self.odom_uz = None
        self.create_subscription(Imu, '/scout_1/imu', self.imu_cb, 20)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)

    def imu_cb(self, msg):
        qx, qy = msg.orientation.x, msg.orientation.y
        self.imu_uz = 1.0 - 2.0 * (qx * qx + qy * qy)
        self._last_imu_cov0 = msg.orientation_covariance[0]

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.odom_uz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world()
    make_bridge_yaml()
    quat = tilt_quaternion(TILT_DEG, AZ_DEG)
    log(f"spawning at tilt={TILT_DEG} az={AZ_DEG} quat={quat}")
    gz_spawn(0.0, 0.5, SPAWN_Z, quat)

    logf = open(f"{LOG_DIR}/bridge_scout_1_imu_odom_check.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                       '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
                       '-p', f'config_file:={BRIDGE_YAML}'],
                      stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while (node.imu_uz is None or node.odom_uz is None) and time.time() - t0 < 15.0:
        node.spin_for(0.2)
    log(f"attached: imu_uz={node.imu_uz} odom_uz={node.odom_uz} "
        f"imu_orientation_covariance[0]={getattr(node, '_last_imu_cov0', None)}")

    t0 = time.time()
    last_sample = -1.0
    while time.time() - t0 < TRACE_WINDOW:
        node.spin_for(0.2)
        elapsed = time.time() - t0
        if elapsed - last_sample >= 2.0:
            log(f"t={elapsed:5.1f}s  imu_uz={node.imu_uz!s:>10}  odom_uz={node.odom_uz!s:>10}  "
                f"diff={None if node.imu_uz is None or node.odom_uz is None else round(node.imu_uz - node.odom_uz, 4)}")
            last_sample = elapsed

    node.destroy_node()
    rclpy.shutdown()
    log("=== done ===")


if __name__ == '__main__':
    main()
