#!/usr/bin/env python3
"""Phase 5 follow-up: diagnose why running attitude_controller alongside
landing_controller prevented settle detection entirely (both trials stuck
in IDLE for 200s+ in the prior session's attempt).

Hypothesis: attitude_controller's SLEEP-DEFEAT ROTOR (imu_callback, "z_cmd
= IDLE_ROTOR_SPEED if z_cmd >= 0 else -IDLE_ROTOR_SPEED") has no
hysteresis around zero. If the underlying self.cmd_vel['z'] (unfloored)
sits near zero with sign noise from real PD control on tiny sensor-level
yaw error/rate, the PUBLISHED z command can flip between +IDLE_ROTOR_SPEED
and -IDLE_ROTOR_SPEED repeatedly -- each flip is a real wheel-direction
reversal, a real reaction-torque kick into the body (unlike holding one
constant sign, which is genuinely zero-torque after the initial ramp).
Repeated kicks would keep velocity_mag above landing_controller's tight
REST_VEL_MAX=0.005 m/s threshold indefinitely.

This script just spawns upright with both nodes and watches the actual
published /scout_1/rw_z_joint_cmd_vel value over time, alongside body
velocity_mag, to confirm or rule this out directly -- no fix applied yet.

Run: python3 diagnose_settle_failure.py
"""
import math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_diag.yaml"
SPAWN_Z = 5.2
OBSERVE = 280.0  # extended: first 60s attempt showed clean monotonic
                  # free-fall (v=g*t almost exactly), never reaching
                  # contact -- need to watch past the ~142-200s this
                  # height took to settle in the single-node runs

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool


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
    with open(BRIDGE_YAML, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')


class Monitor(Node):
    def __init__(self):
        super().__init__('phase5_diag_monitor')
        self.velocity_mag = None
        self.z_cmd = None
        self.z_cmd_history = []
        self.flip_count = 0
        self._last_sign = None
        self.landed = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        # Sniff the ACTUAL published command on the ROS side (before the
        # gz bridge) -- this is what attitude_controller (or anyone else)
        # publishes to /scout_1/rw_z_joint_cmd_vel.
        self.create_subscription(Float64, '/scout_1/rw_z_joint_cmd_vel', self.zcmd_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)

    def landed_cb(self, msg):
        self.landed = msg.data

    def odom_cb(self, msg):
        v = msg.twist.twist.linear
        self.velocity_mag = math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def zcmd_cb(self, msg):
        self.z_cmd = msg.data
        sign = 1 if msg.data > 0 else (-1 if msg.data < 0 else 0)
        if self._last_sign is not None and sign != 0 and sign != self._last_sign:
            self.flip_count += 1
        if sign != 0:
            self._last_sign = sign
        self.z_cmd_history.append((time.time(), msg.data))

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    subprocess.run(['pkill', '-9', '-f', 'bridge_scout_1|landing_scout_1|attitude_scout_1'],
                    capture_output=True)
    time.sleep(1.5)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     f"pose {{ position {{ x: 0 y: 0.5 z: {SPAWN_Z} }} }}"],
                    capture_output=True)
    time.sleep(2)

    make_bridge_yaml()
    for name, cmd in [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
        ('attitude_scout_1', ['ros2', 'run', 'ryugu_sim', 'attitude_controller', 'scout_1',
         '--ros-args', '-r', '__node:=attitude_scout_1']),
    ]:
        logf = open(f"{LOG_DIR}/diag_{name}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    last_print = 0
    while time.time() - t0 < OBSERVE:
        node.spin_for(0.1)
        elapsed = time.time() - t0
        if elapsed - last_print >= 5.0:
            log(f"t={elapsed:.1f}s velocity_mag={node.velocity_mag} "
                f"z_cmd={node.z_cmd} flip_count_so_far={node.flip_count} "
                f"landed={node.landed}")
            last_print = elapsed

    log(f"\n=== DONE: total sign flips on rw_z_joint_cmd_vel over {OBSERVE}s = {node.flip_count} ===")
    # Print a sample of the raw command trace around the densest flip
    # region for direct inspection.
    log(f"Last 30 published z_cmd values:")
    for t, v in node.z_cmd_history[-30:]:
        log(f"  t={t - t0:.3f}  z_cmd={v}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
