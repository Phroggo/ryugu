#!/usr/bin/env python3
"""Quick isolated diagnostic: does gz-sim's simulated IMU orientation match
odometry orientation immediately after an instant teleport to a severe
tilt, or does it lag/diverge? Fully isolated from any other running sim
(unique GZ_PARTITION + ROS_DOMAIN_ID) so it can run alongside the C27 test
without any topic collision. No controllers -- just bridge + a monitor.
"""
import os, math, subprocess, time, json

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
os.environ['GZ_PARTITION'] = 'imu_diag_isolated'
os.environ['ROS_DOMAIN_ID'] = '77'
WORLD = '/home/melvin/ryugu_v2_ws/install/ryugu_sim/share/ryugu_sim/worlds/ryugu.sdf'
LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/imu_diag"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def make_bridge_yaml():
    imu_gz_topic = '/world/ryugu_world/model/scout_1/link/base_link/sensor/imu_sensor/imu'
    entries = [
        ('/scout_1/imu', imu_gz_topic, 'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
        ('/scout_1/odometry', '/model/scout_1/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
    ]
    path = f'{LOG_DIR}/bridge_diag.yaml'
    with open(path, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')
    return path


class Diag(Node):
    def __init__(self):
        super().__init__('imu_diag_monitor')
        self.rows = []
        self.t0 = time.time()
        self.create_subscription(Imu, '/scout_1/imu', self.imu_cb, 50)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 50)
        self.last_imu_uz = None
        self.last_odom_uz = None

    def imu_cb(self, msg):
        q = msg.orientation
        uz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.last_imu_uz = uz
        self._log()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        uz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.last_odom_uz = uz
        self._log()

    def _log(self):
        self.rows.append({
            't': round(time.time() - self.t0, 3),
            'imu_uz': self.last_imu_uz,
            'odom_uz': self.last_odom_uz,
        })


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    subprocess.run(['pkill', '-9', '-f', 'GZ_PARTITION=imu_diag_isolated'], capture_output=True)
    time.sleep(1)

    env = os.environ.copy()
    gz_log = open(f'{LOG_DIR}/gz_sim.log', 'w')
    gz_proc = subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                                stdout=gz_log, stderr=subprocess.STDOUT, env=env)
    print('gz sim starting...', flush=True)
    time.sleep(8)

    cfg = make_bridge_yaml()
    bridge_log = open(f'{LOG_DIR}/bridge.log', 'w')
    bridge_proc = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_diag', '--params-file', '/dev/null',
         '-p', f'config_file:={cfg}'],
        stdout=bridge_log, stderr=subprocess.STDOUT, env=env)
    time.sleep(4)

    # Spawn upright first (baseline sanity), then after a few seconds
    # instantly teleport to a severe tilt (172 deg) and watch both
    # orientation sources for the next several seconds.
    print('Spawning scout_1 upright...', flush=True)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     "sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     "pose { position { x: 0 y: 0.5 z: 6.0 } }"],
                    capture_output=True, env=env)

    rclpy.init()
    node = Diag()
    t_end = time.time() + 6.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    tilt_deg = 172.0
    half = math.radians(tilt_deg) / 2.0
    qx, qy, qz, qw = math.sin(half), 0.0, 0.0, math.cos(half)
    print(f'Teleporting to {tilt_deg} deg tilt instantly (respawn)...', flush=True)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True, env=env)
    time.sleep(1.5)
    teleport_t0 = time.time() - node.t0
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     f"pose {{ position {{ x: 0 y: 0.5 z: 6.0 }} "
                     f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}"],
                    capture_output=True, env=env)

    t_end = time.time() + 15.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    out = f'{LOG_DIR}/diag_results.json'
    with open(out, 'w') as f:
        json.dump({'teleport_t': teleport_t0, 'expected_uz_after_teleport': math.cos(math.radians(tilt_deg)),
                   'rows': node.rows}, f, indent=2)
    print(f'Done. Results: {out}', flush=True)

    node.destroy_node()
    rclpy.shutdown()
    bridge_proc.terminate()
    gz_proc.terminate()


if __name__ == '__main__':
    main()
