#!/usr/bin/env python3
"""Timestep-sensitivity check: rerun the C13 107-deg yaw-slew test (the
cleanest, most precisely-measured live-rerun result from this week) at two
different physics timesteps -- the shipped 1ms and a 4x coarser 4ms -- and
compare convergence time/behavior. If results are consistent, that's a
real, useful confirmation that this week's live-rerun conclusions aren't
an artifact of the specific timestep used throughout. Uses attitude_controller
alone (yaw-hold is "always active, including grounded" per its own code
comment), no need for the full hop/landing stack.
"""
import json, math, os, subprocess, sys, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/timestep_check"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_ts.yaml"
TARGET_YAW_DEG = 107.0

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


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
        super().__init__('timestep_check_monitor')
        self.yaw = None
        self.trace = []
        self.t0 = time.time()
        self.create_subscription(Odometry, '/scout_1/odometry', self.cb, 20)
        self.target_pub = self.create_publisher(Float64, '/scout_1/target_yaw', 10)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.trace.append((round(time.time() - self.t0, 3), math.degrees(self.yaw)))

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def run_one(label, world_file):
    print(f"=== {label} ({world_file}) ===", flush=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'bridge_scout_1|attitude_scout_1'], capture_output=True)
    time.sleep(2)

    gz_log = open(f"{LOG_DIR}/gz_{label}.log", 'w')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', world_file],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)

    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     "sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     "pose { position { x: 0 y: 0.5 z: 6.0 } }"],
                    capture_output=True)
    time.sleep(2)

    make_bridge_yaml()
    bridge_log = open(f"{LOG_DIR}/bridge_{label}.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                       '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
                       '-p', f'config_file:={BRIDGE_YAML}'],
                      stdout=bridge_log, stderr=subprocess.STDOUT)
    time.sleep(3)

    att_log = open(f"{LOG_DIR}/attitude_{label}.log", 'w')
    subprocess.Popen(['ros2', 'run', 'ryugu_sim', 'attitude_controller', 'scout_1',
                       '--ros-args', '-r', '__node:=attitude_scout_1'],
                      stdout=att_log, stderr=subprocess.STDOUT)
    time.sleep(3)

    rclpy.init()
    node = Monitor()
    t0 = time.time()
    while node.yaw is None and time.time() - t0 < 10.0:
        node.spin_for(0.2)

    node.target_pub.publish(Float64(data=math.radians(TARGET_YAW_DEG)))
    print(f"  commanded target_yaw={TARGET_YAW_DEG} deg", flush=True)

    converged_t = None
    t0 = time.time()
    while time.time() - t0 < 30.0:
        node.spin_for(0.2)
        if node.yaw is not None:
            err = abs(math.degrees(node.yaw) - TARGET_YAW_DEG)
            if err < 1.0 and converged_t is None:
                converged_t = time.time() - t0
    final_yaw = math.degrees(node.yaw) if node.yaw is not None else None
    print(f"  converged (<1 deg error) at t={converged_t}, final_yaw={final_yaw}", flush=True)

    trace = node.trace
    node.destroy_node()
    rclpy.shutdown()
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)

    return {"label": label, "world_file": world_file, "converged_t": converged_t,
            "final_yaw_deg": final_yaw, "n_samples": len(trace), "trace": trace}


def main():
    results = []
    results.append(run_one("1ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"))
    results.append(run_one("4ms", f"{LOG_DIR}/ryugu_4ms.sdf"))

    with open(f"{LOG_DIR}/timestep_results.json", 'w') as f:
        json.dump(results, f, indent=2)

    print("\n=== SUMMARY ===")
    for r in results:
        print(f"{r['label']}: converged_t={r['converged_t']}, final_yaw={r['final_yaw_deg']}")


if __name__ == '__main__':
    main()
