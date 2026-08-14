#!/usr/bin/env python3
"""Phase 4: re-run the C13 107-deg yaw-slew spot check (same maneuver as
the original 106.03 deg result, ../../attitude_rerun_20260803/) and the
1ms-vs-4ms timestep comparison (../../timestep_sensitivity_20260805/), now
against the NEW model.sdf (Phase 2 corrected mass/inertia) and the NEW
attitude_controller.py gains (Phase 3: K_ang=0.0394, K_rate=0.0456).

Adapted directly from timestep_sensitivity_20260805/timestep_sensitivity.py
(same method, same target angle, same convergence criterion) -- only
LOG_DIR changed, so this is a true apples-to-apples rerun, not a new test
design. Uses attitude_controller alone (yaw-hold is "always active,
including grounded" per its own code comment), no need for the full
hop/landing stack.

Run:  python3 yaw_slew_revalidation.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_phase4.yaml"
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
        super().__init__('phase4_yaw_slew_monitor')
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

    # BUG FOUND AND FIXED (Phase 24 sanity check, 2026-08-14): the original
    # script published target_yaw immediately here, racing ROS2/DDS
    # discovery of attitude_controller's subscription -- confirmed via
    # attitude_4ms.log showing NO "New target yaw received" line at all
    # (vs. attitude_1ms.log showing it plainly), while gz_4ms.log/
    # bridge_4ms.log showed completely normal startup with no errors. Not
    # an I_wheel-correction instability -- a harness publish-before-
    # subscriber-ready race, more likely to hit the second sequential
    # trial given less settle time. Fixed by waiting for a discovered
    # subscriber before publishing, with a bounded timeout and a retry
    # publish as a belt-and-suspenders guard.
    sub_wait_t0 = time.time()
    while node.target_pub.get_subscription_count() == 0 and time.time() - sub_wait_t0 < 10.0:
        node.spin_for(0.2)
    if node.target_pub.get_subscription_count() == 0:
        print(f"  WARNING: no target_yaw subscriber discovered after 10s", flush=True)

    node.target_pub.publish(Float64(data=math.radians(TARGET_YAW_DEG)))
    time.sleep(0.5)
    node.target_pub.publish(Float64(data=math.radians(TARGET_YAW_DEG)))  # retry guard
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

    with open(f"{LOG_DIR}/phase4_yaw_slew_results.json", 'w') as f:
        json.dump(results, f, indent=2)

    print("\n=== SUMMARY ===")
    for r in results:
        print(f"{r['label']}: converged_t={r['converged_t']}, final_yaw={r['final_yaw_deg']}")

    print("\n=== Comparison against the ORIGINAL C13 result (106.03 deg, "
          "<1 deg by t+9.3s) and the OLD-model timestep check "
          "(1ms: 106.06deg/9.61s, 4ms: 106.15deg/8.70s) ===")


if __name__ == '__main__':
    main()
