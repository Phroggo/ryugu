#!/usr/bin/env python3
"""Phase 8, Priority 4: directional hop validation, replacing the
contradicted "4.3m at 1deg heading error" paper claim
(BASELINE_MANIFEST.md: C9 re-verification achieved ground-track azimuth
122.66deg against a held yaw of -55deg -- NOT -56deg as stated in the
overnight request; using the precise sourced value here).

n trials (as many as fit in the remaining overnight window -- this is
priority 4 of 4 queued batches, results are written incrementally so a
partial batch is still usable) at DISTANCE=5.0m, HEADING_DEG=-55.0,
full spawn+4-node lifecycle per trial (matching every other Phase 7/8
harness), using the proven measurement methodology from
launch_stance_reliability_tests_20260803/c9_directional_hop_harness_FINAL_working_version.py
(publish target_yaw, wait for yaw-hold convergence, command the hop,
track odometry (x,y,yaw) through separation to landing, compute
delivered ground displacement and travel azimuth from start/end (x,y)).

Optionally repeats at a second heading (45deg) afterward if time and n
allow, to check heading-independence, per the request.

Run: python3 directional_hop_validation.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p8dirhop.yaml"
WORLD_FILE = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"

DISTANCE = 5.0
HEADINGS_DEG = [-55.0, 45.0]   # primary heading first, second only if time allows
N_PER_HEADING = 30
DAEMON_RESTART_EVERY = 5

READY_TIMEOUT = 60.0
YAW_ALIGN_WAIT = 20.0          # generous vs. attitude_controller's yaw-hold PD settle
YAW_ALIGN_THRESHOLD = 0.15     # rad, matches hopper_locomotion's own CROUCH gate
SEPARATION_TIMEOUT = 200.0
MAX_FLIGHT_WAIT = 1400.0       # ~23min ceiling, matches the historical C9 precedent;
                                # actual flight times are expected to be much shorter
                                # given Phase 6-8's consistently low (~0.21) delivered
                                # ratio at similar distances, but kept generous
OUT = f"{LOG_DIR}/directional_hop_results.json"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry


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


class HopTest(Node):
    def __init__(self):
        super().__init__('phase8_dirhop')
        self.landed = None
        self.speed = None
        self.separated = False
        self.attitude_error = None
        self.x = None
        self.y = None
        self.yaw_deg = None
        self.start_xy = None
        self.yaw_pub = self.create_publisher(Float64, '/scout_1/target_yaw', 10)
        self.dist_pub = self.create_publisher(Float64, '/scout_1/jump_target_distance', 10)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/separation', self.sep_cb, 10)
        self.create_subscription(Float64, '/scout_1/attitude_error', self.att_err_cb, 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.x, self.y = p.x, p.y
        self.yaw_deg = math.degrees(yaw)
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)

    def landed_cb(self, msg):
        self.landed = msg.data

    def sep_cb(self, msg):
        if msg.data:
            self.separated = True

    def att_err_cb(self, msg):
        self.attitude_error = msg.data

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def kill_all():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world(log):
    log("  (re)starting gz sim daemon...")
    gz_log = open(f"{LOG_DIR}/gz_dirhop.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD_FILE],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def spawn_and_launch_nodes(heading_label, rep):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     "sdf_filename: 'model://spacehopper', name: 'scout_1', "
                     "pose { position { x: 0 y: 0.5 z: 6.0 } }"],
                    capture_output=True)
    time.sleep(2)

    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    time.sleep(1)
    make_bridge_yaml()
    for name, cmd in [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('loco_scout_1', ['ros2', 'run', 'ryugu_sim', 'hopper_locomotion', 'scout_1',
         '--ros-args', '-r', '__node:=loco_scout_1']),
        ('attitude_scout_1', ['ros2', 'run', 'ryugu_sim', 'attitude_controller', 'scout_1',
         '--ros-args', '-r', '__node:=attitude_scout_1']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]:
        logf = open(f"{LOG_DIR}/{name}_dirhop{heading_label}_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def run_one_repeat(heading_deg, heading_label, rep, log):
    spawn_and_launch_nodes(heading_label, rep)

    rclpy.init()
    node = HopTest()

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            break
    log(f"  [hd={heading_deg} rep{rep}] ready check done: landed={node.landed}, speed={node.speed}")

    yaw_rad = math.radians(heading_deg)
    node.yaw_pub.publish(Float64(data=yaw_rad))
    t0 = time.time()
    while time.time() - t0 < YAW_ALIGN_WAIT:
        node.spin_for(0.2)
        if node.attitude_error is not None and abs(node.attitude_error) < YAW_ALIGN_THRESHOLD:
            break
    yaw_error_at_ignition = node.attitude_error
    log(f"  [hd={heading_deg} rep{rep}] yaw align wait done: attitude_error={yaw_error_at_ignition}")

    node.start_xy = (node.x, node.y) if node.x is not None else node.start_xy
    node.separated = False
    node.dist_pub.publish(Float64(data=DISTANCE))
    time.sleep(0.2)
    node.dist_pub.publish(Float64(data=DISTANCE))

    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)

    if not node.separated:
        log(f"  [hd={heading_deg} rep{rep}] TIMEOUT waiting for confirmed separation")
        node.destroy_node()
        rclpy.shutdown()
        return {"heading_deg": heading_deg, "rep": rep, "distance": DISTANCE,
                "status": "no_separation", "yaw_error_at_ignition": yaw_error_at_ignition}

    log(f"  [hd={heading_deg} rep{rep}] separated, tracking flight...")
    node.landed = False
    flight_t0 = time.time()
    while time.time() - flight_t0 < MAX_FLIGHT_WAIT and node.landed is not True:
        node.spin_for(0.3)

    node.destroy_node()
    rclpy.shutdown()

    if node.landed is not True:
        log(f"  [hd={heading_deg} rep{rep}] never landed within {MAX_FLIGHT_WAIT}s -- discarding displacement")
        return {"heading_deg": heading_deg, "rep": rep, "distance": DISTANCE,
                "status": "never_landed", "yaw_error_at_ignition": yaw_error_at_ignition,
                "flight_time_s": time.time() - flight_t0}

    x0, y0 = node.start_xy
    dx, dy = node.x - x0, node.y - y0
    displacement = math.sqrt(dx*dx + dy*dy)
    azimuth = math.degrees(math.atan2(dy, dx))
    flight_time = time.time() - flight_t0
    log(f"  [hd={heading_deg} rep{rep}] LANDED: displacement={displacement:.3f}m "
        f"azimuth={azimuth:.1f}deg flight_time={flight_time:.1f}s")
    return {"heading_deg": heading_deg, "rep": rep, "distance": DISTANCE, "status": "landed",
            "yaw_error_at_ignition": yaw_error_at_ignition, "displacement_m": displacement,
            "azimuth_deg": azimuth, "flight_time_s": flight_time,
            "start_xy": [x0, y0], "end_xy": [node.x, node.y]}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for heading_deg in HEADINGS_DEG:
        heading_label = "m55" if heading_deg < 0 else "p45"
        log(f"=== heading={heading_deg}deg, up to n={N_PER_HEADING} ===")
        kill_all()
        start_world(log)
        for rep in range(1, N_PER_HEADING + 1):
            if rep > 1 and (rep - 1) % DAEMON_RESTART_EVERY == 0:
                log(f"  --- periodic full daemon restart before rep {rep} ---")
                kill_all()
                start_world(log)
            log(f"--- heading={heading_deg} repeat {rep}/{N_PER_HEADING} ---")
            r = run_one_repeat(heading_deg, heading_label, rep, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== directional hop validation complete (or ran out of overnight time) ===")
    for heading_deg in HEADINGS_DEG:
        bucket = [r for r in results if r["heading_deg"] == heading_deg and r["status"] == "landed"]
        if bucket:
            disps = [r["displacement_m"] for r in bucket]
            azs = [r["azimuth_deg"] for r in bucket]
            n = len(disps)
            mean_d = sum(disps) / n
            std_d = math.sqrt(sum((d - mean_d) ** 2 for d in disps) / n) if n > 1 else 0.0
            mean_a = sum(azs) / n
            std_a = math.sqrt(sum((a - mean_a) ** 2 for a in azs) / n) if n > 1 else 0.0
            log(f"heading={heading_deg}: n_landed={n} displacement mean={mean_d:.3f} std={std_d:.3f} "
                f"azimuth mean={mean_a:.1f} std={std_a:.1f}")
        else:
            log(f"heading={heading_deg}: 0 landed trials")


if __name__ == '__main__':
    main()
