#!/usr/bin/env python3
"""Phase 25, item 2 fix attempt: does iterative re-aiming (already-existing
mechanisms -- recompute heading toward the target from wherever the agent
actually landed, and re-hop, matching swarm_manager.py's corrective re-hop
pattern) recover reliable point-to-point delivery, given that the RAW
single-hop azimuth control itself has already been shown NOT to be a
fixable systematic bias (see az_bias_retrospective.py: the existing EMA
heading-bias correction cannot converge on Phase 10's real scatter data --
mean_abs_err stays ~60-80deg and does not improve trial-over-trial).

Phase 8/10's directional_hop_validation tests measure ONE isolated hop's
accuracy, with none of the system's actual closed-loop correction
mechanisms exercised. Every real multi-agent mission test in this project
(Phases 7/13/17/21) DOES use iterative re-aiming (swarm_manager's
corrective re-hop, which recomputes heading toward the target from the
agent's current real position after every hop) and does complete real
point-to-point tasks. This test asks directly: for a single isolated
agent given ONE fixed target 5m away (same scenario as Phase 10, for
direct comparability), does repeating "hop toward wherever the target
still is, from wherever I am now" converge to within a tight tolerance
within a bounded retry budget -- i.e., does the mission-level mechanism
already compensate for the single-hop weakness, even though that
weakness itself is not directly fixable (per the retrospective test)?

ARRIVAL_TOLERANCE = 1.0m (tighter than swarm_manager's own 4.0m
ARRIVAL_RADIUS, which is sized for drill-arm reach, not a meaningful
"did directional correction actually work" criterion at a 5m target
distance -- 4.0m would trivially pass almost any real displacement).
MAX_REHOPS = 5, matching swarm_manager's own MAX_HOP_RETRIES.

Run: python3 iterative_corrective_hop_test.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry

LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p25iter.yaml"
WORLD_FILE = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"
OUT = f"{LOG_DIR}/iterative_corrective_hop_results.json"

TARGET_X, TARGET_Y = 5.0 * math.cos(math.radians(-55.0)), 5.0 * math.sin(math.radians(-55.0))
N_REPEATS = 10
MAX_REHOPS = 5
ARRIVAL_TOLERANCE = 1.0   # m; tighter than swarm_manager's 4.0m drill-reach ARRIVAL_RADIUS

READY_TIMEOUT = 60.0
YAW_ALIGN_WAIT = 20.0
YAW_ALIGN_THRESHOLD = 0.15
SEPARATION_TIMEOUT = 200.0
MAX_FLIGHT_WAIT = 1400.0


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
        super().__init__('phase25_iterative_dirhop')
        self.landed = None
        self.speed = None
        self.separated = False
        self.attitude_error = None
        self.x = None
        self.y = None
        self.yaw_pub = self.create_publisher(Float64, '/scout_1/target_yaw', 10)
        self.dist_pub = self.create_publisher(Float64, '/scout_1/jump_target_distance', 10)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/separation', self.sep_cb, 10)
        self.create_subscription(Float64, '/scout_1/attitude_error', self.att_err_cb, 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

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
    for pat in ['bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1', 'gz sim']:
        subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)
    time.sleep(2)


def start_world(log):
    log("  (re)starting gz sim daemon...")
    gz_log = open(f"{LOG_DIR}/gz_iterhop.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD_FILE],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def spawn_and_launch_nodes(rep):
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
        logf = open(f"{LOG_DIR}/{name}_iterhop_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def do_one_hop(node, hop_idx, cur_x, cur_y, log):
    """Command a hop from (cur_x,cur_y) toward the fixed target, wait for
    confirmed separation, then wait for landing. Returns (status,
    end_x, end_y, flight_time_s)."""
    dx, dy = TARGET_X - cur_x, TARGET_Y - cur_y
    dist_to_target = math.hypot(dx, dy)
    yaw_rad = math.atan2(dy, dx)

    node.attitude_error = None
    node.yaw_pub.publish(Float64(data=yaw_rad))
    t0 = time.time()
    while time.time() - t0 < YAW_ALIGN_WAIT:
        node.spin_for(0.2)
        if node.attitude_error is not None and abs(node.attitude_error) < YAW_ALIGN_THRESHOLD:
            break
    yaw_error = node.attitude_error
    log(f"    [hop{hop_idx}] from ({cur_x:.2f},{cur_y:.2f}), dist_to_target={dist_to_target:.2f}m, "
        f"yaw_error_at_ignition={yaw_error}")

    node.separated = False
    node.dist_pub.publish(Float64(data=dist_to_target))
    time.sleep(0.2)
    node.dist_pub.publish(Float64(data=dist_to_target))

    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)
    if not node.separated:
        log(f"    [hop{hop_idx}] TIMEOUT waiting for confirmed separation")
        return "no_separation", cur_x, cur_y, None

    node.landed = False
    flight_t0 = time.time()
    while time.time() - flight_t0 < MAX_FLIGHT_WAIT and node.landed is not True:
        node.spin_for(0.3)
    flight_time = time.time() - flight_t0
    if node.landed is not True:
        log(f"    [hop{hop_idx}] never landed within {MAX_FLIGHT_WAIT}s")
        return "never_landed", cur_x, cur_y, flight_time

    log(f"    [hop{hop_idx}] LANDED at ({node.x:.2f},{node.y:.2f}) after {flight_time:.1f}s")
    return "landed", node.x, node.y, flight_time


def run_one_repeat(rep, log):
    spawn_and_launch_nodes(rep)
    rclpy.init()
    node = HopTest()

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            break
    log(f"  [rep{rep}] ready check done: landed={node.landed}, speed={node.speed}")

    cur_x, cur_y = (node.x, node.y) if node.x is not None else (0.0, 0.5)
    hops = []
    final_status = None
    for hop_idx in range(1, MAX_REHOPS + 1):
        status, cur_x, cur_y, flight_time = do_one_hop(node, hop_idx, cur_x, cur_y, log)
        dist_remaining = math.hypot(TARGET_X - cur_x, TARGET_Y - cur_y)
        hops.append({"hop": hop_idx, "status": status, "x": cur_x, "y": cur_y,
                      "flight_time_s": flight_time, "dist_remaining_m": dist_remaining})
        if status != "landed":
            final_status = status
            break
        if dist_remaining <= ARRIVAL_TOLERANCE:
            final_status = "arrived"
            log(f"  [rep{rep}] ARRIVED within {ARRIVAL_TOLERANCE}m after {hop_idx} hop(s), "
                f"dist_remaining={dist_remaining:.2f}m")
            break
    else:
        final_status = "budget_exhausted"
        log(f"  [rep{rep}] budget exhausted after {MAX_REHOPS} hops, "
            f"dist_remaining={hops[-1]['dist_remaining_m']:.2f}m")

    node.destroy_node()
    rclpy.shutdown()
    return {"rep": rep, "target": [TARGET_X, TARGET_Y], "final_status": final_status,
            "n_hops": len(hops), "hops": hops,
            "final_dist_remaining_m": hops[-1]["dist_remaining_m"] if hops else None}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world(log)
    make_bridge_yaml()

    results = []
    log(f"=== iterative corrective-hop test: n={N_REPEATS}, target=({TARGET_X:.2f},{TARGET_Y:.2f}) "
        f"[5.0m @ -55deg heading, same scenario as Phase 10], MAX_REHOPS={MAX_REHOPS}, "
        f"ARRIVAL_TOLERANCE={ARRIVAL_TOLERANCE}m ===")
    for rep in range(1, N_REPEATS + 1):
        r = run_one_repeat(rep, log)
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)
        if rep % 3 == 0:
            log("  --- periodic daemon restart ---")
            kill_all()
            start_world(log)

    log("=== iterative corrective-hop test complete ===")
    n = len(results)
    n_arrived = sum(1 for r in results if r["final_status"] == "arrived")
    n_exhausted = sum(1 for r in results if r["final_status"] == "budget_exhausted")
    n_failed = n - n_arrived - n_exhausted
    log(f"n={n}: arrived={n_arrived} ({n_arrived/n:.0%}), budget_exhausted={n_exhausted}, "
        f"other_failure={n_failed}")
    hop_counts = [r["n_hops"] for r in results if r["final_status"] == "arrived"]
    if hop_counts:
        log(f"  hops-to-arrival (arrived only): n={len(hop_counts)} "
            f"mean={sum(hop_counts)/len(hop_counts):.1f} min={min(hop_counts)} max={max(hop_counts)}")
    remaining = [r["final_dist_remaining_m"] for r in results if r["final_dist_remaining_m"] is not None]
    if remaining:
        log(f"  final dist_remaining (all): mean={sum(remaining)/len(remaining):.2f}m "
            f"min={min(remaining):.2f}m max={max(remaining):.2f}m")


if __name__ == '__main__':
    main()
