#!/usr/bin/env python3
"""Phase 19: damping Pareto sweep, launch-velocity + settle-time leg.

Measures delivered launch velocity (same 9.0m degraded-mode methodology
as the friction/timestep sweeps) AND extracts settle time (how long from
spawn to landed=True + speed<0.02, i.e. the harness's own ready-check
loop) from the SAME trials -- no need for a separate settle-time
experiment, it falls out of the launch methodology for free. Bounce
energy (the third Pareto axis) is measured separately in
damping_bounce_sweep.py, since it needs a passive bounce trace rather
than a controlled launch.

n=3 reps per damping value (6 values: 0.005/0.02/0.05 current/0.08/0.12/0.15).

Run: python3 damping_launch_sweep.py
"""
import json, math, os, subprocess, time

PHASE19_DIR = os.path.dirname(__file__)
os.environ['GZ_SIM_RESOURCE_PATH'] = (
    '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models:' + f'{PHASE19_DIR}/variant_models')

LOG_DIR = PHASE19_DIR
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p19launch.yaml"
WORLD = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf'

DISTANCE = 9.0
G = 1.14e-4
SIN2TH = 0.56
V_REQ = math.sqrt(DISTANCE * G / SIN2TH)
# BUG FIX (2026-08-14): was 120.0 (doubled from the proven 60.0 used in
# every other launch harness this project, to give low-damping "pogo"
# cases more time to settle). Instead, the first run showed a genuine
# simulation numerical blowup during flight tracking (a nonphysical
# ~6000 m/s velocity spike) for reps that sat through the full extended
# wait -- reverted to the well-tested 60.0s. Low-damping configs already
# showed settle_time_s=None (never confirmed landed) even at 120s, so no
# real information is lost by the shorter window; they were never going
# to settle within either timeout ("endless pogo", per the pre-redesign
# damping-sweep commit's own finding).
READY_TIMEOUT = 60.0
SEPARATION_TIMEOUT = 200.0
STABILIZE_WINDOW = 75.0
SAMPLE_PERIOD = 2.0
N_REPEATS = 3
OUT = f"{LOG_DIR}/damping_launch_sweep_results.json"

CONFIGS = [
    ("c0.005", "model://spacehopper_damp0p005"),
    ("c0.02", "model://spacehopper_damp0p02"),
    ("c0.05_current", "model://spacehopper"),
    ("c0.08", "model://spacehopper_damp0p08"),
    ("c0.12", "model://spacehopper_damp0p12"),
    ("c0.15", "model://spacehopper_damp0p15"),
]

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


class Calib(Node):
    def __init__(self):
        super().__init__('p19_damping_launch')
        self.landed = None
        self.speed = None
        self.separated = False
        self._last_v = (0, 0, 0)
        self.pub_dist = self.create_publisher(Float64, '/scout_1/jump_target_distance', 10)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/separation', self.sep_cb, 10)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 10)

    def landed_cb(self, msg):
        self.landed = msg.data

    def sep_cb(self, msg):
        if msg.data:
            self.separated = True

    def odom_cb(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        self._last_v = (v.x, v.y, v.z)

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def cosine_sim(a, b):
    dot = sum(x*y for x, y in zip(a, b))
    na = math.sqrt(sum(x*x for x in a))
    nb = math.sqrt(sum(x*x for x in b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return dot / (na * nb)


def kill_all():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], capture_output=True)
    time.sleep(2)


def start_world(log):
    log("  (re)starting gz sim daemon...")
    gz_log = open(f"{LOG_DIR}/gz_damping_launch.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', WORLD],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def spawn_and_launch_nodes(model_uri, label, rep):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req',
                     f"sdf_filename: '{model_uri}', name: 'scout_1', "
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
        logf = open(f"{LOG_DIR}/{name}_launch_{label}_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def run_one_repeat(model_uri, label, rep, log):
    spawn_and_launch_nodes(model_uri, label, rep)

    rclpy.init()
    node = Calib()

    ready_t0 = time.time()
    settle_time_s = None
    while time.time() - ready_t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            settle_time_s = time.time() - ready_t0
            break
    log(f"  [{label} rep{rep}] ready check done: landed={node.landed}, speed={node.speed}, "
        f"settle_time_s={settle_time_s}")

    node.pub_dist.publish(Float64(data=DISTANCE))
    time.sleep(0.2)
    node.pub_dist.publish(Float64(data=DISTANCE))

    node.separated = False
    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)

    if not node.separated:
        log(f"  [{label} rep{rep}] TIMEOUT waiting for confirmed separation ({SEPARATION_TIMEOUT}s)")
        node.destroy_node()
        rclpy.shutdown()
        return {"label": label, "rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "settle_time_s": settle_time_s, "status": "no_separation"}

    samples = []
    t0 = time.time()
    stabilized = False
    delivered = None
    stabilize_time = None
    last_sample_t = 0
    while time.time() - t0 < STABILIZE_WINDOW:
        node.spin_for(0.1)
        if time.time() - last_sample_t >= SAMPLE_PERIOD:
            vx, vy, vz = node._last_v
            mag = math.sqrt(vx*vx + vy*vy + vz*vz)
            samples.append((vx, vy, vz, mag))
            last_sample_t = time.time()
            if len(samples) >= 3:
                last3 = samples[-3:]
                mags = [s[3] for s in last3]
                mag_ok = (max(mags) - min(mags)) / max(mags, default=1e-9) < 0.05 if max(mags) > 1e-9 else False
                cos_ok = all(cosine_sim(last3[i][:3], last3[i+1][:3]) > 0.995 for i in range(2))
                if mag_ok and cos_ok:
                    stabilized = True
                    delivered = sum(mags) / 3.0
                    stabilize_time = time.time() - t0
                    break

    node.destroy_node()
    rclpy.shutdown()

    if stabilized:
        log(f"  [{label} rep{rep}] STABILIZED: delivered={delivered:.5f} "
            f"ratio={delivered/V_REQ:.3f} t={stabilize_time:.1f}s")
        return {"label": label, "rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "settle_time_s": settle_time_s, "delivered": delivered,
                "ratio": delivered / V_REQ, "status": "stabilized",
                "stabilize_time_s": stabilize_time, "n_samples": len(samples)}
    else:
        log(f"  [{label} rep{rep}] separated but never stabilized within {STABILIZE_WINDOW}s")
        return {"label": label, "rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "settle_time_s": settle_time_s, "status": "separated_never_stabilized",
                "n_samples": len(samples)}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for label, model_uri in CONFIGS:
        log(f"=== starting {label} ({model_uri}), {N_REPEATS} repeats ===")
        kill_all()
        start_world(log)
        for rep in range(1, N_REPEATS + 1):
            log(f"--- {label} repeat {rep}/{N_REPEATS} ---")
            r = run_one_repeat(model_uri, label, rep, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== all repeats complete ===")
    for label, _ in CONFIGS:
        vals = [r["ratio"] for r in results
                if r["label"] == label and r.get("status") == "stabilized"]
        settle_vals = [r["settle_time_s"] for r in results
                        if r["label"] == label and r.get("settle_time_s") is not None]
        n_no_sep = sum(1 for r in results if r["label"] == label and r["status"] == "no_separation")
        if vals:
            mean = sum(vals) / len(vals)
            log(f"{label}: n={len(vals)} stabilized (of {N_REPEATS}, {n_no_sep} no_separation) "
                f"ratios={[round(v,3) for v in vals]} mean_ratio={mean:.4f}")
        else:
            log(f"{label}: no stabilized samples ({n_no_sep} no_separation)")
        if settle_vals:
            mean_s = sum(settle_vals) / len(settle_vals)
            log(f"  settle_time_s: n={len(settle_vals)} vals={[round(v,1) for v in settle_vals]} "
                f"mean={mean_s:.1f}")


if __name__ == '__main__':
    main()
