#!/usr/bin/env python3
"""Build a real distribution instead of a single paired sample: 5 repeats
of the same 9.0m degraded-mode launch at each of the shipped 1ms timestep
and a 4x coarser 4ms timestep, to check whether the two timesteps'
distributions actually overlap (timestep is not a driver of the noise) or
are shifted apart (timestep IS a driver) -- something one sample each
cannot distinguish from luck. Same stabilization methodology as before
(3 consecutive 2s samples within 5% magnitude and 0.995 cosine similarity).
Fresh entity respawn between every repeat (not reused), matching the
same-conditions-each-time logic used elsewhere this week.
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/timestep_check_contact"
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_ctd.yaml"
DISTANCE = 9.0
G = 1.14e-4
SIN2TH = 0.56
V_REQ = math.sqrt(DISTANCE * G / SIN2TH)
READY_TIMEOUT = 120.0
SEPARATION_TIMEOUT = 120.0
STABILIZE_WINDOW = 90.0
SAMPLE_PERIOD = 2.0
N_REPEATS = 5
OUT = f"{LOG_DIR}/distribution_results.json"

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
        super().__init__('contact_timestep_dist')
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


def start_world(world_file):
    gz_log = open(f"{LOG_DIR}/gz_dist.log", 'a')
    subprocess.Popen(['gz', 'sim', '-r', '--headless-rendering', world_file],
                      stdout=gz_log, stderr=subprocess.STDOUT)
    time.sleep(8)


def spawn_and_launch_nodes(label, rep):
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

    subprocess.run(['pkill', '-9', '-f', 'loco_scout_1|attitude_scout_1|landing_scout_1'],
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
        logf = open(f"{LOG_DIR}/{name}_{label}_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def run_one_repeat(label, rep, log):
    spawn_and_launch_nodes(label, rep)

    rclpy.init()
    node = Calib()

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            break
    log(f"  [{label} rep{rep}] ready check done: landed={node.landed}, speed={node.speed}")

    node.pub_dist.publish(Float64(data=DISTANCE))
    time.sleep(0.2)
    node.pub_dist.publish(Float64(data=DISTANCE))

    node.separated = False
    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)

    if not node.separated:
        log(f"  [{label} rep{rep}] TIMEOUT waiting for separation")
        node.destroy_node()
        rclpy.shutdown()
        return {"label": label, "rep": rep, "status": "no_separation"}

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
                "delivered": delivered, "ratio": delivered / V_REQ,
                "status": "stabilized", "stabilize_time_s": stabilize_time,
                "n_samples": len(samples)}
    else:
        log(f"  [{label} rep{rep}] never stabilized")
        return {"label": label, "rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "status": "never_stabilized", "n_samples": len(samples)}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for label, world_file in [
        ("1ms", "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"),
        ("4ms", f"{LOG_DIR}/ryugu_4ms.sdf"),
    ]:
        log(f"=== starting {label} world, {N_REPEATS} repeats ===")
        kill_all()
        start_world(world_file)
        for rep in range(1, N_REPEATS + 1):
            log(f"--- {label} repeat {rep}/{N_REPEATS} ---")
            r = run_one_repeat(label, rep, log)
            results.append(r)
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)

    log("=== all repeats complete ===")
    for timestep_label in ["1ms", "4ms"]:
        vals = [r["ratio"] for r in results
                if r["label"] == timestep_label and r.get("status") == "stabilized"]
        if vals:
            mean = sum(vals) / len(vals)
            spread = max(vals) - min(vals)
            log(f"{timestep_label}: n={len(vals)} ratios={[round(v,3) for v in vals]} "
                f"mean={mean:.3f} range={spread:.3f} min={min(vals):.3f} max={max(vals):.3f}")
        else:
            log(f"{timestep_label}: no stabilized samples")


if __name__ == '__main__':
    main()
