#!/usr/bin/env python3
"""Phase 6: V_GAIN re-calibration sweep against the corrected (Phase 2+)
mass model, using the NEW genuine-separation-confirmation launch state
machine (hopper_locomotion.py, 2026-08-08) instead of the old flat
"ramp_ticks + 5" timer.

One run per distance across a small spread of commanded ramp durations
(short ramp / long hop through long ramp / short hop). For each run:
  1. wait for a genuine ready stance (landed=True, quiescent)
  2. publish jump_target_distance
  3. wait for /scout_1/separation == True (now only fires once
     hopper_locomotion has itself confirmed several consecutive velocity
     samples agree in magnitude and direction -- i.e. genuinely off the
     ground, not dragging)
  4. sample velocity for a short post-separation window and confirm it is
     already stable (same 5%-magnitude / 0.995-cosine definition used
     throughout this project) -- expected to converge fast now, since the
     drag that used to corrupt this reading is exactly what the new
     separation gate filters out before FLIGHT begins.

ramp_T for each distance is computed the same way jump_target_callback
does, using the CURRENT (pre-recalibration) V_GAIN=0.12 -- this sweep's
job is to gather (ramp_T, delivered_v) pairs, not to guess the answer
first. New V_GAIN is fit afterward as mean(ramp_T_i * delivered_v_i),
matching the model's assumed form v = V_GAIN / T.

Run: python3 vgain_calibration_sweep.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_vgain6.yaml"
WORLD_FILE = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"

G = 1.14e-4
SIN2TH = 0.56
V_GAIN_OLD = 0.12  # current live value at calibration time, see hopper_locomotion.py
DISTANCES = [1.0, 3.0, 9.0, 20.0, 40.0]

READY_TIMEOUT = 60.0
SEPARATION_TIMEOUT = 200.0   # crouch cap 45s + ramp <=20s + sep-confirm wait <=60s + margin
STABILIZE_WINDOW = 30.0
SAMPLE_PERIOD = 2.0
OUT = f"{LOG_DIR}/vgain_calibration_results.json"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry


def v_req_for(distance):
    return math.sqrt(max(distance, 0.5) * G / SIN2TH)


def ramp_t_for(distance, v_gain):
    return max(1.2, min(20.0, v_gain / v_req_for(distance)))


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
        super().__init__('vgain6_calib')
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


def start_world():
    gz_log = open(f"{LOG_DIR}/gz_vgain6.log", 'a')
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
        logf = open(f"{LOG_DIR}/{name}_calib_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def run_one_distance(distance, rep, log):
    spawn_and_launch_nodes(rep)

    rclpy.init()
    node = Calib()

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            break
    log(f"  [d={distance}m] ready check done: landed={node.landed}, speed={node.speed}")

    v_req = v_req_for(distance)
    ramp_t = ramp_t_for(distance, V_GAIN_OLD)
    node.pub_dist.publish(Float64(data=distance))
    time.sleep(0.2)
    node.pub_dist.publish(Float64(data=distance))

    node.separated = False
    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)

    if not node.separated:
        log(f"  [d={distance}m] TIMEOUT waiting for confirmed separation ({SEPARATION_TIMEOUT}s)")
        node.destroy_node()
        rclpy.shutdown()
        return {"distance": distance, "v_req": v_req, "ramp_t": ramp_t, "status": "no_separation"}

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
        log(f"  [d={distance}m] STABILIZED: delivered={delivered:.5f} "
            f"ratio={delivered/v_req:.3f} ramp_t={ramp_t:.2f}s t={stabilize_time:.1f}s")
        return {"distance": distance, "v_req": v_req, "ramp_t": ramp_t,
                "delivered": delivered, "ratio": delivered / v_req,
                "status": "stabilized", "stabilize_time_s": stabilize_time,
                "n_samples": len(samples)}
    else:
        log(f"  [d={distance}m] separated but never stabilized within {STABILIZE_WINDOW}s")
        return {"distance": distance, "v_req": v_req, "ramp_t": ramp_t,
                "status": "separated_never_stabilized", "n_samples": len(samples)}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world()

    results = []
    for i, distance in enumerate(DISTANCES, start=1):
        log(f"=== distance {distance}m ({i}/{len(DISTANCES)}) ===")
        r = run_one_distance(distance, i, log)
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== sweep complete ===")
    pairs = [(r["ramp_t"], r["delivered"]) for r in results if r.get("status") == "stabilized"]
    log(f"stabilized: {len(pairs)}/{len(DISTANCES)}")
    for r in results:
        log(f"  d={r['distance']}m status={r['status']} "
            f"ramp_t={r.get('ramp_t'):.2f}s "
            f"delivered={r.get('delivered')} ratio={r.get('ratio')}")
    if pairs:
        v_gains = [t * v for t, v in pairs]
        new_v_gain = sum(v_gains) / len(v_gains)
        log(f"per-sample V_GAIN fits: {[round(g, 5) for g in v_gains]}")
        log(f"NEW V_GAIN (mean) = {new_v_gain:.5f} m  (old = {V_GAIN_OLD})")
    else:
        log("NO stabilized samples -- cannot fit V_GAIN")


if __name__ == '__main__':
    main()
