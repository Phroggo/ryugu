#!/usr/bin/env python3
"""Phase 10, item 1: re-run of Phase 8's Priority 2 (n=100 launch delivery
@ 9.0m), against the Phase 9-fixed hopper_locomotion.py/landing_controller.py.
Required because Phase 9's verification (n=5 @ 9.0m) showed a real ratio
shift (0.212 -> 0.218) outside Phase 8's own n=100 std band (~0.0015-0.0019)
-- the pre-fix n=100 result no longer characterizes the current code.
Identical methodology to
phase8_overnight_batch/launch_delivery_batch_n100.py (same
STABILIZE_WINDOW=75s, periodic daemon restart every 5 reps, 95% CI).

Run: python3 launch_delivery_batch_n100_postfix.py
"""
import json, math, os, re, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p10n100.yaml"
WORLD_FILE = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"

DISTANCE = 9.0
G = 1.14e-4
SIN2TH = 0.56
V_REQ = math.sqrt(DISTANCE * G / SIN2TH)
N_REPEATS = 100
DAEMON_RESTART_EVERY = 5

READY_TIMEOUT = 60.0
SEPARATION_TIMEOUT = 200.0
STABILIZE_WINDOW = 75.0
SAMPLE_PERIOD = 2.0
OUT = f"{LOG_DIR}/launch_delivery_n100_postfix_results.json"

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
        super().__init__('phase10_launch_n100')
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
    gz_log = open(f"{LOG_DIR}/gz_batch10_n100.log", 'a')
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
        logf = open(f"{LOG_DIR}/{name}_n100pf_rep{rep}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(5)


def run_one_repeat(rep, log):
    spawn_and_launch_nodes(rep)

    rclpy.init()
    node = Calib()

    t0 = time.time()
    while time.time() - t0 < READY_TIMEOUT:
        node.spin_for(0.2)
        if node.landed is True and node.speed is not None and node.speed < 0.02:
            break
    log(f"  [rep{rep}] ready check done: landed={node.landed}, speed={node.speed}")

    node.pub_dist.publish(Float64(data=DISTANCE))
    time.sleep(0.2)
    node.pub_dist.publish(Float64(data=DISTANCE))

    node.separated = False
    t0 = time.time()
    while time.time() - t0 < SEPARATION_TIMEOUT and not node.separated:
        node.spin_for(0.2)

    if not node.separated:
        log(f"  [rep{rep}] TIMEOUT waiting for confirmed separation ({SEPARATION_TIMEOUT}s)")
        node.destroy_node()
        rclpy.shutdown()
        return {"rep": rep, "distance": DISTANCE, "v_req": V_REQ, "status": "no_separation"}

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
        log(f"  [rep{rep}] STABILIZED: delivered={delivered:.5f} "
            f"ratio={delivered/V_REQ:.3f} t={stabilize_time:.1f}s")
        return {"rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "delivered": delivered, "ratio": delivered / V_REQ,
                "status": "stabilized", "stabilize_time_s": stabilize_time,
                "n_samples": len(samples)}
    else:
        log(f"  [rep{rep}] separated but never stabilized within {STABILIZE_WINDOW}s")
        return {"rep": rep, "distance": DISTANCE, "v_req": V_REQ,
                "status": "separated_never_stabilized", "n_samples": len(samples)}


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    kill_all()
    start_world(log)

    results = []
    for rep in range(1, N_REPEATS + 1):
        if rep > 1 and (rep - 1) % DAEMON_RESTART_EVERY == 0:
            log(f"  --- periodic full daemon restart before rep {rep} ---")
            kill_all()
            start_world(log)
        log(f"--- repeat {rep}/{N_REPEATS} ---")
        r = run_one_repeat(rep, log)
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== batch complete ===")
    n_no_sep = sum(1 for r in results if r["status"] == "no_separation")
    n_stab = sum(1 for r in results if r["status"] == "stabilized")
    n_never_stab = sum(1 for r in results if r["status"] == "separated_never_stabilized")
    ratios = [r["ratio"] for r in results if r.get("status") == "stabilized"]
    log(f"n={N_REPEATS} no_separation={n_no_sep} ({100*n_no_sep/N_REPEATS:.0f}%) "
        f"stabilized={n_stab} separated_never_stabilized={n_never_stab}")
    if ratios:
        ratios_sorted = sorted(ratios)
        n = len(ratios)
        mean_r = sum(ratios) / n
        med_r = ratios_sorted[n // 2]
        var = sum((r - mean_r) ** 2 for r in ratios) / n
        std_r = math.sqrt(var)
        se = std_r / math.sqrt(n) if n > 0 else 0.0
        ci_lo, ci_hi = mean_r - 1.96 * se, mean_r + 1.96 * se
        log(f"ratios={[round(v,3) for v in ratios]}")
        log(f"n={n} mean={mean_r:.4f} median={med_r:.4f} std={std_r:.4f} "
            f"min={min(ratios):.4f} max={max(ratios):.4f} range={max(ratios)-min(ratios):.4f}")
        log(f"95% CI (normal approx) on mean: [{ci_lo:.4f}, {ci_hi:.4f}]")
        log(f"COMPARISON vs Phase 8 pre-fix n=100: mean=0.2121, CI=[0.2118, 0.2124]")

    # Phase 9's tick-based abort-warning path (and its TICK/WALL-TIME
    # MISMATCH marker) no longer exists post-fix -- scanning for it would
    # trivially "pass" without checking anything. Instead: directly scan
    # every loco log's real Crouching->IGNITION gap (should always be
    # >=~9s under the wall-clock CROUCH gate; a burst-corrupted gate would
    # show up here as it did pre-fix). n=100 makes this a much stronger
    # confirmation than Phase 9's own n=5 spot-check.
    log("--- scanning loco_scout_1_n100pf_rep*.log for Crouching->IGNITION timing ---")
    any_bad = False
    n_checked = 0
    for rep in range(1, N_REPEATS + 1):
        p = f"{LOG_DIR}/loco_scout_1_n100pf_rep{rep}.log"
        if not os.path.exists(p):
            continue
        crouch_t, ignition_t = None, None
        with open(p) as f:
            for line in f:
                m = re.search(r'\[(\d+\.\d+)\].*Crouching', line)
                if m:
                    crouch_t = float(m.group(1))
                m = re.search(r'\[(\d+\.\d+)\].*IGNITION', line)
                if m and crouch_t is not None and ignition_t is None:
                    ignition_t = float(m.group(1))
        if crouch_t is not None and ignition_t is not None:
            n_checked += 1
            gap = ignition_t - crouch_t
            if gap < 9.0:
                any_bad = True
                log(f"  rep{rep}: Crouching->IGNITION = {gap:.2f}s -- TOO FAST, GATE BYPASSED")
    log(f"  checked {n_checked}/{N_REPEATS} reps")
    log(f"  CROUCH GATE INTEGRITY: {'FAIL' if any_bad else 'PASS'}")


if __name__ == '__main__':
    main()
