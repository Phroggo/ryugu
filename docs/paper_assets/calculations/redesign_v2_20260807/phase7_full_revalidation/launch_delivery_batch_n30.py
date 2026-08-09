#!/usr/bin/env python3
"""Phase 7: launch delivery batch, n>=30, full distribution (not a point
estimate), same 9.0m degraded-mode scenario as Phase 0/Phase 6. Adapted
from phase6_launch_reliability/launch_reliability_batch.py with the three
harness fixes Phase 6 itself recommended before this batch:

  1. STABILIZE_WINDOW widened 30s -> 75s: Phase 6 found FLIGHT's own
     scripted leg retraction (12s of motion right after separation) can
     repeatedly reset a strict 5%/0.995 consistency streak within a 30s
     budget, producing false non-confirmations for launches that had
     genuinely, separately been confirmed by hopper_locomotion itself.
  2. Periodic FULL daemon restart (not just node respawn) every
     DAEMON_RESTART_EVERY reps: Phase 6 saw DDS shared-memory transport
     warnings ("Failed init_port ... open_and_lock_file failed") before
     reps 9-10 of its 10-rep batch, and rep10's harness-reported timeout
     directly contradicted hopper_locomotion's own console log (which
     showed a real "Separation confirmed" 133s before the harness's
     timeout fired) -- a plausible missed-message/discovery race from
     accumulated stale SHM lock files across many sequential respawns in
     one continuous ROS session.
  3. rep6-style anomaly watch: Phase 6 saw one repeat with 4 launch
     attempts in rapid succession, including an "aborted after 60s
     post-ramp" claim that fired <1 real second after IGNITION -- almost
     certainly a burst of queued tick() callbacks after executor
     starvation. hopper_locomotion.py now logs real wall-clock elapsed
     time alongside the tick-derived figure at that exact abort point
     (Phase 7, see hopper_locomotion.py's SEPARATION_MAX_WAIT_TICKS abort
     block) and explicitly flags a "TICK/WALL-TIME MISMATCH" if it
     recurs; this harness greps all per-rep loco console logs for that
     marker at the end and surfaces it in the summary rather than relying
     on manual log inspection.

Run: python3 launch_delivery_batch_n30.py
"""
import json, math, os, subprocess, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_p7launch.yaml"
WORLD_FILE = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"

DISTANCE = 9.0
G = 1.14e-4
SIN2TH = 0.56
V_REQ = math.sqrt(DISTANCE * G / SIN2TH)
N_REPEATS = 30
DAEMON_RESTART_EVERY = 5

READY_TIMEOUT = 60.0
SEPARATION_TIMEOUT = 200.0
STABILIZE_WINDOW = 75.0
SAMPLE_PERIOD = 2.0
OUT = f"{LOG_DIR}/launch_delivery_n30_results.json"

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
        super().__init__('phase7_launch_batch')
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
    gz_log = open(f"{LOG_DIR}/gz_batch7.log", 'a')
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

    # BUG FIX (found live, Phase 7): this pattern omitted bridge_scout_1,
    # so between DAEMON_RESTART_EVERY boundaries every rep launched a NEW
    # bridge_scout_1 without killing the previous rep's -- 5 stacked
    # bridge processes (matching DAEMON_RESTART_EVERY=5) all fighting over
    # the same DDS domain reliably hung the batch at the 5th rep in a
    # block (confirmed: rep20 hung for 4+ hours with 5 live bridge_scout_1
    # processes found via ps aux). Must match every per-rep-launched node.
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
        logf = open(f"{LOG_DIR}/{name}_n30_rep{rep}.log", 'w')
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
        mean_r = sum(ratios) / len(ratios)
        med_r = ratios_sorted[len(ratios_sorted)//2]
        var = sum((r - mean_r)**2 for r in ratios) / len(ratios)
        std_r = math.sqrt(var)
        log(f"ratios={[round(v,3) for v in ratios]}")
        log(f"mean={mean_r:.3f} median={med_r:.3f} std={std_r:.3f} "
            f"min={min(ratios):.3f} max={max(ratios):.3f} range={max(ratios)-min(ratios):.3f}")

    log("--- scanning loco_scout_1_n30_rep*.log for TICK/WALL-TIME MISMATCH markers ---")
    found_any = False
    for rep in range(1, N_REPEATS + 1):
        p = f"{LOG_DIR}/loco_scout_1_n30_rep{rep}.log"
        if os.path.exists(p):
            with open(p) as f:
                for line in f:
                    if "TICK/WALL-TIME MISMATCH" in line:
                        found_any = True
                        log(f"  rep{rep}: {line.strip()}")
    if not found_any:
        log("  none found -- rep6-style anomaly did not recur in this batch")


if __name__ == '__main__':
    main()
