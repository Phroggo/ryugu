#!/usr/bin/env python3
"""Batch self-righting reliability test for C17/C18: teleport scout_1 to a
controlled tilt near ground level, let the current landing_controller's
self-righting run, and record whether it recovers (uz > 0.9) within a
generous timeout or gives up. Repeats for several moderate-tilt trials and
several full-inversion trials to compare against the paper's claimed
"moderate tilts recover reliably; full inversions recover ~1-in-4."
"""
import json, math, subprocess, time, random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/self_righting_batch_results.json"
SUCCESS_UZ = 0.9
TRIAL_TIMEOUT = 90.0
SETTLE_WAIT = 6.0

# (label, tilt_deg_from_vertical) pairs to run, in order
TRIALS = (
    [("moderate", 45)] * 2 +
    [("full_inversion", 175)] * 2
)

BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1.yaml"
NODE_LOG_DIR = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun"


def kill_scout1_nodes():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1'],
                    capture_output=True)
    time.sleep(1.5)


def launch_scout1_nodes(trial_idx):
    import os
    env = os.environ.copy()
    procs = []
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('loco_scout_1', ['ros2', 'run', 'ryugu_sim', 'hopper_locomotion', 'scout_1',
         '--ros-args', '-r', '__node:=loco_scout_1']),
        ('attitude_scout_1', ['ros2', 'run', 'ryugu_sim', 'attitude_controller', 'scout_1',
         '--ros-args', '-r', '__node:=attitude_scout_1']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{NODE_LOG_DIR}/{name}_trial{trial_idx}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, env=env)
    time.sleep(4)


def tilt_quaternion(tilt_deg, azimuth_deg):
    """Quaternion for a rotation of tilt_deg about an axis in the XY plane
    at azimuth_deg, i.e. tips the body-up vector away from vertical by
    tilt_deg toward a randomized horizontal direction (so trials aren't all
    identical single-axis tips)."""
    half = math.radians(tilt_deg) / 2.0
    az = math.radians(azimuth_deg)
    s = math.sin(half)
    return (s * math.cos(az), s * math.sin(az), 0.0, math.cos(half))


class RightingMonitor(Node):
    def __init__(self):
        super().__init__('righting_monitor')
        self.uz = None
        self.landed = None
        self.msg_count = 0
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 10)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.msg_count += 1

    def landed_cb(self, msg):
        self.landed = msg.data

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


_wake_toggle = [False]


def gz_wake_nudge():
    """Same trick as hopper_locomotion.py's _wake_model(): a sub-millimetre
    in-place set_pose to force a DART state change and defeat gz-sim8's
    quiescent-model sleep. Reads the current pose fresh each call so the
    nudge doesn't fight whatever the physics engine has done since."""
    try:
        result = subprocess.run(['gz', 'topic', '-e', '-t', '/model/scout_1/pose',
                                  '-n', '1', '--json-output'],
                                 capture_output=True, text=True, timeout=2)
        data = json.loads(result.stdout)
        p = data['pose'][0]['position']
        o = data['pose'][0]['orientation']
    except Exception:
        return
    _wake_toggle[0] = not _wake_toggle[0]
    dz = 0.0007 if _wake_toggle[0] else -0.0007
    req = (f"name: 'scout_1', position: {{x: {p['x']}, y: {p['y']}, z: {p['z'] + dz}}}, "
           f"orientation: {{x: {o['x']}, y: {o['y']}, z: {o['z']}, w: {o['w']}}}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/set_pose',
                     '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '1000', '--req', req], capture_output=True)


def gz_respawn(x, y, z, quat):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.0)
    qx, qy, qz, qw = quat
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
           f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


SPAWN_Z = 0.4          # clear of any leg-splay interpenetration at any tilt
LANDED_WAIT_TIMEOUT = 160.0  # real near-zero-g fall from 0.4m + contact + settle


def main():
    results = []

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    for i, (label, tilt_deg) in enumerate(TRIALS):
        az = random.uniform(0, 360)
        quat = tilt_quaternion(tilt_deg, az)
        log(f"--- trial {i+1}/{len(TRIALS)}: {label} ({tilt_deg} deg, az={az:.0f}) ---")

        # Fresh node set per trial: landing_controller's FSM state is not
        # reset by an entity respawn alone (confirmed -- it stayed wedged in
        # FLIGHT across all 12 trials of an earlier run), so kill and
        # relaunch all four scout_1 nodes before every trial.
        kill_scout1_nodes()
        gz_respawn(0.0, 0.5, SPAWN_Z, quat)
        launch_scout1_nodes(i + 1)

        rclpy.init()
        node = RightingMonitor()
        landed_sub_count_before = 0
        count_before = node.msg_count
        t0 = time.time()
        while node.msg_count - count_before < 30 and time.time() - t0 < SETTLE_WAIT:
            node.spin_for(0.2)

        start_uz = node.uz
        log(f"start uz={start_uz} (got {node.msg_count - count_before} fresh msgs); "
            f"spawned at z={SPAWN_Z}m, waiting for a genuine fall+impact to reach "
            f"landed=True (up to {LANDED_WAIT_TIMEOUT:.0f}s)")

        # Real fall from a safe clearance height -> genuine contact-accel
        # spike -> CONTACT_DETECTED -> LANDED. But a motionless fresh spawn
        # under Ryugu's near-zero gravity never accelerates fast enough to
        # avoid gz-sim8's DART quiescent-model sleep threshold (same root
        # cause as the C14 finding) -- so it never starts falling at all
        # unless nudged. Poke it once at the very start (a real fall only
        # needs a tiny push to get going under g=1.14e-4) and again only if
        # it's still completely frozen after a while.
        land_t0 = time.time()
        gz_wake_nudge()
        last_check = (time.time(), node.uz)
        nudges_used = 1
        while time.time() - land_t0 < LANDED_WAIT_TIMEOUT and node.landed is not True:
            node.spin_for(0.3)
            t_chk, uz_chk = last_check
            if time.time() - t_chk > 15.0:
                if node.uz is not None and abs(node.uz - uz_chk) < 1e-6 and nudges_used < 5:
                    gz_wake_nudge()
                    nudges_used += 1
                last_check = (time.time(), node.uz)

        log(f"landed={node.landed} after {time.time()-land_t0:.1f}s, uz={node.uz}, "
            f"nudges_used={nudges_used}")

        trial_t0 = time.time()
        peak_uz = node.uz if node.uz is not None else -1.0
        recovered = False
        recover_time = None
        while time.time() - trial_t0 < TRIAL_TIMEOUT:
            node.spin_for(0.2)
            if node.uz is not None:
                peak_uz = max(peak_uz, node.uz)
                if node.uz > SUCCESS_UZ:
                    recovered = True
                    recover_time = time.time() - trial_t0
                    break

        final_uz = node.uz
        outcome = "recovered" if recovered else "failed"
        log(f"outcome={outcome} final_uz={final_uz} peak_uz={peak_uz} "
            f"time={recover_time if recover_time else TRIAL_TIMEOUT:.1f}s")

        results.append({
            "trial": i + 1, "category": label, "commanded_tilt_deg": tilt_deg,
            "azimuth_deg": az, "start_uz": start_uz, "post_rest_wait_uz": node.uz,
            "final_uz": final_uz, "peak_uz": peak_uz, "outcome": outcome,
            "recover_time_s": recover_time,
        })
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

        node.destroy_node()
        rclpy.shutdown()

    log("=== batch complete ===")


if __name__ == '__main__':
    main()
