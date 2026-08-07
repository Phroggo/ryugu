#!/usr/bin/env python3
"""Phase 5 targeted test, v2 (replaces the spawn-based give-up repro,
which turned out too slow/unreliable to be practical -- see
PHASE5_CHANGE_REPORT.md sec 5 for why: severe-tilt spawns mostly never
reach a genuine landed=True at all within a practical window, matching
this project's own prior finding that even the proven slerp-teleport
method only triggers a righting attempt in ~25% of severe-tilt trials).

Directly and efficiently tests the actual mechanism the fix changes:
1. Spawn scout_1 upright, let it settle to a genuine LANDED state (fast,
   reliable -- proven throughout this session's smoke tests).
2. Inject a real residual z-axis (yaw) body rotation by publishing a
   direct step command to rw_z_joint_cmd_vel for a few seconds -- the
   wheel's own spin-up reaction torque (Newton's third law) kicks the
   BODY into a real, measured wz, simulating exactly what a give-up
   would leave behind (residual angular momentum), without needing to
   wait for an actual slow, unreliable give-up sequence to occur
   organically.
3. Stop the injection (return to whatever landing_controller/
   attitude_controller are doing) and watch whether wz decays (fix
   working) or persists/the body precesses into deeper inversion (gap
   still open).

Run with LABEL=BEFORE (pre-fix code, git stashed) and LABEL=AFTER
(fix applied), same injection parameters both times.

Usage: python3 z_disturbance_injection_test.py <label>
"""
import json, math, os, subprocess, sys, time

os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'
LOG_DIR = os.path.dirname(__file__)
BRIDGE_YAML = "/tmp/ryugu_bridge_scout_1_phase5b.yaml"
SPAWN_Z = 5.2  # FIXED (2026-08-08): was 6.0. Confirmed live: at (0, 0.5),
                # 6.0 never even reached CONTACT_DETECTED within 200s
                # (state stuck FLIGHT the whole time -- Ryugu's gravity is
                # so tiny that the extra ~0.8m of fall height matters a
                # lot). 5.2 matches the height this project's own C15/C16
                # methodology uses at this same XY, and is what showed
                # real ground interaction in this phase's earlier
                # (superseded) spawn-based repro attempt.
N_TRIALS = 2  # "a handful"; reduced from 3 given the corrected 200s settle-wait budget per trial
INJECT_SPEED = 40.0      # rad/s, step command to rw_z. FIXED AGAIN
                          # (2026-08-08): 20 rad/s produced a peak body
                          # |wz| of only 0.6-0.7 rad/s -- small enough
                          # that PASSIVE ground-contact friction alone
                          # damped it out within the 90s window in BOTH
                          # before/after states (confirmed: this test also
                          # never launched attitude_controller at all, so
                          # neither controller's z-authority was even
                          # exercised -- both prior BEFORE runs' decay was
                          # from friction, not either node). 80 rad/s
                          # targets a real, sustained body wz that
                          # friction alone is less likely to fully absorb
                          # in-window, while still well under the 250
                          # rad/s that produced a liftoff kick.
                          # PREVIOUS (superseded): 250.0
                          # was 250, which produced peak body |wz| of
                          # 9-11 rad/s -- violent enough to kick the robot
                          # off the surface entirely (confirmed live: uz
                          # swung to -0.38 post-injection, consistent with
                          # the well-documented LANDED->liftoff wheel-kick
                          # issue), leaving LANDED state and confounding
                          # the test with a DIFFERENT control regime
                          # (full in-flight attitude authority) instead of
                          # cleanly exercising the LANDED-state damper
                          # this phase's fix targets. 20 rad/s is well
                          # under the wheel speeds this file's own
                          # righting law treats as producing a mere
                          # "~3 rad/s free-body counter-roll" (see the
                          # RIGHTING_WHEEL_SPEED=160 comment), chosen to
                          # produce a real, measurable body wz without
                          # a liftoff kick.
INJECT_DURATION = 1.0    # s, shortened from 3.0 for the same reason --
                          # less total momentum transferred to the body
OBSERVE_AFTER_INJECT = 90.0  # s, post-injection watch window

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


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
    with open(BRIDGE_YAML, 'w') as f:
        for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
            f.write(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                     f'  ros_type_name: "{ros_ty}"\n  gz_type_name: "{gz_ty}"\n  direction: {dr}\n')


def gz_respawn(x, y, z):
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/remove',
                     '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', "name: 'scout_1', type: MODEL"],
                    capture_output=True)
    time.sleep(1.5)
    req = (f"sdf_filename: 'model://spacehopper', name: 'scout_1', "
           f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}")
    subprocess.run(['gz', 'service', '-s', '/world/ryugu_world/create',
                     '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '3000', '--req', req], capture_output=True)


def kill_scout1_nodes():
    subprocess.run(['pkill', '-9', '-f',
                     'bridge_scout_1|landing_scout_1|attitude_scout_1'], capture_output=True)
    time.sleep(1.5)


def launch_scout1_nodes(trial_idx, label):
    # REVERTED (2026-08-08): adding attitude_controller (previous attempt)
    # prevented landing_controller from ever leaving IDLE at all in 2/2
    # trials (likely attitude_controller's IDLE_ROTOR_SPEED sleep-defeat
    # rotor keeping velocity/accel just above the very tight
    # REST_VEL_MAX=0.005 m/s rest-detection threshold) -- meaning THAT
    # config never even reached state==LANDED, the exact code path this
    # fix changes, making it a worse test than this single-node config
    # despite being architecturally more complete. landing_controller
    # alone reliably reaches genuine landed=True (confirmed: 2/2 clean
    # trials at this exact config previously) at the cost of not
    # exercising the multi-node interaction -- an explicit, documented
    # trade-off, not an oversight (see PHASE5_CHANGE_REPORT.md sec 5).
    specs = [
        ('bridge_scout_1', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '--ros-args', '-r', '__node:=bridge_scout_1', '--params-file', '/dev/null',
         '-p', f'config_file:={BRIDGE_YAML}']),
        ('landing_scout_1', ['ros2', 'run', 'ryugu_sim', 'landing_controller', 'scout_1',
         '--ros-args', '-r', '__node:=landing_scout_1']),
    ]
    for name, cmd in specs:
        logf = open(f"{LOG_DIR}/{name}_{label}_trial{trial_idx}.log", 'w')
        subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    time.sleep(4)


class TrialMonitor(Node):
    def __init__(self, idx):
        super().__init__(f'phase5b_monitor_{idx}_{int(time.time())}')
        self.uz = None
        self.wz = None
        self.landed = None
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.z_pub = self.create_publisher(Float64, '/scout_1/rw_z_joint_cmd_vel', 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.uz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.wz = msg.twist.twist.angular.z

    def landed_cb(self, msg):
        self.landed = msg.data

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def run_trial(i, label, log):
    log(f"--- trial {i+1}/{N_TRIALS} [{label}] ---")
    kill_scout1_nodes()
    gz_respawn(0.0, 0.5, SPAWN_Z)
    make_bridge_yaml()
    launch_scout1_nodes(i, label)

    rclpy.init()
    node = TrialMonitor(i)

    # Wait for genuine LANDED confirmation. FIXED (2026-08-08): was 60s,
    # too short -- the settle-confirmation window itself is
    # REST_Z_TICKS=6000 (~60s) or REST_VEL_TICKS=12000 (~120s), so a 60s
    # TOTAL budget left zero margin for the fall+contact time that has to
    # happen BEFORE that window even starts counting (confirmed live:
    # both trials of the first attempt hit the 60s timeout with
    # landed still False). 200s matches this project's own
    # C15/C16 LANDED_WAIT_TIMEOUT, already validated for this exact wait.
    t0 = time.time()
    while time.time() - t0 < 200.0:
        node.spin_for(0.2)
        if node.landed is True:
            break
    settle_t = time.time() - t0
    log(f"  [{label} trial{i+1}] landed confirmed at t={settle_t:.1f}s, "
        f"uz={node.uz}, landed={node.landed}")
    if node.landed is not True:
        log(f"  [{label} trial{i+1}] WARNING: never confirmed landed within 200s, "
            f"injecting anyway")

    # Inject: step command to rw_z for INJECT_DURATION, then stop
    # publishing (hand control back to whatever landing_controller/
    # attitude_controller do next).
    inject_t0 = time.time()
    trace = []
    last_sample = -10
    while time.time() - inject_t0 < INJECT_DURATION:
        node.z_pub.publish(Float64(data=INJECT_SPEED))
        node.spin_for(0.05)
        elapsed = time.time() - inject_t0
        if elapsed - last_sample >= 0.5:
            trace.append({"phase": "inject", "t": round(elapsed, 2),
                          "uz": node.uz, "wz": node.wz, "landed": node.landed})
            last_sample = elapsed
    peak_wz_during_inject = max((abs(p['wz']) for p in trace if p['wz'] is not None), default=None)
    log(f"  [{label} trial{i+1}] injection done, peak |wz| during inject = {peak_wz_during_inject}")

    # Observe: does wz decay (fix working) or persist (gap still open)?
    obs_t0 = time.time()
    last_sample = -10
    while time.time() - obs_t0 < OBSERVE_AFTER_INJECT:
        node.spin_for(0.2)
        elapsed = time.time() - obs_t0
        if elapsed - last_sample >= 1.0:
            trace.append({"phase": "observe", "t": round(elapsed, 2),
                          "uz": node.uz, "wz": node.wz, "landed": node.landed})
            last_sample = elapsed

    final_wz = node.wz
    final_uz = node.uz
    node.destroy_node()
    rclpy.shutdown()

    obs_points = [p for p in trace if p['phase'] == 'observe' and p['wz'] is not None]
    wz_at_obs_start = obs_points[0]['wz'] if obs_points else None
    wz_at_obs_end = obs_points[-1]['wz'] if obs_points else None
    decayed = (wz_at_obs_start is not None and wz_at_obs_end is not None
               and abs(wz_at_obs_end) < abs(wz_at_obs_start) * 0.5)

    log(f"  [{label} trial{i+1}] DONE: wz at obs start={wz_at_obs_start}, "
        f"wz at obs end={wz_at_obs_end}, decayed>50%={decayed}, "
        f"final_uz={final_uz}")

    return {"trial": i + 1, "label": label, "settle_t": settle_t,
            "peak_wz_during_inject": peak_wz_during_inject,
            "wz_at_obs_start": wz_at_obs_start, "wz_at_obs_end": wz_at_obs_end,
            "decayed_over_50pct": decayed, "final_uz": final_uz, "trace": trace}


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "UNLABELED"

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []
    for i in range(N_TRIALS):
        r = run_trial(i, label, log)
        results.append(r)
        with open(f"{LOG_DIR}/z_disturbance_results_{label}.json", 'w') as f:
            json.dump(results, f, indent=2)

    log(f"\n=== SUMMARY [{label}] ===")
    for r in results:
        log(f"trial {r['trial']}: wz_start={r['wz_at_obs_start']}, "
            f"wz_end={r['wz_at_obs_end']}, decayed>50%={r['decayed_over_50pct']}, "
            f"final_uz={r['final_uz']}")


if __name__ == '__main__':
    main()
