#!/usr/bin/env python3
"""C14 retest: reproduce the tumble the way the original dev-log measurement
did (walkthrough.md lines 29-31) -- inject an asymmetric torque by forcing
hip_joint_0 to overextend during the launch phase, rather than an artificial
pose injection. Watches for the ignition log line, then overrides
hip_joint_0 (racing hopper_locomotion's own command, last-write-wins) for a
short window right at liftoff, then logs odometry/imu through the ensuing
flight to see whether attitude_controller arrests the resulting tumble."""
import json, math, re, subprocess, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/c14_final4_raw.jsonl"
LOCO_LOG = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/loco_wa6.log"
OVEREXTEND_VAL = -2.8   # near the hip joint's physical limit (±3.14 rad)
INJECT_DURATION = 12.0   # extended further past the ramp to see if a bigger tumble results


class TorqueTest(Node):
    def __init__(self):
        super().__init__('c14_asym_test')
        self.buf = []
        self.hip0_pub = self.create_publisher(Float64, '/scout_1/joint_hip_joint_0_cmd_pos', 10)
        self.dist_pub = self.create_publisher(Float64, '/scout_1/jump_target_distance', 10)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Imu, '/scout_1/imu', self.imu_cb, 20)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.buf.append({"t": time.time(), "type": "odom", "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w})

    def imu_cb(self, msg):
        av = msg.angular_velocity
        self.buf.append({"t": time.time(), "type": "imu_angvel", "x": av.x, "y": av.y, "z": av.z})

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.1, seconds))


def watch_for_ignition(log_path, found_event, start_pos):
    """Poll the loco node's log file for the IGNITION line."""
    while not found_event.is_set():
        try:
            with open(log_path) as f:
                f.seek(start_pos[0])
                for line in f:
                    if 'IGNITION' in line:
                        found_event.set()
                        return
                start_pos[0] = f.tell()
        except FileNotFoundError:
            pass
        time.sleep(0.05)


def main():
    rclpy.init()
    node = TorqueTest()

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    # baseline log position (don't react to old IGNITION lines from earlier tests)
    try:
        start_pos = [__import__('os').path.getsize(LOCO_LOG)]
    except FileNotFoundError:
        start_pos = [0]

    found_event = threading.Event()
    watcher = threading.Thread(target=watch_for_ignition, args=(LOCO_LOG, found_event, start_pos), daemon=True)
    watcher.start()

    # wait for the publisher to actually discover hopper_locomotion's
    # subscriber before publishing, or the command is silently dropped
    wait_t0 = time.time()
    while node.dist_pub.get_subscription_count() < 1 and time.time() - wait_t0 < 10.0:
        node.spin_for(0.1)
    log(f"dist_pub subscriber count={node.dist_pub.get_subscription_count()}")

    log("commanding jump distance=1.5m")
    node.dist_pub.publish(Float64(data=1.5))
    time.sleep(0.2)
    node.dist_pub.publish(Float64(data=1.5))

    t0 = time.time()
    while not found_event.is_set() and time.time() - t0 < 60.0:
        node.spin_for(0.1)

    if not found_event.is_set():
        log("IGNITION never detected within 60s -- aborting")
        rclpy.shutdown()
        return

    ignition_t = time.time()
    log(f"IGNITION detected at t+{ignition_t-t0:.2f}s -- injecting overextended hip_joint_0={OVEREXTEND_VAL} for {INJECT_DURATION}s")

    inject_t0 = time.time()
    while time.time() - inject_t0 < INJECT_DURATION:
        node.hip0_pub.publish(Float64(data=OVEREXTEND_VAL))
        node.spin_for(0.02)

    log("injection done, logging flight for 40s")
    log_t0 = time.time()
    while time.time() - log_t0 < 40.0:
        node.spin_for(0.1)

    with open(OUT, 'w') as f:
        for entry in node.buf:
            f.write(json.dumps(entry) + "\n")
    log(f"=== done, {len(node.buf)} samples written ===")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
