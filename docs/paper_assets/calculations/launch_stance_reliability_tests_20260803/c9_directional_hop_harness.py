#!/usr/bin/env python3
"""C9 retest: attempt to reproduce the headline 4.3m directional-hop claim.
Sets a target yaw (azimuth -56 deg, matching the paper's own reported
value), waits for yaw-hold to converge, commands a directional hop, and
logs odometry position/time throughout to measure actual ground
displacement, heading error, and flight duration."""
import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/c9_directional_hop_raw.jsonl"
AZIMUTH_DEG = -56.0
DISTANCE_M = 3.0
MAX_FLIGHT_WAIT = 1400.0  # up to ~23 min, matching the paper's ~20 min claim


class HopTest(Node):
    def __init__(self):
        super().__init__('c9_test')
        self.buf = []
        self.landed = None
        self.separated = False
        self.start_xy = None
        self.yaw_pub = self.create_publisher(Float64, '/scout_1/target_yaw', 10)
        self.dist_pub = self.create_publisher(Float64, '/scout_1/jump_target_distance', 10)
        self.create_subscription(Odometry, '/scout_1/odometry', self.odom_cb, 20)
        self.create_subscription(Bool, '/scout_1/landed', self.landed_cb, 10)
        self.create_subscription(Bool, '/scout_1/separation', self.sep_cb, 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.buf.append({"t": time.time(), "x": p.x, "y": p.y, "z": p.z, "yaw_deg": math.degrees(yaw)})
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)

    def landed_cb(self, msg):
        self.landed = msg.data

    def sep_cb(self, msg):
        if msg.data:
            self.separated = True

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=min(0.2, seconds))


def main():
    rclpy.init()
    node = HopTest()

    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    wait_t0 = time.time()
    while (node.yaw_pub.get_subscription_count() < 1 or node.dist_pub.get_subscription_count() < 1) and time.time() - wait_t0 < 10.0:
        node.spin_for(0.1)

    yaw_rad = math.radians(AZIMUTH_DEG)
    log(f"commanding target_yaw={AZIMUTH_DEG} deg ({yaw_rad:.4f} rad)")
    node.yaw_pub.publish(Float64(data=yaw_rad))

    t0 = time.time()
    while time.time() - t0 < 15.0:
        node.spin_for(0.2)
    log(f"post-yaw-wait: current buf last yaw={node.buf[-1]['yaw_deg'] if node.buf else None}")

    log(f"commanding jump_target_distance={DISTANCE_M}m")
    node.separated = False
    node.dist_pub.publish(Float64(data=DISTANCE_M))
    time.sleep(0.2)
    node.dist_pub.publish(Float64(data=DISTANCE_M))

    sep_t0 = time.time()
    while not node.separated and time.time() - sep_t0 < 90.0:
        node.spin_for(0.2)

    if not node.separated:
        log("separation never detected within 90s -- aborting")
        with open(OUT, 'w') as f:
            for e in node.buf:
                f.write(json.dumps(e) + "\n")
        rclpy.shutdown()
        return

    log(f"separation detected at t+{time.time()-sep_t0:.1f}s, tracking flight...")
    node.landed = False
    flight_t0 = time.time()
    last_log = 0
    while time.time() - flight_t0 < MAX_FLIGHT_WAIT and node.landed is not True:
        node.spin_for(0.3)
        if time.time() - flight_t0 - last_log > 30:
            last_log = time.time() - flight_t0
            if node.buf:
                p = node.buf[-1]
                log(f"  t+{last_log:.0f}s pos=({p['x']:.2f},{p['y']:.2f},{p['z']:.2f}) yaw={p['yaw_deg']:.1f}")

    with open(OUT, 'w') as f:
        for e in node.buf:
            f.write(json.dumps(e) + "\n")

    log(f"=== done: landed={node.landed} after {time.time()-flight_t0:.1f}s, {len(node.buf)} samples ===")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
