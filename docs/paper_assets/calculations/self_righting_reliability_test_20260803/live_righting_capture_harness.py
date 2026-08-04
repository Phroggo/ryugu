#!/usr/bin/env python3
"""Capture whatever's happening RIGHT NOW -- scout_1 landed badly tilted
during the C9 test and real self-righting engaged. Log until it resolves
(uz>0.9 success, or it gives up) rather than a fixed window."""
import json, math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/c9_incidental_righting_raw.jsonl"

rclpy.init()


class Cap(Node):
    def __init__(self):
        super().__init__('live_righting_capture')
        self.buf = []
        self.create_subscription(Odometry, '/scout_1/odometry', self.cb, 20)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        uz = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.buf.append({"t": time.time(), "uz": uz})


node = Cap()
t0 = time.time()
last_print = 0
while time.time() - t0 < 180:
    rclpy.spin_once(node, timeout_sec=0.2)
    if node.buf and time.time() - last_print > 2:
        print(f"t+{time.time()-t0:.1f}s uz={node.buf[-1]['uz']:.4f}", flush=True)
        last_print = time.time()
    if node.buf and node.buf[-1]['uz'] > 0.9:
        print("RECOVERED (uz>0.9)", flush=True)
        break

with open(OUT, 'w') as f:
    for e in node.buf:
        f.write(json.dumps(e) + "\n")
print(f"=== done, {len(node.buf)} samples ===", flush=True)
rclpy.shutdown()
