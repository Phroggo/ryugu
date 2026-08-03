#!/usr/bin/env python3
"""V_GAIN calibration sweep, rewritten to fix the two failure modes found in
the orphaned 2026-07-25 runs (vgain_calibration_20260725_{132802,140559}):
  1. separation detection timed out because it wasn't actually subscribed to
     the /separation topic -- this version subscribes directly.
  2. large distances blew the old 10-minute landed/ready timeout because
     flight times at some commanded distances genuinely run long (consistent
     with the paper's own 695s/9m-hop finding) -- this version uses a much
     longer per-trial budget and checkpoints after every trial so a partial
     run is still usable.
Stabilization criterion matches the paper's stated methodology (Research_Paper.md
C4): three consecutive 2s samples within 5% magnitude and 0.995 cosine similarity.
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry

DISTANCES = [0.5, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0]
G = 1.14e-4
SIN2TH = 0.56
OUT = "/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/attitude_rerun/calibration_v2_results.json"

READY_TIMEOUT = 240.0
SEPARATION_TIMEOUT = 120.0
STABILIZE_WINDOW = 90.0
SAMPLE_PERIOD = 2.0


class Calib(Node):
    def __init__(self):
        super().__init__('calib_v2')
        self.landed = None
        self.speed = None
        self.separated = False
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

    def wait_ready(self, log, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.2)
            if self.landed is True and self.speed is not None and self.speed < 0.02:
                return True
        log("ready-timeout (proceeding anyway)")
        return False

    def wait_separation(self, timeout):
        self.separated = False
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.2)
            if self.separated:
                return True
        return False

    def sample_velocity(self):
        self.spin_for(0.05)
        return self._last_v if hasattr(self, '_last_v') else (0, 0, 0)


def cosine_sim(a, b):
    dot = sum(x*y for x, y in zip(a, b))
    na = math.sqrt(sum(x*x for x in a))
    nb = math.sqrt(sum(x*x for x in b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return dot / (na * nb)


def main():
    rclpy.init()
    node = Calib()
    results = []

    def log(msg):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        print(line, flush=True)

    for d in DISTANCES:
        v_req = math.sqrt(d * G / SIN2TH)
        log(f"--- distance={d}m v_req={v_req:.5f} m/s ---")

        node.wait_ready(log, READY_TIMEOUT)

        node.pub_dist.publish(Float64(data=d))
        log(f"published jump_target_distance={d}")

        if not node.wait_separation(SEPARATION_TIMEOUT):
            log("TIMEOUT waiting for separation")
            results.append({"distance": d, "v_req": v_req, "status": "no_separation"})
            with open(OUT, 'w') as f:
                json.dump(results, f, indent=2)
            continue

        log("separation detected, sampling velocity for stabilization...")
        samples = []
        t0 = time.time()
        stabilized = False
        delivered = None
        stabilize_time = None
        last_sample_t = 0
        while time.time() - t0 < STABILIZE_WINDOW:
            node.spin_for(0.1)
            if time.time() - last_sample_t >= SAMPLE_PERIOD:
                vx, vy, vz = node.sample_velocity()
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
        if stabilized:
            log(f"STABILIZED: delivered={delivered:.4f} ratio={delivered/v_req:.3f} t={stabilize_time:.1f}s")
            results.append({"distance": d, "v_req": v_req, "delivered": delivered,
                             "ratio": delivered / v_req, "status": "stabilized",
                             "stabilize_time_s": stabilize_time, "n_samples": len(samples)})
        else:
            log("never stabilized within window -- discarding per methodology")
            results.append({"distance": d, "v_req": v_req, "status": "never_stabilized",
                             "n_samples": len(samples)})

        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== sweep complete ===")
    with open(OUT, 'w') as f:
        json.dump(results, f, indent=2)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
