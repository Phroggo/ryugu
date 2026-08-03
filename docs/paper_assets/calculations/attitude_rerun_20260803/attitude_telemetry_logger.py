#!/usr/bin/env python3
import sys, time, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

robot = sys.argv[1] if len(sys.argv) > 1 else "scout_1"
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
outpath = sys.argv[3] if len(sys.argv) > 3 else "/tmp/attitude_log.jsonl"

class Logger(Node):
    def __init__(self):
        super().__init__('attitude_logger')
        self.buf = []
        self.create_subscription(Float64, f'/{robot}/attitude_error', self.err_cb, 200)
        self.create_subscription(Odometry, f'/{robot}/odometry', self.odom_cb, 200)
        self.create_subscription(Imu, f'/{robot}/imu', self.imu_cb, 200)

    def err_cb(self, msg):
        self.buf.append({"t": time.time(), "type": "attitude_error", "data": msg.data})

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.buf.append({"t": time.time(), "type": "odom", "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w})

    def imu_cb(self, msg):
        av = msg.angular_velocity
        self.buf.append({"t": time.time(), "type": "imu_angvel", "x": av.x, "y": av.y, "z": av.z})

def main():
    rclpy.init()
    node = Logger()
    start = time.time()
    while rclpy.ok() and (time.time() - start) < duration:
        rclpy.spin_once(node, timeout_sec=0.01)
    with open(outpath, 'w') as f:
        for entry in node.buf:
            f.write(json.dumps(entry) + "\n")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
