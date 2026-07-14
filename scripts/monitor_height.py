#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class HeightMonitor(Node):
    def __init__(self):
        super().__init__('height_monitor')
        self.sub = self.create_subscription(Odometry, '/scout_1/odom', self.odom_callback, 10)
        self.max_z = 0.0

    def odom_callback(self, msg):
        z = msg.pose.pose.position.z
        if z > self.max_z:
            self.max_z = z
            print(f"New Max Height: {self.max_z:.4f} m", flush=True)

def main():
    rclpy.init()
    node = HeightMonitor()
    # Trigger the jump
    import subprocess
    subprocess.Popen(["/bin/bash", "-c", "source /home/melvin/ryugu_v2_ws/install/setup.bash && ./trigger_jump.sh 5.0"], cwd="/home/melvin/ryugu_v2_ws/src/ryugu_sim/scripts")
    # Wait and spin
    import time
    start_time = time.time()
    while time.time() - start_time < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"Final Max Height after 10s: {node.max_z}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
