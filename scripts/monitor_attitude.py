#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class AttitudeMonitor(Node):
    def __init__(self):
        super().__init__('attitude_monitor')
        self.sub = self.create_subscription(Float64, '/scout_1/attitude_error', self.callback, 10)
        self.initial_error = None
        self.start_time = None
        self.converged = False

    def callback(self, msg):
        err = msg.data
        if self.start_time is None and err > 0.5: # Large initial error from tumble
            self.start_time = self.get_clock().now()
            self.initial_error = err
            print(f"[{self.get_clock().now().nanoseconds / 1e9:.2f}] Tumble detected! Initial Error: {err:.2f} rad")
            
        if self.start_time is not None and not self.converged:
            if err < 0.05: # Converged
                end_time = self.get_clock().now()
                duration = (end_time - self.start_time).nanoseconds / 1e9
                print(f"[{end_time.nanoseconds / 1e9:.2f}] Stabilized! Time to recover: {duration:.2f} seconds.")
                self.converged = True
            elif (self.get_clock().now() - self.start_time).nanoseconds / 1e9 > 15.0:
                print("Failed to converge after 15 seconds.")
                self.converged = True

def main():
    rclpy.init()
    node = AttitudeMonitor()
    
    # Trigger the jump
    import subprocess
    subprocess.Popen(["/bin/bash", "-c", "source /home/melvin/ryugu_v2_ws/install/setup.bash && ./trigger_jump.sh 5.0"], cwd="/home/melvin/ryugu_v2_ws/src/ryugu_sim/scripts")
    
    # Wait until it converges
    while rclpy.ok() and not node.converged:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
