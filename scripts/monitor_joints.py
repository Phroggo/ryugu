import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.create_subscription(Odometry, '/scout_1/odom', self.odom_cb, 10)
        self.z = 0.0
        
    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z

    def js_cb(self, msg):
        if 'hip_joint_0' in msg.name:
            idx_h = msg.name.index('hip_joint_0')
            idx_k = msg.name.index('knee_joint_0')
            print(f"Z: {self.z:.4f} | Hip: {msg.position[idx_h]:.4f} | Knee: {msg.position[idx_k]:.4f}")

rclpy.init()
node = Monitor()
rclpy.spin(node)
