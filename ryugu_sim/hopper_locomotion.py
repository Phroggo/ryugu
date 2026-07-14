#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import sys
import math

class HopperLocomotion(Node):
    # States
    IDLE = 0
    CROUCH = 1
    LAUNCH = 2
    FLIGHT = 3

    def __init__(self, robot_name):
        super().__init__(f'hopper_locomotion_{robot_name}')
        self.robot_name = robot_name
        self.get_logger().info(f'[{self.robot_name}] Tri-Pedal Locomotion Engine: ONLINE')
        
        self.state = self.IDLE
        self.state_timer = 0
        
        # Publishers for Joint Position Controllers
        self.joints = ['hip_joint_0', 'knee_joint_0', 'hip_joint_1', 'knee_joint_1', 'hip_joint_2', 'knee_joint_2']
        self.pubs = {}
        for j in self.joints:
            topic = f'/{self.robot_name}/joint_{j}_cmd_pos'
            self.pubs[j] = self.create_publisher(Float64, topic, 10)
            
        self.jump_init_pub = self.create_publisher(Bool, f'/{self.robot_name}/jump_initiated', 10)
            
        # Subscriber for Target Commands (From Swarm Manager)
        self.sub_target = self.create_subscription(Float64, f'/{self.robot_name}/jump_target_distance', self.jump_target_callback, 10)

        # Subscriber for Landed Status (From Landing Controller)
        self.sub_landed = self.create_subscription(Bool, f'/{self.robot_name}/landed', self.landed_callback, 10)

        # Timer for State Machine (10 Hz)
        self.timer = self.create_timer(0.1, self.tick)

    def landed_callback(self, msg):
        if msg.data and self.state == self.FLIGHT:
            self.get_logger().info(f"[{self.robot_name}] Landing controller reported LANDED. Initiating next jump instantly!")
            self.state = self.CROUCH
            self.state_timer = 0

    def jump_target_callback(self, msg):
        if self.state != self.IDLE:
            self.get_logger().warn(f"[{self.robot_name}] Ignoring jump command, currently not IDLE (state={self.state})")
            return
            
        distance = msg.data
        g = 0.000114 # Ryugu gravity
        v_req = math.sqrt(distance * g)
        
        self.get_logger().info(f"[{self.robot_name}] Target distance: {distance:.2f}m. Required Delta-V: {v_req:.4f} m/s")
        self.get_logger().info(f"[{self.robot_name}] Initiating Tri-Pedal Jump Sequence!")
        
        self.state = self.CROUCH
        self.state_timer = 0

    def set_joints(self, hip_val, knee_val):
        for j in self.joints:
            if 'hip' in j:
                self.pubs[j].publish(Float64(data=hip_val))
            elif 'knee' in j:
                self.pubs[j].publish(Float64(data=knee_val))

    def tick(self):
        if self.state == self.CROUCH:
            if self.state_timer == 0:
                self.get_logger().info("1. Crouching (Compressing Legs & Orienting to Target)")
                                # Physically lean the robot forward (nose down, rear up)
                self.pubs['hip_joint_0'].publish(Float64(data=1.2))
                self.pubs['knee_joint_0'].publish(Float64(data=-1.2))
                self.pubs['hip_joint_1'].publish(Float64(data=0.2))
                self.pubs['knee_joint_1'].publish(Float64(data=-0.2))
                self.pubs['hip_joint_2'].publish(Float64(data=0.2))
                self.pubs['knee_joint_2'].publish(Float64(data=-0.2))
            self.state_timer += 1
            if self.state_timer > 20: # 2.0 seconds to allow Reaction Wheels to spin chassis to target yaw
                self.state = self.LAUNCH
                self.state_timer = 0

        elif self.state == self.LAUNCH:
            if self.state_timer == 0:
                self.get_logger().info("2. IGNITION (Fast directional forward hop to target)")
                                # Symmetric max thrust. Because the robot is leaning forward, this fires it diagonally!
                self.set_joints(-1.0, 1.0)

            self.state_timer += 1
            if self.state_timer >= 2: # 0.2 seconds (2 * 0.1s tick)
                self.get_logger().info("3. Retracting for Landing Phase (FLIGHT Mode)")
                self.set_joints(0.0, 0.0)
                self.state = self.FLIGHT
                self.state_timer = 0
                
                # Signal the landing controller and attitude controller NOW that it is in the air
                self.jump_init_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'scout_1'
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    node = HopperLocomotion(robot_name)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
