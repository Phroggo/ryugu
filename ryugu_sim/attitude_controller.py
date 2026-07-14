#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import sys
import math

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class AttitudeController(Node):
    def __init__(self, robot_name):
        super().__init__(f'attitude_controller_{robot_name}')
        self.robot_name = robot_name
        self.get_logger().info(f'[{self.robot_name}] Reaction Wheel Attitude Controller: ONLINE')
        
        self.pubs = {}
        for axis in ['x', 'y', 'z']:
            topic = f'/{self.robot_name}/rw_{axis}_joint_cmd_vel'
            self.pubs[axis] = self.create_publisher(Float64, topic, 10)
            
        self.error_pub = self.create_publisher(Float64, f'/{self.robot_name}/attitude_error', 10)
        self.rw_speed_pub = self.create_publisher(Float64, f'/{self.robot_name}/rw_speed_max', 10)
            
        self.create_subscription(Imu, f'/{self.robot_name}/imu', self.imu_callback, 10)
        self.create_subscription(Bool, f'/{self.robot_name}/jump_initiated', self.jump_callback, 10)
        self.create_subscription(Float64, f'/{self.robot_name}/target_yaw', self.target_yaw_callback, 10)
        
        # We track our commanded speeds since Gazebo velocity controller achieves them near instantly
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        self.Kp = 20.0
        self.Ki = 0.5
        self.Kd = 5.0
        
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        self.integral_yaw = 0.0
        
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        self.max_rw_speed = 1396.0 # rad/s (H_max = 0.377 Nms / I = 0.00027 kgm^2)
        self.in_flight = False
        
        self.last_time = self.get_clock().now()

    def jump_callback(self, msg):
        if msg.data:
            self.in_flight = True
            self.get_logger().info(f'[{self.robot_name}] Jump initiated. Flight mode active.')

    def target_yaw_callback(self, msg):
        self.target_yaw = msg.data
        self.get_logger().info(f'[{self.robot_name}] New target yaw received: {self.target_yaw:.2f} rad')

    def imu_callback(self, msg):
        # Allow attitude controller to run on the ground so it can orient towards targets

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Yaw control is always active
        error_yaw = self.target_yaw - yaw
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi
        self.integral_yaw += error_yaw * dt
        max_integral = 10.0
        self.integral_yaw = max(-max_integral, min(max_integral, self.integral_yaw))
        new_cmd_z = -(self.Kp * error_yaw + self.Ki * self.integral_yaw - self.Kd * wz)
        self.cmd_vel['z'] += new_cmd_z * dt

        if self.in_flight:
            # Pitch and Roll are ONLY controlled during flight
            error_roll = self.target_roll - roll
            error_pitch = self.target_pitch - pitch
            self.integral_roll += error_roll * dt
            self.integral_pitch += error_pitch * dt
            self.integral_roll = max(-max_integral, min(max_integral, self.integral_roll))
            self.integral_pitch = max(-max_integral, min(max_integral, self.integral_pitch))
            new_cmd_x = -(self.Kp * error_roll + self.Ki * self.integral_roll - self.Kd * wx)
            new_cmd_y = -(self.Kp * error_pitch + self.Ki * self.integral_pitch - self.Kd * wy)
            self.cmd_vel['x'] += new_cmd_x * dt
            self.cmd_vel['y'] += new_cmd_y * dt
        else:
            # On the ground, bleed off pitch/roll commands
            self.cmd_vel['x'] *= 0.9
            self.cmd_vel['y'] *= 0.9
            error_roll = 0.0
            error_pitch = 0.0

        max_speed = max(abs(self.cmd_vel['x']), abs(self.cmd_vel['y']), abs(self.cmd_vel['z']))
        
        if max_speed > self.max_rw_speed * 0.9:
            self.get_logger().warn(f'[{self.robot_name}] Reaction Wheel Saturation Warning! Speed: {max_speed:.1f} rad/s', throttle_duration_sec=2.0)
            self.cmd_vel['x'] *= 0.99
            self.cmd_vel['y'] *= 0.99
            self.cmd_vel['z'] *= 0.99
        
        self.pubs['x'].publish(Float64(data=self.cmd_vel['x']))
        self.pubs['y'].publish(Float64(data=self.cmd_vel['y']))
        self.pubs['z'].publish(Float64(data=self.cmd_vel['z']))
        
        total_error = math.sqrt(error_roll**2 + error_pitch**2)
        self.error_pub.publish(Float64(data=total_error))
        self.rw_speed_pub.publish(Float64(data=max_speed))

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'scout_1'
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    node = AttitudeController(robot_name)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
