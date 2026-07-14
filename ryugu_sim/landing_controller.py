#!/usr/bin/env python3
"""
Landing Controller — Impedance-based compliant landing for micro-gravity hopping.

Based on impedance control research (2024): variable impedance + energy tank
approaches for compliant landings on unknown asteroid surfaces.

Detects ground contact via IMU acceleration spikes, then switches leg joints
from position-control to a soft spring-damper profile to absorb impact and
prevent bouncing — critical in micro-gravity where even small rebound sends
the robot flying for minutes.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
import sys
import math

class LandingController(Node):
    # ── States ──
    IDLE = 0
    FLIGHT = 1
    CONTACT_DETECTED = 2
    SETTLING = 3
    LANDED = 4

    STATE_NAMES = {0: "IDLE", 1: "FLIGHT", 2: "CONTACT", 3: "SETTLING", 4: "LANDED"}

    def __init__(self, robot_name):
        super().__init__(f'landing_controller_{robot_name}')
        self.robot_name = robot_name
        self.state = self.IDLE
        self.settle_counter = 0

        self.get_logger().info(f'[{self.robot_name}] Landing Controller: ONLINE')

        # ── Tunable parameters ──
        # Contact detection: acceleration magnitude threshold (m/s²)
        # In micro-gravity, any accel > ~0.01 m/s² is suspicious; a landing
        # impact typically produces >0.05 m/s² even at very low speeds.
        self.contact_accel_threshold = 0.02  # m/s²
        self.settle_duration_ticks = 200     # ~2s at 100Hz IMU
        self.flight_accel_threshold = 0.005  # below this = free-fall = flight

        # Soft landing joint targets (slight crouch to absorb impact)
        self.soft_hip_target = 0.3    # gentle splay
        self.soft_knee_target = -0.4  # slight bend
        self.soft_p_gain = 0.3        # weak spring — compliant
        self.soft_d_gain = 0.5        # strong damper — energy absorption

        # ── Publishers ──
        self.joint_pubs = {}
        joints = ['hip_joint_0', 'knee_joint_0',
                  'hip_joint_1', 'knee_joint_1',
                  'hip_joint_2', 'knee_joint_2']
        for j in joints:
            topic = f'/{self.robot_name}/joint_{j}_cmd_pos'
            self.joint_pubs[j] = self.create_publisher(Float64, topic, 10)

        # Status publisher (other nodes can check if we've landed)
        self.landed_pub = self.create_publisher(
            Bool, f'/{self.robot_name}/landed', 10)

        # ── Subscribers ──
        self.create_subscription(
            Imu, f'/{self.robot_name}/imu', self.imu_callback, 10)

        # Flight trigger (hopper_locomotion tells us when a jump starts)
        self.create_subscription(
            Bool, f'/{self.robot_name}/jump_initiated', self.jump_callback, 10)

        # Periodic status log
        self.create_timer(5.0, self.log_status)

    def jump_callback(self, msg):
        """Called when hopper_locomotion initiates a jump."""
        if msg.data:
            self.state = self.FLIGHT
            self.settle_counter = 0
            self.get_logger().info(f'[{self.robot_name}] 🚀 Jump detected → FLIGHT mode')

    def imu_callback(self, msg):
        # Compute linear acceleration magnitude (excluding gravity component)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)

        if self.state == self.FLIGHT:
            # In flight, watch for contact spike
            if accel_mag > self.contact_accel_threshold:
                self.state = self.CONTACT_DETECTED
                self.settle_counter = 0
                self.get_logger().info(
                    f'[{self.robot_name}] 🎯 Contact detected! '
                    f'accel={accel_mag:.4f} m/s² → Switching to compliant mode')
                self._apply_soft_landing()

        elif self.state == self.CONTACT_DETECTED:
            # Keep applying soft landing commands
            self._apply_soft_landing()
            self.settle_counter += 1

            # Check if we've settled (low acceleration for sustained period)
            if accel_mag < self.flight_accel_threshold:
                # Still in micro-gravity free-fall = bounced off!
                if self.settle_counter > 50:  # gave it enough time
                    self.state = self.FLIGHT
                    self.get_logger().info(
                        f'[{self.robot_name}] ⚠️ Bounce detected → back to FLIGHT')
            else:
                # Sustained contact
                if self.settle_counter >= self.settle_duration_ticks:
                    self.state = self.LANDED
                    self.get_logger().info(
                        f'[{self.robot_name}] ✅ LANDED — stable contact confirmed')

        elif self.state == self.LANDED:
            pass  # Stay landed until next jump

        # Publish landed status
        self.landed_pub.publish(Bool(data=(self.state == self.LANDED)))

    def _apply_soft_landing(self):
        """Command joints to a compliant spring-damper posture."""
        for j, pub in self.joint_pubs.items():
            if 'hip' in j:
                pub.publish(Float64(data=self.soft_hip_target))
            elif 'knee' in j:
                pub.publish(Float64(data=self.soft_knee_target))

    def log_status(self):
        state_name = self.STATE_NAMES.get(self.state, "UNKNOWN")
        self.get_logger().info(
            f'[{self.robot_name}] Landing controller state: {state_name}')


def main(args=None):
    rclpy.init(args=args)
    robot_name = 'scout_1'
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    node = LandingController(robot_name)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
