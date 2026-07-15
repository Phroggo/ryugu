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
        # Launch joint amplitude (rad), scaled per-jump by requested distance
        # in jump_target_callback. Defaults to the previously-hardcoded value
        # (full ±1.0 rad) for any code path that skips scaling (e.g. idle
        # recovery hops).
        self.launch_amplitude = 1.0

        # Idle-on-ground self-recovery: if landed with no jump command for
        # this many ticks (10Hz -> 3000 ticks = 5 min), self-initiate a small
        # hop via legs rather than sitting dead. Raised from 30s on
        # 2026-07-15: landing settle alone takes ~45-50s in micro-gravity
        # (rest-detection window + settle confirmation), so a 30s timer kept
        # expiring while the rest of the stack was still mid-cycle and
        # repeatedly kicked the robot between normal mission commands. 5 min
        # is still a useful unstick fallback but comfortably exceeds every
        # normal land->decide->dispatch cycle. Known simplification: this
        # doesn't check battery/RECHARGE role (hopper has no visibility into
        # swarm_manager state), so a RECHARGE-role agent could still get a
        # small recovery nudge if idle that long.
        self.idle_ticks = 0
        self.IDLE_RECOVERY_TICKS = 3000
        self.RECOVERY_AMPLITUDE = 0.5
        # Only count idle time toward a recovery hop while actually landed --
        # a freshly-(re)started node, or one whose robot is mid-flight,
        # shouldn't self-fire a hop just because its own uptime passed the
        # threshold (added 2026-07-15 after exactly that happened live).
        self.landed = False
        
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
        self.landed = msg.data
        if msg.data and self.state == self.FLIGHT:
            self.get_logger().info(f"[{self.robot_name}] Landing controller reported LANDED. Settling to IDLE.")
            self.state = self.IDLE
            self.state_timer = 0
            self.idle_ticks = 0

    def jump_target_callback(self, msg):
        if self.state != self.IDLE:
            self.get_logger().warn(f"[{self.robot_name}] Ignoring jump command, currently not IDLE (state={self.state})")
            return
            
        distance = msg.data
        g = 0.000114 # Ryugu gravity
        v_req = math.sqrt(max(distance, 0.0) * g)

        # Scale launch amplitude around the reference calibration point: a 5m
        # target at full +/-1.0 rad amplitude (0.2s) was verified to reach
        # apex ~5.57m (see HANDOFF.md). Previously v_req was computed and
        # logged but never actually used -- every jump fired an identical
        # fixed impulse regardless of requested distance. Clamped to
        # [0.3, 2.5] rad: floor keeps short hops from failing to lift off,
        # ceiling stays well inside the +/-3.14 rad joint limit and avoids
        # slamming the controller's torque cap on long-distance requests.
        v_ref = math.sqrt(5.0 * g)
        scale = (v_req / v_ref) if v_ref > 0 else 1.0
        self.launch_amplitude = max(0.3, min(2.5, scale))

        self.get_logger().info(f"[{self.robot_name}] Target distance: {distance:.2f}m. Required Delta-V: {v_req:.4f} m/s. Launch amplitude: {self.launch_amplitude:.2f} rad")
        self.get_logger().info(f"[{self.robot_name}] Initiating Tri-Pedal Jump Sequence!")
        
        self.state = self.CROUCH
        self.state_timer = 0
        self.idle_ticks = 0

    def set_joints(self, hip_val, knee_val):
        for j in self.joints:
            if 'hip' in j:
                self.pubs[j].publish(Float64(data=hip_val))
            elif 'knee' in j:
                self.pubs[j].publish(Float64(data=knee_val))

    def tick(self):
        if self.state == self.IDLE:
            if not self.landed:
                return
            self.idle_ticks += 1
            if self.idle_ticks >= self.IDLE_RECOVERY_TICKS:
                self.get_logger().info(
                    f"[{self.robot_name}] 🦵 Idle {self.idle_ticks/10:.0f}s with no jump command "
                    f"-- self-initiating recovery hop (legs).")
                self.launch_amplitude = self.RECOVERY_AMPLITUDE
                self.state = self.CROUCH
                self.state_timer = 0
                self.idle_ticks = 0
            return

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
                self.get_logger().info(f"2. IGNITION (amplitude={self.launch_amplitude:.2f} rad, distance-scaled forward hop to target)")
                # Found 2026-07-15: firing all 3 legs to the SAME absolute
                # target (the old set_joints(-amp, amp) call) made leg 0
                # sweep a much larger angle than legs 1/2 within the same
                # fixed 0.2s window, because CROUCH leaves leg 0 bent much
                # further (1.2/-1.2) than legs 1/2 (0.2/-0.2) for the
                # intentional forward lean. Unequal angular travel in a fixed
                # time means unequal per-leg thrust at liftoff -- a real net
                # torque impulse on the chassis at the exact moment it leaves
                # the ground, which is what was causing the robot to start
                # tumbling immediately on every jump. Fix: apply the same
                # DELTA (launch_amplitude) from each leg's own crouch
                # baseline instead of the same absolute target, so every leg
                # travels an equal angular distance -- balanced thrust
                # magnitude -- while the crouch lean still shapes the
                # resulting diagonal thrust *direction* via each leg's
                # differing final position.
                self.pubs['hip_joint_0'].publish(Float64(data=1.2 - self.launch_amplitude))
                self.pubs['knee_joint_0'].publish(Float64(data=-1.2 + self.launch_amplitude))
                self.pubs['hip_joint_1'].publish(Float64(data=0.2 - self.launch_amplitude))
                self.pubs['knee_joint_1'].publish(Float64(data=-0.2 + self.launch_amplitude))
                self.pubs['hip_joint_2'].publish(Float64(data=0.2 - self.launch_amplitude))
                self.pubs['knee_joint_2'].publish(Float64(data=-0.2 + self.launch_amplitude))

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
