#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
import sys
import math

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
        self.create_subscription(Bool, f'/{self.robot_name}/landed', self.landed_callback, 10)
        self.create_subscription(Float64, f'/{self.robot_name}/target_yaw', self.target_yaw_callback, 10)

        # We command wheel velocity directly (proportional to error and
        # damped by body rate), not accumulated acceleration -- see the
        # rewrite note below for why.
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Rewritten 2026-07-14 after the first-ever live closed-loop test
        # (see task.md B6 / walkthrough.md) showed attitude_error oscillating
        # between 1.5-2.8 rad (85-160 deg) instead of converging. Root cause:
        # the old controller used Euler-angle roll/pitch (via
        # euler_from_quaternion) for the tilt correction, which is only a
        # valid small-angle approximation -- at the large tumble angles this
        # robot actually experiences, IMU angular_velocity.x/y (body-frame
        # rates) stop corresponding to roll_dot/pitch_dot, so the old
        # "-Kd*wx" damping term was damping the wrong quantity and could
        # even add energy instead of removing it. It also accumulated PID
        # output into velocity every tick (cmd_vel += out*dt) rather than
        # commanding velocity directly, an extra integrator that made the
        # whole loop more prone to oscillation than necessary.
        #
        # New approach for tilt (roll/pitch-equivalent): instead of decomposing
        # orientation into Euler angles, rotate the body's local +Z ("up")
        # axis into the world frame and take its cross product with world +Z.
        # This gives a rotation-axis-aligned error vector that's valid at ANY
        # tilt magnitude (no small-angle assumption, no gimbal lock), is
        # inherently yaw-independent (matches the original intent -- we don't
        # care which way it's facing while just trying not to be upside-down),
        # and only breaks down at exactly 180 deg (a genuine physical
        # singularity no P-only law can resolve; landing_controller's
        # leg-based self-righting handles full inversions separately).
        # Verified algebraically to match the old controller's sign
        # convention in the small-angle limit (pure roll/pitch cases).
        #
        # Yaw keeps simple atan2-based extraction (a single rotation about a
        # fixed world axis, well-posed except at ~90 deg pitch gimbal lock,
        # which isn't this robot's primary failure mode) but gets the same
        # direct-command structural fix.
        #
        # Integral terms removed entirely: in near-zero-g free flight there's
        # no persistent disturbance torque for an I-term to compensate for
        # (unlike e.g. gravity droop in a terrestrial position servo), so it
        # was pure added windup/overshoot risk for no benefit -- this also
        # removes the manual integral-reset-on-landing workaround the
        # previous fix needed.
        self.Kp_tilt = 300.0
        self.Kd_tilt = 60.0
        self.Kp_yaw = 300.0
        self.Kd_yaw = 60.0

        self.target_yaw = 0.0
        self.max_rw_speed = 1396.0 # rad/s (H_max = 0.377 Nms / I = 0.00027 kgm^2)
        self.in_flight = False
        self.last_imu_time = None

        # Found 2026-07-15 (live telemetry: /scout_1/rw_y_joint_cmd_vel swinging
        # -193 -> +27 -> -206 rad/s across consecutive ~1s samples while the
        # body genuinely tumbled at 1.5-3 rad/s "as soon as the jump started").
        # Root cause: Kp_tilt=300 on a bounded error term commands target
        # velocities the RW motor cannot remotely reach -- the RW joint's
        # torque ceiling is only 0.015 Nm (Research_Paper.md SS3.2, real Maxon
        # EC20 "Continuous" spec), which caps wheel angular acceleration at
        # ~55.6 rad/s^2 (0.015 / 0.00027 kgm^2). Reaching a 300 rad/s target
        # would take 5+ seconds; the outer loop instead recomputes a brand
        # new (often oppositely-signed) target every ~10ms IMU tick, so the
        # low-level JointController sits permanently torque-saturated and the
        # only thing that actually matters physically is the *sign* of the
        # commanded velocity. That sign was flipping essentially every tick
        # -- chattering -- rather than holding steady for the clean
        # accelerate-then-decelerate bang-bang correction the paper's own
        # kinematic model assumes (t ~= 1.07s to correct 90 deg at constant
        # alpha=2.727 rad/s^2, SS3.2). Slew-limiting the *commanded target* to
        # this same physical acceleration ceiling forces the sign to hold for
        # a physically sensible duration instead of chattering, without
        # changing the real torque budget the paper specifies.
        self.max_wheel_accel = 50.0 # rad/s^2, conservative vs the 55.6 physical ceiling

    def jump_callback(self, msg):
        if msg.data:
            self.in_flight = True
            self.get_logger().info(f'[{self.robot_name}] Jump initiated. Flight mode active.')

    def landed_callback(self, msg):
        """Subscribing to /landed at all was the actual bug found 2026-07-14
        (see git history) -- in_flight was previously set True on launch and
        never reset, so tilt correction ran forever post-landing, fighting
        landing_controller's self-righting. Once grounded, landing_controller
        owns orientation correction; this controller only holds yaw."""
        if msg.data and self.in_flight:
            self.in_flight = False
            self.get_logger().info(f'[{self.robot_name}] Landed — tilt correction stopped, RWs handed off to yaw-hold only.')

    def target_yaw_callback(self, msg):
        self.target_yaw = msg.data
        self.get_logger().info(f'[{self.robot_name}] New target yaw received: {self.target_yaw:.2f} rad')

    def imu_callback(self, msg):
        q = msg.orientation
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # --- Yaw hold (always active, including grounded, so the robot can
        # orient toward the next target while sitting still) ---
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        error_yaw = self.target_yaw - yaw
        error_yaw = (error_yaw + math.pi) % (2.0 * math.pi) - math.pi
        cmd_z = -self.Kp_yaw * error_yaw + self.Kd_yaw * wz

        total_error = abs(error_yaw)

        if self.in_flight:
            # Local +Z ("up") axis rotated into the world frame -- this is
            # the 3rd column of the rotation matrix derived from q.
            up_x = 2.0 * (q.x * q.z + q.w * q.y)
            up_y = 2.0 * (q.y * q.z - q.w * q.x)

            # (local_up x world_up); world_up=(0,0,1) so the z-component is
            # always exactly 0 -- this correction never touches yaw.
            err_x = up_y
            err_y = -up_x

            cmd_x = -self.Kp_tilt * err_x + self.Kd_tilt * wx
            cmd_y = -self.Kp_tilt * err_y + self.Kd_tilt * wy

            total_error = math.sqrt(err_x * err_x + err_y * err_y)
        else:
            # On the ground, bleed off any residual tilt command -- legs
            # (not RW) are responsible for ground-contact stability once
            # landed, and landing_controller owns explicit self-righting.
            cmd_x = self.cmd_vel['x'] * 0.9
            cmd_y = self.cmd_vel['y'] * 0.9

        now = self.get_clock().now()
        if self.last_imu_time is None:
            dt = 0.01
        else:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            dt = min(max(dt, 0.0), 0.05)  # guard against clock jumps/dropped ticks
        self.last_imu_time = now

        max_delta = self.max_wheel_accel * dt
        for axis, target in (('x', cmd_x), ('y', cmd_y), ('z', cmd_z)):
            delta = target - self.cmd_vel[axis]
            delta = max(-max_delta, min(max_delta, delta))
            self.cmd_vel[axis] += delta

        max_speed = max(abs(self.cmd_vel['x']), abs(self.cmd_vel['y']), abs(self.cmd_vel['z']))
        if max_speed > self.max_rw_speed:
            scale = self.max_rw_speed / max_speed
            self.cmd_vel['x'] *= scale
            self.cmd_vel['y'] *= scale
            self.cmd_vel['z'] *= scale
            max_speed = self.max_rw_speed
            self.get_logger().warn(f'[{self.robot_name}] Reaction Wheel Saturation — clamped to {self.max_rw_speed:.0f} rad/s', throttle_duration_sec=2.0)

        self.pubs['x'].publish(Float64(data=self.cmd_vel['x']))
        self.pubs['y'].publish(Float64(data=self.cmd_vel['y']))
        self.pubs['z'].publish(Float64(data=self.cmd_vel['z']))

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
