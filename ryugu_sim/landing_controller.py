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
from nav_msgs.msg import Odometry
import sys
import math

class LandingController(Node):
    # ── States ──
    IDLE = 0
    FLIGHT = 1
    CONTACT_DETECTED = 2
    SETTLING = 3
    LANDED = 4
    RIGHTING = 5

    STATE_NAMES = {0: "IDLE", 1: "FLIGHT", 2: "CONTACT", 3: "SETTLING", 4: "LANDED", 5: "RIGHTING"}

    def __init__(self, robot_name):
        super().__init__(f'landing_controller_{robot_name}')
        self.robot_name = robot_name
        self.state = self.IDLE
        self.settle_counter = 0

        self.get_logger().info(f'[{self.robot_name}] Landing Controller: ONLINE')

        # ── Tunable parameters ──
        # Contact detection: acceleration magnitude threshold (m/s²)
        # Found 2026-07-14: this was set to 0.02, but reaction-wheel torque
        # reactions and leg-joint PID corrections routinely produce transient
        # linear accelerations in that same range -- they're driven by motor
        # torque limits (up to 134 mNm for legs), which have nothing to do
        # with how weak gravity is here. Live-caught: a "contact detected"
        # event fired at accel=0.0204 m/s^2 while the robot was still ~4.8m
        # in the air, immediately confirming LANDED (and downstream, gating
        # SAMPLER drill deployment) despite being nowhere near the ground.
        # Raised to 0.08, matching this file's own original comment that a
        # genuine landing impact "typically produces >0.05 m/s^2" -- the old
        # 0.02 threshold was already inconsistent with that reasoning.
        self.contact_accel_threshold = 0.08  # m/s²
        self.settle_duration_ticks = 200     # ~2s at 100Hz IMU
        self.flight_accel_threshold = 0.005  # below this = free-fall = flight

        # Second line of defense against the same false-positive class:
        # require the robot's actual velocity (from odometry) to be small
        # before confirming LANDED, not just a sustained accel reading. A
        # genuinely still-flying robot has non-trivial velocity almost all
        # the time (except a brief instant at apex), so combined with the
        # accel-threshold fix above, a coincidental false accept needs both
        # conditions to align, which is far less likely than either alone.
        self.landed_velocity_threshold = 0.01  # m/s
        self.velocity_mag = 0.0
        self.pos_z = 0.0

        # Found 2026-07-15 (overnight run): a robot RESTING on the ground in
        # micro-gravity reads IMU proper acceleration ~= g ~= 0.0001 m/s^2,
        # BELOW flight_accel_threshold, i.e. indistinguishable from free-fall
        # by accelerometer alone. The old bounce check ("accel < flight
        # threshold -> back to FLIGHT") therefore fired on a robot that had
        # already settled, and once back in FLIGHT there was no new impact
        # spike to re-detect (it was already at rest), so the state machine
        # hung in FLIGHT forever: hopper_locomotion never returned to IDLE
        # (all jump commands ignored) and attitude_controller kept in-flight
        # tilt control active on the ground, winding its reaction wheels up
        # to full 1396 rad/s momentum saturation over the following hours.
        # Two-part fix:
        #  (a) a "bounce" now additionally requires genuine velocity
        #      (bounce_velocity_threshold) -- a free-fall accel reading plus
        #      near-zero velocity means RESTING, and settling continues;
        #  (b) a FLIGHT-state fallback: if altitude stays within a 1 cm band
        #      for 45 s AND velocity stays below 5 mm/s, we are sitting on
        #      something -> CONTACT_DETECTED.
        #      Apex safety must be computed TWO-sided (learned live
        #      2026-07-15: a 2 cm/30 s version of this check fired at the
        #      apex of a slow bounce and declared LANDED midair, because the
        #      band reference can be set just below apex on the way up --
        #      the coast then dwells within the band for up to
        #      2*sqrt(2*band/g) seconds, ~37 s for a 2 cm band). For a 1 cm
        #      band the worst-case apex dwell is 2*sqrt(0.02/0.000114)
        #      ~= 26.5 s < 45 s. The velocity gate adds an independent
        #      guard: any hop with a horizontal component keeps
        #      |v| > 5 mm/s through apex and can never satisfy it.
        self.bounce_velocity_threshold = 0.02   # m/s
        self.rest_z_ref = None
        self.rest_z_ticks = 0
        self.REST_Z_BAND = 0.01        # m
        self.REST_Z_TICKS = 4500       # ~45 s @ 100 Hz IMU
        self.REST_VEL_MAX = 0.005      # m/s

        # Soft landing joint targets (slight crouch to absorb impact)
        self.soft_hip_target = 0.3    # gentle splay
        self.soft_knee_target = -0.4  # slight bend
        self.soft_p_gain = 0.3        # weak spring — compliant
        self.soft_d_gain = 0.5        # strong damper — energy absorption

        # Post-landing stand-up. Found 2026-07-15 (live, definitive): leaving
        # the legs in the splayed soft-landing posture while resting lets the
        # 2 cm foot spheres wedge into heightmap crevices under the joint
        # controllers' sustained push; the 0.134 Nm leg motors then cannot
        # move the legs AT ALL (verified: hip commands echoed on the gz
        # topic, link poses bit-identical before/after, zero body reaction
        # -- yet the same commands moved the legs violently the instant the
        # robot was lifted clear of the terrain). A jammed robot cannot
        # crouch, so every jump silently produced zero thrust. Fix: once
        # LANDED is confirmed, slowly fold the legs up to a neutral stance
        # (feet unloaded, tucked under the body, chassis resting on its
        # belly) so nothing is pressed into the terrain between hops and the
        # next crouch starts from a free, repeatable posture.
        self.stand_hip_target = 0.9
        self.stand_knee_target = -1.0
        self.STAND_DELAY_TICKS = 300   # wait ~3 s after LANDED before folding
        self.landed_ticks = 0

        # Self-righting (leg inversion) parameters. Joint range is +/-3.14 rad
        # (full rotation), which physically supports flipping the body over.
        # Strategy: alternate a "splay" phase (legs out flat for ground grip,
        # whatever surface is currently "up" for an inverted robot) with an
        # asymmetric "sweep" phase (one lead leg drives hard through a big
        # rotation to roll the chassis, the other two brace) -- a symmetric
        # push on all three legs would just cancel out and not roll anything.
        # The lead leg rotates between attempts so a maneuver that doesn't
        # work once isn't just repeated identically forever.
        self.righting_ticks = 0
        self.righting_attempt = 0
        self.RIGHTING_PHASE_TICKS = 150       # ~1.5s per phase @ 100Hz IMU
        self.MAX_RIGHTING_ATTEMPTS = 5
        self.righting_splay_hip = 1.4
        self.righting_splay_knee = -1.4
        self.righting_sweep_lead_hip = -2.8
        self.righting_sweep_lead_knee = 2.2
        self.righting_sweep_brace_hip = 0.3
        self.righting_sweep_brace_knee = -1.0

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

        self.create_subscription(
            Odometry, f'/{self.robot_name}/odometry', self.odom_callback, 10)

        # Flight trigger (hopper_locomotion tells us when a jump starts)
        self.create_subscription(
            Bool, f'/{self.robot_name}/jump_initiated', self.jump_callback, 10)

        # Periodic status log
        self.create_timer(5.0, self.log_status)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.velocity_mag = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.pos_z = msg.pose.pose.position.z

    def jump_callback(self, msg):
        """Called when hopper_locomotion initiates a jump."""
        if msg.data:
            self.state = self.FLIGHT
            self.settle_counter = 0
            self.righting_ticks = 0
            self.righting_attempt = 0
            self.rest_z_ref = None
            self.rest_z_ticks = 0
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
            else:
                # Fallback for the resting-reads-as-free-fall trap (see the
                # bounce_velocity_threshold note in __init__): if altitude
                # has stayed inside a 2 cm band for ~30 s, we are sitting on
                # the ground even though no impact spike was (re)detected.
                if (self.rest_z_ref is None
                        or abs(self.pos_z - self.rest_z_ref) > self.REST_Z_BAND
                        or self.velocity_mag > self.REST_VEL_MAX):
                    self.rest_z_ref = self.pos_z
                    self.rest_z_ticks = 0
                else:
                    self.rest_z_ticks += 1
                    if self.rest_z_ticks >= self.REST_Z_TICKS:
                        self.state = self.CONTACT_DETECTED
                        self.settle_counter = 0
                        self.rest_z_ref = None
                        self.rest_z_ticks = 0
                        self.get_logger().info(
                            f'[{self.robot_name}] 🎯 Altitude static (±{self.REST_Z_BAND*100:.0f} cm '
                            f'for {self.REST_Z_TICKS/100:.0f} s) — grounded without an impact '
                            f'spike, switching to compliant mode')
                        self._apply_soft_landing()

        elif self.state == self.CONTACT_DETECTED:
            # Keep applying soft landing commands
            self._apply_soft_landing()
            self.settle_counter += 1

            # A genuine bounce means we are back in free-fall AND genuinely
            # moving. Free-fall accel with near-zero velocity is a robot at
            # REST (in micro-gravity a resting IMU reads ~g ~= 0.0001 m/s^2,
            # below flight_accel_threshold), so that case falls through to
            # the settle logic below instead of bouncing back to FLIGHT.
            if (accel_mag < self.flight_accel_threshold
                    and self.velocity_mag > self.bounce_velocity_threshold):
                if self.settle_counter > 50:  # gave it enough time
                    self.state = self.FLIGHT
                    self.rest_z_ref = None
                    self.rest_z_ticks = 0
                    self.get_logger().info(
                        f'[{self.robot_name}] ⚠️ Bounce detected (v={self.velocity_mag:.4f} m/s) '
                        f'→ back to FLIGHT')
            else:
                # Sustained contact -- also require real velocity to actually
                # be low (see landed_velocity_threshold note above) before
                # confirming, not just a sustained accel reading.
                if self.settle_counter >= self.settle_duration_ticks:
                    if self.velocity_mag > self.landed_velocity_threshold:
                        self.get_logger().warn(
                            f'[{self.robot_name}] Sustained contact accel but velocity '
                            f'still {self.velocity_mag:.4f} m/s — not actually landed, '
                            f'resetting settle counter (likely a false accel trigger, '
                            f'e.g. RW/leg motor reaction torque).',
                            throttle_duration_sec=2.0)
                        self.settle_counter = 0
                    elif self._is_inverted(msg):
                        self.get_logger().warn(
                            f'[{self.robot_name}] ⚠️ Landed INVERTED (upside-down) — '
                            f'initiating self-righting maneuver')
                        self.state = self.RIGHTING
                        self.righting_ticks = 0
                        self.righting_attempt = 0
                    else:
                        self.state = self.LANDED
                        self.landed_ticks = 0
                        self.get_logger().info(
                            f'[{self.robot_name}] ✅ LANDED — stable contact confirmed')

        elif self.state == self.RIGHTING:
            self._run_righting_sequence(msg)

        elif self.state == self.LANDED:
            # Stay landed until next jump; after a short pause, fold the legs
            # up to the neutral stance so the feet stop bearing load and
            # can't wedge into the terrain (see stand_hip_target note above).
            self.landed_ticks += 1
            if self.landed_ticks == self.STAND_DELAY_TICKS:
                self.get_logger().info(
                    f'[{self.robot_name}] 🧍 Folding legs to neutral stance '
                    f'(unload feet, prevent terrain wedge-in)')
            if self.landed_ticks >= self.STAND_DELAY_TICKS:
                for j, pub in self.joint_pubs.items():
                    if 'hip' in j:
                        pub.publish(Float64(data=self.stand_hip_target))
                    elif 'knee' in j:
                        pub.publish(Float64(data=self.stand_knee_target))

        # Publish landed status
        self.landed_pub.publish(Bool(data=(self.state == self.LANDED)))

    def _is_inverted(self, msg):
        """True if the chassis +Z axis is currently pointing mostly downward
        (upside-down landing), derived from IMU orientation quaternion.
        Standard quaternion-rotation formula: rotating the local +Z axis
        (0,0,1) by orientation q=(x,y,z,w) gives a world-frame Z component
        of 1 - 2*(qx^2 + qy^2). Positive = chassis-up (normal); negative =
        chassis-down (inverted). Independent of qz/qw (yaw has no bearing
        on whether the robot is right-side-up).
        """
        qx = msg.orientation.x
        qy = msg.orientation.y
        world_up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        return world_up_z < 0.0

    def _run_righting_sequence(self, msg):
        """Alternates a splay phase (grip) and an asymmetric sweep phase
        (roll) to flip the chassis over. Re-checks orientation after each
        full splay+sweep cycle; retries with a rotated lead leg on failure,
        up to MAX_RIGHTING_ATTEMPTS before giving up.
        """
        self.righting_ticks += 1
        phase_ticks = self.RIGHTING_PHASE_TICKS
        in_sweep_phase = (self.righting_ticks // phase_ticks) % 2 == 1
        lead = self.righting_attempt % 3

        if not in_sweep_phase:
            for j, pub in self.joint_pubs.items():
                if 'hip' in j:
                    pub.publish(Float64(data=self.righting_splay_hip))
                elif 'knee' in j:
                    pub.publish(Float64(data=self.righting_splay_knee))
        else:
            for i in range(3):
                hip_j, knee_j = f'hip_joint_{i}', f'knee_joint_{i}'
                if i == lead:
                    self.joint_pubs[hip_j].publish(Float64(data=self.righting_sweep_lead_hip))
                    self.joint_pubs[knee_j].publish(Float64(data=self.righting_sweep_lead_knee))
                else:
                    self.joint_pubs[hip_j].publish(Float64(data=self.righting_sweep_brace_hip))
                    self.joint_pubs[knee_j].publish(Float64(data=self.righting_sweep_brace_knee))

        if self.righting_ticks >= phase_ticks * 2:
            if not self._is_inverted(msg):
                self.get_logger().info(
                    f'[{self.robot_name}] ✅ Self-righting successful '
                    f'(attempt {self.righting_attempt + 1}) — re-confirming contact')
                self.state = self.CONTACT_DETECTED
                self.settle_counter = 0
            else:
                self.righting_attempt += 1
                self.righting_ticks = 0
                if self.righting_attempt >= self.MAX_RIGHTING_ATTEMPTS:
                    self.get_logger().error(
                        f'[{self.robot_name}] ❌ Self-righting failed after '
                        f'{self.MAX_RIGHTING_ATTEMPTS} attempts — giving up, marking '
                        f'LANDED anyway so downstream logic (e.g. SAMPLER dispatch) '
                        f'does not hang forever. Robot may still be physically inverted.')
                    self.state = self.LANDED
                    self.landed_ticks = 0
                else:
                    self.get_logger().warn(
                        f'[{self.robot_name}] Still inverted, retrying self-righting '
                        f'(attempt {self.righting_attempt + 1}/{self.MAX_RIGHTING_ATTEMPTS}, '
                        f'lead leg {self.righting_attempt % 3})')

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
