#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
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

        # SENSOR-DATA QoS (best-effort, shallow queue) for the IMU: with 3
        # bots the RELIABLE depth-10 queues back up under load ("message
        # lost" floods), and a controller acting on stale measurements PUMPS
        # instead of damps -- the delayed-feedback lesson yet again, this
        # time delivered by CPU starvation (2026-07-16).
        self.create_subscription(Imu, f'/{self.robot_name}/imu',
                                 self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, f'/{self.robot_name}/jump_initiated', self.jump_callback, 10)
        self.create_subscription(Bool, f'/{self.robot_name}/landed', self.landed_callback, 10)
        self.create_subscription(Float64, f'/{self.robot_name}/target_yaw', self.target_yaw_callback, 10)

        # Odometry velocity, for the at-rest tilt gate (see imu_callback):
        # tilt correction on a GROUNDED body is a rover drive -- internal
        # torque against ground contact rolls the robot around the terrain
        # and launches it off bumps (observed live 2026-07-16: two freshly
        # landed bots re-kicked themselves to 10 m altitude and km-scale
        # drift). MINERVA-II used exactly this physics ON PURPOSE for
        # mobility; we must not use it by accident.
        self.velocity_mag = 0.0
        self.create_subscription(Odometry, f'/{self.robot_name}/odometry',
                                 self.odom_callback, qos_profile_sensor_data)

        # Stand down entirely while landing_controller runs its RW-based
        # self-righting roll (2026-07-16): two nodes publishing wheel
        # commands is a silent last-write-wins fight.
        self.righting_active = False
        self.create_subscription(
            Bool, f'/{self.robot_name}/righting_active',
            self.righting_callback, 10)

        # Full stand-down during ANY ground contact (2026-07-17): wheel
        # torque against a touching surface is a launch impulse; a bouncing
        # bot was being pumped harder each contact. Unlike the righting
        # handback, cmd_vel is NOT reset here -- the wheels hold their last
        # commanded speed through the contact (constant speed = zero
        # torque) and control resumes seamlessly when contact ends.
        self.ground_contact = False
        self.create_subscription(
            Bool, f'/{self.robot_name}/ground_contact',
            lambda m: setattr(self, 'ground_contact', m.data), 10)

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
        #
        # Rewritten AGAIN 2026-07-15 (torque-based momentum pumping). The
        # previous law commanded wheel VELOCITY proportional to attitude
        # error (cmd = -Kp*err + Kd*w, Kp=300/Kd=60) with a 50 rad/s^2 slew
        # limiter. Two structural problems, both confirmed with live
        # telemetry:
        #
        #  (1) A reaction wheel only exchanges angular momentum with the
        #      body while it ACCELERATES (tau_body = -I_w * dw_wheel/dt).
        #      Once the wheel reaches its commanded speed, torque stops.
        #      Caught live: robot sitting still on the ground with a steady
        #      0.42 rad yaw error, cmd_z pinned at ~126.7 rad/s (=300*0.42),
        #      wheel happily spinning at that speed, ZERO torque flowing,
        #      error never decreasing. In flight the same structure leaves a
        #      residual spin: momentum conservation gives an equilibrium at
        #      w_body = L0 / (I_body + I_wheel*Kd) != 0 -- the measured
        #      persistent -1..-2.3 rad/s yaw spin, matching L0 from the
        #      launch torque impulse.
        #
        #  (2) For yaw specifically, the error wraps at +/-pi. With Kp=300 a
        #      spinning body turns the velocity target into a +/-942 rad/s
        #      sawtooth that a 50 rad/s^2 slew-limited command can never
        #      track; the slew direction dithers with near-zero mean, so net
        #      momentum transfer stalls entirely.
        #
        # Correct structure (standard RW attitude control, e.g. Sidi 1997
        # ch. 7): PD law on attitude produces a desired BODY torque, clipped
        # to the physical wheel-motor budget (0.015 Nm, Maxon EC20 spec per
        # Research_Paper.md SS3.2); wheel acceleration = -tau/I_w; integrate
        # that into the wheel velocity command. Momentum keeps flowing until
        # both rate AND angle error are nulled -- the wheel ends at whatever
        # speed absorbs the disturbance momentum, not at a speed proportional
        # to the remaining error.
        #
        # Gain sizing (vs. whole-robot inertia ~0.025 kg m^2 about z: base
        # 0.009 + legs ~0.012 + panel 0.0008 + wheels ~0.0006 + drill):
        #   w_n = sqrt(K_ang/I) = sqrt(0.02/0.025) ~= 0.89 rad/s
        #   zeta = K_rate / (2*sqrt(K_ang*I)) ~= 1.12  (overdamped -> no
        #   oscillation, per explicit tuning requirement)
        # Torque saturates for |w| > 0.3 rad/s (rate-kill at full torque,
        # ~0.6 rad/s^2 body decel) and for angle errors > 0.75 rad; inside
        # those bounds the loop is smooth and overdamped. A hop's coast
        # phase lasts minutes at Ryugu gravity, so a ~5 s convergence is
        # comfortably fast.
        # Retuned 2026-07-17 (bots visibly slow to stabilize; yaw slews
        # outlasted the pre-jump crouch): K_ang 0.02 -> 0.05 raises the
        # closed-loop bandwidth ~1.6x (wn ~1.8-2 rad/s); K_rate re-sized to
        # keep the response overdamped (zeta ~1.1 at the flight-posture
        # inertia). Torque still clips at the same 15 mNm motor budget, so
        # large errors remain bang-bang at unchanged authority -- only the
        # small-error creep gets faster.
        self.K_ang = 0.05    # N m / rad      (attitude stiffness)
        self.K_rate = 0.066  # N m / (rad/s)  (rate damping)
        self.I_wheel = 0.00027  # kg m^2, RW spin-axis inertia (model.sdf)
        self.tau_max = 0.015    # N m, RW motor torque budget (SS3.2)

        self.target_yaw = 0.0
        # Wheel speed ceiling from the actual motor spec: Maxon EC 20 flat
        # no-load speed ~9380 rpm = 982 rad/s. (Was 1396 rad/s = 13,330 rpm,
        # above anything the cited motor can spin -- corrected 2026-07-15 in
        # the scientific-accuracy pass. H_max = I_w * w_max = 0.00027 * 982
        # ~= 0.265 N m s, still ~30x the worst-case single-leg launch
        # momentum of ~0.0084 N m s from Research_Paper.md SS3.2.)
        self.max_rw_speed = 982.0 # rad/s
        self.in_flight = False
        self.last_imu_time = None

        # Wheel acceleration ceiling, conservative vs the physical
        # tau_max/I_wheel = 55.6 rad/s^2.
        self.max_wheel_accel = 50.0 # rad/s^2

        # Grounded tilt-wheel bleed rate. This CANNOT reuse the full control
        # ceiling: decelerating a wheel dumps its momentum into the body
        # (tau = I_w * a), and on Ryugu the ground can only absorb torque
        # through friction of order mu*N*r ~= (1)(2.85e-4 N)(0.2 m) ~=
        # 5.7e-5 N m. Bleeding at 50 rad/s^2 (tau = 0.0135 N m, ~240x the
        # friction capacity) simply torqued the whole robot instead: caught
        # live 2026-07-15 -- every LANDED confirmation was followed ~2 s
        # later by a 0.04-0.05 m/s liftoff, i.e. the stored x/y wheel
        # momentum (~0.007 N m s) was slam-dunked into a ~0.5 rad/s body
        # spin whose leg/ground reaction launched the robot. 0.2 rad/s^2
        # keeps the bleed reaction torque (5.4e-5 N m) at the friction
        # capacity, so the ground genuinely absorbs the momentum. Wheels
        # holding speed for minutes while parked is fine -- they are just
        # flywheels storing it, exactly as the yaw wheel already does by
        # design.
        self.bleed_wheel_accel = 0.2 # rad/s^2

        # Minimum idle speed for the yaw wheel -- the DART sleep-defeat
        # rotor (see the publish block at the end of imu_callback). 2 rad/s
        # is far above DART's quiescence threshold and dynamically
        # irrelevant (constant speed = zero torque).
        self.IDLE_ROTOR_SPEED = 2.0

        # Attitude deadband (1 deg) + rate deadband. Because this controller
        # integrates torque into wheel speed, ANY persistent unreachable
        # error slowly winds the wheel toward momentum saturation. Terrain
        # is never perfectly level (a tripod resting on regolith holds some
        # small tilt no wheel can remove), and the overnight 2026-07-15 run
        # showed exactly that: a ~0.1 deg residual tilt wound the wheels to
        # the full 1396 rad/s pin over several hours. Inside the deadband
        # the axis holds its stored momentum (dumping it would spin the
        # body); only the error outside the deadband drives new torque.
        # Angle deadband only -- rate damping stays ALWAYS active. (First
        # attempt also deadbanded the rate at 0.005 rad/s; live telemetry
        # showed the body then coasting at exactly 0.005 rad/s in a slow
        # +/-1.2 deg limit cycle between the deadband walls, because inside
        # the band nothing removed momentum. Damping cannot cause windup --
        # it only acts while the body is actually rotating -- so it needs no
        # deadband; only the angle term does.)
        self.err_deadband = 0.017   # rad (~1 deg)

    def _deadband(self, e, db):
        if abs(e) <= db:
            return 0.0
        return e - db * (1.0 if e > 0 else -1.0)

    def odom_callback(self, msg):
        v = msg.twist.twist.linear
        self.velocity_mag = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def jump_callback(self, msg):
        if msg.data:
            self.in_flight = True
            self.get_logger().info(f'[{self.robot_name}] Jump initiated. Flight mode active.')

    def landed_callback(self, msg):
        """Subscribing to /landed at all was the actual bug found 2026-07-14
        (see git history) -- in_flight was previously set True on launch and
        never reset, so tilt correction ran forever post-landing, fighting
        landing_controller's self-righting. Once grounded, landing_controller
        owns orientation correction; this controller only holds yaw.

        Reworked 2026-07-15: /landed is now the single source of truth in
        BOTH directions. Previously in_flight only armed on jump_initiated,
        so any unplanned flight (spawn descent, a bounce, the robot getting
        kicked airborne by a leg maneuver -- all seen live) tumbled with
        tilt correction disarmed. landed=False while grounded-but-settling
        just means a little extra stabilization during touchdown, which is
        harmless (the 1-deg deadband bounds any windup to a few tens of
        rad/s over a settle window)."""
        if msg.data and self.in_flight:
            self.in_flight = False
            self.get_logger().info(f'[{self.robot_name}] Landed — tilt correction stopped, RWs handed off to yaw-hold only.')
        elif not msg.data and not self.in_flight:
            self.in_flight = True
            self.get_logger().info(f'[{self.robot_name}] Airborne (landed=False) — tilt correction armed.')

    def target_yaw_callback(self, msg):
        self.target_yaw = msg.data
        self.get_logger().info(f'[{self.robot_name}] New target yaw received: {self.target_yaw:.2f} rad')

    def righting_callback(self, msg):
        if msg.data and not self.righting_active:
            self.get_logger().info(
                f'[{self.robot_name}] RW righting in progress — attitude '
                f'controller standing down.')
            # Drop our integrated wheel commands so we don't step the wheels
            # when we resume (righting hands them back near zero).
            self.cmd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.righting_active = msg.data

    def imu_callback(self, msg):
        # landing_controller owns the wheels during a righting roll, and
        # NOBODY torques the wheels while the feet/chassis touch ground.
        if self.righting_active or self.ground_contact:
            return

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

        # Desired body torque about z (PD on yaw). Sign: error_yaw = -psi for
        # target 0, so tau_z = -K_ang*psi - K_rate*wz = K_ang*error_yaw
        # - K_rate*wz. Clipped to the physical motor budget below.
        tau_z = (self.K_ang * self._deadband(error_yaw, self.err_deadband)
                 - self.K_rate * wz)

        total_error = abs(error_yaw)

        # At-rest gate (2026-07-16): even with in_flight armed (landed has
        # not confirmed), a body that is essentially motionless must NOT be
        # torqued -- tilt correction against ground contact IS a rover drive
        # in ug (it rolled two freshly-landed bots around the terrain and
        # launched them to 10 m). Tilt runs only while there is real motion
        # to stabilize; a resting-but-tilted body is landing_controller's
        # problem (rest-confirm -> inversion check -> RW righting).
        rate_mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        # Rate threshold lowered 0.15 -> 0.03 (2026-07-17): a 0.1 rad/s
        # tumble is visually violent yet sat below the old gate and went
        # uncorrected. 0.03 is still 30x above resting sensor noise, so the
        # grounded rover-drive hazard the gate exists for stays closed.
        really_moving = (self.velocity_mag > 0.008) or (rate_mag > 0.03)

        if self.in_flight and really_moving:
            # Local +Z ("up") axis rotated into the world frame -- this is
            # the 3rd column of the rotation matrix derived from q.
            up_x = 2.0 * (q.x * q.z + q.w * q.y)
            up_y = 2.0 * (q.y * q.z - q.w * q.x)

            # (local_up x world_up); world_up=(0,0,1) so the z-component is
            # always exactly 0 -- this correction never touches yaw.
            # For a small roll +phi about x, err_x = -sin(phi) ~= -phi, so
            # tau_x = -K_ang*phi - K_rate*wx = K_ang*err_x - K_rate*wx
            # (same sign structure as yaw above).
            err_x = up_y
            err_y = -up_x

            tau_x = (self.K_ang * self._deadband(err_x, self.err_deadband)
                     - self.K_rate * wx)
            tau_y = (self.K_ang * self._deadband(err_y, self.err_deadband)
                     - self.K_rate * wy)

            total_error = math.sqrt(err_x * err_x + err_y * err_y)
        else:
            tau_x = None  # grounded: bleed tilt wheels down instead (below)
            tau_y = None

        now = self.get_clock().now()
        if self.last_imu_time is None:
            dt = 0.01
        else:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            dt = min(max(dt, 0.0), 0.05)  # guard against clock jumps/dropped ticks
        self.last_imu_time = now

        max_delta = self.max_wheel_accel * dt
        bleed_delta = self.bleed_wheel_accel * dt
        for axis, tau in (('x', tau_x), ('y', tau_y), ('z', tau_z)):
            if tau is None:
                # Grounded tilt axes: bleed the wheel toward zero speed
                # GENTLY (see bleed_wheel_accel note in __init__ -- the
                # reaction torque must stay within ground-friction capacity
                # or the bleed itself kicks the robot back off the surface).
                # Legs own ground-contact stability; landing_controller owns
                # explicit self-righting.
                delta = max(-bleed_delta, min(bleed_delta, -self.cmd_vel[axis]))
            else:
                # Wheel acceleration that produces the desired body torque:
                # a_wheel = -tau_body / I_wheel (Newton's third law across
                # the motor). The tau clip IS the 0.015 Nm motor budget.
                tau = max(-self.tau_max, min(self.tau_max, tau))
                delta = (-tau / self.I_wheel) * dt
            delta = max(-max_delta, min(max_delta, delta))
            new_cmd = self.cmd_vel[axis] + delta
            # Per-axis momentum-saturation clamp. (Previously one saturated
            # axis rescaled ALL three commands, needlessly destroying the
            # other two axes' control authority -- the wheels are physically
            # independent.)
            self.cmd_vel[axis] = max(-self.max_rw_speed, min(self.max_rw_speed, new_cmd))

        max_speed = max(abs(self.cmd_vel['x']), abs(self.cmd_vel['y']), abs(self.cmd_vel['z']))
        if max_speed >= self.max_rw_speed:
            self.get_logger().warn(f'[{self.robot_name}] Reaction Wheel momentum saturation — wheel pinned at {self.max_rw_speed:.0f} rad/s', throttle_duration_sec=2.0)

        # SLEEP-DEFEAT ROTOR (2026-07-16): gz-sim8's DART integration sleeps
        # a quiescent model even with allow_auto_disable=false, and a
        # sleeping model ignores ALL joint commands (crouches/launches
        # silently do nothing -- the root of every "legs pinned" mystery).
        # In-place set_pose wake nudges proved unreliable (the model
        # re-slept mid-crouch between 2 s nudges). A skeleton with ANY
        # moving joint can never sleep, so: whenever the yaw wheel would
        # otherwise be near-stopped, idle it at a tiny constant speed.
        # Constant speed = zero motor torque = zero disturbance; stored
        # momentum is 2 x 2.7e-4 = 5.4e-4 N*m*s, 0.2% of wheel capacity.
        # GROUNDED ONLY (corrected 2026-07-16): a flying body is never
        # quiescent, so sleep cannot engage mid-flight -- but the rotor's
        # stored momentum in flight makes the whole body counter-rotate
        # visibly (~0.04 rad/s, below the tilt gate) with the wheel floor
        # blocking the yaw-hold from absorbing it. On the ground, contact
        # friction resists the reaction and the rotor does its real job.
        z_cmd = self.cmd_vel['z']
        if (not self.in_flight) and abs(z_cmd) < self.IDLE_ROTOR_SPEED:
            z_cmd = self.IDLE_ROTOR_SPEED if z_cmd >= 0.0 else -self.IDLE_ROTOR_SPEED

        self.pubs['x'].publish(Float64(data=self.cmd_vel['x']))
        self.pubs['y'].publish(Float64(data=self.cmd_vel['y']))
        self.pubs['z'].publish(Float64(data=z_cmd))

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
