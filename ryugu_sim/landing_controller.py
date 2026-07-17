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
from rclpy.qos import qos_profile_sensor_data
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
        #  (b) a FLIGHT/IDLE-state fallback: if altitude stays within a 2 cm
        #      band for 60 s AND velocity stays below 5 mm/s, we are sitting
        #      on something -> CONTACT_DETECTED.
        #      Apex safety must be computed TWO-sided (learned live
        #      2026-07-15: a 2 cm/30 s version of this check fired at the
        #      apex of a slow bounce and declared LANDED midair, because the
        #      band reference can be set just below apex on the way up --
        #      the coast then dwells within the band for up to
        #      2*sqrt(2*band/g) seconds, ~37.5 s for a 2 cm band). The 60 s
        #      window clears that worst case with 1.6x margin, and the
        #      velocity gate adds an independent guard: any hop with a
        #      horizontal component keeps |v| > 5 mm/s through apex and can
        #      never satisfy it. (Band was briefly 1 cm/45 s; relaxed to
        #      2 cm/60 s because real regolith terrain lets a settling robot
        #      creep a few mm as leg contacts shift, breaking a 1 cm band.)
        self.bounce_velocity_threshold = 0.02   # m/s
        self.rest_z_ref = None
        self.rest_z_ticks = 0
        self.REST_Z_BAND = 0.02        # m
        self.REST_Z_TICKS = 6000       # ~60 s @ 100 Hz IMU
        self.REST_VEL_MAX = 0.005      # m/s
        # Velocity-only fallback path for the rest detector. Live deadlock
        # found 2026-07-15: while unconfirmed, the attitude controller's
        # grounded tilt-pump reaction (right at ground-friction capacity)
        # rocks the body a couple of cm, which resets the z-band forever --
        # tilt control prevents the very confirmation that would disarm
        # tilt control. |v| < 5 mm/s sustained for 120 s confirms grounding
        # on its own: a pure-vertical ballistic apex only satisfies the
        # velocity gate for 2*v_gate/g ~= 88 s, so 120 s cannot false-fire
        # in genuine flight (and any hop with a horizontal component never
        # satisfies it at all).
        self.rest_vel_ticks = 0
        self.REST_VEL_TICKS = 12000    # ~120 s @ 100 Hz IMU

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
        # The fold must be RAMPED, not stepped: commanding the stand targets
        # in one step let the position controllers whip the legs at full
        # torque, and the resulting foot/ground impulse threw the 2.5 kg
        # robot clean off the surface at ~0.036 m/s (caught live 2026-07-15
        # -- in micro-gravity that's a ~5 m unplanned ballistic hop, with
        # every safety state disarmed because the state machine said
        # LANDED). Interpolate from the soft-landing pose over ~15 s instead.
        self.STAND_RAMP_TICKS = 1500
        # Liftoff watchdog while LANDED: if the robot is genuinely moving
        # again (velocity above threshold, sustained), it is NOT landed --
        # revert to FLIGHT so contact detection and downstream consumers
        # (attitude tilt control, hopper state) re-arm. Threshold above any
        # grounded rocking residue; 2 s persistence filters transients.
        self.LIFTOFF_VEL = 0.02        # m/s
        self.LIFTOFF_TICKS = 200       # ~2 s @ 100 Hz
        self.liftoff_counter = 0
        # True when CONTACT_DETECTED was entered via the rest-window path
        # (robot already still) rather than an impact spike: in that case
        # the compliant posture is never snapped on (nothing to absorb, and
        # the snap itself kicks the robot airborne), and the stand-up ramp
        # anchors at the flight-retract pose (0,0) instead of the soft pose.
        self.contact_via_rest = False

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

        # Reaction-wheel command publishers + righting arbitration flag
        # (2026-07-16 RW-based self-righting -- see _run_righting_sequence).
        # attitude_controller subscribes to righting_active and stands down
        # completely while it is True: both nodes publishing wheel commands
        # is a silent last-write-wins fight, the same failure class as the
        # stand-pose flood that masked liftoff for five sessions.
        self.rw_pubs = {axis: self.create_publisher(
            Float64, f'/{self.robot_name}/rw_{axis}_joint_cmd_vel', 10)
            for axis in ('x', 'y')}
        self.righting_active_pub = self.create_publisher(
            Bool, f'/{self.robot_name}/righting_active', 10)
        # Wheel speed used for the righting roll. Momentum budget: 150 rad/s
        # x I_w=2.7e-4 = 0.04 N*m*s -> free-body counter-roll ~3 rad/s about
        # the ~0.012 kg*m^2 roll axis; tipping torque needed against Ryugu
        # weight is ~2.9e-5 N*m vs the wheel motor's 0.015 N*m -- a ~500x
        # margin. Kept well under the 982 rad/s clamp so the brake phase is
        # quick and returns net momentum to ~zero (no post-righting bleed
        # kick -- the LANDED->liftoff kick lesson from b876c87).
        self.RIGHTING_WHEEL_SPEED = 150.0
        self.RIGHTING_TIMEOUT_TICKS = 1500  # 15 s per attempt at ~100 Hz

        # ── Subscribers ──
        # Sensor-data QoS (best-effort, shallow queue): under 3-bot load the
        # RELIABLE depth-10 queues back up and the state machine acts on
        # stale measurements (2026-07-16).
        self.create_subscription(
            Imu, f'/{self.robot_name}/imu', self.imu_callback,
            qos_profile_sensor_data)

        self.create_subscription(
            Odometry, f'/{self.robot_name}/odometry', self.odom_callback,
            qos_profile_sensor_data)

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

    def _rest_window_elapsed(self):
        """True once altitude has stayed inside REST_Z_BAND with velocity
        below REST_VEL_MAX for REST_Z_TICKS consecutive IMU ticks -- the
        robot is sitting on something. Resets its own window (and returns
        False) whenever either condition breaks. See the apex-dwell safety
        analysis in __init__ for why the band/velocity/duration values are
        what they are."""
        # Velocity-only path (see REST_VEL_TICKS note in __init__).
        # ALTITUDE-DRIFT GUARD added 2026-07-16: this path previously checked
        # no altitude at all, and free-fall FROM REST stays under the 5 mm/s
        # velocity gate for the first ~44 s (v = g*t) -- once the CPU-
        # starvation fix restored full callback rates, the window elapsed
        # inside that slow-start and confirmed LANDED at 1 m altitude while
        # still falling (live-caught: "landed" at z=5.73 descending at
        # 10 mm/s). A resting robot cannot drift 5 cm; a falling one always
        # does within the window (g*t^2/2 = 5 cm at t=30 s).
        if self.velocity_mag > self.REST_VEL_MAX:
            self.rest_vel_ticks = 0
            self.rest_vel_z_ref = None
        else:
            if getattr(self, 'rest_vel_z_ref', None) is None:
                self.rest_vel_z_ref = self.pos_z
            if abs(self.pos_z - self.rest_vel_z_ref) > 0.05:
                self.rest_vel_ticks = 0
                self.rest_vel_z_ref = self.pos_z
                return False
            self.rest_vel_ticks += 1
            if self.rest_vel_ticks >= self.REST_VEL_TICKS:
                self.rest_vel_ticks = 0
                self.rest_vel_z_ref = None
                self.rest_z_ref = None
                self.rest_z_ticks = 0
                return True

        # Combined z-band + velocity path (faster, 60 s).
        if (self.rest_z_ref is None
                or abs(self.pos_z - self.rest_z_ref) > self.REST_Z_BAND
                or self.velocity_mag > self.REST_VEL_MAX):
            self.rest_z_ref = self.pos_z
            self.rest_z_ticks = 0
            return False
        self.rest_z_ticks += 1
        if self.rest_z_ticks >= self.REST_Z_TICKS:
            self.rest_z_ref = None
            self.rest_z_ticks = 0
            self.rest_vel_ticks = 0
            return True
        return False

    def jump_callback(self, msg):
        """Called when hopper_locomotion initiates a jump."""
        if msg.data:
            self.state = self.FLIGHT
            self.settle_counter = 0
            self.righting_ticks = 0
            self.righting_attempt = 0
            self.rest_z_ref = None
            self.rest_z_ticks = 0
            self.contact_via_rest = False
            self.get_logger().info(f'[{self.robot_name}] 🚀 Jump detected → FLIGHT mode')

    def imu_callback(self, msg):
        # Compute linear acceleration magnitude (excluding gravity component)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)

        if self.state == self.IDLE:
            # Self-arm: a freshly-(re)started node boots IDLE, but the robot
            # may already be airborne (initial spawn descent, or a restart
            # mid-hop). Genuine motion with free-fall accel means flight --
            # arm the landing pipeline instead of watching blind. Conversely,
            # a restart while RESTING must find its way to LANDED (otherwise
            # this node publishes landed=False forever and downstream
            # consumers behave as if airborne) -- the same rest-window
            # detector used in FLIGHT handles that.
            if (accel_mag < self.flight_accel_threshold
                    and self.velocity_mag > self.LIFTOFF_VEL):
                self.state = self.FLIGHT
                self.get_logger().info(
                    f'[{self.robot_name}] 🛫 Airborne while IDLE '
                    f'(v={self.velocity_mag:.3f} m/s, free-fall accel) → FLIGHT mode armed')
            elif self._rest_window_elapsed():
                self.state = self.CONTACT_DETECTED
                self.settle_counter = 0
                # NOTE: no _apply_soft_landing() here -- the robot is already
                # at rest, so there is no impact to absorb, and snapping the
                # legs to the compliant posture at full controller torque was
                # itself kicking the robot off the surface at ~0.025 m/s
                # (caught live by the liftoff watchdog, twice). The ramped
                # stand-up after LANDED handles posture instead.
                self.contact_via_rest = True
                self.get_logger().info(
                    f'[{self.robot_name}] 🎯 Resting while IDLE (altitude static) '
                    f'— confirming ground contact')

        elif self.state == self.FLIGHT:
            # In flight, watch for contact spike
            if accel_mag > self.contact_accel_threshold:
                self.state = self.CONTACT_DETECTED
                self.settle_counter = 0
                self.contact_via_rest = False
                self.get_logger().info(
                    f'[{self.robot_name}] 🎯 Contact detected! '
                    f'accel={accel_mag:.4f} m/s² → hands-off settle '
                    f'(no posture commands at contact)')
            else:
                # Fallback for the resting-reads-as-free-fall trap (see the
                # bounce_velocity_threshold note in __init__): if altitude
                # has stayed inside a 1 cm band for 45 s with velocity below
                # 5 mm/s, we are sitting on the ground even though no impact
                # spike was (re)detected.
                if self._rest_window_elapsed():
                    self.state = self.CONTACT_DETECTED
                    self.settle_counter = 0
                    # No _apply_soft_landing() -- already at rest, nothing to
                    # absorb; snapping legs to the compliant posture was
                    # itself kicking the robot airborne (see IDLE branch).
                    self.contact_via_rest = True
                    self.get_logger().info(
                        f'[{self.robot_name}] 🎯 Altitude static (±{self.REST_Z_BAND*100:.0f} cm '
                        f'for {self.REST_Z_TICKS/100:.0f} s) — grounded without an impact '
                        f'spike, settling in place')

        elif self.state == self.CONTACT_DETECTED:
            # Keep applying soft landing commands -- but only for genuine
            # impact contacts. A rest-path contact (robot already still) must
            # not touch the legs at all: any posture step at rest is itself a
            # launch impulse in micro-gravity.
            # NO posture commands during contact AT ALL (2026-07-15/16).
            # History of this line, because every alternative was tried and
            # live-measured: (1) step to soft posture -> pogo kick, 0.7-0.9 m
            # non-decaying bounces; (2) ~2 s ramped soft posture -> still
            # added energy (in 32 mm/s, out 38 mm/s, 10+ m pogo); (3) zero-
            # stiffness catch mirroring measured joint angles as targets ->
            # WORSE (in 16 mm/s, out 22 mm/s): the bridged joint-state
            # feedback arrives late, the target trails the motion, and the
            # lagged P-torque ends up pumping the rebound instead of damping
            # it. Impact dissipation is now handled where phase lag cannot
            # exist: physical joint <damping> in model.sdf (0.005 -> 0.15).
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
                    elif self._is_badly_tilted(msg):
                        self.get_logger().warn(
                            f'[{self.robot_name}] ⚠️ Settled badly tilted/inverted — '
                            f'initiating RW righting roll')
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
            # Liftoff watchdog: "landed" must remain true in the physics,
            # not just the state machine. If the robot is genuinely moving
            # again (kicked, bounced, disturbed), revert to FLIGHT so
            # contact detection and downstream consumers re-arm.
            if self.velocity_mag > self.LIFTOFF_VEL:
                self.liftoff_counter += 1
                if self.liftoff_counter >= self.LIFTOFF_TICKS:
                    self.state = self.FLIGHT
                    self.liftoff_counter = 0
                    self.rest_z_ref = None
                    self.rest_z_ticks = 0
                    self.get_logger().warn(
                        f'[{self.robot_name}] ⚠️ Liftoff detected while LANDED '
                        f'(v={self.velocity_mag:.3f} m/s sustained) → back to FLIGHT')
                    self.landed_pub.publish(Bool(data=False))
                    return
            else:
                self.liftoff_counter = 0

            # NO post-landing fold (removed 2026-07-16). The "fold to neutral
            # stance" ramp was a launch catapult once the bridge fix made the
            # legs actually obey: first landing at p=1.0/c=0.05 ejected the
            # robot at 0.128 m/s (3x a full jump stroke; ~70 m ballistic arc),
            # caught by the liftoff watchdog. Its original purpose (prevent
            # foot wedge-in) is already handled by the foot-only sphere
            # collisions. After LANDED the legs simply HOLD their landing
            # pose -- the pre-jump crouch re-asserts its own targets anyway.
            self.landed_ticks += 1

        # Publish landed status + righting arbitration flag
        self.landed_pub.publish(Bool(data=(self.state == self.LANDED)))
        self.righting_active_pub.publish(Bool(data=(self.state == self.RIGHTING)))

    def _is_badly_tilted(self, msg):
        """True if the chassis is settled more than ~45 deg from upright
        (u_z < 0.7). Widened from inverted-only (u_z < 0) on 2026-07-17:
        a bot that settles 51 deg tilted passes the old check, then fires
        its next leaned hop 51 deg sideways -- observed live driving bots
        AWAY from their targets. The RW righting roll handles partial
        tilts with the same arbitration and bang-bang structure."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        return (1.0 - 2.0 * (qx * qx + qy * qy)) < 0.7

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
        """Reaction-wheel roll-over (rewritten 2026-07-16).

        The original leg-sweep righting (alternating splay + asymmetric
        sweep) was live-observed failing all 5 attempts on every inverted
        bot after the foot-only-collision change (the sweeps lost their
        ground-hook leverage) and the 0.15 joint damping raise (the sweeps
        lost their dynamic flick). Replaced with the physically dominant
        actuator: spin a lateral reaction wheel at full torque -- the body
        counter-rolls (MINERVA-II's actual mobility principle on the real
        Ryugu); once past horizontal, command the wheel back to zero, which
        brakes the body's roll symmetrically. Net wheel momentum returns to
        ~zero, so handing control back to attitude_controller causes no
        bleed kick. Roll axis/sign alternate per attempt (x+, x-, y+, y-,
        x+) -- a wrong first guess self-corrects on retry.
        """
        self.righting_ticks += 1

        qx, qy = msg.orientation.x, msg.orientation.y
        u_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        axis = 'x' if (self.righting_attempt % 4) < 2 else 'y'
        sign = 1.0 if (self.righting_attempt % 2) == 0 else -1.0

        if self.righting_ticks == 1:
            # Fold legs compact once per attempt so they don't snag the
            # terrain mid-roll. (A posture step is normally a launch impulse
            # in ug -- here the maneuver is deliberately dynamic anyway.)
            for j, pub in self.joint_pubs.items():
                if 'hip' in j:
                    pub.publish(Float64(data=self.stand_hip_target))
                elif 'knee' in j:
                    pub.publish(Float64(data=self.stand_knee_target))
            self.get_logger().info(
                f'[{self.robot_name}] 🔄 RW righting attempt '
                f'{self.righting_attempt + 1}/{self.MAX_RIGHTING_ATTEMPTS}: '
                f'rolling about {axis} ({"+" if sign > 0 else "-"}), '
                f'u_z={u_z:.2f}')

        other = 'y' if axis == 'x' else 'x'
        if u_z < 0.2:
            # Spin phase: full-torque wheel spin-up; body counter-rolls.
            self.rw_pubs[axis].publish(
                Float64(data=sign * self.RIGHTING_WHEEL_SPEED))
            self.rw_pubs[other].publish(Float64(data=0.0))
        else:
            # Brake phase: command the wheel back to zero -- the decel
            # torque stops the body's roll as symmetrically as it started.
            self.rw_pubs[axis].publish(Float64(data=0.0))
            self.rw_pubs[other].publish(Float64(data=0.0))
            if u_z > 0.9:
                self.get_logger().info(
                    f'[{self.robot_name}] ✅ Self-righting successful '
                    f'(attempt {self.righting_attempt + 1}, RW roll about '
                    f'{axis}) — re-confirming contact')
                self.state = self.CONTACT_DETECTED
                self.settle_counter = 0
                return

        if self.righting_ticks >= self.RIGHTING_TIMEOUT_TICKS:
            self.rw_pubs['x'].publish(Float64(data=0.0))
            self.rw_pubs['y'].publish(Float64(data=0.0))
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
                    f'[{self.robot_name}] Still inverted (u_z={u_z:.2f}), retrying '
                    f'with alternate roll axis/sign '
                    f'(attempt {self.righting_attempt + 1}/{self.MAX_RIGHTING_ATTEMPTS})')

    def _apply_soft_landing(self, frac=1.0):
        """Command joints toward the compliant posture, ramped by frac.

        frac ramps 0->1 over the first ~2s of contact (anchored at the
        flight-retract pose (0,0), so frac*target IS the ramp). Added
        2026-07-15 after raising the leg PID gains for launch authority
        (p 0.05->1.0): the old step command from (0,0) to the soft posture
        whipped the now-stiff legs against the ground at every impact,
        kicking the robot 0.7-0.9 m back up in a non-decaying pogo loop
        (observed live: bounce apexes 5.66/5.75/5.74 m, energy being ADDED
        each contact). Same class of bug as the stand-up-ramp fix -- in
        micro-gravity every leg posture step IS a launch impulse."""
        for j, pub in self.joint_pubs.items():
            if 'hip' in j:
                pub.publish(Float64(data=frac * self.soft_hip_target))
            elif 'knee' in j:
                pub.publish(Float64(data=frac * self.soft_knee_target))

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
