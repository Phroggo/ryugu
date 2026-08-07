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
import subprocess

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
        # Full pose, for the RIGHTING-state wake-model gate (2026-08-05) --
        # see the DART-sleep note near _wake_model below.
        self.last_pose = None

        # V_GAIN calibration diagnostic (2026-07-23). Prior calibration
        # inferred delivered launch velocity indirectly from touchdown
        # position/time minutes-to-hours later, over uneven terrain -- too
        # noisy to fit (two near-identical ramp durations gave ratios of
        # 0.52 and 1.20 of requested velocity). Odometry already computes
        # velocity_mag every tick; sampling it after ignition reads the true
        # launch speed directly -- IF the body has actually reached clean
        # ballistic flight. It often hasn't at a fixed short delay: measured
        # live, velocity kept climbing AND rotating in direction for 7+
        # seconds after all commanded leg motion stopped (a tip-over during
        # the post-separation hold dragging a leg across terrain, not a
        # clean liftoff), so a single fixed-time sample can land mid-tumble.
        # LAUNCH_V_WINDOW gives enough real time for that to either settle
        # into true (constant-velocity) ballistic flight or be caught by the
        # CALIBTIMEOUT path and discarded, rather than silently trusting a
        # mid-chaos snapshot.
        self.LAUNCH_V_WINDOW = 90.0
        self.launch_v_deadline = None

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
        # next crouch starts from a free, repeatable posture. The DELAY/RAMP
        # timing constants that scheduled this fold were removed 2026-07-23
        # as confirmed-dead code (the post-landing fold itself was retired --
        # SS3.4 "No posture is commanded at or after touchdown" -- and the
        # tick-count timers went with it), but the target angles below
        # survive: they were repurposed for the severe-tilt leg-fold step
        # inside the reaction-wheel righting sequence (_run_righting_sequence).
        self.stand_hip_target = 0.9
        self.stand_knee_target = -1.0
        # Compact leg-tuck pose used to shrink the contact base before a
        # righting roll (2026-07-23). A splayed tripod is a wide footprint the
        # roll has to lift over; folding the legs in (hip near crouch, knee
        # fully compressed) makes the body roll more like a cylinder. Reuses
        # the locomotion crouch angles (hopper_locomotion CROUCH_HIP/KNEE).
        self.fold_hip_target = 0.33
        self.fold_knee_target = -2.6
        self.landed_ticks = 0
        # (Historical note, kept for the lesson: an earlier version of this
        # fold was commanded in one step rather than ramped, and the
        # resulting foot/ground impulse threw the 2.5 kg robot clean off the
        # surface at ~0.036 m/s -- caught live 2026-07-15, ~5 m unplanned
        # ballistic hop with every safety state disarmed because the state
        # machine said LANDED. Same Law-3 lesson as every other actuator-
        # against-ground-contact bug in this project. The fold itself, and
        # the ramp that would have applied it, were later retired entirely --
        # see the note above.)
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

        # Self-righting (leg inversion) parameters. An earlier leg-sweep
        # strategy -- alternate a "splay" phase (legs out flat for grip)
        # with an asymmetric "sweep" phase (one lead leg drives a big
        # rotation to roll the chassis, the other two brace) -- was retired
        # (see Research_Paper.md SS3.3): it depended on leg-segment ground
        # leverage that vanished once leg collision geometry was reduced to
        # foot spheres, and on stroke dynamics that joint damping (SS3.4)
        # suppressed. The angle constants and phase-timer that strategy used
        # (righting_splay_hip/knee, righting_sweep_lead/brace_hip/knee,
        # RIGHTING_PHASE_TICKS) were removed 2026-07-23 as confirmed-dead
        # code once the reaction-wheel roll below replaced it entirely --
        # a code-usage audit found each defined but never read again.
        self.righting_ticks = 0
        self.righting_attempt = 0
        self.MAX_RIGHTING_ATTEMPTS = 5

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
        # Ground-contact flag (2026-07-17): attitude control must stand
        # down for the duration of EVERY ground contact, not just righting.
        # A tumbling bot's bounce contacts otherwise convert wheel torque
        # into launch impulses (rover-drive physics), observed live as
        # bounces GROWING 0.025 -> 0.15 m/s with spin climbing to 1.7 rad/s.
        self.contact_pub = self.create_publisher(
            Bool, f'/{self.robot_name}/ground_contact', 10)
        # Wheel speed used for the righting roll. Momentum budget: 150 rad/s
        # x I_w=2.7e-4 = 0.04 N*m*s -> free-body counter-roll ~3 rad/s about
        # the ~0.012 kg*m^2 roll axis; tipping torque needed against Ryugu
        # weight is ~2.9e-5 N*m vs the wheel motor's 0.015 N*m -- a ~500x
        # margin. Kept well under the 982 rad/s clamp so the brake phase is
        # quick and returns net momentum to ~zero (no post-righting bleed
        # kick -- the LANDED->liftoff kick lesson from b876c87).
        # Peak-roll wheel speed (2026-07-23, rev 7: 300 -> 160). History: 150
        # (no leg-tuck) stalled the body on its side; 300 (with leg-tuck, which
        # eases rolling) then OVERSHOT -- the body tumbled past upright and back
        # to inverted in a limit cycle. With the tuck reducing contact
        # resistance, 160 is the sweet spot: enough to clear the on-side hump,
        # little enough that the body does not blow past upright. The RW joint
        # has no effort cap in the SDF, so this is a momentum, not torque, budget.
        self.RIGHTING_WHEEL_SPEED = 160.0
        # Floor of the proportional-taper roll speed (the roll authority tapers
        # from RIGHTING_WHEEL_SPEED far from upright down to this value at the
        # 0.9 success threshold): ~0.18 rad/s of body roll, ground kick below
        # the bounce threshold, so the final approach into upright is gentle.
        # TEMPORARY DIAGNOSTIC (2026-08-05): raised 8.0 -> 20.0 to test
        # whether the taper's floor authority near the target is the real
        # limiter. Real telemetry (severe-tilt trial 7, damping already
        # disabled) showed the body settling into a stable-looking
        # equilibrium at u_z~0.82 with omega decaying toward zero -- not
        # oscillating, genuinely coming to rest short of the 0.9 target --
        # at a commanded speed of only ~29 rad/s (out of a 160 ceiling,
        # purely from the taper formula, damping already off). Testing
        # whether more momentum transfer in the final approach clears it.
        # PHASE 1 MASS/CONSTANT AUDIT (2026-08-07): confirmed DEAD/UNUSED --
        # grep of this file shows this value is read nowhere; the rev-2
        # acceleration-integrated taper (RIGHTING_ACCEL_TAPER,
        # RIGHTING_RATE_DAMP_SCALE/FLOOR below) superseded the old
        # proportional speed-lookup this constant used to feed. Left at its
        # diagnostic value (20.0, not reverted to 8.0) with zero behavioral
        # effect either way. Not touched further here -- see
        # docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/
        # for the full audit; a future cleanup pass should delete this
        # constant rather than "re-tune" a value nothing reads.
        self.GENTLE_RIGHTING_SPEED = 20.0
        self.RIGHTING_TIMEOUT_TICKS = 1500  # 15 s per attempt at ~100 Hz
        # DIAGNOSTIC FINDING (2026-08-06, not applied as a fix): temporarily
        # raised attempts 5->10 and this to 30s/attempt to test whether
        # give-ups while oscillating near upright (u_z repeatedly hitting
        # 0.95-0.99 without holding) were a time/attempt-budget shortfall.
        # Result argues AGAINST that: the extended-budget run didn't
        # converge either -- it gave up even later (t=558s) and then drifted
        # into a THIRD failure mode, a slow non-decaying precession toward
        # MORE inversion (u_z 0 -> -0.73 over ~170s, v flat ~0.03-0.045,
        # never decaying). Leading hypothesis: real 3-axis rigid-body
        # coupling that this file's x/y-only LANDED damper (by design, no
        # z-wheel here) cannot arrest, compounded by attitude_controller
        # (the only node that owns the z-wheel) not running at all in the
        # teleport test harness. Reverted to the original values -- this
        # needs an architectural answer (who damps yaw post-give-up), not a
        # bigger attempt budget.

        # ACCELERATION-INTEGRATED TAPER REDESIGN (2026-08-05, rev 2). The
        # proportional-taper approach above commands a WHEEL SPEED lookup
        # from current u_z error; once the actual wheel catches that (capped,
        # sometimes low near the 0.9 threshold -- see the GENTLE_RIGHTING_SPEED
        # note) target, the low-level joint velocity controller has zero
        # remaining error and applies ~zero further torque, so a body that
        # hasn't yet reached u_z=0.9 can stall indefinitely once its wheel
        # matches whatever speed the lookup happened to return -- exactly the
        # "stable-looking equilibrium at u_z~0.82, omega->0, w~29/160"
        # telemetry in the GENTLE_RIGHTING_SPEED note above (captured with
        # rate damping already disabled, ruling that term out as the cause).
        #
        # rev 1 of this fix ported attitude_controller's torque-integrated PD
        # (alpha = -tau/I_wheel) directly. Live telemetry immediately
        # disproved it: cmd_vel ran away monotonically negative regardless of
        # which way correction was actually needed, and the body tumbled
        # (u_z oscillating -0.7..0.88, omega up to 1.6 rad/s) instead of
        # converging -- one trial ended WORSE than it landed. Root cause: in
        # attitude_controller each wheel's commanded velocity directly IS the
        # torque axis (1:1), so a signed error maps cleanly through Newton's
        # third law. Here there is one MAGNITUDE-only scalar w projected onto
        # the x/y wheels via a separately-tracked, periodically-recomputed
        # direction vector d (_roll_dir) -- the old, empirically-tuned design
        # (peak speed history 150 -> 300 -> 160, always positive) relies on w
        # staying >=0 with ALL directionality carried by d. error=0.9-u_z is
        # always positive in the branch that uses it, so feeding it through a
        # torque-sign reinterpretation just pushed the integrator one
        # direction forever, decoupled from d's actual current heading.
        #
        # Fix: integrate a magnitude-only ACCELERATION instead (never lets
        # cmd_vel go negative), preserving the old formula's sign invariant
        # while still fixing the original stall -- a continuous ramp rather
        # than a static lookup the wheel can catch and stop against. Rate
        # damping scales the acceleration down (floor 0.4x, matching the old
        # damping term) but never reverses it, for the same reason.
        self.max_wheel_accel = 50.0       # rad/s^2, RW motor budget derived
                                           # from tau_max=0.015 N m / I_wheel
                                           # =2.7e-4 kg m^2 (matches
                                           # attitude_controller's hardware
                                           # numbers, ~55.6 rounded down)
        self.RIGHTING_ACCEL_TAPER = 0.6   # error at which accel saturates,
                                           # matches the old frac denominator
        # TIGHTENED (2026-08-06): live telemetry showed omega climbing to
        # 1.7-1.8 rad/s within single attempts (accel *= max(FLOOR,
        # 1.0 - omega/SCALE) -- damping only reaches the floor once
        # omega >= SCALE, and even at the floor 40% authority kept adding
        # energy rather than letting the body settle -- the same overshoot
        # class this file's rev 5/6/7 history already fought once with the
        # old speed-lookup law. Lowering SCALE makes the floor engage at a
        # lower rate (0.8 rad/s instead of 1.5); the floor itself is also
        # lowered so peak authority is weaker once engaged.
        self.RIGHTING_RATE_DAMP_SCALE = 0.8   # was 1.5
        self.RIGHTING_RATE_DAMP_FLOOR = 0.25  # was 0.4
        self._righting_cmd_vel = 0.0     # persistent integrated wheel-speed
                                          # setpoint along the roll axis d;
                                          # invariant: always >= 0
        self._righting_last_time = None  # for dt in the integration above
        self.I_wheel = 0.00027  # kg m^2, RW spin-axis inertia (model.sdf);
                                 # used below by the LANDED-state rate damper

        # LANDED-STATE RATE DAMPING (2026-08-05). Neither existing mechanism
        # damps a LANDED body's rotation: attitude_controller intentionally
        # stands down once landed=True (by design -- see its own
        # landed_callback, "Once grounded, landing_controller owns
        # orientation correction"), and the LANDED tilt watchdog above
        # requires velocity_mag < 0.02 before it trusts a tilt reading,
        # which sustained rotation itself defeats (observed live: a
        # give-up leaving residual angular momentum settled into an
        # undamped, torque-free tumble -- u_z oscillating -0.94..0.99
        # indefinitely, v pinned 0.08-0.5 m/s, zero decay over 150+s of
        # observation, even after the ramped-brake-on-timeout fix removed
        # the wheel's own final kick as a cause). Pure rate damping (tau
        # opposes omega; P = -tau*omega <= 0 always) can only remove
        # energy, never add it, so it is safe to run unconditionally
        # whenever LANDED -- reuses attitude_controller's own proven
        # ground-contact dissipation-only gains rather than inventing new
        # ones. As a side effect, killing the rotation should also reduce
        # velocity_mag (much of the reported 0.1-0.5 m/s during a tumble
        # is very likely a CoM-offset measurement artifact of the fast
        # rotation itself), which lets the existing tilt-watchdog gate
        # above start working again on its own -- no separate fix needed
        # there.
        self.LANDED_DAMP_TAU_CAP = 0.006   # N m, matches attitude_controller
        self.LANDED_DAMP_K_RATE = 0.066    # N m / (rad/s), matches attitude_controller
        self._landed_damp_cmd_vel = {'x': 0.0, 'y': 0.0}
        self._landed_damp_last_time = None
        self._hold_ramp_start_speed = 0.0

        # HOLD-CONFIRM (2026-08-05). Root cause of the succeed-then-redrift
        # oscillation found this week (post-redesign batch trial 14; an
        # independent real ground-contact landing test where two genuinely
        # tilted landings both crossed u_z>0.9 and then drifted BACK to a
        # worse tilt than they started at): "success" was declared from
        # u_z alone, with no check that the body had actually stopped
        # rotating. In Ryugu's near-zero gravity there is nothing to arrest
        # residual angular velocity once wheel torque stops -- a body that
        # crossed 0.9 while still turning just kept turning straight through
        # upright and back down. An instant wheel-speed-to-zero command at
        # that moment also dumps its own deceleration reaction-torque into
        # the body, in the wrong direction, at the worst possible time. This
        # is the same class of failure as two prior wheel-speed retunings
        # (150 -> 300 -> 160, see RIGHTING_WHEEL_SPEED above) that both
        # tried to fix an overshoot-past-upright symptom by adjusting
        # magnitude alone, without ever checking whether the body had
        # actually come to rest -- which is why the same symptom kept
        # resurfacing under different conditions.
        self.RIGHTING_HOLD_RELEASE_UZ = 0.85   # drop below this during hold -> not converged, resume correcting
        self.RIGHTING_HOLD_MAX_RATE = 0.15     # rad/s; body must actually be slow, not just briefly aligned
        self.RIGHTING_HOLD_TICKS = 200         # ~2 s of genuinely held upright before declaring success
        self.RIGHTING_HOLD_RAMP_TICKS = 50     # ~0.5 s ramped brake instead of an instant zero-command kick
        self._righting_confirm_ticks = 0
        # Give-up -> uncommanded-liftoff cascade (2026-08-05, rev 2). Two
        # independent real occurrences the first week this was found (an
        # incidental capture during the C9 rerun, and pre-redesign batch
        # trial 21): exhausting all attempts marks LANDED immediately, and
        # the liftoff watchdog -- which has no idea a give-up just happened
        # -- sees the still-settling body's genuine residual velocity and
        # fires within ~2s, kicking the robot into FLIGHT with zero further
        # correction for the rest of the run.
        #
        # rev 1 used a fixed grace window (8s) to suppress the watchdog
        # right after a give-up. Live telemetry from the accel-taper
        # righting redesign disproved that this is enough: a give-up's
        # residual velocity (measured live at 0.1-0.5 m/s, tightly coupled
        # in time with the preceding righting oscillation -- very likely
        # partly a CoM-offset measurement artifact from fast rotation
        # rather than true free-space translation) took on the order of
        # 90s to passively decay in an otherwise-undisturbed run, far
        # longer than the grace window covered.
        #
        # rev 2: a give-up is already an explicit admission of uncertainty
        # ("Robot may still be physically inverted") -- it is not a
        # confident landing the watchdog should be protecting. Re-arming
        # FLIGHT off the just-abandoned attempt's own leftover momentum
        # only restarts the same losing cycle (RIGHTING -> give-up ->
        # LANDED -> liftoff -> FLIGHT -> ...). Suppress the liftoff
        # watchdog for the remainder of this LANDED dwell instead of on a
        # timer; the separate LANDED tilt watchdog (below) is not fooled
        # by transient post-attempt momentum (it requires velocity_mag
        # < 0.02 sustained) and still catches a genuinely bad rest state.
        # Cleared on the next CONFIDENT landing (i.e. NOT via give-up), so
        # a later, unrelated real landing gets full watchdog protection.
        self._righting_gave_up = False

        # RAMPED BRAKE ON ATTEMPT TIMEOUT (2026-08-05, rev 2). The success
        # path (RIGHTING_HOLD_RAMP_TICKS above) already learned this lesson
        # once: an instant wheel-speed-to-zero command dumps its own
        # deceleration reaction-torque into the body. The attempt-timeout
        # path (every retry boundary, and the final give-up) never got the
        # same fix -- it slammed straight to 0.0. Live telemetry after
        # fixing the give-up/liftoff cascade above exposed this directly:
        # with that cascade no longer masking it, the body kept tumbling
        # (u_z swinging -0.77..0.99, v pinned 0.08-0.5 m/s, zero decay) for
        # 150+ seconds after "giving up", instead of settling. Reuses the
        # same ramp duration and mechanism as the hold-confirm brake, on a
        # dedicated tick counter (NOT righting_ticks, which is already far
        # past RIGHTING_TIMEOUT_TICKS at this point and would saturate the
        # ramp to 1.0 -- i.e. an instant zero again -- on the first tick).
        self._righting_timeout_brake_ticks = 0
        self._timeout_brake_start_speed = 0.0

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
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        if self.launch_v_deadline is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - (self.launch_v_deadline - self.LAUNCH_V_WINDOW)
            last = getattr(self, '_launch_v_last_log', -999)
            if elapsed - last >= 2.0:
                self._launch_v_last_log = elapsed
                self.get_logger().info(
                    f'[{self.robot_name}] 📏 CALIBSERIES t={elapsed:.1f}s '
                    f'v={self.velocity_mag:.5f} m/s (vx={vx:.5f} vy={vy:.5f} vz={vz:.5f})')
                # Stabilization check (2026-07-23): post-"separation" ground
                # contact/tumbling was found to keep perturbing velocity for
                # 7+ seconds after all leg motion stops (magnitude climbing
                # AND direction rotating), well past a fixed sample deadline
                # -- a fixed-time snapshot can land mid-tumble and read a
                # meaningless value. True ballistic flight has (near-)
                # constant velocity at this gravity scale, so require 3
                # consecutive 2s samples where both magnitude and direction
                # hold steady before trusting the reading as the real launch
                # velocity.
                hist = getattr(self, '_launch_v_hist', [])
                hist.append((vx, vy, vz, self.velocity_mag))
                hist = hist[-3:]
                self._launch_v_hist = hist
                # BUG FIX (2026-08-05): only hist[0] was checked for nonzero
                # magnitude, but the dot-product below divides by a[3]*b[3]
                # for the (hist[1], hist[2]) pair too -- either being exactly
                # 0 (e.g. a genuinely stalled sample) raised ZeroDivisionError
                # and crashed the node. Require all three samples nonzero.
                if len(hist) == 3 and all(h[3] > 1e-6 for h in hist):
                    mags = [h[3] for h in hist]
                    mag_spread = (max(mags) - min(mags)) / max(mags)
                    dots = []
                    for a, b in ((hist[0], hist[1]), (hist[1], hist[2])):
                        dot = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (a[3]*b[3])
                        dots.append(max(-1.0, min(1.0, dot)))
                    if mag_spread < 0.05 and min(dots) > 0.995:
                        self.get_logger().info(
                            f'[{self.robot_name}] 📏 CALIBSTABLE t={elapsed:.1f}s '
                            f'v={self.velocity_mag:.5f} m/s (vx={vx:.5f} vy={vy:.5f} '
                            f'vz={vz:.5f}) -- 3 consecutive samples steady')
                        self.launch_v_deadline = None
                        return
            if now >= self.launch_v_deadline:
                self.get_logger().warn(
                    f'[{self.robot_name}] 📏 CALIBTIMEOUT t={elapsed:.1f}s -- '
                    f'never stabilized within {self.LAUNCH_V_WINDOW:.0f}s '
                    f'(last v={self.velocity_mag:.5f} m/s) -- likely tumbling/'
                    f'terrain-snagged launch, discard from calibration')
                self.launch_v_deadline = None

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
        """Called when hopper_locomotion initiates a jump (at IGNITION start
        as of 2026-07-17 — the signal used to arrive at ramp end, racing the
        leg retraction whose IMU jerk then read as a landing impact)."""
        if msg.data:
            self.state = self.FLIGHT
            self.settle_counter = 0
            self.righting_ticks = 0
            self.righting_attempt = 0
            self.rest_z_ref = None
            self.rest_z_ticks = 0
            self.contact_via_rest = False
            # Blank contact-spike detection through the whole launch
            # choreography: ramp (<= 20 s) + 0.5 s hold + 8 s clearance +
            # 4 s slow retract, with margin. Every genuine flight lasts
            # >= ~180 s at Ryugu gravity (shortest commanded hop, 0.5 m),
            # so a 40 s blank can never mask a real landing; it only masks
            # the launch stroke's own actuation transients, which used to
            # score every ramped hop as landed-in-place (launch34).
            self.contact_blank_until = self.get_clock().now().nanoseconds / 1e9 + 40.0
            self.launch_v_deadline = self.get_clock().now().nanoseconds / 1e9 + self.LAUNCH_V_WINDOW
            self._launch_v_last_log = -999
            self._launch_v_hist = []
            self.get_logger().info(f'[{self.robot_name}] 🚀 Jump detected → FLIGHT mode '
                                   f'(contact detection blanked 40 s for launch)')

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
            # In flight, watch for contact spike — but not during the
            # launch-choreography blank window (see jump_callback): the
            # stroke/retract actuation transients otherwise read as impacts.
            in_launch_blank = (self.get_clock().now().nanoseconds / 1e9
                               < getattr(self, 'contact_blank_until', 0.0))
            if in_launch_blank:
                pass
            elif accel_mag > self.contact_accel_threshold:
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
                        self._righting_confirm_ticks = 0
                    else:
                        self.state = self.LANDED
                        self.landed_ticks = 0
                        # Confident landing (not a give-up) -- restore full
                        # liftoff-watchdog protection for this dwell.
                        self._righting_gave_up = False
                        self.get_logger().info(
                            f'[{self.robot_name}] ✅ LANDED — stable contact confirmed')

        elif self.state == self.RIGHTING:
            self._run_righting_sequence(msg)

        elif self.state == self.LANDED:
            # Liftoff watchdog: "landed" must remain true in the physics,
            # not just the state machine. If the robot is genuinely moving
            # again (kicked, bounced, disturbed), revert to FLIGHT so
            # contact detection and downstream consumers re-arm. Suppressed
            # for the remainder of this dwell after a forced righting
            # give-up (see the _righting_gave_up note in __init__) -- a
            # give-up already admits uncertainty about the rest state, so
            # this defers to the LANDED tilt watchdog below instead, which
            # is not fooled by transient post-attempt momentum.
            if self._righting_gave_up:
                pass
            elif self.velocity_mag > self.LIFTOFF_VEL:
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

            # LANDED tilt watchdog (2026-07-18): a bot can DEGRADE into a
            # tilt after confirming LANDED (slow roll on a slope, a nudge
            # from a neighbour's launch) -- previously nothing ever righted
            # it, because righting only triggered on the settle-confirm
            # path, and the crouch stance gate (u_z > 0.85) then aborted
            # every hop forever. Require the tilt to be sustained (~3 s)
            # and the body quiescent so a transient rock or an active
            # crouch wobble cannot trip it.
            uz = 1.0 - 2.0 * (msg.orientation.x ** 2 + msg.orientation.y ** 2)
            if uz < 0.85 and self.velocity_mag < 0.02:
                self.landed_tilt_ticks = getattr(self, 'landed_tilt_ticks', 0) + 1
                if self.landed_tilt_ticks >= 300:
                    self.landed_tilt_ticks = 0
                    self.get_logger().warn(
                        f'[{self.robot_name}] ⚠️ Tilted while LANDED '
                        f'(u_z={uz:.2f} sustained) — initiating RW righting roll')
                    self.state = self.RIGHTING
                    self.righting_ticks = 0
                    self.righting_attempt = 0
                    self._righting_confirm_ticks = 0
                    self.landed_pub.publish(Bool(data=False))
                    return
            else:
                self.landed_tilt_ticks = 0

            # LANDED-STATE RATE DAMPING (see the __init__ note for why this
            # is needed at all): tilt axes only (x/y) -- landing_controller
            # has no z-wheel publisher, so yaw is untouched and stays
            # attitude_controller's job, avoiding any fight over that axis.
            # Pure dissipation (tau opposes omega), safe to run
            # unconditionally.
            wx, wy = msg.angular_velocity.x, msg.angular_velocity.y
            omega_tilt = math.sqrt(wx * wx + wy * wy)
            if omega_tilt > 0.02:
                now_t = self.get_clock().now().nanoseconds / 1e9
                dt = 0.01 if self._landed_damp_last_time is None else min(max(
                    now_t - self._landed_damp_last_time, 0.0), 0.05)
                self._landed_damp_last_time = now_t
                for axis, w in (('x', wx), ('y', wy)):
                    tau = max(-self.LANDED_DAMP_TAU_CAP, min(
                        self.LANDED_DAMP_TAU_CAP, -self.LANDED_DAMP_K_RATE * w))
                    delta = (-tau / self.I_wheel) * dt
                    delta = max(-self.max_wheel_accel * dt,
                                min(self.max_wheel_accel * dt, delta))
                    self._landed_damp_cmd_vel[axis] = max(-self.RIGHTING_WHEEL_SPEED, min(
                        self.RIGHTING_WHEEL_SPEED, self._landed_damp_cmd_vel[axis] + delta))
                    self.rw_pubs[axis].publish(Float64(data=self._landed_damp_cmd_vel[axis]))
            else:
                # BUG FIX (2026-08-05, found same night as introduced): this
                # used to instantly zero the wheel commands here -- the same
                # "instant zero-command kick" mistake already fixed twice
                # elsewhere in this file (righting hold-confirm, righting
                # timeout brake), just reintroduced. omega_tilt is only the
                # x/y PROJECTION of rate; during a complex precessing tumble
                # it can dip through this deadband repeatedly while total
                # rotational energy is still high, and slamming accumulated
                # wheel speed to 0 on every dip both discards braking
                # progress AND dumps a fresh reaction-torque kick into the
                # body -- directly contradicting the whole reason this file
                # moved to acceleration-integrated control today. A wheel
                # holding CONSTANT speed applies zero torque on its own, so
                # there is no need to force it to zero at all: just stop
                # adjusting it below the deadband and let it resume from
                # wherever it left off once the disturbance matters again.
                self._landed_damp_last_time = None

        # Publish landed status + righting arbitration flag
        self.landed_pub.publish(Bool(data=(self.state == self.LANDED)))
        self.righting_active_pub.publish(Bool(data=(self.state == self.RIGHTING)))
        self.contact_pub.publish(Bool(data=(self.state == self.CONTACT_DETECTED)))

    def _is_badly_tilted(self, msg):
        """True if the chassis is settled more than ~32 deg from upright
        (u_z < 0.85). Widened from inverted-only (u_z < 0) on 2026-07-17,
        then from 0.7 to 0.85 on 2026-07-18: the hop stance gate requires
        u_z > 0.85, so a bot settled in the 0.7-0.85 band passed the old
        righting check yet aborted EVERY crouch on the stance gate --
        launch35 logged 149 aborted crouches, most stranded in exactly
        that dead band. The righting success threshold (u_z > 0.9) sits
        safely above the trigger, so no oscillation."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        return (1.0 - 2.0 * (qx * qx + qy * qy)) < 0.85

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

    def _wake_model(self):
        """In-place set_pose to wake the model out of DART sleep, ported
        from hopper_locomotion.py's identical mechanism (2026-08-05).
        gz-sim8 sleeps a quiescent model even with allow_auto_disable=false
        in the SDF; a sleeping model ignores all joint commands, including
        reaction-wheel velocity commands. landing_controller had no wake
        mechanism of its own -- found live during severe-tilt telemetry
        review: a righting attempt froze at a fixed u_z with omega reading
        exactly 0.000 across multiple consecutive samples despite a
        continued nonzero wheel command, then the SAME approach (same
        floor speed, damping still off) landed on a completely different,
        arbitrary frozen value in a repeat run -- consistent with a sleep-
        timing artifact, not a repeatable physical equilibrium.

        UNLIKE hopper's per-tick-count-gated version, this is gated on the
        body already reading as genuinely quiescent (low linear AND
        angular rate) -- not fired on a blind timer -- because during an
        active righting roll the body is SUPPOSED to be rotating, and an
        unconditional periodic teleport would repeatedly destroy that real
        motion as a side effect (the exact bug already found and fixed in
        hopper_locomotion's CROUCH wake call). Only firing when the body
        already looks asleep-by-DART's-own-criteria (quiescent) means this
        can only ever intervene when there's no real motion to destroy.
        """
        if self.last_pose is None:
            return
        x, y, z, qx, qy, qz, qw = self.last_pose
        req = (f'name: "{self.robot_name}", '
               f'position: {{x: {x}, y: {y}, z: {z + 0.0005}}}, '
               f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
        subprocess.Popen(
            ['gz', 'service', '-s', '/world/ryugu_world/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _righting_torque_step(self, error, omega_roll, dt):
        """Acceleration-integrated version of the old proportional-taper
        roll (rev 2, see the __init__ note above for why rev 1's torque/
        Newton's-third-law approach was reverted). error must be >= 0 (0
        at/above the 0.9 success threshold). Ramps self._righting_cmd_vel
        UP at up to max_wheel_accel, never below 0 and never reversed --
        only ever scaled down by rate damping (floor 0.4x) -- so the wheel
        keeps accelerating for as long as error stays nonzero instead of
        stalling once it catches a static target, while preserving the old
        formula's sign invariant (all directionality lives in d, not in the
        sign of this scalar)."""
        accel = self.max_wheel_accel * min(1.0, error / self.RIGHTING_ACCEL_TAPER)
        accel *= max(self.RIGHTING_RATE_DAMP_FLOOR,
                     1.0 - abs(omega_roll) / self.RIGHTING_RATE_DAMP_SCALE)
        delta = accel * dt
        self._righting_cmd_vel = max(0.0, min(
            self.RIGHTING_WHEEL_SPEED, self._righting_cmd_vel + delta))
        return self._righting_cmd_vel

    def _run_righting_sequence(self, msg):
        """Reaction-wheel roll-over (rev 4, 2026-07-23).

        The original leg-sweep righting was retired (it lost ground-hook
        leverage after the foot-only-collision change). Replaced with the
        physically dominant actuator: reaction wheels roll the body upright
        (MINERVA-II's actual mobility principle on the real Ryugu), then brake
        symmetrically at upright so net wheel momentum returns to ~zero and the
        handoff to attitude_controller imparts no bleed kick.

        The roll is direction-aware -- the wheels are driven along the measured
        tilt (re-derived every ~0.5 s so it tracks the body as it rotates),
        strong (300 rad/s) while far from upright and gentle (8 rad/s) for the
        final approach, with a compact leg-tuck at the start to shrink the
        contact base. This replaced an earlier blind axis/sign-alternating spin
        that ran only below u_z<0.2 and stalled bodies on their side (~24%
        success in the 2026-07-23 re-verification); see the inline notes.
        """
        self.righting_ticks += 1

        qx, qy = msg.orientation.x, msg.orientation.y
        u_z = 1.0 - 2.0 * (qx * qx + qy * qy)

        # DART-SLEEP WAKE (2026-08-05, gated -- see _wake_model docstring
        # for why this only fires when the body is already quiescent, not
        # on a blind timer). Checked every ~2s; only acts if BOTH linear
        # and angular rate already read near-zero, which is also exactly
        # DART's own precondition for sleeping the body in the first
        # place -- so this can only intervene when there's no real
        # in-progress motion for it to disturb.
        if self.righting_ticks % 200 == 0:
            current_omega = math.sqrt(msg.angular_velocity.x ** 2
                                       + msg.angular_velocity.y ** 2
                                       + msg.angular_velocity.z ** 2)
            if self.velocity_mag < 0.001 and current_omega < 0.001:
                self.get_logger().warn(
                    f'[{self.robot_name}] Body quiescent mid-righting '
                    f'(v={self.velocity_mag:.5f}, omega={current_omega:.5f}) '
                    f'-- possible DART sleep, waking in place.')
                self._wake_model()

        # DIRECTION-AWARE ROLL. A 2026-07-23 re-verification measured the old
        # maneuver at only 24% success: bodies rolled up to u_z~0.2 (on their
        # side) and stalled across all 5 attempts, because a strong spin handed
        # off to a too-weak gentle roll exactly at the on-side stall point and
        # could not lift the body over its contact base, while blind
        # axis/sign alternation wasted attempts on the wrong direction. The
        # roll now (a) aims continuously along the measured tilt, re-derived
        # every ~0.5 s so it tracks the body as it rotates (between per-tick
        # chatter at the degenerate vertical crossing and a stale fixed
        # direction), and (b) uses one continuous proportional-taper speed
        # (below) with no stall-inducing discontinuity. Measured after the fix:
        # the on-side stall is gone and mild-to-moderate landing tilts recover
        # reliably; a body forced to a perfect full inversion and wedged
        # against terrain is the residual hard case (see SS3.3 in the paper).
        if self.righting_ticks == 1 or self.righting_ticks % 50 == 0:
            qz, qw = msg.orientation.z, msg.orientation.w
            up_x = 2.0 * (qx * qz + qw * qy)
            up_y = 2.0 * (qy * qz - qw * qx)
            n = math.hypot(up_x, up_y)
            if n > 1e-6:
                self._roll_dir = (-up_y / n, up_x / n)

        if self.righting_ticks == 1:
            self._right_legs_ext = None   # force a leg-pose publish this attempt
            self._righting_cmd_vel = 0.0
            self._righting_last_time = None
            self.get_logger().info(
                f'[{self.robot_name}] 🔄 RW righting attempt '
                f'{self.righting_attempt + 1}/{self.MAX_RIGHTING_ATTEMPTS}: '
                f'proportional roll, u_z={u_z:.2f}')

        # TWO-PHASE LEG MANAGEMENT (rev 8, 2026-07-23). Tuck the legs compact
        # while far from upright (a splayed tripod is a wide contact base the
        # roll must lift over -- tucking makes the body roll like a cylinder,
        # which took success 24% -> 67%), then DEPLOY the tripod once the body
        # is near upright. Without the deploy, rev 7 rolled the body up to
        # ~u_z 0.70 and stalled there, balancing on an edge with no feet to
        # settle onto; extending the legs gives it the stable upright
        # equilibrium (feet down) to fall into. Published only on the
        # tuck<->deploy transition, so it is not a per-tick leg impulse.
        want_ext = u_z > 0.55
        if want_ext is not self._right_legs_ext:
            self._right_legs_ext = want_ext
            hip = self.stand_hip_target if want_ext else self.fold_hip_target
            knee = self.stand_knee_target if want_ext else self.fold_knee_target
            for j, pub in self.joint_pubs.items():
                if 'hip' in j:
                    pub.publish(Float64(data=hip))
                elif 'knee' in j:
                    pub.publish(Float64(data=knee))

        d = getattr(self, '_roll_dir', None)
        if d is None:
            return  # body momentarily level in the x-y projection; wait a tick

        # RAMPED BRAKE IN PROGRESS (see the _righting_timeout_brake_ticks
        # note in __init__): an attempt just timed out and the wheel speed
        # is being ramped down to zero before the retry/give-up bookkeeping
        # runs. Takes full control of publishing for these few ticks --
        # skip the normal approach/hold-confirm control law entirely.
        if self._righting_timeout_brake_ticks > 0:
            ramp = min(1.0, self._righting_timeout_brake_ticks / self.RIGHTING_HOLD_RAMP_TICKS)
            w = self._timeout_brake_start_speed * (1.0 - ramp)
            self._righting_cmd_vel = w
            self.rw_pubs['x'].publish(Float64(data=w * d[0]))
            self.rw_pubs['y'].publish(Float64(data=w * d[1]))
            self._righting_timeout_brake_ticks += 1
            if self._righting_timeout_brake_ticks > self.RIGHTING_HOLD_RAMP_TICKS:
                self._righting_timeout_brake_ticks = 0
                self._finalize_righting_timeout(u_z)
            return

        # BUG FIX (2026-08-05, same day as the hold-confirm redesign below):
        # the entry gate was originally just "if u_z < 0.9", re-evaluated
        # fresh every tick -- so the instant u_z dipped even slightly below
        # 0.9 during the hold-confirm window (which happens constantly from
        # ordinary residual motion), control fell straight back into the
        # approach-phase branch below, which unconditionally zeroed the
        # confirm counter. The intended hysteresis (tolerate a dip down to
        # RIGHTING_HOLD_RELEASE_UZ=0.85 without failing) never actually ran;
        # real behavior required an unbroken hold with zero tolerance, far
        # stricter than designed. Track "already confirming" as persistent
        # state so a brief dip above 0.85 doesn't kick it back to square one.
        in_hold_confirm = self._righting_confirm_ticks > 0

        now_t = self.get_clock().now().nanoseconds / 1e9
        dt = 0.01 if self._righting_last_time is None else min(max(
            now_t - self._righting_last_time, 0.0), 0.05)
        self._righting_last_time = now_t

        if u_z < 0.9 and not in_hold_confirm:
            # ACCELERATION-INTEGRATED TAPER ROLL (2026-08-05 rev 2) -- see
            # the __init__ note above (max_wheel_accel et al.) for why: a
            # wheel only reaction-torques the body while ACCELERATING, so a
            # static speed target the wheel can catch up to and hold is a
            # structural stall risk, independent of and in addition to the
            # overshoot/damping concerns the taper approach was tuned
            # around. error is clamped to >=0 by construction (this branch
            # only runs while u_z<0.9); _righting_torque_step supplies the
            # same taper (full authority far away, decaying as u_z
            # approaches 0.9) and the same rate damping as before, just
            # integrated as an always-nonnegative acceleration instead of a
            # per-tick speed lookup.
            omega = math.sqrt(msg.angular_velocity.x ** 2
                              + msg.angular_velocity.y ** 2
                              + msg.angular_velocity.z ** 2)
            omega_roll = msg.angular_velocity.x * d[0] + msg.angular_velocity.y * d[1]
            error = 0.9 - u_z  # > 0 by construction in this branch
            w = self._righting_torque_step(error, omega_roll, dt)
            self.rw_pubs['x'].publish(Float64(data=w * d[0]))
            self.rw_pubs['y'].publish(Float64(data=w * d[1]))
            self._righting_confirm_ticks = 0
            if self.righting_ticks % 200 == 0:
                self.get_logger().info(
                    f'[{self.robot_name}] 📐 RIGHTTRACE attempt='
                    f'{self.righting_attempt + 1} u_z={u_z:.4f} w={w:.0f} '
                    f'omega={omega:.3f} omega_roll={omega_roll:.3f} '
                    f'dir=({d[0]:.2f},{d[1]:.2f})')
        else:
            # HOLD-CONFIRM (2026-08-05): u_z>0.9 alone is not "recovered" --
            # see the RIGHTING_HOLD_* comment in __init__ for why. Ramp the
            # brake instead of snapping it to zero, and only declare success
            # once the body has genuinely HELD near-upright at a low rate for
            # RIGHTING_HOLD_TICKS, not just touched it for one sample.
            omega = math.sqrt(msg.angular_velocity.x ** 2
                              + msg.angular_velocity.y ** 2
                              + msg.angular_velocity.z ** 2)
            if u_z < self.RIGHTING_HOLD_RELEASE_UZ or omega > self.RIGHTING_HOLD_MAX_RATE:
                # Didn't actually hold -- resume active correction. Same
                # attempt: righting_attempt/righting_ticks are untouched,
                # this is a continuation, not a fresh try. Uses the same
                # torque-integrated PD step as the approach phase (2026-08-05)
                # rather than a separate speed-lookup formula: this is
                # physically the same wheel and the same control problem
                # (still below the success bar), so reusing the shared
                # integrated state avoids a discontinuous command jump at
                # the branch boundary -- the exact failure class the
                # HOLD-CONFIRM redesign above already had to fix once
                # ("An instant wheel-speed-to-zero command...dumps its own
                # deceleration reaction-torque into the body").
                self._righting_confirm_ticks = 0
                omega_roll = msg.angular_velocity.x * d[0] + msg.angular_velocity.y * d[1]
                error = 0.9 - u_z
                w = self._righting_torque_step(error, omega_roll, dt)
                self.rw_pubs['x'].publish(Float64(data=w * d[0]))
                self.rw_pubs['y'].publish(Float64(data=w * d[1]))
            else:
                # Ramp the ACTUAL current integrated speed to zero (captured
                # once, at the moment the hold begins) rather than a fixed
                # GENTLE_RIGHTING_SPEED constant, which could now disagree
                # with the real wheel state and cause the same discontinuity
                # this branch exists to avoid.
                if self._righting_confirm_ticks == 0:
                    self._hold_ramp_start_speed = self._righting_cmd_vel
                self._righting_confirm_ticks += 1
                ramp = min(1.0, self._righting_confirm_ticks / self.RIGHTING_HOLD_RAMP_TICKS)
                w = self._hold_ramp_start_speed * (1.0 - ramp)
                self._righting_cmd_vel = w
                self.rw_pubs['x'].publish(Float64(data=w * d[0]))
                self.rw_pubs['y'].publish(Float64(data=w * d[1]))
                if self._righting_confirm_ticks >= self.RIGHTING_HOLD_TICKS:
                    self.rw_pubs['x'].publish(Float64(data=0.0))
                    self.rw_pubs['y'].publish(Float64(data=0.0))
                    self._righting_cmd_vel = 0.0
                    self.get_logger().info(
                        f'[{self.robot_name}] ✅ Self-righting successful '
                        f'(attempt {self.righting_attempt + 1}) — held '
                        f'u_z>{self.RIGHTING_HOLD_RELEASE_UZ} for '
                        f'{self.RIGHTING_HOLD_TICKS/100:.1f}s — re-confirming contact')
                    self._righting_confirm_ticks = 0
                    self.state = self.CONTACT_DETECTED
                    self.settle_counter = 0
                    return

        if self.righting_ticks >= self.RIGHTING_TIMEOUT_TICKS:
            # Ramped brake first (see the brake-tick intercept above) --
            # _finalize_righting_timeout does the actual retry/give-up
            # bookkeeping once the wheel speed has been brought to zero
            # gently instead of slammed there.
            self._timeout_brake_start_speed = self._righting_cmd_vel
            self._righting_timeout_brake_ticks = 1

    def _finalize_righting_timeout(self, u_z):
        """Runs once the post-timeout ramped brake has brought the wheel
        speed to zero. Either retries with a fresh attempt or, once
        MAX_RIGHTING_ATTEMPTS is exhausted, gives up."""
        self.rw_pubs['x'].publish(Float64(data=0.0))
        self.rw_pubs['y'].publish(Float64(data=0.0))
        self._righting_cmd_vel = 0.0
        self.righting_attempt += 1
        self.righting_ticks = 0
        # BUG FIX (2026-08-05): confirm_ticks was never reset here, so a
        # stale nonzero value from an interrupted hold-confirm in the
        # attempt that just timed out could carry into the next attempt,
        # making a brand-new attempt (new axis/sign) start as if already
        # mid-hold from leftover bookkeeping.
        self._righting_confirm_ticks = 0
        if self.righting_attempt >= self.MAX_RIGHTING_ATTEMPTS:
            self.get_logger().error(
                f'[{self.robot_name}] ❌ Self-righting failed after '
                f'{self.MAX_RIGHTING_ATTEMPTS} attempts — giving up, marking '
                f'LANDED anyway so downstream logic (e.g. SAMPLER dispatch) '
                f'does not hang forever. Robot may still be physically inverted.')
            self.state = self.LANDED
            self.landed_ticks = 0
            # GIVE-UP -> LIFTOFF SUPPRESSION (see the _righting_gave_up
            # note in __init__): stops the liftoff watchdog from
            # mistaking this just-abandoned attempt's own residual
            # momentum for a fresh disturbance. Cleared on the next
            # confident landing.
            self.liftoff_counter = 0
            self._righting_gave_up = True
        else:
            self.get_logger().warn(
                f'[{self.robot_name}] Still inverted (u_z={u_z:.2f}), retrying '
                f'with alternate roll axis/sign '
                f'(attempt {self.righting_attempt + 1}/{self.MAX_RIGHTING_ATTEMPTS})')

    # _apply_soft_landing() (ramped active-compliance posture at contact) was
    # removed 2026-07-23 as confirmed-dead code: a usage audit found zero
    # call sites left, only comments explaining why each candidate call site
    # deliberately doesn't call it. This is the actively-controlled-
    # compliance approach Research_Paper.md SS3.4.1 documents as measured and
    # rejected (it added energy at contact in all three variants tried) --
    # the code was fully superseded by passive joint damping but the dead
    # method and its four supporting constants (soft_hip_target,
    # soft_knee_target, soft_p_gain, soft_d_gain) were never deleted.

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
