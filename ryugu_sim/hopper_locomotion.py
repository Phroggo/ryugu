#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import subprocess
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
        # Recovery hops use a fixed medium-slow ramp (see the rate-limited
        # launch redesign in jump_target_callback): ~4 s over the full
        # stroke -> ~0.025 m/s separation -> a few metres of travel.
        self.RECOVERY_RAMP_TICKS = 40
        # Per-jump launch ramp duration in 10 Hz ticks; set by
        # jump_target_callback, consumed by the LAUNCH state.
        self.ramp_ticks = 40
        # FLIGHT leg choreography (2026-07-17): hold the launch extension
        # for CLEARANCE_TICKS after separation (~8 s -> ~0.4 m of ground
        # clearance at ramp speeds), then retract to neutral over
        # RETRACT_RAMP_TICKS (~4 s -> foot speed ~0.03 m/s relative to the
        # body, IMU signature negligible). The old instant retraction at
        # FLIGHT entry jerked the IMU at 0.27-0.57 m/s^2 and was read by
        # the landing controller as a landing impact 2 cm off the ground.
        self.CLEARANCE_TICKS = 80
        self.RETRACT_RAMP_TICKS = 40
        self._ext_hip0 = 0.0
        self._ext_hip12 = 0.0
        self._ext_knee = 0.0

        # Redesigned stroke endpoints (2026-07-15) -- see CROUCH/LAUNCH in
        # tick() for the geometry derivation from the empirical joint-angle
        # mapping (thigh-from-vertical ~= 0.57 + hip; calf = thigh + 0.8 +
        # knee). Crouch: compressed, feet planted under the body. Extend:
        # legs nearly straight down, ~0.11 m of vertical stroke.
        #
        # Corrected 2026-07-15 (second revision): the first stroke geometry
        # (crouch hip 0.63/knee -1.65) put the thighs ~69 deg from vertical,
        # so the leg force was mostly horizontal -- and at Ryugu weight the
        # feet's total friction capacity is only u*m*g ~= 2.9e-4 N, so any
        # horizontal force component makes the feet SLIDE outward instead of
        # lifting the body (crouch stalled at +3 mm indefinitely, live-
        # confirmed twice). These targets keep each foot directly under its
        # hip (zigzag leg: calf angled back inward, foot at r ~= 0.07 m)
        # through the whole stroke so the ground reaction stays vertical.
        # Pass tell: the crouch should visibly raise body z by ~0.15-0.2 m.
        self.CROUCH_HIP = 0.33
        self.CROUCH_KNEE = -2.60
        self.EXTEND_HIP = -0.42
        self.EXTEND_KNEE = -1.10

        # Directional lean (added 2026-07-16). The symmetric vertical stroke
        # that fixed foot grip also removed ALL horizontal range -- the first
        # live 3-bot mission showed corrective re-hops repeating "23.2 m
        # short of target" forever: the bots hop straight up and land in
        # place. Fix: swarm_manager's target_yaw already turns the bot to
        # FACE its target (RW yaw-hold, live-verified), so a modest forward
        # lean in the crouch -- leg 0 (the +x leg) bent further, legs 1/2
        # slightly less -- tilts the thrust vector toward body +x and
        # converts part of the stroke into horizontal velocity
        # (~15 deg lean -> roughly 1/4 of separation-v goes horizontal ->
        # meters of range per hop at Ryugu gravity). The induced launch
        # torque from unequal per-leg travel is real but bounded, and the
        # torque-based attitude controller demonstrably damps larger
        # transients (0.24 rad/s) within seconds of liftoff.
        self.LEAN = 0.25  # rad, hip-space forward-lean differential

        # Latest odometry pose, for the in-place set_pose DART-sleep wake
        # (see _wake_model).
        self.last_pose = None
        self.create_subscription(
            Odometry, f'/{self.robot_name}/odometry', self.odom_callback, 10)
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

        # Attitude error (rad) from attitude_controller: while grounded it
        # reports the YAW error to the commanded heading -- exactly what
        # the crouch must wait on before firing a leaned (directional)
        # stroke. Launching mid-slew scatters hops off-heading (observed
        # live 2026-07-17: bots hopping AWAY from their targets).
        self.attitude_error = 0.0
        self.create_subscription(
            Float64, f'/{self.robot_name}/attitude_error',
            lambda m: setattr(self, 'attitude_error', m.data), 10)

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

        # RATE-limited launch (2026-07-17 redesign, third calibration era).
        # The previous scheme modulated the stroke FRACTION (amplitude) and
        # assumed delta-v scales linearly with it (v = frac * V_FULL). Live
        # measurement killed that assumption: the leg joints are position-
        # PID driven with cmd_max = 0.134 Nm, and even a 27% stroke step
        # keeps the PID torque-saturated through most of its travel, so the
        # joints race at near-terminal speed REGARDLESS of frac. Measured
        # 2026-07-17 (hop_telemetry): a frac=0.27 "9 m" hop separated at
        # ~0.19 m/s horizontal and flew >76 m. Delta-v being amplitude-
        # independent is exactly why re-hop distances oscillated instead of
        # converging: every corrective hop was a 50-150 m ballistic shot
        # inside a +/-49 m walled world.
        #
        # New scheme: ALWAYS use the full stroke geometry (amplitude 1.0,
        # preserving the proven foot-under-hip force direction) and
        # modulate the stroke RATE instead: the LAUNCH state interpolates
        # the joint targets from crouch to extension over ramp_ticks. In
        # micro-gravity the body tracks a slow stroke quasi-statically
        # (PID force budget ~0.2 N >> 2.9e-4 N weight) and separates at
        # roughly the stroke's vertical rate: v_sep ~= EFF_STROKE_H / T.
        # Ramp duration is therefore a direct, LINEAR velocity knob with
        # no dependence on the PID saturation curve (valid for T >= ~1.5 s;
        # below that the PID's own terminal speed takes over, which caps
        # per-hop delta-v around the old wild-launch regime).
        #
        # Ballistics: thrust is tilted ~14 deg off vertical by the LEAN
        # crouch (launch elevation ~76 deg), so range = v^2*sin(2*76deg)/g
        # = SIN2TH * v^2 / g with SIN2TH ~= 0.47 -- NOT the 45-deg-optimal
        # v^2/g the old code assumed. RANGE_CAL is the single empirical
        # trim knob (multiplies v_req; recalibrate from hop_telemetry data
        # if measured ranges drift from commanded distances).
        SIN2TH = 0.47
        EFF_STROKE_H = 0.10   # m, vertical stroke travel crouch->extend
        RANGE_CAL = 1.0       # empirical trim, refine from measured hops
        v_req = RANGE_CAL * math.sqrt(max(distance, 0.5) * g / SIN2TH)
        ramp_T = max(1.5, min(20.0, EFF_STROKE_H / v_req))
        self.ramp_ticks = max(1, round(ramp_T * 10))
        self.launch_amplitude = 1.0

        self.recovery_hop = False
        self.get_logger().info(
            f"[{self.robot_name}] Target distance: {distance:.2f}m. Required Delta-V: "
            f"{v_req:.4f} m/s. Launch ramp: {ramp_T:.1f}s (full stroke)")
        self.get_logger().info(f"[{self.robot_name}] Initiating Tri-Pedal Jump Sequence!")
        
        self.state = self.CROUCH
        self.state_timer = 0
        self.idle_ticks = 0

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        # Stance-quality telemetry for the launch gate: uprightness (world-z
        # component of the body z axis; 1.0 = upright, -1.0 = inverted) and
        # speed (|linear twist|; magnitude is frame-invariant, so the body-
        # frame twist is fine).
        self.last_uz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        v = msg.twist.twist.linear
        self.last_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _stance_ok(self):
        """True when the robot is upright and quiescent enough to deliver a
        predictable directional stroke. Launching from a tilted or still-
        bouncing stance was measured (2026-07-17) to scatter both delta-v
        and azimuth wildly -- e.g. an IGNITION 0.17 s after a bounce-
        interrupted LANDED left at 0.10 m/s on a ~76 deg-off heading."""
        return (getattr(self, 'last_uz', 1.0) > 0.85
                and getattr(self, 'last_speed', 0.0) < 0.012)

    def _wake_model(self):
        """In-place set_pose to wake the model out of DART sleep (gz-sim8
        sleeps a quiescent model even with allow_auto_disable=false in the
        SDF -- proven live 2026-07-15; a sleeping model ignores all joint
        commands, silently zeroing crouch/launch). Fire-and-forget CLI call;
        ~50ms of subprocess spawn is irrelevant at these timescales."""
        if self.last_pose is None:
            self.get_logger().warn(f"[{self.robot_name}] No odometry yet — cannot wake model in place.")
            return
        x, y, z, qx, qy, qz, qw = self.last_pose
        # +0.5 mm lift: an EXACTLY in-place set_pose is a no-op that does
        # not wake DART (found 2026-07-16 — the wake previously "worked"
        # only because odometry lagged under CPU load, making the commanded
        # pose accidentally differ; once the starvation fix landed, odometry
        # became current, the set_pose became truly in-place, and the model
        # slept through entire jump sequences with z bit-identical for
        # minutes). Half a millimetre is imperceptible but guarantees a
        # state change.
        req = (f'name: "{self.robot_name}", '
               f'position: {{x: {x}, y: {y}, z: {z + 0.0005}}}, '
               f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
        subprocess.Popen(
            ['gz', 'service', '-s', '/world/ryugu_world/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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
                self.launch_amplitude = 1.0
                self.ramp_ticks = self.RECOVERY_RAMP_TICKS
                # Recovery hops BYPASS the stance gate: they are the only
                # unstick mechanism for a bot stranded inverted after a
                # failed RW righting (marked landed anyway), and a kick from
                # a bad stance -- however scattered -- re-tumbles the body
                # and gives the righting logic a fresh chance.
                self.recovery_hop = True
                self.state = self.CROUCH
                self.state_timer = 0
                self.idle_ticks = 0
            return

        if self.state == self.CROUCH:
            if self.state_timer == 0:
                self.get_logger().info("1. Crouching (waking physics, planting feet under body)")
                # Wake the model FIRST: gz-sim8's DART integration puts a
                # quiescent model to sleep despite allow_auto_disable=false
                # in model.sdf (proven live 2026-07-15: exact-zero velocity,
                # frozen pose, joint commands ignored -- and an in-place
                # set_pose reliably wakes it, which is why every historical
                # "teleport freed the legs" event worked).
                self._wake_model()
                # Redesigned crouch (2026-07-15): symmetric, feet planted
                # UNDER the body, legs compressed. Empirical mapping from
                # the live in-air pose test: thigh angle from down-vertical
                # ~= 0.57 + hip_joint; calf continues at thigh + 0.8(knee
                # mount) + knee_joint. Crouch: thigh 1.2 rad out, calf 0.35
                # (near-vertical) -> feet 0.195 m below hip at 0.26 m stance
                # radius, chassis belly ~0.2 m clear of the ground. (The old
                # asymmetric forward-lean crouch swept the feet sideways
                # under the body at launch -- a scoop, not a downward press
                # -- and delivered ~zero net impulse.)
            # Re-assert the crouch targets EVERY tick, not just once (found
            # 2026-07-15, fourth session): landing_controller's stand-up
            # ramp publishes to the same joint topics at ~100 Hz, and a
            # freshly-landed robot can receive a jump command while that
            # ramp is still running -- a single one-shot crouch publication
            # is overwritten within ~10 ms and the legs launch from the
            # stand pose instead of the crouch (observed live: IGNITION
            # from an un-crouched stance, ~zero impulse). Last-write-wins
            # at the joint controller, so the active state machine must
            # keep asserting its targets for as long as it owns the legs.
            # Leaned crouch: leg 0 (+x) bent LEAN further; legs 1/2 give
            # back LEAN/2 each so the mean hip angle (and thus stance
            # height) is unchanged -- only the thrust direction tilts.
            self.pubs['hip_joint_0'].publish(Float64(data=self.CROUCH_HIP + self.LEAN))
            self.pubs['knee_joint_0'].publish(Float64(data=self.CROUCH_KNEE))
            for i in (1, 2):
                self.pubs[f'hip_joint_{i}'].publish(Float64(data=self.CROUCH_HIP - self.LEAN / 2))
                self.pubs[f'knee_joint_{i}'].publish(Float64(data=self.CROUCH_KNEE))
            self.state_timer += 1
            # Keep-awake: re-nudge every 2 s through the crouch — a slow
            # crouch can cross DART's quiescence window mid-stroke and the
            # model sleeps through its own IGNITION (2026-07-16).
            if self.state_timer % 20 == 0:
                self._wake_model()
            # 10 s crouch (was 2 s): standing the 2.5 kg body up from belly-
            # rest onto planted feet at the leg PIDs' soft forces (~0.2 N
            # total) takes 3-5 s of slow rise; cycle-1 telemetry showed
            # IGNITION firing while the body had only risen 3 mm, so the
            # launch stroke started from unloaded, mid-swing legs and
            # delivered ~nothing. 10 s also still covers the RW yaw slew.
            # Fire only once (a) the RW yaw slew has actually aligned the
            # body to the commanded heading (error < ~9 deg) AND (b) the
            # stance is upright and quiescent (_stance_ok -- added
            # 2026-07-17 after telemetry showed launches from tilted /
            # still-bouncing stances scattering delta-v and azimuth). 45 s
            # cap so a stuck slew cannot deadlock the mission -- but a cap
            # expiry with a BAD STANCE aborts back to IDLE instead of
            # firing a garbage hop (the swarm re-dispatches every 90 s;
            # wasting a re-hop slot beats teleporting 100 m off-target).
            # NOTE: the gate deliberately does NOT use self.landed -- the
            # crouch stand-up itself rises at ~0.03 m/s, which the landing
            # controller flags as "liftoff while LANDED" (81 occurrences in
            # launch33.log), so the landed flag flickers false mid-crouch.
            stance_ok = self._stance_ok() or getattr(self, 'recovery_hop', False)
            if self.state_timer > 450 and not stance_ok:
                self.get_logger().warn(
                    f"[{self.robot_name}] Aborting hop: stance still bad at crouch "
                    f"timeout (uz={getattr(self, 'last_uz', 1.0):.2f}, "
                    f"speed={getattr(self, 'last_speed', 0.0):.3f} m/s). Back to IDLE.")
                self.set_joints(0.0, 0.0)
                self.state = self.IDLE
                self.state_timer = 0
                self.idle_ticks = 0
                return
            if self.state_timer > 100 and stance_ok and (
                    self.attitude_error < 0.15 or self.state_timer > 450):
                if self.attitude_error >= 0.15:
                    self.get_logger().warn(
                        f"[{self.robot_name}] Launching despite yaw error "
                        f"{self.attitude_error:.2f} rad (alignment timeout)")
                self.state = self.LAUNCH
                self.state_timer = 0

        elif self.state == self.LAUNCH:
            if self.state_timer == 0:
                self.get_logger().info(
                    f"2. IGNITION (stroke fraction={self.launch_amplitude:.2f}, "
                    f"ramp={self.ramp_ticks / 10.0:.1f}s)")
                # Re-wake in case the crouch settled into quiescence.
                self._wake_model()
                # Signal flight AT IGNITION, not at ramp end (2026-07-17).
                # Signalling at ramp end raced the retraction: the landing
                # controller entered FLIGHT in the same tick the legs were
                # step-commanded to neutral, read the retraction jerk
                # (0.27-0.57 m/s^2, 3-7x the 0.08 contact threshold) as a
                # landing impact 2-9 ms later, and confirmed LANDED with the
                # body ~2 cm up at 0.047 m/s -- every ramped hop was scored
                # landed-in-place and the swarm never converged (launch34).
                # The landing controller also blanks contact detection for
                # the launch window after this signal (see its jump_callback).
                self.jump_init_pub.publish(Bool(data=True))
            # Rate-limited stroke (2026-07-17): interpolate the joint
            # targets from the leaned crouch to the full extension over
            # ramp_ticks. The joints track the slowly-moving target (PID
            # never saturates for ramps >= ~1.5 s), the body rises quasi-
            # statically with the stroke, and separation velocity is set by
            # the ramp rate -- see jump_target_callback for the rationale
            # (a stepped target races at PID terminal speed no matter the
            # amplitude; measured 0.19 m/s on a "9 m" hop).
            # Re-assert every tick for the same last-write-wins reason as
            # the CROUCH block above.
            s = min(1.0, (self.state_timer + 1) / self.ramp_ticks)
            frac = self.launch_amplitude * s
            knee = self.CROUCH_KNEE + frac * (self.EXTEND_KNEE - self.CROUCH_KNEE)
            # Each leg extends from ITS OWN leaned crouch angle toward the
            # shared extension target -- the lean carries through the stroke
            # so the thrust direction stays tilted toward body +x.
            hip0_start = self.CROUCH_HIP + self.LEAN
            hip12_start = self.CROUCH_HIP - self.LEAN / 2
            self.pubs['hip_joint_0'].publish(Float64(
                data=hip0_start + frac * (self.EXTEND_HIP + self.LEAN - hip0_start)))
            self.pubs['knee_joint_0'].publish(Float64(data=knee))
            for i in (1, 2):
                self.pubs[f'hip_joint_{i}'].publish(Float64(
                    data=hip12_start + frac * (self.EXTEND_HIP - self.LEAN / 2 - hip12_start)))
                self.pubs[f'knee_joint_{i}'].publish(Float64(data=knee))

            self.state_timer += 1
            # Keep-awake through long (up to 20 s) ramps: a slow stroke can
            # cross DART's quiescence window mid-push just like the crouch.
            if self.state_timer % 20 == 0:
                self._wake_model()
            # Hold the extension briefly past ramp end (0.5 s) so the body
            # separates cleanly at ramp speed, then hand over to FLIGHT.
            # Pushing "too long" is harmless in micro-gravity: once the feet
            # leave the ground there is no contact force left to apply.
            if self.state_timer >= self.ramp_ticks + 5:
                self.get_logger().info(
                    "3. Separation — holding extension for clearance, then slow retract (FLIGHT)")
                # Freeze the extension pose FLIGHT will hold/retract from
                # (same formulas as the ramp at s=1.0).
                frac = self.launch_amplitude
                self._ext_hip0 = (self.CROUCH_HIP + self.LEAN
                                  + frac * (self.EXTEND_HIP + self.LEAN
                                            - (self.CROUCH_HIP + self.LEAN)))
                self._ext_hip12 = (self.CROUCH_HIP - self.LEAN / 2
                                   + frac * (self.EXTEND_HIP - self.LEAN / 2
                                             - (self.CROUCH_HIP - self.LEAN / 2)))
                self._ext_knee = (self.CROUCH_KNEE
                                  + frac * (self.EXTEND_KNEE - self.CROUCH_KNEE))
                self.state = self.FLIGHT
                self.state_timer = 0

        elif self.state == self.FLIGHT:
            # Post-separation leg management (2026-07-17). The old code
            # step-commanded the legs to neutral in the tick FLIGHT began:
            # with separation at only ~0.05 m/s the body was ~2 cm off the
            # ground, the saturated-PID retraction (a) jerked the IMU hard
            # enough to read as a landing impact and (b) could re-catch the
            # ground with the feet. Now: hold the extension until the body
            # has genuinely cleared the surface, then retract as a slow ramp
            # whose IMU signature is negligible. Flights last minutes at
            # Ryugu gravity, so 12 s of leg choreography is nothing.
            self.state_timer += 1
            if self.state_timer <= self.CLEARANCE_TICKS:
                # ~8 s * 0.05 m/s = ~0.4 m of clearance before any retraction.
                self.pubs['hip_joint_0'].publish(Float64(data=self._ext_hip0))
                self.pubs['knee_joint_0'].publish(Float64(data=self._ext_knee))
                for i in (1, 2):
                    self.pubs[f'hip_joint_{i}'].publish(Float64(data=self._ext_hip12))
                    self.pubs[f'knee_joint_{i}'].publish(Float64(data=self._ext_knee))
            elif self.state_timer <= self.CLEARANCE_TICKS + self.RETRACT_RAMP_TICKS:
                r = (self.state_timer - self.CLEARANCE_TICKS) / self.RETRACT_RAMP_TICKS
                self.pubs['hip_joint_0'].publish(Float64(data=self._ext_hip0 * (1.0 - r)))
                self.pubs['knee_joint_0'].publish(Float64(data=self._ext_knee * (1.0 - r)))
                for i in (1, 2):
                    self.pubs[f'hip_joint_{i}'].publish(Float64(data=self._ext_hip12 * (1.0 - r)))
                    self.pubs[f'knee_joint_{i}'].publish(Float64(data=self._ext_knee * (1.0 - r)))
            # After the retract ramp: legs at neutral, publish nothing more;
            # the landing controller owns the rest of the flight.

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
