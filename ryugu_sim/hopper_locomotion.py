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
        self.RECOVERY_AMPLITUDE = 0.3  # stroke fraction for recovery hops

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

        # Stroke-fraction scaling (2026-07-15 launch-kinematics redesign):
        # launch_amplitude is now the FRACTION [0..1] of the full crouch->
        # extension stroke (see CROUCH/LAUNCH below for the geometry). The
        # full stroke extends the legs from a compressed feet-under-body
        # crouch to nearly straight-down, ~0.10 m of vertical travel, with
        # an estimated full-stroke delta-v of V_FULL (provisional -- refine
        # from the first measured hop). Floor 0.2 keeps short hops from
        # degenerating into no-ops; cap 1.0 is the physical stroke limit
        # (full stroke is estimated ~0.08 m/s, ~25% of escape velocity, so
        # even the cap is containment-safe).
        # Measured 2026-07-15 from the first successful full-stroke liftoff
        # (frac=1.0 with p=1.0 leg gains): separation velocity 0.0398 m/s.
        V_FULL = 0.04    # m/s, measured full-stroke delta-v
        self.launch_amplitude = max(0.2, min(1.0, v_req / V_FULL))

        self.get_logger().info(f"[{self.robot_name}] Target distance: {distance:.2f}m. Required Delta-V: {v_req:.4f} m/s. Launch amplitude: {self.launch_amplitude:.2f} rad")
        self.get_logger().info(f"[{self.robot_name}] Initiating Tri-Pedal Jump Sequence!")
        
        self.state = self.CROUCH
        self.state_timer = 0
        self.idle_ticks = 0

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

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
        req = (f'name: "{self.robot_name}", '
               f'position: {{x: {x}, y: {y}, z: {z}}}, '
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
                self.launch_amplitude = self.RECOVERY_AMPLITUDE
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
            self.set_joints(self.CROUCH_HIP, self.CROUCH_KNEE)
            self.state_timer += 1
            # 10 s crouch (was 2 s): standing the 2.5 kg body up from belly-
            # rest onto planted feet at the leg PIDs' soft forces (~0.2 N
            # total) takes 3-5 s of slow rise; cycle-1 telemetry showed
            # IGNITION firing while the body had only risen 3 mm, so the
            # launch stroke started from unloaded, mid-swing legs and
            # delivered ~nothing. 10 s also still covers the RW yaw slew.
            if self.state_timer > 100:
                self.state = self.LAUNCH
                self.state_timer = 0

        elif self.state == self.LAUNCH:
            if self.state_timer == 0:
                self.get_logger().info(f"2. IGNITION (stroke fraction={self.launch_amplitude:.2f})")
                # Re-wake in case the 2s crouch settled into quiescence.
                self._wake_model()
                # Redesigned launch (2026-07-15): extend the legs from the
                # compressed crouch TOWARD straight-down (full stroke: thigh
                # 1.2 -> 0.15 rad from vertical, calf 0.35 -> 0.15), pressing
                # the planted feet into the ground through ~0.10 m of
                # vertical travel. launch_amplitude is the FRACTION of that
                # full stroke, so per-leg travel stays symmetric (equal
                # thrust, no launch torque impulse) at every commanded
                # distance. The previous delta scheme swept the feet inward
                # PAST vertical (a sideways scoop) and delivered ~zero
                # impulse -- direction, not magnitude, was the flaw.
            # Re-assert every tick for the same last-write-wins reason as
            # the CROUCH block above.
            frac = self.launch_amplitude
            hip = self.CROUCH_HIP + frac * (self.EXTEND_HIP - self.CROUCH_HIP)
            knee = self.CROUCH_KNEE + frac * (self.EXTEND_KNEE - self.CROUCH_KNEE)
            self.set_joints(hip, knee)

            self.state_timer += 1
            # 1.0 s launch window: extending the loaded stroke through its
            # ~0.10 m of vertical travel at ~0.33 m/s^2 of body acceleration
            # takes ~0.8 s; retracting earlier truncates the push. Pushing
            # "too long" is harmless in micro-gravity: once the feet leave
            # the ground there is no contact force left to apply.
            if self.state_timer >= 10: # 1.0 second (10 * 0.1s tick)
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
