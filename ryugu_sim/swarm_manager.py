#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
from nav_msgs.msg import Odometry
import random
import time
import math

# How close (m) the odometry-reported position must be to a target before a
# SAMPLER is considered "arrived" and allowed to deploy its drill. Real jumps
# don't land exactly on the requested target, so this is a tolerance, not zero.
ARRIVAL_RADIUS = 3.0

# Sample tube carousel capacity per Research_Paper.md's mechanical design.
SAMPLE_CAROUSEL_CAPACITY = 3

# Battery drain (%/2s tick) by role, replacing the old flat random.uniform(0.1,0.4)
# applied unconditionally to every agent regardless of what it was doing.
BATTERY_DRAIN_BY_ROLE = {
    "SCOUT": 0.05,        # locomotion standby + passive LIDAR/camera scanning
    "SAMPLER": 0.15,      # legs/RW actively driving travel hops
    "RELAY": 0.08,        # comms transmission draw
    "Unassigned": 0.05,
}
DRILL_ACTIVE_EXTRA_DRAIN = 0.10  # additional draw while the drill motor is loaded
SOLAR_CHARGE_RATE = 0.6          # %/tick while parked in RECHARGE facing the sun

class MetricsLogger:
    def __init__(self):
        self.data = {
            "scenario": "SpaceHopper Swarm: Cooperative Scientific Mission",
            "anomalies_found": 0,
            "samples_extracted": 0,
            "data_transmitted": 0,
            "role_switches": 0
        }
        
    def log_switch(self):
        self.data["role_switches"] += 1

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        self.get_logger().info("🧠 Ryugu Homogeneous Swarm Manager: ONLINE")
        self.metrics = MetricsLogger()
        
        self.agents = ["scout_1"]
        self.state = {
            agent: {
                "role": "Unassigned",
                "battery": random.uniform(85.0, 100.0), # Ni-MH charge
                "target_x": 0.0,
                "target_y": 0.0,
                "has_sample": False,
                "drill_deployed": False,
                # 3-tube carousel: how many samples currently stored, vs.
                # extracted-but-not-yet-stowed (has_sample tracks the latter).
                "sample_count": 0,
                # Real pose feedback (was previously assumed to always be the
                # origin -- see odometry subscription below).
                "pos_x": 0.0,
                "pos_y": 0.0,
                "landed": True,
                # Short human-readable status for the swarm dashboard GUI --
                # kept in sync at each decision point below, not derived
                # after the fact, so it always matches what actually happened
                # this tick.
                "activity": "Booting up...",
                # %/tick, positive while charging (RECHARGE) negative while
                # draining (every other role) -- for the dashboard's
                # charge/discharge rate readout.
                "power_rate": 0.0,
            } for agent in self.agents
        }

        # Publishers for the Drill Actuator
        self.drill_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/cmd_drill', 10)
            for agent in self.agents
        }

        # Status publishers for the swarm dashboard GUI (swarm_gui.py) -- role
        # and battery were previously only ever printed to the log, with no
        # way for an external monitor to read current state.
        self.role_pubs = {
            agent: self.create_publisher(String, f'/{agent}/status_role', 10)
            for agent in self.agents
        }
        self.activity_pubs = {
            agent: self.create_publisher(String, f'/{agent}/status_activity', 10)
            for agent in self.agents
        }
        self.battery_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/status_battery', 10)
            for agent in self.agents
        }
        self.power_rate_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/status_power_rate', 10)
            for agent in self.agents
        }
        
        self.jump_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/jump_target_distance', 10)
            for agent in self.agents
        }
        
        self.yaw_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/target_yaw', 10)
            for agent in self.agents
        }

        # Real position feedback (odometry) and landing status, so this node no
        # longer has to assume every agent sits at the origin.
        for agent in self.agents:
            self.create_subscription(
                Odometry, f'/{agent}/odometry',
                lambda msg, a=agent: self.odom_callback(a, msg), 10)
            self.create_subscription(
                Bool, f'/{agent}/landed',
                lambda msg, a=agent: self.landed_callback(a, msg), 10)

        self.anomaly_queue = []
        self.timer = self.create_timer(2.0, self.swarm_tick)
        self.get_logger().info("Initiating Cooperative Mission Bidding Protocol...")

    def odom_callback(self, agent, msg):
        self.state[agent]["pos_x"] = msg.pose.pose.position.x
        self.state[agent]["pos_y"] = msg.pose.pose.position.y

    def landed_callback(self, agent, msg):
        self.state[agent]["landed"] = msg.data

    def swarm_tick(self):
        # 1. Ni-MH Battery Simulation & Safety Overrides
        # Drain now depends on what the agent is actually doing, instead of a
        # flat random drain applied to every agent regardless of role. RECHARGE
        # previously didn't actually charge anything -- it kept draining at the
        # same rate as every other role, so a critical-battery agent could
        # never climb back above the 80% exit threshold and would be stuck in
        # RECHARGE forever (or drift to negative battery).
        for agent in self.agents:
            role = self.state[agent]["role"]
            if role == "RECHARGE":
                self.state[agent]["battery"] = min(100.0, self.state[agent]["battery"] + SOLAR_CHARGE_RATE)
                self.state[agent]["activity"] = f"Recharging ({self.state[agent]['battery']:.0f}%)..."
                # Positive = charging, for the dashboard's power-rate readout.
                self.state[agent]["power_rate"] = SOLAR_CHARGE_RATE
            else:
                drain = BATTERY_DRAIN_BY_ROLE.get(role, 0.05)
                if role == "SAMPLER" and self.state[agent]["drill_deployed"]:
                    drain += DRILL_ACTIVE_EXTRA_DRAIN
                self.state[agent]["battery"] = max(0.0, self.state[agent]["battery"] - drain)
                # Negative = discharging.
                self.state[agent]["power_rate"] = -drain

            if self.state[agent]["battery"] < 15.0 and role != "RECHARGE":
                self.get_logger().warn(f"🔋 {agent} BMS Alert! Ni-MH cells critical ({self.state[agent]['battery']:.1f}%). Fleeing to sunlight.")
                self.state[agent]["role"] = "RECHARGE"
                self.metrics.log_switch()
            elif self.state[agent]["battery"] > 80.0 and role == "RECHARGE":
                self.state[agent]["role"] = "Unassigned"
                
        # 2. Bidding Protocol (Assigning SCOUT, SAMPLER, RELAY)
        available_agents = [a for a in self.agents if self.state[a]["role"] not in ["RECHARGE", "SAMPLER", "RELAY"]]
        
        # Ensure we always have at least 1 Relay for transmission (only if swarm has multiple agents)
        if len(self.agents) > 1 and not any(self.state[a]["role"] == "RELAY" for a in self.agents) and available_agents:
            relay = available_agents.pop(0)
            self.state[relay]["role"] = "RELAY"
            self.get_logger().info(f"📡 {relay} assuming RELAY role. Climbing to highest elevation.")
            self.metrics.log_switch()

        # The rest become Scouts by default
        for scout in available_agents:
            if self.state[scout]["role"] != "SCOUT":
                self.state[scout]["role"] = "SCOUT"
                self.get_logger().info(f"🛰️ {scout} assuming SCOUT role. Scanning regolith...")
                self.metrics.log_switch()

        # 3. Mission Execution Logic
        for agent in self.agents:
            role = self.state[agent]["role"]
            
            if role == "SCOUT":
                self.state[agent]["activity"] = "Scanning regolith for anomalies..."
                # Simulate Lidar scanning for anomalies
                if random.random() < 0.15:
                    x, y = random.uniform(-50, 50), random.uniform(-50, 50)
                    self.get_logger().info(f"📍 {agent} detected high-value spectral anomaly at [{x:.1f}, {y:.1f}]!")
                    self.anomaly_queue.append((x, y))
                    self.metrics.data["anomalies_found"] += 1
                    
            elif role == "SAMPLER":
                if not self.state[agent]["has_sample"]:
                    # Only drill once the robot has actually landed AND its real
                    # (odometry-reported) position is near the anomaly -- previously
                    # this fired 2s after dispatch regardless of whether the jump
                    # had even completed, let alone arrived at the right spot.
                    dx = self.state[agent]["target_x"] - self.state[agent]["pos_x"]
                    dy = self.state[agent]["target_y"] - self.state[agent]["pos_y"]
                    dist_to_target = math.hypot(dx, dy)
                    if self.state[agent]["landed"] and dist_to_target <= ARRIVAL_RADIUS:
                        self.get_logger().info(f"⛏️ {agent} arrived (within {dist_to_target:.1f}m) — deploying Core Sampler Drill...")
                        self.drill_pubs[agent].publish(Float64(data=-0.1)) # Extend drill down
                        self.state[agent]["drill_deployed"] = True
                        self.state[agent]["has_sample"] = True
                        self.state[agent]["activity"] = "Deploying core sampler drill..."
                        self.metrics.data["samples_extracted"] += 1
                    else:
                        self.get_logger().info(f"🚁 {agent} en route to anomaly ({dist_to_target:.1f}m remaining, landed={self.state[agent]['landed']})...")
                        self.state[agent]["activity"] = f"En route to anomaly ({dist_to_target:.0f}m remaining)"
                else:
                    if self.state[agent]["drill_deployed"]:
                        self.get_logger().info(f"⛏️ {agent} retracting Core Sampler Drill...")
                        self.drill_pubs[agent].publish(Float64(data=0.0)) # Retract
                        self.state[agent]["drill_deployed"] = False
                    self.state[agent]["sample_count"] += 1
                    self.state[agent]["has_sample"] = False
                    self.get_logger().info(
                        f"🧪 {agent} stowing core in carousel tube "
                        f"({self.state[agent]['sample_count']}/{SAMPLE_CAROUSEL_CAPACITY})...")
                    self.state[agent]["activity"] = (
                        f"Stowing core sample ({self.state[agent]['sample_count']}/{SAMPLE_CAROUSEL_CAPACITY})...")

                    carousel_full = self.state[agent]["sample_count"] >= SAMPLE_CAROUSEL_CAPACITY
                    if not carousel_full and self.anomaly_queue:
                        # Room left and more targets queued -- chain directly to the
                        # next anomaly instead of always returning after one sample
                        # (previously every visit unconditionally reverted to SCOUT,
                        # so the paper's 3-tube carousel never had any effect).
                        target = self.anomaly_queue.pop(0)
                        self._dispatch_sampler(agent, target)
                    else:
                        # Carousel full, or nothing left to visit -- hand off to relay.
                        relay = next((a for a in self.agents if self.state[a]["role"] == "RELAY"), None)
                        if relay:
                            self.get_logger().info(
                                f"📤 {agent} broadcasting {self.state[agent]['sample_count']} "
                                f"stored core(s) to {relay}...")
                            self.state[agent]["activity"] = f"Broadcasting samples to {relay}..."
                            self.state[agent]["role"] = "SCOUT"
                            self.state[agent]["sample_count"] = 0
                        elif carousel_full:
                            self.get_logger().warn(
                                f"📦 {agent} carousel full ({SAMPLE_CAROUSEL_CAPACITY}/"
                                f"{SAMPLE_CAROUSEL_CAPACITY}) and no RELAY available -- "
                                f"holding samples, standing by as SCOUT.")
                            self.state[agent]["activity"] = "Carousel full, no relay -- standing by"
                            self.state[agent]["role"] = "SCOUT"
                            self.state[agent]["sample_count"] = 0

            elif role == "RELAY":
                # Simulate transmitting data to orbiter
                if self.metrics.data["samples_extracted"] > self.metrics.data["data_transmitted"]:
                    self.get_logger().info(f"🛰️ {agent} transmitting scientific packet to Hayabusa2 orbiter!")
                    self.state[agent]["activity"] = "Transmitting scientific packet to orbiter..."
                    self.metrics.data["data_transmitted"] += 1
                else:
                    self.state[agent]["activity"] = "Standing by as relay"

        # 4. Sampler Dispatch
        if self.anomaly_queue:
            # Find an available Scout to convert to a Sampler
            for scout in [a for a in self.agents if self.state[a]["role"] == "SCOUT"]:
                target = self.anomaly_queue.pop(0)
                self.state[scout]["role"] = "SAMPLER"
                self._dispatch_sampler(scout, target)
                break

        # 5. Publish status for the swarm dashboard GUI
        for agent in self.agents:
            self.role_pubs[agent].publish(String(data=self.state[agent]["role"]))
            self.activity_pubs[agent].publish(String(data=self.state[agent]["activity"]))
            self.battery_pubs[agent].publish(Float64(data=self.state[agent]["battery"]))
            self.power_rate_pubs[agent].publish(Float64(data=self.state[agent]["power_rate"]))

    def _dispatch_sampler(self, agent, target):
        """Send an agent already in the SAMPLER role toward a target anomaly.
        Shared by the initial SCOUT->SAMPLER dispatch and by carousel-chaining
        (visiting the next anomaly without returning to SCOUT first, as long
        as there's room left in the sample carousel)."""
        self.state[agent]["target_x"] = target[0]
        self.state[agent]["target_y"] = target[1]

        # Real position from odometry, not an assumed origin.
        dx = target[0] - self.state[agent]["pos_x"]
        dy = target[1] - self.state[agent]["pos_y"]
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)

        self.yaw_pubs[agent].publish(Float64(data=yaw))
        self.jump_pubs[agent].publish(Float64(data=dist))

        self.get_logger().info(f"🚀 {agent} accepting bid for SAMPLER. Navigating to [{target[0]:.1f}, {target[1]:.1f}] via {dist:.1f}m jump.")
        self.metrics.log_switch()

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Aborted by User.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
