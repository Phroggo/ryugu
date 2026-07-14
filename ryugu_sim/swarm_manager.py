#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time
import math

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
                "has_sample": False
            } for agent in self.agents
        }
        
        # Publishers for the Drill Actuator
        self.drill_pubs = {
            agent: self.create_publisher(Float64, f'/{agent}/cmd_drill', 10)
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
        
        self.anomaly_queue = []
        self.timer = self.create_timer(2.0, self.swarm_tick)
        self.get_logger().info("Initiating Cooperative Mission Bidding Protocol...")

    def swarm_tick(self):
        # 1. Ni-MH Battery Simulation & Safety Overrides
        for agent in self.agents:
            self.state[agent]["battery"] -= random.uniform(0.1, 0.4)
            if self.state[agent]["battery"] < 15.0 and self.state[agent]["role"] != "RECHARGE":
                self.get_logger().warn(f"🔋 {agent} BMS Alert! Ni-MH cells critical ({self.state[agent]['battery']:.1f}%). Fleeing to sunlight.")
                self.state[agent]["role"] = "RECHARGE"
                self.metrics.log_switch()
            elif self.state[agent]["battery"] > 80.0 and self.state[agent]["role"] == "RECHARGE":
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
                # Simulate Lidar scanning for anomalies
                if random.random() < 0.15:
                    x, y = random.uniform(-50, 50), random.uniform(-50, 50)
                    self.get_logger().info(f"📍 {agent} detected high-value spectral anomaly at [{x:.1f}, {y:.1f}]!")
                    self.anomaly_queue.append((x, y))
                    self.metrics.data["anomalies_found"] += 1
                    
            elif role == "SAMPLER":
                # Simulate arriving at the anomaly and drilling
                if not self.state[agent]["has_sample"]:
                    self.get_logger().info(f"⛏️ {agent} deploying Core Sampler Drill at anomaly site...")
                    drill_msg = Float64(data=-0.1) # Extend drill down
                    self.drill_pubs[agent].publish(drill_msg)
                    self.state[agent]["has_sample"] = True
                    self.metrics.data["samples_extracted"] += 1
                else:
                    self.get_logger().info(f"🧪 {agent} analyzing extracted regolith core...")
                    # Hand off to relay
                    relay = next((a for a in self.agents if self.state[a]["role"] == "RELAY"), None)
                    if relay:
                        self.get_logger().info(f"📤 {agent} broadcasting spectrometer data to {relay}...")
                        self.state[agent]["role"] = "SCOUT" # Done sampling, back to scouting
                        self.state[agent]["has_sample"] = False
                        
            elif role == "RELAY":
                # Simulate transmitting data to orbiter
                if self.metrics.data["samples_extracted"] > self.metrics.data["data_transmitted"]:
                    self.get_logger().info(f"🛰️ {agent} transmitting scientific packet to Hayabusa2 orbiter!")
                    self.metrics.data["data_transmitted"] += 1

        # 4. Sampler Dispatch
        if self.anomaly_queue:
            # Find an available Scout to convert to a Sampler
            for scout in [a for a in self.agents if self.state[a]["role"] == "SCOUT"]:
                target = self.anomaly_queue.pop(0)
                self.state[scout]["role"] = "SAMPLER"
                self.state[scout]["target_x"] = target[0]
                self.state[scout]["target_y"] = target[1]
                
                # Assume robot is currently near origin for this demo
                dist = (target[0]**2 + target[1]**2)**0.5
                yaw = math.atan2(target[1], target[0])
                
                self.yaw_pubs[scout].publish(Float64(data=yaw))
                self.jump_pubs[scout].publish(Float64(data=dist))
                
                self.get_logger().info(f"🚀 {scout} accepting bid for SAMPLER. Navigating to [{target[0]:.1f}, {target[1]:.1f}] via {dist:.1f}m jump.")
                self.metrics.log_switch()
                break

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
