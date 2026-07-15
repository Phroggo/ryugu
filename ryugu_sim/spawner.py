#!/usr/bin/env python3
import subprocess
import time
import os

# Full 3-bot swarm (scaled up 2026-07-16; previously scout_1 only). Spawn
# points spread ~8-10 m apart so descent drift can't collide them, all at
# z=6 above local terrain (~4.8) for the standard settle-in descent. One
# spacehopper SDF serves all three: every plugin topic (joint controllers,
# odometry, joint_state, IMU) derives from the spawned ENTITY name at
# runtime, not from anything baked into model.sdf.
AGENTS = [
    ("scout_1", 0.0, 0.5, 6.0),
    ("scout_2", 8.0, -5.0, 6.0),
    ("scout_3", -8.0, -5.0, 6.0),
]

def main():
    print("🚀 Initiating Swarm Deployment Sequence...")
    for name, x, y, z in AGENTS:
        cmd = f"gz service -s /world/ryugu_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 --req \"sdf_filename: 'model://spacehopper', name: '{name}', pose {{ position {{ x: {x} y: {y} z: {z} }} }}\""
        print(f"Deploying {name} at [{x}, {y}, {z}]...")
        os.system(cmd)
        time.sleep(1)
        
    print("✅ Swarm Deployment Complete.")

if __name__ == '__main__':
    main()
