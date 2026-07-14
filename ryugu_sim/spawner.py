#!/usr/bin/env python3
import subprocess
import time
import os

AGENTS = [
    ("scout_1", 0.0, 0.5, 6.0)
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
