#!/usr/bin/env python3
import subprocess
import time

print("Triggering jump...")
subprocess.run(["/bin/bash", "-c", "source /home/melvin/ryugu_v2_ws/install/setup.bash && ./trigger_jump.sh 5.0"], cwd="/home/melvin/ryugu_v2_ws/src/ryugu_sim/scripts")

# Wait until apex
print("Waiting for jump apex (approx 2.5s)...")
time.sleep(2.5)

print("Injecting violent tumble using Gazebo topic...")
# We use gz topic to publish a wrench to spin the robot. Wait, Gazebo Harmonic can apply wrench or twist.
# Actually, it's easier to just apply a twist to the base_link via the model/world service or topic?
# Gazebo Harmonic doesn't have an easy twist injection topic by default unless configured.
# Wait, I can just use an SDF modification to jump unevenly.
