import subprocess
import time
import re

p = subprocess.Popen(["gz", "topic", "-e", "-t", "/world/ryugu_world/pose/info"], stdout=subprocess.PIPE, text=True)

max_z = 0.0
start_time = time.time()
in_scout = False
while time.time() - start_time < 15.0:
    line = p.stdout.readline()
    if not line:
        continue
    
    if "name: \"scout_1\"" in line:
        in_scout = True
    elif "name:" in line:
        in_scout = False
        
    if in_scout and "z:" in line:
        try:
            val = float(line.split(":")[1].strip())
            if val > max_z:
                max_z = val
                print(f"Max Z: {max_z}", flush=True)
        except:
            pass
p.terminate()
print(f"Final max height: {max_z}")
