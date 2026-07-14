import subprocess
import time

p = subprocess.Popen(["gz", "topic", "-e", "-t", "/model/scout_1/pose"], stdout=subprocess.PIPE, text=True)

max_z = 0.0
start_time = time.time()
while time.time() - start_time < 15.0:
    line = p.stdout.readline()
    if not line:
        continue
    if "z:" in line:
        try:
            val = float(line.split(":")[1].strip())
            if val > max_z:
                max_z = val
                print(f"Max Z: {max_z}")
        except:
            pass
p.terminate()
print(f"Final max height: {max_z}")
