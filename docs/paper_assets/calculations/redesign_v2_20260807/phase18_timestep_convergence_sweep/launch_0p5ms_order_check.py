#!/usr/bin/env python3
"""Phase 18 follow-up: order-effect control check for 0.5ms's launch-ratio
outlier.

0.5ms's mean ratio (0.2118) came in notably below the tightly-clustered
1/2/4/8ms plateau (0.2184-0.2190) in launch_timestep_convergence.py --
but 0.5ms ran FIRST in that sweep, the exact same "first daemon in the
script" confound the user just caught and confirmed in Phase 17 for a
different experiment. Testing directly rather than writing this into the
report as a genuine timestep effect: run 2 throwaway reps at 1ms first
(consume any cold-start effect on THIS script's daemon), then run the
same n=5 reps at 0.5ms as before. If the low ratio persists with 0.5ms
no longer first, that's evidence for a real timestep effect. If it comes
back up near the 0.218 plateau, that confirms an order artifact, same
as Phase 17.

Run: python3 launch_0p5ms_order_check.py
"""
import json, math, os, subprocess, time
import sys
sys.path.insert(0, os.path.dirname(__file__))
from launch_timestep_convergence import (
    kill_all, start_world, run_one_repeat, make_bridge_yaml, LOG_DIR
)

OUT = f"{LOG_DIR}/launch_0p5ms_order_check_results.json"
WORLD_1MS = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"
WORLD_0P5MS = f"{LOG_DIR}/ryugu_0p5ms.sdf"


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = []

    log("=== throwaway warm-up: 2 reps at 1ms (consume any cold-start effect) ===")
    kill_all()
    start_world(WORLD_1MS, "warmup1ms")
    for rep in range(1, 3):
        r = run_one_repeat("warmup1ms", rep, log)
        results.append(r)

    log("=== 0.5ms, NOT first this time -- n=5 reps ===")
    kill_all()
    start_world(WORLD_0P5MS, "0p5ms_notfirst")
    for rep in range(1, 6):
        r = run_one_repeat("0p5ms_notfirst", rep, log)
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    vals = [r["ratio"] for r in results
            if r["label"] == "0p5ms_notfirst" and r.get("status") == "stabilized"]
    if vals:
        mean = sum(vals) / len(vals)
        log(f"=== 0.5ms (not first) result: n={len(vals)} ratios={[round(v,3) for v in vals]} "
            f"mean={mean:.4f} ===")
        log("Compare vs original 0.5ms-first mean=0.2118 and the 1/2/4/8ms plateau ~0.2184-0.2190")
    else:
        log("=== 0.5ms (not first): no stabilized samples ===")


if __name__ == '__main__':
    main()
