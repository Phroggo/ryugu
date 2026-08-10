#!/usr/bin/env python3
"""Phase 10 supplementary: top up mu=0.75 from n=16 valid to n=20.

friction_sensitivity_sweep_postfix.py's mu=0.75 bucket lost 4 consecutive
reps (2-5) to a transient gz-transport networking fault -- confirmed via
gz_mu_sweep_postfix.log: "NodeShared::RecvSrvRequest() error sending
response: Host unreachable" logged right at that daemon session's start.
The GZ->ROS bridge went silent for the rest of that daemon's life (reps
2-5 both saw landed=None/speed=None -- the monitor node's subscriptions
never received a single message, not a real physics outcome) and
recovered cleanly the moment the periodic full-daemon-restart (before
rep 6) tore down and recreated the bridge. Not friction-dependent
physics, not touched by the Phase 9 fix (pure networking-layer fault).

Reruns 4 fresh trials (labeled rep21-24 to avoid clobbering the original
rep2-5 logs, kept as evidence) and appends them to the same results file.

Run: python3 friction_mu075_topup.py
"""
import json, time
import friction_sensitivity_sweep_postfix as base

OUT = base.OUT
MU = 0.75
MODEL_URI = "model://spacehopper_mu075"
N_TOPUP = 4


def main():
    def log(msg):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    results = json.load(open(OUT))
    log(f"Loaded {len(results)} existing results. Topping up mu={MU} by {N_TOPUP}.")

    base.kill_all()
    base.start_world(log)
    for i in range(N_TOPUP):
        rep = 21 + i
        r = base.run_one_repeat(MODEL_URI, MU, rep, log)
        r["note"] = "topup for reps 2-5, lost to transient gz-transport fault"
        results.append(r)
        with open(OUT, 'w') as f:
            json.dump(results, f, indent=2)

    log("=== top-up complete ===")
    bucket = [r for r in results if r["mu"] == MU and r["status"] == "stabilized"]
    ratios = [r["ratio"] for r in bucket]
    mean_r = sum(ratios) / len(ratios)
    log(f"mu={MU}: n_stabilized={len(bucket)} mean_ratio={mean_r:.4f} "
        f"ratios={[round(v,3) for v in ratios]}")


if __name__ == '__main__':
    main()
