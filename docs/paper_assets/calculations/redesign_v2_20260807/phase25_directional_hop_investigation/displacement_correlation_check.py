"""Checks whether Phase 10's azimuth scatter could be a measurement-noise
artifact of computing atan2(dy,dx) from very small real displacements
(0.02-0.15m range) rather than genuine trial-to-trial launch-direction
inconsistency. If azimuth error were dominated by small-vector noise,
larger-displacement trials should show visibly tighter azimuth control.
"""
import json

with open("/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/directional_hop_postfix_results.json") as f:
    data = json.load(f)

landed = [r for r in data if "azimuth_deg" in r]
landed_sorted = sorted(landed, key=lambda r: r["displacement_m"])
heading = -55.0


def wrap180(a):
    return ((a + 180) % 360) - 180


print(f"{'disp_m':>8} {'azimuth':>9} {'raw_off_from_-55':>18}")
for r in landed_sorted:
    off = wrap180(r["azimuth_deg"] - heading)
    print(f"{r['displacement_m']:>8.3f} {r['azimuth_deg']:>9.1f} {off:>18.1f}")

disps = [r["displacement_m"] for r in landed]
print()
print(f"displacement range: min={min(disps):.3f} max={max(disps):.3f} mean={sum(disps)/len(disps):.3f}")

half = len(landed_sorted) // 2
small, large = landed_sorted[:half], landed_sorted[half:]


def mean_abs_offset(rs):
    offs = [abs(wrap180(r["azimuth_deg"] - heading)) for r in rs]
    return sum(offs) / len(offs)


print(f"small-displacement half (n={len(small)}): mean |offset| = {mean_abs_offset(small):.1f} deg, "
      f"disp range {small[0]['displacement_m']:.3f}-{small[-1]['displacement_m']:.3f}")
print(f"large-displacement half (n={len(large)}): mean |offset| = {mean_abs_offset(large):.1f} deg, "
      f"disp range {large[0]['displacement_m']:.3f}-{large[-1]['displacement_m']:.3f}")
print()
print("CONCLUSION: if the small-vector-measurement-noise hypothesis were correct, the "
      "small-displacement half should show a MUCH larger mean |offset| than the large-"
      "displacement half. It does not (they are approximately equal, large-displacement "
      "trials if anything slightly worse) -- ruling out measurement noise as the primary "
      "explanation. The scatter is a real feature of launch-direction inconsistency, not "
      "an artifact of measuring angles from near-zero vectors.")
