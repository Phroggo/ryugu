"""Retrospective test: would swarm_manager.py's existing az_bias EMA
heading-calibration mechanism (landed_callback) have meaningfully reduced
the azimuth scatter Phase 10 measured, if it had been active during that
test? Phase 10's directional_hop_validation_postfix.py commands
target_yaw directly to hopper_locomotion, bypassing swarm_manager's
az_bias correction entirely -- this replays the same 26 real trials
through the EXACT az_bias update formula, offline, no new sim time.

Assumption (stated, not hidden): each trial's raw offset-from-commanded
(azimuth_i - heading_commanded) is treated as an independent draw from
the same underlying noise process, roughly independent of the specific
commanded angle -- reasonable if the noise source (e.g. leg-release-
timing jitter under near-zero gravity) is roughly isotropic. Under this
assumption, replaying each trial's REAL measured offset through the bias
tracker (using it as the "fresh noise" that would have been added on top
of whatever correction was already learned) is a valid, honest way to
ask "would this correction have helped" without needing new trials.
"""
import json, math

with open("/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/directional_hop_postfix_results.json") as f:
    data = json.load(f)

landed = [r for r in data if "azimuth_deg" in r]
heading_cmd = math.radians(-55.0)

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

bias = 0.0
raw_offsets = []
corrected_offsets = []
bias_history = []

for r in landed:
    az = math.radians(r["azimuth_deg"])
    raw_off = wrap(az - heading_cmd)
    raw_offsets.append(raw_off)

    # Residual error had correction been live: this trial's raw noise,
    # minus whatever bias had already been learned from PRIOR trials.
    corrected_off = wrap(raw_off - bias)
    corrected_offsets.append(corrected_off)

    # Update bias using this trial's raw offset (EMA, alpha=0.5, exact
    # formula from swarm_manager.py's landed_callback).
    new_bias = math.atan2(0.5 * math.sin(bias) + 0.5 * math.sin(raw_off),
                           0.5 * math.cos(bias) + 0.5 * math.cos(raw_off))
    bias_history.append(math.degrees(bias))
    bias = new_bias

def stats(vals_rad):
    degs = [math.degrees(v) for v in vals_rad]
    mean = sum(degs) / len(degs)
    std = math.sqrt(sum((d - mean) ** 2 for d in degs) / len(degs))
    mean_abs = sum(abs(d) for d in degs) / len(degs)
    return mean, std, mean_abs

raw_mean, raw_std, raw_mabs = stats(raw_offsets)
corr_mean, corr_std, corr_mabs = stats(corrected_offsets)

print(f"n={len(landed)} trials")
print(f"RAW offset-from-commanded:       mean={raw_mean:+.1f} std={raw_std:.1f} mean_abs_err={raw_mabs:.1f} (deg)")
print(f"CORRECTED (retrospective bias):  mean={corr_mean:+.1f} std={corr_std:.1f} mean_abs_err={corr_mabs:.1f} (deg)")
print()
print("bias trajectory (deg, value used for each trial i, i.e. learned from trials 1..i-1):")
for i, (b, c) in enumerate(zip(bias_history, corrected_offsets), 1):
    print(f"  trial {i:2d}: bias_used={b:+7.1f}  raw_off={math.degrees(raw_offsets[i-1]):+7.1f}  corrected_off={math.degrees(c):+7.1f}")
print()
print("First-half vs second-half corrected mean_abs_err (does it improve as bias converges?):")
half = len(corrected_offsets) // 2
first_half_mabs = sum(abs(math.degrees(c)) for c in corrected_offsets[:half]) / half
second_half_mabs = sum(abs(math.degrees(c)) for c in corrected_offsets[half:]) / (len(corrected_offsets) - half)
print(f"  first {half}: mean_abs_err={first_half_mabs:.1f} deg")
print(f"  last {len(corrected_offsets)-half}: mean_abs_err={second_half_mabs:.1f} deg")
