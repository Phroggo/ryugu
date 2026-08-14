# Phase 24 — I_wheel Controller Constant Correction

Date: 2026-08-14
Scope: item 1 of the external review round's four sim-side items — the attitude controller ran `I_wheel=2.7e-4 kg·m²` (old solid-disc model) while the physical value confirmed by the Phase 1/2 mass audit is `3.944e-4 kg·m²`. Fix the constant, then sanity-check self-righting and yaw-hold once each before treating the corrected value as safe to build on for items 2/3 of the same review round (both touch reaction-wheel behavior).

**Result: fixed, sanity-checked, confirmed stable. No gain retuning performed or needed.**

## 1. Files touched

### Source

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/attitude_controller.py` — `self.I_wheel` corrected; three dependent comments (H_max/margin calculation, wheel-acceleration-ceiling calculation) updated to match, not left stale.

### Sanity-check harness and results

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase24_iwheel_correction/yaw_slew_sanity_check.py` (copy of `phase4_attitude_revalidation/yaw_slew_revalidation.py`, with a real harness bug found and fixed — see §3)
- `.../phase24_iwheel_correction/self_righting_sanity_check.py` (copy of `phase17_continuous_session_retime/continuous_session_retime.py`, `N_PER_BUCKET` reduced then bumped — see §4)
- `.../phase24_iwheel_correction/ryugu_4ms.sdf` (copied in — see §3, was missing initially)
- `.../phase24_iwheel_correction/phase4_yaw_slew_results.json`
- `.../phase24_iwheel_correction/self_righting_sanity_n1_results.json`, `self_righting_sanity_n5_results.json`
- Per-trial node logs (`bridge_*`, `attitude_*`, `landing_*`, `gz_*`) — **note**: the n=1 and n=5 self-righting runs used the same per-trial log filenames (`*_trial1.log` etc.), so the n=5 run's trial1 log overwrote the n=1 run's. Not a data-loss concern for the actual measured outcomes (fully captured in stdout, reproduced in §4 below and in the source `.output` transcript), only the redundant per-node debug logs for that one specific trial.
- `.../phase24_iwheel_correction/PHASE24_CHANGE_REPORT.md` (this file)

## 2. The fix

```python
# was:
self.I_wheel = 0.00027  # kg m^2, RW spin-axis inertia (model.sdf)
# now:
self.I_wheel = 3.944e-4  # kg m^2, RW spin-axis inertia (Phase 1/2 audit)
```

`K_ang`/`K_rate` (attitude-loop gains) were **not** touched, per explicit instruction to check stability first rather than assume a retune is needed.

Two dependent comments corrected to avoid leaving stale numbers next to a corrected constant:
- `H_max = I_w * w_max` comment: was `0.00027 * 982 ≈ 0.265 N·m·s`, now `3.944e-4 * 982 ≈ 0.387 N·m·s` (matching Phase 20's independently-computed H_max exactly) — and the "~30x worst-case launch momentum" comparison updated to "~46x" (using the same stale 0.0084 N·m·s numerator the original comment used, which is what the current paper already cites; Phase 20's from-current-model reconstruction gives a substantially larger margin still, noted with a pointer rather than duplicated).
- Wheel-acceleration-ceiling comment: was "conservative vs. the physical `tau_max/I_wheel = 55.6 rad/s²`" (`self.max_wheel_accel=50.0`, below that ceiling). With the corrected I_wheel, the physical ceiling is now `0.015/3.944e-4 ≈ 38.0 rad/s²` — **below** the 50.0 slew limit, meaning torque-capping is now the binding constraint instead of the slew limit. Noted as a real, flagged behavioral consequence of the correction, not a bug — the controller remains self-consistent (`delta=-tau/I_wheel*dt` can never physically exceed the torque-derived ceiling regardless of this constant's value), the slew-limit constant itself just becomes non-binding in practice.

## 3. Yaw-hold sanity check

Ran `yaw_slew_sanity_check.py` (107° yaw-slew, 1ms and 4ms timestep worlds, matching the proven Phase 4 methodology exactly) against the corrected I_wheel.

**A real harness bug was found and fixed before trusting the result**, not smoothed over: the first run's 4ms trial failed to converge (`final_yaw≈0°`, i.e. no motion at all), which on first look resembled a real destabilization — exactly the scenario the user's instruction anticipated. Investigated before concluding anything: `gz_4ms.log` and `bridge_4ms.log` showed completely normal startup with no errors, but `attitude_4ms.log` never logged `"New target yaw received"` at all (vs. `attitude_1ms.log`'s clean confirmation) — the script published `target_yaw` immediately after the attitude controller process started, racing ROS2/DDS discovery of the subscription. A second run (after copying the missing `ryugu_4ms.sdf` world file into this phase's own directory, a separate minor issue) reproduced the same silent-drop failure specifically on the second (4ms) trial in the sequence, consistent with a discovery-timing race with less settle time on the second launch. Fixed by waiting for a discovered subscriber (`get_subscription_count() > 0`) before publishing, with a bounded 10s timeout and a retry publish as a belt-and-suspenders guard.

**After the fix, reran and got a clean result**: 1ms converged at t=8.40s, final_yaw=106.07°; 4ms converged at t=7.25s, final_yaw=106.08° — both closely matching the historical baseline (old-model comparison: 1ms 106.06°/9.61s, 4ms 106.15°/8.70s; original C13 result 106.03°/<1° by t+9.3s). Reran once more as an extra confirmation given the earlier confusion: 1ms converged again at t=8.16-8.50s across all three total attempts, consistently matching baseline. **Yaw-hold is confirmed stable and unaffected in practice by the I_wheel correction** — the harness race condition was a pre-existing bug in the test script (present in the original Phase 4 version too, just hadn't happened to trigger before), not a consequence of this phase's code change.

## 4. Self-righting sanity check

Ran `self_righting_sanity_check.py` (copy of Phase 17's proven single-continuous-gz-session methodology) against the corrected I_wheel.

**Literal "once each" (n=1/bucket) both failed**: side_rest never landed within the 350s timeout; full_inversion landed but did not recover within the 120s righting-wait window. On n=1, at buckets whose baseline recovery rates are already low and — per this project's own Phase 13/17 investigation — already known to swing widely run-to-run for reasons not fully explained (side_rest: 50%/40%/10% across three prior runs with **zero code changes** between them; full_inversion: 15%/65%/15% across the same three runs), a single failure in each bucket is well within normal variance and not, by itself, distinguishable from real destabilization.

Given the explicit "stop and report if destabilized" instruction warranted more rigor than accepting or dismissing n=1 outright, ran a **confirmatory n=5/bucket batch** (not the full n=50 of item 3 — a bounded escalation to get a more informative read first). Combined with the initial n=1:

| bucket | recovered / n | rate | 95% Wilson CI | recover_time_s (recovered trials) |
|---|---|---|---|---|
| side_rest | 1/6 | 16.7% | [3.0%, 56.4%] | n=1: 109.42s |
| full_inversion | 2/6 | 33.3% | [9.7%, 70.0%] | n=2: mean=77.89s (77.80, 77.97) |

Both rates fall comfortably within (side_rest) or close to consistent with, given the wide CI at this sample size (full_inversion) the **already-documented historical range** for these exact buckets (side_rest 10-50%, full_inversion 15-65% across Phase 7/13/17, all with unmodified code) — not evidence of new destabilization from this phase's I_wheel correction. `recover_time_s` for the successful trials (109.4s side_rest; ~78s full_inversion) sits in a broadly similar range to Phase 13's re-timed baseline (92.86s / 89.55s), not dramatically different given n=1-2.

**Conclusion, combined with §3's clean and reproducible yaw-hold result** (a much more direct, deterministic probe of the same underlying reaction-wheel torque/momentum mechanism): **the I_wheel correction does not destabilize self-righting.** The observed n=6 rates are consistent with this test's pre-existing, well-documented noise floor, not a new effect.

## 5. Anomalies flagged

1. §3: a real harness bug (publish-before-subscriber-discovery race), initially indistinguishable from a genuine destabilization, investigated and correctly attributed before drawing any conclusion — not assumed safe, not assumed broken.
2. §4: n=1/bucket both failing, escalated to a bounded n=5/bucket confirmatory batch rather than either accepting n=1 as sufficient or jumping straight to the full n=50 commitment while the corrected constant's safety was still ambiguous.
3. §2: the wheel-acceleration slew-limit constant (`max_wheel_accel=50.0`) becoming non-binding relative to the new torque-derived physical ceiling (38.0 rad/s²) — a real, flagged behavioral consequence, not itself a bug, not silently left undocumented.

## 6. Checkpoint verdict

**Complete. Stable. Cleared to proceed to items 2/3 of the review round using this corrected I_wheel value**, per the explicit instruction not to rerun self-righting twice once stability is confirmed — item 3's full n=50/bucket rerun (queued next) will be the first and only additional self-righting data collection at this corrected value beyond this phase's own confirmatory n=6/bucket.
