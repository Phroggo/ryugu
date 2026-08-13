# Phase 15 — Timeout-Discard Fix Verification: Confirmed Firing, No Measurable Benefit

Date: 2026-08-13
Scope: verify Phase 14's `_finalize_righting_timeout` fix (extend an attempt instead of discarding it when u_z is already near-upright at timeout) against full_inversion, n=20. **Result: the fix's extension mechanism fires reliably and as designed, but produces zero additional genuine hold-confirmed successes in this batch — a real, informative negative result, not a confirmation of improvement.**

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase15_righting_timeout_fix_verification/righting_timeout_fix_verification.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase15_righting_timeout_fix_verification/righting_timeout_fix_verification_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase15_righting_timeout_fix_verification/righting_timeout_fix_verification_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase15_righting_timeout_fix_verification/gz_p15_batch.log`
- 20 `bridge_scout_1_full_inversion_trial{N}.log` files and 20 `landing_scout_1_full_inversion_trial{N}.log` files (one pair per trial, 40 files total)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase15_righting_timeout_fix_verification/PHASE15_CHANGE_REPORT.md` (this file)

(45 files total this phase. Complete literal listing appended to the commit that carries them.)

## 2. Recovery rate result — confounded by already-documented baseline instability

| Run | Recovered | Rate | Fisher's exact vs. Phase 15 |
|---|---|---|---|
| Phase 7 original baseline | 3/20 | 15.0% | p=0.273 |
| Phase 13 rerun (same code, unpatched) | 13/20 | 65.0% | p=0.113 |
| **Phase 15 (this run, fixed code)** | **7/20** | **35.0%**, 95% CI [18.1%, 56.7%] | — |

Neither comparison is significant. Given Phase 13 already established that full_inversion's recovery rate swings unexplainably between nominally-identical runs (3/20 → 13/20, p=0.003, root cause not found despite thorough investigation — see Phase 13's report), this batch's 7/20 sits inside that already-documented instability band. **It is not possible to attribute this result to the fix, positively or negatively, given the confound.** Reporting it as a data point, not a verdict.

## 3. What the fix actually did — verified against raw logs, not assumed

Grepped all 20 trial logs directly rather than trusting the recovery-rate number alone:

- **`"Near-upright at timeout"` (the fix's extension path) fired 15 times across 8 of 20 trials** — confirming Phase 14's diagnostic finding (a single trial) generalizes: this near-miss-discarded-at-timeout pattern is real and recurs at a meaningful rate, not a one-off.
- **`HOLD-START` count across all 20 trials: 0.** **`HOLD-LOST` count: 0.** **`"Self-righting successful"` (genuine hold-confirmed convergence) count: 0.** **Give-up count: 20/20 (100%, unchanged from every prior batch).**

So the fix's extension mechanism worked exactly as designed (it detects the near-upright-at-timeout condition and grants the same attempt more time, 15 times, across 8 trials) — but **not once did that extra time result in the per-tick hold-confirm gate even starting**, let alone completing a hold. All 7 "recovered" trials in this batch show `recover_time_s` values (79-173s) consistent with the same pre-existing give-up-then-residual-momentum-drift mechanism established in Phase 12/13, not a fix-attributable genuine convergence.

## 4. Why the fix didn't help — a deeper, more precise characterization of the failure mode

The near-upright reading that triggers the timeout-discard bug is evidently a **transient peak during the brake-ramp's own deceleration dynamics**, not evidence of a body that has actually settled into a low-angular-rate state near upright. By construction, if the body genuinely were stable and slow near upright, the per-tick hold-confirm gate — which runs on *every* tick, not just at timeout — would already have caught and held it during normal operation, well before any 15s timeout was ever reached. The only way a "near-upright at timeout" reading can occur at all is if the crossing happens specifically during the ~0.5s brake-ramp window that bypasses the per-tick gate — and the fact that extending the attempt afterward never produces a `HOLD-START` indicates the body's residual angular rate at that moment is still too high (or continues carrying it back down past u_z=0.9) for the hold-confirm's entry condition to ever engage, even with more time.

This means the real failure mode is **not** "insufficient time was allotted" (which is what Phase 14's fix targeted) but more likely **insufficient damping specifically in the terminal approach to upright** — the roll carries enough angular momentum to overshoot through the target rather than settle at it, and giving the same undamped trajectory more time doesn't change that outcome. A more promising fix candidate for a future phase would target the deceleration/damping profile as u_z approaches 0.9 (e.g., a more aggressive braking response as the target nears, rather than a longer window with the same dynamics) — not attempted this phase; flagging as the concrete next hypothesis rather than guessing further.

## 5. Anomalies flagged this phase

1. Recovery-rate comparison is uninterpretable given Phase 13's already-documented baseline instability (§2) — reported as such, not forced into a "the fix helped/hurt" narrative either direction.
2. The fix fires as designed (15/20 trials show at least one extension in 8 of them) but produces zero measurable benefit (0 HOLD-START, 0 genuine successes attributable to it) — a real, useful negative result about the mechanism, not a failure of the investigation.

## 6. Checkpoint verdict

Bug fix from Phase 14: **still correct** — it stops a real, factually-wrong "still inverted" mislabeling and the associated discarding of near-upright state, which is a legitimate correctness fix regardless of whether it moves the recovery-rate needle. **Does not measurably improve full_inversion recovery** in this batch; the deeper cause is very likely a damping/overshoot problem in the terminal approach, not a timing/attempt-budget problem, per §4's reasoning from direct log evidence (0 HOLD-START despite 15 extensions). Recommend keeping the fix (it's correct on its own terms) but not claiming it as a recovery-rate improvement in the paper. Next candidate hypothesis (terminal-approach damping) flagged, not started — checking in with the user before further overnight batches given the amount of ground already covered tonight.
