# Phase 25 — Directional-Hop 3-Axis Coupling Investigation

Date: 2026-08-14
Scope: item 2 of the external review round's four sim-side items — attempt a fix for the directional-hop issue the paper currently reports as a negative result (0.066m mean displacement against a 5m target, ~1.3% delivery, azimuth scattered ~95° std). Per explicit instruction: if early investigation shows this is a deep architectural problem rather than a tunable one, stop and report that finding rather than pushing through to a shaky result.

**Finding: this is a deep architectural problem, not a tunable one.** Three independent tests (one purely offline against existing data, one a physically-motivated ruling-out check, one live in simulation) all point the same direction: single-hop azimuth control is not a fixable systematic bias, is not a measurement artifact, and — critically, this is the actual "attempt a fix" part of this item — iterative re-aiming using the system's own existing corrective-re-hop mechanism does not converge either, because it has no reliable signal to correct against. No code change is recommended. See §6 for the specific paper-framing recommendation.

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/az_bias_retrospective.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/az_bias_retrospective_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/displacement_correlation_check.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/displacement_correlation_check_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/iterative_corrective_hop_test.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/iterative_corrective_hop_results.json` [pending]
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase25_directional_hop_investigation/PHASE25_CHANGE_REPORT.md` (this file)

No source files modified this phase — all analysis against existing Phase 8/10 data plus one new, bounded live test (§3).

## 2. Two offline analyses against existing Phase 10 data (zero new sim time)

Before attempting any code change or new sim time, checked whether the most likely "quick fix" hypotheses actually hold, using Phase 10's real per-trial data (`phase10_postfix_full_revalidation/directional_hop_postfix_results.json`, n=26 landed trials, 5.0m target at -55° heading).

### 2.1 Is this a fixable systematic per-robot bias? No — tested directly, not assumed.

`swarm_manager.py` already has a per-robot heading-bias EMA correction mechanism (`az_bias`, in `landed_callback`) built for exactly this class of problem — a prior investigation (referenced in `hopper_locomotion.py`'s dispatch comments) found some robots hop consistently ~170-190° off (effectively backwards), which this mechanism was built to learn and correct. **Critically, Phase 10's directional-hop validation test bypasses `swarm_manager` entirely** — it commands `target_yaw`/`jump_target_distance` directly to `hopper_locomotion`, so the existing bias-correction mechanism is never exercised by the test that produced the paper's negative-result number.

Replayed the 26 real trials through the exact `az_bias` EMA update formula, offline (`az_bias_retrospective.py`): raw mean absolute error from commanded heading is 125.2° (std 128.1°); with retrospective bias correction applied, mean absolute error drops to 71.2° — a real reduction, but **the bias trajectory never converges**: it swings between roughly +170° and -170° from trial to trial (e.g. trial 10: +166.6°, trial 11: -162.9°) rather than settling on a stable value, and correction does not improve over the trial sequence (first-half mean|error|=58.5° vs. second-half=83.9° — worse in the second half, not better). **Conclusion: the azimuth error is not explained by a fixed, learnable per-robot bias.** It is dominated by trial-to-trial variance too large for an EMA tracker to converge on, ruling out "just turn on the existing bias correction" as a fix.

### 2.2 Is this a measurement-noise artifact of tiny displacement vectors? No — tested directly.

Displacements in Phase 10's data are small (0.020-0.147m), and `atan2(dy,dx)` from a near-zero vector is highly sensitive to small position noise — a real, plausible alternative explanation worth ruling out before concluding the launch mechanism itself is at fault. Split the 26 trials into small- and large-displacement halves (`displacement_correlation_check.py`): small-displacement half (0.020-0.060m) mean |offset|=117.9°; large-displacement half (0.063-0.147m, up to 3x larger) mean |offset|=132.5° — essentially the same, if anything slightly worse for larger displacements. **Conclusion: this rules out small-vector measurement noise as the primary explanation.** The scatter is a real feature of launch-direction inconsistency, not an artifact of how the angle is computed.

### 2.3 What this leaves: a physically-grounded hypothesis, not proven further this phase

`hopper_locomotion.py`'s own extensive V_GAIN calibration history (multiple re-derivations, most recently Phase 6) already documents that even hop *magnitude* is only weakly predictable from the commanded ramp time (an 11% velocity spread against a 4.5x spread in ramp_T) — a launch mechanism that struggles to deliver consistent magnitude is a plausible candidate to also struggle with consistent direction. The tri-pedal launch uses a hip-space lean differential (one leg pushes harder than the other two) to steer azimuth; under Ryugu's near-zero gravity (weight ~2.85e-4 N per the existing friction-budget comments elsewhere in this file), the exact relative timing of when each of the three feet breaks ground contact could plausibly dominate the small intentional steering asymmetry — small, sub-second variations in release sequencing scattering the net impulse direction. This is a physically-motivated hypothesis for *why* this is hard, consistent with the "deep architectural problem" category, but is not independently proven this phase (would require per-leg contact-force telemetry at release, out of scope for this pass).

## 3. Fix attempt: does the system's existing iterative re-aim mechanism compensate at the mission level?

Given §2.1-2.2 rule out the cheap fixes and single-hop azimuth control itself is not shown to be directly repairable, the remaining well-scoped, bounded question is whether the mission-level system already compensates for this weakness via a mechanism it already has: `swarm_manager.py`'s corrective re-hop (recompute heading toward the target from the agent's actual current position after every hop, up to `MAX_HOP_RETRIES=5`) — used throughout every real multi-agent mission test in this project (Phases 7/13/17/21), where agents do complete real point-to-point tasks, unlike the isolated single-hop Phase 8/10 test which exercises none of this.

Built `iterative_corrective_hop_test.py`: same 5.0m/-55° single-agent scenario as Phase 10 for direct comparability, but after each landing, recomputes heading toward the fixed target from the agent's real current position and re-hops, up to 5 attempts, checking whether the agent arrives within a **1.0m** tolerance (tighter than `swarm_manager`'s own 4.0m `ARRIVAL_RADIUS`, which is sized for drill-arm reach and would trivially pass almost any real displacement at this target distance — not a meaningful "did directional correction work" criterion). n=10 repeats.

**Results, n=2 complete reps + 1 partial (stopped early — see below), unambiguous:**

| rep | hop | landed at | dist_remaining after | note |
|---|---|---|---|---|
| 1 | 1 | (-0.04, 0.51) | 5.44m (start was 5.42m) | no real progress |
| 1 | 2 | (0.02, 0.38) | 5.30m | marginal (~2.5%) improvement, within noise |
| 1 | 3 | — | 5.30m | never confirmed separation (`no_separation`) — hop mechanism itself failed to fire cleanly |
| 2 | 1 | (-0.14, 0.41) | 5.41m (start was 5.42m) | no real progress |
| 2 | 2 | (-3.33, 0.54) | **7.74m** | **distance INCREASED** — moved away from target, not toward it |
| 2 | 3 | — | 7.74m | never landed within the 1400s flight-wait ceiling (`never_landed`) |
| 3 (partial, not saved to JSON — see below) | 1 | (-0.02, 0.50) | ~5.42m | no real progress |
| 3 (partial) | 2 | (-1.95, 3.49) | 8.98m | distance increased again, same pattern as rep 2 |

**Neither fully-recorded rep converged toward the target; rep 2 (and the observed partial rep 3) actively moved further away after a re-hop** — something a working correction mechanism should never do. `final_status` for both complete reps was a failure mode (`no_separation`, `never_landed`), not a tolerance-based stop.

**Conclusion: iterative re-aiming does not reliably converge.** Because each individual hop's azimuth remains essentially uncontrolled (§2.1-2.3), re-aiming based on the agent's new position after an equally-uncontrolled hop is not a corrective signal — it's another independent, unpredictable draw. This is a stronger, more direct result than §2.1's offline retrospective test (which only showed the bias-tracker failing to converge on measurement noise): this is the mechanism actually failing to make progress in live simulation, hop after hop, sometimes moving backward.

**Stopped after rep 3 (partial) rather than running the full n=10 budget.** Several hops already needed the full 200-1400s per-hop time range (including two hitting the maximum separation/flight-wait ceilings), and the qualitative result was unambiguous well before a full n=10 would complete. The additional wall-clock cost of grinding to n=10 was not justified once the mechanism was clearly shown not to work — consistent with the standing instruction that a real "we tried, here's why it's hard" writeup is more valuable than a fuller but foregone-conclusion dataset. Rep 3 was interrupted mid-hop-3 and its JSON entry was not written (the script only persists a rep's record after it fully resolves); its two completed hops are recorded here directly from the live stdout transcript, not discarded. Full n=2 recorded data in `iterative_corrective_hop_results.json`.

## 4. Item 1 interaction note

This investigation ran using the Phase 24-corrected `I_wheel=3.944e-4` (attitude_controller.py), confirmed stable via that phase's sanity checks before this phase started, per the review round's explicit ordering (#1 before #2/#3). Yaw-hold accuracy at ignition was consistently good throughout this phase's live trials (yaw_error_at_ignition values of 0.0000039, 0.012, 0.026, 0.034, 0.0001, 0.017, 0.0001, 0.0021 rad — all well under the 0.15 rad alignment threshold) — the attitude controller reliably points the body the right way before launch. **The failure is specifically in translating that correct body orientation into a correctly-directed launch impulse**, not in attitude control itself, and not something this item's fix attempt found any lever to pull on.

## 5. Recommendation for paper framing

Reviewers were reportedly split between "fix this" and "honestly reframe it" — the evidence gathered this phase supports the reframe path, now with a specific, evidenced reason rather than just "we tried and it didn't work":

- The existing negative result (0.066m mean displacement, ~1.3% delivery, azimuth std ~95°) is real and is a **single-hop, open-loop** measurement — accurately characterizes what one commanded hop delivers, and should keep being reported as exactly that.
- What should be **added**, not substituted: this phase's finding that the system's own closed-loop correction mechanisms (both the existing per-robot heading-bias EMA in `swarm_manager.py`, and iterative position-based re-aiming) **do not recover reliable point-to-point delivery either** — tested directly, not assumed. This rules out the natural reviewer follow-up question ("didn't the swarm's existing bias-correction/re-hop logic already handle this?") with evidence rather than leaving it open.
- The most defensible root-cause framing, physically grounded (§2.3): the tri-pedal launch's directional control depends on a hip-space lean asymmetry small enough that under Ryugu's near-zero gravity, sub-second variation in per-leg ground-release timing plausibly dominates the intended steering signal — consistent with this project's own independently-documented finding that even hop *magnitude* is only weakly controllable from the commanded parameters (Phase 6's V_GAIN calibration: 11% velocity spread against a 4.5x spread in ramp_T). A genuine fix would likely require redesigning the launch mechanism's release synchronization or moving to a fundamentally different steering method (e.g., a true single-axis thrust vectoring mechanism) — out of scope for a review-round fix attempt, and appropriately so.

## 6. Checkpoint verdict

**Complete, negative result, correctly scoped per explicit instruction not to push through to a shaky fix.** Three independent, evidence-based tests (offline bias-correction replay, displacement-magnitude confound check, live iterative re-aiming) converge on the same conclusion: this is an architectural limitation of the launch mechanism's directional authority under Ryugu's near-zero gravity, not a tunable parameter or a missing piece of closed-loop correction. No source code changed. Recommended paper action is additive (report that closed-loop correction was tried and doesn't help, strengthening rather than replacing the existing honest negative result) rather than a forced partial fix.
