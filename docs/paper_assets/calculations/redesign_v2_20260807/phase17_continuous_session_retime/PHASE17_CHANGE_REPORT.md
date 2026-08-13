# Phase 17 — Continuous-Session Retime: Full_Inversion Confirmed, Side_Rest Now Also Anomalous

Date: 2026-08-13
Scope: user identified a real, unacknowledged methodology gap in Phase 13 — the harness restarted the gz-sim daemon between buckets, while Phase 7's original script ran all buckets in one continuous session. Verified the claim directly (`diff` of both scripts) before writing any new code: confirmed exact. This phase reruns side_rest + full_inversion in a single continuous session, byte-diffed against Phase 13's script to confirm the daemon-restart removal is the *only* functional change, to test whether the restart explains Phase 13's full_inversion rate anomaly (3/20 → 13/20).

**Result: confirmed for full_inversion, but a new, different anomaly appeared for side_rest that the same fix does not explain.**

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase17_continuous_session_retime/continuous_session_retime.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase17_continuous_session_retime/continuous_session_retime_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase17_continuous_session_retime/continuous_session_retime_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase17_continuous_session_retime/gz_p17_batch.log`
- 40 `bridge_scout_1_{bucket}_trial{N}.log` files and 40 `landing_scout_1_{bucket}_trial{N}.log` files (one pair per trial, 80 files total)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase17_continuous_session_retime/PHASE17_CHANGE_REPORT.md` (this file)

(85 files total this phase. Complete literal listing appended to the commit that carries them.)

## 2. Methodology gap, confirmed before writing new code

`diff`'d `phase7_full_revalidation/self_righting_batch_3bucket.py` against `phase13_baseline_recovery_timer_retime/baseline_recovery_timer_retime.py`: Phase 7 starts gz-sim once, before its bucket loop, and never restarts it (60 trials, 3 buckets, one continuous session). Phase 13 has `kill_all(); start_world(log)` inside the bucket loop, firing after each bucket's n=20 trials — so Phase 13's full_inversion ran on a freshly restarted daemon, not the same continuously-running process side_rest had already been running on. Phase 13's report checked harness *parameters* (SPAWN_Z, timeouts, etc.) but not this structural difference — a real gap.

`continuous_session_retime.py` is `diff`-verified to be identical to Phase 13's script except for the removed inter-bucket restart and cosmetic renames (output paths, node name, log strings) — confirmed via direct `diff` before running, not assumed.

## 3. Results

| Bucket | Phase 7 (original, continuous) | Phase 13 (restart between buckets) | Phase 17 (this phase, continuous) |
|---|---|---|---|
| side_rest | 10/20 (50.0%) | 8/20 (40.0%) | **2/20 (10.0%)**, 95% CI [2.8%, 30.1%] |
| full_inversion | 3/20 (15.0%) | 13/20 (65.0%) | **3/20 (15.0%)**, 95% CI [5.2%, 36.0%] |

**Full_inversion: Fisher's exact, Phase 17 vs. Phase 7 = p=1.0 (identical counts). Phase 17 vs. Phase 13 = p=0.003.** This is about as clean a confirmation as this kind of test can give: matching Phase 7's continuous-session structure reproduces Phase 7's exact result and is highly significantly different from Phase 13's restarted-daemon result. **Your hypothesis is confirmed — the daemon restart, not genuine physics nondeterminism, was the cause of Phase 13's full_inversion anomaly.**

**Side_rest: Fisher's exact, Phase 17 vs. Phase 7 = p=0.014 (uncorrected). Phase 17 vs. Phase 13 = p=0.065.** Side_rest was *not* flagged as anomalous before (Phase 13's 8/20 vs. Phase 7's 10/20 was unremarkable, p=0.75) — but under the continuous-session structure that just resolved full_inversion's anomaly, side_rest now shows its *own* discrepancy, in a direction the daemon-restart theory does not predict (matching Phase 7's structure more closely produced a *worse*, not better, match for this bucket). With 4 pairwise comparisons made across this investigation (2 buckets × 2 comparisons each), a Bonferroni-corrected threshold is p<0.0125 — side_rest's p=0.014 does not quite survive that correction, so this should not be overclaimed as a second confirmed anomaly, but it is a real, unexplained discrepancy worth flagging rather than ignoring just because it's inconvenient to the tidy "daemon restart explains everything" narrative.

Checked before writing this down: side_rest's 20 trials show no crashes, no harness bugs, no missing telemetry — 17 genuine `failed` (give-up) outcomes, 2 `recovered`, 1 `no_landing`, scattered across the full 0-360° azimuth range with no obvious clustering pattern. This is not a technical artifact.

`recover_time_s` (not in question, re-measured for completeness): side_rest n=2, mean=78.97s (consistent with the ~77-190s range established across Phase 12/13/15); full_inversion n=3, mean=89.35s (also consistent). The recovery-*time* numbers remain stable and trustworthy regardless of the rate discrepancies above.

## 4. A plausible contributing factor not previously considered: unseeded random tilt/azimuth sampling

Neither Phase 7's, Phase 13's, nor this phase's script calls `random.seed(...)` anywhere — each run draws its own, entirely independent sequence of `random.uniform(lo, hi)` tilt and azimuth values from Python's OS-entropy-seeded default PRNG state. `landing_controller.py`'s righting roll is explicitly direction-aware (aimed along the measured tilt azimuth, re-derived as the body rotates) — if recovery success is sensitive to the *specific* azimuth within a bucket's range (plausible, not confirmed), then three runs drawing three different sets of 20 random azimuths could produce genuinely different aggregate outcomes from ordinary sampling variance alone, with no need to invoke daemon warm-up state or deeper physics nondeterminism. This does not compete with the daemon-restart explanation for full_inversion (that comparison came out about as clean as a statistical test can) but may be part of what's going on with side_rest, and is worth naming explicitly rather than reaching for another unverified mechanism.

## 5. Recommendation for Table IX

- **Full_inversion recovery rate: use Phase 7's / Phase 17's matching 3/20 (15%).** The daemon-restart artifact in Phase 13 is now identified and explained — Phase 13's 13/20 should not be used.
- **Side_rest recovery rate: do not finalize yet.** Three different measurements now exist (50%, 40%, 10%) with no methodology difference between the first two and a real (if borderline-significant) drop in the third. Recommend either a larger combined-n estimate across multiple continuous-session runs, or treating this as a second open item alongside the still-not-investigated general question of whether these tilt buckets are more azimuth-sensitive than currently modeled.
- **`recover_time_s` for both buckets: Phase 13's / this phase's re-timed figures remain valid and ready for Table IX** — this was never in question, only the recovery-rate comparison.

## 6. Checkpoint verdict

Full_inversion anomaly: **resolved, root cause confirmed (daemon restart), not a genuine simulation-nondeterminism finding.** Side_rest: **a new, real, borderline-significant discrepancy, not resolved, flagged for a decision rather than picked silently or buried.** Holding Table IX per your instruction until this is fully sorted — recommend proceeding with full_inversion's rate now that it's confirmed, while side_rest's rate stays open.
