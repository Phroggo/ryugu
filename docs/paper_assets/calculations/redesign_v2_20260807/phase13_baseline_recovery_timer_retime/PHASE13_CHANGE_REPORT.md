# Phase 13 — Re-Timed Baseline Recovery Time (side_rest + full_inversion) — and a Major, Unexplained Recovery-Rate Discrepancy

Date: 2026-08-13
Scope: direct extension of Phase 11's recovery-timer fix (applied there to moderate only) to side_rest and full_inversion, per explicit request after the user traced through `landing_controller.py` and confirmed the same root cause applies: a badly-tilted landing goes straight to `RIGHTING`, not `LANDED`, so the original harness's `landed` poll doesn't fire until after all 5 active attempts and the give-up handler force-sets `LANDED` — meaning the old `right_t0` timer measured only the post-give-up settling tail (19.5s/17.0s), not the true active-struggle duration. This reruns baseline (unperturbed model) for both buckets with Phase 11's `/righting_active`-keyed timer.

**This phase's primary deliverable (re-timed `recover_time_s`) is clean and ready for Table IX. But the same rerun also surfaced a large, unexplained shift in full_inversion's recovery RATE that is flagged prominently below and was not smoothed over or averaged away.**

## 1. Files touched

### New script

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase13_baseline_recovery_timer_retime/baseline_recovery_timer_retime.py`

### Results and logs (40 trials: 2 buckets x n=20, baseline/unperturbed)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase13_baseline_recovery_timer_retime/baseline_recovery_timer_retime_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase13_baseline_recovery_timer_retime/baseline_recovery_timer_retime_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase13_baseline_recovery_timer_retime/gz_p13_batch.log`
- 40 `bridge_scout_1_{bucket}_trial{N}.log` files and 40 `landing_scout_1_{bucket}_trial{N}.log` files (one pair per trial, 80 files total)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase13_baseline_recovery_timer_retime/PHASE13_CHANGE_REPORT.md` (this file)

(85 files total this phase. Complete literal listing appended to the commit that carries them.)

## 2. What was run, and a harness bug avoided this time

Same methodology as Phase 7's `self_righting_batch_3bucket.py` (verified parameter-for-parameter identical: `N_PER_BUCKET=20`, `SUCCESS_UZ=0.9`, `SPAWN_Z=5.2`, `LANDED_WAIT_TIMEOUT=350.0`, `RIGHTING_WAIT_TIMEOUT=120.0`) and Phase 11's `moderate_recovery_timer_rerun.py` (timer keyed off `/righting_active`'s first `True` transition). Unperturbed `model://spacehopper` — no I_bot variant.

Having been burned by a missing `make_bridge_yaml()` call in Phase 12, this script's bridge-YAML generation was smoke-tested standalone (parsed as a valid 12-entry YAML list) before launching the full batch, and trial 1's actual output was checked (`start_uz=0.032`, real telemetry) before letting the batch run unattended.

## 3. Re-timed `recover_time_s` (the requested deliverable) — ready for Table IX

| Bucket | n (recovered) | Mean | Std | Min | Max |
|---|---|---|---|---|---|
| side_rest | 8 | 92.86s | 27.65s | 77.55s | 160.24s |
| full_inversion | 13 | 89.55s | 27.99s | 77.57s | 180.46s |

These replace Table IX's old figures (19.5s mean side_rest, 17.0s mean full_inversion), which — as diagnosed — were measuring only the post-give-up settling tail. Consistent with Phase 12's perturbed-config numbers (mean 86.58-99.11s across the four I_bot configs there), confirming this is a real, reproducible measurement of the ~75-190s active-righting-plus-settling duration, not an artifact of this specific run. Give-up cross-check (same method as Phase 12 §3): 19/19 side_rest and 20/20 full_inversion trials that reached an outcome show the give-up message — consistent with Phase 7's original baseline and Phase 12's perturbed configs, confirming this is stable, well-established system behavior, not something specific to this rerun.

## 4. A major, unexplained recovery-rate discrepancy — flagged, not resolved

| Bucket | Phase 7 original baseline | Phase 13 rerun (this phase) | Fisher's exact test |
|---|---|---|---|
| side_rest | 10/20 (50.0%) | 8/20 (40.0%), 95% CI [21.9%, 61.3%] | p=0.751 — consistent with normal sampling variance |
| full_inversion | 3/20 (15.0%) | 13/20 (65.0%), 95% CI [43.3%, 81.9%] | **p=0.003 — highly significant, not sampling noise** |

Side_rest's shift is unremarkable. **Full_inversion's is not**: recovery rate more than quadrupled, and it is now *higher* than either of Phase 12's I_bot-perturbed configs (45%, 35%) despite this run using the unperturbed baseline model. Investigated before writing this down, rather than accepting either number as "the" baseline:

- **Code**: `git log --oneline -- ryugu_sim/landing_controller.py` shows the file was touched in Phase 9 (narrow `Popen()` threading fix only) and Phase 10 (sensor-noise injection in `odom_callback`, default off) since Phase 7's baseline was generated (commit `8f0189e`, which is itself the commit that produced that baseline data). Neither change touches the `RIGHTING` state machine, `_run_righting_sequence()`, or any righting-attempt logic. The active-righting code path is bit-for-bit identical to what generated the original 3/20.
- **World/model files**: `git log` shows `worlds/ryugu.sdf` and `models/spacehopper/model.sdf` last changed well before Phase 7 (pre-redesign and Phase 2 respectively) — unchanged for this entire investigation's timeframe.
- **Noise injection leakage**: grepped all 20 full_inversion trial logs for the `"Odometry orientation noise ACTIVE"` warning Phase 10 added — none found. The env-var-toggled noise is confirmed off.
- **Harness parameter drift**: diffed `N_PER_BUCKET`/`SUCCESS_UZ`/`SPAWN_Z`/`LANDED_WAIT_TIMEOUT`/`RIGHTING_WAIT_TIMEOUT` between this script and Phase 7's original — identical values.
- **Give-up rate**: identical pattern (100%) in both the original baseline and this rerun — rules out "the controller behaves qualitatively differently now," since in both cases every recovery is a give-up-then-drift outcome, not a converged one.

None of these explain the shift. What's left is that this is either (a) genuine run-to-run physics nondeterminism in gz-sim's contact/ODE solver for this specific near-chaotic scenario (full inversion recovery is a bistable tipping process — small differences in exact contact timing could plausibly cascade into very different aggregate outcomes across two independently-seeded 20-trial batches), consistent with `phase0_baseline_lockin/BASELINE_MANIFEST.md`'s own prior note that *"no reliable single-value launch-delivery ratio exists for any commanded distance — run-to-run variance at fixed everything is too large"* — i.e., this general category of instability has already been flagged as a property of this simulation, just not previously quantified this starkly for self-righting; or (b) something not yet identified. **Not resolving this further tonight** — it's a decision-requiring finding, not a quick patch: does the paper report Phase 7's original 3/20, this rerun's 13/20, a pooled estimate across both (16/40 = 40%, CI [25.9%,55.9%]), or flag full_inversion's recovery rate as unreliable pending a larger/repeated study? Flagging for a decision rather than picking one silently.

## 5. Anomalies flagged this phase

1. Full_inversion recovery rate shifted from 15% to 65% between two nominally-identical baseline runs (§4) — investigated thoroughly, root cause not identified, explicitly not resolved or averaged away.
2. Side_rest's rate shift (50%→40%) is unremarkable and consistent with normal sampling variance at n=20 — noted for contrast, not itself an anomaly.
3. Re-timed `recover_time_s` values (§3) are consistent across this phase and Phase 12's four I_bot configs (all in the ~77-190s range, means 87-99s) — this consistency is itself evidence the retiming fix is measuring something real and reproducible, independent of the rate question.

## 6. Checkpoint verdict

Re-timed `recover_time_s`: **PASS, ready for Table IX** (§3) — apply the same "N/A — see recovery mechanism note" treatment is NOT needed here (unlike moderate), since side_rest/full_inversion do genuinely engage active righting; these are real corrected durations. Recovery-rate discrepancy (§4): **flagged for a decision, not resolved** — do not update Table IX's recovery-rate entries from this report alone without deciding how to handle the discrepancy. Moving on to the torque-vs-geometry investigation next, per instruction not to let this block further work.
