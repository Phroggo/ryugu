# Phase 8 — Overnight Simulation Batch (response to the sim-chat request)

Date: 2026-08-10, overnight (01:32-09:09).
Scope: response to the "Overnight simulation batch requests" message, priorities 1-4 (5-7 explicitly deferred, out of overnight scope). Completed cleanly: Priority 2, Priority 3 (both mu and e sweeps). Priority 1 was never started (blocked on a design decision, flagged before any work began). Priority 4 was started, then deliberately stopped after 2 trials when it became clear the data being produced was invalid, not because time ran out.

## 1. Files touched

### Code (new, this repo)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/launch_delivery_batch_n100.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/generate_mu_variants.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/friction_sensitivity_sweep.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/generate_restitution_variants.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sensitivity_sweep.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/directional_hop_validation.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/PHASE8_OVERNIGHT_REPORT.md` (this file)

No changes to `ryugu_sim/` controller source this phase — this was a measurement batch, not a fix phase (unlike Phase 6/7). One live process anomaly was found (SS5) but not patched, consistent with the same caution applied to Phase 7's orientation bug: a significant, poorly-understood control-timing issue gets flagged and confirmed, not patched overnight without sign-off.

### Generated model/world variants (kept out of the real `models/`/`worlds/` trees, per the project's established convention for experiment-specific variants — e.g. Phase 0/4's `ryugu_4ms.sdf`)

- `.../phase8_overnight_batch/variant_models/spacehopper_mu040/` (model.sdf, model.config, meshes/)
- `.../phase8_overnight_batch/variant_models/spacehopper_mu050/`
- `.../phase8_overnight_batch/variant_models/spacehopper_mu075/`
- `.../phase8_overnight_batch/variant_models/spacehopper_mu090/` (mu=0.62 reuses the live, unmodified `model://spacehopper` — not duplicated)
- `.../phase8_overnight_batch/variant_models/regolith_plane_e010/` (model.sdf, model.config, meshes/, materials/)
- `.../phase8_overnight_batch/variant_models/regolith_plane_e020/`
- `.../phase8_overnight_batch/variant_models/regolith_plane_e040/`
- `.../phase8_overnight_batch/variant_worlds/ryugu_e010.sdf`
- `.../phase8_overnight_batch/variant_worlds/ryugu_e020.sdf`
- `.../phase8_overnight_batch/variant_worlds/ryugu_e040.sdf`

### Results/logs

Top-level results/stdout files listed individually below; the ~850 remaining files are per-trial/per-repeat node console logs (bridge/loco/attitude/landing), following the same fully-specified naming convention as every prior phase. The complete literal list (every path) is appended verbatim in SS7.

- `.../phase8_overnight_batch/launch_delivery_n100_results.json`, `launch_delivery_n100_stdout.log`, `gz_batch8_n100.log`
- `.../phase8_overnight_batch/friction_sweep_results.json`, `friction_sweep_stdout.log`, `gz_mu_sweep.log`
- `.../phase8_overnight_batch/restitution_sweep_results.json`, `restitution_sweep_stdout.log`, `gz_e010.log`, `gz_e020.log`, `gz_e040.log`
- `.../phase8_overnight_batch/restitution_sweep_failed_missing_textures/` (first, broken attempt — preserved as evidence, see SS5.2)
- `.../phase8_overnight_batch/directional_hop_results.json`, `directional_hop_stdout.log`, `gz_dirhop.log`

## 2. What was run

1. **Priority 2** (`launch_delivery_batch_n100.py`): n=100 at 9.0m, direct extension of Phase 7's proven n=30 harness (same STABILIZE_WINDOW=75s, periodic daemon restart every 5 reps), plus a 95% CI added to the summary. Command: `python3 launch_delivery_batch_n100.py`.
2. **Priority 3a** (`generate_mu_variants.py` then `friction_sensitivity_sweep.py`): foot-friction mu in {0.40, 0.50, 0.62, 0.75, 0.90}, n=20 launches at 9.0m each (100 runs total). mu=0.62 reuses the live model; the other 4 spawn generated model variants.
3. **Priority 3b** (`generate_restitution_variants.py` then `restitution_sensitivity_sweep.py`): terrain `restitution_coefficient` in {0.1, 0.2, 0.4} (current live value is 0.15, not one of these — all 3 are new variants), 3 deterministic drops each from 1.15m, same fixed-250s-settle-wait methodology as Phase 7's restitution spot-check.
4. **Priority 4** (`directional_hop_validation.py`): commanded heading -55deg (the manifest-sourced value — the request said -56deg, used the precise citation instead), distance 5.0m, full spawn+4-node lifecycle per trial, tracking odometry through separation to landing. **Stopped after 2/30 trials** (SS5.3).
5. Priority 1 (sensor noise): not started. Priorities 5-7: not started.

All batches used the exact same headless `gz sim` + fresh-respawn-per-trial methodology as every prior phase; commands and full node logs are preserved per-trial as listed in SS7.

## 3. Results

### 3.1 Priority 2 — Launch delivery, n=100 (`launch_delivery_n100_results.json`)

no_separation=0 (0%), stabilized=100 (100%).

**mean=0.2121, median=0.2125, std=0.0015, min=0.2067, max=0.2129, range=0.0062.**
**95% CI (normal approx.) on the mean: [0.2118, 0.2124].**

Runtime: 01:32:34-03:56:42 (~2h24m), zero timeouts, zero anomalies, no TICK/WALL-TIME MISMATCH markers found. Directly comparable to Phase 7's n=30 (mean=0.212, range=0.006) — this n=100 result tightens the CI further without shifting the central estimate, exactly as expected from a larger sample of the same underlying distribution.

### 3.2 Priority 3a — Friction (mu) sensitivity, n=20 per value, n=100 total (`friction_sweep_results.json`)

| mu | n stabilized | mean ratio | std | min | max |
|---|---|---|---|---|---|
| 0.40 | 20/20 | 0.2119 | 0.0016 | 0.2069 | 0.2128 |
| 0.50 | 20/20 | 0.2117 | 0.0019 | 0.2070 | 0.2129 |
| 0.62 (live) | 20/20 | 0.2126 | 0.0002 | 0.2123 | 0.2129 |
| 0.75 | 20/20 | 0.2120 | 0.0016 | 0.2070 | 0.2128 |
| 0.90 | 20/20 | 0.2117 | 0.0019 | 0.2070 | 0.2128 |

**100/100 stabilized, 0 no_separation across the entire sweep.** Delivered launch ratio is essentially flat across a >2x range of foot friction (0.40-0.90) -- all 5 means fall within 0.2117-0.2126, well inside each other's spread. The one mild observation: mu=0.62 (the live, tuned value) shows a visibly tighter std (0.0002) than the other 4 (0.0016-0.0019); not chasing this further here, but noting it rather than averaging it away. Launch delivery is not friction-limited over this range -- consistent with (and now empirically supporting) the paper's framing that contact/separation dynamics, not sliding friction, is the binding constraint.

Runtime: 03:57:19-06:21:26 (~2h24m).

### 3.3 Priority 3b — Terrain restitution sensitivity, 3 values x 3 drops (`restitution_sweep_results.json`)

| e_target (surface `restitution_coefficient`) | first-bounce e per drop | 
|---|---|
| 0.1 | 0.1130, 0.1129, 0.1130 |
| 0.2 | 0.1130, 0.1130, 0.1129 |
| 0.4 | 0.1130, 0.1130, 0.1130 |

**All 9 drops (3 values x 3 repeats) converge to e = 0.113, to 3 decimal places, regardless of the surface restitution_coefficient target.** This is a strong, clean null result: the empirically-measured whole-body bounce coefficient is *completely* insensitive to the terrain surface's SDF restitution parameter over a 4x sweep (0.1-0.4). This directly corroborates Phase 7's finding/hypothesis (SS4.5 of that report) that the empirical bounce is dominated by leg-joint damping, not the surface contact model -- now confirmed by direct sensitivity sweep rather than a single-condition measurement. **Interpretive caveat repeated from the harness docstring**: this sweeps the SDF surface parameter directly, which is a different physical quantity from the whole-body coefficient it apparently doesn't control; report this as "surface-restitution insensitivity," not as "the robot's bounce behavior is tunable to 0.1/0.2/0.4."

Runtime (successful rerun only): 06:29:53-09:00:40 (~2h31m). See SS5.2 for the first, failed attempt.

### 3.4 Priority 4 — Directional hop validation: 2/30 trials, both invalid, batch stopped

Commanded: distance=5.0m, heading=-55deg. Only 2 trials completed before the batch was stopped (SS5.3):

| rep | displacement | azimuth | flight_time | yaw_error_at_ignition |
|---|---|---|---|---|
| 1 | 0.027m | -177.7deg | 112.0s | 0.0 (suspicious -- see SS5.3) |
| 2 | 0.005m | -159.8deg | 163.1s | 0.017 rad |

**Both results are invalid and should not be used for anything** -- displacement is near-zero (2.7cm and 0.5cm on a commanded 5.0m hop) and azimuth bears no relation to the commanded -55deg heading. Root cause: a launch-stance-corrupting timing anomaly, detailed in SS5.3. The paper's contradicted "4.3m/-56deg" claim remains unreplaced by this batch -- Priority 4 needs to be rerun after the anomaly in SS5.3 is understood, not just retried.

## 4. What went right (verification notes)

- **Priority 2**: clean on the first attempt, no issues.
- **Priority 3a**: clean on the first attempt. Sanity-checked live (mu=0.40's first trial watched end-to-end) before letting the full 100-run sweep proceed unattended.

## 5. What didn't go as planned

### 5.1 Priority 1 was never attempted (by design, not an oversight)

Flagged before any code was written: Phase 7 fixed `landing_controller.py` to read tilt from **odometry**, not the IMU message, specifically because gz-sim's simulated IMU orientation is unreliable in this environment. The request asks to add Gaussian noise to "IMU orientation" for the self-righting Monte Carlo -- but self-righting's actual decision-making no longer reads IMU orientation at all post-Phase-7. Running this as literally specified would inject noise into an unused data path and produce a Monte Carlo showing zero sensitivity, which would misrepresent robustness rather than test it. This needs a design decision (what channel should the noise perturb -- odometry, which is real ground truth with no noise model at all currently; or IMU linear_acceleration/angular_velocity, which *are* still read for contact detection and rate damping) before it can be run meaningfully. Not something to guess at unilaterally overnight.

### 5.2 Priority 3b's first attempt failed: missing texture/mesh subdirectories

`generate_restitution_variants.py`'s first version copied only `model.sdf` and `model.config` for each `regolith_plane_eXXX` variant, omitting the `meshes/` and `materials/` subdirectories the heightmap collision/visual geometry references via relative `<uri>`s. Result: `gz sim` failed to load the world entirely ("Parser configurations requested resolved uris, but uri [...] could not be resolved" / "Failed to load a world"), the reference-drop step never received any odometry, and `rest_z` stayed `None` for the full 250s wait before the script crashed on the resulting `None + float` arithmetic (background task reported exit code 1). Caught within ~5 minutes via a quick check on the first trial (not left to fail silently for hours) rather than assumed working. Fixed by copying both subdirectories in the generator; verified via a 15s standalone smoke test (clean load, no errors, matching `rest_z` on rerun to the exact value Phase 7 found: `4.802369137838924`) before committing to the full rerun. Failed run's logs preserved in `restitution_sweep_failed_missing_textures/` rather than deleted.

### 5.3 Priority 4 was stopped: a new, pervasive tick/wall-time mismatch corrupted launch stances

**This is the most significant finding of the night and needs attention before Priority 4 is retried.**

Rep 1's `loco_scout_1` console log shows the CROUCH state firing IGNITION only **34 milliseconds** after entering CROUCH (`1786338182.357251550` to `1786338182.391336609`, computed precisely from the raw timestamps, not a display-rounding artifact) -- but `hopper_locomotion.py`'s CROUCH state requires `self.state_timer > 100` (a 10-SECOND minimum dwell at 10Hz) before it is even allowed to consider transitioning to LAUNCH. This is not possible under the code's own logic unless the tick counter itself is being incremented far faster than real time.

Rep 2 makes this unambiguous: its log shows Phase 7's own SEPARATION_MAX_WAIT_TICKS abort instrumentation firing explicitly --
```
Aborting hop: never confirmed genuine separation after 60s post-ramp
(tick-derived; real wall-clock since IGNITION=1.0s) (still dragging/in contact)
```
-- a direct, built-in confirmation that the tick counter claimed 60 seconds elapsed while only ~1 real second passed. This is the exact "rep6-style anomaly" class Phase 6/7 already observed once each and could not confirm the mechanism for with certainty (suspected: a burst of queued `tick()` callbacks firing back-to-back after ROS executor starvation). Phase 7's instrumentation was only added to the SEPARATION_MAX_WAIT_TICKS abort path; this session shows the *same* underlying issue also corrupts the CROUCH gate, a path with no such cross-check. Both trials' resulting launches fired from a body that had not genuinely completed its crouch stand-up or (per rep1's suspicious `attitude_error=0.0` recorded at the very first sample, likely a stale default rather than a real converged reading) its commanded yaw alignment -- producing the observed near-zero displacements and random azimuths.

**Action taken:** stopped the batch after confirming the pattern recurred in trial 2 (not on a single data point), rather than let 28 more trials run overnight producing the same class of invalid data. Did not attempt a fix -- the root cause (why THIS harness triggers it recurringly when Phase 6/7's batches mostly didn't) is not understood well enough to patch confidently, and this is exactly the kind of significant, poorly-understood, control-timing issue that warrants the same sign-off Phase 7's orientation bug got before any code changes, not a unilateral overnight patch. Recommend: instrument the CROUCH gate (and ideally all of `hopper_locomotion.py`'s tick-based timers generically) the same way Phase 7 instrumented SEPARATION_MAX_WAIT_TICKS, then figure out why this specific harness (fresh 4-node stack, immediate `target_yaw` publish before the ready-gate settles?) seems to trigger it more than Phase 6-8's other launch/self-righting batches did.

### 5.4 Priorities 5-7: not started, as scoped in the initial review

No new information here beyond what was already flagged before starting -- restated for completeness of this report. Priority 5 needs new baseline-assignment algorithms implemented, not a parameter sweep, plus full mission-length comparisons per policy. Priority 6 needs a real polyhedral shape model (not confirmed to exist in this repo) and a polyhedron-gravity implementation. Priority 7 needs a new comms simulation layer. None attempted.

## 6. Checkpoint

Requested: raw results, summary statistics (mean/std/n/95% CI), anomalies (not hidden), exact command/script, per batch.

- Priority 2: **delivered in full**, clean.
- Priority 3 (both mu and e): **delivered in full**, clean after one caught-and-fixed harness bug (SS5.2).
- Priority 4: **not delivered** -- attempted, stopped deliberately due to a data-integrity-blocking anomaly (SS5.3), not a time-budget issue. The 2 trials that did run are reported above explicitly as invalid, not omitted.
- Priority 1: **not attempted**, blocked on a design decision flagged before the overnight run began.
- Priorities 5-7: **not attempted**, out of overnight scope as flagged before the overnight run began.

**Overall: PASS on everything actually run** (2, 3a, 3b all clean, verified, anomaly-free) — **with Priority 4 explicitly incomplete and Priority 1 explicitly not started**, both for stated, defensible reasons rather than silently dropped.

## 7. Complete file listing

Every file under `phase8_overnight_batch/` as of this report, full paths, generated via `find docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch -type f | sort` (882 files):

```
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_dirhopm55_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_dirhopm55_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_dirhopm55_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu040_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu050_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu062_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu075_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_mu090_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep100.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep31.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep32.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep33.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep34.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep35.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep36.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep37.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep38.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep39.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep40.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep41.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep42.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep43.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep44.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep45.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep46.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep47.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep48.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep49.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep50.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep51.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep52.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep53.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep54.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep55.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep56.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep57.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep58.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep59.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep60.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep61.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep62.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep63.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep64.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep65.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep66.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep67.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep68.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep69.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep70.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep71.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep72.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep73.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep74.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep75.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep76.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep77.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep78.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep79.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep80.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep81.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep82.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep83.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep84.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep85.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep86.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep87.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep88.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep89.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep90.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep91.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep92.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep93.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep94.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep95.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep96.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep97.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep98.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep99.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/attitude_scout_1_n100_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_dirhopm55_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_dirhopm55_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_dirhopm55_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e010_drop1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e010_drop2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e010_drop3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e010_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e020_drop1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e020_drop2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e020_drop3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e020_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e040_drop1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e040_drop2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e040_drop3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_e040_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu040_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu050_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu062_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu075_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_mu090_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep100.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep31.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep32.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep33.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep34.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep35.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep36.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep37.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep38.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep39.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep40.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep41.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep42.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep43.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep44.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep45.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep46.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep47.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep48.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep49.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep50.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep51.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep52.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep53.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep54.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep55.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep56.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep57.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep58.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep59.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep60.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep61.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep62.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep63.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep64.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep65.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep66.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep67.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep68.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep69.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep70.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep71.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep72.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep73.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep74.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep75.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep76.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep77.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep78.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep79.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep80.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep81.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep82.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep83.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep84.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep85.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep86.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep87.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep88.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep89.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep90.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep91.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep92.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep93.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep94.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep95.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep96.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep97.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep98.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep99.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/bridge_scout_1_n100_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/directional_hop_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/directional_hop_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/directional_hop_validation.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/friction_sensitivity_sweep.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/friction_sweep_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/friction_sweep_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/generate_mu_variants.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/generate_restitution_variants.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_batch8_n100.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_dirhop.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_e010.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_e020.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_e040.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/gz_mu_sweep.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_dirhopm55_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_dirhopm55_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_dirhopm55_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu040_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu050_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu062_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu075_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_mu090_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep100.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep31.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep32.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep33.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep34.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep35.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep36.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep37.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep38.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep39.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep40.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep41.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep42.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep43.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep44.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep45.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep46.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep47.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep48.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep49.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep50.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep51.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep52.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep53.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep54.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep55.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep56.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep57.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep58.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep59.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep60.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep61.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep62.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep63.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep64.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep65.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep66.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep67.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep68.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep69.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep70.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep71.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep72.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep73.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep74.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep75.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep76.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep77.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep78.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep79.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep80.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep81.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep82.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep83.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep84.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep85.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep86.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep87.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep88.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep89.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep90.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep91.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep92.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep93.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep94.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep95.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep96.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep97.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep98.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep99.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/landing_scout_1_n100_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/launch_delivery_batch_n100.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/launch_delivery_n100_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/launch_delivery_n100_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_dirhopm55_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_dirhopm55_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_dirhopm55_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu040_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu050_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu062_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu075_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_mu090_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep100.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep31.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep32.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep33.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep34.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep35.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep36.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep37.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep38.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep39.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep40.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep41.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep42.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep43.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep44.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep45.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep46.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep47.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep48.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep49.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep50.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep51.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep52.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep53.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep54.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep55.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep56.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep57.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep58.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep59.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep60.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep61.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep62.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep63.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep64.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep65.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep66.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep67.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep68.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep69.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep70.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep71.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep72.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep73.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep74.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep75.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep76.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep77.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep78.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep79.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep80.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep81.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep82.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep83.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep84.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep85.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep86.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep87.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep88.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep89.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep90.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep91.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep92.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep93.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep94.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep95.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep96.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep97.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep98.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep99.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/loco_scout_1_n100_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/PHASE8_OVERNIGHT_REPORT.md
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sensitivity_sweep.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sweep_failed_missing_textures/bridge_scout_1_e010_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sweep_failed_missing_textures/gz_e010.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sweep_failed_missing_textures/restitution_sweep_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sweep_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/restitution_sweep_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/materials/textures/diffuse.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/materials/textures/heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/materials/textures/ryugu_heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/materials/textures/ryugu_shape.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/meshes/ryugu_terrain.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/meshes/ryugu_terrain.stl
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e010/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/materials/textures/diffuse.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/materials/textures/heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/materials/textures/ryugu_heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/materials/textures/ryugu_shape.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/meshes/ryugu_terrain.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/meshes/ryugu_terrain.stl
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e020/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/materials/textures/diffuse.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/materials/textures/heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/materials/textures/ryugu_heightmap.png
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/materials/textures/ryugu_shape.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/meshes/ryugu_terrain.obj
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/meshes/ryugu_terrain.stl
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/regolith_plane_e040/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu040/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu040/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu050/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu050/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu075/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu075/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu090/model.config
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models/spacehopper_mu090/model.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_worlds/ryugu_e010.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_worlds/ryugu_e020.sdf
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_worlds/ryugu_e040.sdf
```
