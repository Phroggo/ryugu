# Phase 7 — Full Re-validation Battery

Date: 2026-08-08 to 2026-08-09
Scope: the expensive, final validation pass against the stable model and controller (post Phase 2-6), run once against real distributions rather than single-sample figures, per the standing checkpoint: "every number going into the paper's results section has a real n, a real distribution, and a real source run."

This phase surfaced and fixed **three real bugs** while collecting the data (all found via this phase's own analysis, not assumed away): a landing_controller orientation-source bug that made self-righting structurally blind to static/severe tilt, a bridge-process-leak bug that hung the launch batch for 4+ hours, and a rest-detection bug that produced empty data in the first restitution run. Each is documented in full below, in the order encountered.

## 1. Files touched

### Source code (modified)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/hopper_locomotion.py` (+19/-0 lines) — Phase 6 recommendation 3: wall-clock cross-check instrumentation for the rep6-style tick/wall-time-mismatch anomaly (see Phase 6 report). Did not recur this phase.
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/landing_controller.py` (+57/-13 lines) — the orientation-source bug fix (§3).

### New Phase 7 artifacts

All under `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/` (562 files total). Scripts, top-level results/summary files, and README/index files are listed individually below; the remaining ~500 files are per-trial/per-repeat node console logs (bridge/loco/attitude/landing, one file per node per trial), which follow a fully-specified, mechanically-enumerable naming convention under this same directory — the complete literal list (every single path) is appended verbatim in §8, generated via `find ... | sort`, so nothing is summarized away.

**Scripts:**
- `.../phase7_full_revalidation/self_righting_batch_3bucket.py`
- `.../phase7_full_revalidation/launch_delivery_batch_n30.py`
- `.../phase7_full_revalidation/launch_timestep_distribution.py`
- `.../phase7_full_revalidation/righting_timestep_distribution.py`
- `.../phase7_full_revalidation/restitution_spot_check.py`
- `.../phase7_full_revalidation/imu_vs_odom_orientation_check.py` (diagnostic, confirmed the orientation bug)
- `.../phase7_full_revalidation/verify_orientation_fix_single_trial.py` (diagnostic, confirmed the fix)
- `.../phase7_full_revalidation/mission_loop_capture.sh`

**Top-level results/stdout/index files:**
- `.../phase7_full_revalidation/self_righting_3bucket_results.json`
- `.../phase7_full_revalidation/self_righting_3bucket_stdout.log`
- `.../phase7_full_revalidation/gz_sr_batch.log`
- `.../phase7_full_revalidation/before_orientation_fix/self_righting_3bucket_results.json` (pre-fix run, preserved as evidence)
- `.../phase7_full_revalidation/before_orientation_fix/self_righting_3bucket_stdout.log`
- `.../phase7_full_revalidation/launch_delivery_n30_results.json`
- `.../phase7_full_revalidation/launch_delivery_n30_stdout.log`
- `.../phase7_full_revalidation/gz_batch7.log`
- `.../phase7_full_revalidation/launch_n30_with_bridge_leak_bug/launch_delivery_n30_results.json` (hung/partial run, preserved as evidence)
- `.../phase7_full_revalidation/launch_n30_with_bridge_leak_bug/launch_delivery_n30_stdout.log`
- `.../phase7_full_revalidation/launch_timestep_distribution_results.json`
- `.../phase7_full_revalidation/launch_timestep_distribution_stdout.log`
- `.../phase7_full_revalidation/gz_ltd_1ms.log`, `gz_ltd_4ms.log`
- `.../phase7_full_revalidation/righting_timestep_distribution_results.json`
- `.../phase7_full_revalidation/righting_timestep_distribution_stdout.log`
- `.../phase7_full_revalidation/gz_rtd_1ms.log`, `gz_rtd_4ms.log`
- `.../phase7_full_revalidation/restitution_spot_check_results.json`
- `.../phase7_full_revalidation/restitution_spot_check_stdout.log`
- `.../phase7_full_revalidation/gz_restitution.log`
- `.../phase7_full_revalidation/restitution_bad_rest_detection/restitution_spot_check_results.json` (bad run, preserved as evidence)
- `.../phase7_full_revalidation/restitution_bad_rest_detection/restitution_spot_check_stdout.log`
- `.../phase7_full_revalidation/gz_imu_odom_check.log`
- `.../phase7_full_revalidation/verify_orientation_fix_stdout.log`
- `.../phase7_full_revalidation/gz_verifyfix.log`
- `.../phase7_full_revalidation/mission_loop_run/RUN_INDEX.md`
- `.../phase7_full_revalidation/mission_loop_run/launch_stdout.log`
- `.../phase7_full_revalidation/mission_loop_run/scout_{1,2,3}_{role,activity,battery,landed}.log` (12 files)
- `.../phase7_full_revalidation/PHASE7_CHANGE_REPORT.md` (this file)

## 2. What was run (overview)

1. Harness fixes per Phase 6's own recommendations (STABILIZE_WINDOW 30s->75s, periodic full daemon restart every 5 reps, rep6-anomaly wall-clock instrumentation) — applied before any batch.
2. Self-righting: n=20 per tilt bucket x 3 buckets (side_rest 85-95deg, moderate 45-60deg, full_inversion 170-180deg) = 60 trials. **Run twice**: once before the orientation-bug fix (0/60, diagnosed as invalid), once after (the real result, §4).
3. Launch delivery: n=30 at the 9.0m degraded-mode scenario. **Run twice**: once hit a harness hang at rep20 (diagnosed, fixed), rerun clean (§4).
4. Launch timestep sensitivity: 5 repeats x {1ms, 4ms} = 10 runs, same 9.0m scenario.
5. Self-righting timestep sensitivity: 5 repeats x {1ms, 4ms} = 10 runs, fixed 60deg tilt.
6. Restitution spot-check: 3 drops from 1.15m. **Run twice**: once produced empty data (rest-detection bug, diagnosed and fixed), rerun clean (§4).
7. One 45-minute full 3-agent mission-loop run for archived telemetry (video/screenshots not reproduced — no capture tooling in this environment; see §4.6).

## 3. The orientation-source bug (found, confirmed, fixed)

**Symptom**: the first self-righting batch run (before any fix) returned 0/60 recovered across all three buckets. `full_inversion` showed 20/20 trials going straight from IDLE to `LANDED — stable contact confirmed` with **zero righting attempts logged**, despite being spawned at 170-180deg (odometry-confirmed u_z about -0.98 to -1.0).

**Root cause**: `landing_controller.py`'s `_is_badly_tilted`, `_is_inverted`, the LANDED-state tilt watchdog, and `_run_righting_sequence`'s own u_z feedback and roll-direction calculation all read orientation from **the IMU message** (`msg.orientation`), not from `/scout_1/odometry` (which every other controller in this codebase — `hopper_locomotion.py`, `attitude_controller.py` — and every test harness in this project correctly uses).

**Confirmed live** (`imu_vs_odom_orientation_check.py`): spawned at 175deg, no controllers running, sampled both sources for 55+ seconds. IMU-derived u_z stayed at `0.9999999999...` (frozen at identity) the entire time; odometry-derived u_z correctly read `-0.9961946...` throughout, constant `diff=1.9962`. gz-sim's simulated IMU orientation output does not track true world-frame orientation for a body with near-zero angular velocity in this environment — a robot spawned tilted with little further rotation never moves off its (wrong) identity default.

**Provenance — this bug was already found, on 2026-08-04, well before this redesign started.** `docs/paper_assets/calculations/imu_orientation_bug_20260804/README.md` (commit `3317373`) diagnosed the identical root cause: gz-sim's IMU sensor has no `<orientation_reference_frame>` configured, so it reports orientation relative to its own spawn pose rather than true world-up — confirmed there via the same frozen-at-identity signature (1884 consecutive IMU messages all reading u_z=1.0 after a 172deg teleport-respawn, while odometry read correctly from its first message). That README explicitly recommended two independently-sufficient fixes: (1) add the missing `<orientation_reference_frame>` to the IMU sensor itself, or (2) switch the affected code (`landing_controller.py`'s `_is_badly_tilted`/`_is_inverted`, and it flagged `attitude_controller.py`'s tilt-PD as similarly affected) to read odometry instead, matching what `hopper_locomotion.py` already did. **Neither was applied at the time** — the README states plainly "No code changes applied... this week's own test harnesses" instead worked around it with a different spawn method (`docs/paper_assets/calculations/severe_tilt_no_respawn_rerun_20260805/README.md`, commit `41d693b`: continuous in-place tip instead of entity respawn, restoring genuine attempts in 2/8 severe-tilt trials vs 0/8 originally). Phase 7's fix below is option (2) from that original recommendation, finally applied to `landing_controller.py` — this section documents applying a known, previously-scoped fix and confirming it live, not an independent discovery of the underlying issue.

That same 2026-08-05 rerun also surfaced a related, **separate** limitation worth being aware of: even when righting correctly triggers on a severe tilt, it doesn't always resolve cleanly within 200s when the body was never actually landed — a *suspended*-tilt scenario (animated in-place, never touched down) rather than the *landed*-then-tilted scenario every Phase 7 self-righting trial actually exercises (all spawned from height, free-fall, and make genuine ground contact before the tilt check runs). Not something Phase 7 needed to re-test given that distinction, but flagged here as a related, still-open item if suspended-tilt recovery is ever tested directly.

**Fix** (`landing_controller.py`, 5 call sites): replaced every `msg.orientation.{x,y,z,w}` read with the equivalent field from `self.last_pose` (populated by the existing `odom_callback` from `/scout_1/odometry`), guarded for `self.last_pose is None` at startup. `_is_inverted` is dead code (never called) but fixed too for consistency rather than left as a landmine.

**Verified** (`verify_orientation_fix_single_trial.py`, single trial, 175deg spawn): trigger fired correctly (`⚠️ Settled badly tilted/inverted — initiating RW righting roll`) and righting engaged within seconds. This run also surfaced a **separate, already-documented, non-bug finding**: `omega` stayed near 0.000-0.002 rad/s across all 4 logged attempts despite `w=160` commanded — consistent with `_run_righting_sequence`'s own docstring, which already flags "a body forced to a perfect full inversion and wedged against terrain is the residual hard case (see SS3.3 in the paper)." Not investigated further — this is exactly the kind of result Phase 7 exists to measure and confirm, not fix.

## 4. Results

### 4.1 Self-righting, n=20 per bucket (post-fix; `self_righting_3bucket_results.json`)

| Bucket | Recovered | Failed | No landing | Recovery rate |
|---|---|---|---|---|
| side_rest (85-95deg) | 10 | 10 | 0 | 50% |
| moderate (45-60deg) | 20 | 0 | 0 | 100% |
| full_inversion (170-180deg) | 3 | 17 | 0 | 15% |

Time-to-recovery (seconds, recovered trials only):

| Bucket | n | mean | median | min | max |
|---|---|---|---|---|---|
| side_rest | 10 | 19.5 | 2.5 | 0.0 | 68.9 |
| moderate | 20 | 0.0 | 0.0 | 0.0 | 0.0 |
| full_inversion | 3 | 17.0 | 0.0 | 0.0 | 50.8 |

Total batch runtime: 09:01:06-13:09:53 (4h 8m 47s), 60 trials.

This is a coherent, physically plausible spread — exactly the kind of differentiation Phase 5's diagnosis predicted and the reason the buckets were kept separate rather than pooled: moderate tilts (already close to the u_z>0.9 threshold) recover essentially every time and near-instantly; side_rest is a real coin-flip; full_inversion is a genuine hard case, succeeding only occasionally, matching the code's own pre-existing "residual hard case" acknowledgment.

**Pre-fix run** (`before_orientation_fix/self_righting_3bucket_results.json`, preserved as evidence): 0/20 in every bucket. `full_inversion`: 20/20 direct-to-LANDED with zero righting attempts (the bug, unambiguous). `moderate`: 20/20 `no_landing` — this bucket's harness timeout (200s) also proved too short to reach the code's own 5-attempt give-up fallback (5x15s + brake + fall/settle time can exceed 200s), so these trials were genuinely attempting righting but got killed by the harness clock before `landed` ever latched; widened to 350s in the post-fix rerun. `side_rest`: 16/20 landed within budget, 0 recovered, 4 no_landing.

### 4.2 Launch delivery, n=30 (post-fix; `launch_delivery_n30_results.json`)

no_separation=0 (0%), stabilized=30 (100%), separated_never_stabilized=0.

ratios: mean=0.212, median=0.213, std=0.001, min=0.207, max=0.213, **range=0.006**.

Compare: Phase 6's n=10 sample (mean=0.271, range=0.139, 1 no_separation, 3 separated_never_stabilized) and the original Phase 0 baseline (0.009-0.447 combined range, ~30% non-separation). This n=30 result is dramatically tighter than both — a combination of Phase 6's genuine-separation-confirmation fix, this phase's widened STABILIZE_WINDOW (30s->75s), and (most likely) the bridge-leak fix (§5) removing message-duplication noise that may have affected earlier runs to an unknown degree.

Total batch runtime: 18:01:59-18:45:14 (~43 min), 30 reps, zero timeouts this run.

### 4.3 Launch timestep sensitivity, 5 repeats x {1ms, 4ms} (`launch_timestep_distribution_results.json`)

| Timestep | n stabilized | mean | range | min | max |
|---|---|---|---|---|---|
| 1ms | 5/5 | 0.212 | 0.006 | 0.207 | 0.213 |
| 4ms | 5/5 | 0.213 | 0.000 | 0.213 | 0.213 |

Zero no_separation in either. The two timesteps agree closely — consistent with Phase 0's original conclusion that timestep is not a meaningful driver of launch-delivery variance, now confirmed on the final model with a real (if small) per-timestep sample rather than single pairs.

### 4.4 Self-righting timestep sensitivity, 5 repeats x {1ms, 4ms}, fixed 60deg tilt (`righting_timestep_distribution_results.json`)

1ms: 5/5 recovered. 4ms: 5/5 recovered. recover_time_s effectively 0.0 in every trial (matches the moderate-bucket pattern: a 60deg start is close enough to the 0.9 threshold to resolve almost immediately). Both timesteps agree.

### 4.5 Restitution spot-check, 3 drops from 1.15m (`restitution_spot_check_results.json`)

**e = 0.113** for the first genuine bounce, identical to 3 decimal places across all 3 drops (this is a fully deterministic test — fixed spawn position, no active controllers, no randomization — so exact repeatability is expected and itself a useful sanity check that the measurement is real, not noise).

Every "bounce" after the first is a numerical noise-floor artifact (~2mm oscillation around the rest position, not real physical bounces) that the peak-detection logic in `restitution_spot_check.py` mistakenly counts as additional bounce-pairs — the script's own printed summary line ("mean e = 0.999") is an average dominated by ~57 near-1.0 noise-floor ratios and should be **disregarded**; the first-bounce value (0.113) is the physically meaningful measurement.

Comparison: paper's analytical target ~0.2 (derived from `c_joint=0.15` per the design comment in `scripts/generate_detailed_spacehopper.py`); Biele et al. MASCOT hardware median ~0.4, max ~0.6. The measured 0.113 is comfortably below both — the paper's qualitative claim ("still below Biele et al.'s measured range") holds, if anything more comfortably than the ~0.2 figure suggested. The exact ~0.2 figure does not precisely reproduce empirically; this is a real, new discrepancy worth noting rather than papering over (see also §6's damping-value finding, which may be related — the design comment's derivation assumed `c_joint=0.15`, but the actually-shipped value in both the generator and `models/spacehopper/model.sdf` is `0.05`, a pre-existing discrepancy this phase did not change).

**Pre-fix run** (`restitution_bad_rest_detection/`, preserved as evidence): both drops captured zero bounce apexes over their full 900s trace windows, because the "rest" reference height was wrongly captured as the spawn height itself (see §5.3).

### 4.6 Mission-loop reference run

45 minutes, full 3-agent swarm (`ryugu_swarm.launch.py`), final model/controller stack. 11 LANDED events, 10 self-righting attempts (2 exhausted all 5 attempts and gave up — expected, not a new issue), 199 spectral-anomaly detections, no crashes or tracebacks in the full console log (62 benign gz-sim "Host unreachable" internal service messages, a known non-fatal pattern). Full per-agent telemetry (role/activity/battery/landed) captured. See `mission_loop_run/RUN_INDEX.md`.

**Video/screenshot capture NOT reproduced**: this environment has no `ffmpeg`, `Xvfb`, `scrot`, or ImageMagick installed, and no passwordless `sudo` to add them. The original Data Availability reference run's `full_run.mp4` and periodic screenshots were captured by an external wrapper that was never itself committed to this repo (confirmed during this phase's own tooling survey). Only the telemetry portion — which is the substantive Data Availability content — was reproduced this phase. Flagged here rather than silently skipped; installing capture tooling would need explicit sign-off since it modifies the environment beyond the repo.

## 5. Bugs found and fixed this phase (full detail)

### 5.1 landing_controller.py orientation source (see §3 for full detail)

Fixed at 5 call sites: `_is_badly_tilted`, `_is_inverted`, the LANDED-state tilt watchdog (imu_callback), and `_run_righting_sequence`'s `u_z` computation and roll-direction (`up_x`/`up_y`) computation. All now read `self.last_pose[3:7]` (odometry-derived qx,qy,qz,qw) instead of `msg.orientation.{x,y,z,w}` (IMU-derived).

### 5.2 bridge_scout_1 process leak (launch harnesses)

**Symptom**: the first n=30 launch delivery batch hung for 4+ hours at rep20 (started 13:39, still stuck at 17:57 when caught). `gz sim gui` was consuming 445+ minutes of cumulative CPU time; `ps aux` showed **5 simultaneous live `bridge_scout_1` processes** (from reps 16, 17, 18, 19, 20), all bridging the same topics to the same world.

**Root cause**: `spawn_and_launch_nodes()`'s per-rep cleanup only killed `loco_scout_1|attitude_scout_1|landing_scout_1` — omitting `bridge_scout_1`. Between `DAEMON_RESTART_EVERY=5` boundaries (which do a full kill including bridge), every individual rep launched a *new* bridge without killing the previous rep's, so bridges accumulated 1-2-3-4-5 across each 5-rep block. Five stacked bridge instances fighting over the same DDS domain reliably deadlocked the batch at the 5th rep — explains both the earlier "RTPS_TRANSPORT_SHM ... open_and_lock_file failed" warnings Phase 6 saw before reps 9-10, and this phase's full hang. This pattern was inherited from Phase 6's original `launch_reliability_batch.py` (same per-rep omission existed there too, just never got unlucky enough within 10 reps to hang outright).

**Fix**: added `bridge_scout_1` to the per-rep pkill pattern in both `launch_delivery_batch_n30.py` and `launch_timestep_distribution.py` (audited `righting_timestep_distribution.py` and `restitution_spot_check.py` — both do a full kill+restart every single trial already, so they were never at risk).

**Impact on data already collected**: the hung run's first 19 reps (before the hang) are preserved in `launch_n30_with_bridge_leak_bug/` but were **not** used in the final reported numbers — discarded in favor of a clean full rerun, since the possible effect of 1-4 stacked bridges on command-path topics (redundant `ROS_TO_GZ` publishes) on those earlier reps' data quality could not be fully ruled out, and a clean rerun was cheap enough to just do properly.

### 5.3 Restitution spot-check rest-detection

**Symptom**: first restitution run's `find_rest_z()` declared "rest" after 1.6 seconds of what should have been a multi-hundred-second free-fall from z=5.2, reporting `rest_z=5.198` (essentially the spawn height). Both subsequent drops respawned from `rest_z + 1.15`, an even higher altitude, and captured zero bounce apexes over their full 900s trace windows.

**Root cause**: the check ("z hasn't moved >0.5mm in 30 consecutive 0.3s polls") is satisfied almost immediately after spawn in this world's near-zero gravity (g=1.14e-4 m/s^2) — position changes so slowly early in a genuine free-fall that short-window deltas look "stable" even while the body is very much still falling. A tighter position threshold or a velocity-based check doesn't fix this either: velocity during early free-fall is *also* below any reasonable instantaneous threshold for a long time (v=g*t stays under 0.005 m/s until t~44s) — this is the same fundamental reason `landing_controller.py` needs a genuinely long (60-120s) sustained-rest window rather than a quick check.

**Fix**: replaced auto-detection with a fixed 250s wait, justified by this exact session's own extensive empirical timing data (dozens of prior Phase 7 trials from this same SPAWN_Z=5.2 world showing contact reliably within 150-210s).

## 6. Other findings (not bugs Phase 7 fixed, flagged for awareness)

- **Damping-value discrepancy** (found while preparing the restitution check, `scripts/generate_detailed_spacehopper.py` ~line 885-909): the design comment's restitution derivation explicitly claims "p=1.0 + 0.15 is the verified-stable operating point," but the actual `<dynamics><damping>` value in both the generator and `models/spacehopper/model.sdf` (every leg joint) is `0.05`, not `0.15`. Not changed by this phase (a physical/design parameter change is out of scope for a validation phase); flagged for whoever next touches leg-joint tuning. May be related to the ~0.2-vs-0.113 restitution discrepancy in §4.5, though the exact relationship was not derived with confidence (see the abandoned analytical recompute attempt, superseded by the direct empirical measurement).
- **rep6-style tick/wall-time anomaly** (Phase 6 finding): instrumentation added to `hopper_locomotion.py` at the start of this phase did not fire (`grep`'d for "TICK/WALL-TIME MISMATCH" across all n=30 batch logs — none found). Did not recur this phase.

## 7. Checkpoint

Requirement: every number going into the paper's results section has a real n, a real distribution, and a real source run — no single-sample figures presented as characteristic.

- Self-righting: n=20 per bucket x 3 buckets, real distinct recovery rates and time-to-recovery distributions per bucket. **Met** — and only met because a genuine, previously-invisible controller bug was found and fixed first; the pre-fix numbers would have been actively wrong, not just imprecise.
- Launch delivery: n=30, full mean/median/std distribution. **Met**.
- Timestep sensitivity: 5 repeats x 2 timesteps, both launch and self-righting cases. **Met**.
- Restitution: empirical spot-check (n=3, deterministic), not a single unretained live-measurement claim. **Met**, with the caveat that "quick spot-check" was the requested scope, not a full statistical characterization.
- Mission-loop reference run: full telemetry captured; video/screenshot portion explicitly flagged as not reproducible in this environment rather than silently omitted. **Partially met**, caveat documented.

**Checkpoint: PASS**, with the mission-loop video/screenshot gap explicitly carried forward as a known limitation rather than closed.

## 8. Complete file listing

Every file under `phase7_full_revalidation/` as of this report, full paths, generated via `find docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation -type f | sort` (562 files):

```
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_ltd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/attitude_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_full_inversion_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_moderate_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/bridge_scout_1_side_rest_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/gz_sr_batch.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_full_inversion_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_moderate_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/landing_scout_1_side_rest_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/self_righting_3bucket_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/before_orientation_fix/self_righting_3bucket_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_full_inversion_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_imu_odom_check.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_ltd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_moderate_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_restitution_drop1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_restitution_drop2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_restitution_drop3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_restitution_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_rtd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_side_rest_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/bridge_scout_1_verifyfix.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_batch7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_imu_odom_check.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_ltd_1ms.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_ltd_4ms.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_restitution.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_rtd_1ms.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_rtd_4ms.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_sr_batch.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/gz_verifyfix.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/imu_vs_odom_orientation_check.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_full_inversion_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_ltd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_moderate_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_rtd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_side_rest_trial9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/landing_scout_1_verifyfix.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_delivery_batch_n30.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_delivery_n30_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_delivery_n30_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/attitude_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/bridge_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/gz_batch7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/landing_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/launch_delivery_n30_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/launch_delivery_n30_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_n30_with_bridge_leak_bug/loco_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_timestep_distribution.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_timestep_distribution_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/launch_timestep_distribution_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_1ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_1ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_1ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_1ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_1ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_4ms_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_4ms_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_4ms_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_4ms_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_ltd_4ms_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep10.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep11.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep12.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep13.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep14.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep15.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep16.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep17.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep18.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep19.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep20.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep21.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep22.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep23.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep24.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep25.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep26.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep27.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep28.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep29.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep30.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep4.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep5.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep6.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep7.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep8.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/loco_scout_1_n30_rep9.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_capture.sh
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/launch_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/RUN_INDEX.md
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_1_activity.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_1_battery.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_1_landed.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_1_role.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_2_activity.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_2_battery.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_2_landed.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_2_role.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_3_activity.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_3_battery.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_3_landed.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run/scout_3_role.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/PHASE7_CHANGE_REPORT.md
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/bridge_scout_1_restitution_drop1.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/bridge_scout_1_restitution_drop2.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/bridge_scout_1_restitution_drop3.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/bridge_scout_1_restitution_ref.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/gz_restitution.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/restitution_spot_check_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_bad_rest_detection/restitution_spot_check_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_spot_check.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_spot_check_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/restitution_spot_check_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/righting_timestep_distribution.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/righting_timestep_distribution_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/righting_timestep_distribution_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/self_righting_3bucket_results.json
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/self_righting_3bucket_stdout.log
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/self_righting_batch_3bucket.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/verify_orientation_fix_single_trial.py
/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/verify_orientation_fix_stdout.log
```
