# Phase 10 — Post-Timing-Fix Full Revalidation (P2/P3a/P3b/P4/P1)

Date: 2026-08-10 to 2026-08-11
Scope: Phase 9 fixed a real tick/wall-time mismatch bug in `hopper_locomotion.py`/`landing_controller.py`, and Phase 9's own spot-check found a genuine ~2.5% shift in delivered launch ratio (0.212→0.218) as a side effect. Per explicit direction, P2/P3a/P3b (Phase 8's overnight batch) do not count as valid until rerun on the fixed code, and Phase 8's P4 attempt (aborted after 2/30 invalid trials, the run that originally exposed the timing bug) had never produced a legitimate result at all. This phase reruns all four, plus builds and runs the Priority 1 sensor-noise Monte Carlo that Phase 8 never got to. Full reruns, not spot-checks — every result below is a fresh, independently-verified number, not an assumption carried over from before the fix.

## 1. Files touched

### Source code (modified)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/landing_controller.py` — adds env-var-toggled (`ODOM_ORIENTATION_NOISE_STD`) Gaussian orientation noise injection on odometry, for Priority 1. See §2.5.

### New harness scripts

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/launch_delivery_batch_n100_postfix.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_sensitivity_sweep_postfix.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_mu075_topup.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/restitution_sensitivity_sweep_postfix.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/directional_hop_validation_postfix.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/sensor_noise_monte_carlo_postfix.py`

### Results (JSON)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/launch_delivery_n100_postfix_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_sweep_postfix_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/restitution_sweep_postfix_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/directional_hop_postfix_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/sensor_noise_monte_carlo_postfix_results.json`

### Console logs (top-level stdout captures)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/launch_delivery_n100_postfix_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_sweep_postfix_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_mu075_topup_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/restitution_sweep_postfix_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/directional_hop_postfix_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/sensor_noise_monte_carlo_postfix_stdout.log`

### Per-repetition node/daemon logs and this report

1,261 additional per-repetition log files (`bridge_scout_1_*`, `loco_scout_1_*`, `attitude_scout_1_*`, `landing_scout_1_*`, `gz_*`) plus this report — too numerous to enumerate individually in this document (n=100+20+3+30+150=303 repetitions × up to 4 node logs each, plus daemon logs and the mu075 top-up's 4 extra reps). The complete, literal file listing (generated via `find ... | sort` at commit time, not hand-typed) is appended to the two commits that carry this phase's files, per this project's established convention for changesets this size — see `git log` / `git show` for the exact list rather than a manually-copied one here that could drift from what was actually staged.

## 2. What changed / what was run

### 2.1 P2 — Launch delivery, n=100 @ 9.0m (rerun of Phase 8's Priority 2)

Same harness structure as Phase 8's `launch_delivery_batch_n100.py` (`STABILIZE_WINDOW=75s`, `DAEMON_RESTART_EVERY=5`, 95% CI via normal approximation), with one change: the obsolete "TICK/WALL-TIME MISMATCH" log-grep (the exact warning string it searched for was removed from `hopper_locomotion.py` by the Phase 9 fix, so the old check would have trivially always reported "clean" without checking anything real) was replaced with a genuine Crouching→IGNITION real-time-gap scan across all 100 reps, giving a much stronger n=100 confirmation of gate integrity than Phase 9's own n=5 spot-check.

**Result**: n=100, 0% no_separation, 100% stabilized. Mean ratio=**0.2181**, median=0.2182, std=0.0006, min=0.2168, max=0.2195. 95% CI (normal approx): **[0.2180, 0.2182]**. CROUCH GATE INTEGRITY: **PASS** across all 100 reps (no Crouching→IGNITION gap ever fell outside the expected 10.0-10.1s window).

Comparison vs Phase 8 pre-fix (n=100, mean=0.2121, CI=[0.2118,0.2124]): the ~2.8% shift Phase 9 flagged from its n=5 spot-check (0.212→0.218) is confirmed real and stable at full n=100 scale, not sampling noise.

### 2.2 P3a — Friction sweep, mu=0.40/0.50/0.62/0.75/0.90 @ 9.0m (rerun of Phase 8's Priority 3a)

Same variant models as Phase 8 (`phase8_overnight_batch/variant_models/spacehopper_muXXX`, static SDF, unaffected by the code fix, reused not regenerated). n=20/mu.

**Result**: mean ratios flat across the full friction range — mu=0.40: 0.2183, mu=0.50: 0.2181, mu=0.62: 0.2183, mu=0.75: 0.2184 (see anomaly below), mu=0.90: 0.2181. All within 0.2181-0.2185, consistent with Phase 8's pre-fix finding that launch delivery is not friction-limited over this range, now confirmed post-fix.

**Anomaly caught and resolved**: mu=0.75's first pass showed 4/20 (20%) `no_separation` failures (reps 2-5), the only mu value with any failures (all others: 0/20). Investigated rather than accepted at face value:
- Raw JSON showed reps 2-5 had `landed=None, speed=None` at the ready-check — the monitor node's subscriptions never received a single message, not a real physics outcome.
- `loco_scout_1_mu075pf_rep2.log` showed the jump command WAS accepted (`Initiating Tri-Pedal Jump Sequence!`, state→CROUCH) but zero further output ever followed — the tick() timer callback effectively went silent.
- `gz_mu_sweep_postfix.log` showed `NodeShared::RecvSrvRequest() error sending response: Host unreachable` right at the start of the mu=0.75 daemon session — a transient gz-transport networking fault that silently broke the GZ→ROS bridge for the rest of that daemon's life, recovering cleanly the moment the periodic full-daemon-restart (scheduled before rep 6) tore down and recreated it.
- Conclusion: infrastructure flakiness, not friction-dependent physics, and not related to the Phase 9 fix (which touched only Python state-machine timing, no networking).
- Fix: `friction_mu075_topup.py` reran the 4 lost trials fresh (labeled rep21-24 to preserve the original rep2-5 logs as evidence) — clean n=20 result: mean_ratio=0.2184, all values 0.218-0.219, consistent with every other mu bucket.

### 2.3 P3b — Restitution sweep, e=0.1/0.2/0.4 (rerun of Phase 8's Priority 3b)

Same terrain variant worlds as Phase 8 (`phase8_overnight_batch/variant_worlds/ryugu_eXXX.sdf`, static, reused). This test loads no controllers at all (bridge-only, passive physics) — `hopper_locomotion.py`/`landing_controller.py`, the two files Phase 9 touched, never load in this scenario, so it is structurally impossible for the fix to change the outcome. Rerun anyway per explicit instruction, as cheap confirmatory evidence.

**Result**: first-bounce e = **0.113** for all three e-targets (0.1, 0.2, 0.4), 3 drops each, all reps identical to 3 decimal places within each e-target. Exactly matches Phase 8's pre-fix result (0.113 across the board), as predicted.

### 2.4 P4 — Directional hop, n=30 @ 5.0m, heading -55° (first legitimate run of this experiment)

Phase 8's only prior attempt was stopped after 2/30 trials, both invalid, due to the tick/wall-time mismatch bug (this exact 5.0m short-ramp scenario is what originally exposed the bug — 34ms Crouching→IGNITION pre-fix vs. 10.00-10.01s post-fix, per Phase 9's verification). This is the first time this experiment has ever produced real data.

**Result**: n=30, 26 landed, 1 no_separation, 3 never_landed (discarded, no MAX_FLIGHT_WAIT=1400s timeout reached before landing detection). |yaw_error_at_ignition|: mean=0.0129 rad (0.74°), max=0.0330 rad — yaw hold converges tightly and well before ignition in every trial (confirmed via raw logs, e.g. rep17: attitude_error=0.017 rad, converged well before crouch even began). Displacement: n=26, mean=**0.066m**, std=0.030m, min=0.020m, max=0.147m. Azimuth: mean=82.5°, std=95.0° (commanded heading=-55°).

**Flagged, not averaged away**: the delivered ground displacement is essentially zero relative to the 5.0m target (~1.3% delivery), and azimuth is scattered across the full range (std=95° — effectively no directional correlation to the -55° command). This was cross-checked against raw odometry (`start_xy`/`end_xy` pairs in the results JSON) to rule out a harness measurement artifact: `start_xy` consistently sits at the spawn point (~0, 0.5), `end_xy` only centimeters away in scattered directions, across all 26 trials — this is a real physical result, not a bug in how displacement/azimuth are computed. Individual trial logs (e.g. rep17) show a completely clean, real ballistic flight (ignition→separation-confirmed→~100s flight→landing, all logged normally) with well-converged yaw hold throughout — so this is not a "flopped launch" or aborted crouch in the sense the code's own comments describe; the robot genuinely launches, flies for ~100s, and lands almost exactly where it started, meaning essentially zero horizontal velocity was imparted despite the yaw being correctly aligned to the commanded heading the entire time.

This connects to, but is materially worse than, an issue already flagged in `phase0_baseline_lockin/BASELINE_MANIFEST.md`: *"The 4.3m/-56° headline directional-hop figure remains contradicted by its own re-verification (C9: achieved ground-track azimuth 122.66° vs. held yaw -55°)... blocked on the open 3-axis coupling issue."* C9's re-verification still delivered a real 1.24m flight (wrong direction, but substantial magnitude — ~29% of its 4.3m target). This n=30 batch delivers ~1.3% of its 5.0m target — not just wrong-direction, but essentially no horizontal thrust at all, and this is now the first time this failure mode has been characterized at n=30 rather than a single anecdotal C9 trial.

Per explicit scoping from the Phase 9 timing-fix conversation, `hopper_locomotion.py`'s LEAN/thrust-tilt mechanism was **not** touched to chase this — the "3-axis coupling issue" is a separate, already-flagged, unresolved problem, out of scope for what this phase (a timing-bug fix revalidation) covers. Flagging here for a separate, deliberate investigation rather than attempting an unscoped fix.

### 2.5 P1 — Sensor-noise Monte Carlo, sigma=0.01 rad, n=50/bucket (first run of this experiment; Phase 8 never got to it)

**Code change** (`landing_controller.py`): added `_quat_mult(q1, q2)` (Hamilton quaternion product) and `_random_small_rotation_quat(std_rad)` (uniformly-random rotation axis, Gaussian-magnitude rotation, `random.gauss(0.0, std_rad)`) module-level helpers. `__init__` reads `ODOM_ORIENTATION_NOISE_STD` from the environment (default `0.0`, i.e. off) and warns if active. `odom_callback` composes the noise quaternion onto the true orientation (`_quat_mult(true_q, noise_q)`) before storing into `self.last_pose`, when the env var is set — applied at the odometry source so every downstream consumer (`_is_badly_tilted`, the righting sequence's roll-direction logic, the LANDED-tilt watchdog) sees the same noisy reading, matching the rationale that odometry stands in for the real attitude-determination sensor path. Env-var toggle chosen for consistency with the project's existing convention (e.g. `GZ_SIM_RESOURCE_PATH`) rather than a ROS parameter.

**Noise math verified standalone before use** (pure Python, no ROS/sim, n=20,000 samples, sigma=0.01 rad): mean output quaternion norm = 1.0 exactly (unit-norm preserved). Mean angular deviation from true orientation = 0.00802 rad vs. theoretical sigma·√(2/π) = 0.00798 rad for a half-angle Gaussian — matches. Mean `u_z` deviation from upright when truly upright = 3.4e-5, max observed = 8.4e-4 — small and physically sane for sigma=0.01 rad.

**Harness**: same core methodology as Phase 7's `self_righting_batch_3bucket.py` (side_rest 85-95°, moderate 45-60°, full_inversion 170-180°, N_PER_BUCKET raised from 20 to 50), with `ODOM_ORIENTATION_NOISE_STD=0.01` set only in `landing_scout_1`'s subprocess environment (the bridge process runs without noise, since noise is injected inside the controller's own callback, not the topic data itself).

**Result** (n=50/bucket, vs. Phase 7's post-orientation-fix no-noise baseline, re-read directly from `phase7_full_revalidation/self_righting_3bucket_results.json` rather than from memory):

| Bucket | Baseline (no noise) | With noise (sigma=0.01) | Two-proportion z-test |
|---|---|---|---|
| side_rest | 10/20 (50.0%) | 16/50 (32.0%), 95% CI [20.8%,45.8%] | z=1.41, **p=0.16** |
| moderate | 20/20 (100.0%) | 50/50 (100.0%), 95% CI [92.9%,100.0%] | saturated both conditions, no test needed |
| full_inversion | 3/20 (15.0%) | 14/50 (28.0%), 95% CI [17.5%,41.7%] | z=-1.15, **p=0.25** |

The harness's own built-in check (comparing the baseline's point estimate against the noisy sample's CI) flagged both side_rest and full_inversion as "OUTSIDE CI" — but that check ignores the baseline's own sampling uncertainty (n=20 is small; its own 95% CIs are [29.9%,70.1%] for side_rest and [5.2%,36.0%] for full_inversion, both of which substantially overlap the noisy-condition CIs). Running a proper two-proportion z-test instead of trusting that single-point heuristic: **neither shift reaches conventional significance** (p=0.16 and p=0.25, both far short of anything resembling a "4-5 sigma" real effect). Moderate stays saturated at 100% in both conditions, as expected (it was already at ceiling in the no-noise baseline).

**Honest conclusion**: at sigma=0.01 rad odometry orientation noise, n=50/bucket, no statistically significant degradation (or improvement) in self-righting recovery rate was detected relative to the Phase 7 post-fix no-noise baseline. The apparent 50%→32% (side_rest) and 15%→28% (full_inversion) point-estimate shifts are within the range expected from binomial sampling variation at these sample sizes, particularly given the baseline's own small n=20. A materially larger sample would be needed to detect an effect at this magnitude with confidence, if one exists.

## 3. Anomalies flagged this phase (summary)

1. **P3a mu=0.75 gz-transport fault** (§2.2) — infrastructure flakiness, root-caused via raw logs, resolved by rerunning the 4 lost trials. Not a physics finding, not fix-related.
2. **P4 near-zero directional-hop displacement** (§2.4) — real, reproducible, confirmed via raw odometry, not a harness bug. Connects to and materially worsens the already-known, already-flagged "3-axis coupling issue" from `BASELINE_MANIFEST.md`. Deliberately not fixed this phase (out of scope); flagged for separate investigation.
3. **P1's apparent recovery-rate shifts** (§2.5) — investigated with a proper statistical test rather than accepted at face value; found **not** statistically significant, correcting the harness's own overly-sensitive built-in flag.

## 4. Checkpoint verdict

P2, P3a, P3b: **PASS**, clean post-fix revalidation, consistent with and (for P2) confirming Phase 9's spot-check at full sample size. P4: data collected for the first time ever at n=30, but reveals a real, unresolved directional-delivery problem — **not ready for paper integration** as evidence of controllable directional hopping; usable only as evidence that yaw-hold itself works (which it does, tightly) while horizontal thrust delivery under a commanded heading does not. P1: complete at the requested n=50/bucket, sigma=0.01 rad; result is a **null result** (no significant effect detected), which is itself a valid, reportable finding — not a failure to run the experiment.
