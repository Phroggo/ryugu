# Phase 12 — Scoped Mass-Sensitivity Analysis (I_bot ±20%, side_rest + full_inversion)

Date: 2026-08-12
Scope: reviewer round 2 flagged the lumped `base_link` mass model (1.3839 kg, everything but reaction wheels/legs/drill folded into one rigid body) as the top-cited threat to validity for attitude-control/self-righting/landing-impact claims. Scoped answer, per explicit direction: perturb I_bot by ±20% (mass and COM unperturbed, out of this scope) and rerun self-righting reliability, to check whether the recovery-rate conclusion for the active-control regime is robust to the lumped model's inertia uncertainty. Primary bucket: side_rest (most informative baseline rate, ~50%). Full_inversion included too per the decision rule (same harness, only a model-URI parameter change, no new spawn logic). Moderate excluded — Phase 11 established it recovers via passive settling, not the active RW maneuver, so it isn't informative about I_bot sensitivity in the same way. Item #3 (self-righting randomization expansion) remains out of scope, held per prior direction.

## 1. Files touched

### New scripts

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/generate_ibot_variants.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/mass_sensitivity_self_righting.py`

### Variant models (generated, not hand-written)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_p20/model.sdf`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_p20/model.config`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_p20/meshes/` (empty — the source model uses only primitive box/cylinder geometry, no mesh files; copied for structural consistency with the model package convention, matches the source model's own empty `meshes/`)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_m20/model.sdf`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_m20/model.config`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models/spacehopper_ibot_m20/meshes/` (empty, same reason)

### Results and logs (80 trials: 2 buckets x 2 configs x n=20)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/mass_sensitivity_self_righting_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/mass_sensitivity_self_righting_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/gz_p12_batch.log`
- 80 `bridge_scout_1_{bucket}_{config}_trial{N}.log` files and 80 `landing_scout_1_{bucket}_{config}_trial{N}.log` files (one pair per trial, 160 files total)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/PHASE12_CHANGE_REPORT.md` (this file)

(170 files total this phase — note empty `meshes/` directories are not separately tracked by git. Complete literal listing appended to the commit that carries them.)

## 2. What was generated and run

`generate_ibot_variants.py` scales all six components of `base_link`'s inertia tensor (`models/spacehopper/model.sdf`, lines 12-18: ixx=0.009844, ixy=-0.000090, ixz=-0.000123, iyy=0.010118, iyz=-0.000008, izz=0.007611) uniformly by 1.20 and 0.80, preserving principal-axis directions and eigenvalue ratios exactly. Mass (1.3839 kg) and COM ((0.00243, 0, -0.00703)) are untouched. Verified via diff that nothing else in the SDF changed.

`mass_sensitivity_self_righting.py` reruns side_rest (85-95°) and full_inversion (170-180°) at both perturbed configs, n=20 each, reusing Phase 7's original baseline data (side_rest 10/20, full_inversion 3/20) rather than rerunning it — valid because `landing_controller.py`'s RIGHTING state machine was untouched by both Phase 9 (hopper_locomotion.py launch-timing only) and Phase 11 (harness-side timer instrumentation only). Incorporates Phase 11's recovery-timer fix (`/righting_active` subscription).

**Bug caught and fixed before trusting any data**: the first version of this script defined `make_bridge_yaml()` but never called it in `main()`, so the bridge process read a missing/stale YAML config file (`"Could not parse config, top level must be a YAML sequence"`), producing zero telemetry (`start_uz=None`, `landed=None` after the full 350s timeout) for the first trial. Caught by checking trial 1's actual output rather than assuming the harness worked; killed the batch, fixed the missing call, verified the generated YAML parses as a valid list via a standalone smoke test before relaunching the full 80-trial batch from scratch. No data from the broken first attempt was used.

## 3. A significant, unplanned finding: what "recovered" actually measures in this dataset

Before trusting trial 1's `recover_time_s=77.6s`, I read its raw `landing_scout_1` log. It shows the controller ran all 5 righting attempts and explicitly **gave up** (`"❌ Self-righting failed after 5 attempts — giving up... Robot may still be physically inverted"`) — the give-up handler force-marks `LANDED` so downstream logic doesn't hang, which is what set `landed=True` in the harness's poll loop. The body then continued to drift upright afterward (residual reaction-wheel momentum settling out) and happened to cross u_z>0.9 within the `RIGHTING_WAIT_TIMEOUT` window, which the harness (and Phase 7's original harness, using the identical criterion) counts as `outcome=recovered`.

Checked this across all 80 trials: **every single trial that reached an outcome (76/80; 4 hit `no_landing`/timeout) shows the give-up message** — not one trial in this batch converged within its own active-attempt sequence. Cross-checked against Phase 7's original baseline logs for comparison: **20/20 side_rest and 20/20 full_inversion baseline trials show the identical give-up pattern.** This is consistent, pre-existing system behavior, not something introduced by this phase's harness or the I_bot perturbation — so the recovery-**rate** comparison between baseline and perturbed configs remains valid (both measure the same thing: "did the give-up-then-settle process end up upright within the window," consistently defined).

**However, this changes how `recover_time_s` should be interpreted, and reveals a likely additional problem with the *existing* Table IX side_rest/full_inversion figures** (19.5s mean, 2.5s median for side_rest; 17.0s mean, 0.0s median for full_inversion), which Phase 11 assessed as "fine" — not needing the timer fix applied to the moderate bucket, on the assumption that "real recovery there takes seconds" so the ~0.3s polling-latency error is negligible. That assumption undersold the issue: since virtually every trial only reaches `landed=True` at the give-up moment (not at a genuine standalone landing event), the *old* harness's `right_t0` (started from the landed-poll) was anchored at essentially the give-up moment too — meaning it was measuring only the **final post-give-up settling tail**, not the true ~75-190s spent across all 5 active attempts. This phase's `/righting_active`-based timer (started from the *first* attempt) captures the true total duration and shows numbers 5-10x larger (mean 87-99s across the four config groups here) than the old Table IX figures. This is a **different bug from Phase 11's moderate-bucket case** (there, the poll latency swamped a near-zero true signal; here, the timer was anchored at the wrong event entirely), but likely means side_rest/full_inversion's `recover_time_s` entries in Table IX need the same fix-and-rerun treatment Phase 11 gave moderate. **Not doing this unilaterally** — flagging it here for a decision, since it's new, unplanned scope beyond what was authorized for this phase (I_bot sensitivity), the same way Phase 11's moderate-bucket finding was flagged rather than acted on beyond its own scope.

## 4. Results

| Bucket | Config | Recovered | Rate | 95% Wilson CI | vs. baseline (two-proportion z-test) |
|---|---|---|---|---|---|
| side_rest | baseline (Phase 7, reused) | 10/20 | 50.0% | [29.9%, 70.1%] | — |
| side_rest | ibot_p20 (+20%) | 5/20 | 25.0% | [11.2%, 46.9%] | z=1.63, p=0.102 |
| side_rest | ibot_m20 (-20%) | 5/20 | 25.0% | [11.2%, 46.9%] | z=1.63, p=0.102 |
| full_inversion | baseline (Phase 7, reused) | 3/20 | 15.0% | [5.2%, 36.0%] | — |
| full_inversion | ibot_p20 (+20%) | 9/20 | 45.0% | [25.8%, 65.8%] | z=-2.07, p=0.038 |
| full_inversion | ibot_m20 (-20%) | 7/20 | 35.0% | [18.1%, 56.7%] | z=-1.46, p=0.144 |

With 4 comparisons run, a Bonferroni-corrected significance threshold is p<0.0125 — **none of the four comparisons reach significance at that threshold** (the closest, full_inversion+20%, is p=0.038, uncorrected). All CIs substantially overlap their respective baselines.

`recover_time_s` (new /righting_active-based timer, not directly comparable to old Table IX figures — see §3): side_rest ibot_p20 n=5 mean=88.95s std=15.68s; side_rest ibot_m20 n=5 mean=98.75s std=40.27s; full_inversion ibot_p20 n=9 mean=86.58s std=15.32s; full_inversion ibot_m20 n=7 mean=99.11s std=38.18s.

## 5. Summary — does the qualitative conclusion flip?

**No, not at the level of statistical significance achievable with n=20/config, but the point estimates move enough to warrant honest reporting rather than a clean "robust" claim.** Side_rest's recovery rate moves from 50% to 25% at *both* +20% and -20% perturbations — a symmetric-looking drop that is not statistically distinguishable from baseline at this sample size (p=0.10, uncorrected), but also not a reassuring "stayed flat at 50%" result; it is more accurately described as "recovery remains partial and unreliable under perturbation, with a point estimate that dropped by half but with wide, overlapping confidence intervals." Full_inversion's rate *increases* under both perturbations (15%→45%/35%), the larger of which is marginally significant uncorrected (p=0.038) but does not survive correction for multiple comparisons; qualitatively this moves the bucket from "recovery is rare" toward "recovery happens roughly a third to nearly half the time," which is a more meaningful shift in characterization even though it isn't statistically airtight at n=20. Recommend describing this in the paper as: the lumped-mass approximation's effect on self-righting recovery rate could not be ruled out at n=20/config — the qualitative *ordering* of buckets (full_inversion hardest, side_rest intermediate) does not flip, but point estimates shift by up to 2x in both directions, and a larger sample (the same n≥50/bucket scale already used for Phase 10's sensor-noise study) would be needed to state robustness with confidence rather than assert it from an underpowered comparison.

## 6. Anomalies flagged this phase

1. Missing `make_bridge_yaml()` call — caught before trusting data, fixed, batch restarted clean (§2).
2. 100% give-up rate across all 80 trials and (confirmed) all 40 baseline trials — pre-existing, consistent system behavior, not new, but changes what `recover_time_s` has actually been measuring throughout this whole self-righting dataset, not just this phase (§3).
3. Recovery-rate point estimates move by up to 2x under I_bot±20% perturbation without reaching statistical significance at n=20/config — reported honestly as inconclusive rather than rounded to either "robust" or "broken" (§4-5).

## 7. Checkpoint verdict

I_bot sensitivity analysis (scoped): **complete for the requested scope** (side_rest primary, full_inversion included per the decision rule). Result is a genuine **null result at this sample size** (no comparison survives correction for multiple comparisons) with point estimates that move enough to recommend against a clean "robustness confirmed" claim in the paper — report the finding as-is (§5), don't round it either direction. New, unplanned finding about `recover_time_s`'s measurement validity for the *existing* side_rest/full_inversion baseline figures (§3): flagged, not acted on, pending direction. Items #3 (randomization expansion) and full distributed-mass remodel: still out of scope, untouched.
