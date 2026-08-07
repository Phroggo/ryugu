# Phase 0 baseline manifest — mass/geometry redesign, 2026-08-07

This is the frozen reference point every later redesign phase gets diffed
against. If a number in a later phase's change report doesn't match
something here, that's the comparison that matters.

## Naming convention for this effort (applies from Phase 0 onward)

All artifacts from the mass/geometry redesign live under
`docs/paper_assets/calculations/redesign_v2_20260807/`, one subfolder per
phase (`phase0_baseline_lockin/`, `phase1_.../`, ...). Nothing produced
from this point forward reuses a pre-existing filename or folder from
before the redesign started (e.g. the old scratch folders
`timestep_check_contact/`, `timestep_check_righting/` were never part of
the git repo and are superseded by the copies checked in here). Every
phase's report must state explicitly which files are new vs. reused.

## 1. Frozen code state

- **Tag:** `pre-mass-redesign`
- **Commit:** `4cc7de98b2b7c13028694e53c3a446ca1489b586`
  ("Snapshot pre-redesign controller state (baseline for mass redesign)")
- **model.sdf:** unchanged since commit `63f73b8` — no pending edits existed
  at snapshot time, so no new commit was needed for it.
- **Controller files snapshotted as-is:** `ryugu_sim/attitude_controller.py`,
  `ryugu_sim/hopper_locomotion.py`, `ryugu_sim/landing_controller.py`. Full
  diff of what was carried into this snapshot is in the commit message and
  `git show 4cc7de9`.

### Current gain values (attitude_controller.py:178-179)

```
K_ang  = 0.05   N·m/rad      (attitude stiffness)
K_rate = 0.066  N·m/(rad/s)  (rate damping)
```

### Known-open issue carried into this baseline, unresolved

Diagnosed (not fixed) during the pre-snapshot debugging session: near-
upright give-ups in self-righting are not a time/attempt-budget shortfall
(ruled out directly — an extended-budget run gave up even later and drifted
into a third failure mode, slow non-decaying precession deeper into
inversion). Leading diagnosis: real 3-axis rigid-body coupling that the
LANDED-state damper can't arrest, since it's x/y-only by design (no z-wheel
in `landing_controller.py`) while `attitude_controller` — the only node
that owns the z-wheel — stands down entirely once `landed=True`. This is
an architectural gap, not a tuning value, and is **not resolved** in this
baseline.

Also note: `GENTLE_RIGHTING_SPEED` in `landing_controller.py` is currently
at a temporary-diagnostic value (20.0, raised from a prior 8.0 on
2026-08-05 to test a stall hypothesis) that was never reverted. It is part
of this frozen baseline exactly as it stands — flagging so it isn't
mistaken for an intentional tuned value by a later phase.

## 2. Reference validation dataset

### a. Yaw-slew spot check (C13, pure RW torque, no ground contact)
`../attitude_rerun_20260803/README.md`, `c13_yaw_slew_raw_telemetry.jsonl`
(pre-existing, already committed, unchanged by this phase).

**Result: 107° commanded yaw slew converges to 106.03°, <1° of target by
t+9.3s, overdamped, holding ~0.97° steady-state error for the rest of a
20s window.**

### b. Timestep comparisons already run

| Scenario | 1ms result | 4ms result | Location |
|---|---|---|---|
| C13 yaw-slew (pure torque) | 106.06°, 9.61s to converge | 106.15°, 8.70s | `../timestep_sensitivity_20260805/` (pre-existing) |
| 9.0m degraded launch, single pair | ratio=0.286, 18.0s | ratio=0.292, 18.1s | `contact_launch_timestep_check/` (new this phase, was scratchpad-only) |
| 9.0m degraded launch, 5-repeat distribution | n=4, ratios 0.009–0.301, mean 0.133 | n=3, ratios 0.205–0.447, mean 0.325 | `contact_launch_timestep_check/` (new this phase) |
| Self-righting, 60° tilt, single pair | u_z>0.9 at 111.8s, final 0.99942 | u_z>0.9 at 106.4s, final 0.99941 | `self_righting_timestep_check/` (new this phase, was scratchpad-only) |

**Headline finding carried into the redesign:** the contact-dynamics
launch case shows >30x run-to-run variance at *fixed* timestep (see
`contact_launch_timestep_check/README.md`), which dwarfs any timestep
effect measured anywhere in this table. Any redesign phase that reports a
single-run launch-delivery number should be treated as provisional until
repeated.

### c. Launch-delivery 10-run distribution

Same as row 3 above — the 5-repeat × 2-timestep contact-launch batch is
simultaneously "the timestep comparison" and "the launch-delivery
distribution" referenced by this baseline; there is no separate dataset.
10 runs total, 7 stabilized, 3 timed out without separating (30%).
Original single-sample vgain calibration figure for this same 9.0m case
(`../vgain_calibration_20260803/README.md`) was ratio=0.209 — inside this
distribution's range but not representative of its mean or spread.

### d. Self-righting data

| Dataset | Result | Location |
|---|---|---|
| Pre-redesign controller (commit `5c9e278`), 21 trials, random tilt | **1/21 (4.8%)** recovered | `../pre_redesign_self_righting_baseline_20260804/` (pre-existing) |
| Post-redesign controller (this baseline), 21 trials, same methodology | **1/21 (4.8%)** recovered — identical to pre-redesign | `../post_redesign_self_righting_baseline_20260805/` (pre-existing) |
| Severe-tilt (>120°) subset, no-respawn re-verification, 8 trials | 2/8 triggered a righting attempt at all; 0/8 reached a stable landed state within 200s | `../severe_tilt_no_respawn_rerun_20260805/` (pre-existing) |
| Give-up → uncommanded-liftoff failure cascade | Documented, 2 independent real occurrences | `../self_righting_failure_mode_20260805/` (pre-existing) |
| 60° tilt, single timestep pair (this phase) | See table above | `self_righting_timestep_check/` (new this phase) |

All pre-existing entries are unchanged by this phase — listed here only so
this manifest is a complete, single-document reference set as required by
the Phase 0 checkpoint. None of these files were touched, moved, or
renamed.

## 3. What this baseline does NOT establish

- No statistically meaningful post-redesign self-righting reliability
  number exists (blocked on the open 3-axis coupling issue above — see
  round-2 sim-chat answers, §7).
- No reliable single-value launch-delivery ratio exists for any commanded
  distance — run-to-run variance at fixed everything is too large.
- The 4.3m/-56° headline directional-hop figure remains contradicted by
  its own re-verification (C9: achieved ground-track azimuth 122.66° vs.
  held yaw -55°) and is not part of this reference set as a trustworthy
  number — it's flagged here only so a later phase doesn't accidentally
  treat it as validated baseline behavior.
