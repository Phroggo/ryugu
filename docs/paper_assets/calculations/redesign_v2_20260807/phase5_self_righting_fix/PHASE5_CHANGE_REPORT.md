# Phase 5 — Resume the Self-Righting Architectural Fix — Change Report

Repo: `ryugu_v2_ws/src/ryugu_sim` (git). Phase objective: close the
architectural gap in residual-rotation ownership after a self-righting
give-up (LANDED-state damper is x/y-only; attitude_controller's z
authority isn't cleanly gated off while grounded either), using the
corrected Phase 3 torque/inertia figures, and confirm the fix with
targeted tests.

## 1. Files touched (full paths)

| Status | Full path |
|---|---|
| Modified | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/landing_controller.py` |
| Modified | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/attitude_controller.py` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_injection_test.py` (final, valid test harness) |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_results_BEFORE.json` / `z_disturbance_results_AFTER.json` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_BEFORE_stdout.log` / `z_disturbance_AFTER_stdout.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/gz_BEFORE_inject.log`, `gz_AFTER_inject.log`, `gz_BEFORE.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/bridge_scout_1_{BEFORE,AFTER}_trial{0,1,2}.log`, `landing_scout_1_{BEFORE,AFTER}_trial{0,1,2}.log`, `attitude_scout_1_BEFORE_trial{0,1}.log` (per-node console logs, all iterations — see §5 for which are the final/valid ones) |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/giveup_precession_test.py`, `giveup_precession_results_BEFORE.json`, `giveup_precession_BEFORE_stdout.log` (superseded first test design — kept for transparency, see §5) |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/PHASE5_CHECKPOINT_COMPARISON.md` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/PHASE5_CHANGE_REPORT.md` (this file) |

No files deleted. `model.sdf` not touched this phase (no mass/geometry
change).

## 2. What changed in each file

### `ryugu_sim/landing_controller.py` (+142/-41 net across edits)
- **New `rw_z` publisher**: `self.rw_pubs` now includes `'z'` (was `('x',
  'y')` only) — before this phase, this file was *structurally incapable*
  of ever commanding the z wheel, independent of any logic.
- **LANDED-state rate damper extended to all 3 axes**: reads
  `msg.angular_velocity.z`, includes it in the deadband magnitude check
  (renamed `omega_tilt` → `omega_total`), and adds `('z', wz)` to the
  per-axis damping loop — otherwise identical mechanism to the existing
  x/y damper (same tau cap, same acceleration-limited integration, same
  "don't force to zero below the deadband" fix already applied to x/y).
- **`LANDED_DAMP_K_RATE`**: 0.066 → 0.0456, matching
  `attitude_controller`'s Phase 3 re-derived `K_rate` (the constant was
  always defined as "matches attitude_controller"; kept that invariant
  rather than independently retuning it).
- **New OWNERSHIP INVARIANT comment** (at the damper's `__init__` site):
  states the full authority split (attitude_controller: ignition→first
  contact, all 3 axes; landing_controller: first contact→LANDED/RIGHTING,
  all 3 axes) and explicitly documents the one caveat this phase did
  *not* fully close — see §5.
- **Stale torque/inertia comment corrected**: the `RIGHTING_WHEEL_SPEED`
  sizing comment cited an unrigorous "~0.012 kg·m² roll axis... ~2.9e-5
  N·m... ~500x margin" — replaced with the real, rigorously-computed
  fold/tuck figures (I_pivot=4.275e-02 kg·m², w≈0.345m, τ≈4.5e-05 N·m,
  margin≈330x) from `../phase3_derived_physics/`. **No control-law values
  changed from this correction** — `RIGHTING_WHEEL_SPEED` (160) was
  already empirically tuned against live telemetry, not derived from
  this comment's numbers; the correction fixes what the comment *claims*
  those numbers are, nothing else.

### `ryugu_sim/attitude_controller.py` (+15/-0)
One comment block added at the "Yaw hold (always active, including
grounded...)" line: documents the same ownership invariant from this
file's side, explicitly noting that x/y are cleanly exclusive
(gated to `in_flight`) while z is not (deliberately — needed for
hopper_locomotion's CROUCH-phase yaw alignment), and that this creates a
known, flagged, not-fully-verified last-write-wins possibility with
landing_controller's new z-damper during LANDED. No logic changed.

## 3. What was run (targeted tests, with commands and counts)

This phase's verification went through several design iterations before
reaching a valid test — reported in full per the "didn't go as planned"
requirement, not just the final version:

| Attempt | Script | Trials | Outcome |
|---|---|---|---|
| 1. Spawn-based give-up repro (severe tilt, 165–172°, 200s window) | `giveup_precession_test.py` | 3 | Superseded: robot mostly never reached `landed=True` at all within a practical window (matches this project's own prior finding that severe-tilt trials rarely trigger a clean righting sequence); one trial did show a real, large negative u_z trend (-0.328) but without a clean give-up event to anchor it to |
| 2. Z-disturbance injection, 250 rad/s, both nodes omitted attitude_controller | `z_disturbance_injection_test.py` (early config) | 2 | Superseded: injection strong enough to trigger a full liftoff kick (matches the documented "LANDED→liftoff" issue), leaving LANDED state entirely — confounded, not a clean test of the LANDED-state mechanism |
| 3. Z-disturbance injection, 20 rad/s | same | 2 | Superseded: disturbance small enough that passive ground-contact friction alone fully absorbed it within 90s, in both before and after states — non-discriminating |
| 4. Z-disturbance injection, 80 rad/s, **with** attitude_controller also launched | same | 2 | Superseded: adding attitude_controller prevented the robot from ever leaving `IDLE` in either trial (suspected: its sleep-defeat idle rotor keeps velocity/accel just above the very tight 0.005 m/s rest-detection threshold) — never reached `state==LANDED` at all |
| **5. Z-disturbance injection, 40 rad/s, landing_controller alone (FINAL)** | same | 2 BEFORE + 2 AFTER | **Valid** — see §4 |

Final valid run commands (via `bash <script>.sh BEFORE` / `AFTER`,
sourcing ROS + workspace + `GZ_SIM_RESOURCE_PATH`, headless `gz sim`,
then `python3 z_disturbance_injection_test.py {BEFORE,AFTER}`): 2 trials
each side, ~3.5 min per trial (up to 200s settle-wait + 1s inject + 90s
observe).

## 4. Results (actual behavior before and after the fix)

See `PHASE5_CHECKPOINT_COMPARISON.md` for the full table. Headline:

| | BEFORE | AFTER |
|---|---|---|
| Trial 1: wz start → end | -1.383 → **-2.553** (grew) | -1.187 → **0.0** |
| Trial 1: final u_z | **-0.646** | **0.9993** |
| Trial 2: wz start → end | -1.383 → -1.338 (~static) | -1.413 → **0.0** |
| Trial 2: final u_z | 0.458 | **0.9993** |

Same spawn, same settle-wait logic, same injection magnitude (within
trial-to-trial physics noise), only the code differs. Without the fix,
injected residual z-rotation persists or grows and the body ends up away
from upright (trial 1 reproduces the qualitative failure signature —
unarrested rotation degrading orientation — that motivated this phase).
With the fix, the same disturbance fully decays and the body ends at
essentially perfect upright, both trials.

## 5. Anything that didn't go as planned

This phase's verification step took five design iterations to get right
(§3) — reported in full because the dead ends are informative, not just
the working result:

- **Severe-tilt organic give-up reproduction proved impractical.** The
  original ask ("a handful of runs that previously drifted into deeper
  inversion, rerun against the fix") assumed reaching a genuine give-up
  is fast/reliable. It isn't: most severe-tilt spawns never reach
  `landed=True` at all within a 200s window (stuck oscillating in
  FLIGHT), matching this project's own prior finding
  (`severe_tilt_no_respawn_rerun_20260805`) that even the proven
  slerp-teleport method only triggers a righting attempt ~25% of the
  time. Pivoted to direct disturbance injection instead — verifies the
  same mechanism (does LANDED-state residual z rotation get damped) far
  faster and more reliably than waiting for an organic give-up.
- **Two harness bugs, same recurring classes as earlier phases**: missing
  `GZ_SIM_RESOURCE_PATH` in a bash-launched (not python-launched) `gz
  sim`, and a missing IMU bridge entry — both caused a
  real-looking-but-wrong result (state stuck IDLE/FLIGHT, no real
  physics) rather than an obvious crash. Caught by checking the actual
  node console logs, not just the harness's own summary numbers, before
  trusting any result — same lesson flagged in Phase 4's report,
  recurring again.
- **First "landed" wait timeout (60s) was too short by construction**:
  the settle-confirmation window itself (`REST_Z_TICKS`≈60s or
  `REST_VEL_TICKS`≈120s) is longer than that alone, before accounting for
  fall+contact time. Fixed by using 200s, matching this project's own
  C15/C16 `LANDED_WAIT_TIMEOUT`, already validated for this exact wait.
- **Injection strength required real tuning to find a discriminating
  value**: 250 rad/s triggered a full liftoff (confounds the test with a
  different control regime entirely); 20 rad/s was fully absorbed by
  passive ground friction regardless of the fix (non-discriminating).
  40 rad/s was the value that finally produced a real, measurable,
  discriminating difference — found by iterating, not derived in advance.
- **Launching `attitude_controller` alongside `landing_controller`
  (attempting a more architecturally complete test) broke settle
  detection entirely** — the robot never left `IDLE`. This means **the
  final valid test verifies the LANDED-state z-damper mechanism in
  isolation, not the full multi-node interaction with
  attitude_controller's own grounded z authority** — the specific
  last-write-wins risk flagged in both files' new ownership-invariant
  comments remains **unverified by dynamic test**, resting only on the
  code-level argument in those comments (torque-budget dominance,
  sign-consistency in the passive case). This is a real, explicitly
  flagged limitation of this phase's verification, not silently assumed
  covered — a full regression test under active crouch-phase alignment
  is still open work for a later phase.
- **A stray zombie process from an earlier abandoned test-design
  attempt** (a briefly-launched `attitude_scout_1` node) kept writing to
  a *previously committed Phase 4 log file* via a stale open file handle
  after being orphaned, appending ~34,000 lines of unrelated console
  output to `phase4_attitude_revalidation/attitude_4ms.log`. Caught via
  `git status` before committing anything (not something a diff review
  would have caught after the fact), confirmed no scout_1-related
  process was still alive, and reverted the file to its committed state
  with `git checkout --`. Not a Phase 5 result — flagged so the
  mechanism (long-lived orphaned nodes across many background sim runs
  in one session can silently corrupt unrelated committed files if their
  stdout was ever redirected there) is on record.
- **Result-file naming**: `z_disturbance_results_{label}.json` gets
  overwritten by each rerun under the same label — several intermediate
  (superseded) attempts' data were lost this way before landing on the
  final config. Not a correctness problem (the final files are valid),
  but worth a naming-convention fix (a run index or timestamp suffix) if
  this kind of iterate-until-it-works testing recurs in a later phase.

## 6. Checkpoint verdict

**Checkpoint: "No unassigned-rotation-ownership gap left. The targeted
tests show the fix prevents the previously observed slow precession into
deeper inversion. Does not require the full n≥20 batch yet."**

**PASS**, with one explicit scope limitation carried forward rather than
hidden:
- The specific architectural gap named in the phase instructions (no
  z-wheel publisher in `landing_controller.py`, hence structurally
  incapable of damping residual z rotation while LANDED) is closed — both
  by direct code inspection (the publisher now exists, the damper loop
  now includes z) and by a clean, reproducible, same-conditions
  before/after dynamic test (§4) showing the intended behavior change.
- Ownership is now explicitly documented in both files (the required
  invariant statement), including the one piece that is **not** fully
  exclusive: z-authority during LANDED is still nominally shared with
  attitude_controller's grounded yaw-hold, a known and flagged (not
  silently accepted) last-write-wins possibility that this phase's final
  test methodology could not exercise (§5) because running both nodes
  together broke settle detection entirely in this test's specific
  configuration.
- Per the phase's own explicit allowance, no n≥20 batch was run — 2+2
  targeted trials with a clean, controlled before/after comparison were
  used instead, consistent with "just confirmation the architectural hole
  is closed in a targeted test."

**Recommended before Phase 6 relies on this being fully closed**: a
follow-up regression check of the multi-node (landing_controller +
attitude_controller together) case, ideally using a less disruptive way
to keep the model awake during a long settle-wait than what this
session's attempt hit — not required to close out Phase 5 itself, but
worth not forgetting.
