# Phase 3 — Recompute Derived Physics — Change Report

Repo: `ryugu_v2_ws/src/ryugu_sim` (git). Phase objective: recompute every
downstream-of-mass-distribution derived quantity against Phase 2's
**corrected** model (2.3127 kg, retracted-posture I_bot = 1.090813e-02
kg·m², corrected CG) — calculation only, no sim runs, no code/model
changes.

## 1. Files touched (full paths)

| Status | Full path |
|---|---|
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/compute_derived_physics.py` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/compute_derived_physics_stdout.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/compute_pivot_inertia.py` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/compute_pivot_inertia_stdout.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/PHASE3_CHECKPOINT_COMPARISON.md` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase3_derived_physics/PHASE3_CHANGE_REPORT.md` (this file) |

**No production code or model files touched** — confirmed via `git status`
before writing this report. `attitude_controller.py` still ships the OLD
`K_ang=0.05`/`K_rate=0.066` gains; this phase computes what they *should*
become but does not apply the change, per the phase's explicit
"calculation only, no sim runs yet" scope. Applying the new gains and
verifying them in sim is later-phase work.

## 2. What changed in each file

### `compute_derived_physics.py` (new)
Three-part calculation: (A) recomputes W, friction capacity, and the
illustrative launch thrust at the new mass, confirms both algebraically
and numerically that all three scale linearly with total mass, and
confirms escape velocity and the launch-velocity law have no mass
dependence at all (unchanged formulas, inspected directly from
`Research_Paper.md`, not assumed); (B) re-derives `K_ang`/`K_rate` using
the same target-bandwidth/target-damping design method the original
tuning comments describe, against the new retracted-posture I_bot; (C)
plugs the new gains back into the ζ/ωn formulas to confirm self-
consistency numerically rather than asserting it.

### `compute_pivot_inertia.py` (new)
Computes the moment of inertia about the tripod support-edge pivot axis
(self-righting/tipping dynamics) via the parallel-axis theorem, reusing
`../phase2_physical_model_rebuild/compute_whole_robot_cg_inertia.py`'s
frame-resolution code. Corrects an initial wrong assumption along the way
(see §5) about which leg posture represents the real landed/standing
stance. Runs against both the old and corrected-new model for direct
comparison, and cross-checks both against the paper's existing
$\tau\approx mgw/2$ figure (see §5 — surfaces a new, unrelated
discrepancy).

### `PHASE3_CHECKPOINT_COMPARISON.md` (new) — the checkpoint deliverable
Full old-vs-new comparison for every quantity in scope: mass-dependent
formulas, I_bot, I_pivot, K_ang/K_rate with the derivation shown step by
step (not just the resulting numbers), and the resulting ζ/ωn. See §4 for
the headline numbers.

## 3. What was run this phase

Pure calculation — no Gazebo/sim runs, consistent with the phase's stated
scope.

| Task | Script | Notes |
|---|---|---|
| Mass-only-dependent quantities + escape/launch-velocity mass-independence check | `compute_derived_physics.py` §A | Confirmed via the actual formulas in `Research_Paper.md`, not asserted |
| K_ang/K_rate re-derivation + self-consistency check | `compute_derived_physics.py` §B/§C | Includes a sanity check: old gains run against the old model's *real* I_bot reproduce the original design comment's claimed ζ≈1.1, wn≈1.8-2, validating the method before applying it to new numbers |
| Pivot-axis inertia (I_pivot), all 3 support-triangle edges, old and new models | `compute_pivot_inertia.py` | 6 edge computations total (3 edges × 2 models) |

## 4. Results

### Mass-only quantities (confirmed linear-in-m / mass-independent)
| Quantity | Old | New | Δ |
|---|---|---|---|
| W | 2.850e-04 N | 2.6365e-04 N | −7.49% |
| Friction capacity | 1.767e-04 N | 1.6346e-04 N | −7.49% |
| Illustrative thrust | 1.425e-02 N | 1.3182e-02 N | −7.49% |
| Escape velocity | 0.320 m/s | 0.320 m/s | 0% (no mass term) |
| Launch velocity law | (no mass term) | (no mass term) | 0% |

### I_bot (cited from Phase 2, retracted posture)
1.822e-02 → **1.0908e-02 kg·m²** (−40.1%)

### I_pivot (computed for the first time this phase — no prior figure existed)
1.1789e-01 → **1.0267e-01 kg·m²** (−12.9%), governing (least-stable) edge

### K_ang / K_rate (re-derived, not reused)
K_ang: 0.0500 → **0.0394** N·m/rad
K_rate: 0.0660 → **0.0456** N·m/(rad/s)

### Resulting ζ/ωn (confirmed by plugging new gains back in)
ωn = 1.9000 rad/s (target 1.9), ζ = 1.1000 (target 1.1) — exact match,
self-consistent.

## 5. Anything that didn't go as planned

- **No prior I_pivot calculation exists anywhere in this repo** — the
  phase instructions describe this as "the parallel-axis calc already
  done once for the old model," but a full-tree grep found only the
  paper's static $\tau\approx mgw/2$ torque figure, never a moment-of-
  inertia-about-the-edge calculation. Computed it for the first time this
  phase rather than silently inventing a fake "old" comparison figure —
  flagged explicitly in the checkpoint doc rather than glossed over.
- **New, unrelated discrepancy surfaced:** the paper's stated
  $\tau\approx 2.9\times10^{-5}$ N·m implies a support-edge half-width
  $w\approx0.204$m. The rigorously-computed $w$ from the actual leg
  kinematics (hip radius, thigh/calf lengths, mounting angles — none of
  which Phase 2 touched) is **0.346m (old model) / 0.345m (new model)** —
  not 0.204m, in *either* model. Since leg geometry was never changed by
  the mass redesign, this looks like a pre-existing error in the paper's
  own figure, independent of everything Phase 1-2 did. Flagged, not
  resolved — reconciling it is a documentation question for whoever
  rebuilds §3.3/Table I, not a "recompute the physics" one.
- **Wrong initial assumption caught and corrected before it propagated:**
  first attempt at I_pivot used "splayed (crouch stance)" (hip=0.33,
  knee=-2.60) as "the landed/standing posture," by loose analogy with
  `compute_moi.py`'s own posture label. A quick geometry check (foot
  z-coordinate) showed this puts the foot *above* `base_link`'s origin —
  not a physically possible ground-contact pose. The real standing
  posture is "retracted" (hip=0, knee=0): foot correctly below the
  chassis, and independently confirmed by `hopper_locomotion.py`'s own
  IDLE/landed leg command (`set_joints(0.0, 0.0)`). Caught by checking
  the numbers made physical sense before trusting them, not by assuming
  a label was self-explanatory — documented in both the script's own
  comments and the checkpoint doc so this mistake doesn't get made again
  by someone skimming posture names.
- **The original design's exact old inertia was never rigorously known**
  — `attitude_controller.py`'s comment gives a hand-added estimate
  ("~0.025 kg·m²... base 0.009 + legs ~0.012 + panel 0.0008 + wheels
  ~0.0006 + drill") for the first-pass gains, and the 2026-07-17 retune
  only states an outcome range ("wn ~1.8-2 rad/s"), not an explicit I.
  Resolved by back-solving the implied I range from that stated outcome
  (0.0125–0.0154 kg·m²) and cross-checking it against the old model's
  *real*, Phase-2-computed retracted-posture I_bot (1.822e-02) — close
  enough, and a sanity check using the old gains against that real I_bot
  reproduces the design comment's own claimed ζ≈1.1, confirming both the
  posture choice and the design method before trusting it for the new
  numbers.

## 6. Checkpoint verdict

**Checkpoint (from the phase instructions): "a short written comparison,
old inertia/gains/ζ/ωn vs. new, with the reasoning for the new gain
values shown, not just asserted."**

**PASS.** `PHASE3_CHECKPOINT_COMPARISON.md` contains the full old-vs-new
comparison for I_bot, I_pivot, K_ang, K_rate, ζ, and ωn, plus the
mass-only-dependent quantities the phase instructions asked to be
confirmed rather than assumed. The K_ang/K_rate derivation is shown as an
explicit step-by-step calculation (not just the resulting numbers),
including the design-method sanity check against the old model's real
inertia that validates the method before it's applied to the new one.
Nothing in this phase touched `attitude_controller.py`, `model.sdf`, or
ran a sim — consistent with the phase's explicit "calculation only, no sim
runs yet" scope; applying and verifying the new gains is left for a later
phase.

Two items surfaced this phase are explicitly **not** resolved and should
not be read as closed by this PASS: (1) no prior I_pivot figure existed to
diff against, so the "old" I_pivot in this report is itself a first-time
computation against the old model, not a pre-existing number — flagged,
not hidden; (2) the paper's existing $\tau\approx mgw/2$ figure appears to
rest on a support-edge width that doesn't match either model's real leg
geometry, a discrepancy independent of the mass redesign that this phase
surfaced but did not attempt to fix.
