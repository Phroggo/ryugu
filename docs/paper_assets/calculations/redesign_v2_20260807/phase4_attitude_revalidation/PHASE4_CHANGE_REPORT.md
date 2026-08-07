# Phase 4 — Isolated Re-validation of Attitude Control — Change Report

Repo: `ryugu_v2_ws/src/ryugu_sim` (git). Phase objective: confirm the new
physical model (Phase 2) plus re-tuned gains (Phase 3) reproduce
equivalent closed-loop behavior to the old validated results, as a
single-variable test — model and gains changed, control code untouched.

## 1. Files touched (full paths)

| Status | Full path |
|---|---|
| Modified | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/attitude_controller.py` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/yaw_slew_revalidation.py` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/ryugu_4ms.sdf` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/phase4_yaw_slew_stdout.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/phase4_yaw_slew_results.json` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/gz_1ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/gz_4ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/bridge_1ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/bridge_4ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/attitude_1ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/attitude_4ms.log` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/PHASE4_CHECKPOINT_COMPARISON.md` |
| Added | `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase4_attitude_revalidation/PHASE4_CHANGE_REPORT.md` (this file) |

`model.sdf` was **not** re-touched this phase (already correct from the
Phase 2 correction pass). No other production files modified.

## 2. What changed in each file

### `ryugu_sim/attitude_controller.py`
Exactly the two gain values, plus comments: `K_ang` 0.05→**0.0394**,
`K_rate` 0.066→**0.0456** (Phase 3's re-derived values, applied verbatim).
A new comment block documents the provenance (Phase 2's corrected I_bot,
Phase 3's re-derivation, a pointer to the full derivation doc) directly
above the changed lines. **`I_wheel` (line below, `0.00027`) was
deliberately NOT updated**, even though it's also stale against Phase
1/2's real RW annulus inertia (3.944e-4 kg·m²) — left untouched and
flagged in-place so Phase 4 stays a true single-variable test (gains
only), not a second simultaneous change; noted as a follow-up for a later
phase. 22 insertions, 2 deletions (`git diff --stat`), all within this
one constant-definition block.

### `yaw_slew_revalidation.py` (new)
Adapted directly from the existing, already-validated
`../../timestep_sensitivity_20260805/timestep_sensitivity.py` (same
method, same 107° target, same <1°-error convergence criterion, same
attitude-controller-only setup) — only the output directory changed, so
this is a true apples-to-apples rerun of the original test, not a new
test design. Runs both the 1ms and 4ms timestep cases in one script,
matching the original.

### `ryugu_4ms.sdf` (new, copied)
The same verified 4ms-timestep world variant already used in Phase 0
(`phase0_baseline_lockin/contact_launch_timestep_check/ryugu_4ms.sdf`),
copied here for self-containment — confirmed via `diff` against the
current `worlds/ryugu.sdf` to still be a clean single-line
(`max_step_size`) difference, not stale.

### `PHASE4_CHECKPOINT_COMPARISON.md` (new) — the checkpoint deliverable
Full old-vs-new comparison: final angle, convergence time, overshoot
check (from the raw trace, not just the summary numbers), steady-state
behavior, and a repeat of the timestep-sensitivity comparison. See §4.

## 3. What was run this phase

| Task | Script | Notes |
|---|---|---|
| Yaw-slew spot check + 1ms/4ms timestep comparison | `yaw_slew_revalidation.py` | 2 sim runs (1ms, 4ms), against the corrected `model.sdf` and the new `attitude_controller.py` gains |
| Verified editable-install picked up the gain change without a rebuild | `diff` against `build/ryugu_sim/ryugu_sim/attitude_controller.py` | Confirmed hardlink/same-file, no `colcon build` needed |

## 4. Results

| | Old | New |
|---|---|---|
| 1ms final yaw | 106.03-106.06° | **106.078°** |
| 1ms convergence time | 9.3-9.61s | **8.48s** |
| 4ms final yaw | 106.15° | **106.108°** |
| 4ms convergence time | 8.70s | **8.24s** |
| Overshoot | none | **none** (checked full trace, max 106.11°/106.16°, both < 107° target) |
| Timestep spread (final angle) | 0.09° | **0.03°** (tighter, not worse) |
| Timestep spread (convergence time) | 0.91s | **0.24s** (tighter, not worse) |

Full detail: `PHASE4_CHECKPOINT_COMPARISON.md`.

## 5. Anything that didn't go as planned

- **First run attempt produced a real-looking but completely wrong
  result**: yaw stayed at exactly 0° (2.25e-16, floating-point-zero) for
  the entire run in both timestep cases — not "no oscillation," a total
  non-response. Root cause, found in `attitude_1ms.log`: `Package
  'ryugu_sim' not found`. The harness script sourced
  `/opt/ros/humble/setup.bash` but not
  `/home/melvin/ryugu_v2_ws/install/setup.bash`, so `ros2 run ryugu_sim
  attitude_controller` failed to find the local package and the node
  never started — meaning nothing was ever commanding the wheels, and the
  "0°, never moved" result was actually correct given that. Caught by
  checking the final-yaw number against zero suspiciously exactly, then
  reading the node's own console log rather than trusting the summary
  print. Fixed by sourcing the workspace's own `install/setup.bash` in
  addition to the ROS distro setup; rerun produced the real result in §4.
  This is the same family of "silent tooling failure produces a real-
  looking but wrong number" issue as the recurring `pkill`-exit-1 shell
  abort noted in earlier phases — different specific cause, same lesson:
  check node-level logs, not just the harness's own summary, whenever a
  result looks suspiciously clean (exactly zero, in this case).
- Otherwise, no surprises — the corrected run's numbers landed close to
  the old results and in the expected direction (faster convergence,
  consistent with the deliberately higher target ωn), which is what a
  correctly-executed re-derivation should look like.

## 6. Checkpoint verdict

**Checkpoint (from the phase instructions): "the re-tuned system
converges to a comparable angle in a comparable time, with no
oscillation, under the new model. If this doesn't hold, the problem is in
Phase 2's model or Phase 3's gain re-derivation, stop and fix it here."**

**PASS.**
- Comparable angle: 106.08-106.11° new vs. 106.03-106.15° old — within
  0.03-0.08°, same side of the target (slight undershoot, matching the
  original's own described ~1° steady-state deadband).
- Comparable time: 8.2-8.5s new vs. 8.7-9.6s old — within ~1s, and the
  new numbers being modestly faster is the *expected* signature of Phase
  3's deliberately higher target bandwidth (ωn 1.9 vs. the old gains'
  actual ~1.66 rad/s), not an unexplained deviation.
- No oscillation: confirmed by inspecting the full trace, not just the
  final numbers — zero overshoot past the 107° target in either timestep
  case, tight non-growing steady-state jitter.
- Timestep sensitivity re-checked as instructed: unchanged in character,
  slightly tighter than before.

The one real problem this phase hit (§5) was a harness bug (missing
workspace source), not a Phase 2/Phase 3 foundation problem — diagnosed
and fixed before drawing any conclusion, not papered over. Phase 5
(self-righting redesign) can proceed on this foundation.
