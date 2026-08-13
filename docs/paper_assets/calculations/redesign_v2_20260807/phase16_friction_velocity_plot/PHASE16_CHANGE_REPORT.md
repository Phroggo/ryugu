# Phase 16 — Friction-vs-Delivered-Velocity Plot

Date: 2026-08-13
Scope: reviewer backlog item — "denser friction sweep with a velocity-vs-μ plot... to visually confirm the plateau rather than assert it." Checked before treating as new work: Phase 10's `friction_sweep_postfix_results.json` (post-timing-fix, n=20/μ) already has per-trial `delivered` (m/s) and `ratio` fields — this is purely a plotting task against existing data, no new sim runs. Confirms the suspicion in the original backlog note.

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/generate_friction_velocity_plot.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/friction_velocity_plot.png`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/friction_velocity_plot.svg`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/PHASE16_CHANGE_REPORT.md` (this file)

## 2. What was done

Two-panel plot, both x=μ (0.40-0.90): left is raw delivered separation velocity (individual trial scatter + mean±std), right is the normalized delivery ratio (delivered/v_req). Deliberately two panels rather than one with a v_req reference line — v_req (0.0428 m/s) is ~4.6x the delivered plateau (~0.0093 m/s), so drawing both on one axis would squash the entire plateau into an unreadable sliver near the bottom. The ratio panel is the direct plotted version of what the table already reports numerically.

First draft (not kept) used a single panel with the v_req line included — caught the readability problem by actually looking at the rendered image before finalizing, not just generating and moving on.

## 3. Result

Both panels confirm what the table already states: flat across the full friction range. Left panel means bounce narrowly between 0.00933-0.00935 m/s with all std bars overlapping; right panel's ratio is visually indistinguishable from a flat line at 0.218 across μ=0.40-0.90. No new finding — this was a visualization task, and the visualization confirms the existing numeric result rather than revealing anything new.

## 4. Checkpoint verdict

**Complete.** No sim time required, matching the pre-check. Ready to drop into the paper as a figure.
