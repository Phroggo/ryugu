# Phase 14 — Righting Hold-Confirm Diagnostic: A Real Bug Found, Not Just Torque/Geometry

Date: 2026-08-13
Scope: torque-vs-geometry investigation into full_inversion's failure mode (100% give-up rate per Phase 12/13). Added purely-observational instrumentation to `landing_controller.py`'s hold-confirm logic to distinguish "the roll never got close to upright" from "it got close but couldn't complete the hold" — and the very first trial run under that instrumentation exposed a real, previously-unknown bug in `_finalize_righting_timeout()`, not a torque or geometry limitation.

**This report also documents a mistake made mid-investigation** (§4) — the diagnostic batch was cut short after editing the live controller file while it was still running.

## 1. Files touched

### Source code (modified)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/landing_controller.py` — see §2 (diagnostics) and §3 (bug fix)

### New script

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/righting_hold_diagnostic.py`

### Results and logs (2 clean trials + 1 aborted, see §4)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/righting_hold_diagnostic_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/righting_hold_diagnostic_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/gz_p14_batch.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/bridge_scout_1_full_inversion_trial1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/bridge_scout_1_full_inversion_trial2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/bridge_scout_1_full_inversion_trial3.log` (aborted, no righting data — see §4)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/landing_scout_1_full_inversion_trial1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/landing_scout_1_full_inversion_trial2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/landing_scout_1_full_inversion_trial3.log` (aborted)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase14_righting_hold_diagnostic/PHASE14_CHANGE_REPORT.md` (this file)

## 2. Diagnostic instrumentation (purely observational, no control-law change)

`_run_righting_sequence`'s existing per-tick `u_z<0.9` gate already switches into a HOLD-CONFIRM branch the instant u_z crosses 0.9 (must hold ~2s at low angular rate to declare success) — but nothing distinguished "never got close" from "got close, entered hold, lost it" in the logs. Added two log lines, verified via `git diff` to be the only change at first (before the bug fix in §3 was added on top): `HOLD-START` (first tick of a fresh hold attempt) and `HOLD-LOST` (a hold that had genuinely started got released before completing). Existing `"✅ Self-righting successful"` marks a completed hold.

## 3. The bug found — `_finalize_righting_timeout()` never checks u_z

Trial 1's log (`landing_scout_1_full_inversion_trial1.log`) shows the exact mechanism:

```
[04:18:38] Still inverted (u_z=-0.81), retrying with alternate roll axis/sign (attempt 2/5)
...RIGHTTRACE samples climbing toward w=160 (the wheel-speed ceiling) while u_z oscillates...
[04:18:39] Still inverted (u_z=0.99), retrying with alternate roll axis/sign (attempt 3/5)
```

u_z=0.99 is essentially perfectly upright — yet logged as "Still inverted" and discarded into a fresh attempt (new roll axis/sign, wheel state reset). **Zero `HOLD-START`/`HOLD-LOST` markers fired anywhere in this trial's entire 5-attempt, ~78s sequence**, despite this near-perfect crossing. Root cause, read directly from `_finalize_righting_timeout()` (landing_controller.py, before this phase's fix): it runs unconditionally on every attempt timeout and *never inspects `u_z` at all* — it always logs "still inverted" and always burns an attempt. The per-tick hold-confirm gate that exists specifically to catch a u_z>0.9 crossing is bypassed entirely during the timeout's own brake-ramp sequence (`_righting_timeout_brake_ticks > 0`, which "takes full control of publishing... skip[s] the normal approach/hold-confirm control law entirely" per its own comment) — so a crossing that happens to occur only during that brief ramp-down window, as it did here (the body rolling through on residual momentum from having reached the wheel-speed ceiling), is never registered by anything.

This reframes the "torque vs. geometry" question: it's neither. **The maneuver is intermittently succeeding and then discarding its own success due to a timing/bookkeeping bug**, not failing to reach upright due to insufficient torque or an unfavorable contact geometry.

### Fix implemented

`_finalize_righting_timeout()` now checks `u_z` against `RIGHTING_HOLD_RELEASE_UZ` (the same 0.85 hysteresis floor the hold-confirm logic itself already uses elsewhere) before discarding progress. If the body is already at or above that floor at timeout, the SAME attempt gets a fresh window to complete a genuine hold (`righting_ticks` set to `1`, skipping the tick-1 reset/announce branch so `_righting_cmd_vel`, `_roll_dir`, and wheel history are preserved) instead of incrementing `righting_attempt` and resetting to a new roll axis. Bounded to `MAX_TIMEOUT_EXTENSIONS = 3` (tracked via a new `_righting_timeout_extensions` counter, reset at the start of each genuinely fresh attempt) so a body oscillating near the threshold without ever truly settling still reaches give-up in bounded time rather than looping indefinitely. Diff reviewed line-by-line before applying; syntax-checked (`ast.parse`) clean.

## 4. A mistake, documented honestly

The diagnostic batch was scoped for n=15 trials on the *unpatched* (logging-only) code, specifically to get a clean baseline count of how often the timeout-discard pattern occurs before touching any control logic. While reading trial 1's log and confirming the bug, I began drafting and then **applied the §3 fix directly to the live `landing_controller.py` while the batch was still running** (trial 3 was mid-fall when this happened). Since each trial launches a fresh `landing_controller` subprocess and this is an editable/symlinked install, trial 3 onward would have run under a mixed or ambiguous code version relative to trials 1-2. Caught this immediately upon checking trial 3's status and killed the batch rather than let it produce a contaminated dataset. Trials 1 and 2 are clean (both ran to completion under the original unpatched code, confirmed by their log timestamps predating the edit); trial 3 was killed during its fall phase before producing any righting-related log lines, so there is no ambiguous data to discard — only an incomplete trial. n=2 is a small diagnostic sample, but trial 1 alone is unambiguous, direct evidence of the bug (verbatim "still inverted (u_z=0.99)"), and trial 2 shows the already-established give-up-then-drift pattern (all 5 attempt-boundary u_z values genuinely inverted: -0.98, -0.82, -0.48, -0.39) for contrast.

## 5. Checkpoint verdict

Bug identified and root-caused with direct log evidence (not inferred): **confirmed**. Fix implemented, reviewed, syntax-checked: **done, not yet empirically tested** — that's Phase 15's job (a dedicated before/after comparison batch), started immediately after this report since the code is already in place and there's no reason to wait. The n=2 diagnostic sample here should not be read as a rate estimate of anything; it's process evidence for the bug's existence and mechanism, not a measurement.
