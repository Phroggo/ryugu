# Phase 9 — Tick/Wall-Time Mismatch Fix (narrow + scoped-broad)

Date: 2026-08-10
Scope: fix the tick/wall-time mismatch bug found live during Phase 8's directional-hop batch (§5.3 of `PHASE8_OVERNIGHT_REPORT.md`), per explicit direction: narrow fix first (unconditional), then a scoped version of the broad fix (`hopper_locomotion.py`'s launch-gating counters only, not `landing_controller.py`'s rest-window logic). Verified live before committing, not assumed correct from code review alone.

## 1. Files touched

### Source code (modified)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/hopper_locomotion.py` (+164/-51 net across the file; narrow fix in `_wake_model`, scoped broad fix throughout `tick()`)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/landing_controller.py` (narrow fix only, in `_wake_model`)

### New verification artifacts

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/verify_timing_fix.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/verify_timing_fix_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/verify_timing_fix_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/gz_verify.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d5_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d5_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d5_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d5_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d5_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d9_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d9_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d9_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d9_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/bridge_scout_1_d9_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d5_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d5_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d5_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d5_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d5_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d9_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d9_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d9_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d9_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/loco_scout_1_d9_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d5_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d5_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d5_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d5_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d5_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d9_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d9_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d9_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d9_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/attitude_scout_1_d9_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d5_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d5_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d5_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d5_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d5_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d9_rep1.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d9_rep2.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d9_rep3.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d9_rep4.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/landing_scout_1_d9_rep5.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase9_timing_fix_verification/PHASE9_CHANGE_REPORT.md` (this file)

## 2. What changed

### 2.1 Narrow fix (both files) — `_wake_model()`'s `Popen()` moved off the executor thread

`subprocess.Popen()` is nominally non-blocking, but the `fork()`+`exec()` underneath it is a real synchronous syscall that can stall the calling thread for a non-trivial duration under system load. Both `hopper_locomotion.py` and `landing_controller.py` call it from `_wake_model()`, on the same single-threaded rclpy executor that also runs their timing-critical `tick()`/callback logic. A stall there lets a backlog of missed 10Hz timer periods build up, which then fires in a rapid burst once unblocked. Fixed in both files by spawning the `Popen()` call itself on a throwaway daemon `threading.Thread`, so a slow `fork()` can never again stall callback processing regardless of system load.

### 2.2 Scoped broad fix (`hopper_locomotion.py` only) — wall-clock timing for launch-gating counters

Converted every elapsed-time threshold and interpolation fraction that previously depended on tick-counting to real `time.time()` deltas:

- **CROUCH's 10s/45s dwell gates** (`state_timer > 100` / `> 450`): now `crouch_elapsed = time.time() - self._crouch_entered_at`, compared against `10.0`/`45.0` seconds directly.
- **LAUNCH's ramp-progress fraction** (`(state_timer + 1) / ramp_ticks`): now `launch_elapsed / self.ramp_T`, where `self.ramp_T` (seconds, the real float already computed in `jump_target_callback`) is now stored as an instance attribute rather than only existing as a rounded `ramp_ticks` integer.
- **The post-ramp separation-wait deadline** (`SEPARATION_MAX_WAIT_TICKS`, renamed `SEPARATION_MAX_WAIT_SECONDS = 60.0`): now `sep_wait_elapsed >= 60.0` in real seconds, derived directly from `launch_elapsed`. This removes the need for Phase 7's tick-vs-wall-clock cross-check and the "TICK/WALL-TIME MISMATCH" warning entirely at this specific site — there is no longer a tick count in this decision that could be wrong.
- **FLIGHT's `CLEARANCE_TICKS`/`RETRACT_RAMP_TICKS`** (renamed `CLEARANCE_SECONDS = 8.0`, `RETRACT_RAMP_SECONDS = 4.0`): now gated on `flight_elapsed = time.time() - self._flight_entered_at`, set at both transitions into FLIGHT (mid-stroke tip-abort and genuine-separation-confirmed).
- **The CROUCH/LAUNCH keep-awake nudge cadence** ("every 20 ticks", i.e. every 2s nominal): also converted to a real 2s wall-clock check (`self._last_wake_nudge_at`) -- not explicitly named in the requested scope, but the same variable (`state_timer`) drove it, and leaving it tick-based would have let a burst fire several redundant teleport calls in quick succession (wasteful and, per the code's own extensive warnings elsewhere, each teleport can destroy real velocity if it lands while the body has genuine motion). Converted for consistency with "convert state_timer... to wall-clock deltas" rather than leaving a partially-converted residual.

`self.state_timer` itself is kept, but now used only as a plain incrementing counter and an `== 0` first-tick-of-state flag -- both uses are structurally immune to burst timing (identity and increment-by-1 don't care how fast successive calls happen). `self.ramp_ticks` is likewise kept for logging/legacy reference only. `SEPARATION_SAMPLE_TICKS`/`SEPARATION_CONFIRM_SAMPLES` (the velocity-sampling *cadence* during the separation wait, not the deadline) were deliberately left tick-based -- not in the requested scope, and a burst-clustered sample there is a minor inefficiency, not the class of bug this fix targets. Flagged, not silently expanded into.

**Explicitly not touched, per direction**: `landing_controller.py`'s `REST_Z_TICKS`/`REST_VEL_TICKS` (the apex-dwell rest-window logic). No evidence of corruption there this session, and it underpins an already-published dwell-time figure (SS3.4's 22b/g analysis) -- left as a separate, deliberate task.

## 3. What was run

`verify_timing_fix.py`: n=5 at 9.0m (known-good scenario, checking for regression) and n=5 at 5.0m (the exact short-ramp scenario that exposed the bug in Phase 8's P4 batch), full spawn+4-node lifecycle per trial, same methodology as every prior phase's launch-delivery harness. Also programmatically scans every `loco_scout_1` console log for the real elapsed time between "Crouching" and "IGNITION" and flags anything under 9s (the CROUCH gate should never allow less than 10s) -- a direct, automated check of gate integrity, not just an indirect read from the final ratio.

Command: `python3 verify_timing_fix.py`.

## 4. Results

**CROUCH GATE INTEGRITY: PASS.** All 10 trials (5 at 9.0m, 5 at 5.0m) show Crouching->IGNITION gaps of 10.00-10.08s -- correct, consistent, and specifically confirmed at 5.0m, the exact scenario that previously bypassed the gate in 34ms.

| Distance | n | stabilized | no_separation | mean ratio | ratios |
|---|---|---|---|---|---|
| 9.0m | 5 | 5 | 0 | 0.218 | 0.219, 0.218, 0.219, 0.218, 0.218 |
| 5.0m | 5 | 5 | 0 | 0.293 | 0.292, 0.294, 0.293, 0.294, 0.292 |

No timeouts, no separated-never-stabilized outcomes, no gate bypasses.

## 5. What didn't go as planned

**A real, small behavioral shift at 9.0m, honestly reported rather than glossed over.** Phase 7/8's n=100+30 samples at 9.0m established mean=0.212-0.213 (std ~0.0015-0.0019, max observed 0.213). This verification's 9.0m mean is 0.218 -- outside that established range, not just sampling noise. Most likely explanation: the old tick-based ramp used `ramp_ticks = round(ramp_T * 10)`, a discretized approximation of the real ramp duration (e.g. 28 ticks = exactly 2.8s for a ramp_T of 2.804s, plus whatever small per-tick scheduling overhead rclpy's timer accumulated in practice). The new code targets the exact `ramp_T` float directly via `time.time()` deltas, with no discretization or tick-scheduling drift. Given the established (if weak) inverse relationship between ramp duration and delivered velocity documented in Phase 6's calibration sweep, a slightly shorter/more precise ramp plausibly explains a slightly higher delivered ratio. **This means the Phase 7/8 n=100+130 baseline no longer exactly characterizes the current code's behavior** -- not by much, but by more than noise. Not re-verified at large n this phase (out of scope for a fix-verification pass); flagged here for whoever next needs a precise launch-delivery baseline to decide whether a fresh large-n confirmation is warranted before citing Phase 7/8's exact numbers going forward.

5.0m has no prior directional-comparable baseline (Phase 8's P4 trials were both invalid), so 0.293 is simply the first valid data point at this distance, not a comparison.

## 6. Checkpoint

- Narrow fix: applied to both files, unconditionally as directed. **Done.**
- Scoped broad fix: applied to exactly the four items named (`state_timer`, `SEPARATION_MAX_WAIT_TICKS`, `CLEARANCE_TICKS`, `RETRACT_RAMP_TICKS`) plus the directly-coupled wake-nudge cadence (same variable, flagged rather than silently left inconsistent); `landing_controller.py` deliberately untouched. **Done.**
- Verified live, not assumed: CROUCH gate integrity directly confirmed at both the known-good (9.0m) and previously-broken (5.0m) scenarios, automated pass/fail check, not a manual spot-read. **Done, PASS.**
- Regression check: 9.0m and 5.0m both fully stabilize, no anomalies -- but a real ~2.5% ratio shift at 9.0m is reported plainly rather than hidden, since it's a genuine (if minor) behavioral consequence of the fix.

**Checkpoint: PASS.** Phase 8's Priority 4 (directional hop validation) can now be safely rerun.
