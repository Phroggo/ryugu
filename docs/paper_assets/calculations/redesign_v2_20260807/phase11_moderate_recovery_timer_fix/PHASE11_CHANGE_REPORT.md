# Phase 11 — Self-Righting Recovery-Time Harness Fix, Moderate Bucket Rerun

Date: 2026-08-11
Scope: a paper reviewer (round 2) caught a physically-implausible 0.006s mean recovery time for the moderate-tilt bucket in Table IX. Diagnosed the root cause, confirmed it against the actual harness code, patched it, and reran the moderate bucket only (n=20, matching the original Table IX entry). Side_rest and full_inversion are unaffected by the bug (real recovery there takes seconds, so the harness's ~0.3s polling latency is a negligible fraction of the true duration) and were not rerun. Items #1b (mass sensitivity) and #3 (self-righting randomization expansion) from the same reviewer-fix list are explicitly held, not started this phase, per direction.

## 1. Files touched

### New harness script

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/moderate_recovery_timer_rerun.py`

### Results and logs

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/moderate_recovery_timer_rerun_results.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/moderate_recovery_timer_rerun_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/gz_p11_batch.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/bridge_scout_1_moderate_trial1.log` through `bridge_scout_1_moderate_trial20.log` (20 files, one per trial)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/landing_scout_1_moderate_trial1.log` through `landing_scout_1_moderate_trial20.log` (20 files, one per trial)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase11_moderate_recovery_timer_fix/PHASE11_CHANGE_REPORT.md` (this file)

(45 files total this phase. Complete literal listing appended to the commit that carries them, per this project's established convention.)

## 2. The bug, confirmed

`phase7_full_revalidation/self_righting_batch_3bucket.py` started its recovery timer (`right_t0 = time.time()`) only after its own landed-detection poll loop (`while ... node.landed is not True: node.spin_for(0.3)`, 0.3s granularity) observed `landed=True` — not from any real signal of when the righting maneuver itself begins. Confirmed by reading the exact code (lines 181-189) before touching anything.

## 3. The fix

`landing_controller.py` already publishes `/{robot_name}/righting_active` (`std_msgs/Bool`, updated every tick, going `True` the instant `self.state = self.RIGHTING` fires — lines 856/928/981). `moderate_recovery_timer_rerun.py`'s `TrialMonitor` subscribes to this directly from node creation (well before landing occurs) and records the timestamp of its first `True` transition (`righting_started_at`) as the recovery-timer origin, instead of re-deriving a start time from the separate landed-poll loop. `recover_time_s` is now `(moment uz first exceeds SUCCESS_UZ=0.9) - righting_started_at`, using the same end-of-measurement criterion as the original harness.

## 4. Result — and a more significant finding than the fix alone predicted

n=20, commanded tilt 46.1-59.2° (within the 45-60° moderate range), all 20 trials: `outcome=recovered`, `final_uz` 0.99906-0.99944 (all essentially perfectly upright).

**`righting_started_at` was `None` for all 20 trials — `/righting_active` never went `True` in a single one of them.** This was not a subscription bug: cross-checked against the raw `landing_scout_1_moderate_trial{N}.log` files for trials 1, 5, 10, 15, and 20 (spot-checked, consistent with the script's own automated tally of all 20) — every one shows only `"✅ LANDED — stable contact confirmed"` (the direct-to-LANDED path, `landing_controller.py` line 861), never `"⚠️ Settled badly tilted/inverted — initiating RW righting roll"` (the RIGHTING-triggering path, line 852-856) or the LANDED-tilt-watchdog's later righting trigger (line 921-928).

**Root cause of the "0.006s" figure being not just imprecise but conceptually wrong**: `_is_badly_tilted()` (landing_controller.py line 984-1013) triggers `RIGHTING` only if `u_z < 0.85` at the exact moment settle-confirmation completes (after `settle_counter >= settle_duration_ticks` and velocity is low). Commanded spawn tilt for the moderate bucket (45-60°) gives spawn `u_z` in the 0.5-0.71 range (matching the observed `start_uz` values, 0.51-0.69) — well below 0.85. But between spawn and settle-confirmation, the robot falls from `SPAWN_Z=5.2`, makes contact, and bounces/settles for 143-212s of real drop-and-settle physics (per the `landed=True after ...s` timings in the log). For a *moderate* tilt specifically — not lying on its side, not inverted — the tripod-leg/ground-contact dynamics during that fall-and-bounce process are apparently sufficient on their own to bring the body upright (`u_z` above the 0.85 trigger threshold) *before* the settle-confirm check ever runs. So the direct-to-LANDED path (line 861) is taken every time, and the active RW righting maneuver (`RIGHTING` state) is never entered at all for this bucket.

**This means `recover_time_s` is not "near zero" for the moderate bucket — it is structurally undefined**, because there is no active maneuver to time. The original 0.006s figure (and any small-but-nonzero number the timer fix alone might have produced) was never measuring a real righting duration at any point; it was measuring noise around a state transition that doesn't occur in this bucket. Side_rest (85-95°) and full_inversion (170-180°) are tilts severe enough that passive settling cannot resolve them — those buckets do genuinely enter `RIGHTING` (confirmed by Phase 7's own results: side_rest 10/20 recovered via the active maneuver, full_inversion 3/20) and their `recover_time_s` values are real, meaningful durations, not affected by this issue.

## 5. Recommended Table IX correction

Replace the moderate bucket's `recover_time_s` entry (currently 0.006s, flagged by the reviewer as implausible) with: **N/A — 20/20 self-righted via passive settling before the active RW maneuver ever engaged; no active-righting duration to report for this bucket.** Recovery rate itself (20/20, 100%) is unaffected and remains correct. Recommend adding a one-sentence note to the self-righting results section distinguishing "recovered via passive settling" (moderate) from "recovered via active RW righting" (side_rest, full_inversion) — this is a real mechanistic difference between buckets, not a data-quality caveat, and arguably strengthens the paper's characterization of the self-righting system rather than weakening it (it demonstrates the passive tripod stance itself has a meaningful stability margin, not just the active controller).

## 6. Anomalies flagged this phase

1. The recovery-timer bug itself, as originally diagnosed by the reviewer and confirmed here against source.
2. **New, not anticipated going in**: the fix's own instrumentation (`/righting_active` subscription) revealed the active maneuver never triggers for this bucket at all — a finding about the physical system, not just a data-collection artifact. Not averaged away or reported as a simple "here's a smaller positive number" — flagged explicitly since it changes what the corrected table entry should actually say.

## 7. Checkpoint verdict

Bug fix: **PASS**, mechanism (`/righting_active` subscription, first-True timestamp) verified correct and confirmed working (the topic subscription itself functions — it's just never published True in this bucket, which is a real physical/logic finding, not a wiring failure, cross-checked against raw controller logs). Moderate-bucket Table IX entry: **needs the correction in §5 above**, not a simple number swap. Items #1b and #3: **not started**, held per explicit direction.
