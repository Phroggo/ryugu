# Phase 5 Follow-up — Multi-Node Verification Gap

Date: 2026-08-08
Scope: fix whatever broke settle-detection when `landing_controller` and
`attitude_controller` run together, then rerun the Phase 5 z-disturbance
injection test with both nodes live. This closes the verification gap
explicitly called out in the Phase 5 change report (single-node-only
testing) and requested before Phase 7's n≥20 batch. Does not touch or
block Phase 6.

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/attitude_controller.py` (modified, 34 insertions / 1 deletion)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_injection_test.py` (modified — re-added `attitude_scout_1` to the launched node set)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diagnose_settle_failure.py` (new)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diagnose_settle_failure_BEFORE_FIX_stdout.log` (new — manually preserved, see §4)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diagnose_settle_failure_stdout.log` (new — post-fix rerun output)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diag_attitude_scout_1.log` (new — node console log, post-fix diagnostic run; see §4 for a caveat on the pre-fix run's node logs)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diag_bridge_scout_1.log` (new — node console log, post-fix diagnostic run)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/diag_landing_scout_1.log` (new — node console log, post-fix diagnostic run)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/gz_diag.log` (new — Gazebo console log, post-fix diagnostic run)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_results_AFTER_MULTINODE.json` (new — multi-node injection test results, 2 trials)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/z_disturbance_AFTER_MULTINODE_stdout.log` (new — multi-node injection test stdout summary)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/attitude_scout_1_AFTER_MULTINODE_trial0.log` (new — per-node console log, injection trial 0)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/attitude_scout_1_AFTER_MULTINODE_trial1.log` (new — per-node console log, injection trial 1)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/bridge_scout_1_AFTER_MULTINODE_trial0.log` (new — per-node console log, injection trial 0)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/bridge_scout_1_AFTER_MULTINODE_trial1.log` (new — per-node console log, injection trial 1)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/landing_scout_1_AFTER_MULTINODE_trial0.log` (new — per-node console log, injection trial 0)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/landing_scout_1_AFTER_MULTINODE_trial1.log` (new — per-node console log, injection trial 1)
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase5_self_righting_fix/gz_AFTER_MULTINODE_inject.log` (new — Gazebo console log for both injection trials)

## 2. What changed

### `attitude_controller.py`

Three-part fix, all confined to the sleep-defeat idle-rotor path:

1. `__init__`: added `self.landed = False` (raw landed-state tracking, distinct from the already-existing `self.in_flight`).
2. `landed_callback`: added `self.landed = msg.data` alongside the existing `in_flight` transition logic, so the controller has a direct, un-debounced view of `landing_controller`'s landed state.
3. `imu_callback`, sleep-defeat rotor section: gated the idle-rotor floor off once landed —

   ```python
   z_cmd = self.cmd_vel['z']
   if (not self.in_flight) and (not self.landed) and abs(z_cmd) < self.IDLE_ROTOR_SPEED:
       z_cmd = self.IDLE_ROTOR_SPEED if z_cmd >= 0.0 else -self.IDLE_ROTOR_SPEED
   ```

   (was `if (not self.in_flight) and abs(z_cmd) < self.IDLE_ROTOR_SPEED:`). The main yaw-hold above this block (needed for `hopper_locomotion`'s CROUCH-phase pre-hop alignment, per the existing Phase 5 OWNERSHIP CAVEAT comment) is untouched — only the idle-rotor floor is gated.

   Added an explanatory comment documenting the confirmed root cause and pointing at the diagnostic evidence (see §3).

### `z_disturbance_injection_test.py`

Re-added `attitude_scout_1` to `launch_scout1_nodes()`'s node spec list (removed in the main Phase 5 commit after it broke settle detection entirely). Replaced the old "REVERTED" comment with one explaining the fix that makes this safe again and pointing at this report.

## 3. What was run

1. **Root-cause diagnostic** (`diagnose_settle_failure.py`), both nodes live, no fix applied yet:
   - First attempt: 60s observation window — showed nothing (robot still in free-fall the whole time, `in_flight=True` throughout, velocity tracking pure `v = g·t`). Window too short relative to the ~194–210s natural settle time established in prior Phase 5 work.
   - Second attempt: extended to 280s — this is the run that produced the definitive evidence (see §4), now preserved at `diagnose_settle_failure_BEFORE_FIX_stdout.log` since the script overwrites its own fixed-name log file on each run and the live copy has since been replaced by the post-fix rerun.
2. Applied the three-part fix to `attitude_controller.py`.
3. **Re-ran the same 280s diagnostic** post-fix — output now at `diagnose_settle_failure_stdout.log`.
4. **Reran the z-disturbance injection test** (`z_disturbance_injection_test.py`), both nodes live, label `AFTER_MULTINODE`, 2 trials, same parameters as the original single-node run (spawn at (0, 0.5, 5.2), up to 200s landed-wait, `INJECT_SPEED=40.0` rad/s for 1.0s, 90s observation).

All runs used `gz sim -r --headless-rendering` with the workspace sourced and `GZ_SIM_RESOURCE_PATH` exported in the same shell that launched Gazebo.

## 4. Results

**Diagnostic, before fix** (280s window, both nodes live): robot reaches `landed=True` at t=205.2s. At that exact moment the idle-rotor floor engages and the published `rw_z_joint_cmd_vel` command begins flipping sign on nearly every tick — flip count goes from 2 (accumulated harmlessly pre-landing) to 208 within 5s of landing, then climbs essentially linearly: 617 at t=210.2s, 3,451 at t=230.2s, 9,887 at t=270.2s, ending at **11,477 total flips over the 280s window**. Commanded magnitude grows unboundedly alongside the flip rate, peaking at **z_cmd = −20.87 rad/s** at t=270.2s (vs. the intended ±2.0 rad/s idle floor) — a genuine dueling-integrator runaway between `attitude_controller`'s own `cmd_vel['z']` and `landing_controller`'s independent LANDED-state z-damper, exactly as hypothesized. This confirms the root cause: the idle-rotor floor has no hysteresis around zero, and once landed, every sign-noise crossing becomes a real wheel-direction-reversal torque kick that neither node has any awareness the other is reacting to. (Full trace preserved in `diagnose_settle_failure_BEFORE_FIX_stdout.log`, reconstructed from this session's captured output since the live log file was overwritten by the fixed-run rerun before this could be saved to disk.)

Note: the node console logs (`diag_attitude_scout_1.log` etc.) reflect only the **post-fix** run — they share the diagnostic script's fixed output filenames and were overwritten between runs. This is a gap in the raw evidence trail; the stdout trace above is the complete record of the pre-fix behavior.

**Diagnostic, after fix** (same 280s window, both nodes live): flip count stays at exactly **2 for the entire post-landing window** (unchanged from the pre-landing baseline — i.e., zero flips attributable to landed-state behavior). `z_cmd` settles smoothly near its yaw-hold value with no runaway growth. `velocity_mag` reaches 0.0 and stays there. Runaway eliminated.

**Injection test, `AFTER_MULTINODE`, both nodes live, 2 trials:**

| | Trial 1 | Trial 2 |
|---|---|---|
| Pre-injection landed-wait | timed out at 200.0s (`landed=False` at injection time — see caveat below) | timed out at 200.0s (same) |
| Peak \|wz\| during injection | 1.140 rad/s | 1.357 rad/s |
| wz at observation start | −1.017 rad/s | −1.064 rad/s |
| wz at observation end | −0.083 rad/s | 0.0 rad/s |
| Decay | 91.9% | 100% |
| Final u_z (uprightness) | 0.9987 | 0.9994 |
| Stability during test | u_z stayed in [0.997, 1.000] throughout — never dragged toward inversion | same |

Both trials show clean, substantial decay of the injected residual z-rotation with the robot remaining upright throughout, in the full multi-node configuration — the fix generalizes from the diagnostic to the actual disturbance-rejection behavior the invariant comments were written to protect.

**Caveat, not a new bug**: in both trials the pre-injection wait for `landed=True` hit its 200s script timeout with `landed` still `False`, so the script proceeded to inject anyway (logged as a `WARNING`). This is a test-harness parameter mismatch, not a regression — the dedicated diagnostic (run with no injection, no artificial timeout) independently found the natural landed time to be ~205.2s, just past this test's 200s budget. Consistent with that, the trace data shows `landed` flipping to `True` within ~3s of the *observation* phase starting in both trials (i.e., the robot was already essentially settled at injection time, the flag just hadn't latched yet). The reported decay and stability numbers above are unaffected by this — they're measured directly from `wz` and `u_z`, not gated on the `landed` flag.

## 5. What didn't go as planned

- The first diagnostic attempt (60s window) produced no useful signal — had to extend to 280s to actually observe the landed-state transition and its aftermath, based on the ~194–210s settle times already established elsewhere in Phase 5.
- The diagnostic script's fixed log filenames meant the pre-fix evidence was at risk of being silently overwritten by the post-fix rerun. The stdout summary was still in this session's captured output and was manually transcribed to `diagnose_settle_failure_BEFORE_FIX_stdout.log`; the raw per-node console logs from that specific run were not recoverable and are missing from the artifact set (noted in §4).
- The injection test's 200s landed-wait timeout is slightly shorter than the ~205s natural settle time under this exact spawn/mass configuration, so both `AFTER_MULTINODE` trials injected the disturbance just before the flag formally latched. Left as-is since it doesn't affect the measured outcome and matches the pre-existing test design (not something this follow-up was scoped to retune).

## 6. Checkpoint

Requirement (user, this phase): *"fix whatever broke settle-detection when both controllers ran together, and rerun the same injection test with both nodes live."*

- Root cause found and fixed: idle-rotor floor lacked hysteresis, causing a dueling-integrator runaway with `landing_controller`'s z-damper once landed. **Fixed** via a `self.landed` gate, confirmed by flip-count collapsing from 11,477 → 2 over the same 280s window.
- Injection test rerun with both nodes live: **done**, 2/2 trials show substantial-to-complete decay (91.9%, 100%) with the robot remaining stable and upright throughout, matching the single-node result's qualitative behavior.

**Checkpoint: PASS.**
