# Phase 1 — tunable-constant audit (righting + launch logic)

Per Phase 0 verifier feedback. Every `self.CONST = number` in
`landing_controller.py`, `attitude_controller.py`, and `hopper_locomotion.py`
that interacts with righting or launch logic, checked for stale
diagnostic/temporary values.

| Constant | File | Current value | Status |
|---|---|---|---|
| `GENTLE_RIGHTING_SPEED` | landing_controller.py:269 | 20.0 | **Confirmed DEAD CODE.** Grepped every use of this name in the file: assigned once, referenced only in comments (lines 290, 296, 1120 — none of which read the value). The rev-2 acceleration-integrated taper (`RIGHTING_ACCEL_TAPER`, `RIGHTING_RATE_DAMP_SCALE/FLOOR`) replaced the old proportional speed-lookup this constant used to feed. **Disposition:** neither "revert" nor "record as open parameter" quite fits, since there's nothing live to tune — flagged in-code with an explicit dead-code comment (see change report) rather than silently left ambiguous. A future cleanup pass should delete it outright. It cannot become "the paper's final righting speed" because nothing in the running system reads it. |
| `RIGHTING_WHEEL_SPEED` | landing_controller.py:255 | 160.0 | Live, actively used (clamp ceiling for the integrated wheel-speed command, lines 821-822, 927). Not a diagnostic leftover — unchanged since before this week's debugging session. This is the actual "how fast can the righting wheel spin" parameter, not `GENTLE_RIGHTING_SPEED`. |
| `RIGHTING_ACCEL_TAPER` | landing_controller.py:327 | 0.6 | Live, part of the deliberate rev-2 redesign (not diagnostic). |
| `RIGHTING_RATE_DAMP_SCALE` | landing_controller.py:338 | 0.8 (was 1.5) | Live. Deliberately tightened 2026-08-06 after live telemetry showed overshoot — a real tuning revision, not a stray diagnostic value. |
| `RIGHTING_RATE_DAMP_FLOOR` | landing_controller.py:339 | 0.25 (was 0.4) | Live. Same tightening pass as above. |
| `MAX_RIGHTING_ATTEMPTS` | landing_controller.py:208 | 5 | Live. Was temporarily raised to 10 during the extended-budget diagnostic (Phase 0 baseline), correctly reverted to 5 before the Phase 0 snapshot. |
| `RIGHTING_TIMEOUT_TICKS` | landing_controller.py:270 | 1500 (15s) | Live. Same diagnostic (temporarily 3000/30s) correctly reverted before Phase 0. |
| `RIGHTING_HOLD_RELEASE_UZ` / `_MAX_RATE` / `_TICKS` / `_RAMP_TICKS` | landing_controller.py:393-396 | 0.85 / 0.15 / 200 / 50 | Live, part of the give-up→liftoff cascade fix. No diagnostic markers found. |
| `LANDED_DAMP_TAU_CAP` / `LANDED_DAMP_K_RATE` | landing_controller.py:369-370 | 0.006 / 0.066 | Live. Deliberately synced to `attitude_controller`'s `K_rate` (see below), not diagnostic. |
| `K_ang` / `K_rate` | attitude_controller.py:178-179 | 0.05 / 0.066 | Live, deliberate design values (bandwidth retune, documented 2026-07 rationale). Not diagnostic. Unchanged this session. |
| `IDLE_ROTOR_SPEED` | attitude_controller.py:219 | 2.0 | Live, no diagnostic markers found. |
| `V_GAIN` | hopper_locomotion.py:302 | 0.12 | Live. Comment explicitly says "empirical, from launch37 hop-meter data (unchanged — see note)" — a deliberate calibration, not a stray diagnostic value. **However:** this calibration was fit against the *current* (pre-redesign) mass distribution. Once Phase 2+ changes component masses, this empirical fit is expected to go stale as a natural consequence of the redesign — flagged here as a Phase 5 retune dependency, same bucket as the reaction-wheel inertia change (see AUDIT_TABLE.md row 1), not as a hygiene problem to fix now. |
| `CROUCH_HIP`, `LEAN`, `IDLE_RECOVERY_TICKS`, `RECOVERY_RAMP_TICKS`, `CLEARANCE_TICKS`, `RETRACT_RAMP_TICKS` | hopper_locomotion.py | various | Live, deliberate design constants. No diagnostic markers found. |

## Summary

**One constant found in a genuinely ambiguous state: `GENTLE_RIGHTING_SPEED`.**
It turned out to be dead code, not a live-but-stale value — a better outcome
than either option the Phase 0 verifier posed, but still required a code
touch (an explicit in-place comment, no value change) so a future reader
doesn't mistake it for something live. See the change report for the exact
diff.

**Two constants are flagged as expected-to-go-stale once mass changes land**
(not now, not a Phase 1 problem): `V_GAIN` (launch calibration) and,
separately, the reaction-wheel spin-axis inertia baked into
`attitude_controller`'s torque/acceleration budget comments (`I_wheel=2.7e-4
kg·m²`, referenced in `landing_controller.py`'s `max_wheel_accel` derivation)
— once AUDIT_TABLE.md row 1's real annulus inertia (3.94e-4 kg·m², about 46%
higher) is adopted, that derivation needs redoing. Both are explicitly out of
scope for Phase 1 (audit only) and should be picked up in Phase 5 per the
project's own phase plan.

No other diagnostic/temporary-value markers ("TEMPORARY", "DIAGNOSTIC", raw
TODO-style comments) were found anywhere in the three files' tunable
constants.
