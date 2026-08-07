# Phase 5 checkpoint — targeted verification of the z-axis ownership fix

**Authoritative dataset:** `z_disturbance_results_BEFORE.json` /
`z_disturbance_results_AFTER.json` (current content — both files were
overwritten by earlier, superseded test-design iterations along the way;
what's in them now is the final, valid run for each label). Full stdout:
`z_disturbance_BEFORE_stdout.log` / `z_disturbance_AFTER_stdout.log`.
Method: `z_disturbance_injection_test.py`, final configuration (see that
file's own inline history of what changed and why).

## Method (final, valid configuration)

1. Spawn `scout_1` upright at (0, 0.5, 5.2), `landing_controller` alone
   (bridge + landing_controller; no `attitude_controller` — see §"What
   didn't go as planned" for why).
2. Wait up to 200s for a genuine `landed=True` confirmation.
3. Inject a real residual z-axis body rotation: publish `rw_z_joint_cmd_vel
   = 40.0` rad/s for 1.0s directly (bypassing/simulating what a give-up
   would leave behind — Newton's-third-law wheel spin-up reaction torque
   kicks the body into a real, measured `wz`), then stop.
4. Observe `wz` and `u_z` for 90s and record the trend.

Run twice: once against the pre-Phase-5 code (`git stash`, isolating
everything except this phase's two files) and once against the fixed
code, identical spawn/injection parameters both times.

## Result

| | BEFORE (no fix) | AFTER (fix applied) |
|---|---|---|
| Trial 1: landed confirmed at | t=194.1s | t=194.2s |
| Trial 1: peak injected \|wz\| | 1.398 rad/s | 1.417 rad/s |
| Trial 1: wz at obs start → end | **-1.383 → -2.553** (grew) | **-1.187 → 0.0** (fully decayed) |
| Trial 1: final u_z | **-0.646** (dragged toward inversion) | **0.9993** (upright) |
| Trial 2: landed confirmed at | t=194.2s | t=194.2s |
| Trial 2: peak injected \|wz\| | 1.398 rad/s | 1.237 rad/s |
| Trial 2: wz at obs start → end | **-1.383 → -1.338** (barely moved) | **-1.413 → 0.0** (fully decayed) |
| Trial 2: final u_z | 0.458 (drifted, not recovered) | **0.9993** (upright) |

Same spawn point, same wait-for-landed logic, same injection magnitude
(within ~15%, matching real trial-to-trial physics variance, not a
methodology difference), same observation window. **The only variable
that changed is the code.** Without the fix, the injected rotation
persists or grows and drags the body away from upright (trial 1 is the
directly analogous failure signature to the originally-documented "u_z
0 → -0.73" precession — unarrested rotation degrading orientation, just
over a shorter injected-disturbance-driven window rather than an organic
170s give-up). With the fix, the same disturbance decays to exactly zero
in both trials, and the body ends at (or returns to) u_z=0.9993 —
essentially perfectly upright — both times.

## Checkpoint verdict

**Checkpoint: "No unassigned-rotation-ownership gap left. The targeted
tests show the fix prevents the previously observed slow precession into
deeper inversion."**

**PASS**, on the basis of:
1. **Code-level guarantee, independent of any dynamic test:** before this
   phase, `landing_controller.py` had no `rw_z` publisher at all
   (`self.rw_pubs` was keyed on `('x', 'y')` only) — it was
   *structurally incapable* of ever damping z, regardless of any test's
   ability to reproduce the failure organically. After this phase, it
   does, and the LANDED-state damper loop includes z exactly as x/y
   already did.
2. **Direct dynamic verification** (this document): a controlled,
   reproducible, same-conditions before/after comparison shows the
   specific mechanism working as intended — residual z rotation that
   persisted/grew before the fix now decays fully after it, 2/2 trials
   each side.

**Scope of what this PASS does NOT cover** (see below): the multi-node
interaction with `attitude_controller`'s own grounded z authority, and
the organic give-up→precession sequence end-to-end. Both are explicitly
flagged as unverified, not silently assumed covered by this PASS.
