# Self-righting reliability test (C17/C18), 2026-08-03

Attempted to independently measure the paper's post-redesign self-righting
claims ("mild-to-moderate tilts recover reliably... full inversions recover
in about one case in four") by teleporting scout_1 to controlled tilts and
watching whether the current `landing_controller`/`attitude_controller`
righting logic recovers it. This did not produce a usable result. Recording
the full history here because most of the six attempts uncovered a real,
useful finding along the way, even though none produced clean statistics.

## Root cause, found after this test concluded

All six attempts spawned scout_1 close to the ground (z = 0.05-0.4 m) for
speed. **That was the mistake.** At this particular spawn XY, anything below
roughly 6 m of clearance causes a real Gazebo contact "pop-out" against the
local terrain on creation -- the robot's collision geometry doesn't fully
clear the terrain at spawn, and gz-sim's contact solver resolves the
interpenetration with a genuine velocity kick. Under Ryugu's near-zero
gravity, that kick never decays: the robot just drifts away in a real,
if very slow, ballistic arc, contaminating everything downstream. This was
confirmed directly: the same drift happens with *zero* ROS nodes running
(pure Gazebo physics), disappears entirely when spawning at z=6.0 (matching
`spawner.py`'s own convention, which uses that height for exactly this
reason), and is unrelated to `_wake_model()`, DART sleep, or any of the
robot's control code -- all of which were investigated and ruled out or
fixed first, in the order below, before the actual cause was found.

**If this test is retried, the fix is simple: spawn at z>=6.0, matching
`spawner.py`, not a "convenient" low height.**

## Attempt-by-attempt history

- **Attempt 1** (`attempt1_DISCARDED_stale_imu_subscription_bug.json`) --
  the rclpy monitor subscribed to `/scout_1/attitude_error` and appeared to
  show instant recovery on every trial. A direct raw `gz topic` pose check
  mid-run showed the robot was actually still fully inverted while the
  script logged "recovered" -- the monitor's subscription was returning
  stale/frozen data across entity respawns. Fixed by switching to
  `/scout_1/odometry` and requiring a minimum count of freshly-received
  messages before trusting a reading.
- **Attempt 2** (`attempt2_DISCARDED_dart_sleep_froze_all_trials.json`) --
  with the subscription bug fixed, every trial now showed the robot frozen
  at exactly its commanded spawn tilt for the full 90s window. Root cause:
  gz-sim8 puts a motionless freshly-spawned model to sleep, and there is no
  anti-sleep mechanism active for a body that isn't yet recognized as
  landed (the grounded idle-rotor trick in `attitude_controller.py` is
  gated `if not self.in_flight`). Attempted fix: a periodic small
  in-place `set_pose` nudge to break sleep.
- **Attempt 3** (`attempt3_DISCARDED_landing_fsm_stuck_in_flight.json`) --
  the nudge fix didn't help; `landing.log` showed `landing_controller`'s
  state machine had been wedged in FLIGHT since the very first trial's
  spawn and never re-armed on subsequent entity respawns, because
  respawning the Gazebo *entity* does not reset the Python *node's*
  internal FSM state. Fixed by killing and relaunching all four scout_1
  ROS nodes before every trial, not just respawning the entity.
- **Attempt 4** (`attempt4_DISCARDED_rest_window_never_completed.json`) --
  with fresh nodes per trial, the robot now stayed in IDLE indefinitely.
  `landing_controller.py` has a documented rest-window fallback (60s of
  static altitude + low velocity) meant to declare contact without a real
  impact spike, but it consistently failed to accumulate the needed
  60 consecutive seconds.
- **Attempt 5** (`attempt5_stdout_DISCARDED_dart_sleep_no_fall.log`) -- tried
  spawning higher (0.4 m) for a genuine fall-and-impact instead of relying
  on the rest-window fallback near ground. The robot never moved at all --
  same DART-sleep-before-any-velocity-builds issue as attempt 2, this time
  blocking the fall itself before it could start.
- **Attempt 6** (`attempt6_stdout_final_run.log`,
  `final_run_4_trials_all_failed.json`) -- combined the wake-nudge fix with
  the fresh-node fix. All 4 trials (2 moderate, 2 full-inversion) still
  failed to reach `landed=True` within 160s, each one completely frozen for
  the entire window despite 5 wake-nudges per trial. This is the point at
  which the investigation was paused and the actual root cause (terrain
  clearance at spawn) was found afterward, in a separate investigation --
  see above.

## Incidental failure #2, captured live during the C9 rerun

The batch-test approach above was never revisited after the terrain-height
fix (see `../launch_stance_reliability_tests_20260803/README.md`
instead, which reruns C14 and C9 with the fix applied) -- but a genuine,
organic self-righting failure happened live during that C9 rerun, and it
was caught and logged. This is real evidence, not from a purpose-built
righting test, and it is a direct contradiction of the paper's specific
C17 claim ("mild-to-moderate tilts... recovered reliably, every such case
in the sample").

**Sequence, from `../launch_stance_reliability_tests_20260803/landing_controller_console_c9_final_INCLUDES_righting_cascade.log`:**

1. The C9 hop landed with real contact, but several genuine false
   "not actually landed" resets occurred first (see the C28 note in
   `../../claim_source_citations.md`).
2. It settled badly tilted: u_z=0.07 (~86 deg from vertical) -- "Settled
   badly tilted/inverted -- initiating RW righting roll".
3. Five RIGHTTRACE attempts followed. u_z barely moved across any of
   them, converging on ~0.70-0.703 (~45.4 deg) and never improving further
   -- this is a **moderate tilt by the paper's own definition**, not a
   full inversion, yet it did not recover.
4. `"Self-righting failed after 5 attempts -- giving up, marking LANDED
   anyway... Robot may still be physically inverted."` -- the exact same
   message, verbatim, as an unrelated incidental failure found earlier in
   today's session (during the C12 attempt in `../attitude_rerun_20260803/`,
   u_z stuck at 0.7727). Two independent real occurrences of the identical
   failure mode is a meaningful pattern, not a one-off fluke.
5. Marked LANDED anyway (the code's own documented fallback, to avoid
   hanging downstream logic).
6. `"Liftoff detected while LANDED (v=0.164 m/s sustained) -> back to
   FLIGHT"` -- an uncommanded kick, closely matching the magnitude of the
   paper's own cited "Law 3" example (0.128 m/s). Likely the post-landing
   stand-up/fold posture ramp acting on a still-tilted body.
7. That kick made things *worse*: raw telemetry
   (`incidental_failure_2_during_c9_test_45deg_to_165deg.jsonl`, captured
   live by `live_righting_capture_harness.py`) shows u_z settling at
   **-0.9661 (~165 deg -- essentially fully inverted)** for the rest of the
   180s capture window, apparently DART-asleep and undisturbed after the
   kick (the same in-flight anti-sleep gap documented in
   `../attitude_rerun_20260803/README.md`'s C14 section).

**This is real, organic counter-evidence to C17**: a moderate-tilt
(~45 deg) landing that the paper claims should recover "every... case in
the sample" instead failed all 5 attempts, was marked landed anyway per
the code's own known-limitation fallback, and was then kicked by an
unrelated bug into a full inversion -- worse than where it started.

## Files in this directory

- `self_righting_batch_harness_final_version.py` -- the attempt-6 harness
  (includes the fixes from attempts 1-3; does not include the z>=6.0 spawn
  fix, since that was found afterward).
- `attemptN_*.json` / `attemptN_stdout.log` -- per-attempt results and
  console output, as described above.

## Status

**The purpose-built batch test (attempts 1-6) never produced a valid
measurement.** But an incidental, organic failure captured live during a
separate C9 rerun (see above) provides real counter-evidence to C17: a
moderate (~45 deg) tilt failed to recover in all 5 attempts, matching an
earlier independent incidental failure (~39 deg tilt) from earlier in the
same session. Two real, independent, moderate-tilt failures against a
claim of 100% recovery in this tilt range is a meaningful discrepancy, not
proof the claim is entirely false (neither was from a large, controlled
sample), but it is no longer accurate to say this claim is completely
untested. C15/C16/C18 remain fully unconfirmed. A properly-run batch test
with the z>=6.0 spawn fix (see
`../launch_stance_reliability_tests_20260803/README.md`) would still be
the right way to get a real sample size if that's wanted.
