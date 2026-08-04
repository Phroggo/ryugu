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

## Files in this directory

- `self_righting_batch_harness_final_version.py` -- the attempt-6 harness
  (includes the fixes from attempts 1-3; does not include the z>=6.0 spawn
  fix, since that was found afterward).
- `attemptN_*.json` / `attemptN_stdout.log` -- per-attempt results and
  console output, as described above.

## Status

**Not confirmed or refuted.** No attempt produced a valid measurement of the
current self-righting reliability. A rerun with the spawn-height fix
applied would very likely produce usable data and is the recommended next
step if this claim needs live evidence.
