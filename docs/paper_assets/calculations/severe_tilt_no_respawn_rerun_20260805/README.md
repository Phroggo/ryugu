# Severe-tilt rerun, fixed for the IMU-orientation bug, 2026-08-05

Reruns the 8 severe-tilt (>120 deg) trials from the C15/C16 pre-redesign
baseline (`../pre_redesign_self_righting_baseline_20260804/`) using a
spawn method that avoids the IMU-orientation bug found while investigating
why none of those 8 trials ever triggered a righting attempt at all (see
`../imu_orientation_bug_20260804/README.md`).

## Method

`scout_1` (pre-redesign controller, commit `63f73b8`, temporarily swapped
in and restored to production afterward -- see below) is spawned upright
exactly **once** for the whole batch and never removed/recreated again.
Each trial reaches its target tilt via a smooth in-place quaternion-slerp
animation (`gz service .../set_pose`, ~15 steps over ~2s plus a short
hold), starting from the robot's actual current orientation (read live
from odometry, not assumed) rather than an instant teleport -- keeping
the IMU sensor's reference frame valid throughout, since it never
re-latches to a new spawn pose.

Reruns the exact same 8 (tilt, azimuth) pairs as the original severe-tilt
subset of the C15/C16 batch, for as close to an apples-to-apples
comparison as this spawn-method change allows.

## Result

**2 of 8 trials genuinely triggered a righting attempt** (`RW righting
attempt` logged, matching the historical controller's real tilt), versus
**0 of 8 in the original respawn-based run.** This directly confirms the
IMU-orientation bug was suppressing genuine attempts, not just a
theoretical concern.

| Trial | Commanded tilt | Start u_z (odometry) | Righting triggered? | Final u_z (200s) |
|---|---|---|---|---|
| 1 | 121.6 deg | -0.524 (matches expected -0.524) | **yes** | 0.652 |
| 2 | 143.3 deg | -0.480 (drifted from expected -0.802, see caveat) | no | -0.651 |
| 3 | 168.6 deg | -0.979 (matches expected -0.980) | **yes** | 0.223 |
| 4 | 142.6 deg | -0.292 (drifted from expected -0.794) | no | 0.354 |
| 5 | 159.0 deg | -0.934 (matches expected -0.934) | no | 0.175 |
| 6 | 160.7 deg | -0.874 (drifted from expected -0.944) | no | 0.815 |
| 7 | 138.8 deg | -0.752 (matches expected -0.753) | no | 0.013 |
| 8 | 172.2 deg | -0.990 (matches expected -0.991) | no | -0.551 |

Full data: `results_final.json`. Full harness stdout: `stdout_final.log`.
Per-trial `landing_controller` console output:
`landing_controller_console_trial{1-8}.log`.

**None of the 8 trials reached a stable `landed=True` state within the
200s window** (whether or not righting triggered) -- even the 2 that did
trigger righting ended up in erratic, unsettled final orientations rather
than a clean recovery or a clean failure-in-place. This is a real, useful
secondary finding: the historical controller's righting logic, even when
correctly triggered, doesn't cleanly resolve within a reasonable window
when starting from a suspended (never-actually-touched-down) severe tilt.

## Caveat: trials 2, 4, 6 did not land precisely on their commanded angle

The harness reads the robot's live orientation before each trial's
animation specifically to avoid compounding drift across trials -- this
fixed the wild divergence seen in an earlier attempt (one trial landed
with the wrong *sign* entirely). But 3 of the 8 trials still show a
meaningful gap between commanded and actual start tilt (up to ~30 deg).
Likely cause: `set_pose` overrides position/orientation but not velocity,
so if the previous trial ended with real residual angular velocity (very
plausible after 200s of unresolved tumbling in near-zero-g), that motion
keeps acting on the body *between* successive animation steps, fighting
the intended slerp path. This wasn't chased down further given time
constraints -- all 8 trials still landed in genuinely severe territory
(worst case, trial 4, still reached -0.29, well past the 0.85 threshold
that defines "badly tilted" in the controller's own code), so the
headline finding (IMU fix restores genuine attempts) holds regardless,
but the *specific* commanded-vs-actual angle correspondence for trials
2/4/6 should not be read as precise.

## Status

**Confirms the IMU-orientation bug was suppressing real righting attempts
at severe tilts**, and reveals a second, real limitation: even with
attempts correctly triggered, the historical controller does not cleanly
resolve a severe-tilt righting sequence within 200s when the robot never
had an actual ground-contact landing. `ryugu_sim/landing_controller.py`
has been restored to the production version and rebuilt after this test
-- see the commit for this evidence for confirmation.
