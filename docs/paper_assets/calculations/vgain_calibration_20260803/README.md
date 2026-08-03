# Launch-velocity (V_GAIN) calibration sweep, 2026-08-03

A fresh attempt at the 7-distance launch-velocity calibration sweep behind
the paper's launch-calibration claims (delivered-to-requested velocity
ratio at commanded distances 0.5, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0 m). Two
earlier attempts at this same sweep from 2026-07-25 are archived in
`../vgain_calibration_20260725_failed_attempts/` and produced no usable
data; see that directory's README for why.

## Result: 2 of 7 distances produced clean, usable data

`final_combined_results_n7.json` is the complete, merged result:

| Distance | Status | Delivered ratio | Notes |
|---|---|---|---|
| 0.5 m | stabilized | 0.938 | near-full delivery |
| 1.5 m | no separation | -- | robot's stance never reached uz>0.85 |
| 3.0 m | no separation | -- | same |
| 4.5 m | no separation | -- | same |
| 6.0 m | no separation | -- | same |
| 7.5 m | no separation | -- | same |
| 9.0 m | stabilized | 0.209 | degraded delivery |

The two clean points bracket the paper's own aggregate description
("2/7 near-full (mean 0.95), 5/7 degraded (mean 0.19)") reasonably well:
0.938 is close to the near-full bucket's mean, and 0.209 is close to the
degraded bucket's mean. This is not proof of the full n=7 dataset, but it
is real, independently-measured data consistent with the paper's stated
pattern rather than contradicting it.

## Why the other 5 distances failed, and why a second pass didn't fix it

**Pass 1** (`launch_velocity_calibration_pass1_harness.py`,
`pass1_results_all_seven_distances.json`, `pass1_stdout.log`): all 7
distances attempted. 0.5 m and 9.0 m separated and stabilized cleanly. The
other 5 hit "TIMEOUT waiting for separation" (90-120s). Checking the
`hopper_locomotion` node's own console output for these trials showed the
real cause: each one hit the node's internal crouch-phase stance gate
(`uz > 0.85` and `speed < 0.012`, checked continuously, 45s timeout) and
aborted with "Aborting hop: stance still bad at crouch timeout" before
ever reaching ignition. Pass 1's own readiness check used a looser 0.02
m/s threshold and only a single instantaneous sample, so it was publishing
jump commands before the robot was actually settled enough for the
stricter, continuously-checked internal gate.

**Pass 2** (`launch_velocity_calibration_pass2_tightened_stance_gate_harness.py`,
`pass2_results_five_retried_distances.json`, `pass2_stdout.log`): retried
just the 5 failed distances, this time computing `uz` directly from the
IMU orientation (the same quantity the robot's own gate uses) and
requiring it sustained above 0.85 with speed below 0.012 for 3 consecutive
checks before publishing, with a much longer (400s) timeout. Every one of
the 5 still failed to reach a good stance within the full 400s window and
proceeded anyway once the timeout expired, then still failed to separate.
A live IMU check partway through the sweep found the robot sitting at
roughly 76 degrees of tilt (uz ≈ 0.24), not correcting itself.

**Conclusion:** this is not a bug in the calibration harness -- the
harness now matches the robot's own stance-quality gate exactly. It
reflects a real characteristic of the current self-righting/stance-recovery
behavior: once the robot ends up tilted, it does not reliably return to
upright on its own within the time windows tested here. This is the same
underlying reliability question raised by the self-righting statistics
claims elsewhere in the paper (moderate-tilt and full-inversion recovery
rates) -- getting a complete n=7 calibration dataset and getting reliable
self-righting numbers both depend on the same mechanism.

## To get the remaining 5 distances

The stance-recovery issue would need to be addressed directly (either by
improving the self-righting mechanism, or by deliberately re-establishing
an upright stance immediately before each calibration trial rather than
relying on organic post-landing recovery, which changes what the test
measures -- see the discussion of this tradeoff before treating any future
rerun's numbers as directly comparable to a fully organic sweep).
