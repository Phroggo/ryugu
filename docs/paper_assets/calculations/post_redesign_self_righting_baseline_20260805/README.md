# Post-redesign self-righting batch (C17/C18), 2026-08-05

Same exact methodology as the pre-redesign C15/C16 batch
(`../pre_redesign_self_righting_baseline_20260804/`) -- 21 trials,
uniform-random tilt 20-180 deg, fresh nodes per trial, SPAWN_Z=5.2 -- but
against the **current, shipped** `landing_controller.py` (no code swap),
for a direct, controlled comparison of the redesign's actual effect on
raw recovery rate.

Uses the same entity-respawn spawn method as the pre-redesign batch (not
the no-respawn fix from `../severe_tilt_no_respawn_rerun_20260805/`), so
this result is directly comparable to the pre-redesign 1/21 figure --
both share the same IMU-orientation-bug caveat
(`../imu_orientation_bug_20260804/README.md`) for severe tilts that settle
quickly. The no-respawn rerun is the bug-free reference point if that
specific caveat matters for a given severe-tilt trial.

## Result

**1 of 21 recovered (4.8%)** -- exactly the same recovery count as the
pre-redesign controller's own 1/21 (4.8%) result on this methodology,
despite the redesign's explicit purpose (per its own commit message,
"Rewrite reaction-wheel self-righting: fix the stall-on-side failure
mode"). Full breakdown:

| Outcome | Count |
|---|---|
| Recovered | 1 |
| Failed (landed, did not reach u_z>0.9) | 10 |
| No landing (never registered `landed=True` within 200s) | 10 |

The one recovery was, again, the mildest tilt in the sample (23.4 deg).

Full per-trial data: `results_final_n21.json`. Full harness stdout:
`stdout_final.log`. Per-trial `landing_controller` console output:
`landing_controller_console_trial{1-21}.log`.

## A real qualitative difference, despite the tied headline number

Raw success count is identical, but the *pattern* of failure looks
different from the pre-redesign batch. There, most "failed" trials
stalled almost exactly where they landed (e.g. one pre-redesign trial:
start -0.310 -> final -0.302, essentially zero movement). Here, several
"failed" trials show real, substantial improvement that simply didn't
cross the 0.9 threshold in the 200s+120s window:

- Trial 19: start u_z=0.313 -> final u_z=**0.883** (a 72 deg tilt that
  climbed to within ~0.02 of the recovery threshold -- a genuine near-miss).
- Trial 15: start u_z=-0.710 -> final u_z=-0.446 (real, large improvement).
- Trial 14: start u_z=-0.615 -> final u_z=-0.701 (modest movement, though
  in the wrong direction by the end).

This is consistent with the redesign doing real, active work that the
pre-redesign controller's structurally different approach (fixed wheel
speed, blind axis alternation) mostly didn't -- it just isn't enough to
flip the raw recovery count in this particular n=21 sample.

## The `no_landing` share went up, not down

10 of 21 trials here never registered `landed=True` (vs 7 of 21 in the
pre-redesign batch) -- both controllers share the same landing-detection
code path (this wasn't touched by the self-righting redesign), so this
increase is most likely sampling noise from an independent random tilt
draw rather than a real regression, but it's worth noting rather than
silently omitting: this run's tilt distribution isn't the same one the
pre-redesign batch drew, so the two `no_landing` rates aren't a clean
apples-to-apples comparison on their own.

## Status

**C17/C18: the redesigned controller does not show a higher raw recovery
rate than the code it replaced in this controlled rerun (1/21 vs 1/21),
though qualitative evidence (several large-but-incomplete recoveries)
suggests it is doing more real corrective work per attempt.** Given the
paper's own framing describes post-redesign self-righting as "highly
reliable... from moderate tilts," this is a real, substantial discrepancy
worth flagging directly, not just softening -- though note this batch
used a uniform-random tilt draw across the full 20-180 deg range, not
restricted to "moderate" tilts specifically, so a tighter rerun
restricted to the moderate band would be the fairest direct test of that
specific claim if further precision is wanted.
