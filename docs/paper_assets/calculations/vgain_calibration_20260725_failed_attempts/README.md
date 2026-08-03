# Failed calibration attempts, 2026-07-25

Two earlier attempts at the launch-velocity calibration sweep behind the
paper's C1-C9 launch-calibration claims. Neither produced a usable n=7
dataset. Kept as evidence of what was tried, not as data backing any
number in the paper.

## attempt_1_132802
Trial 1 (d=0.5m) separated but never stabilized within the 90s window
(discarded per methodology). Trials 2-5 (d=1.5 to 6.0m) all hit
"TIMEOUT waiting for separation signal" -- the harness never detected a
genuine separation event. Trials 6-7 (d=7.5, 9.0m) timed out even before
that, waiting for the agent to report landed/ready.

## attempt_2_140559
Trial 1 (d=0.5m) stabilized at ratio 0.534. Trial 2 (d=1.5m) stabilized at
ratio 7.306 -- a roughly 7x overshoot that is very unlikely to be real
physics (landing_controller.log around this trial shows repeated false
"sustained contact accel" resets, consistent with the launch reading being
contaminated by residual motion/false triggers from the prior trial rather
than measuring the actual delivered velocity cleanly). Trials 3-7 (d=3.0m
and up) all timed out waiting for landed/ready -- the robot never settled
enough to start the next trial within the harness's original 10-minute
per-trial window.

## Root cause and fix
See docs/paper_assets/calculations/vgain_calibration_20260803/ for the
rewritten harness (calibration_v2.py) and its rerun, which fixes the
separation-detection bug (subscribes to /separation directly instead of
whatever heuristic the original, now-lost script used) and gives each
trial a much larger time budget consistent with this platform's observed
long flight times at some commanded distances.
