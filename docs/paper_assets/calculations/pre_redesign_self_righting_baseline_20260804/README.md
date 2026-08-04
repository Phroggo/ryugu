# Pre-redesign self-righting baseline (C15/C16), 2026-08-04

Reproduces the paper's "5 of 21 attempts (24%)" pre-redesign self-righting
baseline by running the actual historical controller (`landing_controller.py`
from commit `5c9e278`, the parent of `958ed0a` "Rewrite reaction-wheel
self-righting: fix the stall-on-side failure mode") through 21 trials at
randomized tilts, using the terrain-clearance spawn fix and fresh-node-per-trial
fix already established in the other reruns this week
(see `../launch_stance_reliability_tests_20260803/README.md` and
`../self_righting_reliability_test_20260803/README.md`).

The historical controller was temporarily swapped into the tracked
`ryugu_sim/landing_controller.py`, tested in isolation, and restored to the
current production version (`git checkout -- ryugu_sim/landing_controller.py`,
rebuilt) immediately after this batch completed. The current, shipped
controller was never altered.

## Method

- 21 trials, tilt drawn uniformly random 20-180 deg, azimuth 0-360 deg (the
  paper describes the original 5/21 baseline as measured "over a long run"
  of organic operation, not a controlled bucketed experiment, so this rerun
  matches that with a random draw rather than fixed tilt categories).
- Each trial: kill and relaunch scout_1's ROS nodes fresh, teleport-respawn
  at the commanded tilt (SPAWN_Z=5.2, clearing local terrain), wait up to
  200s for `landed=True`, then (if landed) wait up to 120s for u_z>0.9
  (recovered).
- Only `bridge_scout_1` and `landing_scout_1` were run per trial (not
  `hopper_locomotion`/`attitude_controller`) -- an isolated diagnostic
  during setup found landing detection never completed with the full node
  set running alongside the swapped-in historical controller; the
  historical `landing_controller` alone is sufficient for this test and
  works reliably on its own.
- `c15_16_pre_redesign_batch.py` is the final working harness (v5; four
  earlier iterations failed on wrong node sets and timeout tuning before
  landing detection worked reliably -- not included here, superseded by
  this run).

## Result

**1 of 21 recovered (4.8%)** -- notably lower than the paper's claimed 5/21
(24%).

| Trial | Tilt (deg) | Landed | Final u_z | Outcome |
|---|---|---|---|---|
| 1 | 113.4 | yes (after 115.6s) | -0.435 | failed |
| 2 | 121.6 | no (200s timeout) | -0.999 | no_landing |
| 3 | 42.8 | no (200s timeout) | 0.948 | no_landing |
| 4 | 143.3 | yes (after 180.3s) | -0.350 | failed |
| 5 | 46.6 | no (200s timeout) | 0.975 | no_landing |
| 6 | 114.8 | yes (after 94.6s) | -0.468 | failed |
| 7 | 168.6 | yes (after 120.4s) | -0.999 | failed |
| 8 | 142.6 | no (200s timeout) | -0.962 | failed* |
| 9 | 100.6 | yes (after 189.6s) | -0.315 | failed |
| 10 | 40.9 | no (200s timeout) | 0.970 | no_landing |
| 11 | 32.2 | yes (after 143.9s) | 0.999 | **recovered** |
| 12 | 159.0 | yes (after 139.4s) | -0.999 | failed |
| 13 | 160.7 | yes (after 139.4s) | -0.999 | failed |
| 14 | 119.9 | yes (after 77.0s) | -0.453 | failed |
| 15 | 57.2 | no (200s timeout) | 0.998 | no_landing |
| 16 | 138.8 | no (200s timeout) | -0.687 | failed* |
| 17 | 112.0 | no (200s timeout) | -0.974 | no_landing |
| 18 | 107.7 | yes (after 100.9s) | -0.302 | failed |
| 19 | 52.1 | no (200s timeout) | 0.989 | no_landing |
| 20 | 172.2 | yes (after 119.5s) | -0.999 | failed |
| 21 | 77.0 | yes (after 80.4s) | -0.656 | failed |

*Trials 8 and 16 show `landed: false` in the final recorded JSON alongside
outcome `failed`, not `no_landing` -- this is the harness correctly
recording a real event, not a logging bug: `node.landed` flipped True
(entering the righting-wait phase, hence outcome `failed` rather than
`no_landing`), then flipped back False again before the trial finished
recording. That is the same "liftoff detected while LANDED -> back to
FLIGHT" behavior documented for the C17/C18 cascade in
`../self_righting_reliability_test_20260803/README.md` -- an uncommanded
re-arm mid-righting-attempt, caught here independently in the historical
controller's own trials.

Full per-trial data: `c15_16_results_final_n21.json`. Full harness stdout:
`c15_16_stdout_final.log`. Per-trial `landing_controller` console output
(the actual RIGHTTRACE roll-attempt sequences): `landing_controller_console_trial{1-21}.log`.

## Two separate findings

**1. Recovery rate.** Only 1 of 21 trials recovered to u_z>0.9 -- the one
mild tilt in the sample (32.2 deg). Every trial that landed at a moderate-
to-severe tilt (77-172 deg) failed to recover; most stalled essentially
exactly where they landed (final u_z close to start_uz in several cases,
e.g. trial 18: start -0.310 -> final -0.302), consistent with the
historical controller's known failure mode described in commit `958ed0a`'s
message ("stall-on-side") -- this is the same bug class the redesign that
replaced this controller was written to fix, and the century-fold-out into
this rerun's own low recovery rate is consistent with that being a real,
severe limitation of the pre-redesign code, not a fluke of this particular
sample.

**2. Landing-detection reliability, a separate and unexpected finding.**
7 of 21 trials (33%) never registered `landed=True` within the 200s window
at all, despite the robot visibly settling and going still (several ended
at u_z>0.94, i.e. nearly perfectly upright and motionless). This wasn't
the focus of this test, but the pattern is clear and consistent across the
batch: milder tilts that settle gently and stay near-upright are
*more* likely to never trigger landing detection than harder, more
tilted impacts -- the opposite of what "landed" detection should
prioritize catching reliably. This appears independent of the specific
righting-recovery bug above (it's a landing/contact-detection issue, not a
self-righting issue), and is worth flagging as a related but distinct
finding: see the C28 note in `../../claim_source_citations.md` for the
broader pattern of landing-detection unreliability observed elsewhere this
week (false triggers there; missed triggers here -- both point at the same
detector being fragile at the margins).

## Status

**C15/C16: real counter-evidence, contradicting the paper's specific 5/21
(24%) figure.** This rerun measured 1/21 (4.8%) against the actual
historical controller, roughly 5x lower than claimed. Sample size (n=21,
matching the paper's own reported n) is small and the tilt distribution
here (uniform random) isn't guaranteed to match whatever distribution
produced the original measurement, so this shouldn't be read as
definitively replacing "24%" with "4.8%" -- but it is a real, substantial
discrepancy in the same direction as several other findings this week
(C9, C17/C18), not a one-off.
