# Source citations for paper claims

This records the evidentiary basis retained in this repository for each
quantitative claim in the paper: live simulation reruns with raw telemetry
archived under `docs/paper_assets/calculations/`, and durable references in
the shipped simulation source itself. Where no such independently-retained
evidence exists for a specific figure, that is stated plainly rather than
citing anything that isn't part of the submitted repository.

---

## C1 -- 27% stroke / 0.19 m/s / 76 m
**Paper claim (S:S3.1):** a hop commanded at 27% stroke separated at 0.19 m/s and flew
over 76 m.
**Status:** no evidence independently retained in this repository backs this
specific figure. Recommend softening the paper's wording or arranging a
fresh, logged sim run before submission.

## C7 -- 0.5 rad lean test, 0.85 to 0.38 in 3.5s
**Paper claim (S:S3.1):** uprightness collapsing from 0.85 to 0.38 within a single 3.5s
ramp.
**Status:** no evidence independently retained in this repository backs this
specific figure. Recommend softening the paper's wording or arranging a
fresh, logged sim run before submission.

## C8 -- 9m hop, 695s flight, 0.16m horizontal
**Paper claim (S:S3.1):** a commanded 9m hop measured flying ~695s almost vertically,
0.16m horizontal drift.
**Status:** no evidence independently retained in this repository backs this
specific figure. Recommend softening the paper's wording or arranging a
fresh, logged sim run before submission.

## C10 -- Residual spin was a control-structure bug (pre-redesign)
**Paper claim (S:S3.2):** residual spin and steady-state error, measured before the
redesign.
**Status:** no evidence independently retained in this repository backs this
specific figure (the pre-redesign controller no longer exists in the
current code path). Recommend softening the paper's wording or arranging a
fresh, logged sim run against the historical controller before submission.

## C11 -- Rate-deadband experiment produced a +/-1.2 deg limit cycle
**Paper claim (S:S3.2):** a rate deadband was tried and produced a measurable +/-1.2°
limit cycle at exactly the deadband rate.
**Source, retained directly in the shipped code:** a comment in
`ryugu_sim/attitude_controller.py:230-236` records this history, explaining
why the shipped controller deadbands angle only, not rate.

## C12 -- In-flight body rates 0.005-0.015 rad/s, launch transient 0.24 rad/s
**Paper claim (S:S3.2):** in-flight rates damped to 0.005-0.015 rad/s; launch
transients of 0.24 rad/s.
**2026-08-03 live rerun status:** attempted, inconclusive -- see
`docs/paper_assets/calculations/attitude_rerun_20260803/README.md`
(`c12_liftoff_attempt_raw_telemetry.jsonl`). The commanded jump aborted
mid-crouch; a stray liftoff was captured instead of a clean launch. No
independently-retained evidence currently backs the specific numbers.

## C13 -- 107 deg yaw slew, overdamped, held within 1 deg
**Paper claim (S:S3.2):** 107 degree yaw slew converging overdamped, held within 1
degree at zero rate.

**2026-08-03 live rerun status: CONFIRMED independently.** See
`docs/paper_assets/calculations/attitude_rerun_20260803/c13_yaw_slew_raw_telemetry.jsonl`
-- fresh telemetry, reconstructed yaw from the odometry quaternion, converges
to 106.03°, <1° of target by t+9.3s.

## C14 -- 165 deg tumble damped to 3.6 deg in ~20s
**Paper claim (S:S3.2):** a 165 degree tumble damping to 3.6 degrees in ~20s.

**2026-08-03 live rerun status: recovery mechanism CONFIRMED working;
specific 165 deg magnitude NOT reached.** Three early attempts (artificial
pose injection while airborne/near-ground/at-spawn) all failed to even
engage the controller -- each hit a different test artifact, documented in
`docs/paper_assets/calculations/attitude_rerun_20260803/README.md`. A
fourth attempt using the original measurement's own method (forcing
`hip_joint_0` to overextend during the launch phase, inducing a genuine
asymmetric-torque tumble rather than an artificial pose injection) was
initially blocked by a spawn-height test artifact (fixed -- see
`docs/paper_assets/calculations/launch_stance_reliability_tests_20260803/README.md`),
then succeeded across four follow-up runs:

- Two small-override runs produced only mild wobbles (4.6 deg, 6.5 deg).
- Two large-override runs (-2.8 rad, near the hip joint's physical limit)
  produced genuine tumbles: **54.0 deg** peak, recovered to <1 deg within
  ~6s and held at 0.82 deg for the rest of a 40s window; **28.4 deg** peak,
  recovered to <1 deg within ~5s and held at exactly 1.019 deg for the
  rest of the window.

Both large tumbles show fast, overdamped, no-overshoot recovery to a
stable sub-1.1-degree residual -- qualitatively matching the paper's
description closely. Neither reached 165 deg specifically (a larger
induced tumble would need a different method -- the hip joint was already
near its physical limit). **Treat this as confirming the recovery
mechanism is real and works as described, but not as confirming the
specific 165->3.6 deg numbers.**

## C19 -- Fold-step ejection root cause
**Paper claim (S:S3.3):** a fold step at a marginal tilt was measured ejecting the
robot into a multi-minute parasitic arc.
**Status:** no evidence independently retained in this repository backs this
specific figure. Recommend softening the paper's wording or arranging a
fresh, logged sim run before submission.

## C20 -- Table II: 32/38 mm/s, 16/22 mm/s, 0.7-0.9m kicks
**Paper claim (S:S3.4):** contact-damping scheme comparison table.
**Status:** no evidence independently retained in this repository backs
this table. Recommend softening the paper's wording or arranging a fresh,
logged sim run before submission.

## C21 -- Table III: damping sweep, 39.8/24.9 mm/s, ~14 min landing
**Paper claim (S:S3.4.1):** joint-damping sweep and the deployed-value landing
outcome.
**Status:** no evidence independently retained in this repository backs
this table.

**Independent math check on the 35%-margin figure that accompanies this
table:** the paper's "24.9 mm/s clears a 3 m hop's 18.5 mm/s requirement
with 35% margin" appears to use the 45-degree-optimal launch formula
rather than the platform's own launch law. The correct margin under the
platform's own launch law is ~1%, not 35%. **Caveat: confirm this
correction is reflected in the submitted document (`mantis_draft_2.docx`)
before submission**, independent of whether the table itself is re-verified.

## C22 -- Restitution ~0.96 from 1.15m drop
**Paper claim (S:S3.4):** restitution ~0.96 measured from a 1.15m drop.
**Status:** no evidence independently retained in this repository backs this
specific figure. Recommend softening the paper's wording or arranging a
fresh, logged sim run before submission.

## C24 -- Auction bid 29.1 vs 40.8
**Paper claim (S:S4.3/S5.1):** bids of 29.1 vs 40.8 m-equivalent deciding a contested
target.
**Status:** no evidence independently retained in this repository backs this
specific figure. Would require a fresh full-swarm auction rerun to confirm.

## C25 -- 9-minute run, 41-anomaly backlog
**Paper claim (S:S4.3):** live-verified over a 9-minute run with a 41-anomaly
backlog.
**Status:** no evidence independently retained in this repository backs this
specific figure. Would require a fresh full-swarm rerun to confirm.

## C26 -- Dispatch/arbitration races
**Paper claim (S:S4.3):** two live-caught dispatch races (actuator arbitration).
**Status:** no evidence independently retained in this repository backs this
specific figure. Would require a fresh full-swarm rerun to confirm.

## C29 -- First-boot role allocation
**Paper claim (S:S4.3):** differentiated role allocation on first boot (paper states
RELAY + 2x SCOUT).
**Status:** no evidence independently retained in this repository backs this
specific figure. Would require a fresh full-swarm rerun to confirm.

## C30 -- Grounded actuator motion ejects a landed robot at 0.128 m/s
**Paper claim (S:S8, Law 3):** grounded actuator motion ejecting a resting robot up to
0.128 m/s, three times a nominal launch.
**Status:** no independently-retained evidence backs the specific 0.128 m/s
figure. **Related evidence does exist from this week's live reruns**: an
uncommanded post-landing liftoff of 0.164 m/s was captured incidentally
during the C9/C17 cascade (see C17/C18 below) -- a genuine, independent
observation of the same failure mode (grounded actuator motion ejecting a
landed robot), though a different magnitude and a different specific
trigger than the paper's cited instance.

## C31 -- Twelve-hour stall, 149 aborted launches, 87 self-righting attempts
**Paper claim (S:S8, Law 4):** the fleet scattered itself for twelve hours without
completing a single mission.
**Status:** no evidence independently retained in this repository backs this
specific figure. Would require a fresh full-swarm long-duration rerun to
confirm.

---

## C9 -- headline 4.3m / ~20min directional hop (abstract + S3.1 + S7)
**2026-08-03 live rerun status: real flight measured, and it CONTRADICTS
the paper's specific figures.** See
`docs/paper_assets/calculations/launch_stance_reliability_tests_20260803/README.md`
for full detail and raw telemetry (`c9_success_flight_1.24m_wrong_heading.jsonl`,
9159 samples covering a complete real flight).

- Commanded azimuth -56 deg; **measured yaw at launch -55.03 deg** --
  matches the paper's own stated "-55 deg measured vs -56 deg commanded"
  almost exactly.
- Ground displacement at contact: **1.24 m**, far short of the claimed
  4.3m (commanded distance was 3.0m, not necessarily the same commanded
  value behind the paper's original number, so this isn't strictly
  apples-to-apples on distance).
- **Achieved ground-travel azimuth at contact: 122.66 deg** -- does not
  match the held yaw heading (-55 deg) at all. This is the more
  significant discrepancy, independent of the distance mismatch: the body
  pointed one way, the robot travelled a very different way.
- Yaw spiked to +135 deg at least once during otherwise-stable flight --
  an unexplained mid-flight disturbance.
- Flight time to contact: 6.1 min (shorter than the paper's ~20 min,
  consistent with the smaller commanded distance).
- The landing was not clean: several genuine false "not actually landed"
  resets occurred (see the C28 note below), then it settled badly tilted
  (u_z=0.07) and had to self-right -- see C17/C18 below for what happened
  next.

This is the highest-exposure claim in the paper (appears in the abstract).
Given a real test now directly contradicts both the displacement and
heading figures, this needs attention before submission, not just a
hedge.

## C15, C16 -- pre-redesign self-righting baseline (5 of 21, 24%)
**2026-08-04 live rerun status: real counter-evidence, contradicting the
paper's specific figure.** See
`docs/paper_assets/calculations/pre_redesign_self_righting_baseline_20260804/README.md`
for full detail and raw telemetry/console logs.

Ran the actual historical pre-redesign controller (git commit `5c9e278`,
temporarily swapped in, tested in isolation, then restored to the current
shipped version and rebuilt) through 21 trials at randomized tilts
(20-180 deg), matching the paper's own framing that the original baseline
was measured "over a long run" rather than a controlled bucketed
experiment.

**Result: 1 of 21 recovered (4.8%)** -- roughly 5x lower than the paper's
claimed 5/21 (24%). The one recovery was the mildest tilt in the sample
(32.2 deg); every trial at a moderate-to-severe tilt (77-172 deg) that
landed failed to right itself, several stalling almost exactly where they
landed -- consistent with the known "stall-on-side" failure mode the
later redesign (`958ed0a`) was specifically written to fix.

An unplanned secondary finding: 7 of 21 trials (33%) never registered
`landed=True` within a 200s window at all, despite the robot visibly
settling still -- disproportionately the *milder* tilts that settled
gently near-upright, the opposite of what reliable landing detection
should prioritize. This mirrors the false-positive landing-trigger pattern
noted for C28 below (same detector, opposite failure direction), and two
trials (8, 16) independently reproduced the same uncommanded
"liftoff-detected-while-LANDED" re-arm documented for the C17/C18 cascade.
Sample size (n=21) is small and the tilt distribution isn't guaranteed to
match whatever produced the original figure, so this should be read as a
real, substantial discrepancy rather than a precise replacement value.

## C17, C18 -- post-redesign self-righting statistics
**2026-08-03: real counter-evidence found, though not from a controlled
sample.** The purpose-built batch test (six iterations) never produced a
valid measurement -- see
`docs/paper_assets/calculations/self_righting_reliability_test_20260803/README.md`.
But a genuine, organic self-righting failure was captured live during the
C9 rerun above: the hop landed at a **moderate ~45.6 deg tilt** (u_z=0.70)
-- squarely within the range C17 claims recovers "reliably... every such
case in the sample" -- and failed all 5 righting attempts, u_z barely
moving across any of them. The exact failure message
("Self-righting failed after 5 attempts... Robot may still be physically
inverted") matches, verbatim, an unrelated incidental failure found
earlier in the same session at a different tilt (~39.4 deg, during the
C12 attempt in `attitude_rerun_20260803/`). Two independent real
occurrences of the identical failure mode, both at moderate tilts, is a
real pattern worth taking seriously against a claim of 100% recovery in
this range -- though neither is from a large controlled sample, so this
doesn't establish what the *actual* success rate is, only that it isn't
the clean 100% the paper states.

As an unplanned coda: after the failed righting, the robot was marked
LANDED anyway (the code's own documented fallback), then an uncommanded
liftoff (0.164 m/s, closely matching the paper's own cited Law-3 magnitude
of 0.128 m/s) kicked it from ~45 deg all the way to a near-total **165 deg
inversion** -- worse than where it started. Full sequence and raw
telemetry in the self-righting README above.

## C27 -- sampling cycle: arrival check, drill/dwell/stow, chain to next anomaly
**2026-08-04 live rerun status: CONFIRMED.** Full 3-agent swarm, live,
organic anomaly detection (no seeding). See
`docs/paper_assets/calculations/c27_sampling_cycle_20260804/README.md` for
full detail and raw console log (`swarm_manager_console_full_run.log`).

8 real arrive -> drill -> stow events captured (arrival distances 0.6-3.7 m,
all within the 4.0 m arrival radius -- a richer spread than the paper's
single cited "0.9 m" instance), with multiple real chain-to-next-anomaly
events and correct carousel-full gating. The complete sequence the paper
describes executes for real.

As a bonus from the same run: a post-hoc routing analysis comparing the
swarm's real nearest-neighbor greedy dispatch order against the true
optimal ordering of the same real visited targets found the greedy
routing was **18.6% longer** than optimal for that sequence -- a genuine,
real-data answer (not a synthetic benchmark) to how much the documented
greedy-routing simplification costs in practice. See
`routing_analysis.json` in the same directory.

## Still unconfirmed

**C5, C23 (exact figure)** -- no source note or archived log currently backs
these specific figures, and no live-test attempt was made. If asked for backing,
the honest answer is that none is currently available -- either soften the
paper's wording or arrange a fresh, logged sim run.

**C28** ("zero false landing triggers across the final verification runs") --
not confirmed, and there is real counter-evidence worth knowing about: multiple
genuine false landing-trigger events ("Sustained contact accel but velocity
still X m/s -- not actually landed... likely a false accel trigger") were
observed in *current* code during this week's live-rerun testing (see the node
console logs under `docs/paper_assets/calculations/attitude_rerun_20260803/`
and `launch_stance_reliability_tests_20260803/`). This doesn't disprove the
specific historical run the paper cites (a one-off run this testing wasn't
part of), but "zero... final verification runs" is a real risk if a reviewer
presses on it. Recommend softening the "zero" framing.
