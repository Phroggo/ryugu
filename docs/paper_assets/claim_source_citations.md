# Source citations for paper claims

This records the source passage behind each quantitative claim in the
paper that draws on project development records (engineering notes and
study materials kept during development, alongside the simulation source
and the raw telemetry/logs archived under `docs/paper_assets/`). Each
entry below gives the claim as it appears in the paper, the originating
note, and the exact wording measured or observed at the time.

---

## C1 -- 27% stroke / 0.19 m/s / 76 m
**Paper claim (S:S3.1):** a hop commanded at 27% stroke separated at 0.19 m/s and flew
over 76 m.
**Source:** `docs/Study_Guide.md`, line 326-327:
> a hop commanded at 27% stroke (intended: 9 metres) actually separated at
> 0.19 m/s and flew over 76 metres.

## C7 -- 0.5 rad lean test, 0.85 to 0.38 in 3.5s
**Paper claim (S:S3.1):** uprightness collapsing from 0.85 to 0.38 within a single 3.5s
ramp.
**Source:** `docs/Study_Guide.md`, line 940:
> found it could collapse from 0.85 to 0.38 within a single 3.5-second ramp

## C8 -- 9m hop, 695s flight, 0.16m horizontal
**Paper claim (S:S3.1):** a commanded 9m hop measured flying ~695s almost vertically,
0.16m horizontal drift.
**Source:** `docs/Study_Guide.md`, line 474-475:
> a commanded 9-metre hop flew for over 11 minutes almost perfectly straight
> up, travelling only 0.16 metres horizontally

## C10 -- Residual spin was a control-structure bug (pre-redesign)
**Paper claim (S:S3.2):** residual spin and steady-state error, measured before the
redesign.
**Source:** `docs/walkthrough.md`, line 320-327:
> The RW controller commanded wheel *velocity* proportional to attitude
> error. A wheel only torques the body while *accelerating* -- caught
> red-handed with the robot at rest holding a 0.42 rad yaw error forever...
> In flight the same structure leaves a residual spin ω = L₀/(I_bot +
> I_w·K_d) -- precisely the -1 to -2.3 rad/s that never converged.

**Also:** `docs/HANDOFF.md`, line 232 (same bug, framed as the fix log entry):
> the earlier velocity-proportional law could not null steady-state error
> at all

## C11 -- Rate-deadband experiment produced a +/-1.2 deg limit cycle
**Paper claim (S:S3.2):** a rate deadband was tried and produced a measurable +/-1.2°
limit cycle at exactly the deadband rate.
**Source:** `docs/research_report.md`, line 303-304:
> No rate deadband -- a first attempt deadbanded rate at 0.005 rad/s;
> telemetry then showed the body coasting at *exactly* 0.005 rad/s in a
> slow ±1.2° limit cycle

**Also documented directly in source:** the same history is recorded as a
code comment in `ryugu_sim/attitude_controller.py:230-236`, explaining why
the shipped controller deadbands angle only, not rate.

## C12 -- In-flight body rates 0.005-0.015 rad/s, launch transient 0.24 rad/s
**Paper claim (S:S3.2):** in-flight rates damped to 0.005-0.015 rad/s; launch
transients of 0.24 rad/s.
**Source:** `docs/HANDOFF.md`, line 690-692:
> Flight tumble -- ✅ MEASURED AND PASSED (2026-07-16 mission watch).
> In-flight rates 0.005-0.015 rad/s (essentially still), launch transients
> (0.24 rad/s) decay within seconds, no persistent yaw.

**2026-08-03 live rerun status:** attempted, inconclusive -- see
`docs/paper_assets/calculations/attitude_rerun_20260803/README.md`
(`c12_liftoff_attempt_raw_telemetry.jsonl`). The commanded jump aborted
mid-crouch; a stray liftoff was captured instead of a clean launch.

## C13 -- 107 deg yaw slew, overdamped, held within 1 deg
**Paper claim (S:S3.2):** 107 degree yaw slew converging overdamped, held within 1
degree at zero rate.
**Source, independently repeated in four files** (all citing commit `cb470b7`):
- `docs/walkthrough.md`, line 329-330: "Live: a 107° yaw slew converged and
  held within 1° at zero rate; a 165° tumble damped to 3.6° in ~20s."
- `docs/HANDOFF.md`, line 39, 562-563: "live-verified fixed: 107° yaw slew
  converges and holds at zero rate... 165° tumble damped to 3.6° in ~20 s,
  no oscillation, ζ≈1.1-1.6 overdamped by design"
- `docs/task.md`, line 130-131: "Live-verified: 107° yaw slew converges +
  holds at zero rate; 165° tumble damped to 3.6° in ~20s. (`cb470b7`)"

**2026-08-03 live rerun status: CONFIRMED independently.** See
`docs/paper_assets/calculations/attitude_rerun_20260803/c13_yaw_slew_raw_telemetry.jsonl`
-- fresh telemetry, reconstructed yaw from the odometry quaternion (not
just a repeat of the dev-log claim), converges to 106.03°, <1° of target by
t+9.3s. This is the strongest-evidenced claim in the whole set: both a
contemporaneous dev-log citation AND independently reproduced raw
telemetry from a live rerun.

## C14 -- 165 deg tumble damped to 3.6 deg in ~20s
**Paper claim (S:S3.2):** same source citations as C13 above (both numbers appear in
the same sentences, tied to commit `cb470b7`).

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
**Source:** `docs/Study_Guide.md`, line 847:
> Root cause: the fold command was issued as an instant step, and by that
> point in the [recovery sequence]...

## C20 -- Table II: 32/38 mm/s, 16/22 mm/s, 0.7-0.9m kicks
**Paper claim (S:S3.4):** contact-damping scheme comparison table.
**Source:** `docs/research_report.md`, line 605-611 (full table):
> | Step to soft posture at contact instant | -- | -- | 0.7-0.9 m kicks,
> non-decaying pogo |
> | Same posture ramped over 2 s | 32 | 38 | pogo to 10+ m |
> | Zero-stiffness catch: mirror measured joint angles back as targets |
> 16 | 22 | worst -- feedback lag pumps the rebound |

## C21 -- Table III: damping sweep, 39.8/24.9 mm/s, ~14 min landing
**Paper claim (S:S3.4.1):** joint-damping sweep and the deployed-value landing
outcome.
**Source:** `docs/research_report.md`, line 630-636 and 724-730 (two tables,
same sweep):
> | 0.005 | 39.8 mm/s (multi-meter hops) | restitution ≈0.96, indefinite
> pogo |
> | 0.15 (deployed) | few mm/s (~25 cm ascents) | ζ≈0.45, settles in 2-3
> bounces |
>
> | **0.05 (deployed)** | **24.9 mm/s (apex +2.9 m)** | **settles, LANDED
> confirmed in ~14 min** |

**Also the source of the 35%-margin figure**, line 730: "24.9 mm/s clears a
3 m hop's 18.5 mm/s requirement with 35% margin" -- this used the
45-degree-optimal launch formula rather than the platform's own launch
law. The correct margin under the platform's own launch law is ~1%, not
35%. **Caveat: confirm this correction is reflected in the submitted
document (`mantis_draft_2.docx`) before submission.**

## C22 -- Restitution ~0.96 from 1.15m drop
**Paper claim (S:S3.4):** restitution ~0.96 measured from a 1.15m drop.
**Source:** `docs/research_report.md`, line 602-604:
> measured restitution ≈0.96 from a 1.15 m drop (bounce apexes 5.88 → 5.76
> m -- no meaningful decay)

## C24 -- Auction bid 29.1 vs 40.8
**Paper claim (S:S4.3/S5.1):** bids of 29.1 vs 40.8 m-equivalent deciding a contested
target.
**Source:** `docs/HANDOFF.md`, line 701-702:
> Full swarm mission cycle -- auction/dispatch/re-hop ✅ VERIFIED live
> (2026-07-16: competitive bids "scout_2=29.1, scout_3=40.8 → winner
> scout_2", dispatch, cooldown-paced corrective re-hops).

## C25 -- 9-minute run, 41-anomaly backlog
**Paper claim (S:S4.3):** live-verified over a 9-minute run with a 41-anomaly
backlog.
**Source:** `docs/Study_Guide.md`, line 755-756:
> A 9-minute run produced a 41-anomaly backlog, because finding something
> is a cheap, instant per-tick check, while [visiting it is not]

**Caveat:** the specific coordinate detail ("[-10.0, 0.0], 10.0 m away")
cited in the paper was not independently located in this same passage --
likely present nearby but not verified word-for-word.

## C26 -- Dispatch/arbitration races
**Paper claim (S:S4.3):** two live-caught dispatch races (actuator arbitration).
**Source, recurring theme across multiple files, e.g.** `docs/research_report.md`,
line 587, 707, 709 and `docs/HANDOFF.md`, line 31:
> a last-write-wins race can never be [triggered by the fixed design]...
> multiple publishers on one wheel topic is a silent last-write-wins fight

## C29 -- First-boot role allocation
**Paper claim (S:S4.3):** differentiated role allocation on first boot (paper states
RELAY + 2x SCOUT).
**Source:** `docs/HANDOFF.md`, line 736-739:
> Multi-agent scaling -- ✅ DONE (2026-07-16, `586e239`, pushed). All three
> scouts spawn... the swarm manager ran its first genuinely multi-agent
> allocation on first boot (scout_1 RELAY, scout_2/3 SAMPLER en route to
> anomalies; dashboard all-ONLINE).

**Caveat/nuance:** this specific logged instance shows RELAY + 2x SAMPLER,
not RELAY + 2x SCOUT as the paper states -- plausible (SAMPLER is what a
SCOUT becomes after winning an auction, and anomalies may have already
been available at this particular boot), but it is a different specific
instance than what the paper describes. Worth a look if a reviewer presses
on this one specifically.

## C30 -- Grounded actuator motion ejects a landed robot at 0.128 m/s
**Paper claim (S:S8, Law 3):** grounded actuator motion ejecting a resting robot up to
0.128 m/s, three times a nominal launch.
**Source:** `docs/research_report.md`, line 678-681:
> once the bridge fix made legs obey, the 15 s post-landing stand-fold ramp
> ejected a freshly-landed robot at 0.128 m/s -- 3× a full jump stroke, a
> ~70 m ballistic arc.

## C31 -- Twelve-hour stall, 149 aborted launches, 87 self-righting attempts
**Paper claim (S:S8, Law 4):** the fleet scattered itself for twelve hours without
completing a single mission.
**Source:** `docs/Study_Guide.md`, line 445:
> The fleet spent twelve hours in this loop: 149 aborted launch attempts,
> 87 self-righting attempts, and zero completed sampling missions.

The paper currently understates this citation's own precision -- it could
cite the exact attempt counts (149, 87) rather than just "twelve hours."

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
Not attempted. Reproducing this would require reverting to the
pre-redesign self-righting code (available in git history) and running
~21 trials -- a larger undertaking than the other live-rerun work done so
far. No source note or archived log currently backs these specific
figures independent of the dev-log citation already in this document.

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

## Still unconfirmed

**C5, C23 (exact figure), C27** -- no source note or archived log currently backs
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
