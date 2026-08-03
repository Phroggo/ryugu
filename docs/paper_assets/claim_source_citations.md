# Source citations for paper claims (C1-C31 numbering per the 2026-08-03 evidence audit)

Every quote below was originally cited from `docs/Study_Guide.md`,
`docs/research_report.md`, `docs/HANDOFF.md`, `docs/walkthrough.md`, or
`docs/task.md` -- all removed from the working tree in commit
`ecfafcec4587e7189abe4021689961ba136034a5` ("Repo cleanup for submission")
because they were internal development logs, not submission material.

**Removing those files from HEAD did not destroy their content.** `git rm`
only removes a file from the current snapshot; the full text is still
retrievable from any earlier commit. The quotes below were pulled directly
from the last commit before deletion:

```
git show fb18b2efa5544417c0b076557c6a42252ea2c61c:docs/Study_Guide.md
git show fb18b2efa5544417c0b076557c6a42252ea2c61c:docs/research_report.md
git show fb18b2efa5544417c0b076557c6a42252ea2c61c:docs/HANDOFF.md
git show fb18b2efa5544417c0b076557c6a42252ea2c61c:docs/walkthrough.md
git show fb18b2efa5544417c0b076557c6a42252ea2c61c:docs/task.md
```

This is what to hand a reviewer who asks for backing on any of the claims
below: the exact sentence, which file it came from, and the git command to
pull the original file back out of history if they want to see it in
context. If the working commit hashes above ever change again (further
history rewrites, rebasing, a fresh clone), re-run
`git log --diff-filter=D --format="%H" -- docs/Study_Guide.md` to find the
current deletion commit and use its parent instead.

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

**Also, more durable:** this exact history is *also* recorded as a code
comment in `ryugu_sim/attitude_controller.py:230-236` (current file, not
removed -- see that file directly, no history dig needed).

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

**2026-08-03 live rerun status: DID NOT REPRODUCE, in three independent
attempts.** See
`docs/paper_assets/calculations/attitude_rerun_20260803/README.md` for
full detail:
- Tumble injected while airborne: no recovery in 30s (frozen at ~165°).
- Tumble injected near ground: got stuck at 93° after a teleport-induced
  velocity artifact.
- Tumble baked into a fresh spawn: robot never moved at all (DART physics
  sleep, no anti-sleep mechanism while airborne -- see
  `attitude_controller.py`'s own "SLEEP-DEFEAT ROTOR" comment, ~line 483).

None of the three failure modes prove the original dev-log claim is false
-- each hit a different artifact of *how* the tumble was injected in this
rerun, not necessarily a property of the controller under the conditions
the original measurement used (e.g. a tumble arising from genuine
in-progress flight dynamics, where existing motion would prevent the DART
sleep issue, was not tested). But the claim should currently be treated as
**unconfirmed pending a cleaner test**, not as independently reproduced.
**This is flagged for your decision on whether to soften S3.2's wording --
see the "C14 finding" discussion earlier in this session.**

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

**Also the source of the 35%-margin bug**, line 730: "24.9 mm/s clears a
3 m hop's 18.5 mm/s requirement with 35% margin" -- this used the
45-degree-optimal launch formula rather than the platform's own launch
law; corrected in `Research_Paper.md` to ~1% margin (commit `fc7f7a9`,
current message: "Fix launch-margin figure"). **Caveat: verify this
correction actually made it into `mantis_draft_2.docx` -- the fix was only
applied to the now-removed markdown source, and the docx is
hand-edited/frozen separately from that source. This has not yet been
checked.**

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

## Still genuinely unconfirmed (searched for, not found anywhere, including in the
now-recovered source text above)

C5, C9, C15, C16, C17, C18, C23 (exact figure), C27, C28. No citation exists for these
in any file that was ever in this repo, recovered or not. If a reviewer asks for
backing on these specifically, the honest answer is that none is currently
available -- either soften the paper's wording or arrange a fresh, logged sim run.
