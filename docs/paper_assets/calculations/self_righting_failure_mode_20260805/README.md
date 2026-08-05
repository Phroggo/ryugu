# What happens when self-righting fails, 2026-08-05

The recovery-rate figures in
`../pre_redesign_self_righting_baseline_20260804/`,
`../post_redesign_self_righting_baseline_20260805/`, and
`../severe_tilt_no_respawn_rerun_20260805/` report whether a trial
recovered. This documents what actually happens on the trials that don't
-- a real, evidenced failure cascade, not a hypothetical, found by reading
what the code itself does on give-up and cross-checking it against the
batch data already collected this week.

## The designed fallback, and what it actually leads to

`landing_controller.py`'s righting sequence gives up after
`MAX_RIGHTING_ATTEMPTS` (5) failed attempts and marks the robot `LANDED`
anyway, explicitly so downstream logic (SAMPLER dispatch, etc.) doesn't
hang forever:

```
❌ Self-righting failed after 5 attempts — giving up, marking LANDED
anyway so downstream logic (e.g. SAMPLER dispatch) does not hang
forever. Robot may still be physically inverted.
```

This is a deliberate, documented design choice, not a bug on its own.
But it has a real, observed consequence: **the robot is now marked
LANDED while still genuinely tilted, and the separate liftoff watchdog
doesn't know the difference.** In the one trial across all three
2026-08-04/05 batches (50 trials total) where the give-up path actually
fired -- `pre_redesign_trial21_giveup_to_liftoff_cascade.log` -- the
sequence was:

```
[t+0.0s]  ❌ Self-righting failed after 5 attempts — giving up, marking
          LANDED anyway... (u_z=0.48, ~61 deg from upright -- a
          MODERATE tilt, not even a severe one)
[t+2.0s]  ⚠️ Liftoff detected while LANDED (v=0.030 m/s sustained) →
          back to FLIGHT
[t+2.0s onward] state: FLIGHT, held for the rest of the 70+s observed
          window with no further correction of any kind.
```

An uncommanded liftoff, immediately after the give-up, that leaves the
robot drifting in FLIGHT indefinitely -- no further righting attempt, no
landing re-confirmation, nothing, for as long as this test observed it.

**This is not a one-off.** It is the second independent, real occurrence
of the exact same cascade this week: the incidental capture during the
C9 rerun (`../self_righting_reliability_test_20260803/README.md`,
"Incidental failure #2") shows the identical sequence -- give up after 5
attempts, marked LANDED, uncommanded liftoff (0.164 m/s there vs 0.030
m/s here, both real, both sustained), then no recovery, ending at a
**worse** inversion (165 deg) than where the failure started (~45 deg).
Two independent, real instances of "give up -> mark landed -> uncommanded
liftoff -> no further correction" is a genuine pattern, not noise.

## A second, structural gap: the safety net can't see the oscillation failure mode

Separately, `../post_redesign_self_righting_baseline_20260805/` found a
different failure pattern in several trials (most clearly trial 14):
the body successfully crosses the u_z>0.9 success threshold, gets marked
`LANDED`, then drifts back down into "badly tilted" territory within a
few seconds and gets re-triggered -- repeating this cycle multiple times
without ever holding a stable recovery.

Reading the code: `self.righting_attempt` resets to 0 every time a
righting excursion begins (both on the initial "settled badly tilted"
trigger and on the "tilted while LANDED" re-trigger). A brief crossing of
u_z>0.9 counts as success and exits the righting state entirely --
it does not require the body to *hold* that orientation. This means the
5-attempt give-up fallback **cannot fire** for a robot stuck in this
succeed-then-immediately-redrift oscillation, because each brief success
resets the counter before it can accumulate toward 5. A robot in this
specific failure mode has no documented fallback at all in the code: it
just keeps cycling.

## Why corrections don't hold: the code's own comment gives the answer

`landing_controller.py` explains its own two-phase leg strategy (tuck
while correcting, deploy once near upright) this way: *"extending the
legs gives it the stable upright equilibrium (feet down) to fall into."*
That design explicitly depends on the feet actually reaching the ground
once deployed. None of this week's synthetic spawn-based self-righting
tests (pre-redesign, post-redesign, or the severe-tilt rerun) ever
involve real ground contact -- they spawn/hold the robot at z=5.2-6.0 m,
well above the ~4.8 m local terrain, with no leg/hop controller running
to bring it down. The "stable equilibrium to fall into" the code counts
on literally isn't present in these tests: the legs deploy into open air.

This is a plausible, code-substantiated explanation for why corrections
across all three batches tend not to *hold* even when they briefly
succeed -- and it means the current recovery-rate figures likely
understate real-world reliability (a genuine post-hop landing, with real
foot-terrain contact, gives the controller the physical foundation its
own design assumes) while simultaneously the give-up/liftoff cascade
above is a real, independent risk that doesn't depend on this test
artifact at all -- it was first caught during an organic, real-landing
test (the C9 rerun), not a synthetic one.

## Status

**Real, repeatable failure cascade documented**: give-up after 5 attempts
-> marked LANDED while still tilted -> uncommanded liftoff within
seconds -> no further correction observed. Two independent occurrences
this week, one from a synthetic batch test, one from an organic real
landing -- this is not an artifact of either test method specifically.
Separately, a structural gap means the give-up safety net cannot catch
robots stuck in a succeed-then-redrift oscillation. Both are worth
stating plainly in the paper alongside the recovery-rate numbers: a
low-probability-but-real failure mode exists where an unresolved
self-righting attempt leaves the robot silently drifting with no active
control and no further recovery attempt, which is a materially different
(and more serious) risk than "recovery sometimes takes longer than
X seconds."
