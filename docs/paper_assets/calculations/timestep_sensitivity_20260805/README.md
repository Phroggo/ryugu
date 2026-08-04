# Physics-timestep sensitivity check, 2026-08-05

New question this week: at the force/velocity scales this platform
actually operates at (10^-4 - 10^-2 N, mm/s), has anyone checked whether
results are timestep-invariant? Nobody had. This checks it directly by
rerunning the cleanest, most precisely-measured live-rerun result from
this week -- the C13 107 deg yaw-slew convergence
(`../attitude_rerun_20260803/README.md`) -- at two different physics
timesteps and comparing.

## Method

The shipped world file (`worlds/ryugu.sdf`) uses `<max_step_size>0.001</max_step_size>`
(1ms). Created an unmodified copy with only that one value changed to
0.004 (4ms, a 4x coarser step) and reran the identical test against each:
spawn `scout_1` upright, run `attitude_controller` alone (yaw-hold is
"always active, including grounded" per its own code comment -- no need
for the full hop/landing stack), command `target_yaw=107 deg`, and record
time-to-converge (<1 deg error) and final yaw.

## Result

| Timestep | Converged at | Final yaw |
|---|---|---|
| 1ms (shipped) | 9.61 s | 106.06 deg |
| 4ms (4x coarser) | 8.70 s | 106.15 deg |

Convergence time differs by under a second; final yaw differs by 0.09
deg. Both are consistent with each other and with the original C13
live-rerun figure (106.03 deg, <1 deg by t+9.3s) documented earlier this
week -- three independent measurements of the same phenomenon, in close
agreement.

Full data: `timestep_results.json` (includes the full yaw trace for both
runs, not just the summary numbers). Harness: `timestep_sensitivity.py`.

## Status

**Timestep-invariant, at least for this test.** No evidence that this
week's live-rerun conclusions are an artifact of the specific 1ms
timestep used throughout. This checked one representative case (a yaw
slew driven by reaction-wheel torque, in the same force/torque regime as
the rest of the platform's control authority); it wasn't run against
every claim in this week's evidence set, so treat this as a spot-check
supporting methodology validity generally, not an exhaustive
per-claim verification.
