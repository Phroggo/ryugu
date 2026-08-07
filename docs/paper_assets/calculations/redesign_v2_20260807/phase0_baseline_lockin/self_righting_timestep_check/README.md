# Self-righting maneuver timestep sensitivity: 60deg tilt

Counterpart to the already-committed C13 yaw-slew timestep spot check
(`../../timestep_sensitivity_20260805/`), which only exercised pure
reaction-wheel torque control with no ground contact during the maneuver.
This exercises the self-righting maneuver instead (contact + friction +
fast leg motion), against the **pre-mass-redesign** controller state
(commit `4cc7de9`, tag `pre-mass-redesign`).

Method: spawn `scout_1` at a commanded 60deg tilt (azimuth 30deg), current
(pre-redesign) `landing_controller` only, u_z traced at 1Hz for up to 180s.
One run per timestep (1ms shipped, 4ms 4x coarser).

## Result (`righting_timestep_results.json`, `righting_stdout_SUCCESSFUL_run.log`)

| Timestep | Time to u_z>0.9 | Final u_z (t=180s) |
|---|---|---|
| 1ms | 111.8s | 0.99942 |
| 4ms | 106.4s | 0.99941 |

Both converged to essentially identical final orientation. Convergence
time differs by ~5s (~5%, 4ms faster) -- same direction and similar
magnitude to the C13 yaw-slew spot check (9.61s vs 8.70s, ~9%, 4ms also
faster there).

**Caveat:** this is a single run per timestep, not a distribution. The
sibling contact-launch check in this same phase (`../contact_launch_timestep_check/`)
found >30x run-to-run variance at fixed timestep for a different contact-
dynamics scenario. Self-righting also involves continuous ground contact,
so this single pair cannot rule out the same kind of noise swamping a real
timestep effect. Treat as "no red flag," not "confirmed timestep-invariant."
A same-treatment 5-repeat x 2-timestep batch would be needed to cite this
at the same strength as the C13 result.

## Infra note (see `righting_stdout_FAILED_missing_resource_path.log`)

First two attempts at this run failed silently/loudly before producing any
real data:
1. The harness script initially didn't set `GZ_SIM_RESOURCE_PATH`, so `gz
   sim` failed to load the world (`Unable to find uri[model://skydome]`)
   and both "runs" produced `uz=None` throughout. Fixed by setting the env
   var explicitly in the script (matching the other timestep-check
   harnesses in this repo).
2. After that fix, reruns still silently produced the exact same stale
   (pre-fix) output file, with the invoking shell exiting 1 with zero
   output before the Python script ever ran. Root cause: a leading `pkill
   -9 -f "..."` with nothing to match returns exit 1, and the shell used to
   invoke these background runs aborts on any nonzero exit without
   printing anything (stderr had been redirected to `/dev/null`). Worked
   around by routing through `run_it_v2.sh` (checked in here) which does
   not rely on a bare `pkill` as its first statement executing cleanly.

`righting_stdout_FAILED_missing_resource_path.log` and `righting_timestep_check.py`
(the corrected, currently-checked-in version, with `GZ_SIM_RESOURCE_PATH`
set at import time) are both included for anyone re-running this to avoid
repeating the same dead end.
