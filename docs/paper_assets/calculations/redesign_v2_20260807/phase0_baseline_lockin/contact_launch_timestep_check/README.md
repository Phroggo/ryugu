# Contact-dynamics timestep sensitivity: 9.0m degraded-mode launch

Two runs, done together, both against the **pre-mass-redesign** controller state
(commit `4cc7de9`, tag `pre-mass-redesign`):

## 1. Single-pair check (`contact_timestep_check.py` / `contact_timestep_results.json`)

1ms (shipped `worlds/ryugu.sdf`) vs 4ms (4x coarser, `ryugu_4ms.sdf`, identical
world otherwise), one run each, same 9.0m commanded-hop scenario as the
V_GAIN calibration's confirmed degraded case (original figure: ratio=0.209).

| Timestep | Delivered ratio | Stabilize time |
|---|---|---|
| 1ms | 0.286 | 18.0s |
| 4ms | 0.292 | 18.1s |

~2% apart, but neither reproduces the original 0.209 -- prompted the 5-repeat
distribution below.

## 2. 5-repeat x 2-timestep distribution (`contact_timestep_distribution.py` / `distribution_results.json` / `distribution_stdout.log`)

Same scenario, 5 fresh-respawn repeats per timestep (10 runs total).

| Timestep | n stabilized (of 5) | ratios | mean | range |
|---|---|---|---|---|
| 1ms | 4 (1 timeout) | 0.009, 0.301, 0.013, 0.210 | 0.133 | 0.009-0.301 |
| 4ms | 3 (2 timeout) | 0.324, 0.447, 0.205 | 0.325 | 0.205-0.447 |

**Finding: run-to-run variance at fixed timestep (>30x spread, 1ms) swamps
any timestep effect, and 3/10 runs (30%) never separated at all within the
120s timeout.** The two runs that happened to start from a genuine
`landed=True` state (not just under the 0.02 m/s ready-gate) landed at
nearly identical ratios (0.210 @ 1ms, 0.205 @ 4ms) -- suggesting pre-launch
crouch/stance quality, not physics timestep, is the dominant confound.
**The single 0.209 figure in the original vgain calibration should not be
treated as representative** -- it is one draw from a very wide distribution.

Full per-node console logs (`attitude_scout_1_*`, `bridge_scout_1_*`,
`landing_scout_1_*`, `loco_scout_1_*`, `gz_*.log`) included for every run.
