# Phase 0 — Baseline Lock-in — Change Report

Repo: `ryugu_v2_ws/src/ryugu_sim` (git). Phase objective: freeze a named,
diffable baseline before any redesign work begins. No controller logic,
gains, or model geometry were changed in this phase — this is a
snapshot-and-consolidate phase only.

## 1. Files touched

**Commit `4cc7de98b2b7c13028694e53c3a446ca1489b586`** (tag `pre-mass-redesign`):
- `ryugu_sim/attitude_controller.py` (modified)
- `ryugu_sim/hopper_locomotion.py` (modified)
- `ryugu_sim/landing_controller.py` (modified)

**Commit `40eb920f03c2dd40f4692bbd9ff4a2af2b626826`**:
- `docs/paper_assets/calculations/redesign_v2_20260807/phase0_baseline_lockin/BASELINE_MANIFEST.md` (new)
- `docs/paper_assets/calculations/redesign_v2_20260807/phase0_baseline_lockin/contact_launch_timestep_check/` — 51 files (new): `README.md`, `contact_timestep_check.py`, `contact_timestep_distribution.py`, `contact_timestep_results.json`, `distribution_results.json`, `distribution_stdout.log`, `contact_timestep_stdout.log`, `ryugu_4ms.sdf`, plus full per-node console logs (`attitude_scout_1_*`, `bridge_scout_1_*`, `landing_scout_1_*`, `loco_scout_1_*`, `gz_*.log`) for both the single-pair run and all 10 distribution-batch runs.
- `docs/paper_assets/calculations/redesign_v2_20260807/phase0_baseline_lockin/self_righting_timestep_check/` — 10 files (new): `README.md`, `righting_timestep_check.py`, `run_it_v2.sh`, `righting_timestep_results.json`, `righting_stdout_SUCCESSFUL_run.log`, `righting_stdout_FAILED_missing_resource_path.log`, plus per-node console/gz logs.

No files were deleted. `model.sdf` was **not** touched — it had no
pending changes at the time of this snapshot (last real edit was commit
`63f73b8`, prior to this session).

## 2. What changed in each file

### `ryugu_sim/attitude_controller.py`
`landed_callback`: the yaw-target bootstrap (adopt current heading as
`target_yaw`, clear `in_flight`/`commanded_flight`) is now unconditional
on `msg.data` (i.e. `landed=True`), where it was previously gated on
`self.in_flight`. The gated version left `target_yaw` at its stale init
value forever for any `landed=True` reached without a preceding
`jump_callback` (teleport-based tests, or a controller that starts already
grounded) — a stale-target position-PD is non-dissipative and was
implicated in a persistent post-give-up tilt tumble. No gain values
(`K_ang=0.05`, `K_rate=0.066`) were changed.

### `ryugu_sim/hopper_locomotion.py`
- New `righting_active` subscriber + `_righting_active_callback`: aborts
  leg motion to `IDLE` and rejects new jump commands while
  `landing_controller` reports a righting maneuver in progress (guards
  against last-write-wins contention on shared leg joint topics).
- `jump_target_callback`: now also rejects new jumps while
  `self.righting_active` is true (previously only checked `state != IDLE`).
- `tick()`, IDLE branch: also returns early while `righting_active`.
- CROUCH-state keep-awake (`_wake_model()` call at `state_timer % 20 == 0`):
  now additionally gated on `last_speed < 0.001`, matching the existing
  gate already used in the LAUNCH state. Previously ungated; `_wake_model`'s
  `set_pose` zeroes body velocity as a side effect, which was destroying
  the real ~0.03 m/s stand-up velocity CROUCH builds.

### `ryugu_sim/landing_controller.py`
Largest diff (+533/-65 net over the whole file). Summary of the distinct
fixes folded into this snapshot:
- Division-by-zero fix in the stabilization check.
- `righting_attempt` / `righting_ticks` counters now correctly reset on
  retry/entry (previously could carry stale state across attempts).
- Reaction-wheel righting law rewritten: was a proportional wheel-speed
  *lookup* from current u_z error (stalls whenever the wheel catches a low
  target speed before u_z reaches the 0.9 success threshold — measured
  live stalling at u_z≈0.82, ω→0, wheel at only ~29/160 rad/s); now an
  acceleration-integrated taper (`max_wheel_accel=50 rad/s²`,
  `RIGHTING_ACCEL_TAPER=0.6`) that never lets `cmd_vel` go negative,
  `RIGHTING_RATE_DAMP_SCALE=0.8` / `RIGHTING_RATE_DAMP_FLOOR=0.25` (tightened
  from 1.5/0.4 after live telemetry showed ω climbing to 1.7–1.8 rad/s).
- Give-up → uncommanded-liftoff cascade fixed with a persistent flag that
  preserves the LANDED tilt watchdog across the give-up transition.
- Instant-zero-kick on attempt timeout replaced with a ramped brake.
- New LANDED-state tilt-axis rate damper (x/y axes only, by design — this
  file has no z-wheel).
- **Not fixed, carried forward as-is:** `GENTLE_RIGHTING_SPEED` is at 20.0,
  a temporary diagnostic value (raised from 8.0 on 2026-08-05 to test a
  stall hypothesis) that was never reverted to a deliberately-tuned value.
- **Not fixed:** the 3-axis rigid-body coupling gap described in the
  manifest — a diagnostic run (attempts 5→10, timeout 15s→30s) ruled out
  a time/attempt-budget shortfall and instead found a third failure mode
  (slow precession deeper into inversion over ~170s); diagnostic parameter
  values for that test were reverted, this underlying gap was not.

Full diff: `git show 4cc7de9 -- ryugu_sim/landing_controller.py` (not
inlined here — 540 lines).

### `docs/paper_assets/calculations/redesign_v2_20260807/phase0_baseline_lockin/*`
New documentation/data only — consolidates pre-existing and previously
scratchpad-only validation results into one committed, versioned location.
No production code in this subtree. Content detailed in §4 below.

## 3. What was run this phase

**No new sim runs were executed in Phase 0.** All numbers in this phase's
output are from runs already executed earlier in this session (before the
redesign was announced) and are being consolidated/committed here, not
regenerated. For the record, the runs being consolidated:

| Run | Script | Count | Approx. duration |
|---|---|---|---|
| 9.0m launch, single-pair timestep check | `contact_timestep_check.py` | 2 runs (1ms, 4ms) | ~48s per run |
| 9.0m launch, 5×2 timestep distribution | `contact_timestep_distribution.py` | 10 runs (5 per timestep) | ~2-3 min per run, ~31 min total |
| 60° self-righting, single-pair timestep check | `righting_timestep_check.py` (via `run_it_v2.sh`) | 2 runs (1ms, 4ms), preceded by 2 failed attempts (0 valid data) | ~3 min per valid run |

## 4. Results

- **Yaw-slew (C13, pre-existing, unchanged):** 107° commanded → 106.03°
  achieved, <1° of target by t+9.3s.
- **Timestep, C13 (pre-existing, unchanged):** 1ms 106.06°/9.61s vs. 4ms
  106.15°/8.70s.
- **Timestep, 9.0m launch, single pair (this phase, newly committed):**
  1ms ratio=0.286/18.0s vs. 4ms ratio=0.292/18.1s.
- **Timestep, 9.0m launch, 5×2 distribution (this phase, newly committed):**
  1ms: n=4/5 stabilized, ratios [0.009, 0.301, 0.013, 0.210], mean=0.133.
  4ms: n=3/5 stabilized, ratios [0.324, 0.447, 0.205], mean=0.325. 3/10
  runs total (30%) never separated within the 120s timeout.
- **Timestep, self-righting 60° tilt, single pair (this phase, newly
  committed):** 1ms u_z>0.9 at t=111.8s, final u_z=0.99942. 4ms u_z>0.9 at
  t=106.4s, final u_z=0.99941.
- **Self-righting reliability (pre-existing, unchanged):** pre-redesign
  controller 1/21 (4.8%); post-redesign (this baseline) controller 1/21
  (4.8%) — identical. Severe-tilt (>120°) subset: 2/8 triggered a righting
  attempt at all, 0/8 reached stable landed within 200s.
- **Gain values (this baseline):** K_ang=0.05 N·m/rad, K_rate=0.066
  N·m/(rad/s).

## 5. Anything that didn't go as planned

- The self-righting timestep check failed twice before producing valid
  data: first because the harness script didn't set
  `GZ_SIM_RESOURCE_PATH`, causing `gz sim` to fail loading the world
  (`Unable to find uri[model://skydome]`) with both "runs" silently
  producing `uz=None` for their full duration; second, after that fix,
  because a leading bare `pkill -9 -f "..."` returns exit 1 when nothing
  matches and the invoking shell aborted the entire script before Python
  ever ran, with `2>/dev/null` hiding the only diagnostic that would have
  explained it. Both failed attempts and the working fix are preserved in
  `self_righting_timestep_check/` (see that folder's README) so a future
  rerun doesn't repeat either dead end.
- `GENTLE_RIGHTING_SPEED` in `landing_controller.py` was left at a
  temporary diagnostic value rather than being reverted or deliberately
  tuned before this snapshot — called out explicitly in the manifest so a
  later phase doesn't mistake it for an intentional design choice.
- The 3-axis coupling gap in self-righting (see §2 above) remains
  unresolved. This baseline documents it but does not fix it — any later
  phase that touches righting behavior inherits this open issue.
- The 4.3m/-56° headline directional-hop figure is NOT part of this
  reference set as a trustworthy number (its own re-verification found
  achieved ground-track azimuth 122.66° against a held yaw of -55°) —
  flagged in the manifest to prevent it being treated as validated
  baseline behavior in a later phase.

## 6. Checkpoint verdict

**Checkpoint (from the phase instructions): "we have a named, frozen
baseline to compare every later phase against."**

**PASS.** A named, frozen baseline exists:
- Tag `pre-mass-redesign` → commit `4cc7de9`, covering `model.sdf` (via
  its last real-edit commit `63f73b8`, unchanged) and all three controller
  files exactly as they stood at snapshot time, including un-reverted
  diagnostic values (documented, not hidden).
- A single consolidated manifest
  (`redesign_v2_20260807/phase0_baseline_lockin/BASELINE_MANIFEST.md`)
  covering all four required reference datasets (yaw-slew spot check,
  timestep comparisons, launch-delivery distribution, self-righting data),
  with every previously ephemeral (scratchpad-only) dataset now copied
  into the git repo so it survives independent of this session.
- The redesign_v2_20260807/ naming convention is established and in use;
  no pre-redesign filename was reused or overwritten.

Known limitation of this baseline (not a checkpoint failure, but worth
flagging for whoever reviews this next): two of the four reference
categories (timestep-check launch/righting) rest on very small sample
sizes (n=2–5 per condition), and the launch-delivery distribution itself
demonstrates that small samples are unreliable in this regime. The
baseline is frozen and complete per the stated checkpoint; it is not
claimed to be statistically strong everywhere.
