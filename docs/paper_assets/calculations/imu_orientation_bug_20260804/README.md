# IMU orientation is spawn-relative, not world-relative (found 2026-08-04)

While investigating why 12 of the 21 C15/C16 pre-redesign trials never
triggered a self-righting attempt at all despite being severely tilted
(see `../pre_redesign_self_righting_baseline_20260804/README.md`), traced
it to a real, verifiable issue in how the simulated IMU reports
orientation, not a bug in that test's harness.

## The finding

`ryugu_sim/model.sdf`'s IMU sensor block has no
`<orientation_reference_frame>` configured:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
</sensor>
```

With no reference frame specified, gz-sim's IMU sensor reports orientation
**relative to whatever pose the sensor itself was created at**, not
relative to true world "up." A robot that is directly spawned already
tilted, and that doesn't undergo much real rotational motion afterward,
will read as "upright" on its own IMU topic -- because it hasn't moved
relative to *itself* -- even though its true world-frame tilt is severe.
Odometry (`/model/<agent>/odometry`, from the separate odometry-publisher
plugin) is unaffected: it reports genuine world-frame orientation
correctly and immediately, with no such reference-latching behavior.

## Confirmed via two independent isolated diagnostics

Both spawn `scout_1` alone (no controllers), instantly teleport it to a
172 deg tilt via entity remove+create (`expected u_z = -0.990`), and
compare `/scout_1/imu` orientation-derived u_z against
`/scout_1/odometry` orientation-derived u_z over the following ~15-20 s.

- **`diag_v1_persistent_bridge.py`** (bridge stays alive across the
  respawn): after the teleport, `odom_uz` correctly jumps to -0.99
  immediately; `imu_uz` stays frozen at the pre-teleport reading (1.0)
  for the entire remainder of the run.
- **`diag_v2_fresh_bridge.py`** (matches the real C15/C16 harness's own
  ordering exactly: entity removed and recreated at the target tilt
  *first*, then a completely fresh bridge process started *after* --
  ruling out stale-subscription-across-respawn as the cause): **1884
  consecutive IMU messages, every single one reporting u_z=1.0**; odometry
  correct from its very first message (377 messages, all -0.99).

Raw results: `diag_v1_results.json`, `diag_v2_results.json`.

## Scope -- what this does and doesn't affect

**Affected:** `ryugu_sim/landing_controller.py`'s `_is_badly_tilted()` and
`_is_inverted()`, both called from `imu_callback()` using `msg.orientation`
directly from the IMU topic -- present in both the pre-redesign controller
(commit `63f73b8`) and the current shipped controller, unchanged in this
respect by the redesign. `ryugu_sim/attitude_controller.py`'s yaw-hold and
tilt-PD logic (also driven from `imu_callback()`) reads the same field and
would be similarly affected under the same trigger condition.

**Not affected:**
- `hopper_locomotion.py`'s `_stance_ok()` launch gate uses
  `last_uz`, computed from **odometry**, not IMU -- confirmed by reading
  `odom_callback()` directly. Unaffected.
- Every outcome measurement made by this week's own test harnesses (C9,
  C13, C14, C15/C16, C17/C18) used **odometry**, not IMU, to judge
  recovered/failed/landed -- so none of those pass/fail results are
  contaminated by this bug. It only affects the *robot's own internal
  decision* of whether to attempt a righting/correction maneuver in the
  first place, not how any of this week's tests measured the outcome.
- C13 and C14 specifically are unaffected by the mechanism itself: both
  reached their tilt through continuous real motion (yaw-hold convergence
  and an in-flight induced tumble, respectively) on an entity that was
  never removed/recreated mid-test, so the IMU sensor's reference was
  never re-latched during the observation window.

## Why this wasn't visible before this week

Entity remove+create is a **test-harness convenience** (fast trial
cycling), not something that happens during normal operation or, as far
as this repo's own record shows, during the original historical
measurements the paper cites -- a robot deployed once at mission start and
never artificially respawned would establish its IMU reference at genuine
upright deployment and never hit this failure mode again for the rest of
its run. This is very likely why it was never caught before: it's
specifically a consequence of the rapid-respawn methodology introduced
for this week's batch reruns, not a bug that would have been visible in
organic single-run operation.

## Recommendation

Two independent fixes, either sufficient on its own:
1. Add an explicit `<orientation_reference_frame>` to the IMU sensor in
   `model.sdf` so it reports true world-frame orientation regardless of
   spawn history (the more correct, permanent fix -- makes the simulated
   sensor behave like the odometry-based ground truth already used
   everywhere else).
2. Switch `_is_badly_tilted()` / `_is_inverted()` (and the analogous
   `attitude_controller.py` logic) to read orientation from odometry
   instead of the IMU topic, matching what `hopper_locomotion.py` already
   does and what every test harness already trusts.

Neither has been applied to the shipped code -- this README documents the
finding; **`ryugu_sim/landing_controller.py` and
`ryugu_sim/attitude_controller.py` are unmodified.** For this week's own
severe-tilt rerun, the practical workaround is to avoid entity respawn
entirely when starting a trial already-tilted (see
`../severe_tilt_no_respawn_rerun_20260804/README.md` once that rerun is
complete).
