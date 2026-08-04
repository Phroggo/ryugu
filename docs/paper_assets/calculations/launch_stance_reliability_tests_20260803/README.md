# Launch-stance reliability tests: C14 (tumble recovery) and C9 (headline hop), 2026-08-03

Two separate retest attempts, both blocked by the same issue: neither
scout_1 could reliably clear `hopper_locomotion.py`'s launch-stance gate
(`uz > 0.85` and `speed < 0.012`, continuously checked, 45s crouch timeout)
from a cold spawn.

## Root cause (found afterward, applies to both)

Both attempts spawned scout_1 close to the ground (z = 0.05-0.06 m) for
speed. This causes a real Gazebo terrain "pop-out" on spawn at this XY
location -- see
`../self_righting_reliability_test_20260803/README.md` for the full
investigation and confirmation. The practical effect here: the robot never
stops moving/settling, so `_stance_ok()` never passes within the 45s crouch
window, and the crouch aborts every time
("Aborting hop: stance still bad at crouch timeout"). One abort itself
also appears to have triggered a live, unplanned reproduction of the
paper's own "Law 3" finding (grounded actuator motion is a propulsion
event) -- a crouch abort left the robot drifting at z=18m sometime later in
one C14 attempt, consistent with the crouch's own leg motion, not the spawn
pop-out, having kicked it further.

**Fix for a rerun: spawn at z>=6.0 (matching `spawner.py`'s convention),
not a low height.**

## C14 attempts (asymmetric-launch-torque tumble method)

Tries to reproduce the paper's 165->3.6deg tumble claim the way the
original dev-log measurement apparently did it (recovered from git
history, `walkthrough.md`): inject an asymmetric torque by forcing
`hip_joint_0` to overextend during the launch phase itself, rather than
the artificial pose-injection methods tried in the first attitude rerun
(`../attitude_rerun_20260803/`), which never got the controller to even
attempt a correction.

- `c14_asymmetric_torque_harness.py` -- the original script (attempts 1-4,
  all blocked by the spawn-height gate, see above).
- `c14_attempt1_stdout.log` through `c14_attempt4_stdout.log` -- the four
  blocked attempts. All four failed at the crouch-stance gate before ever
  reaching IGNITION (see `hopper_locomotion_console_c14_attempts.log`).
- `c14_asymmetric_torque_harness_FINAL_working_version.py` -- same test
  logic, but spawning at z=4.95 (just above the ~4.8m local terrain) so a
  real crouch/ignition cycle could actually happen. Reached IGNITION
  reliably across four follow-up runs.

**Result: the recovery mechanism is confirmed working, though the induced
tumble never reached the full 165 deg magnitude.**

| File | Override value / duration | Peak tilt | Recovery |
|---|---|---|---|
| `c14_success_5deg_mild_wobble.jsonl` | -0.35 rad / 1.5s | 4.6 deg | held ~1.4 deg |
| `c14_success_6deg_mild_wobble.jsonl` | -0.35 rad / 8.0s | 6.5 deg (still rising at window end) | inconclusive, too short |
| `c14_success_54deg_tumble_recovered.jsonl` | -2.8 rad / 8.0s | **54.0 deg** at t+14.4s | to <1 deg by t+20s, held at 0.82 deg for the rest of the 40s window |
| `c14_success_28deg_tumble_recovered.jsonl` | -2.8 rad / 12.0s | **28.4 deg** at t+17.3s | to <1 deg by t+22.4s, held at exactly 1.019 deg for the rest of the window |

The two large-override runs are clean, real, induced tumbles (not
artificial pose injection) recovered by the actual flight-mode tilt-PD:
fast (recovery to <1 deg within ~5-6s of peak), overdamped (no
oscillation/overshoot visible in either trace), and settling to a stable
sub-1.1-degree residual that holds indefinitely -- qualitatively exactly
what the paper describes. Neither run reached anywhere near 165 deg
(-2.8 rad is close to the hip joint's physical limit of +/-3.14 rad; a
larger tumble would need a different induction method, e.g. combining the
override with a knee-joint disturbance, or timing it to coincide with
actual separation rather than mid-ramp). **Treat this as confirming the
recovery mechanism works and is overdamped, not as confirming the specific
165->3.6 deg numbers.**

## C9 attempt (headline 4.3m / ~20min directional hop)

Sets `target_yaw` to the paper's own reported azimuth (-56 deg), waits for
yaw-hold to converge (a mechanism independently confirmed working in
`../attitude_rerun_20260803/`), then commands a directional hop and logs
odometry position/time throughout to measure actual displacement, heading
error, and flight duration against the claimed 4.3m / ~20min figures.

- `c9_directional_hop_harness.py` / `c9_attempt1_stdout.log` -- the
  original blocked attempt (crouch-stance-gate failure, uz=0.84).
- `c9_directional_hop_harness_FINAL_working_version.py` -- same logic,
  spawned at z=4.95. Reached IGNITION and flew a complete, real flight.

**Result: a real flight was measured, and it does not match the paper's
claim.** Full data in `c9_success_flight_1.24m_wrong_heading.jsonl`
(9159 samples) and `c9_final_test_stdout.log`; console logs in
`attitude_controller_console_c9_final.log`,
`hopper_locomotion_console_c9_final.log`, and
`landing_controller_console_c9_final_INCLUDES_righting_cascade.log`.

- Commanded azimuth -56 deg; measured yaw at launch -55.03 deg -- this part
  matches the paper's own stated "-55 deg measured vs -56 deg commanded"
  almost exactly.
- Commanded distance 3.0m (not the paper's specific commanded value, which
  isn't stated). Genuine separation, flight, and landing all occurred.
- Ground displacement at the moment of contact: only **1.24 m**, far short
  of the claimed 4.3m (note this used a smaller commanded distance than
  whatever produced the paper's headline number, so this is not a strict
  apples-to-apples comparison on distance).
- **Achieved ground-travel azimuth at contact: 122.66 deg** -- this does
  *not* match the held yaw heading of -55 deg at all. The body pointed one
  way; the robot travelled a very different way. This is the more
  significant discrepancy, independent of the distance mismatch.
- Yaw during the "clean" pre-contact flight was mostly held near -55 deg
  (mean -55.24 deg) but spiked to +135 deg at least once mid-flight --
  a real, unexplained disturbance during what the paper describes as
  yaw-hold-stabilized flight.
- Flight time to contact: 365.6s (6.1 min) -- shorter than the paper's
  ~20 min, consistent with the smaller commanded distance.
- **The landing itself was not clean**: contact triggered several genuine
  false "not actually landed" resets (see the C28 note in
  `claim_source_citations.md`), then the robot settled badly tilted
  (u_z=0.07, ~86 deg) and self-righting had to engage -- see
  `../self_righting_reliability_test_20260803/README.md` for the full
  cascade that followed. This directly contradicts the paper's "confirmed
  landed without a false trigger" framing for this class of hop.

## Status

**C14: recovery mechanism confirmed working (qualitatively), specific
165 deg magnitude not reached.** **C9: real flight measured, and it
contradicts the paper's specific displacement (1.24m vs 4.3m) and heading
(122.66 deg travel vs -55 deg yaw) figures**, though the yaw-hold accuracy
figure itself (-55.03 deg vs -56 deg commanded) does match. Given this
directly contradicts rather than merely fails to confirm the claim, the
paper's wording should be revisited.
