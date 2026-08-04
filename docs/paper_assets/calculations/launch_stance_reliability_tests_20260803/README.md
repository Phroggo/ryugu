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

- `c14_asymmetric_torque_harness.py` -- the test script. Watches
  `hopper_locomotion`'s console log for the IGNITION line, then overrides
  `hip_joint_0`'s commanded position for 1.5s right at liftoff (racing the
  normal launch choreography, last-write-wins) before logging odometry/IMU
  through the resulting flight.
- `c14_attempt1_stdout.log` through `c14_attempt4_stdout.log` -- four
  attempts. All four failed at the crouch-stance gate before ever reaching
  IGNITION (see `hopper_locomotion_console_c14_attempts.log` for the exact
  abort messages -- uz values of 0.83, 0.84, and one crouch that was kicked
  airborne to z=18m by its own aborted attempt). The torque-injection logic
  itself was never actually exercised.

## C9 attempt (headline 4.3m / ~20min directional hop)

Sets `target_yaw` to the paper's own reported azimuth (-56 deg), waits for
yaw-hold to converge (a mechanism independently confirmed working in
`../attitude_rerun_20260803/`), then commands a directional hop and logs
odometry position/time throughout to measure actual displacement, heading
error, and flight duration against the claimed 4.3m / ~20min figures.

- `c9_directional_hop_harness.py` -- the test script.
- `c9_attempt1_stdout.log` -- one attempt (see
  `hopper_locomotion_console_c9_attempt.log` for the raw abort message:
  uz=0.84, speed=0.084). Same crouch-stance-gate failure as C14; never
  reached IGNITION, so no displacement/timing data was collected.

## Status

**Neither C14 nor C9 was confirmed or refuted.** Both harnesses are ready
to rerun with the z>=6.0 spawn fix; that's the recommended next step if
live evidence is still wanted for either claim.
