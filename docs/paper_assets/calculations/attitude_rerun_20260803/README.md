# Attitude-control rerun, 2026-08-03

Raw telemetry captured by directly instrumenting a live scout_1 instance
(ROS2 Humble + Gazebo Harmonic, headless), independent of any narrative dev
log. Logger: attitude_telemetry_logger.py in this directory (subscribes to
/scout_1/{attitude_error,odometry,imu} and dumps every sample with
wall-clock receipt time). All *.jsonl files are one-JSON-object-per-line.

## Files in this directory

| File | What it is |
|---|---|
| `attitude_telemetry_logger.py` | The rclpy logger used to capture every `*.jsonl` file below. |
| `c13_yaw_slew_raw_telemetry.jsonl` | C13 test: 107 deg commanded yaw slew. **Confirms** the paper's claim. |
| `c12_liftoff_attempt_raw_telemetry.jsonl` | C12 test: aborted jump + stray liftoff. Inconclusive. |
| `c14_tumble_airborne_no_recovery.jsonl` | C14 test 1/3: 165 deg tumble injected mid-flight. No recovery in 30s. |
| `c14_tumble_ground_stuck_at_93deg.jsonl` | C14 test 2/3: same tumble, teleported to ground. Stuck at 93 deg. |
| `c14_tumble_natural_spawn_frozen_dart_sleep.jsonl` | C14 test 3/3: tumble baked into spawn. Robot frozen entirely (DART sleep). |
| `attitude_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log` | Node console output, session 1 (see below). |
| `hopper_locomotion_console_session1_covers_c12_liftoff_attempt.log` | Node console output, session 1 (see below). |
| `landing_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log` | Node console output, session 1 (see below). |
| `attitude_controller_console_session2_covers_c14_natural_fall_test.log` | Node console output, session 2 (see below). |
| `landing_controller_console_session2_covers_c14_natural_fall_test.log` | Node console output, session 2 (see below). |

## c13_yaw_slew_raw_telemetry.jsonl -- CONFIRMS the paper's C13 claim
Commanded target_yaw = 1.8675 rad (107 deg) while scout_1 was airborne
(tilt-PD flight mode). True yaw reconstructed from the odometry quaternion
(NOT from the attitude_error topic -- see caveat below). Result: monotonic,
overdamped convergence to 106.03 deg, <1 deg of target by t+9.3s, holding
at 0.97 deg steady-state error for the rest of the 20s window. Matches the
paper's "107 degree yaw slew converging overdamped, held within 1 degree at
zero rate" closely.

CAVEAT discovered in the process: the /{robot}/attitude_error ROS topic is
NOT a reliable yaw-error signal during flight. attitude_controller.py
computes total_error = abs(error_yaw) at one point (grounded/general path)
but OVERWRITES it with total_error = sqrt(err_x^2+err_y^2) -- a roll/pitch
tilt-only metric -- inside the in-flight tilt-PD branch. So while airborne,
the published attitude_error reflects tilt-hold quality, not yaw-hold
quality, regardless of target_yaw. Anyone instrumenting this topic for yaw
analysis should reconstruct yaw from the odometry quaternion instead.

## c12_liftoff_attempt_raw_telemetry.jsonl -- INCONCLUSIVE for C12
Commanded jump (distance=1.0m) was aborted mid-crouch (bad stance, robot
was still recovering from an unrelated self-righting attempt -- see
landing_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log). A stray 0.157 m/s liftoff followed anyway. Peak angular rate
0.74 rad/s (vs. the paper's cited 0.24 rad/s launch transient), decaying to
exactly 0.0 by t+2.5s and staying there for the rest of the window. Real
data, but this is an anomalous kick-off, not a clean commanded launch --
does not confirm or refute C12's specific numbers.

## c14_tumble_airborne_no_recovery.jsonl -- CONTRADICTS C14 as stated
Injected a genuine 165 deg tumble (gz set_pose, quaternion for 165 deg
rotation about the body X axis) while scout_1 was airborne (tilt-PD armed).
True tilt-from-vertical reconstructed from odometry quaternion (uz = 1 -
2(qx^2+qy^2), tilt = arccos(uz)). Result: NO recovery over 30s -- tilt drifted
from 165.000 to 164.953 deg, i.e. essentially frozen. The in-flight tilt-PD
branch does not appear to correct large tumbles at all within this window;
compare to the paper's "165 degree tumble damped to 3.6 degrees in ~20s".

## c14_tumble_ground_stuck_at_93deg.jsonl -- ALSO does not reproduce C14
Follow-up: teleported the same 165 deg tumble down near ground level to try
to trigger landing_controller's separate self-righting system (RIGHTTRACE),
since the in-flight controller clearly wasn't the mechanism. The teleport's
instantaneous position jump produced a nonphysical ~400 m/s velocity
artifact (see landing_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log: "Airborne while IDLE (v=400.518 m/s, free-fall
accel)"), which triggered real (if artifact-driven) tumbling motion from
165 down to 93 deg over ~5s -- then got permanently stuck at 92.99 deg for
the remaining 40s of the window, with the landing controller stuck in
FLIGHT state rather than transitioning to LANDED+RIGHTING. Self-righting
never engaged. Caveat: this test's trigger mechanism is artifact-contaminated
(teleport velocity spike), so it should not be read as a clean disproof of
C14 -- but combined with the airborne test above, and the unrelated
self-righting FAILURE incidentally captured during the C12 attempt (5
RIGHTTRACE attempts, u_z stuck at 0.7727 = 39.4 deg tilt, gave up after 5
attempts -- see landing_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log lines ~1785736835-1785736846), there are now
three independent real-sim data points in one session where tilt recovery
either didn't engage or didn't complete, none resembling C14's clean
165->3.6 deg claim.

## C11 (rate deadband -> +/-1.2 deg limit cycle): not re-tested live
Not necessary -- attitude_controller.py:230-236 contains a code comment
(not narrative-doc prose, survives the doc purge on its own) written by
whoever built this, stating verbatim: "First attempt also deadbanded the
rate at 0.005 rad/s; live telemetry [showed] +/-1.2 deg limit cycle between
the deadband walls" -- and explaining that the *current* shipped controller
deliberately does NOT deadband rate (only angle) because of this exact
result. This is durable, in-repo evidence for C11 independent of
research_report.md/Study_Guide.md/HANDOFF.md.

## Node console logs, session 1 (covers C12, C13, and the first two C14 tests)
Three files, one per ROS2 node, all from the same continuous session
(scout_1 was respawned several times within it, but the nodes themselves
were not restarted until session 2 below):

- `attitude_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log`
  -- stdout of `attitude_controller.py`. Shows the "Airborne" / "Contact --
  commanded-flight latch cleared" / "New target yaw received" / "RW
  righting in progress" state transitions referenced throughout this file.
- `hopper_locomotion_console_session1_covers_c12_liftoff_attempt.log`
  -- stdout of `hopper_locomotion.py`. Shows the aborted jump command
  ("Initiating Tri-Pedal Jump Sequence!" followed by no ignition) behind
  the C12 attempt.
- `landing_controller_console_session1_covers_c12_c13_c14_airborne_and_ground_tests.log`
  -- stdout of `landing_controller.py`. Contains both incidental findings:
  the "400.518 m/s" fake-velocity artifact from the ground-teleport C14
  test, and the real self-righting failure (5 RIGHTTRACE attempts, u_z
  stuck at 0.7727, gave up) around lines 1785736835-1785736846.

## Node console logs, session 2 (covers the third C14 test, natural spawn)
Nodes were killed and cleanly restarted before this test (session 1's logs
carried stale FSM state from earlier respawns that would have corrupted
the landed/contact detection otherwise). Same three nodes, same content
structure as session 1:

- `attitude_controller_console_session2_covers_c14_natural_fall_test.log`
- `landing_controller_console_session2_covers_c14_natural_fall_test.log`
  -- contains the "583.097 m/s" fake-velocity-on-spawn artifact referenced
  below.
- (hopper_locomotion wasn't exercised in this test -- no session-2 file
  for it.)

## c14_tumble_natural_spawn_frozen_dart_sleep.jsonl -- third independent C14 attempt, different failure mode
Follow-up requested after the first two C14 tests: try a "cleaner" trigger
by baking the 165 deg tumble orientation directly into the initial gz
entity-create call (no mid-flight teleport this time), spawned at a modest
z=0.3m for a short natural fall under Ryugu gravity, then logged for 90s.

Result: the robot never moved AT ALL for the full 87.8s window -- tilt
pinned at exactly 165.006 deg the entire time, not even drifting under
gravity. landing_controller_console_session2_covers_c14_natural_fall_test.log shows the same nonphysical high-velocity artifact
on spawn ("Airborne while IDLE (v=583.097 m/s, free-fall accel)") that
appeared on the teleport test too -- this turns out to be inherent to how
the velocity estimator reacts to a freshly-created entity, not specific to
mid-flight teleporting.

The complete lack of motion (not even gravity) is explained by
attitude_controller.py's own documented DART-sleep-defeat mechanism (see
the "SLEEP-DEFEAT ROTOR" comment ~line 483): gz-sim8 puts a quiescent model
to sleep regardless of allow_auto_disable=false, and the only anti-sleep
trick in the code (idling the yaw wheel at a small constant speed) is
explicitly gated `if (not self.in_flight)` -- grounded only. An airborne,
undisturbed robot has no anti-sleep mechanism at all, so a motionless
in-flight spawn just freezes indefinitely in whatever orientation it starts.

Taken together, all three C14 attempts (airborne injection, ground
teleport, natural spawn) hit a different real technical obstacle, and none
reproduced anything resembling the paper's "165 -> 3.6 deg in ~20s" claim.
This is not proof the claim is unreproducible under all conditions (e.g. a
tumble induced by genuine in-progress flight dynamics, with existing motion
preventing DART sleep, is untested), but it's a consistent enough pattern
across independent methods that the claim should be treated as unconfirmed
pending a cleaner test setup, not treated as confirmed.
