# Attitude-control rerun, 2026-08-03

Raw telemetry captured by directly instrumenting a live scout_1 instance
(ROS2 Humble + Gazebo Harmonic, headless), independent of any narrative dev
log. Logger: scripts/log_attitude.py in this directory's parent scratch dir
(subscribes to /scout_1/{attitude_error,odometry,imu} and dumps every sample
with wall-clock receipt time). All *.jsonl files are one-JSON-object-per-line.

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
landing.log). A stray 0.157 m/s liftoff followed anyway. Peak angular rate
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
artifact (see landing.log: "Airborne while IDLE (v=400.518 m/s, free-fall
accel)"), which triggered real (if artifact-driven) tumbling motion from
165 down to 93 deg over ~5s -- then got permanently stuck at 92.99 deg for
the remaining 40s of the window, with the landing controller stuck in
FLIGHT state rather than transitioning to LANDED+RIGHTING. Self-righting
never engaged. Caveat: this test's trigger mechanism is artifact-contaminated
(teleport velocity spike), so it should not be read as a clean disproof of
C14 -- but combined with the airborne test above, and the unrelated
self-righting FAILURE incidentally captured during the C12 attempt (5
RIGHTTRACE attempts, u_z stuck at 0.7727 = 39.4 deg tilt, gave up after 5
attempts -- see landing.log lines ~1785736835-1785736846), there are now
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

## attitude.log / loco.log / landing.log
Full stdout of the three ROS2 controller nodes across this whole session
(multiple respawns of scout_1). Includes the incidental self-righting
failure referenced above.
