#!/usr/bin/env python3
"""Phase 20a: reconstruct the worst-case single-leg unbalanced launch
angular momentum against the current, corrected mass model.

EXPLICITLY A RECONSTRUCTION, not a verified rerun of whatever originally
produced Research_Paper.md SS3.2's ~=0.0084 N m s figure -- no
derivation script for that number exists anywhere in the repo or its
git history (searched exhaustively before this). Methodology, per
explicit user direction:
  - Single leg's own angular momentum about the hip pitch axis
  - Hip + knee combined (calf's ABSOLUTE angular rate = hip rate + the
    knee's own rate relative to the thigh, since both joints rotate
    about the same axis)
  - Treated as fully uncancelled by the other two legs (the standard
    conservative single-actuator bound)
  - At the code-enforced fastest stroke: ramp_T = 1.2s
    (hopper_locomotion.py: `ramp_T = max(1.2, min(20.0, V_GAIN/v_req))`)

All mass/inertia/geometry values read directly from
models/spacehopper/model.sdf (leg 0: thigh_0, calf_0, hip_joint_0) --
current, post Phase 1/2 mass-and-inertia rebuild.

Run: python3 momentum_budget_recompute.py
"""
import math

# ---- current model.sdf values (leg 0, verified identical across legs 1/2) ----
M_THIGH = 0.0219       # kg
IYY_THIGH_OWN = 0.000043  # kg m^2, about thigh's own CoM, thigh-local Y (= hip axis direction)
THIGH_LEN = 0.15        # m (visual cylinder length; CoM at midpoint, verified: ixx/iyy match
                         # the standard about-centroid cylinder formula m(3r^2+L^2)/12 exactly)
THIGH_PITCH = 1.2       # rad, thigh_0's own pose orientation relative to base_link

M_CALF = 0.0143        # kg
IYY_CALF_OWN = 0.000027   # kg m^2, about calf's own CoM
CALF_LEN = 0.15
CALF_REL_PITCH = 0.8    # rad, calf_0's pose orientation relative to thigh_0 (additional pitch)

# hip_joint_0's <axis><xyz>0 1 0</xyz> with no expressed_in override defaults to the joint
# frame, which is coincident with thigh_0's frame (pose relative_to="thigh_0": all zeros).
# Rotation about Y does not move the Y axis itself, so the hip axis direction in
# base_link/world coordinates is simply (0,1,0) regardless of thigh's own 1.2 rad pitch --
# verified by construction, not assumed.

CROUCH_HIP, EXTEND_HIP = 0.33, -0.42     # hopper_locomotion.py
CROUCH_KNEE, EXTEND_KNEE = -2.60, -1.10  # hopper_locomotion.py
RAMP_T_WORST_CASE = 1.2  # s, code-enforced floor (max(1.2, ...))


def main():
    hip_sweep = abs(EXTEND_HIP - CROUCH_HIP)
    knee_sweep = abs(EXTEND_KNEE - CROUCH_KNEE)
    omega_hip = hip_sweep / RAMP_T_WORST_CASE
    omega_knee_rel = knee_sweep / RAMP_T_WORST_CASE
    omega_calf_abs = omega_hip + omega_knee_rel  # same axis, rates add

    print(f"hip sweep = {hip_sweep:.3f} rad, knee sweep = {knee_sweep:.3f} rad")
    print(f"worst-case ramp_T = {RAMP_T_WORST_CASE}s (code-enforced floor)")
    print(f"omega_hip (avg) = {omega_hip:.4f} rad/s, omega_knee_rel (avg) = {omega_knee_rel:.4f} rad/s")
    print(f"omega_calf_absolute (avg) = {omega_calf_abs:.4f} rad/s")
    print()

    # --- thigh CoM distance from hip axis ---
    # Thigh's own CoM, in thigh's own local frame: (0, 0, THIGH_LEN/2) -- a vector with
    # zero Y-component. ANY rotation about Y preserves both the Y-component (stays zero)
    # and the vector's magnitude, so the perpendicular distance from the Y-axis (the hip
    # rotation axis) equals this vector's magnitude regardless of thigh's absolute pitch.
    r_thigh = THIGH_LEN / 2.0
    print(f"r_thigh (perp. distance, hip axis to thigh CoM) = {r_thigh:.4f} m "
          f"(rotation-invariant, see comment)")

    # --- calf CoM distance from hip axis ---
    # Work in thigh_0's local frame (its own absolute 1.2 rad pitch is an overall rotation
    # of the whole planar 2-link chain and does not change perpendicular distance from the
    # hip axis -- verified via the same rotation-invariance argument, only the RELATIVE
    # knee angle (0.8 rad) matters for calf's position relative to the hip).
    # calf origin, in thigh_0 frame: (0,0,THIGH_LEN) [attached at thigh's far end]
    # calf CoM, in calf's own frame: (0,0,CALF_LEN/2); rotate into thigh_0 frame by the
    # relative 0.8 rad pitch:
    cx = (CALF_LEN / 2.0) * math.sin(CALF_REL_PITCH)
    cz = THIGH_LEN + (CALF_LEN / 2.0) * math.cos(CALF_REL_PITCH)
    r_calf = math.hypot(cx, cz)
    print(f"r_calf (perp. distance, hip axis to calf CoM) = {r_calf:.4f} m")
    print()

    I_thigh_about_hip = IYY_THIGH_OWN + M_THIGH * r_thigh ** 2
    I_calf_about_hip = IYY_CALF_OWN + M_CALF * r_calf ** 2
    print(f"I_thigh_about_hip = {IYY_THIGH_OWN} + {M_THIGH}*{r_thigh:.4f}^2 "
          f"= {I_thigh_about_hip:.6e} kg m^2")
    print(f"I_calf_about_hip  = {IYY_CALF_OWN} + {M_CALF}*{r_calf:.4f}^2 "
          f"= {I_calf_about_hip:.6e} kg m^2")
    print()

    # --- Method A: average angular rate over the ramp ---
    L_thigh_avg = I_thigh_about_hip * omega_hip
    L_calf_avg = I_calf_about_hip * omega_calf_abs
    L_total_avg = L_thigh_avg + L_calf_avg
    print("=== Method A: average rate (sweep / ramp_T) ===")
    print(f"L_thigh = {L_thigh_avg:.6e} N m s")
    print(f"L_calf  = {L_calf_avg:.6e} N m s")
    print(f"L_leg_total (avg-rate method) = {L_total_avg:.6e} N m s")
    print()

    # --- Method B: peak instantaneous rate, accounting for the quadratic ease-in profile ---
    # hopper_locomotion.py: "Quadratic ease-in (2026-07-18): rate peaks at release" --
    # s = (t/ramp_T)^2, so ds/dt = 2t/ramp_T^2, peaking at t=ramp_T at ds/dt = 2/ramp_T,
    # i.e. exactly 2x the average rate for a profile that reaches s=1 at t=ramp_T.
    peak_factor = 2.0
    L_total_peak = L_total_avg * peak_factor
    print("=== Method B: peak instantaneous rate at release (quadratic ease-in, 2x average) ===")
    print(f"L_leg_total (peak-rate method) = {L_total_peak:.6e} N m s")
    print()

    # --- Hmax and margin, using the corrected flywheel inertia already in use ---
    I_WHEEL_CORRECTED = 3.944e-4  # kg m^2, Phase 1/2 real RW annulus (attitude_controller.py comment)
    MAX_RW_SPEED = 982.0          # rad/s, Maxon EC20 no-load speed
    H_max = I_WHEEL_CORRECTED * MAX_RW_SPEED
    print(f"H_max (corrected I_wheel={I_WHEEL_CORRECTED}, max_rw_speed={MAX_RW_SPEED}) "
          f"= {H_max:.6f} N m s")
    print()
    print(f"Margin (Method A, avg-rate): H_max / L = {H_max/L_total_avg:.1f}x")
    print(f"Margin (Method B, peak-rate): H_max / L = {H_max/L_total_peak:.1f}x")
    print()
    print(f"For reference, the stale pre-redesign figure cited in Research_Paper.md SS3.2 "
          f"is ~=0.0084 N m s ({H_max/0.0084:.1f}x margin at the corrected H_max).")
    print(f"Ratio of this reconstruction to the stale figure: "
          f"Method A = {L_total_avg/0.0084:.3f}x, Method B = {L_total_peak/0.0084:.3f}x")


if __name__ == '__main__':
    main()
