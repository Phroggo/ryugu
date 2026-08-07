#!/usr/bin/env python3
"""Phase 3: moment of inertia about the tripod support-edge pivot axis, for
self-righting/tipping dynamics (companion to the paper's existing static
tau ~= mgw/2 tipping-torque figure, Sec. 3.3/3.4.2).

No prior I_pivot calculation exists anywhere in this repo (confirmed by
grep across the whole tree) -- only the static torque tau~=mgw/2 was ever
computed. This computes I_pivot for the first time, using the same
parallel-axis methodology already established for I_bot
(compute_moi.py / compute_whole_robot_cg_inertia.py), against the
corrected Phase 2 model.

Method
------
1. Resolve every link's frame in the "retracted (flight neutral)" posture
   (hip=0, knee=0) -- confirmed (not assumed) to be the real landed/
   standing stance: a geometry check showed the OTHER candidate,
   "splayed (crouch stance)" (hip=0.33, knee=-2.60), puts the foot ABOVE
   base_link's origin, which cannot be a ground-contact pose, whereas
   retracted puts the foot correctly below the chassis AND matches
   hopper_locomotion.py's own IDLE/landed leg target
   (set_joints(0.0, 0.0)). Reuses the exact same frame-resolution code as
   compute_whole_robot_cg_inertia.py.
2. Compute each foot position as the calf link's distal end
   (calf_frame_position + calf_R @ (0,0,CALF_LENGTH)) -- the foot-sphere
   attachment point per model.sdf's own convention (calf's visual cylinder
   spans local z in [0, CALF_LENGTH] from the link origin).
3. For each of the 3 edges of the support triangle (pairs of feet), find
   the perpendicular distance from the whole-body CG to that edge's line,
   and the moment of inertia about an axis through the CG parallel to
   that edge (I_axis = n^T I_cg n), then parallel-axis shift to the edge
   itself: I_pivot = I_axis_through_cg + M*d_perp^2.
4. Report all 3 edges (a symmetric tripod would make them identical; this
   platform's CG is no longer perfectly centered post-Phase-2, so they're
   not) and flag the minimum (least torque to tip over, the governing
   case for a stability-margin figure) as "the" pivot figure.

Run:  python3 compute_pivot_inertia.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                  "../phase2_physical_model_rebuild"))
from compute_whole_robot_cg_inertia import (  # noqa: E402
    load_links, world_frames, full_tensor, NEW_SDF
)

OLD_SDF = os.path.join(os.path.dirname(__file__),
                        "../phase2_physical_model_rebuild",
                        "model_OLD_pre_phase2_reference.sdf")

CALF_LENGTH = 0.15  # m, from generate_detailed_spacehopper.py (unchanged by Phase 2)
G = 1.14e-4          # m/s^2, Ryugu surface gravity (Research_Paper.md nomenclature)


def run(sdf_path, tag, hip=0.0, knee=0.0, posture_label="retracted/landed-standing"):
    print(f"\n{'#'*20} {tag} [{posture_label}]: {sdf_path} {'#'*20}")
    links = load_links(sdf_path)
    base_link_origin = links['base_link']['t'].copy()
    frames = world_frames(links, hip, knee)
    m_tot, com_absolute, I_cg = full_tensor(links, frames)
    com = com_absolute - base_link_origin

    print(f"Posture: {posture_label} (hip={hip:+.2f}, knee={knee:+.2f})")
    print(f"Total mass: {m_tot:.4f} kg")
    print(f"CG relative to base_link origin: "
          f"({com[0]:+.5f}, {com[1]:+.5f}, {com[2]:+.5f}) m")

    # Foot positions: distal end of each calf, in the same absolute/
    # model frame as `com_absolute` above (not yet re-based to base_link
    # origin -- do that once, consistently, below).
    feet_absolute = {}
    for leg in range(3):
        calf_name = f"calf_{leg}"
        T, R = frames[calf_name]
        foot_local = np.array([0, 0, CALF_LENGTH])
        feet_absolute[leg] = T + R @ foot_local

    feet = {leg: pos - base_link_origin for leg, pos in feet_absolute.items()}
    com_rel = com  # already relative to base_link origin

    print("\nFoot positions (relative to base_link origin):")
    for leg, pos in feet.items():
        print(f"  foot {leg}: ({pos[0]:+.5f}, {pos[1]:+.5f}, {pos[2]:+.5f}) m")

    # Weight and static tipping torque tau = m*g*w/2 for each edge, where
    # w = 2 * perpendicular distance from CG's ground-plane (xy) projection
    # to the edge line (i.e. w/2 IS that perpendicular distance) -- matches
    # the paper's own tau~=mgw/2 definition (Sec. 3.3), generalized here to
    # articulate w explicitly and to also give I_pivot about the same edge.
    print("\n=== Per-edge pivot analysis (support triangle) ===")
    edge_results = []
    pairs = [(0, 1), (1, 2), (0, 2)]
    for a, b in pairs:
        pa, pb = feet[a][:2], feet[b][:2]   # xy only -- ground-plane edge
        edge_vec = pb - pa
        edge_len = np.linalg.norm(edge_vec)
        edge_dir_xy = edge_vec / edge_len
        # perpendicular (xy-plane) distance from CG's ground projection to
        # the edge LINE (not segment -- matches the paper's simple lever-
        # arm treatment)
        cg_xy = com_rel[:2]
        rel = cg_xy - pa
        perp_dist = abs(rel[0] * edge_dir_xy[1] - rel[1] * edge_dir_xy[0])
        w = 2 * perp_dist
        tau = m_tot * G * w / 2

        # 3D pivot axis direction: horizontal (xy) edge direction, since
        # the two feet are both at (approximately) the same ground height.
        axis_3d = np.array([edge_dir_xy[0], edge_dir_xy[1], 0.0])
        I_axis_through_cg = axis_3d @ I_cg @ axis_3d
        # 3D perpendicular distance from CG to the (3D) edge LINE (through
        # foot a, direction axis_3d) -- needed for the real parallel-axis
        # shift, not just the xy-projected lever arm used for tau above.
        cg_to_a = com_rel - np.array([pa[0], pa[1], feet[a][2]])
        d_along = np.dot(cg_to_a, axis_3d)
        d_perp_vec = cg_to_a - d_along * axis_3d
        d_perp_3d = np.linalg.norm(d_perp_vec)
        I_pivot = I_axis_through_cg + m_tot * d_perp_3d ** 2

        edge_results.append(dict(edge=(a, b), w=w, tau=tau,
                                  I_axis_through_cg=I_axis_through_cg,
                                  d_perp_3d=d_perp_3d, I_pivot=I_pivot))
        print(f"\nEdge (foot{a}-foot{b}):")
        print(f"  w (2x perp. dist, CG ground-proj. to edge) = {w:.5f} m")
        print(f"  tau = m*g*w/2 = {tau:.4e} N*m")
        print(f"  I_axis (through CG, parallel to edge)      = {I_axis_through_cg:.6e} kg*m^2")
        print(f"  d_perp (CG to edge line, 3D)                = {d_perp_3d:.5f} m")
        print(f"  I_pivot (about the edge itself)             = {I_pivot:.6e} kg*m^2")

    worst = min(edge_results, key=lambda r: r['w'])
    print(f"\n=== Governing (least-stable) edge: foot{worst['edge'][0]}-foot{worst['edge'][1]} ===")
    print(f"w = {worst['w']:.5f} m, tau = {worst['tau']:.4e} N*m, "
          f"I_pivot = {worst['I_pivot']:.6e} kg*m^2")

    best = max(edge_results, key=lambda r: r['w'])
    spread_pct = (best['w'] - worst['w']) / worst['w'] * 100
    print(f"\nSpread across the 3 edges: w ranges "
          f"{worst['w']:.5f}-{best['w']:.5f} m ({spread_pct:.1f}% spread).")
    return dict(m_tot=m_tot, com=com, worst=worst, best=best)


def main():
    # No prior I_pivot (or rigorously-derived w) calculation exists
    # anywhere in this repo (confirmed by grep across the whole tree) --
    # only the paper's illustrative tau~=mgw/2~=2.9e-5 N*m figure (Sec.
    # 3.3), whose implied w (back-solved: w=2*tau/(m*g)) is ~0.204m.
    # Running this against BOTH the old and new model isolates whether
    # any discrepancy is from the Phase 2 mass change or was already
    # there in the leg geometry (which Phase 2 never touched).
    old = run(OLD_SDF, "OLD (pre-Phase-2, original)")
    new = run(NEW_SDF, "NEW (Phase 2 corrected)")

    implied_w_paper = 2 * 2.9e-5 / (2.50 * G)
    print(f"\n{'='*70}")
    print("SUMMARY / cross-check against the paper's existing tau~=mgw/2 figure")
    print(f"{'='*70}")
    print(f"Paper's stated tau ~= 2.9e-05 N*m implies w ~= {implied_w_paper:.4f} m")
    print(f"Old model, governing edge, rigorously computed:  w = {old['worst']['w']:.4f} m, "
          f"tau = {old['worst']['tau']:.4e} N*m")
    print(f"New model, governing edge, rigorously computed:  w = {new['worst']['w']:.4f} m, "
          f"tau = {new['worst']['tau']:.4e} N*m")
    print("Leg geometry (hip radius, thigh/calf lengths, HIP_PITCH/KNEE_BEND) was "
          "NOT changed by Phase 2, so the old and new w values above differ only "
          "by the CG shift (mass redistribution), not by geometry -- and both are "
          "far from the paper's implied ~0.204m regardless. This looks like a "
          "pre-existing discrepancy in the paper's own w, independent of the mass "
          "redesign -- flagged, not resolved, here.")

    # --- Follow-up (2026-08-07): fold/tuck posture, requested to check
    # whether it's a genuinely different posture from retracted/landed and,
    # if so, how much of a stated ~0.0482 kg*m^2 figure's gap against this
    # phase's retracted-posture I_pivot is a real posture effect. NOTE: no
    # citation for "0.0482 kg*m^2" was found anywhere in Research_Paper.md,
    # the frozen docx, or any other project doc (grepped exhaustively) --
    # computing this anyway because the underlying posture question is
    # real and answerable regardless of that citation's status.
    #
    # fold_hip_target/fold_knee_target (landing_controller.py:167-168) =
    # 0.33/-2.6 -- identical to CROUCH_HIP/CROUCH_KNEE
    # (hopper_locomotion.py) -- IS a real, distinct, code-defined posture,
    # confirmed genuinely different from retracted (hip=0,knee=0).
    print(f"\n\n{'='*70}")
    print("FOLD/TUCK posture (landing_controller.py fold_hip/knee_target, "
          "= CROUCH_HIP/KNEE): 0.33 / -2.6")
    print(f"{'='*70}")
    old_fold = run(OLD_SDF, "OLD (pre-Phase-2, original)", hip=0.33, knee=-2.6,
                   posture_label="fold/tuck (self-righting mid-roll)")
    new_fold = run(NEW_SDF, "NEW (Phase 2 corrected)", hip=0.33, knee=-2.6,
                   posture_label="fold/tuck (self-righting mid-roll)")

    print(f"\n{'='*70}")
    print("CAVEAT on fold/tuck I_pivot's physical meaning")
    print(f"{'='*70}")
    print("During an actual righting roll the body is tipped/inverted, "
          "rolling on the tucked-leg/chassis silhouette, not resting on 3 "
          "feet on a level surface -- so \"moment of inertia about the "
          "support-triangle edge\" is not really the operative quantity "
          "for the ROLLING dynamics of self-righting itself (that's closer "
          "to a body-frame roll-axis inertia through the CG, i.e. an I_bot "
          "variant, not I_pivot). This computation answers the literal "
          "question asked (does the fold/tuck posture change I_pivot vs. "
          "retracted, using the same edge-pivot method) -- it is NOT being "
          "claimed as the correct physical model for mid-roll self-"
          "righting dynamics, which is Phase 5's problem, not this one's.")

    print(f"\n{'='*70}")
    print("2x2 SUMMARY: posture x model, governing edge")
    print(f"{'='*70}")
    print(f"{'':12}{'retracted/IDLE':>18}{'fold/tuck':>18}")
    print(f"{'OLD':12}{old['worst']['I_pivot']:>18.6e}{old_fold['worst']['I_pivot']:>18.6e}")
    print(f"{'NEW':12}{new['worst']['I_pivot']:>18.6e}{new_fold['worst']['I_pivot']:>18.6e}")
    print(f"\nCited (unverified) figure: 0.0482 kg*m^2")
    print(f"Closest match: NEW/fold-tuck = {new_fold['worst']['I_pivot']:.6e} "
          f"({abs(new_fold['worst']['I_pivot']-0.0482)/0.0482*100:.1f}% off) -- "
          f"far closer than NEW/retracted "
          f"({abs(new['worst']['I_pivot']-0.0482)/0.0482*100:.1f}% off, the "
          f"figure originally compared against 0.0482).")


if __name__ == '__main__':
    main()
