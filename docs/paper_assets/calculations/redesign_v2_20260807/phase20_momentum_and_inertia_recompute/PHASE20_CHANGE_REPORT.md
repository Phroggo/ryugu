# Phase 20 — Momentum-Budget Reconstruction and Geometric-Proxy Inertia Estimate

Date: 2026-08-14
Scope: two of the four held items authorized in the "go ahead on all four held items" message — item 4 (momentum-budget reconstruction) and item 3 (CAD inertia validation, scope-adjusted to a geometric proxy). Items 1 (comms-loss ablation) and 2 (auction baseline comparison) are tracked separately and not covered here.

**Both items are reconstructions/estimates against the current model, explicitly not verified reruns of whatever originally produced the stale paper figures.** Both are labeled that way everywhere they surface. Item 3 also required stopping mid-task to flag a real scope fork (§2.3) before proceeding, per the user's own pre-authorization to do so.

## 1. Files touched

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase20_momentum_and_inertia_recompute/momentum_budget_recompute.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase20_momentum_and_inertia_recompute/momentum_budget_recompute_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase20_momentum_and_inertia_recompute/geometric_proxy_inertia.py`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase20_momentum_and_inertia_recompute/geometric_proxy_inertia_stdout.log`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase20_momentum_and_inertia_recompute/PHASE20_CHANGE_REPORT.md` (this file)

No source/model files modified — both scripts are standalone reconstructions reading directly from `models/spacehopper/model.sdf`, `ryugu_sim/hopper_locomotion.py`, and `ryugu_sim/attitude_controller.py`.

## 2. Item 4 — Momentum-budget reconstruction

### 2.1 Methodology (per explicit user direction)

No derivation script for the paper's stale ≈0.0084 N·m·s figure exists anywhere in the repo or its git history (searched exhaustively in an earlier session — full history, all calculations directories, code comments). Per the user's own proposed approach, reconstructed as: a single leg's own angular momentum about the hip pitch axis, hip + knee combined (calf's absolute angular rate = hip rate + knee's rate relative to the thigh, since both joints share the same rotation axis), treated as fully uncancelled by the other two legs, at the code-enforced fastest stroke (`ramp_T=1.2s`, the floor in `hopper_locomotion.py`'s `ramp_T = max(1.2, min(20.0, V_GAIN/v_req))`).

All mass/inertia/geometry inputs read directly from `model.sdf` (leg 0: `thigh_0`, `calf_0`, `hip_joint_0`), current post-Phase-1/2 values. The hip axis's direction in world/base_link coordinates is `(0,1,0)` regardless of `thigh_0`'s own 1.2 rad pitch, since rotation about an axis preserves that axis's own direction — verified by construction, not assumed. The same rotation-invariance argument gives the thigh CoM's perpendicular distance from the hip axis as exactly half its length (0.075m) independent of absolute pitch; the calf CoM's distance (0.2093m) is computed by rotating its position into the thigh's local frame by the relative 0.8 rad knee angle.

Two methods computed to bound sensitivity to the launch profile's shape:
- **Method A (average rate)**: sweep angle / ramp_T.
- **Method B (peak instantaneous rate)**: `hopper_locomotion.py`'s quadratic ease-in profile (`s=(t/ramp_T)²`) peaks at exactly 2x the average rate at release — this is the physically relevant worst-instant rate, not just a sanity bound.

### 2.2 Results

```
I_thigh_about_hip = 1.661875e-04 kg m^2
I_calf_about_hip  = 6.533529e-04 kg m^2

Method A (avg-rate):  L_leg_total = 1.328904e-03 N m s
Method B (peak-rate): L_leg_total = 2.657808e-03 N m s

H_max (corrected I_wheel=3.944e-4 kg m^2, max_rw_speed=982.0 rad/s) = 0.387301 N m s

Margin, Method A: 291.4x
Margin, Method B: 145.7x
```

Full derivation output: `momentum_budget_recompute_stdout.log`.

### 2.3 Comparison to the stale paper figure

The stale figure (≈0.0084 N·m·s, cited as a 31x margin against the pre-correction H_max in an earlier Research_Paper.md snapshot, and as ~46x against the corrected H_max per the user's report) is **6-8x larger than this reconstruction** (Method A: 0.158x of the stale figure; Method B: 0.316x). Both reconstructed methods land the margin substantially higher than either previously-cited figure — 145.7x (peak-rate, the more physically conservative of the two) to 291.4x (average-rate).

**This is a real, flagged discrepancy, not a refinement that lands close to the old number.** Two candidate explanations, neither confirmed:
1. The original ≈0.0084 N·m·s figure may have used different (older/uncorrected) leg mass or inertia values, or a different worst-case scenario definition (e.g., a different joint or combination), predating this reconstruction's known-good Phase 1/2 inputs.
2. The original figure's methodology is simply unknown — it may have used a materially different physical assumption (e.g., a faster stroke, a different lever arm, or momentum from a different source entirely) than the one specified for this reconstruction.

Not resolved here — flagged per the standing "no smoothing over inconvenient results" rule. The margin is dramatically larger either way (over 145x even by the more conservative peak-rate method), so the underlying conclusion ("saturation by a single hop is not credible") is unaffected and if anything strengthened — but the ~6-8x gap between the reconstructed number and the previously-cited number should not be presented as if the reconstruction simply confirms the old figure with updated masses. It doesn't; it's substantially smaller, for reasons not fully accounted for here.

## 3. Item 3 — Geometric-proxy inertia estimate

### 3.1 Scope, as reframed by what the model actually contains

`model.sdf`'s top-level link list (`base_link`, `solar_panel`, `rw_x`, `rw_y`, `rw_z`, `drill_link`, `thigh_0/1/2`, `calf_0/1/2`) shows that reaction wheels, all three leg segments, the solar panel, and the drill are **already separate links with real masses and positions** — not part of any lumped body. The reviewer's "lumped model" critique, once checked against the actual file, applies specifically and only to `base_link`'s own 1.3839 kg. This significantly narrows the task from "redo the whole robot's inertia" to "de-lump base_link's own mass using its real internal geometry."

### 3.2 Mid-task scope fork: flagged and resolved with the user

While mapping `base_link`'s internal visual elements against Phase 1's `AUDIT_TABLE.md` component masses, found that the audited masses reconcile almost exactly against `base_link`'s total (chassis 0.492 + MLI 0.098 + antennas 0.03 + cameras 0.03-0.04 + avionics/battery/comms 0.428 + leg motors 0.09 + leg gearheads 0.15 + RW motors 0.066 ≈ 1.384 kg, vs. the model's actual 1.3839 kg) — but **only 47% of that mass (chassis, MLI, antennas, cameras) has a real `<visual><pose>` anywhere in the file.** The other 53% (all 6 leg motors, all 6 leg gearheads, all 3 RW motors, flight computer, attitude-sensing suite, comms transceiver, battery cells, BMS) has no distinguishable visual element at all — confirmed by checking `base_link`'s full visual list (58 named visuals: hull/seams/bolts, 8 corner brackets, MLI x6, antenna clusters, cameras, LEDs, louvers, RW/drill housings) and by confirming `thigh_0`/`calf_0`'s own masses in `model.sdf` exactly match Phase 1's bare-link audit rows with no motor mass folded in.

This is exactly the "bigger undertaking than expected" scenario the user pre-authorized flagging on. Stopped and asked via AskUserQuestion rather than deciding unilaterally; user chose to extend the geometry-only approach with estimated placements for the physically-obvious cases (leg motors/gearheads near each hip axis, RW motors near each RW housing), keeping the truly unplaceable avionics/battery/comms cluster centered at the origin — exactly the resolution the user specified.

### 3.3 Three-tier methodology

Every component is tagged with its evidence tier, since two different kinds of "position" are used and they carry different rigor:

- **Tier 1, geometry-derived (0.660 kg, 47.3%)**: read directly from a real `<visual><pose>` in `model.sdf`. Chassis (own origin, approximated as a thin spherical shell of the chassis's characteristic radius — a labeled engineering approximation, since no exact hollow-box-shell formula is available for the mixed CFRP-skin/Al-frame construction), MLI ×6, both antenna clusters, all 4 camera bodies.
- **Tier 2, estimated placement (0.306 kg, 22.0%)**: no visual exists, but positioned by engineering judgment at the one physically-obvious mounting point — leg motor+gearhead pairs (0.08 kg/leg) at each leg's real hip attachment point (`thigh_N`'s own top-level pose, converted to base_link-relative coordinates), RW motors (0.022 kg each) at each RW link's own position. These are **assumptions, not readings**, flagged as such in the script and here.
- **Tier 3, no placement data (0.428 kg, 30.7%)**: flight computer, attitude-sensing suite, comms transceiver, battery cells ×4, BMS. No visual, and no single physically-obvious mounting point the way an actuator-to-its-joint or a motor-to-its-wheel is. Kept at `base_link`'s origin — identical to the current lumped treatment for this specific mass. This is the part of the estimate that genuinely does not improve on lumping.

A bug was caught and fixed before these numbers were finalized: an earlier draft of the script subtracted `base_link`'s own model-frame offset (0,0,0.25) from the antenna and camera positions, on the mistaken assumption those poses were given in the model frame. They are not — antennas, cameras, and MLI are all child `<visual>` elements *inside* `base_link`'s own `<link>` block, so their poses are already relative to `base_link`'s origin directly. (The Tier-2 leg/RW positions genuinely do need that subtraction, since `thigh_N`/`rw_*` are top-level sibling links given in the model frame — verified this distinction explicitly for each component rather than applying one convention uniformly.)

**Known one-directional limitation**: every non-chassis component (Tier 1 antennas/cameras/MLI and all of Tier 2) is treated as a point mass with zero own-inertia, since no dimensional data exists for any of them. This systematically *underestimates* their true contribution to the tensor — real components have nonzero size. The reported delta from the current lumped value should be read as a lower bound on the true geometric effect, not a tight estimate.

### 3.4 Results

Mass reconciliation: Tier1 + Tier2 + Tier3 = 1.3940 kg vs. `base_link`'s actual 1.3839 kg (+0.0101 kg, 0.7%) — the residual is fully explained by the known 3-vs-4-camera audit/geometry mismatch (§3.5), not an accounting error.

```
                lumped (about origin)   geometric-proxy         diff
ixx             9.912394e-03            7.588105e-03    -2.324288e-03 (-23.4%)
iyy             1.019457e-02            7.827855e-03    -2.366710e-03 (-23.2%)
izz             7.619172e-03            5.665830e-03    -1.953341e-03 (-25.6%)
```

Full component-by-component breakdown: `geometric_proxy_inertia_stdout.log`.

The geometric proxy comes in **~23-26% below** the current lumped `base_link` tensor on all three principal axes, fairly uniformly. This is very plausibly an artifact of the point-mass approximation noted in §3.3 (the current lumped tensor already encodes real "own-inertia" for the true, presumably better-modeled mass distribution it was built to represent; this proxy's point-mass components contribute inertia only via their offset squared, undercounting their own physical extent) — not necessarily evidence that the current lumped model is wrong by that amount. **Not asserting this proxy is more accurate than the current lumped values; only that it is a labeled, tiered, best-effort geometric estimate with a known systematic bias, per the user's explicit "geometric-proxy estimate, not CAD-derived" framing requirement.**

### 3.5 Anomaly flagged, not resolved

Phase 1's `AUDIT_TABLE.md` lists "3 cameras (stereo hazcams ×2 + navcam ×1)" but `model.sdf` has 4 distinct camera-like visual bodies (`camera_body`, `sci_camera_body`, `hazcam_left_body`, `hazcam_right_body`). This script costs all 4 at 0.010 kg each (matching what's actually placed in the model), which is the likely source of the +0.0101 kg mass-reconciliation residual in §3.4. Whether the audit table under-counted or the model over-built is not resolved here — flagged for a future pass, not silently picked one way or the other.

## 4. Checkpoint verdict

**Both items complete, verified against raw script output, and clearly labeled per the user's requirements.** Item 4's reconstruction diverges meaningfully from the previously-cited figure (§2.3) — flagged, not smoothed over; underlying paper conclusion unaffected either way. Item 3 required a mid-task stop-and-flag on a genuine scope fork (§3.2), resolved per explicit user direction, and produced a three-tier, evidence-labeled estimate rather than a single undifferentiated number. Neither number is presented as more authoritative than its methodology supports.
