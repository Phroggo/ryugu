# Phase 2 checkpoint — total mass, CG, and full inertia tensor (old vs. new)

Computed by `compute_whole_robot_cg_inertia.py`, which extends
`../../compute_moi.py`'s already-established parallel-axis method (same
posture set, same frame-resolution logic) to the full 3×3 tensor and a
CoM-offset-aware read of `<inertial><pose>` (needed because `base_link` now
has one). Run against both the old model (`model_OLD_pre_phase2_reference.sdf`,
a saved copy of `model.sdf` at the `pre-mass-redesign` tag) and the new,
deployed `model.sdf`, across the same 3 leg postures `compute_moi.py` already
uses, for direct comparison. Full stdout: `compute_whole_robot_cg_inertia_stdout.log`.

**All CG coordinates below are (x, y, z) relative to `base_link`'s own
origin frame** — i.e. `base_link`'s absolute model-frame position (0, 0,
0.25 m) has already been subtracted out. This is the "relative to the
chassis base" frame the Phase 3 pivot-torque calculation needs.

## Total mass

| | Old | New |
|---|---|---|
| Total mass (posture-independent) | **2.5000 kg** | **2.2977 kg** |

Confirmed changed, not a coincidence — matches Phase 1's audited component
sum (≈2.30 kg) closely. The 0.2023 kg (8.1%) reduction is real: legs went
from solid rods (implied density not a real material) to real 1mm-wall
CFRP tubes, the reaction wheels went from an oversized solid disc to a
properly-sized stainless annulus, and several previously-unmodeled or
overweight items (solar panel, avionics/power breakdown) were replaced
with sourced or geometry-computed figures.

## Center of gravity (x, y, z), relative to base_link origin

| Posture | Old CG (m) | New CG (m) |
|---|---|---|
| Retracted (flight neutral) | (−0.00000, +0.00000, −0.00944) | (+0.00172, +0.00039, **−0.01553**) |
| Splayed (crouch stance) | (−0.00000, +0.00000, −0.01233) | (+0.00172, +0.00039, **−0.01643**) |
| Extended (launch release) | (−0.00000, +0.00000, −0.00630) | (+0.00172, +0.00039, **−0.01455**) |

The old model's CG sat essentially exactly on the central z-axis (x, y ≈ 0
to floating-point precision) — an artifact of the old lumped `base_link`
being perfectly symmetric by construction (a uniform box with the same
inertia on all three axes) and the 3-fold leg symmetry canceling in x/y.
The new CG has a small but real x/y offset (≈1.8mm) from off-center
component placement (antenna, cameras, asymmetric leg-motor + battery
positions), and sits **noticeably lower** in z (roughly 60–70% farther
below `base_link`'s origin than before) — driven mainly by the leg
motor+gearhead mass (0.24 kg total) sitting at z ≈ −0.10m while contributing
a larger *fraction* of a lighter total body mass than before.

**This is the number Phase 3's τ ≈ mgw/2 pivot-torque calculation needs**:
the new CG sits lower relative to `base_link`'s origin than the old model
assumed, in every posture. It is not yet expressed relative to the tripod
support-edge plane — that conversion (chassis-frame CG height → height
above the foot-contact plane in a given landed posture) is explicitly
Phase 3's job, not this phase's.

## Full inertia tensor about the CG (kg·m²)

### Old, retracted posture
```
[ 1.865777e-02  -1.301043e-18  -8.131516e-20]
[-1.301043e-18   1.865777e-02   2.168404e-19]
[-8.131516e-20   2.168404e-19   1.821813e-02]
```
(off-diagonals are floating-point noise, ~1e-18 — confirms the old model
really was diagonal/symmetric)

### New, retracted posture
```
[ 1.357790e-02  -5.205546e-05  -2.219215e-04]
[-5.205546e-05   1.388428e-02  -1.126049e-04]
[-2.219215e-04  -1.126049e-04   1.082771e-02]
```
Real, non-trivial off-diagonal terms — the new mass distribution is
genuinely asymmetric (as expected from real component placement), not an
artifact.

Full 3×3 tensors for all 3 postures (old and new) are in
`compute_whole_robot_cg_inertia_stdout.log`; summary diagonal-only view:

| Posture | Old I_zz | New I_zz | Old I_xx=I_yy | New I_xx | New I_yy |
|---|---|---|---|---|---|
| Retracted | 1.822e-02 | 1.083e-02 | 1.866e-02 | 1.358e-02 | 1.388e-02 |
| Splayed | 1.869e-02 | 1.097e-02 | 1.992e-02 | 1.392e-02 | 1.422e-02 |
| Extended | 1.596e-02 | 1.017e-02 | 1.762e-02 | 1.332e-02 | 1.363e-02 |

I_zz dropped by roughly 35-40% across all three postures — driven mostly by
the 8% total-mass reduction plus the legs' mass moving much closer to the
axis (real thin tubes vs. the old, disproportionately heavy solid rods).
**This directly affects the platform-level I_bot figure currently cited in
the paper (§3.2/§7.1, "I_bot = 0.016–0.019 kg·m²," backed by
`../../I_bot_derivation.md`/`compute_moi.py`) — that figure is now stale
and needs recomputing against the new model.sdf.** Flagged here, not fixed:
`compute_moi.py` itself only reports I_zz for the exact 3 postures already
in this table, so re-running it directly (unchanged script, new model.sdf)
would produce the updated paper figure; not done in this phase since
updating the paper's cited number is downstream/documentation work, not
part of rebuilding the physical model itself.

## base_link's own lumped mass/CoM/inertia (component only, not whole robot)

For reference — this is what actually went into `model.sdf`'s
`<inertial>` block for `base_link` itself (before summing in the other 8
links that already have their own bodies):

| | Old | New |
|---|---|---|
| Mass | 1.350 kg | 1.3689 kg |
| CoM (relative to base_link origin) | (0, 0, 0) *(assumed)* | (0.00289, 0.00066, −0.00822) m |
| I_xx | 9.000e-03 | 9.612e-03 |
| I_yy | 9.000e-03 | 9.914e-03 |
| I_zz | 9.000e-03 | 7.529e-03 |
| I_xy | 0 | −5.140e-05 |
| I_xz | 0 | −1.927e-04 |
| I_yz | 0 | −1.064e-04 |

Full derivation: `compute_new_inertial_model.py` /
`compute_new_inertial_model_stdout.log`.
