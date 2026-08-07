# Phase 2 checkpoint — total mass, CG, and full inertia tensor

**Updated 2026-08-07 for the battery/S-Band-antenna correction pass** — now
a 3-way comparison (original pre-redesign model, the first Phase 2 rebuild,
and this corrected rebuild), since the correction moved real numbers and
the checkpoint requires seeing how much.

Computed by `compute_whole_robot_cg_inertia.py`, which extends
`../../compute_moi.py`'s already-established parallel-axis method (same
posture set, same frame-resolution logic) to the full 3×3 tensor and a
CoM-offset-aware read of `<inertial><pose>`. Run against three saved
`model.sdf` snapshots: `model_OLD_pre_phase2_reference.sdf` (the
`pre-mass-redesign` tag), `model_PHASE2_v1_reference.sdf` (commit
`65c541a`, the first Phase 2 rebuild — battery mislabeled, S-Band antenna
silently zero), and the current, corrected `models/spacehopper/model.sdf`.
Full stdout: `compute_whole_robot_cg_inertia_stdout.log`.

**All CG coordinates below are (x, y, z) relative to `base_link`'s own
origin frame.**

## What the correction actually changed

- **Battery:** confirmed 4× Li-ion 18650 (Table I) is the intended design
  — real, sourced, form-factor-specific part, vs. an unspecified generic
  "Ni-MH" label with no real part behind it anywhere in the project. Fixed
  `swarm_manager.py`'s comments/log strings (3 sites) and
  `generate_detailed_spacehopper.py`'s `battery_visuals()` (12 placeholder
  cells → 4 real-18650-dimensioned cells) to match. **Zero mass/inertia
  impact** — Phase 1's audited figure (4×47.5g) and Phase 2's first
  rebuild were already using the correct Li-ion mass; only the code
  labels and an already-dead-code visual function were wrong. Also
  confirmed `battery_visuals()` was never actually invoked anywhere
  (dead code, independent of the chemistry bug) — left unwired
  deliberately, since the cells sit inside the opaque hull box and
  wouldn't render either way; see the function's own docstring.
- **Antenna:** added the previously-silently-zero S-Band patch antenna
  (≈15g, ESTIMATE — see `AUDIT_TABLE.md` row 10b), at its real existing
  visual position. **+0.015 kg**, the only real mass change this pass.

## Total mass

| | Original (pre-redesign) | Phase 2 v1 (first rebuild) | Phase 2 corrected |
|---|---|---|---|
| Total mass | 2.5000 kg | 2.2977 kg | **2.3127 kg** |
| Δ vs. v1 | | | **+0.0150 kg** (the S-Band antenna, exactly) |

## Center of gravity (x, y, z), relative to base_link origin

| Posture | Original | Phase 2 v1 | Phase 2 corrected |
|---|---|---|---|
| Retracted | (−0.00000, +0.00000, −0.00944) | (+0.00172, +0.00039, −0.01553) | (**+0.00145, +0.00000, −0.01477**) |
| Splayed | (−0.00000, +0.00000, −0.01233) | (+0.00172, +0.00039, −0.01643) | (**+0.00145, +0.00000, −0.01567**) |
| Extended | (−0.00000, +0.00000, −0.00630) | (+0.00172, +0.00039, −0.01455) | (**+0.00145, +0.00000, −0.01380**) |

The S-Band patch antenna sits at (−0.04, −0.06, 0.101) — its y-offset
partially cancels the UHF whip antenna's y-offset (0.06, 0.06), which is
why the corrected model's CG y-component moved to essentially exactly
zero (from +0.00039). The x-offset shrank slightly (+0.00172 → +0.00145)
for the same reason. z barely moved (~1mm shift, less negative — the
antenna sits above `base_link`'s origin, pulling CG very slightly upward
relative to v1). **This correction is a second-order refinement on top of
Phase 2's real, first-order finding — the CG is still meaningfully lower
than the original model assumed, in every posture, by roughly the same
margin as the first rebuild found.**

## Full inertia tensor about the CG (kg·m²) — retracted posture

### Phase 2 v1 (first rebuild)
```
[ 1.357790e-02  -5.205546e-05  -2.219215e-04]
[-5.205546e-05   1.388428e-02  -1.126049e-04]
[-2.219215e-04  -1.126049e-04   1.082771e-02]
```

### Phase 2 corrected
```
[ 1.383516e-02  -9.000000e-05  -1.490323e-04]
[-9.000000e-05   1.411245e-02  -8.000000e-06]
[-1.490323e-04  -8.000000e-06   1.090813e-02]
```

Diagonal terms increased slightly (the added antenna mass sits off-axis,
contributing a small parallel-axis term); off-diagonal terms changed
shape (Ixz shrank, Iyz shrank toward zero) reflecting the more
symmetric x/y mass distribution now that both antennas are accounted for.
Full 3×3 tensors for all 3 postures, all 3 models, are in
`compute_whole_robot_cg_inertia_stdout.log`.

| Posture | Original I_zz | Phase 2 v1 I_zz | Phase 2 corrected I_zz |
|---|---|---|---|
| Retracted | 1.822e-02 | 1.083e-02 | **1.091e-02** |
| Splayed | 1.869e-02 | 1.097e-02 | **1.105e-02** |
| Extended | 1.596e-02 | 1.017e-02 | **1.025e-02** |

I_zz moved up by <1% from v1 to corrected — this correction pass is a
minor refinement (+15g out of 2.3kg), not a repeat of Phase 2's original
large swing (which came from the mass-budget rebuild itself, not from
this fix). **The paper's currently-cited I_bot figure
(0.016–0.019 kg·m²) is still stale against either Phase 2 version** — that
finding from the first Phase 2 rebuild is unchanged by this correction.

## base_link's own lumped mass/CoM/inertia (component only, not whole robot)

| | Phase 2 v1 | Phase 2 corrected |
|---|---|---|
| Mass | 1.3689 kg | **1.3839 kg** |
| CoM (relative to base_link origin) | (0.00289, 0.00066, −0.00822) m | (**0.00243, 0.00000, −0.00703**) m |
| I_xx | 9.612e-03 | 9.844e-03 |
| I_yy | 9.914e-03 | 1.012e-02 |
| I_zz | 7.529e-03 | 7.611e-03 |
| I_xy | −5.140e-05 | −9.000e-05 |
| I_xz | −1.927e-04 | −1.232e-04 |
| I_yz | −1.064e-04 | −8.100e-06 |

Full derivation: `compute_new_inertial_model.py` /
`compute_new_inertial_model_stdout.log`.
