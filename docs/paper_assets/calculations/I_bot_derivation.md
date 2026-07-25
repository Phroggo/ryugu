# Robot Moment of Inertia (I_bot) — Worked Calculation

**Superseded by `compute_moi.py` in this same directory.** This file's
single-static-pose result (0.018218 kg·m², SDF default/rest pose only) was
the first-pass check and is kept below for the record, but it was never a
real posture sweep — it used whatever leg angles happen to be the SDF's
resting default, not the robot's actual commanded postures. `compute_moi.py`
instead rotates each leg through the three joint angles the deployed nodes
actually command (retracted flight-neutral, crouch, launch-extended) and
gives a properly posture-varying range: **0.0160–0.0187 kg·m²**. That range
does *not* reach the paper's previously-stated 0.012 lower bound — the
paper text has been corrected to $I_{bot} = 0.016$–$0.019$ kg·m² (§3.2 and
§7.1) to match what this model.sdf actually produces under commanded
postures, rather than restating an unverified figure.

Original note, retained for the record — backed the claim in §3.2 before
correction: "$I_{bot} = 0.012$–$0.020$ kg·m² about the body z-axis,
posture-dependent (legs retracted vs. splayed), computed from the model's
per-link inertias via the parallel-axis theorem."

**Method:** for each of the 12 links in the deployed
`models/spacehopper/model.sdf`, rotate that link's local inertia tensor
into the body frame (`I_body = R · I_local · R^T`), then add the
parallel-axis shift `m·d_perp²` (`d_perp` = distance from the body's
central z-axis, in the XY-plane). Sum the `zz` component across all
links for `I_bot` about the body z-axis through the model origin.

Reproducible via `word_build/compute_ibot.py`, run directly against the
SDF (script parses XML, does not hand-copy numbers).

**Gotcha caught and fixed:** the three calf links' `<pose>` elements are
declared `relative_to="thigh_N"` (their parent), not the body frame — an
initial pass that read all link poses as body-frame absolute positions
under-counted the calves' true distance from the z-axis and gave
I_bot ≈ 0.0114 kg·m² (just below the reported range). Composing each
calf's pose through its parent thigh's own body-frame transform first
(proper SE(3) chaining) gives the corrected result below.

## Per-link table (SDF default/rest posture)

| link | mass (kg) | x (m) | y (m) | I_zz,own rotated (kg·m²) | d_perp² (m²) | I_zz about origin (kg·m²) |
|---|---|---|---|---|---|---|
| base_link | 1.350 | 0.0000 | 0.0000 | 9.000e-03 | 0.000000 | 9.000e-03 |
| solar_panel | 0.150 | 0.0000 | 0.0000 | 8.100e-04 | 0.000000 | 8.100e-04 |
| rw_x | 0.150 | 0.0000 | 0.0000 | 1.400e-04 | 0.000000 | 1.400e-04 |
| rw_y | 0.150 | 0.0000 | 0.0000 | 1.400e-04 | 0.000000 | 1.400e-04 |
| rw_z | 0.150 | 0.0000 | 0.0000 | 2.700e-04 | 0.000000 | 2.700e-04 |
| drill_link | 0.250 | 0.0000 | 0.0000 | 2.800e-05 | 0.000000 | 2.800e-05 |
| thigh_0 | 0.050 | 0.0700 | 0.0000 | 8.505e-05 | 0.004900 | 3.301e-04 |
| calf_0 | 0.050 | 0.2098 | 0.0000 | 7.907e-05 | 0.044019 | 2.280e-03 |
| thigh_1 | 0.050 | -0.0350 | 0.0606 | 8.505e-05 | 0.004900 | 3.301e-04 |
| calf_1 | 0.050 | -0.1049 | 0.1817 | 7.907e-05 | 0.044019 | 2.280e-03 |
| thigh_2 | 0.050 | -0.0350 | -0.0606 | 8.505e-05 | 0.004900 | 3.301e-04 |
| calf_2 | 0.050 | -0.1049 | -0.1817 | 7.907e-05 | 0.044019 | 2.280e-03 |

**Total mass: 2.500 kg** — exact match to the paper's stated 2.50 kg
operational mass (Table I), an independent cross-check that the model.sdf
used for this calculation is the same one behind every other reported
number.

**Total I_bot (this posture): 0.018218 kg·m²** — inside the paper's
reported 0.012–0.020 kg·m² range, consistent with this particular SDF
default pose being close to a splayed/extended leg configuration (which
pushes calf mass outward, toward the upper end of the range). Retracted
(tucked, per §3.3's tuck-then-deploy righting maneuver) postures pull the
calf links' `d_perp` back down toward the thighs' own ~0.07 m radius,
consistent with the reported lower bound near 0.012 kg·m².
