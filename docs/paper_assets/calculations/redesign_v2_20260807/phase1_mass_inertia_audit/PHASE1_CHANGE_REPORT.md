# Phase 1 — Component-Level Mass and Inertia Audit — Change Report

Repo: `ryugu_v2_ws/src/ryugu_sim` (git). Phase objective: get a real,
sourced mass/inertia number for every subsystem (not just the reaction
wheels), and audit tunable righting/launch constants for stale diagnostic
values. This phase produces the audit table only — it does **not** modify
`model.sdf` or apply any new mass/inertia values to the running sim. That
is explicitly later-phase work.

## 1. Files touched

- `ryugu_sim/landing_controller.py` (modified) — one comment-only edit, no
  behavior change.
- `docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/compute_geometry_masses.py` (new)
- `docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/compute_geometry_masses_stdout.log` (new)
- `docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/AUDIT_TABLE.md` (new) — the deliverable
- `docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/TUNABLE_CONSTANTS_AUDIT.md` (new)
- `docs/paper_assets/calculations/redesign_v2_20260807/phase1_mass_inertia_audit/PHASE1_CHANGE_REPORT.md` (new, this file)

No files deleted. `model.sdf` was **not** touched.

## 2. What changed in each file

### `ryugu_sim/landing_controller.py`
One comment block added at `GENTLE_RIGHTING_SPEED`'s definition (was line
269, now offset by the added comment). No value changed, no logic changed.
The new comment records this phase's finding: the constant is dead code
(confirmed by grepping every use of the name in the file — it's assigned
once and referenced only inside other comments, never read by any live
branch), explains why (superseded by the rev-2 acceleration-integrated
taper), and points to this phase's audit folder. This is the "if it's
touched in any way this phase, flag that change explicitly" requirement —
touched, but only as an annotation; the number itself (20.0, the
un-reverted Phase 0 diagnostic value) is unchanged and inert.

### `compute_geometry_masses.py` (new)
Reproducible script computing every "computed-from-geometry" row in the
audit table: the reaction-wheel annulus (stainless steel, real density,
assumed rim/wall dimensions), the chassis (CFRP skin + 7075-T6 edge frame,
real densities, assumed panel thickness/frame cross-section), and the leg
tubes (CFRP, real density, model.sdf's existing outer radii, assumed wall
thickness). Every assumption is commented in-line and flagged in the
output. Matches this project's existing convention (`compute_moi.py`) of
computing from geometry via a checked-in script rather than hand-derived
numbers.

### `AUDIT_TABLE.md` (new) — the deliverable
One row per component (12 rows, two components split into sub-rows for
leg motor/gearhead and thigh/calf, matching how the components are
physically distinct). Every row has a mass, a source classification
(datasheet / vendor-typical / computed-from-geometry / ESTIMATE), and
either an inertia value or the dimensions needed to compute one, per the
checkpoint requirement. See §4 for the numbers themselves.

### `TUNABLE_CONSTANTS_AUDIT.md` (new)
Full table of every tunable constant in the three controller files that
touches righting or launch logic, its current value, and whether it shows
any diagnostic/temporary marker. One finding (`GENTLE_RIGHTING_SPEED`),
detailed in §4/§5 below.

## 3. What was run this phase

No Gazebo/sim runs this phase — this was a research and computation phase.
What was executed:

| Task | Tool/script | Count | Notes |
|---|---|---|---|
| Web research: real datasheets/vendor specs for 9 subsystem categories | `WebSearch` / `WebFetch` | ~25 queries/fetches across the session | Several official manufacturer pages (maxongroup.com product pages, RS Online, Farnell live product pages) returned bot-block errors or timeouts; mirrored/cached copies of the same datasheets (Scribd, GlobalSpec, a directly-fetched Farnell PDF) were used instead where that happened, and are cited as such rather than presented as if the primary source succeeded cleanly. |
| Geometry-based mass/inertia computation | `compute_geometry_masses.py` | 1 run | Reaction wheel, chassis, thigh, calf — see §4 for output |
| model.sdf inspection | `grep`/`sed` on `models/spacehopper/model.sdf` | — | Confirmed current per-link masses, geometry, and implied densities for every component being audited |
| Tunable-constant grep audit | `grep` across `landing_controller.py`, `attitude_controller.py`, `hopper_locomotion.py` | — | Every `self.CONST = number` assignment checked for diagnostic markers and live-usage |

## 4. Results

### Mass/inertia audit table (full detail in `AUDIT_TABLE.md`)

| Component | Qty | Mass each | Source type |
|---|---|---|---|
| Reaction wheel (annulus, stainless steel) | 3 | 0.185 kg | computed-from-geometry |
| RW motor (Maxon EC 20 flat, 5W) | 3 | 0.022 kg | **datasheet** |
| Leg motor (Maxon RE max 13, 1.2W) | 6 | 0.015 kg | **datasheet** |
| Leg gearhead (Maxon GP13) | 6 | ≈0.025 kg | vendor-listed, inferred by subtraction |
| Leg thigh (CFRP tube) | 3 | 0.0219 kg | computed-from-geometry |
| Leg calf (CFRP tube) | 3 | 0.0143 kg | computed-from-geometry |
| Chassis (CFRP skin + 7075-T6 frame) | 1 | 0.492 kg | computed-from-geometry |
| Flight computer | 1 | 0.094 kg | vendor-typical (ISIS iOBC) |
| Attitude-sensing suite | 1 | ≈0.030 kg | **ESTIMATE** |
| Comms | 1 | 0.094 kg | **datasheet** (EnduroSat UHF Transceiver II) |
| Battery cell (18650) | 4 | 0.0475 kg | **datasheet** (Panasonic NCR18650B) |
| BMS | 1 | ≈0.020 kg | **ESTIMATE** |
| Payload (drill + carousel) | 1 | 0.25 kg | **ESTIMATE, unresolved** — no real comparable found |
| Solar panel | 1 | 0.0152 kg | computed-from-geometry |
| Antenna | 1 | ≈0.015 kg | **ESTIMATE, weakly bounded** |
| Cameras | 3 | 0.010 kg | vendor-typical (OV5640-class) |
| Thermal MLI | 1 | ≈0.098 kg | computed-from-geometry |

Bottom-line sum (sanity check only, not a validated budget): **≈2.30 kg**
against the paper's current 2.50 kg.

### Reaction wheel geometry fix (the "known-wrong" component, per instructions)
Current model.sdf: solid disc, r=60mm, L=20mm, m=0.15kg — **implied density
663 kg/m³, which is not any real structural material.** Real annulus
(stainless steel, 304, ρ=8000 kg/m³), r_outer=50mm, r_inner=42mm, L=10mm:
**m=0.185kg, I_zz(spin)=3.944e-4 kg·m², I_xx=I_yy=1.987e-4 kg·m².** The real
annulus's spin-axis inertia is **46% higher** than the current model's,
despite only 23% more mass — a thin rim concentrates mass at radius more
efficiently than a solid disc of similar outer envelope, so the current
model actually *understates* available righting/pointing authority per
unit mass, not overstates it.

### Open sourcing questions surfaced (mirroring the EC 20 flat variant question)
- **EC 20 flat variant: resolved.** Real datasheet confirms 8 order numbers
  (351005–351008 Hall-sensor / 351054–351057 sensorless, 6/9/12/24V), all
  identical mass (22g) and rotor inertia (5.1 g·cm²) regardless of variant
  — so the mass audit doesn't depend on picking one, but a specific part
  (voltage + Hall vs. sensorless) still needs choosing before procurement.
- **New: "RE 13" vs. "RE max 13" ambiguity.** Table I says "Maxon RE 13."
  Maxon's real catalog has both an "RE 13" line (precision, graphite/
  precious-metal brushes) and a separate "RE max 13" line (cost-optimized).
  Only the RE max 13 datasheet was reachable this session (official RE 13
  pages 403'd). Flagged as unresolved, same category as the EC 20 flat
  question.
- **New: comms band mismatch.** Table I says "S-Band comms." The sourced
  comms part (EnduroSat UHF Transceiver II) and the platform's own
  previously-documented comms model (round-2 sim-chat answers, §10 — "UHF
  mesh diffracts around boulder-scale obstructions") both say UHF, not
  S-Band. This needs an explicit decision, not a silent pick either way.

## 5. Anything that didn't go as planned

- **Official Maxon product pages (maxongroup.com) consistently returned
  bot-block error pages (E1120) to `WebFetch`.** Worked around by finding
  mirrored/cached copies of the same datasheets on Scribd and via a direct
  Farnell-hosted PDF (which rendered correctly as a real, readable
  datasheet page — used for the RE max 13 figures). RS Online and Farnell's
  live product pages also timed out repeatedly; GlobalSpec returned HTTP
  403. None of these failures blocked the audit, but they did shape which
  specific Maxon sub-family (RE max 13 vs. RE 13) ended up sourced — flagged
  above rather than glossed over.
- **Leg gearhead (GP13) mass is not independently sourced.** The 40g
  combined motor+gearhead figure comes from a single distributor listing
  description, not a datasheet; the ≈25g gearhead figure in the table is
  that 40g minus the separately-sourced 15g bare-motor figure — an
  inference, not two independently confirmed numbers. Flagged in the table
  itself (row 3b) rather than presented as equally solid to row 3.
  Attempts to reach the official GP13 datasheet PDF directly also hit the
  same maxongroup.com bot-block.
- **Payload (drill/corer) mass could not be sourced to any real comparable
  hardware this pass.** Searches surfaced drilling *methodology* papers
  (Rosetta SD2, ExoMars, Mars Sample Return prototypes) but no usable mass
  figures for hardware at a remotely comparable scale. This is the single
  weakest row in the table — kept at the current model.sdf placeholder
  (0.25kg) rather than inventing a number, and called out explicitly both
  in the table and here so it doesn't get mistaken for a sourced figure
  later.
- **Antenna mass is only weakly bounded**, not matched to a real comparable
  part — every real UHF antenna product found (210–250g class) is a
  cubesat-bus-scale deployable system, almost certainly the wrong size
  class for this platform's simple fixed antenna. The 15g figure in the
  table is a scale-down guess, explicitly flagged as such.
- **`GENTLE_RIGHTING_SPEED` turned out to be dead code**, not merely a
  stale-but-live diagnostic value as the Phase 0 verifier's framing assumed.
  This is a better outcome than either option posed (revert vs. record as
  open parameter) but meant neither instruction applied cleanly — resolved
  by documenting the actual state precisely rather than forcing it into one
  of the two offered boxes.

## 6. Checkpoint verdict

**Checkpoint (from the phase instructions): "every row has a source.
Nothing is a bare number with no citation."**

**PASS, with two rows explicitly flagged sub-checkpoint quality rather than
silently passed.** All 17 rows in `AUDIT_TABLE.md` carry an explicit source
classification and citation. However, per the phase's own instruction not
to let estimates quietly become real numbers:
- **Payload** (row 8) and **antenna** (row 10) are labeled `ESTIMATE` in
  the table precisely because no real comparable was found — they have a
  *citation trail explaining why they're estimates*, not a citation to a
  real sourced figure. They technically satisfy "nothing is a bare number
  with no citation" (the estimate itself is explained and bounded where
  possible), but they do **not** satisfy the spirit of "get a real number
  from a datasheet," and should not be carried into a rebuilt Table I
  without further work.
- **Attitude-sensing suite** (row 6b) and **BMS** (row 7b) are similarly
  `ESTIMATE`-flagged, order-of-magnitude only.
- Every other row (13 of 17) is either `datasheet`, `vendor-typical`, or
  `computed-from-geometry` with explicitly flagged assumptions where
  dimensions weren't taken directly from `model.sdf`.

The tunable-constant audit's own checkpoint (`GENTLE_RIGHTING_SPEED`
resolved, not left silently ambiguous) is **PASS** — see §4/§5.
