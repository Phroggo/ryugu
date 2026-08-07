# Phase 1 deliverable — Component mass & inertia audit

One row per component. **Source** column: `datasheet` (real vendor PDF, fetched
and read this phase), `vendor-typical` (real named commercial part, comparable
class of hardware, cited), `computed-from-geometry` (real material density ×
stated dimensions — dimensions taken from model.sdf where marked, otherwise an
explicit engineering **ASSUMPTION**, flagged inline), or `ESTIMATE` (no real
comparable found this pass — flagged prominently, not to be read as sourced).

Current `model.sdf` figures are given alongside every row for comparison —
this table does **not** modify `model.sdf`; it is the audited reference for a
later phase to apply.

| # | Component | Qty | Mass (each) | Source | Inertia / key dims | Current model.sdf |
|---|---|---|---|---|---|---|
| 1 | **Reaction wheel** (annulus) | 3 | **0.185 kg** | computed-from-geometry: stainless steel (304, ρ=8000 kg/m³) [1]; r_outer=50mm, r_inner=42mm, L=10mm — outer envelope taken from current model, wall/length are **ASSUMPTIONS** (no COTS part exists for this bespoke wheel) | I_zz(spin)=3.944e-4 kg·m², I_xx=I_yy=1.987e-4 kg·m² | Solid disc (WRONG geometry), m=0.15kg, I_zz=2.70e-4 kg·m². Implied density 663 kg/m³ — not a real material. |
| 2 | **RW motor** (Maxon EC 20 flat, 5W) | 3 | **0.022 kg** | **datasheet** [2] (Maxon catalog, mirrored PDF, read directly) | Rotor inertia 5.1e-7 kg·m² (5.1 g·cm²), max speed 15,000 rpm | Not modeled as a separate link (lumped into base_link) |
| 3 | **Leg motor** (Maxon RE max 13, 1.2W) | 6 | **0.015 kg** | **datasheet** [3] (Maxon catalog "RE max 13" 1.2W, order no. 203890 family, read directly) | Rotor inertia ≈2.9–3.6e-8 kg·m² (0.29–0.36 g·cm², varies slightly by voltage winding) | Not modeled as a separate link |
| 3b | **Leg gearhead** (Maxon GP13 planetary) | 6 | **≈0.025 kg** | **vendor-listed, inferred**: combined motor+gearhead assembly listed at 40g [4]; gearhead mass = 40g − 15g (row 3) bare motor. Not a discrete datasheet figure — official GP13 PDF was inaccessible this pass (bot-blocked). **Flagged for re-sourcing.** | Not sourced this pass | Not modeled as a separate link |
| 4 | **Leg structure** (thigh, CFRP tube) | 3 | **0.0219 kg** | computed-from-geometry: CFRP (ρ=1600 kg/m³, mid-range of 1400–1900 kg/m³ [5]); r_outer=15mm (from model.sdf visual), r_inner=14mm (1mm wall — **ASSUMPTION**), L=150mm (from model.sdf) | See script | Solid cylinder, 0.05kg (implied density 472 kg/m³ — also not a real material) |
| 4b | **Leg structure** (calf, CFRP tube) | 3 | **0.0143 kg** | Same method as row 4; r_outer=10mm (model.sdf), r_inner=9mm (**ASSUMPTION**), L=150mm (model.sdf) | See script | Solid cylinder, 0.05kg |
| 5 | **Chassis** (CFRP skin + 7075-T6 frame) | 1 | **0.492 kg** | computed-from-geometry: 7075-T6 (ρ=2810 kg/m³ [6]) + CFRP (ρ=1600 kg/m³ [5]); 0.2m cube envelope from model.sdf; 1mm CFRP skin + 4×4mm Al edge frame — both cross-sections are **ASSUMPTIONS** (no structural CAD exists yet) | See script | Solid-looking 0.2×0.2×0.2m box visual, m=1.35kg **lumped with avionics/power/leg-motors/RW-motors** — not a chassis-only figure |
| 6a | **Flight computer** | 1 | **0.094 kg** | **vendor-typical**: ISIS iOBC, real cubesat on-board computer [7] | — | Lumped into base_link |
| 6b | **Attitude-sensing suite** (IMU + sun/star sensing) | 1 | **≈0.030 kg** | **ESTIMATE** — order-of-magnitude only. Real miniaturized cubesat star trackers run "dozens of grams" [8] and MEMS IMU boards a few grams each; no single named part matches the paper's generic "onboard attitude-sensing suite" description. **Needs a specific part chosen before this stops being an estimate.** | — | Lumped into base_link |
| 6c | **Comms** | 1 | **0.094 kg** | **datasheet/vendor**: EnduroSat UHF Transceiver II, dry mass 0.094 kg [9]. **Flag: Table I currently says "S-Band comms"; this sourced part and the platform's own comms-model finding (round-2 sim-chat, §10) both describe UHF. This mismatch needs resolving, not silently picking one.** | — | Lumped into base_link |
| 7a | **Battery cells** (18650) | 4 | **0.0475 kg** | **datasheet**: Panasonic NCR18650B, max weight 47.5g [10] | 3400 mAh, 3.7V nominal | Not modeled as a separate link |
| 7b | **BMS** | 1 | **≈0.020 kg** | **ESTIMATE**. Cubesat BMS mass budgets typically run under 100g for a full multi-string pack [11]; 20g is a reasoned fraction for a single 4-cell string, not a specific named part. | — | Not modeled as a separate link |
| 8 | **Payload** (rotary-percussive micro-corer + carousel) | 1 | **0.25 kg** | **ESTIMATE, unresolved**. No comparable real hardware mass was found this research pass (searches returned drilling *principles* for asteroid/planetary corers — SD2, ExoMars drill — not mass figures). Kept at the current model.sdf value as a placeholder. **This is the weakest-sourced row in the table and should be revisited before Table I is rebuilt.** | — | Solid cylinder, m=0.25kg |
| 9 | **Solar panel** | 1 | **0.0152 kg** | computed-from-geometry: GaAs triple-junction areal mass ≈0.47 kg/m² (mid-range of 0.4–0.54 kg/m² from EnduroSat panel data and flexible-ELO-cell literature [12]); panel area 0.18×0.18m = 0.0324 m² (from model.sdf) | — | Flat box, m=0.15kg — ~10× the geometry-based figure |
| 10 | **Antenna** | 1 | **≈0.015 kg** | **ESTIMATE, weakly bounded**. Real comparable products found (EnduroSat 2U deployable UHF antenna 210g [13]; a dual-band UHF patch design at 250g) are cubesat-bus-scale deployable systems, almost certainly oversized for this platform's fixed/simple antenna. 15g is a rough scale-down, not a matched part. **Needs a specific small-antenna comparable.** | — | Visual only, 0 kg |
| 11 | **Cameras** (stereo hazcams ×2 + navcam ×1) | 3 | **0.010 kg** | **vendor-typical**: OV5640-class miniature board camera module, ≈10g [14] | — | Visual only, 0 kg |
| 12 | **Thermal MLI** | 1 (blanket) | **≈0.098 kg** | computed-from-geometry: JPL standard 15-layer MLI areal mass 0.6–0.7 kg/m² [15]; coverage area 0.15 m² — **ASSUMPTION** (≈70% of the chassis's 0.24 m² outer surface, not a wrap pattern that's been designed) | — | Visual only, 0 kg |

## Bottom-line sanity check (not a rebuilt Table I — that's later-phase work)

Summing every row × quantity gives **≈2.30 kg**, against the paper's current
2.50 kg total. This is a rough cross-check, not a validated mass budget: two
rows (payload, antenna) are explicitly unresolved estimates, the chassis
figure depends on assumed panel/frame cross-sections, and nothing here has
been reconciled against the platform's actual required strength/stiffness or
power budget. Treat "≈2.30 kg is in the right ballpark" as mildly reassuring,
not as validation.

## Sources

1. NASA GSFC CubeSat Symposium, "Reaction Wheel for CubeSat Attitude Control" (stainless-steel/tungsten flywheel design practice); ASPINA, "Reaction wheel for satellite and CubeSat."
2. Maxon Group, EC 20 flat Ø20mm brushless 5W catalog datasheet (order nos. 351005–351008 Hall-sensor, 351054–351057 sensorless), mirrored copy read directly this phase.
3. Maxon Group, "RE max 13 Ø13mm, Precious Metal Brushes CLL, 1.2 Watt" catalog datasheet (order no. family 201352/203881–203894), read directly this phase (Farnell-hosted PDF).
4. DC Motor Shop, "Maxon DC Motor RE13 Micro 13mm Coreless Planetary Gearbox Gear Motor" listing (40g combined assembly).
5. Multiple materials references (ScienceDirect, chinacarbonfibers.com) on structural CFRP density range.
6. matmatch.com / theworldmaterial.com, 7075-T6 aluminum alloy density.
7. ISIS – Innovative Solutions In Space, iOBC On-Board Computer product page (satcatalog.com mirror).
8. General cubesat ADCS/star-tracker miniaturization literature (NASA SST State-of-the-Art GNC survey).
9. EnduroSat, UHF Transceiver II product datasheet (satcatalog.com/orbitaltransports.com mirrors).
10. Panasonic, NCR18650B cell datasheet (orbtronic.com / alldatasheet.com mirrors).
11. CubeSat BMS design-challenges review (Springer, Discover Applied Sciences).
12. EnduroSat solar panel product masses; triple-junction flexible-cell (ELO) areal-mass literature.
13. EnduroSat, UHF Antenna 2U product page.
14. OV5640-class USB/board camera module commercial listings.
15. JPL standard 15-layer MLI blanket areal-mass reference (science.gov / general spacecraft thermal literature).

## Tooling note

Several official manufacturer pages (maxongroup.com product pages, RS
Online/Farnell distributor product pages) returned bot-block or timeout
errors when fetched directly this session. Where that happened, the number
used is from a mirrored/cached copy of the same datasheet (Scribd, GlobalSpec)
or a distributor listing, cited as such above rather than silently presented
as if the primary source had been reached cleanly.
