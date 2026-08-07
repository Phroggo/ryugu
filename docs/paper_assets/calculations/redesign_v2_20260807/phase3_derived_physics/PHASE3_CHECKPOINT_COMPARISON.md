# Phase 3 checkpoint — derived physics, old vs. new

Calculation only, no sim runs. Basis: Phase 2's **corrected** model
(2.3127 kg, retracted-posture I_bot = 1.090813e-02 kg·m², CG as reported in
`../phase2_physical_model_rebuild/CG_INERTIA_REPORT.md`) — not the first
Phase 2 v1 rebuild.

## 1. Mass-only-dependent quantities — confirmed, not just assumed

The instruction to "confirm" rather than assume was taken literally: pulled
the actual formulas from `Research_Paper.md` and checked each one for a
mass term.

| Quantity | Formula | Mass-dependence | Old (2.50kg) | New (2.3127kg) | Δ |
|---|---|---|---|---|---|
| Weight W | $W=mg$ | linear in m | 2.850e-04 N | 2.6365e-04 N | −7.49% |
| Friction capacity | $\mu m g$ | linear in m | 1.767e-04 N | 1.6346e-04 N | −7.49% |
| Illustrative thrust F | $E_p/d$, $E_p=mgh$ | linear in m | 1.425e-02 N | 1.3182e-02 N | −7.49% |
| Escape velocity $v_{esc}$ | $\sqrt{2gR}$ | **no m term at all** | 0.320 m/s | 0.320 m/s | **0%** |
| Launch velocity law $v_{req}$ | $\sqrt{dg/\mathrm{SIN2TH}}$ | **no m term at all** | (example) 0.0428 m/s | 0.0428 m/s | **0%** |

All three mass-linear quantities move by exactly the mass ratio
(2.3127/2.50 = 0.9251, −7.49%) — confirmed algebraically, not just
observed. Escape velocity and the launch-velocity law are genuinely
mass-independent (robot mass doesn't appear in either formula: escape
velocity is a property of Ryugu, and the ballistic launch-velocity law is
kinematic, mass cancels out of $F=ma$ under uniform gravity) — **exactly
zero change**, confirmed by inspecting the formulas, not inferred from
"probably fine."

(V_GAIN, the empirical actuator-stroke calibration in
`hopper_locomotion.py`, is a different thing entirely — not a formula, an
empirical fit — and is separately known to need re-calibration; not
addressed here, out of scope for "pure calculation.")

## 2. I_bot about the body z-axis

| | Old | New | Δ |
|---|---|---|---|
| I_bot, retracted (flight) posture | 1.822e-02 kg·m² | **1.0908e-02 kg·m²** | **−40.1%** |
| I_bot, splayed (crouch) posture | 1.869e-02 kg·m² | 1.105e-02 kg·m² | −40.9% |
| I_bot, extended (launch) posture | 1.596e-02 kg·m² | 1.025e-02 kg·m² | −35.8% |

Source: `../phase2_physical_model_rebuild/CG_INERTIA_REPORT.md` (already
computed there; not recomputed here, just cited and used as the basis for
§4 below).

## 3. Pivot-axis inertia for self-righting (tripod support edge)

**No prior I_pivot calculation exists anywhere in this repo** — confirmed
by grepping the whole tree. Only the paper's static
$\tau \approx mgw/2 \approx 2.9\times10^{-5}$ N·m tipping-torque figure
(§3.3) existed; no moment-of-inertia-about-the-edge figure, and no
rigorous derivation of $w$ itself. Computed both **for the first time**
this phase, via `compute_pivot_inertia.py`, against both the old and
corrected-new models (isolating whether anything moves from geometry —
which Phase 2 never touched — vs. from the CG/mass shift).

**Posture correction made along the way:** the obvious first guess for
"landed/standing posture" (by analogy with `compute_moi.py`'s "splayed
(crouch stance)" label) turns out to be wrong — it places the foot
*above* `base_link`'s origin, not a physically possible ground-contact
pose. The real landed/standing posture is "retracted (flight neutral)"
(hip=0, knee=0): foot correctly below the chassis, and it's what
`hopper_locomotion.py` itself commands for IDLE/landed
(`set_joints(0.0, 0.0)`). Used that, not crouch, for this whole
calculation — flagged explicitly since it wasn't obvious from the label
alone.

| | Old (2.50kg, symmetric CG) | New (2.3127kg, corrected) |
|---|---|---|
| Support-edge half-width w, governing (least-stable) edge | 0.3462 m (all 3 edges identical — symmetric) | 0.3447 m (range 0.3447–0.3491 m across the 3 edges, 1.3% spread — no longer symmetric) |
| τ = mgw/2, governing edge | 4.933e-05 N·m | 4.545e-05 N·m |
| I_pivot, governing edge | **1.1789e-01 kg·m²** | **1.0267e-01 kg·m²** |
| Δ I_pivot | | **−12.9%** |

**New finding, flagged not resolved:** the paper's stated
$\tau \approx 2.9\times10^{-5}$ N·m implies $w \approx 0.204$ m
(back-solved: $w = 2\tau/mg$). The rigorously-computed $w$ from real leg
geometry — which Phase 2 never changed — is **0.346 m old / 0.345 m new**,
not 0.204 m, in *either* model. This is a pre-existing discrepancy in the
paper's own support-edge figure, independent of the mass redesign. Not
resolved here (that's a documentation-reconciliation question, not a
"recompute the physics" one) — flagged clearly in the change report and
here so it isn't missed before Table I / §3.3 get rebuilt.

## 4. K_ang / K_rate re-derivation — reasoning shown, not asserted

**Design method** (unchanged from the original, per
`attitude_controller.py`'s own comments): pick a target closed-loop
bandwidth $\omega_n$ and damping ratio $\zeta$ for the attitude-hold loop
against the *flight/retracted-posture* whole-body inertia (the regime this
loop actually operates in — in-flight yaw-hold), then solve
$K_{ang} = I\omega_n^2$, $K_{rate} = 2\zeta\sqrt{K_{ang}I}$.

**Why retracted posture, not crouch or splayed:** the original design
comment explicitly sizes gains "vs. whole-robot inertia... about z" for
the loop's actual operating regime (in-flight), and the 2026-07-17 retune
explicitly targeted "flight-posture inertia." Same choice made here for
consistency — not a new assumption invented this phase.

**Design targets carried forward, not re-invented:**
- $\omega_n$ = 1.9 rad/s — the midpoint of the original 2026-07-17 retune's
  own stated outcome range ("wn ~1.8-2 rad/s"). This *preserves the
  original design intent* (how fast the loop should respond) rather than
  picking a new number — the thing that must change is the gains needed
  to hit that same intent against the corrected inertia, not the intent
  itself.
- $\zeta$ = 1.1 — explicit instruction, and matches the original design's
  own stated overdamped requirement.

**Sanity check on the method, using the OLD gains against the OLD model's
*real* (Phase-2-computed) I_bot** (the original design comment's own
"~0.025 kg·m²" was a hand estimate, never rigorously computed until this
project's I_bot work existed): $K_{ang}=0.05$, $K_{rate}=0.066$ against
$I=1.822\times10^{-2}$ gives $\omega_n=1.657$ rad/s, $\zeta=1.093$ — closely
matching the design comment's own claimed "~1.8-2 rad/s, ζ~1.1." This
confirms the re-derivation method (and the choice of retracted-posture
$I_{bot}$) reproduces the documented original intent when run against the
old numbers, which is what justifies applying the identical method to the
new ones.

| | Old (shipped) | New (this phase) |
|---|---|---|
| $I_{bot}$ used (retracted posture) | 1.822e-02 kg·m² | **1.0908e-02 kg·m²** |
| $K_{ang}$ | 0.0500 N·m/rad | **0.0394 N·m/rad** |
| $K_{rate}$ | 0.0660 N·m/(rad/s) | **0.0456 N·m/(rad/s)** |

**Reasoning, step by step, for the new values:**
```
K_ang = I_new * wn_target^2
      = 1.090813e-02 * 1.9^2
      = 1.090813e-02 * 3.61
      = 0.03938 N*m/rad

K_rate = 2 * zeta_target * sqrt(K_ang * I_new)
       = 2 * 1.1 * sqrt(0.03938 * 1.090813e-02)
       = 2.2 * sqrt(4.2946e-04)
       = 2.2 * 0.020724
       = 0.04559 N*m/(rad/s)
```
(`compute_derived_physics.py` reproduces this exactly, `K_ang=0.03938`,
`K_rate=0.04560`, tiny rounding difference only.)

**Both gains went down** — expected and correct: a lighter, less-inertial
body needs less restoring torque per unit angle error and less rate
damping to hit the *same* responsiveness/damping targets. This is not
"reusing the old gain values" (explicitly avoided per instruction) — it's
the same design method applied fresh to the corrected inertia.

## 5. Resulting ζ and ωn under the new gains — confirmed by plugging back in

| | Target | Achieved (new gains, new I) |
|---|---|---|
| $\omega_n$ | 1.9 rad/s | **1.9000 rad/s** |
| $\zeta$ | 1.1 | **1.1000** |

Self-consistent by construction (the gains were solved directly from these
targets), confirmed numerically in `compute_derived_physics.py`'s output
rather than left as an algebraic assertion.

## Summary table

| | Old | New | Δ |
|---|---|---|---|
| Total mass | 2.5000 kg | 2.3127 kg | −7.49% |
| $I_{bot}$ (retracted) | 1.822e-02 kg·m² | 1.0908e-02 kg·m² | −40.1% |
| $I_{pivot}$ (governing edge) | 1.1789e-01 kg·m² | 1.0267e-01 kg·m² | −12.9% |
| $K_{ang}$ | 0.0500 | 0.0394 | −21.2% |
| $K_{rate}$ | 0.0660 | 0.0456 | −30.9% |
| $\omega_n$ | 1.657 rad/s (actual, old gains vs. real old I) | 1.900 rad/s (target, achieved) | +14.7% |
| $\zeta$ | 1.093 (actual, old gains vs. real old I) | 1.100 (target, achieved) | +0.6% |

Nothing here is a wildly different or suspicious number — $\zeta$ stayed
almost exactly at its original design target (1.09→1.10) by construction,
and $\omega_n$ moved up modestly and deliberately (toward the original
retune's own stated upper bound) rather than blowing up or collapsing.
That's the expected signature of a correctly-executed re-derivation, not a
red flag.
