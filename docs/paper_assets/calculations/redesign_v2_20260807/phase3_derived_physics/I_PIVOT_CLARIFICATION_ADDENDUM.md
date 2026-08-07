# I_pivot clarification addendum (blocks Phase 5, not Phase 4)

Answers to the two clarification questions raised before Phase 5. Computed
via an extended `compute_pivot_inertia.py` (now posture-parameterized);
full output in `compute_pivot_inertia_stdout.log`.

## 0. Citation check, done first

**Could not find "0.0482 kg·m²" cited anywhere in this project.** Grepped
`Research_Paper.md`, every file in `docs/paper_assets/calculations/`, and
searched every paragraph of the frozen `docs/word/mantis_draft_2.docx` for
"0.0482", "fold/tuck", and "I_pivot" — no match anywhere. Also checked
`HANDOFF.md`, `research_report.md`, `walkthrough.md`, `Study_Guide.md`,
`task.md`, `implementation_plan.md` — nothing. The paper's §3.3 currently
contains only the static $\tau\approx mgw/2\approx2.9\times10^{-5}$ N·m
figure (already found in Phase 3) — no I_pivot number of any kind.

Flagging this directly rather than silently treating the citation as
confirmed: I don't know where "0.0482" comes from, and can't verify it
against a real source. That said, the underlying question (is fold/tuck a
different posture, and does it matter) is real and answerable on its own
merits, computed below regardless of the citation's status. If you have a
specific location for that figure, worth pointing me at it — a real
citation is a *stronger* check than mine, not a redundant one.

## 1. Posture equivalence — genuinely different, confirmed by code

**"Fold/tuck" and "retracted/IDLE" are NOT the same stance.** Confirmed
directly in `landing_controller.py:167-168`:
```python
self.fold_hip_target = 0.33
self.fold_knee_target = -2.6
```
— identical to `hopper_locomotion.py`'s `CROUCH_HIP`/`CROUCH_KNEE`, and
genuinely different from `retracted` (hip=0, knee=0), which is what
Phase 3 used (correctly — confirmed by geometry and by
`hopper_locomotion.py`'s own IDLE-state leg command).

Computed I_pivot for fold/tuck (hip=0.33, knee=-2.6) against both models,
same method as Phase 3:

| | retracted/IDLE (Phase 3) | fold/tuck (this addendum) |
|---|---|---|
| OLD, governing edge | 1.1789e-01 kg·m² | **5.0068e-02 kg·m²** |
| NEW, governing edge | 1.0267e-01 kg·m² | **4.2752e-02 kg·m²** |

Fold/tuck's I_pivot is roughly 2.3–2.4× *smaller* than retracted's — legs
pulled in tight, mass closer to the roll axis, exactly the effect the
tuck is designed for ("makes the body roll like a cylinder"). **This is a
real, large posture effect**, not noise: using the wrong posture for a
given question genuinely changes the answer by more than 2×, which is
close to the magnitude of gap originally described.

**On matching the cited 0.0482 figure specifically:** NEW-model fold/tuck
(4.2752e-02) is 11.3% off 0.0482 — much closer than NEW-model retracted
(1.0267e-01, 113% off, the comparison that produced the original "2.2–2.4×
discrepancy" framing). Close, not exact — plausibly the same posture
computed with a slightly different method or an earlier/rougher model
snapshot, but I can't confirm that without knowing where 0.0482 actually
came from (see §0).

**Important caveat on what fold/tuck-posture I_pivot actually means
physically:** during a real righting roll the body is tipped or inverted,
rolling on the tucked-leg/chassis silhouette — it is not resting on 3
feet on a level surface. "Moment of inertia about the support-triangle
edge" is a standing-stability concept; it isn't really the operative
quantity for the *rolling* dynamics of self-righting itself (that's
closer to a body-frame roll-axis inertia through the CG — an I_bot
variant, not I_pivot). This computation answers the literal question
asked (does fold/tuck change I_pivot, using the same edge-pivot method) —
it is **not** a claim that this is the right physical model for Phase 5's
actual rolling dynamics. That's Phase 5's problem to frame correctly, not
this addendum's.

## 2. Support-edge width w — confirmed, and the margin direction corrected

**Real w (governing/least-stable edge, retracted/standing posture — the
posture the paper's static-stability claim is actually about):**
- Old model: **0.3462 m** (all 3 edges identical — symmetric CG)
- New model: **0.3447–0.3491 m** (range across the 3 edges, no longer
  symmetric)

Both real numbers, **not** the paper's implied w≈0.204m (back-solved from
its stated τ≈2.9e-5). Leg geometry was never touched by Phase 2, so this
gap predates the mass redesign entirely.

**Correction needed on the margin-direction claim:** a larger real w does
**not** make the margin bigger — it's the opposite, and the sign matters
here. $\tau = mgw/2$ is the torque *required* to tip the robot; a larger
$w$ means a **larger** required torque (more leverage needed to tip a
wider-based stance over), which means **less** spare wheel-torque capacity
relative to that requirement, i.e. a **smaller** margin, not larger.

```
Paper's stated figure:  tau ~= 2.9e-05 N*m  ->  margin = 0.015/2.9e-05 ~= 517x
Real (old model):       tau = 4.933e-05 N*m ->  margin = 0.015/4.933e-05 ~= 304x
Real (new model):       tau = 4.545e-05 N*m ->  margin = 0.015/4.545e-05 ~= 330x
```

**The real margin is smaller than stated (≈304–330×, not ≈517×) — an
overstatement that needs correcting downward, not an understatement.**
To be direct about this since it runs opposite to the hypothesis in the
request: I'd double-check this arithmetic independently before treating
it as final, since it's a plain sign/direction correction and worth a
second look rather than taking my word for it. Either way, ≈300–330× is
still an enormous margin by any standard — this doesn't change the
paper's qualitative claim ("overwhelming authority for this task"), only
the specific number.

## Status

Both clarifications answered with real numbers; nothing here has been
written back into the paper or applied to any model/code file — that's
explicitly deferred to the Phase 5/Phase 8 decision the request asked for.
Phase 4 is unaffected (uses I_bot and K_ang/K_rate only) and proceeds
separately.
