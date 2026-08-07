# Phase 4 checkpoint — isolated attitude-control re-validation

Single-variable test: only `model.sdf` (Phase 2 corrected) and
`attitude_controller.py`'s `K_ang`/`K_rate` (Phase 3 re-derived) changed.
Control logic itself untouched.

## Result

| | Original (old model, old gains) | New (Phase 2 model, Phase 3 gains) |
|---|---|---|
| 1ms: final yaw | 106.03° (C13 rerun) / 106.06° (timestep check) | **106.078°** |
| 1ms: time to <1° error | <1° by t+9.3s (C13) / 9.61s (timestep check) | **8.48s** |
| 4ms: final yaw | 106.15° | **106.108°** |
| 4ms: time to <1° error | 8.70s | **8.24s** |
| Overshoot | none reported | **none** (max yaw 106.11°/106.16°, both < target 107°) |
| Steady-state behavior | small jitter, no growth | small jitter (~±0.03-0.05°), no growth |

**Final angle: comparable** — within 0.03-0.08° of the original results, on
the same side of the 107° target (slight undershoot, consistent with the
~1° steady-state deadband the original design already described).

**Convergence time: comparable, modestly faster** — 8.2-8.5s vs. 8.7-9.6s
old. This is expected, not a red flag: Phase 3 deliberately targeted
ωn=1.9 rad/s (vs. the old gains' actual ~1.66 rad/s against the real old
I_bot — see `../phase3_derived_physics/PHASE3_CHECKPOINT_COMPARISON.md`),
so a somewhat faster convergence is exactly what re-deriving the gains for
a lighter, less-inertial body while keeping the same damping ratio should
produce.

**No oscillation** — checked the full trace (`phase4_yaw_slew_results.json`),
not just the summary numbers: max yaw across the entire run never exceeds
the 107° target (106.11° 1ms, 106.16° 4ms), and the tail of both traces
(last ~5s) shows a tight, non-growing jitter band (~±0.03-0.05°, consistent
with quaternion-derived-yaw numerical noise, not real oscillation) —
monotonic-style overdamped settling, matching the ζ=1.1 design target.

## Timestep sensitivity, old vs. new

| | Old model | New model |
|---|---|---|
| 1ms vs 4ms final-angle spread | 106.06° vs 106.15° (0.09°) | 106.078° vs 106.108° (0.03°) |
| 1ms vs 4ms convergence-time spread | 9.61s vs 8.70s (0.91s, 4ms faster) | 8.48s vs 8.24s (0.24s, 4ms faster) |

**The new model is not more timestep-sensitive than the old one — if
anything, slightly less** (tighter spread on both final angle and
convergence time). Same qualitative pattern as before (4ms converges
slightly faster than 1ms in both cases) — consistent, not a new artifact.

## Checkpoint verdict

**PASS.** The re-tuned system converges to a comparable angle
(within ~0.1°) in a comparable time (both within ~1s of the original, and
in the expected direction given the deliberately higher target bandwidth),
with no oscillation, under the new model. Timestep sensitivity was
re-checked as instructed and found unchanged in character.
