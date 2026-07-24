# Supporting Calculations — Advisor Review Round 1

Backup derivations for numbers now stated (not just asserted) in the paper.
Cross-referenced to the paper section that uses each result.

## 1. Leg motor torque margin (§3.1, comment #8)

Launch thrust: F = E_p / d = 1.4e-2 N (already derived in §3.1 from the
illustrative 5 m hop energy budget).

Leg segment (thigh/calf) length: 0.15 m (§3.1, "zigzag" leg posture).

Required joint torque ≈ F x lever arm = 1.4e-2 N x 0.15 m = 2.1e-3 N.m = 2.1 mNm.

Motor capacity (Maxon RE 13 through 67:1 GP 13 gearhead [11]): 134 mNm.

Margin = 134 / 2.1 ≈ 63.8x → reported as ">60x".

## 2. V_GAIN functional form (§3.1, comment #11)

Launch ramp duration: T = V_GAIN / v_req.

Justification: the stroke covers a FIXED crouch-to-extension travel
distance regardless of requested speed. Average joint rate over the ramp
(and hence delivered separation velocity) therefore scales as
(fixed travel) / T, i.e. v_req ∝ 1/T, equivalently T ∝ 1/v_req.

V_GAIN is the proportionality constant of that relation (units of length,
an effective stroke-rate calibration length — not a physical stroke
dimension). The *form* T = V_GAIN/v_req follows from this fixed-travel
argument. The *value*, V_GAIN = 0.12 m, is empirically fitted because the
joints' torque-limited tracking response under load is not itself
closed-form — see §3.1's dedicated calibration investigation (the n=7
hop dataset) for how the value was measured.

## 3. Hop-splitting: launch energy vs. total system energy (§5.2, comment #31)

Given: m = 2.5 kg, g = 1.14e-4 m/s^2, SIN2TH = 0.56 (§3.1.1).

E(d) = m*g / (2*SIN2TH) * d

Single 9 m hop:
  E(9) = (2.5 * 1.14e-4) / (2*0.56) * 9
       = 2.85e-4 / 1.12 * 9
       = 2.545e-4 * 9
       = 2.29e-3 J

Three 3 m hops:
  E(3) = 2.545e-4 * 3 = 7.634e-4 J
  3 * E(3) = 2.29e-3 J

→ Identical to the single 9 m hop, confirming the symbolic invariance
result numerically.

### Total system energy counterpoint (the advisor's point: splitting
### costs more real energy, even though launch KE doesn't change)

Per-hop fixed time overhead (already established, §5.2/§3.4/§7):
  - crouch + yaw-align: up to 45 s
  - launch ramp: 1.2-20 s
  - post-landing settle-confirmation: ~14 min (840 s) for a full-stroke
    hop (§7 "confirmed LANDED in ~14 min")

Continuous system power draw during all of this (§4.1 table):
  avionics 2.00 W + reaction-wheel attitude hold 1.50 W = 3.53 W avg.

One 9 m hop, full cycle:
  t ≈ 45 + 20 + 840 = 905 s
  Energy = 3.53 W * 905 s ≈ 3195 J ≈ 3.2 kJ

Three 3 m hops, each incurring its own full cycle (conservative — uses
the same representative overhead per hop since per-distance overhead
scaling isn't separately measured in the dataset):
  3 * 3195 J ≈ 9585 J ≈ 9.6 kJ

Result: ~3x more total system energy for the same 9 m of net travel when
split into three hops, driven entirely by tripling the elapsed time the
continuously-drawing subsystems must stay powered — not by any additional
physical hardware. This is the mechanism behind the advisor's "more
components / more power" concern, restated precisely: it's more *time*
powered, not more *components*.
