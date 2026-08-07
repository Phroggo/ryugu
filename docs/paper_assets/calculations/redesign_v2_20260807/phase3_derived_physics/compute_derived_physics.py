#!/usr/bin/env python3
"""Phase 3: recompute every derived-physics quantity downstream of the
Phase 2 mass-model rebuild, calculation only (no sim runs).

Covers:
  A. Mass-only-dependent quantities (W, thrust, friction capacity) --
     confirmed linear in m, recomputed; escape velocity confirmed
     mass-independent, unchanged.
  B. K_ang/K_rate re-derivation, same design method as the original
     tuning pass (attitude_controller.py's own comments): pick a target
     closed-loop bandwidth omega_n and damping ratio zeta for the
     flight/retracted-posture whole-body inertia, solve K_ang = I*wn^2,
     K_rate = 2*zeta*sqrt(K_ang*I).
  C. Resulting zeta/omega_n under the new gains, confirmed by plugging
     back in (self-consistency check, not just asserted).

I_bot and I_pivot are computed by the sibling scripts in this folder
(compute_whole_robot_cg_inertia.py in ../phase2_physical_model_rebuild/,
compute_pivot_inertia.py here) -- not duplicated here, just cited.
"""
import math

# ---------------------------------------------------------------------------
# A. Mass-only-dependent quantities
G_RYUGU = 1.14e-4      # m/s^2 (Research_Paper.md nomenclature)
MU = 0.62               # foot-regolith friction coefficient (unchanged material property)
RYUGU_RADIUS = 450.0    # m (Research_Paper.md Sec 3.1.1)
LEG_STROKE_D = 0.1      # m, illustrative crouch-to-extension travel (Sec 3.1, "illustrative", unchanged)
HOP_HEIGHT_ILLUSTRATIVE = 5.0  # m, the paper's illustrative upper-bound energy-budget case

M_OLD = 2.50     # kg
M_NEW = 2.3127   # kg, Phase 2 corrected total

print("=== A. Mass-only-dependent quantities ===\n")

for label, m in [("OLD", M_OLD), ("NEW", M_NEW)]:
    W = m * G_RYUGU
    friction_capacity = MU * m * G_RYUGU
    Ep = m * G_RYUGU * HOP_HEIGHT_ILLUSTRATIVE
    F_illustrative = Ep / LEG_STROKE_D
    print(f"{label} (m={m} kg):")
    print(f"  W = m*g              = {W:.4e} N")
    print(f"  friction cap = mu*m*g = {friction_capacity:.4e} N")
    print(f"  F (illustrative, E_p/d) = {F_illustrative:.4e} N")
    print()

delta_pct = (M_NEW - M_OLD) / M_OLD * 100
print(f"All three above scale by exactly the mass ratio: "
      f"{M_NEW}/{M_OLD} = {M_NEW/M_OLD:.4f} ({delta_pct:+.2f}%). Confirmed "
      f"linear-in-m, not just assumed -- W, friction capacity, and the "
      f"illustrative thrust figure all appear as bare m*(...) products in "
      f"the paper's own formulas (Sec 3.1/3.1.1), no other mass-dependent "
      f"term anywhere in them.\n")

v_esc = math.sqrt(2 * G_RYUGU * RYUGU_RADIUS)
print(f"Escape velocity v_esc = sqrt(2*g*R) = {v_esc:.4f} m/s -- "
      f"CONFIRMED mass-independent (robot mass does not appear in the "
      f"formula at all; this is a property of Ryugu, not the robot). "
      f"UNCHANGED by Phase 2: {v_esc:.3f} m/s both before and after.\n")

v_req_9m = math.sqrt(9.0 * G_RYUGU / 0.56)
print(f"Launch velocity law v_req = sqrt(d*g/SIN2TH) (Sec 3.1) also has NO "
      f"mass term -- confirmed mass-independent too (kinematic, not "
      f"dynamic). Example, 9m hop: v_req = {v_req_9m:.4f} m/s, unchanged "
      f"by Phase 2. (V_GAIN itself, hopper_locomotion.py, is a separate "
      f"empirically-fitted actuator-stroke calibration, not this formula -- "
      f"already flagged elsewhere as needing re-calibration against real "
      f"hardware response, independent of this mass-dependence question.)\n")

# ---------------------------------------------------------------------------
# B. K_ang / K_rate re-derivation
print("=== B. K_ang / K_rate re-derivation ===\n")

I_OLD = 0.025   # kg*m^2, the ORIGINAL design's hand-estimated whole-body
                 # inertia ("base 0.009 + legs ~0.012 + panel 0.0008 +
                 # wheels ~0.0006 + drill", attitude_controller.py comment)
                 # -- used for the FIRST-pass gains (K_ang=0.02) only.
K_ANG_OLD_FIRST_PASS = 0.02
K_RATE_OLD_FIRST_PASS = None  # not given directly in the original comment

I_OLD_RETUNE = None  # not stated as an exact number, only "~1.8-2 rad/s"
                       # resulting wn -- back-solved below.
K_ANG_OLD = 0.05     # current shipped value (2026-07-17 retune)
K_RATE_OLD = 0.066   # current shipped value

wn_old = math.sqrt(K_ANG_OLD_FIRST_PASS / I_OLD)
zeta_old_first_pass_placeholder = None
print(f"Original FIRST-PASS design (K_ang={K_ANG_OLD_FIRST_PASS}, hand-estimated "
      f"I~={I_OLD} kg*m^2): wn = sqrt(K_ang/I) = {wn_old:.3f} rad/s "
      f"(matches the attitude_controller.py comment's own ~0.89 rad/s).\n")

# Back-solve what I the 2026-07-17 retune (K_ang: 0.02->0.05) implicitly
# assumed, from its own stated "wn ~1.8-2 rad/s" outcome.
I_implied_lo = K_ANG_OLD / 2.0**2
I_implied_hi = K_ANG_OLD / 1.8**2
print(f"The shipped retune (K_ang={K_ANG_OLD}, K_rate={K_RATE_OLD}) states "
      f"wn~1.8-2 rad/s as its outcome, which implies an assumed I in "
      f"[{I_implied_lo:.4f}, {I_implied_hi:.4f}] kg*m^2 -- close to the OLD "
      f"model's real retracted-posture I_bot (1.822e-02, see Phase 2's "
      f"CG_INERTIA_REPORT.md), i.e. the retune was implicitly using the "
      f"flight/retracted posture, not the crouch posture the original "
      f"0.025 hand-estimate mixed in. This phase makes that choice "
      f"explicit and uses the NEW, rigorously-computed retracted-posture "
      f"I_bot instead of a hand estimate.\n")

zeta_old = K_RATE_OLD / (2 * math.sqrt(K_ANG_OLD * 1.822e-02))
wn_old_actual = math.sqrt(K_ANG_OLD / 1.822e-02)
print(f"OLD gains against the OLD model's real (Phase-2-computed) "
      f"retracted-posture I_bot=1.822e-02 kg*m^2:")
print(f"  wn = sqrt(K_ang/I) = {wn_old_actual:.4f} rad/s")
print(f"  zeta = K_rate/(2*sqrt(K_ang*I)) = {zeta_old:.4f}\n")

# NEW: same design method (target wn, target zeta), solved against the
# corrected Phase 2 retracted-posture I_bot.
I_NEW = 1.090813e-02  # kg*m^2, Phase 2 CORRECTED, retracted (flight-neutral) posture
WN_TARGET = 1.9        # rad/s -- midpoint of the retune's own stated 1.8-2.0 rad/s
                        # target range; SAME design intent (responsiveness) as
                        # before, not a new target invented for this phase.
ZETA_TARGET = 1.1       # explicit instruction: overdamped, zeta ~= 1.1, matching
                        # the original design's own explicit tuning requirement.

K_ANG_NEW = I_NEW * WN_TARGET**2
K_RATE_NEW = 2 * ZETA_TARGET * math.sqrt(K_ANG_NEW * I_NEW)

print(f"NEW design: same method (pick target wn, target zeta; solve "
      f"K_ang=I*wn^2, K_rate=2*zeta*sqrt(K_ang*I)), same target wn "
      f"({WN_TARGET} rad/s, the midpoint of the original retune's own "
      f"stated 1.8-2.0 rad/s range -- preserving design intent, not "
      f"re-litigating it) and target zeta ({ZETA_TARGET}, explicit "
      f"instruction), against the NEW, corrected retracted-posture "
      f"I_bot = {I_NEW:.6e} kg*m^2:")
print(f"  K_ang = I*wn^2 = {I_NEW:.6e} * {WN_TARGET}^2 = {K_ANG_NEW:.5f} N*m/rad")
print(f"  K_rate = 2*zeta*sqrt(K_ang*I) = {K_RATE_NEW:.5f} N*m/(rad/s)\n")

# ---------------------------------------------------------------------------
# C. Confirm resulting zeta/omega_n under the NEW gains (self-consistency)
print("=== C. Resulting zeta/omega_n under the new gains (confirmation) ===\n")
wn_check = math.sqrt(K_ANG_NEW / I_NEW)
zeta_check = K_RATE_NEW / (2 * math.sqrt(K_ANG_NEW * I_NEW))
print(f"Plugging the new gains back in against I_NEW:")
print(f"  wn = sqrt(K_ang/I) = {wn_check:.4f} rad/s  (target was {WN_TARGET})")
print(f"  zeta = K_rate/(2*sqrt(K_ang*I)) = {zeta_check:.4f}  (target was {ZETA_TARGET})")
print(f"  Self-consistent: {'YES' if abs(wn_check-WN_TARGET)<1e-6 and abs(zeta_check-ZETA_TARGET)<1e-6 else 'NO -- CHECK'}\n")

print("=== Summary table ===")
print(f"{'':20}{'OLD':>15}{'NEW':>15}")
print(f"{'I_bot (retracted)':20}{'1.822e-02':>15}{I_NEW:>15.4e}")
print(f"{'K_ang':20}{K_ANG_OLD:>15.4f}{K_ANG_NEW:>15.4f}")
print(f"{'K_rate':20}{K_RATE_OLD:>15.4f}{K_RATE_NEW:>15.4f}")
print(f"{'wn (rad/s)':20}{wn_old_actual:>15.4f}{wn_check:>15.4f}")
print(f"{'zeta':20}{zeta_old:>15.4f}{zeta_check:>15.4f}")
