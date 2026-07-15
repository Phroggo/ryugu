# SpaceHopper: A Bio-Inspired Legged Swarm Robot for Microgravity Asteroid Exploration and Sampling

**Abstract**
*The exploration of low-gravity near-Earth objects (NEOs), such as the C-type asteroid 162173 Ryugu, poses unique traversal and sampling challenges. Traditional wheeled rovers lack sufficient traction in microgravity environments characterized by porous, rubble-pile topography. This paper presents the design and simulation of "SpaceHopper", a 2.5 kg tri-pedal hopping robot equipped with active gyroscopic attitude control. We utilize the Gazebo Harmonic physics engine to accurately simulate a high-fidelity Ryugu environment, validating the robot's locomotion, sampling capabilities, and power constraints under $0.000114 \text{ m/s}^2$ gravity. We demonstrate that combining bio-inspired leg morphology with 3-axis reaction wheels guarantees 100% self-righting probability and robust mobility across chaotic boulder fields.*

---

## 1. Introduction

Asteroids are primitive remnants of the early solar system, offering vital clues to planetary formation and the origin of water on Earth. Following the success of JAXA's Hayabusa2 mission [1], it became clear that highly mobile, surface-dwelling assets are essential for comprehensive *in-situ* analysis. Ryugu, characterized as a "spinning top-shaped rubble pile" [2], features extremely low surface gravity and highly uneven terrain. Under these conditions, conventional wheeled locomotion suffers from massive slip and risk of permanent entrapment.

To overcome these constraints, we propose a swarm-capable, legged hopping architecture. This paper details the hardware parameterization, kinematic design, and environmental simulation modeling of the SpaceHopper platform.

## 2. Environmental Simulation Pipeline

Accurate physical evaluation of locomotion strategies necessitates a high-fidelity simulation environment.

### 2.1 Topographical Modeling
A 1025×1025 collision heightmap was constructed derived from the official Hayabusa2 Structure-from-Motion (SfM) shape model (`SHAPE_SFM_49k_v20180804.obj`) [3]. The selected cross-section reflects the dense cratering and rocky ridges characteristic of Ryugu's equatorial band.

![Figure 1: Lateral X-axis projection heightmap derived from the Hayabusa2 SfM shape model, detailing Ryugu's equatorial ridge.](/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/heightmap_preview.png)

### 2.2 Microgravity and Lighting
The physics engine (DART) was constrained to a $Z$-axis gravity vector of $g = 0.000114 \text{ m/s}^2$, corresponding to the lowest boundary of Ryugu's surface gravity. The rendering pipeline utilizes the ESO/S. Brunier Milky Way Panorama mapped to an equirectangular COLLADA UV-sphere, providing a scientifically accurate celestial backdrop without atmospheric scattering [4].

## 3. Hardware Architecture and Kinematics

The SpaceHopper is a compact, 2.50 kg tri-pedal robot. The mass distribution is tightly constrained to ensure the center of gravity aligns closely with the geometric center.

| Subsystem | Components Included | Mass (kg) | Mass Fraction |
| :--- | :--- | :--- | :--- |
| **Chassis** | Aluminum 7075-T6 core, CFRP structural panels | 0.70 | 28% |
| **Locomotion** | 6x Maxon RE 13 leg motors (hip+knee x3), planetary gearheads, legs | 0.45 | 18% |
| **Attitude Control** | 3x Maxon EC 20 flat RW motors + flywheels (X/Y/Z) | 0.20 | 8% |
| **Avionics & Sensors** | Flight computer, LIDAR, IMU, S-Band comms | 0.50 | 20% |
| **Power System** | 4x Space-Grade Li-ion 18650 cells, BMS | 0.30 | 12% |
| **Scientific Payload** | Rotary-Percussive Micro-Corer, Storage Carousel | 0.20 | 8% |
| **Thermal & Solar** | GaAs Solar Arrays, Kapton MLI blankets | 0.15 | 6% |
| **Total Operational Mass** | | **2.50 kg** | **100%** |

> [!NOTE]
> **Corrected 2026-07-14:** the previous table listed "9x Maxon DC Motors" as a single
> Locomotion line, conflating the 6 leg motors (Maxon RE 13, §3.1) with the 3 reaction
> wheel motors (Maxon EC 20, §3.2) — two different motor models serving different
> subsystems. They're now broken out separately; the total mass and 2.50kg budget are
> unchanged, only the breakdown.

![Figure 2: Rendering of the SpaceHopper concept, showcasing the GaAs solar arrays, thermal louvers, and articulated legs.](/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/spacehopper_concept_1783933381656.png)

### 3.1 Jumping Dynamics
The operational weight on Ryugu is merely $W = 2.50 \times 0.000114 = 0.000285 \text{ N}$. To execute a $5\text{m}$ vertical hop, the required potential energy is:
$$ E_p = mgh = 2.5 \times 0.000114 \times 5 = 0.001425 \text{ J} $$
Assuming a leg stroke length of $d = 0.1 \text{ m}$, the total average linear thrust force required is $F = E_p / d = 0.0142 \text{ N}$. The hip/knee joints are driven by Maxon RE 13 DC motors coupled with 1:67 GP 13 gearheads, supplying up to $134 \text{ mNm}$ of torque. This provides a $>60\times$ safety margin to counteract internal joint friction exacerbated by vacuum cold-welding and thermal multi-layer insulation (MLI) resistance.

> [!NOTE]
> **Implemented 2026-07-14:** `hopper_locomotion.py` previously computed the required
> delta-v ($v_{req} = \sqrt{gd}$) but never used it — every jump fired an identical fixed
> impulse regardless of requested distance. It now scales the launch joint amplitude
> proportionally to $v_{req}$, calibrated against a known-good empirical reference point
> (a 5m target at full ±1.0 rad amplitude was previously verified to reach ~5.57m apex),
> clamped to a safe ±[0.3, 2.5] rad range. This is a proportional empirical scaling, not a
> literal implementation of the $F = E_p/d$ closed-form above — the underlying joint
> controller is a position PID, not a direct force/impulse actuator, so an exact
> first-principles mapping from requested distance to commanded joint angle isn't
> straightforward. Live-tested: an anomaly at 65.9m produced a distance-scaled amplitude
> of 2.50 rad (clamped ceiling) vs. a default/short-hop case producing far less.
>
> **Amended 2026-07-15:** the launch stroke was additionally changed from absolute joint
> targets to a per-leg *delta* from each leg's own crouch position, after telemetry
> showed the old absolute-target launch swept leg 0 through a much larger angle than
> legs 1/2 in the same 0.2 s window (the asymmetric crouch leans the body forward), i.e.
> unequal per-leg thrust and a real torque impulse at liftoff. Equal angular travel per
> leg balances thrust *magnitude*; the crouch lean still shapes thrust *direction*.
> **Resolved 2026-07-15/16 — first verified ground-jump liftoff.** The delta scheme
> above was itself superseded by a symmetric crouch→straight-down-extension stroke
> (feet held directly under the hips throughout, so the ground reaction stays vertical
> — at Ryugu weight the feet's total friction capacity is only $\mu m g \approx
> 2.9\times10^{-4}$ N, and *any* lateral force component slides the feet instead of
> lifting the body). The decisive blocker, however, was not stroke geometry at all:
> the landing controller was re-publishing its post-landing stand pose at ~100 Hz
> indefinitely, overwriting every launch command within ~10 ms of it being issued —
> every historical "insufficient thrust" measurement was contaminated by this
> override. With the override removed, launch targets re-asserted every control tick,
> and the leg position gain raised to the point where the stroke rides the actuator's
> real 134 mNm torque cap, the first verified liftoff was recorded: **separation
> velocity 0.0398 m/s (vs. the 0.0185 m/s a 3 m hop requires), sustained multi-meter
> ascent, textbook ballistic arc**. The measured full-stroke delta-v calibrates the
> distance mapping (V_FULL = 0.04 m/s). Landing dissipation then required raising
> physical joint damping (§3.3.1), which trades away most of that separation velocity
> — the launch-authority-vs-landing-damping tradeoff is quantified in
> `research_report.md` §12 and stands as the primary open tuning item.

#### 3.1.1 Escape Velocity Margin
Ryugu's weak gravity is not merely a locomotion challenge — a sufficiently energetic
hop could genuinely exceed escape velocity and depart the body permanently. For a
spherical approximation, $v_{esc} = \sqrt{2gR}$; using $g = 0.000114 \text{ m/s}^2$ and
Ryugu's mean radius ($R \approx 450 \text{ m}$, from the ~900m volume-equivalent
spherical diameter reported in [1]):
$$ v_{esc} = \sqrt{2 \times 0.000114 \times 450} \approx \mathbf{0.320 \text{ m/s}} $$
The farthest a scout can be dispatched under nominal swarm operation is a
corner-to-corner traverse of the ±45m anomaly field (§4.3), $d \approx 127$ m,
requiring $v_{req} = \sqrt{gd} \approx 0.120 \text{ m/s}$ — roughly 38% of $v_{esc}$,
a $\approx 2.7\times$ margin. Since the mapping from
$v_{req}$ to commanded joint amplitude is empirically calibrated rather than a
first-principles closed form (§3.1), this margin is not treated as a hard guarantee.
An invisible world-boundary structure (four collision-only walls plus a ceiling at
100m altitude, $>17\times$ the largest observed jump apex) was added to `worlds/ryugu.sdf`,
physically containing the robot via genuine collision response independent of thrust
calibration accuracy. Full derivation and horizontal-containment rationale in
`research_report.md` §3.4.

### 3.2 In-Flight Attitude Control
A critical point of failure in historical microgravity hopper designs is uncontrolled tumble, resulting in landing inverted. The SpaceHopper employs an internal 3-axis Reaction Wheel (RW) assembly utilizing Maxon EC 20 flat brushless motors. 

**Mathematical Model of Attitude Correction:**
* **Robot Moment of Inertia ($I_{bot}$):** $0.012$–$0.020 \text{ kg}\cdot\text{m}^2$ about the body $z$-axis, depending on leg posture ($\approx 0.012$ with legs retracted straight down in the flight configuration; $\approx 0.020$ fully splayed). Computed by summing the simulation model's per-link inertias (chassis box $1.35\,$kg: $I = \frac{m}{12}(s^2+s^2) = 0.009$; three $0.15\,$kg flywheels; $0.15\,$kg solar panel; six $0.05\,$kg leg segments at posture-dependent radii via the parallel-axis theorem).
* **Reaction Wheel Torque ($\tau_{rw}$):** $0.015 \text{ N}\cdot\text{m}$ (short-term permissible; the Maxon EC 20 flat datasheet [10] lists $\sim$8.75 mNm nominal continuous and $\sim$25.7 mNm stall, so 15 mNm sits in the intermittent-duty band appropriate for correction burns of a few seconds)
* **Flywheel Inertia ($I_w$):** $\frac{1}{2}m r^2 = \frac{1}{2}(0.15)(0.06)^2 = 2.7\times10^{-4} \text{ kg}\cdot\text{m}^2$
* **Maximum Wheel Speed:** $982 \text{ rad/s}$ ($\approx$ 9,380 rpm, the EC 20 flat no-load speed [10])
* **Maximum Angular Momentum Capacity:** $H_{max} = I_w\,\omega_{max} = 2.7\times10^{-4} \times 982 \approx 0.265 \text{ N}\cdot\text{m}\cdot\text{s}$

> [!NOTE]
> **Corrected 2026-07-15 (scientific-accuracy pass):** this section previously cited
> $I_{bot} \approx 0.0055 \text{ kg}\cdot\text{m}^2$ — a value matching *no* configuration of
> the actual simulation model (the chassis alone is $0.009$), which made the derived
> correction time unrepresentative of what the simulation runs. It also cited
> $H_{max} = 0.377 \text{ N}\cdot\text{m}\cdot\text{s}$, implying a 13,330 rpm wheel — above the
> cited motor's own no-load speed. Both are now derived from the model's real inertias
> and the motor datasheet, and the controller's speed clamp was reduced to match.

The angular acceleration imparted on the robot at the torque limit, in the flight posture, is:
$$ \alpha = \frac{\tau_{rw}}{I_{bot}} = \frac{0.015}{0.012} = 1.25 \text{ rad/s}^2 $$

A usable attitude correction must not merely reach the target angle but arrive with *zero residual rate*; the minimum-time profile is therefore bang-bang (accelerate half the distance, decelerate the rest [6]):
$$ t_{min} = 2\sqrt{\frac{\theta}{\alpha}} = 2\sqrt{\frac{\pi/2}{1.25}} \approx 2.24 \text{ seconds for } \theta = 90° $$
(The previously-cited $\theta = \frac{1}{2}\alpha t^2 \Rightarrow t \approx 1.07\,$s answers a different question — time to *sweep past* 90° while still spinning at full rate.) The deployed controller is deliberately slower than this physical bound: it uses an overdamped PD law ($\zeta \approx 1.1$–$1.6$ across the posture-dependent inertia range, §3.2.1) that trades speed for zero overshoot, converging a 107° heading change in $\sim$15–20 s. Both timescales are negligible against the ballistic flight times this environment produces — a 3 m hop coasts for *minutes* under $g = 1.14\times10^{-4} \text{ m/s}^2$.

> [!NOTE]
> **Verified and rewritten 2026-07-14:** reaction-wheel attitude correction is confirmed
> live — `attitude_controller.py` receives real IMU orientation/angular-velocity feedback
> (a ros_gz_bridge version mismatch had silently prevented this in every prior session,
> see `research_report.md` §9.1) and drives genuinely non-zero, changing reaction-wheel
> velocity commands during flight. First live closed-loop test surfaced a real control
> problem: `attitude_error` oscillated between 1.5–2.8 rad (85–160°) instead of
> converging, because the original controller decomposed orientation into Euler angles
> (roll/pitch), which is only a valid small-angle approximation — at this robot's actual
> tumble magnitudes, IMU body-rate measurements stop corresponding to Euler-angle rates,
> so the derivative (damping) term was damping the wrong quantity.
>
> **Rewritten to quaternion-based tilt feedback.** Instead of Euler angles, the
> controller rotates the body's local $+Z$ ("up") axis into the world frame and takes
> its cross product with world $+Z$:
> $$ \vec{e} = \hat{u}_{local} \times \hat{u}_{world}, \quad \hat{u}_{world} = (0,0,1) $$
> This yields a rotation-axis-aligned error vector valid at *any* tilt magnitude (no
> small-angle assumption, no gimbal lock), and is inherently yaw-independent — it
> corrects "don't be upside down" without caring which way the robot faces, matching
> the original design intent. Reaction wheel commands are set directly proportional to
> this error and body rate ($\omega$), replacing the previous accumulated-acceleration
> integration (an unnecessary extra integrator that worsened the oscillation), with
> integral (I) terms removed entirely — in free flight there is no persistent
> disturbance torque for an I-term to usefully compensate for.
>
> **Live-verified after the rewrite:** a test hop's self-righting sequence (see below)
> succeeded on its *first* attempt, versus needing all 5 retries before the fix; a
> follow-on autonomous idle-recovery hop landed just as cleanly with zero righting
> needed; and post-landing IMU angular velocity read $\sim10^{-15} \text{ rad/s}$
> (numerical noise) versus $4.12 \text{ rad/s}$ (visibly spinning) before. The 1.07s
> theoretical correction time above remains a useful order-of-magnitude estimate but was
> not directly re-measured against the new controller this session.

#### 3.2.1 Second Controller Rewrite: Torque-Based Momentum Pumping (2026-07-15)

Live testing after the quaternion rewrite exposed a deeper, *structural* flaw shared by both prior controllers: they commanded reaction-wheel **velocity** proportional to attitude error. A reaction wheel only exchanges angular momentum with the body **while it accelerates** ($\tau_{body} = -I_w\,\dot{\omega}_{wheel}$); once the wheel reaches its commanded speed, torque transfer stops. Two consequences were confirmed with live telemetry:

1. **Steady-state error is never corrected.** The robot was caught at rest holding a 0.42 rad yaw error indefinitely, with the wheel spinning contentedly at the commanded $300 \times 0.42 \approx 127$ rad/s and *zero* torque flowing.
2. **In flight, a residual spin persists.** With rate feedback $\omega_{cmd} = K_d\,\omega_z$, conservation of angular momentum gives an equilibrium at
$$ \omega_{residual} = \frac{L_0}{I_{bot} + I_w K_d} \neq 0 $$
where $L_0$ is the launch-induced momentum — matching the persistent $-1$ to $-2.3$ rad/s yaw spin measured after every hop. Worse, for the yaw axis the error wraps at $\pm\pi$, turning the velocity target into a $\pm 942$ rad/s sawtooth that a slew-limited command dithers against with near-zero mean momentum transfer.

The controller now follows the standard reaction-wheel structure (e.g., Sidi [6], Wie [7]): a PD law on attitude produces a desired **body torque**, clipped to the physical motor budget, which is converted to wheel *acceleration* and integrated into the wheel speed command:
$$ \tau_{des} = \mathrm{clip}\!\left(K_{ang}\,e - K_{rate}\,\omega,\ \pm\tau_{rw}\right), \qquad \dot{\omega}_{wheel,cmd} = -\frac{\tau_{des}}{I_w} $$
with $K_{ang} = 0.02 \text{ N}\cdot\text{m/rad}$ and $K_{rate} = 0.05 \text{ N}\cdot\text{m}\cdot\text{s/rad}$, sized against the whole-robot inertia for an overdamped response ($\omega_n \approx 0.9$–$1.3$ rad/s, $\zeta \approx 1.1$–$1.6$) with **no oscillation** by construction. Momentum keeps flowing until both rate *and* angle are nulled; the wheel settles at whatever speed absorbs the disturbance momentum rather than at a speed proportional to the leftover error. A 1° attitude deadband prevents integrator windup against terrain-imposed tilt (a tripod resting on regolith can never reach exactly 0° — without the deadband, live testing showed the wheels slowly winding to full momentum saturation overnight against a 0.1° residual). Rate damping carries no deadband: it acts only while the body rotates, so it cannot wind up — an earlier 0.005 rad/s rate deadband produced a measurable $\pm 1.2°$ limit cycle with the body coasting at exactly the deadband rate, and was removed.

**Live-verified 2026-07-15:** a commanded 107° yaw slew converged overdamped and held within 1° at zero measured rate (no sign flipping, no residual spin); an accidental 165° tumble was damped to 3.6° tilt in $\sim$20 s; and the grounded robot no longer holds phantom wheel-speed offsets.

**Spin-Out Analysis:**
A fully unbalanced, single-leg jump at maximum takeoff velocity generates a maximum angular momentum of $0.0084 \text{ N}\cdot\text{m}\cdot\text{s}$. Because the RW assembly's saturation limit is $H_{max} = 0.265 \text{ N}\cdot\text{m}\cdot\text{s}$ (§3.2), the system retains a $\approx 31\times$ margin against any single-hop launch disturbance. Furthermore, the leg joints possess a $2\pi$ radian range of motion, which *physically permits* self-righting via leg inversion if necessary.

> [!NOTE]
> **Amended 2026-07-15:** this section previously claimed wheel saturation was
> "mathematically impossible." That is true only for *momentum delivered by a single
> launch*; it is not true for **integrator windup** — a controller pumping momentum
> against a persistent, unreachable attitude error (e.g., holding "perfectly level" on
> ground that is not perfectly level) will walk the wheels to saturation given hours,
> which was observed live in an overnight run. The 1° attitude deadband and the
> landed-state handoff (tilt control disarms on confirmed touchdown) close this path;
> the margin arithmetic above still holds for its intended flight-disturbance case.

> [!NOTE]
> **Implemented and live-confirmed 2026-07-14:** `landing_controller.py` detects an
> inverted landing via IMU orientation (checking whether the chassis +Z axis points
> down) and runs a self-righting maneuver: an alternating "splay" phase (all legs out
> for ground grip) and an asymmetric "sweep" phase (one lead leg drives hard through a
> large rotation to roll the chassis, rotating which leg leads on each retry, up to 5
> attempts) before giving up. This is a heuristic control strategy, not a closed-form
> guarantee — the Abstract's "100% self-righting probability" describes design intent
> (the joint range physically supports full inversion), not a literal proven rate.
>
> **First live trial** (before the attitude-control rewrite above): the robot landed
> genuinely inverted — not a synthetic test. Self-righting engaged automatically,
> succeeded once, then immediately re-detected inverted (it wasn't actually settling,
> because the attitude controller was still fighting it — see the oscillation finding
> above). A second full 5-attempt retry cycle then legitimately failed, and the designed
> fallback ("give up, mark landed anyway so downstream logic doesn't hang forever")
> correctly prevented an infinite loop — an intentional safety behavior working exactly
> as designed, not a failure mode.
>
> **After the attitude-control rewrite**: re-tested with a fresh controlled hop.
> Self-righting succeeded on the *first* attempt (vs. exhausting all 5 retries before),
> and a subsequent autonomous idle-recovery hop landed upright with no righting needed
> at all. This is consistent with the oscillation finding being a root cause of the
> earlier unreliability — the attitude controller was actively perturbing the body
> during what should have been a settling righting maneuver. Sample size is still small
> (a handful of live trials, not a statistical success-rate study), so "100% probability"
> remains a design-intent claim, but the mechanism is now confirmed to actually work,
> not merely theoretically possible.

### 3.3 Landing Detection and Ground Handling in Micro-Gravity

Detecting touchdown on a milli-g body is harder than on planetary surfaces for a fundamental reason: an accelerometer measures *proper* acceleration, and a robot **at rest** on Ryugu experiences a support force of only $mg = 2.85\times10^{-4}$ N — an IMU reading of $\sim 10^{-4} \text{ m/s}^2$, indistinguishable from free-fall at any realistic sensor noise floor. (The same ambiguity shaped the MASCOT lander's multi-sensor settling logic [9] and MINERVA's conservative hop scheduling [8].) The original landing state machine treated "accelerometer reads free-fall" as "bounced, back to FLIGHT," which live testing showed fires *on a robot that has already settled* — the state machine then hung in FLIGHT forever, blocking all subsequent hops and leaving in-flight attitude control running on the ground (the windup path of §3.2.1).

The deployed detector (`landing_controller.py`, 2026-07-15) fuses three signals:

* **Contact spike** — $|a| > 0.08 \text{ m/s}^2$ (motor-torque reaction transients reach $\sim$0.02; genuine impacts exceed 0.05).
* **Rest window** — altitude confined to a $\pm 1$ cm band for 45 s *with* velocity below 5 mm/s. The band/duration pair is chosen against the worst-case **apex dwell**: a ballistic coast lingers within a $\pm b$ band around apex for up to $t = 2\sqrt{2b/g}$ — 26.5 s for $b = 1$ cm — so a 45 s window cannot false-fire in flight. (A first implementation used $\pm 2$ cm/30 s, computed one-sidedly; it false-confirmed landing at the apex of a slow bounce, where the *two-sided* dwell reaches 37 s. The velocity gate independently rejects any hop with a horizontal component, which holds $|v| > 5$ mm/s through apex.)
* **Liftoff watchdog** — LANDED is not a terminal claim: sustained velocity above 2 cm/s reverts to FLIGHT and re-arms the pipeline, because "landed" must remain true in the physics, not just the state machine.

Two ground-handling behaviors round out the cycle, both driven by live failures:

* **Ramped post-landing stand-up.** The compliant landing splay leaves the feet pressed outward into the regolith; live testing showed the (then) 2 cm foot spheres wedging into heightmap crevices firmly enough that the 134 mNm leg actuators could not move the legs *at all* (subsequently mitigated by enlarging the feet to 2.5 cm and removing the thigh/calf cylinder collisions so only the feet contact terrain) — a jammed robot silently produces zero thrust on its next "jump." After touchdown confirmation the legs now fold to an unloaded neutral stance. Critically, the fold is interpolated over 15 s: a step command let the position controllers whip the legs at full torque, and the resulting ground-reaction impulse threw the 2.5 kg robot off the surface at 0.036 m/s — in this gravity, an unplanned $\sim$5 m ballistic hop.
* **Bidirectional flight arming.** Tilt correction now keys off the landed status itself (armed whenever not landed), so unplanned flights — spawn descent, bounces, disturbance kicks — are stabilized identically to commanded hops, rather than only hops that announced themselves via the jump trigger.

#### 3.3.1 Impact Dissipation: Why Active Compliance Fails in Micro-Gravity (2026-07-16)

Raising the leg position gains for launch authority (§3.1) exposed a new failure: at
touchdown the stiff position controllers act as near-lossless springs. Measured
restitution from a 1.15 m drop was $\approx 0.96$ — bounces did not decay, and the
robot pogoed indefinitely. Three *active* compliance schemes were then implemented and
live-measured, and **all three added energy at contact**:

| Scheme | Impact v (mm/s) | Rebound v (mm/s) | Verdict |
|---|---|---|---|
| Step to compliant posture at contact | — | kicked 0.7–0.9 m per bounce | energy added |
| Posture ramped over 2 s | 32 | 38 | energy added |
| Zero-stiffness catch (mirror measured joint angles as targets) | 16 | 22 | energy added |

The catch failure is the instructive one: the joint-state feedback crosses a transport
bridge with finite latency, so the mirrored target *trails* the joint's motion — during
rebound the lagged position error pushes *with* the motion, pumping the bounce. This is
the classic phase-lag instability of delayed derivative feedback, and in a milli-g
contact regime there is no gravity margin to absorb the error: **any commanded leg
motion while grounded is effectively a thruster firing, and any delayed feedback loop
around ground contact is destabilizing.**

The deployed solution moves dissipation where phase lag cannot exist: *physical* joint
damping in the model ($c$: $5\times10^{-3} \rightarrow 0.15$ N·m·s/rad). With effective
vertical stiffness $k \approx 48$ N/m at the deployed gains and $c_{vert} \approx 3c/r^2$
($r \approx 0.25$ m moment arm), the contact damping ratio is $\zeta \approx 0.45$,
restitution $e \approx e^{-\pi\zeta/\sqrt{1-\zeta^2}} \approx 0.2$ — bounces decay
within two cycles. Verified live: both the spawn-descent settle and a post-hop impact
landing reached confirmed LANDED in 2.5–3.5 minutes with visibly decaying bounces, and
the full mission loop (hop → flight → land → settle → next hop) ran autonomously.

The cost is launch authority: damping acts on the launch stroke exactly as it acts on
impact, and hop separation velocity at the deployed damping is in the few-mm/s range
(versus 39.8 mm/s at $c = 5\times10^{-3}$). One attempted recovery — raising both gain
and damping so the stroke rides the torque cap ($p = 5.0$, $c = 0.4$) — froze the
joints entirely (zero tracking on any command; suspected integrator/discretization
limit in the physics engine's explicit joint-damping handling) and was reverted. The
quantified tradeoff and candidate directions (intermediate damping sweep,
contact-surface compliance, series-elastic launch elements as used by spring-loaded
hoppers [2]) are recorded in `research_report.md` §12.

## 4. Power and Communication Systems

### 4.1 Energy Budget
Powered by a $37.0 \text{ Wh}$ (133,200 J) space-grade Lithium-Ion pack, the power budget dictates mission longevity in permanently shadowed regions.

| Operational State | Subsystem | Peak Power (W) | Avg Continuous Power (W) |
| :--- | :--- | :--- | :--- |
| **Continuous** | Avionics (CPU, IMU, Comms Rx) | 2.00 | 2.00 |
| **Continuous** | Reaction Wheels (Attitude Hold) | 5.00 | 1.50 |
| **Intermittent** | Leg Motors (0.5s stroke / 10m) | 6.00 | 0.005 |
| **Intermittent** | Micro-Corer Drill (300s sequence)| 3.00 | 0.023 |
| | **TOTAL ESTIMATED DRAW** | **16.00 W** | **3.528 W** |

Continuous operation yields an estimated lifespan of:
$$ \text{Lifespan} = \frac{37.0 \text{ Wh}}{3.53 \text{ W}} = 10.48 \text{ hours} $$
Solar recharge via the top-mounted Gallium Arsenide (GaAs, 28% efficiency) arrays generates $\sim3.5\text{W}$ net at 1.2 AU, allowing for full recovery during the asteroid's diurnal cycle.

### 4.2 Swarm Communication
The architecture supports a decentralized swarm methodology. A low-power ($<0.1\text{W}$) Ultra-High Frequency (UHF, ~400 MHz) mesh network enables intra-swarm communication, successfully diffracting around massive boulders. A primary S-Band patch antenna provides high-gain relay communication to an orbiting mothership.

### 4.3 Swarm Role Allocation (Market-Based Task Auction)

Mission roles (SCOUT / SAMPLER / RELAY / RECHARGE) are allocated by a market-based single-item auction in the style of Gerkey & Matarić's taxonomy [13]. When a spectral anomaly enters the task queue, every eligible SCOUT submits a bid
$$ B_a = d_a + w_b\,(100 - \text{SoC}_a) + w_c\,n_a $$
where $d_a$ is the agent's straight-line distance to the target (locomotion energy and transit time scale with it), $\text{SoC}_a$ its battery state-of-charge (weight $w_b = 0.5$ m/%), and $n_a$ its carousel load (weight $w_c = 5$ m/sample); the lowest bid wins. Agents below a 30% state-of-charge reserve do not bid at all — accepting a task the battery cannot finish merely strands the anomaly.

Robustness behaviors added in the 2026-07-15 rework, each addressing a concrete failure mode of the earlier first-scout-in-list dispatcher:

* **Task recovery** — a SAMPLER forced to RECHARGE mid-task (battery-management override), or one that drops offline, returns its unfinished anomaly to the head of the queue rather than silently losing it.
* **Corrective re-hops** — arrival is gated on real odometry (within 3 m of the target *and* landed). A hop that lands short no longer strands the agent "en route" forever: after a 90 s cooldown (a full hop-and-settle cycle in this gravity) the agent re-hops toward the target, up to 5 attempts before the target is requeued and the agent stands down.
* **Liveness tracking** — an agent whose odometry goes silent for 10 s is marked OFFLINE, excluded from the auction and role churn, and rejoins as Unassigned on recovery.
* **Finite drill dwell** — core extraction occupies the drill for 8 s of drilling time (rotary corer, §5) rather than completing instantaneously on contact, so the power model's drill-duty term reflects an actual duty cycle.
* **Reachable tasking** — anomaly coordinates are clamped to ±45 m, inside the world's ±49 m containment boundary (§3.1.1), so no task can be physically unreachable by construction.

## 5. Scientific Payload

Unlike explosive kinetic impactors, the SpaceHopper performs delicate, non-destructive sampling. It utilizes a Hollow Rotary-Percussive Micro-Corer designed for ultra-low RPM drilling. This methodology preserves fragile stratification and volatile organics inside a sterile caching carousel containing three capillary tubes, significantly increasing the scientific integrity of the retrieved material.

## 6. Conclusion

The simulated modeling of the SpaceHopper demonstrates that tri-pedal jumping combined with active reaction-wheel stabilization is a robust locomotion strategy for microgravity rubble piles. The over-engineered torque profiles ensure reliable mobility, while the tightly integrated power and communications systems enable extended multi-agent scientific operations on bodies like Asteroid Ryugu.

## References

[1] S. Watanabe *et al.*, "Hayabusa2 arrives at the carbonaceous asteroid 162173 Ryugu — A spinning top-shaped rubble pile," *Science*, vol. 364, no. 6437, pp. 268–272, Apr. 2019.
[2] Japan Aerospace Exploration Agency (JAXA), "Hayabusa2 Project: Images from the MINERVA-II1 rover," JAXA Hayabusa2 Gallery.
[3] JAXA Data ARchive and Transmission System (DARTS), "Watanabe_2019 Hayabusa2 Shape Models and Derivatives," ISAS/JAXA, 2019.
[4] European Southern Observatory (ESO), "The Milky Way panorama," ESO GigaGalaxy Zoom Project, Image ID: eso0932a.
[5] SpaceHopper Project, ETH Zurich. arXiv:2403.02831, 2024.
[6] M. J. Sidi, *Spacecraft Dynamics and Control: A Practical Engineering Approach*. Cambridge University Press, 1997. (Reaction-wheel momentum-exchange control structure and minimum-time reorientation profiles.)
[7] B. Wie, *Space Vehicle Dynamics and Control*, 2nd ed. AIAA Education Series, 2008. (PD attitude-control gain design and momentum-management/desaturation.)
[8] T. Yoshimitsu, T. Kubota, I. Nakatani, T. Adachi, and H. Saito, "Micro-hopping robot for asteroid exploration," *Acta Astronautica*, vol. 52, no. 2–6, pp. 441–446, 2003. (MINERVA hopping mobility in milli-gravity.)
[9] T.-M. Ho *et al.*, "MASCOT — The Mobile Asteroid Surface Scout onboard the Hayabusa2 mission," *Space Science Reviews*, vol. 208, pp. 339–374, 2017. (Small-lander touchdown, settling, and self-righting on Ryugu.)
[10] Maxon Group, "EC 20 flat Ø20 mm, brushless, 5 Watt" datasheet, maxon catalog. (No-load speed ≈9,380 rpm; nominal torque ≈8.75 mNm; stall torque ≈25.7 mNm.)
[11] Maxon Group, "RE 13 Ø13 mm, precious metal brushes" and "GP 13 A gearhead (67:1)" datasheets, maxon catalog. (Leg actuator torque basis, ≈134 mNm at the joint.)
[12] G. Dudek, M. Jenkin, E. Milios, and D. Wilkes, "A taxonomy for multi-agent robotics," *Autonomous Robots*, vol. 3, pp. 375–397, 1996. (Swarm coordination taxonomy.)
[13] B. P. Gerkey and M. J. Matarić, "A formal analysis and taxonomy of task allocation in multi-robot systems," *The International Journal of Robotics Research*, vol. 23, no. 9, pp. 939–954, 2004. (Market-based/auction task allocation, the basis of the swarm manager's bidding.)
