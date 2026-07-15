# Ryugu Simulation Environment: Research Report

This document serves as a comprehensive log of the steps taken, assets used, and methodologies employed to construct the asteroid simulation environment. It is intended to be used as a reference for academic documentation and research papers.

## 1. Asteroid Surface Construction (Regolith Plane)

### 1.1 Methodology
To simulate a realistic asteroid environment for robotic navigation, a massive flat plane was replaced with a `Gazebo Heightmap` (`<heightmap>` geometry). This allows the physics engine (DART) to accurately calculate collisions and uneven terrain traversal (craters, rocks, bumpiness) which is critical for evaluating swarm robot mobility in microgravity.

### 1.2 Surface Topography (Heightmap)
The heightmap was derived from the official JAXA Hayabusa2 Structure-from-Motion (SfM) shape model of asteroid 162173 Ryugu, published as supplementary data to Watanabe et al. (2019) [6]. The specific model used is `SHAPE_SFM_49k_v20180804.obj`, a 49,152-polygon Wavefront OBJ file containing 25,350 vertices in the asteroid body-fixed reference frame (coordinates in km).

**Data Source and Citation:**
- **File:** `SHAPE_SFM_49k_v20180804.obj` (2.5 MB)
- **Repository:** JAXA DARTS (Data ARchive and Transmission System), Hayabusa2 Watanabe_2019 archive [7]
- **URL:** `https://darts.isas.jaxa.jp/pub/hayabusa2/paper/Watanabe_2019/`
- **Method:** Structure-from-Motion (SfM) using Agisoft PhotoScan 1.4, from Optical Navigation Camera (ONC) imagery

**Conversion Pipeline:**
1. The OBJ file was parsed to extract all vertex positions (X, Y, Z in km).
2. The equatorial +X hemisphere (X > 0) was selected, representing a prominent cross-section of Ryugu's characteristic equatorial ridge.
3. The Y and Z coordinates were used as ground-plane positions, with X as elevation (projecting from the side).
4. Scipy's `griddata` with linear interpolation was used to project the irregular 3D point cloud onto a regular 1025×1025 grid (2^10 + 1, required by Gazebo's heightmap geometry).
5. Regions outside the convex hull of the data were filled using nearest-neighbor interpolation.
6. The result was normalized and saved as a 16-bit PNG (65,536 gray levels) for maximum depth precision.
7. In Gazebo, the geometry is mapped to `<size>100 100 5</size>`, flattening the massive ridge into a massive walkable terrain map.

![Heightmap derived from the official JAXA Hayabusa2 SfM shape model (SHAPE_SFM_49k_v20180804.obj). This represents a lateral X-axis projection of Ryugu's equatorial ridge. Brighter regions indicate higher elevation (the ridge peak); darker regions indicate lower elevation.](/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/heightmap_preview.png)

### 1.3 Surface Texture (Diffuse Map)
To provide realistic visual rendering while avoiding data artifacts:
- **Source Selection:** Initially, a genuine surface image (`ryugu_heightmap.png`) was used. However, testing revealed it contained a baked-in scientific calibration grid (black dots), which disrupted the immersion of the simulation environment.
- **Resolution and Scaling:** A photorealistic, seamless, tileable regolith texture was synthesized and applied as the diffuse map. To prevent visual degradation and blurriness at close proximities (rover camera scale), the texture scaling parameter (`<size>`) in the heightmap definition was optimized from 5 meters to 1 meter. This creates a dense tiling effect that preserves high-fidelity pebble and dust details when observed at ground level.

### 1.4 References
[1] Japan Aerospace Exploration Agency (JAXA), "Hayabusa2 Project: Images from the MINERVA-II1 rover," JAXA Hayabusa2 Gallery. [Online]. Available: https://www.hayabusa2.jaxa.jp. [Accessed: Jul. 7, 2026].
[2] Open Robotics, "Gazebo Simulator (Ignition) SDF 1.8 Specification: Heightmap Geometry," Gazebo Documentation. [Online]. Available: https://gazebosim.org. [Accessed: Jul. 7, 2026].
[6] S. Watanabe *et al.*, "Hayabusa2 arrives at the carbonaceous asteroid 162173 Ryugu — A spinning top-shaped rubble pile," *Science*, vol. 364, no. 6437, pp. 268–272, Apr. 2019, doi: 10.1126/science.aav8032.
[7] JAXA Data ARchive and Transmission System (DARTS), "Watanabe_2019 Hayabusa2 Shape Models and Derivatives," ISAS/JAXA. [Online]. Available: https://darts.isas.jaxa.jp/pub/hayabusa2/paper/Watanabe_2019/. [Accessed: Jul. 7, 2026].

---

## 2. Space Environment Background (Skybox)

### 2.1 Motivation
A realistic simulation environment requires not only accurate surface topography but also a visually faithful celestial background. Asteroid 162173 Ryugu is a Near-Earth Object (NEO) of the Apollo group, orbiting between Earth and Mars with a semi-major axis of approximately 1.19 AU [3]. From its surface, the observable sky would consist of the Milky Way galactic band, dense starfields, and distant solar system bodies — all unobscured by any atmosphere.

### 2.2 Background Image Source
To ensure scientific accuracy, we utilized a genuine astronomical observation rather than a synthetic rendering:
- **Source:** The ESO/S. Brunier Milky Way Panorama (catalog ID: `eso0932a`), a 360-degree equirectangular composite photograph of the full celestial sphere captured by the European Southern Observatory (ESO) [4]. This image was chosen because the starfield visible from Ryugu's orbit in the inner solar system is astronomically identical to that observed from Earth, differing only in the absence of atmospheric scattering and extinction.
- **Resolution:** The source image measures 6000 × 3000 pixels in equirectangular projection.

### 2.3 Rendering Pipeline
Gazebo Harmonic's Ogre2 rendering engine was found to have two critical limitations: (i) the SDF PBR material pipeline (`<albedo_map>` in `<pbr><metal>`) fails to resolve texture file paths, and (ii) primitive sphere geometry strictly enforces backface culling, preventing inside-out rendering required for a skydome. 

To bypass both limitations, we employed a procedurally generated COLLADA (`.dae`) sphere mesh:
- A UV-sphere with 64 longitude segments × 32 latitude rings (2,145 vertices, 8,192 double-sided triangles) was generated programmatically at a radius of 500 meters.
- **Double-sided triangles** were emitted (both clockwise and counter-clockwise winding orders) to guarantee visibility from inside the sphere regardless of Ogre2's culling behavior.
- **Equirectangular UV mapping** was applied directly: each vertex's texture coordinates are derived from its spherical coordinates as `u = φ / 2π` and `v = θ / π`, where `φ` is the azimuthal angle and `θ` is the polar angle. This maps the ESO panorama seamlessly around the sphere without requiring cubemap face conversion.
- The texture is referenced via the COLLADA `<library_images>` and `<profile_COMMON>` material pipeline (loaded through Assimp), completely bypassing Gazebo's broken SDF material system. Materials were configured with both `<emission>` and `<diffuse>` texture bindings to ensure self-illumination regardless of the scene's directional lighting.

---

## 3. Simulation Physics

To accurately evaluate the mobility of the swarm robots, the simulation physics must closely match the environmental conditions of Asteroid Ryugu.

### 3.1 Microgravity
Ryugu has an extremely weak gravitational field. The surface gravity varies depending on latitude due to the asteroid's rapid rotation and non-spherical shape (the centrifugal force counteracts gravity more at the equator). According to the JAXA gravitational model based on the SfM shape and assuming a bulk density of 1.19 g/cm³, the surface gravity ranges from 0.114 to 0.146 mm/s² [6]. 
- **Implementation:** The world `<gravity>` parameter was set to exactly `-0.000114` m/s² on the Z-axis, simulating the lowest boundary of Ryugu's surface gravity (most challenging for traction and bouncing).

### 3.2 Illumination and Vacuum Environment
There is no atmosphere on Ryugu, which means there is no atmospheric scattering (Rayleigh scattering) to provide ambient light to areas in shadow.
- **Implementation:** The world `<ambient>` lighting was reduced to a near-zero value (`0.01 0.01 0.01`). The primary light source is a single directional `<light>` simulating solar irradiance. This creates the scientifically accurate harsh, pitch-black shadows characteristic of space photography.

### 3.3 Surface Restitution (Bounciness)
Ryugu's surface is a rubble pile composed of highly porous, loose regolith. When landers (like MASCOT) or rovers (like MINERVA-II1) impact the surface, the loose gravel absorbs a significant portion of the kinetic energy, resulting in a low coefficient of restitution [3].
- **Implementation:** The collision `<surface>` physics for the regolith plane includes a `<bounce>` profile with a `<restitution_coefficient>` of `0.15`. This critically dampens collisions and affects how the hopping rovers rebound, accurately reflecting the energy dissipation observed during the Hayabusa2 mission deployments.

### 3.4 Escape Velocity and Containment
Ryugu's gravity is weak enough that this is not a theoretical concern: a sufficiently
energetic hop could genuinely exceed the body's escape velocity and never return.

**Escape velocity calculation.** For a spherical approximation, $v_{esc} = \sqrt{2gR}$,
where $R$ is Ryugu's mean radius. Using the simulated surface gravity
($g = 0.000114 \text{ m/s}^2$) and Ryugu's mean radius ($R \approx 450 \text{ m}$,
derived from the SfM shape model's effective spherical-equivalent diameter of
~900 m [6]):
$$ v_{esc} = \sqrt{2 \times 0.000114 \times 450} = \sqrt{0.1026} \approx \mathbf{0.320 \text{ m/s}} $$

**Margin under nominal operation.** The locomotion controller derives a target launch
velocity from requested hop distance $d$ via $v_{req} = \sqrt{gd}$. For the farthest
anomaly a scout can be dispatched to (targets are seeded within ±50m in X and Y, so
up to ~70m diagonal distance):
$$ v_{req} = \sqrt{0.000114 \times 70} \approx 0.0894 \text{ m/s} $$
This is only ~28% of escape velocity (a $\approx 3.6\times$ margin) under intended
operation. However, the mapping from this target velocity to actual commanded joint
amplitude is an empirically-calibrated proportional scaling (see `hopper_locomotion.py`
and the jump-distance-targeting note in §3.1 of `Research_Paper.md`), not a rigorously
derived closed-form relation — so this margin is reassuring but was not treated as a
guarantee.

**Hard physical containment (implemented).** Rather than relying solely on the velocity
margin above, an invisible world-boundary structure (four walls plus a ceiling, all
collision-only — no visual geometry, so nothing renders) was added enclosing the full
100×100m modeled terrain, with a ceiling at 100m altitude — more than $17\times$ the
largest *commanded*-jump apex observed in live testing (~5.6m; later pogo-bounce
episodes during the 2026-07-16 landing-dissipation work briefly reached ~10.3m —
still >9x under the ceiling, and that failure mode is now fixed, see SS12.2). This guarantees containment via
genuine physics collision response regardless of how any given launch impulse is
calibrated, both against exceeding escape velocity vertically and against drifting
horizontally past the edge of the modeled terrain (which is only 100×100m — a bot
crossing that boundary would fall off the modeled world into undefined space
regardless of velocity). Implementation: `worlds/ryugu.sdf`, static `world_boundary`
model.

### 3.5 References
[3] S. Watanabe *et al.*, "Hayabusa2 arrives at the carbonaceous asteroid 162173 Ryugu — A spinning top-shaped rubble pile," *Science*, vol. 364, no. 6437, pp. 268–272, Apr. 2019, doi: 10.1126/science.aav8032.
[4] European Southern Observatory (ESO), "The Milky Way panorama," ESO GigaGalaxy Zoom Project, Image ID: eso0932a. [Online]. Available: https://www.eso.org/public/images/eso0932a/. [Accessed: Jul. 7, 2026].
[5] Open Robotics, "Gazebo Harmonic: Rendering with Ogre2," Gazebo Documentation. [Online]. Available: https://gazebosim.org/docs/harmonic. [Accessed: Jul. 7, 2026].



---
- **Terrain:** The surface is characterized by highly porous, fragile breccia and massive boulder fields. Traversing this requires leaping/hopping rather than wheeled locomotion.

## 2. Mass & Weight Breakdown
To achieve high fidelity within the physics engine, we target a total mass of exactly **2.50 kg**. The following table details the mass distribution of the robot's subsystems:

| Subsystem | Components Included | Mass (kg) | Percentage |
| :--- | :--- | :--- | :--- |
| **Chassis** | Aluminum 7075-T6 core, CFRP structural panels | 0.70 | 28% |
| **Locomotion** | 6x Maxon RE 13 leg motors (hip+knee x3), planetary gearheads, legs | 0.45 | 18% |
| **Attitude Control** | 3x Maxon EC 20 flat RW motors + flywheels (X/Y/Z) | 0.20 | 8% |
| **Avionics & Sensors** | Flight computer, LIDAR, IMU, S-Band comms | 0.50 | 20% |
| **Power System** | 4x Space-Grade Li-ion 18650 cells, BMS | 0.30 | 12% |
| **Scientific Payload** | Rotary-Percussive Micro-Corer, Storage Carousel | 0.20 | 8% |
| **Thermal & Solar** | GaAs Solar Arrays, Kapton MLI blankets | 0.15 | 6% |
| **TOTAL** | | **2.50 kg** | **100%** |

> [!NOTE]
> **Corrected 2026-07-14:** previously "9x Maxon DC Motors" conflated the 6 leg motors
> (Maxon RE 13) with the 3 reaction wheel motors (Maxon EC 20) as one line. Split out;
> total mass unchanged. Also see the LIDAR caveat in §9.3 below — it's listed here as
> intended hardware but is not currently present in the simulated `model.sdf`.

### 2.1 Gravitational Weight Calculations
The operational weight of the robot dictates the required jumping force and joint stiffness.
* **Mass ($m$):** $2.50 \text{ kg}$
* **Ryugu Gravity ($g$):** $0.000114 \text{ m/s}^2$

$$ W = m \times g $$
$$ W = 2.50 \times 0.000114 = \mathbf{0.000285 \text{ N}} $$

## 3. Jumping Physics & Locomotion
To execute a controlled 5-meter high hop to clear boulders:
- **Potential Energy Required:** $E_p = m g h = 2.5 \times 0.000114 \times 5 = 0.001425 \text{ Joules}$.
- **Takeoff Velocity:** $v = \sqrt{2gh} \approx 0.0337 \text{ m/s}$.
- **Force Applied:** If the legs extend by $d = 0.1 \text{ m}$ during takeoff, the total average linear force required is $F = E / d = 0.001425 / 0.1 = \mathbf{0.0142 \text{ N}}$.
- **Leg Motor Selection:** Maxon RE 13 (Brushed DC) with GP 13 gearheads (1:67 reduction) providing up to **134 mNm** of torque. This provides a massive 62x safety factor against the required jumping torque. This over-engineering is necessary to overcome internal friction, cold-welding in the vacuum of space, and the stiffness of thermal insulation blankets at the joints.

## 4. Attitude Control & Reaction Wheels
- **In-flight Stabilization:** The robot utilizes 3 internal Reaction Wheels (RWs) to dynamically stabilize pitch, roll, and yaw mid-flight. 
- **Hardware:** Maxon EC 20 flat (Brushless DC); datasheet no-load speed ≈9,380 rpm (982 rad/s), nominal torque ≈8.75 mNm, stall ≈25.7 mNm. The simulation's 15 mNm torque budget sits in the intermittent-duty band (between nominal and stall) appropriate for correction burns of a few seconds.

### 4.1 Mathematical Model of Attitude Correction

> [!NOTE]
> **Corrected 2026-07-15 (scientific-accuracy pass):** this section previously used
> $I_{bot} \approx 0.0055$, which matches *no* configuration of the simulated model
> (`base_link` alone is $0.009 \text{ kg}\cdot\text{m}^2$: a 1.35 kg, 0.2 m box gives
> $\frac{m}{12}(s^2{+}s^2) = 0.009$), and $H_{max} = 0.377 \text{ N·m·s}$, which implies a
> 13,330 rpm wheel — above the cited motor's own no-load speed. Corrected values below;
> the controller's wheel-speed clamp was likewise reduced 1396 → 982 rad/s.

* **Robot Moment of Inertia ($I_{bot}$, about body $z$):** posture-dependent, $\approx 0.012$ (flight posture, legs retracted straight down) to $\approx 0.020 \text{ kg}\cdot\text{m}^2$ (fully splayed). Breakdown at flight posture: chassis $0.009$ + flywheels $\approx 0.00055$ (one spin-axis $\frac{1}{2}mr^2 = 0.00027$ + two transverse $0.00014$) + solar panel $0.00081$ + legs $\approx 0.0015$–$0.0098$ by parallel-axis at posture radii.
* **Reaction Wheel Torque ($\tau_{rw}$):** $0.015 \text{ N}\cdot\text{m}$ (short-term permissible, see hardware note above)
* **Flywheel Inertia ($I_w$):** $\frac{1}{2}(0.15)(0.06)^2 = 2.7\times10^{-4} \text{ kg}\cdot\text{m}^2$
* **Maximum Angular Momentum Capacity:** $H_{max} = I_w\,\omega_{max} = 2.7\times10^{-4} \times 982 \approx \mathbf{0.265 \text{ N}\cdot\text{m}\cdot\text{s}}$

The angular acceleration imparted on the robot at the torque limit (flight posture) is:
$$ \alpha = \frac{\tau_{rw}}{I_{bot}} = \frac{0.015}{0.012} = \mathbf{1.25 \text{ rad/s}^2} $$

A usable correction must arrive at the target angle with zero residual rate, so the minimum-time profile is bang-bang — accelerate for half the angle, decelerate for the rest:
$$ t_{min} = 2\sqrt{\frac{\theta}{\alpha}} = 2\sqrt{\frac{\pi/2}{1.25}} \approx \mathbf{2.24 \text{ s}} \text{ for } \theta = 90° $$
(The old $\theta = \frac{1}{2}\alpha t^2 \Rightarrow 1.07\,$s formula computed time to *sweep past* 90° while still at full spin — a different, less useful quantity.) The deployed controller (§4.1.2) is deliberately slower than the physical bound: overdamped PD, converging a 107° slew in ~15–20 s with zero overshoot — negligible against multi-minute ballistic flight times at $g = 1.14\times10^{-4}$ m/s².

- **Uneven Terrain:** When jumping off uneven terrain with only 1 or 2 legs touching the ground, the jump will induce severe off-center torque. The RWs act as active gyroscopic dampeners, applying counter-torque to keep the bot level as it pushes off.
- **Worst Case Scenario (Spin-out Analysis):** A fully unbalanced jump at maximum takeoff velocity generates a maximum angular momentum of $0.0084 \text{ N}\cdot\text{m}\cdot\text{s}$; against $H_{max} = 0.265 \text{ N}\cdot\text{m}\cdot\text{s}$ that is a **~31x margin** per launch. *Amended 2026-07-15:* saturation is impossible per-launch, but NOT via **integrator windup** — a controller pumping momentum against a persistent unreachable error (holding "perfectly level" on terrain that isn't level) walks the wheels to the pin given hours, observed live in an overnight run. Closed by the 1° attitude deadband plus the landed-state handoff (§4.1.2).
- **Recovery (Inversion):** If the bot lands on its back, the hip/knee joints have been granted a near 360-degree range of motion, which would allow it to flip its legs over its body to push off the ground and self-right.
  > [!NOTE]
  > **Implemented and live-confirmed 2026-07-14:** `landing_controller.py` detects
  > inversion via IMU orientation and runs an alternating splay/asymmetric-sweep
  > righting maneuver (up to 5 retries, rotating the lead leg each time). First live
  > trial: a genuinely inverted landing (not synthetic) triggered the maneuver, which
  > succeeded once but then immediately re-detected inverted — traced to the attitude
  > controller's Euler-angle PID oscillating at large tumble angles and actively
  > fighting the righting maneuver (§4.1.1 below). A second 5-attempt cycle then
  > legitimately failed and the "give up, mark landed anyway" fallback correctly
  > prevented a hang. **After rewriting the attitude controller** (§4.1.1), a fresh test
  > succeeded on the first attempt, and a follow-on autonomous hop landed upright with
  > no righting needed. See `walkthrough.md` for the full blow-by-blow log excerpt.

### 4.1.1 Attitude Controller Rewrite: Euler Angles → Quaternion Tilt Feedback (2026-07-14)
The first live closed-loop test of the reaction-wheel controller (made possible only
after fixing the ros_gz_bridge issue in §9.1 — this control loop had never actually run
against real sensor data before this session) surfaced a genuine instability:
`attitude_error` oscillated between 1.5–2.8 rad (85–160°) instead of converging toward
zero, and the robot remained visibly spinning (`angular_velocity.z = 4.12 rad/s`)
minutes after being marked `LANDED`.

**Root cause:** the original controller decomposed IMU orientation into Euler angles
(roll, pitch) and applied independent PID loops per axis. This decomposition is only a
valid *small-angle* approximation. At the large tumble angles this robot legitimately
experiences (launch-induced tumble, post-landing settling), body-frame IMU angular
velocity ($\omega_x, \omega_y$) no longer corresponds to Euler-angle rates
($\dot{roll}, \dot{pitch}$), so the derivative (damping) term was damping the wrong
quantity — in some configurations amplifying rather than suppressing oscillation. A
secondary contributor: the original controller accumulated PID output into wheel
velocity every tick (`cmd_vel += output * dt`), an unnecessary additional integrator in
the control loop.

**Fix — quaternion cross-product tilt feedback.** Rather than Euler angles, rotate the
body's local $+Z$ ("up") axis into the world frame:
$$ \hat{u}_{local,x} = 2(q_x q_z + q_w q_y), \quad \hat{u}_{local,y} = 2(q_y q_z - q_w q_x) $$
and take its cross product with world $+Z = (0,0,1)$:
$$ \vec{e} = \hat{u}_{local} \times \hat{u}_{world} = (\hat{u}_{local,y},\ -\hat{u}_{local,x},\ 0) $$
This vector is a rotation-axis-aligned error signal valid at *any* tilt magnitude — no
small-angle assumption, no gimbal lock — and its $z$-component is identically zero,
meaning the correction never touches yaw (matching the original design intent: correct
"upside-down", not heading). Reaction wheel commands are now set directly proportional
to this error and damped by body rate, replacing the accumulated-acceleration
structure, and the integral (I) terms were removed entirely — near-zero-g free flight
has no persistent disturbance torque for an I-term to usefully compensate for, so it
was pure windup risk. The new formula was verified algebraically to match the old
Euler-angle formula's sign convention exactly in the small-angle limit (checked pure
roll and pure pitch cases), so this is a strict improvement at large angles with no
behavior change at small ones.

**Live verification:** self-righting success on the first attempt (vs. 5/5 retries
before), a follow-on autonomous idle-recovery hop landing cleanly with zero righting
needed, and post-landing `angular_velocity` of $\sim10^{-15}$ rad/s (numerical noise)
versus 4.12 rad/s before. Implementation: `ryugu_sim/attitude_controller.py`,
committed `b68ca4f`.

### 4.1.2 Second Rewrite: Velocity Commands → Torque-Based Momentum Pumping (2026-07-15)

The quaternion rewrite fixed the error *signal*; live testing then exposed a structural
flaw in the *actuation*: both prior controllers commanded reaction-wheel **velocity**
proportional to attitude error. A reaction wheel exchanges momentum with the body only
while it **accelerates** ($\tau_{body} = -I_w \dot{\omega}_{wheel}$); once the wheel
reaches its commanded speed, torque stops flowing. Confirmed live in two forms:

1. **Grounded:** the robot held a steady 0.42 rad yaw error indefinitely, with `cmd_z`
   pinned at $300 \times 0.42 \approx 126.7$ rad/s, the wheel spinning at exactly that
   speed, and zero torque flowing. A velocity-proportional law physically cannot null
   a steady-state attitude error.
2. **In flight:** with rate feedback $\omega_{cmd} = K_d\,\omega$, momentum conservation
   gives a *nonzero* equilibrium spin. Body + wheel momentum is conserved during free
   flight, so with the wheel servo-locked to $K_d\,\omega$:
   $$ I_{bot}\,\omega + I_w K_d\,\omega = L_0 \;\Rightarrow\; \omega_{res} = \frac{L_0}{I_{bot} + I_w K_d} \neq 0 $$
   With the then-current $K_d = 60$: $\omega_{res} = L_0 / (0.012 + 0.0162)$ — matching
   the persistent $-1$ to $-2.3$ rad/s yaw spin telemetry showed after every hop.
   Additionally, yaw error wraps at $\pm\pi$: at $K_p = 300$ a spinning body turns the
   velocity target into a $\pm 942$ rad/s sawtooth that the slew-limited command chases
   with near-zero mean, stalling momentum transfer entirely.

**Fix — the standard RW control structure** (Sidi 1997 ch. 7; Wie 2008): PD on attitude
produces a desired **body torque**, clipped to the physical 15 mNm budget; wheel
acceleration $= -\tau/I_w$; integrate into the wheel speed command:
$$ \tau = \mathrm{clip}(K_{ang}\,e - K_{rate}\,\omega,\ \pm 0.015), \qquad \omega_{wheel,cmd} \mathrel{+}= -\frac{\tau}{I_w}\,dt $$
Gains $K_{ang} = 0.02$ N·m/rad, $K_{rate} = 0.05$ N·m·s/rad sized against the
whole-robot inertia: $\omega_n = \sqrt{K_{ang}/I} \approx 0.9$–$1.3$ rad/s,
$\zeta = K_{rate}/(2\sqrt{K_{ang} I}) \approx 1.1$–$1.6$ — **overdamped by
construction**, per the explicit no-oscillation tuning requirement. Torque saturates
above 0.3 rad/s body rate (clean full-torque rate-kill) and above ~0.75 rad angle
error; inside those bounds the response is smooth.

Guards, each traced to a live failure:
* **1° angle deadband** — a tripod on regolith never sits at exactly 0° tilt; without
  the deadband the torque integrator walked the wheels to full momentum saturation
  overnight against a ~0.1° residual (observed pinned at the speed clamp).
* **No rate deadband** — a first attempt deadbanded rate at 0.005 rad/s; telemetry then
  showed the body coasting at *exactly* 0.005 rad/s in a slow ±1.2° limit cycle between
  the deadband walls (nothing removed momentum inside the band). Damping only acts
  while the body rotates, so it cannot wind up and needs no deadband. Removed.
* **Per-axis speed clamp** — the old clamp rescaled *all three* axes when one
  saturated, needlessly destroying the healthy axes' authority; the wheels are
  physically independent.
* **Landed handoff both directions** — `in_flight` now tracks the `/landed` topic
  itself: tilt correction disarms on confirmed touchdown (windup guard) and *arms on
  any* airborne condition (spawn descent, bounces, disturbance kicks), not just
  announced jumps.

**Live-verified 2026-07-15:** 107° commanded yaw slew converged overdamped and held
within 1° at zero measured rate; an accidental 165° tumble damped to 3.6° in ~20 s; a
grounded robot no longer holds phantom wheel-speed offsets. Committed `9de61d2`.

## 5. Power Budget & Battery Calculations

To accurately model battery depletion, the robot utilizes a 4-cell Space-Qualified Lithium-Ion 18650 pack (e.g., Saft VES16), wired in a 2S2P configuration (7.4V nominal).
* **Total Capacity ($C_{bat}$):** $5000 \text{ mAh} \times 7.4 \text{ V} = \mathbf{37.0 \text{ Wh}}$ (or $133,200 \text{ Joules}$)

### 5.1 Subsystem Power Draw

| Operational State | Subsystem | Duty Cycle | Peak Power (W) | Avg Continuous Power (W) |
| :--- | :--- | :--- | :--- | :--- |
| **Continuous** | Avionics (CPU, IMU, Comms Rx) | 100% | 2.00 | 2.00 |
| **Continuous** | Reaction Wheels (Flight Stabilization) | 100% | 5.00 | 1.50 |
| **Intermittent** | Leg Motors (Jumping Phase) | < 0.1% | 6.00 | 0.005 |
| **Intermittent** | Micro-Corer Drill (Sampling) | < 1.0% | 3.00 | 0.02 |
| | **TOTAL ESTIMATED DRAW** | | **16.00 W** | **~3.525 W** |

### 5.2 Battery Usage Mathematical Breakdown

**1. Locomotion Energy per Jump:**
Assuming 4 leg motors fire at their nominal $1.5 \text{ W}$ rating for $t = 0.5 \text{ s}$ to clear an obstacle:
$$ E_{jump} = P_{peak} \times t = 6.0 \text{ W} \times 0.5 \text{ s} = 3.0 \text{ Joules} $$
If the bot jumps once every 10 minutes (600 seconds):
$$ P_{avg\_jump} = \frac{E_{jump}}{600} = \frac{3.0}{600} = 0.005 \text{ W} $$

**2. Drilling Sequence Energy:**
Assuming the drill operates at $3.0 \text{ W}$ for $5 \text{ minutes}$ (300 seconds):
$$ E_{drill} = 3.0 \text{ W} \times 300 \text{ s} = 900 \text{ Joules} $$
Averaged over a 10.5-hour ($37,800 \text{ s}$) mission profile:
$$ P_{avg\_drill} = \frac{900}{37800} \approx 0.023 \text{ W} $$

**3. Operational Lifespan Calculation:**
Given the total average continuous power draw ($P_{total} \approx 3.5 \text{ W}$):
$$ \text{Lifespan} = \frac{C_{bat}}{P_{total}} = \frac{37.0 \text{ Wh}}{3.5 \text{ W}} = \mathbf{10.57 \text{ hours}} $$
This indicates the robot can operate continuously for just over 10.5 hours in shadow before suffering critical battery depletion.

## 6. Solar Recharge
- **Solar Array:** High-efficiency Gallium Arsenide (GaAs, 28% efficiency) covering $0.0324 \text{ m}^2$.
- **Power Generation:** With Ryugu at ~1.2 AU, irradiance is $\sim 950 \text{ W/m}^2$. Peak generation is 8.6 W. Accounting for off-angle incidence, net generation averages **~3.5 W**.
- **Recharge Time:** To recharge 30 Wh while asleep (drawing 0.5 W for survival systems), the recharge time is **~10 hours**. The true worst-case scenario is landing in a permanently shadowed crater, resulting in total power loss after 10.5 hours.

## 7. Sampler Design & Scientific Payload
- **Tool:** Hollow Rotary-Percussive Micro-Corer.
- **Scientific Value:** Drilling slowly at pristine locations (away from thruster contamination from Hayabusa2) preserves fragile stratification and volatile organics/hydrated minerals that would otherwise be destroyed by blast sampling.
- **Storage:** The tungsten-carbide bit retracts directly upward into a **sterile caching carousel** housed securely inside the lower chassis block. The carousel holds 3 interchangeable capillary tubes.

## 8. Communications Protocol
- **Intra-Swarm Mesh Network (UHF):** For robot-to-robot communication over the boulder-strewn surface, the bots use a low-power Ultra-High Frequency (UHF, ~400 MHz) mesh network. UHF diffracts well around large boulders.
- **Mothership Uplink (S-Band):** To communicate with Earth, each bot has an S-Band (2 GHz) patch antenna to transmit data directly to the orbiting Hayabusa2 mothership, which acts as a high-gain relay.

## 9. Simulation-ROS Bridge Debugging (2026-07-14)

### 9.1 The IMU/Odometry Bridge Was Silently Dead From the Start
A methodological finding significant enough to record here: reaction-wheel attitude
control, landing contact-detection, and the odometry-based position tracking added
earlier this session were all **never actually functional in any prior session**,
despite `attitude_controller.py` and `landing_controller.py` existing, running without
errors, and being described in `task.md`/`HANDOFF.md` as implemented. The root cause was
two compounding bugs, both in infrastructure rather than robot logic:

1. **IMU sensor topic scoping.** The IMU sensor's SDF declared an explicit
   `<topic>imu</topic>`, which gz-sim treats as a literal, unscoped global topic name
   (`/imu`) rather than the auto-scoped per-entity topic (e.g.
   `/world/ryugu_world/model/scout_1/link/base_link/sensor/imu_sensor/imu`) that
   `ros_gz_bridge` was configured to listen on. Fix: remove the explicit `<topic>`
   override and let gz-sim auto-generate the scoped name, updating the bridge config to
   match — the same pattern the joint controllers and odometry publisher already used
   correctly (no explicit topic override).
2. **Gazebo version mismatch in the installed `ros_gz_bridge` package.** The system's
   `ros-humble-ros-gz-bridge` (the un-suffixed default) was built against Gazebo
   **Fortress** (`ignition-transport11`/`ignition-msgs8`), while the actual simulator
   installed is Gazebo **Harmonic** (`gz-sim8`, using `gz-transport13`/`gz-msgs10`).
   Confirmed via `ldd` on the `parameter_bridge` binary and the model's IMU plugin
   library. These transport generations are not wire-compatible for the more complex
   message types (IMU, Odometry) — simple types (`std_msgs/Float64` for joint commands)
   happened to still work, which is why leg/drill motion had been visually verified
   already while RW/landing verification kept turning up "not yet observed." Fixed by
   installing the Harmonic-paired bridge package (`ros-humble-ros-gzharmonic-bridge`,
   requires `sudo apt install` — it replaces the same binary path via package
   alternatives, no launch file path changes needed) and pointing the IMU model plugin
   at the matching Harmonic library (`gz-sim-imu-system` instead of the old
   `ignition-gazebo-imu-system`, which was also present on the system and silently
   loaded instead of erroring).

**Post-fix verification:** IMU now publishes to ROS at a clean 99.9Hz, odometry reports
real changing position, and `rw_speed_max` reads genuine non-zero, changing values
during flight (0.88, 0.87 rad/s in consecutive samples) — confirming the attitude PID
loop is actually closing on real sensor feedback, not running open-loop. This also means
`landed` status is now accurate in real time (previously frozen at its default `True`),
which the SAMPLER arrival-gating logic in `swarm_manager.py` depends on.

**Takeaway for future debugging:** if a ROS topic backed by a Gazebo sensor/plugin
"exists" (`ros2 topic list` shows it, `ros2 topic info` shows connected pub/subs) but
never actually delivers data, check `gz topic -i -t <topic>` on the Gazebo-native side —
if it reports no subscriber despite ROS-side subscribers being connected, suspect a
transport-generation mismatch between the bridge and the simulator, and check `ldd` on
both the bridge binary and the relevant plugin `.so` for their linked transport/msgs
library versions.

### 9.2 Drill Housing/Retraction Geometry
The drill was a bare rigid 0.1m rod translating within a 0–0.1m prismatic joint range —
even at the "retracted" rest position, the entire shaft still hung fully exposed below
the chassis, because nothing ever visually contained it (user-reported: "the drill looks
detached/exposed even at rest"). Fixed by (a) adding a fixed turret/mounting-plate visual
attached to `base_link` at the mount point, and (b) raising the drill's rest pose so the
shaft tucks up into that turret when retracted, only extending fully below it when
commanded to sample.

### 9.3 LIDAR: Design Intent vs. Simulated Reality
`Research_Paper.md`/this document's mass table (§2) and Avionics line item describe
LIDAR as part of the intended hardware BOM. **No LIDAR sensor currently exists in
`model.sdf`** — a prior session's generator script comment simply notes "LIDAR removed
to prevent performance stuttering," and no sensor block was ever re-added. The
`ros_gz_bridge` config still declares a `/scout_1/lidar` bridge entry, but with no
publisher on the Gazebo side it never receives data. Consequently, no LIDAR-based
terrain/hazard-awareness swarm behavior was implemented this session — it was considered
and deliberately dropped rather than reintroducing the sensor and risking the
performance regression it was removed to avoid. Future work: either accept a lighter-
weight LIDAR configuration (fewer rays / lower update rate) or scope hazard awareness to
a substitute signal (e.g., IMU-derived roughness from recent landings).

### 9.4 Visual Realism Additions
A full PBR (physically-based rendering) material pass replaced the model's original
flat ambient/diffuse/specular-only materials, which rendered as painted plastic with no
metallic response to lighting. All 6 chassis faces are now wrapped in gold MLI foil
(previously alternated gold/silver on partial-coverage panels — real landers like
MASCOT and Hayabusa2 itself are wrapped almost entirely in gold foil), the chassis
reads as brushed aluminum, legs are differentiated into carbon-fiber thigh segments and
anodized-aluminum calf segments, and the antenna's glossy white ball tip (identified as
the single biggest "toy" visual cue) was replaced with a slim mast and a proper flat
S-Band patch antenna. Small hip/knee-actuator, reaction-wheel, drill, and comms status
LEDs were added throughout.

An attempt to give these LEDs **real point lights** (rather than just emissive-colored
surfaces, which don't illuminate the scene — easy to miss against Ryugu's near-total
darkness away from direct sun) initially had to be reverted: Gazebo's GUI draws a
persistent wireframe gizmo at every light's position to aid selection, and with several
small lights spread across the legs this produced a confusing overlapping mesh with no
obvious way to disable. Root cause found: `sdf::Light` has a documented `<visualize>`
element (SDF 1.10 spec, default `true`) specifically for suppressing this gizmo without
affecting the light's actual illumination. With that applied, real point lights were
reinstated (14 status LEDs plus 2 forward-facing headlight spotlights flanking the nav
camera — added specifically so the cameras have something to image away from direct
sun, since Ryugu's ambient light is deliberately set to near-zero for scientific
accuracy) with no GUI clutter.

### 9.5 Swarm Status Dashboard (Verification Tooling)
A Tkinter-based dashboard (`ryugu_sim/swarm_gui.py`) was added to give a persistent,
at-a-glance view of swarm state without reading ROS log output — critical once multiple
agents are running concurrently, where interleaved log lines become hard to parse by
eye. Per agent: current role and activity (published by `swarm_manager.py`'s new
`/status_role`/`/status_activity`/`/status_battery`/`/status_power_rate` topics, which
previously only ever went to the log with no way for an external monitor to read
current state), battery level with live charge/discharge rate, landed/flight state, a
square artificial-horizon "gyro" indicator driven by IMU orientation, commanded leg
joint angles, drill state, and reaction-wheel speeds. Agents that have never published
anything (not yet spawned) show as OFFLINE rather than blank or stale data. Wired into
the launch file with an automatic window-layout step (`wmctrl`, via a delayed
`TimerAction`) that docks the simulator at the left 3/4 of the screen and the dashboard
at the right 1/4 on every launch.

## 10. Micro-Gravity Landing Detection & Ground Handling (2026-07-15)

### 10.1 Why "Resting" Is Indistinguishable From Free-Fall

An IMU measures *proper* acceleration. A robot resting on Ryugu experiences a support
force of $mg = 2.85\times10^{-4}$ N, i.e. a reading of $\sim 10^{-4}$ m/s² — below any
realistic noise/threshold floor and identical, to a detector, to coasting in free-fall.
The original state machine treated "accel below the flight threshold (0.005 m/s²)" as
"bounced → back to FLIGHT." Live consequence: the robot settled, the detector declared
a bounce, the state machine re-entered FLIGHT, and — since a robot already at rest
never produces a new impact spike — hung there **forever**. Downstream: the hopper
never returned to IDLE (all jump commands ignored), and the attitude controller's
in-flight tilt loop kept running on the ground, winding the wheels to momentum
saturation overnight (§4.1.2). MASCOT's multi-sensor settling logic on the real Ryugu
faced this same ambiguity class (Ho et al. 2017).

### 10.2 The Deployed Detector (three fused signals)

1. **Contact spike:** $|a| > 0.08$ m/s² (motor reaction transients reach ~0.02;
   genuine impacts exceed ~0.05).
2. **Rest window:** altitude within a ±2 cm band for 60 s AND $|v| < 5$ mm/s.
   The dwell analysis must be **two-sided**: a coast lingers within ±b of apex for
   up to $t = 2\sqrt{2b/g}$ — 37.5 s for b = 2 cm — because the band reference can
   be captured just below apex on the way up. (A first 2 cm/30 s implementation,
   computed one-sidedly, false-confirmed landing at a bounce apex, live. A 1 cm/45 s
   version then proved too strict — a settling robot creeps a few mm as leg contacts
   shift.) The velocity gate independently rejects any hop with a horizontal
   component, which keeps $|v| > 5$ mm/s through apex.
3. **Bounce discrimination:** free-fall accel + genuine velocity (> 2 cm/s) = bounce;
   free-fall accel + near-zero velocity = RESTING → continue settling.

Plus a **liftoff watchdog** (LANDED is not terminal: sustained $|v| > 2$ cm/s reverts
to FLIGHT and re-arms everything — caught a real leg-actuation kick within 2 s of its
first live occurrence) and **IDLE self-arming** in both directions (a restarted node
finds its way to FLIGHT if airborne, or to LANDED via the rest window if grounded,
instead of publishing a stale state forever).

### 10.3 Leg/Terrain Mechanics: Two Real Failure Modes

* **Terrain wedge-in (jam):** left in the splayed compliant-landing pose, the 2 cm
  foot spheres wedge into heightmap crevices under the position controllers'
  sustained push. Verified drastically: hip commands echoed on the gz topic, link
  poses **bit-identical** before/after, zero body reaction — then the same command
  moved the leg violently the instant the robot was teleported clear of the terrain.
  A jammed robot cannot crouch, so every jump silently produced zero thrust. Fix:
  after LANDED confirms, fold legs to an unloaded neutral stance (hip 0.9 / knee
  −1.0, feet tucked, chassis on its belly).
* **Actuation kicks:** in this gravity *any* leg-posture step is a launch event — the
  first stand-up implementation (step command) threw the robot off the surface at
  0.036 m/s ≈ a 5 m unplanned hop; even the compliant-landing posture application
  kicked it at 0.023 m/s (caught by the liftoff watchdog). Posture changes at rest
  are now interpolated over 15 s.
* **Near-lossless bounce ring-down:** with essentially zero joint damping
  (10⁻⁵ N·m·s/rad), the leg position-PIDs act as almost-lossless springs: a 3–5 mm/s
  touchdown bounced for tens of minutes (z oscillating 4.81–4.99 m, ~2% energy loss
  per cycle) — too soft for spike detection, too mobile for the rest window. Joint
  damping raised to 5×10⁻³ N·m·s/rad in the model (later raised again to 0.15 for impact dissipation — see §12.2–12.3 for why and what it costs; still below the 134 mNm
  actuator budget at launch-stroke speeds), dissipating contact energy in a couple
  of cycles. This is the physically-honest fix — real legged landers land on damped,
  compliant joints, not ideal springs.

### 10.4 Idle-Recovery Timer

The hopper's self-initiated "unstick" hop fired after 30 s of IDLE — but a full
micro-gravity land-and-settle cycle alone takes ~45–60 s, so the timer kept kicking
the robot between normal mission phases (observed repeatedly during testing). Raised
to 5 minutes and gated on actually-landed status.

## 11. Swarm Role-Assignment Rework (2026-07-15)

`swarm_manager.py`'s dispatcher previously took the *first* SCOUT in list order —
"market-based" in name only. Reworked into a single-item auction (Gerkey & Matarić
2004 taxonomy): every eligible SCOUT bids
$$ B_a = d_a + 0.5\,(100 - \text{SoC}_a) + 5\,n_a $$
(distance dominates; battery depletion and carousel load penalize), lowest bid wins;
agents under a 30% state-of-charge reserve don't bid. Robustness additions, each
mapped to a concrete prior failure mode:

| Failure mode (before) | Mechanism (after) |
| :--- | :--- |
| SAMPLER forced to RECHARGE mid-task silently lost its anomaly | Target re-queued at head of queue on battery override |
| Hop landing outside the 3 m arrival radius stranded the agent "en route" forever (jump only ever issued once) | Corrective re-hop after a 90 s cooldown, max 5 retries, then target requeued + agent stands down |
| Dead/silent agent still assigned tasks | 10 s odometry-liveness watchdog → OFFLINE: excluded from auction, in-progress target requeued, rejoins as Unassigned on recovery |
| Anomalies generated at ±50 m — beyond the ±49 m containment walls (physically unreachable) | Generation clamped to ±45 m |
| Core extraction completed instantaneously on contact | 8 s drill dwell (4 ticks) before the sample counts, so the power model's drill duty term reflects reality |

## 12. The Liftoff Campaign and the Contact-Dissipation Tradeoff (2026-07-15/16)

### 12.1 The decisive root cause (and a lesson in contaminated experiments)

Five diagnostic sessions attacked "the crouch stalls at millimetres, the legs deliver
no thrust" with successively deeper theories: DART auto-sleep (real, fixed with an
in-place `set_pose` wake), stroke direction (real — the delta-launch scheme swept the
feet sideways under the body), and micro-gravity friction geometry (real — with total
foot friction capacity µ·m·g ≈ 2.9×10⁻⁴ N, any lateral force component slides the feet
instead of lifting; the fix is a zigzag stroke holding each foot directly under its
hip). All three were genuine bugs, and none was THE bug.

The decisive instrument was an echo on the *Gazebo-side* joint command topic during a
stroke: it showed a continuous ~100 Hz stream of `data: 0.9` — the landing
controller's post-landing stand pose — swamping the hopper's one-shot crouch/launch
targets within ~10 ms of each being published. The landing controller's LANDED branch
ran inside the IMU callback and never stopped re-publishing after its stand-up ramp
completed. Every stalled-crouch measurement across every session had been taken with
this override active; the theories built on those measurements were fitted to
contaminated data. Two structural fixes: (1) publishers stop when their phase ends —
the ramp goes silent on completion; (2) the hopper re-asserts its targets every
control tick for as long as it owns the legs, so a last-write-wins race can never be
lost silently again (commit `5d37147`).

With the override gone, one further authority fix (leg PID p 0.05 → 1.0; at p = 0.05
the stroke tracked at ~4 mm/s, rate-limited 25× below the untouched 134 mNm torque
cap) produced the first verified liftoff in the project's history:

* separation velocity **0.0398 m/s** (2.2× the 18.5 mm/s a 3 m hop requires),
* sustained ascent from z = 4.91 m to 6.95 m and climbing at sample end,
* apex energy consistent with v²/2g to within measurement noise (textbook ballistic
  coast), and V_FULL (full-stroke delta-v) calibrated to 0.04 m/s from the measured
  hop.

### 12.2 Contact dissipation: three active schemes, all energy-positive

The stiff launch gains turned touchdown into a trampoline: measured restitution ≈0.96
from a 1.15 m drop (bounce apexes 5.88 → 5.76 m — no meaningful decay). Three active
compliance schemes were implemented and live-measured; **every one added energy at
contact**:

| Scheme | In (mm/s) | Out (mm/s) | Outcome |
|---|---|---|---|
| Step to soft posture at contact instant | — | — | 0.7–0.9 m kicks, non-decaying pogo |
| Same posture ramped over 2 s | 32 | 38 | pogo to 10+ m |
| Zero-stiffness catch: mirror measured joint angles back as targets | 16 | 22 | worst — feedback lag pumps the rebound |

The catch is the theoretically interesting failure: target-follows-measurement is
correct with zero-latency feedback, but the joint states cross a transport bridge, so
the target *trails* the joint — during rebound the lagged position error torques with
the motion. Delayed feedback around a contact event is destabilizing, and in milli-g
there is no weight margin to eat the error. Generalization (now thrice-confirmed in
this project): **on a milli-g body, every commanded leg motion while grounded is a
thruster firing.**

Physical joint damping — phase-perfect by construction — solved it (commit `bb922ee`):
c raised 5×10⁻³ → 0.15 N·m·s/rad, giving contact ζ ≈ 0.45, restitution ≈ 0.2. Verified
live: spawn-descent settle and post-hop impact landing both reached confirmed LANDED
in 2.5–3.5 min with decaying bounces, and the full autonomous mission loop ran
(auction → dispatch → hop → flight → land → settle → next hop).

### 12.3 The open tradeoff, quantified

Damping acts on the launch stroke exactly as on impact. Measured operating points:

| Joint damping (N·m·s/rad) | Hop separation v | Landing behavior |
|---|---|---|
| 0.005 | 39.8 mm/s (multi-meter hops) | restitution ≈0.96, indefinite pogo |
| 0.15 (deployed) | few mm/s (~25 cm ascents) | ζ≈0.45, settles in 2–3 bounces |
| 0.4 (with p=5.0) | — | ⛔ joints freeze entirely (zero tracking; suspected DART explicit-damping discretization limit) — reverted |

Candidate directions for closing the gap, in rough cost order: an intermediate damping
sweep (0.03–0.08); contact-surface compliance on the foot spheres so the *ground*
absorbs the impact (dartsim parameter support unverified); and the real-robot answer —
a series-elastic launch element charged slowly and released through a latch, which
decouples launch delta-v from joint damping entirely (the mechanism spring-loaded
hoppers and SpaceHopper's parabolic-flight prototype use). This is the primary open
engineering item; HANDOFF.md checklist item 2 carries the working notes.
