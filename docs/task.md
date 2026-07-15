# Task Tracker

> See `implementation_plan.md` for full context on the phase below. Older completed
> phases (model detail, control nodes, git recovery, DART auto-sleep fix) are preserved
> further down this file.

## ACTIVE: Realism + Swarm Intelligence + Sampler Fix (started 2026-07-14, completed same day)

### Phase A — Functional fixes (prerequisites)
- `[x]` A1. Wire up drill/sampler actuator: JointPositionController plugin, spring-to-rest dynamics, bridge topic, regenerate + diff-check model.sdf. **Verified live via screenshots**: drill extends to -0.1 and retracts to flush-0.0 on command, sits properly attached (no dangle) at rest. Also reconciled several other generator/model.sdf drift issues found along the way: leg joint limits (were ±1.0/±1.5, should be ±3.14), leg controller gains (were p=1/d=0.1 uncapped, should be p=0.05/d=0.01 with ±0.134 cmd caps), knee spring dynamics (was missing), and mass constants for base_link/solar_panel/RW/drill (generator's old constants summed to 2.0kg total, not the paper's declared 2.50kg — now corrected and verified to sum to exactly 2.50kg).
- `[x]` A2. Fix swarm_manager SAMPLER logic: gate drill deploy on actual `/landed` arrival + proximity (`ARRIVAL_RADIUS`), retract after sampling, fix "assumes robot at origin" position bug via real odometry subscription. Live-verified: `swarm_manager` correctly logged "en route ... landed=False" during flight and only deployed once landed+arrived.
- `[x]` A3. Fix jump-distance targeting in hopper_locomotion.py — launch amplitude now scales proportionally to `v_req` around a calibrated 5m↔1.0rad reference, clamped [0.3, 2.5] rad. Live-verified: a 65.9m dispatch produced amplitude=2.50 (clamped ceiling) vs. shorter hops producing proportionally less.
- `[x]` A4. Add idle-on-ground recovery: replaced the old "instant re-launch on any landing" behavior (which meant the robot never really idled) with clean settle-to-IDLE + a 30s self-timeout recovery hop if nothing else commands a jump.
- `[x]` A5. **(not originally scoped, added mid-session per explicit user request)** Self-righting via leg inversion: `landing_controller.py` detects inverted landings via IMU orientation and runs an alternating splay/asymmetric-sweep maneuver, up to 5 retries with rotating lead leg.
- `[x]` A6. **(found during Phase B, not originally scoped)** Fixed the IMU/odometry ROS↔Gazebo bridge, which had never actually delivered data in any prior session (topic-scoping bug + a Gazebo-Fortress-vs-Harmonic package version mismatch requiring `sudo apt install ros-humble-ros-gzharmonic-bridge`). See `research_report.md` §9.1 for the full writeup — this is the reason Phase B below was previously unverifiable.

### Phase B — Verify RW + leg motion (item 4)
- `[x]` B1. Live-tested: `rw_speed_max` reads genuine non-zero, changing values during flight (0.88 → 0.87 rad/s across samples) — confirms the attitude PID loop is closing on real IMU feedback post-bridge-fix, not running open-loop.
- `[x]` B2. Live-tested via logs: leg joints move through crouch ("Compressing Legs")/launch ("IGNITION")/retract cycles on every dispatched jump, amplitude now varying with `v_req` (A3).
- `[x]` B3. IMU confirmed publishing to ROS at a clean 99.9Hz; odometry confirmed reporting real changing position (not frozen at origin).
- `[x]` B4. **First-ever confirmed full jump→apex→fall→ground-contact cycle, observed
  start-to-finish** (later same day, after a laptop crash interrupted and restarted the
  test). A background watcher confirmed 60s of sustained `LANDED` state on a controlled
  3m hop. The robot landed **inverted**; self-righting engaged automatically, cycled
  through retries, and the "give up after 5 attempts, mark LANDED anyway" fallback
  fired exactly as designed. Full detail in `walkthrough.md`.
- `[x]` B5. **(found live during B4, not originally scoped)** Root-caused and fixed a
  real bug: `attitude_controller.py` never reset `in_flight` after landing (no
  subscription to `/landed` existed at all), so roll/pitch correction ran forever
  post-touchdown, actively fighting self-righting. Live-confirmed via IMU
  (`wz=4.12 rad/s` minutes after LANDED). Fixed, committed `7ba3977`, re-verification
  with a fresh hop in progress as of this writing.
- `[x]` B6. **Fixed** — rewrote tilt (roll/pitch-equivalent) correction from
  Euler-angle PID to quaternion cross-product feedback (rotate local +Z into world
  frame, cross with world +Z), which is valid at any tilt magnitude instead of only
  small angles. Also switched from accumulated-acceleration to direct velocity
  commands, and removed integral terms (no persistent disturbance torque in free
  flight for an I-term to compensate for). Committed `b68ca4f`. **Live-verified**: a
  fresh hop landed cleanly with self-righting succeeding on the *first* attempt
  (previously needed all 5 retries), a follow-on idle-recovery hop completed just as
  cleanly, and post-landing `angular_velocity` read ~1e-15 rad/s (float noise) vs.
  4.12 rad/s before — confirmed genuinely still, not spinning.

### Phase C — Swarm manager improvements (item 3, open-ended)
- `[ ]` C1. LIDAR-informed hazard/terrain awareness — **dropped**, not implemented. No LIDAR sensor exists in `model.sdf` (removed in an earlier session for performance reasons); see `research_report.md` §9.3. Do not wire hazard logic against `/scout_1/lidar` without first re-adding a real sensor and confirming it doesn't reintroduce the stuttering it was removed for.
- `[x]` C2. Battery drain now tied to role/activity (`BATTERY_DRAIN_BY_ROLE`, plus extra draw while drilling) instead of flat random drain applied to every agent regardless of what it's doing. Also fixed RECHARGE, which previously still drained at the same rate as every other role and could never climb back above its own 80% exit threshold — it now actually gains charge (`SOLAR_CHARGE_RATE`).
- `[x]` C3. 3-tube sample carousel (`SAMPLE_CAROUSEL_CAPACITY`) now has real effect: a SAMPLER chains directly to the next queued anomaly instead of always returning to SCOUT after one sample, as long as the carousel has room and the queue isn't empty.
- `[x]` C4. Multi-agent readiness reviewed by code inspection (publishers/subscribers/dispatch already per-agent-keyed, should generalize). Not verified by actually spawning more agents — that would additionally require `spawner.py`/launch-file changes, which was out of scope for a "sanity check."

### Phase D — Model realism pass (item 1)
- `[x]` D1. Fresh screenshots taken (via `gz service /gui/screenshot` + manual camera repositioning) during drill/LED verification.
- `[x]` D2. Drill/sampler housing: added a fixed turret/mounting-plate visual on `base_link`, raised the shaft's rest pose so it tucks up into the housing when retracted instead of always hanging fully exposed.
- `[x]` D3. **(user-requested mid-session)** Added small hip-actuator status LEDs (real point lights, not just emissive material — first pass was emissive-only and user correctly called out that it doesn't actually illuminate anything), small stereo hazcams flanking the nav camera, and foot-pad proximity-sensor lenses.

### Phase E — Research docs accuracy pass (item 2)
- `[x]` E1. Fixed RW motor mass-table conflation in both `Research_Paper.md` and `research_report.md` — "9x Maxon DC Motors" split into separate Locomotion (6x RE 13) / Attitude Control (3x EC 20) lines, same total mass.
- `[x]` E2. Self-righting claim updated from "not implemented" to "implemented, live end-to-end recovery not yet directly observed" in both docs.
- `[x]` E3. Jump-distance/velocity math caveat in §3.1 updated to describe the actual empirical-scaling implementation (not the literal $F=E_p/d$ formula) and its live-verified behavior.
- `[x]` E4. Logged and fixed additional discrepancies found along the way: LIDAR claimed-vs-actual status (§9.3), IMU/bridge methodology writeup (§9.1), drill housing writeup (§9.2), visual additions writeup (§9.4).

### Phase F — Final verification + doc sync (item 6)
- `[x]` F1. Final rebuild + relaunch after every change this session (many iterations); no runtime errors on the final swarm-manager-improvements build.
- `[x]` F2. Final sweep of `HANDOFF.md`/`task.md`/`walkthrough.md`/research docs completed — see `HANDOFF.md`'s "Guidance for next agent" section for what's still open and how to approach it.

### Phase G — World containment safety (user-requested, later same day)
- `[x]` G1. Prevent the bot from ever exceeding escape velocity / flying off into
  space permanently. Ryugu's escape velocity (~0.32 m/s, computed from g and its
  ~450m mean radius) is genuinely reachable given how weak gravity here is — not a
  theoretical concern. Solved via a hard physical ceiling (see G2) rather than
  retuning launch thrust math, since a real collision boundary guarantees
  containment regardless of how any given impulse is calibrated.
- `[x]` G2. Invisible boundary walls + ceiling added (`worlds/ryugu.sdf`, static
  `world_boundary` model, collision-only/no `<visual>`) fully enclosing the 100x100m
  terrain — 4 walls just inside the heightmap's edge (49m) plus a ceiling at 100m
  (far above any observed jump apex, ~5.6m largest so far). Committed `13f6011`.
  Verified: loads cleanly (`gz model -m world_boundary -p` confirms correct pose),
  confirmed genuinely invisible in a wide-angle screenshot. **Not yet observed live**
  — no test flight has actually reached a boundary to confirm the collision response
  in practice (not practical to wait for given how slowly the robot moves; trusting
  Gazebo/DART's well-established box collision handling here).

---

## Previously active: Verify Safe Landing (Claude Code, 2026-07-14) — folded into Phase B/A above
- `[x]` Found & fixed the actual blocker: DART auto-sleep was freezing the robot mid-flight. No jump had ever completed a landing before this fix.
- `[ ]` Ensure legs act as dampeners upon ground contact (restitution coefficient $= 0.15$) — restitution confirmed present in `regolith_plane/model.sdf`, landing_controller logic confirmed sound by code review, but a live end-to-end landing was not yet observed (Ryugu's gravity makes flights very slow; ~5-6x real-time-factor ceiling on this hardware). Revisit as part of Phase B live-testing above.
- `[ ]` Verify self-righting (leg inversion) capabilities if landing fails — **FOUND: not implemented anywhere in code.** Joint range physically supports it (±3.14 rad hip limits) but nothing drives it. **Scoping decision (user, 2026-07-14): documented as future work, not blocking.**

## Bugs found during verification (Claude Code, 2026-07-14)
- `[x]` **DART auto-sleep bug (critical)**: fixed with `<allow_auto_disable>false</allow_auto_disable>`.
- `[x]` **Generator/model.sdf drift**: generator referenced a non-existent plugin and had different mass/inertia than the tested, committed model.sdf. Restored to match known-good + the one fix.
- `[ ]` `hopper_locomotion.py` computes `v_req` but never uses it — folded into Phase A3 above.
- `[ ]` `monitor_height.py`/`monitor_joints.py` are dead (subscribe to nonexistent topics) — use `monitor_gz_pose.py`/`monitor_height2.py` instead.
- `[x]` All uncommitted work committed and pushed to `github.com/Phroggo/ryugu`.
- `[x]` Fixed `setup.py` packaging gap (worlds/models never synced to install/).
- `[x]` Committed the auto-sleep + setup.py fixes locally (`a18928b`) — **not yet pushed**, push deferred per user request.

---

## Phase 1: Realistic Model Detail
- `[x]` Add MLI thermal blanket panels, antenna mast, camera housing, thermal louvers, corner brackets, status LED strip, foot pads, joint housings, RW housing ring — all done in prior session
- `[x]` Regenerate SDF and verify in Gazebo

## Phase 2: Research-Backed Improvements
- `[x]` Create landing_controller.py (impedance-based compliant landing)
- `[x]` Refactor hopper_locomotion.py (timer-based state machine)
- `[x]` Add LIDAR sensor to model SDF
- `[x]` Enhance attitude_controller.py (full PID + momentum desaturation)

## Phase 3: Launch & Build Updates
- `[x]` Update setup.py with new entry points
- `[x]` Update ryugu_swarm.launch.py with new nodes and bridges
- `[x]` Final build, launch, and verify (git recovery + auto-sleep fix session, 2026-07-14)

## Single-Bot Kinematics (prior session)
- `[x]` Jumping controller, in-flight attitude control, jump speed optimization, targeted scientific traversal — all complete per prior walkthrough.md

---

## Phase H: Deep actuation tuning + swarm logic + docs pass (Fable, 2026-07-15)

User request: go through all actuation (RWs, drill, legs), tune properly, kill RW
oscillation; improve swarm role-assignment; upgrade research docs with references;
full scientific-accuracy pass; update HANDOFF with an explicit checklist.

- `[x]` **Root-caused and fixed the persistent yaw spin**: velocity-proportional RW
  commands transfer zero momentum at steady state (wheel reaches speed, torque stops).
  Rewrote to torque-based momentum pumping (PD -> body torque clipped to 0.015 Nm ->
  wheel accel integral), gains sized to whole-robot inertia, overdamped (zeta ~1.1-1.6).
  Live-verified: 107° yaw slew converges + holds at zero rate; 165° tumble damped to
  3.6° in ~20s. (`9de61d2`)
- `[x]` 1° attitude deadband (stops overnight windup-to-saturation against terrain
  tilt); no rate deadband (caused ±1.2° limit cycle, verified + removed); per-axis
  speed clamp; RW speed ceiling corrected to the Maxon EC 20 flat datasheet (982 rad/s).
- `[x]` **Landing detection rebuilt for micro-gravity**: resting reads as free-fall to
  an accelerometer (~1e-4 m/s²); old logic bounced back to FLIGHT forever on a settled
  robot. Now: velocity-gated bounce discrimination + rest-window detector (2cm/60s +
  5mm/s, two-sided apex-dwell analysis) + liftoff watchdog in LANDED + IDLE self-arming
  both directions. Each mechanism live-verified at least once. (`9de61d2`, `b876c87`)
- `[x]` **Found + fixed the LANDED→liftoff kick loop**: grounded RW bleed at 50 rad/s²
  dumps wheel momentum 240x faster than ground friction can absorb → ~0.5 rad/s body
  spin → legs slap terrain → unplanned hop after every landing. Bleed now 0.2 rad/s²
  (at friction capacity). Also: rest-path contacts no longer snap the compliant
  posture (a posture step at rest is itself a launch impulse — measured 0.023-0.055
  m/s kicks), and the post-landing stand-up ramps over 15s. (`b876c87`)
- `[x]` **Terrain wedge-in jam found**: feet left splayed under position-controller
  load wedge into heightmap crevices — leg commands then move NOTHING (verified: gz
  topic echo fine, link poses bit-identical) until lifted clear. Post-landing stand-up
  to unloaded neutral stance prevents it. (`9de61d2`)
- `[x]` Leg joints given real damping (1e-5 → 5e-3 N·m·s/rad in the generator): with
  none, touchdown bounced near-losslessly for tens of minutes. (`b876c87`)
- `[x]` Idle-recovery timer 30s → 5 min and landed-gated (kept kicking the robot
  between normal mission phases; a µg land+settle cycle alone takes ~45-60s).
- `[x]` **swarm_manager role assignment reworked**: real auction (distance + battery +
  carousel bid, 30% SoC reserve), corrective re-hops for short landings (90s cooldown,
  max 5, then requeue), task requeue on RECHARGE-flee/offline, 10s odometry liveness
  watchdog, anomalies clamped to ±45m (walls at ±49m), 8s drill dwell. (`b876c87`)
  *Auction/re-hop logic is code-complete but not yet observed end-to-end live.*
- `[x]` Research_Paper.md + research_report.md: corrected I_bot (0.0055 → 0.012-0.020
  posture-dependent, derivation shown), H_max (0.377 → 0.265 N·m·s per motor spec),
  bang-bang correction time (1.07s → 2.24s, and why the old formula answered the wrong
  question), spin-out margin (44x → 31x + windup amendment); added §3.2.1/§4.1.2
  (momentum-pumping rewrite), §3.3/§10 (µg landing detection + ground handling),
  §4.3/§11 (swarm auction); references extended with Sidi, Wie, MINERVA, MASCOT,
  Maxon datasheets, Gerkey & Matarić.
- `[x]` **✅ SOLVED: ground-jump liftoff (5th session, `5d37147`)** — first verified
  liftoff ever: separation 0.0398 m/s (2.2× threshold), sustained multi-meter ascent.
  Decisive root cause: landing_controller published the stand pose at 100 Hz forever
  post-landing, overwriting every hopper leg command — all prior "stalled crouch"
  data was contaminated. Fixes: stand-pose publish stops after ramp; hopper
  re-asserts targets every tick; leg PID p 0.05→1.0 d 0.01→0.05 (torque cap
  untouched); impact soft-posture now ramped ~2 s (step-at-contact was pogo-kicking
  0.7–0.9 m bounces with the stiffer legs). V_FULL calibrated 0.08→0.04 (measured).
  Full story: HANDOFF checklist item 1.
- `[ ]` OPEN: post-liftoff flight-phase tumble measurement with the new controller
  (expected: rates damped to ~0 within seconds, no persistent wz).
- `[ ]` OPEN: swarm auction/re-hop observed live end-to-end.


## Phase I: Liftoff completion + landing settle (2026-07-15/16)

- `[x]` **🚀 LIFTOFF** (`5d37147`, pushed): sep-v 0.0398 m/s, multi-meter ascent. Root
  cause of every prior stall: landing_controller's 100 Hz stand-pose flood overwriting
  hopper leg commands. + tick re-assertion, leg PID p 0.05→1.0, ramped impact posture.
- `[x]` **Landing settle** (`bb922ee`, pushed): hands-off contact + physical joint
  damping 0.15 (three active-control approaches all ADDED energy — full data in commit
  msg + HANDOFF checklist 1b). Spawn settle AND post-hop impact landing both confirmed
  LANDED with decaying bounces; full mission loop ran under swarm_manager.
- `[x]` Joint-state telemetry: `/scout_1/joint_states` (plugin + bridge) now live.
- `[x]` V_FULL calibrated 0.08→0.04 (measured full-stroke hop).
- `[ ]` OPEN (#1): recover strong hops without pogo — damping tradeoff data + candidate
  directions in HANDOFF checklist item 2. ⛔ p=5/damping=0.4 freezes joints (item 1b).
- `[ ]` OPEN: post-liftoff flight tumble measurement (HANDOFF item 3).
- `[ ]` OPEN: full swarm mission cycle incl. drill dwell/stow/chain (HANDOFF item 5).
