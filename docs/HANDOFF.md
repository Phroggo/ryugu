# 🚀 Project Handoff: Ryugu Asteroid Swarm Hopping Robot Simulation

> [!IMPORTANT]
> **2026-07-16 (LATEST): LIFTOFF ACHIEVED AND LANDING SETTLE SOLVED — READ THE
> "✅ CHECKLIST FOR THE NEXT AGENT" SECTION FIRST (user-mandated).** The first
> verified ground jump in project history happened 2026-07-15 (separation velocity
> 0.0398 m/s, multi-meter ascent — commit `5d37147`); the decisive root cause of every
> historical "crouch stalls / legs pinned" failure was **landing_controller publishing
> its stand pose at ~100 Hz forever after landing, silently overwriting every hopper
> leg command within ~10 ms**. The subsequent pogo-landing problem was solved with
> physical joint damping after THREE active-control approaches all measurably ADDED
> energy on contact (`bb922ee`). Current stable operating point: leg PID p=1.0/d=0.05,
> joint damping 0.15 — clean settles and confirmed LANDED cycles, full mission loop
> runs, but hops are weak (few mm/s; recovering strong hops without pogo is checklist
> item 2, the #1 open item). **Everything is committed AND pushed to `origin/master`
> through `bb922ee`.** ⛔ Two hard-won warnings: never let any node publish leg
> targets outside its owned state-machine phase (last-write-wins fights are silent),
> and do not raise joint damping ≥0.4 (freezes the joints — see checklist 1b).

> [!IMPORTANT]
> **Superseded 2026-07-15 status (kept for history — Fable deep-tuning session).**
> This session rewrote the RW attitude
> controller to torque-based momentum pumping (the persistent yaw spin was a
> control-STRUCTURE bug: wheel-velocity-proportional commands transfer zero momentum at
> steady state — live-verified fixed: 107° yaw slew converges and holds at zero rate,
> 165° tumble damped to 3.6° in ~20 s, no oscillation, ζ≈1.1–1.6 overdamped by design);
> rebuilt landing detection for micro-gravity (resting is *indistinguishable from
> free-fall* to an accelerometer — rest-window + velocity gates + liftoff watchdog, each
> live-verified); found and fixed a whole family of "ground handling" failures unique to
> µg (every leg-posture step is a launch impulse; the grounded RW bleed was kicking the
> robot airborne after EVERY landing; feet wedge into heightmap crevices and silently
> zero all jump thrust); reworked swarm_manager role assignment into a real auction with
> re-hops/liveness/task-recovery; and did a full scientific-accuracy pass on both
> research docs (I_bot, H_max, correction-time math were all wrong vs. the actual
> model). **STILL OPEN at session end: a ground jump has NOT yet been observed lifting
> off under the delta-based launch scheme** — thrust physics understood and instrumented
> (measured 8 mm/s delivered vs 18.5 mm/s needed at amplitude 0.77; scales with
> amplitude), foot-only leg collisions + amplitude recalibration were the in-flight fix
> attempt when the session ended. Details in the checklist below.

> [!IMPORTANT]
> **Superseded 2026-07-14 status (kept for history): Landing verification complete,
> attitude-control bug fixed, world containment added, swarm dashboard GUI built.** All of `task.md`'s Phase A–G
> items are done, including the entire "Verify Safe Landing" section (B1–B6) —
> first-ever confirmed full jump→land cycle, self-righting engaging on a real inverted
> landing, and a genuine attitude-control instability (Euler-angle PID oscillating at
> large tumble angles, plus a separate bug where roll/pitch correction never stopped
> after landing) found via the user's own live observation ("it's spinning around") and
> fixed by rewriting to quaternion-based tilt feedback (`b68ca4f`). Post-fix:
> self-righting succeeds on the first attempt instead of needing all 5 retries, and the
> robot is measurably motionless after landing (`angular_velocity` ~1e-15 rad/s vs. 4.12
> rad/s before). Also added: an invisible world-boundary containment box (`13f6011`, 4
> walls + ceiling) so the robot can't exceed escape velocity (~0.32 m/s — genuinely
> reachable given how weak Ryugu's gravity is, see `Research_Paper.md` §3.1.1) or drift
> off the 100×100m terrain edge; and a Tkinter swarm-status dashboard GUI (`329c552`,
> `swarm_gui.py`) showing per-bot role/activity/battery/charge-rate/attitude-gyro/
> legs/drill/RW telemetry for all 3 bots (scout_2/3 show OFFLINE — not yet spawned),
> auto-docked at the right 1/4 of the screen via `wmctrl` on every launch (sim takes the
> left 3/4). **Nothing is pushed to `origin/master` past `ccca17f` yet** — see the Git
> Checkpoint section immediately below for the exact commit list and what's local-only.
> **If picking this up cold, read the "Guidance for next agent" section near the bottom
> of this file first** — it lists exactly what's still open (LIDAR decision,
> multi-agent scaling) and how to approach each item. `task.md` has the full checkbox
> detail; `walkthrough.md` has the narrative version; `research_report.md` §9 has the
> deepest methodology writeup (especially §9.1, a real infrastructure bug that silently
> broke IMU/RW/landing verification in *every* prior session, not just this one).

> [!WARNING]
> **Recurring pattern: the user's machine/session has crashed and killed all
> background sim processes at least twice this session.** Symptoms: `ps aux | grep gz`
> comes back empty, and any background task IDs you were tracking report `stopped`/
> `failed` with no completion record. This is NOT a bug in the sim or your code — it's
> the sandboxed environment tearing down. **Recovery is fast and mechanical**: (1)
> confirm nothing survived (`ps aux | grep -E "gz sim|ros2 launch"`), (2) re-launch per
> the Build & Run Commands below, (3) if you had a background monitoring script running
> in the scratchpad (`/tmp/claude-*/.../scratchpad/`), check if the directory survived —
> it may have been wiped too, in which case just recreate the script and restart it. Git
> state survives these crashes fine (already-committed/pushed work is untouched) — only
> ephemeral background processes and scratch files are at risk. Don't panic-diagnose the
> sim itself when this happens; just check `ps aux` first.

> [!IMPORTANT]
> **2026-07-14: Project moved from Antigravity to Claude Code.**
> Claude Code now works from `/home/melvin/ryugu_v2_ws/CLAUDE.md`, which points back to
> this file. This file, `task.md`, and `walkthrough.md` are being kept updated per the
> user's request as work continues. This session also discovered and fixed a **real gap**:
> almost everything built after "Checkpoint 1" (Jul 8) was never committed to git — see
> the Git Checkpoint section below, now corrected.

> [!SUCCESS]
> **PREVIOUS ISSUES RESOLVED**
> The broken skydome has been fully restored from the `git` checkpoint (the issue was corrupted PBR material overrides in SDF). The Milky Way skybox renders perfectly.
> Physics for a single isolated bot (`scout_1`) are fully parameterized: 2.5kg mass, joint inversion limits, reaction wheels, motor torque capped at 134 mNm to match the Maxon RE 13 spec. A 5m vertical jump test previously reached apex ~5.57m.

> **PURPOSE**: This file is a complete handoff document. If you are a new agent reading this, it contains everything you need to understand the project, what has been done, what remains, and how to continue. **You MUST update this file, the task.md, and the walkthrough.md as you make progress.**

---

## 🛑 Git Checkpoint (Environment Restoration) — CORRECTED 2026-07-14

The real git repo is at **`/home/melvin/ryugu_v2_ws/src/ryugu_sim/`** (not the workspace
root — `ryugu_v2_ws/` itself is not a repo). Remote: `https://github.com/Phroggo/ryugu.git`.

- **Checkpoint 1** (`fdcc902`, Jul 8): worlds/, and the environment models (regolith
  heightmap, skydome, ryugu). This is the "perfect, working version of the skydome and
  terrain" — revert to this specific commit if world generation/rendering breaks again.
- **Checkpoint 2** (`108593b`, Jul 14): everything built since — all `ryugu_sim/*.py`
  control nodes, `launch/`, `models/spacehopper/model.sdf`, `setup.py`, `package.xml`,
  `scripts/`. **Before this session, none of this was committed or backed up anywhere** —
  it existed only on local disk. It is now committed and pushed to `origin/master`.

- `a18928b` (Jul 14): the DART auto-sleep fix + `setup.py` packaging fix described below.
- `01c9f20` (Jul 14): the full realism + swarm intelligence pass — drill/sampler
  SAMPLER-role fixes, real odometry tracking, jump-distance targeting, idle recovery,
  self-righting, drill housing geometry, LEDs/hazcams, and the IMU/odometry
  gz-transport bridge fix (see "Guidance for next agent" below for the bridge fix
  specifically — it's the single most important thing to know about this environment).
- `6aee345` (Jul 14): reverted an initial attempt at real LED point lights back to
  emissive-only (Gazebo GUI gizmo clutter issue, later actually fixed — see `ccca17f`).
- `ccca17f` (Jul 14): full visual/material realism pass — PBR everywhere, all 6 chassis
  faces gold MLI foil, differentiated leg materials, redesigned antenna, 16 real lights
  including 2 functional headlight spotlights, all using the documented SDF
  `<visualize>false</visualize>` element to suppress GUI gizmos without killing the
  actual light (the real fix to the `6aee345` issue).

**✅ AS OF 2026-07-16, EVERYTHING BELOW IS ALSO PUSHED to `origin/master`** (through
`bb922ee`; the user requested pushes twice on Jul 15–16 — local and remote are in
sync, working tree clean). Commit-by-commit history:

- `7ba3977` (Jul 14): fixed `attitude_controller.py` never exiting flight mode after
  landing (no `/landed` subscription existed at all — `in_flight` stayed `True` forever
  once a jump started).
- `b68ca4f` (Jul 14): rewrote attitude control from Euler-angle PID to quaternion
  cross-product tilt feedback — the Euler-angle approach was found live-oscillating at
  large tumble angles (1.5–2.8 rad instead of converging). See the "Guidance for next
  agent" section and `research_report.md` §4.1.1 for the full math/derivation.
- `13f6011` (Jul 14): added the invisible `world_boundary` containment model (4 walls +
  ceiling) to `worlds/ryugu.sdf` — user-requested, prevents exceeding escape velocity or
  drifting off the terrain edge.
- `329c552` (Jul 15): added the Tkinter swarm-status dashboard GUI (`swarm_gui.py`) plus
  `swarm_manager.py` status-topic publishing and the `wmctrl` auto-window-layout launch
  step.
- `c461cc2` (Jul 15): **fixed a real false-positive landing-detection bug**, found via
  direct user observation (a screenshot showing the robot floating ~4.8m above the
  ground while the dashboard read `LANDED`). The contact-detection acceleration
  threshold (0.02 m/s²) was far too sensitive — RW/leg-motor reaction torque routinely
  produces transient accelerations in that same range, unrelated to actual ground
  contact. This was very likely the real explanation behind an earlier report of the
  drill appearing to deploy while airborne (a false `landed=True` reaching
  `swarm_manager`'s SAMPLER gating). Fixed with a raised threshold (0.08 m/s²) plus a
  new odometry-based velocity cross-check — live-verified to correctly reject 5
  false-positive triggers (at 0.10–0.25 m/s, genuinely still flying) in the first 15s of
  a single test flight. **If you ever see the robot registering `LANDED` while visibly
  airborne again, this class of bug is the first thing to suspect** — check
  `landing_controller.py`'s new velocity gate and its log warnings.
- `9de61d2` (Jul 15, Fable): **torque-based momentum-pumping rewrite of
  `attitude_controller.py`** (fixes the persistent yaw spin at its structural root) +
  micro-gravity landing-detection rebuild (rest-window detector, bounce velocity gate,
  IDLE self-arming) + post-landing stand-up + idle-recovery timer 30s→5min. See
  `research_report.md` §4.1.2 and §10 for full derivations.
- `b876c87` (Jul 15, Fable): **fixed the LANDED→liftoff kick loop** (grounded RW bleed
  50→0.2 rad/s² — was dumping wheel momentum 240x faster than Ryugu-weight friction can
  absorb, launching the robot ~2s after every landing; rest-path contacts no longer
  snap the compliant posture; stand-up ramps over 15s) + **swarm_manager auction
  rework** (bid = distance+battery+carousel, corrective re-hops, offline liveness, task
  requeue, ±45m anomaly clamp, 8s drill dwell) + real leg joint damping (1e-5→5e-3;
  touchdowns previously bounced near-losslessly for tens of minutes) + RW speed clamp
  corrected to the motor datasheet (1396→982 rad/s).
- `fafdb03` (Jul 15, Fable): untrack `__pycache__`, add `.gitignore`.
- `85032c8` (Jul 15, Fable): velocity-only rest path (breaks the tilt-pump/
  landing-confirm deadlock), launch window 0.2s→0.5s, foot-only leg collisions +
  2.5cm foot spheres (wedge-jam fix), amplitude recalibration.
- `e2f8e16` (Jul 15): drill mid-flight jiggle fix, per-leg-delta launch (asymmetric
  launch-torque fix), RW command slew limiter (superseded by `9de61d2`'s rewrite).
- `d1ce594` (Jul 15, Fable): DART-sleep `set_pose` wake before CROUCH/IGNITION +
  stroke-fraction launch scaffolding.
- `d50f9f8` (Jul 15): zigzag stroke targets (feet directly under hips — µg friction
  fix; feet grip instead of sliding).
- `5d37147` (Jul 15): **🚀 LIFTOFF — first verified ground jump** (sep-v 0.0398 m/s,
  multi-meter ascent). Root cause of ALL prior stalls: landing_controller's ~100 Hz
  stand-pose publication overwriting every hopper leg command. + tick re-assertion
  in hopper, leg PID p 0.05→1.0, ramped impact posture.
- `bb922ee` (Jul 16): **landing settle solved** — hands-off contact + physical joint
  damping 0.15 (three active-control approaches all ADDED energy, full data in the
  commit message); joint-state telemetry plugin+bridge; V_FULL 0.08→0.04; ⛔ p=5/
  damping=0.4 freeze warning. Current stable operating point.

**Going forward: commit early and often, confirm with the user before pushing** (it's a
real remote, `github.com/Phroggo/ryugu`, authenticated as `Phroggo` via `gh`) — earlier
in this session the user proactively asked for pushes twice, so it's reasonable to
check in after any further coherent chunk of work rather than
defaulting to "always ask first" if they've established a pattern of wanting it pushed.

---

## Project Overview

We are building a **Gazebo Ignition (gz-sim) simulation** of a swarm of 3 asteroid-hopping robots ("scouts") exploring the surface of asteroid **162173 Ryugu** in micro-gravity (g = 0.000114 m/s²). The robots are inspired by **ETH Zurich's SpaceHopper** but use **reaction wheels** for mid-air attitude control (this is a hard requirement from the user — do NOT remove reaction wheels).

Each robot has:
- A cubic chassis body (0.2m)
- 3 legs with hip + knee revolute joints (for hopping)
- 3 reaction wheels (X/Y/Z axes, for attitude stabilization during flight)
- Solar panel on top, core sampler drill underneath
- IMU sensor, LIDAR sensor
- Detailed visual elements: MLI panels, antenna, cameras, thermal louvers, corner brackets, LEDs, foot pads, joint housings

The swarm is managed by a central `swarm_manager` node that assigns exploration tasks via a market-based auction system. Currently only `scout_1` is spawned/active for single-bot testing (swarm manager logic is present but the launch file only spawns one agent).

---

## Workspace & File Map

| Path | Description |
|------|-------------|
| `/home/melvin/ryugu_v2_ws/` | **ROS 2 workspace root** — see `CLAUDE.md` there for the Claude Code-oriented version of this map |
| `src/ryugu_sim/` | Main ROS 2 package (**and the actual git repo root**) |
| `src/ryugu_sim/ryugu_sim/attitude_controller.py` | Reaction-wheel attitude controller. **Torque-based momentum pumping** (rewritten again `9de61d2` — PD attitude law → body torque clipped to the real 15 mNm budget → wheel-acceleration integral; the earlier velocity-proportional law could not null steady-state error at all). Quaternion cross-product tilt error (`b68ca4f`), 1° angle deadband (windup guard), rate damping always active, per-axis 982 rad/s clamp, GENTLE grounded bleed (0.2 rad/s² — see the µg ground-handling gotcha). `in_flight` tracks `/landed` in both directions. |
| `src/ryugu_sim/ryugu_sim/hopper_locomotion.py` | Tri-pedal jump state machine: IDLE→CROUCH→LAUNCH→FLIGHT→(back to IDLE on landing). Launch is a per-leg **delta** from each leg's crouch pose (balanced thrust), 0.5s launch window, amplitude scales with `v_req` (calibration redone 2026-07-15 against measured delivered delta-v). 5-min idle recovery hop, gated on actually-landed. |
| `src/ryugu_sim/ryugu_sim/swarm_manager.py` | Swarm coordination node — **real market-based auction** since `b876c87` (bid = distance + battery + carousel penalties, 30% SoC reserve), corrective re-hops (90s cooldown, max 5, then requeue), 10s odometry-liveness/OFFLINE handling, task requeue on RECHARGE-flee, ±45m anomaly clamp, 8s drill dwell. Publishes `jump_target_distance` + `target_yaw` + `/status_*` topics. Fires autonomously within seconds of spawn — races any manually-triggered jump unless killed first (`pkill -9 -f "lib/ryugu_sim/swarm_manager"`) |
| `src/ryugu_sim/ryugu_sim/landing_controller.py` | Micro-gravity landing state machine: contact spike + rest-window detector (2cm/60s z-band AND a 120s velocity-only path) + bounce velocity gate + liftoff watchdog in LANDED + IDLE self-arming both directions (all `9de61d2`/`b876c87`/final commit). Post-landing ramped stand-up to an unloaded stance (wedge-jam prevention). Detects inverted landings and runs self-righting (splay/asymmetric-sweep, 5 retries). |
| `src/ryugu_sim/ryugu_sim/swarm_gui.py` | **New (`329c552`)**: Tkinter mission-control-style dashboard — per-bot role/activity/battery+rate/landed-state/attitude-gyro/legs/drill/RW telemetry for all 3 bots (offline bots shown greyed out). Run via `ros2 run ryugu_sim swarm_gui` standalone, or launched automatically as part of `ryugu_swarm.launch.py`. |
| `src/ryugu_sim/ryugu_sim/spawner.py` | Model spawner utility |
| `src/ryugu_sim/launch/ryugu_swarm.launch.py` | Main launch file — spawns 1 scout + all control nodes + the dashboard GUI, and (since `329c552`) auto-positions windows via a delayed `wmctrl` call (sim at left 3/4 of screen, dashboard at right 1/4) |
| `src/ryugu_sim/setup.py` | Package setup with console_scripts (includes `swarm_gui` entry point) |
| `src/ryugu_sim/worlds/ryugu.sdf` | World SDF (gravity, lighting, ground, skydome, and since `13f6011` the invisible `world_boundary` containment model) |
| `src/ryugu_sim/models/spacehopper/model.sdf` | **Robot SDF** (generated by script below — never hand-edit) |
| `src/ryugu_sim/models/regolith_plane/` | Ground plane model with rocky texture, `restitution_coefficient=0.15` |
| `src/ryugu_sim/models/skydome/` | Space skybox dome (PBR material for Ogre2) |
| `src/ryugu_sim/scripts/trigger_jump.sh`, `monitor_*.py` | Manual test tools. **`monitor_height.py`/`monitor_joints.py` are broken** — they subscribe to `/scout_1/odom` and `/joint_states`, neither of which exist (no odometry publisher plugin, nothing bridges joint_states). Use `monitor_gz_pose.py`/`monitor_height2.py` instead — they read Gazebo transport topics directly and actually work. |

### SDF Generator Script
| Path | Description |
|------|-------------|
| `/home/melvin/.gemini/antigravity-ide/brain/a61c73d1-b230-4862-89c8-26d7f5a72a09/scratch/generate_detailed_spacehopper.py` | **Python script that generates model.sdf** — all model changes go here, then run `python3` on it to regenerate |

### Agent Tracking Documents (Artifact Directory)

**Artifact root (current)**: `/home/melvin/.gemini/antigravity-ide/brain/534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/`
An older/superseded artifact dir `a61c73d1-b230-4862-89c8-26d7f5a72a09/` has stale
duplicate filenames from the Jul 7-8 session — the `534489f2-...` dir is authoritative.

| File | Description |
|------|-------------|
| `HANDOFF.md` (this file) | Complete project context for agent handoff |
| `implementation_plan.md` | Approved implementation plan for the current/most recent phase |
| `task.md` | **Task checklist — UPDATE THIS as you work** |
| `walkthrough.md` | Summary of completed work — **UPDATE when done** |
| `research_report.md` | Methodology log (heightmap, skybox, physics) for academic reference |
| `Research_Paper.md` | Formal write-up of SpaceHopper design (abstract, math, tables) — note some claims (e.g. "100% self-righting probability") describe intended design, not verified/implemented behavior. See Remaining Work. |

---

## Technology Stack

- **ROS 2 Humble** (Ubuntu 22.04)
- **Gazebo Sim (Harmonic, `gz` 8.14)** with Ogre2 renderer
- **Python 3** for all ROS nodes
- **SDF 1.8** format for models/worlds
- **colcon** build system

### Build & Run Commands
```bash
# Build
cd /home/melvin/ryugu_v2_ws && colcon build

# Run (must source first)
source install/setup.bash && ros2 launch ryugu_sim ryugu_swarm.launch.py

# Regenerate robot SDF from generator script
python3 /home/melvin/.gemini/antigravity-ide/brain/a61c73d1-b230-4862-89c8-26d7f5a72a09/scratch/generate_detailed_spacehopper.py

# Trigger a jump manually (swarm_manager will race this within a few seconds of
# spawn unless killed first — pkill -9 -f "lib/ryugu_sim/swarm_manager")
bash src/ryugu_sim/scripts/trigger_jump.sh <distance>
```

**Process hygiene**: `gz sim`, `ros2 launch`, and `parameter_bridge` processes do not
reliably die with `killall gz ruby` — this session found 10+ orphaned launch sessions
from Jul 13 still running a day later. Kill by exact PID
(`ps aux | grep -E "gz sim|ros2 launch|parameter_bridge"`) to be sure before relaunching.

---

## Critical Design Decisions & Gotchas

### ⛔ Joint pose bug
SDF joints MUST have `<pose relative_to="CHILD_LINK">0 0 0 0 0 0</pose>` — NOT `relative_to="__model__"`. Using `__model__` puts the joint pivot at world origin, causing legs to detach and fly away. This was the #1 bug that took hours to fix.

### ⛔ ros_gz_bridge Gazebo-version mismatch (found & fixed 2026-07-14 — READ THIS FIRST)
**If IMU, odometry, or any other GZ→ROS sensor topic "exists" (`ros2 topic list` shows
it, nodes are connected per `ros2 topic info -v`) but never actually delivers data,
this is almost certainly the same bug recurring** — e.g. after a machine/environment
reset, a fresh `apt upgrade`, or a new dev machine.

**Root cause:** this machine's simulator is Gazebo **Harmonic** (`gz-sim8`,
`gz-transport13`, `gz-msgs10`), but the plain `ros-humble-ros-gz-bridge` apt package is
built for Gazebo **Fortress** (`ignition-transport11`, `ignition-msgs8`). These are not
wire-compatible for complex message types (IMU, Odometry, LaserScan) — simple types
(`std_msgs/Float64`, used for joint commands) happened to still work, which is why
leg/drill motion looked fine while RW/landing verification kept coming up empty. This
silently broke IMU delivery to `attitude_controller.py` and `landing_controller.py` in
**every session before 2026-07-14**, without a single error message anywhere.

**How to diagnose it again:** `gz topic -i -t <the gz-side topic>` — if it reports **no
subscriber** despite `ros2 topic info -v` showing connected ROS subscribers, suspect
this. Confirm with `ldd $(which parameter_bridge) | grep -i transport` — if it shows
`libignition-transport11`, that's the Fortress build. Cross-check against
`ldd /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/lib*.so | grep -i transport` — should show
`libgz-transport13` for a Harmonic install. A mismatch between those two is the bug.

**Fix:** `sudo apt install ros-humble-ros-gzharmonic-bridge` (needs sudo — ask the user
to run `sudo -v` in their own terminal first if you don't have cached credentials, since
typing a password into chat isn't a good idea; **don't just guess and skip this step**,
it silently invalidates any RW/landing/self-righting verification). It replaces
`/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge` in place via package alternatives —
no launch file path changes needed, just verify with the `ldd` check above afterward.
Separately, also check the IMU sensor plugin in `model.sdf`/the generator script isn't
still `ignition-gazebo-imu-system` (old) instead of `gz-sim-imu-system` (current) — this
machine has both installed side-by-side and gz-sim will silently load the wrong one
without erroring. And in `launch.py`'s bridge config, GZ→ROS entries (the ones using the
`[` bracket direction) need the actual wire-level type name (`gz.msgs.IMU`, confirmed via
`gz topic -i`), not the legacy `ignition.msgs.IMU` alias — the ROS→GZ direction (`]`
bracket) tolerates the legacy alias fine, but GZ→ROS did not in this bridge build.

### ⛔ Ogre2 material bug
The Ogre2 renderer rejects fixed-function pipeline materials (phong shaders in .dae files). All materials must use `<pbr><metal>` blocks in SDF, or at minimum have `<ambient>`, `<diffuse>`, and `<specular>` tags. Missing `<specular>` causes a black screen crash.

### ⚠️ Reaction wheels are MANDATORY
The user explicitly requires reaction wheels for attitude control. Do NOT replace them with leg-based reorientation (even though the real SpaceHopper uses legs). The user's design is a hybrid: legs for hopping, reaction wheels for stabilization.

### ℹ️ Skydome material
The skydome uses PBR material in its SDF (not the embedded Collada phong material) because Ogre2 rejects the Collada's fixed-function shader. The texture is `materials/textures/space_skybox.png`.

### ✅ RESOLVED (kept for history): jump distance targeting was non-functional
Fixed `01c9f20` (amplitude scales with `v_req`), then substantially re-worked 2026-07-15:
the launch stroke is now a per-leg *delta* from each leg's own crouch position (equal
angular travel = balanced thrust, fixing a launch torque impulse), the launch window is
0.5s, and the amplitude calibration was redone against measured delivered delta-v (see
the checklist — final liftoff verification was still in flight at session end).

### ⛔ Micro-gravity ground handling — every one of these was a REAL, live-caught bug (2026-07-15)
1. **Resting is indistinguishable from free-fall to an accelerometer** (a resting robot
   reads ~1e-4 m/s² proper acceleration). Never write "accel below threshold = flying".
   `landing_controller.py`'s rest-window detector + velocity gates exist for this.
2. **Every leg-posture step at rest is a launch impulse** (measured 0.023–0.055 m/s
   kicks = multi-meter unplanned hops). All at-rest posture changes must be ramped
   (stand-up interpolates over 15s); rest-path contacts must not snap the compliant pose.
3. **The grounded RW bleed must stay within ground-friction torque capacity**
   (µ·N·r ≈ 5.7e-5 N·m at Ryugu weight). Bleeding wheels at the control ceiling dumps
   their momentum into the BODY (0.5 rad/s spin → leg-slap → liftoff ~2s after every
   landing — this was live-observed on every single landing until fixed). Bleed is now
   0.2 rad/s²; wheels parking with stored speed for minutes is fine and physical.
4. **Feet/legs wedge into heightmap crevices** hard enough that the 0.134 N·m leg
   motors cannot move them AT ALL (commands echo gz-side, link poses bit-identical, zero
   thrust — verified by teleport-free test). Fix: foot-only leg collisions (thigh/calf
   cylinder collisions removed — standard legged-sim practice), 2.5cm foot spheres,
   post-landing stand-up to an unloaded stance. If jumps ever silently produce zero
   motion again, check for this FIRST (compare `dynamic_pose` link poses before/after a
   manual hip command).
5. **Near-zero joint damping = near-lossless bouncing** (3–5 mm/s touchdowns rang for
   tens of minutes at ~2% energy loss/cycle). Leg joints carry 5e-3 N·m·s/rad damping
   in the generator now — don't zero it.
6. **The tilt-pump/landing-confirm deadlock**: in-flight tilt control (armed because
   not-landed) rocks the grounded robot just enough to keep resetting the landing
   detector that would disarm it. The velocity-only rest path (120s under 5 mm/s —
   apex-safe because a pure-vertical apex only satisfies it for ≤88s) breaks the loop.

### ⚠️ `colcon build` silently never synced `worlds/` or `models/` (found & fixed 2026-07-14)
`setup.py`'s `data_files` only listed `package.xml` and `launch/*.py` — `worlds/` and
`models/` were never copied to `install/share/ryugu_sim/`. Editing `worlds/ryugu.sdf`
(e.g. to change gravity or `real_time_factor`) had **zero effect** after a rebuild,
because the launch file loads the world from the install share directory while
`colcon build` kept serving a stale copy frozen at whatever it was when `install/` was
first populated. (Robot/environment models were unaffected because the launch file
separately overrides `GZ_SIM_RESOURCE_PATH` to point straight at `src/.../models/` — only
the world file was stale.) Fixed by adding a `package_files()` helper in `setup.py` that
recursively installs everything under `worlds/` and `models/`. **Always run
`colcon build` after editing `worlds/ryugu.sdf` and confirm with
`diff src/ryugu_sim/worlds install/ryugu_sim/share/ryugu_sim/worlds` if a world-level
change doesn't seem to take effect.**

### ⛔ DART auto-sleep was freezing every jump mid-flight (found & fixed 2026-07-14)
Ryugu's gravity (0.000114 m/s²) produces velocities small enough that DART's default
auto-sleep threshold was putting `scout_1` to sleep mid-flight, **permanently freezing
it in the air**. Confirmed empirically: the model's pose (to 10 significant figures)
was bit-identical across multiple samples spanning real minutes. This means no jump had
ever actually completed a flight-to-landing cycle before this session — "safe landing"
could not have been verified previously because the robot never reached the ground.
**Fix**: added `<allow_auto_disable>false</allow_auto_disable>` to
`generate_detailed_spacehopper.py`'s model template (and regenerated `model.sdf`).
While fixing this, also discovered the generator script had **drifted significantly**
from the actual tested/committed `model.sdf` — it referenced a non-existent plugin
(`gz-sim-joint-velocity-controller-system` instead of the real
`gz-sim-joint-controller-system`) and had different base_link mass/inertia values.
Someone previously hand-patched `model.sdf` directly without back-porting the fix into
the generator, silently breaking the "always regenerate via script" rule this doc
itself states. **Before trusting the generator script again, diff its output against
the last known-good committed `model.sdf` line-by-line** — don't assume it's in sync.

### ✅ RESOLVED (kept for history): live "safe landing" observation
Completed 2026-07-14 (see "Guidance" §1 below) and then substantially hardened
2026-07-15 (see the micro-gravity ground-handling gotcha above — the original
"confirmed" landings turned out to sit atop several real, then-unknown failure modes).
Original 2026-07-14 notes follow:

### (historical) Live "safe landing" observation not completed this session (2026-07-14)
With the auto-sleep bug fixed, the robot genuinely flies and moves — confirmed via
changing (non-frozen) pose samples. However, a full natural jump→apex→fall→contact
cycle was **not observed end-to-end** within this session, for two compounding reasons:
1. Ryugu's gravity is so weak that even short hops take a very long time to complete
   (multiple minutes of *simulated* time were still in progress with the robot barely
   descending from its post-launch altitude).
2. This machine's Gazebo instance has a hard real-time-factor ceiling around 5-6x
   (confirmed CPU-bound, not GUI-rendering-bound — headless `-s` mode didn't raise it),
   so even sped-up, multi-minute sim-time flights still take many real minutes to watch.

Attempted shortcuts (teleporting the robot to a low altitude via
`/world/ryugu_world/set_pose`, and temporarily boosting gravity via
`/world/ryugu_world/set_physics`) both introduced confounding artifacts — teleporting
mid-oscillation carried over residual velocity that made the robot climb instead of
fall, and a landing-controller state inconsistency appeared that wasn't fully
root-caused before the session ended. **These were live `gz service` calls only, not
file edits — nothing about this is persisted; the committed world/launch files are
back to their original values.**

**What IS confirmed** (by code + config review, not live observation):
- `regolith_plane/model.sdf` correctly sets `<restitution_coefficient>0.15</restitution_coefficient>`.
- `landing_controller.py`'s contact-detection and soft-landing state machine
  (IDLE→FLIGHT→CONTACT_DETECTED→SETTLING→LANDED, with a bounce-back-to-FLIGHT path) is
  structurally sound on inspection.

**What is NOT yet confirmed**: that this logic actually produces a damped (not
bouncy) landing when a real contact event occurs. Recommend either (a) a long
unattended/overnight run left to complete a natural landing cycle, or (b) fixing the
jump-distance-targeting bug below first (shorter, controlled hops would make this much
faster to test), or (c) adding a documented dev-only launch arg to temporarily scale up
gravity for fast iteration (do not leave this on by default — it would invalidate the
physics accuracy the rest of the project cares about).

### ✅ RESOLVED (kept for history): self-righting via leg inversion
Was unimplemented when first flagged (2026-07-14 AM); implemented the same day
(`landing_controller.py` RIGHTING state: splay/asymmetric-sweep, rotating lead leg, 5
retries, give-up fallback) and **live-verified on a genuine inverted landing** — first
attempt success after the attitude-controller rewrite. See `research_report.md` §4.1.

---

## Completed Work ✅

- [x] ROS 2 package structure (ryugu_sim)
- [x] World SDF with micro-gravity (0.000114 m/s²), regolith ground plane, space skydome,
      **invisible world-boundary containment** (static `world_boundary` model — 4 walls
      + a ceiling, collision-only, fully enclosing the 100x100m terrain at 49m/100m —
      prevents both escaping past the terrain edge and exceeding escape velocity
      (~0.32 m/s, genuinely reachable given how weak gravity is here). If you ever see
      the robot mysteriously stop/bounce with nothing visible nearby, **this is why —
      it's not a bug**, check `worlds/ryugu.sdf`'s `world_boundary` model before
      assuming something's wrong.
- [x] Robot SDF with 3 legs (hip+knee), 3 reaction wheels, solar panel, drill (now with a
      proper mounting turret), IMU, full visual detail (MLI, antenna, cameras + stereo
      hazcams, louvers, brackets, emissive-material LEDs, foot pads + proximity sensors,
      joint housings) — **no LIDAR sensor exists despite being listed in the mass table;
      see "Guidance for next agent" below**. LEDs: after an initial revert (GUI gizmo
      clutter), they now have real point lights using `<visualize>false</visualize>` to
      suppress the gizmos (`ccca17f`) — the "reverted to emissive-only" note that used
      to live here is stale. Leg collisions are **foot-sphere-only** since 2026-07-15
      (see the wedge-jam gotcha).
- [x] **Legs properly attached** (joint `relative_to` child link fix)
- [x] **Skydome restored** (PBR material fix for Ogre2)
- [x] Swarm manager node — market-based task auction, real odometry-based position
      tracking (was previously always assumed-origin), arrival-gated SAMPLER drill
      logic, 3-tube carousel with real chaining behavior, role-based battery drain with
      working RECHARGE
- [x] Hopper locomotion node — timer-based state machine, crouch → launch → retract,
      launch amplitude now genuinely scales with requested distance, clean idle-then-
      self-recover behavior
- [x] Attitude controller node — full PID: IMU → reaction wheel velocity commands,
      momentum desaturation, saturation warnings. **Now confirmed actually receiving
      real IMU data** (was silently disconnected in every prior session — see the
      ros_gz_bridge gotcha above)
- [x] Landing controller node — impedance-based compliant landing on IMU contact
      detection, **now also detects inverted landings and runs a self-righting
      maneuver** (splay/asymmetric-sweep, up to 5 retries)
- [x] Launch file spawning scout_1 with all bridges and nodes (bridge config now uses
      correct GZ-side topic names/types — see gotcha above); `setup.py`/`package.xml`
      wired with all entry points
- [x] Research: SpaceHopper paper, MINERVA-II, reaction wheel gyrostat papers, impedance
      control (LIDAR terrain sensing was researched but not implemented — no sensor
      exists in the sim)
- [x] Physics finalized: 2.5kg total mass, motor torque capped at 134 mNm (Maxon RE 13
      spec), knee spring stiffness, joints allow full inversion range
- [x] **The ros_gz_bridge Gazebo Fortress/Harmonic version mismatch found and fixed**
      (2026-07-14) — this had silently prevented IMU/odometry from ever reaching the
      control nodes in any prior session. See gotcha above; this is the single most
      important thing to understand about this environment going forward.
- [x] **All of the above committed to git and pushed to `origin/master`** (through
      `ccca17f`), per explicit user request.
- [x] **Full visual/material realism pass** (2026-07-14, later) — PBR materials
      throughout, all 6 chassis faces wrapped in gold MLI foil (previously alternated
      gold/silver, read as much less coverage than the user pictured for "real space
      bots"), 16 real point/spot lights including 2 functional headlight spotlights
      near the nav camera (addresses "how will the camera see anything in the dark" —
      Ryugu's ambient light is deliberately near-zero for scientific accuracy). All
      lights use the documented SDF `<visualize>false</visualize>` element to suppress
      Gazebo's GUI gizmo markers.

---

## Remaining Work 🔲

### Verify Safe Landing (task.md item 9) — mostly done, one piece still open
- [x] DART auto-sleep bug (robot froze mid-flight, never landed) — fixed, prior session.
- [x] The deeper reason landing/RW verification kept failing — the IMU bridge was
      silently dead — found and fixed this session (see gotcha above).
- [x] Self-righting is now implemented (was previously flagged as unimplemented) — see
      `landing_controller.py`'s RIGHTING state.
- [x] Jump-distance targeting now works — launch amplitude scales with requested
      distance instead of firing an identical impulse every time.
- [ ] **Still open: a full jump→apex→fall→ground-contact cycle hasn't been directly
      observed completing end-to-end**, in this session or any prior one. The sensor
      pipeline feeding that behavior is now confirmed live (IMU at 99.9Hz, RW spinning
      on real feedback), so this is now a "watch it happen" task, not a "find out why it
      never works" task. Ryugu's gravity makes each attempt take many real minutes; a
      long unattended run left to complete naturally is the straightforward path.
- [ ] **Still open: self-righting's actual success rate is unverified.** The maneuver is
      implemented and logic-reviewed, but no genuine inverted landing has been observed
      triggering and completing it live.

### Phase 1: Realistic Model Detail — COMPLETE (drill housing + LEDs/hazcams added 2026-07-14)
### Phase 2 & 3: Research-Backed Control + Launch/Build — COMPLETE

---

## Research Sources (for context)

- **SpaceHopper (ETH Zurich, ICRA 2024)**: arXiv:2403.02831 — 3-legged, 5.2kg, 245mm, DRL attitude control, ESA parabolic flight tested
- **Reaction wheel gyrostat papers (2023-2024)**: Active momentum exchange reduces mid-air angular deviations by >65%, landing errors <3.5°
- **MINERVA-II (JAXA/Hayabusa2)**: Internal torquer mass hopping, first mobile robots on an asteroid
- **NASA ALHAT**: LiDAR-based hazard detection for autonomous landing
- **Impedance control (2024)**: Variable impedance + energy tanks for compliant landings on unknown surfaces

---

## ✅ CHECKLIST FOR THE NEXT AGENT (user-mandated, 2026-07-15 — work through this IN ORDER)

**Done and live-verified this session** (don't redo, but don't regress either):
- [x] RW attitude control: torque-based momentum pumping; 107° yaw slew converges +
      holds within 1° at zero rate; 165° tumble → 3.6° in ~20s; no oscillation
      (overdamped ζ≈1.1–1.6 by construction); no more windup-to-saturation (1° deadband);
      no more grounded phantom wheel offsets. (`9de61d2`)
- [x] Persistent yaw spin: root-caused (velocity-law momentum equilibrium
      ω=L₀/(I+I_w·K_d) + wrap-sawtooth dithering) and eliminated. (`9de61d2`)
- [x] Drill: verified holding rock-solid at commanded position through flights (fixed
      in `e2f8e16`, re-confirmed via dashboard during this session's flights).
- [x] Landing detection: resting-vs-free-fall ambiguity solved (rest windows + velocity
      gates + liftoff watchdog + IDLE self-arm) — each mechanism observed firing
      correctly live at least once, including the watchdog catching real kicks within 2s.
- [x] LANDED→liftoff kick loop: fixed (gentle 0.2 rad/s² RW bleed at friction capacity,
      no posture snaps at rest, 15s stand-up ramp) — verified: a full landing+stand-up
      completed with zero liftoff events and ≤1 mm/s velocity throughout. (`b876c87`)
- [x] Leg joints: real damping (5e-3) so touchdowns dissipate instead of ringing for
      tens of minutes (observed live before the fix). (`b876c87`)
- [x] swarm_manager: real auction + re-hop + liveness + task-recovery logic written and
      code-reviewed. (`b876c87`)
- [x] Research docs: scientific-accuracy pass done (I_bot 0.0055→0.012–0.020
      posture-dependent with derivation; H_max 0.377→0.265 N·m·s per Maxon datasheet;
      correction time 1.07s→2.24s bang-bang; margin 44x→31x + windup amendment;
      §3.2.1/§3.3/§4.3 added to the paper, §4.1.2/§10/§11 to the report; references
      extended with Sidi, Wie, MINERVA, MASCOT, Maxon datasheets, Gerkey & Matarić).

**OPEN — in priority order:**
- [x] **1. Ground-jump liftoff — ✅ SOLVED (2026-07-15, fifth session — commit
      `5d37147`).** First verified liftoff in project history: separation velocity
      0.0398 m/s (2.2× the 0.0185 m/s pass threshold), sustained ascent z 4.91→6.95+
      over 30 s, projected apex ~7 m. THE decisive root cause (found by echoing the
      gz-side joint topic during a stroke): **landing_controller's LANDED branch
      published the stand pose at ~100 Hz forever after the stand-up ramp finished,
      overwriting every hopper leg command within ~10 ms.** Every prior "crouch
      stalls at millimetres" result — and the µg-friction/geometry theories built on
      those results — was contaminated by this override. Four fixes, all in
      `5d37147`: (1) stand-pose publication now stops when the ramp completes;
      (2) hopper re-asserts CROUCH/LAUNCH targets every tick (wins any last-write
      race); (3) leg PID p 0.05→1.0, d 0.01→0.05 (at p=0.05 the full stroke
      delivered only ~4 mm/s — rate-limited 25× below the untouched 0.134 Nm torque
      cap); (4) impact-path soft posture is now a ~2 s ramp — with stiff legs the
      old step-at-contact pogo-kicked the robot 0.7–0.9 m back up every touchdown,
      a non-decaying bounce loop. **µg cardinal rule confirmed twice: every leg
      posture step IS a launch impulse.** V_FULL calibrated 0.08→0.04 from the
      measured hop (next commit). The zigzag stroke geometry (previous update below)
      remains correct and necessary — it was required for grip, just not sufficient.
- [x] **1b. Landing settle after real hops — ✅ SOLVED (2026-07-16, commit `bb922ee`,
      pushed).** The liftoff fix created a pogo problem: with p=1.0 leg gains the
      touchdown restitution was ~0.96 (bounces from a 1.15 m drop never decayed).
      **Every active mitigation ADDED energy** (all live-measured): stepped soft
      posture (kicked 0.7–0.9 m), 2 s ramped posture (in 32 mm/s → out 38 mm/s,
      pogoed to 10+ m), zero-stiffness catch mirroring measured joint angles
      (in 16 → out 22 mm/s — bridged joint-state feedback lags and PUMPS the
      rebound). **µg law, thrice-confirmed: every commanded leg motion while
      grounded is a thruster, and delayed feedback is phase-poison.** Solution:
      contact is fully hands-off; impact energy dissipates in PHYSICAL joint
      damping (`model.sdf`, 0.005 → 0.15 N·m·s/rad; ζ≈0.45, restitution ≈0.2).
      Verified live: spawn settle AND post-hop impact landing both reached
      confirmed LANDED in 2.5–3.5 min with decaying bounces; the full mission
      loop (jump → flight → land → settle → next hop) ran under swarm_manager.
      **The tradeoff:** at damping 0.15 the launch stroke is damping-limited —
      hops drop from 0.0398 m/s separation (damping 0.005) to the few-mm/s range
      (slow ~25 cm ascents). ⛔ **Do NOT try p=5.0 + damping=0.4** to recover
      authority — tried 2026-07-16, it froze the leg joints entirely (zero motion
      on both stand-fold and crouch, joint_states-verified; suspected DART
      explicit-damping discretization limit; warning comment in the generator).
      `/scout_1/joint_states` telemetry (plugin + bridge) was added during this
      work and is available for future tuning.
      Progress this round (commit `d1ce594`):
      (a) **DART sleep: fixed and verified.** `hopper_locomotion._wake_model()` fires an
      in-place `set_pose` before CROUCH and again at IGNITION; the model demonstrably
      responds to joint commands afterward. (gz-sim8 sleeps a quiescent model despite
      `allow_auto_disable=false`; sleeping models ignore ALL joint commands.)
      (b) **Redesigned symmetric crouch/extend stroke with fraction scaling** is in
      place (10 s crouch, 1 s launch window), replacing the sideways-scoop delta scheme.
      (c) **Remaining blocker, precisely isolated over two failed test cycles:** the
      crouch stand-up stalls after ~3 mm of body rise. Root cause is fundamental to
      micro-gravity: total friction capacity is µ·m·g ≈ 2.9e-4 N, so ANY horizontal
      component of leg-ground force makes the feet SLIDE outward instead of lifting the
      body — and the current crouch pose (thigh 69° from vertical) is mostly-horizontal
      push. **The stroke must keep the feet directly UNDER the hips** (zigzag leg: calf
      angled back inward, foot at the hip's lateral radius r≈0.07 m) so the ground
      reaction stays ~vertical throughout.
      **Derived corrected targets (mapping: θ_thigh≈0.57+hip; θ_calf=θ_thigh+0.8+knee;
      negative θ_calf = inward):**
      • CROUCH: θ_t=0.9, θ_c=−0.9 → `hip 0.33, knee −2.60` (feet at r=0.07, 0.186 m
        below hip)
      • EXTEND: θ_t=0.15, θ_c=−0.15 → `hip −0.42, knee −1.10` (0.297 m below hip)
      • Vertical stroke 0.11 m with near-zero lateral foot travel.
      **FOURTH-SESSION UPDATE (2026-07-15, later): corrected targets swapped in and
      live-tested once (commit with this update). Geometry/grip SOLVED — new bottleneck
      is stroke RATE.** Test data (rest at z=4.8177 → `jump_target_distance 3.0`):
      body rose steadily through the crouch (no slide-stall — feet demonstrably
      gripping now) and the launch produced a real but microscopic ballistic hop
      (peak +10.5 mm at T+14s, decayed back over ~10s; consistent with separation
      vz ≈ 0.9 mm/s). But the body rise rate was ~0.7-1 mm/s throughout vs the
      ~100 mm/s stroke rate needed for 18.5 mm/s separation — the legs track their
      targets ~100x too slowly under load. Candidate causes for the next attempt,
      cheapest-first:
      (a) **DART re-sleep mid-stroke**: `_wake_model()` fires only at CROUCH start
          and IGNITION; the near-quiescent 10s crouch may cross the ~15s sleep
          window. Try a periodic wake every 3-5s through CROUCH+LAUNCH.
      (b) **Leg position controllers too soft to track at speed** (p=0.05, d=0.01,
          cmd_max=0.134): check actual joint angles vs commanded targets during the
          1s launch window via `dynamic_pose` — if they lag far behind, raise p/d
          for the leg joints (generator script, then regenerate model.sdf).
      (c) Joint `damping=5e-3` interacting with the soft PID (raised this session
          for landing dissipation — may need a compromise value or asymmetric
          handling).
      Delivered-v calibration (`V_FULL` in `jump_target_callback`, provisional
      0.08 m/s) must still be re-measured from the first real hop.
- [x] **2. Strong hops + settling landings — ✅ RESOLVED (2026-07-16, `a2ac862`,
      pushed): joint damping c=0.05 locked in.** First honest measurement: separation
      24.9 mm/s (apex +2.9 m; 35% margin over a 3 m hop's needs), landing settles and
      confirms in ~14 min. Full table in the commit message. **But the sweep's real
      story is the bug hunt it forced — read `9fdc3d4` before trusting ANY old
      leg-related conclusion:**
      ⛔ **The bridge NEVER delivered leg/drill commands in recent sessions**: gz-sim 8's
      JointPositionController subscribes ONLY to the joint-indexed topic
      (`.../joint/<j>/0/cmd_pos`, verbose-server-verified); the un-indexed variant the
      bridge published to has no subscriber. ROS remaps can't express "/0/" (numeric
      token), so the bridge now uses a YAML `config_file` per agent. Verify with
      `gz topic -i` (subscriber present?) whenever leg behavior looks wrong.
      Other landmines fixed en route, each live-verified: false mid-air LANDED
      (velocity-only rest path had no altitude guard; free-fall from rest stays under
      5 mm/s for ~44 s); exactly-in-place set_pose is a no-op that does NOT wake DART
      (wake now lifts +0.5 mm); sleep-defeat idle rotor (yaw wheel never commanded
      below 2 rad/s — a skeleton with a moving joint can never sleep); post-landing
      stand-fold REMOVED (with obedient legs it catapulted the robot off at
      0.128 m/s — after LANDED, legs hold their landing pose, no exceptions).
- [x] **3. Flight tumble — ✅ MEASURED AND PASSED (2026-07-16 mission watch).**
      In-flight rates 0.005–0.015 rad/s (essentially still), launch transients
      (0.24 rad/s) decay within seconds, no persistent yaw. Old failure signatures
      gone. Known minor: the sleep-defeat rotor's one-time spin-up leaves a
      ~0.03 rad/s free-flight yaw residual (below the 0.15 rad/s tilt gate);
      harmless — a future fix is spinning the rotor up before launch while
      grounded so friction absorbs the reaction.
- [x] **4. Post-landing cycle after a REAL hop — ✅ verified 2026-07-16** (impact-path
      contact → settle → LANDED → fold ramp → next jump accepted, zero watchdog trips;
      the impact soft-posture snap this item warned about was removed entirely, see 1b).
- [ ] **5. Full swarm mission cycle — auction/dispatch/re-hop ✅ VERIFIED live**
      (2026-07-16: competitive bids "scout_2=29.1, scout_3=40.8 → winner scout_2",
      dispatch, cooldown-paced corrective re-hops). **BUT the mission cannot complete
      yet: the symmetric vertical stroke has ZERO horizontal range** — re-hops
      repeated "23.2 m short" forever; the bots hop in place. A forward-lean
      directional stroke (LEAN=0.25 rad, `f5afaa2`) was implemented and
      **range-VERIFIED (`023d7de`): 9.1 m horizontal per full-stroke hop**
      (~15 mm/s horizontal / ~22 mm/s vertical, thrust tilt ~34°, clean arc,
      confirmed landing). With ~9 m/hop the 5-attempt re-hop budget converges on
      20–40 m targets — the autonomous loop is complete. REMAINING for this item:
      watch the drill-dwell → stow → carousel-chain tail live under swarm_manager
      (all machinery verified except this last leg), and optionally teach
      swarm_manager a range-per-hop model (currently it requests the full distance
      per jump and relies on re-hops; requesting min(distance, ~9 m) legs would be
      cleaner).
- [x] **6. Self-righting — ✅ REWRITTEN AND VERIFIED (2026-07-16, `09de868`, pushed).**
      The leg-sweep maneuver was live-observed failing all 5 attempts on every inverted
      bot (this item's fear confirmed). Replaced with a reaction-wheel roll (bang-bang
      spin/brake, ~500x torque margin, MINERVA-II's principle) + a
      `/scout_N/righting_active` flag that stands the attitude controller down during
      the roll. Verified via forced set_pose inversion: upright in ~9 s (2 attempts,
      axis/sign alternation self-corrected a wrong first guess), stable after.
      **Same commit, two more critical fixes:** (a) the joint_state publisher was
      flooding at 515 Hz and had pinned all landing controllers at 100% CPU (state
      machines frozen; landing took 7+ min or never) — removed entirely, landing now
      confirms in ~2 min; ⛔ never feed a per-physics-step topic to a Python
      subscriber. (b) at-rest gate on tilt correction — grounded tilt torque is a
      rover drive in µg and was rolling landed bots around and launching them to
      10 m; tilt now runs only when |v|>8mm/s or |ω|>0.15 rad/s. All sensor
      subscriptions moved to sensor-data QoS (stale RELIABLE backlogs pump).
- [ ] **7. Amplitude calibration refinement** once item 2 restores strong hops:
      re-measure V_FULL (currently 0.04 m/s, measured at damping 0.005 — it is LOWER
      at damping 0.15) and achieved distance for 2–3 fractions, then update the §3.1
      paper note (still flagged provisional).
- [ ] 8. (Carried over) LIDAR decision — no sensor exists in the model; mass table
      lists it as intended hardware. See Guidance §3.
- [x] 9. **Multi-agent scaling — ✅ DONE (2026-07-16, `ddab006`, pushed).** All three
      scouts spawn (8-10 m apart), each gets its own controller trio + bridge, and the
      swarm manager ran its first genuinely multi-agent allocation on first boot
      (scout_1 RELAY, scout_2/3 SAMPLER en route to anomalies; dashboard all-ONLINE).
      One model.sdf serves all three — plugin topics derive from the entity name;
      odometry frame labels were de-hardcoded. **Watch:** RTF drops to ~58% with 3x
      physics load; and the mm/s hop weakness (item 2) now applies to every en-route
      leg, so full mission cycles are slow until item 2 is solved.
- [ ] 10. Docs: `research_report.md` has a dated addendum covering the liftoff
      campaign and landing-damping saga (added 2026-07-16); once item 2 restores
      strong hops, update `Research_Paper.md` §3.1's "still open" caveat and the
      addendum's closing status line.

## Instructions for Receiving Agent

1. **Read this file first**, then `task.md` for the checkbox-level detail, then
   `walkthrough.md` for the narrative version if you want more context on *why*.
2. **After each significant step**: update `task.md`'s checkboxes, and this file's
   "Completed/Remaining Work" sections if the change is significant enough to matter to
   the next agent. Don't let uncommitted work pile up — commit locally after each
   coherent chunk (ask before pushing).
3. **To test changes**: kill any running gz/ros2 processes **by PID** (see Process
   hygiene above — this environment accumulates orphaned processes across sessions,
   check `ps aux | grep -E "gz sim|ros2 launch|parameter_bridge"` before assuming a
   clean slate), regenerate SDF via the Python script (never hand-edit `model.sdf`),
   `colcon build`, then launch.
4. **The user cares deeply about visual quality** — the model must look realistic, not
   like basic primitives. They will notice small things (proportions, whether an LED
   actually lights up vs. just looks colored, whether a mechanism looks mounted vs.
   free-floating) and will tell you directly if something looks wrong — take that
   feedback at face value and fix it, don't argue that "it's probably fine."
5. **If you're Antigravity (or any agent unfamiliar with this session) picking this up
   cold**, the single most important thing to internalize is the ros_gz_bridge gotcha
   near the top of this file. It cost significant debugging time to find because it
   produces *no errors* — topics look like they exist, nodes look connected, and the
   robot visibly moves (leg/drill commands still work) — so the natural assumption is
   "the bridge is fine, the bug must be in the control logic." It wasn't. **If you're
   about to spend time debugging why `attitude_controller.py`/`landing_controller.py`
   "isn't working," check the bridge first**, especially after any environment change
   (fresh machine, `apt upgrade`, container rebuild).

## Guidance for Next Agent — What's Open and How to Approach It

### 1. Watch a full landing cycle complete — ✅ DONE (2026-07-14)
**Confirmed.** A short controlled 3m hop (after killing `swarm_manager` with
`pkill -9 -f "lib/ryugu_sim/swarm_manager"` so it doesn't race a manual trigger) was
watched all the way through to a sustained `LANDED` state (background watcher polling
`/scout_1/landed` every 10s, 6 consecutive `true` readings required). Full story in
`walkthrough.md`'s "First-ever confirmed full landing cycle" section — the short
version: it landed **inverted**, self-righting engaged and eventually exhausted its
retry budget, and the designed "give up after 5 attempts, mark LANDED anyway" fallback
worked correctly. If re-running this for any reason, the recipe is:
```bash
pkill -9 -f "lib/ryugu_sim/swarm_manager"
bash src/ryugu_sim/scripts/trigger_jump.sh 3
# then poll /scout_1/landed every ~10s (via `ros2 topic echo --once`, backgrounded with
# an explicit kill after each sample -- `ros2 topic echo` without --once can hang past
# its wrapping `timeout` in this environment) until it reads true 6 times in a row.
```

### 2. Verify self-righting actually works — ✅ DONE (2026-07-14)
**What happened live:** self-righting genuinely engaged on a real inverted landing (not
a synthetic test). First test: succeeded once, immediately re-detected inverted, then a
second full 5-attempt retry cycle legitimately failed and the designed fallback
correctly prevented a hang. Root-caused to two compounding bugs, **both now fixed**:
1. `attitude_controller.py` never reset `in_flight` after landing (no `/landed`
   subscription existed at all) — fixed in `7ba3977` (subscribes to `/landed`, clears
   `in_flight` on touchdown).
2. The attitude PID itself was unstable at large tumble angles — `attitude_error`
   measured oscillating between 1.5–2.8 rad (85–160°) rather than converging, using
   gains that had literally never been dynamically tested before this session (the IMU
   was silently disconnected in every prior session). **Rewritten** from Euler-angle
   PID to quaternion cross-product tilt feedback (rotate local +Z into world frame,
   cross with world +Z — valid at any tilt magnitude, no small-angle assumption, no
   gimbal lock), with direct velocity commands instead of accumulated acceleration and
   integral terms removed entirely. Committed `b68ca4f`. Full derivation and
   small-angle-limit sign verification in the code comments and commit message.

**Live-verified after the rewrite**: a fresh hop landed with self-righting succeeding
on the *first* attempt (vs. all 5 retries before), a follow-on idle-recovery hop landed
just as cleanly with zero righting needed, and post-landing IMU `angular_velocity` read
~1e-15 rad/s (float noise) vs. `wz=4.12 rad/s` before — genuinely motionless. This was
the user's own live observation ("it's spinning around in the air") that triggered this
whole investigation. As of this writing, `task.md`'s "Verify Safe Landing" section is
fully checked off — every item that was open is now done and live-verified.

### 3. LIDAR — decide whether to re-add it, and if so, do it carefully
No LIDAR sensor exists in `model.sdf` right now; it was removed in an earlier session
"to prevent performance stuttering" per a generator-script comment, and never
re-added. The mass table in both research docs still lists it as intended hardware —
that's fine, it's describing the intended design. If asked to implement LIDAR-based
hazard/terrain awareness (task.md's dropped C1 item), **first re-add a minimal-cost
LIDAR sensor** (low ray count, low update rate) and confirm frame rate is still
acceptable before wiring any swarm logic to it — don't build hazard-avoidance logic
against a sensor that isn't there, and don't reintroduce the performance problem it was
removed to avoid.

### 4. Multi-agent scaling (scout_2, scout_3, ...)
`swarm_manager.py`'s logic was reviewed by inspection this session and looks
structurally ready to generalize (everything is already keyed per-agent), but this
was **not verified by actually spawning more agents** — that would also require
changes to `spawner.py` and `ryugu_swarm.launch.py`'s agent loop (currently hardcoded
to `["scout_1"]` in both places). If you do this, watch for: (a) the RELAY-assignment
guard (`len(self.agents) > 1`) — should naturally start working once there's a second
agent, (b) whether the shared single `anomaly_queue` and `MetricsLogger` need any
per-agent partitioning, (c) whether the IMU/odometry topic-scoping fix (per-entity
auto-generated topics) continues to work correctly with multiple spawned entities of
the same model — it should, since it's keyed by spawned entity name, but hasn't been
tested with more than one.

### 5. General visual polish
The user gives fast, specific feedback on visual issues (see the drill housing and LED
light history above) — if you make a model change, expect follow-up correction and
treat it as normal, not a sign you did something wrong. If revisiting the LED-lighting
idea (real `<light>` sources vs. the current emissive-only spheres), check whether
gz-sim Harmonic's GUI has a way to hide light-source gizmos before trying it again;
that's what forced the revert last time, not anything wrong with the lights themselves.

---

*Last updated: 2026-07-15 (Fable — deep actuation tuning, swarm auction, docs accuracy pass; see the ✅ CHECKLIST section)*
