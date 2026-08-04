# Sampling-cycle rerun (C27) + routing analysis, 2026-08-04

Live rerun of the full 3-agent swarm (bridge + hopper_locomotion +
attitude_controller + landing_controller per agent, plus spawner and
swarm_manager -- the actual production stack, no code changes) to
independently capture the paper's "arrive -> drill -> cache -> chain"
sampling-cycle claim: *"on a SAMPLER reaching its anomaly (arrival check
against live odometry, 0.9 m from target on the verified run), the swarm
layer autonomously deployed the core drill, completed the extraction
dwell, stowed the core in carousel tube 1/3, and immediately re-tasked the
agent to its next queued anomaly."*

No test hooks, no seeded anomalies -- scouts detected anomalies
organically (15%/tick per active SCOUT) and won them via the real auction
logic the whole time.

## Result: confirmed, with real telemetry

The run went for **3.75 hours** (12:31 min to first anomaly detection,
which itself happened within 4 seconds of boot) before it was
inadvertently killed by an unrelated diagnostic script's overly broad
`pkill -f "gz sim"` pattern (see note at the end) -- by which point it had
already logged multiple complete, real cycles:

```
🚀 scout_3 accepting bid for SAMPLER. Navigating to [45.0, -12.3] via 0.6m jump.
⛏️ scout_3 arrived (within 0.6m) — deploying Core Sampler Drill...
🧪 scout_3 stowing core in carousel tube (1/3)...
🚀 scout_3 accepting bid for SAMPLER. Navigating to [45.0, -12.5] via 0.4m jump.
```

Full console log: `swarm_manager_console_full_run.log`.

- **8 real arrive -> drill -> stow events** across scout_2 and scout_3,
  at real odometry-measured arrival distances of 0.6, 1.5, 3.7, 2.7, 2.5,
  0.8, 2.0, and 2.8 m -- all inside the 4.0 m `ARRIVAL_RADIUS`, and a much
  richer spread than the paper's single cited "0.9 m" instance.
- **Multiple real chain events**: each stow was immediately followed by a
  fresh "accepting bid for SAMPLER" dispatch to the next queued anomaly,
  with no return to SCOUT in between, when the carousel had room.
- **Carousel behavior confirmed**: scout_3 filled its 3-tube carousel
  (1/3 -> 2/3 -> 3/3) more than once in the run, each time correctly
  gating further chaining until it was emptied (handled by standing down,
  matching the code's documented carousel-full behavior).
- **An incidental, related finding**: battery (BMS) constraints repeatedly
  interrupted missions -- multiple "Ni-MH cells critical (~15%) -- Fleeing
  to sunlight" events, including at least one SAMPLER task abandoned
  mid-flight with its anomaly requeued. Real, and worth knowing alongside
  the sampling-cycle claim itself: the swarm's autonomy loop works, but
  power constraints visibly throttle throughput over a long run.

**Status: C27 confirmed.** The complete arrive -> drill -> cache -> chain
sequence executes for real, matching the paper's description, with a
richer and more varied sample than the paper's own single cited instance.

## Routing analysis (real greedy vs. optimal, same data)

`swarm_manager.py`'s own comments describe its anomaly-chaining choice as
a deliberate nearest-neighbor greedy simplification of the
Orienteering-Problem framing, not a claim of optimality. Using scout_3's
actual real visited-anomaly sequence from this run (6 real stops, in the
order the swarm actually dispatched them):

```
(45.0, -12.3) -> (-17.3, -15.2) -> (-11.5, -30.3) -> (-45.0, -37.2) -> (-45.0, -37.0) -> (21.0, 45.0)
```

- **Real (as-executed) path length: 218.21 m**
- **Optimal ordering of the same 6 stops (exact, brute-force): 184.01 m**
- **The real greedy routing was 18.6% longer than optimal** for this
  actual sequence.

This is a genuine, real-data answer, not a synthetic benchmark: same
targets, same starting point, same launch-physics-limited platform --
just comparing the order the code actually chose against the
provably-best order for that exact same set of stops. Full data and the
optimal ordering: `routing_analysis.json`.

## Note on the interrupted run

The run was not stopped deliberately -- an unrelated diagnostic script
(investigating a separate IMU-orientation bug for the self-righting
severe-tilt rerun) used `pkill -f "gz sim"` to clean up its own throwaway
Gazebo instance between test iterations. That pattern matches on command-
line text only, not process ownership or environment isolation, so it
also killed this run's Gazebo instance as a side effect. The console log
was unaffected (already on disk) and the result above is unaffected by
the interruption -- the claim was already fully demonstrated by the time
this happened. Restarting was not necessary given the data already
captured.
