# Phase 21 — Auction Baseline Comparison

Date: 2026-08-14
Scope: item 2 of the four held items authorized in "go ahead on all four held items" — implement FIFO, nearest-agent-only, and distance-only dispatch as baselines against the current distance+battery+carousel auction, and compare mission completion time, total distance, energy, hop count, and target latency across all four.

**New code, not a parameter sweep**, per the user's own framing: three simplified dispatch policies added to `swarm_manager.py`, plus new metrics instrumentation (none of distance/energy/hop-count/target-latency were tracked anywhere before this phase — confirmed by research before starting: `MetricsLogger` only ever tracked `anomalies_found`/`samples_extracted`/`data_transmitted`/`role_switches`, in memory, never persisted).

## 1. Files touched

### Source (swarm_manager.py — dispatch-policy switch + metrics instrumentation)

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/ryugu_sim/swarm_manager.py`

### Harness and results

- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase21_auction_baseline_comparison/run_dispatch_comparison.sh`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase21_auction_baseline_comparison/results_summary.json`
- `/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase21_auction_baseline_comparison/PHASE21_CHANGE_REPORT.md` (this file)
- Four per-policy run directories (`run_auction/`, `run_distance_only/`, `run_nearest/`, `run_fifo/`), 14 files each (56 total): `launch_stdout.log`, `metrics_json.log`, and per-agent (`scout_1/2/3`) `_role.log`, `_activity.log`, `_battery.log`, `_landed.log` — full `find`-verified file count: 58 files in this phase directory including the two top-level files above.

## 2. Correcting a prior assumption: "carousel" is not round-robin scheduling

Before implementing, research (an Explore-agent pass plus my own direct read of `swarm_manager.py`) corrected an assumption carried in earlier project memory that the current auction was "distance+SoC+carousel" with carousel meaning a round-robin fairness mechanism. It does not. `_bid()` (originally lines 258-269, now shifted slightly by this phase's edits) computes `cost = distance + (100 - battery%) * 0.5 + sample_count * 5.0` — "carousel" here is a penalty for how many samples the agent's physical 3-tube carousel already holds (biasing toward emptier agents), not a turn-based scheduler. No separate round-robin mechanism exists anywhere in the file. Corrected in project memory.

## 3. Dispatch-policy switch (`swarm_manager.py`)

Added `DISPATCH_POLICY = os.environ.get("SWARM_DISPATCH_POLICY", "auction")` at module scope — unset/default behavior is **byte-for-byte identical** to before this phase (confirmed: the "auction" branch is the exact same full-queue combinatorial search against `_bid()` that existed previously, now factored into a new `_select_winner()` method rather than inlined in `swarm_tick()`, with no logic change).

Four policies, implemented in `_select_winner()`:
- **`auction`** (default): current shipped behavior, full-queue search, cost = `_bid` (distance + battery + carousel).
- **`distance_only`**: same full-queue combinatorial search as auction, cost = pure euclidean distance. Isolates how much the battery/carousel penalty terms matter on their own, holding the whole-queue route-planning mechanism fixed.
- **`nearest`**: only the oldest queued target is considered (no whole-queue search), assigned to whichever eligible bidder is physically nearest to it.
- **`fifo`**: only the oldest queued target is considered, assigned to the first eligible bidder in agent list order — no distance comparison at all, the floor baseline.

**Interpretation flagged as chosen, not given**: the original ask named these three baselines without fully specifying how "nearest-agent-only" and "distance-only" should differ, given the current auction already does a full-queue combinatorial distance search. Resolved by treating them as an ordered ablation ladder — fifo (no spatial/state awareness) → nearest (adds distance-based agent choice for a fixed, oldest-first target) → distance_only (adds whole-queue search on top) → auction (adds battery/carousel awareness on top) — each step adding exactly one mechanism. Documented in the source comment above `_select_winner()` as well, so the choice is visible to a future reader, not just this report.

Verified via a standalone unit test (no ROS2 required, `SwarmManager.__new__` bypassing `Node.__init__`, synthetic 3-agent/3-target state) that all four policies compute the mathematically-expected winner before any live run — auction correctly favored a farther-but-fresher agent over a closer-but-depleted one; distance_only picked the objectively nearest pair; nearest and fifo both correctly restricted to the oldest queued target with the expected agent-selection rule.

## 4. Metrics instrumentation (`swarm_manager.py`)

`MetricsLogger` extended with `distance_by_agent` (real odometry-integrated path length, accumulated in `odom_callback` — not commanded leg length, so it captures actual in-flight/bounce/corrective-rehop path), `energy_by_agent` (cumulative %-drain, this sim's battery model is chemistry-agnostic 0-100% with no Wh/J curve, so this is a relative comparison metric across policies, not an absolute energy figure), `hop_count_by_agent` (every jump command actually published, incremented in `_dispatch_sampler`/`_dispatch_scout_search`), and `target_latencies_s` (wall-clock seconds from an anomaly first entering the queue to its core sample being successfully extracted, tracked via a `note_queued`/`note_extracted` pair keyed by rounded target coordinates).

Since missions here are torn down with a hard kill (no clean shutdown hook to rely on for a one-shot final dump), metrics are published every tick as a JSON blob on a new `/swarm_manager/metrics_json` topic — the same scrape-a-published-topic pattern already proven for `status_role`/`status_activity`/`status_battery` — so the harness reads the last line of an `ros2 topic echo` capture for each run's final snapshot.

## 5. Mission harness and a real environment bug found before trusting any run

`run_dispatch_comparison.sh` launches the full `ryugu_swarm.launch.py` stack once per policy via `SWARM_DISPATCH_POLICY`, fixed 45-minute window per run (matching Phase 7's established full-mission precedent — long enough for multiple hop-sample-return cycles given a single 9m hop's ~13-minute flight time at Ryugu gravity), capturing `/swarm_manager/metrics_json` plus per-agent status/landed topics.

**Before trusting a 3-hour run to this harness, isolated testing found that `pkill`/`pgrep -f` reliably abort the entire script when run under this environment's background-task execution** — confirmed via a sequence of minimal isolated tests (a bare `pkill -f "nonexistent_pattern" || true` alone aborted a background task; a bare `pgrep -f` alone did too; `kill -9 <literal_pid>` did not). Root cause not fully diagnosed (plausibly a sandbox/namespace restriction specific to pattern-based process scanning under the background-task supervisor, not a bash `set -e` issue in the ordinary sense), but the practical fix was verified directly: capture every backgrounded process's PID explicitly via `$!`, and for `ros2 launch`'s fanned-out children (`gz sim`, spawner, per-agent nodes, which don't die with their parent on a plain `kill`) use read-only `ps -eo pid,cmd | grep | awk` to find PIDs, then `kill -9` each by exact PID — this combination was verified clean (a 2-policy, 30-second-window dry run completed with zero leftover processes afterward, checked via `ps`) before the real run was launched. Documented here since a future harness in this environment should avoid `pkill`/`pgrep -f` from the outset rather than rediscovering this.

## 6. Results

Fixed 45-minute window per policy, n=1 run each (see §8 for why n=1, not repeated, and what that limits).

| policy | anomalies found | samples extracted | total distance (m) | total energy (%-drain) | total hops | mean target latency |
|---|---|---|---|---|---|---|
| auction (current) | 141 | 2 | 87.04 | 372.78 | 10 | 2068.0s (34.5 min), n=2 |
| distance_only | 184 | 3 | 140.30 | 334.10 | 12 | 916.7s (15.3 min), n=3 |
| nearest | 38 | 0 | 332.06 | 219.41 | 8 | n/a — none completed |
| fifo | 136 | 0 | 157.46 | 360.48 | 7 | n/a — none completed |

Per-agent distance and hop-count breakdowns, and the full raw metrics snapshots, are in `results_summary.json`.

### 6.1 The clearest, most surprising finding: distance-aware policies (auction, distance_only) completed real science within the window; fifo and nearest completed zero

This is the headline result, and it is not subtle — 2-3 completed samples vs. 0. Investigated against raw logs (`launch_stdout.log`, not just the metrics summary) before writing this up, per standing project discipline.

**Root cause, evidenced, not inferred**: under `nearest`, scout_2 was dispatched to a target only 7.5m away — a perfectly reasonable single-hop distance — but needed at least 2 corrective re-hops (7.5m short, then 7.2m short of target after each attempt) and triggered at least one RW self-righting recovery event (`"RW righting in progress"` in the attitude controller log) before a completely new dispatch superseded it. Under `fifo`, which ignores distance entirely for target selection, dispatched targets were much farther (one corrective re-hop was 18.2m short, another 17.5m short — implying an original target roughly 2-4x farther than `nearest`'s worst case), with RW self-righting events again appearing in the trace for multiple agents. **This connects directly to an already-documented, independently-discovered, still-unfixed platform issue** (Phase 10, this project's own memory: directional-hop testing "delivers essentially zero horizontal velocity... near-random azimuth" after the post-Phase-9 timing fix) — hops frequently under/overshoot their commanded target by a wide margin, and each corrective re-hop consumes real flight time (minutes, not seconds, at Ryugu gravity) plus occasional tip-over recovery time. `fifo` and `nearest` both, by construction, are more likely to draw long-distance target/agent pairings than `auction`/`distance_only`'s distance-weighted search, and a longer commanded distance means more hop legs, each independently exposed to the existing accuracy problem — compounding into a real chance of consuming the entire 45-minute window without ever reaching arrival radius.

**This is a genuinely informative result for the paper's dispatch-policy claim**: it is not simply "the smarter auction finds shorter paths" in the abstract — the mechanism is concrete and measurable (fewer/shorter hop legs → fewer exposures to the platform's known hop-accuracy and tip-over failure modes → higher probability of completing within a bounded mission window). This is a stronger, more specific justification for the current auction's distance-weighting than "it's more efficient" alone.

### 6.2 A second, separate surprising finding: `distance_only` beat `auction` on time-to-completion by ~2.3x

Both auction and distance_only completed real samples, so this comparison is on firmer ground than §6.1's zero-vs-nonzero gap. Mean target latency: auction 34.5 min (n=2), distance_only 15.3 min (n=3, tightly clustered: 926.0s/912.0s/912.0s). Distance_only also extracted one more sample in the same window (3 vs. 2) and used less total energy (334.10 vs. 372.78) despite covering more total distance (140.30m vs. 87.04m) — consistent with distance_only more often just picking the single truly-closest pairing, while auction's battery/carousel penalty terms sometimes steer a task toward a farther-but-fresher agent instead, at a real, measured time cost.

**Not a criticism of the current auction's design** — the battery/carousel awareness exists specifically to avoid stranding a task on an agent that can't finish it, a failure mode this particular short comparison window wouldn't necessarily surface. But it is a real, measured tradeoff that should be stated plainly rather than assumed away: in this comparison, the current auction's extra state-awareness cost roughly 2.3x the time-to-first-completion relative to pure distance, in exchange for robustness properties this experiment didn't specifically stress-test.

## 7. Anomalies flagged

1. §6.1: `nearest`/`fifo` completing zero samples, traced to specific corrective-re-hop and RW-self-righting events in the raw log, connected to the pre-existing Phase 10 hop-accuracy finding rather than treated as an unexplained result.
2. §6.2: `distance_only` outperforming the current `auction` on completion count and time within this window — reported plainly, not smoothed into "the current auction is still best."
3. §5: the `pkill`/`pgrep -f` background-task-abort behavior — an environment quirk unrelated to the simulation itself, but real, reproducible, and worth a future harness knowing about.
4. §2: the "carousel = round-robin" assumption in prior project memory was wrong; corrected here and in memory.

## 8. Limitation, stated plainly: n=1 run per policy

Each policy was run once, not repeated, given the ~3-hour total wall-clock already required for 4×45-minute runs in this session (alongside Phase 20's work earlier the same day and Phase 22's ablation queued immediately after). This means §6.1's zero-vs-nonzero gap and §6.2's 2.3x latency gap are each a **single sample per policy** — real, evidenced by raw logs, and mechanistically well-explained, but **not statistically established as the policies' typical behavior** the way Phase 18/19's n=3-5-per-config sweeps were. A single unlucky hop-accuracy/tip-over sequence could plausibly happen under any policy given enough time; what this run shows is that it happened for both zero-completion policies and neither completion-positive one, which is suggestive and mechanistically grounded, not proof of a deterministic policy ranking. Flagged explicitly rather than presented with more confidence than n=1 supports — if this comparison needs to go in the paper with a stronger statistical claim, it should be rerun with multiple repetitions per policy.

## 9. Checkpoint verdict

**Complete.** Three new baseline dispatch policies implemented and unit-verified before any live run; new metrics infrastructure added where none existed; a real environment-level process-management bug found and fixed before trusting a multi-hour run to it; results gathered and cross-checked against raw logs, not just the metrics summary. Two genuinely surprising findings (zero-completion baselines, distance_only's latency advantage over the current auction) reported plainly with their mechanistic explanation, alongside an explicit statement of the n=1-per-policy limitation on how far these specific numbers should be trusted.
