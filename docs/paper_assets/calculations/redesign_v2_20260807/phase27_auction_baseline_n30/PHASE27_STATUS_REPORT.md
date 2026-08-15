# Phase 27 — Auction Baseline Rerun at n=30/policy — PAUSED, NOT COMPLETE

Date started: 2026-08-15
Status: **paused mid-run at user request (higher-priority work), 2026-08-15. Not committed — this is a status report for resumption, not a completion report.**

Scope: item 4 of the external review round's four sim-side items — rerun all four dispatch policies (current auction, distance_only, nearest, fifo) at n=30 each, reporting the same columns as the existing Table XI (samples extracted, mean target latency, total distance) plus confidence intervals this time.

## 1. Design (unchanged from Phase 21, extended)

- `run_auction_baseline_n30.sh`, at `docs/paper_assets/calculations/redesign_v2_20260807/phase27_auction_baseline_n30/`.
- Reuses Phase 21's existing rep1/policy (already clean, no contamination) — this script runs reps 2-30 (29 new reps × 4 policies = 116 runs), giving n=30 total per policy once combined and complete.
- Rep-major run order (not policy-major): all 4 policies interleaved within each rep, so time-of-run drift over the ~90h total doesn't confound with policy identity (same discipline as Phase 23).
- 45-minute window per run, matching Phase 21 exactly, for direct comparability.
- 20s inter-run cooldown (up from Phase 22's 5s, which was found insufficient there).
- Same pkill/pgrep-free process management as Phases 21-23/26 (this environment's `pkill`/`pgrep -f` reliably abort scripts under the background-task harness) — direct `$!` PID capture + `ps`/`awk`/`kill` by exact PID.

## 2. State at pause (2026-08-15)

Launched, confirmed booted cleanly, ran for **~12.1 hours** before the user requested a pause for higher-priority work. Stopped cleanly via the harness's task-stop mechanism, not a raw kill; all simulation processes (gz sim, swarm_manager, swarm_gui, per-agent nodes) confirmed fully terminated afterward via a `ps` sweep — **no leftover processes**.

**15 fully-complete runs** (full 2700s+ window each):

| run | elapsed_s | samples_extracted | total distance (m) |
|---|---|---|---|
| run_auction_rep2 | 2712.0 | 2 | 119.07 |
| run_auction_rep3 | 2714.0 | 4 | 119.34 |
| run_auction_rep4 | 2714.0 | 6 | 95.91 |
| run_auction_rep5 | 2714.0 | 5 | 103.23 |
| run_distance_only_rep2 | 2714.0 | 2 | 134.45 |
| run_distance_only_rep3 | 2714.0 | 4 | 46.38 |
| run_distance_only_rep4 | 2714.0 | 5 | 105.28 |
| run_distance_only_rep5 | 2714.0 | 4 | 130.95 |
| run_fifo_rep2 | 2714.0 | 0 | 113.80 |
| run_fifo_rep3 | 2714.0 | 0 | **0.00 — SUSPECT CONTAMINATED, see §3** |
| run_fifo_rep4 | 2714.0 | 0 | 58.50 |
| run_nearest_rep2 | 2714.0 | 0 | **0.00 — SUSPECT CONTAMINATED, see §3** |
| run_nearest_rep3 | 2714.0 | 0 | 182.42 |
| run_nearest_rep4 | 2714.0 | 0 | 120.05 |
| run_nearest_rep5 | 2714.0 | 0 | 202.60 |

**1 incomplete run, not usable as-is**:

| run | elapsed_s | note |
|---|---|---|
| run_fifo_rep5 | 2480.0 (of 2700 target) | cut short mid-window by the pause; partial data only, needs a full rerun |

Combined with Phase 21's existing rep1 (already committed), this gives, as of the pause: **auction n=5, distance_only n=5, nearest n=4 (rep5 complete but see table), fifo n=4 usable (rep2/4 complete, rep3 contaminated, rep5 incomplete)** — well short of the n=30 target, 15/116 of this phase's own planned runs (12.9%) plus the pre-existing rep1.

None of this phase's data has been committed — per standing practice, only complete, verified work gets committed, and this batch is neither complete nor has its contamination been resolved yet.

## 3. Contamination found mid-run, same signature as Phase 22 — not yet resolved

`run_fifo_rep3` and `run_nearest_rep2` both show all three agents' `distance_by_agent` frozen at exactly 0.0 for the full window, despite `hop_count_by_agent` showing 1 for two of the three agents — the same "hop command published but never executed downstream, agent stuck in IDLE" signature Phase 22 first identified and attributed to insufficient inter-run cooldown.

**This phase already used a 20s cooldown** (quadrupled from Phase 22's 5s, specifically to address that finding) — and still shows a similar contamination rate (2/15 complete runs ≈ 13%, versus Phase 22's 2/10 ≈ 20%). This suggests cooldown duration alone may not be the full explanation, or a longer cooldown still than 20s is needed, or there's a different/additional contributing factor not yet identified. **Not investigated further before pausing** — flagged here as the first thing worth a closer look at when this work resumes, before just re-running the affected slots with the same 20s cooldown and hoping for better luck.

## 4. To resume

1. Decide on the contamination question in §3 (investigate root cause further, or accept the ~13-20% rate and plan to exclude+backfill contaminated runs as they occur, matching Phase 22's precedent).
2. Rerun `run_fifo_rep5` (incomplete, not just contaminated — genuinely cut short).
3. Continue the rep-major loop. The script (`run_auction_baseline_n30.sh`) currently starts fresh at `REP=2`; resuming exactly where this run left off (rather than restarting from rep2, which would discard the 15 good runs already collected) requires either (a) editing the script's `for REP in $(seq 2 30)` to skip combinations already present as complete run directories, or (b) simply re-invoking the script as-is, which will overwrite `run_*_rep2` through `run_*_rep5` and repeat that already-done work (simpler, no bookkeeping risk, costs ~12h of redundant runtime). **Recommend asking the user which they prefer** before resuming, given the meaningful time cost of option (b) and the small bookkeeping risk of option (a).
4. Do not report item 4 as complete, and do not write a "PHASE27_CHANGE_REPORT.md" (the completion-style report, distinct from this status report), until a resumed run actually reaches n=30/policy with contamination resolved or accounted for.

## 5. Checkpoint verdict

**Paused, not complete, by explicit user request — not a phase failure or an anomaly in the work itself.** All processes cleanly terminated, no data lost, partial results preserved on disk for resumption. One real open question (§3, contamination rate not fully explained despite the cooldown increase) is worth resolving before or during resumption rather than repeating the same partially-understood mitigation and hoping.
