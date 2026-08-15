#!/bin/bash
# Phase 27: external review round item 4 -- rerun all four dispatch
# policies (current auction, distance_only, nearest, fifo) at n=30 each
# instead of Phase 21's n=1, reporting the same columns as Table XI
# (samples extracted, mean target latency, total distance) plus
# confidence intervals this time. Independent of items 1-3 (no shared
# code path -- swarm_manager.py's dispatch logic doesn't touch
# attitude_controller.py or the self-righting/directional-hop stack).
#
# SCALE, stated up front: 4 policies x 30 reps x 45min = 5400 min = 90h
# (~3.75 days) of continuous wall-clock sim time. 45-minute window kept
# identical to Phase 21 for direct comparability and because Phase 21/22
# both found shorter windows underpowered for the completion-rate metrics
# this ablation reports -- shortening the window would compromise the
# exact deliverable requested, not just take less time.
#
# ORDER: rep-major (not policy-major), same order-confound discipline
# applied in Phase 23 -- all 4 policies interleaved within each rep
# rather than run as 4 back-to-back blocks, so any time-of-run drift
# over ~90h doesn't confound with policy identity.
#
# Reuses Phase 21's existing rep1 per policy (already clean, no
# contamination) -- this script runs reps 2-30 (29 new reps x 4
# policies = 116 runs), giving n=30 total per policy once combined.
#
# Process management: same pkill/pgrep-free approach as Phase 21/22/23
# (this environment's pkill/pgrep -f reliably abort scripts under the
# background-task harness) -- direct $! PID capture + ps/awk/kill by
# exact PID. 20s inter-run cooldown (Phase 22's 5s was found insufficient,
# causing 2/10 contaminated runs; Phase 23 used 20s with no issues).
set -x
source /opt/ros/humble/setup.bash
source /home/melvin/ryugu_v2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/melvin/ryugu_v2_ws/src/ryugu_sim/models

D="/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase27_auction_baseline_n30"
RUN_DURATION_S=2700   # 45 min, matches Phase 21 exactly

for REP in $(seq 2 30); do
  for POLICY in auction distance_only nearest fifo; do
    echo "=== Starting policy=$POLICY rep=$REP ==="
    PD="$D/run_${POLICY}_rep${REP}"
    mkdir -p "$PD"
    cd "$PD"

    export SWARM_DISPATCH_POLICY="$POLICY"
    ros2 launch ryugu_sim ryugu_swarm.launch.py > launch_stdout.log 2>&1 &
    LAUNCH_PID=$!

    sleep 15

    ros2 topic echo /swarm_manager/metrics_json --full-length > metrics_json.log 2>&1 &
    METRICS_PID=$!

    sleep $RUN_DURATION_S

    kill -9 $METRICS_PID 2>/dev/null || true
    kill -9 $LAUNCH_PID 2>/dev/null || true
    sleep 5
    SURVIVORS=$(ps -eo pid,cmd 2>/dev/null | grep -E "gz sim|swarm_manager|swarm_gui|spawner|scout_1|scout_2|scout_3" | grep -v grep | awk '{print $1}')
    for p in $SURVIVORS; do kill -9 $p 2>/dev/null || true; done
    JOBPIDS=$(jobs -p 2>/dev/null || true)
    for p in $JOBPIDS; do kill -9 $p 2>/dev/null || true; done

    unset SWARM_DISPATCH_POLICY
    sleep 20

    echo "=== Finished policy=$POLICY rep=$REP ==="
  done
done

echo "AUCTION_BASELINE_N30_ALL_DONE"
