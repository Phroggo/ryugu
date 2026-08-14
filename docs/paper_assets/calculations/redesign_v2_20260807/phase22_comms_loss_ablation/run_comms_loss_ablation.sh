#!/bin/bash
# Phase 22: comms-loss ablation. Runs the full swarm mission stack at each
# packet-loss level (0/5/10/20/30%, see swarm_manager.py's Phase 22
# COMMS_LOSS_PCT / _comms_drop() additions), dispatch policy left at the
# default "auction" (current shipped behavior) throughout -- this ablation
# isolates comms loss specifically, not dispatch policy (that's Phase 21).
#
# SCOPE NOTE (stated up front, not just in the report): n=2 repetitions
# per level, 25-minute windows -- 5 levels x 2 reps x 25 min = 250 min
# (~4.2h) total. This is fewer repetitions than would be ideal for tight
# statistical power (Phase 21's mission comparison used n=1 x 45 min per
# policy for the same practical-time reason), chosen to keep total
# wall-clock tractable given this ablation runs after Phase 21's own ~3h
# comparison in the same session. Enough to see whether there's a real,
# consistent trend across loss levels and to sanity-check repeatability
# via the n=2 spread -- not enough for a tight confidence interval on any
# single level. Stated as a real limitation, not hidden.
#
# Same process-management approach as Phase 21's run_dispatch_comparison.sh:
# this environment's pkill/pgrep -f reliably abort the whole script under
# the harness's background-task execution, so no pattern-based process
# killing is used anywhere here -- only directly-captured child PIDs via
# $!, plus a ps+awk+kill sweep (read-only discovery, then kill by exact
# PID) for ros2 launch's fanned-out children (gz sim, spawner, per-agent
# nodes) that don't die with their parent.
set -x
source /opt/ros/humble/setup.bash
source /home/melvin/ryugu_v2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/melvin/ryugu_v2_ws/src/ryugu_sim/models

D="/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase22_comms_loss_ablation"
RUN_DURATION_S=1500   # 25 min

for PCT in 0 5 10 20 30; do
  for REP in 1 2; do
    echo "=== Starting loss=${PCT}% rep=${REP} ==="
    PD="$D/loss_${PCT}pct_rep${REP}"
    mkdir -p "$PD"
    cd "$PD"

    export SWARM_COMMS_LOSS_PCT="$PCT"
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

    unset SWARM_COMMS_LOSS_PCT
    sleep 5

    echo "=== Finished loss=${PCT}% rep=${REP} ==="
  done
done

echo "COMMS_LOSS_ABLATION_ALL_DONE"
