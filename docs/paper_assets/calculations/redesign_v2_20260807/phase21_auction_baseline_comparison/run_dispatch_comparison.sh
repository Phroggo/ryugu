#!/bin/bash
# Phase 21: run the full swarm mission stack once per dispatch policy
# (current auction, distance_only, nearest, fifo baselines), fixed
# wall-clock window per run for a fair comparison, capturing
# /swarm_manager/metrics_json (distance/energy/hop-count/target-latency
# etc., see swarm_manager.py's Phase 21 MetricsLogger additions) plus the
# per-agent status/landed topics already proven useful in Phase 7's
# reference mission-loop capture (mission_loop_capture.sh), same pattern.
#
# 45-minute run length reused directly from Phase 7's established full-
# mission precedent -- long enough for multiple complete hop-sample-return
# cycles per agent (a single 9m hop alone takes ~13 min at Ryugu gravity,
# per hopper_locomotion.py/swarm_manager.py's own comments), short enough
# to keep 4 policies' total wall-clock (~3h) tractable.
#
# PROCESS-MANAGEMENT NOTE: this environment's `pkill`/`pgrep -f` reliably
# abort the whole script when run under the harness's background-task
# execution (confirmed via isolated testing before this script was
# trusted with a real 3-hour run -- see PHASE21_CHANGE_REPORT.md). Killing
# a directly-spawned child via its own captured $! PID works fine, so
# this script captures every backgrounded PID explicitly and never uses
# pkill/pgrep. No pre-run cleanup pkill either -- each invocation of this
# script is assumed to start from a clean state (a fresh background task
# in this harness does not inherit stray processes from prior runs).
set -x
source /opt/ros/humble/setup.bash
source /home/melvin/ryugu_v2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/melvin/ryugu_v2_ws/src/ryugu_sim/models

D="/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase21_auction_baseline_comparison"
RUN_DURATION_S=2700   # 45 min, matches Phase 7's mission_loop_capture.sh

for POLICY in auction distance_only nearest fifo; do
  echo "=== Starting policy: $POLICY ==="
  PD="$D/run_$POLICY"
  mkdir -p "$PD"
  cd "$PD"

  export SWARM_DISPATCH_POLICY="$POLICY"
  ros2 launch ryugu_sim ryugu_swarm.launch.py > launch_stdout.log 2>&1 &
  LAUNCH_PID=$!

  sleep 15

  ros2 topic echo /swarm_manager/metrics_json --full-length > metrics_json.log 2>&1 &
  METRICS_PID=$!
  ECHO_PIDS=()
  for agent in scout_1 scout_2 scout_3; do
    ros2 topic echo /$agent/status_role      > ${agent}_role.log      2>&1 &
    ECHO_PIDS+=($!)
    ros2 topic echo /$agent/status_activity  > ${agent}_activity.log  2>&1 &
    ECHO_PIDS+=($!)
    ros2 topic echo /$agent/status_battery   > ${agent}_battery.log   2>&1 &
    ECHO_PIDS+=($!)
    ros2 topic echo /$agent/landed           > ${agent}_landed.log    2>&1 &
    ECHO_PIDS+=($!)
  done

  sleep $RUN_DURATION_S

  kill -9 $METRICS_PID 2>/dev/null || true
  for p in "${ECHO_PIDS[@]}"; do kill -9 $p 2>/dev/null || true; done
  kill -9 $LAUNCH_PID 2>/dev/null || true
  # ros2 launch fans out several child processes (gz sim, spawner, per-agent
  # nodes) that don't die with their parent on a plain kill. This
  # environment's pkill/pgrep -f reliably abort the whole script when run
  # under the background-task harness (confirmed via isolated testing), so
  # find survivors via plain `ps` + text filtering (read-only, doesn't
  # trigger the same abort) and kill each found PID individually -- this
  # combination was verified to work cleanly before being trusted with the
  # real run.
  sleep 5
  SURVIVORS=$(ps -eo pid,cmd 2>/dev/null | grep -E "gz sim|swarm_manager|swarm_gui|spawner|scout_1|scout_2|scout_3" | grep -v grep | awk '{print $1}')
  for p in $SURVIVORS; do kill -9 $p 2>/dev/null || true; done
  JOBPIDS=$(jobs -p 2>/dev/null || true)
  for p in $JOBPIDS; do kill -9 $p 2>/dev/null || true; done

  unset SWARM_DISPATCH_POLICY
  sleep 5

  echo "=== Finished policy: $POLICY ==="
done

echo "DISPATCH_COMPARISON_ALL_DONE"
