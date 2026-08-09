#!/bin/bash
# Phase 7: full end-to-end mission-loop run for archived telemetry, same
# style/node-set as the existing 45-minute reference run
# (docs/paper_assets/sim_captures/run_20260724_180901/), against the
# FINAL model and controller stack (post Phase 2-6).
#
# LIMITATION (documented, not silently skipped): the original run also
# captured a 45-minute screen recording (full_run.mp4) and periodic
# desktop screenshots via an external ffmpeg/screenshot wrapper that was
# never itself committed to this repo. This environment has no ffmpeg,
# Xvfb, scrot, or ImageMagick installed and no passwordless sudo to add
# them, so video/screenshot capture is NOT reproduced here -- only the
# telemetry (per-agent role/activity/battery/landed topic dumps + full
# node console log), which is the substantive Data Availability content.
set -x
source /opt/ros/humble/setup.bash
source /home/melvin/ryugu_v2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/melvin/ryugu_v2_ws/src/ryugu_sim/models
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "scout_1|scout_2|scout_3|swarm_manager|swarm_gui|spawner" 2>/dev/null
sleep 3

D="/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase7_full_revalidation/mission_loop_run"
mkdir -p "$D"
cd "$D"

ros2 launch ryugu_sim ryugu_swarm.launch.py > launch_stdout.log 2>&1 &
LAUNCH_PID=$!

sleep 15

for agent in scout_1 scout_2 scout_3; do
  ros2 topic echo /$agent/status_role      > ${agent}_role.log      2>&1 &
  ros2 topic echo /$agent/status_activity  > ${agent}_activity.log  2>&1 &
  ros2 topic echo /$agent/status_battery   > ${agent}_battery.log   2>&1 &
  ros2 topic echo /$agent/landed           > ${agent}_landed.log    2>&1 &
done

sleep 2700   # 45 minutes

pkill -9 -f "ros2 topic echo" 2>/dev/null
kill -9 $LAUNCH_PID 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "scout_1|scout_2|scout_3|swarm_manager|swarm_gui|spawner" 2>/dev/null
echo "MISSION_LOOP_CAPTURE_DONE"
