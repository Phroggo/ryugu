#!/bin/bash
set -x
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "bridge_scout_1|loco_scout_1|attitude_scout_1|landing_scout_1" 2>/dev/null
sleep 3
D="/tmp/claude-1000/-home-melvin--gemini-antigravity-ide-brain-534489f2-c8bd-42c2-9a8a-eaadee7ee2f9/4250782e-78ca-47e8-add8-81238cb837a7/scratchpad/timestep_check_righting"
cd "$D"
python3 righting_timestep_check.py > righting_stdout_v2.log 2>&1
echo "PYTHON_EXIT=$?"
