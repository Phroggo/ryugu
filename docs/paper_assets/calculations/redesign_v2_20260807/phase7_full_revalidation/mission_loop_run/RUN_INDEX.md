# Phase 7 mission-loop reference run

Date: 2026-08-09. Full 3-agent swarm mission (`ros2 launch ryugu_sim
ryugu_swarm.launch.py`), 45 minutes, against the final post-Phase-2-7
model and controller stack. Same node set and telemetry-logging
convention as the original Data Availability reference run
(`docs/paper_assets/sim_captures/run_20260724_180901/`), produced via
`../mission_loop_capture.sh`.

**Video/screenshot capture NOT reproduced** (see that script's header
comment): this environment has no ffmpeg/Xvfb/scrot/ImageMagick
installed and no passwordless sudo to add them. Only telemetry was
captured this run — see the Phase 7 change report for the full caveat.

## Contents

- `launch_stdout.log` (497KB) — full node console output, all 3 agents +
  swarm_manager + swarm_gui + spawner.
- `scout_{1,2,3}_role.log`, `_activity.log`, `_battery.log`, `_landed.log`
  — per-agent `ros2 topic echo` dumps of `/scout_N/status_role`,
  `/status_activity`, `/status_battery`, `/landed`. Same `ros2 topic echo`
  gotcha as the original run: filter the `---` YAML separator lines
  before parsing (`grep -v -- '^---$'`).

## Quick summary (from launch_stdout.log)

- 11 `✅ LANDED — stable contact confirmed` events across the 3 agents.
- 10 RW self-righting attempts triggered; 2 exhausted all 5 attempts and
  force-marked LANDED anyway (scout_1, twice) -- expected per Phase 7's
  own self-righting findings (full/severe inversion recovery is a known,
  now-quantified hard case), not a new issue.
- 199 swarm_manager spectral-anomaly detections logged.
- 8 SAMPLER-related log lines.
- 62 benign `gz-1 NodeShared::RecvSrvRequest() error sending response:
  Host unreachable` lines (a known gz-sim internal service message, not a
  crash -- seen throughout this project's other long-running captures).
  No Python tracebacks or node crashes in the full log.
