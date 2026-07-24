# Mission Capture Run — 2026-07-24 18:09–18:54

Full 3-agent swarm mission run, captured for advisor review round 1
(comments #12, #13, #23, #28, #32, #33 — real data logs/screenshots/video
backing the paper's results, and material for replacing the mismatched
"landed" figure).

- `full_run.mp4` — 45m22s screen recording, 1920x1080 @ 8fps, Gazebo +
  live telemetry dashboard side by side (as auto-tiled by the launch file).
- `screenshots/` — 90 full-desktop screenshots, one every 30s.
- `logs/launch_stdout.log` — full ROS node console output (all 3 agents).
- `logs/<agent>_{role,activity,battery,landed}.log` — per-agent topic
  dumps, timestamped (unix epoch seconds). Note: `ros2 topic echo` emits
  a `---` YAML separator after every message; filter with
  `grep -v -- '^[0-9.]* ---$'` before parsing.

## Key events located in this run (cross-referenced timestamp -> screenshot)

| Event | Timestamp (unix) | Evidence |
|---|---|---|
| scout_2 SAMPLER bid accepted, 9.0 m jump to anomaly | 1784902167 | launch_stdout.log |
| scout_1 reaches sustained LANDED (305 s stable) | 1784902180–1784902485 | scout_1_landed.log; screenshots 2–11 |
| SAMPLER activity window (scout_2/scout_3) | ~1784902530–1784902590 | scout_2/3_role.log; screenshot 14 |
| scout_1 enters RECHARGE, battery climbing 16%→27%+ | ~1784903969–1784904200 | scout_1_activity.log; screenshots 59–69 |
| scout_3 settles badly tilted, RW self-righting triggered | 1784904446 | launch_stdout.log |
| scout_3 self-righting **succeeded on attempt 1/5** (~4 s) | 1784904450 | launch_stdout.log; screenshots 76–77 bracket it |

## Known limitation

The Gazebo camera stayed at its default wide view (entity-tree panel open)
for the whole run — there was no interactive camera control during this
unattended capture. Screenshots are reliable for **verifying dashboard
telemetry state** (role/battery/landed/drill), not for close-up "hero"
robot photography. A separate short targeted capture
(`get_closeup_run/`, if present) was done afterward specifically to get a
properly framed, confirmed-landed close-up for the Fig. 8 replacement.
