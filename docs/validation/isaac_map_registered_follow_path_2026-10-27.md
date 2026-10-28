# Map-registered Isaac FollowPath and settled residual — 2026-10-27

## Independent map start

The earlier `map_start_pose=(0.3717, 0)` m produced a 0.263 m mismatch after
Isaac returned to odometry zero. To check the start independently of SLAM
Toolbox, `tests/integration_tests/estimate_isaac_map_start.py` matched three
stationary, 720-beam live scans to occupied cells of the saved
`loop_20260928` map. The script used Isaac's `base_link → lidar_link`
translation `(0.15, 0)` m, the map's 0.05 m resolution and origin, and a
bounded x/y/yaw search. It capped each endpoint distance at 0.4 m so a walking
person and unmapped returns could not dominate the fit. Raw scans remain under
ignored `.agent-work/`.

The three scans independently selected map start `(0.000, 0.035, -0.005)`
(x, y, yaw). Across 2,087 valid returns, the capped mean distance to occupied
map cells was 0.0116 m and 97.4% were within 0.10 m. At the old start guess,
only about 54% of returns were that close; the earlier localizer estimate
`(0.112, 0.042)` m placed about 91–94% within 0.10 m. Reinitializing saved-map
localization at the scan-derived pose produced live start
`(-0.0051, 0.0129)` m, 0.023 m from the independent estimate. The original
start convention, rather than a different Isaac room, explains the 0.263 m
start-gate failure.

## Guarded route results

The direct domain-42 stream probe again saw one `/clock`, scan, odometry, TF,
RGB, depth, camera-info, and ground-truth publisher, plus Isaac's final-command
subscriber. Before each goal, collision monitor, planner, and controller were
active; the runner checked the complete software command graph, map-start
residual below 0.15 m, fresh scan/TF, clear start and path cells, and absence
of a physical driver. It independently aborted on travel above 0.65 m, LiDAR
range at or below 0.55 m, stale scan, or 50 s. Each run latched the software
e-stop and measured zero late drift; the no-driver stack was restarted before
the next run.

| Goal offset (map x, y) | Action | Odom travel | Settled map residual | Fixed-alignment odom residual | Min LiDAR range | Sim / wall elapsed |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `(0.40, 0)` m | succeeded | 0.254 m | 0.171 m | 0.143 m | 2.082 m | 5.40 / 9.01 s |
| `(0.50, 0)` m | succeeded | 0.354 m | 0.177 m | 0.146 m | 1.776 m | 6.65 / 11.43 s |
| `(0.35, 0.20)` m | succeeded | 0.294 m | 0.246 m | 0.131 m | 1.513 m | 10.77 / 18.31 s |

The three paths had 15, 18, and 16 poses, with maximum sampled cost zero.
The start residuals were 0.023, 0.003, and 0.034 m. The first two routes
published 54 and 67 scans during execution; the turning route published 107,
with 22 scan-matched pose updates and 0.274 rad/s peak final angular command.
Peak final forward speeds were 0.0667, 0.0833, and 0.0500 m/s. The e-stop
blocked 18, 19, and 18 later test commands, with 0.000 m late travel in each.

The fixed-alignment odom residual applies the scan-derived map-to-odom
transform at the reset start to each Isaac odometry endpoint. It is a
geometric reference, **not exported robot ground truth**; the exported
`/ground_truth/obstacles` topic covers obstacles. Both straight endpoints and
the turning endpoint entered the configured 0.15 m circle by that reference.
The action results are consistent with success near tolerance entry followed
by a map-frame correction of 0.03–0.12 m. The map pose at the exact goal-check
instant was not recorded, so that ordering remains an inference.

Two stationary post-turn scans independently matched the map at about
`(0.875, 0.145, 0.164)` (x, y, yaw), 0.008 m from the fixed-alignment odometry
estimate `(0.882, 0.148)` m. Their 1,380 valid returns had a 0.0159 m capped
mean endpoint distance and 96.3% within 0.10 m. The settled localizer XY
`(0.782, 0.090)` m was 0.108 m from that independent scan fit. This supports
localization correction as the source of the enlarged turning residual.

## Limits and next check

The map registration is a LiDAR-to-occupancy-map consistency check in one Isaac
room; it does not independently measure physical pose or validate hardware.
The saved-map localizer's turn-induced map-frame shift needs investigation
before claiming tight goal accuracy. The goal checker tolerance was not
changed. Head-on, lateral, and diagonal obstacle-zone runs require the Windows
scene to be restarted with the corresponding `-Scenario` setting.

A later stationary restart initialized SLAM Toolbox at the post-turn scan
estimate `(0.875, 0.145, 0.164)`. Its map TF stayed at `(0.985, 0.108)` m
for ten seconds, and a separate 12 s `/pose` subscription received no new
pose. This is not a convergence test: the configured 0.05 m travel and
0.05 rad heading thresholds admit no fresh scan-match update while the base
is stationary. A bounded moving replay with robot-pose reference is needed
to isolate the turn shift.
