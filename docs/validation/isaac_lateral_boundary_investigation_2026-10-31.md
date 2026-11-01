# Lateral stop-boundary investigation — 2026-10-31

## Evidence recovered

The 2026-10-28 combined stopped-state check recorded one nonzero final strafe
command and 0.00175 m of odometry motion. The collision monitor's own log
changed from stop to slowdown, then normal, then stop in 0.225 s (wall time).
The later isolated strafe and forward checks stayed at zero. No scan or command
bag was recorded during the failed check, so the scan consumed by each monitor
decision cannot be identified retrospectively.

Nav2 Humble's [stop/slowdown implementation](https://api.nav2.org/nav2-humble/html/collision__monitor__node_8cpp_source.html)
triggers a polygon only when its transformed point count is **greater than**
`max_points` (three here). The [scan source](https://api.nav2.org/nav2-humble/html/scan_8cpp_source.html)
contributes no points if its sample is stale or its time-shifted TF lookup
fails. The monitor's normal output therefore does not establish a clear
physical stop zone. The separate scan-health gate has its own scan and TF
subscription; its passing a command does not prove what the monitor consumed.

Nine saved single scans bracketed earlier phases of the lateral run. One scan
near the eventual stop position had three stop-square returns and 56 slow-square
returns; the next two had seven stop-square returns each. The later stationary
tracker bag contains 175 scans, all recorded **after** the failed challenge.
Applying the recorded base-to-LiDAR transform `(0.15, 0, 0)` m and the configured
squares gave stop counts from 4 to 18 and slow counts from 53 to 57. The
stationary stop count sits near the `>3` trigger: moving scan points outward by
just 2 mm in a sensitivity calculation lowered 22/175 scans to three or fewer
stop returns; 5 mm lowered 160/175. Those shifts are hypothetical, not a
measurement of TF error or sensor motion.

The boundary sensitivity can explain a brief stop-to-slow transition. The
normal transition needs a different explanation because the slow square held
more than 50 returns in every later recorded scan. A transient omitted scan
source, TF timing failure, or unrecorded scan anomaly remains possible. The
available log has no source-timeout or TF error line, but it does not contain
per-decision scan IDs or point counts. The precise cause remains unverified.

## Diagnostic change and next check

`measure_lateral_zone.py --trace-output <path>` now writes local receipt events
for raw, collision-limited, recovery-limited, scan-health-limited and final
commands, collision-monitor action logs, complete scans, locally transformed
stop/slow counts, clock, and odometry. It writes the trace even if the probe
fails, after latching e-stop. The trace uses local receipts; it still cannot
identify the exact scan consumed inside the monitor, which exposes no such
stamp in this installed version.

The Isaac scene is not publishing on domain 42 as of this investigation, so no
live motion was attempted. A rerun needs the lateral scene restored, the same
single-clock/map-start/fresh-scan/full-chain/no-driver admission gates, and
independent travel, proximity, and time aborts. Record `/scan`, `/tf`,
`/odom/wheel`, `/cmd_vel`, each command stage, and `/rosout` through the
challenge. Compare each nonzero output with the trace's latest scan and TF
availability; if those disagree with the monitor's action, instrument the
monitor's consumed scan stamp and point counts in an isolated build before
changing the zone threshold or command policy. Continuous stop authority at
this boundary remains unverified.

## Live lateral rerun, 2026-10-31

The restored direct Ethernet profile and domain-42 scene passed a 10 s probe:
one `/clock` publisher; fresh scan, odometry, TF, RGB, depth, and camera-info
streams; and Isaac as the final-command actuator subscriber. Three stationary
scans registered the saved map at `(0.000, 0.040, -0.005)` with 93.6% of 2,084
returns within 0.10 m of occupied cells. The no-driver profile stack passed
the full graph, lifecycle, freshness, and 0.15 m independent map-start gates.

The initially occupied slow square was cleared with a bounded sideways retreat.
The live map residual was 0.030 m. The clear phase stopped after 0.0762 m in
its immediate odometry snapshot; its settled pose was 0.0978 m from the start.
The trace shows 0.0217 m of subsequent odometry change during zero-command
settling, including 0.0133 m after the first observed zero final command. The
pose then held steady. This accounts for the earlier unexplained difference
between an immediate retreat reading and a later start; the trace does not
separate queued command delivery from simulated dynamics. Three clear-start
scans registered `(-0.005, -0.060, -0.005)` with 93.7% of 2,079 returns near
occupied cells.

A 0.10 m/s raw lateral approach, limited to 1.00 m travel, 0.43 m proximity,
60 s wall time, fresh scan/TF, and e-stop teardown, stopped after 0.794 m at a
0.544 m nearest return. The start map residual was 0.024 m. The trace recorded
stop/slow alternation near the boundary. One locally received scan counted
three stop-square and 56 slow-square returns immediately before a slowdown
log; stop returned on the next decision. This supports sensitivity to the
configured `>3` stop threshold, but local receipt order does not identify the
scan consumed inside Nav2. No normal transition occurred in this approach.

Three stopped scans registered `(0.015, 0.735, -0.010)` with 89.4% of 2,068
returns near occupied map cells. A combined reverse/strafe/forward challenge
and six further combined challenges each passed their independent map-start,
freshness, command-graph, travel, proximity, time, and e-stop checks. The rear
box contained 25 returns, so reverse recovery stayed blocked. The first
challenge yielded 11 zero final samples each for strafe and forward and zero
odometry travel; the six repeats also had zero odometry span and 5–9 minimum
stop-square returns across their scans. None logged a normal transition or
reproduced the earlier nonzero final strafe command. The paired bag contains
`/scan`, `/tf`, `/tf_static`, `/odom/wheel`, `/clock`, `/rosout`, and all five
command stages; local traces and the bag are under ignored
`.agent-work/lateral_20261031/`.

The separate FollowPath `--verify-stopped` helper did not complete: its
preflight saw no occupancy grid within its wait, so it sent no command. The
stopped-state traces themselves recorded zero final commands and zero odometry
span across the seven challenges.

Recording initially made the harness reject the command graph before motion:
the graph check expected exactly the actuator and probe subscribers. It now
allows the passive `rosbag2_recorder` while still requiring Isaac and the full
chain. The rerun exercised this check. No zone threshold or command policy
was changed. The old stop/slow/normal/stop event has no paired scan, and these
passing repeats cannot establish its cause or continuous stop authority. A
fresh scene reset and an instrumented monitor that reports its consumed scan
stamp, TF result, and polygon counts are needed to resolve that event.

## Reset verification and paused rerun, 2026-10-31

After the laptop reset, domain 42 still had one `/clock` publisher and Isaac
subscribed to `/cmd_vel/safety_limited`. A live `/odom/wheel` sample put the
robot at `(0, 0, 0)` m, and `/ground_truth/obstacles` still placed
`scenario_obstacle` at its original `(0.0, 1.4)` m. No Jetson motion command
was sent.

Three passive stream probes failed the live freshness gate. The second observed
only 1.667 s of simulation-clock progress over 20.0 s of wall time: 17 scans,
101 clock samples, and maximum wall arrival gaps of 3.341 s for scans and
2.653 s for clock/odometry. Camera streams were similarly slow. This is
consistent with slow Isaac stepping rather than a missing DDS publisher: the
clock stamps advanced in roughly 0.017 s increments without going backward.
The third observed only 0.267 s of simulation-clock progress in 10.0 s wall
time, with three scans. The bounded approach and boundary challenge were
therefore not admitted.

A matching Nav2 Humble 1.1.20 collision-monitor source was staged in ignored
`.agent-work/nav2_humble_diag/` and built with diagnostic logs for each command
decision. The scratch build reports the scan stamp and whether it was missing,
stale, rejected by TF, or accepted, then records per-polygon point counts and
the monitor output. It has not been run live. The next session must first
restore normal scan/TF freshness, recheck the independent map start, and keep
the same bounded motion and e-stop gates.
