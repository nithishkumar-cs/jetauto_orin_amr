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
