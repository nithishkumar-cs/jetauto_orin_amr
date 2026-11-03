# Lateral boundary repro — 2026-11-01

## Admission and bounded approach

The laptop's reset lateral scene published one `/clock`, an odometry pose near
`(0, -0.039)` m, and `scenario_obstacle` at `(0, 1.4)` m. The Jetson's `eno1`
held `169.254.71.212/16`. The profile-mode, no-driver stack used synthetic
camera stages and the full software command chain. The observed clock rate was
about 23–30 Hz against Isaac's 60 Hz simulation clock, above the requested
0.3 real-time-factor admission gate. Three stationary scans independently
registered the saved-map start at `(0.000, 0.000, -0.005)`; 93.7% of 2,075
valid returns were within 0.10 m of occupied map cells.

Two harness attempts sent no motion because a new process lacked the running
stack's `FASTRTPS_DEFAULT_PROFILES_FILE`. Fresh, no-daemon graph discovery saw
Isaac but none of the local safety nodes. Using the stack's profile restored
the full graph. The harness now names the actual endpoints when this gate
fails. The bounded clear phase passed all graph, scan/TF, map-start, no-driver,
travel, proximity, time, and e-stop checks. It settled at odometry
`(0, -0.0977)` m with both zones clear; its map-start residual was 0.011 m.

The first approach also passed admission (map-start residual 0.065 m). A raw
`+0.10 m/s` sideways command slowed at a 0.350 ratio and reached a sustained
monitor stop after 0.793 m odometry travel, at a 0.548 m nearest scan return.
It took 44.42 s wall and 21.92 s simulation time. The approach was limited to
1.00 m travel, 0.43 m nearest left return, 60 s wall time, fresh scan/TF and
odometry, with e-stop teardown. Its final scan counted 11 stop-square and 56
slow-square returns. No Nav2 goal or physical driver was involved.

## Stopped-state threshold dip and test abort

Three stationary stop-position scans registered `(0.005, 0.730, -0.005)` in the
saved map; 89.4% of 2,055 valid returns were within 0.10 m of occupied cells.
The stopped-state challenge passed preflight and saw an occupied rear box. The
harness required every reverse output to remain zero. A scan with stamp
`614.266698702` and 3 stop-square / 56 slow-square returns arrived while the
raw reverse command was `-0.06 m/s`. The live monitor changed stop to slow,
then `/cmd_vel/collision_limited`, recovery, scan-health, and final output
carried `-0.021 m/s`. The harness caught nine nonzero final samples, aborted
the challenge, and latched e-stop. This was a test-rule mismatch: with
`max_points: 3`, exactly three stop-square returns do not trigger stop, while
56 slow-square returns select the configured 0.35 ratio. The owner's later
decision A updates the test expectation to match that rule. No strafe or
forward challenge was sent.
The trace's odometry was unchanged during the short check; settled odometry
later showed 0.00245 m reverse motion. This is simulated odometry, not a
physical stopping-distance measurement.

The live monitor also logged “continue normal operation” before returning to
stop. A separate, isolated DDS-domain replay of the paired bag through the
instrumented Nav2 Humble 1.1.20 monitor produced these decisions:

| Raw command | Consumed scan stamp (s) | Scan/TF | Stop / slow points | Output |
| --- | ---: | --- | ---: | ---: |
| `-0.06 m/s` | 614.166698697 | accepted | 11 / 57 | `0` (stop) |
| `-0.06 m/s` | 614.266698702 | accepted | 3 / 56 | `-0.021 m/s` (slow) |
| `0` | 614.266698702 | accepted | 3 / 56 | `0` (normal action) |
| `0` | 614.366698707 | accepted | 15 / 56 | `0` (stop) |

The replay identifies a three-point stop-count dip as sufficient for this
nonzero output under the configured `max_points: 3` rule. Its normal-action
transition corresponds to a zero raw command even though the slow square was
occupied; that log line alone is not evidence of an unmonitored nonzero
command. Replay scheduling can differ from live DDS scheduling, and the live
monitor was not instrumented, so the replay does not independently prove the
exact scan consumed by the live monitor. The 2026-10-28 nonzero strafe output
still lacks a paired bag and has no established cause. The current zone
dimensions, threshold, and recovery policy remain unchanged. This run does
not establish continuous stop authority at a noisy geometric boundary.

## Evidence and remaining work

The ignored `.agent-work/lateral_20261101_sol/` directory holds the full
13.2 MiB paired bag (58,329 messages over 160.88 s), complete harness traces,
three start and three stop-position scans, and the isolated diagnostic replay
logs. The paired bag contains `/scan`, `/tf`, `/tf_static`, `/odom/wheel`,
`/clock`, `/rosout`, and raw, collision-limited, recovery-limited,
scan-health-limited, and final command topics. The isolated replay used DDS
domain 142 with no Isaac actuator subscriber.

Only one of the requested five approaches ran. The old test assertion stopped
the challenge, and another approach requires the laptop's local
`scripts/windows/reset_command_drive.ps1` flag-file reset. No ROS reset
topic or service was present on domain 42. After a reset, independently
verify the origin, obstacle position, map start, stream freshness, and full
safety graph before any repeat. Investigate the boundary's point-count
sensitivity and the older strafe event before claiming continuous stop
authority or changing policy. No physical-robot result or safety-rated claim
follows from this simulation run.

## 2026-11-02 matrix run 1 (decision A rule)

Escape now counts only when StopZone points exceed `max_points` (3), via
`STOP_ZONE_MAX_POINTS` in `tests/integration_tests/measure_lateral_zone.py`.
Stack relaunched on the Jetson (stale 22 h bringup stopped); Isaac about 0.6x
real time. Artifacts are in ignored `.agent-work/lateral_20261102/`.

- Approach 1: 0.799 m odom travel, nearest return 0.549 m, slowdown ratio
  0.350, stopped with 4 StopZone points and zero final command.
- Challenge 1 (preflight start residual 0.010 m): rear box occupied (26 points),
  reverse blocked (0.000 m), strafe and forward each 12 zero final samples and
  0.000 m travel.
- The earlier 0.00175 m strafe leak was not reproduced. The check still pairs
  each output row with the stop count at the next scan, not the consumed scan.
- Approach 2 (after scene reset): 0.804 m travel, nearest return 0.543 m,
  slowdown ratio 0.350, stopped with 7 StopZone points, zero final command.
  Challenge 2 (start residual 0.025 m): rear box 25 points, reverse 0.000 m,
  strafe and forward 12 zero final samples each, 0.000 m travel.
- Approach 3 (after scene reset): 0.808 m travel, nearest return 0.544 m,
  slowdown ratio 0.350, stopped with 7 StopZone points, zero final command.
  Challenge 3 (start residual 0.022 m): rear box 26 points, reverse 0.000 m,
  strafe and forward 12 zero final samples each, 0.000 m travel.
- Approach 4 (after scene reset): 0.791 m travel, nearest return 0.551 m,
  slowdown ratio 0.350, stopped with 5 StopZone points, zero final command.
  Challenge 4 (start residual 0.018 m): rear box 25 points, reverse 0.000 m,
  strafe and forward 12 zero final samples each, 0.000 m travel.
- Approach 5 awaits a laptop scene reset; four of five completed.
