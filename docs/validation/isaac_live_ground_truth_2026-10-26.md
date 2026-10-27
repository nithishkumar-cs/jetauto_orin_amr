# Live Isaac Ethernet and exported-ground-truth validation — 2026-10-26

## Setup and motion gates

The Windows command-drive scene was reported running with a walking person,
camera, ground truth, and reset support, without `-Scenario`. On the Jetson,
`eno1` had `169.254.71.212/16`, a 1 Gb/s carrier, and a route to the Windows
peer `169.254.71.211`. The ROS graph on domain 42 independently showed one
`/clock`, `/scan`, `/odom/wheel`, `/tf`, RGB, depth, camera-info, and
`/ground_truth/obstacles` publisher, plus Isaac's sole subscriber to
`/cmd_vel/safety_limited`. No physical driver or competing final-command
publisher appeared. The initial odometry was `(0, 0)` and a scan had no points
inside either safety polygon; the nearest returns were 2.60 m ahead and
2.06 m behind.

The 20 s stream gate observed 115 scans and 115 each of RGB, depth, and camera
info (5.75 Hz wall time), 696 clocks (34.8 Hz), and 690 odometry and TF
messages (34.5 Hz). All 115 RGB/depth stamps matched. Carrier and interface
error counters did not change. The direct command-drive check measured 0.0567 m
forward travel for a 0.10 m/s command, 76.4 ms median first-motion latency
over eight trials (95.2 ms p95), zero late travel after Isaac's command
watchdog, and 453/453 timestamp-matched odometry/TF poses. With the local
safety chain active, clear-path gated travel was 0.0617 m, the latched e-stop
held zero with zero measured late travel, and reset restored 0.0383 m of
bounded motion. The test latched e-stop on exit and removed its publishers.

The first live safety harness attempt exposed a DDS profile error: binding
Fast DDS only to `eno1` let the Jetson see Isaac but prevented its CLI from
discovering local safety nodes. Adding loopback to the transport whitelist
fixed local lifecycle discovery. The second attempt exposed a stale test
assumption: both collision monitor and stop-zone recovery subscribe to raw
`/cmd_vel`. The probe now waits for the complete monitor → recovery →
scan-health → e-stop graph before sending commands. The final wired gate
passed. These are software integration checks, not physical stopping-distance
or safety-rated evidence.

## Paired tracker recording

With the robot stationary, the profile-mode detector, depth projection,
LiDAR clustering, camera/LiDAR trackers, and fusion ran against the live Isaac
topics. `rosbag/record_tracker_scoring.sh` recorded an 88.17 s wall-time bag:
9,629 messages, including 509 exported ground-truth messages, 509 scans,
498 camera and LiDAR track arrays each, and 497 fused arrays. The simulator
time covered about 50.8 s. The ROS bag stopped cleanly and contains metadata;
its gate wrapper returned an error after recording because its shell status
variable was lost, so bag metadata and subsequent scorer success are the
recording integrity evidence. The raw bag and score JSON are in ignored
`.agent-work/live_tracker_gt_20261026` and
`.agent-work/live_tracker_gt_scores_20261026.json`.

The scorer uses a 0.75 m center gate and evaluates all four exported objects
in 360° unless noted. Walls and other scene geometry are not labeled.

| Output | Labeled matches / expected | Person matches | ID switches | Unmatched track observations |
| --- | ---: | ---: | ---: | ---: |
| Camera | 563/1,988 | 497/497 | 4 | 13/576 |
| LiDAR | 1,992/1,992 | 498/498 | 32 | 11,882/13,874 |
| Fusion | 1,988/1,988 | 497/497 | 27 | 11,853/13,841 |

The camera's aggregate 28.3% recall is from a 360° all-object denominator,
including boxes outside its useful view; its person result is the relevant
visible-target check. LiDAR labeled 686 of 1,494 matched static-object frames
as dynamic, and fusion labeled 694 of 1,491 that way. The many unmatched
LiDAR/fusion outputs include unlabeled walls and other returns; they are not a
false-positive rate. Center errors also include the distance from an object's
visible surface to its simulator center.

## Two versus three LiDAR points on the same live source

The recorded inputs were replayed through the real clustering and tracking
nodes at 1× in isolated ROS domains, changing only
`perception_lidar_clustering.min_cluster_points` from two to three. Both
output bags retained TF and exported ground truth. All 508 cluster-array
stamps and 507 track-array stamps were common to both runs.

| Measure, common stamps | Two points | Three points |
| --- | ---: | ---: |
| Clusters, 508 frames | 9,494 | 9,006 |
| Confirmed LiDAR track observations, 507 frames | 14,084 | 13,433 |
| Unmatched to four exported centers | 12,060 | 11,409 |
| Labeled matches / expected | 2,024/2,028 | 2,024/2,028 |
| Person matches / expected | 506/507 | 506/507 |
| ID switches, all four labels | 34 | 38 |

The two-point minimum admitted 488 more clusters and published 651 more
confirmed track observations; the aggregate unmatched count also rose by 651.
It did not change labeled recall in this run. Separate replays can vary in DDS
scheduling, so the switch-count difference does not establish that two points
improves identity stability. The two-point default remains provisional: this
single scene cannot classify the extra outputs as false detections or prove a
better threshold for hardware.

## Simulator time and navigation status

For every recorded ground-truth message, the most recent `/clock` sample at
its bag receipt was 0.0834 simulation seconds ahead of `stamp_sim`. Shifting
ground-truth timestamps by ±0.0833 s left LiDAR and fusion labeled recall at
1.0; LiDAR median center error ranged 0.3058–0.3077 m, fusion
0.2912–0.2985 m, and camera 0.2509–0.2772 m. This is a timing sensitivity
check, not proof that either shifted timestamp is the physical sample time.
The scorer's default uses the exported `stamp_sim` unchanged.

The saved-map Nav2 stack was started without a driver or goal. Its optional
odometry EKF could not load because the installed `robot_localization` binary
requires `libdiagnostic_updater.so`, which the installed diagnostic-updater
package does not provide. The supported Isaac selection
`enable_localization:=false` left Isaac's `odom → base_link` and SLAM Toolbox's
saved-map localization active; collision monitor, planner, and controller
reached active. A no-motion 0.40 m plan had 14 poses, length 0.389 m, and
cost zero at the start, goal, and along the path. A fresh TF listener needed
about 3.5 s to see a valid map pose, so the runner now waits up to 15 s.

The localized start was `(0.9575, -0.0163)` m. The position predicted from
the prior verified map alignment and current Isaac odometry `(0.8458, 0)` was
`(1.2175, 0)` m, a 0.260 m difference. That comparison depends on the earlier
map alignment and needs a fresh scene/map ground-truth survey. The runner now
requires an independently verified expected map start within 0.15 m before
`--execute`; no Nav2 goal was sent. The robot was left stationary at odometry
x ≈ 0.846 m, with the Jetson Nav2 process stopped. A reset and verified map
pose are required before the longer/turning route. Lateral and diagonal zone
tests still need the Windows scene launched with `-Scenario`.

