# Head-on Isaac zone, reverse recovery, and tracker check — 2026-10-28

## Scene and admission

The Windows Isaac scene was already running on ROS domain 42 with command
drive, camera, walking person, and exported ground truth. Its JSON named a
`scenario_obstacle` at odom `(1.4, 0)` m, consistent with the requested head-on
scene. No physical driver ran. The Jetson's saved `jetauto-direct-test` profile
had become inactive; `eno1` had carrier but no IPv4 address, and a route to
`169.254.71.211` incorrectly used Docker. After reactivation, `eno1` had
`169.254.71.212/16`, a 1 Gb/s carrier, and the peer route. A Fast DDS whitelist
containing both that address and loopback passed the 12 s scan/RGB/depth/TF
stream gate. The unwhitelisted probe had lost scan/RGB/depth messages while
clock and camera-info continued; the transport-selection cause is an inference.

Three stationary, 720-beam scans independently matched the saved map at
`(0.000, 0.040, -0.005)` (x, y, yaw). Across 2,134 valid returns, 93.6% were
within 0.10 m of occupied map cells; the capped mean endpoint distance was
0.0270 m. With the Isaac-only saved-map stack active, the live map start was
0.022 m from that estimate. The motion preflight saw 37 scans in 9 s, a
0.150 s scan age, a complete monitor → recovery → scan-health → e-stop command
graph, one `/clock`, active collision-monitor, planner, and controller
lifecycle nodes, free start cost, and no `base_driver`.

## Bounded head-on approach

`measure_collision_zones.py` sent a hand-built 1.20 m FollowPath toward the
box, with a 0.90 m odometry travel cap, 0.45 m proximity abort, and 50 s time
limit. The robot started with 16 scan points already in the 1.25 m slowdown
square and none in the 0.55 m stop square. Thus this run measures slowdown
ratio and stop transition, but cannot locate slowdown onset.

| Measure | Live result |
| --- | ---: |
| Slowdown ratio | 0.350, 202 sampled events |
| First observed slowdown, nearest return | 1.241 m |
| First observed monitor stop | 0.695 m odom travel, 0.548 m nearest return |
| Final odometry travel, nearest return | 0.697 m, 0.542 m |
| Controller / limited / final peak forward command | 0.100 / 0.035 / 0.035 m/s |

The controller still requested 0.0825 m/s at the first sampled monitor stop;
the monitor output was zero. The first locally paired scan counted two returns
in the stop square, while a later scan counted 20. Commands have no scan stamp,
so callback timing does not prove the configured `max_points: 3` threshold.
The FollowPath result was still pending at the 50 s limit; this was a timed
measurement, not a successful navigation goal. The harness latched e-stop but
its timeout branch had failed to cancel the action. It now routes both time
limit and other aborts through the same latch-and-cancel path, covered by a
regression test. The active stack was shut down after this run.

## Stop-zone recovery and e-stop

A stationary survey after the approach found 20–21 points in the stop square,
nearest forward return about 0.545 m, and **zero** returns in the recovery
node's one-metre rear box. The initial live reverse attempt moved 0.000 m.
With e-stop latched, new raw reverse commands produced zero
collision-monitor output samples, so the recovery node had no monitor decision
to act on. Nav2 Humble's [`publishVelocity` logic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp)
ceases repeated zero output after `stop_pub_timeout` from the first stop,
even when new raw commands arrive. The previous 2 s setting made delayed
reverse recovery unavailable.

`stop_pub_timeout` is now 86,400 simulation seconds. This keeps monitor zero
decisions available for a delayed operator reverse; recovery still requires
a fresh pure-reverse raw command, fresh clear rear scan, and monitor zero.
The monitor was restarted and its active parameter read back as 86,400.0.
The bounded 2.5 s wall-command test then observed ten capped recovery samples
while the stop square was occupied and the monitor held zero. The robot
reversed 0.0245 m over 1.25 simulated seconds (3.39 wall seconds), clearing
the stop square from 21 to zero points. After e-stop latched, 137 nonzero
commands reached its input; 140 sampled final outputs were zero and late
odometry drift was 0.000 m. After the configured window expires, recovery
fails closed. An eventual physical driver still needs an independent
command-loss watchdog.

## Paired head-on tracker bag

The profile-mode stack recorded a clean 36.53 s wall-time bag in ignored
`.agent-work/head_on_20261028/tracker_bag`: 3,282 messages, including 170
exported ground-truth frames and scans, 155 camera, 169 LiDAR, and 153 fused
track arrays. Scoring used all five exported labels, a 0.75 m center gate,
and the scorer's unchanged `stamp_sim`. The latest bag `/clock` at each of
169 ground-truth receipts led `stamp_sim` by a median 0.083376 s.

| Output | Labeled matches / expected | Scenario box | Walking person | Unmatched observations |
| --- | ---: | ---: | ---: | ---: |
| Camera | 154/770 | 0/154 | 154/154 | 0/154 |
| LiDAR | 611/840 | 168/168 | 106/168 | 2,927/3,538 |
| Fusion | 613/765 | 153/153 | 153/153 | 2,663/3,276 |

The camera result is under a 360° all-object denominator, including objects
outside its useful view. LiDAR matched the chair in only 1/168 frames, and
215 of 505 matched static-object frames were labelled dynamic; fusion labelled
197 of 460 that way. The many unmatched LiDAR/fusion outputs include room
geometry absent from exported labels and are not a false-positive rate. This
bag was recorded after the reverse test, with the robot stationary; it does
not score tracker behaviour during the zone approach.

## Profile metrics and remaining limits

The live profile stack published 47–48 metric statuses per node in a 12 s
sample from all six perception nodes. Every status contained `input_age_ms`
and `dropped_total`; the detector also published `cuda_inference_ms`. Median
`input_age_ms` ranged from 16.667 to 200.0 ms across stages. These values
are quantized against Isaac's simulation clock and are not wall latency.
The runtime profile, synthetic-output, and variant-guard tests passed.

Only head-on has been run with this scenario setup. Lateral and diagonal
scenes need separate Windows `-Scenario` restarts and fresh admission checks.
The stop-square point threshold remains unproven at a single scan stamp.
No physical stopping distance, hardware driver, or safety-rated behaviour
was tested. The saved-map localizer's turn-induced shift from the previous
validation also remains open.
