# Lateral Isaac collision-zone and tracker run — 2026-10-28

## Scene and admission

The Windows Isaac scene remained on domain 42 with command drive, camera,
walking person, and exported ground truth. The exported `scenario_obstacle`
was a 0.3 m box centered at odom `(0, 1.4)` m, directly to the robot's left.
Direct Ethernet used `eno1` at `169.254.71.212/16` and the Fast DDS profile
whitelisting that interface and loopback. ROS showed one `/clock` publisher,
fresh scan/TF/odometry, the complete safety command chain ending at Isaac's
sole actuator subscriber, no physical driver, and active collision-monitor,
planner, and controller lifecycle nodes.

Three stationary scans initially registered the saved map at
`(0.000, 0.035, -0.005)` (x, y, yaw); 93.6% of 2,072 valid returns lay within
0.10 m of occupied cells. The live map pose was 0.032 m from that independent
estimate. The obstacle already contributed 12 points to the slowdown square,
so a bounded 0.30 m-cap retreat commanded `-0.10 m/s` sideways to clear both
zones. The phase measured 0.085 m odom travel before teardown; the later
approach started at odom y=`-0.110` m. The additional 0.025 m between those
readings was not isolated from callback lag or continued motion. The harness
now latches e-stop before its settled retreat reading; that adjustment has
not had a live rerun. At the clear start, both polygons counted zero points.
Three new scans registered `(0.005, -0.075, -0.005)` with 94.1% of 2,082
returns within 0.10 m of occupied cells. The live approach start differed by
0.042 m, below the 0.15 m admission gate.

## Bounded sideways approach

The Isaac-only probe sent `+0.10 m/s` lateral commands through `/cmd_vel`.
Every cycle checked fresh scan/TF, a 1.00 m odom travel cap, a 0.43 m
side-proximity abort, and a 60 s wall limit; e-stop latched on exit.

| Measure | Lateral result | Head-on comparison |
| --- | ---: | ---: |
| First sampled slowdown | 0.097 m travel; 1.248 m nearest left return | Start already slow; 1.241 m first sample |
| Sampled slowdown ratio | 0.350 across 200 events | 0.350 across 202 events |
| First sampled monitor stop | 0.804 m travel; 0.548 m nearest return | 0.695 m travel; 0.548 m nearest return |
| Final travel / nearest return | 0.805 m / 0.548 m | 0.697 m / 0.542 m |

The lateral run met the sustained-stop condition after 21.90 simulation
seconds and 44.39 wall seconds. Its final scan had nine points in the stop
square. At sampled slowdown the raw / collision-limited / final lateral
commands were 0.100 / 0.035 / 0.035 m/s; at sampled stop they were
0.100 / 0.000 / 0.000 m/s. Commands and scan callbacks have no common
decision stamp; the first locally paired slowdown scan counted two
slow-square points, so it cannot
prove the configured `max_points: 3` trigger or its exact onset. This raw
sideways command test did not send a Nav2 FollowPath goal. Three stationary
post-approach scans registered `(0.005, 0.730, -0.005)` with 89.4% of 2,053
returns within 0.10 m of the saved map.

## Stopped-state commands

The lateral box put 25 returns in the recovery node's one-metre rear
clearance box. Pure reverse was therefore ineligible under the existing
recovery rule, which requires a clear rear. A 0.5 s bounded reverse check
observed 11 zero final commands and 0.000 m odom movement. Separate 0.5 s
forward and strafe checks observed 12 zero final commands each and
0.000 m movement. Each phase latched e-stop on exit.

An earlier combined reverse/strafe/forward check stopped during its strafe
step: at least one sampled final command exceeded the probe's 0.001 m/s
zero threshold, and odom y moved from 0.69475 to 0.69650 m (0.00175 m).
The forward step of that combined check was not reached. The collision-monitor
log switched from stop through slowdown and normal back to stop over roughly
0.22 s during this check. The later isolated strafe and forward checks passed,
so the nonzero output was intermittent. The probe latched e-stop through its
failure teardown. The scan used by each monitor decision was not
recorded with a shared timestamp, so this run does not prove which return
count caused the transition. It is a real gap in a claim of continuous stop
authority at this boundary and needs a scan/decision-paired investigation.

## Paired tracker bag

The profile stack recorded a 35.29 s wall-time stationary bag after the
approach in ignored `.agent-work/lateral_20261028/tracker_bag`. It contains
3,634 messages, including 173 exported ground-truth frames, 175 scans, and
170/172/170 camera/LiDAR/fused track arrays. Scoring kept the five exported
labels and unchanged `stamp_sim`; the latest bag `/clock` led ground-truth
`stamp_sim` by a median 0.083370 s at 173 receipts.

| Output | Labeled matches / expected | Scenario box | Walking person | Unmatched observations |
| --- | ---: | ---: | ---: | ---: |
| Camera | 309/845 | 0/169 | 169/169 | 0/309 |
| LiDAR | 855/855 | 171/171 | 171/171 | 4,254/5,109 |
| Fusion | 850/850 | 170/170 | 170/170 | 4,228/5,078 |

The camera denominator spans 360° exported objects, including ones outside
the camera view. LiDAR matched the chair 171/171 from this stopped pose, but
classified 137/684 matched static-object frames as dynamic; fusion classified
143/680 that way. Unmatched LiDAR and fusion outputs include room geometry
absent from the five labels and are not a measured false-positive rate. The
bag was recorded with the robot stopped, not during the approach. This is
simulation wiring and scene-specific scoring evidence, not physical braking
distance or a safety-rated result. Isaac did not export an independent robot
ground-truth pose in this run; scan-to-map fit is not pose truth.
