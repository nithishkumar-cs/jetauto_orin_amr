# Isaac command drive over direct Ethernet — 2026-10-13

## Setup

The Windows laptop ran one headless Isaac Sim command-drive instance (reported
PID 38528) on ROS domain 42, consuming `/cmd_vel/safety_limited`. The Jetson
used `eno1` at `169.254.71.212/16` and an Ethernet-only Fast DDS profile.
Jetson-side ROS graph inspection independently found exactly one `/clock`
publisher, one Isaac subscriber on `/cmd_vel/safety_limited`, and one publisher
each for `/odom/wheel`, `/tf`, and `/scan`. No physical base driver was started.

These were short kinematic simulator motions, not Nav2 navigation or hardware
braking tests. The laptop-side Isaac commit was reported as `791e158`; its
session log is
`D:\Projects\robotics\jetauto_isaac_sim\.agent-work\isaac_command_drive_session.log`.

## Jetson-to-Isaac command-drive probe

Ran `tests/integration_tests/probe_isaac_command_drive.py` with its required
`--direct-isaac-simulation-only` flag. Before publishing, it rejected any
second `/clock` source, non-Isaac command subscriber, or other publisher on
the final command topic. It streamed a 0.10 m/s forward Twist for one wall
second, published zero, then separately let the simulator watchdog expire.

| Observation | Result |
| --- | ---: |
| Forward displacement from `/odom/wheel` | 0.0917 m |
| Lateral displacement | 0.0000 m |
| Jetson publish to first observed odometry motion | 207.5 ms, one trial |
| Movement during watchdog phase | 0.1100 m |
| Movement over the final 0.3 s after commands ceased | 0.0000 m |
| Timestamp-matched odometry/TF positions | 226 matches, 0 mismatches |

The watchdog result establishes that the simulator eventually stopped after
the command stream ended. The probe did **not** measure the exact timeout
edge over the network; the laptop-local harness separately reported a 0.500 s
timeout and 16 ms first-motion latency. The 207.5 ms figure is a single
Jetson-side observation, not a latency distribution or a real-time guarantee.

## Full software safety chain into Isaac

Launched `robot_bringup` in `profile` mode with `backend:=isaac`, safety enabled,
and drivers, teleop, localization, navigation, perception, and other optional
components disabled. Collision monitor reached `active [3]`; live Isaac scans
fed both it and `scan_health_gate`. ROS graph inspection found collision
monitor as the sole `/cmd_vel` subscriber, `estop_gate` as the sole publisher
of `/cmd_vel/safety_limited`, and Isaac as its sole non-test subscriber.

Ran `tests/integration_tests/probe_isaac_safety_motion.py` with the required
`--isaac-simulation-only` flag. It continuously published 0.10 m/s on
`/cmd_vel`, then engaged e-stop while commands continued, reset it, and
finally re-engaged e-stop and sent zero:

| Observation | Result |
| --- | ---: |
| Clear-path movement through all gates | 0.1067 m |
| Final movement while e-stop latched and commands continued | 0.0000 m |
| Movement after reset | 0.0633 m |

Both probes passed. The four Jetson safety processes were stopped cleanly
after the test; the Windows Isaac instance was left running. The first
bringup attempt used `debug` logging and produced excessive ROS middleware
output; it was stopped and the successful run used `profile`/INFO logging.
No test process or command publisher remains on the Jetson.

## Limits and next gate

- No Nav2 `FollowPath` goal was sent. The saved-map localizer, planner, local
  costmap, and controller have not yet been tested as a **moving** closed loop
  with this command-drive scene.
- Collision-zone stop/slow behavior under a moving obstacle was not exercised.
  The clear-path pass and e-stop check do not validate physical stopping
  distances or the provisional zone dimensions.
- Scan-loss behavior has offline tests, but the Isaac LiDAR source was not
  deliberately interrupted during this live drive.
- The simulator applies idealized kinematic motion; no motors, traction,
  acceleration limits, mechanical brake, or safety-rated stop are represented.

The next controlled-motion gate is a short, prechecked Nav2 `FollowPath`
request in Isaac only, using a scene-matched saved map and verified start
pose/costmap. This report does not authorize a hardware drive.

## Automated wired acceptance before the camera fix

The Jetson-side sequence is now a single guarded command:

```bash
python3 scripts/validation/run_isaac_wired_validation.py --isaac-simulation-only
```

It generates a DDS profile bound to the current `eno1` address, rejects a
missing/slow link or unexpected peer route, samples all expected Isaac topics,
runs the direct command and watchdog probe, starts only the safety nodes,
runs e-stop/reset against odometry, and cleans up its own launch. It records
machine-readable results and individual logs; the sustained run's artifacts
are at `/tmp/jetauto_wired_validation_jdunrafb/`.

The 60-second observation received 3,711 `/clock`, 619 `/scan`, 3,706 wheel
odometry, and 3,707 dynamic TF messages. These streams had no backward
timestamps; their maximum wall arrival gaps were 0.027 s for clock/odom/TF
and 0.133 s for scans. The clock progressed 61.833 simulation seconds in
60 wall seconds. The direct-command probe passed eight Jetson-publish to
first-observed-odometry-motion trials at 60.6 ms median / 89.5 ms p95, with
749 timestamp-matched odom/TF positions and no mismatch. The watchdog again
stopped motion. The safety-chain probe moved 0.1017 m on a clear path, stopped
with 0.0000 m late movement while e-stop was latched and commands continued,
then moved 0.0650 m after reset. The safety launch exited cleanly, no Jetson
final-command publisher remained, and `eno1` recorded zero carrier changes,
errors, or drops during the run.

**Result at this stage: incomplete (runner exit 2).** The command-drive scene
published zero `/camera/rgb/image_rect_color`,
`/camera/aligned_depth_to_rgb/image_raw`, and `/camera/rgb/camera_info`
messages, with **zero DDS publishers** on all three topics. The earlier wired
perception run did have all three camera streams, so this is a command-drive
scene regression or mode difference, not evidence of an Ethernet bandwidth
limit. The apparent 1.03× simulation speed here was measured without camera
traffic and must not be compared as a full-perception performance result.
The Isaac-side camera path was then repaired and the acceptance suite rerun;
the final result is below.

The Jetson runner cannot start Isaac on Windows: Windows port 22 was not
reachable over the cable. A Windows-side wrapper could start the headless
scene, invoke the Jetson runner over the already-working Windows→Jetson SSH
path, and stop the scene. That cross-machine launcher is not yet implemented.
No Nav2 path goal, moving obstacle-stop trial, cable-unplug recovery test,
or hardware experiment was performed by this acceptance suite.

## Final camera-enabled wired acceptance

The Windows Isaac project added camera publishers to opt-in command-drive
mode (reported local commit `d4988d1`, not pushed) and restarted one headless
instance, reported PID 11816. The unchanged Jetson runner then **exited 0**.
Its machine-readable record is
`/tmp/jetauto_wired_validation_9_u4jsqe/summary.json`; the full runner log is
`/tmp/isaac_wired_run_camera.log`.

Over 60 wall seconds on the direct 1 Gb/s link, the Jetson received 2,128
clock messages, 354 scans, 354 RGB frames, 354 aligned-depth frames, 2,125
CameraInfo messages, and 2,124 each of odometry and dynamic TF. All 354
RGB/depth timestamps matched exactly. No sampled stream had a backward
timestamp. The maximum wall gaps were 0.268 s for RGB, 0.266 s for depth,
0.245 s for scans, and 0.055 s for clock/odom/TF. There was exactly one
publisher each for clock, scan, odom, dynamic TF, RGB, depth, and CameraInfo;
the three expected static-transform nodes published `/tf_static`.
Ethernet carrier, error, and drop counters did not change during the suite.
An independent post-run Jetson graph check again found the RGB/depth
publishers, exactly one clock source, and zero Jetson final-command publishers.

With the camera enabled, the clock advanced only 35.45 simulation seconds
in 60 wall seconds (0.59× real time), and RGB/depth/scan arrived at about
5.9 Hz wall time despite their approximately 10 Hz simulation-time cadence.
CameraInfo arrived at approximately 35.4 Hz wall time; that rate is inherited
from the default scene. This is acceptable for the current functional test,
but wall-time performance is a measured limitation, not a real-time claim.

The direct command/watchdog stage passed. Its first forward pulse took
225.9 ms from Jetson publication to first observed odometry motion; eight
subsequent short trials had 71.1 ms median / 81.2 ms maximum. The simulator
stopped after command timeout, with 0.0000 m late movement. Odometry and TF
agreed on 491 timestamp-matched samples. The full safety chain moved 0.0633 m
on a clear-path command, stopped with 0.0000 m late movement while e-stop
blocked continuing commands, and moved 0.0467 m after reset. Its launch
exited cleanly and left no Jetson command publisher.

**Ethernet functional acceptance is complete for this camera-enabled Isaac
command-drive scene.** The Windows simulator still requires separate
startup/shutdown; a one-click Windows wrapper is optional automation work,
not a transport failure. The next robotics gate remains a scene-matched,
sim-time Nav2 `FollowPath` trial with localization and live costmap checked
before a low-speed goal. No moving obstacle-stop, cable-disconnect recovery,
real-time guarantee, or physical base behavior is established here.
