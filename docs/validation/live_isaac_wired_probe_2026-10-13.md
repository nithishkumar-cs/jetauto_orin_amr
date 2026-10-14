# Live Isaac / direct-Ethernet probe — 2026-10-13

## Scope

Jetson validation while Isaac Sim ran on the Windows laptop. No controller,
motor driver, Nav2 motion control, or physical base was started. A brief
synthetic command was sent only through the software safety chain after ROS
confirmed there was no driver or Isaac command subscriber. Isaac was left
running. The wired transport, single-source, perception, and live-scan safety
checks passed after the initial configuration issues below were resolved.

## Tests completed

- The laptop reported Ethernet `169.254.71.211/16`; the Jetson test profile
  assigned `169.254.71.212/16` to `eno1`. Windows `Test-NetConnection
  169.254.71.212 -Port 22` succeeded with `InterfaceAlias: Ethernet` and
  `SourceAddress: 169.254.71.211`. The link negotiated 1 Gb/s full duplex.
- ROS domain 42 discovery showed `/clock`, `/scan`, `/tf`, `/tf_static`,
  `/odom/wheel`, and RGB, aligned-depth, and camera-info topics. An unrestricted
  subscriber received 640×480 RGB8 and 32FC1 depth, 720-point scans, wheel
  odometry, and the expected static TF edges.
- After restoring `jetauto-direct-test`, a test-only Fast DDS UDPv4 profile
  whitelisted **only** the Jetson's `169.254.71.212` interface and disabled
  built-in transports. On that profile, the Jetson received 10 each of RGB,
  depth, and scans in about 1.5 wall seconds; 61 clocks, 57 odometry and TF
  messages, and the expected static TF edges were also received. RGB and
  depth were both 640×480 with matching timestamps in the sampled data.
  This established a working wired ROS data path.
- A separate 20-second Ethernet-only freshness probe received 744 `/clock`,
  124 scans, 123 RGB, 124 depth, and 740 odometry messages. Despite sustained
  delivery, the latest RGB/depth and clock stamps differed by about 110.7 s
  at multiple sample points, and the reported stamp progress alternated
  forward/backward. Thus both conflicting timelines are present on the cable,
  not merely discovered through another network interface.
- Local verification passed: all 11 selected AMR `colcon test` packages had
  zero failures, 19 rosbag/replay pytest cases passed, both offline safety
  integration harnesses passed, and the device-backed detector checks passed
  (three CUDA preprocessing cases plus the real TensorRT/ROS node test).
- After the extra Isaac publisher was stopped, `/clock` had exactly one
  publisher. In a 60-second **wall-time**, Ethernet-only probe, the Jetson
  received 1,921 clocks, 319 scans, 319 RGB images, 318 depth images, 1,916
  odometry messages, and 1,916 dynamic TF messages. All seven sampled streams
  had zero backward timestamp jumps. Maximum wall arrival gaps were 0.254 s
  for RGB, 0.253 s for depth, 0.251 s for scans, and 0.048 s for odometry.
  Maximum simulation-time gaps were 0.117 s for RGB/depth and 0.100 s for
  scans. The clock advanced only 32.0 simulation seconds in 60 wall seconds
  (about 0.53× real time); this run does **not** establish real-time operation.
  `eno1` retained its static test address and NetworkManager logged no new
  carrier loss during this interval.
- The six-node perception-only bringup ran against the single wired Isaac
  source. Over 20 wall seconds, the observer received 99 nonempty 2D
  detections, 99 nonempty 3D detections, 100 nonempty LiDAR cluster arrays,
  99 camera-track arrays, 100 LiDAR-track arrays, and 99 fused arrays. The
  pedestrian was classified in all 99 camera and fused samples, with one
  stable person track ID in each stream. All six Jetson nodes exited cleanly.
- Safety-only bringup started collision monitor, lifecycle manager,
  scan-health gate, and e-stop gate; the monitor reached `active [3]`. ROS
  reported only collision monitor subscribed to `/cmd_vel` and no subscriber
  to `/cmd_vel/safety_limited` before the observer joined. With live Isaac
  scans, a synthetic 0.2 m/s command passed the software chain; e-stop
  latched and blocked a subsequent command; reset restored pass-through;
  the observer sent a final zero command. All four Jetson safety processes
  exited cleanly.

## Initial failures resolved during this session

1. `ros2 topic info -v` showed **two publishers each** for `/clock`, `/scan`,
   and RGB, with duplicate Isaac node names but distinct endpoint GIDs. A
   20-second freshness observer saw message timestamps alternate between
   simulation timelines roughly 111 seconds apart. For example, RGB could
   be ~110.7 simulation seconds behind the latest `/clock`; odometry and
   `/clock` also jumped between timelines. One duplicate Isaac process was
   stopped before any live perception or safety result was accepted.
2. NetworkManager logged repeated `eno1` carrier changes earlier in the
   session. After a further link drop at 20:14 local time, the test-only
   static-IP profile was replaced by the existing auto-connecting DHCP
   profile. `eno1` retained a physical 1 Gb/s link but no IPv4 address.
   An initial Ethernet-only Fast DDS subscriber reported `All whitelist
   interfaces were filtered out` and received zero messages because the
   required local address had disappeared. Re-activating the existing static
   test profile fixed that test. The link must still be checked for future
   flaps during a longer run. No further flap appeared during the 60-second
   wired-only observer run.

## Remaining limits / next gate

- The scene's LiDAR tracker emitted 162 distinct track IDs in 100 sampled
  arrays, with 18–31 objects per array. The pedestrian fusion ID was stable,
  but background clutter/track churn should be reviewed before claiming
  robust multi-object tracking.
- The 0.53× simulation rate and previously observed cable carrier flaps need
  longer performance/stability checks. The static `jetauto-direct-test`
  profile does not auto-reconnect after a carrier drop; the pre-existing DHCP
  profile may take over again.
- This was a no-motor software test. Controlled Nav2 motion, Isaac command
  reception, physical braking distance, real sensor/driver failures, and
  safety-rated stopping were **not** validated. No driver or controller was
  enabled, and no map/localization run was repeated in this stationary scene.
