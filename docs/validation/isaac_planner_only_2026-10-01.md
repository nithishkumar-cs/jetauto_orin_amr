# Isaac saved-map planner-only validation — 2026-10-01

This is the first Nav2 milestone after saved-map localization. It tests global
planning against the existing recorded Isaac scan/TF stream, not autonomous
motion. No Nav2 controller, BT navigator, base driver, or teleop node ran.

## Setup

Jetson ROS Humble, isolated `ROS_DOMAIN_ID=82`. Saved SLAM Toolbox pose graph:
`/home/nithish/jetauto_isaac_maps/loop_20260928`. Source bag:
`/home/nithish/jetauto_live_validation_20260930/localization_bag_retry`.
The source bag's recorded `map -> odom` transforms were removed with a zero-drift
copy so live SLAM Toolbox alone owned that TF edge:

```bash
source /opt/ros/humble/setup.bash
python3 rosbag/inject_odometry_tf_drift.py \
  /home/nithish/jetauto_live_validation_20260930/localization_bag_retry \
  /tmp/jetauto_planner_only_replay_20261001 \
  --x-m 0 --y-m 0 --yaw-rad 0 --ramp-duration-s 1
```

Start the stack from this repository's root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=82
ros2 launch robot_bringup robot_stack.launch.py \
  instrumentation_mode:=profile backend:=rosbag \
  slam_mode:=localization navigation_mode:=planner_only \
  pose_graph_prefix:=/home/nithish/jetauto_isaac_maps/loop_20260928 \
  map_start_pose:='[-0.5, -1.0, 0.0]' \
  enable_drivers:=false enable_detector:=false \
  enable_geometry:=false enable_lidar_clustering:=false \
  enable_tracking:=false enable_fusion:=false enable_safety:=false \
  enable_diagnostics:=false enable_benchmarks:=false \
  enable_calibration_validator:=false enable_localization:=false \
  enable_navigation:=true enable_teleop:=false
```

In another terminal with the same ROS domain, play the filtered bag:

```bash
ros2 bag play /tmp/jetauto_planner_only_replay_20261001
```

The bag **already contains `/clock`**. Do not add `--clock`: an initial replay
with that option produced competing simulated-time streams and repeated TF
"jump back in time" warnings. Restarting both the stack and player without it
removed that harness error.

## Results

- `ros2 lifecycle get /planner_server`: `active [3]`.
- The graph contained `slam_toolbox`, `planner_server`, its lifecycle manager,
  and `global_costmap`; it had `/plan` and `/global_costmap/costmap`, but no
  `/cmd_vel` topic or motion-controller node.
- `python3 rosbag/probe_planner_only.py --start -0.5 -1.0 --goal 2.5 0.0
  --expect success --inspect-costmap`: action status 4 (succeeded), 126 map-frame
  poses, 3.181 m path. The 167×126 global costmap at 0.05 m resolution had
  2,264 lethal and 436 unknown cells in the retrieved snapshot. A second short
  replay with the final probe also checked all 126 path poses against the live
  costmap: 3.183 m path, maximum sampled cost 0, and no pose in a lethal or
  unknown cell. Snapshot cell counts varied slightly with replay time.
- `python3 rosbag/probe_planner_only.py --start -0.5 -1.0 --goal -2.0 0.0
  --expect failure`: action status 6 (aborted), zero path poses; this goal lies
  in the saved map's wall.

Both action requests used an explicit map-frame start (`use_start=true`), so
this validates planner/costmap behavior independently of the robot's pose at
the exact request time. It does not prove autonomous control, live obstacle
avoidance during motion, or hardware-safe footprint clearance. The distinct-
route final-turn localization dropout was diagnosed separately in
`docs/validation/isaac_pose_dropout_2026-10-01.md`. Direct loop-closure
evidence remains open; this planner-only milestone did not investigate it.
