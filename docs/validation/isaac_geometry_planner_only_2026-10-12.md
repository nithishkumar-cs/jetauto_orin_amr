# Nav2 planner-only check on the distinct Isaac geometry — 2026-10-12

## Outcome

The existing `robot_bringup` **planner-only** mode planned five paths on the
new pillar-cluster map, including a path that detoured around the cluster.
Four requests followed successive portions of the separately recorded
interior route; a fifth crossed the cluster's blocked direct line. Every
sampled pose of all five returned paths was in a non-lethal, known cell of
the *live Nav2 global costmap*. A goal inside the cluster and one outside the
map were both rejected. Live saved-map localization remained bounded through
the full input bag. No controller, BT navigator, driver, teleop, safety-chain
node, or `/cmd_vel` topic was present. This is path planning, **not motion**.

## Setup

The saved 1.0 m-test-radius pose graph from the
[distinct-geometry comparison](isaac_distinct_geometry_slam_2026-10-12.md)
was loaded from `/tmp/jetauto_geometry_radius1_20261012/final_map`.
The raw interior-route bag was filtered to
`/tmp/jetauto_geometry_localization_clean_20261012`: `/clock`, `/scan`,
`/tf`, `/tf_static`, and unchanged odometry as
`/validation/reference_odom`; it contains no previous `/pose`, `/map`, or
`map -> odom` output. Bag playback used 0.5× speed and an isolated ROS domain
109. The source bag already contains `/clock`, so playback did **not** add a
second clock.

`robot_bringup` ran with `backend:=rosbag`, `slam_mode:=localization`,
`navigation_mode:=planner_only`, `enable_navigation:=true`, and every
motion-related toggle off. The global costmap used the repository's
`configs/navigation/planner_only.yaml`: 0.33 m provisional robot radius,
static map layer, live `/scan` obstacle layer, and 0.40 m inflation radius.
The lifecycle manager brought `/planner_server` to `active [3]` **after** bag
playback supplied TF and the live localizer supplied `map -> odom`. A first
harness attempt waited for activation *before* playback and could not pass
the costmap's TF wait. ROS CLI readiness checks also needed `--no-daemon` in
this sandbox. Both attempts were stopped; the final run used the correct
ordering and daemon-free checks.

## Path results

All requests used `rosbag/probe_planner_only.py` with explicit map-frame
starts (`use_start=true`). Starts and goals were sampled from the interior
route's exact odometry, whose coordinates aligned with the saved map within
the localization error measured below. The probe checked every returned
path pose against a fresh `/global_costmap/get_costmap` response and failed
if any pose was in a lethal or unknown cell.

| Start → goal (map m) | Result | Path poses | Length | Highest sampled cost |
| --- | --- | ---: | ---: | ---: |
| `(-0.45,-0.06) → (-0.50,1.14)` | Success | 47 | 1.198 m | 0 |
| `(-0.50,1.14) → (2.27,1.20)` | Success | 111 | 2.815 m | 0 |
| `(2.27,1.20) → (2.30,-1.26)` | Success | 97 | 2.475 m | 0 |
| `(2.30,-1.26) → (-0.47,-1.00)` | Success | 110 | 2.778 m | 0 |
| `(0,0) → (2,0)`, across pillars | Success, detour | 115 | **2.904 m** | 212 |
| `(0,0) → (1.20,0)`, inside cluster | Aborted, no path | 0 | — | — |
| `(0,0) → (-2.30,0)`, off map | Aborted, no path | 0 | — | — |

The straight `(0,0) → (2,0)` line is 2.0 m and passed through the cluster:
78 of 101 evenly spaced samples had published occupancy cost at least 99
on a later recorded costmap snapshot. Nav2 instead returned the 2.904 m
route. Its sampled maximum cost 212 was below inscribed/lethal values, and
none of its poses was unknown. The other four routes sampled cost 0. This
checks discrete path poses against a costmap snapshot, **not** continuous
robot-footprint collision clearance or time-varying obstacle response.

The final output bag contains five `/plan` messages, 46 global-costmap
snapshots, and 250 fresh `/pose` messages. No `/cmd_vel` topic appeared in
the live topic list. The node list contained `/slam_toolbox`,
`/planner_server`, `/lifecycle_manager_planner`, and
`/global_costmap/global_costmap`, but no controller or motor driver.

## Localization and caveats

All 250 fresh localization poses matched Isaac odometry within 0.002 s.
Position error was **0.0453 / 0.1661 / 0.1868 m** and heading error was
**0.0121 / 0.0313 / 0.0362 rad** (median / p95 / maximum). Poses ran from
0.25 to 53.25 s; the last reference-odometry stamp was 53.73 s, and the
largest gap between fresh poses was 1.8 s. These values are similar to the
separate localizer-only replay and show no obvious planner-induced loss of
localization on this bag.

The costmap's TF message filter emitted **four throttled drop notices**
in this final replay (which had 534 source scans). The notices do **not**
establish that exactly four scans were dropped; the actual count is unknown.
The timing diagnosis and a temporary TF-wait experiment are in the
[follow-up report](isaac_planner_tf_filter_2026-10-12.md). The costmap's static
layer resized between 167×127 and 167×128 cells several times while SLAM Toolbox updated
`/map`. Planning succeeded, but these effects merit investigation before
controlled driving. The map, route, and odometry are simulated; the odometry
has no realistic slip, and the 0.33 m radius is an Isaac-only provisional
footprint. Explicit starts also mean these action calls do not exercise
Nav2's choice of a live robot start pose. No local planner, controller,
velocity command, collision-monitor response, or e-stop path was tested.

Logs, node/topic snapshots, individual action responses, and the output bag
are at `/tmp/jetauto_geometry_planner_only_pillar_20261012/`. The temporary
runner `/tmp/jetauto_geometry_planner_only_20261012.sh` records the exact
bringup toggles, replay rate, starts/goals, and lifecycle ordering. These
`/tmp` artifacts may disappear after reboot. The source bags, saved map,
production parameters, and Isaac project were not changed.

To reproduce from separate terminals after sourcing ROS Humble and this
workspace overlay, use the planner-only launch command in the
[robot_bringup README](../../src/orchestration/robot_bringup/README.md)
with `pose_graph_prefix:=/tmp/jetauto_geometry_radius1_20261012/final_map`;
start playback of the clean bag at 0.5× before waiting for planner activation.
Then, for example:

```bash
ros2 bag play /tmp/jetauto_geometry_localization_clean_20261012 \
  --rate 0.5 --disable-keyboard-controls
python3 rosbag/probe_planner_only.py --start 0 0 --goal 2 0 \
  --expect success --inspect-costmap
```

The next distinct milestone is controlled motion in Isaac with the actual
Nav2 command-to-simulator path and safety chain connected. This report does
not authorize hardware driving or changing the production SLAM radius.
The [1× live-start follow-up](isaac_live_start_planner_tf_2026-10-13.md)
found that the current costmap TF wait discarded most replayed scans;
address that timing gap before controlled motion.
