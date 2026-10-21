# JetAuto Orin AMR

Production-style indoor AMR perception, mapping, fusion, navigation, and safety
stack for a Hiwonder JetAuto Standard Kit / Without Controller with a Jetson
Orin Nano Developer Kit.

The current project authority is
[`jetauto_orin_amr_project_spec.md`](jetauto_orin_amr_project_spec.md). Treat it
as the source of truth for structure and development order.

Measured milestones, limitations, and evidence-backed résumé drafts are in
the [engineering log](docs/engineering_log.md).

## Workspace Layout

```text
jetauto_orin_amr/
  configs/                  Project-level robot, sensor, model, calibration, and mode config
  docker/                   Jetson and development container assets
  docs/                     Architecture, calibration, deployment, benchmark, and failure docs
  scripts/                  Setup, flashing, networking, and development helpers
  bags/                     Local rosbag workspace, not for large committed data
  rosbag/                   Playback helpers kept outside robot bring-up
  datasets/                 Local dataset workspace, not for large committed data
  src/
    orchestration/robot_bringup/  System orchestration entrypoint
    amr_interfaces/         Shared message contracts (TrackedObstacle, TrackedObstacleArray)
    perception/             Detector, depth projection, LiDAR clustering, tracking, fusion, metrics
    localization/           Odometry fusion, SLAM mapping/localization (upstream SLAM Toolbox)
    navigation/             Placeholder for custom tasks; Nav2 is upstream
    safety/                 Collision avoidance chain and operator stop (estop_gate, scan_health_gate, stop_zone_recovery); not safety-rated
    drivers/ bridges/       Empty until hardware drivers and sim/replay adapters are rebuilt
    tools/                  Diagnostics, benchmarks, evaluation, bag and calibration tools
  tests/                    Cross-package system, integration, replay, and performance tests
```

## First Bring-Up

```bash
cd ~/projects/robotics/jetauto_orin_amr
./scripts/setup/install_jetson_dependencies.sh
source /opt/ros/humble/setup.bash
./scripts/dev_helpers/build_mode.sh production
source install/setup.bash
ros2 launch robot_bringup robot_stack.launch.py instrumentation_mode:=debug
```

Simulator packages and assets do not live in this Jetson robot repository. If
simulation is used later, it should run from a separate workstation project and
publish compatible ROS topics or bags into this stack.

## Development Flow

Build the project in the order defined by the spec:

1. System design
2. Repository skeleton and development environment
3. Platform bring-up
4. TF and calibration foundation
5. Replay support
6. CUDA foundation and preprocessing
7. Inference core and detector backends
8. Geometry, fusion, tracking, localization, safety, navigation, tools, tests
