# Jetson Setup

Target platform:

- Jetson Orin Nano Developer Kit
- JetPack 6.x
- ROS 2 Humble
- CUDA and TensorRT from JetPack

Initial setup:

This project uses native Jetson installation for the first hardware bring-up. Docker support is a later reproducibility path, not the default hardware workflow.

```bash
cd ~/projects/robotics/jetauto_orin_amr
./scripts/setup/install_jetson_dependencies.sh
source /opt/ros/humble/setup.bash
./scripts/dev_helpers/build_mode.sh production
source install/setup.bash
```

The apt SLAM Toolbox 2.6.10 filters out in-place turns before scan matching.
For saved-map localization, build the pinned heading-aware overlay as well:

```bash
bash scripts/setup/build_slam_toolbox_heading_fix.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix slam_toolbox  # must print this repository's install path
```

This leaves `/opt/ros/humble` untouched. The patch and validation are recorded
in `docs/validation/isaac_heading_fix_2026-10-01.md`.

Run the debug bring-up:

```bash
ros2 launch robot_bringup robot_stack.launch.py instrumentation_mode:=debug
```

Debug mode uses the debug detector by default. Switch to the YOLO TensorRT config once an engine exists:

```bash
ros2 launch robot_bringup robot_stack.launch.py \
  instrumentation_mode:=debug \
  detector_params:=$(ros2 pkg prefix perception_inference)/share/perception_inference/config/yolo_detector.yaml
```

Model engines should live under `/opt/jetauto_orin_amr/models`, not in git.

## Simulation Boundary

Simulation does not run on the Jetson as part of this repository. The Jetson
stack supports live hardware and rosbag replay inputs. Any future simulator
should live outside this repo and publish compatible topics or recorded bags.
