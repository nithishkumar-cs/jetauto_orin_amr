# Jetson Setup

Target platform:

- Jetson Orin Nano Developer Kit
- JetPack 6.x
- ROS 2 Humble
- CUDA and TensorRT from JetPack

Initial setup:

This project uses native Jetson installation for the first hardware bring-up. Docker support is a later reproducibility path, not the default camera/robot workflow.

```bash
cd ~/projects/robotics/jetauto_orin_amr
./scripts/setup/install_jetson_dependencies.sh
source /opt/ros/humble/setup.bash
./scripts/dev_helpers/build_release.sh
source install/setup.bash
```

Verify the USB camera:

```bash
v4l2-ctl --list-devices
ros2 launch robot_bringup webcam_perception.launch.py
```

The webcam launch uses the debug detector by default. Switch to the YOLO TensorRT config once an engine exists:

```bash
ros2 launch robot_bringup webcam_perception.launch.py \
  detector_params:=$(ros2 pkg prefix perception_inference)/share/perception_inference/config/yolo_detector.yaml
```

Model engines should live under `/opt/jetauto_orin_amr/models`, not in git.

## Simulation Boundary

Simulation does not run on the Jetson as part of this repository. The Jetson
stack supports live hardware and rosbag replay inputs. Any future simulator
should live outside this repo and publish compatible topics or recorded bags.
