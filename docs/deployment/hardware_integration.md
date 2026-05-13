# Hardware Integration Notes

Target hardware stack:

- Jetson Orin Nano Developer Kit
- JetAuto Standard Kit base electronics
- JetAuto depth camera
- LiDAR
- 7-inch display

Immediate bring-up order:

1. Boot Jetson from NVMe and confirm cooling under load.
2. Mount Jetson and document power routing.
3. Bring up the camera and LiDAR vendor drivers.
4. Run `robot_stack.launch.py instrumentation_mode:=debug`.
5. Capture a short bag for detector/preproc replay.
6. Replace placeholder TF values with measured transforms.
7. Run `robot_stack.launch.py instrumentation_mode:=profile` once the depth path is wired.

Mechanical and electrical assumptions should be written here as the robot is assembled. This document is part of the project, not after-the-fact polish.
