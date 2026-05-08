# Hardware Integration Notes

Current known hardware:

- Jetson Orin Nano Developer Kit
- USB webcam
- JetAuto Standard Kit expected later with depth camera, LiDAR, base electronics, and display

Immediate bring-up order:

1. Boot Jetson from NVMe and confirm cooling under load.
2. Verify USB camera bandwidth with `v4l2-ctl`.
3. Run `webcam_perception.launch.py`.
4. Capture a short bag for detector/preproc replay.
5. Mount Jetson and document power routing.
6. Bring up depth camera and LiDAR as vendor drivers.
7. Replace placeholder TF values with measured transforms.
8. Enable `robot_depth_dev.launch.py`.

Mechanical and electrical assumptions should be written here as the robot is assembled. This document is part of the project, not after-the-fact polish.

