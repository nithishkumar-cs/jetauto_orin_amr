# localization_odometry_fusion

This package launches the maintained `robot_localization` EKF for local
odometry. It does not implement another filter. Its input and output contract
is:

```text
/odom/wheel (+ /imu/data on hardware) -> ekf_node -> /odometry/filtered
```

The Isaac profile uses wheel odometry alone because the current Isaac scene
does not publish an IMU. It fuses Isaac pose and planar velocity so its output
agrees with Isaac's `odom -> base_link` transform, which the EKF does not
publish. The hardware profile fuses wheel planar velocity and
IMU yaw rate, and the EKF owns `odom -> base_link`. The hardware driver must
not publish the same transform. Both profiles allow lateral velocity because
JetAuto has a mecanum base.

Run independently:

```bash
ros2 launch localization_odometry_fusion odometry_fusion.launch.py backend:=isaac
```

`robot_bringup` starts this launch when localization is enabled. The `rosbag`
profile follows the Isaac TF contract: the bag must include `odom -> base_link`
and `/clock`. ROS bag playback is started separately.

The Isaac validation bag contains zero covariance on `/odom/wheel`. The filter
can run on it, but the uncertainty output cannot be trusted until Isaac and
hardware publishers provide measured, nonzero covariance. The current Isaac
bag also cannot establish whether motion came from `/cmd_vel/safety_limited`:
that command topic was not recorded. Verify the command-to-motion path in a
future live laptop run before testing Nav2 control.
