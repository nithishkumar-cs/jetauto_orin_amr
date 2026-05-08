#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
profile="${JETAUTO_CHECK_PROFILE:-host}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

host_packages=(
  amr_interfaces
  base_interface
  benchmarks
  evaluation_tools
  localization_mapping
  logging_and_diagnostics
  navigation_tasks
  perception_geometry
  safety_layer
  sensor_drivers
  sensor_fusion
  teleop_tools
  tf_and_calibration
  tracking_fusion
)

build_args=(--merge-install --symlink-install)
test_args=(--merge-install)

case "${profile}" in
  host)
    build_args+=(--packages-select "${host_packages[@]}")
    test_args+=(--packages-select "${host_packages[@]}")
    ;;
  full)
    ;;
  *)
    echo "Unknown JETAUTO_CHECK_PROFILE='${profile}'. Use 'host' or 'full'." >&2
    exit 2
    ;;
esac

colcon list --base-paths src
colcon build "${build_args[@]}"
colcon test "${test_args[@]}" --event-handlers console_cohesion+
colcon test-result --verbose
