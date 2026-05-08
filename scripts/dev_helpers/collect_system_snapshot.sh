#!/usr/bin/env bash
set -euo pipefail

out="${1:-system_snapshot.txt}"
{
  date --iso-8601=seconds
  uname -a
  echo
  echo "CUDA:"
  nvcc --version || true
  echo
  echo "Jetson:"
  dpkg-query -W 'nvidia-l4t-core' 2>/dev/null || true
  echo
  echo "ROS:"
  printenv ROS_DISTRO || true
  ros2 --version || true
  echo
  echo "Cameras:"
  v4l2-ctl --list-devices || true
  echo
  echo "Disk:"
  df -h
  echo
  echo "Memory:"
  free -h
} > "${out}"

echo "Wrote ${out}"

