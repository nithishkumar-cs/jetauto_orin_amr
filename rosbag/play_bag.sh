#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 /path/to/rosbag [ros2 bag play args...]" >&2
  exit 2
fi

ros2 bag play "$@"
