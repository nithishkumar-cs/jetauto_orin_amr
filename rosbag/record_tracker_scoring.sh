#!/usr/bin/env bash
# Record the Jetson-side bag that rosbag/score_trackers.py scores against Isaac
# ground truth. Run with Isaac (-GroundTruth) and the perception stack already
# publishing; the preflight refuses to start while any topic has no publisher,
# so a missing stream is caught before the run instead of after it.
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 OUTPUT_BAG_DIR [extra ros2 bag record args or topics...]" >&2
  exit 2
fi
output="$1"
shift
if [[ -e "$output" ]]; then
  echo "refusing to overwrite existing $output" >&2
  exit 2
fi
if ! command -v ros2 >/dev/null; then
  echo "source /opt/ros/humble/setup.bash and install/setup.bash first" >&2
  exit 2
fi

# Detections and clusters are the tracker inputs: recording them allows offline
# re-tracking with different thresholds against the same ground truth.
topics=(
  /clock /tf /tf_static /scan
  /ground_truth/obstacles
  /perception/detections_3d /perception/lidar/clusters
  /perception/camera/tracks /perception/lidar/tracks /perception/fused_obstacles
)

published="$(ros2 topic list -v | awk '/^Published topics:/ {p = 1; next} /^Subscribed topics:/ {p = 0} p && $1 == "*" {print $2}')"
missing=()
for topic in "${topics[@]}"; do
  grep -qxF -- "$topic" <<<"$published" || missing+=("$topic")
done
if ((${#missing[@]})); then
  echo "no publisher for: ${missing[*]}" >&2
  exit 1
fi

exec ros2 bag record -o "$output" "$@" "${topics[@]}"
