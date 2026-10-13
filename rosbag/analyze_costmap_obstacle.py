#!/usr/bin/env python3
"""Check Nav2 OccupancyGrid snapshots for a synthetic mark/clear transition."""

import argparse
import json
import math
from pathlib import Path

try:
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this tool.") from error


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def lethal_near(grid, x_m, y_m, radius_m):
    origin = grid.info.origin
    orientation = origin.orientation
    yaw = math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y**2 + orientation.z**2),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx, dy = x_m - origin.position.x, y_m - origin.position.y
    grid_x = (cosine * dx + sine * dy) / grid.info.resolution
    grid_y = (-sine * dx + cosine * dy) / grid.info.resolution
    reach = math.ceil(radius_m / grid.info.resolution) + 1
    result = 0
    for row in range(max(0, math.floor(grid_y) - reach),
                     min(grid.info.height, math.floor(grid_y) + reach + 1)):
        for column in range(max(0, math.floor(grid_x) - reach),
                            min(grid.info.width, math.floor(grid_x) + reach + 1)):
            wx = origin.position.x + cosine * (column + 0.5) * grid.info.resolution \
                - sine * (row + 0.5) * grid.info.resolution
            wy = origin.position.y + sine * (column + 0.5) * grid.info.resolution \
                + cosine * (row + 0.5) * grid.info.resolution
            if math.hypot(wx - x_m, wy - y_m) <= radius_m and \
                    grid.data[row * grid.info.width + column] == 100:
                result += 1
    return result


def analyze(path, x_m, y_m, radius_m, start_s, end_s):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    snapshots = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/global_costmap/costmap":
            continue
        grid = deserialize_message(data, OccupancyGrid)
        snapshots.append((stamp_seconds(grid.header.stamp), lethal_near(grid, x_m, y_m, radius_m)))
    if not snapshots:
        raise ValueError("No /global_costmap/costmap snapshots in bag")
    before = [(t, n) for t, n in snapshots if t < start_s]
    exposure = [(t, n) for t, n in snapshots if start_s <= t < end_s + 2]
    after = [(t, n) for t, n in snapshots if t >= end_s + 4]
    marked = [(t, n) for t, n in exposure if n > 0]
    cleared = [(t, n) for t, n in snapshots if t > marked[-1][0] and n == 0] if marked else []
    return {
        "costmap_snapshots": len(snapshots),
        "before_peak_lethal_cells": max((n for _, n in before), default=None),
        "exposure_peak_lethal_cells": max((n for _, n in exposure), default=None),
        "after_peak_lethal_cells": max((n for _, n in after), default=None),
        "first_mark_sim_s": marked[0][0] if marked else None,
        "first_clear_sim_s": cleared[0][0] if cleared else None,
        "last_lethal_cells": snapshots[-1][1],
        "samples": [{"sim_s": round(t, 3), "lethal_cells": n} for t, n in snapshots],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--target-x", type=float, default=2.0)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--radius-m", type=float, default=0.15)
    parser.add_argument("--start-s", type=float, default=18.0)
    parser.add_argument("--end-s", type=float, default=22.0)
    parser.add_argument("--assert-mark-clear", action="store_true")
    args = parser.parse_args()
    result = analyze(
        args.bag, args.target_x, args.target_y, args.radius_m, args.start_s, args.end_s
    )
    print(json.dumps(result, indent=2))
    if args.assert_mark_clear and not (
        result["before_peak_lethal_cells"] == 0
        and (result["exposure_peak_lethal_cells"] or 0) > 0
        and result["after_peak_lethal_cells"] == 0
        and result["last_lethal_cells"] == 0
    ):
        raise SystemExit("Mark/clear assertion failed")


if __name__ == "__main__":
    main()
