#!/usr/bin/env python3
"""Compare mapping /pose with Isaac's exact odometry in the initial map frame."""

import argparse
import json
import math
from pathlib import Path

from analyze_saved_map_localization import (
    angle_error,
    metric,
    nearest,
    read_odometry,
    read_poses,
)


def analyze(reference_bag, pose_bag, max_skew_s, end_s, sample_times):
    if max_skew_s <= 0 or not math.isfinite(max_skew_s):
        raise ValueError("Timestamp skew limit must be finite and positive")
    if end_s is not None and (end_s <= 0 or not math.isfinite(end_s)):
        raise ValueError("End time must be finite and positive")

    odometry = read_odometry(reference_bag)
    poses = read_poses(pose_bag)
    odom_stamps = [sample[0] for sample in odometry]
    matched = []
    for stamp, x, y, heading in poses:
        if end_s is not None and stamp > end_s:
            continue
        truth = nearest(odometry, odom_stamps, stamp)
        skew = abs(truth[0] - stamp)
        if skew > max_skew_s:
            continue
        matched.append(
            {
                "stamp_s": stamp,
                "position_error_m": math.hypot(x - truth[1], y - truth[2]),
                "heading_error_rad": angle_error(heading, truth[3]),
                "stamp_skew_s": skew,
            }
        )

    if not matched:
        raise ValueError("No mapping poses match reference odometry")
    samples = []
    for target in sample_times:
        if not math.isfinite(target):
            raise ValueError("Sample times must be finite")
        closest = min(matched, key=lambda row: abs(row["stamp_s"] - target))
        samples.append(
            {
                "requested_s": target,
                "pose_stamp_s": round(closest["stamp_s"], 4),
                "position_error_m": round(closest["position_error_m"], 4),
                "heading_error_rad": round(closest["heading_error_rad"], 4),
            }
        )
    return {
        "pose_messages": len(poses),
        "matched_poses": len(matched),
        "position_error_m": metric([row["position_error_m"] for row in matched]),
        "heading_error_rad": metric([row["heading_error_rad"] for row in matched]),
        "pose_odom_stamp_skew_s": metric([row["stamp_skew_s"] for row in matched]),
        "samples": samples,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_bag", type=Path)
    parser.add_argument("pose_bag", type=Path)
    parser.add_argument("--max-skew-s", type=float, default=0.02)
    parser.add_argument("--end-s", type=float)
    parser.add_argument("--sample-time-s", type=float, action="append", default=[])
    args = parser.parse_args()
    print(
        json.dumps(
            analyze(
                args.reference_bag,
                args.pose_bag,
                args.max_skew_s,
                args.end_s,
                args.sample_time_s,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
