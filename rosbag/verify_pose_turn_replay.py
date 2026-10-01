#!/usr/bin/env python3
"""Assert scan-matched localization covers the recorded Isaac final turn."""

import argparse
from pathlib import Path

from analyze_saved_map_localization import angle_error, read_poses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose_bag", type=Path)
    args = parser.parse_args()

    # Bounds belong to the completed 2026-09-30 ValidationRoute recording.
    turn = [pose for pose in read_poses(args.pose_bag) if 79.35 < pose[0] <= 84.567]
    if len(turn) < 10:
        raise SystemExit(f"FAIL: only {len(turn)} matched poses during the turn")

    latest = turn[-1][0]
    largest_gap = max(next_pose[0] - pose[0] for pose, next_pose in zip(turn, turn[1:]))
    heading_change = angle_error(turn[-1][3], turn[0][3])
    print(
        f"turn_poses={len(turn)} latest_stamp_s={latest:.3f} "
        f"largest_gap_s={largest_gap:.3f} matched_heading_change_rad={heading_change:.3f}"
    )
    if latest < 84.35 or largest_gap > 0.75 or heading_change < 2.4:
        raise SystemExit("FAIL: scan-matched poses do not cover the final rotation")
    print("PASS: scan-matched poses cover the final rotation")


if __name__ == "__main__":
    main()
