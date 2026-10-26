#!/usr/bin/env python3
"""Score LiDAR gap and point-count settings on the static Isaac mapping scene.

Uses the nominal geometry in the 2026-09-30 Jetson scene snapshot and the
recorded odom -> base_link TF. This is static-scene simulation evidence only.
Requires ROS 2 Humble and NumPy; no Isaac runtime or workspace build is needed.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


# Planar intersections at the scene's 0.24 m LiDAR height. The chair seat is
# above that plane; its four legs are separate surfaces at this height.
RECTANGLES = (
    ("left_wall", -2.0, 6.0, 2.95, 3.05),
    ("right_wall", -2.0, 6.0, -3.05, -2.95),
    ("back_wall", -2.05, -1.95, -3.0, 3.0),
    ("front_left", 5.95, 6.05, -3.0, -0.6),
    ("front_right", 5.95, 6.05, 0.6, 3.0),
    ("box_near", 1.65, 2.35, -2.55, -1.85),
    ("box_far", 3.8, 4.6, 1.3, 2.1),
    *((f"chair_{x}_{y}", x - 0.035, x + 0.035, y - 0.035, y + 0.035)
      for x in (3.3, 3.7) for y in (-0.72, -0.28)),
)
BOX_IDS = (6, 7)  # Labels are one-based indices into RECTANGLES.
CHAIR_LEG_IDS = (8, 9, 10, 11)
GAPS_M = (0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50)
MIN_POINTS = (2, 3, 5, 8)


def stamp_s(header):
    """Return a ROS message header time in seconds."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    """Return planar yaw from a normalized ROS quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_recording(path):
    """Read scans and recorded odom -> base_link poses from a rosbag2 directory."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topics = {item.name for item in reader.get_all_topics_and_types()}
    if not {"/scan", "/tf"} <= topics:
        raise ValueError("bag must contain /scan and /tf")
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/scan", "/tf"]))
    scans, poses = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/scan":
            scans.append(deserialize_message(data, LaserScan))
        else:
            for transform in deserialize_message(data, TFMessage).transforms:
                if (transform.header.frame_id.lstrip("/") == "odom" and
                        transform.child_frame_id.lstrip("/") == "base_link"):
                    position = transform.transform.translation
                    poses.append((stamp_s(transform.header), position.x, position.y,
                                  yaw_from_quaternion(transform.transform.rotation)))
    if not scans or len(poses) < 2:
        raise ValueError("bag has no scans or too few odom -> base_link poses")
    poses = np.asarray(sorted(poses), dtype=float)
    poses[:, 3] = np.unwrap(poses[:, 3])
    return scans, poses


def raycast(ox, oy, dx, dy):
    """Return the nearest nominal surface range and one-based surface ID per ray."""
    nearest = np.full(dx.shape, np.inf)
    labels = np.zeros(dx.shape, dtype=np.int8)
    for label, (_, xmin, xmax, ymin, ymax) in enumerate(RECTANGLES, start=1):
        with np.errstate(divide="ignore", invalid="ignore"):
            tx0, tx1 = (xmin - ox) / dx, (xmax - ox) / dx
            ty0, ty1 = (ymin - oy) / dy, (ymax - oy) / dy
        near = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
        far = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
        hit = (far >= np.maximum(near, 0)) & (near > 0) & (near < nearest)
        nearest[hit] = near[hit]
        labels[hit] = label
    return nearest, labels


def prepare_scan(scan, poses, tolerance_m):
    """Project a scan and label returns that agree with the first scene surface.

    Returns (local_x, local_y, valid, labels, is_moving, scene_match_count).
    Pose interpolation is rejected across a TF gap larger than 0.2 s.
    """
    stamp = stamp_s(scan.header)
    index = np.searchsorted(poses[:, 0], stamp)
    if index == 0 or index == len(poses) or poses[index, 0] - poses[index-1, 0] > 0.2:
        return None
    px, py, yaw = (np.interp(stamp, poses[:, 0], poses[:, column])
                   for column in (1, 2, 3))
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    local_x, local_y = ranges * np.cos(angles), ranges * np.sin(angles)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    world_angles = angles + yaw
    expected, nominal_labels = raycast(px + 0.15 * math.cos(yaw),
                                       py + 0.15 * math.sin(yaw),
                                       np.cos(world_angles), np.sin(world_angles))
    matching = valid & (np.abs(ranges - expected) <= tolerance_m)
    labels = np.where(matching, nominal_labels, 0)
    # Route speed is measured from the recorded TF, not inferred from run time.
    before = max(0, index - 6)
    after = min(len(poses) - 1, index + 5)
    dt = poses[after, 0] - poses[before, 0]
    moving = (math.hypot(poses[after, 1] - poses[before, 1],
                         poses[after, 2] - poses[before, 2]) / dt > 0.02 or
              abs(poses[after, 3] - poses[before, 3]) / dt > 0.05)
    return local_x, local_y, valid, labels, moving, int(matching.sum())


def cluster_indices(local_x, local_y, valid, gap_m, min_points, full_circle):
    """Mirror LidarClusterer adjacency, scan breaks, seam join, and size filter."""
    adjacent_distance_sq = np.diff(local_x) ** 2 + np.diff(local_y) ** 2
    breaks = ~valid[:-1] | ~valid[1:] | (adjacent_distance_sq > gap_m ** 2)
    starts = np.flatnonzero(valid & np.r_[True, breaks])
    ends = np.flatnonzero(valid & np.r_[breaks, True]) + 1
    groups = [np.arange(start, end) for start, end in zip(starts, ends)]
    if (len(groups) > 1 and full_circle and valid[0] and valid[-1] and
            (local_x[0] - local_x[-1]) ** 2 +
            (local_y[0] - local_y[-1]) ** 2 <= gap_m ** 2):
        groups[0] = np.r_[groups[-1], groups[0]]
        groups.pop()
    return [group for group in groups if len(group) >= min_points]


def score_one(labels, groups, target_id):
    """Return box capture, split, and merge outcomes for one scan."""
    truth_points = int(np.count_nonzero(labels == target_id))
    expected = int(truth_points >= 3)
    if not expected:
        return (0, 0, 0, 0, 0, 0.0)
    pure_sizes = []
    merged = 0
    for group in groups:
        counts = np.bincount(labels[group], minlength=len(RECTANGLES) + 1)
        if counts[target_id] < 3:
            continue
        # A candidate carrying at least three returns from another known
        # surface is a merge, even if most points still belong to the box.
        if np.any(np.delete(counts[1:], target_id - 1) >= 3):
            merged = 1
        if counts[target_id] / len(group) >= 0.8:
            pure_sizes.append(int(counts[target_id]))
    largest_fraction = max(pure_sizes, default=0) / truth_points
    significant_parts = sum(size >= max(3, 0.1 * truth_points) for size in pure_sizes)
    return (1, int(largest_fraction >= 0.5), int(largest_fraction >= 0.8),
            max(0, significant_parts - 1), merged, largest_fraction)


def score_thin_feature(labels, groups, target_id):
    """Count visible chair-leg features and those retained in a pure candidate."""
    expected = int(np.count_nonzero(labels == target_id) >= 2)
    if not expected:
        return (0, 0)
    retained = any(
        np.count_nonzero(labels[group] == target_id) >= 2 and
        np.count_nonzero(labels[group] == target_id) / len(group) >= 0.8
        for group in groups
    )
    return (1, int(retained))


def evaluate(prepared):
    """Score candidate settings over all scans and the moving-scan subset."""
    results = {}
    for gap in GAPS_M:
        for minimum in MIN_POINTS:
            rows = np.zeros((2, len(BOX_IDS), 6), dtype=float)
            thin_features = np.zeros((2, len(CHAIR_LEG_IDS), 2), dtype=int)
            total_clusters = 0
            unsupported_clusters = 0
            for x, y, valid, labels, moving, _, full_circle in prepared:
                groups = cluster_indices(x, y, valid, gap, minimum, full_circle)
                total_clusters += len(groups)
                unsupported_clusters += sum(np.count_nonzero(labels[group]) == 0
                                            for group in groups)
                for box_index, target in enumerate(BOX_IDS):
                    outcome = score_one(labels, groups, target)
                    rows[0, box_index] += outcome
                    if moving:
                        rows[1, box_index] += outcome
                for feature_index, target in enumerate(CHAIR_LEG_IDS):
                    outcome = score_thin_feature(labels, groups, target)
                    thin_features[0, feature_index] += outcome
                    if moving:
                        thin_features[1, feature_index] += outcome
            results[f"{gap:.2f}/{minimum}"] = {
                "all": rows[0].tolist(), "moving": rows[1].tolist(),
                "thin_features_all": thin_features[0].sum(axis=0).tolist(),
                "thin_features_moving": thin_features[1].sum(axis=0).tolist(),
                "clusters_per_scan": total_clusters / len(prepared),
                "unsupported_clusters_per_scan": unsupported_clusters / len(prepared),
            }
    return results


def main():
    """Read a bag, score the fixed candidate grid, and print JSON results."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--label-tolerance-m", type=float, default=0.12)
    args = parser.parse_args()
    if args.label_tolerance_m <= 0:
        parser.error("--label-tolerance-m must be positive")
    scans, poses = load_recording(args.bag)
    prepared = []
    for scan in scans:
        item = prepare_scan(scan, poses, args.label_tolerance_m)
        if item is None:
            continue
        beam_width = abs(scan.angle_increment)
        full_circle = abs(scan.angle_max - scan.angle_min) + beam_width >= (
            2 * math.pi - beam_width * 0.51)
        prepared.append((*item, full_circle))
    print(json.dumps({
        "source_scans": len(scans), "scored_scans": len(prepared),
        "moving_scans": int(sum(item[4] for item in prepared)),
        "matched_return_fraction": sum(item[5] for item in prepared) / sum(
            int(item[2].sum()) for item in prepared),
        "label_tolerance_m": args.label_tolerance_m,
        "box_ids": {str(index): RECTANGLES[index-1][0] for index in BOX_IDS},
        "metric_order": ["expected", "capture_50pct", "capture_80pct",
                         "extra_significant_fragments", "merged_frames",
                         "largest_fraction_sum"],
        "candidates": evaluate(prepared),
    }, indent=2))


if __name__ == "__main__":
    main()
