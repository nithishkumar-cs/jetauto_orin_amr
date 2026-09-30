#!/usr/bin/env python3
"""Compare SLAM poses with Isaac odometry in a fixed saved-map frame."""

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this analyzer.") from error


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def bag_messages(bag_path, wanted):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
        if topic.name in wanted
    }
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name in types:
            yield name, deserialize_message(data, types[name])


def metric(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(values),
        "median": round(statistics.median(values), 4),
        "p95": round(ordered[math.ceil(0.95 * len(ordered)) - 1], 4),
        "max": round(ordered[-1], 4),
    }


def nearest(samples, stamps, stamp):
    index = bisect.bisect_left(stamps, stamp)
    return min(samples[max(0, index - 1) : index + 1], key=lambda sample: abs(sample[0] - stamp))


def read_map_alignment(mapping_bag):
    alignment = None
    latest_stamp = -math.inf
    for _, message in bag_messages(mapping_bag, {"/tf"}):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                stamp = seconds(transform.header.stamp)
                if stamp >= latest_stamp:
                    latest_stamp = stamp
                    translation = transform.transform.translation
                    alignment = (translation.x, translation.y, yaw(transform.transform.rotation))
    if alignment is None:
        raise ValueError("Mapping bag has no map -> odom TF")
    return alignment


def read_odometry(bag_path):
    odometry = []
    for _, message in bag_messages(bag_path, {"/odom/wheel"}):
        position = message.pose.pose.position
        odometry.append(
            (
                seconds(message.header.stamp),
                position.x,
                position.y,
                yaw(message.pose.pose.orientation),
            )
        )
    if not odometry:
        raise ValueError("Reference bag has no /odom/wheel")
    odometry.sort(key=lambda sample: sample[0])
    return odometry


def read_poses(bag_path):
    poses = []
    for _, message in bag_messages(bag_path, {"/pose"}):
        if message.header.frame_id != "map":
            raise ValueError(f"Unexpected /pose frame: {message.header.frame_id}")
        position = message.pose.pose.position
        poses.append(
            (
                seconds(message.header.stamp),
                position.x,
                position.y,
                yaw(message.pose.pose.orientation),
            )
        )
    if not poses:
        raise ValueError("Pose bag has no /pose")
    poses.sort(key=lambda sample: sample[0])
    return poses


def read_perturbed_tf(bag_path):
    transforms = []
    for _, message in bag_messages(bag_path, {"/tf"}):
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                position = transform.transform.translation
                transforms.append(
                    (
                        seconds(transform.header.stamp),
                        position.x,
                        position.y,
                        yaw(transform.transform.rotation),
                    )
                )
    if not transforms:
        raise ValueError("Perturbed bag has no odom -> base_link TF")
    transforms.sort(key=lambda sample: sample[0])
    return transforms


def analyze(mapping_bag, reference_bag, pose_bag, perturbed_bag, max_skew_s):
    if max_skew_s <= 0 or not math.isfinite(max_skew_s):
        raise ValueError("Timestamp skew limit must be finite and positive")
    alignment = read_map_alignment(mapping_bag)
    odometry = read_odometry(reference_bag)
    poses = read_poses(pose_bag)
    odom_stamps = [sample[0] for sample in odometry]
    perturbed = read_perturbed_tf(perturbed_bag) if perturbed_bag else []
    perturbed_stamps = [sample[0] for sample in perturbed]

    position_errors = []
    heading_errors = []
    input_drifts = []
    stamp_skews = []
    tx, ty, theta = alignment
    for stamp, x, y, heading in poses:
        reference = nearest(odometry, odom_stamps, stamp)
        skew = abs(reference[0] - stamp)
        if skew > max_skew_s:
            continue
        truth_x = tx + math.cos(theta) * reference[1] - math.sin(theta) * reference[2]
        truth_y = ty + math.sin(theta) * reference[1] + math.cos(theta) * reference[2]
        position_errors.append(math.hypot(x - truth_x, y - truth_y))
        heading_errors.append(angle_error(heading, theta + reference[3]))
        stamp_skews.append(skew)
        if perturbed:
            input_tf = nearest(perturbed, perturbed_stamps, stamp)
            if abs(input_tf[0] - stamp) <= max_skew_s:
                input_drifts.append(
                    math.hypot(input_tf[1] - reference[1], input_tf[2] - reference[2])
                )

    if not position_errors:
        raise ValueError("No poses match reference odometry within the timestamp limit")
    if perturbed_bag and not input_drifts:
        raise ValueError("No poses match perturbed TF within the timestamp limit")
    return {
        "map_to_odom_alignment_xy_yaw": [round(value, 4) for value in alignment],
        "pose_messages": len(poses),
        "matched_poses": len(position_errors),
        "position_error_m": metric(position_errors),
        "heading_error_rad": metric(heading_errors),
        "input_drift_m": metric(input_drifts),
        "pose_odom_stamp_skew_s": metric(stamp_skews),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mapping_bag", type=Path)
    parser.add_argument("reference_bag", type=Path)
    parser.add_argument("pose_bag", type=Path)
    parser.add_argument("--perturbed-bag", type=Path)
    parser.add_argument("--max-skew-s", type=float, default=0.15)
    args = parser.parse_args()
    print(
        json.dumps(
            analyze(
                args.mapping_bag,
                args.reference_bag,
                args.pose_bag,
                args.perturbed_bag,
                args.max_skew_s,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
