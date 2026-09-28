#!/usr/bin/env python3
"""Summarize timing, synchronization, and semantic track IDs in a ROS 2 bag."""

import argparse
import math
from collections import Counter, defaultdict
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ModuleNotFoundError as error:
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
        "and this workspace's install/setup.bash before running this command."
    ) from error


STAMP_PAIRS = (
    ("/camera/rgb/image_rect_color", "/camera/aligned_depth_to_rgb/image_raw"),
    ("/camera/rgb/image_rect_color", "/perception/detections_2d"),
    ("/perception/detections_2d", "/perception/detections_3d"),
    ("/perception/camera/tracks", "/perception/fused_obstacles"),
)


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def message_stamp_ns(topic, message):
    if topic == "/clock" and hasattr(message, "clock"):
        return message.clock.sec * 1_000_000_000 + message.clock.nanosec
    if not hasattr(message, "header"):
        return None
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def frequency_hz(timestamps_ns):
    if len(timestamps_ns) < 2 or timestamps_ns[-1] <= timestamps_ns[0]:
        return math.nan
    return (len(timestamps_ns) - 1) / ((timestamps_ns[-1] - timestamps_ns[0]) / 1e9)


def intervals_s(timestamps_ns):
    return [
        (current - previous) / 1e9
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
    ]


def format_number(value):
    return "-" if not math.isfinite(value) else f"{value:.3f}"


def analyze(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    received_ns = defaultdict(list)
    stamps_ns = defaultdict(list)
    image_formats = {}
    obstacle_counts = defaultdict(list)
    class_counts = defaultdict(Counter)
    class_track_ids = defaultdict(lambda: defaultdict(set))

    while reader.has_next():
        topic, serialized, received_timestamp_ns = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        received_ns[topic].append(received_timestamp_ns)
        stamp_ns = message_stamp_ns(topic, message)
        if stamp_ns is not None:
            stamps_ns[topic].append(stamp_ns)

        if hasattr(message, "encoding"):
            image_formats[topic] = (
                message.width,
                message.height,
                message.encoding,
                message.step,
                len(message.data),
            )

        if hasattr(message, "obstacles"):
            obstacle_counts[topic].append(len(message.obstacles))
            for obstacle in message.obstacles:
                class_id = obstacle.classification.class_id
                if class_id:
                    class_counts[topic][class_id] += 1
                    class_track_ids[topic][class_id].add(obstacle.track_id)

    return {
        "received_ns": received_ns,
        "stamps_ns": stamps_ns,
        "image_formats": image_formats,
        "obstacle_counts": obstacle_counts,
        "class_counts": class_counts,
        "class_track_ids": class_track_ids,
    }


def print_report(bag_path, analysis, expected_period_s, gap_factor):
    received_ns = analysis["received_ns"]
    stamps_ns = analysis["stamps_ns"]
    print(f"Bag: {bag_path}")
    print()
    print("Topic timing")
    print(
        "topic                                      count record_hz stamp_hz "
        "dt50_s dt95_s max_dt_s"
    )
    for topic in sorted(received_ns):
        stamp_intervals = [
            interval for interval in intervals_s(stamps_ns[topic]) if interval >= 0.0
        ]
        print(
            f"{topic:42} {len(received_ns[topic]):5d} "
            f"{format_number(frequency_hz(received_ns[topic])):>9} "
            f"{format_number(frequency_hz(stamps_ns[topic])):>8} "
            f"{format_number(percentile(stamp_intervals, 0.50)):>6} "
            f"{format_number(percentile(stamp_intervals, 0.95)):>6} "
            f"{format_number(max(stamp_intervals) if stamp_intervals else math.nan):>8}"
        )

    if analysis["image_formats"]:
        print()
        print("Image formats")
        for topic, (width, height, encoding, step, size) in sorted(
            analysis["image_formats"].items()
        ):
            print(f"{topic}: {width}x{height} {encoding}, step={step}, bytes={size}")

    print()
    print("Exact header-stamp coverage")
    for first, second in STAMP_PAIRS:
        if first not in stamps_ns or second not in stamps_ns:
            continue
        first_stamps = set(stamps_ns[first])
        second_stamps = set(stamps_ns[second])
        print(
            f"{first} <-> {second}: common={len(first_stamps & second_stamps)}, "
            f"only_first={len(first_stamps - second_stamps)}, "
            f"only_second={len(second_stamps - first_stamps)}"
        )

    gap_threshold_s = expected_period_s * gap_factor
    print()
    print(f"Header gaps over {gap_threshold_s:.3f} s (expected period {expected_period_s:.3f} s)")
    for topic in sorted(stamps_ns):
        gaps = [
            interval
            for interval in intervals_s(stamps_ns[topic])
            if interval > gap_threshold_s
        ]
        if gaps:
            print(f"{topic}: count={len(gaps)}, max={max(gaps):.3f} s")

    if analysis["obstacle_counts"]:
        print()
        print("Tracked obstacle summary")
        for topic, counts in sorted(analysis["obstacle_counts"].items()):
            average = sum(counts) / len(counts)
            print(
                f"{topic}: messages={len(counts)}, obstacles={sum(counts)}, "
                f"average={average:.2f}/message"
            )
            for class_id, count in sorted(analysis["class_counts"][topic].items()):
                ids = sorted(analysis["class_track_ids"][topic][class_id])
                print(f"  {class_id}: samples={count}, track_ids={ids}")


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--expected-period",
        type=float,
        default=0.1,
        help="expected sensor period in seconds for gap reporting (default: 0.1)",
    )
    parser.add_argument(
        "--gap-factor",
        type=float,
        default=1.5,
        help="report intervals larger than expected-period times this factor (default: 1.5)",
    )
    arguments = parser.parse_args()
    if arguments.expected_period <= 0.0 or arguments.gap_factor <= 1.0:
        parser.error("expected-period must be positive and gap-factor must be greater than one")
    if not arguments.bag.is_dir():
        parser.error(f"bag directory does not exist: {arguments.bag}")
    return arguments


def main():
    arguments = parse_arguments()
    analysis = analyze(arguments.bag)
    print_report(arguments.bag, analysis, arguments.expected_period, arguments.gap_factor)


if __name__ == "__main__":
    main()
