#!/usr/bin/env python3
"""Build a replay bag with a controlled odom -> base_link TF drift ramp."""

import argparse
import json
import math
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from tf2_msgs.msg import TFMessage
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this tool.") from error


REPLAY_TOPICS = {"/clock", "/scan", "/tf", "/tf_static"}
REFERENCE_TOPIC = "/validation/reference_odom"


def apply_yaw_drift(quaternion, delta):
    """Left-multiply a yaw rotation, preserving any existing roll and pitch."""
    sine = math.sin(delta / 2.0)
    cosine = math.cos(delta / 2.0)
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    quaternion.x = cosine * x - sine * y
    quaternion.y = sine * x + cosine * y
    quaternion.z = cosine * z + sine * w
    quaternion.w = cosine * w - sine * z


def inject(source_bag, output_bag, x_m, y_m, yaw_rad, start_s, duration_s):
    if output_bag.exists():
        raise ValueError(f"Output already exists: {output_bag}")
    if duration_s <= 0 or not all(
        math.isfinite(v) for v in (x_m, y_m, yaw_rad, start_s, duration_s)
    ):
        raise ValueError("Drift values must be finite and ramp duration must be positive")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    metadata = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    required = REPLAY_TOPICS | {"/odom/wheel"}
    missing = sorted(required - metadata.keys())
    if missing:
        raise ValueError(f"Source bag is missing required topics: {missing}")
    if REFERENCE_TOPIC in metadata:
        raise ValueError(f"Source bag already has {REFERENCE_TOPIC}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in metadata.values():
        if topic.name in REPLAY_TOPICS:
            writer.create_topic(topic)
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=REFERENCE_TOPIC,
            type="nav_msgs/msg/Odometry",
            serialization_format="cdr",
        )
    )

    counts = {name: 0 for name in REPLAY_TOPICS | {REFERENCE_TOPIC}}
    removed_map_tf = 0
    while reader.has_next():
        name, data, recorded_stamp = reader.read_next()
        if name == "/odom/wheel":
            writer.write(REFERENCE_TOPIC, data, recorded_stamp)
            counts[REFERENCE_TOPIC] += 1
            continue
        if name not in REPLAY_TOPICS:
            continue
        if name == "/tf":
            message = deserialize_message(data, TFMessage)
            kept = []
            for transform in message.transforms:
                if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    removed_map_tf += 1
                    continue
                if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                    stamp = transform.header.stamp
                    sim_time = stamp.sec + stamp.nanosec / 1e9
                    fraction = min(1.0, max(0.0, (sim_time - start_s) / duration_s))
                    transform.transform.translation.x += x_m * fraction
                    transform.transform.translation.y += y_m * fraction
                    apply_yaw_drift(transform.transform.rotation, yaw_rad * fraction)
                kept.append(transform)
            if not kept:
                continue
            message.transforms = kept
            data = serialize_message(message)
        writer.write(name, data, recorded_stamp)
        counts[name] += 1

    return {
        "output_bag": str(output_bag),
        "messages": counts,
        "removed_map_to_odom_transforms": removed_map_tf,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--x-m", type=float, required=True)
    parser.add_argument("--y-m", type=float, required=True)
    parser.add_argument("--yaw-rad", type=float, required=True)
    parser.add_argument("--ramp-start-s", type=float, default=0.0)
    parser.add_argument("--ramp-duration-s", type=float, required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            inject(
                args.source_bag,
                args.output_bag,
                args.x_m,
                args.y_m,
                args.yaw_rad,
                args.ramp_start_s,
                args.ramp_duration_s,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
