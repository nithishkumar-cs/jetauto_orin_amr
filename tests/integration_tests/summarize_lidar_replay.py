#!/usr/bin/env python3
"""Summarize LiDAR cluster and track output bags from perception replay sweeps.

The reported counts measure output volume and track churn, not obstacle
accuracy. Use labeled detections and moving-object ground truth before
calibrating recall, identity, velocity, or the dynamic classification threshold.
Requires ROS 2 Humble and the workspace to be sourced.
"""

import argparse
from collections import Counter
from pathlib import Path
from statistics import median

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CLUSTERS = "/perception/lidar/clusters"
TRACKS = "/perception/lidar/tracks"


def summarize_bag(path):
    """Stream one replay bag and return cluster and track count statistics."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = {CLUSTERS, TRACKS} - topic_types.keys()
    if missing:
        raise ValueError(f"{path}: missing {', '.join(sorted(missing))}")
    message_types = {topic: get_message(topic_types[topic]) for topic in (CLUSTERS, TRACKS)}

    cluster_messages = cluster_total = track_messages = track_total = dynamic_total = 0
    track_frames = Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == CLUSTERS:
            cluster_messages += 1
            cluster_total += len(message.detections)
        else:
            track_messages += 1
            track_total += len(message.obstacles)
            for obstacle in message.obstacles:
                track_frames[obstacle.track_id] += 1
                dynamic_total += bool(obstacle.is_dynamic)
    return {
        "scan_outputs": cluster_messages,
        "clusters_per_scan": cluster_total / cluster_messages if cluster_messages else 0.0,
        "track_outputs": track_messages,
        "tracks_per_output": track_total / track_messages if track_messages else 0.0,
        "unique_ids": len(track_frames),
        "median_id_frames": median(track_frames.values()) if track_frames else 0,
        "dynamic_percent": 100 * dynamic_total / track_total if track_total else 0.0,
    }


def main():
    """Print a compact comparison of one or more replay output bags."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", type=Path, nargs="+", help="replay output bag directories")
    args = parser.parse_args()
    print("bag\tscans\tclusters/scan\ttrack msgs\ttracks/msg\tIDs\tmedian ID frames\tdynamic %")
    for path in args.bags:
        result = summarize_bag(path)
        print(
            f"{path.name}\t{result['scan_outputs']}\t{result['clusters_per_scan']:.1f}"
            f"\t{result['track_outputs']}\t{result['tracks_per_output']:.1f}"
            f"\t{result['unique_ids']}\t{result['median_id_frames']:.0f}"
            f"\t{result['dynamic_percent']:.1f}"
        )


if __name__ == "__main__":
    main()
