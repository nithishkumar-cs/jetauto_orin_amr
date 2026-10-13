#!/usr/bin/env python3
"""Add a temporary synthetic scan return for an offline Nav2 costmap test.

The original /scan is untouched. The derived /scan_costmap uses Isaac's
reference odometry to aim a five-beam return at a fixed map point. This is a
validation fixture, not a model of a moving obstacle or a production sensor.
"""

import argparse
import bisect
import json
import math
from pathlib import Path

try:
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this tool.") from error


SOURCE_SCAN = "/scan"
COSTMAP_SCAN = "/scan_costmap"
REFERENCE_ODOM = "/validation/reference_odom"


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def reference_poses(path):
    reader = open_reader(path)
    poses = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != REFERENCE_ODOM:
            continue
        odom = deserialize_message(data, Odometry)
        orientation = odom.pose.pose.orientation
        yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y**2 + orientation.z**2),
        )
        poses.append(
            (
                stamp_seconds(odom.header.stamp),
                 odom.pose.pose.position.x, odom.pose.pose.position.y, yaw)
        )
    if not poses:
        raise ValueError(f"Missing {REFERENCE_ODOM} in source bag")
    poses.sort()
    return poses


def target_beam(scan, pose, target_map_xy, map_to_odom, lidar_x_m):
    """Find beam/range to a map point, using a fixed test map alignment."""
    _, base_x, base_y, base_yaw = pose
    map_x, map_y = target_map_xy
    tf_x, tf_y, tf_yaw = map_to_odom
    delta_x, delta_y = map_x - tf_x, map_y - tf_y
    odom_x = math.cos(tf_yaw) * delta_x + math.sin(tf_yaw) * delta_y
    odom_y = -math.sin(tf_yaw) * delta_x + math.cos(tf_yaw) * delta_y
    lidar_x = base_x + lidar_x_m * math.cos(base_yaw)
    lidar_y = base_y + lidar_x_m * math.sin(base_yaw)
    dx, dy = odom_x - lidar_x, odom_y - lidar_y
    distance = math.hypot(dx, dy)
    angle = math.atan2(dy, dx) - base_yaw
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    index = round((angle - scan.angle_min) / scan.angle_increment)
    if not scan.range_min < distance < min(scan.range_max, 4.0):
        return None
    if index < 2 or index + 2 >= len(scan.ranges):
        return None
    return index, distance


def inject(source_bag, output_bag, start_s, end_s, target_map_xy, map_to_odom, lidar_x_m):
    if output_bag.exists():
        raise ValueError(f"Output already exists: {output_bag}")
    if not 0 <= start_s < end_s:
        raise ValueError("Expected 0 <= start_s < end_s")
    poses = reference_poses(source_bag)
    pose_stamps = [item[0] for item in poses]
    reader = open_reader(source_bag)
    metadata = {item.name: item for item in reader.get_all_topics_and_types()}
    required = {SOURCE_SCAN, REFERENCE_ODOM, "/clock", "/tf", "/tf_static"}
    if required - metadata.keys():
        raise ValueError(f"Source is missing topics: {sorted(required - metadata.keys())}")
    if COSTMAP_SCAN in metadata:
        raise ValueError(f"Source already contains {COSTMAP_SCAN}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in metadata.values():
        writer.create_topic(topic)
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=COSTMAP_SCAN, type="sensor_msgs/msg/LaserScan", serialization_format="cdr"
        )
    )
    counts = {"source_scans": 0, "cloned_scans": 0, "injected_scans": 0,
              "modified_beams": 0, "skipped_no_pose": 0,
              "skipped_target_not_visible": 0, "skipped_occluded": 0}
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        writer.write(topic, data, recorded_ns)
        if topic != SOURCE_SCAN:
            continue
        counts["source_scans"] += 1
        scan = deserialize_message(data, LaserScan)
        stamp = stamp_seconds(scan.header.stamp)
        if start_s <= stamp < end_s:
            index = bisect.bisect_left(pose_stamps, stamp)
            candidates = poses[max(0, index - 1):min(len(poses), index + 1)]
            pose = min(candidates, key=lambda item: abs(item[0] - stamp))
            if abs(pose[0] - stamp) > 0.02:
                counts["skipped_no_pose"] += 1
            else:
                beam = target_beam(scan, pose, target_map_xy, map_to_odom, lidar_x_m)
                if beam is None:
                    counts["skipped_target_not_visible"] += 1
                else:
                    center, distance = beam
                    changed = 0
                    for offset in range(-2, 3):
                        beam_index = center + offset
                        original = scan.ranges[beam_index]
                        if not math.isfinite(original) or original > distance:
                            scan.ranges[beam_index] = distance
                            changed += 1
                    if changed:
                        counts["injected_scans"] += 1
                        counts["modified_beams"] += changed
                    else:
                        counts["skipped_occluded"] += 1
        writer.write(COSTMAP_SCAN, serialize_message(scan), recorded_ns)
        counts["cloned_scans"] += 1
    return {"source": str(source_bag), "output": str(output_bag), **counts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--start-s", type=float, default=18.0)
    parser.add_argument("--end-s", type=float, default=22.0)
    parser.add_argument("--target-x", type=float, default=2.0)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--map-to-odom-x", type=float, default=-0.0312)
    parser.add_argument("--map-to-odom-y", type=float, default=-0.0057)
    parser.add_argument("--map-to-odom-yaw", type=float, default=0.0041)
    parser.add_argument("--lidar-x", type=float, default=0.15)
    args = parser.parse_args()
    print(json.dumps(inject(
        args.source_bag, args.output_bag, args.start_s, args.end_s,
        (args.target_x, args.target_y),
        (args.map_to_odom_x, args.map_to_odom_y, args.map_to_odom_yaw),
        args.lidar_x,
    ), indent=2))


if __name__ == "__main__":
    main()
