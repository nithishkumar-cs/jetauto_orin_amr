#!/usr/bin/env python3
"""Count Nav2 scan TF-filter events and estimate scan age at release.

Run the planner with the `tf2_ros_message_filter` logger at DEBUG and record
`/clock` and `/scan` in a separate bag. Ages are observer estimates: the log
and bag recorder run in different processes, although on the same machine.
"""

import argparse
import bisect
import json
import math
import re
import statistics
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this tool.") from error


EVENT = re.compile(
    r"\[DEBUG\] \[(?P<wall>\d+\.\d+)\] \[tf2_ros_message_filter\]: "
    r"MessageFilter \[target=map\s*\]: "
    r"(?P<kind>Added message|Message ready|Discarding message) "
    r"in frame \S+ at time (?P<stamp>\d+\.\d+)"
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def metric(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median": round(statistics.median(ordered), 4),
        "p95": round(ordered[math.ceil(0.95 * len(ordered)) - 1], 4),
        "max": round(ordered[-1], 4),
    }


def interpolate_clock(clock_samples, wall_time):
    walls = [sample[0] for sample in clock_samples]
    index = bisect.bisect_left(walls, wall_time)
    if index == 0 or index == len(walls):
        return None
    before_wall, before_ros = clock_samples[index - 1]
    after_wall, after_ros = clock_samples[index]
    fraction = (wall_time - before_wall) / (after_wall - before_wall)
    return before_ros + fraction * (after_ros - before_ros)


def read_observer_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    clock_samples = []
    scan_receipts = {}
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        recorded_s = recorded_ns / 1e9
        if topic == "/clock":
            clock = deserialize_message(data, Clock)
            clock_samples.append((recorded_s, seconds(clock.clock)))
        elif topic == "/scan":
            scan = deserialize_message(data, LaserScan)
            scan_receipts[round(seconds(scan.header.stamp), 3)] = recorded_s
    if len(clock_samples) < 2 or not scan_receipts:
        raise ValueError("Observer bag needs /clock and /scan")
    clock_samples.sort()
    return clock_samples, scan_receipts


def analyze(log_path, bag_path):
    clock_samples, scan_receipts = read_observer_bag(bag_path)
    counts = {"Added message": 0, "Message ready": 0, "Discarding message": 0}
    age_ros = {"Message ready": [], "Discarding message": []}
    latency_wall = {"Message ready": [], "Discarding message": []}
    unmatched = 0
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        event = EVENT.search(line)
        if not event:
            continue
        kind = event.group("kind")
        counts[kind] += 1
        if kind == "Added message":
            continue
        wall_time = float(event.group("wall"))
        stamp = float(event.group("stamp"))
        received = scan_receipts.get(round(stamp, 3))
        ros_time = interpolate_clock(clock_samples, wall_time)
        if received is None or ros_time is None:
            unmatched += 1
            continue
        latency_wall[kind].append(wall_time - received)
        age_ros[kind].append(ros_time - stamp)
    return {
        "observed_scan_messages": len(scan_receipts),
        "filter_received": counts["Added message"],
        "filter_ready": counts["Message ready"],
        "filter_discarded": counts["Discarding message"],
        "terminal_events_unmatched_to_observer": unmatched,
        "ready_wall_latency_s": metric(latency_wall["Message ready"]),
        "ready_ros_age_s": metric(age_ros["Message ready"]),
        "discard_wall_latency_s": metric(latency_wall["Discarding message"]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("observer_bag", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.log, args.observer_bag), indent=2))


if __name__ == "__main__":
    main()
