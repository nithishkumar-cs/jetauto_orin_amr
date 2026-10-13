#!/usr/bin/env python3
"""Estimate map->odom availability when each scan reaches a bag observer.

Both topics must be recorded by one observer. Receipt times are approximate
cross-process timing evidence, not exact Nav2 filter callback times.
"""

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
except ModuleNotFoundError as error:
    raise SystemExit("Source /opt/ros/humble/setup.bash before running this tool.") from error


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


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    scans = []
    transforms = []
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        receipt_s = recorded_ns / 1e9
        if topic == "/scan":
            scan = deserialize_message(data, LaserScan)
            scans.append((receipt_s, seconds(scan.header.stamp)))
        elif topic == "/tf":
            for transform in deserialize_message(data, TFMessage).transforms:
                if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                    transforms.append((receipt_s, seconds(transform.header.stamp)))
    if not scans or not transforms:
        raise ValueError("Observer bag must contain /scan and map -> odom /tf")
    scans.sort()
    transforms.sort()
    return scans, transforms


def analyze_samples(
    scans, transforms, scan_tolerance_s=0.05, localizer_transform_timeout_s=None
):
    if scan_tolerance_s < 0 or not math.isfinite(scan_tolerance_s):
        raise ValueError("scan_tolerance_s must be finite and nonnegative")
    if localizer_transform_timeout_s is not None and (
        localizer_transform_timeout_s < 0 or not math.isfinite(localizer_transform_timeout_s)
    ):
        raise ValueError("localizer_transform_timeout_s must be finite and nonnegative")
    receipt_times = [item[0] for item in transforms]
    stamp_leads = []
    waits = []
    covered_at_receipt = 0
    uncovered_at_end = 0
    no_tf_at_receipt = 0
    for receipt_s, scan_stamp in scans:
        first_future = bisect.bisect_right(receipt_times, receipt_s)
        needed_stamp = scan_stamp + scan_tolerance_s
        if first_future:
            stamp_leads.append(transforms[first_future - 1][1] - scan_stamp)
            if transforms[first_future - 1][1] >= needed_stamp:
                covered_at_receipt += 1
                waits.append(0.0)
                continue
        else:
            no_tf_at_receipt += 1
        for index in range(first_future, len(transforms)):
            future_receipt, future_stamp = transforms[index]
            if future_stamp >= needed_stamp:
                waits.append(future_receipt - receipt_s)
                break
        else:
            uncovered_at_end += 1
    source_scan_to_first_tf_lags = []
    if localizer_transform_timeout_s is not None:
        scan_receipts = {round(stamp, 3): receipt for receipt, stamp in scans}
        first_tf_receipts = {}
        for receipt, stamp in transforms:
            source_stamp = round(stamp - localizer_transform_timeout_s, 3)
            if source_stamp in scan_receipts:
                first_tf_receipts.setdefault(source_stamp, receipt)
        source_scan_to_first_tf_lags = [
            receipt - scan_receipts[stamp]
            for stamp, receipt in first_tf_receipts.items()
        ]
    return {
        "observed_scans": len(scans),
        "observed_map_tf": len(transforms),
        "covered_at_scan_receipt": covered_at_receipt,
        "no_map_tf_at_scan_receipt": no_tf_at_receipt,
        "uncovered_at_end": uncovered_at_end,
        "latest_tf_stamp_minus_scan_stamp_s": metric(stamp_leads),
        "wait_until_covering_tf_receipt_s": metric(waits),
        "first_tf_receipt_minus_source_scan_receipt_wall_s": metric(
            source_scan_to_first_tf_lags
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("observer_bag", type=Path)
    parser.add_argument("--scan-tolerance-s", type=float, default=0.05)
    parser.add_argument("--localizer-transform-timeout-s", type=float)
    args = parser.parse_args()
    print(json.dumps(analyze_samples(
        *read_bag(args.observer_bag), args.scan_tolerance_s,
        args.localizer_transform_timeout_s,
    ), indent=2))


if __name__ == "__main__":
    main()
