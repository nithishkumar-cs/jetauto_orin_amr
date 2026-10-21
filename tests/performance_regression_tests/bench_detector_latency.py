#!/usr/bin/env python3
"""Measure detector publish-to-result latency for the running detector_node.

Publishes a generated rgb8 image, waits for the Detection2DArray with the same
header stamp, and prints min/mean/max latency in ms. The node under test must
already be running with the deployed config. Exit code 1 if any frame dropped.
"""
import argparse
import statistics
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


def main() -> int:
    """Run the measurement, print a summary line, and return the exit code."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--width", type=int, default=810)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--timeout", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("bench_detector_latency")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(Image, "/camera/rgb/image_rect_color", qos)
    got = {}
    node.create_subscription(
        Detection2DArray, "/perception/detections_2d",
        lambda m: got.setdefault((m.header.stamp.sec, m.header.stamp.nanosec),
                                 time.perf_counter()), qos)

    rng = np.random.default_rng(0)
    data = rng.integers(0, 255, args.height * args.width * 3, dtype=np.uint8).tobytes()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    lat, dropped = [], 0
    for i in range(args.warmup + args.frames):
        msg = Image(height=args.height, width=args.width, encoding="rgb8",
                    step=args.width * 3, data=data)
        msg.header.frame_id = "camera"
        stamp = node.get_clock().now().to_msg()
        msg.header.stamp = stamp
        key = (stamp.sec, stamp.nanosec)
        t0 = time.perf_counter()
        pub.publish(msg)
        while key not in got and time.perf_counter() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.01)
        if key in got:
            if i >= args.warmup:
                lat.append((got[key] - t0) * 1e3)
        elif i >= args.warmup:
            dropped += 1
        time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()
    if lat:
        print(f"n={len(lat)} dropped={dropped} min={min(lat):.2f} "
              f"mean={statistics.mean(lat):.2f} max={max(lat):.2f} ms")
    else:
        print(f"no results, dropped={dropped}")
    return 1 if dropped or not lat else 0


if __name__ == "__main__":
    raise SystemExit(main())
