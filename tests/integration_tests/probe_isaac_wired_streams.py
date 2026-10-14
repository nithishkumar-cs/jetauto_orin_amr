#!/usr/bin/env python3
"""Observe one live Isaac DDS source over Ethernet without sending commands."""

import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage


TOPICS = {
    "clock": ("/clock", Clock),
    "scan": ("/scan", LaserScan),
    "odom": ("/odom/wheel", Odometry),
    "tf": ("/tf", TFMessage),
    "rgb": ("/camera/rgb/image_rect_color", Image),
    "depth": ("/camera/aligned_depth_to_rgb/image_raw", Image),
    "camera_info": ("/camera/rgb/camera_info", CameraInfo),
}
EXPECTED_STATIC_TF_NODES = {
    "_World_JetAutoROS_OpticalTF",
    "_World_JetAutoROS_CameraMountTF",
    "_World_JetAutoROS_LidarTF",
}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class StreamProbe(Node):
    def __init__(self):
        super().__init__("isaac_wired_stream_probe")
        self.samples = {name: [] for name in TOPICS}
        self._held_subscriptions = []
        for name, (topic, kind) in TOPICS.items():
            self._held_subscriptions.append(
                self.create_subscription(
                    kind, topic, lambda message, key=name: self.on_message(key, message),
                    qos_profile_sensor_data,
                )
            )

    def on_message(self, name, message):
        if name == "clock":
            stamp = message.clock
        elif name == "tf":
            if not message.transforms:
                return
            stamp = message.transforms[-1].header.stamp
        else:
            stamp = message.header.stamp
        self.samples[name].append((time.monotonic(), stamp_ns(stamp)))


def stream_metrics(samples, duration):
    if not samples:
        return {
            "count": 0,
            "wall_rate_hz": 0.0,
            "backward_stamps": 0,
            "max_wall_gap_s": 0.0,
            "max_stamp_gap_s": 0.0,
            "stamp_progress_s": 0.0,
        }
    wall_gaps = [current[0] - previous[0] for previous, current in zip(samples, samples[1:])]
    stamp_gaps = [(current[1] - previous[1]) / 1e9 for previous, current in zip(samples, samples[1:])]
    return {
        "count": len(samples),
        "wall_rate_hz": round(len(samples) / duration, 3),
        "backward_stamps": sum(gap < 0 for gap in stamp_gaps),
        "max_wall_gap_s": round(max(wall_gaps, default=0.0), 3),
        "max_stamp_gap_s": round(max(stamp_gaps, default=0.0), 3),
        "stamp_progress_s": round((samples[-1][1] - samples[0][1]) / 1e9, 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--require-camera", action="store_true")
    args = parser.parse_args()
    if args.duration < 10.0:
        parser.error("--duration must be at least 10 seconds")

    rclpy.init()
    node = StreamProbe()
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        endpoints = {
            name: len(node.get_publishers_info_by_topic(topic))
            for name, (topic, _) in TOPICS.items()
        }
        static_tf_nodes = [
            endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic("/tf_static")
        ]
        endpoints["tf_static"] = len(static_tf_nodes)
        command_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel/safety_limited")
        metrics = {
            name: stream_metrics(samples, args.duration)
            for name, samples in node.samples.items()
        }
        rgb_stamps = {stamp for _, stamp in node.samples["rgb"]}
        depth_stamps = {stamp for _, stamp in node.samples["depth"]}
        matched_frames = len(rgb_stamps & depth_stamps)

        core_errors = []
        camera_errors = []
        for name in ("clock", "scan", "odom", "tf"):
            if endpoints[name] != 1:
                core_errors.append(f"{name}: expected one publisher, got {endpoints[name]}")
            if metrics[name]["count"] < args.duration * (1.0 if name == "scan" else 5.0):
                core_errors.append(f"{name}: insufficient live messages")
            if metrics[name]["backward_stamps"]:
                core_errors.append(f"{name}: backward timestamps")
            if metrics[name]["max_wall_gap_s"] > (1.0 if name == "scan" else 0.5):
                core_errors.append(f"{name}: excessive wall arrival gap")
        if set(static_tf_nodes) != EXPECTED_STATIC_TF_NODES or len(static_tf_nodes) != 3:
            core_errors.append(f"tf_static: unexpected publishers {static_tf_nodes}")
        if len(command_subscribers) != 1 or "CmdVelSub" not in command_subscribers[0].node_name:
            core_errors.append("expected the sole final-command subscriber to be Isaac")

        if args.require_camera:
            for name in ("rgb", "depth", "camera_info"):
                if endpoints[name] != 1:
                    camera_errors.append(f"{name}: expected one publisher, got {endpoints[name]}")
                if metrics[name]["count"] < args.duration:
                    camera_errors.append(f"{name}: insufficient live messages")
                if metrics[name]["backward_stamps"]:
                    camera_errors.append(f"{name}: backward timestamps")
                if metrics[name]["max_wall_gap_s"] > 1.0:
                    camera_errors.append(f"{name}: excessive wall arrival gap")
            if matched_frames < 10:
                camera_errors.append(f"RGB/depth exact timestamp matches: {matched_frames} < 10")

        report = {
            "duration_wall_s": args.duration,
            "publishers": endpoints,
            "static_tf_publisher_nodes": sorted(static_tf_nodes),
            "isaac_command_subscribers": [endpoint.node_name for endpoint in command_subscribers],
            "streams": metrics,
            "rgb_depth_stamp_matches": matched_frames,
            "core_errors": core_errors,
            "camera_errors": camera_errors,
        }
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        if core_errors:
            return 1
        if camera_errors:
            return 2
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
