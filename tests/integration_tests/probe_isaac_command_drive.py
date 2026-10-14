#!/usr/bin/env python3
"""Short, direct-Ethernet Isaac command-drive smoke test; never use on hardware."""

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__("isaac_command_drive_probe")
        self.odom = []
        self.tf = {}
        self.clocks = 0
        self.command = self.create_publisher(Twist, "/cmd_vel/safety_limited", 10)
        self.create_subscription(Odometry, "/odom/wheel", self.on_odom, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)

    def on_odom(self, message):
        pose = message.pose.pose.position
        self.odom.append((time.monotonic(), stamp_ns(message.header.stamp), pose.x, pose.y))

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == "base_link":
                point = transform.transform.translation
                self.tf[stamp_ns(transform.header.stamp)] = (point.x, point.y)

    def on_clock(self, _):
        self.clocks += 1

    def send(self, speed):
        message = Twist()
        message.linear.x = speed
        self.command.publish(message)


def spin_for(node, duration, speed=None):
    deadline = time.monotonic() + duration
    next_command = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if speed is not None and now >= next_command:
            node.send(speed)
            next_command = now + 0.05
        rclpy.spin_once(node, timeout_sec=0.01)


def wait_for(node, predicate, seconds=10.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("timed out waiting for Isaac ROS data")


def distance(a, b):
    return math.hypot(a[2] - b[2], a[3] - b[3])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--direct-isaac-simulation-only", action="store_true", required=True)
    args = parser.parse_args()
    if not args.direct_isaac_simulation_only:
        raise SystemExit("explicit simulation-only flag required")

    rclpy.init()
    node = Probe()
    try:
        wait_for(node, lambda: len(node.odom) >= 10 and node.clocks >= 10)
        clock_publishers = node.get_publishers_info_by_topic("/clock")
        command_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel/safety_limited")
        command_publishers = node.get_publishers_info_by_topic("/cmd_vel/safety_limited")
        assert len(clock_publishers) == 1, f"expected one clock publisher, got {len(clock_publishers)}"
        assert len(command_subscribers) == 1, command_subscribers
        assert "CmdVelSub" in command_subscribers[0].node_name, command_subscribers[0].node_name
        assert len(command_publishers) == 1, "another command publisher is active"
        print("PASS single /clock publisher and sole Isaac command subscriber", flush=True)

        spin_for(node, 0.3)
        initial = node.odom[-1]
        first_command_wall = time.monotonic()
        spin_for(node, 1.0, speed=0.10)
        node.send(0.0)
        spin_for(node, 0.5)
        first_motion = next(
            (sample for sample in node.odom if sample[0] >= first_command_wall and distance(initial, sample) > 0.002),
            None,
        )
        assert first_motion is not None, "no odometry response to forward command"
        after_forward = node.odom[-1]
        forward_m = after_forward[2] - initial[2]
        lateral_m = after_forward[3] - initial[3]
        assert forward_m > 0.04, f"forward response too small: {forward_m:.4f} m"
        assert abs(lateral_m) < 0.03, f"unexpected lateral response: {lateral_m:.4f} m"
        latency_ms = (first_motion[0] - first_command_wall) * 1000.0
        print(
            f"PASS forward 0.10 m/s: dx={forward_m:.4f} m, dy={lateral_m:.4f} m, "
            f"first-motion latency={latency_ms:.1f} ms",
            flush=True,
        )

        watchdog_start = node.odom[-1]
        spin_for(node, 0.6, speed=0.10)
        last_command_wall = time.monotonic()
        spin_for(node, 1.1)
        watchdog_samples = [sample for sample in node.odom if sample[0] >= last_command_wall]
        assert len(watchdog_samples) >= 10, "not enough odometry to assess watchdog"
        late_samples = [sample for sample in watchdog_samples if sample[0] >= last_command_wall + 0.8]
        assert len(late_samples) >= 3, "not enough late odometry to assess stop"
        movement_m = distance(watchdog_start, watchdog_samples[-1])
        late_m = distance(late_samples[0], late_samples[-1])
        assert movement_m > 0.02, f"watchdog phase did not move: {movement_m:.4f} m"
        assert late_m < 0.004, f"robot still moved after watchdog: {late_m:.4f} m"
        print(
            f"PASS watchdog stop: phase movement={movement_m:.4f} m; "
            f"late 0.3 s movement={late_m:.4f} m",
            flush=True,
        )

        latencies_ms = []
        for trial in range(8):
            spin_for(node, 0.3)
            baseline = node.odom[-1]
            start = time.monotonic()
            spin_for(node, 0.6, speed=0.05)
            node.send(0.0)
            spin_for(node, 0.35)
            first = next(
                (sample for sample in node.odom if sample[0] >= start and distance(baseline, sample) > 0.001),
                None,
            )
            assert first is not None, f"no motion in latency trial {trial + 1}"
            latencies_ms.append((first[0] - start) * 1000.0)
        ordered = sorted(latencies_ms)
        p95 = ordered[math.ceil(0.95 * len(ordered)) - 1]
        print(
            f"PASS eight Jetson-to-first-odometry-motion trials: "
            f"median={statistics.median(latencies_ms):.1f} ms, "
            f"p95={p95:.1f} ms, max={ordered[-1]:.1f} ms, "
            f"all_ms={[round(value, 1) for value in latencies_ms]}",
            flush=True,
        )

        matches = 0
        mismatches = 0
        for sample in node.odom:
            pose = node.tf.get(sample[1])
            if pose is None:
                continue
            matches += 1
            if math.hypot(sample[2] - pose[0], sample[3] - pose[1]) > 0.001:
                mismatches += 1
        assert matches >= 10, f"too few timestamp-matched odom/TF poses: {matches}"
        assert mismatches == 0, f"odom/TF mismatches: {mismatches}/{matches}"
        print(f"PASS odom/TF consistency: {matches} matched timestamps, 0 mismatches", flush=True)
    finally:
        for _ in range(5):
            node.send(0.0)
            spin_for(node, 0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
