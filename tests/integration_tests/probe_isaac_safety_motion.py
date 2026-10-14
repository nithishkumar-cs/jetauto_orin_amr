#!/usr/bin/env python3
"""Exercise the software safety chain against Isaac only; never use on hardware."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Probe(Node):
    def __init__(self):
        super().__init__("isaac_safety_motion_probe")
        self.odom = []
        self.output = []
        self.scans = 0
        self.clocks = 0
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.estop = self.create_publisher(Bool, "/estop/engaged", 10)
        self.reset = self.create_publisher(Bool, "/estop/reset", 10)
        self.create_subscription(Odometry, "/odom/wheel", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel/safety_limited", self.on_output, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)

    def on_odom(self, message):
        pose = message.pose.pose.position
        self.odom.append((time.monotonic(), pose.x, pose.y))

    def on_output(self, message):
        self.output.append((time.monotonic(), message.linear.x))

    def on_scan(self, _):
        self.scans += 1

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
    raise AssertionError("timed out waiting for live Isaac safety-chain data")


def distance(a, b):
    return math.hypot(a[1] - b[1], a[2] - b[2])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    args = parser.parse_args()
    if not args.isaac_simulation_only:
        raise SystemExit("explicit simulation-only flag required")

    rclpy.init()
    node = Probe()
    try:
        wait_for(node, lambda: len(node.odom) >= 10 and node.scans >= 5 and node.clocks >= 10)
        assert len(node.get_publishers_info_by_topic("/clock")) == 1
        command_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel")
        assert len(command_subscribers) == 1 and command_subscribers[0].node_name == "collision_monitor"
        output_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel/safety_limited")
        assert len(output_subscribers) == 2, output_subscribers  # Isaac plus this observer.
        assert any("CmdVelSub" in endpoint.node_name for endpoint in output_subscribers)
        output_publishers = node.get_publishers_info_by_topic("/cmd_vel/safety_limited")
        assert len(output_publishers) == 1 and output_publishers[0].node_name == "estop_gate"
        print("PASS one clock, live scans, and sole safety-chain output to Isaac", flush=True)

        initial = node.odom[-1]
        output_start = len(node.output)
        spin_for(node, 1.2, speed=0.10)
        moving = node.odom[-1]
        forwarded = [speed for _, speed in node.output[output_start:]]
        assert any(speed > 0.05 for speed in forwarded), "clear-path command did not pass gate"
        assert distance(initial, moving) > 0.04, "Isaac did not move from gated command"
        print(f"PASS clear-path gated motion: {distance(initial, moving):.4f} m", flush=True)

        node.estop.publish(Bool(data=True))
        estop_wall = time.monotonic()
        spin_for(node, 1.1, speed=0.10)
        blocked = [speed for stamp, speed in node.output if stamp > estop_wall + 0.30]
        late_pose = [pose for pose in node.odom if pose[0] > estop_wall + 0.70]
        assert len(blocked) >= 3 and all(abs(speed) < 1e-6 for speed in blocked), blocked
        assert len(late_pose) >= 3 and distance(late_pose[0], late_pose[-1]) < 0.004
        print(
            f"PASS latched e-stop blocks continuous commands; "
            f"late motion={distance(late_pose[0], late_pose[-1]):.4f} m",
            flush=True,
        )

        node.reset.publish(Bool(data=True))
        spin_for(node, 0.2)
        resumed_from = node.odom[-1]
        output_start = len(node.output)
        spin_for(node, 0.8, speed=0.10)
        resumed = node.odom[-1]
        assert any(speed > 0.05 for _, speed in node.output[output_start:])
        assert distance(resumed_from, resumed) > 0.025, "reset did not restore Isaac motion"
        print(f"PASS reset restores gated motion: {distance(resumed_from, resumed):.4f} m", flush=True)
    finally:
        # Leave the software chain latched, then stop all command publication.
        for _ in range(5):
            node.estop.publish(Bool(data=True))
            node.send(0.0)
            spin_for(node, 0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
