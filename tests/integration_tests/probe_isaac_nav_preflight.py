#!/usr/bin/env python3
"""Stationary and tiny-motion localization preflight for Isaac FollowPath only."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(orientation):
    return 2.0 * math.atan2(orientation.z, orientation.w)


class Probe(Node):
    def __init__(self):
        super().__init__("isaac_nav_preflight")
        self.odom = None
        self.pose = None
        self.pose_count = 0
        self.clock = None
        self.scans = 0
        self.global_costmap = None
        self.local_costmap = None
        self.final_commands = []
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom/wheel", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel/safety_limited", self.on_command, 10)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global_costmap, costmap_qos
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local_costmap, costmap_qos
        )

    def on_odom(self, message):
        self.odom = message

    def on_pose(self, message):
        self.pose = message
        self.pose_count += 1

    def on_clock(self, message):
        self.clock = message

    def on_scan(self, _):
        self.scans += 1

    def on_command(self, message):
        self.final_commands.append(message.linear.x)

    def on_global_costmap(self, message):
        self.global_costmap = message

    def on_local_costmap(self, message):
        self.local_costmap = message

    def spin_for(self, duration, speed=None):
        deadline = time.monotonic() + duration
        next_command = time.monotonic()
        while time.monotonic() < deadline:
            if speed is not None and time.monotonic() >= next_command:
                command = Twist()
                command.linear.x = speed
                self.command.publish(command)
                next_command += 0.05
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    args = parser.parse_args()
    if not args.isaac_simulation_only:
        raise SystemExit("Isaac-only acknowledgement required")

    rclpy.init()
    node = Probe()
    try:
        node.spin_for(3.0)
        assert node.odom is not None and node.clock is not None and node.scans >= 5
        assert node.global_costmap is not None and node.local_costmap is not None
        assert len(node.get_publishers_info_by_topic("/clock")) == 1
        assert len(node.get_publishers_info_by_topic("/cmd_vel/safety_limited")) == 1
        final_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel/safety_limited")
        assert len(final_subscribers) == 2  # Isaac and this observer.
        assert any("CmdVelSub" in endpoint.node_name for endpoint in final_subscribers)
        assert len(node.get_subscriptions_info_by_topic("/cmd_vel")) == 1

        initial = node.odom.pose.pose.position
        initial_xy = (initial.x, initial.y)
        first_pose_count = node.pose_count
        first_command_count = len(node.final_commands)
        node.spin_for(3.0, speed=0.05)
        node.spin_for(0.5, speed=0.0)
        node.spin_for(2.0)
        final = node.odom.pose.pose.position
        displacement = math.hypot(final.x - initial_xy[0], final.y - initial_xy[1])
        print(
            json.dumps(
                {
                    "preflight_observed": {
                        "motion_m": displacement,
                        "pose_updates": node.pose_count - first_pose_count,
                        "scan_count": node.scans,
                        "sim_clock_s": seconds(node.clock.clock),
                        "odom_stamp_s": seconds(node.odom.header.stamp),
                        "last_pose_stamp_s": (
                            seconds(node.pose.header.stamp) if node.pose is not None else None
                        ),
                        "global_costmap_size": [
                            node.global_costmap.info.width,
                            node.global_costmap.info.height,
                        ],
                        "local_costmap_size": [
                            node.local_costmap.info.width,
                            node.local_costmap.info.height,
                        ],
                    }
                },
                indent=2,
            ),
            flush=True,
        )
        assert 0.04 < displacement < 0.20, displacement
        assert any(value > 0.025 for value in node.final_commands[first_command_count:])
        assert node.pose_count > first_pose_count, "no fresh scan-matched /pose after motion"
        assert node.pose.header.frame_id == "map"
        pose_age = seconds(node.clock.clock) - seconds(node.pose.header.stamp)
        # SLAM Toolbox publishes a scan-matched pose on accepted motion,
        # not continuously while the robot is parked after this pulse.
        assert -0.1 < pose_age < 4.0, pose_age
        pose_position = node.pose.pose.pose.position
        odom_position = node.odom.pose.pose.position
        position_error = math.hypot(pose_position.x - odom_position.x, pose_position.y - odom_position.y)
        heading_error = abs(
            math.remainder(
                yaw(node.pose.pose.pose.orientation) - yaw(node.odom.pose.pose.orientation),
                2.0 * math.pi,
            )
        )
        assert position_error < 0.20, position_error
        assert heading_error < 0.15, heading_error
        result = {
            "status": "PASS",
            "initial_odom_xy_m": initial_xy,
            "final_odom_xy_m": [odom_position.x, odom_position.y],
            "motion_m": displacement,
            "pose_updates": node.pose_count - first_pose_count,
            "pose_age_sim_s": pose_age,
            "localization_position_error_m": position_error,
            "localization_heading_error_rad": heading_error,
            "scans_observed": node.scans,
            "global_costmap_size": [node.global_costmap.info.width, node.global_costmap.info.height],
            "local_costmap_size": [node.local_costmap.info.width, node.local_costmap.info.height],
        }
        print(json.dumps(result, indent=2), flush=True)
    finally:
        # Keep the simulation still even if any preflight assertion fails.
        node.spin_for(0.5, speed=0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
