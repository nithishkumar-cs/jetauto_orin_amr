#!/usr/bin/env python3
"""Exercise bounded reverse recovery from a live Isaac stop zone, never hardware."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.time import Time
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tests.integration_tests.measure_collision_zones import Measurement, most_recent_before


def seconds(stamp):
    """Return a ROS stamp as seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class RecoveryProbe(Measurement):
    """Observe each command stage while commanding only a capped reverse escape."""

    def __init__(self):
        """Add recovery and scan-health taps to the existing zone survey node."""
        super().__init__()
        self.scan_wall = None
        self.recovered_commands = []
        self.scan_health_commands = []
        self.reset = self.create_publisher(Bool, "/estop/reset", 10)
        self.create_subscription(Twist, "/cmd_vel/recovery_limited", self.on_recovered, 10)
        self.create_subscription(
            Twist, "/cmd_vel/scan_health_limited", self.on_scan_health, 10
        )

    def on_scan(self, message):
        """Keep the last scan and its wall receipt time for the freshness gate."""
        super().on_scan(message)
        self.scan_wall = time.monotonic()

    def on_recovered(self, message):
        """Record the recovery output with contemporaneous stop-square occupancy."""
        points = None
        if self.scan is not None and self.tf_buffer.can_transform(
            "base_link", self.scan.header.frame_id, Time()
        ):
            points = self.zone_state()["stop_zone_points"]
        self.recovered_commands.append((time.monotonic(), message.linear.x, points))

    def on_scan_health(self, message):
        """Record the command entering the final e-stop gate."""
        self.scan_health_commands.append((time.monotonic(), message.linear.x))

    def rear_box_points(self):
        """Count returns in the recovery node's one-metre rear box."""
        return sum(
            -1.0 <= x <= 0.0 and abs(y) <= 0.65
            for x, y in self.scan_points_in_base()
        )

    def chain_ready(self):
        """Require each software command stage and Isaac's sole actuator subscriber."""
        expected = {
            "/cmd_vel": ({"collision_zone_measurement"}, {"collision_monitor", "stop_zone_recovery", "collision_zone_measurement"}),
            "/cmd_vel/collision_limited": ({"collision_monitor"}, {"stop_zone_recovery", "collision_zone_measurement"}),
            "/cmd_vel/recovery_limited": ({"stop_zone_recovery"}, {"scan_health_gate", "collision_zone_measurement"}),
            "/cmd_vel/scan_health_limited": ({"scan_health_gate"}, {"estop_gate", "collision_zone_measurement"}),
            "/cmd_vel/safety_limited": ({"estop_gate"}, {"_World_JetAutoROS_CmdVelSub", "collision_zone_measurement"}),
        }
        for topic, (publishers, subscribers) in expected.items():
            actual_publishers = {
                item.node_name for item in self.get_publishers_info_by_topic(topic)
            }
            actual_subscribers = {
                item.node_name for item in self.get_subscriptions_info_by_topic(topic)
            }
            if actual_publishers != publishers or actual_subscribers != subscribers:
                return False
        return True

    def assert_fresh(self):
        """Abort when the scan, clock, or moving sensor transform becomes unusable."""
        assert self.clock_message is not None and self.scan is not None
        assert self.scan_wall is not None and time.monotonic() - self.scan_wall < 0.5
        age = seconds(self.clock_message.clock) - seconds(self.scan.header.stamp)
        assert -0.1 < age < 0.3, f"scan age {age:.3f} s"
        assert self.tf_buffer.can_transform("odom", "base_link", Time())
        assert self.tf_buffer.can_transform(
            "base_link", self.scan.header.frame_id, Time()
        )


def run_reverse(node, duration_s):
    """Publish pure reverse under independent travel, proximity, and time limits."""
    start = node.odom.pose.pose.position
    start_xy = (start.x, start.y)
    start_clock = seconds(node.clock_message.clock)
    start_wall = time.monotonic()
    command = Twist()
    command.linear.x = -0.06
    try:
        while time.monotonic() - start_wall < duration_s:
            node.assert_fresh()
            assert node.rear_box_points() == 0, "rear box became occupied"
            assert node.zone_state()["nearest_ahead_m"] > 0.40, "front proximity abort"
            pose = node.odom.pose.pose.position
            travel = math.hypot(pose.x - start_xy[0], pose.y - start_xy[1])
            assert travel < 0.16, "reverse travel cap exceeded"
            node.command.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.command.publish(Twist())
        node.latch_estop()
    end = node.odom.pose.pose.position
    return {
        "odom_reverse_m": start_xy[0] - end.x,
        "odom_travel_m": math.hypot(end.x - start_xy[0], end.y - start_xy[1]),
        "sim_elapsed_s": seconds(node.clock_message.clock) - start_clock,
        "wall_elapsed_s": time.monotonic() - start_wall,
    }


def verify_latched_stop(node):
    """Demand nonzero input to a latched e-stop and zero final output and drift."""
    start = node.odom.pose.pose.position
    start_xy = (start.x, start.y)
    input_start = len(node.scan_health_commands)
    output_start = len(node.final_commands)
    command = Twist()
    command.linear.x = -0.06
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        node.assert_fresh()
        node.command.publish(command)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.command.publish(Twist())
    node.spin_for(0.4)
    inputs = [speed for _, speed in node.scan_health_commands[input_start:]]
    outputs = [speed for _, speed in node.final_commands[output_start:]]
    end = node.odom.pose.pose.position
    drift = math.hypot(end.x - start_xy[0], end.y - start_xy[1])
    assert any(speed < -0.005 for speed in inputs), "no nonzero input challenged e-stop"
    assert len(outputs) >= 3 and all(abs(speed) < 1e-6 for speed in outputs)
    assert drift < 0.004, f"late drift {drift:.4f} m"
    return {"nonzero_input_count": sum(speed < -0.005 for speed in inputs),
            "blocked_output_count": len(outputs), "late_drift_m": drift}


def main():
    """Validate a short live Isaac recovery from a verified occupied stop zone."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    parser.add_argument("--duration-s", type=float, default=2.5)
    args = parser.parse_args()
    if not args.isaac_simulation_only or not 0.5 <= args.duration_s <= 3.0:
        parser.error("explicit Isaac-only flag and 0.5-3.0 s duration required")

    rclpy.init()
    node = RecoveryProbe()
    try:
        node.spin_for(5.0)
        assert node.odom is not None
        assert len(node.get_publishers_info_by_topic("/clock")) == 1
        assert "base_driver" not in node.get_node_names()
        assert node.chain_ready(), "software command graph incomplete"
        node.assert_fresh()
        before = node.zone_state()
        assert before["stop_zone_points"] > 3, "start is outside occupied stop zone"
        assert before["nearest_ahead_m"] > 0.40, "front too close"
        assert node.rear_box_points() == 0, "rear is not clear"

        node.reset.publish(Bool(data=True))
        node.spin_for(0.2)
        result = run_reverse(node, args.duration_s)
        evidence = [
            (speed, points, most_recent_before(node.limited_commands, stamp))
            for stamp, speed, points in node.recovered_commands
            if points is not None and points > 3 and speed < -0.045
        ]
        assert any(abs(limited) < 1e-6 for _, _, limited in evidence if limited is not None), (
            "no capped reverse while collision monitor held zero"
        )
        assert 0.02 < result["odom_reverse_m"] < 0.16, result
        after = node.zone_state()
        assert after["stop_zone_points"] <= 3, "reverse did not clear stop square"
        stop = verify_latched_stop(node)
        print(json.dumps({"stop_zone_recovery": {
            "before": before, "after": after, "capped_recovery_samples_in_stop": len(evidence),
            **result, "estop": stop,
        }}, indent=2), flush=True)
    finally:
        node.latch_estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
