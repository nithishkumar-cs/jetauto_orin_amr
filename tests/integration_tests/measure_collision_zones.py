#!/usr/bin/env python3
"""Measure where the collision monitor actually stops a moving simulated base.

The stop (0.55 m) and slowdown (1.25 m) squares in collision_monitor.yaml were
carried over from a deleted radial design and have never been measured against
a moving robot. This drives a hand-built straight path toward a real obstacle
and records which layer stops the base and at what separation.

The path is built here rather than requested from the planner on purpose: the
global planner routes *around* obstacles, so it cannot produce the approach
this test needs. That makes the collision monitor, the controller's own
collision checking, and the goal checker the only things that can stop motion,
and comparing /cmd_vel against /cmd_vel/collision_limited says which one did.

Isaac-only. Two independent bounds keep the simulated base off the obstacle: a
travel cap and an abort on measured obstacle proximity. Both are checked every
scan, and either one engages the software e-stop.
"""

import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

# collision_monitor.yaml: square half-extents and the "more than N points
# inside" trigger. Mirrored here only to predict and explain the result.
STOP_HALF_EXTENT_M = 0.55
SLOW_HALF_EXTENT_M = 1.25
SLOWDOWN_RATIO = 0.35
MAX_POINTS = 3


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


class Measurement(Node):
    def __init__(self):
        super().__init__("collision_zone_measurement")
        self.clock_message = None
        self.odom = None
        self.scan = None
        self.controller_commands = []
        self.limited_commands = []
        self.final_commands = []
        self.monitor_logs = []
        self.samples = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.controller = ActionClient(self, FollowPath, "/follow_path")
        self.estop = self.create_publisher(Bool, "/estop/engaged", 10)
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Odometry, "/odom/wheel", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_controller_command, 10)
        self.create_subscription(
            Twist, "/cmd_vel/collision_limited", self.on_limited_command, 10
        )
        self.create_subscription(Twist, "/cmd_vel/safety_limited", self.on_final_command, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)

    def on_clock(self, message):
        self.clock_message = message

    def on_odom(self, message):
        self.odom = message

    def on_scan(self, message):
        self.scan = message

    def on_controller_command(self, message):
        self.controller_commands.append((time.monotonic(), message.linear.x))

    def on_limited_command(self, message):
        self.limited_commands.append((time.monotonic(), message.linear.x))

    def on_final_command(self, message):
        self.final_commands.append((time.monotonic(), message.linear.x))

    def on_log(self, message):
        if message.name.endswith("collision_monitor"):
            self.monitor_logs.append((time.monotonic(), message.msg))

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def map_pose(self):
        transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        translation = transform.transform.translation
        return translation.x, translation.y, yaw_of(transform.transform.rotation)

    def scan_points_in_base(self):
        """Current scan as (x, y) in base_link, mirroring the monitor's frame."""
        scan = self.scan
        transform = self.tf_buffer.lookup_transform("base_link", scan.header.frame_id, Time())
        offset = transform.transform.translation
        rotation = yaw_of(transform.transform.rotation)
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment + rotation
            points.append((offset.x + distance * math.cos(angle),
                           offset.y + distance * math.sin(angle)))
        return points

    def zone_state(self):
        points = self.scan_points_in_base()
        in_stop = [p for p in points if abs(p[0]) <= STOP_HALF_EXTENT_M
                   and abs(p[1]) <= STOP_HALF_EXTENT_M]
        in_slow = [p for p in points if abs(p[0]) <= SLOW_HALF_EXTENT_M
                   and abs(p[1]) <= SLOW_HALF_EXTENT_M]
        ahead = [p for p in points if p[0] > 0.0 and abs(p[1]) <= STOP_HALF_EXTENT_M]
        nearest_ahead = min((math.hypot(*p) for p in ahead), default=float("inf"))
        nearest_any = min((math.hypot(*p) for p in points), default=float("inf"))
        return {
            "stop_zone_points": len(in_stop),
            "slow_zone_points": len(in_slow),
            "nearest_ahead_m": nearest_ahead,
            "nearest_any_m": nearest_any,
        }

    def straight_path(self, distance_m, spacing_m=0.05):
        start_x, start_y, yaw = self.map_pose()
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.clock_message.clock
        steps = max(2, int(round(distance_m / spacing_m)) + 1)
        for index in range(steps):
            travelled = min(distance_m, index * spacing_m)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.clock_message.clock
            pose.pose.position.x = start_x + travelled * math.cos(yaw)
            pose.pose.position.y = start_y + travelled * math.sin(yaw)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path, (start_x, start_y, yaw)

    def latch_estop(self):
        for _ in range(6):
            self.estop.publish(Bool(data=True))
            self.command.publish(Twist())
            self.spin_for(0.05)
        self.spin_for(0.4)


def most_recent_before(series, when):
    candidates = [value for stamp, value in series if stamp <= when]
    return candidates[-1] if candidates else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    parser.add_argument("--distance-m", type=float, default=1.60)
    parser.add_argument("--travel-cap-m", type=float, default=1.40)
    parser.add_argument("--abort-range-m", type=float, default=0.45)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument(
        "--survey-only", action="store_true", help="Report zone occupancy without moving"
    )
    args = parser.parse_args()
    if not args.isaac_simulation_only:
        raise SystemExit("this test is simulation-only")
    if not 0.30 <= args.distance_m <= 2.00:
        raise SystemExit("route must be 0.30-2.00 m")
    if not 0.30 <= args.travel_cap_m <= 1.60:
        raise SystemExit("travel cap must be 0.30-1.60 m")
    if args.abort_range_m < 0.40:
        raise SystemExit("abort range below 0.40 m leaves no clearance")

    rclpy.init()
    node = Measurement()
    started_motion = False
    try:
        node.spin_for(3.0)
        assert node.clock_message is not None, "no /clock"
        assert node.odom is not None, "no /odom/wheel"
        assert node.scan is not None, "no /scan"
        assert len(node.get_publishers_info_by_topic("/clock")) == 1
        final_subscribers = node.get_subscriptions_info_by_topic("/cmd_vel/safety_limited")
        assert len(final_subscribers) == 2 and any(
            "CmdVelSub" in endpoint.node_name for endpoint in final_subscribers
        ), "Isaac command subscriber missing from the final safety topic"
        assert "base_driver" not in node.get_node_names(), "a driver is running"
        assert node.controller.wait_for_server(timeout_sec=5.0)

        before = node.zone_state()
        if args.survey_only:
            start_x, start_y, start_yaw = node.map_pose()
            print(
                json.dumps(
                    {
                        "survey": {
                            "map_xy_m": [start_x, start_y],
                            "yaw_rad": start_yaw,
                            **before,
                        }
                    },
                    indent=2,
                ),
                flush=True,
            )
            return
        assert before["stop_zone_points"] <= MAX_POINTS, (
            f"stop zone already occupied by {before['stop_zone_points']} points; "
            "the robot cannot move and the test would prove nothing"
        )
        assert before["nearest_ahead_m"] > args.abort_range_m, "obstacle already too close"

        path, start = node.straight_path(args.distance_m)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        initial = node.odom.pose.pose.position
        initial_xy = (initial.x, initial.y)
        handle_future = node.controller.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while not handle_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        handle = handle_future.result()
        assert handle is not None and handle.accepted, "controller rejected the straight path"
        started_motion = True
        result_future = handle.get_result_async()

        abort_reason = None
        deadline = time.monotonic() + args.timeout_s
        last_scan_stamp = None
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            position = node.odom.pose.pose.position
            travel = math.hypot(position.x - initial_xy[0], position.y - initial_xy[1])
            stamp = node.scan.header.stamp
            if (stamp.sec, stamp.nanosec) != last_scan_stamp:
                last_scan_stamp = (stamp.sec, stamp.nanosec)
                state = node.zone_state()
                now = time.monotonic()
                state.update(
                    {
                        "t": now,
                        "travel_m": travel,
                        "controller_x": most_recent_before(node.controller_commands, now),
                        "limited_x": most_recent_before(node.limited_commands, now),
                        "final_x": most_recent_before(node.final_commands, now),
                    }
                )
                node.samples.append(state)
                if state["nearest_ahead_m"] < args.abort_range_m:
                    abort_reason = (
                        f"obstacle within {args.abort_range_m} m "
                        f"({state['nearest_ahead_m']:.3f} m) - nothing stopped the base"
                    )
                    break
            if travel > args.travel_cap_m:
                abort_reason = f"travel cap {args.travel_cap_m} m exceeded ({travel:.3f} m)"
                break

        if abort_reason is not None:
            node.latch_estop()
            if not result_future.done():
                cancel = handle.cancel_goal_async()
                end = time.monotonic() + 5.0
                while not cancel.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
            status = None
        elif result_future.done():
            status = result_future.result().status
        else:
            abort_reason = f"timed out after {args.timeout_s} s"
            status = None

        node.spin_for(1.0)
        final_position = node.odom.pose.pose.position
        travel = math.hypot(final_position.x - initial_xy[0], final_position.y - initial_xy[1])
        after = node.zone_state()

        # The monitor intervened wherever it forwarded less than the controller
        # asked for. Pair each limited sample with the controller sample in
        # effect at that moment.
        stop_event = None
        slow_events = []
        for sample in node.samples:
            controller_x, limited_x = sample["controller_x"], sample["limited_x"]
            if controller_x is None or limited_x is None or controller_x <= 1e-6:
                continue
            ratio = limited_x / controller_x
            if limited_x <= 1e-6 and stop_event is None:
                stop_event = sample
            elif 0.05 < ratio < 0.95:
                slow_events.append((sample, ratio))

        report = {
            "configured_zones": {
                "stop_half_extent_m": STOP_HALF_EXTENT_M,
                "slow_half_extent_m": SLOW_HALF_EXTENT_M,
                "slowdown_ratio": SLOWDOWN_RATIO,
                "max_points": MAX_POINTS,
            },
            "route": {
                "requested_m": args.distance_m,
                "start_map_xy_m": [start[0], start[1]],
                "start_yaw_rad": start[2],
                "path_poses": len(path.poses),
            },
            "before": before,
            "after": after,
            "odom_travel_m": travel,
            "follow_status": status,
            "abort_reason": abort_reason,
            "scan_samples": len(node.samples),
            "max_controller_x_mps": max((v for _, v in node.controller_commands), default=0.0),
            "max_limited_x_mps": max((v for _, v in node.limited_commands), default=0.0),
            "max_final_x_mps": max((v for _, v in node.final_commands), default=0.0),
            "monitor_log_lines": [message for _, message in node.monitor_logs][-10:],
        }
        if stop_event is not None:
            report["monitor_stop"] = {
                "travel_m": stop_event["travel_m"],
                "nearest_ahead_m": stop_event["nearest_ahead_m"],
                "nearest_any_m": stop_event["nearest_any_m"],
                "stop_zone_points": stop_event["stop_zone_points"],
                "controller_x_mps": stop_event["controller_x"],
            }
        if slow_events:
            ratios = [ratio for _, ratio in slow_events]
            report["monitor_slowdown"] = {
                "events": len(slow_events),
                "first_travel_m": slow_events[0][0]["travel_m"],
                "first_nearest_ahead_m": slow_events[0][0]["nearest_ahead_m"],
                "median_ratio": sorted(ratios)[len(ratios) // 2],
                "min_ratio": min(ratios),
                "max_ratio": max(ratios),
            }
        print(json.dumps({"collision_zone_measurement": report}, indent=2), flush=True)
    finally:
        if started_motion:
            node.latch_estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
