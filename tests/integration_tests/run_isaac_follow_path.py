#!/usr/bin/env python3
"""Bounded, simulation-only Nav2 plan and optional FollowPath over the safety chain."""

import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def closest_scan_range(scan):
    """Return the nearest valid LiDAR range, or zero when a scan is unusable."""
    return min(
        (
            value for value in scan.ranges
            if math.isfinite(value) and scan.range_min <= value < scan.range_max
        ),
        default=0.0,
    )


def wait_future(node, future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("Nav2 action call timed out")
    return future.result()


class Runner(Node):
    def __init__(self):
        """Subscribe to live state and prepare bounded Nav2 action clients."""
        super().__init__("isaac_follow_path_validation")
        self.clock_message = None
        self.odom = None
        self.final_commands = []
        self.pose_count = 0
        self.scan_count = 0
        self.scan_stamp_s = None
        self.latest_scan_range_m = 0.0
        self.min_scan_range_m = math.inf
        self.grid = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "/follow_path")
        self.estop = self.create_publisher(Bool, "/estop/engaged", 10)
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Odometry, "/odom/wheel", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel/safety_limited", self.on_command, 10)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_grid, costmap_qos)

    def on_clock(self, message):
        self.clock_message = message

    def on_odom(self, message):
        self.odom = message

    def on_command(self, message):
        self.final_commands.append(
            (time.monotonic(), message.linear.x, message.linear.y, message.angular.z)
        )

    def on_pose(self, _):
        self.pose_count += 1

    def on_scan(self, message):
        """Track scan freshness and the nearest return for the independent abort."""
        self.scan_count += 1
        self.scan_stamp_s = stamp_seconds(message.header.stamp)
        self.latest_scan_range_m = closest_scan_range(message)
        self.min_scan_range_m = min(self.min_scan_range_m, self.latest_scan_range_m)

    def on_grid(self, message):
        self.grid = message

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def map_pose(self):
        transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        return transform.transform.translation.x, transform.transform.translation.y

    def cost(self, x, y):
        info = self.grid.info
        column = math.floor((x - info.origin.position.x) / info.resolution)
        row = math.floor((y - info.origin.position.y) / info.resolution)
        assert 0 <= column < info.width and 0 <= row < info.height, (x, y)
        return self.grid.data[row * info.width + column]

    def stop(self):
        # Software e-stop remains latched after this test; reset deliberately
        # before another simulation run.
        for _ in range(6):
            self.estop.publish(Bool(data=True))
            self.command.publish(Twist())
            self.spin_for(0.05)
        self.spin_for(0.6)
        stopped = self.odom.pose.pose.position
        stopped_xy = (stopped.x, stopped.y)
        output_start = len(self.final_commands)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            command = Twist()
            command.linear.x = 0.05
            self.command.publish(command)
            self.spin_for(0.05)
        last = self.odom.pose.pose.position
        stop_drift = math.hypot(last.x - stopped_xy[0], last.y - stopped_xy[1])
        blocked = [entry[1] for entry in self.final_commands[output_start:]]
        print(
            json.dumps(
                {"estop_check": {"late_drift_m": stop_drift, "blocked_command_count": len(blocked)}}
            ),
            flush=True,
        )
        assert stop_drift < 0.004 and len(blocked) >= 3 and all(
            abs(speed) < 1e-6 for speed in blocked
        )


def command_chain_ready(node):
    """Check every safety command stage and the sole Isaac actuator subscriber."""
    expected = {
        "/cmd_vel": (None, {"collision_monitor", "stop_zone_recovery"}),
        "/cmd_vel/collision_limited": ("collision_monitor", {"stop_zone_recovery"}),
        "/cmd_vel/recovery_limited": ("stop_zone_recovery", {"scan_health_gate"}),
        "/cmd_vel/scan_health_limited": ("scan_health_gate", {"estop_gate"}),
        "/cmd_vel/safety_limited": ("estop_gate", {"_World_JetAutoROS_CmdVelSub", "isaac_follow_path_validation"}),
    }
    for topic, (publisher, subscribers) in expected.items():
        observed_subscribers = {
            endpoint.node_name for endpoint in node.get_subscriptions_info_by_topic(topic)
        }
        if observed_subscribers != subscribers:
            return False
        if publisher is not None:
            observed_publishers = {
                endpoint.node_name for endpoint in node.get_publishers_info_by_topic(topic)
            }
            if observed_publishers != {publisher}:
                return False
    return True


def verify_start_pose(actual_xy, expected_xy, tolerance_m=0.15):
    """Reject a route when the live map pose disagrees with the scene preflight."""
    if not all(math.isfinite(value) for value in (*actual_xy, *expected_xy)):
        raise ValueError("start poses must be finite")
    residual = math.hypot(actual_xy[0] - expected_xy[0], actual_xy[1] - expected_xy[1])
    if residual > tolerance_m:
        raise ValueError(
            f"map start residual {residual:.3f} m exceeds {tolerance_m:.3f} m "
            f"(actual={actual_xy}, expected={expected_xy})"
        )
    return residual


def main():
    """Plan and optionally execute a bounded Isaac-only Nav2 route."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    parser.add_argument("--execute", action="store_true", help="Send the planned path to Nav2")
    parser.add_argument("--verify-stopped", action="store_true", help="Check the latched stop only")
    parser.add_argument("--distance-m", type=float, default=0.40)
    parser.add_argument("--lateral-m", type=float, default=0.0)
    parser.add_argument("--expected-start-map-x", type=float)
    parser.add_argument("--expected-start-map-y", type=float)
    args = parser.parse_args()
    if not args.isaac_simulation_only or not 0.30 <= args.distance_m <= 0.50:
        raise SystemExit("Isaac-only flag and a 0.30–0.50 m forward route are required")
    if not math.isfinite(args.lateral_m) or abs(args.lateral_m) > 0.20:
        parser.error("--lateral-m must be within ±0.20 m")
    if args.execute and (args.expected_start_map_x is None or args.expected_start_map_y is None):
        parser.error("--execute requires an independently verified expected map start x and y")

    rclpy.init()
    node = Runner()
    started_motion = False
    try:
        node.spin_for(3.0)
        assert node.clock_message is not None and node.odom is not None and node.grid is not None
        if args.verify_stopped:
            node.stop()
            return
        assert len(node.get_publishers_info_by_topic("/clock")) == 1
        chain_deadline = time.monotonic() + 15.0
        while not command_chain_ready(node) and time.monotonic() < chain_deadline:
            node.spin_for(0.2)
        assert command_chain_ready(node), "safety command chain is incomplete"
        assert "base_driver" not in node.get_node_names()
        assert node.planner.wait_for_server(timeout_sec=5.0)
        assert node.controller.wait_for_server(timeout_sec=5.0)

        transform_deadline = time.monotonic() + 15.0
        while not node.tf_buffer.can_transform("map", "base_link", Time()):
            if time.monotonic() >= transform_deadline:
                raise TimeoutError("fresh map-to-base transform unavailable")
            node.spin_for(0.2)
        start_x, start_y = node.map_pose()
        if args.execute:
            verify_start_pose(
                (start_x, start_y), (args.expected_start_map_x, args.expected_start_map_y)
            )
            assert node.scan_stamp_s is not None, "no live scan at start"
            scan_age = stamp_seconds(node.clock_message.clock) - node.scan_stamp_s
            assert -0.25 < scan_age < 0.5, f"stale scan at start: {scan_age:.3f} s"
            assert node.latest_scan_range_m > 0.55, "start zone is not clear"
        goal_x, goal_y = start_x + args.distance_m, start_y + args.lateral_m
        start_cost, goal_cost = node.cost(start_x, start_y), node.cost(goal_x, goal_y)
        assert 0 <= start_cost < 90 and 0 <= goal_cost < 90, (start_cost, goal_cost)

        goal = ComputePathToPose.Goal()
        goal.planner_id = "GridBased"
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = node.clock_message.clock
        goal.goal.pose.position.x = goal_x
        goal.goal.pose.position.y = goal_y
        goal.goal.pose.orientation.w = 1.0
        plan_handle = wait_future(node, node.planner.send_goal_async(goal), 10.0)
        assert plan_handle.accepted, "planner rejected short path"
        plan_result = wait_future(node, plan_handle.get_result_async(), 15.0)
        assert plan_result.status == GoalStatus.STATUS_SUCCEEDED, plan_result.status
        path = plan_result.result.path
        assert len(path.poses) >= 2
        endpoint = path.poses[-1].pose.position
        assert math.hypot(endpoint.x - goal_x, endpoint.y - goal_y) < 0.10
        length = sum(
            math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y,
            )
            for a, b in zip(path.poses, path.poses[1:])
        )
        costs = [node.cost(p.pose.position.x, p.pose.position.y) for p in path.poses]
        assert 0.20 < length < 0.85, length
        assert all(0 <= cost < 99 for cost in costs), max(costs)
        report = {
            "start_map_xy_m": [start_x, start_y],
            "goal_map_xy_m": [goal_x, goal_y],
            "costmap_start_goal": [start_cost, goal_cost],
            "plan_poses": len(path.poses),
            "plan_length_m": length,
            "plan_max_cost": max(costs),
        }
        print(json.dumps({"plan": report}, indent=2), flush=True)
        if not args.execute:
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = "FollowPath"
        follow_goal.goal_checker_id = "general_goal_checker"
        initial_odom = node.odom.pose.pose.position
        initial_xy = (initial_odom.x, initial_odom.y)
        command_start = len(node.final_commands)
        pose_start = node.pose_count
        scan_start = node.scan_count
        node.min_scan_range_m = math.inf
        wall_start = time.monotonic()
        sim_start = stamp_seconds(node.clock_message.clock)
        follow_handle = wait_future(node, node.controller.send_goal_async(follow_goal), 10.0)
        assert follow_handle.accepted, "controller rejected path"
        started_motion = True
        follow_future = follow_handle.get_result_async()
        deadline = time.monotonic() + 50.0
        abort_reason = None
        while not follow_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            latest = node.odom.pose.pose.position
            if math.hypot(latest.x - initial_xy[0], latest.y - initial_xy[1]) > 0.65:
                abort_reason = "odometry travel exceeded 0.65 m"
                break
            if node.latest_scan_range_m <= 0.55:
                abort_reason = "LiDAR proximity fell below 0.55 m"
                break
            scan_age = stamp_seconds(node.clock_message.clock) - node.scan_stamp_s
            if not -0.25 < scan_age < 0.5:
                abort_reason = f"LiDAR scan became stale: {scan_age:.3f} s"
                break
        if abort_reason or not follow_future.done():
            if not follow_future.done():
                wait_future(node, follow_handle.cancel_goal_async(), 5.0)
            raise TimeoutError(abort_reason or "FollowPath timed out")
        follow_result = follow_future.result()
        node.spin_for(1.0)
        final = node.odom.pose.pose.position
        travel = math.hypot(final.x - initial_xy[0], final.y - initial_xy[1])
        settled_x, settled_y = node.map_pose()
        speeds = [abs(entry[1]) for entry in node.final_commands[command_start:]]
        angular_speeds = [abs(entry[3]) for entry in node.final_commands[command_start:]]
        report.update(
            {
                "follow_status": follow_result.status,
                "odom_travel_m": travel,
                "odom_start_xy_m": initial_xy,
                "odom_end_xy_m": [final.x, final.y],
                "settled_map_xy_m": [settled_x, settled_y],
                "settled_map_goal_residual_m": math.hypot(goal_x - settled_x, goal_y - settled_y),
                "max_final_linear_x_mps": max(speeds, default=0.0),
                "max_final_angular_z_radps": max(angular_speeds, default=0.0),
                "min_scan_range_during_follow_m": node.min_scan_range_m,
                "pose_updates_during_follow": node.pose_count - pose_start,
                "scans_during_follow": node.scan_count - scan_start,
                "sim_clock_s": stamp_seconds(node.clock_message.clock),
                "sim_elapsed_s": stamp_seconds(node.clock_message.clock) - sim_start,
                "wall_elapsed_s": time.monotonic() - wall_start,
            }
        )
        print(json.dumps({"execution": report}, indent=2), flush=True)
        assert follow_result.status == GoalStatus.STATUS_SUCCEEDED
        assert 0.15 < travel < 0.65
        assert speeds and max(speeds) <= 0.12
        assert node.pose_count > pose_start
        assert node.scan_count - scan_start >= 10
    finally:
        if started_motion:
            node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
