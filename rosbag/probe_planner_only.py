#!/usr/bin/env python3
"""Send a read-only ComputePathToPose request to a running planner-only stack."""

import argparse
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient


def pose(x, y):
    result = PoseStamped()
    result.header.frame_id = "map"
    result.pose.position.x = x
    result.pose.position.y = y
    result.pose.orientation.w = 1.0
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    start_mode = parser.add_mutually_exclusive_group(required=True)
    start_mode.add_argument("--start", nargs=2, type=float)
    start_mode.add_argument(
        "--live-start", action="store_true", help="let Nav2 use the current robot TF pose"
    )
    parser.add_argument("--goal", nargs=2, type=float, required=True)
    parser.add_argument("--expect", choices=("success", "failure"))
    parser.add_argument("--inspect-costmap", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("planner_only_probe")
    try:
        client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/compute_path_to_pose is unavailable")

        request = ComputePathToPose.Goal()
        if args.start is not None:
            request.start = pose(*args.start)
        request.goal = pose(*args.goal)
        request.planner_id = "GridBased"
        request.use_start = not args.live_start
        send = client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
        if not send.done() or not send.result().accepted:
            raise RuntimeError("planner did not accept goal")

        future = send.result().get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if not future.done():
            raise RuntimeError("planner did not return a result")

        response = future.result()
        path = response.result.path
        length = sum(
            math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
            for a, b in zip(path.poses, path.poses[1:])
        )
        print(
            f"status={response.status} succeeded={response.status == GoalStatus.STATUS_SUCCEEDED} "
            f"start_source={'live_tf' if args.live_start else 'explicit'} "
            f"poses={len(path.poses)} "
            f"length_m={length:.3f} frame={path.header.frame_id}"
        )
        if path.poses:
            first = path.poses[0].pose.position
            print(
                f"path_start=({first.x:.3f},{first.y:.3f}) "
                f"path_stamp={path.header.stamp.sec}.{path.header.stamp.nanosec:09d}"
            )
        if args.expect is not None:
            succeeded = response.status == GoalStatus.STATUS_SUCCEEDED
            if succeeded != (args.expect == "success"):
                raise RuntimeError(f"expected {args.expect}, got status {response.status}")

        if args.inspect_costmap:
            costmap_client = node.create_client(GetCostmap, "/global_costmap/get_costmap")
            if not costmap_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("global costmap service is unavailable")
            costmap_future = costmap_client.call_async(GetCostmap.Request())
            rclpy.spin_until_future_complete(node, costmap_future, timeout_sec=10.0)
            if not costmap_future.done():
                raise RuntimeError("global costmap did not return a snapshot")
            grid = costmap_future.result().map
            print(
                f"costmap={grid.metadata.size_x}x{grid.metadata.size_y} "
                f"resolution_m={grid.metadata.resolution:.3f} "
                f"lethal_cells={sum(cost == 254 for cost in grid.data)} "
                f"unknown_cells={sum(cost == 255 for cost in grid.data)}"
            )
            if path.poses:
                path_costs = []
                for stamped in path.poses:
                    column = math.floor(
                        (stamped.pose.position.x - grid.metadata.origin.position.x)
                        / grid.metadata.resolution
                    )
                    row = math.floor(
                        (stamped.pose.position.y - grid.metadata.origin.position.y)
                        / grid.metadata.resolution
                    )
                    if not (0 <= column < grid.metadata.size_x and 0 <= row < grid.metadata.size_y):
                        raise RuntimeError("path leaves the global costmap")
                    path_costs.append(grid.data[row * grid.metadata.size_x + column])
                print(
                    f"max_path_cost={max(path_costs)} "
                    f"lethal_or_unknown_path_poses={sum(cost >= 254 for cost in path_costs)}"
                )
                if any(cost >= 254 for cost in path_costs):
                    raise RuntimeError("path crosses lethal or unknown costmap cells")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
