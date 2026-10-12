#!/usr/bin/env python3
"""Compare diagnostic Karto loop-match transforms with reference odometry.

The mapper must be built with the optional loop-closure probe patch. A small
distance between scan centers is *not* required for a valid match: this tool
compares the transform proposed by scan matching with the true transform.
"""

import argparse
import bisect
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

from analyze_saved_map_localization import read_odometry


NUMBER = r"[-+\w.eE]+"
MATCH_PATTERN = re.compile(
    r"KARTO_LOOP_MATCH scan_id=(?P<current_id>\d+) "
    rf"scan_time=(?P<current_time>{NUMBER}) "
    r"closest_id=(?P<closest_id>\d+) "
    rf"closest_time=(?P<closest_time>{NUMBER}) "
    rf"fine_response=(?P<fine_response>{NUMBER}) "
    rf"prior_sensor_pose=(?P<prior_x>{NUMBER}) (?P<prior_y>{NUMBER}) (?P<prior_yaw>{NUMBER}) "
    rf"matched_sensor_pose=(?P<matched_x>{NUMBER}) (?P<matched_y>{NUMBER}) (?P<matched_yaw>{NUMBER}) "
    rf"closest_sensor_pose=(?P<closest_x>{NUMBER}) (?P<closest_y>{NUMBER}) (?P<closest_yaw>{NUMBER})"
)
GATE_PATTERN = re.compile(
    r"KARTO_LOOP_LINK_GATE scan_id=(?P<current_id>\d+) "
    r"closest_id=(?P<closest_id>\d+) "
    rf"squared_distance=(?P<squared_distance>{NUMBER}) "
    rf"limit_squared=(?P<limit_squared>{NUMBER}) passed=(?P<passed>[01])"
)
EDGE_PATTERN = re.compile(
    r"KARTO_GRAPH_EDGE_CREATED source_id=(?P<source_id>\d+) "
    r"target_id=(?P<target_id>\d+)"
)
FINISHED_PATTERN = re.compile(
    r"KARTO_FINE_ACCEPTED_LOOP_CANDIDATE scan_id=(?P<current_id>\d+)"
)


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(origin, target):
    dx = target.x - origin.x
    dy = target.y - origin.y
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    return Pose2(
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        wrap_angle(target.yaw - origin.yaw),
    )


def sensor_pose(base, offset):
    cosine = math.cos(base.yaw)
    sine = math.sin(base.yaw)
    return Pose2(
        base.x + cosine * offset.x - sine * offset.y,
        base.y + sine * offset.x + cosine * offset.y,
        wrap_angle(base.yaw + offset.yaw),
    )


def position_error(first, second):
    return math.hypot(first.x - second.x, first.y - second.y)


def pose_from_match(groups, prefix):
    return Pose2(*(float(groups[f"{prefix}_{axis}"]) for axis in ("x", "y", "yaw")))


def parse_events(log_text):
    events = []
    pending = None
    active_link_attempt = None
    saw_edge_probe = False
    for line in log_text.splitlines():
        match = MATCH_PATTERN.search(line)
        if match:
            if pending is not None:
                raise ValueError("Loop match without subsequent link-gate result")
            if active_link_attempt is not None:
                raise ValueError("Link attempt without completion marker")
            groups = match.groupdict()
            pending = {
                "current_id": int(groups["current_id"]),
                "current_time": float(groups["current_time"]),
                "closest_id": int(groups["closest_id"]),
                "closest_time": float(groups["closest_time"]),
                "fine_response": float(groups["fine_response"]),
                "prior": pose_from_match(groups, "prior"),
                "matched": pose_from_match(groups, "matched"),
                "closest": pose_from_match(groups, "closest"),
            }
            continue
        gate = GATE_PATTERN.search(line)
        if gate:
            if pending is None:
                raise ValueError("Link-gate result without preceding loop match")
            groups = gate.groupdict()
            if (
                int(groups["current_id"]) != pending["current_id"]
                or int(groups["closest_id"]) != pending["closest_id"]
            ):
                raise ValueError("Loop match and link-gate scan IDs differ")
            pending["gate_distance_m"] = math.sqrt(float(groups["squared_distance"]))
            pending["gate_limit_m"] = math.sqrt(float(groups["limit_squared"]))
            pending["gate_passed"] = groups["passed"] == "1"
            pending["new_graph_edge"] = None
            events.append(pending)
            active_link_attempt = pending
            pending = None
            continue
        edge = EDGE_PATTERN.search(line)
        if edge:
            saw_edge_probe = True
            if active_link_attempt is not None and (
                int(edge["source_id"]) == active_link_attempt["closest_id"]
                and int(edge["target_id"]) == active_link_attempt["current_id"]
            ):
                active_link_attempt["new_graph_edge"] = True
            continue
        finished = FINISHED_PATTERN.search(line)
        if finished and active_link_attempt is not None:
            if int(finished["current_id"]) != active_link_attempt["current_id"]:
                raise ValueError("Link completion marker scan ID differs")
            active_link_attempt = None
    if pending is not None:
        raise ValueError("Loop match without subsequent link-gate result")
    if active_link_attempt is not None:
        raise ValueError("Link attempt without completion marker")
    if saw_edge_probe:
        for event in events:
            if event["new_graph_edge"] is None:
                event["new_graph_edge"] = False
    return events


def nearest_truth(odometry, stamps, stamp, max_skew_s):
    index = bisect.bisect_left(stamps, stamp)
    candidate = min(
        odometry[max(0, index - 1):index + 1],
        key=lambda row: abs(row[0] - stamp),
    )
    if abs(candidate[0] - stamp) > max_skew_s:
        raise ValueError(f"No reference odometry within {max_skew_s} s of {stamp}")
    return Pose2(candidate[1], candidate[2], candidate[3])


def audit(events, odometry, offset, max_skew_s):
    stamps = [row[0] for row in odometry]
    rows = []
    for event in events:
        old_truth = sensor_pose(
            nearest_truth(odometry, stamps, event["closest_time"], max_skew_s), offset
        )
        current_truth = sensor_pose(
            nearest_truth(odometry, stamps, event["current_time"], max_skew_s), offset
        )
        true_relative = relative_pose(old_truth, current_truth)
        matched_relative = relative_pose(event["closest"], event["matched"])
        prior_relative = relative_pose(event["closest"], event["prior"])
        rows.append(
            {
                "current_id": event["current_id"],
                "current_time_s": event["current_time"],
                "closest_id": event["closest_id"],
                "closest_time_s": event["closest_time"],
                "fine_response": event["fine_response"],
                "gate_passed": event["gate_passed"],
                "new_graph_edge": event["new_graph_edge"],
                "gate_distance_m": round(event["gate_distance_m"], 4),
                "truth_center_distance_m": round(position_error(old_truth, current_truth), 4),
                "relative_position_error_m": round(position_error(matched_relative, true_relative), 4),
                "relative_heading_error_rad": round(
                    abs(wrap_angle(matched_relative.yaw - true_relative.yaw)), 4
                ),
                "prior_relative_position_error_m": round(
                    position_error(prior_relative, true_relative), 4
                ),
                "prior_relative_heading_error_rad": round(
                    abs(wrap_angle(prior_relative.yaw - true_relative.yaw)), 4
                ),
                "match_innovation_m": round(position_error(event["matched"], event["prior"]), 4),
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_bag", type=Path)
    parser.add_argument("mapper_log", type=Path)
    parser.add_argument("--sensor-x-m", type=float, default=0.15)
    parser.add_argument("--sensor-y-m", type=float, default=0.0)
    parser.add_argument("--sensor-yaw-rad", type=float, default=0.0)
    parser.add_argument("--max-skew-s", type=float, default=0.02)
    parser.add_argument("--all", action="store_true", help="Show failed-gate candidates too")
    args = parser.parse_args()
    if args.max_skew_s <= 0 or not all(
        math.isfinite(value)
        for value in (args.sensor_x_m, args.sensor_y_m, args.sensor_yaw_rad, args.max_skew_s)
    ):
        parser.error("Sensor offset must be finite and timestamp tolerance positive")
    events = parse_events(args.mapper_log.read_text())
    rows = audit(
        events,
        read_odometry(args.reference_bag),
        Pose2(args.sensor_x_m, args.sensor_y_m, args.sensor_yaw_rad),
        args.max_skew_s,
    )
    selected = rows if args.all else [row for row in rows if row["gate_passed"]]
    print(
        json.dumps(
            {
                "fine_accepted_candidates": len(rows),
                "gate_passes": sum(row["gate_passed"] for row in rows),
                "new_graph_edges": (
                    sum(row["new_graph_edge"] is True for row in rows)
                    if any(row["new_graph_edge"] is not None for row in rows)
                    else None
                ),
                "rows": selected,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
