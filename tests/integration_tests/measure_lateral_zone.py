#!/usr/bin/env python3
"""Probe the Isaac lateral collision zone with bounded raw mecanum commands."""

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

from probe_isaac_stop_zone_recovery import RecoveryProbe, seconds


class LateralProbe(RecoveryProbe):
    """Observe both planar command components and the scan geometry of a side obstacle."""

    def __init__(self):
        """Prepare optional per-callback evidence for a bounded lateral run."""
        super().__init__()
        self.trace_events = None

    def capture(self, stage, **values):
        """Record a local receipt event with wall and simulation times when enabled."""
        if self.trace_events is not None:
            odom = self.odom
            self.trace_events.append({"stage": stage, "wall_ns": time.time_ns(),
                                      "sim_s": seconds(self.clock_message.clock)
                                      if self.clock_message is not None else None,
                                      "latest_scan_stamp_s": seconds(self.scan.header.stamp)
                                      if self.scan is not None else None,
                                      "odom": {
                                          "stamp_s": seconds(odom.header.stamp),
                                          "x": odom.pose.pose.position.x,
                                          "y": odom.pose.pose.position.y,
                                          "qz": odom.pose.pose.orientation.z,
                                          "qw": odom.pose.pose.orientation.w,
                                      } if odom is not None else None,
                                      **values})

    def on_scan(self, message):
        """Preserve each scan and its locally transformed counts for later replay."""
        super().on_scan(message)
        if self.trace_events is None:
            return
        try:
            state = self.zone_state()
            counts = {"stop": state["stop_zone_points"],
                      "slow": state["slow_zone_points"]}
        except Exception as exc:  # Capture a missing TF without changing the safety probe.
            counts = {"tf_error": str(exc)}
        self.capture("scan", stamp_s=seconds(message.header.stamp),
                     frame=message.header.frame_id, counts=counts,
                     angle_min=message.angle_min, angle_max=message.angle_max,
                     angle_increment=message.angle_increment,
                     time_increment=message.time_increment, scan_time=message.scan_time,
                     range_min=message.range_min, range_max=message.range_max,
                     ranges=[float(value) if math.isfinite(value) else None
                             for value in message.ranges],
                     intensities=[float(value) if math.isfinite(value) else None
                                  for value in message.intensities])

    def on_log(self, message):
        """Keep collision-monitor action transitions in the same trace as scans."""
        super().on_log(message)
        if message.name.endswith("collision_monitor"):
            self.capture("monitor_log", stamp_s=seconds(message.stamp),
                         text=message.msg)

    def chain_ready(self):
        """Require the safety chain and Isaac while allowing a passive bag recorder."""
        expected = {
            "/cmd_vel": ({"collision_zone_measurement", "controller_server"},
                         {"collision_monitor", "stop_zone_recovery", "collision_zone_measurement"}),
            "/cmd_vel/collision_limited": ({"collision_monitor"},
                                           {"stop_zone_recovery", "collision_zone_measurement"}),
            "/cmd_vel/recovery_limited": ({"stop_zone_recovery"},
                                          {"scan_health_gate", "collision_zone_measurement"}),
            "/cmd_vel/scan_health_limited": ({"scan_health_gate"},
                                             {"estop_gate", "collision_zone_measurement"}),
            "/cmd_vel/safety_limited": ({"estop_gate"},
                                        {"_World_JetAutoROS_CmdVelSub", "collision_zone_measurement"}),
        }
        for topic, (publishers, subscribers) in expected.items():
            actual_publishers = {item.node_name for item in self.get_publishers_info_by_topic(topic)}
            actual_subscribers = {item.node_name for item in self.get_subscriptions_info_by_topic(topic)}
            actual_subscribers.discard("rosbag2_recorder")
            if actual_publishers != publishers or actual_subscribers != subscribers:
                return False
        return True

    def on_controller_command(self, message):
        """Keep raw planar commands for comparison with each safety stage."""
        self.controller_commands.append((time.monotonic(), message.linear.x, message.linear.y))
        self.capture("raw", x=message.linear.x, y=message.linear.y)

    def on_limited_command(self, message):
        """Keep the collision monitor's planar output."""
        self.limited_commands.append((time.monotonic(), message.linear.x, message.linear.y))
        self.capture("collision_limited", x=message.linear.x, y=message.linear.y)

    def on_recovered(self, message):
        """Keep the recovery node's planar output and observed stop occupancy."""
        ready = self.scan is not None and self.tf_buffer.can_transform(
            "base_link", self.scan.header.frame_id, Time()
        )
        self.recovered_commands.append(
            (time.monotonic(), message.linear.x, message.linear.y,
             self.zone_state()["stop_zone_points"] if ready else None)
        )
        self.capture("recovery_limited", x=message.linear.x, y=message.linear.y)

    def on_scan_health(self, message):
        """Keep the planar command entering the final e-stop gate."""
        self.scan_health_commands.append((time.monotonic(), message.linear.x, message.linear.y))
        self.capture("scan_health_limited", x=message.linear.x, y=message.linear.y)

    def on_final_command(self, message):
        """Keep the planar command actually offered to Isaac."""
        self.final_commands.append((time.monotonic(), message.linear.x, message.linear.y))
        self.capture("safety_limited", x=message.linear.x, y=message.linear.y)

    def zone_state(self):
        """Add nearest left and right returns to the existing square-zone counts."""
        state = super().zone_state()
        points = self.scan_points_in_base()
        state["nearest_left_m"] = min(
            (math.hypot(x, y) for x, y in points if y > 0 and abs(x) <= 0.55),
            default=float("inf"),
        )
        state["nearest_right_m"] = min(
            (math.hypot(x, y) for x, y in points if y < 0 and abs(x) <= 0.55),
            default=float("inf"),
        )
        return state


def planar_distance(a, b):
    """Return planar odometry distance between two saved positions."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def position(node):
    """Return the latest odometry position as a detached pair."""
    pose = node.odom.pose.pose.position
    return (pose.x, pose.y)


def preflight(node, expected_xy):
    """Require fresh Isaac inputs, active command graph, and independently checked map start."""
    node.spin_for(5.0)
    assert node.odom is not None and node.scan is not None and node.clock_message is not None
    assert len(node.get_publishers_info_by_topic("/clock")) == 1, "not one clock"
    assert "base_driver" not in node.get_node_names(), "physical driver present"
    assert node.chain_ready(), "software command graph incomplete"
    node.assert_fresh()
    assert node.tf_buffer.can_transform("map", "base_link", Time()), "map TF missing"
    map_pose = node.map_pose()
    residual = planar_distance(map_pose[:2], expected_xy)
    assert residual <= 0.15, f"map start residual {residual:.3f} m exceeds 0.15 m"
    return {"map_pose_xy_yaw": map_pose, "start_residual_m": residual,
            "clock_publishers": 1, "driver_absent": True, "chain_ready": True}


def drive(node, command, *, limit_s, travel_cap_m, abort_range_m, stop_when,
          expect_zero=False):
    """Send one bounded command while scan, TF, odometry, time, and proximity remain valid."""
    start_xy = position(node)
    start_clock = seconds(node.clock_message.clock)
    start_wall = time.monotonic()
    samples = []
    final_start = len(node.final_commands)
    last_scan_stamp = None
    reason = "time limit"
    try:
        while time.monotonic() - start_wall < limit_s:
            rclpy.spin_once(node, timeout_sec=0.04)
            node.assert_fresh()
            odom_age = seconds(node.clock_message.clock) - seconds(node.odom.header.stamp)
            assert -0.1 < odom_age < 0.3, f"odometry age {odom_age:.3f} s"
            if command.linear.x < 0 and not expect_zero:
                assert node.rear_box_points() == 0, "rear box became occupied"
            state = node.zone_state()
            if expect_zero:
                escaped = next((row for row in node.final_commands[final_start:]
                                if abs(row[1]) >= 0.001 or abs(row[2]) >= 0.001), None)
                assert escaped is None, (
                    f"forbidden input ({command.linear.x:.3f}, {command.linear.y:.3f}) "
                    f"reached final output {escaped[1:]} with latest stop points "
                    f"{state['stop_zone_points']}"
                )
            travel = planar_distance(position(node), start_xy)
            assert travel <= travel_cap_m, f"travel cap {travel_cap_m} m exceeded"
            nearest = state["nearest_left_m"] if command.linear.y > 0 else (
                state["nearest_right_m"] if command.linear.y < 0 else state["nearest_ahead_m"]
            )
            assert nearest >= abort_range_m, f"proximity abort at {nearest:.3f} m"
            stamp = (node.scan.header.stamp.sec, node.scan.header.stamp.nanosec)
            if stamp != last_scan_stamp:
                last_scan_stamp = stamp
                now = time.monotonic()
                state.update({"wall_s": now - start_wall, "travel_m": travel,
                              "raw": [command.linear.x, command.linear.y],
                              "limited": list(node.limited_commands[-1][1:])
                              if node.limited_commands else None,
                              "final": list(node.final_commands[-1][1:])
                              if node.final_commands else None})
                samples.append(state)
                if stop_when(samples):
                    reason = "condition met"
                    break
            node.command.publish(command)
    finally:
        for _ in range(5):
            node.command.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
    return {"reason": reason, "samples": samples,
            "odom_travel_m": planar_distance(position(node), start_xy),
            "start_odom_xy_m": start_xy, "end_odom_xy_m": position(node),
            "sim_elapsed_s": seconds(node.clock_message.clock) - start_clock,
            "wall_elapsed_s": time.monotonic() - start_wall}


def clear_start(node):
    """Move a short distance away from the lateral box until both zones are clear."""
    before = node.zone_state()
    assert before["stop_zone_points"] <= 3, "stop square occupied at start"
    assert before["nearest_right_m"] > 0.8, "retreat side is not clear"
    node.reset.publish(Bool(data=True))
    node.spin_for(0.3)
    command = Twist()
    command.linear.y = -0.10
    result = drive(node, command, limit_s=12.0, travel_cap_m=0.30,
                   abort_range_m=0.55,
                   stop_when=lambda rows: rows[-1]["slow_zone_points"] <= 3
                   and rows[-1]["nearest_left_m"] > 1.31)
    node.latch_estop()
    node.spin_for(0.7)
    after = node.zone_state()
    assert result["reason"] == "condition met" and after["slow_zone_points"] <= 3
    return {"before": before, "after": after,
            "settled_odom_xy_m": position(node),
            **{k: v for k, v in result.items() if k != "samples"}}


def approach(node):
    """Strafe toward the lateral box until the monitor stops the robot."""
    before = node.zone_state()
    assert before["stop_zone_points"] <= 3 and before["slow_zone_points"] <= 3, (
        "both zones must be clear before the lateral approach"
    )
    assert before["nearest_left_m"] > 1.30, "lateral obstacle starts inside slow square"
    node.reset.publish(Bool(data=True))
    node.spin_for(0.3)
    command = Twist()
    command.linear.y = 0.10

    def stopped(rows):
        """Accept a sustained monitor stop with negligible final odometry drift."""
        recent = [row for row in rows if row["limited"] is not None
                  and abs(row["limited"][1]) < 0.001 and row["stop_zone_points"] > 3]
        return bool(recent) and rows[-1]["wall_s"] - recent[0]["wall_s"] >= 1.5 \
            and abs(rows[-1]["travel_m"] - recent[0]["travel_m"]) < 0.02

    result = drive(node, command, limit_s=60.0, travel_cap_m=1.00,
                   abort_range_m=0.43, stop_when=stopped)
    node.latch_estop()
    samples = result.pop("samples")
    slow = [row for row in samples if row["limited"] is not None
            and 0.01 < row["limited"][1] < 0.09]
    stop = [row for row in samples if row["limited"] is not None
            and abs(row["limited"][1]) < 0.001 and row["stop_zone_points"] > 3]
    assert result["reason"] == "condition met", "sustained lateral monitor stop not observed"
    assert slow, "no slowdown observed"
    ratios = [row["limited"][1] / command.linear.y for row in slow]
    return {"before": before, "after": node.zone_state(), "scan_samples": len(samples),
            "slowdown": {"events": len(slow), "first": slow[0],
                         "median_ratio": sorted(ratios)[len(ratios) // 2]},
            "stop": stop[0], **result}


def challenge(node):
    """Test reverse eligibility and blocked strafe/forward in the occupied square."""
    before = node.zone_state()
    assert before["stop_zone_points"] > 3, "stop square is not occupied"
    rear_points = node.rear_box_points()
    node.reset.publish(Bool(data=True))
    node.spin_for(0.3)
    reverse = Twist()
    reverse.linear.x = -0.06
    reverse_start = len(node.final_commands)
    reverse_result = drive(node, reverse,
                           limit_s=1.5 if rear_points == 0 else 0.5,
                           travel_cap_m=0.12 if rear_points == 0 else 0.04,
                           abort_range_m=0.43, stop_when=lambda rows: False,
                           expect_zero=rear_points > 0)
    after_reverse = node.zone_state()
    assert after_reverse["stop_zone_points"] > 3, "reverse cleared stop square before challenges"
    admitted = [row for row in node.recovered_commands
                if row[1] < -0.04 and row[3] is not None and row[3] > 3]
    if rear_points == 0:
        assert admitted, "no capped pure reverse recovery while stop zone occupied"
    else:
        assert not admitted, "reverse recovery admitted with occupied rear box"
        assert planar_distance(position(node), reverse_result["start_odom_xy_m"]) < 0.01
        assert node.final_commands[reverse_start:], "no final reverse decision observed"
    blocked = {}
    for name, x, y in (("strafe", 0.0, 0.06), ("forward", 0.06, 0.0)):
        command = Twist()
        command.linear.x, command.linear.y = x, y
        assert node.zone_state()["stop_zone_points"] > 3
        begin = len(node.final_commands)
        start = position(node)
        result = drive(node, command, limit_s=0.5, travel_cap_m=0.04,
                       abort_range_m=0.43, stop_when=lambda rows: False,
                       expect_zero=True)
        outputs = node.final_commands[begin:]
        assert outputs and all(abs(row[1]) < 0.001 and abs(row[2]) < 0.001
                               for row in outputs), f"{name} escaped the stop zone"
        assert planar_distance(position(node), start) < 0.01, f"{name} moved the base"
        blocked[name] = {"final_zero_samples": len(outputs),
                         "odom_travel_m": result["odom_travel_m"]}
    node.latch_estop()
    return {"before": before, "rear_box_points": rear_points,
            "after_reverse": after_reverse,
            "reverse": {k: v for k, v in reverse_result.items() if k != "samples"},
            "capped_recovery_samples_in_stop": len(admitted), "blocked": blocked}


def blocked_challenge(node, direction):
    """Check a short forward or sideways input at the occupied side boundary."""
    before = node.zone_state()
    assert before["stop_zone_points"] > 3, "stop square is not occupied"
    node.reset.publish(Bool(data=True))
    node.spin_for(0.3)
    command = Twist()
    if direction == "forward":
        command.linear.x = 0.06
    else:
        command.linear.y = 0.06
    start = len(node.final_commands)
    result = drive(node, command, limit_s=0.5, travel_cap_m=0.04,
                   abort_range_m=0.43, stop_when=lambda rows: False,
                   expect_zero=True)
    outputs = node.final_commands[start:]
    assert outputs and result["odom_travel_m"] < 0.01
    return {"before": before, "after": node.zone_state(),
            "direction": direction,
            "final_zero_samples": len(outputs),
            **{key: value for key, value in result.items() if key != "samples"}}


def forward_challenge(node):
    """Check a short forward input through the occupied stop square."""
    return blocked_challenge(node, "forward")


def strafe_challenge(node):
    """Check a short sideways input toward the occupied stop square."""
    return blocked_challenge(node, "strafe")


def reverse_challenge(node):
    """Require zero reverse output when the lateral box occupies the rear clearance box."""
    before = node.zone_state()
    rear_points = node.rear_box_points()
    assert before["stop_zone_points"] > 3 and rear_points > 0
    node.reset.publish(Bool(data=True))
    node.spin_for(0.3)
    command = Twist()
    command.linear.x = -0.06
    begin = len(node.final_commands)
    result = drive(node, command, limit_s=0.5, travel_cap_m=0.04,
                   abort_range_m=0.43, stop_when=lambda rows: False,
                   expect_zero=True)
    outputs = node.final_commands[begin:]
    assert outputs and result["odom_travel_m"] < 0.01
    return {"before": before, "after": node.zone_state(),
            "rear_box_points": rear_points, "final_zero_samples": len(outputs),
            **{key: value for key, value in result.items() if key != "samples"}}


def main():
    """Run one explicitly selected, independently bounded Isaac-only phase."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-simulation-only", action="store_true", required=True)
    parser.add_argument("--phase", choices=("clear", "approach", "challenge", "forward",
                                            "strafe", "reverse"),
                        required=True)
    parser.add_argument("--expected-start-map-x", type=float, required=True)
    parser.add_argument("--expected-start-map-y", type=float, required=True)
    parser.add_argument("--trace-output", type=Path,
                        help="write local scan and command receipts, including on failure")
    args = parser.parse_args()
    if not args.isaac_simulation_only:
        parser.error("explicit Isaac-only flag required")
    rclpy.init()
    node = LateralProbe()
    try:
        ready = preflight(node, (args.expected_start_map_x, args.expected_start_map_y))
        if args.trace_output is not None:
            node.trace_events = []
        result = {"clear": clear_start, "approach": approach,
                  "challenge": challenge, "forward": forward_challenge,
                  "strafe": strafe_challenge, "reverse": reverse_challenge}[args.phase](node)
        print(json.dumps({"lateral_zone": {"phase": args.phase, "preflight": ready,
                                           "result": result}}, indent=2), flush=True)
    finally:
        try:
            node.latch_estop()
        finally:
            try:
                if args.trace_output is not None and node.trace_events is not None:
                    failure = sys.exc_info()[1]
                    args.trace_output.write_text(json.dumps({
                        "phase": args.phase,
                        "failure": str(failure) if failure is not None else None,
                        "events": node.trace_events,
                    }))
            finally:
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
