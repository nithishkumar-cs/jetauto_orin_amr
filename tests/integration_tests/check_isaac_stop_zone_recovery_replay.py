#!/usr/bin/env python3
"""Replay recorded Isaac scans/TF through the full chain to exercise stop_zone_recovery.

Every recorded return inside a 1.4 m base_link square (outside both zones) is blanked so the
scenes are deterministic; synthetic front/rear returns are then injected per
window while a scripted /cmd_vel is sent. No controller or driver is started.
Not safety-rated; checks command-chain behaviour only.
"""

import argparse
import math
from pathlib import Path
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from check_isaac_safety_replay import DEFAULT_BAG, safety_chain, spin_until

LIDAR_X = 0.15  # lidar_link x offset in base_link (recorded static TF)
BLANK_HALF_SIDE = 1.4  # covers the 1.25 m slow square
FRONT_HALF_ANGLE = 0.3
REAR_HALF_ANGLE = 0.1
FRONT_RANGE = 0.35  # base x ~0.49: inside the 0.55 m stop square
REAR_RANGE = 0.7  # base x ~-0.55: inside the rear recovery box
END_SIM = 11.5
GAP_COMMAND = (-0.2, 0.0)

# name, window [start, end) sim seconds, front obstacle, rear obstacle, (vx, vy)
SCENES = (
    ("clear scene reverse passes unchanged", (1.5, 3.0), False, False, (-0.2, 0.0)),
    ("stop zone + clear rear: capped reverse", (3.5, 5.0), True, False, (-0.2, 0.0)),
    ("stop zone + rear return: held at zero", (5.5, 7.0), True, True, (-0.2, 0.0)),
    ("stop zone + forward command: zero", (7.5, 9.0), True, False, (0.2, 0.0)),
    ("stop zone + reverse with strafe: zero", (9.5, 11.0), True, False, (-0.2, 0.1)),
)


def scene_at(sim):
    """Return the scene covering `sim` seconds, or None between windows."""
    for scene in SCENES:
        if scene[1][0] <= sim < scene[1][1]:
            return scene
    return None


def shape_scan(scan, front, rear):
    """Blank near returns, then inject the requested front/rear obstacles in place."""
    ranges = list(scan.ranges)
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        angle = scan.angle_min + i * scan.angle_increment
        bx = LIDAR_X + r * math.cos(angle)
        by = r * math.sin(angle)
        if abs(bx) < BLANK_HALF_SIDE and abs(by) < BLANK_HALF_SIDE:
            ranges[i] = math.inf
    for i in range(len(ranges)):
        angle = scan.angle_min + i * scan.angle_increment
        if front and abs(angle) < FRONT_HALF_ANGLE:
            ranges[i] = FRONT_RANGE
        if rear and math.pi - abs(angle) < REAR_HALF_ANGLE:
            ranges[i] = REAR_RANGE
    scan.ranges = ranges
    return scan


def play_bag(node, bag, failure):
    """Publish bag clock/scan/TF at 1x simulated time with scene-shaped scans."""
    try:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag), storage_id="sqlite3"),
            ConverterOptions("cdr", "cdr"),
        )
        start_sim = start_wall = None
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == "/clock":
                message = deserialize_message(raw, Clock)
                sim = message.clock.sec + message.clock.nanosec / 1e9
                if start_sim is None:
                    start_sim, start_wall = sim, time.monotonic()
                if sim - start_sim > END_SIM:
                    break
                remaining = start_wall + sim - start_sim - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                node.sim_seconds = sim - start_sim
                node.clock_pub.publish(message)
            elif topic == "/scan":
                scene = scene_at(node.sim_seconds)
                front, rear = (scene[2], scene[3]) if scene else (False, False)
                node.scan_pub.publish(
                    shape_scan(deserialize_message(raw, LaserScan), front, rear)
                )
            elif topic == "/tf":
                node.tf_pub.publish(deserialize_message(raw, TFMessage))
            elif topic == "/tf_static":
                node.static_tf_pub.publish(deserialize_message(raw, TFMessage))
    except Exception as exc:
        failure.append(exc)
    finally:
        node.sim_seconds = END_SIM


def verify_scene(name, samples, window, check):
    """Apply `check` to final linear.x samples inside the window (margin trimmed)."""
    selected = [v for t, v in samples if window[0] + 0.4 <= t < window[1]]
    if len(selected) < 5:
        raise AssertionError(f"{name}: only {len(selected)} output messages")
    ok, detail = check(selected)
    if not ok:
        raise AssertionError(f"{name}: {detail}")
    print(f"PASS {name}: {len(selected)} outputs, {detail}", flush=True)


def tolerant(expected):
    """Build a check: >=85% of outputs equal `expected`, the rest exactly zero.

    The Python replay publisher can jitter (stale command/scan -> fail-safe zero),
    so zero blips are tolerated; any other nonzero value fails.
    """

    def check(values):
        full = sum(abs(v - expected) < 1e-6 for v in values)
        clean = all(abs(v - expected) < 1e-6 or v == 0.0 for v in values)
        return clean and full >= 0.85 * len(values), f"{full}/{len(values)} at {expected}"

    return check


check_passes = tolerant(-0.2)  # reverse passes at the commanded speed
check_capped = tolerant(-0.05)  # reverse emitted at the recovery cap, never faster


def check_zero(values):
    """Output stays at zero."""
    return all(abs(v) < 1e-6 for v in values), f"max |v| {max(map(abs, values)):.3f}"


CHECKS = (check_passes, check_capped, check_zero, check_zero, check_zero)


def run(args):
    """Run the scripted scenes against the live chain and verify each window."""
    with safety_chain(args.bag) as node:
        failure = []
        player = threading.Thread(
            target=play_bag, args=(node, args.bag, failure), daemon=True
        )
        player.start()
        next_command = time.monotonic()
        while player.is_alive():
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= next_command:
                scene = scene_at(node.sim_seconds)
                command = Twist()
                # Between windows drive a clear-scene reverse: nonzero output resets
                # collision_monitor's stop_pub_timeout so zero windows are observable.
                command.linear.x, command.linear.y = scene[4] if scene else GAP_COMMAND
                node.command_pub.publish(command)
                next_command = time.monotonic() + 0.05
        player.join()
        if failure:
            raise failure[0]
        spin_until(node, lambda: False, timeout=0.2)
        for scene, check in zip(SCENES, CHECKS):
            verify_scene(scene[0], node.finals, scene[1], check)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    run(parser.parse_args())
