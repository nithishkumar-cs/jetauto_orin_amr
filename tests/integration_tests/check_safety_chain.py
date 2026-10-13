#!/usr/bin/env python3
"""Exercise the software command chain without a controller or driver.

Source Humble and the workspace overlay, then run:
    python3 tests/integration_tests/check_safety_chain.py

The script chooses an isolated ROS_DOMAIN_ID and localhost-only DDS. It starts
only collision_monitor, its lifecycle manager, scan_health_gate, and
estop_gate. Nothing
subscribes to /cmd_vel/safety_limited except this test observer.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


ROOT = Path(__file__).resolve().parents[2]
COLLISION_CONFIG = ROOT / "configs/safety/collision_monitor.yaml"
ESTOP_CONFIG = ROOT / "configs/safety/estop_gate.yaml"
SCAN_HEALTH_CONFIG = ROOT / "configs/safety/scan_health_gate.yaml"


def spin_until(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.02)
    return predicate()


def executable(package, name):
    return Path(get_package_prefix(package)) / "lib" / package / name


class SafetyObserver(Node):
    def __init__(self):
        super().__init__("offline_safety_chain_observer")
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.estop_pub = self.create_publisher(Bool, "/estop/engaged", 10)
        self.reset_pub = self.create_publisher(Bool, "/estop/reset", 10)
        self.collision_outputs = []
        self.scan_health_outputs = []
        self.final_outputs = []
        self.create_subscription(
            Twist,
            "/cmd_vel/collision_limited",
            self.collision_outputs.append,
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel/scan_health_limited",
            self.scan_health_outputs.append,
            10,
        )
        self.create_subscription(
            Twist, "/cmd_vel/safety_limited", self.final_outputs.append, 10
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)
        self.state_client = self.create_client(
            GetState, "/collision_monitor/get_state"
        )

    def feed_scan(self, range_m, stamp_age_sec=0.0, unusable=False):
        scan = LaserScan()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.angle_max = math.pi - scan.angle_increment
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [5.0] * 360
        if range_m is not None:
            for index in range(178, 183):
                scan.ranges[index] = range_m
        if unusable:
            scan.ranges = [math.nan] * 360
        for _ in range(8):
            scan.header.stamp = (
                self.get_clock().now() - Duration(seconds=stamp_age_sec)
            ).to_msg()
            self.scan_pub.publish(scan)
            rclpy.spin_once(self, timeout_sec=0.04)

    def send_command(self, speed=0.4):
        before_collision = len(self.collision_outputs)
        before_health = len(self.scan_health_outputs)
        before_final = len(self.final_outputs)
        command = Twist()
        command.linear.x = speed
        self.command_pub.publish(command)
        if not spin_until(
            self,
            lambda: len(self.collision_outputs) > before_collision
            and len(self.scan_health_outputs) > before_health
            and len(self.final_outputs) > before_final,
            timeout=3.0,
        ):
            raise AssertionError("command did not traverse the chain")
        # The health gate also publishes timer-driven zeros. Collect a short
        # post-command window so a later nonzero cannot hide behind one.
        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        return (
            self.collision_outputs[-1].linear.x,
            [
                message.linear.x
                for message in self.final_outputs[before_final:]
            ],
        )

    def send_bool(self, publisher, value=True):
        message = Bool()
        message.data = value
        publisher.publish(message)


def assert_speeds(name, observed, expected):
    collision, final_outputs = observed
    expected_collision, expected_final = expected
    if (
        abs(collision - expected_collision) > 1e-5
        or not final_outputs
        or any(abs(actual - expected_final) > 1e-5 for actual in final_outputs)
    ):
        raise AssertionError(f"{name}: expected {expected}, got {observed}")
    print(
        f"PASS {name}: collision={collision}, final={expected_final} "
        f"({len(final_outputs)} messages)", flush=True,
    )


def run_checks(node):
    if not spin_until(
        node, lambda: node.state_client.service_is_ready(), timeout=10.0
    ):
        raise AssertionError(
            "collision monitor lifecycle service not available"
        )
    state = None
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        future = node.state_client.call_async(GetState.Request())
        if not spin_until(node, lambda: future.done(), timeout=2.0):
            raise AssertionError(
                "collision monitor lifecycle state request timed out"
            )
        state = future.result().current_state
        if state.id == 3:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if state is None or state.id != 3:
        raise AssertionError(f"collision monitor is not active: {state}")
    print("PASS collision monitor lifecycle active [3]", flush=True)

    if not spin_until(
        node,
        lambda: node.command_pub.get_subscription_count() >= 1
        and node.scan_pub.get_subscription_count() >= 2
        and node.estop_pub.get_subscription_count() >= 1
        and node.reset_pub.get_subscription_count() >= 1,
        timeout=5.0,
    ):
        raise AssertionError("safety-chain subscriptions not discovered")

    assert_speeds("no scan at startup", node.send_command(), (0.4, 0.0))

    node.feed_scan(None)
    assert_speeds("clear scan", node.send_command(), (0.4, 0.4))

    node.feed_scan(0.9)
    assert_speeds("slow zone", node.send_command(), (0.14, 0.14))

    node.feed_scan(0.4)
    assert_speeds("stop zone", node.send_command(), (0.0, 0.0))

    node.feed_scan(None)
    before_final = len(node.final_outputs)
    node.send_bool(node.estop_pub)
    if not spin_until(node, lambda: len(node.final_outputs) > before_final):
        raise AssertionError("e-stop did not publish an immediate zero")
    if abs(node.final_outputs[-1].linear.x) > 1e-5:
        raise AssertionError("e-stop immediate output was not zero")
    print("PASS e-stop immediate zero", flush=True)
    assert_speeds("e-stop latched", node.send_command(), (0.4, 0.0))

    node.feed_scan(None)
    node.send_bool(node.reset_pub)
    # Drain reset callback before sending a command on a separate topic.
    rclpy.spin_once(node, timeout_sec=0.1)
    assert_speeds("e-stop reset", node.send_command(), (0.4, 0.4))

    # Humble ignores stale observations. The new gate must stop independently,
    # including before another command arrives.
    before_timeout = len(node.final_outputs)
    time.sleep(0.45)
    if not spin_until(
        node,
        lambda: any(
            message.linear.x == 0.0
            for message in node.final_outputs[before_timeout:]
        ),
    ):
        raise AssertionError(
            "scan timeout did not publish zero without a command"
        )
    print("PASS timeout publishes zero without new command", flush=True)
    assert_speeds("stale scan", node.send_command(), (0.4, 0.0))

    node.feed_scan(None, stamp_age_sec=1.0)
    assert_speeds("old-stamped scan", node.send_command(), (0.4, 0.0))

    node.feed_scan(None, stamp_age_sec=-1.0)
    assert_speeds("future-stamped scan", node.send_command(), (0.4, 0.0))

    node.feed_scan(None)
    assert_speeds("fresh-scan recovery", node.send_command(), (0.4, 0.4))

    node.feed_scan(None, unusable=True)
    assert_speeds("unusable scan", node.send_command(), (0.4, 0.0))

    node.send_bool(node.estop_pub)
    rclpy.spin_once(node, timeout_sec=0.1)
    assert_speeds("e-stop after scan loss", node.send_command(), (0.4, 0.0))
    node.feed_scan(None)
    assert_speeds(
        "e-stop remains latched after scan recovery",
        node.send_command(), (0.4, 0.0),
    )
    node.feed_scan(None)
    node.send_bool(node.reset_pub)
    rclpy.spin_once(node, timeout_sec=0.1)
    assert_speeds("reset after scan recovery", node.send_command(), (0.4, 0.4))


def main():
    os.environ["ROS_DOMAIN_ID"] = str(130 + os.getpid() % 70)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/jetauto_safety_chain_ros_logs")
    print(f"isolated ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}", flush=True)
    rclpy.init()
    processes = []
    node = None
    try:
        with tempfile.TemporaryDirectory(
            prefix="jetauto_safety_chain_"
        ) as log_dir:
            specs = [
                (
                    "scan_health_gate", "scan_health_gate_node",
                    SCAN_HEALTH_CONFIG, None,
                ),
                ("estop_gate", "estop_gate_node", ESTOP_CONFIG, None),
                (
                    "nav2_collision_monitor", "collision_monitor",
                    COLLISION_CONFIG, None,
                ),
                (
                    "nav2_lifecycle_manager", "lifecycle_manager",
                    COLLISION_CONFIG,
                    "lifecycle_manager_safety",
                ),
            ]
            for package, name, config, node_name in specs:
                log = open(
                    Path(log_dir) / f"{name}.log", "w", encoding="utf-8"
                )
                command = [
                    str(executable(package, name)), "--ros-args",
                    "--params-file", str(config),
                ]
                if node_name:
                    command.extend(["-r", f"__node:={node_name}"])
                process = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append((process, log))
            node = SafetyObserver()
            try:
                run_checks(node)
            except Exception:
                for process, log in processes:
                    log.flush()
                    print(f"--- {process.args[0]} log ---", flush=True)
                    print(
                        Path(log.name).read_text(encoding="utf-8")[-4000:],
                        flush=True,
                    )
                raise
    finally:
        if node is not None:
            node.destroy_node()
        for process, log in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            log.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
