#!/usr/bin/env python3
"""Exercise the software command chain without a controller or driver.

Source Humble and the workspace overlay, then run:
    python3 tests/integration_tests/check_safety_chain.py

The script chooses an isolated ROS_DOMAIN_ID and localhost-only DDS. It starts
only collision_monitor, its lifecycle manager, and estop_gate. Nothing
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


ROOT = Path(__file__).resolve().parents[2]
COLLISION_CONFIG = ROOT / "configs/safety/collision_monitor.yaml"
ESTOP_CONFIG = ROOT / "configs/safety/estop_gate.yaml"


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
        self.final_outputs = []
        self.create_subscription(
            Twist,
            "/cmd_vel/collision_limited",
            self.collision_outputs.append,
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

    def feed_scan(self, range_m):
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
        for _ in range(8):
            scan.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(scan)
            rclpy.spin_once(self, timeout_sec=0.04)

    def send_command(self, speed=0.4):
        before_collision = len(self.collision_outputs)
        before_final = len(self.final_outputs)
        command = Twist()
        command.linear.x = speed
        self.command_pub.publish(command)
        if not spin_until(
            self,
            lambda: len(self.collision_outputs) > before_collision
            and len(self.final_outputs) > before_final,
            timeout=3.0,
        ):
            raise AssertionError("command did not traverse both nodes")
        return (
            self.collision_outputs[-1].linear.x,
            self.final_outputs[-1].linear.x,
        )

    def send_bool(self, publisher, value=True):
        message = Bool()
        message.data = value
        publisher.publish(message)


def assert_speeds(name, observed, expected):
    if any(
        abs(actual - target) > 1e-5
        for actual, target in zip(observed, expected)
    ):
        raise AssertionError(f"{name}: expected {expected}, got {observed}")
    print(f"PASS {name}: collision/final={observed}", flush=True)


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
        and node.scan_pub.get_subscription_count() >= 1
        and node.estop_pub.get_subscription_count() >= 1
        and node.reset_pub.get_subscription_count() >= 1,
        timeout=5.0,
    ):
        raise AssertionError("safety-chain subscriptions not discovered")

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
    assert_speeds(
        "e-stop immediate", (0.0, node.final_outputs[-1].linear.x),
        (0.0, 0.0),
    )
    assert_speeds("e-stop latched", node.send_command(), (0.4, 0.0))

    node.send_bool(node.reset_pub)
    # Drain reset callback before sending a command on a separate topic.
    rclpy.spin_once(node, timeout_sec=0.1)
    assert_speeds("e-stop reset", node.send_command(), (0.4, 0.4))

    # Nav2 Humble ignores stale observations. Explicitly observe this behavior
    # because it is NOT the fail-closed safety contract a driver would need.
    time.sleep(0.45)
    stale = node.send_command()
    print(
        f"OBSERVED scan older than source_timeout: collision/final={stale}",
        flush=True,
    )
    node.send_bool(node.estop_pub)
    if not spin_until(node, lambda: node.final_outputs[-1].linear.x == 0.0):
        raise AssertionError("e-stop did not stop after scan loss")
    assert_speeds("e-stop after scan loss", node.send_command(), (0.4, 0.0))
    if stale[0] != 0.0 or stale[1] != 0.0:
        raise AssertionError(
            "SAFETY GAP: stale/missing LiDAR scan allows nonzero command "
            "through both nodes"
        )


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
