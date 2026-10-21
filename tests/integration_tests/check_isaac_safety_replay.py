#!/usr/bin/env python3
"""Replay recorded Isaac scans/TF through the command chain with controlled loss.

This starts no controller or driver. The source bag is read-only; /cmd_vel is
synthetic and /cmd_vel/safety_limited has only this test as a subscriber.
"""

import argparse
import contextlib
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BAG = Path(
    "/home/nithish/jetauto_geometry_validation_20261012/localization_bag"
)
CONFIGS = ROOT / "configs/safety"
SPECS = (
    ("scan_health_gate", "scan_health_gate_node", "scan_health_gate.yaml", None),
    ("estop_gate", "estop_gate_node", "estop_gate.yaml", None),
    (
        "stop_zone_recovery", "stop_zone_recovery_node",
        "stop_zone_recovery.yaml", None,
    ),
    ("nav2_collision_monitor", "collision_monitor", "collision_monitor.yaml", None),
    (
        "nav2_lifecycle_manager", "lifecycle_manager",
        "collision_monitor.yaml", "lifecycle_manager_safety",
    ),
)


def spin_until(node, predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


class ReplayObserver(Node):
    def __init__(self):
        super().__init__("isaac_safety_replay_observer")
        self.sim_seconds = 0.0
        self.collisions = []
        self.finals = []
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.static_tf_pub = self.create_publisher(
            TFMessage, "/tf_static", static_qos
        )
        self.create_subscription(
            Twist,
            "/cmd_vel/collision_limited",
            lambda msg: self.collisions.append((self.sim_seconds, msg.linear.x)),
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel/safety_limited",
            lambda msg: self.finals.append((self.sim_seconds, msg.linear.x)),
            10,
        )
        self.state_client = self.create_client(
            GetState, "/collision_monitor/get_state"
        )

    def send_command(self):
        command = Twist()
        command.linear.x = 0.4
        self.command_pub.publish(command)


def wait_for_chain(node):
    if not spin_until(node, node.state_client.service_is_ready):
        raise AssertionError("collision monitor lifecycle service unavailable")
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        request = node.state_client.call_async(GetState.Request())
        if not spin_until(node, request.done, timeout=3.0):
            raise AssertionError("collision monitor lifecycle request timed out")
        if request.result().current_state.id == 3:
            break
    else:
        raise AssertionError("collision monitor did not become active")
    if not spin_until(
        node,
        lambda: node.scan_pub.get_subscription_count() >= 2
        and node.command_pub.get_subscription_count() >= 1
        and node.tf_pub.get_subscription_count() >= 2,
    ):
        raise AssertionError("replay publishers did not discover the safety chain")


def play_bag(node, bag, failure):
    """Publish original messages at 1x simulated time, omitting selected data."""
    try:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(bag), storage_id="sqlite3"),
            ConverterOptions("cdr", "cdr"),
        )
        start_sim = None
        start_wall = None
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == "/clock":
                message = deserialize_message(raw, Clock)
                sim = message.clock.sec + message.clock.nanosec / 1e9
                if start_sim is None:
                    start_sim = sim
                    start_wall = time.monotonic()
                if sim - start_sim > 11.0:
                    break
                remaining = start_wall + sim - start_sim - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                node.sim_seconds = sim - start_sim
                node.clock_pub.publish(message)
            elif topic == "/scan":
                if not 3.4 <= node.sim_seconds < 5.3:
                    node.scan_pub.publish(deserialize_message(raw, LaserScan))
            elif topic == "/tf":
                if not 6.7 <= node.sim_seconds < 8.5:
                    node.tf_pub.publish(deserialize_message(raw, TFMessage))
            elif topic == "/tf_static":
                node.static_tf_pub.publish(deserialize_message(raw, TFMessage))
        node.sim_seconds = 11.0
    except Exception as exc:
        failure.append(exc)
        node.sim_seconds = 11.0


def verify_window(name, samples, window, require_motion):
    selected = [speed for stamp, speed in samples if window[0] <= stamp < window[1]]
    if len(selected) < 3:
        raise AssertionError(f"{name}: only {len(selected)} output messages")
    moving = sum(abs(speed) > 0.05 for speed in selected)
    if require_motion and moving == 0:
        raise AssertionError(f"{name}: no nonzero command in {len(selected)} outputs")
    if not require_motion and moving != 0:
        raise AssertionError(f"{name}: {moving}/{len(selected)} outputs were nonzero")
    print(f"PASS {name}: {len(selected)} outputs, {moving} nonzero", flush=True)


@contextlib.contextmanager
def safety_chain(bag):
    """Start the software safety chain on an isolated domain and yield a ReplayObserver.

    On failure prints each process log tail; always tears the chain down.
    """
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"no rosbag metadata: {bag}")
    os.environ["ROS_DOMAIN_ID"] = str(130 + os.getpid() % 70)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/jetauto_isaac_safety_replay_logs")
    rclpy.init()
    processes = []
    node = None
    try:
        with tempfile.TemporaryDirectory(prefix="jetauto_safety_replay_") as log_dir:
            for package, executable, config, node_name in SPECS:
                command = [
                    str(Path(get_package_prefix(package)) / "lib" / package / executable),
                    "--ros-args", "--params-file", str(CONFIGS / config),
                    "-p", "use_sim_time:=true",
                ]
                if node_name:
                    command.extend(["-r", f"__node:={node_name}"])
                log = open(Path(log_dir) / f"{executable}.log", "w", encoding="utf-8")
                process = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                processes.append((process, log))
            node = ReplayObserver()
            try:
                wait_for_chain(node)
                yield node
            except Exception:
                for process, log in processes:
                    log.flush()
                    print(f"--- {process.args[0]} log tail ---", flush=True)
                    print(Path(log.name).read_text(encoding="utf-8")[-4000:], flush=True)
                raise
    finally:
        if node is not None:
            node.destroy_node()
        for process, _ in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process, log in processes:
            if process.poll() is None:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
            log.close()
        rclpy.shutdown()


def run(args):
    """Replay the recorded bag with scan and TF dropouts and verify the final output."""
    with safety_chain(args.bag) as node:
        node.send_command()
        if not spin_until(node, lambda: len(node.finals) >= 2):
            raise AssertionError("startup command did not reach final topic")
        if any(abs(speed) > 0.05 for _, speed in node.finals):
            raise AssertionError("startup before first scan was not stopped")
        print("PASS startup before first scan: final output zero", flush=True)

        failure = []
        player = threading.Thread(
            target=play_bag, args=(node, args.bag, failure), daemon=True
        )
        player.start()
        next_command = time.monotonic()
        while player.is_alive():
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= next_command:
                node.send_command()
                next_command = time.monotonic() + 0.05
        player.join()
        if failure:
            raise failure[0]
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.01)

        verify_window("recorded scan + moving TF", node.finals, (1.5, 3.0), True)
        verify_window("scan dropout", node.finals, (4.1, 5.0), False)
        verify_window("scan recovery", node.finals, (5.8, 6.5), True)
        verify_window("TF dropout", node.finals, (7.3, 8.1), False)
        verify_window("TF recovery", node.finals, (9.0, 10.3), True)
        verify_window(
            "Nav2 fail-open during TF dropout", node.collisions,
            (7.3, 8.1), True,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    run(parser.parse_args())
