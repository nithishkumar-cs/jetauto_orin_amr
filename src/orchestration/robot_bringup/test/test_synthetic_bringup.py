"""Checks for variant admission and synthetic output contracts."""

import sys
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray
from amr_interfaces.msg import TrackedObstacleArray

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "scripts"))

from launch_lib import actions, config  # noqa: E402
from launch_lib.tiers import TIERS  # noqa: E402
from synthetic_source import SyntheticSource, empty_output  # noqa: E402


@pytest.mark.parametrize(
    ("source_type", "output_type", "frame"),
    [
        (Image, Detection2DArray, None),
        (Detection2DArray, Detection3DArray, None),
        (Detection3DArray, TrackedObstacleArray, "base_link"),
    ],
)
def test_empty_output_preserves_stamp_without_mutating_source(source_type, output_type, frame):
    """Synthetic arrays retain source timing and do not modify input headers."""
    source = source_type()
    source.header.stamp.sec = 42
    source.header.frame_id = "camera_frame"
    output = empty_output(source, output_type, frame)
    assert output.header.stamp.sec == 42
    assert output.header.frame_id == (frame or "camera_frame")
    assert source.header.frame_id == "camera_frame"
    assert len(output.detections if hasattr(output, "detections") else output.obstacles) == 0


def test_disabled_required_components_launch_synthetic_sources(monkeypatch, tmp_path):
    """Bringup starts stand-ins for perception and zero-command safety."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    calls = []

    def capture_node(**kwargs):
        """Capture the launch specification without starting a ROS process."""
        calls.append(kwargs)
        return object()

    monkeypatch.setattr(actions, "Node", capture_node)
    enabled = {name: False for name in TIERS}
    resolved = config.ResolvedConfig(
        args={}, instrumentation_mode="debug", backend="isaac", slam_mode="off",
        navigation_mode="off", pose_graph_prefix="", map_start_pose=None,
        log_level="info", runtime_behaviour={}, enabled=enabled,
        synthetic=["detector", "safety"],
    )
    actions.build_actions(resolved)
    assert [call["name"] for call in calls] == [
        "synthetic_detector", "synthetic_safety"
    ]
    assert all(call["executable"] == "synthetic_source.py" for call in calls)


def test_unvalidated_variant_combinations_are_rejected():
    """Teleop, unmatched navigation, and camera-only perception fail closed."""
    enabled = {name: False for name in TIERS}
    enabled["teleop"] = True
    with pytest.raises(RuntimeError, match="teleop"):
        config.check_variant_constraints(enabled, "off", "isaac")
    enabled["teleop"] = False
    enabled["navigation"] = True
    with pytest.raises(RuntimeError, match="navigation"):
        config.check_variant_constraints(enabled, "off", "isaac")
    enabled["navigation"] = False
    enabled["detector"] = True
    with pytest.raises(RuntimeError, match="Camera-only"):
        config.check_variant_constraints(enabled, "off", "isaac")
    enabled["lidar_clustering"] = True
    config.check_variant_constraints(enabled, "off", "isaac")
    enabled["drivers"] = True
    with pytest.raises(RuntimeError, match="odom -> base_link"):
        config.check_variant_constraints(enabled, "off", "hardware")
    enabled["localization"] = True
    config.check_variant_constraints(enabled, "off", "hardware")


def test_production_locks_required_and_preserves_variant_defaults():
    """Production does not silently enable navigation or unbuilt teleop."""
    defaults = {name: False for name in TIERS}
    args = {f"enable_{name}": "auto" for name in TIERS}
    resolved = config.resolve_toggles(args, "production", defaults)
    assert resolved["detector"] and resolved["safety"] and resolved["drivers"]
    assert not resolved["navigation"] and not resolved["teleop"]


def test_synthetic_detector_relay_uses_live_ros_topics(monkeypatch, tmp_path):
    """The installed node shape publishes an empty result for an RGB frame."""
    monkeypatch.setenv("ROS_DOMAIN_ID", "187")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init(args=["--ros-args", "-p", "component:=detector"])
    source = None
    probe = None
    executor = SingleThreadedExecutor()
    try:
        source = SyntheticSource()
        probe = rclpy.create_node("synthetic_probe")
        image_pub = probe.create_publisher(
            Image, "/camera/rgb/image_rect_color", qos_profile_sensor_data
        )
        received = []
        probe.create_subscription(
            Detection2DArray, "/perception/detections_2d", received.append, 10
        )
        executor.add_node(source)
        executor.add_node(probe)
        image = Image()
        image.header.stamp.sec = 42
        image.header.frame_id = "camera_frame"
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not received:
            image_pub.publish(image)
            executor.spin_once(timeout_sec=0.1)
        assert received
        assert received[0].header.stamp.sec == 42
        assert received[0].header.frame_id == "camera_frame"
        assert not received[0].detections
    finally:
        executor.shutdown()
        if source is not None:
            source.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
