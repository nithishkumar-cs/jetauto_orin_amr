import sys
from pathlib import Path

import pytest
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch_lib import actions, config  # noqa: E402
from launch_lib.tiers import TIERS  # noqa: E402


def resolved(slam_mode, backend="isaac"):
    return config.ResolvedConfig(
        args={},
        instrumentation_mode="debug",
        backend=backend,
        slam_mode=slam_mode,
        pose_graph_prefix="/tmp/test_map",
        map_start_pose=(-0.5, -1.0, 0.0),
        log_level="info",
        runtime_behaviour={},
        enabled={name: False for name in TIERS},
        synthetic=[],
    )


def test_slam_mapping_is_opt_in(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    off_actions = actions.build_actions(resolved("off"))
    mapping_actions = actions.build_actions(resolved("mapping"))

    assert not any(isinstance(action, IncludeLaunchDescription) for action in off_actions)
    includes = [
        action for action in mapping_actions if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    assert dict(includes[0].launch_arguments) == {"backend": "isaac", "log_level": "info"}


def test_unknown_slam_mode_rejected():
    with pytest.raises(RuntimeError, match="Unknown slam_mode"):
        config.resolve_slam_mode({"slam_mode": "invalid"})


def test_localization_requires_complete_serialized_pose_graph(tmp_path):
    prefix = tmp_path / "test_map"
    args = {"pose_graph_prefix": str(prefix)}
    with pytest.raises(RuntimeError, match="missing"):
        config.resolve_pose_graph_prefix(args, "localization")

    (tmp_path / "test_map.posegraph").touch()
    with pytest.raises(RuntimeError, match="test_map.data"):
        config.resolve_pose_graph_prefix(args, "localization")

    (tmp_path / "test_map.data").touch()
    assert config.resolve_pose_graph_prefix(args, "localization") == str(prefix)
    assert config.resolve_pose_graph_prefix(args, "mapping") == ""


@pytest.mark.parametrize("value", ["", "relative/map", "/tmp/map.posegraph", "/tmp/map.yaml"])
def test_localization_rejects_invalid_pose_graph_prefix(value):
    with pytest.raises(RuntimeError, match="pose_graph_prefix"):
        config.resolve_pose_graph_prefix({"pose_graph_prefix": value}, "localization")


@pytest.mark.parametrize(
    "value",
    ["", "[0, 0]", "[0, 0, true]", "[0, 0, .nan]", "[0, 0, 0, 0]", "not a pose"],
)
def test_localization_rejects_invalid_start_pose(value):
    with pytest.raises(RuntimeError, match="map_start_pose"):
        config.resolve_map_start_pose({"map_start_pose": value}, "localization")


def test_localization_parses_start_pose():
    args = {"map_start_pose": "[-0.5, -1, 0.0]"}
    assert config.resolve_map_start_pose(args, "localization") == (-0.5, -1.0, 0.0)
    assert config.resolve_map_start_pose(args, "off") is None


def test_localization_selects_node_and_map_prefix(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append((spec, log_level, use_sim_time, backend, extra_parameters))
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    selected = actions.build_actions(resolved("localization", backend="rosbag"))

    assert selected
    assert len(calls) == 1
    spec, level, sim_time, backend, parameters = calls[0]
    assert spec["executable"] == "localization_slam_toolbox_node"
    assert (level, sim_time, backend) == ("info", True, "rosbag")
    assert parameters == {
        "map_file_name": "/tmp/test_map",
        "map_start_pose": [-0.5, -1.0, 0.0],
    }


def test_hardware_mapping_waits_for_ekf(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        actions, "package_available", lambda package: package != "robot_localization"
    )
    hardware = resolved("mapping", backend="hardware")
    hardware.enabled["localization"] = True

    selected = actions.build_actions(hardware)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in selected)


def test_hardware_localization_waits_for_ekf(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        actions, "package_available", lambda package: package != "robot_localization"
    )
    hardware = resolved("localization", backend="hardware")
    hardware.enabled["localization"] = True

    selected = actions.build_actions(hardware)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in selected)
    assert not any(isinstance(action, Node) for action in selected)
