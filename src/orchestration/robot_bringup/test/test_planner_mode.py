import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch_lib import actions, config  # noqa: E402
from launch_lib.tiers import TIERS  # noqa: E402


def resolved(navigation_mode):
    enabled = {name: False for name in TIERS}
    enabled["navigation"] = True
    return config.ResolvedConfig(
        args={},
        instrumentation_mode="debug",
        backend="rosbag",
        slam_mode="localization",
        navigation_mode=navigation_mode,
        pose_graph_prefix="/tmp/test_map",
        map_start_pose=(-0.5, -1.0, 0.0),
        log_level="info",
        runtime_behaviour={},
        enabled=enabled,
        synthetic=[],
    )


def test_unknown_navigation_mode_rejected():
    with pytest.raises(RuntimeError, match="Unknown navigation_mode"):
        config.resolve_navigation_mode({"navigation_mode": "controller"})


@pytest.mark.parametrize("backend", ["isaac", "rosbag"])
def test_planner_only_allows_simulation_backends(backend):
    enabled = resolved("planner_only").enabled
    config.check_navigation_constraints("planner_only", backend, "localization", enabled)


@pytest.mark.parametrize(
    ("backend", "slam_mode", "disabled", "message"),
    [
        ("hardware", "localization", None, "validated only"),
        ("rosbag", "mapping", None, "slam_mode=localization"),
        ("rosbag", "localization", "navigation", "enable_navigation=true"),
        ("rosbag", "localization", "drivers", "drivers and teleop disabled"),
        ("rosbag", "localization", "teleop", "drivers and teleop disabled"),
    ],
)
def test_planner_only_rejects_unsafe_selection(backend, slam_mode, disabled, message):
    enabled = resolved("planner_only").enabled
    if disabled == "navigation":
        enabled[disabled] = False
    elif disabled:
        enabled[disabled] = True
    with pytest.raises(RuntimeError, match=message):
        config.check_navigation_constraints("planner_only", backend, slam_mode, enabled)


def test_planner_only_launches_only_localizer_planner_and_manager(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append((spec["executable"], use_sim_time, extra_parameters))
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(resolved("planner_only"))

    assert [call[0] for call in calls] == [
        "localization_slam_toolbox_node",
        "planner_server",
        "lifecycle_manager",
    ]
    assert all(call[1] for call in calls)
    assert all(call[0] not in ("controller_server", "bt_navigator") for call in calls)


def test_navigation_mode_off_does_not_launch_planner(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append(spec["executable"])
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(resolved("off"))
    assert calls == ["localization_slam_toolbox_node"]


def test_missing_planner_package_does_not_start_planner_manager(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        actions, "package_available", lambda package: package != "nav2_navfn_planner"
    )
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append(spec["executable"])
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(resolved("planner_only"))
    assert calls == ["localization_slam_toolbox_node"]


def test_missing_localizer_does_not_start_planner(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda package: package != "slam_toolbox")
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append(spec["executable"])
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(resolved("planner_only"))
    assert calls == []
