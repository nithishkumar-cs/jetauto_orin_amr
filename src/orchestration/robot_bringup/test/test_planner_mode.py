import sys
from pathlib import Path

import pytest
import yaml

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


def follow_path_resolved():
    result = resolved("follow_path_test")
    result.backend = "isaac"
    result.enabled["safety"] = True
    return result


def test_unknown_navigation_mode_rejected():
    with pytest.raises(RuntimeError, match="Unknown navigation_mode"):
        config.resolve_navigation_mode({"navigation_mode": "controller"})


def test_planner_scan_source_accepts_lidar_height():
    project_root = Path(__file__).resolve().parents[4]
    planner_config = yaml.safe_load(
        (project_root / "configs/navigation/planner_only.yaml").read_text()
    )
    costmap = planner_config["global_costmap"]["global_costmap"]["ros__parameters"]
    scan = costmap["obstacle_layer"]["scan"]
    assert scan["topic"] == "/scan"
    assert scan["max_obstacle_height"] > 0.24
    assert scan["marking"] is True
    assert scan["clearing"] is True


def test_follow_path_config_uses_live_scan_and_conservative_command():
    project_root = Path(__file__).resolve().parents[4]
    path_config = yaml.safe_load(
        (project_root / "configs/navigation/follow_path_test.yaml").read_text()
    )
    controller = path_config["controller_server"]["ros__parameters"]
    costmap = path_config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert controller["odom_topic"] == "/odom/wheel"
    assert controller["FollowPath"]["desired_linear_vel"] <= 0.10
    assert controller["FollowPath"]["use_collision_detection"] is True
    assert costmap["global_frame"] == "odom"
    # Nav2 Humble declares these dimensions as integers and aborts if YAML
    # supplies doubles (3.0), even though both denote three metres.
    assert type(costmap["width"]) is int
    assert type(costmap["height"]) is int
    assert costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert costmap["obstacle_layer"]["scan"]["max_obstacle_height"] > 0.24
    assert path_config["lifecycle_manager_controller"]["ros__parameters"][
        "node_names"
    ] == ["controller_server"]


def test_planner_localizer_tf_override_is_scoped():
    project_root = Path(__file__).resolve().parents[4]
    base = yaml.safe_load(
        (project_root / "configs/localization/slam_localization.yaml").read_text()
    )
    override = yaml.safe_load(
        (project_root / "configs/localization/slam_localization_planner_only.yaml").read_text()
    )
    assert base["slam_toolbox"]["ros__parameters"]["transform_timeout"] == 0.2
    assert override["slam_toolbox"]["ros__parameters"]["transform_timeout"] == 0.8


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
        calls.append(
            (spec["executable"], use_sim_time, extra_parameters, spec.get("parameters", []))
        )
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
    assert calls[0][3][-1].endswith("slam_localization_planner_only.yaml")


def test_navigation_mode_off_does_not_launch_planner(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append((spec["executable"], spec.get("parameters", [])))
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(resolved("off"))
    assert [call[0] for call in calls] == ["localization_slam_toolbox_node"]
    assert calls[0][1][-1].endswith("slam_localization.yaml")


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


def test_follow_path_mode_requires_isaac_localization_safety_and_no_drivers():
    selected = follow_path_resolved()
    config.check_navigation_constraints(
        selected.navigation_mode, selected.backend, selected.slam_mode, selected.enabled
    )
    for backend, slam_mode, change, message in [
        ("hardware", "localization", None, "restricted to the isaac backend"),
        ("rosbag", "localization", None, "restricted to the isaac backend"),
        ("isaac", "mapping", None, "slam_mode=localization"),
        ("isaac", "localization", ("navigation", False), "enable_navigation=true"),
        ("isaac", "localization", ("drivers", True), "drivers and teleop disabled"),
        ("isaac", "localization", ("teleop", True), "drivers and teleop disabled"),
        ("isaac", "localization", ("safety", False), "enable_safety=true"),
    ]:
        enabled = selected.enabled.copy()
        if change:
            enabled[change[0]] = change[1]
        with pytest.raises(RuntimeError, match=message):
            config.check_navigation_constraints("follow_path_test", backend, slam_mode, enabled)


def test_follow_path_launch_is_guarded_and_ends_at_safety_topic(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(actions, "package_available", lambda _: True)
    calls = []

    def capture(spec, log_level, use_sim_time, backend, extra_parameters=None):
        calls.append((spec["executable"], use_sim_time, spec.get("remappings", [])))
        return object()

    monkeypatch.setattr(actions, "_node_action", capture)
    actions.build_actions(follow_path_resolved())
    assert [call[0] for call in calls] == [
        "stop_zone_recovery_node",
        "scan_health_gate_node",
        "estop_gate_node",
        "collision_monitor",
        "lifecycle_manager",
        "localization_slam_toolbox_node",
        "planner_server",
        "lifecycle_manager",
        "controller_server",
        "lifecycle_manager",
    ]
    assert all(call[1] for call in calls)
    assert calls[-2][2] == [("cmd_vel", "/cmd_vel")]
    assert "bt_navigator" not in [call[0] for call in calls]


def test_follow_path_does_not_start_controller_without_safety(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        actions, "package_available", lambda package: package != "scan_health_gate"
    )
    calls = []
    monkeypatch.setattr(
        actions, "_node_action",
        lambda spec, *args, **kwargs: calls.append(spec["executable"]) or object(),
    )
    actions.build_actions(follow_path_resolved())
    assert calls == ["localization_slam_toolbox_node"]
