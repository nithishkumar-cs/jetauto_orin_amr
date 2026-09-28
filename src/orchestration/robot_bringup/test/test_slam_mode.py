import sys
from pathlib import Path

import pytest
from launch.actions import IncludeLaunchDescription

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch_lib import actions, config  # noqa: E402
from launch_lib.tiers import TIERS  # noqa: E402


def resolved(slam_mode, backend="isaac"):
    return config.ResolvedConfig(
        args={},
        instrumentation_mode="debug",
        backend=backend,
        slam_mode=slam_mode,
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
    assert dict(includes[0].launch_arguments) == {
        "backend": "isaac", "log_level": "info"
    }


def test_unknown_slam_mode_rejected():
    with pytest.raises(RuntimeError, match="Unknown slam_mode"):
        config.resolve_slam_mode({"slam_mode": "localization"})


def test_hardware_mapping_waits_for_ekf(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        actions, "package_available", lambda package: package != "robot_localization"
    )
    hardware = resolved("mapping", backend="hardware")
    hardware.enabled["localization"] = True

    selected = actions.build_actions(hardware)
    assert not any(isinstance(action, IncludeLaunchDescription) for action in selected)
