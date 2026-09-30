"""Pure calculations and input checks used by the odometry-drift bag tools."""

import importlib.util
import math
from pathlib import Path

import pytest

pytest.importorskip("rosbag2_py")
from geometry_msgs.msg import Quaternion  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def load_tool(name):
    path = ROOT / "rosbag" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_yaw_drift_preserves_rotation_norm():
    tool = load_tool("inject_odometry_tf_drift")
    quaternion = Quaternion(z=math.sin(0.2), w=math.cos(0.2))

    tool.apply_yaw_drift(quaternion, 0.12)

    assert math.atan2(2 * quaternion.w * quaternion.z, 1 - 2 * quaternion.z**2) == (
        pytest.approx(0.52)
    )
    assert sum(
        value**2 for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    ) == (pytest.approx(1.0))


def test_inject_rejects_existing_output(tmp_path):
    tool = load_tool("inject_odometry_tf_drift")
    with pytest.raises(ValueError, match="already exists"):
        tool.inject(tmp_path / "source", tmp_path, 0.6, -0.3, 0.12, 0.0, 45.0)


def test_inject_rejects_nonfinite_ramp_duration(tmp_path):
    tool = load_tool("inject_odometry_tf_drift")
    with pytest.raises(ValueError, match="finite"):
        tool.inject(tmp_path / "source", tmp_path / "output", 0.6, -0.3, 0.12, 0.0, math.nan)


def test_analyzer_percentiles_and_nearest_sample():
    tool = load_tool("analyze_saved_map_localization")
    assert tool.metric([5, 1, 4, 3, 2]) == {"count": 5, "median": 3, "p95": 5, "max": 5}
    samples = [(1.0, "first"), (2.0, "second")]
    assert tool.nearest(samples, [1.0, 2.0], 1.6) == samples[1]


def test_analyzer_rejects_unpaired_perturbed_tf(monkeypatch):
    tool = load_tool("analyze_saved_map_localization")
    monkeypatch.setattr(tool, "read_map_alignment", lambda _: (0.0, 0.0, 0.0))
    monkeypatch.setattr(tool, "read_odometry", lambda _: [(1.0, 0.0, 0.0, 0.0)])
    monkeypatch.setattr(tool, "read_poses", lambda _: [(1.0, 0.0, 0.0, 0.0)])
    monkeypatch.setattr(tool, "read_perturbed_tf", lambda _: [(2.0, 0.0, 0.0, 0.0)])
    with pytest.raises(ValueError, match="No poses match perturbed TF"):
        tool.analyze("map", "reference", "pose", "perturbed", 0.15)
