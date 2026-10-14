import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from launch_lib import actions  # noqa: E402


def test_safety_chain_has_scan_interlock_before_estop():
    root = Path(__file__).resolve().parents[4]
    config_dir = root / "configs" / "safety"

    def params(file_name, node_name):
        return yaml.safe_load((config_dir / file_name).read_text())[
            node_name
        ]["ros__parameters"]

    collision = params("collision_monitor.yaml", "collision_monitor")
    recovery = params("stop_zone_recovery.yaml", "stop_zone_recovery")
    health = params("scan_health_gate.yaml", "scan_health_gate")
    estop = params("estop_gate.yaml", "estop_gate")

    assert collision["cmd_vel_in_topic"] == "/cmd_vel"
    assert collision["cmd_vel_out_topic"] == recovery["limited_cmd_vel_in_topic"]
    assert recovery["raw_cmd_vel_in_topic"] == collision["cmd_vel_in_topic"]
    assert recovery["cmd_vel_out_topic"] == health["cmd_vel_in_topic"]
    assert recovery["scan_in_topic"] == collision["scan"]["topic"]
    assert 0.0 < recovery["max_reverse_speed"] <= 0.05
    assert health["cmd_vel_out_topic"] == estop["cmd_vel_in_topic"]
    assert estop["cmd_vel_out_topic"] == "/cmd_vel/safety_limited"
    assert health["scan_in_topic"] == collision["scan"]["topic"]
    assert health["scan_timeout_sec"] == collision["source_timeout"]

    safety_nodes = [spec["name"] for spec in actions.NODE_SPECS["safety"]]
    assert safety_nodes == [
        "stop_zone_recovery", "scan_health_gate", "estop_gate", "collision_monitor",
        "lifecycle_manager_safety",
    ]
