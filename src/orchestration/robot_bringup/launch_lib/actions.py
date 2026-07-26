from launch.actions import LogInfo
from launch_ros.actions import Node

from launch_lib.config import ResolvedConfig
from launch_lib.paths import package_available
from launch_lib.tiers import TIERS

#: toggle name -> node spec, for components that have been rebuilt.
#:
#: The stack is being rebuilt package by package. A toggle with no entry here is
#: enabled-but-not-yet-implemented: bringup reports it and carries on instead of
#: throwing from get_package_share_directory(), which is what made the whole
#: launch file unusable after the reset.
#:
#: Add an entry as each package returns, e.g.
#:     "safety": {
#:         "package": "safety_gate",
#:         "executable": "safety_gate_node",
#:         "name": "safety_gate_node",
#:     },
#: safety_gate currently ships the policy library only — no node yet.
NODE_SPECS: dict = {}


def _node_action(spec: dict, log_level: str) -> Node:
    return Node(
        package=spec["package"],
        executable=spec["executable"],
        name=spec.get("name", spec["executable"]),
        output="screen",
        parameters=spec.get("parameters", []),
        remappings=spec.get("remappings", []),
        arguments=["--ros-args", "--log-level", log_level],
    )


def build_actions(resolved: ResolvedConfig) -> list:
    on = [name for name, enabled in resolved.enabled.items() if enabled]
    off = resolved.synthetic

    actions = [
        LogInfo(
            msg=(
                f"robot_bringup mode={resolved.instrumentation_mode} "
                f"log_level={resolved.log_level} "
                f"enabled=[{', '.join(sorted(on))}] "
                f"disabled=[{', '.join(off)}]"
            )
        )
    ]

    launched, pending = [], []
    for name in sorted(on):
        spec = NODE_SPECS.get(name)
        if spec is None:
            pending.append(name)
            continue
        if not package_available(spec["package"]):
            pending.append(f"{name} (package '{spec['package']}' not built)")
            continue
        actions.append(_node_action(spec, resolved.log_level))
        launched.append(name)

    if pending:
        actions.append(
            LogInfo(
                msg=(
                    "robot_bringup: enabled but not yet rebuilt, nothing launched for "
                    f"[{', '.join(pending)}]"
                )
            )
        )

    # Synthetic substitution: a `false` toggle is meant to spawn a stand-in
    # publisher on that component's output contract so downstream nodes still
    # receive data. That needs the contracts to exist, which means the packages
    # have to come back first. Until then the intent is reported, not silently
    # dropped.
    for name in off:
        actions.append(
            LogInfo(
                msg=(
                    f"robot_bringup: '{name}' ({TIERS[name]}) is disabled — "
                    "synthetic substitution is not implemented yet"
                )
            )
        )

    if not launched:
        actions.append(
            LogInfo(
                msg=(
                    "robot_bringup: no nodes launched. The stack is mid-rebuild; "
                    "register packages in launch_lib/actions.py NODE_SPECS as they return."
                )
            )
        )

    return actions
