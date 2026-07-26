from launch.actions import LogInfo
from launch_ros.actions import Node

from launch_lib.config import ResolvedConfig
from launch_lib.paths import package_available, project_config
from launch_lib.tiers import TIERS

#: toggle name -> LIST of node specs, for components that have been rebuilt.
#: A list because one component can need several processes (see "safety").
#:
#: The stack is being rebuilt package by package. A toggle with no entry here is
#: enabled-but-not-yet-implemented: bringup reports it and carries on instead of
#: throwing from get_package_share_directory(), which is what made the whole
#: launch file unusable after the reset.
#:
#: Add an entry as each package returns, e.g.
#:     "detector": {
#:         "package": "perception_inference",
#:         "executable": "detector_node",
#:     },
NODE_SPECS: dict = {
    # Obstacle-zone half of the safety chain. Upstream package, so it is
    # available as soon as ros-humble-nav2-collision-monitor is installed —
    # unlike our own packages it does not wait on the rebuild.
    #
    #   /cmd_vel -> safety_gate (e-stop) -> /cmd_vel_raw
    #            -> collision_monitor    -> /cmd_vel/safety_limited -> driver
    #
    # It has no source configured yet (no sensor driver exists), so it will run
    # and gate nothing until drivers/ returns and /scan appears.
    "safety": [
        {
            "package": "nav2_collision_monitor",
            "executable": "collision_monitor",
            "name": "collision_monitor",
            "parameters": [str(project_config("safety", "collision_monitor.yaml"))],
        },
        # Required. collision_monitor is a lifecycle node: on its own it stays
        # `unconfigured` and gates nothing, while still appearing in
        # `ros2 node list`. This drives it to `active`.
        {
            "package": "nav2_lifecycle_manager",
            "executable": "lifecycle_manager",
            "name": "lifecycle_manager_safety",
            "parameters": [str(project_config("safety", "collision_monitor.yaml"))],
        },
    ],
    # safety_gate ships the e-stop latch library only — no node yet, so there is
    # deliberately no entry for it. When the node lands it joins this chain
    # UPSTREAM of collision_monitor, publishing /cmd_vel_raw.
}


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
        specs = NODE_SPECS.get(name)
        if not specs:
            pending.append(name)
            continue

        # A component may need more than one process — collision_monitor is a
        # lifecycle node and is inert without its lifecycle manager, so the two
        # are launched together or not at all.
        missing = [s["package"] for s in specs if not package_available(s["package"])]
        if missing:
            pending.append(f"{name} (packages not built: {', '.join(sorted(set(missing)))})")
            continue

        actions.extend(_node_action(spec, resolved.log_level) for spec in specs)
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
