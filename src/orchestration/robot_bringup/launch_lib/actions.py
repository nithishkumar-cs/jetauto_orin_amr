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
    "geometry": [
        {
            "package": "perception_detection_depth_projection",
            "executable": "detection_depth_projection_node",
            "name": "detection_depth_projection",
            "parameters": [str(project_config("perception", "detection_depth_projection.yaml"))],
        },
    ],
    # Obstacle-zone half of the safety chain. Upstream package, so it is
    # available as soon as ros-humble-nav2-collision-monitor is installed —
    # unlike our own packages it does not wait on the rebuild.
    #
    #   /cmd_vel -> collision_monitor -> /cmd_vel/collision_limited
    #            -> estop_gate (operator stop) -> /cmd_vel/safety_limited -> driver
    #
    # It has no source configured yet (no sensor driver exists), so it will run
    # and gate nothing until drivers/ returns and /scan appears.
    "safety": [
        {
            "package": "estop_gate",
            "executable": "estop_gate_node",
            "name": "estop_gate",
            "parameters": [str(project_config("safety", "estop_gate.yaml"))],
        },
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
}

# Hardware is the only backend that launches an AMR-side motion-edge node.
# Isaac's own ROS 2 graph is configured to use our canonical topics directly;
# rosbag replay will be added once its manifest and /clock policy exist.
MOTION_BACKEND_SPECS: dict = {
    "hardware": {
        "package": "base_driver",
        "executable": "base_driver_node",
        "name": "base_driver",
    },
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
                f"backend={resolved.backend} "
                f"log_level={resolved.log_level} "
                f"enabled=[{', '.join(sorted(on))}] "
                f"disabled=[{', '.join(off)}]"
            )
        )
    ]

    launched, pending = [], []
    for name in sorted(on):
        # This required component is selected by the backend, below, rather
        # than by a generic node entry.
        if name == "drivers":
            continue
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

    if resolved.enabled["drivers"]:
        if resolved.backend == "isaac":
            actions.append(
                LogInfo(
                    msg=(
                        "robot_bringup: backend=isaac launches no adapter. Configure Isaac's ROS 2 "
                        "graph to use the canonical AMR topics directly."
                    )
                )
            )
            launched.append("backend:isaac")
        elif resolved.backend == "rosbag":
            pending.append(
                "drivers (rosbag backend selected; replay launch waits for a bag manifest and /clock policy)"
            )
        else:
            spec = MOTION_BACKEND_SPECS["hardware"]
            if not package_available(spec["package"]):
                pending.append(f"drivers ({resolved.backend} package not built: {spec['package']})")
            else:
                actions.append(_node_action(spec, resolved.log_level))
                launched.append(f"drivers:{resolved.backend}")

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
