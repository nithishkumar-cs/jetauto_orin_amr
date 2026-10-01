from pathlib import Path

from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_lib.config import ResolvedConfig
from launch_lib.paths import package_available, package_file, project_config, read_yaml
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
    "detector": [
        {
            "package": "perception_detector",
            "executable": "detector_node",
            "name": "perception_detector",
            "parameters": [str(project_config("perception", "detector.yaml"))],
            # The node is required and fail-fast. Report it as pending if its
            # device-local engine or matching labels are ever removed.
            "required_file_parameters": ["engine_path", "labels_path"],
        },
    ],
    "geometry": [
        {
            "package": "perception_detection_depth_projection",
            "executable": "detection_depth_projection_node",
            "name": "detection_depth_projection",
            "parameters": [str(project_config("perception", "detection_depth_projection.yaml"))],
        },
    ],
    "lidar_clustering": [
        {
            "package": "perception_lidar_clustering",
            "executable": "lidar_clustering_node",
            "name": "perception_lidar_clustering",
            "parameters": [str(project_config("perception", "lidar_clustering.yaml"))],
        },
    ],
    # Camera and LiDAR observations are tracked independently so identities are
    # stable before the fusion stage decides whether the two streams describe
    # the same physical obstacle.
    "tracking": [
        {
            "package": "perception_tracking",
            "executable": "tracking_node",
            "name": "perception_camera_tracker",
            "parameters": [str(project_config("perception", "tracking.yaml"))],
        },
        {
            "package": "perception_tracking",
            "executable": "tracking_node",
            "name": "perception_lidar_tracker",
            "parameters": [str(project_config("perception", "tracking.yaml"))],
        },
    ],
    "fusion": [
        {
            "package": "perception_fusion",
            "executable": "fusion_node",
            "name": "perception_fusion",
            "parameters": [str(project_config("perception", "fusion.yaml"))],
        },
    ],
    "localization": [
        {
            "package": "localization_odometry_fusion",
            "launch_file": "odometry_fusion.launch.py",
            "required_packages": ["robot_localization"],
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

SLAM_MAPPING_SPEC = {
    "package": "localization_slam_mapping",
    "launch_file": "slam_mapping.launch.py",
    "required_packages": ["slam_toolbox"],
}

SLAM_LOCALIZATION_SPEC = {
    "package": "slam_toolbox",
    "executable": "localization_slam_toolbox_node",
    "name": "slam_toolbox",
    "parameters": [str(project_config("localization", "slam_localization.yaml"))],
}


def _node_action(
    spec: dict,
    log_level: str,
    use_sim_time: bool,
    backend: str,
    extra_parameters: dict | None = None,
):
    if "launch_file" in spec:
        launch_file = package_file(spec["package"], "launch", spec["launch_file"])
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)),
            launch_arguments={"backend": backend, "log_level": log_level}.items(),
        )
    return Node(
        package=spec["package"],
        executable=spec["executable"],
        name=spec.get("name", spec["executable"]),
        output="screen",
        parameters=[
            *spec.get("parameters", []),
            {"use_sim_time": use_sim_time, **(extra_parameters or {})},
        ],
        remappings=spec.get("remappings", []),
        arguments=["--ros-args", "--log-level", log_level],
    )


def _missing_required_files(spec: dict) -> list[str]:
    required_names = spec.get("required_file_parameters", [])
    if not required_names:
        return []

    parameter_values = {}
    for parameter_source in spec.get("parameters", []):
        if not isinstance(parameter_source, str):
            continue
        config = read_yaml(Path(parameter_source))
        node_config = config.get(spec.get("name", spec["executable"]), {})
        parameter_values.update(node_config.get("ros__parameters", {}))

    missing = []
    for name in required_names:
        value = parameter_values.get(name)
        if not isinstance(value, str) or not value or not Path(value).is_file():
            missing.append(f"{name}={value or '<not configured>'}")
    return missing


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
        required_packages = [
            package
            for spec in specs
            for package in [spec["package"], *spec.get("required_packages", [])]
        ]
        missing = [package for package in required_packages if not package_available(package)]
        if missing:
            pending.append(f"{name} (packages not built: {', '.join(sorted(set(missing)))})")
            continue

        missing_files = [
            missing_file for spec in specs for missing_file in _missing_required_files(spec)
        ]
        if missing_files:
            pending.append(f"{name} (runtime files missing: {', '.join(missing_files)})")
            continue

        actions.extend(
            _node_action(spec, resolved.log_level, resolved.backend == "isaac", resolved.backend)
            for spec in specs
        )
        launched.append(name)

    if resolved.enabled["drivers"]:
        if resolved.backend == "isaac":
            actions.append(
                LogInfo(
                    msg=(
                        "robot_bringup: backend=isaac launches no adapter. "
                        "Configure Isaac's ROS 2 graph to use the canonical "
                        "AMR topics directly."
                    )
                )
            )
            launched.append("backend:isaac")
        elif resolved.backend == "rosbag":
            pending.append(
                "drivers (rosbag backend selected; replay launch waits for a bag manifest "
                "and /clock policy)"
            )
        else:
            spec = MOTION_BACKEND_SPECS["hardware"]
            if not package_available(spec["package"]):
                pending.append(f"drivers ({resolved.backend} package not built: {spec['package']})")
            else:
                actions.append(_node_action(spec, resolved.log_level, False, resolved.backend))
                launched.append(f"drivers:{resolved.backend}")

    if resolved.slam_mode in ("mapping", "localization"):
        spec = SLAM_MAPPING_SPEC if resolved.slam_mode == "mapping" else SLAM_LOCALIZATION_SPEC
        required = [spec["package"], *spec.get("required_packages", [])]
        if resolved.backend == "hardware":
            required.extend(("localization_odometry_fusion", "robot_localization"))
        missing = [package for package in required if not package_available(package)]
        if missing:
            pending.append(f"{resolved.slam_mode} (packages not built: {', '.join(missing)})")
        else:
            actions.append(
                _node_action(
                    spec,
                    resolved.log_level,
                    resolved.backend != "hardware",
                    resolved.backend,
                    (
                        {
                            "map_file_name": resolved.pose_graph_prefix,
                            "map_start_pose": list(resolved.map_start_pose),
                        }
                        if resolved.slam_mode == "localization"
                        else None
                    ),
                )
            )
            launched.append(resolved.slam_mode)

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
