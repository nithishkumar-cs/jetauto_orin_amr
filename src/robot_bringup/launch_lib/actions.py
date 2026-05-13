from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_lib.config import ResolvedConfig
from launch_lib.paths import package_file


def build_actions(resolved: ResolvedConfig) -> list:
    args = resolved.args
    node_arguments = ["--ros-args", "--log-level", resolved.log_level]

    actions = [
        LogInfo(
            msg=(
                f"robot_bringup mode={resolved.instrumentation_mode} "
                f"config_set={resolved.config_set_name} detector={resolved.detector_backend} "
                f"base={resolved.enable_base_interface} sensors={resolved.enable_sensor_drivers} "
                f"depth={resolved.enable_depth_pipeline} localization={resolved.enable_localization} "
                f"navigation={resolved.enable_navigation_tasks}"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_file("tf_and_calibration", "launch", "static_tf.launch.py"))),
        ),
    ]

    if resolved.enable_sensor_drivers:
        actions.append(
            Node(
                package="sensor_drivers",
                executable="usb_camera_node",
                name="rgb_camera_node",
                output="screen",
                parameters=[args["camera_params"]],
                remappings=[
                    ("image_raw", "/sensors/rgb/image_raw"),
                    ("camera_info", "/sensors/rgb/camera_info"),
                    ("health", "/diagnostics/rgb_camera/health"),
                ],
                arguments=node_arguments,
            )
        )

    if resolved.enable_base_interface:
        actions.append(
            Node(
                package="base_interface",
                executable="jetauto_base_interface_node",
                name="jetauto_base_interface_node",
                output="screen",
                parameters=[args["base_params"], {"transport_mode": resolved.base_transport_mode}],
                arguments=node_arguments,
            )
        )

    if resolved.enable_teleop:
        actions.append(
            Node(
                package="teleop_tools",
                executable="twist_safety_relay_node",
                name="twist_safety_relay_node",
                output="screen",
                parameters=[args["teleop_params"]],
                arguments=node_arguments,
            )
        )

    if resolved.enable_preproc:
        actions.append(
            Node(
                package="perception_preproc",
                executable="preproc_node",
                name="preproc_node",
                output="screen",
                parameters=[args["preproc_params"]],
                arguments=node_arguments,
            )
        )

    actions.extend([
        Node(
            package="perception_inference",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[str(resolved.detector_params)],
            arguments=node_arguments,
        ),
        Node(
            package="tf_and_calibration",
            executable="calibration_validator_node",
            name="calibration_validator_node",
            output="screen",
            parameters=[args["calibration_params"]],
            arguments=node_arguments,
        ),
    ])

    if resolved.enable_depth_pipeline:
        actions.extend([
            Node(
                package="perception_geometry",
                executable="depth_obstacle_projector_node",
                name="depth_obstacle_projector_node",
                output="screen",
                parameters=[args["geometry_params"]],
                arguments=node_arguments,
            ),
            Node(
                package="tracking_fusion",
                executable="obstacle_tracker_node",
                name="obstacle_tracker_node",
                output="screen",
                parameters=[args["tracking_params"]],
                arguments=node_arguments,
            ),
            Node(
                package="sensor_fusion",
                executable="obstacle_fusion_node",
                name="obstacle_fusion_node",
                output="screen",
                parameters=[args["fusion_params"]],
                arguments=node_arguments,
            ),
            Node(
                package="safety_layer",
                executable="safety_monitor_node",
                name="safety_monitor_node",
                output="screen",
                parameters=[args["safety_params"]],
                arguments=node_arguments,
            ),
        ])

    if resolved.enable_localization:
        if resolved.localization_mode == "mapping":
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(package_file("localization_mapping", "launch", "mapping.launch.py"))),
                    launch_arguments={"slam_params": args["slam_params"]}.items(),
                )
            )
        else:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(package_file("localization_mapping", "launch", "navigation.launch.py"))),
                    launch_arguments={
                        "params_file": args["nav2_params"],
                        "map": args["map_file"],
                    }.items(),
                )
            )

    if resolved.enable_navigation_tasks:
        actions.append(
            Node(
                package="navigation_tasks",
                executable="waypoint_task_manager_node",
                name="waypoint_task_manager_node",
                output="screen",
                parameters=[args["navigation_task_params"]],
                arguments=node_arguments,
            )
        )

    actions.append(
        Node(
            package="logging_and_diagnostics",
            executable="health_aggregator_node",
            name="health_aggregator_node",
            output="screen",
            parameters=[args["health_params"], {"health_topics": resolved.health_topics}],
            arguments=node_arguments,
        )
    )

    if resolved.enable_benchmarks:
        actions.append(
            Node(
                package="benchmarks",
                executable="latency_probe_node",
                name="latency_probe_node",
                output="screen",
                parameters=[args["latency_params"]],
                arguments=node_arguments,
            )
        )

    return actions
