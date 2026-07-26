from launch.actions import DeclareLaunchArgument

from launch_lib.paths import package_file, share_path

ARGUMENT_NAMES = [
    "instrumentation_mode",
    "detector_backend",
    "enable_sensor_drivers",
    "enable_preproc",
    "enable_depth_pipeline",
    "enable_teleop",
    "enable_localization",
    "enable_navigation_tasks",
    "enable_benchmarks",
    "localization_mode",
    "map_file",
    "system_modes_config",
    "runtime_modes_config",
    "robot_profile_config",
    "topic_contracts_config",
    "calibration_manifest",
    "camera_params",
    "preproc_params",
    "detector_params",
    "geometry_params",
    "tracking_params",
    "fusion_params",
    "safety_params",
    "calibration_params",
    "health_params",
    "latency_params",
    "teleop_params",
    "navigation_task_params",
    "slam_params",
    "nav2_params",
]


def declared_arguments() -> list:
    bringup_share = share_path("robot_bringup")
    project_config_root = bringup_share / "project_configs"

    return [
        DeclareLaunchArgument("instrumentation_mode", default_value="auto"),
        DeclareLaunchArgument("detector_backend", default_value="auto"),
        DeclareLaunchArgument("enable_sensor_drivers", default_value="auto"),
        DeclareLaunchArgument("enable_preproc", default_value="auto"),
        DeclareLaunchArgument("enable_depth_pipeline", default_value="auto"),
        DeclareLaunchArgument("enable_teleop", default_value="auto"),
        DeclareLaunchArgument("enable_localization", default_value="auto"),
        DeclareLaunchArgument("enable_navigation_tasks", default_value="auto"),
        DeclareLaunchArgument("enable_benchmarks", default_value="auto"),
        DeclareLaunchArgument("localization_mode", default_value="auto"),
        DeclareLaunchArgument("map_file", default_value=""),
        DeclareLaunchArgument(
            "system_modes_config", default_value=str(bringup_share / "system_modes.yaml")
        ),
        DeclareLaunchArgument(
            "runtime_modes_config",
            default_value=str(project_config_root / "runtime_modes" / "runtime_modes.yaml"),
        ),
        DeclareLaunchArgument(
            "robot_profile_config",
            default_value=str(project_config_root / "robot" / "robot_profile.yaml"),
        ),
        DeclareLaunchArgument(
            "topic_contracts_config",
            default_value=str(project_config_root / "sensors" / "topic_contracts.yaml"),
        ),
        DeclareLaunchArgument(
            "calibration_manifest",
            default_value=str(project_config_root / "calibration" / "calibration_manifest.yaml"),
        ),
        DeclareLaunchArgument(
            "camera_params",
            default_value=str(package_file("sensor_drivers", "config", "rgb_camera.yaml")),
        ),
        DeclareLaunchArgument(
            "preproc_params",
            default_value=str(package_file("perception_preproc", "config", "yolo_preproc.yaml")),
        ),
        DeclareLaunchArgument("detector_params", default_value=""),
        DeclareLaunchArgument(
            "geometry_params",
            default_value=str(
                package_file("perception_geometry", "config", "depth_projection.yaml")
            ),
        ),
        DeclareLaunchArgument(
            "tracking_params",
            default_value=str(package_file("tracking_fusion", "config", "tracking.yaml")),
        ),
        DeclareLaunchArgument(
            "fusion_params",
            default_value=str(package_file("sensor_fusion", "config", "fusion.yaml")),
        ),
        DeclareLaunchArgument(
            "safety_params",
            default_value=str(package_file("safety_layer", "config", "safety.yaml")),
        ),
        DeclareLaunchArgument(
            "calibration_params",
            default_value=str(
                package_file("tf_and_calibration", "config", "calibration_validator.yaml")
            ),
        ),
        DeclareLaunchArgument(
            "health_params",
            default_value=str(
                package_file("logging_and_diagnostics", "config", "health_aggregator.yaml")
            ),
        ),
        DeclareLaunchArgument(
            "latency_params",
            default_value=str(package_file("benchmarks", "config", "latency_probe.yaml")),
        ),
        DeclareLaunchArgument(
            "teleop_params",
            default_value=str(package_file("teleop_tools", "config", "teleop_relay.yaml")),
        ),
        DeclareLaunchArgument(
            "navigation_task_params",
            default_value=str(package_file("navigation_tasks", "config", "navigation_tasks.yaml")),
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=str(
                package_file("localization_mapping", "config", "slam_toolbox_async.yaml")
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=str(package_file("localization_mapping", "config", "nav2_params.yaml")),
        ),
    ]
