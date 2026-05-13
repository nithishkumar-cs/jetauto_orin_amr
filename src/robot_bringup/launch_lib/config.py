from dataclasses import dataclass
from pathlib import Path

from launch.substitutions import LaunchConfiguration

from launch_lib.arguments import ARGUMENT_NAMES
from launch_lib.paths import detector_config_for_backend, read_yaml


@dataclass
class ResolvedConfig:
    args: dict
    config_set_name: str
    instrumentation_mode: str
    detector_backend: str
    detector_params: Path
    log_level: str
    enable_base_interface: bool
    enable_sensor_drivers: bool
    enable_preproc: bool
    enable_depth_pipeline: bool
    enable_teleop: bool
    enable_localization: bool
    enable_navigation_tasks: bool
    enable_benchmarks: bool
    base_transport_mode: str
    localization_mode: str
    health_topics: list[str]


def parse_bool(raw_value: str, name: str) -> bool:
    value = raw_value.strip().lower()
    if value == "true":
        return True
    if value == "false":
        return False
    raise RuntimeError(f"Launch argument '{name}' must be 'true' or 'false', got '{raw_value}'.")


def resolve_toggle(raw_value: str, default_value: bool, name: str) -> bool:
    if raw_value == "auto":
        return default_value
    return parse_bool(raw_value, name)


def resolve_string(raw_value: str, default_value: str) -> str:
    if raw_value == "auto":
        return default_value
    return raw_value


def launch_args_from_context(context) -> dict:
    return {name: LaunchConfiguration(name).perform(context) for name in ARGUMENT_NAMES}


def validate_required_files(args: dict) -> None:
    required_files = [
        Path(args["system_modes_config"]),
        Path(args["runtime_modes_config"]),
        Path(args["robot_profile_config"]),
        Path(args["topic_contracts_config"]),
        Path(args["calibration_manifest"]),
        Path(args["base_params"]),
        Path(args["camera_params"]),
        Path(args["preproc_params"]),
        Path(args["geometry_params"]),
        Path(args["tracking_params"]),
        Path(args["fusion_params"]),
        Path(args["safety_params"]),
        Path(args["calibration_params"]),
        Path(args["health_params"]),
        Path(args["latency_params"]),
        Path(args["teleop_params"]),
        Path(args["navigation_task_params"]),
        Path(args["slam_params"]),
        Path(args["nav2_params"]),
    ]
    for path in required_files:
        if not path.exists():
            raise RuntimeError(f"Required bringup resource is missing: {path}")


def resolve_config(context) -> ResolvedConfig:
    args = launch_args_from_context(context)
    validate_required_files(args)

    system_modes = read_yaml(Path(args["system_modes_config"]))
    runtime_modes = read_yaml(Path(args["runtime_modes_config"]))
    robot_profile = read_yaml(Path(args["robot_profile_config"]))
    topic_contracts = read_yaml(Path(args["topic_contracts_config"]))
    calibration_manifest = read_yaml(Path(args["calibration_manifest"]))

    for key in ("defaults", "config_sets"):
        if key not in system_modes:
            raise RuntimeError(f"system_modes.yaml is missing required key '{key}'.")
    if "instrumentation_modes" not in runtime_modes:
        raise RuntimeError("runtime_modes.yaml is missing 'instrumentation_modes'.")
    if "robot" not in robot_profile:
        raise RuntimeError("robot_profile.yaml is missing 'robot'.")
    if "topics" not in topic_contracts:
        raise RuntimeError("topic_contracts.yaml is missing 'topics'.")
    if "required_files" not in calibration_manifest:
        raise RuntimeError("calibration_manifest.yaml is missing 'required_files'.")

    instrumentation_mode = args["instrumentation_mode"]
    if instrumentation_mode == "auto":
        instrumentation_mode = system_modes["defaults"].get("instrumentation_mode", "debug")

    config_set_by_mode = system_modes["defaults"].get("config_set_by_mode", {})
    if instrumentation_mode not in config_set_by_mode:
        raise RuntimeError(f"Unknown instrumentation_mode '{instrumentation_mode}'.")
    if instrumentation_mode not in runtime_modes["instrumentation_modes"]:
        raise RuntimeError(f"runtime_modes.yaml is missing instrumentation mode '{instrumentation_mode}'.")

    config_set_name = config_set_by_mode[instrumentation_mode]
    if config_set_name not in system_modes["config_sets"]:
        raise RuntimeError(f"system_modes.yaml is missing config set '{config_set_name}'.")

    mode_defaults = system_modes["config_sets"][config_set_name]
    runtime_mode = runtime_modes["instrumentation_modes"][instrumentation_mode]

    detector_backend = args["detector_backend"]
    if detector_backend == "auto":
        detector_backend = mode_defaults.get(
            "detector_backend",
            system_modes["defaults"].get("detector_backend_defaults", {}).get(instrumentation_mode, "debug"),
        )

    detector_params = Path(args["detector_params"]) if args["detector_params"] else detector_config_for_backend(detector_backend)
    if not detector_params.exists():
        raise RuntimeError(f"Detector params file is missing: {detector_params}")

    enable_base_interface = resolve_toggle(
        args["enable_base_interface"], mode_defaults.get("enable_base_interface", False), "enable_base_interface"
    )
    enable_sensor_drivers = resolve_toggle(
        args["enable_sensor_drivers"], mode_defaults.get("enable_sensor_drivers", False), "enable_sensor_drivers"
    )
    enable_preproc = resolve_toggle(
        args["enable_preproc"], mode_defaults.get("enable_preproc", True), "enable_preproc"
    )
    enable_depth_pipeline = resolve_toggle(
        args["enable_depth_pipeline"], mode_defaults.get("enable_depth_pipeline", False), "enable_depth_pipeline"
    )
    enable_teleop = resolve_toggle(
        args["enable_teleop"], mode_defaults.get("enable_teleop", False), "enable_teleop"
    )
    enable_localization = resolve_toggle(
        args["enable_localization"], mode_defaults.get("enable_localization", False), "enable_localization"
    )
    enable_navigation_tasks = resolve_toggle(
        args["enable_navigation_tasks"], mode_defaults.get("enable_navigation_tasks", False), "enable_navigation_tasks"
    )
    enable_benchmarks = resolve_toggle(
        args["enable_benchmarks"], mode_defaults.get("enable_benchmarks", False), "enable_benchmarks"
    )

    base_transport_mode = resolve_string(args["base_transport_mode"], mode_defaults.get("base_transport_mode", "stub"))
    localization_mode = resolve_string(args["localization_mode"], mode_defaults.get("localization_mode", "mapping"))
    if localization_mode not in ("mapping", "navigation"):
        raise RuntimeError(f"Unsupported localization_mode '{localization_mode}'.")
    if enable_localization and localization_mode == "navigation" and not args["map_file"]:
        raise RuntimeError("Localization navigation mode requires map_file.")

    health_topics = ["/diagnostics/calibration/health", "/diagnostics/detector/health"]
    if enable_sensor_drivers:
        health_topics.append("/diagnostics/rgb_camera/health")
    if enable_preproc:
        health_topics.append("/diagnostics/preproc/health")
    if enable_depth_pipeline:
        health_topics.extend([
            "/diagnostics/geometry/health",
            "/diagnostics/tracking/health",
            "/diagnostics/fusion/health",
        ])
    if enable_base_interface:
        health_topics.append("/diagnostics/base/health")

    return ResolvedConfig(
        args=args,
        config_set_name=config_set_name,
        instrumentation_mode=instrumentation_mode,
        detector_backend=detector_backend,
        detector_params=detector_params,
        log_level=runtime_mode.get("log_level", "info"),
        enable_base_interface=enable_base_interface,
        enable_sensor_drivers=enable_sensor_drivers,
        enable_preproc=enable_preproc,
        enable_depth_pipeline=enable_depth_pipeline,
        enable_teleop=enable_teleop,
        enable_localization=enable_localization,
        enable_navigation_tasks=enable_navigation_tasks,
        enable_benchmarks=enable_benchmarks,
        base_transport_mode=base_transport_mode,
        localization_mode=localization_mode,
        health_topics=health_topics,
    )
