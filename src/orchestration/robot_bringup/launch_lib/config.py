from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml
from launch.substitutions import LaunchConfiguration

from launch_lib.arguments import ARGUMENT_NAMES
from launch_lib.paths import read_yaml
from launch_lib.tiers import LOCKED_MODES, REQUIRED, TIERS, TOGGLE_NAMES, tier_of

VALID_MODES = ("debug", "profile", "production")
VALID_BACKENDS = ("hardware", "isaac", "rosbag")
VALID_SLAM_MODES = ("off", "mapping", "localization")


@dataclass
class ResolvedConfig:
    args: dict
    instrumentation_mode: str
    backend: str
    slam_mode: str
    pose_graph_prefix: str
    map_start_pose: tuple[float, float, float] | None
    log_level: str
    runtime_behaviour: dict
    #: toggle name -> enabled. Every name in TOGGLE_NAMES is present.
    enabled: dict
    #: toggle names that are off and therefore need a synthetic stand-in.
    synthetic: list = field(default_factory=list)


def parse_bool(raw_value: str, name: str) -> bool:
    value = raw_value.strip().lower()
    if value == "true":
        return True
    if value == "false":
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be 'true', 'false' or 'auto', got '{raw_value}'."
    )


def launch_args_from_context(context) -> dict:
    return {name: LaunchConfiguration(name).perform(context) for name in ARGUMENT_NAMES}


def validate_required_files(args: dict) -> None:
    for key in ("system_modes_config", "runtime_modes_config"):
        path = Path(args[key])
        if not path.exists():
            raise RuntimeError(f"Required bringup resource is missing: {path} (from '{key}')")


def resolve_mode(args: dict, system_modes: dict, runtime_modes: dict) -> str:
    mode = args["instrumentation_mode"]
    if mode == "auto":
        mode = system_modes.get("default_mode", "debug")
    if mode not in VALID_MODES:
        raise RuntimeError(f"Unknown instrumentation_mode '{mode}'. Expected one of {VALID_MODES}.")
    if mode not in runtime_modes["instrumentation_modes"]:
        raise RuntimeError(f"runtime_modes.yaml is missing instrumentation mode '{mode}'.")
    return mode


def resolve_backend(args: dict) -> str:
    backend = args["backend"]
    if backend not in VALID_BACKENDS:
        raise RuntimeError(f"Unknown backend '{backend}'. Expected one of {VALID_BACKENDS}.")
    return backend


def resolve_slam_mode(args: dict) -> str:
    slam_mode = args["slam_mode"]
    if slam_mode not in VALID_SLAM_MODES:
        raise RuntimeError(f"Unknown slam_mode '{slam_mode}'. Expected one of {VALID_SLAM_MODES}.")
    return slam_mode


def resolve_pose_graph_prefix(args: dict, slam_mode: str) -> str:
    if slam_mode != "localization":
        return ""

    raw = args["pose_graph_prefix"].strip()
    if not raw:
        raise RuntimeError("slam_mode=localization requires pose_graph_prefix.")
    prefix = Path(raw)
    if not prefix.is_absolute():
        raise RuntimeError("pose_graph_prefix must be an absolute path.")
    if prefix.suffix in (".posegraph", ".data", ".pgm", ".yaml"):
        raise RuntimeError("pose_graph_prefix must omit the file suffix.")
    missing = [
        str(prefix) + suffix
        for suffix in (".posegraph", ".data")
        if not Path(str(prefix) + suffix).is_file()
    ]
    if missing:
        raise RuntimeError(f"Serialized pose graph is incomplete; missing: {missing}")
    return str(prefix)


def resolve_map_start_pose(args: dict, slam_mode: str) -> tuple[float, float, float] | None:
    if slam_mode != "localization":
        return None
    raw = args["map_start_pose"].strip()
    try:
        values = yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise RuntimeError("map_start_pose must be [x, y, yaw].") from error
    if (
        not isinstance(values, list)
        or len(values) != 3
        or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values)
        or not all(math.isfinite(float(value)) for value in values)
    ):
        raise RuntimeError("slam_mode=localization requires a finite map_start_pose [x, y, yaw].")
    return tuple(float(value) for value in values)


def check_toggle_coverage(yaml_toggles: dict) -> None:
    """Every YAML toggle must be classified, and every classified name present.

    Without this, adding a toggle to system_modes.yaml and forgetting to give it
    a tier would silently make it behave like an optional node — including in
    production, where required nodes are supposed to be locked on.
    """
    unclassified = sorted(set(yaml_toggles) - set(TIERS))
    if unclassified:
        raise RuntimeError(
            f"system_modes.yaml toggles have no tier in tiers.py: {unclassified}. "
            "Classify them as required/optional/variant."
        )
    missing = sorted(set(TIERS) - set(yaml_toggles))
    if missing:
        raise RuntimeError(f"system_modes.yaml is missing toggles declared in tiers.py: {missing}.")


def resolve_toggles(args: dict, mode: str, yaml_toggles: dict) -> dict:
    """Apply tier rules to produce the final on/off map.

    In a locked mode (production) the YAML toggles are ignored — everything
    defaults on — but the launch arguments are not blanket-ignored: disabling a
    required node is an error, while optional instrumentation stays a free
    toggle so it can be switched off on a real robot without a rebuild.
    """
    locked = mode in LOCKED_MODES
    enabled = {}

    for name in TOGGLE_NAMES:
        arg_name = f"enable_{name}"
        raw = args[arg_name]
        explicit = None if raw == "auto" else parse_bool(raw, arg_name)

        if locked:
            if explicit is False and tier_of(name) == REQUIRED:
                raise RuntimeError(
                    f"'{arg_name}=false' is not allowed in {mode} mode: "
                    f"'{name}' is a required component and is locked on."
                )
            # Optional and variant components remain selectable; required ones
            # are on regardless of what the YAML says.
            enabled[name] = True if explicit is None else explicit
            continue

        enabled[name] = explicit if explicit is not None else bool(yaml_toggles.get(name, True))

    return enabled


def check_constraints(enabled: dict, mode: str) -> None:
    """Cross-component rules that no single toggle can express."""
    # CLAUDE.md: safety may be false only when drivers is false. Gating a real
    # base with no safety layer is the one combination that must never launch.
    if not enabled["safety"] and enabled["drivers"]:
        raise RuntimeError(
            "Invalid combination: safety=false with drivers=true. The collision-avoidance "
            "only be disabled when no real base is being driven (drivers=false)."
        )


def resolve_config(context) -> ResolvedConfig:
    args = launch_args_from_context(context)
    validate_required_files(args)

    system_modes = read_yaml(Path(args["system_modes_config"]))
    runtime_modes = read_yaml(Path(args["runtime_modes_config"]))

    if "instrumentation_modes" not in runtime_modes:
        raise RuntimeError("runtime_modes.yaml is missing 'instrumentation_modes'.")
    if "toggles" not in system_modes:
        raise RuntimeError("system_modes.yaml is missing 'toggles'.")

    check_toggle_coverage(system_modes["toggles"])

    mode = resolve_mode(args, system_modes, runtime_modes)
    backend = resolve_backend(args)
    slam_mode = resolve_slam_mode(args)
    pose_graph_prefix = resolve_pose_graph_prefix(args, slam_mode)
    map_start_pose = resolve_map_start_pose(args, slam_mode)
    runtime_behaviour = runtime_modes["instrumentation_modes"][mode] or {}

    enabled = resolve_toggles(args, mode, system_modes["toggles"])
    check_constraints(enabled, mode)
    if slam_mode != "off" and backend == "hardware" and not enabled["localization"]:
        raise RuntimeError("Hardware SLAM requires localization to provide odom TF.")

    return ResolvedConfig(
        args=args,
        instrumentation_mode=mode,
        backend=backend,
        slam_mode=slam_mode,
        pose_graph_prefix=pose_graph_prefix,
        map_start_pose=map_start_pose,
        log_level=runtime_behaviour.get("log_level", "info"),
        runtime_behaviour=runtime_behaviour,
        enabled=enabled,
        synthetic=sorted(name for name, on in enabled.items() if not on),
    )
