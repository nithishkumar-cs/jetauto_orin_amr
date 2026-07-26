from dataclasses import dataclass, field
from pathlib import Path

from launch.substitutions import LaunchConfiguration

from launch_lib.arguments import ARGUMENT_NAMES
from launch_lib.paths import read_yaml
from launch_lib.tiers import LOCKED_MODES, REQUIRED, TIERS, TOGGLE_NAMES, tier_of

VALID_MODES = ("debug", "profile", "production")


@dataclass
class ResolvedConfig:
    args: dict
    instrumentation_mode: str
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
            "Invalid combination: safety=false with drivers=true. The safety gate may "
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
    runtime_behaviour = runtime_modes["instrumentation_modes"][mode] or {}

    enabled = resolve_toggles(args, mode, system_modes["toggles"])
    check_constraints(enabled, mode)

    return ResolvedConfig(
        args=args,
        instrumentation_mode=mode,
        log_level=runtime_behaviour.get("log_level", "info"),
        runtime_behaviour=runtime_behaviour,
        enabled=enabled,
        synthetic=sorted(name for name, on in enabled.items() if not on),
    )
