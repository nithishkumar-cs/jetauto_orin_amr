"""Component tiers — the single source of truth for what may be switched off.

CLAUDE.md settles three tiers:

  required  core data path (sensors / perception / safety / motion). Locked on
            in production. In debug/profile a `false` means "substitute a
            synthetic output", never "run without it". Drivers are the
            exception: false means no local actuator process.
  optional  instrumentation. A free toggle in every mode; nothing depends on it.
  variant   a choice among the implemented, validated combinations enforced
            by config.py.

Toggle names must match the keys in configs/runtime_modes/system_modes.yaml.
resolve_config() cross-checks both directions, so adding a toggle to the YAML
without classifying it here is an error rather than a silent default.
"""

REQUIRED = "required"
OPTIONAL = "optional"
VARIANT = "variant"

TIERS = {
    "drivers": REQUIRED,
    "detector": REQUIRED,
    "geometry": REQUIRED,
    # This processing node is selected with a LiDAR-equipped sensor suite.
    "lidar_clustering": VARIANT,
    "tracking": REQUIRED,
    "fusion": REQUIRED,
    "safety": REQUIRED,
    "diagnostics": OPTIONAL,
    "benchmarks": OPTIONAL,
    "calibration_validator": OPTIONAL,
    "localization": VARIANT,
    "navigation": VARIANT,
    "teleop": VARIANT,
}

TOGGLE_NAMES = tuple(TIERS)

#: Modes that lock required components on.
LOCKED_MODES = ("production",)


def tier_of(name: str) -> str:
    return TIERS[name]
