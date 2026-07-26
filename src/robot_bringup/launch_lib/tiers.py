"""Component tiers — the single source of truth for what may be switched off.

CLAUDE.md settles three tiers:

  required  core data path (sensors / perception / safety / motion). Locked on
            in production. In debug/profile a `false` means "substitute a
            synthetic source", never "run without it".
  optional  instrumentation. A free toggle in every mode; nothing depends on it.
  variant   a choice among *validated* combinations, not a free toggle. Which
            localization or teleop you run is a selection, not an on/off.

Toggle names must match the keys in configs/runtime_modes/system_modes.yaml.
resolve_config() cross-checks both directions, so adding a toggle to the YAML
without classifying it here is an error rather than a silent default.
"""

REQUIRED = "required"
OPTIONAL = "optional"
VARIANT = "variant"

TIERS = {
    "drivers": REQUIRED,
    "preproc": REQUIRED,
    "detector": REQUIRED,
    "geometry": REQUIRED,
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

#: Modes that ignore toggles entirely and run the full stack.
LOCKED_MODES = ("production",)


def tier_of(name: str) -> str:
    return TIERS[name]


def names_in_tier(tier: str) -> tuple:
    return tuple(n for n, t in TIERS.items() if t == tier)
