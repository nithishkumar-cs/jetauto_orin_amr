from launch.actions import DeclareLaunchArgument

from launch_lib.paths import share_path
from launch_lib.tiers import TOGGLE_NAMES

#: Every per-node toggle is exposed as `enable_<name>`. "auto" defers to
#: system_modes.yaml; "true"/"false" override it, subject to the tier rules in
#: config.py (production rejects overrides of required nodes).
TOGGLE_ARGUMENT_NAMES = tuple(f"enable_{name}" for name in TOGGLE_NAMES)

ARGUMENT_NAMES = (
    "instrumentation_mode",
    "backend",
    "slam_mode",
    "pose_graph_prefix",
    "map_start_pose",
    "system_modes_config",
    "runtime_modes_config",
) + TOGGLE_ARGUMENT_NAMES


def declared_arguments() -> list:
    # configs/ is installed wholesale into share/robot_bringup/project_configs,
    # so both mode files are read from the *installed* tree. With
    # --symlink-install that makes edits live without a rebuild.
    project_config_root = share_path("robot_bringup") / "project_configs"
    runtime_modes_dir = project_config_root / "runtime_modes"

    arguments = [
        DeclareLaunchArgument(
            "instrumentation_mode",
            default_value="auto",
            description=(
                "debug | profile | production. 'auto' uses system_modes.yaml default_mode."
            ),
        ),
        DeclareLaunchArgument(
            "backend",
            default_value="hardware",
            description=(
                "hardware | isaac | rosbag. Selects the one source at the edge of the stack."
            ),
        ),
        DeclareLaunchArgument(
            "slam_mode",
            default_value="off",
            description="off | mapping | localization. SLAM is opt-in in every runtime mode.",
        ),
        DeclareLaunchArgument(
            "pose_graph_prefix",
            default_value="",
            description=(
                "Absolute saved SLAM Toolbox pose-graph prefix for localization mode; "
                "omit the .posegraph/.data suffix."
            ),
        ),
        DeclareLaunchArgument(
            "map_start_pose",
            default_value="",
            description=(
                "Required for localization: approximate initial map-frame [x, y, yaw] "
                "in metres and radians, for example '[-0.5, -1.0, 0.0]'."
            ),
        ),
        DeclareLaunchArgument(
            "system_modes_config",
            default_value=str(runtime_modes_dir / "system_modes.yaml"),
            description="Mode selection and per-node toggles.",
        ),
        DeclareLaunchArgument(
            "runtime_modes_config",
            default_value=str(runtime_modes_dir / "runtime_modes.yaml"),
            description="Per-mode behaviour: log level, assertions, timing, metrics.",
        ),
    ]

    arguments.extend(
        DeclareLaunchArgument(
            f"enable_{name}",
            default_value="auto",
            description=f"Enable the {name} component. 'auto' defers to system_modes.yaml.",
        )
        for name in TOGGLE_NAMES
    )

    return arguments
