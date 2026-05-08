# Configs

Project-level configuration lives here. Package-local ROS parameters stay inside
the owning package under `src/<domain>/<package>/config`.

Current shared config files:

- `robot/robot_profile.yaml`: robot identity, frames, dimensions, and limits
- `sensors/topic_contracts.yaml`: normalized sensor and robot-state topic names
- `models/model_registry.yaml`: detector backend registry and artifact paths
- `calibration/calibration_manifest.yaml`: calibration files and required frames
- `runtime_modes/runtime_modes.yaml`: hardware/replay and debug/profile/production modes
